"""
The live screen: watch the robot while it runs.

`run_live()` is the whole feature -- give it a robot factory and the rows to
replay and it opens the window, runs the robot in a worker thread, and returns
when the run ends or the window closes.

Why a thread: the robot blocks. A step holds for dt (2*dt on two channels) and
clamping freezes for ~2.7 s, so a display cannot share its thread. The robot
gets its own; pygame keeps the main thread.

Why HD/30 and not FHD/60: rendering holds the GIL, and an FHD frame costs about
8.7 ms of CPU against ~1.1 ms at HD -- enough to delay the robot thread's 10 ms
wakeups. At HD/30 with a shortened GIL switch interval, step timing lands within
0.6 ms of running with no panel at all. Raising either trades that margin away.
"""

import os
import sys
import threading
import time

# Must be set before aviar.panel is imported: the layout is computed at import.
os.environ.setdefault("AVIAR_RESOLUTION", "HD")

import pygame

from . import panel as _panel
from .config import CH_CATH
from .live import (
    PHASE_DONE,
    PHASE_PAUSE,
    PHASE_RETRACT,
    PHASE_RUN,
    LiveState,
    TappedLink,
    tap_stream,
)
from .panel import (
    BG, BLx, BLy, BRx, BRy, DIV_COL, GREEN, GREY, GREY_DK, H, ORANGE, PH,
    QH_B, QH_T, QW, SCALE, SILVER, TEAL, TEAL_DIM, TLx, TLy, TRx, TRy, W,
    Dial, build_fonts, draw_geometry, draw_rotation,
)
from .replay import replay
from .retract import reverse

FPS = 30                 # see the module docstring before raising this
GIL_SWITCH_S = 0.0005    # hand the GIL over faster than the 5 ms default

# A live run issues a new command every dt (0.1 s), faster than the exporter's
# 0.2 s. At the default 0.35 s ease the needle never catches up -- it reaches
# only ~92% of the peak with 2.4 deg/s of lag. 0.15 s tracks the peaks exactly
# and still reads as smooth.
EASE_S = 0.15
_panel.EASE_S = EASE_S

# Dial ranges to choose from, smallest first.
_TRANS_STEPS = (1, 2, 5, 10, 20, 30, 50)
_ROT_STEPS = (5, 10, 30, 60, 90, 120, 180, 270, 360)


def _dial_range(peak, steps, headroom=1.15):
    """
    Smallest range from `steps` that fits `peak` with a little headroom.

    A dial fixed at the protocol maximum is useless on a gentle log: a run
    peaking at 100 deg/s on a +/-360 dial never moves the needle more than a
    tenth of its sweep, and a median of ~2 deg/s looks like a dead instrument.
    Scaling to the data is what makes the dial readable.
    """
    want = abs(peak) * headroom
    for step in steps:
        if step >= want:
            return step
    return steps[-1]


# =========================
# RENDERING
# =========================
def draw_status(surf, fonts, s):
    """The bottom strip: phase, progress, raw frames, clamp state, chart."""
    y0 = QH_T + QH_B
    pygame.draw.line(surf, DIV_COL, (0, y0), (W, y0), max(1, int(2 * SCALE)))
    pad = int(14 * SCALE)
    ft, fs = fonts["tiny"], fonts["small"]

    # phase + progress
    phase = s["phase"]
    col = {PHASE_RETRACT: ORANGE, PHASE_DONE: GREEN}.get(phase, TEAL)
    surf.blit(fs.render(phase, True, col), (pad, y0 + pad))

    total = s["total_rows"]
    prog = f"row {s['row']}" + (f" / {total}" if total else "")
    surf.blit(ft.render(prog, True, SILVER), (pad, y0 + pad + int(18 * SCALE)))
    surf.blit(ft.render(f"{s['elapsed_s']:6.1f} s", True, SILVER),
              (pad, y0 + pad + int(32 * SCALE)))

    bar_x = pad + int(110 * SCALE)
    bar_w = int(300 * SCALE)
    bar_y = y0 + pad + int(4 * SCALE)
    bar_h = int(8 * SCALE)
    pygame.draw.rect(surf, GREY_DK, (bar_x, bar_y, bar_w, bar_h))
    if total:
        frac = max(0.0, min(1.0, s["row"] / total))
        pygame.draw.rect(surf, col, (bar_x, bar_y, int(bar_w * frac), bar_h))

    # cumulative estimate
    cx = bar_x + bar_w + int(30 * SCALE)
    for i, (lab, val) in enumerate((
            ("insert", f"{s['guide_pos_mm']:+8.2f} mm"),
            ("angle",  f"{s['guide_angle_deg']:+8.1f} deg"),
            ("cath",   f"{s['cath_pos_mm']:+8.2f} mm"))):
        yy = y0 + pad + i * int(16 * SCALE)
        surf.blit(ft.render(f"{lab:>6}", True, GREY), (cx, yy))
        surf.blit(ft.render(val, True, TEAL), (cx + int(48 * SCALE), yy))

    # raw frames + clamp state, per channel
    fx = cx + int(190 * SCALE)
    surf.blit(ft.render("frame  msg/ch/clamp/trans/rot", True, GREY), (fx, y0 + pad))
    clamp_name = {0: "idle", 1: "released", 2: "CLAMPED"}
    for i, ch in enumerate(s["channels"]):
        yy = y0 + pad + int((16 + i * 16) * SCALE)
        f = s["last_frame"].get(ch)
        txt = "/".join(str(v) for v in (f[0], ch, f[1], f[2], f[3])) if f else "-"
        live = (s["active_ch"] == ch)
        surf.blit(ft.render(f"CH{ch}", True, TEAL if live else GREY_DK), (fx, yy))
        surf.blit(ft.render(txt, True, SILVER if live else GREY_DK),
                  (fx + int(34 * SCALE), yy))
        cs = s["clamp_state"].get(ch, 0)
        surf.blit(ft.render(clamp_name[cs], True, ORANGE if cs == 2 else GREY),
                  (fx + int(190 * SCALE), yy))

    # rolling speed chart
    hx = fx + int(260 * SCALE)
    hw, hh = W - hx - pad, PH - 2 * pad
    hy = y0 + pad
    if hw > int(60 * SCALE) and hh > int(20 * SCALE):
        pygame.draw.rect(surf, GREY_DK, (hx, hy, hw, hh), max(1, int(SCALE)))
        mid = hy + hh // 2
        pygame.draw.line(surf, GREY_DK, (hx, mid), (hx + hw, mid), 1)
        hist = s["history"]
        if len(hist) > 1:
            t0, t1 = hist[0][0], hist[-1][0]
            span = max(1e-6, t1 - t0)
            for idx, (scale, colr) in enumerate(((50.0, ORANGE), (360.0, TEAL))):
                pts = []
                for (t, v2, w2, v4) in hist:
                    val = (v4 if idx == 0 else w2) / scale
                    px = hx + int(hw * (t - t0) / span)
                    py = mid - int((hh / 2 - 2) * max(-1.0, min(1.0, val)))
                    pts.append((px, py))
                if len(pts) > 1:
                    pygame.draw.lines(surf, colr, False, pts, max(1, int(SCALE)))
        surf.blit(ft.render("v4 / w2", True, GREY), (hx + 4, hy + 2))


def render(surf, fonts, dials, s, now):
    surf.fill(BG)

    # The left dial shows whichever translation this run actually drives: the
    # catheter when there is one, otherwise the guidewire. On a single-channel
    # run v4 is always 0 (no catheter), so showing it would waste the dial.
    left_key = "v4_mm_s" if CH_CATH in s["channels"] else "v2_mm_s"
    # restart_on_repeat=False: render() runs every frame, and restarting the
    # ease each time would freeze the needle at 0 (see Dial.set).
    dials["trans"].set(s[left_key], now, restart_on_repeat=False)
    dials["rot"].set(s["w2_deg_s"], now, restart_on_repeat=False)
    for d in dials.values():
        d.update(now)
        d.draw(surf, fonts["small"], fonts["val"])

    draw_geometry(surf, s["cath_pos_mm"], s["guide_pos_mm"],
                  s["guide_angle_deg"], s["v2_mm_s"], fonts)
    draw_rotation(surf, s["guide_angle_deg"], s["w2_deg_s"], s["v2_mm_s"], fonts)

    pygame.draw.line(surf, DIV_COL, (QW, 0), (QW, QH_T + QH_B), max(1, int(2 * SCALE)))
    pygame.draw.line(surf, DIV_COL, (0, QH_T), (W, QH_T), max(1, int(2 * SCALE)))
    left_title = "CATHETER SPEED" if CH_CATH in s["channels"] else "GUIDEWIRE SPEED"
    for x, y, title in ((TLx, TLy, left_title), (TRx, TRy, "GUIDEWIRE ROTATION"),
                        (BLx, BLy, "GEOMETRY"), (BRx, BRy, "ORIENTATION")):
        surf.blit(fonts["tiny"].render(title, True, TEAL_DIM),
                  (x + int(14 * SCALE), y + int(10 * SCALE)))

    draw_status(surf, fonts, s)



# =========================
# ENTRY POINT
# =========================
def _run_robot(state, make_robot, rows, max_steps, label,
               retract, retract_pause_s):
    """
    The worker thread: replay the rows, then optionally the reverse.

    Same sequence as retract.replay_with_retract -- one clamp at the start, the
    grip HELD through the pause, one release at the very end -- with the
    panel's phase set at each transition.
    """
    state.begin()
    robot = make_robot()
    try:
        # hold the grip across the pause when a retract follows
        replay(robot, tap_stream(rows, state, PHASE_RUN),
               max_steps=max_steps, label=label,
               release=not retract, close=False)

        if retract and not state.stop_requested.is_set():
            if retract_pause_s > 0:
                state.set_phase(PHASE_PAUSE)
                print(f"Holding clamp, pausing {retract_pause_s:.1f}s before retract...")
                robot.stop(retract_pause_s)
            print("Retracting...")
            replay(robot, tap_stream(reverse(rows), state, PHASE_RETRACT),
                   max_steps=max_steps, label="retract",
                   clamp=False, close=False)
    finally:
        state.finish()
        robot.close()


def run_live(make_robot, rows, channels, *, max_steps=None, label="",
             initial_angle_deg=0.0, initial_pos_mm=0.0,
             retract=False, retract_pause_s=2.0, fps=FPS):
    """
    Replay `rows` with the live screen open. Returns the number of rows run.

    `make_robot` takes one argument, the RobotLink to use, and returns a Robot
    built with it -- the link is how the panel sees the frames going out, and a
    fresh Robot is needed per pass because replay() closes the one it is given:

        run_live(lambda link: Robot(channels=CHANNELS, link=link),
                 rows, CHANNELS)

    Closing the window or pressing ESC ends the run at the next row boundary;
    the robot is released and closed either way.
    """
    rows = list(rows)
    n_rows = min(len(rows), max_steps or len(rows))
    state = LiveState(initial_angle_deg=initial_angle_deg,
                      initial_pos_mm=initial_pos_mm,
                      total_rows=n_rows * 2 if retract else n_rows,
                      channels=tuple(channels))

    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption(f"AVIAR — live  ({label})" if label else "AVIAR — live")
    clock = pygame.time.Clock()
    fonts = build_fonts()
    # Scale each dial to what this log actually contains, so the needle uses
    # its sweep instead of sitting near zero all run.
    dual = CH_CATH in state.channels
    scanned = rows[:n_rows]      # only what will actually be replayed
    peak_trans = max((abs(r.v4_mm_s if dual else r.v2_mm_s) for r in scanned), default=0.0)
    peak_rot = max((abs(r.w2_deg_s) for r in scanned), default=0.0)
    t_max = _dial_range(peak_trans, _TRANS_STEPS)
    r_max = _dial_range(peak_rot, _ROT_STEPS)
    print(f"Dials scaled to this log: translation +/-{t_max} mm/s, "
          f"rotation +/-{r_max} deg/s")

    dials = {
        "trans": Dial(TLx + QW // 2, TLy + int(QH_T * 0.58), int(96 * SCALE),
                      "CATHETER" if dual else "GUIDEWIRE",
                      "mm/s", -t_max, t_max, ORANGE),
        "rot": Dial(TRx + QW // 2, TRy + int(QH_T * 0.58), int(96 * SCALE),
                    "ROTATION", "deg/s", -r_max, r_max, TEAL),
    }

    old_switch = sys.getswitchinterval()
    sys.setswitchinterval(GIL_SWITCH_S)

    worker = threading.Thread(
        target=_run_robot,
        args=(state, lambda: make_robot(TappedLink(state)), rows,
              max_steps, label, retract, retract_pause_s),
        daemon=True)
    worker.start()

    try:
        running = True
        while running:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT or (
                        ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                    running = False
            if not running:
                state.stop_requested.set()

            render(screen, fonts, dials, state.snapshot(), time.time())
            pygame.display.flip()
            clock.tick(fps)

            # The run is over, but leave the window up showing the final state
            # until it is closed -- otherwise it vanishes the moment the last
            # row finishes and there is nothing to read.
        # window closed -> stop the robot and wait for it to unwind
        state.stop_requested.set()
        worker.join(timeout=15.0)
    finally:
        state.stop_requested.set()
        worker.join(timeout=15.0)
        sys.setswitchinterval(old_switch)
        pygame.quit()

    return state.snapshot()["row"]
