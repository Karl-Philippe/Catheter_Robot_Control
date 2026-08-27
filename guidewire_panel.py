"""
Guidewire Control Monitor — offline replay of a recorded log (pygame).

  TL : Catheter translation speed   (v4, arc dial)
  TR : Guidewire rotation speed     (w2, arc dial)
  BL : Catheter / guidewire geometry — straightening effect
  BR : Guidewire orientation (compass + accumulated rotation)

Reads a log with a wall_time column, replays it against a virtual clock, and
either renders to an MP4 or shows an interactive window. It never opens a
socket and never talks to the robot -- see live_panel.py to watch a real run.

The drawing code lives in aviar/panel.py and is shared with live_panel.py.

Requirements:  pip install pygame numpy opencv-python
Run:           python guidewire_panel.py
"""

import math, sys, time, pygame
import numpy as np
import cv2
from pathlib import Path
from dataclasses import dataclass
from typing import Iterator

# Layout, palette, fonts and every draw_* helper come from the library.
from aviar.panel import (
    BG, TEAL, TEAL_DK, TEAL_DIM, ORANGE, GREY, GREY_DK, SILVER, SILVER_DK,
    VESSEL_BG, VESSEL_WALL, GREEN, DIV_COL,
    W, H, QW, QH_T, QH_B, PH, TLx, TLy, TRx, TRy, BLx, BLy, BRx, BRy,
    SCALE, GEO_SCALE, RESOLUTION,
    R_MM, VESSEL_RADIUS_MM, CATHETER_RADIUS_MM, EASE_S,
    Dial, build_fonts, draw_geometry, draw_rotation,
    lerp, lerp_color, ease_inout, polar,
    glow_line, glow_circle, glow_polyline,
)

# ─── CONFIGURATION ────────────────────────────────────────────────────────────
LOG_PATH     = "data/control_logs.txt"   # <- set your path here
ASSUME_UNITS = "rad"                      # "deg" or "rad"
DT_TARGET    = 0.2
STEP_S       = 0.1    # seconds per log step (real robot DT = 0.1 s, x2 for visibility)
EASE_S       = 0.35   # seconds for dial needles to ease toward new value (lower = snappier)
CLAMP_S      = 0.0    # pre-roll pause (s) — set >0 for interactive feel
RELEASE_S    = 0.0    # post-roll pause (s)
R_MM             = 20.0   # guidewire radius of curvature (mm)
VESSEL_RADIUS_MM = 6.5    # vessel inner half-width (mm) — increase for wider vessel
CATHETER_RADIUS_MM = 3.8  # catheter half-width (mm)

# ─── VIDEO EXPORT ─────────────────────────────────────────────────────────────
VIDEO_EXPORT           = True    # False = interactive pygame window
VIDEO_PATH             = "videos/guidewire_replay.mp4"
VIDEO_FPS              = 30
VIDEO_EXPORT_QUADRANTS = True    # also write one video per quadrant
VIDEO_QUAD_PATH        = "videos/guidewire_{name}.mp4"   # {name} filled with quadrant key
# ─── LOG PARSER (mirrors the control script) ──────────────────────────────────
@dataclass
class StepCmd:
    v2_mm_s:  float   # guidewire translation speed
    w2_deg_s: float   # guidewire rotation speed
    v4_mm_s:  float   # catheter translation speed


def iter_stepcmds_from_log(log_path, dt_target=DT_TARGET, assume_units=ASSUME_UNITS):
    path = Path(log_path)
    if not path.exists():
        raise FileNotFoundError(f"Log not found: {path}")
    with path.open("r", encoding="utf-8") as f:
        header = None
        for line in f:
            line = line.strip()
            if line:
                header = line.split()
                break
        if header is None:
            return

        def idx(name):
            if name not in header:
                raise ValueError(f"Missing column '{name}'. Found: {header}")
            return header.index(name)

        i_t  = idx("wall_time")
        i_w  = idx("guide_rotation_speed_cmd")
        i_v2 = idx("guide_speed_cmd")
        i_v4 = idx("cath_speed_cmd")
        next_t = None

        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < len(header):
                continue
            t, w, v2, v4 = (float(parts[i_t]), float(parts[i_w]),
                             float(parts[i_v2]), float(parts[i_v4]))
            if assume_units == "rad":
                w *= 180.0 / math.pi
            if next_t is None:
                next_t = t
            if t >= next_t:
                yield StepCmd(v2_mm_s=v2, w2_deg_s=w, v4_mm_s=v4)
                steps  = max(1, int((t - next_t) // dt_target) + 1)
                next_t += steps * dt_target


def load_cmds(log_path):
    return list(iter_stepcmds_from_log(log_path))



# ─── CHROME: DIVIDERS + QUADRANT LABELS ───────────────────────────────────────
def draw_chrome(surf, fonts, phase, log_lines, step, total):
    ft = fonts['tiny']

    # Dividers
    pygame.draw.line(surf, DIV_COL, (QW, 0),    (QW, QH_T + QH_B), 1)
    pygame.draw.line(surf, DIV_COL, (0,  QH_T), (W,  QH_T),        1)
    pygame.draw.line(surf, DIV_COL, (0,  QH_T + QH_B), (W, QH_T + QH_B), 1)

    # Quadrant labels
    for lbl, x, y in [
        ("CATHETER  ·  TRANSLATION  SPEED",     QW // 2,       10),
        ("GUIDEWIRE  ·  ROTATION  SPEED",        QW + QW // 2, 10),
        ("CATHETER  ·  GUIDEWIRE  GEOMETRY",     QW // 2,       QH_T + 10),
        ("GUIDEWIRE  ·  ORIENTATION",            QW + QW // 2, QH_T + 10),
    ]:
        t = ft.render(lbl, True, GREY_DK)
        surf.blit(t, t.get_rect(center=(x, y)))

    # ── Progress panel ────────────────────────────────────────────────────────
    py = QH_T + QH_B
    pygame.draw.rect(surf, (5, 13, 13), (0, py, W, PH))

    bx0, bw0, bh0 = 20, W - 40, 6
    bar_y = py + 10
    frac  = min(step, total) / total if total > 0 else 0.0
    pygame.draw.rect(surf, TEAL_DK,  (bx0, bar_y, bw0, bh0), border_radius=3)
    fw = int(bw0 * frac)
    if fw > 0:
        pygame.draw.rect(surf, TEAL, (bx0, bar_y, fw, bh0), border_radius=3)

    ct = ft.render(f"{min(step, total)} / {total} steps", True, GREY_DK)
    surf.blit(ct, (bx0, bar_y + bh0 + 4))

    badge_col = TEAL if phase == "RUN" else (ORANGE if phase == "DONE" else GREY_DK)
    bt = ft.render(phase, True, badge_col)
    surf.blit(bt, (W - bt.get_width() - 20, bar_y + bh0 + 4))

    if log_lines:
        lt = ft.render(log_lines[-1], True, (42, 90, 80))
        surf.blit(lt, (bx0, py + PH - 16))

    # Run / Replay button
    btn = pygame.Rect(W // 2 - 70, py + 36, 140, 20)
    active  = phase in ("IDLE", "DONE")
    btn_col = TEAL if active else GREY_DK
    pygame.draw.rect(surf, (5, 13, 13), btn, border_radius=4)
    pygame.draw.rect(surf, btn_col, btn, 1, border_radius=4)
    lbl = ("Run Sequence" if phase == "IDLE" else
           "Replay"       if phase == "DONE" else "Running...")
    tl  = ft.render(lbl, True, btn_col)
    surf.blit(tl, tl.get_rect(center=btn.center))
    return btn



# ─── SHARED SIMULATION STATE ──────────────────────────────────────────────────
def build_sim_state(cmds, fonts):
    """Return a fresh mutable simulation dict."""
    dial_v4 = Dial(QW // 2,      QH_T // 2, int(92 * SCALE), "catheter speed (v4)", "mm/s",    -50,  50,  ORANGE)
    dial_w2 = Dial(QW + QW // 2, QH_T // 2, int(92 * SCALE), "guide rot. speed (w2)", "deg/s", -360, 360, TEAL)
    return dict(
        cmds=cmds, fonts=fonts,
        dial_v4=dial_v4, dial_w2=dial_w2,
        guide_pos=0.0, cath_pos=0.0, guide_angle=0.0,
        cur_v2=0.0, cur_w2=0.0, cur_v4=0.0,
        phase="CLAMP", step=-1,
        vtime=0.0,           # virtual time (seconds)
        step_start=0.0,
        clamp_start=0.0,
        release_start=0.0,
        log_lines=[f"> Loaded {len(cmds)} steps from {Path(LOG_PATH).name}",
                   "> Clamping CH2 + CH4..."],
    )


def sim_set_cmd(s, idx):
    c = s['cmds'][idx]
    s['cur_v2'], s['cur_w2'], s['cur_v4'] = c.v2_mm_s, c.w2_deg_s, c.v4_mm_s
    s['dial_v4'].set(c.v4_mm_s,  s['vtime'])
    s['dial_w2'].set(c.w2_deg_s, s['vtime'])
    s['log_lines'].append(
        f"> [{idx + 1}/{len(s['cmds'])}]"
        f"  v2={c.v2_mm_s:+.1f}  w2={c.w2_deg_s:+.1f}  v4={c.v4_mm_s:+.1f}"
    )


def sim_tick(s, dt):
    """Advance simulation by dt virtual seconds. Returns True while still running."""
    s['vtime'] += dt
    now = s['vtime']
    phase = s['phase']

    if phase == "CLAMP":
        if now - s['clamp_start'] > CLAMP_S:
            s['log_lines'].append("> Clamp done. Replaying...")
            s['phase'] = "RUN"
            s['step'] = 0
            s['step_start'] = now
            sim_set_cmd(s, 0)

    elif phase == "RUN":
        s['guide_pos']   += s['cur_v2'] * dt
        s['cath_pos']    += s['cur_v4'] * dt
        s['guide_angle'] += s['cur_w2'] * dt

        if now - s['step_start'] > STEP_S:
            s['step'] += 1
            if s['step'] >= len(s['cmds']):
                s['cur_v2'] = s['cur_w2'] = s['cur_v4'] = 0.0
                s['dial_v4'].set(0, now)
                s['dial_w2'].set(0, now)
                s['log_lines'].append("> Done. Releasing...")
                s['phase'] = "RELEASE"
                s['release_start'] = now
            else:
                s['step_start'] = now
                sim_set_cmd(s, s['step'])

    elif phase == "RELEASE":
        if now - s['release_start'] > RELEASE_S:
            s['log_lines'].append("> Released. All stop.")
            s['phase'] = "DONE"
            return False   # signal: finished

    s['dial_v4'].update(now)
    s['dial_w2'].update(now)
    return True


def render_frame(screen, s):
    """Draw one frame onto screen."""
    fonts = s['fonts']
    screen.fill(BG)

    # Subtle top glow
    gl = pygame.Surface((W, QH_T), pygame.SRCALPHA)
    for ri in range(220, 0, -40):
        a = max(0, 7 - (220 - ri) // 40)
        pygame.draw.ellipse(gl, (0, 40, 32, a),
                            (W // 2 - ri, -ri // 2, ri * 2, ri), 2)
    screen.blit(gl, (0, 0))

    s['dial_v4'].draw(screen, fonts['small'], fonts['val'])
    s['dial_w2'].draw(screen, fonts['small'], fonts['val'])
    draw_geometry(screen, s['cath_pos'], s['guide_pos'], s['guide_angle'], s['cur_v2'], fonts)
    draw_rotation(screen, s['guide_angle'], s['cur_w2'], s['cur_v2'], fonts)

    current_step = min(s['step'], len(s['cmds'])) if s['step'] >= 0 else 0
    draw_chrome(screen, fonts, s['phase'], s['log_lines'], current_step, len(s['cmds']))


# ─── MAIN ─────────────────────────────────────────────────────────────────────
def main():
    try:
        print(f"Loading {LOG_PATH} ...")
        cmds = load_cmds(LOG_PATH)
        print(f"Loaded {len(cmds)} steps.")
    except FileNotFoundError as e:
        print(f"ERROR: {e}\nSet LOG_PATH at the top of the script.")
        sys.exit(1)
    except ValueError as e:
        print(f"ERROR parsing log: {e}")
        sys.exit(1)

    if VIDEO_EXPORT:
        # ── Offscreen render → video file ──────────────────────────────────────
        pygame.init()
        # Use a hidden display surface (no window needed)
        screen = pygame.Surface((W, H))

        fonts = build_fonts()

        Path(VIDEO_PATH).parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(
            VIDEO_PATH,
            cv2.VideoWriter_fourcc(*"mp4v"),
            VIDEO_FPS,
            (W, H),
        )
        if not writer.isOpened():
            print("ERROR: Could not open video writer. Check cv2 / codec.")
            sys.exit(1)

        # Quadrant crop regions: (x, y, w, h) in screen coords
        QUADS = {
            "cath_speed":   (TLx, TLy, QW, QH_T),
            "guide_rot":    (TRx, TRy, QW, QH_T),
            "geometry":     (BLx, BLy, QW, QH_B),
            "orientation":  (BRx, BRy, QW, QH_B),
        }

        quad_writers = {}
        if VIDEO_EXPORT_QUADRANTS:
            for name, (qx, qy, qw, qh) in QUADS.items():
                path = VIDEO_QUAD_PATH.format(name=name)
                qw_obj = cv2.VideoWriter(
                    path,
                    cv2.VideoWriter_fourcc(*"mp4v"),
                    VIDEO_FPS,
                    (qw, qh),
                )
                if not qw_obj.isOpened():
                    print(f"WARNING: Could not open quadrant writer for '{name}'. Skipping.")
                else:
                    quad_writers[name] = (qw_obj, qx, qy, qw, qh)
                    print(f"  Quadrant '{name}' → {path}  ({qw}×{qh})")

        dt        = 1.0 / VIDEO_FPS
        sim       = build_sim_state(cmds, fonts)
        frame_idx = 0
        # Total expected frames: clamp(1s) + steps*STEP_S + release(1s) + buffer
        total_est = int((1.0 + len(cmds) * STEP_S + 1.5) * VIDEO_FPS)

        print(f"Rendering ~{total_est} frames at {VIDEO_FPS} fps → {VIDEO_PATH}")
        still_running = True
        while still_running:
            still_running = sim_tick(sim, dt)
            render_frame(screen, sim)

            # Grab frame: pygame surface → numpy RGB → BGR for cv2
            arr = pygame.surfarray.array3d(screen)   # shape (W, H, 3)  x-first
            arr = arr.transpose(1, 0, 2)              # → (H, W, 3)
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            writer.write(arr)

            # Write cropped quadrant frames
            for qw_obj, qx, qy, qw, qh in quad_writers.values():
                qw_obj.write(arr[qy:qy + qh, qx:qx + qw])

            frame_idx += 1
            if frame_idx % VIDEO_FPS == 0:
                print(f"  {frame_idx}/{total_est} frames  "
                      f"({frame_idx/VIDEO_FPS:.1f}s)  phase={sim['phase']}", flush=True)

        writer.release()
        for qw_obj, *_ in quad_writers.values():
            qw_obj.release()
        pygame.quit()
        saved = [VIDEO_PATH] + [VIDEO_QUAD_PATH.format(name=n) for n in quad_writers]
        print(f"Done. Saved {frame_idx} frames → {', '.join(saved)}")

    else:
        # ── Interactive window ─────────────────────────────────────────────────
        pygame.init()
        screen = pygame.display.set_mode((W, H))
        pygame.display.set_caption("Guidewire Control Monitor — AVIAR")
        clock  = pygame.time.Clock()

        fonts = build_fonts()
        sim   = build_sim_state(cmds, fonts)

        def reset():
            nonlocal sim
            sim = build_sim_state(cmds, fonts)

        running = True
        while running:
            dt = clock.tick(60) / 1000.0

            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False
                if ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                    running = False
                if ev.type == pygame.MOUSEBUTTONDOWN and sim['phase'] in ("IDLE", "DONE"):
                    btn_rect = pygame.Rect(W // 2 - 70, QH_T + QH_B + 36, 140, 20)
                    if btn_rect.collidepoint(ev.pos):
                        reset()

            sim_tick(sim, dt)
            render_frame(screen, sim)
            pygame.display.flip()

        pygame.quit()
        sys.exit()


if __name__ == "__main__":
    main()