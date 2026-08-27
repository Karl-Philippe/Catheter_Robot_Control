"""
Drawing primitives for the AVIAR panels.

Pure rendering: every function takes a target surface plus plain values and
draws on it. Nothing here reads a log, opens a socket, or holds run state, so
both the offline replay panel (guidewire_panel.py) and the live panel
(live_panel.py) draw with the same code.

Layout is a 940x646 design canvas scaled to the chosen RESOLUTION, split into
four quadrants with a status strip beneath.
"""

import math
import os

import pygame

# ─── PHYSICAL CONSTANTS (geometry view) ───────────────────────────────────────
R_MM = 20.0                 # guidewire radius of curvature (mm)
VESSEL_RADIUS_MM = 6.5      # vessel inner half-width (mm)
CATHETER_RADIUS_MM = 3.8    # catheter half-width (mm)

# Seconds for a dial needle to ease toward a new value. The offline exporter
# replays at 0.2 s per step so 0.35 looks smooth there; a live run changes the
# command every dt (0.1 s), where 0.35 is too slow to keep up -- the needle
# reaches only ~92% of the peak. aviar.livepanel lowers this at import.
EASE_S = 0.35

# ─── RESOLUTION ───────────────────────────────────────────────────────────────
# Pick one label:
#   "SD"   720  x  480   fast preview
#   "HD"   1280 x  720
#   "FHD"  1920 x 1080   (default, good balance)
#   "2K"   2560 x 1440
#   "4K"   3840 x 2160
# Set AVIAR_RESOLUTION to pick a size without editing this file. The live panel
# uses it to render smaller: an FHD frame costs ~8.7 ms of CPU against ~1.1 ms
# at HD, and that work is held under the GIL, which delays the robot thread's
# 10 ms wakeups. Offline export ignores this and stays at FHD.
RESOLUTION = os.environ.get("AVIAR_RESOLUTION", "FHD")

_PRESETS = {
    "SD":  ( 720,  480),
    "HD":  (1280,  720),
    "FHD": (1920, 1080),
    "2K":  (2560, 1440),
    "4K":  (3840, 2160),
}
_BASE_W, _BASE_H = 940, 646          # original design canvas
_TW, _TH = _PRESETS[RESOLUTION]
SCALE     = min(_TW / _BASE_W, _TH / _BASE_H)
GEO_SCALE = 3.6 * SCALE

# ─── WINDOW / LAYOUT ──────────────────────────────────────────────────────────
W    = _TW
H    = _TH
QW   = W // 2
QH_T = int(268 * SCALE)
QH_B = int(318 * SCALE)
PH   = H - QH_T - QH_B          # remaining height for progress panel

# Quadrant absolute origins (derived, do not edit)
TLx, TLy = 0,   0
TRx, TRy = QW,  0
BLx, BLy = 0,   QH_T
BRx, BRy = QW,  QH_T

# ─── PALETTE ──────────────────────────────────────────────────────────────────
BG         = (4,   11,  11)
TEAL       = (0,  255, 200)
TEAL_DK    = (13,  32,  32)
TEAL_DIM   = (0,   70,  55)
ORANGE     = (255, 159,  28)
GREY       = (90, 138, 138)
GREY_DK    = (26,  64,  64)
SILVER     = (160, 180, 190)
SILVER_DK  = (45,  65,  75)
VESSEL_BG  = (7,   18,  18)
VESSEL_WALL= (18,  50,  50)
GREEN      = (80,  255, 120)
DIV_COL    = (16,  42,  42)

# ─── FONTS ────────────────────────────────────────────────────────────────────
_FONT_CHAIN = ["Courier New", "DejaVu Sans Mono", "monospace", None]


def _font(size):
    for name in _FONT_CHAIN:
        try:
            return pygame.font.SysFont(name, size) if name else pygame.font.Font(None, size)
        except Exception:
            continue
    return pygame.font.Font(None, size)


def build_fonts():
    """The three sizes every panel uses, scaled to RESOLUTION."""
    return {
        "tiny":  _font(int(11 * SCALE)),
        "small": _font(int(13 * SCALE)),
        "val":   _font(int(26 * SCALE)),
    }

# ─── MATHS & DRAWING HELPERS ──────────────────────────────────────────────────
def lerp(a, b, t):
    return a + (b - a) * max(0.0, min(1.0, t))


def lerp_color(c1, c2, t):
    t = max(0.0, min(1.0, t))
    return tuple(int(a + (b - a) * t) for a, b in zip(c1, c2))


def ease_inout(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def polar(cx, cy, r, deg):
    a = math.radians(deg - 90)
    return cx + r * math.cos(a), cy + r * math.sin(a)


def _glow_surf(surf):
    return pygame.Surface(surf.get_size(), pygame.SRCALPHA)


def _needle_polygon(p1, p2, half_w):
    """Return a tapered 4-point polygon: wide at base, sharp at tip."""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    length = math.hypot(dx, dy) or 1
    nx, ny = -dy / length, dx / length   # normal
    b1 = (p1[0] + nx * half_w, p1[1] + ny * half_w)
    b2 = (p1[0] - nx * half_w, p1[1] - ny * half_w)
    return [b1, p2, b2]


def glow_line(surf, col, p1, p2, w, layers=3):
    half = max(1, w)
    pts = _needle_polygon(p1, p2, half)
    pygame.draw.polygon(surf, col, pts)
    pygame.draw.aalines(surf, col, True, pts)


def glow_circle(surf, col, cen, r, w=0, layers=3):
    for i in range(layers, 0, -1):
        s = _glow_surf(surf)
        if w == 0:
            pygame.draw.circle(s, (*col[:3], int(90 / i)), cen, r + i * 2)
        else:
            pygame.draw.circle(s, (*col[:3], int(90 / i)), cen, r + i * 2, w + i)
        surf.blit(s, (0, 0))
    pygame.draw.circle(surf, col, cen, r, w)


def glow_polyline(surf, col, pts, w, alpha=45, layers=2):
    if len(pts) < 2:
        return
    for i in range(layers, 0, -1):
        s = _glow_surf(surf)
        pygame.draw.lines(s, (*col[:3], int(alpha / i)), False, pts, w + i * 4)
        surf.blit(s, (0, 0))
    pygame.draw.lines(surf, col, False, pts, w)


# ─── ARC DIAL ─────────────────────────────────────────────────────────────────
class Dial:
    START = -135
    SPAN  =  270

    def __init__(self, cx, cy, R, label, unit, vmin, vmax, col=TEAL):
        self.cx, self.cy, self.R = cx, cy, R
        self.label, self.unit = label, unit
        self.vmin, self.vmax = vmin, vmax
        self.col = col
        self.cur = 0.0
        self.tgt = 0.0
        self.t0  = 0.0

    def set(self, val, now, restart_on_repeat=True):
        """
        Aim the needle at `val`, easing from wherever it is now.

        With `restart_on_repeat` (the default) this always restarts the ease,
        which is what the offline exporter wants: it calls set() once per log
        step, so each step re-eases toward the value.

        A live panel instead pushes the current reading EVERY frame. There the
        restart is fatal -- `now - t0` is always 0, ease_inout(0) is 0, and the
        needle stays pinned at its starting value forever. Those callers pass
        restart_on_repeat=False so an unchanged value is a no-op.
        """
        val = float(val)
        if restart_on_repeat:
            # original behaviour, byte-for-byte: restart the ease from the
            # needle's stored position.
            self.t0 = now
            self.tgt = val
            return
        if val == self.tgt:
            return
        self.cur = self._eased(now)   # continue from where the needle actually is
        self.t0 = now
        self.tgt = val

    def _eased(self, now):
        return lerp(self.cur, self.tgt, ease_inout((now - self.t0) / EASE_S))

    def update(self, now):
        self.cur = self._eased(now)

    def _ang(self, v):
        return self.START + (v - self.vmin) / (self.vmax - self.vmin) * self.SPAN

    def draw(self, surf, fs, fv):
        cx, cy, R, col = self.cx, self.cy, self.R, self.col
        v = self.cur

        # Dark circle background
        pygame.draw.circle(surf, VESSEL_BG, (cx, cy), int(R + 8 * SCALE))

        # Cut out the bottom gap (the 90° span not covered by the dial)
        gap_start = self.START + self.SPAN   # +135°
        gap_end   = self.START + 360         # +225°  (= -135° + 360)
        gap_pts   = [(cx, cy)]
        for gi in range(21):
            ga = gap_start + gi * (gap_end - gap_start) / 20
            gap_pts.append(tuple(int(x) for x in polar(cx, cy, R + 12 * SCALE, ga)))
        pygame.draw.polygon(surf, BG, gap_pts)

        # Ticks — only inside the 270° dial span
        for i in range(72):
            a     = i * 5.0
            rel = (a - (self.START + 180)) % 360
            in_span = rel <= (self.SPAN + 10)
            if not in_span:
                continue
            major = (i % 6 == 0)   # every 30°
            med   = (i % 3 == 0)   # every 15°
            r1    = R * (0.74 if major else (0.80 if med else 0.88))
            p1    = tuple(int(x) for x in polar(cx, cy, r1, a))
            p2    = tuple(int(x) for x in polar(cx, cy, R * 0.97, a))
            tick_col = (55, 110, 110) if major else ((38, 80, 80) if med else GREY_DK)
            pygame.draw.line(surf, tick_col, p1, p2, 2 if major else 1)

        # Outer arc border — 270° only (no ring at the gap)
        arc_border = [tuple(int(x) for x in polar(cx, cy, R + 8 * SCALE,
                            self.START + i * self.SPAN / 270))
                      for i in range(271)]
        pygame.draw.lines(surf, GREY_DK, False, arc_border, 1)

        # Tick labels at major positions
        for i in [0, 5, 10, 15, 20, 25, 30]:
            a  = self.START + i / 30 * self.SPAN
            lv = self.vmin + i / 30 * (self.vmax - self.vmin)
            px, py = polar(cx, cy, R * 0.60, a)
            t = fs.render(str(int(lv)), True, GREY_DK)
            surf.blit(t, t.get_rect(center=(int(px), int(py))))

        # Active arc (zero → value), drawn inside the ring
        a0 = min(self._ang(0), self._ang(v))
        a1 = max(self._ang(0), self._ang(v))
        if abs(v) > 0.3:
            n   = max(2, int(abs(a1 - a0) / 2))
            arc = [(int(x), int(y))
                   for x, y in (polar(cx, cy, R * 0.90, a0 + i / n * (a1 - a0))
                                 for i in range(n + 1))]
            if len(arc) > 1:
                pygame.draw.lines(surf, col, False, arc, max(1, int(3 * SCALE)))

        # Needle
        na   = self._ang(v)
        tip  = tuple(int(x) for x in polar(cx, cy, R * 0.86, na))
        base = tuple(int(x) for x in polar(cx, cy, R * 0.16, na + 180))
        glow_line(surf, col, base, tip, max(1, int(2 * SCALE)))
        pygame.draw.circle(surf, VESSEL_BG, (cx, cy), int(7 * SCALE))
        pygame.draw.circle(surf, col, (cx, cy), int(7 * SCALE), 2)
        pygame.draw.circle(surf, col, (cx, cy), int(3 * SCALE))

        # Value
        sign = "+" if v >= 0 else ""
        vt = fv.render(f"{sign}{v:.0f}", True, col)
        surf.blit(vt, vt.get_rect(center=(cx, cy + int(R * 0.42))))
        ut = fs.render(self.unit, True, GREY)
        surf.blit(ut, ut.get_rect(center=(cx, cy + int(R * 0.60))))
        lt = fs.render(self.label.upper(), True, GREY_DK)
        surf.blit(lt, lt.get_rect(center=(cx, cy + R + 20)))


# ─── GEOMETRY QUADRANT (BL) ───────────────────────────────────────────────────
def draw_geometry(surf, cath_pos, guide_pos, guide_angle_deg, cur_v2, fonts):
    """
    BL quadrant: side-view of catheter (straight) + guidewire (curved arc).
    The free length of guidewire beyond the catheter tip = max(0, guide_pos-cath_pos)
    maps to bend_angle = free_len / R_MM  (capped at π).
    The bend plane rotates with the accumulated guide_angle.
    """
    R_PX = R_MM * GEO_SCALE           # e.g. 72 px
    g_rad = math.radians(guide_angle_deg)
    rel   = guide_pos - cath_pos       # mm  (can be negative → catheter leads)
    rel_c = max(0.0, rel)              # clamped free length ≥ 0
    bend  = min(rel_c / R_MM, math.pi) # radians [0 … π]

    # Reference: catheter tip position in absolute screen coords
    rx = BLx + QW // 2                        # vessel centre x  (= 235)
    ry = BLy + int(QH_B * 0.60)              # catheter tip y   (= 268 + 190 = 458)

    # ── Vessel channel ────────────────────────────────────────────────────────
    vhw = int(VESSEL_RADIUS_MM   * GEO_SCALE)   # vessel half-width in px
    chw = int(CATHETER_RADIUS_MM * GEO_SCALE)   # catheter half-width in px

    # Outer wall
    pygame.draw.rect(surf, VESSEL_WALL,
                     (rx - vhw - 4, BLy, (vhw + 4) * 2, QH_B))
    # Lumen
    pygame.draw.rect(surf, VESSEL_BG,
                     (rx - vhw, BLy, vhw * 2, QH_B))
    # Wall highlight lines
    for sx in [rx - vhw, rx + vhw]:
        pygame.draw.line(surf, (30, 60, 60), (sx, BLy), (sx, BLy + QH_B), 1)

    # ── Guidewire inside catheter (dim teal line from bottom to catheter tip) ──
    pygame.draw.line(surf, TEAL_DIM,
                     (rx, BLy + QH_B - 5), (rx, ry), 2)

    # ── Catheter body ─────────────────────────────────────────────────────────
    pygame.draw.rect(surf, SILVER_DK,
                     (rx - chw, ry, chw * 2, BLy + QH_B - ry))
    # Side rails
    for sx in [rx - chw, rx + chw]:
        pygame.draw.line(surf, SILVER, (sx, ry), (sx, BLy + QH_B), 1)
    # Tip cap (bright edge)
    pygame.draw.rect(surf, SILVER, (rx - chw, ry - 1, chw * 2, 3))
    # Interior shadow
    pygame.draw.rect(surf, (55, 75, 85),
                     (rx - chw + 2, ry + 3, chw * 2 - 4, BLy + QH_B - ry - 6))

    # ── Guidewire curved arc (beyond catheter tip) ────────────────────────────
    # 3-D projection:  x = rx + R*(1-cos t)*sin(guide_angle)
    #                  y = ry - R*sin(t)
    # t=0 starts at the catheter tip going straight up; the bend rotates with
    # guide_angle. sin() (not cos) so this matches the compass, where 0 deg is
    # 12 o'clock: at 0 deg the bend points AWAY from the viewer and projects to
    # a straight vertical line, and at 90 deg it deflects fully to the right.
    n_arc = 120
    arc_pts = []
    for i in range(n_arc + 1):
        t  = i / n_arc * bend
        px = rx + R_PX * (1 - math.cos(t)) * math.sin(g_rad)
        py = ry - R_PX * math.sin(t)
        arc_pts.append((int(px), int(py)))

    if bend > 0.03:
        glow_polyline(surf, TEAL, arc_pts, 2, alpha=40, layers=2)
        tip_pt = arc_pts[-1]
    else:
        # Essentially straight — tiny nub above catheter tip
        tip_pt = (rx, ry - 12)
        pygame.draw.line(surf, TEAL, (rx, ry), tip_pt, 2)

    # Guidewire tip glow + dot
    s = _glow_surf(surf)
    pygame.draw.circle(s, (*TEAL, 55), tip_pt, 12)
    surf.blit(s, (0, 0))
    pygame.draw.circle(surf, TEAL, tip_pt, 4)

    # ── Labels ────────────────────────────────────────────────────────────────
    fs = fonts['small']
    ft = fonts['tiny']
    ct = fs.render("catheter tip ►", True, SILVER)
    surf.blit(ct, (rx + chw + 6, ry - 8))
    gt = fs.render("guide tip ►", True, TEAL)
    surf.blit(gt, (tip_pt[0] + 6, tip_pt[1] - 7))

    # ── Measurement arrow (free guidewire length along vessel axis) ───────────
    if rel_c > 0.5:
        arr_x  = rx - chw - 24
        tip_axis_y = ry - int(R_PX * math.sin(bend))   # projected onto vessel axis
        # Vertical bracket line
        pygame.draw.line(surf, GREY_DK, (arr_x, ry), (arr_x, tip_axis_y), 1)
        for y_e in [ry, tip_axis_y]:
            pygame.draw.line(surf, GREY_DK, (arr_x - 4, y_e), (arr_x + 4, y_e), 1)
        mid_y = (ry + tip_axis_y) // 2
        lbl_m = ft.render(f"{rel_c:.1f} mm", True, GREY_DK)
        surf.blit(lbl_m, (arr_x - lbl_m.get_width() - 2, mid_y - 6))

    # ── Catheter-leads indicator ───────────────────────────────────────────────
    if rel < -0.5:
        warn = ft.render(f"cath. leads by {abs(rel):.1f} mm", True, ORANGE)
        surf.blit(warn, warn.get_rect(center=(BLx + QW // 2, ry + 14)))

    # ── Straightening progress bar ────────────────────────────────────────────
    max_free  = R_MM * math.pi / 2     # 31.4 mm = full 90° bend
    spct      = max(0.0, 1.0 - rel_c / max_free)
    bar_w, bar_h = int(130 * SCALE), int(8 * SCALE)
    bar_x = BLx + (QW - bar_w) // 2
    bar_y = BLy + QH_B - 48
    pygame.draw.rect(surf, GREY_DK, (bar_x, bar_y, bar_w, bar_h), border_radius=4)
    fill = int(bar_w * min(1.0, spct))
    if fill > 0:
        pygame.draw.rect(surf, lerp_color(ORANGE, GREEN, spct),
                         (bar_x, bar_y, fill, bar_h), border_radius=4)
    lbl_sp = ft.render(f"straightened: {int(min(spct, 1.0) * 100)}%", True, GREY)
    surf.blit(lbl_sp, lbl_sp.get_rect(center=(BLx + QW // 2, bar_y - 11)))
    bend_lbl = ft.render(f"bend  {math.degrees(bend):.0f}°   free  {rel_c:.1f} mm", True, GREY_DK)
    surf.blit(bend_lbl, bend_lbl.get_rect(center=(BLx + QW // 2, bar_y + bar_h + 10)))

    # ── End-on (cross-section) inset — top-right of BL ───────────────────────
    # Shows the guidewire tip deflection when looking down the vessel axis.
    # x_cross = (1-cos bend)*R * cos(guide_angle)
    # y_cross = (1-cos bend)*R * sin(guide_angle)
    ins_cx, ins_cy, ins_r = BLx + QW - int(48 * SCALE), BLy + int(50 * SCALE), int(30 * SCALE)
    pygame.draw.circle(surf, VESSEL_WALL, (ins_cx, ins_cy), ins_r + 4)
    pygame.draw.circle(surf, VESSEL_BG,   (ins_cx, ins_cy), ins_r)
    pygame.draw.circle(surf, GREY_DK,     (ins_cx, ins_cy), ins_r, 1)
    # Catheter cross-section
    pygame.draw.circle(surf, SILVER_DK, (ins_cx, ins_cy), int(chw * 0.75))
    # Guide tip deflection dot
    defl_mm  = (1.0 - math.cos(bend)) * R_MM
    ins_scale = ins_r / (2.0 * R_MM)
    # 0 deg up, 90 deg right -- same convention as the compass and the arc above
    tdx = int(defl_mm * ins_scale * math.sin(g_rad))
    tdy = int(-defl_mm * ins_scale * math.cos(g_rad))
    tip_ins = (ins_cx + tdx, ins_cy + tdy)
    if tip_ins != (ins_cx, ins_cy):
        pygame.draw.line(surf, TEAL_DIM, (ins_cx, ins_cy), tip_ins, 1)
    pygame.draw.circle(surf, TEAL, tip_ins, 3)
    lbl_ins = ft.render("end-on", True, GREY_DK)
    surf.blit(lbl_ins, lbl_ins.get_rect(center=(ins_cx, ins_cy + ins_r + 10)))

    # ── Guide push speed (v2) readout at bottom ───────────────────────────────
    v2_t = ft.render(f"guide push (v2): {cur_v2:+.0f} mm/s", True, GREY_DK)
    surf.blit(v2_t, v2_t.get_rect(center=(BLx + QW // 2, BLy + QH_B - 12)))


# ─── ROTATION QUADRANT (BR) ───────────────────────────────────────────────────
def draw_rotation(surf, guide_angle_deg, cur_w2, cur_v2, fonts):
    """
    BR quadrant: compass showing accumulated guidewire rotation + v2 bar.
    """
    fs = fonts['small']
    ft = fonts['tiny']
    fv = fonts['val']

    cx  = BRx + QW // 2             # compass centre x  (= 470 + 235 = 705)
    cy  = BRy + int(QH_B * 0.44)   # compass centre y  (= 268 + 140 = 408)
    r   = int(102 * SCALE)           # compass radius px

    # ── Compass background ────────────────────────────────────────────────────
    pygame.draw.circle(surf, VESSEL_BG, (cx, cy), r + 8)

    # ── Tick marks ────────────────────────────────────────────────────────────
    for i in range(72):         # one tick every 5°
        a     = i * 5.0
        major = (i % 18 == 0)  # every 90°
        med   = (i % 9  == 0)  # every 45°
        r1    = r * (0.74 if major else (0.80 if med else 0.88))
        p1    = tuple(int(x) for x in polar(cx, cy, r1, a))
        p2    = tuple(int(x) for x in polar(cx, cy, r * 0.97, a))
        col   = (55, 110, 110) if major else ((38, 80, 80) if med else GREY_DK)
        pygame.draw.line(surf, col, p1, p2, 2 if major else 1)

    # Cardinal labels
    for a, lbl in [(0, "0"), (90, "90"), (180, "180"), (270, "270")]:
        px, py = polar(cx, cy, r * 0.62, a)
        t = ft.render(lbl + "°", True, GREY_DK)
        surf.blit(t, t.get_rect(center=(int(px), int(py))))

    pygame.draw.circle(surf, GREY_DK, (cx, cy), r, 1)

    cur_mod = guide_angle_deg % 360

    # ── Needle ────────────────────────────────────────────────────────────────
    tip  = tuple(int(x) for x in polar(cx, cy, r * 0.86, cur_mod))
    base = tuple(int(x) for x in polar(cx, cy, r * 0.14, cur_mod + 180))
    glow_line(surf, TEAL, base, tip, 2)
    pygame.draw.circle(surf, VESSEL_BG, (cx, cy), 7)
    pygame.draw.circle(surf, TEAL, (cx, cy), 7, 2)
    pygame.draw.circle(surf, TEAL, (cx, cy), 3)

    # ── Accumulated angle readout ─────────────────────────────────────────────
    sign = "+" if guide_angle_deg >= 0 else ""
    av   = fv.render(f"{sign}{guide_angle_deg:.0f}°", True, TEAL)
    surf.blit(av, av.get_rect(center=(cx, cy + int(r * 0.42))))


    # ── v2 translation speed bar ──────────────────────────────────────────────
    bx0    = BRx + int(30 * SCALE)
    by0    = BRy + int(QH_B * 0.86)
    bw0    = QW - int(60 * SCALE)
    bh0    = int(10 * SCALE)
    mid_bx = bx0 + bw0 // 2
    pygame.draw.rect(surf, TEAL_DK, (bx0, by0, bw0, bh0), border_radius=5)
    frac   = min(1.0, abs(cur_v2) / 50.0)
    fill   = int(bw0 / 2 * frac)
    if fill > 0:
        if cur_v2 > 0:
            pygame.draw.rect(surf, TEAL, (mid_bx, by0, fill, bh0), border_radius=5)
        else:
            pygame.draw.rect(surf, TEAL, (mid_bx - fill, by0, fill, bh0), border_radius=5)
    pygame.draw.line(surf, GREY, (mid_bx, by0 - 3), (mid_bx, by0 + bh0 + 3), 1)
    v2_lbl = ft.render(f"guide translation (v2): {cur_v2:+.0f} mm/s", True, GREY)
    surf.blit(v2_lbl, v2_lbl.get_rect(center=(mid_bx, by0 - 12)))

