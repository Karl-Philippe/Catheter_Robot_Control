"""
Guidewire Control Monitor — Python / pygame   (4-quadrant layout)

  TL : Catheter translation speed   (v4, arc dial)
  TR : Guidewire rotation speed     (w2, arc dial)
  BL : Catheter / guidewire geometry — straightening effect
  BR : Guidewire orientation (compass + accumulated rotation)

Requirements:  pip install pygame numpy opencv-python
Run:           python guidewire_panel.py
"""

import math, sys, time, pygame
import numpy as np
import cv2
from pathlib import Path
from dataclasses import dataclass
from typing import Iterator

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

# ─── RESOLUTION ───────────────────────────────────────────────────────────────
# Pick one label:
#   "SD"   720  x  480   fast preview
#   "HD"   1280 x  720
#   "FHD"  1920 x 1080   (default, good balance)
#   "2K"   2560 x 1440
#   "4K"   3840 x 2160
RESOLUTION = "FHD"

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

# ─── VIDEO EXPORT ─────────────────────────────────────────────────────────────
VIDEO_EXPORT           = True    # False = interactive pygame window
VIDEO_PATH             = "videos/guidewire_replay.mp4"
VIDEO_FPS              = 30
VIDEO_EXPORT_QUADRANTS = True    # also write one video per quadrant
VIDEO_QUAD_PATH        = "videos/guidewire_{name}.mp4"   # {name} filled with quadrant key

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

    def set(self, val, now):
        self.t0  = now
        self.tgt = float(val)

    def update(self, now):
        self.cur = lerp(self.cur, self.tgt, ease_inout((now - self.t0) / EASE_S))

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
    # 3-D projection:  x = rx + R*(1-cos t)*cos(guide_angle)
    #                  y = ry - R*sin(t)
    # (t=0 starts at catheter tip going straight upward; bend rotates with guide_angle)
    n_arc = 120
    arc_pts = []
    for i in range(n_arc + 1):
        t  = i / n_arc * bend
        px = rx + R_PX * (1 - math.cos(t)) * math.cos(g_rad)
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
    tdx = int(defl_mm * ins_scale * math.cos(g_rad))
    tdy = int(defl_mm * ins_scale * math.sin(g_rad))
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

        def make_font(size):
            for name in ["Courier New", "DejaVu Sans Mono", "monospace", None]:
                try:
                    return (pygame.font.SysFont(name, size) if name
                            else pygame.font.Font(None, size))
                except Exception:
                    pass
            return pygame.font.Font(None, size)

        fonts = {'tiny': make_font(int(11 * SCALE)), 'small': make_font(int(13 * SCALE)), 'val': make_font(int(26 * SCALE))}

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

        def make_font(size):
            for name in ["Courier New", "DejaVu Sans Mono", "monospace", None]:
                try:
                    return (pygame.font.SysFont(name, size) if name
                            else pygame.font.Font(None, size))
                except Exception:
                    pass
            return pygame.font.Font(None, size)

        fonts = {'tiny': make_font(int(11 * SCALE)), 'small': make_font(int(13 * SCALE)), 'val': make_font(int(26 * SCALE))}
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