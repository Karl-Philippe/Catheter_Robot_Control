"""
The robot's wire language: how a physical speed becomes a command number, and
the StepCmd record that carries one row of motion.

Pure functions and data -- no sockets, no timing, no I/O. Everything here can
be unit-tested without a robot.

Command frame (see RobotLink):
    <msg_no>/<channel>/<clamp_cmd>/<translation_cmd>/<rotation_cmd>
"""

from dataclasses import dataclass

from .config import DEFAULT_DT, ROT_RATE_MAX, TRANS_SPEED_MAX

# Clamp field values
CLAMP_NONE = 0
CLAMP_RELEASE = 1
CLAMP_ENGAGE = 2


def encode_translation_signed(speed_mm_s: float) -> int:
    """
    Signed speed (mm/s) -> translation_cmd.

      forward  => 100 + |speed|
      backward => 200 + |speed|
      zero     => 0

    Magnitude is rounded and clamped to 1..TRANS_SPEED_MAX.
    """
    s = float(speed_mm_s)
    if abs(s) < 1e-9:
        return 0
    mag = int(round(abs(s)))
    mag = max(1, min(TRANS_SPEED_MAX, mag))
    base = 100 if s > 0 else 200
    return base + mag


def encode_rotation_signed(rate_deg_s: float) -> tuple[int, float]:
    """
    Signed rate (deg/s) -> (rotation_cmd, omega_eff_deg_s).

      cw  => 100 + r
      ccw => 200 + r

    where r = round(|rate| / 10) clamped to 1..36, so the achievable rate is
    quantised to omega_eff = 10 * r. The caller needs omega_eff to work out how
    long to hold the command for a target angle.
    """
    w = float(rate_deg_s)
    if abs(w) < 1e-9:
        return 0, 0.0
    w_mag = min(abs(w), ROT_RATE_MAX)
    r = int(round(w_mag / 10.0))
    r = max(1, min(36, r))
    base = 100 if w > 0 else 200
    return base + r, 10.0 * r


@dataclass
class StepCmd:
    """
    One row of motion: hold these speeds for `dt` seconds.

    v2/w2 are the guidewire (CH2) translation and rotation; v4 is the catheter
    (CH4) translation. A single-channel Robot ignores v4.
    """

    v2_mm_s: float          # CH2 translation speed (signed)
    w2_deg_s: float         # CH2 rotation rate (signed, deg/s)
    v4_mm_s: float          # CH4 translation speed (signed)
    dt: float = DEFAULT_DT  # how long to hold this command, in seconds
