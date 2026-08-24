"""
Shared core for the AVIAR catheter robot.

Holds the pieces that control.py (single channel, CH2) and dual_control.py
(CH2 + CH4) both need: network settings, calibration, command encoding, the
StepCmd record, the speed+dt log parser, and the UDP frame transport.
"""

import socket
import time
from dataclasses import dataclass
from math import pi
from pathlib import Path
from typing import Iterator

# =========================
# NETWORK / ROBOT SETTINGS
# =========================
ROBOT_IP = "192.0.0.12"
ROBOT_PORT = 2020
LOCAL_BIND_PORT = 54111

CH_GUIDE = 2   # guidewire
CH_CATH  = 4   # catheter

# Control timestep: used when a log row carries no "dt" column
DEFAULT_DT = 0.1

# Frame resend interval inside a hold window.
# The robot is driven by a continuous stream: a datagram is not a latched
# command, so a held command is re-sent every TX_DT (100 Hz) to keep the axis
# moving and to make a dropped datagram cost 10 ms instead of the whole move.
# This is the refresh rate, NOT the channel alternation rate (see INTERLEAVE).
TX_DT = 0.01

# How finely dual_control alternates between CH4 and CH2 within one log row.
# Each row's dt is cut into INTERLEAVE slices per channel, alternating
# CH4/CH2/CH4/CH2..., so the swap period is dt/INTERLEAVE instead of dt.
# Each channel still accumulates the full dt of motion, so displacement is
# unchanged -- only the interleaving granularity changes.
#   1  = one CH4 block then one CH2 block (coarsest, maximally out of phase)
#   4  = 8 swaps per row, the channels track each other much more closely
# A slice shorter than TX_DT degenerates to a single frame, so keep
# dt/INTERLEAVE >= TX_DT.
INTERLEAVE = 4

# =========================
# CALIBRATION / LIMITS
# =========================
TRANS_SPEED_MAX = 50        # mm/s magnitude
ROT_RATE_MAX = 360          # deg/s magnitude (protocol max)
K_ROT = 1.5                 # compensate rotation rate: w_cmd = w_des / K_ROT

# Clamp / release (open-loop)
CLAMP_TIME_S = 2.5
RELEASE_TIME_S = 2.5
SETTLE_S = 0.2

DEG_PER_RAD = 180.0 / pi


# =========================
# ENCODING
# =========================
def encode_translation_signed(speed_mm_s: float) -> int:
    """forward => 100 + |speed|, backward => 200 + |speed|, 0 => 0."""
    s = float(speed_mm_s)
    if abs(s) < 1e-9:
        return 0
    mag = int(round(abs(s)))
    mag = max(1, min(TRANS_SPEED_MAX, mag))
    base = 100 if s > 0 else 200
    return base + mag


def encode_rotation_signed(rate_deg_s: float) -> tuple[int, float]:
    """
    cw => 100 + r, ccw => 200 + r, where r = round(|rate|/10) clamped to 1..36.
    Returns (cmd, omega_eff_deg_s); the rate is quantized in ~10 deg/s steps.
    """
    w = float(rate_deg_s)
    if abs(w) < 1e-9:
        return 0, 0.0
    w_mag = min(abs(w), ROT_RATE_MAX)
    r = int(round(w_mag / 10.0))
    r = max(1, min(36, r))
    base = 100 if w > 0 else 200
    cmd = base + r
    omega_eff = 10.0 * r
    return cmd, omega_eff


# =========================
# STEP COMMAND (one log row)
# =========================
@dataclass
class StepCmd:
    v2_mm_s: float          # CH2 guidewire translation speed (signed)
    w2_deg_s: float         # CH2 guidewire rotation rate (signed)
    v4_mm_s: float          # CH4 catheter translation speed (signed)
    dt: float = DEFAULT_DT  # how long to hold this command, in seconds


# =========================
# LOG PARSER
# =========================
def iter_stepcmds_from_log(
    log_path: str | Path,
    trans_col: str,
    rot_col: str,
    cath_col: str | None = None,
    default_dt: float = DEFAULT_DT,
    assume_units: str = "rad",  # "rad" if the rotation column is rad/s, else "deg"
) -> Iterator[StepCmd]:
    """
    Yield one StepCmd per data row, held for the row's "dt" column (or
    default_dt when the log has none). Rows with dt <= 0 are skipped.

    The caller names the columns it wants, so each controller owns its own
    header:
      control.py       trans/rot only          -> v4_mm_s is 0.0
      dual_control.py  trans/rot + cath_col    -> both channels

    rot_col is converted rad/s -> deg/s when assume_units == "rad".
    """
    path = Path(log_path)
    if not path.exists():
        raise FileNotFoundError(f"Log not found: {path}")

    wanted = [trans_col, rot_col] + ([cath_col] if cath_col else [])

    with path.open("r", encoding="utf-8") as f:
        header = None
        for line in f:
            line = line.strip()
            if line:
                header = line.split()
                break
        if header is None:
            return

        for name in wanted:
            if name not in header:
                raise ValueError(f"Missing column '{name}' in log header. Found: {header}")
        i_trans, i_rot = header.index(trans_col), header.index(rot_col)
        i_cath = header.index(cath_col) if cath_col else None
        # "dt" is optional: logs recorded before it existed fall back to default_dt
        i_dt = header.index("dt") if "dt" in header else None

        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < len(header):
                continue

            dt = default_dt if i_dt is None else float(parts[i_dt])
            if dt <= 0:
                continue

            w = float(parts[i_rot])
            if assume_units == "rad":
                w *= DEG_PER_RAD

            yield StepCmd(
                v2_mm_s=float(parts[i_trans]),
                w2_deg_s=w,
                v4_mm_s=0.0 if i_cath is None else float(parts[i_cath]),
                dt=dt,
            )


# =========================
# TRANSPORT
# =========================
class RobotLink:
    """
    UDP frame transport. Frames are "<msg>/<ch>/<clamp>/<trans>/<rot>", resent
    every TX_DT while a command is held.
    """

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", LOCAL_BIND_PORT))
        self.msg = 1000

    def send_frame(self, ch: int, clamp: int, trans_cmd: int, rot_cmd: int):
        self.msg += 1
        payload = f"{self.msg}/{ch}/{clamp}/{trans_cmd}/{rot_cmd}".encode("ascii")
        self.sock.sendto(payload, (ROBOT_IP, ROBOT_PORT))

    def hold_channel(self, ch: int, clamp: int, trans_cmd: int, rot_cmd: int, duration_s: float):
        """Hold one channel command for duration_s, resending every TX_DT."""
        t_end = time.time() + float(duration_s)
        # always send at least one frame, even for a very short hold window
        while True:
            self.send_frame(ch, clamp, trans_cmd, rot_cmd)
            if time.time() >= t_end:
                break
            time.sleep(min(TX_DT, max(0.0, t_end - time.time())))

    def close(self):
        self.sock.close()
