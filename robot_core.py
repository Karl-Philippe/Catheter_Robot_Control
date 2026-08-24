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

# Frame resend interval inside a hold window
TX_DT = 0.01

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
    default_dt: float = DEFAULT_DT,
    assume_units: str = "rad",  # "rad" if guide_rotation_speed_cmd is rad/s; "deg" if deg/s
    require_cath: bool = True,
) -> Iterator[StepCmd]:
    """
    Yield one StepCmd per data row. Each row is executed for its own duration,
    taken from the "dt" column when the log has one, else from default_dt.

    require_cath=False lets a guidewire-only consumer (control.py, CH2) replay a
    log that has no cath_speed_cmd column; v4_mm_s is then reported as 0.0.
    """
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

        def idx(name: str) -> int:
            if name not in header:
                raise ValueError(f"Missing column '{name}' in log header. Found: {header}")
            return header.index(name)

        i_w = idx("guide_rotation_speed_cmd")
        i_v2 = idx("guide_speed_cmd")
        i_v4 = idx("cath_speed_cmd") if require_cath else (
            header.index("cath_speed_cmd") if "cath_speed_cmd" in header else None
        )
        # "dt" is optional: logs recorded before it existed fall back to default_dt
        i_dt = header.index("dt") if "dt" in header else None

        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < len(header):
                continue

            w = float(parts[i_w])
            v2 = float(parts[i_v2])
            v4 = 0.0 if i_v4 is None else float(parts[i_v4])

            if assume_units == "rad":
                w = w * DEG_PER_RAD

            dt = default_dt if i_dt is None else float(parts[i_dt])
            if dt <= 0:
                continue

            yield StepCmd(v2_mm_s=v2, w2_deg_s=w, v4_mm_s=v4, dt=dt)


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
