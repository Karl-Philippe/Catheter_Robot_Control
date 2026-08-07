import socket
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Optional, Sequence

# =========================
# NETWORK / ROBOT SETTINGS
# =========================
ROBOT_IP = "192.0.0.12"
ROBOT_PORT = 2020
LOCAL_BIND_PORT = 54111

CH_GUIDE = 2   # guidewire
CH_CATH  = 4   # catheter

# Control timestep: 0.1 s total
DT = 0.1
SUB_DT = 0.05   # 0.05 s catheter + 0.05 s guidewire

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

# =========================
# ENCODING
# =========================
def encode_translation_signed(speed_mm_s: float) -> int:
    s = float(speed_mm_s)
    if abs(s) < 1e-9:
        return 0
    mag = int(round(abs(s)))
    mag = max(1, min(TRANS_SPEED_MAX, mag))
    base = 100 if s > 0 else 200
    return base + mag


def encode_rotation_signed(rate_deg_s: float) -> tuple[int, float]:
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
# STEP COMMAND (per 0.1 s)
# =========================
@dataclass
class StepCmd:
    v2_mm_s: float   # CH2 translation speed (signed)
    w2_deg_s: float  # CH2 rotation rate (signed)
    v4_mm_s: float   # CH4 translation speed (signed)


# =========================
# LOG PARSER
# =========================
def iter_stepcmds_from_log(
    log_path: str | Path,
    dt_target: float = 0.1,
    assume_units: str = "rad",  # "rad" if guide_rotation_speed_cmd is rad/s; "deg" if deg/s
) -> Iterator[StepCmd]:
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

        i_t = idx("wall_time")
        i_w = idx("guide_rotation_speed_cmd")
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

            t = float(parts[i_t])
            w = float(parts[i_w])
            v2 = float(parts[i_v2])
            v4 = float(parts[i_v4])

            if assume_units == "rad":
                w = w * (180.0 / 3.141592653589793)

            if next_t is None:
                next_t = t

            if t >= next_t:
                yield StepCmd(v2_mm_s=v2, w2_deg_s=w, v4_mm_s=v4)
                steps = max(1, int((t - next_t) // dt_target) + 1)
                next_t = next_t + steps * dt_target


# =========================
# CONTROLLER
# =========================
class AVIAREnvController:
    """
    Each control step (0.1 s) is split into:
      1) CH4 catheter command for 0.05 s
      2) CH2 guidewire command for 0.05 s

    So they are NOT driven simultaneously.
    """

    def __init__(self, apply_krot: bool = True):
        self.apply_krot = apply_krot
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", LOCAL_BIND_PORT))
        self.msg = 1000

    def _send_frame(self, ch: int, clamp: int, trans_cmd: int, rot_cmd: int):
        self.msg += 1
        payload = f"{self.msg}/{ch}/{clamp}/{trans_cmd}/{rot_cmd}".encode("ascii")
        self.sock.sendto(payload, (ROBOT_IP, ROBOT_PORT))

    def _hold_channel(self, ch: int, clamp: int, trans_cmd: int, rot_cmd: int, duration_s: float):
        """
        Hold one channel command for a given duration.
        We repeatedly send the same frame during the hold window.
        """
        t_end = time.time() + duration_s
        while time.time() < t_end:
            self._send_frame(ch, clamp, trans_cmd, rot_cmd)
            # small resend interval inside the hold window
            time.sleep(0.01)

    def stop_all(self, duration_s: float = 0.3):
        t_end = time.time() + duration_s
        while time.time() < t_end:
            self._send_frame(CH_GUIDE, 0, 0, 0)
            self._send_frame(CH_CATH,  0, 0, 0)
            time.sleep(0.01)

    def clamp_both(self):
        t_end = time.time() + CLAMP_TIME_S
        while time.time() < t_end:
            # same time-sharing pattern even for clamp
            self._hold_channel(CH_CATH,  2, 0, 0, SUB_DT)
            self._hold_channel(CH_GUIDE, 2, 0, 0, SUB_DT)
        self.stop_all(SETTLE_S)

    def release_both(self):
        t_end = time.time() + RELEASE_TIME_S
        while time.time() < t_end:
            self._hold_channel(CH_CATH,  1, 0, 0, SUB_DT)
            self._hold_channel(CH_GUIDE, 1, 0, 0, SUB_DT)
        self.stop_all(SETTLE_S)

    def step(self, cmd: StepCmd):
        """
        One 0.1 s control step:
          - CH4 for 0.05 s
          - CH2 for 0.05 s
        """
        # CH4 catheter translation
        t4 = encode_translation_signed(cmd.v4_mm_s)

        # CH2 guidewire translation + rotation
        t2 = encode_translation_signed(cmd.v2_mm_s)
        w2 = cmd.w2_deg_s
        if self.apply_krot and abs(w2) > 1e-9:
            w2 = w2 / K_ROT
        r2, _ = encode_rotation_signed(w2)

        # First catheter for 0.05 s
        self._hold_channel(CH_CATH, 0, t4, 0, SUB_DT)

        # Then guidewire for 0.05 s
        self._hold_channel(CH_GUIDE, 0, t2, r2, SUB_DT)

    def close(self):
        try:
            self.stop_all(0.2)
        finally:
            self.sock.close()


# =========================
# RUNNERS
# =========================
def _run_stream(ctrl: AVIAREnvController, stream: Iterator[StepCmd], max_steps: Optional[int] = None):
    n = 0
    for sc in stream:
        t0 = time.time()
        ctrl.step(sc)
        n += 1
        if max_steps is not None and n >= max_steps:
            break

        dt_left = DT - (time.time() - t0)
        if dt_left > 0:
            time.sleep(dt_left)
    return n


def run_from_log(log_path: str, max_steps: Optional[int] = None, assume_units: str = "rad"):
    ctrl = AVIAREnvController(apply_krot=True)
    try:
        ctrl.stop_all(0.2)
        print("Clamping CH2 and CH4...")
        ctrl.clamp_both()

        n = _run_stream(
            ctrl,
            iter_stepcmds_from_log(log_path, dt_target=DT, assume_units=assume_units),
            max_steps=max_steps
        )

        ctrl.stop_all(0.5)
        print("Releasing CH2 and CH4...")
        ctrl.release_both()

        print(f"Done. Replayed {n} steps from {log_path}")
    finally:
        ctrl.close()


def run_from_cmds(cmds: Sequence[StepCmd], max_steps: Optional[int] = None):
    ctrl = AVIAREnvController(apply_krot=True)
    try:
        ctrl.stop_all(0.2)
        print("Clamping CH2 and CH4...")
        ctrl.clamp_both()

        n = _run_stream(ctrl, iter(cmds), max_steps=max_steps)

        ctrl.stop_all(0.5)
        print("Releasing CH2 and CH4...")
        ctrl.release_both()

        print(f"Done. Replayed {n} steps from cmds list")
    finally:
        ctrl.close()


# =========================
# MAIN
# =========================
if __name__ == "__main__":
    # Option A: replay from log file
    run_from_log("data/control_logs.txt", max_steps=None, assume_units="rad")

    # Option B: replay from explicit commands
    cmds = [
        StepCmd(v2_mm_s=+10, w2_deg_s=-90, v4_mm_s=+10),
        StepCmd(v2_mm_s=+10, w2_deg_s=-90, v4_mm_s=+10),
        StepCmd(v2_mm_s=+10, w2_deg_s=-30, v4_mm_s=+0),
        StepCmd(v2_mm_s=+10, w2_deg_s=0,   v4_mm_s=+0),
    ]
    #run_from_cmds(cmds)