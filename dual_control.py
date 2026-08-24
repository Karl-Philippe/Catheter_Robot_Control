"""
Dual-channel (CH2 guidewire + CH4 catheter) log replay.

Each log row is one robot move: hold the row's commanded speeds for the row's
dt. The two channels are time-shared rather than driven simultaneously, and
each gets the FULL dt so the commanded displacement is realised exactly --
see "Displacement" below.
"""

import time
from typing import Iterator, Optional, Sequence

from robot_core import (
    CH_CATH,
    CH_GUIDE,
    CLAMP_TIME_S,
    DEFAULT_DT,
    INTERLEAVE,
    K_ROT,
    RELEASE_TIME_S,
    SETTLE_S,
    TX_DT,
    RobotLink,
    StepCmd,
    encode_rotation_signed,
    encode_translation_signed,
    iter_stepcmds_from_log,
)

__all__ = ["AVIAREnvController", "StepCmd", "run_from_log", "run_from_cmds"]


# =========================
# CONTROLLER
# =========================
class AVIAREnvController:
    """
    Each control step of duration dt is time-shared between the channels. The
    step is cut into `interleave` slices per channel, alternating
    CH4/CH2/CH4/CH2..., each slice lasting dt/interleave.

    So they are NOT driven simultaneously, and a step occupies 2*dt of
    wall-clock regardless of `interleave`.

    Displacement: each channel is energised for a total of dt, so a row asking
    for v mm/s over dt advances v*dt mm on that channel, as recorded. Splitting
    dt in half between the channels instead would have moved only half the
    commanded distance on each.

    `interleave` therefore changes only HOW FINELY the channels alternate, not
    how far they travel:
      1 = one CH4 block then one CH2 block (default, coarsest)
      4 = 8 swaps per row, the two channels track each other far more closely
    Keep dt/interleave >= TX_DT or each slice degenerates to a single frame.
    """

    def __init__(self, apply_krot: bool = True, interleave: int = INTERLEAVE):
        self.apply_krot = apply_krot
        self.interleave = max(1, int(interleave))
        self.link = RobotLink()

    def _send_frame(self, ch: int, clamp: int, trans_cmd: int, rot_cmd: int):
        self.link.send_frame(ch, clamp, trans_cmd, rot_cmd)

    def _hold_channel(self, ch: int, clamp: int, trans_cmd: int, rot_cmd: int, duration_s: float):
        self.link.hold_channel(ch, clamp, trans_cmd, rot_cmd, duration_s)

    def stop_all(self, duration_s: float = 0.3):
        t_end = time.time() + duration_s
        while time.time() < t_end:
            self._send_frame(CH_GUIDE, 0, 0, 0)
            self._send_frame(CH_CATH,  0, 0, 0)
            time.sleep(TX_DT)

    def clamp_both(self):
        t_end = time.time() + CLAMP_TIME_S
        while time.time() < t_end:
            # same time-sharing pattern even for clamp
            self._hold_channel(CH_CATH,  2, 0, 0, DEFAULT_DT)
            self._hold_channel(CH_GUIDE, 2, 0, 0, DEFAULT_DT)
        self.stop_all(SETTLE_S)

    def release_both(self):
        t_end = time.time() + RELEASE_TIME_S
        while time.time() < t_end:
            self._hold_channel(CH_CATH,  1, 0, 0, DEFAULT_DT)
            self._hold_channel(CH_GUIDE, 1, 0, 0, DEFAULT_DT)
        self.stop_all(SETTLE_S)

    def step(self, cmd: StepCmd):
        """
        One control step, alternating CH4/CH2 `interleave` times:
          - CH4 for cmd.dt / interleave
          - CH2 for cmd.dt / interleave
          ... repeated `interleave` times
        Each channel accumulates cmd.dt in total; wall-clock is 2*cmd.dt.
        """
        dt = float(cmd.dt)

        # CH4 catheter translation
        t4 = encode_translation_signed(cmd.v4_mm_s)

        # CH2 guidewire translation + rotation
        t2 = encode_translation_signed(cmd.v2_mm_s)
        w2 = cmd.w2_deg_s
        if self.apply_krot and abs(w2) > 1e-9:
            w2 = w2 / K_ROT
        r2, _ = encode_rotation_signed(w2)

        slice_s = dt / self.interleave
        for _ in range(self.interleave):
            # catheter slice, then guidewire slice
            self._hold_channel(CH_CATH, 0, t4, 0, slice_s)
            self._hold_channel(CH_GUIDE, 0, t2, r2, slice_s)

    def close(self):
        try:
            self.stop_all(0.2)
        finally:
            self.link.close()


# =========================
# RUNNERS
# =========================
def _run_stream(ctrl: AVIAREnvController, stream: Iterator[StepCmd], max_steps: Optional[int] = None):
    n = 0
    for sc in stream:
        # step() already occupies 2*sc.dt (full dt per channel), so no extra pacing
        ctrl.step(sc)
        n += 1
        if max_steps is not None and n >= max_steps:
            break
    return n


def run_from_log(
    log_path: str,
    max_steps: Optional[int] = None,
    assume_units: str = "rad",
    default_dt: float = DEFAULT_DT,
    interleave: int = INTERLEAVE,
):
    ctrl = AVIAREnvController(apply_krot=True, interleave=interleave)
    try:
        ctrl.stop_all(0.2)
        print("Clamping CH2 and CH4...")
        ctrl.clamp_both()

        n = _run_stream(
            ctrl,
            iter_stepcmds_from_log(log_path, default_dt=default_dt, assume_units=assume_units),
            max_steps=max_steps
        )

        ctrl.stop_all(0.5)
        print("Releasing CH2 and CH4...")
        ctrl.release_both()

        print(f"Done. Replayed {n} steps from {log_path}")
    finally:
        ctrl.close()


def run_from_cmds(cmds: Sequence[StepCmd], max_steps: Optional[int] = None,
                  interleave: int = INTERLEAVE):
    ctrl = AVIAREnvController(apply_krot=True, interleave=interleave)
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
    # Option A: replay from log file.
    # Rows use the log's "dt" column; logs without one fall back to default_dt.
    # Raise interleave (e.g. 4) to alternate CH4/CH2 more finely within each row.
    run_from_log("data/control_logs.txt", max_steps=None, assume_units="rad",
                 default_dt=DEFAULT_DT, interleave=INTERLEAVE)

    # Option B: replay from explicit commands (dt is per-command)
    cmds = [
        StepCmd(v2_mm_s=+10, w2_deg_s=-90, v4_mm_s=+10, dt=0.1),
        StepCmd(v2_mm_s=+10, w2_deg_s=-90, v4_mm_s=+10, dt=0.1),
        StepCmd(v2_mm_s=+10, w2_deg_s=-30, v4_mm_s=+0,  dt=0.2),
        StepCmd(v2_mm_s=+10, w2_deg_s=0,   v4_mm_s=+0,  dt=0.5),
    ]
    #run_from_cmds(cmds)
