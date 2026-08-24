"""
Single-channel (CH2 guidewire) control.

Two ways to drive the robot:
  1) Distance/angle API: translate_by(distance_mm), rotate_by(angle_deg)
  2) Speed+dt log replay: run_from_log(...) / step(StepCmd)

Encoding, calibration and the log parser are shared with dual_control.py via
robot_core.py.
"""

from dataclasses import dataclass
from typing import Iterator, Optional, Sequence

from robot_core import (
    CH_GUIDE,
    CLAMP_TIME_S,
    DEFAULT_DT,
    K_ROT,
    RELEASE_TIME_S,
    ROT_RATE_MAX,
    SETTLE_S,
    TRANS_SPEED_MAX,
    RobotLink,
    StepCmd,
    encode_rotation_signed,
    encode_translation_signed,
    iter_stepcmds_from_log,
)

# Channel this module drives
CH = CH_GUIDE

# Defaults for the distance/angle API
DEFAULT_TRANS_SPEED = 25      # mm/s
DEFAULT_ROT_SPEED = 90        # deg/s


@dataclass
class Params:
    clamp_time_s: float = CLAMP_TIME_S
    release_time_s: float = RELEASE_TIME_S
    settle_s: float = SETTLE_S
    k_rot: float = K_ROT
    default_trans_speed: float = DEFAULT_TRANS_SPEED
    default_rot_speed: float = DEFAULT_ROT_SPEED


class AVIARRobot:
    """
    Control API:
      - translate_by(distance_mm, speed_mm_s=None)  # signed distance, unsigned speed
      - rotate_by(angle_deg, rate_deg_s=None)       # signed angle, unsigned speed
      - step(StepCmd)                               # signed speeds held for cmd.dt

    Only CH2 is driven, so there is no time-sharing: a step holds the commanded
    speeds for the full dt and covers exactly v*dt / w*dt.
    """

    def __init__(self, params: Params = Params(), apply_krot: bool = True):
        self.p = params
        self.apply_krot = apply_krot
        self.link = RobotLink()

    def _send_frame(self, clamp: int, trans: int, rot: int):
        self.link.send_frame(CH, clamp, trans, rot)

    def _stream(self, duration_s: float, clamp: int, trans: int, rot: int):
        self.link.hold_channel(CH, clamp, trans, rot, duration_s)

    def stop(self, duration_s: float = 0.1):
        self._stream(duration_s, clamp=0, trans=0, rot=0)

    def clamp(self):
        self._stream(self.p.clamp_time_s, clamp=2, trans=0, rot=0)
        self.stop(self.p.settle_s)

    def release(self):
        self._stream(self.p.release_time_s, clamp=1, trans=0, rot=0)
        self.stop(self.p.settle_s)

    # -------------------------
    # Speed + duration (log replay)
    # -------------------------
    def step(self, cmd: StepCmd):
        """
        Hold cmd's guidewire speeds for cmd.dt.

        Translation and rotation go out in the SAME frame, so both run for the
        whole dt: the move covers v2*dt mm and w2*dt deg. cmd.v4_mm_s (catheter,
        CH4) is ignored here -- use dual_control.py to drive both channels.
        """
        t2 = encode_translation_signed(cmd.v2_mm_s)

        w2 = cmd.w2_deg_s
        if self.apply_krot and abs(w2) > 1e-9:
            w2 = w2 / self.p.k_rot
        r2, _ = encode_rotation_signed(w2)

        self._stream(float(cmd.dt), clamp=0, trans=t2, rot=r2)

    # -------------------------
    # Signed distance, unsigned speed
    # -------------------------
    def translate_by(self, distance_mm: float, speed_mm_s: float | None = None):
        """
        distance_mm: signed (positive=forward, negative=backward)
        speed_mm_s: unsigned magnitude (mm/s). If None, uses default_trans_speed.

        Open-loop duration: t = |distance| / speed
        """
        d = float(distance_mm)
        if abs(d) < 1e-9:
            return

        speed = self.p.default_trans_speed if speed_mm_s is None else float(speed_mm_s)
        if speed <= 0:
            raise ValueError("speed_mm_s must be > 0 (unsigned).")

        speed = max(1.0, min(float(TRANS_SPEED_MAX), speed))
        duration_s = abs(d) / speed

        cmd = encode_translation_signed(speed if d > 0 else -speed)

        print(f"[TRANS] {d:+.1f} mm @ {speed:.1f} mm/s -> cmd={cmd}, t={duration_s:.3f}s")
        self._stream(duration_s, clamp=0, trans=cmd, rot=0)
        self.stop(self.p.settle_s)

    def rotate_by(self, angle_deg: float, rate_deg_s: float | None = None):
        """
        angle_deg: signed (positive=CW, negative=CCW)
        rate_deg_s: unsigned magnitude (deg/s). If None, uses default_rot_speed.

        Protocol rate is quantized in steps of ~10 deg/s.
        Uses your calibration:
            theta_actual ~= k_rot * (omega_eff * t)
            => t = |theta_target| / (k_rot * omega_eff)
        """
        a = float(angle_deg)
        if abs(a) < 1e-9:
            return

        rate = self.p.default_rot_speed if rate_deg_s is None else float(rate_deg_s)
        if rate <= 0:
            raise ValueError("rate_deg_s must be > 0 (unsigned).")

        rate = min(rate, float(ROT_RATE_MAX))
        rot_cmd, omega_eff = encode_rotation_signed(rate if a > 0 else -rate)
        if rot_cmd == 0:
            return

        duration_s = abs(a) / (self.p.k_rot * omega_eff)

        print(f"[ROT] {a:+.1f} deg @ {rate:.1f} deg/s -> cmd={rot_cmd} "
              f"(omega_eff={omega_eff:.0f}), k_rot={self.p.k_rot:.3f}, t={duration_s:.3f}s")
        self._stream(duration_s, clamp=0, trans=0, rot=rot_cmd)
        self.stop(self.p.settle_s)

    def close(self):
        try:
            self.stop(0.2)
        finally:
            self.link.close()


# =========================
# RUNNERS
# =========================
def _run_stream(robot: AVIARRobot, stream: Iterator[StepCmd], max_steps: Optional[int] = None):
    n = 0
    for sc in stream:
        # step() already occupies the full sc.dt, so no extra pacing
        robot.step(sc)
        n += 1
        if max_steps is not None and n >= max_steps:
            break
    return n


def run_from_log(
    log_path: str,
    max_steps: Optional[int] = None,
    assume_units: str = "rad",
    default_dt: float = DEFAULT_DT,
):
    """Replay a log's guidewire columns on CH2. cath_speed_cmd is not required."""
    robot = AVIARRobot()
    try:
        robot.stop(0.2)
        print("Clamping CH2...")
        robot.clamp()

        n = _run_stream(
            robot,
            iter_stepcmds_from_log(log_path, default_dt=default_dt,
                                   assume_units=assume_units, require_cath=False),
            max_steps=max_steps
        )

        robot.stop(0.5)
        print("Releasing CH2...")
        robot.release()

        print(f"Done. Replayed {n} steps from {log_path}")
    finally:
        robot.close()


def run_from_cmds(cmds: Sequence[StepCmd], max_steps: Optional[int] = None):
    robot = AVIARRobot()
    try:
        robot.stop(0.2)
        print("Clamping CH2...")
        robot.clamp()

        n = _run_stream(robot, iter(cmds), max_steps=max_steps)

        robot.stop(0.5)
        print("Releasing CH2...")
        robot.release()

        print(f"Done. Replayed {n} steps from cmds list")
    finally:
        robot.close()


# =========================
# EXAMPLE
# =========================
def main():
    robot = AVIARRobot()

    try:
        print("Selecting channel (settle)...")
        robot.stop(0.2)

        print("Clamping...")
        robot.clamp()

        # Signed distance, unsigned speed (defaults to 25 mm/s)
        robot.translate_by(+40)           # 40 mm forward @ 25 mm/s
        robot.translate_by(-40, 35)       # 40 mm backward @ 35 mm/s

        # Signed angle, unsigned speed (defaults to 90 deg/s), with k_rot compensation
        robot.rotate_by(+180)             # 180 deg CW @ 90 deg/s (quantized), k_rot=1.5
        robot.rotate_by(-180, 360)        # 180 deg CCW @ 360 deg/s

        print("Releasing...")
        robot.release()

        print("Done.")
    finally:
        robot.close()


if __name__ == "__main__":
    main()

    # Or replay a log on CH2 (speeds + dt per row):
    #run_from_log("data/control_logs.txt", assume_units="rad", default_dt=DEFAULT_DT)
