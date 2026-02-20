import socket
import time
from dataclasses import dataclass

# =========================
# NETWORK / ROBOT SETTINGS
# =========================
ROBOT_IP = "192.0.0.12"
ROBOT_PORT = 2020
LOCAL_BIND_PORT = 54111

CH = 2
TX_HZ = 100
DT = 1.0 / TX_HZ

# =========================
# CALIBRATION (from your tests)
# =========================
TRANS_SPEED_MAX = 50          # allowed by your tests: 0..50 mm/s
K_ROT = 1.5                   # according to tested global rotation
DEFAULT_TRANS_SPEED = 25      # mm/s
DEFAULT_ROT_SPEED = 90        # deg/s

# Clamp/release timing (open-loop assumptions)
CLAMP_TIME_S = 2.5
RELEASE_TIME_S = 2.5
SETTLE_S = 0.2

# =========================
# COMMAND ENCODING
# =========================
def encode_translation(direction: str, speed_mm_s: float) -> int:
    """
    Protocol encoding:
      forward  => 100 + speed
      backward => 200 + speed
    speed is integer magnitude.
    """
    speed = float(speed_mm_s)
    if speed <= 0:
        return 0
    mag = int(round(speed))
    mag = max(1, min(TRANS_SPEED_MAX, mag))
    base = 100 if direction == "forward" else 200
    return base + mag


def encode_rotation(direction: str, rate_deg_s: float) -> tuple[int, float, int]:
    """
    Protocol encoding:
      cw  => 100 + r
      ccw => 200 + r
    where r = round(rate/10), clamped to 1..36

    Returns: (rot_cmd, omega_eff_deg_s, r)
    """
    rate = float(rate_deg_s)
    if rate <= 0:
        return 0, 0.0, 0
    r = int(round(rate / 10.0))
    r = max(1, min(36, r))
    base = 100 if direction == "cw" else 200
    cmd = base + r
    omega_eff = 10.0 * r
    return cmd, omega_eff, r


@dataclass
class Params:
    tx_hz: int = TX_HZ
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
    """

    def __init__(self, params: Params = Params()):
        self.p = params
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", LOCAL_BIND_PORT))
        self.msg = 1000
        self.dt = 1.0 / float(self.p.tx_hz)

    def _send_frame(self, clamp: int, trans: int, rot: int):
        self.msg += 1
        payload = f"{self.msg}/{CH}/{clamp}/{trans}/{rot}".encode("ascii")
        self.sock.sendto(payload, (ROBOT_IP, ROBOT_PORT))

    def _stream(self, duration_s: float, clamp: int, trans: int, rot: int):
        t_end = time.time() + float(duration_s)
        while time.time() < t_end:
            self._send_frame(clamp=clamp, trans=trans, rot=rot)
            time.sleep(self.dt)

    def stop(self, duration_s: float = 0.1):
        self._stream(duration_s, clamp=0, trans=0, rot=0)

    def clamp(self):
        self._stream(self.p.clamp_time_s, clamp=2, trans=0, rot=0)
        self.stop(self.p.settle_s)

    def release(self):
        self._stream(self.p.release_time_s, clamp=1, trans=0, rot=0)
        self.stop(self.p.settle_s)

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
        direction = "forward" if d > 0 else "backward"
        duration_s = abs(d) / speed

        cmd = encode_translation(direction, speed)

        print(f"[TRANS] {d:+.1f} mm @ {speed:.1f} mm/s -> dir={direction}, cmd={cmd}, t={duration_s:.3f}s")
        self._stream(duration_s, clamp=0, trans=cmd, rot=0)
        self.stop(self.p.settle_s)

    def rotate_by(self, angle_deg: float, rate_deg_s: float | None = None):
        """
        angle_deg: signed (positive=CW, negative=CCW)
        rate_deg_s: unsigned magnitude (deg/s). If None, uses default_rot_speed.

        Protocol rate is quantized in steps of ~10 deg/s.
        Uses your calibration:
            theta_actual ≈ k_rot * (omega_eff * t)
            => t = |theta_target| / (k_rot * omega_eff)
        """
        a = float(angle_deg)
        if abs(a) < 1e-9:
            return

        rate = self.p.default_rot_speed if rate_deg_s is None else float(rate_deg_s)
        if rate <= 0:
            raise ValueError("rate_deg_s must be > 0 (unsigned).")

        direction = "cw" if a > 0 else "ccw"
        rot_cmd, omega_eff, r = encode_rotation(direction, rate)
        if rot_cmd == 0:
            return

        duration_s = abs(a) / (self.p.k_rot * omega_eff)

        print(f"[ROT] {a:+.1f} deg @ {rate:.1f} deg/s -> dir={direction}, "
              f"cmd={rot_cmd} (r={r}, omega_eff={omega_eff:.0f}), k_rot={self.p.k_rot:.3f}, t={duration_s:.3f}s")
        self._stream(duration_s, clamp=0, trans=0, rot=rot_cmd)
        self.stop(self.p.settle_s)

    def close(self):
        try:
            self.stop(0.2)
        finally:
            self.sock.close()


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
