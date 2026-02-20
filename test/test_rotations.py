"""
Rotation accuracy test (constant angle = 180 deg, variable speed) WITH k_r compensation,
written in the SAME style as your translation sweep script (same clamp/release behavior).

For each requested rotation rate (deg/s):
- Encode to r=round(rate/10) (1..36) => effective_rate = 10*r deg/s
- Look up k_r for that r and direction (cw/ccw). Default k_r=1.0 if not provided.
- Compute duration to target 180 deg using:
      duration = TARGET_ANGLE_DEG / (k_r * effective_rate)
- Rotate CW then CCW at that rate
- Log to CSV with k_r used + durations + placeholders for measured angle

How to fill k_r:
- After a run, measure the actual angle for each (direction, r).
- k_r(direction, r) = measured_angle / TARGET_ANGLE_DEG
  (Example: if you commanded 180° and measured 165°, k_r = 165/180 = 0.9167)
- Put that value in K_R_CW or K_R_CCW for the corresponding r.
"""

import csv
import socket
import time
from pathlib import Path

ROBOT_IP = "192.0.0.12"
ROBOT_PORT = 2020
LOCAL_BIND_PORT = 54111

CHANNEL = 2
TX_HZ = 100
DT = 1.0 / TX_HZ

CLAMP_TIME_S = 2.5
RELEASE_TIME_S = 2.5
SETTLE_S = 0.2
INTER_TEST_PAUSE_S = 1.0

TARGET_ANGLE_DEG = 180.0

# Requested rates (deg/s). These will quantize to steps of ~10 deg/s by encoding.
RATES_DEG_S = [30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330, 360]

RESULTS_CSV = Path("rotation_180deg_accuracy_vs_rate_with_kr.csv")

# ---------------------------------------------------------------------
# k_r calibration tables (indexed by encoded r = round(rate/10), 1..36)
# Fill these after measuring. Defaults to 1.0 if missing.
# Example:
#   At requested 90 deg/s -> r=9, effective_rate=90 deg/s
#   If CW measured 165 deg for a 180-deg command => k_r = 165/180 = 0.9167
#   Put: K_R_CW[9] = 0.9167
# ---------------------------------------------------------------------
K_R_CW = {
    # 9: 0.9167,
}
K_R_CCW = {
    # 9: 0.9500,
}
DEFAULT_KR = 1.5


def encode_rotation(direction: str, rate_deg_s: float) -> tuple[int, int, float]:
    """
    Protocol encoding (from your doc):
      r = round(rate/10), clamped to 1..36
      cw  => 100 + r
      ccw => 200 + r
      effective_rate = 10*r deg/s
    """
    r = int(round(float(rate_deg_s) / 10.0))
    r = max(1, min(36, r))
    base = 100 if direction == "cw" else 200
    cmd = base + r
    effective_rate = r * 10.0
    return cmd, r, effective_rate


def get_k_r(direction: str, r: int) -> float:
    if direction == "cw":
        return float(K_R_CW.get(r, DEFAULT_KR))
    if direction == "ccw":
        return float(K_R_CCW.get(r, DEFAULT_KR))
    raise ValueError("direction must be 'cw' or 'ccw'")


def theoretical_rotation_deg(direction: str, angle_deg: float) -> tuple[float, float]:
    """
    Returns:
      abs_deg: |angle|
      signed_deg: +abs for cw, -abs for ccw
    """
    abs_deg = float(angle_deg)
    signed_deg = abs_deg if direction == "cw" else -abs_deg
    return abs_deg, signed_deg


class AVIARRobot:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", LOCAL_BIND_PORT))
        self.msg_no = 1000

    def _send_frame(self, clamp: int, trans: int, rot: int):
        self.msg_no += 1
        payload = f"{self.msg_no}/{CHANNEL}/{clamp}/{trans}/{rot}".encode("ascii")
        self.sock.sendto(payload, (ROBOT_IP, ROBOT_PORT))

    def _stream(self, duration_s: float, clamp: int, trans: int, rot: int):
        t_end = time.time() + float(duration_s)
        while time.time() < t_end:
            self._send_frame(clamp=clamp, trans=trans, rot=rot)
            time.sleep(DT)

    def stop(self, duration_s: float = 0.1):
        self._stream(duration_s, clamp=0, trans=0, rot=0)

    def clamp(self):
        self._stream(CLAMP_TIME_S, clamp=2, trans=0, rot=0)
        self.stop(SETTLE_S)

    def release(self):
        self._stream(RELEASE_TIME_S, clamp=1, trans=0, rot=0)
        self.stop(SETTLE_S)

    def rotate_for(self, rate_deg_s: float, duration_s: float, direction: str):
        cmd, r, eff_rate = encode_rotation(direction, rate_deg_s)
        self._stream(duration_s, clamp=0, trans=0, rot=cmd)
        self.stop(SETTLE_S)
        return cmd, r, eff_rate

    def close(self):
        try:
            self.stop(0.2)
        finally:
            self.sock.close()


def ensure_results_header(path: Path):
    if path.exists():
        return
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "timestamp_iso",
            "channel",
            "direction",
            "requested_rate_deg_s",
            "encoded_r",
            "effective_rate_deg_s",
            "k_r_used",
            "target_angle_deg",
            "duration_uncalibrated_s",
            "duration_calibrated_s",
            "rotation_cmd",
            "theoretical_rotation_deg",
            "signed_theoretical_rotation_deg",
            "measured_actual_rotation_deg",
            "measurement_notes",
        ])


def append_row(path: Path, row: list):
    with path.open("a", newline="") as f:
        csv.writer(f).writerow(row)


def main():
    ensure_results_header(RESULTS_CSV)
    robot = AVIARRobot()

    try:
        print(f"=== Channel {CHANNEL} rotation 180deg accuracy vs rate (with k_r) ===")
        print(f"Rates: {RATES_DEG_S} deg/s (quantized to ~10 deg/s steps)")
        print("Will do CW then CCW for each requested rate.\n")

        print("Clamping...")
        robot.clamp()

        for rate in RATES_DEG_S:
            for direction in ("cw", "ccw"):
                cmd_tmp, r, eff_rate = encode_rotation(direction, rate)

                # Uncalibrated duration (ideal world)
                duration_uncal = TARGET_ANGLE_DEG / eff_rate

                # Calibrated duration using k_r (real world correction)
                k_r = get_k_r(direction, r)
                duration_cal = TARGET_ANGLE_DEG / (k_r * eff_rate)

                abs_deg, signed_deg = theoretical_rotation_deg(direction, TARGET_ANGLE_DEG)

                print(f"\nRotate {direction.upper()} at requested {rate} deg/s "
                      f"(r={r} => effective {eff_rate:.0f} deg/s, k_r={k_r:.4f}) "
                      f"for {duration_cal:.3f}s (uncal: {duration_uncal:.3f}s) "
                      f"(theoretical: {signed_deg:+.1f} deg)")
                cmd, r2, eff2 = robot.rotate_for(rate_deg_s=rate, duration_s=duration_cal, direction=direction)

                ts = time.strftime("%Y-%m-%dT%H:%M:%S")
                append_row(RESULTS_CSV, [
                    ts,
                    CHANNEL,
                    direction,
                    rate,
                    r2,
                    eff2,
                    f"{k_r:.6f}",
                    TARGET_ANGLE_DEG,
                    f"{duration_uncal:.6f}",
                    f"{duration_cal:.6f}",
                    cmd,
                    f"{abs_deg:.3f}",
                    f"{signed_deg:.3f}",
                    "",   # <- fill this after measuring
                    "",   # <- optional notes
                ])

                print(f"  Sent rotation_cmd={cmd}")
                print(f"  Measure actual rotation now, then continuing in {INTER_TEST_PAUSE_S:.1f}s...")
                time.sleep(INTER_TEST_PAUSE_S)

        print("\nReleasing...")
        robot.release()
        print(f"\nDone. Results saved to: {RESULTS_CSV}")

    finally:
        robot.close()


if __name__ == "__main__":
    main()
