"""
Channel-2 translation sweep test
- Runs translation at speeds: 10,15,20,25,30,35,40,45,50 (mm/s) for 3 seconds each
- Forward then backward for each speed (so you can see asymmetry / backlash)
- Writes a results CSV with placeholders for measured displacement + theoretical displacement

IMPORTANT:
Your protocol document says translation speed is 1..15 mm/s. You explicitly asked for up to 50+.
This script will still send those values (no clamping), but be aware the robot may ignore/saturate them.
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

DURATION_S = 3.0
SPEEDS_MM_S = [10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75]

RESULTS_CSV = Path("test\translation_accuracy_tables.csv")


def encode_translation(direction: str, speed_mm_s: float) -> int:
    """
    Protocol encoding (from your doc):
      forward  => 100 + speed
      backward => 200 + speed
    (No clamping / no validation here by request.)
    """
    s = int(speed_mm_s)
    base = 100 if direction == "forward" else 200
    return base + s


def theoretical_displacement_mm(direction: str, speed_mm_s: float, duration_s: float) -> tuple[float, float]:
    """
    Returns:
      abs_mm: |speed| * duration
      signed_mm: +abs_mm for forward, -abs_mm for backward
    """
    abs_mm = float(speed_mm_s) * float(duration_s)
    signed_mm = abs_mm if direction == "forward" else -abs_mm
    return abs_mm, signed_mm


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

    def translate_for(self, speed_mm_s: float, duration_s: float, direction: str):
        cmd = encode_translation(direction, speed_mm_s)
        self._stream(duration_s, clamp=0, trans=cmd, rot=0)
        self.stop(SETTLE_S)
        return cmd

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
            "speed_mm_s",
            "duration_s",
            "translation_cmd",
            "theoretical_displacement_mm",
            "signed_theoretical_displacement_mm",
            "measured_actual_displacement_mm",
            "measurement_notes",
        ])


def append_row(path: Path, row: list):
    with path.open("a", newline="") as f:
        csv.writer(f).writerow(row)


def main():
    ensure_results_header(RESULTS_CSV)
    robot = AVIARRobot()

    try:
        print(f"=== Channel {CHANNEL} translation sweep ===")
        print(f"Speeds: {SPEEDS_MM_S} mm/s, duration: {DURATION_S}s each")
        print("Will do FORWARD then BACKWARD for each speed.\n")

        print("Clamping...")
        robot.clamp()

        for s in SPEEDS_MM_S:
            for direction in ("forward", "backward"):
                abs_mm, signed_mm = theoretical_displacement_mm(direction, s, DURATION_S)

                print(f"\nTranslate {direction.upper()} at {s} mm/s for {DURATION_S:.1f}s "
                      f"(theoretical: {signed_mm:+.1f} mm)")
                cmd = robot.translate_for(speed_mm_s=s, duration_s=DURATION_S, direction=direction)

                ts = time.strftime("%Y-%m-%dT%H:%M:%S")
                append_row(RESULTS_CSV, [
                    ts,
                    CHANNEL,
                    direction,
                    s,
                    DURATION_S,
                    cmd,
                    f"{abs_mm:.3f}",
                    f"{signed_mm:.3f}",
                    "",   # <- fill this after measuring
                    "",   # <- optional notes
                ])

                print(f"  Sent translation_cmd={cmd}")
                print(f"  Measure actual displacement now, then continuing in {INTER_TEST_PAUSE_S:.1f}s...")
                time.sleep(INTER_TEST_PAUSE_S)

        print("\nReleasing...")
        robot.release()
        print(f"\nDone. Results saved to: {RESULTS_CSV}")

    finally:
        robot.close()


if __name__ == "__main__":
    main()
