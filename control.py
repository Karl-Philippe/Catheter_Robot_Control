import socket
import time
from dataclasses import dataclass
from threading import Thread, Lock
from typing import Optional, Tuple

# ----------------------------
# Data models
# ----------------------------
@dataclass
class Telemetry:
    resp_no: int
    channel: int
    clamp_status: int
    displacement_mm: float
    limit_reached: int
    addr: Tuple[str, int]

    @property
    def is_clamped(self) -> bool:
        return self.clamp_status == 10

    @property
    def is_released(self) -> bool:
        return self.clamp_status == 1000


# ----------------------------
# Controller
# ----------------------------
class AviarUDPController:
    """
    Control the robot via UDP protocol:
      msgNo/channel/clamp/translation/rotation

    Network from your screenshot:
      - Send commands to robot SW: 192.0.0.12:2020
      - Receive telemetry on PC:   192.0.0.20:54111
    """

    def __init__(self,
                 robot_ip="192.0.0.12",
                 robot_port=2020,
                 pc_ip="192.0.0.20",
                 pc_port=54111,
                 tx_rate_hz=100):
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.pc_ip = pc_ip
        self.pc_port = pc_port
        self.tx_period = 1.0 / float(tx_rate_hz)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.pc_ip, self.pc_port))     # ensures source port == 54111
        self.sock.settimeout(0.02)

        self._msg_no = 1000
        self._lock = Lock()
        self._last_tel: Optional[Telemetry] = None

        self._rx_thread = Thread(target=self._rx_loop, daemon=True)
        self._running = True
        self._rx_thread.start()

    # --- Low-level protocol helpers ---
    def _encode(self, msg_no, ch, clamp, trans, rot) -> bytes:
        return f"{msg_no}/{ch}/{clamp}/{trans}/{rot}".encode("ascii")

    def _parse_tel(self, data: bytes, addr) -> Optional[Telemetry]:
        try:
            parts = data.decode("ascii", errors="ignore").strip().split("/")
            if len(parts) < 5:
                return None
            return Telemetry(
                resp_no=int(parts[0]),
                channel=int(parts[1]),
                clamp_status=int(parts[2]),
                displacement_mm=float(parts[3]),
                limit_reached=int(parts[4]),
                addr=addr,
            )
        except Exception:
            return None

    def _rx_loop(self):
        while self._running:
            try:
                data, addr = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            tel = self._parse_tel(data, addr)
            if tel:
                with self._lock:
                    self._last_tel = tel

    def last_telemetry(self) -> Optional[Telemetry]:
        with self._lock:
            return self._last_tel

    def _send_raw(self, ch: int, clamp: int, trans: int, rot: int) -> int:
        self._msg_no += 1
        payload = self._encode(self._msg_no, ch, clamp, trans, rot)
        self.sock.sendto(payload, (self.robot_ip, self.robot_port))
        return self._msg_no

    # --- Command encoding ---
    @staticmethod
    def translation_cmd(direction: str, speed_mm_s: int) -> int:
        """
        direction: 'forward' or 'backward'
        speed_mm_s: 1..15
        returns: 100+speed (forward) or 200+speed (backward), or 0 for none
        """
        if speed_mm_s <= 0:
            return 0
        speed = max(1, min(15, int(speed_mm_s)))
        if direction == "forward":
            return 100 + speed
        elif direction == "backward":
            return 200 + speed
        else:
            raise ValueError("direction must be 'forward' or 'backward'")

    @staticmethod
    def rotation_cmd(direction: str, speed_deg_s: float) -> int:
        """
        direction: 'cw' or 'ccw'
        speed_deg_s up to ~360.
        Protocol uses integer ~36 representing speed/10.
        Example: 109 -> cw at 90 deg/s => 9 * 10
        """
        if speed_deg_s <= 0:
            return 0
        r = int(round(speed_deg_s / 10.0))
        r = max(1, min(36, r))
        if direction == "cw":
            return 100 + r
        elif direction == "ccw":
            return 200 + r
        else:
            raise ValueError("direction must be 'cw' or 'ccw'")

    # --- High-level actions (explicit) ---
    def stop(self, ch: int):
        """Stop translation + rotation on channel ch."""
        self._send_raw(ch, clamp=0, trans=0, rot=0)

    def clamp(self, ch: int, timeout_s=2.0) -> bool:
        """Command clamp and wait until telemetry reports clamped (status=10)."""
        self._send_raw(ch, clamp=2, trans=0, rot=0)
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            tel = self.last_telemetry()
            if tel and tel.channel == ch and tel.is_clamped:
                return True
            time.sleep(0.01)
        return False

    def release(self, ch: int, timeout_s=2.0) -> bool:
        """Command release and wait until telemetry reports released (status=1000)."""
        self._send_raw(ch, clamp=1, trans=0, rot=0)
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            tel = self.last_telemetry()
            if tel and tel.channel == ch and tel.is_released:
                return True
            time.sleep(0.01)
        return False

    def translate_mm(self, ch: int, delta_mm: float, speed_mm_s: int, timeout_s=10.0) -> bool:
        """
        Move by a desired length in mm, using displacement telemetry as feedback.
        delta_mm > 0 => forward, < 0 => backward
        """
        tel = self.last_telemetry()
        if not tel:
            raise RuntimeError("No telemetry received. Click Start in robot SW and check networking.")

        start = tel.displacement_mm
        target = start + float(delta_mm)

        direction = "forward" if delta_mm >= 0 else "backward"
        trans = self.translation_cmd(direction, speed_mm_s)

        t0 = time.time()
        while time.time() - t0 < timeout_s:
            tel = self.last_telemetry()
            if not tel:
                continue
            if tel.limit_reached == 1:
                self.stop(ch)
                return False

            # Decide if we reached target (crossing logic)
            d = tel.displacement_mm
            if delta_mm >= 0 and d >= target:
                self.stop(ch)
                return True
            if delta_mm < 0 and d <= target:
                self.stop(ch)
                return True

            # keep streaming motion command
            self._send_raw(ch, clamp=0, trans=trans, rot=0)
            time.sleep(self.tx_period)

        self.stop(ch)
        return False

    def rotate_deg(self, ch: int, delta_deg: float, speed_deg_s: float, timeout_s=10.0) -> bool:
        """
        Best-effort rotation-by-angle using time only.
        WARNING: No telemetry feedback for rotation angle in your protocol.
        """
        if speed_deg_s <= 0:
            raise ValueError("speed_deg_s must be > 0")

        direction = "cw" if delta_deg >= 0 else "ccw"
        rot = self.rotation_cmd(direction, abs(speed_deg_s))

        duration = abs(delta_deg) / float(speed_deg_s)
        duration = min(duration, timeout_s)

        t_end = time.time() + duration
        while time.time() < t_end:
            self._send_raw(ch, clamp=0, trans=0, rot=rot)
            time.sleep(self.tx_period)

        self.stop(ch)
        return True

    def close(self):
        self._running = False
        time.sleep(0.05)
        self.sock.close()


# ----------------------------
# Example usage
# ----------------------------
if __name__ == "__main__":
    robot = AviarUDPController(
        robot_ip="192.0.0.12", robot_port=2020,
        pc_ip="192.0.0.20", pc_port=54111,
        tx_rate_hz=100
    )

    CH = 2

    # Wait a moment for telemetry
    time.sleep(0.2)
    print("Last telemetry:", robot.last_telemetry())

    # Explicit, control-like commands:
    ok = robot.clamp(CH)
    print("clamp:", ok)

    ok = robot.translate_mm(CH, delta_mm=25.0, speed_mm_s=10)   # move 25 mm forward at 10 mm/s
    print("translate 25mm:", ok)

    ok = robot.rotate_deg(CH, delta_deg=45.0, speed_deg_s=90.0) # approx: 45 degrees at 90 deg/s
    print("rotate 45deg (timed):", ok)

    ok = robot.release(CH)
    print("release:", ok)

    robot.close()
