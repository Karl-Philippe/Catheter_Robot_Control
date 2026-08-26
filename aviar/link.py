"""
UDP transport: the only place in the package that touches a socket.

The robot is driven by a continuous stream -- a datagram is not a latched
command -- so holding a command means re-sending the same frame every TX_DT
until the hold window ends.
"""

import socket
import time

from .config import LOCAL_BIND_PORT, ROBOT_IP, ROBOT_PORT, TX_DT


class RobotLink:
    """
    Sends command frames "<msg_no>/<ch>/<clamp>/<trans>/<rot>" over UDP.

    `msg_no` increments per frame so the robot can spot gaps.
    """

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", LOCAL_BIND_PORT))
        self.msg = 1000

    def send_frame(self, ch: int, clamp: int, trans_cmd: int, rot_cmd: int):
        """Send exactly one frame."""
        self.msg += 1
        payload = f"{self.msg}/{ch}/{clamp}/{trans_cmd}/{rot_cmd}".encode("ascii")
        self.sock.sendto(payload, (ROBOT_IP, ROBOT_PORT))

    def hold_channel(self, ch: int, clamp: int, trans_cmd: int, rot_cmd: int,
                     duration_s: float):
        """
        Hold one channel's command for duration_s, resending every TX_DT.

        Always sends at least one frame, even when duration_s is shorter than
        TX_DT, and never overshoots the window on its final sleep.
        """
        t_end = time.time() + float(duration_s)
        while True:
            self.send_frame(ch, clamp, trans_cmd, rot_cmd)
            if time.time() >= t_end:
                break
            time.sleep(min(TX_DT, max(0.0, t_end - time.time())))

    def close(self):
        self.sock.close()
