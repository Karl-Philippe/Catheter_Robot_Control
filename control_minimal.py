import socket
import time

ROBOT_IP = "192.0.0.12"
ROBOT_PORT = 2020
LOCAL_BIND_PORT = 54111

CH = 2
TX_HZ = 100
DT = 1.0 / TX_HZ

CLAMP_TIME_S = 2.1  # Assumption
RELEASE_TIME_S = 2.1 # Assumption

TRANS_SPEED = 30
TRANS_TIME_S = 2.0

ROT_DEG_S = 90
ROT_TIME_S = 2.0
DO_ROTATE = True

def trans_cmd(direction: str, speed: int) -> int:
    speed = max(1, min(15, int(speed)))
    return (100 if direction == "forward" else 200) + speed

def rot_cmd(direction: str, deg_s: float) -> int:
    r = max(1, min(36, int(round(float(deg_s) / 10.0))))
    return (100 if direction == "cw" else 200) + r

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", LOCAL_BIND_PORT))
    msg = 1000

    def send(clamp: int, trans: int, rot: int):
        nonlocal msg
        msg += 1
        payload = f"{msg}/{CH}/{clamp}/{trans}/{rot}".encode("ascii")
        sock.sendto(payload, (ROBOT_IP, ROBOT_PORT))

    def stream(duration_s: float, clamp: int, trans: int, rot: int):
        t_end = time.time() + duration_s
        while time.time() < t_end:
            send(clamp, trans, rot)
            time.sleep(DT)

    def stop(duration_s: float = 0.3):
        stream(duration_s, clamp=0, trans=0, rot=0)

    print("Selecting channel (settle)...")
    stop(0.3)

    print(f"CLAMP for {CLAMP_TIME_S:.1f}s ...")
    stream(CLAMP_TIME_S, clamp=2, trans=0, rot=0)
    stop(0.5)

    print(f"FORWARD translate {TRANS_TIME_S:.1f}s at speed {TRANS_SPEED} ...")
    stream(TRANS_TIME_S, clamp=0, trans=trans_cmd("forward", TRANS_SPEED), rot=0)
    stop(0.3)

    print(f"BACKWARD translate {TRANS_TIME_S:.1f}s at speed {TRANS_SPEED} ...")
    stream(TRANS_TIME_S, clamp=0, trans=trans_cmd("backward", TRANS_SPEED), rot=0)
    stop(0.3)


    print(f"ROTATE CW {ROT_TIME_S:.1f}s at ~{ROT_DEG_S} deg/s ...")
    stream(ROT_TIME_S, clamp=0, trans=0, rot=rot_cmd("cw", ROT_DEG_S))
    stop(0.3)

    print(f"ROTATE CCW {ROT_TIME_S:.1f}s at ~{ROT_DEG_S} deg/s ...")
    stream(ROT_TIME_S, clamp=0, trans=0, rot=rot_cmd("ccw", ROT_DEG_S))
    stop(0.3)

    print(f"RELEASE for {RELEASE_TIME_S:.1f}s ...")
    stream(RELEASE_TIME_S, clamp=1, trans=0, rot=0)
    stop(0.5)

    sock.close()
    print("Done.")

if __name__ == "__main__":
    main()
