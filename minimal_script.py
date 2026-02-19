import socket
import time

ROBOT_IP = "192.0.0.12"
ROBOT_CMD_PORT = 2020

PC_IP = "192.0.0.20"
PC_RECV_PORT = 54111

def make_cmd(msg_no: int, ch: int, clamp: int, trans: int, rot: int) -> bytes:
    # "numbers separated by /"
    s = f"{msg_no}/{ch}/{clamp}/{trans}/{rot}"
    return s.encode("ascii")

def parse_telemetry(data: bytes):
    # Expected: response_no/channel/status/displacement/limit
    try:
        parts = data.decode("ascii", errors="ignore").strip().split("/")
        if len(parts) < 5:
            return None
        resp_no = int(parts[0])
        ch = int(parts[1])
        clamp_status = int(parts[2])
        disp_mm = float(parts[3])
        limit = int(parts[4])
        return resp_no, ch, clamp_status, disp_mm, limit
    except Exception:
        return None

# One socket can do both if we bind it.
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((PC_IP, PC_RECV_PORT))
sock.settimeout(0.01)

msg_no = 1000

def send(ch, clamp, trans, rot):
    global msg_no
    msg_no += 1
    payload = make_cmd(msg_no, ch, clamp, trans, rot)
    sock.sendto(payload, (ROBOT_IP, ROBOT_CMD_PORT))
    return msg_no

print("Listening for telemetry on", (PC_IP, PC_RECV_PORT))
print("Sending commands to", (ROBOT_IP, ROBOT_CMD_PORT))

# --- sanity ping (no-op) ---
send(0, 0, 0, 0)

# Example: Channel 2, clamp, translate forward 5 mm/s for 0.5 s, then stop
CH = 2

# Clamp command: 2 = clamp, 1 = release, 0 = no command
send(CH, 2, 0, 0)

t_end = time.time() + 0.5
while time.time() < t_end:
    # translation: 100+speed forward, 200+speed backward, speed 1..15
    send(CH, 0, 100 + 5, 0)
    time.sleep(0.01)  # 100 Hz refresh (adjust if needed)

# Stop
send(CH, 0, 0, 0)

# Read telemetry for a moment
t_end = time.time() + 1.0
while time.time() < t_end:
    try:
        data, addr = sock.recvfrom(2048)
    except socket.timeout:
        continue
    tel = parse_telemetry(data)
    if tel:
        resp_no, ch, clamp_status, disp_mm, limit = tel
        print(f"TEL from {addr}: resp={resp_no} ch={ch} clampStat={clamp_status} disp={disp_mm:.2f}mm limit={limit}")
