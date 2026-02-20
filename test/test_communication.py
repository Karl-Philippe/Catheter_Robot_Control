import socket
import time

ROBOT_IP = "192.0.0.12"
ROBOT_CMD_PORT = 2020
PC_RECV_PORT = 54111

def make_cmd(msg_no: int, ch: int, clamp: int, trans: int, rot: int) -> bytes:
    return f"{msg_no}/{ch}/{clamp}/{trans}/{rot}".encode("ascii")

def parse_telemetry(data: bytes):
    try:
        parts = data.decode("ascii", errors="ignore").strip().split("/")
        if len(parts) < 5:
            return None
        return int(parts[0]), int(parts[1]), int(parts[2]), float(parts[3]), int(parts[4])
    except Exception:
        return None

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", PC_RECV_PORT))          # robust bind
sock.settimeout(0.05)

msg_no = 1000
def send(ch, clamp, trans, rot):
    global msg_no
    msg_no += 1
    sock.sendto(make_cmd(msg_no, ch, clamp, trans, rot), (ROBOT_IP, ROBOT_CMD_PORT))
    return msg_no

print(f"Listening for telemetry on UDP :{PC_RECV_PORT}")
print(f"Sending commands to {ROBOT_IP}:{ROBOT_CMD_PORT}")

# 1) Wait briefly for telemetry (proves robot dialog Remote IP/port + Start is correct)
print("Waiting for telemetry (2s)...")
t_end = time.time() + 2.0
got = False
while time.time() < t_end:
    try:
        data, addr = sock.recvfrom(2048)
    except socket.timeout:
        continue
    tel = parse_telemetry(data)
    if tel:
        print("Telemetry OK from", addr, "sample:", tel)
        got = True
        break

if not got:
    print("No telemetry received. Check robot SW UDP dialog Remote IP=192.0.0.20, Remote Port=54111, and click Start. Also check firewall.")
    raise SystemExit(1)

CH = 2

# 2) Stream clamp briefly (200ms)
t_end = time.time() + 0.2
while time.time() < t_end:
    send(CH, 2, 0, 0)
    time.sleep(0.01)

# 3) Translate forward at speed code 5 for 0.5s
t_end = time.time() + 0.5
while time.time() < t_end:
    send(CH, 0, 100 + 5, 0)
    time.sleep(0.01)

# 4) Stop
for _ in range(5):  # send stop a few times (cheap insurance)
    send(CH, 0, 0, 0)
    time.sleep(0.01)

# 5) Print telemetry for 2 seconds
print("Telemetry stream (2s):")
t_end = time.time() + 2.0
while time.time() < t_end:
    try:
        data, addr = sock.recvfrom(2048)
    except socket.timeout:
        continue
    tel = parse_telemetry(data)
    if tel:
        resp_no, ch, clamp_status, disp_mm, limit = tel
        print(f"TEL {addr}: resp={resp_no} ch={ch} clamp={clamp_status} disp={disp_mm:.2f}mm limit={limit}")
