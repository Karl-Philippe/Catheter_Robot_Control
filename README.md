# Catheter Robot UDP Control (AVIAR)

This repository contains a small Python controller that drives the AVIAR robot **over UDP** in **open loop**.  
You control the robot in **physical units** (mm, deg) and **unsigned speeds** (mm/s, deg/s). The code converts these to the robot’s UDP command format.

Current code layout:

- Main controller/API: `control.py`
- Communication + calibration scripts: `test/`
- Calibration result tables: `test/data/`

---

## 1) System prerequisites

### Network

- Robot PC / robot software IP: `192.0.0.12`
- Your external PC IP: `192.0.0.20` (static on the Ethernet interface)
- Robot command UDP port: `2020`
- Your PC receive/bind port: `54111`

You already validated connectivity with:

- `ping 192.0.0.12` ✅

### Robot software UDP dialog (AVIAR_simulator.exe)

In the robot UI (UDP comm section):

- **Local IP** = `192.0.0.12`
- **Local Port** = `2020`
- **Remote IP** = `192.0.0.20`
- **Remote Port** = `54111`
- Click **Start**
- You should see **send/recv counters increment** when your script runs.

### Windows firewall (inbound)

Allow UDP on your PC port:

- UDP inbound: `54111`

### Python

- Python `3.10+` (the code uses hints like `float | None`)
- No third-party dependencies (standard library only)

---

## 2) Safety model (important)

This controller is **open-loop**:

- The robot is commanded by sending speed commands repeatedly at ~100 Hz.
- Distances/angles are achieved by running that speed for a computed time.
- Real motion depends on friction, load, cable slip, inertia, and internal controller behavior.

**Your current calibration:**

- Translation speeds validated: **-50 to +50 mm/s**
- Rotation calibration factor: **k_rot = 1.5**
- Rotation rates validated (after calibration): **-360 to +360 deg/s** using the 180° test.

---

## 3) Robot UDP message format

The robot protocol uses a **slash-separated** ASCII frame:

### External PC → Robot (command frame)

```bash
<msg_no>/<channel>/<clamp_cmd>/<translation_cmd>/<rotation_cmd>
```

- `msg_no`: increasing integer (timestamp-like)
- `channel`: `0..4` (typically `1..4`), where `0` = “not selected”
- `clamp_cmd`: `0/1/2`
  - `0`: no command
  - `1`: release
  - `2`: clamp
- `translation_cmd`: `0` or encoded `100/200 + speed`
- `rotation_cmd`: `0` or encoded `100/200 + r`

### Robot → External PC (telemetry frame) *(if enabled in UI)*

```bash
<resp_no>/<channel>/<clamp_status>/<displacement_mm>/<limit_flag>
```

- `resp_no`: robot’s last received message number
- `channel`: operating channel (0..4)
- `clamp_status`: state encoding from vendor doc
- `displacement_mm`: displacement (mm)
- `limit_flag`: 0/1 (limit reached)

> The current control script **does not require telemetry** to function, but telemetry is useful for monitoring.

---

## 4) Command encoding details

### 4.1 Clamp command

`clamp_cmd` field:

- `0`: no clamp action
- `1`: release
- `2`: clamp

Clamping is run as a **time-based action** in open loop (ex: 2.5 s).

### 4.2 Translation command (translation_cmd)

Vendor encoding:

- Forward: `100 + speed`
- Backward: `200 + speed`

Where:

- `speed` is an integer magnitude.
- Vendor doc originally states `1..15 mm/s`, but **your tests show 1..50 mm/s works**.

Examples:

- Forward at 25 mm/s → `125`
- Backward at 40 mm/s → `240`

### 4.3 Rotation command (rotation_cmd)

Vendor encoding:

- CW: `100 + r`
- CCW: `200 + r`

Where:

- `r = round(rate_deg_s / 10)` (integer)
- `r` is clamped to `1..36`
- The robot’s effective rotation rate is quantized:
  - `omega_eff ≈ 10 * r` deg/s

Examples:

- CW at 90 deg/s → `r=9` → cmd `109`
- CCW at 360 deg/s → `r=36` → cmd `236`

---

## 5) High-level Python API (what you call)

The script exposes a clean API:

### Translation

```python
robot.translate_by(distance_mm, speed_mm_s=None)
```

- `distance_mm` is **signed**:
  - `+` => forward
  - `-` => backward
- `speed_mm_s` is **unsigned** magnitude (optional):
  - Default = **25 mm/s**
  - Clipped to max **50 mm/s**
- Duration computed:
  - `t = abs(distance_mm) / speed_mm_s`

### Rotation

```python
robot.rotate_by(angle_deg, rate_deg_s=None)
```

- `angle_deg` is **signed**:
  - `+` => CW
  - `-` => CCW
- `rate_deg_s` is **unsigned** magnitude (optional):
  - Default = **90 deg/s**
  - Quantized to `omega_eff = 10 * r`, `r∈[1..36]`
- Duration computed using your calibration:
  - `t = abs(angle_deg) / (k_rot * omega_eff)`
  - with `k_rot = 1.5`

---

## 6) Mapping: API ↔ Robot command frames

### 6.1 Translation example

Call:

```python
robot.translate_by(+40, 25)
```

API interprets:

- direction = forward
- speed = 25 mm/s
- duration = 40/25 = 1.6 s

Robot command frames streamed at ~100 Hz:

```bash
<msg>/<CH>/0/125/0
```

### 6.2 Rotation example (with k_rot)

Call:

```python
robot.rotate_by(-180, 360)
```

API interprets:

- direction = CCW
- requested rate = 360 deg/s
- r = round(360/10)=36
- omega_eff = 360 deg/s
- duration = 180 / (1.5 * 360) = 0.333... s

Robot command frames streamed at ~100 Hz:

```bash
<msg>/<CH>/0/0/236
```

### 6.3 Clamp example

Call:

```python
robot.clamp()
```

Robot command frames streamed for `CLAMP_TIME_S`:

```bash
<msg>/<CH>/2/0/0
```

---

## 7) Defaults and calibration constants

In code:

- Default translation speed: `25 mm/s`
- Default rotation rate: `90 deg/s`
- Translation max speed: `50 mm/s`
- Rotation gain: `k_rot = 1.5`
- Default channel: `CH = 2`
- Streaming rate: `TX_HZ = 100 Hz`

If the robot behavior changes (different disposables, different load, different instrument),
you should re-run calibration and update:

- `TRANS_SPEED_MAX` (if needed)
- `K_ROT`
- default speeds if desired

---

## 8) Example usage

```python
from control import AVIARRobot

robot = AVIARRobot()
try:
    robot.clamp()

    robot.translate_by(+40)        # forward 40 mm @ default 25 mm/s
    robot.translate_by(-20, 40)    # backward 20 mm @ 40 mm/s

    robot.rotate_by(+90)           # CW 90 deg @ default 90 deg/s (quantized) with k_rot
    robot.rotate_by(-180, 360)     # CCW 180 deg @ 360 deg/s with k_rot

    robot.release()
finally:
    robot.close()
```

### 8.1 Running the scripts in this repo

From the repository root:

```bash
python3 control.py
python3 test/test_communication.py
python3 test/test_control.py
python3 test/test_translation.py
python3 test/test_rotation.py
```

Notes:

- These are hardware-in-the-loop scripts (not unit tests).
- `test/test_translation.py` and `test/test_rotation.py` append results into `test/data/*.csv`.

---

## 9) Troubleshooting

### No motion / only clamp works

- Ensure robot UI UDP dialog is **Start**-ed.
- Ensure you’re targeting a channel that supports rotation (vendor doc: rotation typically channel 2 and 3).
- Make sure channel selection in UI matches your script’s `CH`.

### No telemetry

- Telemetry requires the robot UI UDP dialog remote settings to be correct:
  - Remote IP: your PC IP
  - Remote Port: your bind port (54111)
- Firewall inbound UDP rule for 54111.

### Motion timing feels off

- Translation: check the -50..+50 mm/s still holds for your current setup.
- Rotation: if angles drift again, re-estimate `k_rot` using the 180° test at a few speeds.

---

## 10) Files

- `control.py`: high-level API + command encoding + demo motion sequence
- `test/test_communication.py`: telemetry/connectivity sanity check
- `test/test_control.py`: simple end-to-end sequence (clamp, translation, rotation, release)
- `test/test_translation.py`: translation sweep script (logs expected/measured displacement)
- `test/test_rotation.py`: 180-degree rotation sweep script with `k_r` compensation
- `test/data/translation_accuracy_tables.csv`: translation sweep results
- `test/data/rotation_accuracy_tables.csv`: rotation sweep results
- `doc/AVIAR_User's Manual_LNR.pdf`: vendor/user manual copy

---

## 11) Disclaimer

This is research/prototyping code. It is open-loop. Use in a safe test setup, start with low speeds,
and assume the robot can do surprising things when friction changes.
