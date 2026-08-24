# Catheter Robot UDP Control (AVIAR)

This repository contains a small Python controller that drives the AVIAR robot **over UDP** in **open loop**.  
You control the robot in **physical units** (mm, deg) and **unsigned speeds** (mm/s, deg/s). The code converts these to the robot’s UDP command format.

There are two ways to drive the robot:

1. **Scripted motion** (`control.py`) — you call `translate_by()` / `rotate_by()` with physical
   distances and angles on a single channel.
2. **Log replay** (`dual_control.py`) — you replay a recorded simulation log, which drives the
   guidewire (CH2) and catheter (CH4) together, one move per log row for that row's `dt`. See §5.

Current code layout:

- Scripted controller/API: `control.py`
- Dual-channel log replay: `dual_control.py`
- Replay visualization / video export: `guidewire_panel.py`
- Input logs for replay: `data/`
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
- `control.py` and `dual_control.py`: no third-party dependencies (standard library only)
- `guidewire_panel.py` (visualization only, not needed to drive the robot):

```bash
pip install pygame numpy opencv-python
```

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

## 5) Log replay input format (`dual_control.py`)

`dual_control.py` replays a recorded simulation log against the real robot. The log is the
**whitespace-separated text file** produced by the SOFA/RL environment, e.g. `data/control_logs.txt`.

### 5.1 File structure

- First non-empty line is a **header** naming the columns.
- Remaining lines are data rows; blank lines and lines starting with `#` are skipped.
- Columns are located **by name**, not by position, so extra columns are harmless and column
  order may change. A row is skipped if it has fewer fields than the header.

### 5.2 Required columns

Only these four are read. Every other column in the log is ignored.

| Column | Unit | Used for | Required |
| --- | --- | --- | --- |
| `guide_rotation_speed_cmd` | **rad/s** | CH2 guidewire rotation | yes |
| `guide_speed_cmd` | mm/s | CH2 guidewire translation | yes |
| `cath_speed_cmd` | mm/s | CH4 catheter translation | yes |
| `dt` | seconds | how long to hold that row's command | no — see §5.4 |

Columns present in the current logs but **not** used: `wall_time`, `run_id`, `pid`, `env_id`,
`episode`, `episode_step`, `sofa_step`, `action_rot`, `action_ins`, `rotation_cmd_deg`,
`rotation_cmd_rad`.

### 5.3 Rotation units (important)

`guide_rotation_speed_cmd` is in **rad/s**, so the parser is called with `assume_units="rad"`
and converts with `w_deg_s = w * 180/pi`.

You can verify this on any log: `rotation_cmd_deg / rotation_cmd_rad = 57.296` (= 180/pi), and
`guide_rotation_speed_cmd = rotation_cmd_rad * 10` (the per-step angle divided by the 0.1 s step).
Equivalently, `guide_rotation_speed_cmd` in deg/s equals `rotation_cmd_deg / DT`.

> Setting `assume_units="deg"` on a rad/s log silently destroys the rotation profile: a typical
> `-4.59` gets treated as `-4.59 deg/s`, which rounds to `r=0`, is clamped up to `r=1`, and
> commands a flat 10 deg/s for nearly every step regardless of the log.

### 5.4 Timestep (`dt`)

**One log row = one robot move.** Every data row is replayed, in file order, and each row is
executed for its own duration:

- If the log has a `dt` column, that row's `dt` (in seconds) is the hold duration.
- If it does not, every row falls back to `default_dt`, which defaults to `DEFAULT_DT = 0.1 s`.
  This keeps the existing `data/control_logs.txt` and `data/control_logs_V0.txt` replayable
  unchanged, at the 0.1 s step they were recorded at.

Rows with `dt <= 0` are skipped. `wall_time` is no longer read at all — earlier versions used it
to decimate rows onto a fixed 0.1 s grid, which could drop rows; the `dt` column now carries the
timing explicitly, so the replay is driven by the file rather than by recording wall-clock.

Override the fallback per run:

```python
run_from_log("data/control_logs.txt", assume_units="rad", default_dt=0.05)
```

### 5.5 How a step is executed

The two channels are **not** driven simultaneously. Each step of duration `dt` is time-shared, and
each channel gets the **full** `dt`, split into `INTERLEAVE` alternating slices of
`dt / INTERLEAVE` each:

1. CH4 (catheter) translation for `dt / INTERLEAVE`
2. CH2 (guidewire) translation + rotation for `dt / INTERLEAVE`
3. ... repeated `INTERLEAVE` times

At the default `INTERLEAVE = 1` that is simply one CH4 block then one CH2 block. Frames are resent
every `TX_DT` = 0.01 s inside each hold window, and at least one frame is always sent per channel
even when a slice is shorter than that. The runner adds no extra pacing sleep on top.

#### `TX_DT` vs `INTERLEAVE` — two different rates

These are easy to conflate but do unrelated jobs:

| Constant | Default | What it controls |
| --- | --- | --- |
| `TX_DT` | 0.01 s | **Refresh rate.** How often a *held* command is re-sent. |
| `INTERLEAVE` | 1 | **Alternation rate.** How many times the step swaps CH4↔CH2. |

`TX_DT` is still doing real work: the robot is driven by a continuous stream, so a datagram is not
a latched command. Re-sending at 100 Hz keeps the axis moving for the whole hold and caps the cost
of a dropped datagram at 10 ms instead of the entire move. It has nothing to do with which channel
is active.

`INTERLEAVE` changes only how finely the channels alternate — **not** how far they travel. Each
channel still accumulates a full `dt`, so displacement is identical at every setting, and the step
still occupies `2 * dt` of wall-clock:

| `INTERLEAVE` | Slice at `dt = 0.2 s` | Swaps per row | Guidewire travel @ 10 mm/s |
| --- | --- | --- | --- |
| 1 | 200 ms | 2 | 2.0 mm |
| 4 | 50 ms | 8 | 2.0 mm |
| 10 | 20 ms | 20 | 2.0 mm |

Raise it when you want the two channels to track each other more closely instead of moving in one
long block each. The constants do interact at the bottom end: a slice shorter than `TX_DT`
degenerates to a single frame, so keep `dt / INTERLEAVE >= TX_DT` (at `dt = 0.1 s` that caps
`INTERLEAVE` at 10).

```python
run_from_log("data/control_logs.txt", assume_units="rad", interleave=4)
```

Or change the `INTERLEAVE` default in `robot_core.py` to affect every replay.

#### Why the full `dt`, not `dt / 2`

Time-sharing is physically real: a channel only moves while it is the one being commanded. Giving
each channel `dt / 2` therefore covers **half** the commanded displacement — a row asking for
10 mm/s over `dt = 0.1 s` should advance 1.0 mm but only advances 0.5 mm. Over the 39 rows of
`data/control_logs_V0.txt` that is 19.5 mm of guidewire insertion instead of 39 mm.

Holding each channel for the full `dt` restores the exact commanded distance (`v * dt`) and angle
(`w * dt`) without inflating the speeds into the protocol clamps. The trade-off is wall-clock: a
step occupies `2 * dt`, so a dual-channel replay runs at half real-time. Single-channel replay in
`control.py` has no such trade-off (§7).

### 5.6 Running a replay

```bash
python3 dual_control.py
```

This clamps both channels, replays `data/control_logs.txt`, then releases. To do a short dry run
first, edit the call at the bottom of the file:

```python
run_from_log("data/control_logs.txt", max_steps=20, assume_units="rad")
```

You can also bypass the log entirely and replay an explicit list:

```python
run_from_cmds([
    StepCmd(v2_mm_s=+10, w2_deg_s=-90, v4_mm_s=+10, dt=0.1),
    StepCmd(v2_mm_s=+10, w2_deg_s=-30, v4_mm_s=+0,  dt=0.5),
])
```

Note that `StepCmd.w2_deg_s` is in **deg/s** — the rad→deg conversion happens in the parser, not
in `StepCmd`. `StepCmd.dt` defaults to `DEFAULT_DT` (0.1 s) when omitted.

---

## 6) Visualization (`guidewire_panel.py`)

Renders the same log as a 4-quadrant monitor (catheter speed dial, guidewire rotation dial,
geometry view, orientation compass).

```bash
python3 guidewire_panel.py
```

Configuration is at the top of the file:

- `LOG_PATH` — log to visualize (default `data/control_logs.txt`)
- `ASSUME_UNITS` — `"rad"`, matching `dual_control.py`
- `DT_TARGET` — `0.2`, deliberately **2x** the 0.1 s recording step for visual clarity, so the
  panel shows half as many steps as the robot actually executes. The panel keeps its own
  `wall_time`-based decimation; only `dual_control.py` switched to per-row `dt`.
- `RESOLUTION` — `"SD"` / `"HD"` / `"FHD"` / `"2K"` / `"4K"`
- `VIDEO_EXPORT` — `True` writes `videos/guidewire_replay.mp4` offscreen; `False` opens an
  interactive pygame window
- `VIDEO_EXPORT_QUADRANTS` — also writes one video per quadrant

This script never opens a socket and never commands the robot.

---

## 7) High-level Python API (`control.py`)

`control.py` drives **CH2 (guidewire) only** and offers two ways in: the distance/angle API below,
and the same speed+`dt` log replay as §5.

### 7.0 Log replay on CH2

```python
from control import run_from_log
run_from_log("data/control_logs.txt", assume_units="rad", default_dt=0.1)
```

Reads the same columns as §5.2, but `cath_speed_cmd` is **optional** here (it is parsed with
`require_cath=False`) — a guidewire-only log with just `guide_rotation_speed_cmd`,
`guide_speed_cmd` and `dt` replays fine, and `cath_speed_cmd` is ignored if present.

Because only one channel is driven there is no time-sharing: translation and rotation go out in
the **same frame** and both run for the whole `dt`, so a row covers exactly `v * dt` mm and
`w * dt` deg, and a step occupies `dt` of wall-clock (not `2 * dt` as in §5.5). `robot.step(cmd)`
executes a single `StepCmd` directly.

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

## 8) Mapping: API ↔ Robot command frames

### 8.1 Translation example

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

### 8.2 Rotation example (with k_rot)

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

### 8.3 Clamp example

Call:

```python
robot.clamp()
```

Robot command frames streamed for `CLAMP_TIME_S`:

```bash
<msg>/<CH>/2/0/0
```

---

## 9) Defaults and calibration constants

Shared by both controllers, defined once in `robot_core.py`:

- Translation max speed: `TRANS_SPEED_MAX = 50 mm/s`
- Rotation rate ceiling: `ROT_RATE_MAX = 360 deg/s`
- Rotation gain: `K_ROT = 1.5`
- Guidewire channel `CH_GUIDE = 2`, catheter channel `CH_CATH = 4`
- Fallback timestep: `DEFAULT_DT = 0.1 s` (used when a log has no `dt` column)
- Resend interval inside a hold window: `TX_DT = 0.01 s` (100 Hz refresh, not alternation)
- Channel alternation granularity: `INTERLEAVE = 1` (dual-channel only — see §5.5)
- Clamp/release/settle: `CLAMP_TIME_S = RELEASE_TIME_S = 2.5 s`, `SETTLE_S = 0.2 s`

`control.py` (single channel, CH2):

- Default translation speed: `25 mm/s` (distance API only)
- Default rotation rate: `90 deg/s` (angle API only)
- Log replay: one row = `dt` of wall-clock, translation + rotation in the same frame
- `cath_speed_cmd` optional in the log

`dual_control.py` (two channels, log replay):

- Control step: per-row `dt` in full on each channel, alternating in `INTERLEAVE` slices
  (`2 * dt` wall-clock, displacement independent of `INTERLEAVE`)
- No default speeds — every value comes from the log
- `cath_speed_cmd` required in the log

### 9.1 Two ways of applying `K_ROT`

Both files use the same calibration model, `theta_actual ≈ K_ROT * omega_eff * t`, but apply it
at different points, because one controls duration and the other does not:

- `control.py` picks its own duration, so it compensates there:
  `t = |theta| / (K_ROT * omega_eff)` — commands the requested rate for a shorter time.
- `dual_control.py` takes each step's duration from the log (`dt`), so it compensates the rate
  instead: `w_cmd = w_des / K_ROT` — commands a slower rate for the full step.

These are equivalent: holding `w_des / K_ROT` for `t` sweeps the same angle as holding `w_des`
for `t / K_ROT`. The difference is not a discrepancy between the two files.

If the robot behavior changes (different disposables, different load, different instrument),
re-run calibration and update `K_ROT` (and `TRANS_SPEED_MAX` / default speeds if needed) in
**both** files — they hold independent copies of these constants.

---

## 10) Example usage

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

Speed + `dt` instead of distance, on CH2 only:

```python
from control import run_from_log, run_from_cmds
from robot_core import StepCmd

run_from_log("data/control_logs.txt", assume_units="rad")

run_from_cmds([
    StepCmd(v2_mm_s=+10, w2_deg_s=-90, v4_mm_s=0, dt=0.1),   # 1.0 mm, -9 deg
    StepCmd(v2_mm_s=+10, w2_deg_s=-30, v4_mm_s=0, dt=0.5),   # 5.0 mm, -15 deg
])
```

### 10.1 Running the scripts in this repo

From the repository root:

```bash
python3 control.py              # scripted demo sequence (CH2)
python3 dual_control.py         # replay data/control_logs.txt on CH2 + CH4
python3 guidewire_panel.py      # render the replay to videos/ (no robot needed)

python3 test/test_communication.py
python3 test/test_control.py
python3 test/test_translation.py
python3 test/test_rotation.py
```

Notes:

- These are hardware-in-the-loop scripts (not unit tests). Only `guidewire_panel.py` runs
  without the robot.
- `test/test_translation.py` and `test/test_rotation.py` append results into `test/data/*.csv`.
- All paths in these scripts are relative to the repository root, so run them from there.

---

## 11) Troubleshooting

### No motion / only clamp works

- Ensure robot UI UDP dialog is **Start**-ed.
- Ensure you’re targeting a channel that supports rotation (vendor doc: rotation typically channel 2 and 3).
- Make sure channel selection in UI matches your script’s `CH` (or `CH_GUIDE` / `CH_CATH`).

### Replay rotates far too slowly (nearly constant slow spin)

Symptom: every step commands roughly the same small rotation regardless of the log.

Cause: the log is being read as deg/s when it is rad/s. Check that `assume_units="rad"` in
`dual_control.py` and `ASSUME_UNITS = "rad"` in `guidewire_panel.py`. See §5.3.

### `Missing column '<name>' in log header`

The log header doesn't contain one of the four required columns. Check the first line of your
log against §5.2 — the parser matches names exactly and is whitespace-separated.

### The panel shows fewer steps than the replay executes

Expected, and it applies to `guidewire_panel.py` only. The panel still decimates rows onto its own
`DT_TARGET = 0.2` grid using `wall_time`, so the video shows about half the steps the robot
executes. `dual_control.py` no longer decimates: it replays **every** data row, one move per row
(§5.4).

### No telemetry

- Telemetry requires the robot UI UDP dialog remote settings to be correct:
  - Remote IP: your PC IP
  - Remote Port: your bind port (54111)
- Firewall inbound UDP rule for 54111.

### Motion timing feels off

- Translation: check the -50..+50 mm/s still holds for your current setup.
- Rotation: if angles drift again, re-estimate `k_rot` using the 180° test at a few speeds.

---

## 12) Files

Controllers:

- `robot_core.py`: **shared core** — network settings, calibration, command encoding, `StepCmd`,
  the speed+`dt` log parser, and the UDP transport (`RobotLink`). Imported by both controllers, so
  the encoders and parser exist in one place only.
- `control.py`: single-channel (CH2) — distance/angle API, CH2 log replay, demo motion sequence
- `dual_control.py`: dual-channel (CH2 + CH4) log replay, time-shared with the full `dt` per channel
- `guidewire_panel.py`: pygame 4-quadrant visualization + MP4 export of a replay. Standalone — it
  keeps its own parser copy and still decimates on `wall_time`, so it does not use `dt`.

Data:

- `data/control_logs.txt`: current replay input log
- `data/control_logs_V0.txt`: earlier run, same format
- `videos/`: rendered output from `guidewire_panel.py` (full panel + one file per quadrant)

Tests and calibration (hardware-in-the-loop):

- `test/test_communication.py`: telemetry/connectivity sanity check
- `test/test_control.py`: simple end-to-end sequence (clamp, translation, rotation, release)
- `test/test_translation.py`: translation sweep script (logs expected/measured displacement)
- `test/test_rotation.py`: 180-degree rotation sweep script with `k_r` compensation
- `test/data/translation_accuracy_tables.csv`: translation sweep results
- `test/data/rotation_accuracy_tables.csv`: rotation sweep results

Reference:

- `doc/AVIAR_User's Manual_LNR.pdf`: vendor/user manual copy

---

## 13) Disclaimer

This is research/prototyping code. It is open-loop. Use in a safe test setup, start with low speeds,
and assume the robot can do surprising things when friction changes.
