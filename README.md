# Catheter Robot UDP Control (AVIAR)

This repository contains a small Python controller that drives the AVIAR robot **over UDP** in **open loop**.  
You control the robot in **physical units** (mm, deg) and **unsigned speeds** (mm/s, deg/s). The code converts these to the robot’s UDP command format.

The code is a small **library** plus three **use-case scripts** built on it:

```
aviar/              the library (see §7)
  config.py         constants and Params  -- edit tuning values here
  protocol.py       command encoding, StepCmd
  link.py           RobotLink, the UDP transport
  robot.py          Robot, the controller
  logs.py           reading recorded logs
  replay.py         running a StepCmd stream
  retract.py        reversing a run to retrace it back to the start
  live.py           LiveState + TappedLink, for watching a run happen
  livepanel.py      run_live(), the live screen
  panel.py          pygame drawing primitives (shared by both panels)

control.py          USE CASE: replay a 1-instrument log on CH2          (§5)
dual_control.py     USE CASE: replay a 2-instrument log on CH4+CH2      (§5)
manual_control.py   USE CASE: drive one channel by hand, in mm and deg  (§7.2)
live_panel.py       shortcut for dual_control.py with the screen on   (§7.4)
```

Each script names its log columns or its list of moves and calls the library; all the machinery
lives in `aviar/`.

Supporting files:

- Offline replay visualization / video export: `guidewire_panel.py`
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

Each controller declares the columns it reads as constants at the top of its own file, and passes
them to the shared parser. To change a header, edit those constants — nothing else. Every other
column in the log is ignored.

| Controller | Constants | Columns read |
| --- | --- | --- |
| `control.py` (CH2) | `TRANS_COL`, `ROT_COL` | `device_translation_speed_cmd`, `device_rotation_speed_cmd` |
| `dual_control.py` (CH2+CH4) | `TRANS_COL`, `ROT_COL`, `CATH_COL` | `guide_speed_cmd`, `guide_rotation_speed_cmd`, `cath_speed_cmd` |

Plus the optional `dt` column in both cases (§5.4). Translation columns are mm/s; rotation columns
are **rad/s** by default (§5.3). Running the wrong log through a controller raises
`Missing column '<name>'` naming the column it wanted.

`control.py`'s form is instrument-agnostic on purpose: `device_*` is whichever tool is mounted on
CH2, so the same log drives a guidewire or a catheter without renaming columns.

Columns present in the recorded dual logs but **not** used: `wall_time`, `run_id`, `pid`, `env_id`,
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

Override the fallback where the parser is called (the scripts leave it at `DEFAULT_DT`):

```python
iter_stepcmds_from_log(LOG_PATH, TRANS_COL, ROT_COL, default_dt=0.05)
```

Or change `DEFAULT_DT` in `aviar/config.py` to affect every replay.

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
INTERLEAVE = 4              # SETTINGS block of dual_control.py
```

`control.py` has no `INTERLEAVE`: with one channel there is nothing to alternate with.

Or change the `INTERLEAVE` default in `aviar/config.py` to affect every replay.

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

This clamps both channels, replays `data/control_logs.txt`, then releases. For a short dry run,
edit the SETTINGS block:

```python
MAX_STEPS = 20              # None = the whole log
```

You can also bypass the log and replay an explicit list, straight against the library:

```python
from aviar import CH_CATH, CH_GUIDE, Robot, StepCmd, replay

replay(Robot(channels=(CH_CATH, CH_GUIDE)), [
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

## 7) The `aviar` library

Everything importable lives in `aviar/`. The scripts in the repository root are use cases built on
it; write your own the same way.

```python
from aviar import Robot, StepCmd, replay, iter_stepcmds_from_log, CH_GUIDE, CH_CATH
```

| Module | Holds | Touches |
| --- | --- | --- |
| `aviar/config.py` | constants + `Params` | nothing (leaf) |
| `aviar/protocol.py` | `encode_*`, `StepCmd`, clamp values | pure functions, unit-testable |
| `aviar/link.py` | `RobotLink` | the **only** socket in the package |
| `aviar/robot.py` | `Robot` | the controller |
| `aviar/logs.py` | `iter_stepcmds_from_log` | file reading |
| `aviar/replay.py` | `replay()` | one clamped session |

### 7.1 `Robot` — one class, channel-driven

```python
Robot()                              # CH2 alone      — control.py
Robot(channels=(CH_CATH, CH_GUIDE))  # CH4 then CH2   — dual_control.py
Robot(channels=(CH_CATH,))           # any one channel — manual_control.py
```

`channels` is in **command order**. Only `step()` depends on how many channels there are;
clamp/release/stop, the calibration moves and `replay()` work the same either way.

Two ways to command motion:

| Method | You give | Used by |
| --- | --- | --- |
| `step(cmd)` | speed + duration (one `StepCmd`) | log replay, continuous |
| `translate_by` / `rotate_by` | distance + speed | calibration, settles after each move |

`Robot` is a context manager, so `with Robot() as robot:` closes the socket for you.

The calibration moves act on `cal_channel` — the guidewire whenever this `Robot` drives it, else
the single channel it does drive. Deliberately *not* `channels[0]`, which is the catheter when
both are driven. Pass `channel=` to override per move.

### 7.2 `manual_control.py` — driving by hand

Edit the two blocks at the top and run it:

```python
CHANNEL = CH_GUIDE      # or CH_CATH

SEQUENCE = [
    ("translate_by", (+40,)),        # 40 mm forward @ default 25 mm/s
    ("translate_by", (-40, 35)),     # 40 mm backward @ 35 mm/s
    ("rotate_by",    (+180,)),       # 180 deg CW @ default 90 deg/s
    ("rotate_by",    (-180, 360)),   # 180 deg CCW @ 360 deg/s
]
```

```bash
python3 manual_control.py
```

It clamps, runs each move in order, and releases. Each move settles for `SETTLE_S` afterwards, so
these are **discrete** positioning moves — deliberately not the continuous path used for replay.

### 7.3 Log replay on CH2 (one instrument)

```bash
python3 control.py
```

Everything it does is set in the SETTINGS block at the top of the file — the log path, the
column names, and `MAX_STEPS`. `main()` takes no arguments: edit the block, run the file.

Reads the columns named by `TRANS_COL` / `ROT_COL` at the top of `control.py`, plus the optional
`dt` (§5.2). A dual log is rejected here — its columns have different names.

**Motion is continuous.** Because only one channel is driven there is no time-sharing at all:
translation and rotation go out in the **same frame** and both run for the whole `dt`, so a row
covers exactly `v * dt` mm and `w * dt` deg, and a step occupies `dt` of wall-clock (not `2 * dt`
as in §5.5). Rows are streamed back-to-back with no stop between them — the command simply changes
at each row boundary while the axis keeps moving. The only stop is the final one before releasing
the clamp.

`INTERLEAVE` does **not** apply here: there is no second channel to alternate with, so it is a
dual-channel concept only (§5.5). `robot.step(cmd)` executes a single `StepCmd` directly.

### 7.4 Live panel — watching a run

The panel is a **switch on the replay scripts**, not a separate one. Set `LIVE_PANEL = True` in
the SETTINGS block of `control.py` or `dual_control.py` and run it as usual:

```bash
python3 control.py         # CH2, with the screen
python3 dual_control.py    # CH4 + CH2, with the screen
python3 live_panel.py      # shortcut: dual_control.py with LIVE_PANEL forced on
```

It replays whatever that script's settings say — same log, columns, interleave and retract — so
there is one place to configure a run. Set `INITIAL_ANGLE_DEG` / `INITIAL_POS_MM` to wherever you
set the instrument up by hand; the panel tracks from there.

With `LIVE_PANEL = False` (the default) nothing changes and **pygame is never imported**, so the
headless path keeps working on a machine without it.

The same four quadrants as `guidewire_panel.py`, but driven by the commands actually being sent
rather than by a file read in advance, plus a status strip: phase and progress, the integrated
insertion/angle estimate, the raw UDP frames per channel with clamp state, and a rolling speed
chart. Close the window or press ESC to stop — the run ends at the next row boundary and the robot
is released and closed properly. When the run finishes the window stays up showing the final state
until you close it.

**Robot timing is not affected.** The robot runs in a worker thread and the panel on the main
thread, sharing one lock-guarded `LiveState`. The tap costs ~0.24 µs per frame against the 10 ms
send interval. Rendering is the part that could interfere — it holds the GIL — so the live panel
renders at HD/30 fps rather than the exporter's FHD/60 and shortens `sys.setswitchinterval`.
Measured: step timing lands within **0.6 ms** of running with no panel at all. Raising
`AVIAR_RESOLUTION` or `FPS` trades that margin away.

### 7.5 Retract — retracing a run backwards

Set `RETRACT = True` in `control.py`, `dual_control.py` or `live_panel.py`. The run replays, pauses
for `RETRACT_PAUSE_S`, then runs `aviar.retract.reverse(rows)`: the same rows last-to-first with
every speed negated. Each row keeps its own `dt`, so the reverse covers exactly the same distance
and angle and the instrument ends where it began (verified: 0.000 mm, 0.00 deg).

```python
from aviar import reverse, net_travel
net_travel(rows + reverse(rows))     # ~(0.0, 0.0, 0.0)
```

**There is no speed multiplier, deliberately.** Retracting faster means scaling the speeds up, and
rotation is already near the 360 deg/s protocol ceiling: on `data/control_logs.txt` a 2x factor
pushes 22 of 70 rows past it, they silently under-rotate, and the "reverse" leaves ~10 deg of
residual twist. Translation would survive (10 mm/s is far from its 50 mm/s limit) but rotation
would not, so the reverse runs at the recorded speed.

### 7.6 Calibration moves

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

All defined once in `aviar/config.py`:

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
- Log replay: one row = `dt` of wall-clock, translation + rotation in the same frame, continuous
  across rows (no stop between steps)
- Log format: single-instrument `device_*` columns (§5.2); `INTERLEAVE` does not apply

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
for `t / K_ROT`. Both live on the same class — `rotate_by` shortens the duration, `step()`
compensates the rate — so this is a deliberate difference between the two methods, not a
discrepancy between files.

If the robot behavior changes (different disposables, different load, different instrument),
re-run calibration and update `K_ROT` (and `TRANS_SPEED_MAX` / default speeds if needed) in
`aviar/config.py` — there is a single copy, read by every entry point. Per-run overrides go
through `Params`, e.g. `Robot(params=Params(k_rot=1.8))`.

---

## 10) Example usage

```python
from aviar import Robot

robot = Robot()          # CH2; or Robot(channels=(CH_CATH,))
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

Speed + `dt` instead of distance, on CH2 only — continuous, no stop between rows:

```python
from aviar import CH_GUIDE, Robot, StepCmd, replay
import control

control.main()                                             # runs its SETTINGS block

replay(Robot(channels=(CH_GUIDE,)), [                      # or an explicit list
    StepCmd(v2_mm_s=+10, w2_deg_s=-90, v4_mm_s=0, dt=0.1),   # 1.0 mm, -9 deg
    StepCmd(v2_mm_s=+10, w2_deg_s=-30, v4_mm_s=0, dt=0.5),   # 5.0 mm, -15 deg
])
```

### 10.1 Running the scripts in this repo

From the repository root:

```bash
python3 control.py              # replay data/device_logs.txt on CH2
python3 dual_control.py         # replay data/control_logs.txt on CH4 + CH2
python3 manual_control.py       # run the hand-written move sequence on one channel
python3 live_panel.py           # dual_control.py with the live screen on
python3 guidewire_panel.py      # render an offline replay to videos/ (no robot needed)

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

The log header doesn't contain a required column. Check the first line of your log against §5.2 —
the parser matches names exactly and is whitespace-separated.

Most often this means the **wrong controller for the log**: the two formats use different column
names, so a single-instrument log passed to `dual_control.py` (or vice versa) fails here. The error
names the column that was expected — compare it against the constants at the top of the controller
you ran (§5.2).

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

The library (`aviar/`, see §7) — contains no log column names of its own:

- `aviar/config.py`: every constant, plus `Params` for per-run overrides
- `aviar/protocol.py`: command encoding and `StepCmd` — pure, no I/O
- `aviar/link.py`: `RobotLink`, the only socket in the package
- `aviar/robot.py`: the `Robot` controller
- `aviar/logs.py`: `iter_stepcmds_from_log`, caller-named columns
- `aviar/replay.py`: `replay()`, one clamped session
- `aviar/retract.py`: `reverse()` — retrace a run back to its start
- `aviar/live.py`: `LiveState` + `TappedLink` — watch a run without disturbing its timing
- `aviar/livepanel.py`: `run_live()` — the window, the worker thread and the status strip
- `aviar/panel.py`: pygame drawing primitives, shared by both panels

Use cases (repository root, ~40 lines each):

- `control.py`: replay a one-instrument log on CH2 — names its `device_*` columns
- `dual_control.py`: replay a two-instrument log on CH4+CH2 — names its `guide_*`/`cath_*` columns
- `manual_control.py`: drive one channel by hand in mm/deg — a channel and a list of moves
- `live_panel.py`: shortcut for `dual_control.py` with `LIVE_PANEL` on
- `guidewire_panel.py`: pygame 4-quadrant visualization + MP4 export of a replay. Standalone — it
  keeps its own parser copy and still decimates on `wall_time`, so it does not use `dt`.

Data:

- `data/control_logs.txt`: current replay input log, **two-instrument** format (`dual_control.py`)
- `data/control_logs_V0.txt`: earlier run, same two-instrument format
- `data/device_logs.txt`: one-instrument format (`control.py`)
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
