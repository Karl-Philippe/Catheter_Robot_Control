"""
Every tunable value for the AVIAR robot, in one place.

Nothing here imports from the rest of the package, so this module is safe to
read (or edit) on its own. Change a constant here and every entry point picks
it up; override per run with `Params` instead of editing the file.
"""

from dataclasses import dataclass
from math import pi

# =========================
# NETWORK
# =========================
ROBOT_IP = "192.0.0.12"
ROBOT_PORT = 2020
LOCAL_BIND_PORT = 54111

# =========================
# CHANNELS
# =========================
CH_GUIDE = 2   # guidewire
CH_CATH = 4    # catheter

# =========================
# TIMING
# =========================
# Control timestep: used when a log row carries no "dt" column.
DEFAULT_DT = 0.1

# Frame resend interval inside a hold window.
# The robot is driven by a continuous stream: a datagram is not a latched
# command, so a held command is re-sent every TX_DT (100 Hz) to keep the axis
# moving and to make a dropped datagram cost 10 ms instead of the whole move.
# This is the refresh rate, NOT the channel alternation rate (see INTERLEAVE).
TX_DT = 0.01

# How finely a multi-channel step alternates between channels within one row.
# Each row's dt is cut into INTERLEAVE slices per channel, alternating
# CH4/CH2/CH4/CH2..., so the swap period is dt/INTERLEAVE instead of dt.
# Each channel still accumulates the full dt of motion, so displacement is
# unchanged -- only the interleaving granularity changes.
#   1  = one CH4 block then one CH2 block (coarsest, maximally out of phase)
#   4  = 8 swaps per row, the channels track each other much more closely
# A slice shorter than TX_DT degenerates to a single frame, so keep
# dt/INTERLEAVE >= TX_DT.
INTERLEAVE = 4

# Clamp / release (open-loop)
CLAMP_TIME_S = 2.5
RELEASE_TIME_S = 2.5
SETTLE_S = 0.2

# =========================
# LIMITS / CALIBRATION
# =========================
TRANS_SPEED_MAX = 50        # mm/s magnitude
ROT_RATE_MAX = 360          # deg/s magnitude (protocol max)
K_ROT = 1.5                 # rotation gain, from the 180-degree calibration

# Defaults for the calibration moves (translate_by / rotate_by)
DEFAULT_TRANS_SPEED = 25    # mm/s
DEFAULT_ROT_SPEED = 90      # deg/s

DEG_PER_RAD = 180.0 / pi


@dataclass
class Params:
    """
    Per-run overrides. Pass one to `Robot(params=...)` to change behaviour for
    a single session without editing the constants above.
    """

    clamp_time_s: float = CLAMP_TIME_S
    release_time_s: float = RELEASE_TIME_S
    settle_s: float = SETTLE_S
    k_rot: float = K_ROT
    default_trans_speed: float = DEFAULT_TRANS_SPEED
    default_rot_speed: float = DEFAULT_ROT_SPEED
