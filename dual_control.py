"""
USE CASE: replay a two-instrument log on the catheter (CH4) + guidewire (CH2).

Log header:
    guide_speed_cmd  guide_rotation_speed_cmd  cath_speed_cmd  [dt]

Edit the SETTINGS block below, then run:

    python3 dual_control.py
"""

from aviar import CH_CATH, CH_GUIDE, Robot, iter_stepcmds_from_log, replay

# =========================
# SETTINGS
# =========================
LOG_PATH = "data/control_logs.txt"
MAX_STEPS = None            # None = the whole log

# Log columns to read (plus the optional "dt" column).
TRANS_COL = "guide_speed_cmd"
ROT_COL = "guide_rotation_speed_cmd"
CATH_COL = "cath_speed_cmd"
ROT_UNITS = "rad"           # units of ROT_COL: "rad" (rad/s) or "deg" (deg/s)

# How finely each row alternates CH4 <-> CH2.
# The channels cannot be driven at once, so a row is cut into INTERLEAVE slices
# per channel: CH4/CH2/CH4/CH2... Each channel is still energised for the FULL
# dt, so both cover their commanded displacement and a row takes 2*dt of
# wall-clock. INTERLEAVE changes only how finely they alternate, never how far
# they travel:
#     1  = one CH4 block then one CH2 block (coarsest)
#     4  = 8 swaps per row, the channels track each other much more closely
# Keep dt/INTERLEAVE >= 0.01 s (aviar.TX_DT) or a slice becomes a single frame.
INTERLEAVE = 4


def main():
    """Replay LOG_PATH on CH4 + CH2."""
    robot = Robot(channels=(CH_CATH, CH_GUIDE), interleave=INTERLEAVE)
    rows = iter_stepcmds_from_log(LOG_PATH, TRANS_COL, ROT_COL, CATH_COL,
                                  assume_units=ROT_UNITS)
    return replay(robot, rows, max_steps=MAX_STEPS, label=LOG_PATH)


if __name__ == "__main__":
    main()
