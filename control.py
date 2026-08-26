"""
USE CASE: replay a one-instrument log on the guidewire (CH2).

Log header:
    dt  device_translation_speed_cmd  device_rotation_speed_cmd

Edit the SETTINGS block below, then run:

    python3 control.py
"""

from aviar import CH_GUIDE, Robot, iter_stepcmds_from_log, replay

# =========================
# SETTINGS
# =========================
LOG_PATH = "data/device_logs.txt"
MAX_STEPS = None            # None = the whole log

# Log columns to read (plus the optional "dt" column).
TRANS_COL = "device_translation_speed_cmd"
ROT_COL = "device_rotation_speed_cmd"
ROT_UNITS = "rad"           # units of ROT_COL: "rad" (rad/s) or "deg" (deg/s)

# No INTERLEAVE here: only one channel is driven, so there is nothing to
# alternate with. Translation and rotation go out in the SAME frame and both
# run for the whole dt, so a row covers exactly v*dt mm and w*dt deg and takes
# dt of wall-clock. See dual_control.py for the two-channel case.


def main():
    """Replay LOG_PATH on CH2."""
    robot = Robot(channels=(CH_GUIDE,))
    rows = iter_stepcmds_from_log(LOG_PATH, TRANS_COL, ROT_COL,
                                  assume_units=ROT_UNITS)
    return replay(robot, rows, max_steps=MAX_STEPS, label=LOG_PATH)


if __name__ == "__main__":
    main()
