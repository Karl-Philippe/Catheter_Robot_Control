"""
USE CASE: replay a one-instrument log on the guidewire (CH2).

Log header:
    dt  device_translation_speed_cmd  device_rotation_speed_cmd

Edit the SETTINGS block below, then run:

    python3 control.py

Set LIVE_PANEL = True to watch it on screen while it runs.
"""

from aviar import CH_GUIDE, Robot, iter_stepcmds_from_log, replay
from aviar.retract import replay_with_retract

# =========================
# SETTINGS
# =========================
LOG_PATH = "data/control_branch_3.txt"
MAX_STEPS = None            # None = the whole log

# Log columns to read (plus the optional "dt" column).
TRANS_COL = "device_translation_speed_cmd"
ROT_COL = "device_rotation_speed_cmd"
ROT_UNITS = "rad"           # units of ROT_COL: "rad" (rad/s) or "deg" (deg/s)

# Retrace the path backwards when the run finishes, so the instrument ends
# where it started. The reverse pass runs at the recorded speed.
RETRACT = True
RETRACT_PAUSE_S = 2.0       # pause between the forward and reverse passes

# Show the live screen while the run happens (needs pygame). The window stays
# up at the end until you close it; ESC or closing it stops the run early.
LIVE_PANEL = True
INITIAL_ANGLE_DEG = 0.0     # where the instrument starts, since you set it by hand
INITIAL_POS_MM = 0.0

# No INTERLEAVE here: only one channel is driven, so there is nothing to
# alternate with. Translation and rotation go out in the SAME frame and both
# run for the whole dt, so a row covers exactly v*dt mm and w*dt deg and takes
# dt of wall-clock. See dual_control.py for the two-channel case.


CHANNELS = (CH_GUIDE,)


def main():
    """Replay LOG_PATH on CH2."""
    rows = iter_stepcmds_from_log(LOG_PATH, TRANS_COL, ROT_COL,
                                  assume_units=ROT_UNITS)

    if LIVE_PANEL:
        # imported here so the headless path never needs pygame
        from aviar.livepanel import run_live
        return run_live(lambda link: Robot(channels=CHANNELS, link=link),
                        rows, CHANNELS, max_steps=MAX_STEPS, label=LOG_PATH,
                        initial_angle_deg=INITIAL_ANGLE_DEG,
                        initial_pos_mm=INITIAL_POS_MM,
                        retract=RETRACT, retract_pause_s=RETRACT_PAUSE_S)

    def make_robot():
        return Robot(channels=CHANNELS)

    if RETRACT:
        return replay_with_retract(make_robot, list(rows), RETRACT_PAUSE_S,
                                   max_steps=MAX_STEPS, label=LOG_PATH)
    return replay(make_robot(), rows, max_steps=MAX_STEPS, label=LOG_PATH)


if __name__ == "__main__":
    main()
