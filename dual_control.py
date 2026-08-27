"""
USE CASE: replay a two-instrument log on the catheter (CH4) + guidewire (CH2).

Log header:
    guide_speed_cmd  guide_rotation_speed_cmd  cath_speed_cmd  [dt]

Edit the SETTINGS block below, then run:

    python3 dual_control.py

Set LIVE_PANEL = True to watch it on screen while it runs.
"""

from aviar import CH_CATH, CH_GUIDE, Robot, iter_stepcmds_from_log, replay
from aviar.retract import replay_with_retract

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

# Retrace the path backwards when the run finishes, so the instrument ends
# where it started. The reverse pass runs at the recorded speed.
RETRACT = False
RETRACT_PAUSE_S = 2.0       # pause between the forward and reverse passes

# Show the live screen while the run happens (needs pygame). The window stays
# up at the end until you close it; ESC or closing it stops the run early.
LIVE_PANEL = True
INITIAL_ANGLE_DEG = 0.0     # where the instrument starts, since you set it by hand
INITIAL_POS_MM = 0.0


CHANNELS = (CH_CATH, CH_GUIDE)


def main():
    """Replay LOG_PATH on CH4 + CH2."""
    rows = iter_stepcmds_from_log(LOG_PATH, TRANS_COL, ROT_COL, CATH_COL,
                                  assume_units=ROT_UNITS)

    if LIVE_PANEL:
        # imported here so the headless path never needs pygame
        from aviar.livepanel import run_live
        return run_live(lambda link: Robot(channels=CHANNELS,
                                           interleave=INTERLEAVE, link=link),
                        rows, CHANNELS, max_steps=MAX_STEPS, label=LOG_PATH,
                        initial_angle_deg=INITIAL_ANGLE_DEG,
                        initial_pos_mm=INITIAL_POS_MM,
                        retract=RETRACT, retract_pause_s=RETRACT_PAUSE_S)

    def make_robot():
        return Robot(channels=CHANNELS, interleave=INTERLEAVE)

    if RETRACT:
        return replay_with_retract(make_robot, list(rows), RETRACT_PAUSE_S,
                                   max_steps=MAX_STEPS, label=LOG_PATH)
    return replay(make_robot(), rows, max_steps=MAX_STEPS, label=LOG_PATH)


if __name__ == "__main__":
    main()
