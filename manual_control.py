"""
USE CASE: drive one channel by hand, in physical units. For calibration.

Pick a CHANNEL, list the moves you want in SEQUENCE, run the file:

    python3 manual_control.py

You give distances and angles; the duration is worked out open-loop from the
speed. Each move settles for SETTLE_S afterwards, so the moves are discrete --
this is deliberately NOT the continuous path that control.py / dual_control.py
use for log replay.

    translate_by(distance_mm, speed_mm_s=None)   + forward, - backward
    rotate_by(angle_deg, rate_deg_s=None)        + CW,      - CCW

Speeds default to 25 mm/s and 90 deg/s. Rotation angles are corrected by
K_ROT; see README §9.1.
"""

from aviar import CH_CATH, CH_GUIDE, Robot  # noqa: F401  (CH_CATH for editing)

# --- which instrument -----------------------------------------------------
CHANNEL = CH_GUIDE      # CH_GUIDE (2) or CH_CATH (4)

# --- the moves, in order --------------------------------------------------
# (method name, positional args) -- edit freely.
SEQUENCE = [
    ("translate_by", (+40,)),        # 40 mm forward @ default 25 mm/s
    ("translate_by", (-40, 35)),     # 40 mm backward @ 35 mm/s
    ("rotate_by",    (+180,)),       # 180 deg CW @ default 90 deg/s
    ("rotate_by",    (-180, 360)),   # 180 deg CCW @ 360 deg/s
]


def run(sequence=SEQUENCE, channel=CHANNEL):
    """Clamp, run each move in order, release."""
    with Robot(channels=(channel,)) as robot:
        print(f"Clamping CH{channel}...")
        robot.clamp()

        for name, args in sequence:
            getattr(robot, name)(*args)

        print(f"Releasing CH{channel}...")
        robot.release()
        print(f"Done. {len(sequence)} moves on CH{channel}.")


if __name__ == "__main__":
    run()
