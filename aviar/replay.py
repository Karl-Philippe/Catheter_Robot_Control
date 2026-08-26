"""
Running a stream of StepCmds against a Robot.

One session: clamp, stream every row back-to-back, release. This is the whole
lifecycle an entry point needs, so `control.py` and `dual_control.py` are one
call each.
"""

from typing import Iterable

from .protocol import StepCmd
from .robot import Robot


def replay(robot: Robot, stream: Iterable[StepCmd],
           max_steps: int | None = None, label: str = "") -> int:
    """
    Clamp, stream every StepCmd, then release. Returns the number of steps run.

    Motion is CONTINUOUS: no stop is inserted between steps, and step() already
    occupies each row's dt, so no extra pacing is needed. The only stop is the
    final one before releasing the clamp.

    The robot is always closed, even if the stream raises.
    """
    chans = "+".join(f"CH{c}" for c in robot.channels)
    try:
        robot.stop(0.2)
        print(f"Clamping {chans}...")
        robot.clamp()

        n = 0
        for cmd in stream:
            robot.step(cmd)
            n += 1
            if max_steps is not None and n >= max_steps:
                break

        robot.stop(0.5)   # halt before unclamping (not a mid-sequence pause)
        print(f"Releasing {chans}...")
        robot.release()

        print(f"Done. Replayed {n} steps{' from ' + label if label else ''}")
        return n
    finally:
        robot.close()
