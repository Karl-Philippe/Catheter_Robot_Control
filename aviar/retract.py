"""
Retracing a run backwards.

`reverse()` turns a recorded sequence into the moves that undo it: the rows run
last-to-first with every speed negated, at the speed they were recorded at.
Because each row keeps its own dt, the reverse pass covers exactly the same
distance and angle as the forward pass and returns the instrument to where it
started.

There is deliberately no speed multiplier. Retracting faster means scaling the
speeds up, and the protocol caps rotation at ROT_RATE_MAX (360 deg/s): on a
typical log a 2x factor pushes most rows past that ceiling, they silently
under-rotate, and the "reverse" leaves ~10 deg of residual twist. Saving a few
seconds is not worth an unfaithful retrace.
"""

from dataclasses import replace
from typing import Iterable, Sequence

from .protocol import StepCmd


def reverse(rows: Iterable[StepCmd]) -> list[StepCmd]:
    """
    Return the moves that undo `rows`: reversed order, negated speeds, same dt.

    Takes and returns a list -- the input must be materialised to be reversed,
    so read the log once and reuse the list for both passes:

        rows = list(iter_stepcmds_from_log(...))
        replay(robot, rows)
        replay(robot, reverse(rows))
    """
    return [
        replace(row,
                v2_mm_s=-row.v2_mm_s,
                w2_deg_s=-row.w2_deg_s,
                v4_mm_s=-row.v4_mm_s)
        for row in reversed(list(rows))
    ]


def replay_with_retract(make_robot, rows: Sequence[StepCmd],
                        pause_s: float = 2.0, max_steps: int | None = None,
                        label: str = "") -> int:
    """
    Replay `rows`, pause, then replay the reverse so the instrument ends where
    it began. Returns the total number of steps run.

    The grip is held THROUGHOUT -- there is one clamp at the start and one
    release at the very end:

        clamp -> forward rows -> pause (still clamped) -> reversed rows -> release

    Letting go between the passes would drop the instrument and lose the
    position the retract is meant to unwind, so both passes share a single
    robot and a single clamp.

    `make_robot` is a zero-argument factory. It is called once here; the same
    robot serves both passes and is closed at the end.
    """
    from .replay import replay   # imported here to avoid a circular import

    robot = make_robot()
    try:
        n = replay(robot, rows, max_steps=max_steps, label=label,
                   release=False, close=False)

        if pause_s > 0:
            print(f"Holding clamp, pausing {pause_s:.1f}s before retract...")
            robot.stop(pause_s)

        n += replay(robot, reverse(rows), max_steps=max_steps,
                    label=f"{label} (retract)" if label else "retract",
                    clamp=False, close=False)
        return n
    finally:
        robot.close()


def net_travel(rows: Sequence[StepCmd]) -> tuple[float, float, float]:
    """
    Open-loop distance and angle a sequence covers: (guide_mm, guide_deg, cath_mm).

    Handy as a check: `net_travel(rows + reverse(rows))` should be ~(0, 0, 0).
    """
    return (
        sum(r.v2_mm_s * r.dt for r in rows),
        sum(r.w2_deg_s * r.dt for r in rows),
        sum(r.v4_mm_s * r.dt for r in rows),
    )
