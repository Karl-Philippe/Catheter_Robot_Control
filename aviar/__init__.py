"""
aviar -- control library for the AVIAR catheter robot (UDP, open loop).

Typical use:

    from aviar import Robot, StepCmd, replay, iter_stepcmds_from_log, CH_GUIDE

    # replay a recorded log
    robot = Robot(channels=(CH_GUIDE,))
    replay(robot, iter_stepcmds_from_log("log.txt", "speed_cmd", "rot_cmd"))

    # or drive it by hand
    with Robot() as robot:
        robot.clamp()
        robot.translate_by(+40)
        robot.rotate_by(-90)
        robot.release()

Modules:
    config     constants and Params (edit tuning values here)
    protocol   command encoding and StepCmd (pure, testable)
    link       RobotLink, the UDP transport
    robot      Robot, the controller
    logs       reading recorded logs into StepCmds
    replay     running a StepCmd stream as one clamped session
    retract    reversing a run to retrace it back to the start
    live       LiveState + TappedLink, for watching a run as it happens
    panel      pygame drawing primitives (needs pygame)

`live` and `panel` are not imported here so the core stays dependency-free --
import them directly (`from aviar.live import LiveState`).
"""

from .config import (
    CH_CATH,
    CH_GUIDE,
    CLAMP_TIME_S,
    DEFAULT_DT,
    DEFAULT_ROT_SPEED,
    DEFAULT_TRANS_SPEED,
    INTERLEAVE,
    K_ROT,
    RELEASE_TIME_S,
    ROT_RATE_MAX,
    SETTLE_S,
    TRANS_SPEED_MAX,
    TX_DT,
    Params,
)
from .link import RobotLink
from .logs import iter_stepcmds_from_log
from .protocol import StepCmd, encode_rotation_signed, encode_translation_signed
from .replay import replay
from .retract import net_travel, reverse
from .robot import Robot

__all__ = [
    # controller
    "Robot",
    "Params",
    "RobotLink",
    # data + protocol
    "StepCmd",
    "encode_translation_signed",
    "encode_rotation_signed",
    # logs + running
    "iter_stepcmds_from_log",
    "replay",
    # retract
    "reverse",
    "net_travel",
    # channels
    "CH_GUIDE",
    "CH_CATH",
    # tuning
    "DEFAULT_DT",
    "INTERLEAVE",
    "TX_DT",
    "K_ROT",
    "TRANS_SPEED_MAX",
    "ROT_RATE_MAX",
    "CLAMP_TIME_S",
    "RELEASE_TIME_S",
    "SETTLE_S",
    "DEFAULT_TRANS_SPEED",
    "DEFAULT_ROT_SPEED",
]
