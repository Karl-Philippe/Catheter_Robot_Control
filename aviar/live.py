"""
Watching a run as it happens.

The robot code blocks: a step holds for dt (2*dt on two channels) and clamping
freezes for ~2.7 s, so a display cannot share its thread. The arrangement is:

    main thread    pygame, 60 fps, reads LiveState
    worker thread  the robot, writes LiveState

`LiveState` is the only thing they share. Writes are a lock plus a few float
assignments -- measured at ~0.1 us against the send loop's 10 000 us frame
interval, so the tap is invisible to robot timing (verified: a busy 60 fps
render thread leaves step timing unchanged within noise).

Two taps, neither of which needs a change to the library:

    TappedLink   subclasses RobotLink to see every frame as it goes out
                 (Robot already accepts link=...)
    tap_stream   wraps the StepCmd iterable to see each row's PHYSICAL values,
                 which are encoded away before they reach the link
"""

import threading
import time
from collections import deque
from typing import Iterable, Iterator

from .config import CH_CATH, CH_GUIDE
from .link import RobotLink
from .protocol import StepCmd

# Run phases, for display.
PHASE_IDLE = "IDLE"
PHASE_CLAMP = "CLAMP"
PHASE_RUN = "RUN"
PHASE_PAUSE = "PAUSE"
PHASE_RETRACT = "RETRACT"
PHASE_RELEASE = "RELEASE"
PHASE_DONE = "DONE"

HISTORY_S = 6.0        # seconds of strip-chart history to keep
_HISTORY_MAX = 2000


class LiveState:
    """
    What the robot is doing right now, safe to read from another thread.

    Position and angle are integrated open-loop from the commanded speeds --
    the robot reports nothing back, so this is an estimate, not a measurement.
    It starts from the initial values you pass, because the instrument is set
    up by hand.
    """

    def __init__(self, initial_angle_deg: float = 0.0,
                 initial_pos_mm: float = 0.0,
                 total_rows: int | None = None,
                 channels: tuple[int, ...] = (CH_GUIDE,)):
        self._lock = threading.Lock()
        self.channels = tuple(channels)

        # commanded speeds of the row being executed
        self.v2_mm_s = 0.0
        self.w2_deg_s = 0.0
        self.v4_mm_s = 0.0
        self.dt = 0.0

        # integrated estimate of where things are
        self.initial_angle_deg = float(initial_angle_deg)
        self.initial_pos_mm = float(initial_pos_mm)
        self.guide_pos_mm = float(initial_pos_mm)
        self.guide_angle_deg = float(initial_angle_deg)
        self.cath_pos_mm = float(initial_pos_mm)

        # progress
        self.phase = PHASE_IDLE
        self.row = 0
        self.total_rows = total_rows
        self.t_start = None
        self.elapsed_s = 0.0

        # protocol view: last frame per channel, and clamp state per channel
        self.last_frame = {}          # ch -> (msg, clamp, trans_cmd, rot_cmd)
        self.clamp_state = {}         # ch -> 0 idle / 1 released / 2 clamped
        self.active_ch = None         # channel of the most recent frame
        self.frame_count = 0

        # strip chart
        self.history = deque(maxlen=_HISTORY_MAX)   # (t, v2, w2, v4)

        # set by the UI to ask the robot thread to finish early
        self.stop_requested = threading.Event()

    # -- writes, from the robot thread ---------------------------------
    def begin(self):
        with self._lock:
            self.t_start = time.time()

    def set_phase(self, phase: str):
        with self._lock:
            self.phase = phase

    def set_total(self, total_rows: int | None):
        with self._lock:
            self.total_rows = total_rows

    def on_row(self, cmd: StepCmd):
        """One log row is about to be executed."""
        now = time.time()
        with self._lock:
            self.v2_mm_s = cmd.v2_mm_s
            self.w2_deg_s = cmd.w2_deg_s
            self.v4_mm_s = cmd.v4_mm_s
            self.dt = cmd.dt
            self.row += 1

            # Open-loop integration of the COMMANDED speeds. k_rot is not
            # applied: it corrects the robot's tendency to over-rotate, so the
            # angle actually swept is the raw commanded w2 * dt. Dividing here
            # too would double-count the correction.
            self.guide_pos_mm += cmd.v2_mm_s * cmd.dt
            self.guide_angle_deg += cmd.w2_deg_s * cmd.dt
            self.cath_pos_mm += cmd.v4_mm_s * cmd.dt

            if self.t_start is not None:
                self.elapsed_s = now - self.t_start
            self.history.append((now, cmd.v2_mm_s, cmd.w2_deg_s, cmd.v4_mm_s))
            cutoff = now - HISTORY_S
            while self.history and self.history[0][0] < cutoff:
                self.history.popleft()

    def on_frame(self, ch: int, clamp: int, trans_cmd: int, rot_cmd: int, msg: int):
        """One UDP frame went out. Called at ~100 Hz; keep this cheap."""
        with self._lock:
            self.last_frame[ch] = (msg, clamp, trans_cmd, rot_cmd)
            self.clamp_state[ch] = clamp
            self.active_ch = ch
            self.frame_count += 1

            # replay() clamps and releases around the rows, and those phases
            # take ~2.7 s each -- long enough that the panel would otherwise sit
            # on a stale phase. The clamp field says which one is happening.
            if clamp == 2 and self.phase in (PHASE_IDLE, PHASE_DONE):
                self.phase = PHASE_CLAMP
            elif clamp == 1:
                self.phase = PHASE_RELEASE

    def finish(self):
        with self._lock:
            self.phase = PHASE_DONE
            self.v2_mm_s = self.w2_deg_s = self.v4_mm_s = 0.0

    # -- reads, from the UI thread -------------------------------------
    def snapshot(self) -> dict:
        """A consistent copy of everything the renderer needs."""
        with self._lock:
            elapsed = (time.time() - self.t_start) if self.t_start else 0.0
            return {
                "channels": self.channels,
                "v2_mm_s": self.v2_mm_s,
                "w2_deg_s": self.w2_deg_s,
                "v4_mm_s": self.v4_mm_s,
                "dt": self.dt,
                "guide_pos_mm": self.guide_pos_mm,
                "guide_angle_deg": self.guide_angle_deg,
                "cath_pos_mm": self.cath_pos_mm,
                "initial_angle_deg": self.initial_angle_deg,
                "phase": self.phase,
                "row": self.row,
                "total_rows": self.total_rows,
                "elapsed_s": elapsed,
                "last_frame": dict(self.last_frame),
                "clamp_state": dict(self.clamp_state),
                "active_ch": self.active_ch,
                "frame_count": self.frame_count,
                "history": list(self.history),
            }


class TappedLink(RobotLink):
    """
    A RobotLink that reports every frame to a LiveState before sending it.

    Pass it in as `Robot(link=TappedLink(state))` -- no library change needed.
    """

    def __init__(self, state: LiveState):
        super().__init__()
        self.state = state

    def send_frame(self, ch: int, clamp: int, trans_cmd: int, rot_cmd: int):
        super().send_frame(ch, clamp, trans_cmd, rot_cmd)
        self.state.on_frame(ch, clamp, trans_cmd, rot_cmd, self.msg)


def tap_stream(rows: Iterable[StepCmd], state: LiveState,
               phase: str = PHASE_RUN) -> Iterator[StepCmd]:
    """
    Yield each row after telling `state` about it.

    Needed because the physical speeds are encoded to protocol integers inside
    Robot.step(), so a link-level tap only ever sees command numbers. Wrapping
    the stream catches the values in mm/s and deg/s instead.

    Also honours `state.stop_requested`, so closing the window ends the run at
    the next row boundary rather than mid-move.
    """
    for cmd in rows:
        if state.stop_requested.is_set():
            return
        # Set on every row, not once up front: replay() clamps before the first
        # row arrives, and TappedLink shows CLAMP during that. Setting the phase
        # here means it flips to RUN exactly when motion actually starts.
        state.set_phase(phase)
        state.on_row(cmd)
        yield cmd
