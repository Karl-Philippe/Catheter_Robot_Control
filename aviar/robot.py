"""
The Robot controller: one class, parameterised by the channels it drives.

    Robot()                              CH2 alone      -> control.py
    Robot(channels=(CH_CATH, CH_GUIDE))  CH4 then CH2   -> dual_control.py
    Robot(channels=(CH_CATH,))           any single ch  -> manual_control.py

Two ways to command motion:

    step(cmd)                 speed + duration, one log row (replay)
    translate_by/rotate_by    distance + speed, one discrete move (calibration)

Only step() depends on how many channels there are. Everything else -- clamp,
release, stop, the calibration moves -- works the same either way.
"""

import time
from typing import Iterable

from .config import (
    CH_CATH,
    CH_GUIDE,
    DEFAULT_DT,
    INTERLEAVE,
    ROT_RATE_MAX,
    TRANS_SPEED_MAX,
    TX_DT,
    Params,
)
from .link import RobotLink
from .protocol import (
    CLAMP_ENGAGE,
    CLAMP_NONE,
    CLAMP_RELEASE,
    StepCmd,
    encode_rotation_signed,
    encode_translation_signed,
)


class Robot:
    """
    Open-loop controller for one or more channels.

    `channels` is in COMMAND ORDER: with two instruments each step drives the
    catheter first, then the guidewire.
    """

    def __init__(self, channels: Iterable[int] = (CH_GUIDE,),
                 interleave: int = INTERLEAVE,
                 params: Params = Params(),
                 apply_krot: bool = True,
                 link: RobotLink | None = None):
        self.channels = tuple(channels)
        if not self.channels:
            raise ValueError("Robot needs at least one channel.")
        self.interleave = max(1, int(interleave))
        self.p = params
        self.apply_krot = apply_krot
        self.link = link if link is not None else RobotLink()

    def __repr__(self):
        chans = "+".join(f"CH{c}" for c in self.channels)
        return f"<Robot {chans} interleave={self.interleave}>"

    @property
    def cal_channel(self) -> int:
        """
        Channel the calibration moves act on.

        The guidewire whenever this Robot drives it, else the first channel --
        deliberately NOT channels[0], which is the catheter when both are
        driven, since `channels` is in command order.
        """
        return CH_GUIDE if CH_GUIDE in self.channels else self.channels[0]

    # -- low-level helpers ---------------------------------------------
    def _hold(self, ch: int, clamp: int, trans: int, rot: int, duration_s: float):
        self.link.hold_channel(ch, clamp, trans, rot, duration_s)

    def _rot_cmd(self, rate_deg_s: float) -> int:
        """Encode a rotation rate, applying the k_rot compensation."""
        w = float(rate_deg_s)
        if self.apply_krot and abs(w) > 1e-9:
            w = w / self.p.k_rot
        cmd, _ = encode_rotation_signed(w)
        return cmd

    # -- clamp / release / stop ----------------------------------------
    def stop(self, duration_s: float = 0.2):
        """Zero every channel for duration_s."""
        t_end = time.time() + float(duration_s)
        while True:
            for ch in self.channels:
                self.link.send_frame(ch, CLAMP_NONE, 0, 0)
            if time.time() >= t_end:
                break
            time.sleep(TX_DT)

    def _clamp_all(self, clamp: int, duration_s: float):
        """
        Apply a clamp state to every channel for duration_s TOTAL.

        With several channels the state is time-shared the same way motion is:
        the window is filled by cycling the channels in DEFAULT_DT slices, so
        clamping two channels still takes duration_s, not 2*duration_s.
        """
        if len(self.channels) == 1:
            self._hold(self.channels[0], clamp, 0, 0, duration_s)
            return

        t_end = time.time() + float(duration_s)
        while time.time() < t_end:
            for ch in self.channels:
                self._hold(ch, clamp, 0, 0, DEFAULT_DT)

    def clamp(self):
        """Grip the instrument(s), then settle."""
        self._clamp_all(CLAMP_ENGAGE, self.p.clamp_time_s)
        self.stop(self.p.settle_s)

    def release(self):
        """Let go of the instrument(s), then settle."""
        self._clamp_all(CLAMP_RELEASE, self.p.release_time_s)
        self.stop(self.p.settle_s)

    def close(self):
        """Stop the robot and close the socket."""
        try:
            self.stop(0.2)
        finally:
            self.link.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    # -- speed + duration (one log row) --------------------------------
    def step(self, cmd: StepCmd):
        """
        Hold cmd's speeds for cmd.dt.

        One channel: translation and rotation go out in the SAME frame and both
        run for the whole dt, so the move covers exactly v*dt mm and w*dt deg
        and takes dt of wall-clock. cmd.v4_mm_s is ignored.

        Two channels: they are NOT driven simultaneously. The step is cut into
        `interleave` slices per channel, alternating CH4/CH2/CH4/CH2..., each
        slice lasting dt/interleave. Each channel still accumulates the FULL dt,
        so each covers its own commanded displacement -- giving a channel dt/2
        would move it only half the commanded distance. Wall-clock is 2*dt
        regardless of `interleave`; `interleave` changes only how finely the
        channels alternate. Keep dt/interleave >= TX_DT or a slice degenerates
        to a single frame.
        """
        dt = float(cmd.dt)
        t2 = encode_translation_signed(cmd.v2_mm_s)   # guidewire translation
        r2 = self._rot_cmd(cmd.w2_deg_s)              # guidewire rotation

        if len(self.channels) == 1:
            self._hold(self.channels[0], CLAMP_NONE, t2, r2, dt)
            return

        t4 = encode_translation_signed(cmd.v4_mm_s)   # catheter translation
        slice_s = dt / self.interleave
        for _ in range(self.interleave):
            self._hold(CH_CATH, CLAMP_NONE, t4, 0, slice_s)
            self._hold(CH_GUIDE, CLAMP_NONE, t2, r2, slice_s)

    # -- distance + speed (one discrete move) --------------------------
    # These settle afterwards, so they are discrete positioning moves and NOT
    # part of the continuous replay path. Used for calibration.
    def translate_by(self, distance_mm: float, speed_mm_s: float | None = None,
                     channel: int | None = None):
        """
        distance_mm: signed (positive=forward, negative=backward)
        speed_mm_s: unsigned magnitude (mm/s). If None, uses default_trans_speed.
        channel: defaults to cal_channel.

        Open-loop duration: t = |distance| / speed
        """
        d = float(distance_mm)
        if abs(d) < 1e-9:
            return

        speed = self.p.default_trans_speed if speed_mm_s is None else float(speed_mm_s)
        if speed <= 0:
            raise ValueError("speed_mm_s must be > 0 (unsigned).")

        speed = max(1.0, min(float(TRANS_SPEED_MAX), speed))
        duration_s = abs(d) / speed
        cmd = encode_translation_signed(speed if d > 0 else -speed)
        ch = self.cal_channel if channel is None else channel

        print(f"[TRANS] {d:+.1f} mm @ {speed:.1f} mm/s -> cmd={cmd}, t={duration_s:.3f}s")
        self._hold(ch, CLAMP_NONE, cmd, 0, duration_s)
        self.stop(self.p.settle_s)

    def rotate_by(self, angle_deg: float, rate_deg_s: float | None = None,
                  channel: int | None = None):
        """
        angle_deg: signed (positive=CW, negative=CCW)
        rate_deg_s: unsigned magnitude (deg/s). If None, uses default_rot_speed.
        channel: defaults to cal_channel.

        The protocol rate is quantised in ~10 deg/s steps. Using the calibration
            theta_actual ~= k_rot * (omega_eff * t)
            => t = |theta_target| / (k_rot * omega_eff)
        the rate goes out uncompensated and k_rot shortens the duration instead.
        (step() compensates the rate instead -- both sweep the same angle.)
        """
        a = float(angle_deg)
        if abs(a) < 1e-9:
            return

        rate = self.p.default_rot_speed if rate_deg_s is None else float(rate_deg_s)
        if rate <= 0:
            raise ValueError("rate_deg_s must be > 0 (unsigned).")

        rate = min(rate, float(ROT_RATE_MAX))
        rot_cmd, omega_eff = encode_rotation_signed(rate if a > 0 else -rate)
        if rot_cmd == 0:
            return

        duration_s = abs(a) / (self.p.k_rot * omega_eff)
        ch = self.cal_channel if channel is None else channel

        print(f"[ROT] {a:+.1f} deg @ {rate:.1f} deg/s -> cmd={rot_cmd} "
              f"(omega_eff={omega_eff:.0f}), k_rot={self.p.k_rot:.3f}, t={duration_s:.3f}s")
        self._hold(ch, CLAMP_NONE, 0, rot_cmd, duration_s)
        self.stop(self.p.settle_s)
