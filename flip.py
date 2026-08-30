"""One roll flip, as a state machine teleop advances at its own 33 Hz.

Nothing in the firmware flips; this drives it from the host the way the demo
scripts do. Angle mode would fight a rotation past 90 degrees, so roll and
pitch go to RATE mode for the spin -- the roll value sent is then degrees per
second -- and back to ANGLE mode for the catch. The mode switch is a parameter
write with its own latency, so a rate value must never be on the wire while
the drone still reads it as an angle: both switches get a couple of ticks of
zeros around them (ARM_S).

Phases:  climb (full thrust, gain speed)  ->  arm (rate mode, zeros)
      -> spin (SPIN_RATE, low thrust) until TURN_DEG from the gyro or SPIN_MAX_S
      -> settle (zero rate, angle mode)  ->  catch (full thrust)  ->  done.

ponytail: numbers are for a Crazyflie 1.0 hovering at ~64% throttle, which has
only ~1.5x hover in reserve; expect to lose about a metre. SPIN_THRUST and
CLIMB_S are the knobs.
"""
from __future__ import annotations

from flight import MAX_THRUST, setting

MIN_VBAT = setting("flip.min_vbat", 3.7)   # below this the catch has no margin; refuse
CLIMB_S = 0.4
ARM_S = 0.06             # two ticks for a mode switch to land before it matters
SPIN_RATE = 1500.0       # deg/s; the gyro reads to 2000
SPIN_THRUST = 20000
SPIN_MAX_S = 0.6         # blind cap if the gyro count never arrives
TURN_DEG = 330.0         # stop a bit short: the drone keeps turning while the catch bites
CATCH_S = 0.5


class Flip:
    """Call tick(now) every loop; feed(rates, dt) with new gyro roll rates."""

    def __init__(self, set_rate_mode, now: float) -> None:
        self._set_rate_mode = set_rate_mode      # set_rate_mode(True) -> RATE, False -> ANGLE
        self.phase = "climb"
        self._since = now
        self.turned = 0.0

    def feed(self, rates, dt: float) -> None:
        self.turned += sum(rates) * dt

    def _enter(self, phase: str, now: float) -> None:
        self.phase = phase
        self._since = now

    def tick(self, now: float) -> tuple[float, float, float, int] | None:
        """(roll, pitch, yaw_rate, thrust) to send now, or None once finished."""
        t = now - self._since
        if self.phase == "climb":
            if t >= CLIMB_S:
                self._set_rate_mode(True)
                self._enter("arm", now)
            return 0.0, 0.0, 0.0, MAX_THRUST
        if self.phase == "arm":
            if t >= ARM_S:
                self._enter("spin", now)
            return 0.0, 0.0, 0.0, MAX_THRUST
        if self.phase == "spin":
            if abs(self.turned) >= TURN_DEG or t >= SPIN_MAX_S:
                self._enter("settle", now)
                return 0.0, 0.0, 0.0, SPIN_THRUST
            return SPIN_RATE, 0.0, 0.0, SPIN_THRUST
        if self.phase == "settle":
            if t >= ARM_S:
                self._set_rate_mode(False)
                self._enter("catch", now)
            return 0.0, 0.0, 0.0, SPIN_THRUST
        if self.phase == "catch":
            if t >= CATCH_S:
                self.phase = "done"
                return None
            return 0.0, 0.0, 0.0, MAX_THRUST
        return None
