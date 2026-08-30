"""teleop: the keyboard mapping and the flight loop, flown off-line."""
from __future__ import annotations

import inspect
import math

import pytest
from conftest import FakeCrazyflie, descends_monotonically

import flight
import teleop


class Holding:
    """A keyboard with a fixed set of keys held down."""

    def __init__(self, *keys):
        self.keys = set(keys)

    def down(self, key):
        return key in self.keys


def test_up_arrow_flies_forward():
    """Regression: up sent -MAX_ANGLE, which flew the drone backwards."""
    assert teleop.held_axis(Holding("up"), "down", "up", 0.0, 15.0) == 15.0
    assert teleop.held_axis(Holding("down"), "down", "up", 0.0, 15.0) == -15.0
    assert teleop.held_axis(Holding(), "down", "up", 10.0, 15.0) == 10.0 * flight.DECAY
    assert ('held_axis(inp, "down", "up", self.pitch, MAX_ANGLE)'
            in inspect.getsource(teleop.Session.read_keyboard))


class ScriptedInput:
    """A keyboard whose keys are held for a scripted number of ticks, then released."""

    def __init__(self, *script: tuple[str, int]) -> None:
        self.script = list(script)          # (key, ticks held)
        self.current: str | None = None

    def poll(self) -> list[str]:
        if self.script and self.script[0][1] <= 0:
            self.script.pop(0)
        if not self.script:
            self.current = None
            return []
        key, ticks = self.script[0]
        self.script[0] = (key, ticks - 1)
        first = self.current != key
        self.current = key
        return [key] if first else []

    def down(self, key: str) -> bool:
        return key == self.current


def fake_scf(cf) -> object:
    """A SyncCrazyflie stand-in whose link stays up until close_link()."""
    toc = type("T", (), {"toc": {}})()
    cf.param = type("P", (), {"toc": toc, "set_value": lambda *_a: None})()
    cf.log = type("L", (), {"toc": toc})()

    class Scf:
        def __init__(self):
            self.cf = cf
            self.open = True

        def is_link_open(self):
            return self.open

        def close_link(self):
            self.open = False

    return Scf()


@pytest.mark.usefixtures("no_telemetry")
def test_session_flies_up_and_lands_on_quit():
    """Holding w raises thrust a step per tick; q ramps it down to zero, never
    up again, and closes the link -- the whole loop, off-line."""
    drone = FakeCrazyflie()
    scf = fake_scf(drone)
    session = teleop.Session(scf, flight.Trim(0.5, -1.0), mag_offset=None, gamepad=False)
    session.fly(ScriptedInput(("w", 10), ("q", 1)), flight.Interruptible())
    thrusts = drone.commander.thrusts

    assert max(thrusts) == flight.MIN_THRUST + 9 * flight.THRUST_STEP
    assert thrusts[-1] == 0
    assert descends_monotonically(thrusts)
    assert not scf.open
    assert session.trim == (0.5, -1.0)


def test_keyboard_yaw_releases_to_zero():
    """Yaw snaps to 0 on release, so the heading hold can tell a centred stick."""
    session = teleop.Session(fake_scf(FakeCrazyflie()), flight.Trim(),
                             mag_offset=None, gamepad=False)
    session.read_keyboard(Holding("d"))
    assert session.yaw_rate == flight.MAX_YAW_RATE
    session.read_keyboard(Holding())
    assert session.yaw_rate == 0


def test_heading_hold_trims_yaw_toward_the_release_heading():
    """Stick centred and airborne: the heading where it was released is held,
    the correction takes the short way round the circle and is capped, and
    touching the stick or cutting thrust lets go of it."""
    session = teleop.Session(fake_scf(FakeCrazyflie()), flight.Trim(),
                             mag_offset=(0.0, 0.0, 0.0), gamepad=False)
    session.has_mag = True
    session.thrust = 30000

    def tick(hdg: float, *keys: str) -> None:
        """One loop tick: the compass reads `hdg`, level, then sticks, then the hold."""
        session.battery.append({"mag.x": math.cos(math.radians(hdg)),
                                "mag.y": math.sin(math.radians(hdg)), "mag.z": 0.0,
                                "stabilizer.roll": 0.0, "stabilizer.pitch": 0.0})
        session.read_keyboard(Holding(*keys))
        session.hold_heading()

    tick(10)
    assert session.ref_hdg == pytest.approx(10) and session.yaw_rate == 0

    tick(5)                                        # drifted 5 deg low
    assert session.yaw_rate == pytest.approx(teleop.HDG_SIGN * teleop.HDG_KP * 5)

    tick(350)                                      # 20 deg low through the wrap, capped
    assert session.yaw_rate == pytest.approx(teleop.HDG_SIGN * teleop.HDG_MAX_RATE)

    tick(350, "d")                                 # the stick moves the reference
    assert session.ref_hdg is None and session.yaw_rate == flight.MAX_YAW_RATE
    tick(300)                                      # taken afresh on release
    assert session.ref_hdg == pytest.approx(300) and session.yaw_rate == 0

    session.thrust = 0.0                           # on the ground: nothing to hold
    tick(300)
    assert session.ref_hdg is None and session.yaw_rate == 0


def test_trim_keys_step_and_reset():
    session = teleop.Session(fake_scf(FakeCrazyflie()), flight.Trim(),
                             mag_offset=None, gamepad=False)
    session.handle_keys(["]", "]", "'"])
    assert session.trim == pytest.approx((2 * teleop.TRIM_STEP, teleop.TRIM_STEP))
    session.handle_keys(["0"])
    assert session.trim == (0.0, 0.0)
    for _ in range(100):
        session.handle_keys(["["])
    assert session.trim.roll == -flight.TRIM_LIMIT
