"""teleop: the keyboard mapping and the flight loop, flown off-line."""
from __future__ import annotations

import inspect

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
