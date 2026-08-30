"""Stand-ins for the drone, shared by every test file.

None of the tests need a radio or a drone: the Crazyflie is stubbed, so the
thrust profile, the landing behaviour and the trim direction can be checked
without anything spinning up.
"""
from __future__ import annotations

import pytest

import cfenv
import hoptest


class FakeCommander:
    def __init__(self) -> None:
        self.thrusts: list[int] = []
        self.stopped = False

    def send_setpoint(self, roll, pitch, yawrate, thrust) -> None:
        self.thrusts.append(thrust)

    def send_stop_setpoint(self) -> None:
        self.stopped = True


class FakePlatform:
    """Reports -1, matching a Crazyflie 1.0's legacy protocol."""

    def get_protocol_version(self) -> int:
        return -1


class FakeCrazyflie:
    def __init__(self) -> None:
        self.commander = FakeCommander()
        self.platform = FakePlatform()


class SilentKeyboard:
    """Stands in for the raw-mode reader; reports no keypresses."""

    held: dict[str, float] = {}

    def __enter__(self) -> SilentKeyboard:
        return self

    def __exit__(self, *exc) -> None:
        pass

    def poll(self) -> list[str]:
        return []


class NullRecorder:
    """cfenv.record_log that never receives a sample."""

    def __enter__(self) -> list:
        return []

    def __exit__(self, *exc) -> bool:
        return False


def descends_monotonically(thrusts: list[int]) -> bool:
    peak = thrusts.index(max(thrusts))
    tail = thrusts[peak:]
    return all(tail[i] >= tail[i + 1] for i in range(len(tail) - 1))


@pytest.fixture
def cf(monkeypatch) -> FakeCrazyflie:
    """A drone for hoptest.hop(), with the keyboard silenced so nothing aborts it."""
    monkeypatch.setattr(hoptest, "Keyboard", SilentKeyboard)
    return FakeCrazyflie()


@pytest.fixture
def no_telemetry(monkeypatch) -> None:
    """Make cfenv.record_log deliver nothing, for loops that fly without a drone."""
    monkeypatch.setattr(cfenv, "record_log", lambda *_a, **_k: NullRecorder())
