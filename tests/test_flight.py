"""Offline tests for the safety-critical flight logic.

None of these need a radio or a drone: the Crazyflie is stubbed, so the thrust
profile, the landing behaviour and the trim direction can be checked without
anything spinning up. The rules being protected are:

  * thrust never rebounds after the descent starts
  * every exit path ends at zero thrust, ramped rather than cut
  * Ctrl-C lands the drone instead of abandoning it in the air
  * a reported drift moves the trim the opposite way
"""
from __future__ import annotations

import os
import signal
import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import flight  # noqa: E402
import hoptest  # noqa: E402
import orient  # noqa: E402


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


@pytest.fixture
def cf(monkeypatch) -> FakeCrazyflie:
    monkeypatch.setattr(hoptest, "Keyboard", SilentKeyboard)
    return FakeCrazyflie()


def descends_monotonically(thrusts: list[int]) -> bool:
    peak = thrusts.index(max(thrusts))
    tail = thrusts[peak:]
    return all(tail[i] >= tail[i + 1] for i in range(len(tail) - 1))


# --- hop profile ----------------------------------------------------------

def test_hop_rises_then_falls_to_zero(cf):
    reason = hoptest.hop(cf, thrust=36000, hold=0.3, roll_trim=0, pitch_trim=0)
    thrusts = cf.commander.thrusts

    assert reason == "completed"
    assert max(thrusts) == 36000
    assert thrusts[-1] == 0
    assert descends_monotonically(thrusts)


def test_hop_does_not_rebound_on_landing(cf):
    """Regression: the landing ramp used to restart from the hover thrust,
    giving an unwanted second hop just as the drone touched down."""
    hoptest.hop(cf, thrust=36000, hold=0.3, roll_trim=0, pitch_trim=0)
    thrusts = cf.commander.thrusts

    peak = thrusts.index(max(thrusts))
    assert max(thrusts[peak:]) == 36000, "thrust climbed again after the descent"


def test_hop_applies_trim_while_airborne(monkeypatch):
    monkeypatch.setattr(hoptest, "Keyboard", SilentKeyboard)
    seen = []

    class RecordingCommander(FakeCommander):
        def send_setpoint(self, roll, pitch, yawrate, thrust):
            super().send_setpoint(roll, pitch, yawrate, thrust)
            if thrust > flight.MIN_THRUST:
                seen.append((roll, pitch))

    drone = FakeCrazyflie()
    drone.commander = RecordingCommander()
    hoptest.hop(drone, thrust=30000, hold=0.2, roll_trim=-0.3, pitch_trim=-1.9)

    assert seen, "no airborne setpoints recorded"
    assert all(rp == (-0.3, -1.9) for rp in seen)


# --- interruption ---------------------------------------------------------

def test_ctrl_c_mid_hover_still_lands(cf):
    """First Ctrl-C must land the drone, not abandon it with motors running."""
    def interrupt_once_airborne():
        while not (cf.commander.thrusts and max(cf.commander.thrusts) >= 30000):
            time.sleep(0.01)
        os.kill(os.getpid(), signal.SIGINT)

    threading.Thread(target=interrupt_once_airborne, daemon=True).start()
    reason = hoptest.hop(cf, thrust=30000, hold=5.0, roll_trim=0, pitch_trim=0)
    thrusts = cf.commander.thrusts

    assert reason == "interrupted"
    assert thrusts[-1] == 0
    assert descends_monotonically(thrusts)
    assert len(thrusts[thrusts.index(max(thrusts)):]) > 5, "cut instead of ramped"


def test_sigint_handler_is_restored(cf):
    before = signal.getsignal(signal.SIGINT)
    hoptest.hop(cf, thrust=20000, hold=0.2, roll_trim=0, pitch_trim=0)
    assert signal.getsignal(signal.SIGINT) is before


def test_stop_motors_ignores_sigint():
    """The stop sequence must complete even if Ctrl-C arrives during it."""
    drone = FakeCrazyflie()

    def interrupt_soon():
        time.sleep(0.02)
        os.kill(os.getpid(), signal.SIGINT)

    threading.Thread(target=interrupt_soon, daemon=True).start()
    flight.stop_motors(drone, from_thrust=40000, dt=0.01)

    assert drone.commander.thrusts[-1] == 0
    assert signal.getsignal(signal.SIGINT) is not signal.SIG_IGN


# --- trim direction -------------------------------------------------------

@pytest.mark.parametrize("answer, axis, sign", [
    ("r", "roll", -1),
    ("l", "roll", +1),
    ("f", "pitch", +1),   # pitch is inverted on the wire by send_setpoint
    ("b", "pitch", -1),
])
def test_correction_opposes_reported_drift(answer, axis, sign):
    assert hoptest.CORRECTIONS[answer] == (axis, sign)


def test_backward_drift_lowers_pitch_trim():
    """Regression: answering 'back' used to raise pitch trim, which made the
    drift worse on every subsequent hop."""
    axis, direction = hoptest.CORRECTIONS["b"]
    assert axis == "pitch"
    assert direction * hoptest.CORRECTION_STEP < 0


# --- trim persistence -----------------------------------------------------

def test_trim_roundtrips(tmp_path, monkeypatch):
    monkeypatch.setattr(flight, "TRIM_FILE", tmp_path / "trim.json")
    flight.save_trim(-0.3, -1.9)
    assert flight.load_trim() == (-0.3, -1.9)


def test_missing_trim_file_reads_as_zero(tmp_path, monkeypatch):
    monkeypatch.setattr(flight, "TRIM_FILE", tmp_path / "absent.json")
    assert flight.load_trim() == (0.0, 0.0)


def test_corrupt_trim_file_reads_as_zero(tmp_path, monkeypatch):
    bad = tmp_path / "trim.json"
    bad.write_text("{not json")
    monkeypatch.setattr(flight, "TRIM_FILE", bad)
    assert flight.load_trim() == (0.0, 0.0)


# --- orientation ----------------------------------------------------------

@pytest.mark.parametrize("d_roll, d_pitch, axis, lifted", [
    (0.0, -30.0, "pitch", "BACK"),
    (0.0, +30.0, "pitch", "FRONT"),
    (+30.0, 0.0, "roll", "LEFT"),
    (-30.0, 0.0, "roll", "RIGHT"),
])
def test_classify_names_the_lifted_edge(d_roll, d_pitch, axis, lifted):
    got_axis, got_lifted, got_opposite = orient.classify(d_roll, d_pitch)
    assert (got_axis, got_lifted) == (axis, lifted)
    assert got_opposite != got_lifted


def test_classify_rejects_a_corner_lift():
    axis, reason, _ = orient.classify(20.0, 20.0)
    assert axis is None
    assert "corner" in reason


def test_classify_rejects_too_little_tilt():
    axis, reason, _ = orient.classify(1.0, 1.0)
    assert axis is None
    assert "not enough tilt" in reason


# --- telemetry must never hang --------------------------------------------

class FakeLogConfig:
    """Stands in for cflib's LogConfig; never delivers data."""

    def __init__(self, name=None, period_in_ms=100):
        self.valid = True
        self.data_received_cb = _CallbackList()
        self.error_cb = _CallbackList()
        self.started = False

    def add_variable(self, name, ctype=None):
        pass

    def start(self):
        self.started = True

    def stop(self):
        self.started = False

    def delete(self):
        pass


class _CallbackList:
    def __init__(self):
        self.callbacks = []

    def add_callback(self, cb):
        self.callbacks.append(cb)


class _FakeLog:
    def __init__(self):
        self.toc = type("T", (), {"toc": {"pm": {}, "stabilizer": {}}})()

    def add_config(self, config):
        pass


class _FakeSyncCf:
    def __init__(self):
        self.cf = type("C", (), {"log": _FakeLog()})()


def test_stream_log_times_out_instead_of_hanging(monkeypatch):
    """Regression: the firmware can accept and start a log block and then send
    nothing. SyncLogger's iterator blocks forever on that, so every caller hung
    until Ctrl-C."""
    import cfenv

    monkeypatch.setattr(cfenv, "LogConfig", FakeLogConfig)
    started = time.monotonic()

    with pytest.raises(cfenv.LinkLost) as excinfo:
        cfenv.stream_log(_FakeSyncCf(), {"pm.vbat": "float"},
                         lambda _data: True, timeout=0.5)

    assert time.monotonic() - started < 5, "stream_log did not return promptly"
    assert "no telemetry" in str(excinfo.value).lower()
