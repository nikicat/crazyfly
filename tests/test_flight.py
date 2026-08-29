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

import cfenv as cfenv_module  # noqa: E402
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
    ("f", "pitch", -1),
    ("b", "pitch", +1),   # measured: positive pitch drops the front, flies forward
])
def test_correction_opposes_reported_drift(answer, axis, sign):
    assert hoptest.CORRECTIONS[answer] == (axis, sign)


def test_backward_drift_raises_pitch_trim():
    """Measured on the ground with motorcheck: the front motor is m1, and a
    positive pitch command drives it to a standstill, so the front drops and
    the drone flies forward. Correcting a backward drift therefore needs MORE
    pitch. Inferring this from in-flight drift gave the opposite answer twice,
    because the drone was pivoting on a leg rather than flying."""
    axis, direction = hoptest.CORRECTIONS["b"]
    assert axis == "pitch"
    assert direction * hoptest.CORRECTION_STEP > 0


def test_teleop_up_arrow_flies_forward():
    """Regression: up sent -MAX_ANGLE, which flew the drone backwards."""
    import inspect

    import teleop

    class Holding:
        def __init__(self, *keys):
            self.keys = set(keys)

        def down(self, key):
            return key in self.keys

    assert teleop.held_axis(Holding("up"), "down", "up", 0.0, 15.0) == 15.0
    assert teleop.held_axis(Holding("down"), "down", "up", 0.0, 15.0) == -15.0
    assert teleop.held_axis(Holding(), "down", "up", 10.0, 15.0) == 10.0 * flight.DECAY
    assert 'held_axis(inp, "down", "up", pitch, MAX_ANGLE)' in inspect.getsource(teleop.run)


# --- gamepad --------------------------------------------------------------

def js_event(kind: int, number: int, value: int, initial: bool = False) -> bytes:
    """One Linux joystick event, as the kernel would write it."""
    import struct

    return struct.pack("IhBB", 0, value, kind | (0x80 if initial else 0), number)


def test_gamepad_maps_xbox_controls_to_teleop_keys(tmp_path):
    """B cuts thrust, Menu lands, the D-pad trims, and pushing either stick
    up reads positive: more thrust on the left, forward pitch on the right."""
    path = tmp_path / "js0"
    os.mkfifo(path)

    class TypedQ:
        def poll(self):
            return ["q"]

    with flight.Gamepad(str(path), keyboard=TypedQ()) as pad:
        writer = os.open(path, os.O_WRONLY)
        os.write(writer, b"".join([
            js_event(1, 1, 1, initial=True),     # state dump on open: not a press
            js_event(1, 1, 1),                   # B
            js_event(1, 11, 1),                  # Menu
            js_event(2, 7, -32767),              # D-pad up
            js_event(2, 7, 0),                   # D-pad released: no event
            js_event(2, 1, -32767),              # left stick full up
            js_event(2, 3, -16000),              # right stick half up
            js_event(2, 2, 2000),                # right stick X inside the deadzone
            js_event(2, 5, -32767, initial=True),  # LT at rest
            js_event(2, 4, 32767),               # RT fully pulled
        ]))
        events = pad.poll()

        assert events == ["q", " ", "q", "'"]      # keyboard q first, then the pad
        assert pad.axis("thrust") == pytest.approx(1.0)
        assert 0.3 < pad.axis("pitch") < 0.5
        assert pad.axis("roll") == 0.0
        assert pad.trigger("yaw_left") == 0.0
        assert pad.trigger("yaw_right") == pytest.approx(1.0)

        os.close(writer)
        with pytest.raises(OSError):
            pad.poll()                           # writer gone == pad disconnected


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


def test_classify_rejects_a_between_arms_lift():
    axis, reason, _ = orient.classify(20.0, 20.0)
    assert axis is None
    assert "two arms" in reason


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


# --- flightcheck stays inside a small room --------------------------------

def test_travel_estimate_matches_integrated_motion():
    """The closed form a*t^2 should match integrating the two lean phases."""
    import math

    import flightcheck

    lean, seconds = 3.0, 0.5
    accel = flightcheck.GRAVITY * math.tan(math.radians(lean))

    dt, velocity, position = 0.0005, 0.0, 0.0
    for phase_accel in (accel, -accel):
        for _ in range(int(seconds / dt)):
            velocity += phase_accel * dt
            position += velocity * dt

    assert flightcheck.travel_estimate(lean, seconds) == pytest.approx(position,
                                                                      rel=0.02)
    assert velocity == pytest.approx(0.0, abs=1e-6), "counter-lean must stop it"


def test_default_probe_stays_within_a_small_room():
    """Regression: holding one lean through the descent covered 1-2 m."""
    import flightcheck

    travel = flightcheck.travel_estimate(flightcheck.PROBE_PITCH,
                                         flightcheck.LEAN_SECONDS)
    assert travel < 0.30, f"probe would travel {travel * 100:.0f} cm"


def test_flight_leans_cancel_out(monkeypatch):
    """The two leans must be equal and opposite, or velocity is left over."""
    import flightcheck

    pitches = []

    class RecordingCommander(FakeCommander):
        def send_setpoint(self, roll, pitch, yawrate, thrust):
            super().send_setpoint(roll, pitch, yawrate, thrust)
            pitches.append(pitch)

    class NullRecorder:
        def __enter__(self):
            return []

        def __exit__(self, *exc):
            return False

    monkeypatch.setattr(cfenv_module, "record_log",
                        lambda *_a, **_k: NullRecorder())
    monkeypatch.setattr(flightcheck, "RAMP_SECONDS", 0.1)

    drone = FakeCrazyflie()
    drone.commander = RecordingCommander()
    scf = type("S", (), {"cf": drone})()
    commands, _ = flightcheck.fly(scf, thrust=30000, base_pitch=0.0,
                                  lean=3.0, lean_seconds=0.2)

    offsets = [offset for _, offset in commands]
    assert +3.0 in offsets and -3.0 in offsets
    assert offsets.count(3.0) == offsets.count(-3.0), "leans are not symmetric"
    assert sum(offsets) == pytest.approx(0.0, abs=1e-9)


def test_flight_applies_the_base_trim(monkeypatch):
    """The probe leans around the saved trim, so the drone does not fly off on
    its own bias during the ramps -- that alone covered about a metre."""
    import flightcheck

    pitches = []

    class RecordingCommander(FakeCommander):
        def send_setpoint(self, roll, pitch, yawrate, thrust):
            super().send_setpoint(roll, pitch, yawrate, thrust)
            pitches.append(pitch)

    class NullRecorder:
        def __enter__(self):
            return []

        def __exit__(self, *exc):
            return False

    monkeypatch.setattr(cfenv_module, "record_log",
                        lambda *_a, **_k: NullRecorder())
    monkeypatch.setattr(flightcheck, "RAMP_SECONDS", 0.1)

    drone = FakeCrazyflie()
    drone.commander = RecordingCommander()
    scf = type("S", (), {"cf": drone})()
    flightcheck.fly(scf, thrust=30000, base_pitch=-1.8, lean=3.0,
                    lean_seconds=0.2)

    # Ramp phases command the trim itself, not zero.
    assert pytest.approx(-1.8) in pitches
    assert pytest.approx(1.2) in pitches    # -1.8 + 3.0
    assert pytest.approx(-4.8) in pitches   # -1.8 - 3.0


# --- flightcheck must see through telemetry lag ---------------------------

def synth_flight(true_lag, lean=3.0, lean_seconds=0.4, dt=0.03,
                 sample_dt=0.05, cycles=3, gain=-1.0):
    """Commands plus estimator samples delayed by `true_lag`.

    The response is inverted by default, matching cflib transmitting -pitch.
    """
    start = 1000.0
    phases = [(0.0, 0.8)]
    for _ in range(cycles):
        phases.append((+lean, lean_seconds))
        phases.append((-lean, lean_seconds))
    phases.append((0.0, 0.8))

    commands, clock = [], start
    for offset, duration in phases:
        for _ in range(int(duration / dt)):
            commands.append((clock, offset))
            clock += dt

    samples, stamp = [], start
    while stamp < clock:
        source = stamp - true_lag
        offset = 0.0
        for command_time, command_offset in commands:
            if command_time <= source:
                offset = command_offset
            else:
                break
        samples.append((stamp, gain * offset))
        stamp += sample_dt
    return commands, samples


@pytest.mark.parametrize("true_lag", [0.0, 0.1, 0.2, 0.35, 0.5])
def test_best_fit_recovers_the_telemetry_lag(true_lag):
    import flightcheck

    commands, samples = synth_flight(true_lag)
    lag, gain, correlation = flightcheck.best_fit(commands, samples)

    assert lag == pytest.approx(true_lag, abs=0.06)
    assert gain == pytest.approx(-1.0, abs=0.25)
    assert abs(correlation) > 0.9


def test_ignoring_lag_hides_the_response():
    """Regression: slicing samples by wall-clock phase averaged the previous
    lean into the current one, so a flying drone measured as barely moving."""

    commands, samples = synth_flight(0.35)

    import signals
    naive = signals.fit_at_lag(commands, samples, 0.0)
    _, corrected_gain, _ = signals.best_fit(commands, samples)

    assert naive is not None
    assert abs(naive[1]) < abs(corrected_gain), "lag correction must help"
    assert abs(corrected_gain) == pytest.approx(1.0, abs=0.25)


def test_gain_is_bounded_so_a_bad_fit_cannot_claim_188_percent():
    """Regression: selecting the lag that maximised the raw response reported
    188% of a physically possible answer, at the edge of the search range.
    Gain is a regression slope, so a real tracking response cannot exceed ~1."""
    import flightcheck

    for true_lag in (0.0, 0.15, 0.3, 0.45):
        commands, samples = synth_flight(true_lag)
        _, gain, _ = flightcheck.best_fit(commands, samples)
        assert abs(gain) < flightcheck.IMPLAUSIBLE_GAIN, (
            f"gain {gain:+.2f} exceeds what the drone can physically do")


def test_grounded_drone_reads_as_not_tracking():
    """On the ground the attitude cannot follow, so the gain collapses."""
    import flightcheck

    commands, samples = synth_flight(0.2, gain=-0.05)
    _, gain, _ = flightcheck.best_fit(commands, samples)

    assert abs(gain) < flightcheck.TRACKING_GAIN


def test_noise_alone_does_not_correlate():
    """Random attitude must not be mistaken for a tracking response."""
    import flightcheck

    commands, _ = synth_flight(0.0)
    values = [0.7, -1.3, 0.2, 1.9, -0.4, 0.9, -1.1, 0.3, 1.4, -0.8]
    samples = [(t, values[i % len(values)])
               for i, (t, _offset) in enumerate(commands[::2])]

    _, _, correlation = flightcheck.best_fit(commands, samples)
    assert abs(correlation) < flightcheck.MIN_CORRELATION


def test_bias_drift_estimate_explains_a_metre():
    """A 1.79 deg resting bias flown untrimmed covers about a metre."""
    import flightcheck

    assert flightcheck.travel_from_bias(1.79) == pytest.approx(1.0, abs=0.15)


# --- answering the hop prompt ---------------------------------------------

def apply_answer(answer, roll_trim=0.0, pitch_trim=0.0, invert_pitch=False):
    """Mirror of hoptest's answer handling, for testing the parsing rules."""
    directions = [c for c in dict.fromkeys(answer) if c in hoptest.CORRECTIONS]
    if not directions:
        return None
    axes = {hoptest.CORRECTIONS[c][0] for c in directions}
    if len(axes) < len(directions):
        return None
    for letter in directions:
        axis, direction = hoptest.CORRECTIONS[letter]
        if axis == "pitch" and invert_pitch:
            direction = -direction
        if axis == "roll":
            roll_trim += direction * hoptest.CORRECTION_STEP
        else:
            pitch_trim += direction * hoptest.CORRECTION_STEP
    return roll_trim, pitch_trim


def test_diagonal_answer_moves_both_axes():
    """Regression: 'rb' was not a key in CORRECTIONS, so it was silently
    dropped -- the trim never moved and the drone flew the same way again."""
    result = apply_answer("rb")

    assert result is not None, "'rb' must be accepted"
    roll_trim, pitch_trim = result
    assert roll_trim != 0.0, "right drift must move roll trim"
    assert pitch_trim != 0.0, "back drift must move pitch trim"


def test_diagonal_matches_the_two_single_answers():
    combined = apply_answer("rb")
    separately = apply_answer("b", *apply_answer("r"))
    assert combined == separately


@pytest.mark.parametrize("answer", ["rl", "fb", "lr"])
def test_opposite_answers_on_one_axis_are_rejected(answer):
    """They would cancel, leaving the trim unmoved with no explanation."""
    assert apply_answer(answer) is None


@pytest.mark.parametrize("answer", ["", "x", "zz", "?"])
def test_answers_with_no_direction_are_rejected(answer):
    assert apply_answer(answer) is None


def test_answer_order_does_not_matter():
    assert apply_answer("rb") == apply_answer("br")


def test_repeated_letters_apply_once():
    assert apply_answer("rr") == apply_answer("r")


# --- the command router ---------------------------------------------------

def test_router_exposes_every_tool():
    """Each script's run() must be reachable through cf.py, so the router and
    direct invocation cannot drift apart."""
    import cf

    registered = {command.name for command in cf.app.registered_commands}
    expected = {"scan", "link", "boot", "info", "teleop", "hover",
                "trim", "hop", "orient", "motors", "airborne"}
    assert expected <= registered


def test_router_commands_are_the_modules_own_run():
    """The router wraps run() rather than reimplementing the options, so a
    changed default cannot mean two different things."""
    import cf
    import hoptest
    import info

    by_name = {c.name: c.callback for c in cf.app.registered_commands}
    assert by_name["hop"] is hoptest.run
    assert by_name["info"] is info.run


def test_every_command_has_help_text():
    import cf

    for command in cf.app.registered_commands:
        assert command.help, f"{command.name} has no help"


def test_hop_and_teleop_share_one_trim_file():
    """Trim found by hopping must be what teleop flies with, without flags."""
    import flightcheck
    import hoptest
    import teleop

    assert teleop.TRIM_FILE is flight.TRIM_FILE
    assert hoptest.TRIM_FILE is flight.TRIM_FILE
    assert teleop.load_trim is flight.load_trim
    assert hoptest.save_trim is flight.save_trim
    # flightcheck flies at the saved trim so its probe is not swamped by drift
    assert flightcheck.load_trim is flight.load_trim


def test_hop_result_is_what_teleop_starts_from(tmp_path, monkeypatch):
    monkeypatch.setattr(flight, "TRIM_FILE", tmp_path / "trim.json")

    flight.save_trim(-1.2, 2.8)          # as hoptest does on exit
    roll, pitch = flight.load_trim()      # as teleop does on entry

    assert (roll, pitch) == (-1.2, 2.8)


# --- frame geometry detection ---------------------------------------------

def frame_report(gains):
    """Build the shape motorcheck.summarise returns, from four gains."""
    import motorcheck

    return {motor: {"gain": g, "correlation": 0.97, "low": 8000, "high": 20000}
            for motor, g in zip(motorcheck.MOTORS, gains, strict=True)}


def test_detects_plus_frame():
    """Two motors answer pitch and two barely stir -- the pattern measured on
    the Crazyflie 1.0: m1 -237, m3 +236, m2 and m4 around +3."""
    import motorcheck

    frame, ranked, ratio = motorcheck.detect_frame(
        frame_report([-236.9, 2.2, 236.4, -1.7]))

    assert frame == "plus"
    assert ratio < motorcheck.PLUS_RATIO
    assert set(ranked[:2]) == {"motor.m1", "motor.m3"}


def test_detects_x_frame():
    """All four answer about equally, so each end of the axis is a pair."""
    import motorcheck

    frame, _ranked, ratio = motorcheck.detect_frame(
        frame_report([-118.0, -121.0, 119.5, 117.0]))

    assert frame == "x"
    assert ratio > motorcheck.X_RATIO


def test_ambiguous_frame_is_not_guessed():
    """Between the two patterns the reading means nothing, and saying so beats
    picking one: the wrong mixer flies 45 degrees off and reads as bad trim."""
    import motorcheck

    frame, _ranked, ratio = motorcheck.detect_frame(
        frame_report([-200.0, 70.0, 190.0, -60.0]))

    assert frame is None
    assert motorcheck.PLUS_RATIO <= ratio <= motorcheck.X_RATIO


def test_silent_motors_do_not_divide_by_zero():
    import motorcheck

    frame, _ranked, ratio = motorcheck.detect_frame(frame_report([0.0] * 4))

    assert frame is None
    assert ratio == 0.0


@pytest.mark.parametrize("gains, expected_movers", [
    ([-236.9, 2.2, 236.4, -1.7], 1),        # plus: one motor per end
    ([-118.0, -121.0, 119.5, 117.0], 2),    # X: a pair per end
])
def test_movers_per_end_follows_the_frame(gains, expected_movers):
    """On X both motors at one end drop together, so step 2 must watch two."""
    import motorcheck

    report = frame_report(gains)
    frame, _ranked, _ratio = motorcheck.detect_frame(report)
    movers = 1 if frame == "plus" else 2

    assert movers == expected_movers

    by_gain = sorted(motorcheck.MOTORS, key=lambda m: report[m]["gain"])
    low_on_plus = by_gain[:movers]
    low_on_minus = by_gain[-movers:]

    assert not set(low_on_plus) & set(low_on_minus), "ends must not overlap"
    assert report[low_on_plus[0]]["gain"] < 0 < report[low_on_minus[0]]["gain"]
