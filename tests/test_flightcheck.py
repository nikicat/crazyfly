"""flightcheck and signals: the probe stays in a small room, and the fit sees
through telemetry lag."""
from __future__ import annotations

import math

import pytest
from conftest import FakeCommander, FakeCrazyflie

import flightcheck
import signals

# --- staying inside a small room ------------------------------------------

def test_travel_estimate_matches_integrated_motion():
    """The closed form a*t^2 should match integrating the two lean phases."""
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
    travel = flightcheck.travel_estimate(flightcheck.PROBE_PITCH,
                                         flightcheck.LEAN_SECONDS)
    assert travel < 0.30, f"probe would travel {travel * 100:.0f} cm"


def test_bias_drift_estimate_explains_a_metre():
    """A 1.79 deg resting bias flown untrimmed covers about a metre."""
    assert flightcheck.travel_from_bias(1.79) == pytest.approx(1.0, abs=0.15)


# --- the flight itself ----------------------------------------------------

def fly_recording_pitch(monkeypatch, base_pitch: float) -> tuple[list, list[float]]:
    """Run flightcheck.fly() against a fake drone; (commands, pitches sent)."""
    pitches = []

    class RecordingCommander(FakeCommander):
        def send_setpoint(self, roll, pitch, yawrate, thrust):
            super().send_setpoint(roll, pitch, yawrate, thrust)
            pitches.append(pitch)

    monkeypatch.setattr(flightcheck, "RAMP_SECONDS", 0.1)
    drone = FakeCrazyflie()
    drone.commander = RecordingCommander()
    scf = type("S", (), {"cf": drone})()
    commands, _ = flightcheck.fly(scf, thrust=30000, base_pitch=base_pitch,
                                  lean=3.0, lean_seconds=0.2)
    return commands, pitches


@pytest.mark.usefixtures("no_telemetry")
def test_flight_leans_cancel_out(monkeypatch):
    """The two leans must be equal and opposite, or velocity is left over."""
    commands, _ = fly_recording_pitch(monkeypatch, base_pitch=0.0)

    offsets = [offset for _, offset in commands]
    assert +3.0 in offsets and -3.0 in offsets
    assert offsets.count(3.0) == offsets.count(-3.0), "leans are not symmetric"
    assert sum(offsets) == pytest.approx(0.0, abs=1e-9)


@pytest.mark.usefixtures("no_telemetry")
def test_flight_applies_the_base_trim(monkeypatch):
    """The probe leans around the saved trim, so the drone does not fly off on
    its own bias during the ramps -- that alone covered about a metre."""
    _, pitches = fly_recording_pitch(monkeypatch, base_pitch=-1.8)

    # Ramp phases command the trim itself, not zero.
    assert pytest.approx(-1.8) in pitches
    assert pytest.approx(1.2) in pitches    # -1.8 + 3.0
    assert pytest.approx(-4.8) in pitches   # -1.8 - 3.0


# --- seeing through telemetry lag -----------------------------------------

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
    commands, samples = synth_flight(true_lag)
    lag, gain, correlation = signals.best_fit(commands, samples)

    assert lag == pytest.approx(true_lag, abs=0.06)
    assert gain == pytest.approx(-1.0, abs=0.25)
    assert abs(correlation) > 0.9


def test_ignoring_lag_hides_the_response():
    """Regression: slicing samples by wall-clock phase averaged the previous
    lean into the current one, so a flying drone measured as barely moving."""
    commands, samples = synth_flight(0.35)

    naive = signals.fit_at_lag(commands, samples, 0.0)
    _, corrected_gain, _ = signals.best_fit(commands, samples)

    assert naive is not None
    assert abs(naive[1]) < abs(corrected_gain), "lag correction must help"
    assert abs(corrected_gain) == pytest.approx(1.0, abs=0.25)


def test_gain_is_bounded_so_a_bad_fit_cannot_claim_188_percent():
    """Regression: selecting the lag that maximised the raw response reported
    188% of a physically possible answer, at the edge of the search range.
    Gain is a regression slope, so a real tracking response cannot exceed ~1."""
    for true_lag in (0.0, 0.15, 0.3, 0.45):
        commands, samples = synth_flight(true_lag)
        _, gain, _ = signals.best_fit(commands, samples)
        assert abs(gain) < flightcheck.IMPLAUSIBLE_GAIN, (
            f"gain {gain:+.2f} exceeds what the drone can physically do")


def test_grounded_drone_reads_as_not_tracking():
    """On the ground the attitude cannot follow, so the gain collapses."""
    commands, samples = synth_flight(0.2, gain=-0.05)
    _, gain, _ = signals.best_fit(commands, samples)

    assert abs(gain) < flightcheck.TRACKING_GAIN


def test_noise_alone_does_not_correlate():
    """Random attitude must not be mistaken for a tracking response."""
    commands, _ = synth_flight(0.0)
    values = [0.7, -1.3, 0.2, 1.9, -0.4, 0.9, -1.1, 0.3, 1.4, -0.8]
    samples = [(t, values[i % len(values)])
               for i, (t, _offset) in enumerate(commands[::2])]

    _, _, correlation = signals.best_fit(commands, samples)
    assert abs(correlation) < flightcheck.MIN_CORRELATION
