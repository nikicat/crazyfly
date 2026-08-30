"""hoptest: the hop profile, landing on Ctrl-C, and which way an answer moves the trim.

  * thrust never rebounds after the descent starts
  * every exit path ends at zero thrust, ramped rather than cut
  * Ctrl-C lands the drone instead of abandoning it in the air
  * a reported drift moves the trim the opposite way
"""
from __future__ import annotations

import os
import signal
import threading
import time

import pytest
from conftest import FakeCommander, FakeCrazyflie, SilentKeyboard, descends_monotonically

import flight
import hoptest

# --- hop profile ----------------------------------------------------------

def test_hop_rises_then_falls_to_zero(cf):
    reason = hoptest.hop(cf, thrust=36000, hold=0.3, trim=flight.Trim())
    thrusts = cf.commander.thrusts

    assert reason == "completed"
    assert max(thrusts) == 36000
    assert thrusts[-1] == 0
    assert descends_monotonically(thrusts)


def test_hop_does_not_rebound_on_landing(cf):
    """Regression: the landing ramp used to restart from the hover thrust,
    giving an unwanted second hop just as the drone touched down."""
    hoptest.hop(cf, thrust=36000, hold=0.3, trim=flight.Trim())
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
    hoptest.hop(drone, thrust=30000, hold=0.2, trim=flight.Trim(-0.3, -1.9))

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
    reason = hoptest.hop(cf, thrust=30000, hold=5.0, trim=flight.Trim())
    thrusts = cf.commander.thrusts

    assert reason == "interrupted"
    assert thrusts[-1] == 0
    assert descends_monotonically(thrusts)
    assert len(thrusts[thrusts.index(max(thrusts)):]) > 5, "cut instead of ramped"


def test_sigint_handler_is_restored(cf):
    before = signal.getsignal(signal.SIGINT)
    hoptest.hop(cf, thrust=20000, hold=0.2, trim=flight.Trim())
    assert signal.getsignal(signal.SIGINT) is before


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


# --- answering the hop prompt ---------------------------------------------

def apply_answer(answer, roll_trim=0.0, pitch_trim=0.0, invert_pitch=False):
    """The trim after answering the hop prompt with `answer`; None if it is rejected."""
    try:
        directions, _ignored = hoptest.parse_drift(answer)
    except ValueError:
        return None
    return hoptest.correct(flight.Trim(roll_trim, pitch_trim), directions, invert_pitch)


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


def test_unknown_letters_are_reported_not_dropped():
    directions, ignored = hoptest.parse_drift("rxb")
    assert directions == ["r", "b"]
    assert ignored == ["x"]


def test_correction_stops_at_the_trim_limit():
    trim = flight.Trim(-flight.TRIM_LIMIT + 0.1, 0.0)
    assert hoptest.correct(trim, ["r"]).roll == -flight.TRIM_LIMIT


# --- the trim handoff to teleop -------------------------------------------

def test_hop_and_teleop_share_one_trim_file():
    """Trim found by hopping must be what teleop flies with, without flags."""
    import flightcheck
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
