"""motorcheck: reading the frame geometry off the motor responses."""
from __future__ import annotations

import pytest

import motorcheck


def frame_report(gains):
    """Build the shape motorcheck.summarise returns, from four gains."""
    return {motor: motorcheck.MotorResponse(g, correlation=0.97, low=8000, high=20000)
            for motor, g in zip(motorcheck.MOTORS, gains, strict=True)}


def test_detects_plus_frame():
    """Two motors answer pitch and two barely stir -- the pattern measured on
    the Crazyflie 1.0: m1 -237, m3 +236, m2 and m4 around +3."""
    frame, ranked, ratio = motorcheck.detect_frame(
        frame_report([-236.9, 2.2, 236.4, -1.7]))

    assert frame == "plus"
    assert ratio < motorcheck.PLUS_RATIO
    assert set(ranked[:2]) == {"motor.m1", "motor.m3"}


def test_detects_x_frame():
    """All four answer about equally, so each end of the axis is a pair."""
    frame, _ranked, ratio = motorcheck.detect_frame(
        frame_report([-118.0, -121.0, 119.5, 117.0]))

    assert frame == "x"
    assert ratio > motorcheck.X_RATIO


def test_ambiguous_frame_is_not_guessed():
    """Between the two patterns the reading means nothing, and saying so beats
    picking one: the wrong mixer flies 45 degrees off and reads as bad trim."""
    frame, _ranked, ratio = motorcheck.detect_frame(
        frame_report([-200.0, 70.0, 190.0, -60.0]))

    assert frame is None
    assert motorcheck.PLUS_RATIO <= ratio <= motorcheck.X_RATIO


def test_silent_motors_do_not_divide_by_zero():
    frame, _ranked, ratio = motorcheck.detect_frame(frame_report([0.0] * 4))

    assert frame is None
    assert ratio == 0.0


@pytest.mark.parametrize("gains, expected_movers", [
    ([-236.9, 2.2, 236.4, -1.7], 1),        # plus: one motor per end
    ([-118.0, -121.0, 119.5, 117.0], 2),    # X: a pair per end
])
def test_movers_per_end_follows_the_frame(gains, expected_movers):
    """On X both motors at one end drop together, so step 2 must watch two."""
    report = frame_report(gains)
    frame = motorcheck.detect_frame(report)

    assert frame.movers == expected_movers

    low_on_plus, low_on_minus = motorcheck.axis_ends(report, frame.movers)

    assert not set(low_on_plus) & set(low_on_minus), "ends must not overlap"
    assert report[low_on_plus[0]].gain < 0 < report[low_on_minus[0]].gain
