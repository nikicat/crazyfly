"""orient: naming the lifted arm from how the attitude changed."""
from __future__ import annotations

import pytest

import orient


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
