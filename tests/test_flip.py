"""flip: the state machine never sends a rate while the firmware is in angle mode."""
from __future__ import annotations

import flight
import flip as flipping


def test_flip_never_sends_a_rate_while_in_angle_mode():
    """Rate mode must be set before any nonzero roll rate goes out, cleared
    only after zeros, and the spin must end on the gyro count, not the cap."""
    modes = []
    f = flipping.Flip(lambda on: modes.append(on), now=0.0)
    log = []
    now = 0.0
    while True:
        command = f.tick(now)
        if command is None:
            break
        if f.phase == "spin":
            f.feed([flipping.SPIN_RATE], 0.03)     # the gyro agrees with the command
        log.append((now, f.phase, tuple(modes), command))
        now += 0.03

    phases = [entry[1] for entry in log]
    assert phases[0] == "climb" and phases[-1] == "catch"
    assert [p for i, p in enumerate(phases) if p != phases[i - 1]] == \
        ["climb", "arm", "spin", "settle", "catch"]
    for _t, _phase, modes_so_far, (roll, _p, _y, _thrust) in log:
        if roll != 0:
            assert modes_so_far and modes_so_far[-1] is True    # RATE was already set
    assert modes == [True, False]
    spin_time = sum(0.03 for e in log if e[1] == "spin")
    assert spin_time < flipping.SPIN_MAX_S                       # ended by the gyro, not the cap
    assert log[-1][3][3] == flight.MAX_THRUST                    # catching at full thrust
