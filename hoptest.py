#!/usr/bin/env python3
"""Bounded hop tests for finding trim in a small room.

Flies a short, low, automatically-terminated hop, asks which way it went,
adjusts the trim and repeats. The drone lands on a timer rather than on your
reaction, so it cannot run away into a wall.

Drift grows with the square of time -- at a 0.3 degree lean it is about 3 cm
over 1 s but 2.8 m over 10 s. Keeping each hop short is what makes this safe
indoors, so --hold is capped.

It hops as soon as you press Enter, with no countdown, so have the drone
placed and your hand clear before you do.

Get the bulk of the trim from trimcheck.py first; this is for confirming and
fine-tuning it, which is the part that genuinely needs flight.

  python hoptest.py                 # interactive trim loop
  python hoptest.py --thrust 40000  # if it does not leave the ground
"""
from __future__ import annotations

import sys
import time
from collections import deque

import cfenv
from flight import (
    DT,
    MIN_THRUST,
    TRIM_FILE,
    TRIM_LIMIT,
    Interruptible,
    Keyboard,
    Trim,
    load_trim,
    phase_level,
    ramp_level,
    save_trim,
    stop_motors,
    ticks,
)

RAMP_SECONDS = 0.8       # spin up and down over this, rather than stepping
MAX_HOLD = 2.0           # refuse longer hops indoors; drift is quadratic
DEFAULT_HOLD = 1.0       # a 0.3 deg lean drifts about 3 cm in this long
DEFAULT_THRUST = 36000   # deliberately low; raise until it just lifts
CORRECTION_STEP = 0.4    # degrees per answered hop

# Which way to move each trim when the drone drifts a given way.
#
# Pitch is measured, on the ground, with motorcheck.py. On this drone the
# front motor is m1 and the back is m3, and a positive pitch command drives m1
# to a standstill -- the front drops, so the drone accelerates forward. A
# backward drift therefore needs MORE pitch, not less.
#
# That was worth measuring: inferring it from in-flight drift gave the opposite
# answer twice, because the drone was pivoting on a leg rather than flying.
#
# Roll has not been measured the same way. It is sent unchanged rather than
# negated, so a positive roll argument is expected to lean right, but run
# `motorcheck.py --axis roll` before trusting it.
CORRECTIONS = {
    "r": ("roll", -1),
    "l": ("roll", +1),
    "f": ("pitch", -1),
    "b": ("pitch", +1),
}


def hop(cf, thrust: int, hold: float, trim: Trim) -> str:
    """One ramp-up / hold / ramp-down cycle. Returns why it ended."""
    reason = "completed"
    level = float(MIN_THRUST)

    def send(value: float) -> None:
        cf.commander.send_setpoint(trim.roll, trim.pitch, 0, int(value))

    cf.commander.send_setpoint(0, 0, 0, 0)
    with Keyboard() as kb, Interruptible() as interrupt:
        try:
            for phase, duration in (("up", RAMP_SECONDS), ("hold", hold), ("down", RAMP_SECONDS)):
                for frac, _now in ticks(duration):
                    if kb.poll() or kb.held:        # any key aborts
                        reason = "aborted"
                        break
                    if interrupt.requested:
                        reason = "interrupted"
                        break
                    level = phase_level(phase, thrust, frac)
                    send(level)
                if reason != "completed":
                    break

            # Only needed when we bailed out mid-air; a completed hop has
            # already ramped down through its "down" phase. Ramping again from
            # `thrust` would push it back up and give a second hop on landing.
            if reason != "completed":
                steps = max(1, int(RAMP_SECONDS / DT))
                from_level = level
                for i in range(steps):
                    # Track `level` as it falls: stop_motors continues from it,
                    # and a stale value would make it ramp up again first.
                    level = ramp_level(from_level, 1 - i / steps)
                    send(level)
                    time.sleep(DT)
                level = MIN_THRUST
        finally:
            # Whatever happened -- abort, Ctrl-C, or an error mid-hop -- the
            # drone must not be left airborne with nothing driving it.
            stop_motors(cf, from_thrust=level, dt=DT)

    return reason


def parse_drift(answer: str) -> tuple[list[str], list[str]]:
    """Split an answer into its direction letters and the letters to ignore.

    A drift is usually diagonal, so any combination of r, l, f, b is accepted.
    Raises ValueError, carrying the message to show, when there is no direction
    in it or two opposite ones that would cancel -- silently dropping those
    once meant an answer changed nothing, with no sign it had been dropped.
    """
    letters = list(dict.fromkeys(answer))
    directions = [c for c in letters if c in CORRECTIONS]
    ignored = [c for c in letters if c not in CORRECTIONS]
    if not directions:
        raise ValueError(f"'{answer}' has no direction in it, so nothing changed."
                         "\n  Use r, l, f, b -- combined if it went diagonally.")
    if len({CORRECTIONS[c][0] for c in directions}) < len(directions):
        raise ValueError(f"'{answer}' asks for two opposite corrections on one "
                         "axis, which\n  cancel out. Nothing changed.")
    return directions, ignored


def correct(trim: Trim, directions: list[str], invert_pitch: bool = False) -> Trim:
    """Move the trim one CORRECTION_STEP against each reported drift direction."""
    for letter in directions:
        axis, direction = CORRECTIONS[letter]
        if axis == "pitch" and invert_pitch:
            direction = -direction
        trim = trim.nudge(axis, direction * CORRECTION_STEP)
    return trim


def run(
    thrust: int = DEFAULT_THRUST,
    hold: float = DEFAULT_HOLD,
    invert_pitch: bool = False,
    reset_trim: bool = False,
    roll_trim: float | None = None,
    pitch_trim: float | None = None,
    uri: str | None = None,
) -> None:
    """Bounded indoor hops that converge on a trim setting."""

    if hold > MAX_HOLD:
        print(f"Capping hop at {MAX_HOLD}s -- drift grows with time squared.")
        hold = MAX_HOLD

    # Start from zero or from the saved trim, then let explicit values win.
    if reset_trim:
        print("Starting from zero trim.")
    trim = (Trim() if reset_trim else load_trim()).override(roll_trim, pitch_trim)

    with cfenv.session(uri) as scf:
        vbat = cfenv.vbat(scf)
        print(f"Connected. Battery {vbat:.2f} V\n")
        if vbat < 3.6:
            sys.exit("Battery too low for a meaningful hop -- it will need more\n"
                     "thrust as it sags and the result will not be repeatable.\n"
                     "Charge it first.")

        print("Place the drone in the clearest spot you have, nose away from you.")
        print(f"Each hop: {RAMP_SECONDS}s up, {hold}s hover, {RAMP_SECONDS}s down.")
        print("Press any key mid-hop to abort and land.\n")

        recent: deque[str] = deque(maxlen=3)     # the last answers, normalised
        while True:
            print(f"Trim: roll {trim.roll:+.1f}, pitch {trim.pitch:+.1f}   thrust {thrust}")
            if input("Enter to hop (no countdown), or 'q' to finish: ").strip().lower() == "q":
                break

            reason = hop(scf.cf, thrust, hold, trim)
            print(f"  landed ({reason}).")
            if reason == "interrupted":
                # Leave via the normal exit so the trim so far is still saved.
                print("  Interrupted -- keeping the trim found so far.")
                break

            answer = input(
                "  Which way did it go?  [r]ight [l]eft [f]orward [b]ack\n"
                "  Combine them for a diagonal, e.g. 'rb' for right and back.\n"
                "  [n] it stayed put   [t] it never lifted   [enter] skip: "
            ).strip().lower()

            if not answer:
                continue
            if "n" in answer:
                print("\nTrimmed. Saving.")
                break
            if "t" in answer:
                thrust = min(60000, thrust + 2000)
                print(f"  raising thrust to {thrust}")
                continue

            try:
                directions, ignored = parse_drift(answer)
            except ValueError as why:
                print(f"  {why}")
                continue
            if ignored:
                print(f"  (ignoring {', '.join(ignored)})")

            # Same drift repeating means the trim is moving the wrong way --
            # the correction should reduce it, not leave it unchanged.
            recent.append("".join(sorted(directions)))
            if len(recent) == recent.maxlen and len(set(recent)) == 1:
                axes = {CORRECTIONS[c][0] for c in directions}
                print(f"\n  That is 3 hops drifting the same way, so the "
                      f"{' and '.join(sorted(axes))}\n"
                      "  correction is going backwards. Stop and restart with:")
                flag = "" if invert_pitch or "pitch" not in axes else " --invert-pitch"
                print(f"    hoptest.py --reset-trim{flag}")
                print("  (or answer the opposite direction to walk it back)\n")

            trim = correct(trim, directions, invert_pitch)
            print(f"  trim -> roll {trim.roll:+.1f}, pitch {trim.pitch:+.1f}")
            if max(abs(trim.roll), abs(trim.pitch)) >= TRIM_LIMIT:
                print("  Trim is at its limit. Something is mechanically off --\n"
                      "  check props, motor mounts and battery position.")

        save_trim(*trim)
        print(f"Saved to {TRIM_FILE.name}: roll {trim.roll:+.1f}, pitch {trim.pitch:+.1f}")
        print("teleop.py picks this up automatically.")


if __name__ == "__main__":
    cfenv.cli(run)
