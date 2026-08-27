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

import statistics
import sys
import time

import typer

import cfenv
from flight import (
    DT,
    MIN_THRUST,
    TRIM_FILE,
    TRIM_LIMIT,
    Interruptible,
    Keyboard,
    clamp,
    load_trim,
    save_trim,
    stop_motors,
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


def battery(scf) -> float:
    """Current pack voltage. Raises LinkLost if telemetry stops, rather than
    returning 0.0 and tripping the low-battery check for the wrong reason."""
    series = cfenv.sample_series(scf, {"pm.vbat": "float"}, count=3)
    return statistics.fmean(series["pm.vbat"])


def hop(cf, thrust: int, hold: float, roll_trim: float, pitch_trim: float) -> str:
    """One ramp-up / hold / ramp-down cycle. Returns why it ended."""
    reason = "completed"
    level = float(MIN_THRUST)

    def send(value: float) -> None:
        cf.commander.send_setpoint(roll_trim, pitch_trim, 0, int(value))

    cf.commander.send_setpoint(0, 0, 0, 0)
    with Keyboard() as kb, Interruptible() as interrupt:
        try:
            phases = (("up", RAMP_SECONDS), ("hold", hold), ("down", RAMP_SECONDS))
            for phase, duration in phases:
                start = time.time()
                while True:
                    elapsed = time.time() - start
                    if elapsed >= duration:
                        break
                    if kb.poll() or kb.held:        # any key aborts
                        reason = "aborted"
                        break
                    if interrupt.requested:
                        reason = "interrupted"
                        break
                    frac = elapsed / duration
                    if phase == "up":
                        level = MIN_THRUST + (thrust - MIN_THRUST) * frac
                    elif phase == "hold":
                        level = thrust
                    else:
                        level = MIN_THRUST + (thrust - MIN_THRUST) * (1 - frac)
                    send(level)
                    time.sleep(DT)
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
                    level = MIN_THRUST + (from_level - MIN_THRUST) * (1 - i / steps)
                    send(level)
                    time.sleep(DT)
                level = MIN_THRUST
        finally:
            # Whatever happened -- abort, Ctrl-C, or an error mid-hop -- the
            # drone must not be left airborne with nothing driving it.
            stop_motors(cf, from_thrust=level, dt=DT)

    return reason


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
        start_roll, start_pitch = 0.0, 0.0
        print("Starting from zero trim.")
    else:
        start_roll, start_pitch = load_trim()
    roll_trim = start_roll if roll_trim is None else roll_trim
    pitch_trim = start_pitch if pitch_trim is None else pitch_trim

    cfenv.init()
    uri = cfenv.resolve_uri(uri)
    print(f"Connecting to {uri} ...")
    with cfenv.connect(uri) as scf:
        cf = scf.cf
        scf.wait_for_params()

        vbat = battery(scf)
        print(f"Connected. Battery {vbat:.2f} V\n")
        if vbat < 3.6:
            sys.exit("Battery too low for a meaningful hop -- it will need more\n"
                     "thrust as it sags and the result will not be repeatable.\n"
                     "Charge it first.")

        print("Place the drone in the clearest spot you have, nose away from you.")
        print(f"Each hop: {RAMP_SECONDS}s up, {hold}s hover, {RAMP_SECONDS}s down.")
        print("Press any key mid-hop to abort and land.\n")

        last_answer = None
        repeats = 0
        while True:
            print(f"Trim: roll {roll_trim:+.1f}, pitch {pitch_trim:+.1f}   "
                  f"thrust {thrust}")
            if input("Enter to hop (no countdown), or 'q' to finish: ").strip().lower() == "q":
                break

            reason = hop(cf, thrust, hold, roll_trim, pitch_trim)
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

            # A drift is usually diagonal, so accept any combination rather
            # than only the four single letters. Silently ignoring anything
            # else meant an answer of "rb" changed nothing at all, with no
            # sign that it had been dropped.
            directions = [c for c in dict.fromkeys(answer) if c in CORRECTIONS]
            unknown = [c for c in dict.fromkeys(answer) if c not in CORRECTIONS]
            if not directions:
                print(f"  '{answer}' has no direction in it, so nothing changed."
                      f"\n  Use r, l, f, b -- combined if it went diagonally.")
                continue
            if unknown:
                print(f"  (ignoring {', '.join(unknown)})")

            axes = {CORRECTIONS[c][0] for c in directions}
            if len(axes) < len(directions):
                print(f"  '{answer}' asks for two opposite corrections on one "
                      f"axis, which\n  cancel out. Nothing changed.")
                continue

            # Same drift repeating means the trim is moving the wrong way --
            # the correction should reduce it, not leave it unchanged.
            normalised = "".join(sorted(directions))
            repeats = repeats + 1 if normalised == last_answer else 1
            last_answer = normalised
            if repeats >= 3:
                moved = " and ".join(sorted(axes))
                print(f"\n  That is 3 hops drifting the same way, so the {moved}\n"
                      "  correction is going backwards. Stop and restart with:")
                flag = "" if invert_pitch or "pitch" not in axes else " --invert-pitch"
                print(f"    hoptest.py --reset-trim{flag}")
                print("  (or answer the opposite direction to walk it back)\n")

            for letter in directions:
                axis, direction = CORRECTIONS[letter]
                if axis == "pitch" and invert_pitch:
                    direction = -direction
                if axis == "roll":
                    roll_trim = clamp(roll_trim + direction * CORRECTION_STEP,
                                      -TRIM_LIMIT, TRIM_LIMIT)
                else:
                    pitch_trim = clamp(pitch_trim + direction * CORRECTION_STEP,
                                       -TRIM_LIMIT, TRIM_LIMIT)

            print(f"  trim -> roll {roll_trim:+.1f}, pitch {pitch_trim:+.1f}")
            if abs(roll_trim) >= TRIM_LIMIT or abs(pitch_trim) >= TRIM_LIMIT:
                print("  Trim is at its limit. Something is mechanically off --\n"
                      "  check props, motor mounts and battery position.")

        save_trim(roll_trim, pitch_trim)
        print(f"Saved to {TRIM_FILE.name}: roll {roll_trim:+.1f}, "
              f"pitch {pitch_trim:+.1f}")
        print("teleop.py picks this up automatically.")


if __name__ == "__main__":
    cfenv.run(lambda: typer.run(run))
