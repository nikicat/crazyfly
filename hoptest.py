#!/usr/bin/env python3
"""Bounded hop tests for finding trim in a small room.

Flies a short, low, automatically-terminated hop, asks which way it went,
adjusts the trim and repeats. The drone lands on a timer rather than on your
reaction, so it cannot run away into a wall.

Drift grows with the square of time -- at a 0.3 degree lean it is about 11 cm
over 2 s but 2.8 m over 10 s. Keeping each hop to a couple of seconds is what
makes this safe indoors, so --hold is capped.

Get the bulk of the trim from trimcheck.py first; this is for confirming and
fine-tuning it, which is the part that genuinely needs flight.

  python hoptest.py                 # interactive trim loop
  python hoptest.py --thrust 40000  # if it does not leave the ground
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time

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
MAX_HOLD = 4.0           # refuse longer hops indoors; drift is quadratic
DEFAULT_THRUST = 36000   # deliberately low; raise until it just lifts
CORRECTION_STEP = 0.4    # degrees per answered hop

# Which way to move each trim when the drone drifts a given way.
#
# Both directions are measured on this hardware, not assumed:
#
#   roll  - a positive roll argument leans right. The measured -0.33 deg roll
#           bias predicted the observed rightward drift, so drifting right
#           needs less roll.
#   pitch - a positive pitch argument leans BACK, not forward. Commander
#           .send_setpoint transmits -pitch, which flips the axis relative to
#           the raw sign. Confirmed in flight: raising pitch trim to correct a
#           backward drift made the drift worse, so backward needs less pitch.
#
# That matches trimcheck.py, which recommends -pitch_bias for the same reason.
CORRECTIONS = {
    "r": ("roll", -1),
    "l": ("roll", +1),
    "f": ("pitch", +1),
    "b": ("pitch", -1),
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


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--thrust", type=int, default=DEFAULT_THRUST,
                   help=f"hover thrust, 10001-60000 (default {DEFAULT_THRUST})")
    p.add_argument("--hold", type=float, default=2.0,
                   help=f"seconds airborne, capped at {MAX_HOLD}")
    p.add_argument("--invert-pitch", action="store_true",
                   help="flip pitch correction direction if it makes drift worse")
    p.add_argument("--reset-trim", action="store_true",
                   help="start from zero trim instead of loading trim.json")
    p.add_argument("--roll-trim", type=float, default=None,
                   help="start from this roll trim (e.g. from trimcheck.py)")
    p.add_argument("--pitch-trim", type=float, default=None,
                   help="start from this pitch trim (e.g. from trimcheck.py)")
    p.add_argument("--uri", default=None)
    args = p.parse_args()

    hold = min(args.hold, MAX_HOLD)
    if hold < args.hold:
        print(f"Capping hop at {MAX_HOLD}s -- drift grows with time squared.")

    if args.reset_trim:
        roll_trim, pitch_trim = 0.0, 0.0
        print("Starting from zero trim.")
    else:
        roll_trim, pitch_trim = load_trim()
    if args.roll_trim is not None:
        roll_trim = args.roll_trim
    if args.pitch_trim is not None:
        pitch_trim = args.pitch_trim

    cfenv.init()
    uri = cfenv.resolve_uri(args.uri)
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
                  f"thrust {args.thrust}")
            if input("Enter to hop, or 'q' to finish: ").strip().lower() == "q":
                break

            for count in (3, 2, 1):
                print(f"  {count} ...", flush=True)
                time.sleep(0.7)
            reason = hop(cf, args.thrust, hold, roll_trim, pitch_trim)
            print(f"  landed ({reason}).")
            if reason == "interrupted":
                # Leave via the normal exit so the trim so far is still saved.
                print("  Interrupted -- keeping the trim found so far.")
                break

            answer = input(
                "  Which way did it go?  [r]ight [l]eft [f]orward [b]ack\n"
                "  [n] it stayed put   [t] it never lifted   [enter] skip: "
            ).strip().lower()

            if answer == "n":
                print("\nTrimmed. Saving.")
                break
            if answer == "t":
                args.thrust = min(60000, args.thrust + 2000)
                print(f"  raising thrust to {args.thrust}")
                continue
            if answer not in CORRECTIONS:
                continue

            # Same drift repeating means the trim is moving the wrong way --
            # the correction should reduce it, not leave it unchanged.
            repeats = repeats + 1 if answer == last_answer else 1
            last_answer = answer
            if repeats >= 3:
                axis_name = CORRECTIONS[answer][0]
                print(f"\n  That is 3 hops drifting the same way, so the {axis_name}\n"
                      "  correction is going backwards. Stop and restart with:")
                if axis_name == "pitch":
                    flag = "" if args.invert_pitch else " --invert-pitch"
                    print(f"    hoptest.py --reset-trim{flag}")
                else:
                    print("    hoptest.py --reset-trim")
                print("  (or answer the opposite direction to walk it back)\n")

            axis, direction = CORRECTIONS[answer]
            if axis == "pitch" and args.invert_pitch:
                direction = -direction
            if axis == "roll":
                roll_trim = clamp(roll_trim + direction * CORRECTION_STEP,
                                  -TRIM_LIMIT, TRIM_LIMIT)
            else:
                pitch_trim = clamp(pitch_trim + direction * CORRECTION_STEP,
                                   -TRIM_LIMIT, TRIM_LIMIT)
            print(f"  {axis} trim -> {roll_trim if axis == 'roll' else pitch_trim:+.1f}")
            if abs(roll_trim) >= TRIM_LIMIT or abs(pitch_trim) >= TRIM_LIMIT:
                print("  Trim is at its limit. Something is mechanically off --\n"
                      "  check props, motor mounts and battery position.")

        save_trim(roll_trim, pitch_trim)
        print(f"Saved to {TRIM_FILE.name}: roll {roll_trim:+.1f}, "
              f"pitch {pitch_trim:+.1f}")
        print("teleop.py picks this up automatically.")


if __name__ == "__main__":
    cfenv.run(main)
