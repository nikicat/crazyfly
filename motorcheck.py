#!/usr/bin/env python3
"""Find the pitch sign on the ground, by watching which motor slows down.

Working the trim direction out in flight needs the drone to travel far enough
to see which way it went, which in a small room means hitting a wall before
the measurement is any good. This asks the same question without lifting off.

At a thrust too low to fly, the drone still runs its attitude controller. Ask
it for a pitch it cannot reach and it drives one motor up and the opposite one
down, harder and harder, until the low side is barely turning or has stopped.
A stopped propeller is obvious; four spinning ones are not, which is why this
looks for the slow motor rather than the fast one.

The Crazyflie 1.0 flies in "+" configuration: one motor at the front, one at
the back, one each side. Pitch is the front motor against the back one, and
roll is the other pair, so exactly two motors should respond here. Which two
tells us which pair is the pitch axis; which of them slows tells us the sign.

  * the motor that slows lets its side drop
  * a dropping front means nose-down, which flies forward

SAFETY: the propellers spin. Put the drone on a flat surface, hands clear. It
cannot lift at this thrust but it may skitter, so hold it by the battery.

  uv run motorcheck.py
"""
from __future__ import annotations

import argparse
import statistics
import time

import cfenv
from flight import DT, Interruptible, stop_motors
from signals import best_fit

PROBE_PITCH = 20.0       # degrees; deliberately unreachable, to saturate output
PHASE_SECONDS = 0.8
CYCLES = 3
HOLD_SECONDS = 4.0       # steady lean, long enough to watch the props
DEFAULT_THRUST = 15000   # low enough that the losing motor reaches zero
MIN_CORRELATION = 0.5

MOTORS = ("motor.m1", "motor.m2", "motor.m3", "motor.m4")
VARIABLES = dict.fromkeys(MOTORS, "int32_t")


def drive(scf, thrust: int, schedule) -> tuple[list, dict[str, list]]:
    """Command each (pitch, duration) in turn, recording motor outputs."""
    cf = scf.cf
    commands: list[tuple[float, float]] = []
    series: dict[str, list[tuple[float, float]]] = {m: [] for m in MOTORS}
    taken = 0

    with cfenv.record_log(scf, VARIABLES, period_ms=50) as raw, \
            Interruptible() as interrupt:
        try:
            cf.commander.send_setpoint(0, 0, 0, 0)
            for pitch, duration in schedule:
                start = time.time()
                while time.time() - start < duration:
                    if interrupt.requested:
                        raise KeyboardInterrupt
                    now = time.time()
                    cf.commander.send_setpoint(0, pitch, 0, thrust)
                    commands.append((now, pitch))
                    while taken < len(raw):
                        for motor in MOTORS:
                            series[motor].append((now, float(raw[taken][motor])))
                        taken += 1
                    time.sleep(DT)
        finally:
            stop_motors(cf, from_thrust=thrust, dt=DT)

    return commands, series


def summarise(commands, series) -> dict[str, dict]:
    """Per-motor gain against the commanded pitch, plus the range it covered."""
    report = {}
    for motor, samples in series.items():
        if len(samples) < 8:
            continue
        _lag, gain, correlation = best_fit(commands, samples)
        values = [value for _t, value in samples]
        report[motor] = {
            "gain": gain,
            "correlation": correlation,
            "low": min(values),
            "high": max(values),
            "mean": statistics.fmean(values),
        }
    return report


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--thrust", type=int, default=DEFAULT_THRUST,
                   help=f"motor thrust, must not lift (default {DEFAULT_THRUST})")
    p.add_argument("--pitch", type=float, default=PROBE_PITCH,
                   help=f"degrees of pitch to demand (default {PROBE_PITCH})")
    p.add_argument("--uri", default=None)
    args = p.parse_args()

    cfenv.init()
    uri = cfenv.resolve_uri(args.uri)
    print(f"Connecting to {uri} ...")
    with cfenv.connect(uri) as scf:
        scf.wait_for_params()
        print("Connected.\n")

        print("The propellers will spin, at a thrust far too low to lift.")
        print("Put the drone on a flat surface, props clear, hands away.")
        print("Ctrl-C stops the motors at any point.\n")
        if input("Type 'go' to start: ").strip().lower() != "go":
            print("Nothing done.")
            return

        # Step 1: alternate, to find which pair is the pitch axis.
        print(f"\nStep 1: alternating {args.pitch:+.0f} deg, {CYCLES} cycles ...")
        schedule = []
        for _ in range(CYCLES):
            schedule.append((+args.pitch, PHASE_SECONDS))
            schedule.append((-args.pitch, PHASE_SECONDS))
        commands, series = drive(scf, args.thrust, schedule)
        report = summarise(commands, series)

        if len(report) < 4:
            print("Not all motors reported. Power-cycle and retry.")
            return

        print("\n  motor      gain   correlation      range")
        for motor in MOTORS:
            row = report[motor]
            print(f"  {motor}  {row['gain']:+8.1f}     {row['correlation']:+.2f}"
                  f"      {row['low']:.0f} - {row['high']:.0f}")

        responding = [m for m in MOTORS
                      if abs(report[m]["correlation"]) >= MIN_CORRELATION]
        if len(responding) < 2:
            print("\nThe motors do not track the command. Raise --thrust so the\n"
                  "controller has room to push, and retry.")
            return

        ranked = sorted(MOTORS, key=lambda m: abs(report[m]["gain"]), reverse=True)
        pitch_pair, other_pair = ranked[:2], ranked[2:]
        print(f"\nPitch axis is {pitch_pair[0]} and {pitch_pair[1]}.")
        print(f"Roll axis is {other_pair[0]} and {other_pair[1]} "
              f"(they should barely move here).")
        if report[pitch_pair[0]]["gain"] * report[pitch_pair[1]]["gain"] > 0:
            print("\nBoth moved the same way, which is thrust rather than pitch.\n"
                  "Raise --pitch or --thrust and retry.")
            return

        low_on_plus = min(pitch_pair, key=lambda m: report[m]["gain"])
        low_on_minus = max(pitch_pair, key=lambda m: report[m]["gain"])

        # Step 2: stop each end of the axis in turn. Which propeller STOPS is
        # something you can actually see; which is merely slower is not. The
        # positions are relative to where you are standing, so nothing depends
        # on a marking being right -- and asking twice cross-checks the answer.
        print("\nStep 2: identifying the two pitch arms.")
        print("Leave the drone exactly where it is and do not turn it between\n"
              "the two runs. Answer relative to YOU.\n")

        seen = {}
        for label, pitch, motor in ((f"{args.pitch:+.0f}", +args.pitch, low_on_plus),
                                    (f"{-args.pitch:+.0f}", -args.pitch, low_on_minus)):
            print(f"  Holding {label} deg for {HOLD_SECONDS:.0f}s -- watch for "
                  f"the propeller that STOPS.")
            input("  Press Enter when ready: ")
            _c, held = drive(scf, args.thrust, [(pitch, HOLD_SECONDS)])
            floor = min((v for _t, v in held[motor]), default=None)
            if floor is not None:
                if floor > 0:
                    print(f"  ({motor} bottomed out at {floor:.0f}, so it slowed "
                          f"but never stopped.\n"
                          f"   If you could not tell, Ctrl-C and retry with "
                          f"--thrust {max(8000, args.thrust - 4000)}.)")
                else:
                    print(f"  ({motor} reached zero, so that propeller stopped.)")
            answer = input("  Which arm stopped?  [n] nearest you  [f] furthest  "
                           "[l] left  [r] right: ").strip().lower()[:1]
            if answer not in ("n", "f", "l", "r"):
                print("\n  Not a recognised answer. Re-run and watch again.")
                return
            seen[motor] = answer
            print()

        names = {"n": "NEAREST you", "f": "FURTHEST from you",
                 "l": "on your LEFT", "r": "on your RIGHT"}
        for motor, where in seen.items():
            print(f"  {motor} is the arm {names[where]}")

        opposites = {"n": "f", "f": "n", "l": "r", "r": "l"}
        first, second = seen[low_on_plus], seen[low_on_minus]
        if opposites[first] != second:
            print("\nThose two arms are not opposite each other, so one of the\n"
                  "readings was misread -- the pitch pair must sit across the\n"
                  "frame from one another. Run it again and watch carefully.")
            return

        print("\nConsistent: the two arms are opposite, as the pitch axis must be.")
        print(f"\nA positive pitch command drops the arm {names[first]},\n"
              f"so the drone accelerates that way.")
        print(f"A negative pitch command drops the arm {names[second]}.")

        print("\nSo, standing where you are now:")
        print(f"  positive pitch trim  ->  drone moves {names[first]}")
        print(f"  negative pitch trim  ->  drone moves {names[second]}")
        print("\nWhichever of those two directions you have been calling "
              "'backward',\ntell me and I will set hoptest's correction from "
              "this measurement.")


if __name__ == "__main__":
    cfenv.run(main)
