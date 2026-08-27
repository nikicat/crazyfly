#!/usr/bin/env python3
"""Find the pitch sign on the ground, by watching which motor slows down.

Working the trim direction out in flight needs the drone to travel far enough
to see which way it went, which in a small room means hitting a wall before
the measurement is any good. This asks the same question without lifting off.

At a thrust too low to fly, the drone still runs its attitude controller. Ask
it for a pitch it cannot reach and it drives one motor up and the opposite one
down until the losing one stops outright. A stopped propeller is obvious;
which of four spinning ones is slightly slower is not, so this drives one to a
standstill rather than asking anyone to compare speeds.

The angle demanded is worked out from the response, not guessed. Base thrust
stays put: brushed motors need more duty to break away from rest than to keep
turning, so lowering it to stop one motor stops all four instead.

The Crazyflie 1.0 flies in "+" configuration: one motor at the front, one at
the back, one each side. Pitch is the front motor against the back one, and
roll is the other pair, so exactly two motors should respond here. Which two
tells us which pair is the pitch axis; which of them slows tells us the sign.

  * the motor that stops lets its side drop
  * the drone then accelerates toward that side

Which arm that turns out to be is a fact about how the IMU is mounted, not
about the paint on the frame. If it is the left or right arm, then what cflib
calls pitch moves this drone sideways, and a front-back drift is a roll
problem -- worth knowing before trimming the wrong axis.

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
DEFAULT_THRUST = 15000   # high enough that every motor reliably starts
MIN_CORRELATION = 0.5

# The losing motor floors at (thrust - differential). Getting it to zero by
# lowering the base thrust starves every motor at once: brushed motors need
# more duty to break away from rest than to keep turning, so below roughly
# 10000 none of them start and there is nothing to watch. Raise the commanded
# angle instead -- the differential grows with the error, so the losing motor
# reaches zero while the others merely spin faster.
PITCH_MARGIN = 1.3       # overshoot the angle that would just reach zero
MAX_PROBE_PITCH = 70.0   # beyond this the firmware clamps the setpoint anyway
MAX_ATTEMPTS = 3

MOTORS = ("motor.m1", "motor.m2", "motor.m3", "motor.m4")
VARIABLES = dict.fromkeys(MOTORS, "int32_t")


def drive(scf, thrust: int, schedule, axis: str = "pitch"
          ) -> tuple[list, dict[str, list]]:
    """Command each (angle, duration) in turn, recording motor outputs."""
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
                    if axis == "roll":
                        cf.commander.send_setpoint(pitch, 0, 0, thrust)
                    else:
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
                   help=f"degrees to demand (default {PROBE_PITCH})")
    p.add_argument("--axis", choices=("pitch", "roll"), default="pitch",
                   help="which axis to identify (default pitch)")
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
        print(f"\nStep 1: alternating {args.pitch:+.0f} deg of {args.axis}, {CYCLES} cycles ...")
        schedule = []
        for _ in range(CYCLES):
            schedule.append((+args.pitch, PHASE_SECONDS))
            schedule.append((-args.pitch, PHASE_SECONDS))
        commands, series = drive(scf, args.thrust, schedule, args.axis)
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
        other = "roll" if args.axis == "pitch" else "pitch"
        print(f"\n{args.axis.capitalize()} axis is {pitch_pair[0]} and {pitch_pair[1]}.")
        print(f"The {other} axis is {other_pair[0]} and {other_pair[1]} "
              f"(they should barely move here).")
        if report[pitch_pair[0]]["gain"] * report[pitch_pair[1]]["gain"] > 0:
            print("\nBoth moved the same way, which is thrust rather than pitch.\n"
                  "Raise --pitch or --thrust and retry.")
            return

        low_on_plus = min(pitch_pair, key=lambda m: report[m]["gain"])
        low_on_minus = max(pitch_pair, key=lambda m: report[m]["gain"])

        # Step 2: stop each end of the axis in turn. Which propeller STOPS is
        # something you can actually see; which is merely slower is not.
        # Asking about both ends cross-checks the answer, since the pitch pair
        # must sit across the frame from one another.
        print(f"\nStep 2: identifying the two {args.axis} arms.")
        print("Answers are relative to the drone's own front, as you know it.\n")

        # Step 1 already showed how far the differential moves per degree, so
        # start from an angle predicted to bottom the motor out rather than
        # from the probing angle, which demonstrably does not.
        step1_reach = args.thrust - min(report[low_on_plus]["low"],
                                        report[low_on_minus]["low"])
        scale = args.thrust / max(1.0, step1_reach)
        hold_pitch = min(MAX_PROBE_PITCH, args.pitch * scale * PITCH_MARGIN)
        print(f"Step 1 moved the motors {step1_reach:.0f} of the {args.thrust} "
              f"needed to reach zero,\nso holding {hold_pitch:.0f} deg rather "
              f"than {args.pitch:.0f}. Base thrust stays at {args.thrust} so "
              f"every\nmotor keeps turning; only the losing one should stop.\n")

        seen = {}
        for sign, motor in ((+1.0, low_on_plus), (-1.0, low_on_minus)):
            pitch = sign * hold_pitch
            for attempt in range(MAX_ATTEMPTS):
                print(f"  Holding {pitch:+.0f} deg for {HOLD_SECONDS:.0f}s -- "
                      f"watch for the propeller that STOPS.")
                input("  Press Enter when ready: ")
                _c, held = drive(scf, args.thrust, [(pitch, HOLD_SECONDS)], args.axis)
                floor = min((v for _t, v in held[motor]), default=0.0)
                if floor <= 0:
                    print(f"  ({motor} reached zero -- that propeller stopped.)")
                    break

                harder = min(MAX_PROBE_PITCH,
                             abs(pitch) * args.thrust / max(1.0, args.thrust - floor)
                             * PITCH_MARGIN)
                if harder <= abs(pitch) * 1.05 or attempt == MAX_ATTEMPTS - 1:
                    print(f"  ({motor} bottomed out at {floor:.0f} and will not "
                          f"reach zero.\n   Watch for the slowest propeller "
                          f"instead.)")
                    break

                print(f"  ({motor} only reached {floor:.0f}, still spinning. "
                      f"Demanding\n   {harder:.0f} deg instead -- watch again.)")
                pitch = sign * harder

            answer = input("  Which arm stopped?  [f] front  [b] back  "
                           "[l] left  [r] right: ").strip().lower()[:1]
            if answer not in ("f", "b", "l", "r"):
                print("\n  Not a recognised answer. Re-run and watch again.")
                return
            seen[motor] = answer
            print()

        names = {"f": "FRONT", "b": "BACK", "l": "LEFT", "r": "RIGHT"}
        opposites = {"f": "b", "b": "f", "l": "r", "r": "l"}
        first, second = seen[low_on_plus], seen[low_on_minus]

        for motor, where in seen.items():
            print(f"  {motor} is the {names[where]} arm")

        if opposites[first] != second:
            print("\nThose two arms are not opposite each other, and the pitch\n"
                  "pair has to sit across the frame. One reading was misread --\n"
                  "run it again and watch for the propeller that stops.")
            return

        print("\nConsistent: the two arms are opposite, as the pitch axis must be.")

        # Whichever arm drops on a positive pitch command is the direction a
        # positive command sends the drone. That is the whole answer; whether
        # it matches the arm you call the front is a separate question.
        print(f"\n  positive {args.axis}  ->  drops the {names[first]} arm  ->  "
              f"drone accelerates {names[first]}")
        print(f"  negative {args.axis}  ->  drops the {names[second]} arm  ->  "
              f"drone accelerates {names[second]}")

        if first in ("l", "r"):
            print(f"\nSo what cflib calls pitch moves this drone along its\n"
                  f"{names[first]}-{names[second]} axis, not front-back. The IMU is\n"
                  f"mounted 90 degrees from the arm you call the front, which is\n"
                  f"normal -- the frame is symmetric and the firmware picks the\n"
                  f"axis, not the paint.\n")
            print("Two consequences worth knowing:")
            print(f"  * fly with the {names[first]} arm leading and the controls\n"
                  f"    behave conventionally, pitch forward and roll sideways")
            print("  * a drift along your front-back axis is a ROLL problem on\n"
                  "    this drone, so pitch trim was never going to fix it")
            print("\nThat would explain trimming pitch making no difference.")
        else:
            forward = "f" if first == "f" else "b"
            print(f"\nSo a positive pitch command flies the drone {names[forward]}.")
            if first == "f":
                print("A backward drift is corrected by INCREASING pitch trim.")
                print('\n  hoptest CORRECTIONS should read  "b": ("pitch", +1)')
            else:
                print("A backward drift is corrected by DECREASING pitch trim.")
                print('\n  hoptest CORRECTIONS should read  "b": ("pitch", -1)')

        print("\nTell me the result and I will set it from this measurement.")


if __name__ == "__main__":
    cfenv.run(main)
