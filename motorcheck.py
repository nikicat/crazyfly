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
DEFAULT_THRUST = 20000   # spins the motors, nowhere near enough to lift
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

        slows = min(pitch_pair, key=lambda m: report[m]["gain"])
        print(f"\nWith a positive pitch commanded, {slows} slows down.")

        # Step 2: hold it steady so the slow propeller can actually be seen.
        print(f"\nStep 2: holding {args.pitch:+.0f} deg for {HOLD_SECONDS:.0f}s.")
        print("Watch for the ONE propeller that is clearly slowest, or stopped.")
        input("Press Enter when ready: ")
        drive(scf, args.thrust, [(+args.pitch, HOLD_SECONDS)])

        print("\nWhich arm was that? Positions are relative to the front you\n"
              "marked with orient.py, with the front pointing away from you.\n")
        answer = input("  [f] front  [b] back  [l] left  [r] right: ").strip().lower()

        if answer.startswith(("l", "r")):
            print("\nThat is the roll axis, so the edge you marked as the front\n"
                  "is 90 degrees out. Re-run orient.py, re-mark, and try again.")
            return

        if answer.startswith("f"):
            print("\nThe FRONT motor slows, so the nose drops: a positive pitch\n"
                  "command flies the drone FORWARD.")
            print("A backward drift is therefore corrected by INCREASING pitch "
                  "trim.")
            mapping = '"b": ("pitch", +1)'
        elif answer.startswith("b"):
            print("\nThe BACK motor slows, so the tail drops and the nose rises:\n"
                  "a positive pitch command flies the drone BACKWARD.")
            print("A backward drift is therefore corrected by DECREASING pitch "
                  "trim.")
            mapping = '"b": ("pitch", -1)'
        else:
            print("\nNo answer recorded. Re-run and watch for the slow prop.")
            return

        print(f"\n  hoptest.py CORRECTIONS should read  {mapping}")
        print("\nTell me which one and I will set it from this measurement "
              "rather than by inference.")


if __name__ == "__main__":
    cfenv.run(main)
