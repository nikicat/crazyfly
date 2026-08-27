#!/usr/bin/env python3
"""Find the pitch sign on the ground, by watching which motors spin up.

Working the trim direction out in flight needs the drone to fly far enough to
see which way it went, which in a small room means hitting a wall before the
measurement is any good. This asks the same question without leaving the
ground.

At a thrust too low to lift, the drone still runs its attitude controller. Ask
it for a pitch it cannot reach and it pushes harder and harder on the motors
that would get it there. Those motors are the answer: whichever pair speeds up
is the pair the controller is using to lift that side.

  * motors on the side that lifts -> that side goes up -> the drone would
    accelerate toward the opposite side

So a positive pitch command that speeds up the REAR motors means positive pitch
flies forward, and vice versa. You only have to look at which props spin up.

SAFETY: the propellers spin. Put the drone on a flat surface and keep hands
clear. It will not lift at this thrust, but it may skitter, so hold it down by
the battery if it wanders.

  uv run motorcheck.py
"""
from __future__ import annotations

import argparse
import time

import cfenv
from flight import DT, Interruptible, stop_motors
from signals import best_fit

PROBE_PITCH = 8.0        # degrees; large, since it never has to be reached
PHASE_SECONDS = 0.8
CYCLES = 3
DEFAULT_THRUST = 18000   # spins the motors, nowhere near enough to lift
MIN_CORRELATION = 0.5

MOTORS = ("motor.m1", "motor.m2", "motor.m3", "motor.m4")
VARIABLES = dict.fromkeys(MOTORS, "int32_t")


def run(scf, thrust: int, pitch: float
        ) -> tuple[list[tuple[float, float]], dict[str, list[tuple[float, float]]]]:
    """Alternate the commanded pitch; record commands and each motor output."""
    cf = scf.cf
    commands: list[tuple[float, float]] = []
    series: dict[str, list[tuple[float, float]]] = {m: [] for m in MOTORS}
    taken = 0

    with cfenv.record_log(scf, VARIABLES, period_ms=50) as raw, \
            Interruptible() as interrupt:
        try:
            cf.commander.send_setpoint(0, 0, 0, 0)
            phases = []
            for _ in range(CYCLES):
                phases.append(+pitch)
                phases.append(-pitch)

            for offset in phases:
                start = time.time()
                while time.time() - start < PHASE_SECONDS:
                    if interrupt.requested:
                        raise KeyboardInterrupt
                    now = time.time()
                    cf.commander.send_setpoint(0, offset, 0, thrust)
                    commands.append((now, offset))
                    while taken < len(raw):
                        for motor in MOTORS:
                            series[motor].append((now, float(raw[taken][motor])))
                        taken += 1
                    time.sleep(DT)
        finally:
            stop_motors(cf, from_thrust=thrust, dt=DT)

    return commands, series


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
        print("Put the drone on a flat surface with the props clear of "
              "everything, and keep your hands away.")
        print(f"It will demand {args.pitch:+.0f} deg, then {-args.pitch:+.0f}, "
              f"{CYCLES} times, {PHASE_SECONDS}s each.")
        print("Watch which propellers speed up. Ctrl-C stops the motors.\n")
        if input("Type 'go' to start: ").strip().lower() != "go":
            print("Nothing done.")
            return

        commands, series = run(scf, args.thrust, args.pitch)

        print("\n  motor   gain per degree commanded   correlation")
        gains = {}
        for motor in MOTORS:
            samples = series[motor]
            if len(samples) < 8:
                print(f"  {motor}   too few samples")
                continue
            _lag, gain, correlation = best_fit(commands, samples)
            gains[motor] = (gain, correlation)
            arrow = "UP  " if gain > 0 else "DOWN"
            print(f"  {motor}   {gain:+9.1f}  {arrow}            {correlation:+.2f}")

        usable = {m: g for m, (g, c) in gains.items()
                  if abs(c) >= MIN_CORRELATION}
        if len(usable) < 2:
            print("\nThe motor outputs do not track the command well enough to\n"
                  "read. Raise --thrust a little (the controller needs headroom\n"
                  "to push against) and try again.")
            return

        speeds_up = sorted(usable, key=lambda m: usable[m], reverse=True)
        rising = [m for m in speeds_up if usable[m] > 0]
        falling = [m for m in speeds_up if usable[m] < 0]

        # A pitch command tilts the drone by driving one pair up and the other
        # down. If every motor moved the same way we are reading the thrust
        # changing, not the pitch, and it says nothing about direction.
        if not rising or not falling:
            print("\nEvery motor moved the same way, so this is thrust rather\n"
                  "than pitch and says nothing about direction.\n"
                  "Raise --pitch or --thrust and retry.")
            return

        print(f"\nCommanding {args.pitch:+.0f} deg speeds UP: "
              f"{', '.join(rising) if rising else 'none'}")
        print(f"                          slows DOWN: "
              f"{', '.join(falling) if falling else 'none'}")

        print("\nNow tell me where those motors are. With the drone's front\n"
              "pointing away from you, were the propellers that sped up the\n"
              "two NEAREST you, or the two FURTHEST away?\n")
        answer = input("  [n] nearest (rear)   [f] furthest (front): ").strip().lower()

        if answer.startswith("n"):
            # Rear lifts -> nose drops -> accelerates forward.
            print("\nRear motors lift, so the nose drops and the drone "
                  "accelerates FORWARD.")
            print("So a positive pitch trim flies it FORWARD, and a backward\n"
                  "drift is corrected by INCREASING pitch trim.")
            print("\n    hoptest.py CORRECTIONS should map 'b' to pitch +1")
        elif answer.startswith("f"):
            # Front lifts -> nose rises -> accelerates backward.
            print("\nFront motors lift, so the nose rises and the drone "
                  "accelerates BACKWARD.")
            print("So a positive pitch trim flies it BACKWARD, and a backward\n"
                  "drift is corrected by DECREASING pitch trim.")
            print("\n    hoptest.py CORRECTIONS should map 'b' to pitch -1")
        else:
            print("\nNo answer recorded. Re-run and watch the props.")
            return

        print("\nRun orient.py first if you are unsure which edge is the front.")


if __name__ == "__main__":
    cfenv.run(main)
