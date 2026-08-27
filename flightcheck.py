#!/usr/bin/env python3
"""Is the drone actually flying, and which way does pitch trim lean it?

Trimming is meaningless until the drone genuinely leaves the ground. One that
is dragging a leg pivots on that leg instead of translating, so "it drifted
back" describes the tipping, not the flight, and trim corrections based on it
point in whichever direction the drone happened to fall. That produces exactly
the contradiction of a correction making things worse in both directions.

This tells the two apart without asking you to judge anything. It commands a
deliberate lean and records what the attitude estimate actually does:

  * airborne and controlled -> the measured pitch tracks the commanded one
  * still on the ground     -> the ground holds it, so it cannot follow

Once it does track, the same measurement gives the trim direction directly --
no guessing about which way "back" maps onto the sign.

  uv run flightcheck.py --thrust 42000
"""
from __future__ import annotations

import argparse
import statistics
import time

import cfenv
from flight import DT, MIN_THRUST, Interruptible, stop_motors

RAMP_SECONDS = 0.8
HOLD_SECONDS = 1.2
PROBE_PITCH = 5.0        # degrees of deliberate lean to command
TRACKING_FRACTION = 0.4  # measured/commanded above this counts as following

VARIABLES = {"stabilizer.pitch": "float", "stabilizer.roll": "float"}


def hop_and_record(scf, thrust: int, pitch_trim: float) -> list[float]:
    """Fly one hop holding `pitch_trim`; return pitch samples from the hover."""
    cf = scf.cf
    level = float(MIN_THRUST)
    hover: list[float] = []

    with cfenv.record_log(scf, VARIABLES) as samples, Interruptible() as interrupt:
        try:
            cf.commander.send_setpoint(0, 0, 0, 0)
            phases = (("up", RAMP_SECONDS), ("hold", HOLD_SECONDS),
                      ("down", RAMP_SECONDS))
            for phase, duration in phases:
                start = time.time()
                mark = len(samples)
                while time.time() - start < duration:
                    if interrupt.requested:
                        raise KeyboardInterrupt
                    frac = (time.time() - start) / duration
                    if phase == "up":
                        level = MIN_THRUST + (thrust - MIN_THRUST) * frac
                    elif phase == "hold":
                        level = thrust
                    else:
                        level = MIN_THRUST + (thrust - MIN_THRUST) * (1 - frac)
                    cf.commander.send_setpoint(0, pitch_trim, 0, int(level))
                    time.sleep(DT)
                if phase == "hold":
                    hover = [s["stabilizer.pitch"] for s in samples[mark:]]
        finally:
            stop_motors(cf, from_thrust=level, dt=DT)

    return hover


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--thrust", type=int, default=42000,
                   help="hover thrust to test (default 42000)")
    p.add_argument("--uri", default=None)
    args = p.parse_args()

    cfenv.init()
    uri = cfenv.resolve_uri(args.uri)
    print(f"Connecting to {uri} ...")
    with cfenv.connect(uri) as scf:
        scf.wait_for_params()
        vbat = statistics.fmean(
            cfenv.sample_series(scf, {"pm.vbat": "float"}, count=3)["pm.vbat"])
        print(f"Connected. Battery {vbat:.2f} V\n")

        resting = statistics.fmean(
            cfenv.sample_series(scf, {"stabilizer.pitch": "float"},
                                count=10)["stabilizer.pitch"])
        print(f"Resting pitch on the ground: {resting:+.2f} deg\n")

        print(f"Two hops at thrust {args.thrust}, each commanding a "
              f"{PROBE_PITCH:.0f} deg lean, one each way.")
        print("Clear space around the drone. Ctrl-C aborts and lands.\n")
        input("Press Enter to start: ")

        results = {}
        for trim in (PROBE_PITCH, -PROBE_PITCH):
            print(f"  hop with pitch trim {trim:+.0f} ...", flush=True)
            hover = hop_and_record(scf, args.thrust, trim)
            if not hover:
                print("    no telemetry during the hover")
                continue
            mean = statistics.fmean(hover)
            results[trim] = mean - resting
            print(f"    measured pitch {mean:+.2f} deg "
                  f"({mean - resting:+.2f} from resting)")
            time.sleep(1.0)

        print()
        if len(results) < 2:
            print("Not enough data. Re-run once telemetry is flowing.")
            return

        plus, minus = results[PROBE_PITCH], results[-PROBE_PITCH]
        separation = abs(plus - minus)
        expected = 2 * PROBE_PITCH

        print(f"  commanded {PROBE_PITCH:+.0f} -> {plus:+.2f} deg")
        print(f"  commanded {-PROBE_PITCH:+.0f} -> {minus:+.2f} deg")
        print(f"  separation {separation:.2f} deg of a possible {expected:.0f}\n")

        if separation < expected * TRACKING_FRACTION:
            print("The attitude barely moved between the two commands, so the\n"
                  "drone is NOT flying -- the ground is holding it. Trimming now\n"
                  "is meaningless, and a leg touching down is the giveaway.\n")
            print(f"Raise the thrust and retry:\n"
                  f"    uv run flightcheck.py --thrust {args.thrust + 3000}\n")
            print("Increase in steps of ~2000 until it lifts cleanly. If it never\n"
                  "does, the battery is too flat to hover -- charge it first.")
            return

        print("The attitude follows the command, so the drone is flying.\n")
        # Sign of the response is what settles the trim direction, without
        # anyone having to judge which way it drifted.
        if plus < minus:
            print("A positive pitch trim drives the estimate NEGATIVE.")
            leans = "positive pitch trim leans the drone one way"
        else:
            print("A positive pitch trim drives the estimate POSITIVE.")
            leans = "positive pitch trim leans the drone the other way"
        print(f"  ({leans}; run orient.py to tie that to a physical edge.)\n")
        print(f"Use this thrust for hoptest:\n"
              f"    uv run hoptest.py --thrust {args.thrust} --reset-trim")


if __name__ == "__main__":
    cfenv.run(main)
