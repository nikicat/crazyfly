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

The lean is immediately followed by an equal one the other way, which cancels
the speed the first built up. Holding a single lean until touchdown instead
means the drone is still accelerating as it lands, and covers metres rather
than centimetres.

  uv run flightcheck.py --thrust 42000
  uv run flightcheck.py --lean 2 --lean-time 0.4   # even less room needed
"""
from __future__ import annotations

import argparse
import math
import statistics
import time

import cfenv
from flight import DT, MIN_THRUST, Interruptible, stop_motors

GRAVITY = 9.81
RAMP_SECONDS = 0.8
PROBE_PITCH = 3.0        # degrees of deliberate lean to command
LEAN_SECONDS = 0.5       # each lean; the second one cancels the first
TRACKING_FRACTION = 0.4  # measured/commanded above this counts as following

VARIABLES = {"stabilizer.pitch": "float", "stabilizer.roll": "float"}


def travel_estimate(lean_deg: float, lean_seconds: float) -> float:
    """Metres the drone covers over a lean-then-counter-lean pair.

    Accelerating at a for t covers a*t^2/2 and reaches speed a*t; the opposite
    lean brings that back to a standstill over another a*t^2/2. So the total is
    a*t^2, and it ends stationary rather than coasting into a wall.
    """
    accel = GRAVITY * math.tan(math.radians(abs(lean_deg)))
    return accel * lean_seconds ** 2


def probe(scf, thrust: int, lean: float, lean_seconds: float
          ) -> tuple[list[float], list[float]]:
    """One hop: lean one way, then the other. Returns pitch samples for each.

    The second lean is not just for symmetry -- it cancels the speed the first
    one built up. Holding a single lean through the descent instead means the
    drone is still accelerating when it lands, which covers metres rather than
    centimetres.
    """
    cf = scf.cf
    level = float(MIN_THRUST)
    captured: dict[str, list[float]] = {"plus": [], "minus": []}

    with cfenv.record_log(scf, VARIABLES) as samples, Interruptible() as interrupt:
        try:
            cf.commander.send_setpoint(0, 0, 0, 0)
            phases = (
                ("up", RAMP_SECONDS, 0.0, None),
                ("plus", lean_seconds, lean, "plus"),
                ("minus", lean_seconds, -lean, "minus"),
                ("down", RAMP_SECONDS, 0.0, None),
            )
            for phase, duration, pitch, capture in phases:
                start = time.time()
                mark = len(samples)
                while time.time() - start < duration:
                    if interrupt.requested:
                        raise KeyboardInterrupt
                    frac = (time.time() - start) / duration
                    if phase == "up":
                        level = MIN_THRUST + (thrust - MIN_THRUST) * frac
                    elif phase == "down":
                        level = MIN_THRUST + (thrust - MIN_THRUST) * (1 - frac)
                    else:
                        level = thrust
                    cf.commander.send_setpoint(0, pitch, 0, int(level))
                    time.sleep(DT)
                if capture:
                    # Skip the first samples: the attitude loop needs a moment
                    # to reach the new lean, and averaging the transient in
                    # would understate the response.
                    window = samples[mark:]
                    captured[capture] = [s["stabilizer.pitch"]
                                         for s in window[len(window) // 3:]]
        finally:
            stop_motors(cf, from_thrust=level, dt=DT)

    return captured["plus"], captured["minus"]


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--thrust", type=int, default=42000,
                   help="hover thrust to test (default 42000)")
    p.add_argument("--lean", type=float, default=PROBE_PITCH,
                   help=f"degrees of lean to command (default {PROBE_PITCH})")
    p.add_argument("--lean-time", type=float, default=LEAN_SECONDS,
                   help=f"seconds per lean (default {LEAN_SECONDS})")
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

        travel = travel_estimate(args.lean, args.lean_time)
        print(f"One hop at thrust {args.thrust}: {args.lean:.0f} deg lean for "
              f"{args.lean_time:.1f}s each way.")
        print(f"The second lean cancels the first, so it should travel about "
              f"{travel * 100:.0f} cm and stop.")
        print("Ctrl-C aborts and lands.\n")
        input("Press Enter to start: ")

        plus_samples, minus_samples = probe(scf, args.thrust, args.lean,
                                            args.lean_time)
        if not plus_samples or not minus_samples:
            print("\nNo telemetry during the leans. Re-run once it is flowing.")
            return

        plus = statistics.fmean(plus_samples) - resting
        minus = statistics.fmean(minus_samples) - resting
        separation = abs(plus - minus)
        expected = 2 * args.lean

        print()
        print(f"  commanded {args.lean:+.0f} -> {plus:+.2f} deg from resting")
        print(f"  commanded {-args.lean:+.0f} -> {minus:+.2f} deg from resting")
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
