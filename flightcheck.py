#!/usr/bin/env python3
"""Is the drone actually flying, and which way does pitch trim lean it?

Trimming is meaningless until the drone genuinely leaves the ground. One that
is dragging a leg pivots on that leg instead of translating, so "it drifted
back" describes the tipping, not the flight, and trim corrections based on it
point in whichever direction the drone happened to fall.

This commands a deliberate lean each way and measures what the attitude
estimate actually does, so nobody has to judge a drift direction.

Two things make that harder than it sounds on a Crazyflie 1.0:

  * Telemetry lags. A 250K link carrying 33 setpoints/s has log packets
    queueing behind them, and the delay is a good fraction of a second. Slicing
    the samples by wall-clock phase boundaries therefore averages the previous
    lean into the current one, and both leans come out looking the same. So the
    lag is measured rather than assumed: the commanded signal is compared
    against the samples at a range of offsets, and the best fit wins.

  * The drone flies off on its own bias. With no trim the estimator's offset
    becomes a real lean held for the whole flight, which covers about a metre
    even when the probe itself would only cover ten centimetres. So the flight
    is flown at the saved trim, and the probe leans around that.

  uv run flightcheck.py --thrust 42000
  uv run flightcheck.py --lean 2 --lean-time 0.4   # even less room needed
"""
from __future__ import annotations

import argparse
import math
import statistics
import time

import cfenv
from flight import DT, MIN_THRUST, Interruptible, load_trim, stop_motors

GRAVITY = 9.81
RAMP_SECONDS = 0.8
PROBE_PITCH = 3.0        # degrees of deliberate lean to command
LEAN_SECONDS = 0.5       # each lean; the second one cancels the first
TRACKING_FRACTION = 0.4  # measured/commanded above this counts as following
MAX_LAG = 1.0            # widest telemetry delay to search for, seconds
LAG_STEP = 0.05

VARIABLES = {"stabilizer.pitch": "float", "stabilizer.roll": "float"}


def travel_estimate(lean_deg: float, lean_seconds: float) -> float:
    """Metres covered over a lean-then-counter-lean pair.

    Accelerating at a for t covers a*t^2/2 and reaches speed a*t; the opposite
    lean brings that back to a standstill over another a*t^2/2, so the total is
    a*t^2 and it ends stationary rather than coasting into a wall.
    """
    accel = GRAVITY * math.tan(math.radians(abs(lean_deg)))
    return accel * lean_seconds ** 2


def fly(scf, thrust: int, base_pitch: float, lean: float, lean_seconds: float
        ) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    """Fly ramp / lean / counter-lean / ramp.

    Returns (commands, samples), each a list of (wall_clock, pitch_degrees).
    Both are timestamped on arrival so the lag between them can be recovered
    afterwards instead of being assumed to be zero.
    """
    cf = scf.cf
    level = float(MIN_THRUST)
    commands: list[tuple[float, float]] = []
    samples: list[tuple[float, float]] = []

    with cfenv.record_log(scf, VARIABLES) as raw, Interruptible() as interrupt:
        try:
            cf.commander.send_setpoint(0, 0, 0, 0)
            phases = (
                ("up", RAMP_SECONDS, 0.0),
                ("plus", lean_seconds, lean),
                ("minus", lean_seconds, -lean),
                ("down", RAMP_SECONDS, 0.0),
            )
            for phase, duration, offset in phases:
                start = time.time()
                while True:
                    now = time.time()
                    elapsed = now - start
                    if elapsed >= duration:
                        break
                    if interrupt.requested:
                        raise KeyboardInterrupt
                    frac = elapsed / duration
                    if phase == "up":
                        level = MIN_THRUST + (thrust - MIN_THRUST) * frac
                    elif phase == "down":
                        level = MIN_THRUST + (thrust - MIN_THRUST) * (1 - frac)
                    else:
                        level = thrust

                    pitch = base_pitch + offset
                    cf.commander.send_setpoint(0, pitch, 0, int(level))
                    commands.append((now, offset))
                    # Drain whatever telemetry has arrived, stamped with now.
                    while len(samples) < len(raw):
                        samples.append((now, raw[len(samples)]["stabilizer.pitch"]))
                    time.sleep(DT)
        finally:
            stop_motors(cf, from_thrust=level, dt=DT)

    return commands, samples


def response_at_lag(commands, samples, lag: float) -> float | None:
    """Mean pitch while +lean was commanded, minus while -lean was, at `lag`.

    Returns None when either group is empty at this offset.
    """
    if not commands:
        return None
    schedule = [(t, offset) for t, offset in commands]
    plus: list[float] = []
    minus: list[float] = []

    index = 0
    for stamp, pitch in samples:
        target = stamp - lag
        while index + 1 < len(schedule) and schedule[index + 1][0] <= target:
            index += 1
        while index > 0 and schedule[index][0] > target:
            index -= 1
        offset = schedule[index][1]
        if offset > 0:
            plus.append(pitch)
        elif offset < 0:
            minus.append(pitch)

    if not plus or not minus:
        return None
    return statistics.fmean(plus) - statistics.fmean(minus)


def best_fit(commands, samples) -> tuple[float, float]:
    """Find the telemetry lag giving the strongest response, and that response."""
    best_lag, best_response = 0.0, 0.0
    lag = 0.0
    while lag <= MAX_LAG:
        response = response_at_lag(commands, samples, lag)
        if response is not None and abs(response) > abs(best_response):
            best_lag, best_response = lag, response
        lag += LAG_STEP
    return best_lag, best_response


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--thrust", type=int, default=42000,
                   help="hover thrust to test (default 42000)")
    p.add_argument("--lean", type=float, default=PROBE_PITCH,
                   help=f"degrees of lean to command (default {PROBE_PITCH})")
    p.add_argument("--lean-time", type=float, default=LEAN_SECONDS,
                   help=f"seconds per lean (default {LEAN_SECONDS})")
    p.add_argument("--pitch-trim", type=float, default=None,
                   help="fly at this pitch trim (default: from trim.json)")
    p.add_argument("--uri", default=None)
    args = p.parse_args()

    saved_roll, saved_pitch = load_trim()
    base_pitch = saved_pitch if args.pitch_trim is None else args.pitch_trim

    cfenv.init()
    uri = cfenv.resolve_uri(args.uri)
    print(f"Connecting to {uri} ...")
    with cfenv.connect(uri) as scf:
        scf.wait_for_params()
        vbat = statistics.fmean(
            cfenv.sample_series(scf, {"pm.vbat": "float"}, count=3)["pm.vbat"])
        print(f"Connected. Battery {vbat:.2f} V")

        resting = statistics.fmean(
            cfenv.sample_series(scf, {"stabilizer.pitch": "float"},
                                count=10)["stabilizer.pitch"])
        print(f"Resting pitch on the ground: {resting:+.2f} deg")

        suggested = -resting
        if base_pitch == 0.0:
            print(f"\nFlying at zero pitch trim, so the {resting:+.2f} deg bias\n"
                  f"becomes a real lean for the whole flight -- expect roughly\n"
                  f"{travel_from_bias(resting):.1f} m of drift on top of the probe.\n"
                  f"Pass --pitch-trim {suggested:+.1f} to cancel it.")
        else:
            print(f"Flying at pitch trim {base_pitch:+.1f} deg.")

        travel = travel_estimate(args.lean, args.lean_time)
        print(f"\nOne hop at thrust {args.thrust}: {args.lean:.0f} deg lean for "
              f"{args.lean_time:.1f}s each way, about {travel * 100:.0f} cm.")
        print("Ctrl-C aborts and lands.\n")
        input("Press Enter to start: ")

        commands, samples = fly(scf, args.thrust, base_pitch, args.lean,
                                args.lean_time)
        if len(samples) < 4:
            print("\nToo little telemetry to judge. Is the log task wedged?")
            return

        lag, response = best_fit(commands, samples)
        expected = 2 * args.lean

        print(f"\n  telemetry lag  {lag * 1000:.0f} ms")
        print(f"  response       {response:+.2f} deg of a possible "
              f"{expected:.0f} ({abs(response) / expected * 100:.0f}%)\n")

        if abs(response) < expected * TRACKING_FRACTION:
            print("The attitude does not follow the command, so the drone is\n"
                  "NOT flying -- the ground is holding it.\n")
            print(f"Raise the thrust and retry:\n"
                  f"    uv run flightcheck.py --thrust {args.thrust + 3000}\n")
            print("If it never lifts, the battery is too flat -- charge it.")
            return

        print("The attitude follows the command, so the drone is flying.\n")
        # Negative response means a positive pitch argument drives the estimate
        # down, which is what cflib's -pitch on the wire produces.
        direction = "NEGATIVE" if response < 0 else "POSITIVE"
        print(f"A positive pitch trim drives the estimate {direction}.")
        print(f"So holding the drone level needs pitch trim {suggested:+.1f}, "
              f"cancelling the {resting:+.2f} deg resting bias.\n")
        print(f"    uv run hoptest.py --thrust {args.thrust} --reset-trim "
              f"--pitch-trim {suggested:+.1f}")


def travel_from_bias(bias_deg: float, seconds: float = 2.6) -> float:
    """Metres an uncancelled resting bias covers over a whole flight."""
    accel = GRAVITY * math.tan(math.radians(abs(bias_deg)))
    return 0.5 * accel * seconds ** 2


if __name__ == "__main__":
    cfenv.run(main)
