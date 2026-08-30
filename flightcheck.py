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

import math
import statistics

import cfenv
from flight import (
    DT,
    MIN_THRUST,
    Interruptible,
    Trim,
    load_trim,
    phase_level,
    stop_motors,
    ticks,
)
from signals import LAG_STEP, MAX_LAG, MIN_PAIRS, best_fit

GRAVITY = 9.81
RAMP_SECONDS = 0.8
PROBE_PITCH = 3.0        # degrees of deliberate lean to command
LEAN_SECONDS = 0.4       # each lean; the next one cancels it
LEAN_CYCLES = 3          # repeat, so there is enough signal to fit a lag to

TRACKING_GAIN = 0.4      # |gain| above this counts as following the command
MIN_CORRELATION = 0.5    # below this the fit is noise, whatever the gain says
IMPLAUSIBLE_GAIN = 2.0   # above this the fit is wrong, not the drone

VARIABLES = {"stabilizer.pitch": "float", "stabilizer.roll": "float"}


def lean_accel(lean_deg: float) -> float:
    """Horizontal acceleration, m/s^2, of a drone holding this lean."""
    return GRAVITY * math.tan(math.radians(abs(lean_deg)))


def travel_estimate(lean_deg: float, lean_seconds: float) -> float:
    """Metres covered over a lean-then-counter-lean pair.

    Accelerating at a for t covers a*t^2/2 and reaches speed a*t; the opposite
    lean brings that back to a standstill over another a*t^2/2, so the total is
    a*t^2 and it ends stationary rather than coasting into a wall.
    """
    return lean_accel(lean_deg) * lean_seconds ** 2


def travel_from_bias(bias_deg: float, seconds: float = 2.6) -> float:
    """Metres an uncancelled resting bias covers over a whole flight."""
    return 0.5 * lean_accel(bias_deg) * seconds ** 2


def fly(scf, thrust: int, base_pitch: float, lean: float, lean_seconds: float,
        base_roll: float = 0.0, cycles: int = LEAN_CYCLES
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

    # Repeat the pair: one cycle is too little signal to fit a lag to, and
    # each cycle cancels its own velocity so travel stays bounded.
    phases = [("up", RAMP_SECONDS, 0.0)]
    phases += [("plus", lean_seconds, lean), ("minus", lean_seconds, -lean)] * cycles
    phases.append(("down", RAMP_SECONDS, 0.0))

    with cfenv.record_log(scf, VARIABLES) as raw, Interruptible() as interrupt:
        try:
            cf.commander.send_setpoint(0, 0, 0, 0)
            for phase, duration, offset in phases:
                for frac, now in ticks(duration):
                    if interrupt.requested:
                        raise KeyboardInterrupt
                    level = phase_level(phase, thrust, frac)
                    cf.commander.send_setpoint(base_roll, base_pitch + offset, 0, int(level))
                    commands.append((now, offset))
                    # Drain whatever telemetry has arrived, stamped with now.
                    while len(samples) < len(raw):
                        samples.append((now, raw[len(samples)]["stabilizer.pitch"]))
        finally:
            stop_motors(cf, from_thrust=level, dt=DT)

    return commands, samples


def warn_about_bias(base: Trim, resting: Trim, suggested: Trim) -> None:
    """Say what flying at `base` trim will do about the measured resting bias."""
    if base.pitch == 0.0:
        print(f"\nFlying at zero pitch trim, so the {resting.pitch:+.2f} deg bias\n"
              f"becomes a real lean for the whole flight -- expect roughly\n"
              f"{travel_from_bias(resting.pitch):.1f} m of drift on top of the probe.\n"
              f"Pass --pitch-trim {suggested.pitch:+.1f} to cancel it.")
        return
    print(f"Flying at pitch trim {base.pitch:+.1f}, roll trim {base.roll:+.1f} deg.")
    if base.roll == 0.0 and abs(resting.roll) > 0.3:
        print(f"  Roll trim is zero while the roll bias is "
              f"{resting.roll:+.2f} deg, so expect sideways drift.\n"
              f"  Pass --roll-trim {suggested.roll:+.1f} to cancel it.")


def judge(fit: tuple[float, float, float], sample_count: int, thrust: int,
          resting_pitch: float, suggested: Trim) -> None:
    """Read the fit: not flying, unreliable, or flying -- and which way trim leans it."""
    lag, gain, correlation = fit
    print(f"\n  telemetry lag  {lag * 1000:.0f} ms")
    print(f"  correlation    {correlation:+.2f}")
    print(f"  gain           {gain:+.2f} deg per deg commanded ({abs(gain) * 100:.0f}%)")
    print(f"  samples        {sample_count}\n")

    if abs(correlation) < MIN_CORRELATION:
        print("The measured attitude does not correlate with what was\n"
              "commanded, so this tells us nothing either way. Usually too\n"
              "few samples got through. Power-cycle the drone and retry;\n"
              "raising --lean-time gives the fit more to work with.")
        return

    if abs(gain) > IMPLAUSIBLE_GAIN:
        print(f"A gain of {gain:+.2f} is not physically possible -- the drone\n"
              "cannot lean more than it was told to. Treat this run as\n"
              "unreliable rather than as a measurement, and retry.")
        return

    if lag >= MAX_LAG - LAG_STEP:
        print(f"The best fit sits at the {MAX_LAG * 1000:.0f} ms edge of the\n"
              "search, so the real lag may be larger and this fit suspect.")

    if abs(gain) < TRACKING_GAIN:
        print("The attitude barely follows the command, so the drone is\n"
              "NOT flying freely -- the ground is still holding it.\n")
        print(f"Raise the thrust and retry:\n"
              f"    uv run flightcheck.py --thrust {thrust + 3000}\n")
        print("If it never lifts, the battery is too flat -- charge it.")
        return

    print("The attitude follows the command, so the drone is flying.\n")
    # A negative gain is the expected one: cflib transmits -pitch, so a
    # positive pitch argument drives the estimate down.
    direction = "NEGATIVE" if gain < 0 else "POSITIVE"
    print(f"A positive pitch trim drives the estimate {direction}, gain {gain:+.2f}.")
    print(f"Holding the drone level needs pitch trim {suggested.pitch:+.1f} to "
          f"cancel the {resting_pitch:+.2f} deg resting bias.\n")
    print(f"    uv run hoptest.py --thrust {thrust} --reset-trim "
          f"--pitch-trim {suggested.pitch:+.1f} --roll-trim {suggested.roll:+.1f}")


def run(
    thrust: int = 42000,
    lean: float = PROBE_PITCH,
    lean_time: float = LEAN_SECONDS,
    pitch_trim: float | None = None,
    roll_trim: float | None = None,
    uri: str | None = None,
) -> None:
    """Confirm the drone is airborne before trusting a trim result."""

    base = load_trim().override(roll_trim, pitch_trim)

    with cfenv.session(uri) as scf:
        vbat = cfenv.vbat(scf)
        print(f"Connected. Battery {vbat:.2f} V")

        rest = cfenv.sample_series(scf, VARIABLES, count=10)
        resting = Trim(statistics.fmean(rest["stabilizer.roll"]),
                       statistics.fmean(rest["stabilizer.pitch"]))
        print(f"Resting on the ground: pitch {resting.pitch:+.2f}, "
              f"roll {resting.roll:+.2f} deg")

        # Roll passes through unchanged, so its trim is the bias as measured;
        # pitch is negated on the wire, so its trim is the negated bias.
        suggested = Trim(resting.roll, -resting.pitch)
        warn_about_bias(base, resting, suggested)

        travel = travel_estimate(lean, lean_time) * LEAN_CYCLES
        print(f"\nOne hop at thrust {thrust}: {LEAN_CYCLES} cycles of "
              f"{lean:.0f} deg for {lean_time:.1f}s each way, "
              f"about {travel * 100:.0f} cm.")
        print("Ctrl-C aborts and lands.\n")
        input("Press Enter to start: ")

        commands, samples = fly(scf, thrust, base.pitch, lean, lean_time, base_roll=base.roll)
        if len(samples) < MIN_PAIRS:
            print(f"\nOnly {len(samples)} telemetry samples arrived, too few to\n"
                  "judge. Is the log task wedged? Power-cycle and retry.")
            return

        judge(best_fit(commands, samples), len(samples), thrust, resting.pitch, suggested)


if __name__ == "__main__":
    cfenv.cli(run)
