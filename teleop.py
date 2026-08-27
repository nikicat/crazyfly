#!/usr/bin/env python3
"""Manual keyboard flight.

Sends raw roll/pitch/yaw/thrust setpoints, so it needs no positioning deck --
but it also means nothing holds the drone up except you. Fly over a clear
area, keep a hand near ESC, and start with small thrust.

  w / s     thrust up / down          arrows    roll and pitch
  a / d     yaw left / right          space     cut thrust to zero
  ESC / q   land and quit

  [ / ]     trim roll  left / right   ; / '     trim pitch fwd / back
  0         reset trim to zero

Trim is a constant offset added to every setpoint, to cancel a steady drift.
Adjust it in the air a notch at a time; it is saved to trim.json on exit and
reloaded next run. Trim only cancels a *steady* drift -- if the drone accelerates
away rather than holding a lean, fix the mechanical cause instead. Run
trimcheck.py first to tell those apart.

Releasing a key returns that axis to neutral: attitude inputs decay unless
they are being held, which keeps a dropped keypress from latching a roll.
"""
from __future__ import annotations

import argparse
import time

import cfenv
from flight import (
    DECAY,
    DT,
    MAX_ANGLE,
    MAX_THRUST,
    MAX_YAW_RATE,
    MIN_THRUST,
    THRUST_STEP,
    TRIM_FILE,
    TRIM_LIMIT,
    Interruptible,
    Keyboard,
    clamp,
    load_trim,
    save_trim,
    stop_motors,
)

TRIM_STEP = 0.2          # degrees per keypress, live in the air


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--roll-trim", type=float, default=None,
                   help="roll offset in degrees; negative leans left")
    p.add_argument("--pitch-trim", type=float, default=None,
                   help="pitch offset in degrees; negative leans forward")
    p.add_argument("--uri", default=None)
    args = p.parse_args()

    saved_roll, saved_pitch = load_trim()
    roll_trim = saved_roll if args.roll_trim is None else args.roll_trim
    pitch_trim = saved_pitch if args.pitch_trim is None else args.pitch_trim

    cfenv.init()
    uri = cfenv.resolve_uri(args.uri)

    print(f"Connecting to {uri} ...")
    with cfenv.connect(uri) as scf:
        cf = scf.cf
        print(__doc__)
        print(f"Trim: roll {roll_trim:+.1f} deg, pitch {pitch_trim:+.1f} deg"
              f"{' (loaded from trim.json)' if TRIM_FILE.exists() else ''}")
        print("Ready. Motors are armed once you raise thrust.\n")

        thrust = 0.0
        roll = pitch = yaw_rate = 0.0

        # A zero setpoint unlocks the commander; the firmware refuses thrust
        # until it has seen one.
        cf.commander.send_setpoint(0, 0, 0, 0)

        with Keyboard() as kb, Interruptible() as interrupt:
            try:
                while True:
                    loop_start = time.time()
                    events = kb.poll()
                    if "ESC" in events or "q" in events or interrupt.requested:
                        break
                    if " " in events:
                        thrust = 0.0

                    if kb.down("w"):
                        thrust = clamp(thrust + THRUST_STEP, MIN_THRUST, MAX_THRUST)
                    elif kb.down("s"):
                        thrust = thrust - THRUST_STEP
                        thrust = 0.0 if thrust < MIN_THRUST else thrust

                    if kb.down("left"):
                        roll = -MAX_ANGLE
                    elif kb.down("right"):
                        roll = MAX_ANGLE
                    else:
                        roll *= DECAY

                    if kb.down("up"):
                        pitch = -MAX_ANGLE
                    elif kb.down("down"):
                        pitch = MAX_ANGLE
                    else:
                        pitch *= DECAY

                    if kb.down("a"):
                        yaw_rate = -MAX_YAW_RATE
                    elif kb.down("d"):
                        yaw_rate = MAX_YAW_RATE
                    else:
                        yaw_rate *= DECAY

                    # Trim steps on discrete presses, not on hold, so a leaned-on
                    # key cannot run the offset away while you are flying.
                    for key in events:
                        if key == "[":
                            roll_trim = clamp(roll_trim - TRIM_STEP, -TRIM_LIMIT, TRIM_LIMIT)
                        elif key == "]":
                            roll_trim = clamp(roll_trim + TRIM_STEP, -TRIM_LIMIT, TRIM_LIMIT)
                        elif key == ";":
                            pitch_trim = clamp(pitch_trim - TRIM_STEP, -TRIM_LIMIT, TRIM_LIMIT)
                        elif key == "'":
                            pitch_trim = clamp(pitch_trim + TRIM_STEP, -TRIM_LIMIT, TRIM_LIMIT)
                        elif key == "0":
                            roll_trim = pitch_trim = 0.0

                    cf.commander.send_setpoint(roll + roll_trim, pitch + pitch_trim,
                                               yaw_rate, int(thrust))

                    bar = "#" * int(20 * thrust / MAX_THRUST)
                    print(f"\r thrust {int(thrust):>6} |{bar:<20}| "
                          f"roll {roll:>+6.1f} pitch {pitch:>+6.1f} yaw {yaw_rate:>+6.1f} "
                          f"| trim r{roll_trim:>+5.1f} p{pitch_trim:>+5.1f} ",
                          end="", flush=True)

                    time.sleep(max(0.0, DT - (time.time() - loop_start)))
            finally:
                # Ramp down rather than cutting instantly. Keep the trim applied
                # while there is still thrust: the drone is flying during this
                # descent and would drift without it. stop_motors then finishes
                # the job, and ignores Ctrl-C so it cannot be left half done.
                try:
                    while thrust > MIN_THRUST:
                        thrust = max(MIN_THRUST, thrust - THRUST_STEP)
                        cf.commander.send_setpoint(roll_trim, pitch_trim, 0,
                                                   int(thrust))
                        time.sleep(DT)
                except KeyboardInterrupt:
                    pass        # second Ctrl-C: skip the gentle descent
                stop_motors(cf, from_thrust=thrust, step=THRUST_STEP, dt=DT)
        print("\nLanded, motors stopped.")
        if (roll_trim, pitch_trim) != (saved_roll, saved_pitch):
            save_trim(roll_trim, pitch_trim)
            print(f"Trim saved to {TRIM_FILE.name}: "
                  f"roll {roll_trim:+.1f}, pitch {pitch_trim:+.1f}")


if __name__ == "__main__":
    cfenv.run(main)
