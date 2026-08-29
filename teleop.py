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

With --gamepad (Xbox controller on /dev/input/js0):

  left stick    up = thrust, centre = motors off
  right stick   roll and pitch, proportional     B        cut thrust to zero
  LT / RT       yaw left / right, proportional
  D-pad         trim roll / pitch                View     reset trim
  Menu          land and quit
  keyboard      q / ESC / space still work

Trim is a constant offset added to every setpoint, to cancel a steady drift.
Adjust it in the air a notch at a time; it is saved to trim.json on exit and
reloaded next run. Trim only cancels a *steady* drift -- if the drone accelerates
away rather than holding a lean, fix the mechanical cause instead. Run
trimcheck.py first to tell those apart.

Releasing a key returns that axis to neutral: attitude inputs decay unless
they are being held, which keeps a dropped keypress from latching a roll.
"""
from __future__ import annotations

import time
from contextlib import nullcontext

import typer

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
    VBAT_CRITICAL,
    Gamepad,
    Interruptible,
    Keyboard,
    clamp,
    load_trim,
    save_trim,
    stop_motors,
)

TRIM_STEP = 0.2          # degrees per keypress, live in the air
QUIT_KEYS = {"q", "ESC"}  # the gamepad's Menu button arrives as "q"


def held_axis(inp: Keyboard, neg: str, pos: str, current: float, full: float) -> float:
    """Full deflection while a key is held, decaying back to neutral otherwise."""
    if inp.down(pos):
        return full
    if inp.down(neg):
        return -full
    return current * DECAY


def wait_for_link(uri: str, inp, interrupt: Interruptible):
    """Open the link, retrying until the drone answers; None if the user gives up.

    An absent drone fails an attempt in about a second and a half, so this
    stays responsive to q / Menu / Ctrl-C between attempts. Used both at start
    and after the drone drops off the air mid-flight.
    """
    print(f"Connecting to {uri} ...", flush=True)
    waiting = False
    while not interrupt.requested:
        if QUIT_KEYS & set(inp.poll()):
            break
        scf = cfenv.connect(uri)
        try:
            scf.open_link()
        except (cfenv.ConnectTimeout, cfenv.LinkLost):
            if not waiting:
                print("No answer -- switch the drone on. q, Menu or Ctrl-C gives up.",
                      flush=True)
                waiting = True
            time.sleep(1.0)
            continue
        print("Connected.", flush=True)
        return scf
    print("Gave up waiting.", flush=True)
    return None


def run(
    roll_trim: float | None = None,
    pitch_trim: float | None = None,
    uri: str | None = None,
    gamepad: bool = False,
) -> None:
    """Manual keyboard or gamepad flight, with persistent trim."""

    saved_roll, saved_pitch = load_trim()
    roll_trim = saved_roll if roll_trim is None else roll_trim
    pitch_trim = saved_pitch if pitch_trim is None else pitch_trim

    cfenv.init()
    uri = cfenv.resolve_uri(uri)

    print(__doc__)
    print(f"Trim: roll {roll_trim:+.1f} deg, pitch {pitch_trim:+.1f} deg"
          f"{' (loaded from trim.json)' if TRIM_FILE.exists() else ''}")

    # Input and Ctrl-C handling outlive any one connection: whether the drone
    # is off at start or drops off the air mid-flight, the loop below waits
    # for it rather than making you restart, and you can still quit meanwhile.
    # The keyboard stays live under the gamepad too, so q, ESC and space work
    # either way.
    with Keyboard() as kb, (Gamepad(keyboard=kb) if gamepad else nullcontext(kb)) as inp, \
            Interruptible() as interrupt:
        scf = wait_for_link(uri, inp, interrupt)
        while scf is not None:
            cf = scf.cf
            print("Ready. Motors are armed once you raise thrust.\n")

            thrust = 0.0
            roll = pitch = yaw_rate = 0.0
            link_lost = False

            # A zero setpoint unlocks the commander; the firmware refuses thrust
            # until it has seen one.
            cf.commander.send_setpoint(0, 0, 0, 0)

            # Battery voltage rides along at 2 Hz: one small log packet every
            # half second costs the 250K link nothing next to 33 setpoints/s.
            with cfenv.record_log(scf, {"pm.vbat": "float"}, period_ms=500) as battery:
                try:
                    while True:
                        loop_start = time.time()
                        try:
                            events = inp.poll()
                        except OSError as err:
                            print(f"\n{err} -- landing.", flush=True)
                            break
                        if QUIT_KEYS & set(events) or interrupt.requested:
                            break
                        if not scf.is_link_open():
                            # cflib gives up after ~100 lost acks: the drone
                            # powered off (flat battery, idle timeout) or
                            # rebooted on USB. Nothing is left to land, and
                            # setpoints into the void would just look like
                            # flying.
                            print("\nLink lost -- the drone powered off or rebooted.",
                                  flush=True)
                            thrust = 0.0
                            link_lost = True
                            break
                        if " " in events:
                            thrust = 0.0

                        if gamepad:
                            # Sticks spring back on their own, so no decay.
                            # The left stick is a throttle: centre is motors
                            # off, thrust grows with how far up it is pushed.
                            # It follows the stick up at once but comes down
                            # no faster than the landing ramp, so letting go
                            # is a descent rather than a drop.
                            target = MAX_THRUST * max(0.0, inp.axis("thrust"))
                            thrust = max(target, thrust - THRUST_STEP)
                            roll = MAX_ANGLE * inp.axis("roll")
                            pitch = MAX_ANGLE * inp.axis("pitch")
                            yaw_rate = MAX_YAW_RATE * (inp.trigger("yaw_right")
                                                       - inp.trigger("yaw_left"))
                        else:
                            if inp.down("w"):
                                thrust = clamp(thrust + THRUST_STEP, MIN_THRUST, MAX_THRUST)
                            elif inp.down("s"):
                                thrust -= THRUST_STEP
                            roll = held_axis(inp, "left", "right", roll, MAX_ANGLE)
                            # Positive pitch drops the front motor and flies
                            # forward, measured with motorcheck.py, so up is
                            # positive. It was the other way round, which
                            # flew the drone backwards when you pressed up.
                            pitch = held_axis(inp, "down", "up", pitch, MAX_ANGLE)
                            yaw_rate = held_axis(inp, "a", "d", yaw_rate, MAX_YAW_RATE)
                        # The firmware ignores thrust at or below MIN_THRUST;
                        # below it means motors off on either input.
                        thrust = 0.0 if thrust < MIN_THRUST else thrust

                        # Trim steps on discrete presses, not on hold, so a
                        # leaned-on key cannot run the offset away while you
                        # are flying.
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
                        vbat = battery[-1]["pm.vbat"] if battery else None
                        bat = ("?" if vbat is None else
                               f"{vbat:.2f}V{' LOW' if vbat < VBAT_CRITICAL else ''}")
                        print(f"\r thrust {int(thrust):>6} |{bar:<20}| "
                              f"roll {roll:>+6.1f} pitch {pitch:>+6.1f} yaw {yaw_rate:>+6.1f} "
                              f"| trim r{roll_trim:>+5.1f} p{pitch_trim:>+5.1f} "
                              f"| bat {bat:<9}",
                              end="", flush=True)

                        time.sleep(max(0.0, DT - (time.time() - loop_start)))
                finally:
                    # Ramp down rather than cutting instantly. Keep the trim
                    # applied while there is still thrust: the drone is flying
                    # during this descent and would drift without it.
                    # stop_motors then finishes the job, and ignores Ctrl-C so
                    # it cannot be left half done.
                    try:
                        while thrust > MIN_THRUST:
                            thrust = max(MIN_THRUST, thrust - THRUST_STEP)
                            cf.commander.send_setpoint(roll_trim, pitch_trim, 0,
                                                       int(thrust))
                            time.sleep(DT)
                    except KeyboardInterrupt:
                        pass        # second Ctrl-C: skip the gentle descent
                    stop_motors(cf, from_thrust=thrust, step=THRUST_STEP, dt=DT)
                    scf.close_link()

            if not link_lost:
                print("\nLanded, motors stopped.")
                break
            scf = wait_for_link(uri, inp, interrupt)

    if (roll_trim, pitch_trim) != (saved_roll, saved_pitch):
        save_trim(roll_trim, pitch_trim)
        print(f"Trim saved to {TRIM_FILE.name}: "
              f"roll {roll_trim:+.1f}, pitch {pitch_trim:+.1f}")


if __name__ == "__main__":
    cfenv.run(lambda: typer.run(run))
