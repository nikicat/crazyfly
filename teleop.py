#!/usr/bin/env python3
"""Manual keyboard or gamepad flight.

Sends raw roll/pitch/yaw/thrust setpoints, so it needs no positioning deck --
but it also means nothing holds the drone up except you. Fly over a clear
area, keep a hand near ESC, and start with small thrust. The controls are in
KEYBOARD_HELP and GAMEPAD_HELP below; the one in use is printed at start.

Trim is a constant offset added to every setpoint, to cancel a steady drift.
Adjust it in the air a notch at a time; it is saved to trim.json on exit and
reloaded next run. Trim only cancels a *steady* drift -- if the drone accelerates
away rather than holding a lean, fix the mechanical cause instead. Run
trimcheck.py first to tell those apart.

Releasing a key returns that axis to neutral: attitude inputs decay unless
they are being held, which keeps a dropped keypress from latching a roll.
"""
from __future__ import annotations

import sys
import time
from contextlib import nullcontext
from pathlib import Path

import typer

import cfenv
import flip as flipping
from flight import (
    DECAY,
    DT,
    JS_DEVICE,
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
    heading,
    load_mag_offset,
    load_trim,
    save_trim,
    stop_motors,
)

TRIM_STEP = 0.2          # degrees per keypress, live in the air

KEYBOARD_HELP = """\
  w / s     thrust up / down     arrows   roll and pitch     a / d   yaw
  space     cut thrust           [ ] ; '  trim roll / pitch  0       reset trim
  h         height hold on/off (then w / s climb / sink)    ESC / q  land and quit
  f         roll flip -- needs 3.9 V and two metres of air"""
GAMEPAD_HELP = """\
  left stick   thrust (centre = off)   right stick   roll and pitch    LT / RT  yaw
  B            cut thrust              D-pad         trim roll / pitch  View     reset trim
  A            height hold on/off (then left stick climbs / sinks)   Menu / q  land and quit
  Y            roll flip -- needs 3.9 V and two metres of air"""
QUIT_KEYS = {"q", "ESC"}  # the gamepad's Menu button arrives as "q"

# Height hold, firmware 2017.06: with flightmode.althold set the thrust word is
# a climb rate instead -- HOLD_CENTRE holds, full scale is 1 m/s either way --
# and the firmware makes thrust itself as vzPID * 1000 + posCtlPid.thrustBase.
ALTHOLD = "flightmode.althold"
THRUST_BASE = "posCtlPid.thrustBase"
HOLD_CENTRE = 32767
HOLD_SPAN = 32767
Z_LOG = "posEstimatorAlt.estimatedZ"
MAG_LOGS = {"mag.x": "float", "mag.y": "float", "mag.z": "float",
            "stabilizer.roll": "FP16", "stabilizer.pitch": "FP16"}
KEY_CLIMB = 0.5          # fraction of the full climb rate while w / s is held


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
    print(f"Connecting to {uri} ...", end="", flush=True)
    waiting = False
    while not interrupt.requested:
        if QUIT_KEYS & set(inp.poll()):
            break
        scf = cfenv.connect(uri, check_radio=not waiting)
        try:
            scf.open_link()
        except (cfenv.ConnectTimeout, cfenv.LinkLost):
            if not waiting:
                print(" no answer. Switch the drone on; q or Ctrl-C gives up.",
                      end="", flush=True)
                waiting = True
            time.sleep(1.0)
            continue
        print(" connected.", flush=True)
        return scf
    print(" gave up.", flush=True)
    return None


def run(
    roll_trim: float | None = None,
    pitch_trim: float | None = None,
    uri: str | None = None,
    gamepad: bool = False,
) -> None:
    """Manual keyboard or gamepad flight, with persistent trim."""

    saved_roll, saved_pitch = load_trim()
    mag_offset = load_mag_offset()
    roll_trim = saved_roll if roll_trim is None else roll_trim
    pitch_trim = saved_pitch if pitch_trim is None else pitch_trim

    if gamepad and not Path(JS_DEVICE).exists():
        sys.exit(f"No gamepad at {JS_DEVICE}. Press the Xbox button to wake it, then retry.")

    cfenv.init()
    uri = cfenv.resolve_uri(uri)

    print(GAMEPAD_HELP if gamepad else KEYBOARD_HELP)
    print(f"Trim: roll {roll_trim:+.1f}, pitch {pitch_trim:+.1f} deg"
          f"{' (trim.json)' if TRIM_FILE.exists() else ''}")

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
            scf.wait_for_params()
            has_hold = "flightmode" in cf.param.toc.toc
            has_mag = mag_offset is not None and "mag" in cf.log.toc.toc

            thrust = 0.0
            roll = pitch = yaw_rate = climb = 0.0
            link_lost = hold = False

            def set_hold(on: bool, cf=cf) -> None:      # cf bound per connection
                nonlocal hold
                cf.param.set_value(ALTHOLD, "1" if on else "0")
                hold = on

            def set_rate_mode(on: bool, cf=cf) -> None:
                for axis in ("stabModeRoll", "stabModePitch"):
                    cf.param.set_value(f"flightmode.{axis}", "0" if on else "1")

            flip = None
            flip_log = None
            fed = 0

            def end_flip() -> None:
                nonlocal flip, flip_log
                if flip_log is not None:
                    flip_log.__exit__(None, None, None)
                set_rate_mode(False)
                flip = flip_log = None

            # Height hold survives a link loss but not a reboot, and with it
            # on even a zero setpoint spins the motors at the firmware's
            # thrustMin. Clear it before the first setpoint goes out.
            if has_hold:
                set_hold(False)
                set_rate_mode(False)        # a session that died mid-flip leaves RATE set
                time.sleep(0.2)

            # A zero setpoint unlocks the commander; the firmware refuses thrust
            # until it has seen one.
            cf.commander.send_setpoint(0, 0, 0, 0)

            # Battery voltage rides along at 2 Hz: one small log packet every
            # half second costs the 250K link nothing next to 33 setpoints/s.
            variables = {"pm.vbat": "float"}
            if has_hold:
                variables[Z_LOG] = "float"
            if has_mag:
                variables.update(MAG_LOGS)      # 24 of the packet's 26 bytes, all in
            with cfenv.record_log(scf, variables, period_ms=500) as battery:
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
                        if "h" in events:
                            if hold:
                                set_hold(False)
                            elif not has_hold:
                                print("\nThis firmware has no height hold.", flush=True)
                            elif thrust > MIN_THRUST:
                                # The thrust you hover at is the best hover
                                # estimate there is, battery sag included; the
                                # firmware's I term trims the rest.
                                cf.param.set_value(THRUST_BASE, str(int(thrust)))
                                set_hold(True)
                            else:
                                print("\nTake off first, then hold.", flush=True)
                        if "f" in events and flip is None:
                            vbat = battery[-1]["pm.vbat"] if battery else 0.0
                            if not has_hold:
                                print("\nThis firmware has no rate mode; no flip.", flush=True)
                            elif thrust <= MIN_THRUST:
                                print("\nTake off first, then flip.", flush=True)
                            elif vbat < flipping.MIN_VBAT:
                                print(f"\nBattery {vbat:.2f} V is under {flipping.MIN_VBAT} V; "
                                      "no margin to catch a flip.", flush=True)
                            else:
                                if hold:
                                    set_hold(False)
                                flip_log = cfenv.record_log(scf, {"gyro.x": "float"}, period_ms=10)
                                gyro = flip_log.__enter__()
                                fed = 0
                                flip = flipping.Flip(set_rate_mode, time.time())
                        if " " in events:
                            if hold:
                                set_hold(False)
                            if flip is not None:
                                end_flip()
                            thrust = 0.0

                        if gamepad:
                            # Sticks spring back on their own, so no decay.
                            # The left stick is a throttle: centre is motors
                            # off, thrust grows with how far up it is pushed.
                            # It follows the stick up at once but comes down
                            # no faster than the landing ramp, so letting go
                            # is a descent rather than a drop.
                            climb = inp.axis("thrust")
                            if not hold:    # holding: manual thrust waits for release
                                target = MAX_THRUST * max(0.0, climb)
                                thrust = max(target, thrust - THRUST_STEP)
                            roll = MAX_ANGLE * inp.axis("roll")
                            pitch = MAX_ANGLE * inp.axis("pitch")
                            yaw_rate = MAX_YAW_RATE * (inp.trigger("yaw_right")
                                                       - inp.trigger("yaw_left"))
                        else:
                            climb = (KEY_CLIMB if inp.down("w")
                                     else -KEY_CLIMB if inp.down("s") else 0.0)
                            if hold:
                                pass        # manual thrust waits for release
                            elif inp.down("w"):
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

                        word = HOLD_CENTRE + HOLD_SPAN * climb if hold else thrust
                        trim_r, trim_p = roll_trim, pitch_trim
                        if flip is not None:
                            flip.feed([s["gyro.x"] for s in gyro[fed:]], 0.01)
                            fed = len(gyro)
                            command = flip.tick(time.time())
                            if command is None:
                                end_flip()          # the stick has thrust again next tick
                            else:
                                roll, pitch, yaw_rate, word = command
                                trim_r = trim_p = 0.0
                        cf.commander.send_setpoint(roll + trim_r, pitch + trim_p,
                                                   yaw_rate, int(word))

                        bar = "#" * int(20 * thrust / MAX_THRUST)
                        vbat = battery[-1]["pm.vbat"] if battery else None
                        bat = ("?" if vbat is None else
                               f"{vbat:.2f}V{' LOW' if vbat < VBAT_CRITICAL else ''}")
                        z = ""
                        if has_hold and battery:
                            z = f"z {battery[-1][Z_LOG] - battery[0][Z_LOG]:+.2f}m"
                        hdg = ""
                        if has_mag and battery:
                            last = battery[-1]
                            degrees = heading(last["mag.x"], last["mag.y"], last["mag.z"],
                                              last["stabilizer.roll"], last["stabilizer.pitch"],
                                              mag_offset)
                            hdg = f"hdg {degrees:3.0f}"
                        mode = (f"FLIP {flip.phase} {flip.turned:4.0f}" if flip is not None
                                else f"HOLD {climb:+.1f}" if hold else "")
                        print(f"\r{int(thrust):>6} {bar:<20} "
                              f"roll {roll:+5.1f} pitch {pitch:+5.1f} yaw {yaw_rate:+6.1f}  "
                              f"trim {roll_trim:+.1f}/{pitch_trim:+.1f}  bat {bat:<9} "
                              f"{z:<9} {hdg:<8} {mode:<9}",
                              end="", flush=True)

                        time.sleep(max(0.0, DT - (time.time() - loop_start)))
                finally:
                    # Ramp down rather than cutting instantly. Keep the trim
                    # applied while there is still thrust: the drone is flying
                    # during this descent and would drift without it.
                    # stop_motors then finishes the job, and ignores Ctrl-C so
                    # it cannot be left half done.
                    if flip is not None and not link_lost:
                        end_flip()          # back to angle mode before the ramp
                    if hold and not link_lost:
                        set_hold(False)     # the ramp below sends thrust, not climb rate
                        time.sleep(0.2)
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
        print(f"Trim saved: roll {roll_trim:+.1f}, pitch {pitch_trim:+.1f}")


if __name__ == "__main__":
    cfenv.run(lambda: typer.run(run))
