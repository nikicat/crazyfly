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
    QUIT_KEYS,
    THRUST_STEP,
    TRIM_FILE,
    VBAT_CRITICAL,
    Gamepad,
    Interruptible,
    Keyboard,
    Trim,
    clamp,
    heading,
    load_mag_offset,
    load_trim,
    save_trim,
    stop_motors,
)

TRIM_STEP = 0.2          # degrees per keypress, live in the air
TRIM_KEYS = {"[": ("roll", -1), "]": ("roll", +1), ";": ("pitch", -1), "'": ("pitch", +1)}

KEYBOARD_HELP = """\
  w / s     thrust up / down     arrows   roll and pitch     a / d   yaw
  space     cut thrust           [ ] ; '  trim roll / pitch  0       reset trim
  h         height hold on/off (then w / s climb / sink)    ESC / q  land and quit
  f         roll flip -- battery above flip.min_vbat (config.json), two metres of air"""
GAMEPAD_HELP = """\
  left stick   thrust (centre = off)   right stick   roll and pitch    LT / RT  yaw
  B            cut thrust              D-pad         trim roll / pitch  View     reset trim
  A            height hold on/off (then left stick climbs / sinks)   Menu / q  land and quit
  Y            roll flip -- battery above flip.min_vbat (config.json), two metres of air"""

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


class Session:
    """One connection's worth of flying: the state the loop mutates, and its steps.

    A new one is made each time the drone is (re)connected; trim carries over
    from the previous one. fly() runs the loop and always lands on the way out.
    """

    def __init__(self, scf, trim: Trim, mag_offset, gamepad: bool) -> None:
        self.scf = scf
        self.cf = scf.cf
        self.trim = trim
        self.mag_offset = mag_offset
        self.gamepad = gamepad
        self.has_hold = "flightmode" in self.cf.param.toc.toc
        self.has_mag = mag_offset is not None and "mag" in self.cf.log.toc.toc

        self.thrust = 0.0
        self.roll = self.pitch = self.yaw_rate = self.climb = 0.0
        self.hold = False
        self.link_lost = False
        self.battery: list[dict] = []          # the 2 Hz log, newest last

        self.flip: flipping.Flip | None = None
        self._flip_log = None                  # the gyro recorder, open during a flip
        self._gyro: list[dict] = []
        self._fed = 0                          # gyro samples already fed to the flip

    # --- firmware modes -------------------------------------------------------

    def set_hold(self, on: bool) -> None:
        self.cf.param.set_value(ALTHOLD, "1" if on else "0")
        self.hold = on

    def set_rate_mode(self, on: bool) -> None:
        for axis in ("stabModeRoll", "stabModePitch"):
            self.cf.param.set_value(f"flightmode.{axis}", "0" if on else "1")

    def prepare(self) -> None:
        """Clear modes left over from a previous session and unlock the commander."""
        # Height hold survives a link loss but not a reboot, and with it on
        # even a zero setpoint spins the motors at the firmware's thrustMin.
        # Clear it before the first setpoint goes out.
        if self.has_hold:
            self.set_hold(False)
            self.set_rate_mode(False)       # a session that died mid-flip leaves RATE set
            time.sleep(0.2)
        # A zero setpoint unlocks the commander; the firmware refuses thrust
        # until it has seen one.
        self.cf.commander.send_setpoint(0, 0, 0, 0)

    def log_variables(self) -> dict[str, str]:
        """Battery at 2 Hz, plus height and the compass where the firmware has them."""
        variables = {"pm.vbat": "float"}
        if self.has_hold:
            variables[Z_LOG] = "float"
        if self.has_mag:
            variables.update(MAG_LOGS)      # 24 of the packet's 26 bytes, all in
        return variables

    @property
    def vbat(self) -> float | None:
        return self.battery[-1]["pm.vbat"] if self.battery else None

    # --- one-shot keys --------------------------------------------------------

    def toggle_hold(self) -> None:
        if self.hold:
            self.set_hold(False)
        elif not self.has_hold:
            print("\nThis firmware has no height hold.", flush=True)
        elif self.thrust > MIN_THRUST:
            # The thrust you hover at is the best hover estimate there is,
            # battery sag included; the firmware's I term trims the rest.
            self.cf.param.set_value(THRUST_BASE, str(int(self.thrust)))
            self.set_hold(True)
        else:
            print("\nTake off first, then hold.", flush=True)

    def start_flip(self) -> None:
        vbat = self.vbat or 0.0
        if not self.has_hold:
            print("\nThis firmware has no rate mode; no flip.", flush=True)
        elif self.thrust <= MIN_THRUST:
            print("\nTake off first, then flip.", flush=True)
        elif vbat < flipping.MIN_VBAT:
            print(f"\nBattery {vbat:.2f} V is under {flipping.MIN_VBAT} V; "
                  "no margin to catch a flip.", flush=True)
        else:
            if self.hold:
                self.set_hold(False)
            self._flip_log = cfenv.record_log(self.scf, {"gyro.x": "float"}, period_ms=10)
            self._gyro = self._flip_log.__enter__()
            self._fed = 0
            self.flip = flipping.Flip(self.set_rate_mode, time.time())

    def end_flip(self) -> None:
        if self._flip_log is not None:
            self._flip_log.__exit__(None, None, None)
        self.set_rate_mode(False)
        self.flip = self._flip_log = None

    def cut(self) -> None:
        if self.hold:
            self.set_hold(False)
        if self.flip is not None:
            self.end_flip()
        self.thrust = 0.0

    def handle_keys(self, events: list[str]) -> None:
        if "h" in events:
            self.toggle_hold()
        if "f" in events and self.flip is None:
            self.start_flip()
        if " " in events:
            self.cut()
        # Trim steps on discrete presses, not on hold, so a leaned-on key
        # cannot run the offset away while you are flying.
        for key in events:
            if key in TRIM_KEYS:
                axis, sign = TRIM_KEYS[key]
                self.trim = self.trim.nudge(axis, sign * TRIM_STEP)
            elif key == "0":
                self.trim = Trim()

    # --- sticks ---------------------------------------------------------------

    def read_sticks(self, inp) -> None:
        if self.gamepad:
            self.read_gamepad(inp)
        else:
            self.read_keyboard(inp)
        # The firmware ignores thrust at or below MIN_THRUST; below it means
        # motors off on either input.
        self.thrust = 0.0 if self.thrust < MIN_THRUST else self.thrust

    def read_gamepad(self, inp: Gamepad) -> None:
        # Sticks spring back on their own, so no decay. The left stick is a
        # throttle: centre is motors off, thrust grows with how far up it is
        # pushed. It follows the stick up at once but comes down no faster
        # than the landing ramp, so letting go is a descent rather than a drop.
        self.climb = inp.axis("thrust")
        if not self.hold:               # holding: manual thrust waits for release
            target = MAX_THRUST * max(0.0, self.climb)
            self.thrust = max(target, self.thrust - THRUST_STEP)
        self.roll = MAX_ANGLE * inp.axis("roll")
        self.pitch = MAX_ANGLE * inp.axis("pitch")
        self.yaw_rate = MAX_YAW_RATE * (inp.trigger("yaw_right") - inp.trigger("yaw_left"))

    def read_keyboard(self, inp: Keyboard) -> None:
        self.climb = KEY_CLIMB if inp.down("w") else -KEY_CLIMB if inp.down("s") else 0.0
        if self.hold:
            pass                        # manual thrust waits for release
        elif inp.down("w"):
            self.thrust = clamp(self.thrust + THRUST_STEP, MIN_THRUST, MAX_THRUST)
        elif inp.down("s"):
            self.thrust -= THRUST_STEP
        self.roll = held_axis(inp, "left", "right", self.roll, MAX_ANGLE)
        # Positive pitch drops the front motor and flies forward, measured
        # with motorcheck.py, so up is positive. It was the other way round,
        # which flew the drone backwards when you pressed up.
        self.pitch = held_axis(inp, "down", "up", self.pitch, MAX_ANGLE)
        self.yaw_rate = held_axis(inp, "a", "d", self.yaw_rate, MAX_YAW_RATE)

    # --- output ---------------------------------------------------------------

    def send(self) -> None:
        """One setpoint: the sticks plus trim, or whatever a flip in progress wants."""
        word = HOLD_CENTRE + HOLD_SPAN * self.climb if self.hold else self.thrust
        trim = self.trim
        if self.flip is not None:
            self.flip.feed([s["gyro.x"] for s in self._gyro[self._fed:]], 0.01)
            self._fed = len(self._gyro)
            command = self.flip.tick(time.time())
            if command is None:
                self.end_flip()         # the stick has thrust again next tick
            else:
                self.roll, self.pitch, self.yaw_rate, word = command
                trim = Trim()
        self.cf.commander.send_setpoint(self.roll + trim.roll, self.pitch + trim.pitch,
                                        self.yaw_rate, int(word))

    def status(self) -> str:
        bar = "#" * int(20 * self.thrust / MAX_THRUST)
        vbat = self.vbat
        bat = "?" if vbat is None else f"{vbat:.2f}V{' LOW' if vbat < VBAT_CRITICAL else ''}"
        z = hdg = ""
        if self.has_hold and self.battery:
            z = f"z {self.battery[-1][Z_LOG] - self.battery[0][Z_LOG]:+.2f}m"
        if self.has_mag and self.battery:
            last = self.battery[-1]
            degrees = heading(last["mag.x"], last["mag.y"], last["mag.z"],
                              last["stabilizer.roll"], last["stabilizer.pitch"],
                              self.mag_offset)
            hdg = f"hdg {degrees:3.0f}"
        mode = (f"FLIP {self.flip.phase} {self.flip.turned:4.0f}" if self.flip is not None
                else f"HOLD {self.climb:+.1f}" if self.hold else "")
        return (f"\r{int(self.thrust):>6} {bar:<20} "
                f"roll {self.roll:+5.1f} pitch {self.pitch:+5.1f} yaw {self.yaw_rate:+6.1f}  "
                f"trim {self.trim.roll:+.1f}/{self.trim.pitch:+.1f}  bat {bat:<9} "
                f"{z:<9} {hdg:<8} {mode:<9}")

    # --- the loop -------------------------------------------------------------

    def step(self, inp, interrupt: Interruptible) -> bool:
        """One tick: read input, send a setpoint, redraw. False when it is time to land."""
        try:
            events = inp.poll()
        except OSError as err:
            print(f"\n{err} -- landing.", flush=True)
            return False
        if QUIT_KEYS & set(events) or interrupt.requested:
            return False
        if not self.scf.is_link_open():
            # cflib gives up after ~100 lost acks: the drone powered off (flat
            # battery, idle timeout) or rebooted on USB. Nothing is left to
            # land, and setpoints into the void would just look like flying.
            print("\nLink lost -- the drone powered off or rebooted.", flush=True)
            self.thrust = 0.0
            self.link_lost = True
            return False
        self.handle_keys(events)
        self.read_sticks(inp)
        self.send()
        print(self.status(), end="", flush=True)
        return True

    def land(self) -> None:
        """Ramp down with the trim still applied, then stop the motors for good.

        The drone is flying during this descent and would drift without trim.
        stop_motors finishes the job and ignores Ctrl-C, so it cannot be left
        half done.
        """
        if self.flip is not None and not self.link_lost:
            self.end_flip()             # back to angle mode before the ramp
        if self.hold and not self.link_lost:
            self.set_hold(False)        # the ramp below sends thrust, not climb rate
            time.sleep(0.2)
        try:
            while self.thrust > MIN_THRUST:
                self.thrust = max(MIN_THRUST, self.thrust - THRUST_STEP)
                self.cf.commander.send_setpoint(self.trim.roll, self.trim.pitch, 0,
                                                int(self.thrust))
                time.sleep(DT)
        except KeyboardInterrupt:
            pass                        # second Ctrl-C: skip the gentle descent
        stop_motors(self.cf, from_thrust=self.thrust, step=THRUST_STEP, dt=DT)
        self.scf.close_link()

    def fly(self, inp, interrupt: Interruptible) -> None:
        """Run the loop until quit, Ctrl-C or a lost link; always lands on the way out."""
        self.prepare()
        # One small log packet every half second costs the 250K link nothing
        # next to 33 setpoints/s.
        with cfenv.record_log(self.scf, self.log_variables(), period_ms=500) as battery:
            self.battery = battery
            try:
                while True:
                    loop_start = time.time()
                    if not self.step(inp, interrupt):
                        break
                    time.sleep(max(0.0, DT - (time.time() - loop_start)))
            finally:
                self.land()


def run(
    roll_trim: float | None = None,
    pitch_trim: float | None = None,
    uri: str | None = None,
    gamepad: bool = False,
) -> None:
    """Manual keyboard or gamepad flight, with persistent trim."""

    saved = load_trim()
    trim = saved.override(roll_trim, pitch_trim)
    mag_offset = load_mag_offset()

    if gamepad and not Path(JS_DEVICE).exists():
        sys.exit(f"No gamepad at {JS_DEVICE}. Press the Xbox button to wake it, then retry.")

    cfenv.init()
    uri = cfenv.resolve_uri(uri)

    print(GAMEPAD_HELP if gamepad else KEYBOARD_HELP)
    print(f"Trim: roll {trim.roll:+.1f}, pitch {trim.pitch:+.1f} deg"
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
            scf.wait_for_params()
            session = Session(scf, trim, mag_offset, gamepad)
            session.fly(inp, interrupt)
            trim = session.trim
            if not session.link_lost:
                print("\nLanded, motors stopped.")
                break
            scf = wait_for_link(uri, inp, interrupt)

    if trim != saved:
        save_trim(*trim)
        print(f"Trim saved: roll {trim.roll:+.1f}, pitch {trim.pitch:+.1f}")


if __name__ == "__main__":
    cfenv.run(lambda: typer.run(run))
