#!/usr/bin/env python3
"""Manual keyboard or gamepad flight.

Sends roll/pitch/yaw setpoints and, by default, flies height by reference: the
thrust stick moves a target height and the firmware's barometric hold is
steered at it, so taking off is raising the target off the ground and landing
is lowering it back. h / A switches to raw thrust on the stick, where nothing
holds the drone up but you. Either way fly over a clear area and keep a hand
near ESC. The controls are in KEYBOARD_HELP and GAMEPAD_HELP below; the one in
use is printed at start.

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
import flightlog
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
    charge,
    clamp,
    heading,
    load_hover,
    load_mag_offset,
    load_trim,
    rest_voltage,
    save_hover,
    save_trim,
    stop_motors,
)

TRIM_STEP = 0.2          # degrees per keypress, live in the air
TRIM_KEYS = {"[": ("roll", -1), "]": ("roll", +1), ";": ("pitch", -1), "'": ("pitch", +1)}

KEYBOARD_HELP = """\
  w / s     height up / down     arrows   roll and pitch     a / d   yaw
  space     cut thrust           [ ] ; '  trim roll / pitch  0       reset trim
  h         height mode off / on (off: w / s is raw thrust)  ESC / q  land and quit
  f         roll flip -- battery above flip.min_vbat (config.json), two metres of air
  Raise the height to take off; lower it onto the ground to land."""
GAMEPAD_HELP = """\
  left stick   height up / down        right stick   roll and pitch    LT / RT  yaw
  B            cut thrust              D-pad         trim roll / pitch  View     reset trim
  A            height mode off / on (off: left stick is a throttle)   Menu / q  land and quit
  Y            roll flip -- battery above flip.min_vbat (config.json), two metres of air
  Raise the height to take off; lower it onto the ground to land."""

# Height, firmware 2017.06: with flightmode.althold set the thrust word is a
# climb rate -- HOLD_CENTRE holds, full scale 1 m/s -- and the firmware makes
# thrust itself as vzPID * 1000 + posCtlPid.thrustBase, never below its
# thrustMin. Teleop keeps a height reference the stick moves, as it does for
# heading, and steers the climb rate at it. Raising it off the ground takes
# off. It has landed once it is asked to sink well below where it is while
# the firmware has sat at thrustMin for a while: the floor is holding the
# drone up and the vertical-speed loop's I term has wound thrust down.
ALTHOLD = "flightmode.althold"
THRUST_BASE = "posCtlPid.thrustBase"
HOLD_CENTRE = 32767
HOLD_SPAN = 32767
Z_LOG = "posEstimatorAlt.estimatedZ"
FW_THRUST_LOG = "stabilizer.thrust"
FW_THRUST_MIN = 20000    # posCtlPid.thrustMin, the firmware default
HOVER_DEFAULT = 42000    # thrustBase until a flight has taught hover.json better
Z_RATE = 1.0             # m/s the reference moves at, full stick
Z_KP = 1.0               # climb rate per metre of height error
Z_MAX_RATE = 0.5         # m/s, the most the loop asks for
TAKEOFF_ABOVE = 0.10     # reference this far above the ground lifts off
LAND_BELOW = 0.15        # reference this far below the drone, with the
TOUCHDOWN_S = 1.0        # firmware at thrustMin for this long: on the ground
HOVER_SAMPLES = 20       # of the firmware's thrust while still, to trust a mean
MAG_LOGS = {"mag.x": "float", "mag.y": "float", "mag.z": "float",
            "stabilizer.roll": "FP16", "stabilizer.pitch": "FP16"}
KEY_CLIMB = 0.5          # fraction of the full climb rate while w / s is held

# The flight recording (flightlog.py): one row of what went out and the state
# behind it every tick, and a second log block for how the drone answered.
# ponytail: 20 Hz is untested on the air. A flip streams gyro.x at 100 Hz for
# a moment, so the acks should carry it beside the 10 Hz block; if a recording
# shows fewer than 20 dyn rows a second, the link is dropping them -- use 100 ms.
DYN_LOGS = {"stabilizer.yaw": "FP16", "gyro.x": "FP16", "gyro.y": "FP16", "gyro.z": "FP16",
            "motor.m1": "uint16_t", "motor.m2": "uint16_t",
            "motor.m3": "uint16_t", "motor.m4": "uint16_t",
            "acc.z": "FP16", "posEstimatorAlt.estVZ": "FP16"}     # 20 of the packet's 26 bytes
DYN_PERIOD_MS = 50
CMD_FIELDS = ["roll", "pitch", "yaw_rate", "thrust", "trim_roll", "trim_pitch", "climb",
              "ref_z", "ref_hdg", "hdg", "hold", "height_mode", "flip"]
COLUMNS = [*CMD_FIELDS, "pm.vbat", Z_LOG, FW_THRUST_LOG, *MAG_LOGS, *DYN_LOGS]

# Heading hold. The firmware already turns the yaw stick into a held angle
# (controller_pid.c integrates the rate into a yaw setpoint the attitude PID
# tracks), but on the gyro alone, which wanders ~6 deg/min. With the compass
# calibrated, teleop trims that reference against hdg while the stick is
# centred. A positive stick rate lowers stabilizer.yaw, and hdg runs the other
# way to it on a level turn (mag.csv), so positive yaw_rate raises hdg. If the
# drone turns steadily *away* from where it pointed, flip HDG_SIGN.
HDG_SIGN = +1
HDG_KP = 1.0             # deg/s of yaw per degree of heading error
HDG_MAX_RATE = 10.0      # a bent compass reading can only turn it this fast


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

    def __init__(self, scf, trim: Trim, mag_offset, gamepad: bool,
                 hover: float | None = None,
                 recorder: flightlog.Recorder | None = None) -> None:
        self.scf = scf
        self.cf = scf.cf
        self.trim = trim
        self.mag_offset = mag_offset
        self.gamepad = gamepad
        self.recorder = recorder
        self.has_hold = "flightmode" in self.cf.param.toc.toc
        self.has_mag = mag_offset is not None and "mag" in self.cf.log.toc.toc

        self.thrust = 0.0
        self.roll = self.pitch = self.yaw_rate = self.climb = 0.0
        self.hold = False                      # flightmode.althold is set
        self.height_mode = self.has_hold       # the stick moves a height reference
        self.ref_z: float | None = None        # that reference, in the estimator's frame
        self.hover = HOVER_DEFAULT if hover is None else hover   # thrustBase at takeoff
        self.hover_samples: list[float] = []   # the firmware's thrust while holding still
        self.lifting_at = 0.0                  # when the firmware last pushed above thrustMin
        self.ref_hdg: float | None = None      # compass heading being held, if any
        self.link_lost = False
        self.battery: list[dict] = []          # the 10 Hz log, newest last
        self.dyn: list[dict] = []              # the recording's 20 Hz log
        self._taken = {"log": 0, "dyn": 0}     # samples of each already recorded

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

    def engage_hold(self, base: float, now: float | None = None) -> None:
        """Hand thrust to the firmware's vertical-speed loop, around `base`."""
        self.cf.param.set_value(THRUST_BASE, str(int(base)))
        self.set_hold(True)
        # Parameter writes are fire-and-forget; give them the air before the
        # next setpoint, or a climb-rate word lands as raw thrust for a tick.
        time.sleep(0.1)
        self.lifting_at = time.time() if now is None else now

    def release_hold(self) -> None:
        """Take thrust back at whatever the firmware was flying with, so nothing drops."""
        if self.hold:
            self.thrust = self.fw_thrust or self.thrust
            self.set_hold(False)

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
        """Battery at 10 Hz, plus height, thrust and the compass where the firmware has them."""
        variables = {"pm.vbat": "float"}
        if self.has_hold:
            variables[Z_LOG] = "float"
            variables[FW_THRUST_LOG] = "uint16_t"
        if self.has_mag:
            variables.update(MAG_LOGS)      # fills the packet's 26 bytes exactly
        return variables

    def dyn_variables(self) -> dict[str, str]:
        """DYN_LOGS, less whatever groups this firmware does not have."""
        groups = self.cf.log.toc.toc
        return {name: ctype for name, ctype in DYN_LOGS.items() if name.split(".")[0] in groups}

    @property
    def vbat(self) -> float | None:
        return self.battery[-1]["pm.vbat"] if self.battery else None

    @property
    def z(self) -> float | None:
        """Height from the latest sample, in the estimator's frame; None without a barometer."""
        return self.battery[-1][Z_LOG] if self.has_hold and self.battery else None

    @property
    def fw_thrust(self) -> float | None:
        """What the firmware is actually driving the motors with, from the latest sample."""
        return self.battery[-1][FW_THRUST_LOG] if self.has_hold and self.battery else None

    @property
    def airborne(self) -> bool:
        return self.hold or self.thrust > 0

    def learned_hover(self) -> float | None:
        """Mean firmware thrust while holding still this flight, given enough of it."""
        if len(self.hover_samples) < HOVER_SAMPLES:
            return None
        return sum(self.hover_samples) / len(self.hover_samples)

    @property
    def hdg(self) -> float | None:
        """Compass heading from the latest sample; None without a calibrated compass."""
        if not (self.has_mag and self.battery):
            return None
        last = self.battery[-1]
        return heading(last["mag.x"], last["mag.y"], last["mag.z"],
                       last["stabilizer.roll"], last["stabilizer.pitch"], self.mag_offset)

    # --- one-shot keys --------------------------------------------------------

    def toggle_height_mode(self) -> None:
        """h / A: height by reference (the default), or raw thrust on the stick."""
        if not self.has_hold:
            print("\nThis firmware has no height hold.", flush=True)
        elif self.height_mode:
            self.release_hold()
            self.height_mode = False
            self.ref_z = None
        else:
            self.height_mode = True
            if self.thrust > MIN_THRUST:
                # The thrust you hover at is the best hover estimate there is,
                # battery sag included; the firmware's I term trims the rest.
                self.engage_hold(self.thrust)

    def start_flip(self) -> None:
        vbat = self.vbat or 0.0
        if not self.has_hold:
            print("\nThis firmware has no rate mode; no flip.", flush=True)
        elif not self.airborne:
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
        if self.height_mode and self.ref_z is not None:
            self.engage_hold(self.hover)    # the height loop climbs back to the reference

    def cut(self) -> None:
        if self.flip is not None:
            self.end_flip()
        if self.hold:
            self.set_hold(False)
        self.thrust = 0.0

    def handle_keys(self, events: list[str]) -> None:
        if "h" in events:
            self.toggle_height_mode()
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
        # Sticks spring back on their own, so no decay. In height mode the
        # left stick moves the reference; otherwise it is a throttle: centre
        # is motors off, thrust grows with how far up it is pushed. It follows
        # the stick up at once but comes down no faster than the landing ramp,
        # so letting go is a descent rather than a drop.
        self.climb = inp.axis("thrust")
        if not (self.hold or self.height_mode):    # else the stick moves the height
            target = MAX_THRUST * max(0.0, self.climb)
            self.thrust = max(target, self.thrust - THRUST_STEP)
        self.roll = MAX_ANGLE * inp.axis("roll")
        self.pitch = MAX_ANGLE * inp.axis("pitch")
        self.yaw_rate = MAX_YAW_RATE * (inp.trigger("yaw_right") - inp.trigger("yaw_left"))

    def read_keyboard(self, inp: Keyboard) -> None:
        self.climb = KEY_CLIMB if inp.down("w") else -KEY_CLIMB if inp.down("s") else 0.0
        if self.hold or self.height_mode:
            pass                        # the keys move the height instead
        elif inp.down("w"):
            self.thrust = clamp(self.thrust + THRUST_STEP, MIN_THRUST, MAX_THRUST)
        elif inp.down("s"):
            self.thrust -= THRUST_STEP
        self.roll = held_axis(inp, "left", "right", self.roll, MAX_ANGLE)
        # Positive pitch drops the front motor and flies forward, measured
        # with motorcheck.py, so up is positive. It was the other way round,
        # which flew the drone backwards when you pressed up.
        self.pitch = held_axis(inp, "down", "up", self.pitch, MAX_ANGLE)
        # Yaw releases to an exact zero rather than decaying: the firmware
        # integrates every last deg/s into its heading, and the hold below
        # only engages once the stick is centred.
        self.yaw_rate = MAX_YAW_RATE * (inp.down("d") - inp.down("a"))

    def hold_heading(self) -> None:
        """Stick centred and airborne: trim yaw_rate so the compass heading stays put.

        The reference is wherever the heading was when the stick came back to
        centre. Touching the stick releases it; it is taken afresh on release.
        On the ground the stick turns the reference instead, so the drone can
        be aimed before takeoff and swings round to it, at HDG_MAX_RATE, once
        it is flying. Landing keeps it.
        """
        hdg = self.hdg
        if hdg is None or self.flip is not None:
            self.ref_hdg = None
            return
        if not self.airborne:
            if self.yaw_rate:
                aim = hdg if self.ref_hdg is None else self.ref_hdg
                self.ref_hdg = (aim + HDG_SIGN * self.yaw_rate * DT) % 360
            return
        if self.yaw_rate:
            self.ref_hdg = None
            return
        if self.ref_hdg is None:
            self.ref_hdg = hdg
        error = (self.ref_hdg - hdg + 180) % 360 - 180
        self.yaw_rate = HDG_SIGN * clamp(HDG_KP * error, -HDG_MAX_RATE, HDG_MAX_RATE)

    def hold_height(self, now: float | None = None) -> None:
        """Height mode: the stick moves ref_z, and the climb rate steers z to it.

        On the ground the motors are off and the reference rests wherever the
        barometer says the ground is, so pressure drift cannot lift it; push
        it TAKEOFF_ABOVE up and the hold engages around the hover thrust.
        Flying, the loop asks for a climb rate towards it, and samples the
        firmware's thrust while still for the next takeoff. Asking for well
        below the drone while the firmware sits at thrustMin means the floor is
        holding it up: hold off, motors off, reference back on the ground.
        """
        z = self.z
        if not self.height_mode or z is None or self.flip is not None:
            return
        now = time.time() if now is None else now
        if self.ref_z is None:
            self.ref_z = z
        self.ref_z += self.climb * Z_RATE * DT
        if not self.hold:
            self.thrust = 0.0
            if self.climb <= 0:
                self.ref_z = z          # resting on the ground, wherever the barometer puts it
            elif self.ref_z > z + TAKEOFF_ABOVE:
                self.engage_hold(self.hover, now)
            return
        error = self.ref_z - z
        self.climb = clamp(Z_KP * error, -Z_MAX_RATE, Z_MAX_RATE)
        thrust = self.fw_thrust or 0.0
        if thrust > FW_THRUST_MIN * 1.1:
            self.lifting_at = now
            if abs(error) < 0.1:
                self.hover_samples.append(thrust)
        if error < -LAND_BELOW and now - self.lifting_at > TOUCHDOWN_S:
            self.set_hold(False)
            self.thrust = 0.0
            self.ref_z = z
            self.hover = self.learned_hover() or self.hover

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
                self.end_flip()         # the height loop, or the stick, has thrust again
                word = HOLD_CENTRE if self.hold else self.thrust
            else:
                self.roll, self.pitch, self.yaw_rate, word = command
                trim = Trim()
        roll, pitch, thrust = self.roll + trim.roll, self.pitch + trim.pitch, int(word)
        self.cf.commander.send_setpoint(roll, pitch, self.yaw_rate, thrust)
        self.record(roll, pitch, self.yaw_rate, thrust)

    def record(self, roll: float, pitch: float, yaw_rate: float, thrust: int) -> None:
        """One cmd row: what just went out and the state behind it; then the log so far."""
        if self.recorder is None:
            return
        self.recorder.write("cmd", {
            "roll": roll, "pitch": pitch, "yaw_rate": yaw_rate, "thrust": thrust,
            "trim_roll": self.trim.roll, "trim_pitch": self.trim.pitch, "climb": self.climb,
            "ref_z": self.ref_z, "ref_hdg": self.ref_hdg, "hdg": self.hdg,
            "hold": self.hold, "height_mode": self.height_mode,
            "flip": None if self.flip is None else self.flip.phase})
        self.drain()

    def drain(self) -> None:
        """Record the log samples that have arrived since the last drain."""
        if self.recorder is None:
            return
        for src, samples in (("log", self.battery), ("dyn", self.dyn)):
            new = samples[self._taken[src]:]     # sliced first: the rx thread keeps appending
            self._taken[src] += len(new)
            for sample in new:
                self.recorder.write(src, sample)

    def status(self) -> str:
        thrust = (self.fw_thrust or 0.0) if self.hold else self.thrust
        bar = "#" * int(20 * thrust / MAX_THRUST)
        vbat = self.vbat
        if vbat is None:
            bat = "?"
        else:
            recent = [rest_voltage(s["pm.vbat"],                   # ~2 s of the 10 Hz log,
                                   bool(s.get(FW_THRUST_LOG, self.airborne)))
                      for s in self.battery[-20:]]                   # each sample by its own state
            pct = charge(sum(recent) / len(recent))
            bat = f"{vbat:.2f}V {pct:3.0f}%{' LOW' if vbat < VBAT_CRITICAL else ''}"
        z = hdg = ""
        if self.z is not None:
            ground = self.battery[0][Z_LOG]
            aim = "" if self.ref_z is None else f">{self.ref_z - ground:+.2f}"
            z = f"z {self.z - ground:+.2f}{aim}m"
        if self.hdg is not None:
            held = "" if self.ref_hdg is None else f">{self.ref_hdg:3.0f}"
            hdg = f"hdg {self.hdg:3.0f}{held}"
        mode = (f"FLIP {self.flip.phase} {self.flip.turned:4.0f}" if self.flip is not None
                else f"HOLD {self.climb:+.1f}" if self.hold
                else "HEIGHT" if self.height_mode else "")
        return (f"\r{int(thrust):>6} {bar:<20} "
                f"roll {self.roll:+5.1f} pitch {self.pitch:+5.1f} yaw {self.yaw_rate:+6.1f}  "
                f"trim {self.trim.roll:+.1f}/{self.trim.pitch:+.1f}  bat {bat:<14} "
                f"{z:<15} {hdg:<12} {mode:<9}")

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
        self.hold_heading()
        self.hold_height()
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
            self.release_hold()         # the ramp below sends thrust, not a climb rate
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
        # Ten small log packets a second ride down in the setpoint acks and
        # cost the 250K link nothing; the height loop wants the fresh sample.
        dyn = self.dyn_variables() if self.recorder else {}
        with cfenv.record_log(self.scf, self.log_variables(), period_ms=100) as battery, \
                (cfenv.record_log(self.scf, dyn, period_ms=DYN_PERIOD_MS) if dyn
                 else nullcontext([])) as dyn_samples:
            self.battery, self.dyn = battery, dyn_samples
            try:
                while True:
                    loop_start = time.time()
                    if not self.step(inp, interrupt):
                        break
                    time.sleep(max(0.0, DT - (time.time() - loop_start)))
            finally:
                self.land()
                self.drain()                # the samples from the ramp down


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
    saved_hover = hover = load_hover()

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
            Interruptible() as interrupt, \
            flightlog.Recorder(flightlog.new_path(), COLUMNS) as recorder:
        scf = wait_for_link(uri, inp, interrupt)
        while scf is not None:
            scf.wait_for_params()
            session = Session(scf, trim, mag_offset, gamepad, hover, recorder)
            session.fly(inp, interrupt)
            trim = session.trim
            hover = session.learned_hover() or hover
            if not session.link_lost:
                print("\nLanded, motors stopped.")
                break
            scf = wait_for_link(uri, inp, interrupt)

    if trim != saved:
        save_trim(*trim)
        print(f"Trim saved: roll {trim.roll:+.1f}, pitch {trim.pitch:+.1f}")
    if hover is not None and hover != saved_hover:
        save_hover(hover)
        print(f"Hover thrust saved: {hover:.0f}")
    if recorder.rows:
        print(f"Flight recorded: {recorder.path} -> {flightlog.render(recorder.path)}")
    else:
        recorder.path.unlink()          # never got off the ground: nothing to look at


if __name__ == "__main__":
    cfenv.run(lambda: typer.run(run))
