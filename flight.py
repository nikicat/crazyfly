"""Flight primitives shared by the interactive tools.

Keyboard input, trim persistence, the setpoint limits, and the one routine that
must always work: bringing the motors down. No cflib connection logic lives
here -- that is cfenv -- so this stays importable and testable on its own.
"""
from __future__ import annotations

import json
import os
import select
import signal
import struct
import sys
import termios
import time
import tty
from pathlib import Path

# A 250K link tops out around 35 setpoints/s once each send blocks on its ack,
# so asking for 50 Hz just means the sleep never fires. Pace to what the link
# can actually carry, which keeps the thrust ramp rate predictable.
RATE_HZ = 33
DT = 1.0 / RATE_HZ

MIN_THRUST = 10001       # firmware ignores thrust at or below 10000
MAX_THRUST = 50000       # well short of the 65535 ceiling, on purpose
THRUST_STEP = 800        # per tick while a thrust key is held
MAX_ANGLE = 15.0         # degrees of roll/pitch at full deflection
MAX_YAW_RATE = 90.0      # degrees per second
DECAY = 0.75             # attitude return-to-neutral per tick when idle

# How long a key stays "held" after its last repeat, in seconds. Terminal key
# repeat is ~30 ms, so this rides over the gaps without feeling sticky.
HOLD = 0.12

TRIM_LIMIT = 10.0        # refuse to trim past this; beyond it something is bent
VBAT_CRITICAL = 3.4      # volts; land now. Full is ~4.2, sags a few tenths under load
TRIM_FILE = Path(__file__).with_name("trim.json")

JS_DEVICE = "/dev/input/js0"
JS_DEADZONE = 0.15       # stick centre wobble below this reads as neutral

# Joystick numbering measured on an Xbox Series controller over Bluetooth
# (hid-microsoft). Plugged in over USB, xpad numbers the buttons differently --
# remeasure with `cat /proc/bus/input/devices` and a js dump before trusting it.
# Y axes read negative when pushed up, hence the sign on thrust and pitch.
JS_AXES = {"thrust": (1, -1), "roll": (2, +1), "pitch": (3, -1)}
JS_TRIGGERS = {"yaw_left": 5, "yaw_right": 4}   # LT / RT, analog, rest at -1
# Buttons and D-pad emit the key they stand in for, so teleop handles both alike.
JS_BUTTONS = {0: "h", 1: " ", 10: "0", 11: "q"}         # A hold, B cut, View reset trim, Menu land
JS_HATS = {(6, -1): "[", (6, +1): "]",          # D-pad left/right: roll trim
           (7, -1): "'", (7, +1): ";"}          # D-pad up/down: pitch trim fwd/back


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def load_trim() -> tuple[float, float]:
    try:
        saved = json.loads(TRIM_FILE.read_text())
        return float(saved["roll"]), float(saved["pitch"])
    except (OSError, ValueError, KeyError):
        return 0.0, 0.0


def save_trim(roll: float, pitch: float) -> None:
    TRIM_FILE.write_text(json.dumps({"roll": round(roll, 2),
                                     "pitch": round(pitch, 2)}, indent=2) + "\n")


def stop_motors(cf, from_thrust: float = 0.0, step: float = THRUST_STEP,
                dt: float = DT) -> None:
    """Ramp thrust to zero and release the commander.

    Ignores Ctrl-C for the second or so this takes. Interrupting the stop
    sequence is the one thing that must not be possible -- it would leave the
    drone flying with nothing driving it.
    """
    previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        thrust = float(from_thrust)
        while thrust > MIN_THRUST:
            thrust = max(MIN_THRUST, thrust - step)
            cf.commander.send_setpoint(0, 0, 0, int(thrust))
            time.sleep(dt)
        for _ in range(3):
            cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(dt)
        # send_stop_setpoint uses the generic setpoint port, which the
        # Crazyflie 1.0 does not implement; the zero setpoints above are the
        # equivalent there.
        if cf.platform.get_protocol_version() >= 0:
            cf.commander.send_stop_setpoint()
    finally:
        signal.signal(signal.SIGINT, previous)


class Keyboard:
    """Raw-mode stdin reader that tracks which keys are currently held."""

    def __init__(self) -> None:
        self.fd = sys.stdin.fileno()
        self.saved = termios.tcgetattr(self.fd)
        self.held: dict[str, float] = {}

    def __enter__(self) -> Keyboard:
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc) -> None:
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)

    def poll(self) -> list[str]:
        """Drain pending input, refresh hold times, return one-shot key events."""
        events = []
        now = time.time()
        while select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch == "\x1b":
                # Either a bare ESC or the start of an arrow-key sequence.
                if not select.select([sys.stdin], [], [], 0.002)[0]:
                    events.append("ESC")
                    continue
                rest = sys.stdin.read(2)
                key = {"[A": "up", "[B": "down", "[C": "right", "[D": "left"}.get(rest)
                if key:
                    self.held[key] = now
            else:
                self.held[ch.lower()] = now
                events.append(ch.lower())
        # Expire keys that have stopped repeating.
        for key in [k for k, t in self.held.items() if now - t > HOLD]:
            del self.held[key]
        return events

    def down(self, key: str) -> bool:
        return key in self.held


class Gamepad:
    """Linux joystick reader that speaks the same events as Keyboard.

    Sticks come back from axis() as -1..1 with the spring doing the return to
    neutral, so there is nothing to decay. Buttons and the D-pad are one-shot
    events named after the key they replace, and a Keyboard passed in is polled
    too, so its keys still quit or cut thrust. A read failure means the pad is
    gone -- Bluetooth dropped or battery flat -- and is raised so the flight
    loop lands rather than holding the last setpoint.
    """

    def __init__(self, path: str = JS_DEVICE, keyboard: Keyboard | None = None) -> None:
        self.path = path
        self.keyboard = keyboard    # its keys keep working as a backstop: q, ESC, space
        self.fd = -1
        self.axes: dict[int, float] = {}

    def __enter__(self) -> Gamepad:
        self.fd = os.open(self.path, os.O_RDONLY | os.O_NONBLOCK)
        return self

    def __exit__(self, *exc) -> None:
        os.close(self.fd)

    def poll(self) -> list[str]:
        """Drain pending joystick events, return one-shot key-style events."""
        events = self.keyboard.poll() if self.keyboard else []
        while select.select([self.fd], [], [], 0)[0]:
            chunk = os.read(self.fd, 8 * 64)
            if not chunk:
                raise OSError("gamepad disconnected")
            for _stamp, value, kind, number in struct.iter_unpack("IhBB", chunk):
                initial = kind & 0x80        # state dump on open, not a press
                key = None
                if kind & 0x7F == 2:         # JS_EVENT_AXIS
                    self.axes[number] = value / 32767
                    key = JS_HATS.get((number, (value > 0) - (value < 0)))
                elif kind & 0x7F == 1 and value:   # JS_EVENT_BUTTON, pressed
                    key = JS_BUTTONS.get(number)
                if key and not initial:
                    events.append(key)
        return events

    def axis(self, name: str) -> float:
        """Stick deflection in -1..1, zero inside the deadzone, then linear."""
        number, sign = JS_AXES[name]
        value = self.axes.get(number, 0.0)
        if abs(value) < JS_DEADZONE:
            return 0.0
        scaled = (abs(value) - JS_DEADZONE) / (1.0 - JS_DEADZONE)
        return sign * scaled * (1 if value > 0 else -1)

    def trigger(self, name: str) -> float:
        """Trigger pull in 0..1; the joystick API reports it as -1 at rest."""
        pull = (self.axes.get(JS_TRIGGERS[name], -1.0) + 1.0) / 2.0
        return 0.0 if pull < JS_DEADZONE else pull


class Interruptible:
    """Turn Ctrl-C into a flag instead of an exception, while flying.

    A KeyboardInterrupt raised at an arbitrary point leaves the motors running
    at whatever they were doing: the script dies, setpoints stop, and nothing
    cuts thrust until the firmware watchdog notices a second or two later. With
    this, the first Ctrl-C asks the flight loop to land through its normal
    ramp-down, and a second one escalates to the usual exception.
    """

    def __init__(self, message: str = "\nStopping -- landing. "
                                      "Ctrl-C again to cut motors now."):
        self.requested = False
        self._message = message
        self._previous = None
        self._count = 0

    def _handler(self, _signum, _frame) -> None:
        self._count += 1
        self.requested = True
        if self._count == 1:
            print(self._message, flush=True)
        else:
            raise KeyboardInterrupt

    def __enter__(self) -> Interruptible:
        self._previous = signal.signal(signal.SIGINT, self._handler)
        return self

    def __exit__(self, *exc) -> None:
        signal.signal(signal.SIGINT, self._previous)
