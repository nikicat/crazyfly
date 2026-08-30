"""teleop: the keyboard mapping and the flight loop, flown off-line."""
from __future__ import annotations

import csv
import inspect
import math
from contextlib import nullcontext

import pytest
from conftest import FakeCrazyflie, descends_monotonically

import cfenv
import flight
import flightlog
import teleop


class Holding:
    """A keyboard with a fixed set of keys held down."""

    def __init__(self, *keys):
        self.keys = set(keys)

    def down(self, key):
        return key in self.keys


def test_up_arrow_flies_forward():
    """Regression: up sent -MAX_ANGLE, which flew the drone backwards."""
    assert teleop.held_axis(Holding("up"), "down", "up", 0.0, 15.0) == 15.0
    assert teleop.held_axis(Holding("down"), "down", "up", 0.0, 15.0) == -15.0
    assert teleop.held_axis(Holding(), "down", "up", 10.0, 15.0) == 10.0 * flight.DECAY
    assert ('held_axis(inp, "down", "up", self.pitch, MAX_ANGLE)'
            in inspect.getsource(teleop.Session.read_keyboard))


class ScriptedInput:
    """A keyboard whose keys are held for a scripted number of ticks, then released."""

    def __init__(self, *script: tuple[str, int]) -> None:
        self.script = list(script)          # (key, ticks held)
        self.current: str | None = None

    def poll(self) -> list[str]:
        if self.script and self.script[0][1] <= 0:
            self.script.pop(0)
        if not self.script:
            self.current = None
            return []
        key, ticks = self.script[0]
        self.script[0] = (key, ticks - 1)
        first = self.current != key
        self.current = key
        return [key] if first else []

    def down(self, key: str) -> bool:
        return key == self.current


def fake_scf(cf) -> object:
    """A SyncCrazyflie stand-in whose link stays up until close_link()."""
    toc = type("T", (), {"toc": {}})()
    cf.param = type("P", (), {"toc": toc, "set_value": lambda *_a: None})()
    cf.log = type("L", (), {"toc": toc})()

    class Scf:
        def __init__(self):
            self.cf = cf
            self.open = True

        def is_link_open(self):
            return self.open

        def close_link(self):
            self.open = False

    return Scf()


@pytest.mark.usefixtures("no_telemetry")
def test_session_flies_up_and_lands_on_quit():
    """Holding w raises thrust a step per tick; q ramps it down to zero, never
    up again, and closes the link -- the whole loop, off-line."""
    drone = FakeCrazyflie()
    scf = fake_scf(drone)
    session = teleop.Session(scf, flight.Trim(0.5, -1.0), mag_offset=None, gamepad=False)
    session.fly(ScriptedInput(("w", 10), ("q", 1)), flight.Interruptible())
    thrusts = drone.commander.thrusts

    assert max(thrusts) == flight.MIN_THRUST + 9 * flight.THRUST_STEP
    assert thrusts[-1] == 0
    assert descends_monotonically(thrusts)
    assert not scf.open
    assert session.trim == (0.5, -1.0)


def test_keyboard_yaw_releases_to_zero():
    """Yaw snaps to 0 on release, so the heading hold can tell a centred stick."""
    session = teleop.Session(fake_scf(FakeCrazyflie()), flight.Trim(),
                             mag_offset=None, gamepad=False)
    session.read_keyboard(Holding("d"))
    assert session.yaw_rate == flight.MAX_YAW_RATE
    session.read_keyboard(Holding())
    assert session.yaw_rate == 0


def test_heading_hold_trims_yaw_toward_the_release_heading():
    """Stick centred and airborne: the heading where it was released is held,
    the correction takes the short way round the circle and is capped, and
    touching the stick lets go of it. On the ground the stick aims the
    reference instead, and the next takeoff swings round to it."""
    session = teleop.Session(fake_scf(FakeCrazyflie()), flight.Trim(),
                             mag_offset=(0.0, 0.0, 0.0), gamepad=False)
    session.has_mag = True
    session.thrust = 30000

    def tick(hdg: float, *keys: str) -> None:
        """One loop tick: the compass reads `hdg`, level, then sticks, then the hold."""
        session.battery.append({"mag.x": math.cos(math.radians(hdg)),
                                "mag.y": math.sin(math.radians(hdg)), "mag.z": 0.0,
                                "stabilizer.roll": 0.0, "stabilizer.pitch": 0.0})
        session.read_keyboard(Holding(*keys))
        session.hold_heading()

    tick(10)
    assert session.ref_hdg == pytest.approx(10) and session.yaw_rate == 0

    tick(5)                                        # drifted 5 deg low
    assert session.yaw_rate == pytest.approx(teleop.HDG_SIGN * teleop.HDG_KP * 5)

    tick(350)                                      # 20 deg low through the wrap, capped
    assert session.yaw_rate == pytest.approx(teleop.HDG_SIGN * teleop.HDG_MAX_RATE)

    tick(350, "d")                                 # the stick moves the reference
    assert session.ref_hdg is None and session.yaw_rate == flight.MAX_YAW_RATE
    tick(300)                                      # taken afresh on release
    assert session.ref_hdg == pytest.approx(300) and session.yaw_rate == 0

    session.thrust = 0.0                           # landed: kept, but no correction
    tick(300)
    assert session.ref_hdg == pytest.approx(300) and session.yaw_rate == 0

    turned = teleop.HDG_SIGN * flight.MAX_YAW_RATE * flight.DT
    tick(300, "d")                                 # aim it from the ground
    tick(300)
    assert session.ref_hdg == pytest.approx(300 + turned) and session.yaw_rate == 0

    session.thrust = 30000                         # airborne: swings round to the aim
    tick(300)
    assert session.yaw_rate == pytest.approx(teleop.HDG_SIGN * teleop.HDG_KP * turned)


def test_trim_keys_step_and_reset():
    session = teleop.Session(fake_scf(FakeCrazyflie()), flight.Trim(),
                             mag_offset=None, gamepad=False)
    session.handle_keys(["]", "]", "'"])
    assert session.trim == pytest.approx((2 * teleop.TRIM_STEP, teleop.TRIM_STEP))
    session.handle_keys(["0"])
    assert session.trim == (0.0, 0.0)
    for _ in range(100):
        session.handle_keys(["["])
    assert session.trim.roll == -flight.TRIM_LIMIT


def test_height_mode_takes_off_holds_and_lands_by_reference():
    """Height mode: the stick moves the reference. Raising it off the ground
    engages the firmware hold around the hover thrust, the loop steers the
    climb rate at it and learns the hover thrust while still, and asking for
    well below the drone while the firmware sits at thrustMin lands."""
    drone = FakeCrazyflie()
    session = teleop.Session(fake_scf(drone), flight.Trim(), mag_offset=None, gamepad=False)
    session.has_hold = session.height_mode = True
    params = {}
    session.cf.param.set_value = lambda name, value: params.__setitem__(name, value)
    step = teleop.KEY_CLIMB * teleop.Z_RATE * flight.DT     # metres per tick of w / s

    def tick(z: float, thrust: float, *keys: str, now: float = 0.0) -> None:
        """One loop tick: the log reads height z and firmware thrust, then sticks, then the loop."""
        session.battery.append({"pm.vbat": 3.9, teleop.Z_LOG: z, teleop.FW_THRUST_LOG: thrust})
        session.read_keyboard(Holding(*keys))
        session.hold_height(now)

    tick(5.0, 0)                                   # landed: the reference rests on the ground
    assert session.ref_z == 5.0 and not session.hold and session.thrust == 0
    tick(4.9, 0)                                   # ...and follows the barometer's drift
    assert session.ref_z == 4.9
    for _ in range(3):
        tick(4.9, 0, "w")                          # a short push: not enough to lift off
    assert session.ref_z == pytest.approx(4.9 + 3 * step) and not session.hold
    tick(4.9, 0)                                   # released: back on the ground
    assert session.ref_z == 4.9
    while not session.hold:
        tick(4.9, 0, "w")                          # held: takes off past TAKEOFF_ABOVE
    assert session.ref_z > 4.9 + teleop.TAKEOFF_ABOVE
    assert params[teleop.THRUST_BASE] == str(teleop.HOVER_DEFAULT) and params[teleop.ALTHOLD] == "1"

    tick(4.9, 30000)                               # airborne, under the reference: climb
    assert session.climb == pytest.approx(teleop.Z_KP * (session.ref_z - 4.9))
    session.send()
    assert drone.commander.thrusts[-1] == int(teleop.HOLD_CENTRE + teleop.HOLD_SPAN * session.climb)

    ref = session.ref_z
    for _ in range(teleop.HOVER_SAMPLES):
        tick(ref, 41000)                           # holding still: that thrust is hover
    assert session.learned_hover() == pytest.approx(41000)
    tick(ref + 1.0, 41000)                         # a metre too high: sink, capped
    assert session.climb == -teleop.Z_MAX_RATE

    for _ in range(40):
        tick(ref, 41000, "s")                      # stick down: the reference goes under the floor
    assert session.ref_z == pytest.approx(ref - 40 * step) and session.hold
    tick(ref, teleop.FW_THRUST_MIN, now=0.5)       # firmware bottomed out, not for long enough
    assert session.hold
    tick(ref, teleop.FW_THRUST_MIN, now=1.5)       # ...now it has: landed
    assert not session.hold and session.thrust == 0 and session.ref_z == ref
    assert params[teleop.ALTHOLD] == "0" and session.hover == pytest.approx(41000)


def test_flight_is_recorded_a_row_per_tick_with_the_log_beside_it(tmp_path, monkeypatch):
    """Every setpoint leaves a cmd row -- with the trim folded in, as sent -- and
    each log sample a row of its own, once, however many ticks it sits there."""
    sample = {"ts": 1234, "pm.vbat": 3.9}
    monkeypatch.setattr(cfenv, "record_log", lambda *_a, **_k: nullcontext([dict(sample)]))
    path = tmp_path / "flight.csv"
    with flightlog.Recorder(path, teleop.COLUMNS) as recorder:
        session = teleop.Session(fake_scf(FakeCrazyflie()), flight.Trim(0.5, -1.0),
                                 mag_offset=None, gamepad=False, recorder=recorder)
        session.fly(ScriptedInput(("w", 3), ("q", 1)), flight.Interruptible())
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))

    cmd = [r for r in rows if r["src"] == "cmd"]
    assert len(cmd) == 3 and recorder.rows == 4
    assert [r["thrust"] for r in cmd] == [str(flight.MIN_THRUST + i * flight.THRUST_STEP)
                                          for i in range(3)]
    assert cmd[0]["roll"] == "0.5" and cmd[0]["pitch"] == "-1.0" and cmd[0]["hold"] == "0"
    assert cmd[0]["ref_z"] == "" and cmd[0]["flip"] == ""
    log = [r for r in rows if r["src"] == "log"]
    assert len(log) == 1 and log[0]["ts"] == "1234" and log[0]["pm.vbat"] == "3.9"
    assert log[0]["roll"] == "" and float(log[0]["t"]) >= float(cmd[0]["t"])


def test_configured_params_are_written_to_the_firmware_on_prepare(monkeypatch):
    """Every entry of config.json's "params" is applied at connect -- the
    firmware forgets them at power-off -- and unknown names are skipped."""
    session = teleop.Session(fake_scf(FakeCrazyflie()), flight.Trim(),
                             mag_offset=None, gamepad=False)
    session.cf.param.toc.toc.update({"posEst": {"vAccDeadband": object()}})
    written = {}
    session.cf.param.set_value = lambda name, value: written.__setitem__(name, value)
    monkeypatch.setattr(teleop, "setting", lambda name, default:
                        {"params": {"posEst.vAccDeadband": 0.01, "nosuch.knob": 1}}
                        .get(name, default))
    session.prepare()
    assert written == {"posEst.vAccDeadband": "0.01"}
