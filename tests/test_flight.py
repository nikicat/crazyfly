"""flight: the primitives every tool shares -- stopping, input devices, saved state."""
from __future__ import annotations

import math
import os
import signal
import struct
import threading
import time

import pytest
from conftest import FakeCrazyflie

import flight

# --- stopping -------------------------------------------------------------

def test_stop_motors_ignores_sigint():
    """The stop sequence must complete even if Ctrl-C arrives during it."""
    drone = FakeCrazyflie()

    def interrupt_soon():
        time.sleep(0.02)
        os.kill(os.getpid(), signal.SIGINT)

    threading.Thread(target=interrupt_soon, daemon=True).start()
    flight.stop_motors(drone, from_thrust=40000, dt=0.01)

    assert drone.commander.thrusts[-1] == 0
    assert signal.getsignal(signal.SIGINT) is not signal.SIG_IGN


# --- gamepad --------------------------------------------------------------

def js_event(kind: int, number: int, value: int, initial: bool = False) -> bytes:
    """One Linux joystick event, as the kernel would write it."""
    return struct.pack("IhBB", 0, value, kind | (0x80 if initial else 0), number)


def test_gamepad_maps_xbox_controls_to_teleop_keys(tmp_path):
    """B cuts thrust, Menu lands, the D-pad trims, and pushing either stick
    up reads positive: more thrust on the left, forward pitch on the right."""
    path = tmp_path / "js0"
    os.mkfifo(path)

    class TypedQ:
        def poll(self):
            return ["q"]

    with flight.Gamepad(str(path), keyboard=TypedQ()) as pad:
        writer = os.open(path, os.O_WRONLY)
        os.write(writer, b"".join([
            js_event(1, 1, 1, initial=True),     # state dump on open: not a press
            js_event(1, 1, 1),                   # B
            js_event(1, 11, 1),                  # Menu
            js_event(2, 7, -32767),              # D-pad up
            js_event(2, 7, 0),                   # D-pad released: no event
            js_event(2, 1, -32767),              # left stick full up
            js_event(2, 3, -16000),              # right stick half up
            js_event(2, 2, 2000),                # right stick X inside the deadzone
            js_event(2, 5, -32767, initial=True),  # LT at rest
            js_event(2, 4, 32767),               # RT fully pulled
        ]))
        events = pad.poll()

        assert events == ["q", " ", "q", "'"]      # keyboard q first, then the pad
        assert pad.axis("thrust") == pytest.approx(1.0)
        assert 0.3 < pad.axis("pitch") < 0.5
        assert pad.axis("roll") == 0.0
        assert pad.trigger("yaw_left") == 0.0
        assert pad.trigger("yaw_right") == pytest.approx(1.0)

        os.close(writer)
        with pytest.raises(OSError):
            pad.poll()                           # writer gone == pad disconnected


# --- trim persistence -----------------------------------------------------

def test_trim_roundtrips(tmp_path, monkeypatch):
    monkeypatch.setattr(flight, "TRIM_FILE", tmp_path / "trim.json")
    flight.save_trim(-0.3, -1.9)
    assert flight.load_trim() == (-0.3, -1.9)


def test_missing_trim_file_reads_as_zero(tmp_path, monkeypatch):
    monkeypatch.setattr(flight, "TRIM_FILE", tmp_path / "absent.json")
    assert flight.load_trim() == (0.0, 0.0)


def test_corrupt_trim_file_reads_as_zero(tmp_path, monkeypatch):
    bad = tmp_path / "trim.json"
    bad.write_text("{not json")
    monkeypatch.setattr(flight, "TRIM_FILE", bad)
    assert flight.load_trim() == (0.0, 0.0)


# --- config.json ----------------------------------------------------------

def test_setting_reads_config_or_falls_back(tmp_path, monkeypatch):
    path = tmp_path / "config.json"
    monkeypatch.setattr(flight, "CONFIG_FILE", path)
    assert flight.setting("flip.min_vbat", 3.7) == 3.7          # no file
    path.write_text('{"flip": {"min_vbat": 3.5}}')
    assert flight.setting("flip.min_vbat", 3.7) == 3.5
    assert flight.setting("flip.nothing", 1.0) == 1.0           # missing key
    path.write_text("not json")
    assert flight.setting("flip.min_vbat", 3.7) == 3.7          # broken file


# --- frame layout ---------------------------------------------------------

def test_frame_layout_needs_all_four_arms(tmp_path, monkeypatch):
    path = tmp_path / "frame.json"
    monkeypatch.setattr(flight, "FRAME_FILE", path)
    assert flight.load_frame() is None                       # no file
    path.write_text('{"m1": "front", "m2": "right", "m3": "back", "m4": "left"}')
    assert flight.load_frame() == {"m1": "front", "m2": "right", "m3": "back", "m4": "left"}
    path.write_text('{"m1": "front", "m2": "front", "m3": "back", "m4": "left"}')
    assert flight.load_frame() is None                       # an arm named twice


# --- magnetic heading -----------------------------------------------------

def test_heading_is_level_corrected_and_offset_free():
    """Field straight ahead is 0, to the left is 90 (y points left), and
    tilting the drone must not turn the heading once the field is levelled."""
    assert flight.heading(1, 0, 0, 0, 0) == 0
    assert flight.heading(0, 1, 0, 0, 0) == 90
    nose_up = 30
    assert flight.heading(math.cos(math.radians(nose_up)), 0, -math.sin(math.radians(nose_up)),
                          0, nose_up) == pytest.approx(0, abs=1e-6)
    right_down = math.radians(20)
    assert flight.heading(0, math.cos(right_down), -math.sin(right_down),
                          20, 0) == pytest.approx(90, abs=1e-6)
    assert flight.heading(1 + 1.7, -1.6, -0.3, 0, 0, offset=(1.7, -1.6, -0.3)) == 0


def test_charge_maps_voltage_to_percent_with_flying_sag():
    """The generic curve interpolates and clamps; flying reads the same charge
    BATTERY_SAG lower, and the correction never depends on the thrust value."""
    assert flight.charge(4.20, airborne=False) == 100
    assert flight.charge(4.35, airborne=False) == 100
    assert flight.charge(3.30, airborne=False) == 0
    assert flight.charge(2.95, airborne=False) == 0
    assert flight.charge(3.70, airborne=False) == pytest.approx(32.5)
    assert (flight.charge(3.70 - flight.BATTERY_SAG, airborne=True)
            == pytest.approx(flight.charge(3.70, airborne=False)))
