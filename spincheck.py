#!/usr/bin/env python3
"""Drive one motor by hand and note where it starts and stops turning.

  uv run cf.py spin              start on m1 at power 0
  uv run cf.py spin --motor 4

  w / s         power +1000 / -1000   1 2 3 4   pick a motor (the previous one goes to 0)
  d / a         power +100 / -100     0         all four together, same power
  space         power 0
  m             mark: it STARTED turning at this power
  n             mark: it STOPPED turning at this power
  q / ESC       stop the motor and quit, printing the marks

Drives the motor directly through motorPowerSet, bypassing the flight
controller; the status line shows the power and the battery voltage. You watch
the propeller and press m / n when you see it start or stop; the marks are
listed per motor at the end -- switch motors with the number keys, there is
no need to restart. Brushed motors need more to break away than to
keep going, so start > stop is normal; a motor whose start is well above the
others', or whose gap is much wider, is the tired one.

SAFETY: the propeller spins. Drone on a flat surface, props clear, hold it by
the battery -- one motor cannot lift it but it will try to turn it. Ctrl-C or
q stops the motor at any point.
"""
from __future__ import annotations

import sys
import time

import typer

import cfenv
from flight import MAX_THRUST, QUIT_KEYS, Keyboard, clamp

BIG_STEP = 1000
SMALL_STEP = 100
DT = 0.1
ALL = (1, 2, 3, 4)
STEPS = {"w": BIG_STEP, "s": -BIG_STEP, "d": SMALL_STEP, "a": -SMALL_STEP}


def check_selftest(cf) -> None:
    """Refuse to go on if the drone failed its power-on self test."""
    if cf.param.values.get("system", {}).get("selftestPassed") != "0":
        return
    # The firmware tests MPU6050, then HMC5883L, then MS5611, and stops at
    # the first failure, so the first 0 is the one that matters.
    tests = cf.param.values.get("imu_tests", {})
    failed = next((k for k in ("MPU6050", "HMC5883L", "MS5611") if tests.get(k) == "0"),
                  "a sensor")
    sys.exit(f"The drone failed its self test at boot ({failed}), so it runs neither "
             "motors nor telemetry.\nPower it off and on again, resting still and "
             "level -- the gyro test fails if it is moving at power-on.")


class Bench:
    """The motor(s) under test, their power, and the marks noted so far."""

    def __init__(self, cf, motor: int) -> None:
        self.cf = cf
        self.power = 0
        self.selected = (motor,)               # ALL means all four together
        self.marks: dict[str, list[str]] = {}

    @property
    def label(self) -> str:
        return "all" if self.selected == ALL else f"m{self.selected[0]}"

    def set_power(self, value: int) -> None:
        self.power = int(clamp(value, 0, MAX_THRUST))
        for m in self.selected:
            self.cf.param.set_value(f"motorPowerSet.m{m}", str(self.power))

    def handle(self, key: str) -> None:
        if key == " ":
            self.set_power(0)
        elif key in "01234":
            self.set_power(0)              # the previous motor goes to 0
            self.selected = ALL if key == "0" else (int(key),)
        elif key == "m":
            self.marks.setdefault(self.label, []).append(f"starts at {self.power}")
        elif key == "n":
            self.marks.setdefault(self.label, []).append(f"stops at {self.power}")
        elif key in STEPS:
            self.set_power(self.power + STEPS[key])   # holding the key repeats it

    def status(self, vbat: float | None) -> str:
        bat = f"{vbat:.2f}V" if vbat is not None else "?    "
        return (f"\r  {self.label:<4} power {self.power:>6}   bat {bat}   "
                f"marks: {', '.join(self.marks.get(self.label, [])) or '-':<40}")

    def stop(self) -> None:
        for m in ALL:
            self.cf.param.set_value(f"motorPowerSet.m{m}", "0")
        self.cf.param.set_value("motorPowerSet.enable", "0")
        time.sleep(0.2)


def run(motor: int = 1, uri: str | None = None) -> None:
    """Drive one motor by hand and note where it starts and stops turning."""
    if motor not in ALL:
        raise SystemExit("motor must be 1-4")

    with cfenv.session(uri) as scf:
        cf = scf.cf
        if "motorPowerSet" not in cf.param.toc.toc:
            sys.exit("This firmware has no motorPowerSet; it cannot drive one motor directly.")
        check_selftest(cf)

        print(__doc__.split("\n\n", 1)[1].split("Drives the motor")[0])

        bench = Bench(cf, motor)
        with cfenv.record_log(scf, {"pm.vbat": "float"}, period_ms=500) as battery, \
                Keyboard() as kb:
            try:
                cf.param.set_value("motorPowerSet.enable", "1")
                bench.set_power(0)
                quit_ = False
                while not quit_:
                    for key in kb.poll():
                        if key in QUIT_KEYS:
                            quit_ = True
                            break
                        bench.handle(key)
                    print(bench.status(battery[-1]["pm.vbat"] if battery else None),
                          end="", flush=True)
                    time.sleep(DT)
            finally:
                bench.stop()

        print("\n\nMotors stopped.")
        for which, notes in bench.marks.items():
            print(f"  {which}: {', '.join(notes)}")


if __name__ == "__main__":
    cfenv.run(lambda: typer.run(run))
