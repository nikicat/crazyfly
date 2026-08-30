#!/usr/bin/env python3
"""Connect to the Crazyflie and report what it is, plus battery and attitude.

Handles both the Crazyflie 1.0 and the 2.x. They expose different parameters
(the 1.0 has version.revision*, no decks and no estimator/controller choice),
so the fields shown depend on which one answers.
"""
from __future__ import annotations

import statistics

import typer

import cfenv
from flight import VBAT_CRITICAL


def sample(scf, variables: dict[str, str], count: int = 5) -> dict[str, float]:
    """Average `count` samples of the given log variables."""
    series = cfenv.sample_series(scf, variables, count)
    return {name: statistics.fmean(values) for name, values in series.items()}


def param(cf, name: str, default: str = "?") -> str:
    group, _, key = name.partition(".")
    return cf.param.values.get(group, {}).get(key, default)


def run(uri: str | None = None) -> None:
    """Connect and report model, firmware, battery and attitude."""
    with cfenv.session(uri) as scf:
        cf = scf.cf
        legacy = cf.platform.get_protocol_version() < 0
        print("Connected.\n")

        print(f"Model         : {'Crazyflie 1.0 (legacy CRTP)' if legacy else 'Crazyflie 2.x'}")

        if legacy:
            # The 1.0 reports its firmware as a git revision split over two words.
            rev0, rev1 = param(cf, "version.revision0"), param(cf, "version.revision1")
            if rev0.isdigit():
                print(f"Firmware rev  : {int(rev0):08x}-{int(rev1):x}")
            print(f"MCU flash     : {param(cf, 'cpu.flash')} KB")
            cpu_id = "".join(f"{int(param(cf, f'cpu.id{i}', '0')):08x}" for i in range(3))
            print(f"CPU id        : {cpu_id}")
        else:
            print(f"Firmware      : {param(cf, 'firmware.revision0')}")
            print(f"Estimator     : {param(cf, 'stabilizer.estimator')}")
            print(f"Controller    : {param(cf, 'stabilizer.controller')}")
            decks = [
                name[1:]
                for name in cf.param.values.get("deck", {})
                if name.startswith("b") and param(cf, f"deck.{name}") == "1"
            ]
            print(f"Decks         : {', '.join(decks) if decks else 'none detected'}")

        print(f"Params / logs : {len(cf.param.toc.toc)} groups / "
              f"{len(cf.log.toc.toc)} log groups")

        readings = sample(scf, {
            "pm.vbat": "float",
            "stabilizer.roll": "float",
            "stabilizer.pitch": "float",
        })
        vbat = readings["pm.vbat"]
        print(f"\nBattery       : {vbat:.2f} V")
        if vbat < VBAT_CRITICAL:
            print("                CRITICAL -- charge before doing anything else")
        elif vbat < 3.7:
            print("                LOW -- charge before flying (full is ~4.2 V)")
        else:
            print("                OK")
        print(f"Attitude      : roll {readings['stabilizer.roll']:+.1f} deg, "
              f"pitch {readings['stabilizer.pitch']:+.1f} deg")

        print()
        if legacy:
            print("Crazyflie 1.0: no decks and no autonomous positioning.\n"
                  "Fly it manually with teleop.py.")
        else:
            print("Use teleop.py for manual flight, or hover.py with a Flow deck.")


if __name__ == "__main__":
    cfenv.run(lambda: typer.run(run))
