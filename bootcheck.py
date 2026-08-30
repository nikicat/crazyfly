#!/usr/bin/env python3
"""Look for a Crazyflie sitting in bootloader mode.

The bootloader answers the radio even when the firmware is missing or corrupt,
so this works when a normal connection does not. It is the radio-only way to
tell "firmware is broken" apart from "hardware is dead".

Getting into the bootloader differs by model:

  Crazyflie 1.0   The bootloader only runs for a moment at power-on, so start
                  this scan FIRST, then within 5 s switch the drone on from
                  its battery (not USB). No button hold, no blinking LEDs.
  Crazyflie 2.x   With the drone off, hold the power button ~3 s until the
                  blue LEDs blink fast, then run this.

If it finds the bootloader, the radio and the MCU are alive and the fix is a
reflash: flash.py does that, backup first.
"""
from __future__ import annotations

from cflib.bootloader import Bootloader
from cflib.bootloader.boottypes import BootVersion

import cfenv

SCAN_WINDOW = 10         # seconds cflib listens for a cold-booted bootloader


def find_bootloader() -> Bootloader | None:
    """Scan for a cold-booted bootloader. The caller closes what it gets."""
    print(f"Listening {SCAN_WINDOW} s for a bootloader -- "
          "Crazyflie 1.0: switch it on from battery NOW.", flush=True)
    bl = Bootloader()
    if bl.start_bootloader(warm_boot=False):
        return bl
    bl.close()
    return None


def run() -> None:
    """Look for a Crazyflie sitting in bootloader mode."""
    cfenv.init()

    bl = find_bootloader()
    if bl is None:
        print("\nNo bootloader found.\n"
              "Crazyflie 1.0: run this again and switch the drone on within 5 s.\n"
              "Crazyflie 2.x: hold the power button ~3 s until the blue LEDs blink, "
              "then run this.")
        return
    try:
        model = "Crazyflie 2.x" if BootVersion.is_cf2(bl.protocol_version) else "Crazyflie 1.0"
        print(f"\nFound the bootloader ({model}, protocol {bl.protocol_version:#x}). "
              "The drone is recoverable.\n")
        for target in bl._cload.targets.values():
            print(f"  target {target.id:#x}: {target.flash_pages} flash pages of "
                  f"{target.page_size} B, firmware from page {target.start_page}")
        print("\nTo reflash:  uv run cf.py flash --firmware <file>.bin   (backs up first)")
    finally:
        bl.close()


if __name__ == "__main__":
    cfenv.cli(run)
