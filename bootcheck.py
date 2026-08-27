#!/usr/bin/env python3
"""Look for a Crazyflie sitting in bootloader (DFU) mode.

The bootloader lives in the nRF51 and answers even when the STM32 firmware is
missing or corrupt, so this works when a normal connection does not. It is the
radio-only way to tell "firmware is broken" apart from "hardware is dead".

Put the drone in bootloader mode first:
  power it off, then hold the power button ~3 s until the blue LEDs blink fast.

Then run this. If it finds the bootloader, the radio and both MCUs are alive
and the fix is a reflash.
"""
from __future__ import annotations

import typer
from cflib.bootloader import Bootloader

import cfenv


def run() -> None:
    """Look for a Crazyflie sitting in bootloader mode."""
    cfenv.init()

    print("Scanning for a Crazyflie in bootloader mode (cold boot) ...")
    bl = Bootloader()
    try:
        if not bl.start_bootloader(warm_boot=False):
            print(
                "\nNo bootloader found.\n"
                "Make sure the drone is OFF, then hold the power button about 3 s\n"
                "until the blue LEDs blink rapidly, and run this again."
            )
            return

        print("\nFound the bootloader. The drone is recoverable.\n")
        print(f"  connected over : {bl.protocol_version}")
        for target in bl.targets.values():
            print(f"  target {target.id}: flash pages {target.flash_pages}, "
                  f"page size {target.page_size}, version {target.protocol_version}")
        print(
            "\nTo reflash with current official firmware:\n"
            "  1. download the latest cf2 firmware .zip from\n"
            "     https://github.com/bitcraze/crazyflie-release/releases\n"
            "  2. uv run python -m cfloader flash <file>.zip stm32-fw\n"
        )
    finally:
        bl.close()


if __name__ == "__main__":
    cfenv.run(lambda: typer.run(run))
