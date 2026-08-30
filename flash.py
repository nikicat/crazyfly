#!/usr/bin/env python3
"""Back up the firmware over the radio, and flash a new one.

The Crazyflie 1.0 is flashed by its own bootloader over the radio -- no USB.
cflib's Bootloader.flash() assumes a 2.x (it looks up the nRF51 target, which
a 1.0 does not have), so this drives the page writer directly for the single
stm32 target.

  uv run cf.py flash                                 back up only
  uv run cf.py flash --firmware cf1-2017.06.bin      back up, then flash

Run it, then within 5 s switch the drone on from its battery: the bootloader
only runs at power-on. The backup is every flash page after the bootloader,
which is exactly what --firmware writes, so it restores the same way:
--firmware data/firmware-backup.bin. An existing backup is never overwritten; the
first one is the original firmware.

2017.06 is the last firmware with Crazyflie 1.0 support, and the first this
drone needs for altitude hold on its barometer:
https://github.com/bitcraze/crazyflie-firmware/releases/tag/2017.06
"""
from __future__ import annotations

import sys
from pathlib import Path

import typer
from cflib.bootloader import Bootloader, FlashArtifact, Target
from cflib.bootloader.boottypes import TargetTypes

import bootcheck
import cfenv
from flight import DATA

BACKUP = DATA / "firmware-backup.bin"


def read_flash(bl: Bootloader, target) -> bytes | None:
    """Every flash page from the firmware start to the end, or None on a miss."""
    pages = []
    for page in range(target.start_page, target.flash_pages):
        data = bl._cload.read_flash(target.id, page)
        if data is None:
            return None
        pages.append(data)
        print(".", end="", flush=True)
    print()
    return b"".join(pages)


def run(firmware: Path | None = None, backup: Path = BACKUP) -> None:
    """Back up the drone's firmware over the radio, and flash a new one."""
    image = firmware.read_bytes() if firmware else None

    cfenv.init()
    bl = bootcheck.find_bootloader()
    if bl is None:
        sys.exit("No bootloader answered. Run it again and switch the drone on "
                 "from battery within 5 s.")
    try:
        target = bl._cload.targets[TargetTypes.STM32]
        room = (target.flash_pages - target.start_page) * target.page_size
        print(f"Bootloader found: {target.flash_pages} pages of {target.page_size} B, "
              f"firmware from page {target.start_page}, {room // 1024} KB available.")
        if image is not None and len(image) > room:
            sys.exit(f"{firmware} is {len(image)} B, only {room} B fit. Nothing written.")

        if backup.exists():
            print(f"{backup.name} exists, keeping it.")
        else:
            print(f"Reading {target.flash_pages - target.start_page} pages into "
                  f"{backup.name} ", end="", flush=True)
            data = read_flash(bl, target)
            if data is None:
                sys.exit("\nThe bootloader stopped answering the flash read. "
                         "No backup made, nothing written.")
            backup.write_bytes(data)
            print(f"Backed up {len(data)} B.")

        if image is not None:
            bl._internal_flash(FlashArtifact(image, Target("cf1", "stm32", "fw", [], []), None))
            print(f"Flashed {firmware.name} ({len(image)} B).")

        bl.reset_to_firmware()
        print("Restarted into firmware.")
    finally:
        bl.close()


if __name__ == "__main__":
    cfenv.run(lambda: typer.run(run))
