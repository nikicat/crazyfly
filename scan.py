#!/usr/bin/env python3
"""Scan every radio channel and datarate for Crazyflies in range."""
import cflib.crtp

import cfenv


def run() -> None:
    """Sweep every channel and datarate for Crazyflies in range."""
    cfenv.init()

    print("Scanning all channels and datarates (slow, ~20 s) ...")
    found = cflib.crtp.scan_interfaces()

    if not found:
        print("\nNo Crazyflie found. Check that it is powered on and in range.")
        return

    print(f"\nFound {len(found)}:")
    for uri, comment in found:
        print(f"  {uri}  {comment}")
    print(f"\nPin it for other scripts:\n  set -x CF_URI {found[0][0]}")


if __name__ == "__main__":
    cfenv.cli(run)
