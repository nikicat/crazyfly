#!/usr/bin/env python3
"""Autonomous takeoff, hover, land. Requires a Flow deck.

Without a positioning deck the drone has no idea how high it is and will not
hold altitude, so this script checks for the deck and refuses to arm without
one. Use teleop.py for manual flight instead.

  python hover.py [--height 0.4] [--seconds 5]
"""
from __future__ import annotations

import sys
import time

import typer
from cflib.crazyflie import Crazyflie
from cflib.positioning.motion_commander import MotionCommander

import cfenv

FLOW_DECK_PARAMS = ("deck.bcFlow2", "deck.bcFlow")


def has_flow_deck(cf: Crazyflie) -> bool:
    for param in FLOW_DECK_PARAMS:
        try:
            if cf.param.get_value(param) == "1":
                return True
        except KeyError:
            continue
    return False


def run(
    height: float = 0.4,
    seconds: float = 5.0,
    uri: str | None = None,
) -> None:
    """Autonomous takeoff, hover and land. Requires a Flow deck."""

    cfenv.init()
    uri = cfenv.resolve_uri(uri)

    print(f"Connecting to {uri} ...")
    with cfenv.connect(uri) as scf:
        cf = scf.cf

        if cf.platform.get_protocol_version() < 0:
            sys.exit(
                "This is a Crazyflie 1.0. It has no deck support and no\n"
                "high-level commander, so autonomous hovering is not possible.\n"
                "Fly it manually with teleop.py."
            )

        if not has_flow_deck(cf):
            sys.exit(
                "No Flow deck detected -- refusing to take off.\n"
                "Nothing would hold altitude and the drone would climb until it\n"
                "hit something. Attach a Flow deck, or fly manually with teleop.py."
            )

        print(f"Flow deck found. Taking off to {height} m ...")
        with MotionCommander(scf, default_height=height):
            time.sleep(seconds)
            print("Landing ...")
        print("Landed.")


if __name__ == "__main__":
    cfenv.run(lambda: typer.run(run))
