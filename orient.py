#!/usr/bin/env python3
"""Work out which arm of the drone is the front.

A Crazyflie 1.0 flies in "+" configuration: one motor at the front, one at the
back, one each side. So the front is a single arm, not a gap between two -- but
the frame looks the same from every side, and which arm is the front is set by
how the IMU is mounted rather than by anything visible.

Method: lie the drone flat to get a baseline, then lift ONE ARM by its motor
and hold it. In plus configuration that is a pure single-axis tilt, so whichever
of roll/pitch moves tells you which axis that arm is on, and the sign tells you
which end.

The reasoning, for the pitch axis:

  * the controller settles where the attitude estimate equals the setpoint
  * cflib transmits -pitch (see Commander.send_setpoint), so commanding a
    positive pitch drives the estimate negative
  * if lifting edge E drives pitch negative, then negative pitch means E is up
    and the opposite edge is down -- so the drone accelerates that way
  * a positive pitch command is "forward" by convention, so the front is the
    edge opposite E

Roll is the same argument without the sign flip, since roll is sent unchanged.
"""
from __future__ import annotations

import statistics
import sys

import typer

import cfenv

SAMPLES = 15
MIN_TILT = 10.0     # degrees; below this the reading is ambiguous
DOMINANCE = 2.0     # one axis must move this many times more than the other


def read(scf) -> tuple[float, float]:
    series = cfenv.sample_series(
        scf, {"stabilizer.roll": "float", "stabilizer.pitch": "float"}, SAMPLES)
    return (statistics.fmean(series["stabilizer.roll"]),
            statistics.fmean(series["stabilizer.pitch"]))


def classify(d_roll: float, d_pitch: float) -> tuple[str | None, str, str]:
    """Name the lifted arm from how the attitude changed.

    Returns (axis, lifted_arm, opposite_arm), or (None, reason, "") when the
    tilt is too small or too diagonal to read.
    """
    if max(abs(d_roll), abs(d_pitch)) < MIN_TILT:
        return None, ("That was not enough tilt to read. Lift the arm further "
                      "-- about 30 degrees -- and try again."), ""

    if abs(d_pitch) > abs(d_roll) * DOMINANCE:
        # Positive pitch command -> estimate negative (cflib sends -pitch), so
        # the drone flies toward whichever arm drops when pitch is negative.
        return ("pitch", "BACK", "FRONT") if d_pitch < 0 else ("pitch", "FRONT", "BACK")

    if abs(d_roll) > abs(d_pitch) * DOMINANCE:
        # Roll is sent unchanged: positive roll means the right side is down.
        return ("roll", "LEFT", "RIGHT") if d_roll > 0 else ("roll", "RIGHT", "LEFT")

    return None, ("Both axes moved by similar amounts, so you lifted between\n"
                  "two arms. Lift a single motor arm instead."), ""


def run(uri: str | None = None) -> None:
    """Work out which arm of the drone is the front, by tilting it."""
    with cfenv.session(uri) as scf:
        print("Connected.\n")

        print("1. Put the drone flat on the table and leave it alone.")
        input("   Press Enter: ")
        base_roll, base_pitch = read(scf)
        print(f"   baseline: roll {base_roll:+.1f}, pitch {base_pitch:+.1f}\n")

        print("2. Pick ONE ARM. Lift that motor about 30 degrees and HOLD it.")
        print("   Lift the arm itself, not the corner between two arms.")
        print("   Keep holding while you press Enter.")
        input("   Press Enter while holding: ")
        roll, pitch = read(scf)

        d_roll = roll - base_roll
        d_pitch = pitch - base_pitch
        print(f"   tilted:   roll {d_roll:+.1f}, pitch {d_pitch:+.1f} "
              f"(change from flat)\n")

        axis, lifted, opposite = classify(d_roll, d_pitch)
        if axis is None:
            sys.exit(lifted)        # holds the explanation when unreadable

        trim_flag = "--pitch-trim " if axis == "pitch" else "--roll-trim  "
        print(f"The arm you lifted is the {lifted}.")
        print(f"The opposite arm is the {opposite}.\n")
        print(f"So on the {axis} axis:")
        print(f"  positive {trim_flag} leans toward the {opposite}")
        print(f"  negative {trim_flag} leans toward the {lifted}")

        print("\nMark that arm with tape or a dab of paint -- you will want to\n"
              "know which way it points every time you fly. Fly with it pointing\n"
              "away from you and the drone's left and right match your own.")
        print("\nRun this again lifting an arm at 90 degrees to that one to pin\n"
              "down the other axis.")


if __name__ == "__main__":
    cfenv.run(lambda: typer.run(run))
