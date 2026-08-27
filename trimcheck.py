#!/usr/bin/env python3
"""Measure the resting attitude estimate to work out why the drone drifts.

Put the drone on a flat surface, leave it still, and run this. It reads what
the stabilizer believes the attitude is while the drone is definitely not
moving, which separates the causes of drift:

  * gyro not settled   -> yaw creeps while sitting still; redo the power-on
                          calibration (a 1.0 zeroes gyro bias at switch-on and
                          must be still and level then)
  * attitude offset    -> the estimator holds a false level, so the controller
                          flies the drone to that tilt and it slides away
  * neither            -> mechanical: props, bent motor mount, battery position

A single reading cannot tell a sensor offset from a sloped table. Use
--rotate to remove that doubt: measure, spin the drone 180 degrees on the spot,
measure again. Surface slope reverses sign between the two, a sensor offset
does not, so averaging isolates the offset and differencing reveals the slope.
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from collections import deque

import cfenv

SAMPLES = 50          # at 100 ms -> about 5 seconds
STILL_DEG = 0.15      # per-sample spread below this counts as "not moving"
LEVEL_DEG = 0.3       # attitude within this of zero is close enough
YAW_DRIFT_DEG = 2.0   # yaw creep over the window that means gyro is unsettled

# After the drone is moved or rotated the attitude filter has to re-converge,
# and while it does the estimate ramps rather than sits still. Sampling through
# that ramp looks exactly like a drone being jostled, so wait for the estimate
# to go quiet before starting to record.
STEADY_WINDOW = 20    # samples (2 s) that must be quiet before recording
SETTLE_TIMEOUT = 40.0  # seconds; give up waiting and record anyway


class Reading:
    def __init__(self, roll: list[float], pitch: list[float], yaw: list[float]):
        self.roll = statistics.fmean(roll)
        self.pitch = statistics.fmean(pitch)
        self.roll_sd = statistics.pstdev(roll)
        self.pitch_sd = statistics.pstdev(pitch)
        self.yaw_drift = yaw[-1] - yaw[0]

    @property
    def steady(self) -> bool:
        return max(self.roll_sd, self.pitch_sd) <= STILL_DEG

    def show(self, label: str) -> None:
        print(f"  {label}")
        print(f"    roll      {self.roll:+7.2f} deg   (spread {self.roll_sd:.2f})")
        print(f"    pitch     {self.pitch:+7.2f} deg   (spread {self.pitch_sd:.2f})")
        print(f"    yaw drift {self.yaw_drift:+7.2f} deg over {SAMPLES / 10:.0f} s")


VARIABLES = {
    "stabilizer.roll": "float",
    "stabilizer.pitch": "float",
    "stabilizer.yaw": "float",
}


def measure(scf) -> Reading:
    roll: list[float] = []
    pitch: list[float] = []
    yaw: list[float] = []
    window: deque[tuple[float, float]] = deque(maxlen=STEADY_WINDOW)
    state = {"settled": False, "started": time.time()}

    def handle(data) -> bool:
        r = data["stabilizer.roll"]
        p = data["stabilizer.pitch"]

        if not state["settled"]:
            window.append((r, p))
            if len(window) == window.maxlen:
                spread = max(statistics.pstdev([w[0] for w in window]),
                             statistics.pstdev([w[1] for w in window]))
                if spread <= STILL_DEG:
                    state["settled"] = True
            waited = time.time() - state["started"]
            if not state["settled"] and waited > SETTLE_TIMEOUT:
                print("\r    estimate never went fully quiet; recording anyway")
                state["settled"] = True
            if not state["settled"]:
                print(f"\r    waiting for the estimate to settle ... {waited:4.1f}s",
                      end="", flush=True)
                return False
            print("\r" + " " * 50 + "\r", end="")

        roll.append(r)
        pitch.append(p)
        yaw.append(data["stabilizer.yaw"])
        print(f"\r    sampling {len(roll)}/{SAMPLES} ...", end="", flush=True)
        return len(roll) >= SAMPLES

    # Allow for the settle wait plus the recording itself.
    cfenv.stream_log(scf, VARIABLES, handle,
                     timeout=SETTLE_TIMEOUT + SAMPLES * 0.1 * 3 + 10)

    print("\r" + " " * 34 + "\r", end="")
    return Reading(roll, pitch, yaw)


def check_steady(reading: Reading) -> None:
    """Stop only if the estimate is genuinely unsettled, and say by how much."""
    if reading.steady:
        return
    spread = max(reading.roll_sd, reading.pitch_sd)
    sys.exit(
        f"\nThe attitude estimate is still moving: spread {spread:.2f} deg, "
        f"above the {STILL_DEG} deg limit.\n"
        "If the drone really is sitting still, the filter had not finished\n"
        "re-converging. Leave it untouched a little longer and run it again;\n"
        "a soft or springy surface will also keep it moving imperceptibly."
    )


def advise(roll_bias: float, pitch_bias: float, yaw_drift: float,
           slope: tuple[float, float] | None) -> None:
    print()
    if abs(yaw_drift) > YAW_DRIFT_DEG:
        print(f"Yaw crept {yaw_drift:+.1f} deg while the drone sat still, so the\n"
              "gyro bias has not settled. Fix that before anything else:\n"
              "  switch off, set it level and still, switch on, wait ~5 s.\n")
        return

    print("Gyro is settled (yaw is stable while still).")

    if slope is not None:
        print(f"Surface slope removed by the 180-degree flip: "
              f"roll {slope[0]:+.2f}, pitch {slope[1]:+.2f} deg.")

    if max(abs(roll_bias), abs(pitch_bias)) <= LEVEL_DEG:
        print("Attitude estimate is level, so the sensors are fine.\n"
              "\nA drift with a level estimate is mechanical. Check, in order:\n"
              "  - props fully seated, correct type on the correct motor, undamaged\n"
              "  - motor mounts straight; all four arms in one plane (sight along it)\n"
              "  - battery centred, cable not pulling the frame to one side\n"
              "  - motors all spin up smoothly; a tired motor drops that corner\n")
        return

    print(f"Attitude estimate is offset: roll {roll_bias:+.2f}, "
          f"pitch {pitch_bias:+.2f} deg.")
    print("The controller flies to this false level, so the drone holds a real\n"
          "tilt and accelerates that way. Cancel it with trim:\n")
    # The controller settles where the estimate equals the setpoint, so holding
    # the drone truly level needs a setpoint equal to the estimator's bias.
    #
    # Roll passes through unchanged, so roll trim is the bias as measured.
    # Pitch does not: cflib transmits -pitch (see Commander.send_setpoint),
    # so the argument must be negated to land on the same estimator value.
    print(f"    uv run teleop.py --roll-trim {roll_bias:+.1f} "
          f"--pitch-trim {-pitch_bias:+.1f}")
    print("\n  or adjust live in teleop with [ ] and ; ' -- it saves on exit.")
    print("  Trim cancels a steady lean. If the drone instead accelerates away\n"
          "  harder and harder, the cause is mechanical -- see the checks above.")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--rotate", action="store_true",
                   help="take a second reading rotated 180 deg to cancel surface slope")
    p.add_argument("--uri", default=None)
    args = p.parse_args()

    cfenv.init()
    uri = cfenv.resolve_uri(args.uri)

    print(f"Connecting to {uri} ...")
    with cfenv.connect(uri) as scf:
        print("Connected.\n")

        print("Keep the drone still on a flat surface.")
        first = measure(scf)
        first.show("as placed:")
        check_steady(first)

        if not args.rotate:
            advise(first.roll, first.pitch, first.yaw_drift, slope=None)
            print("\nIf you are unsure the surface is level, re-run with --rotate\n"
                  "to measure the sensor offset independently of it.")
            return

        print("\nNow rotate the drone 180 degrees on the spot -- same place on the\n"
              "same surface, just facing the opposite way. Keep it flat.")
        input("Press Enter when it is settled: ")
        second = measure(scf)
        second.show("rotated 180:")
        check_steady(second)

        # bias stays with the drone, slope reverses with it
        roll_bias = (first.roll + second.roll) / 2
        pitch_bias = (first.pitch + second.pitch) / 2
        slope = ((first.roll - second.roll) / 2, (first.pitch - second.pitch) / 2)

        print(f"\n  sensor offset : roll {roll_bias:+.2f}, pitch {pitch_bias:+.2f} deg")
        print(f"  surface slope : roll {slope[0]:+.2f}, pitch {slope[1]:+.2f} deg")

        worst_drift = max(first.yaw_drift, second.yaw_drift, key=abs)
        advise(roll_bias, pitch_bias, worst_drift, slope=slope)


if __name__ == "__main__":
    cfenv.run(main)
