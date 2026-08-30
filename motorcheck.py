#!/usr/bin/env python3
"""Find the pitch sign on the ground, by watching which motor slows down.

Working the trim direction out in flight needs the drone to travel far enough
to see which way it went, which in a small room means hitting a wall before
the measurement is any good. This asks the same question without lifting off.

At a thrust too low to fly, the drone still runs its attitude controller. Ask
it for a pitch it cannot reach and it drives one motor up and the opposite one
down until the losing one stops outright. A stopped propeller is obvious;
which of four spinning ones is slightly slower is not, so this drives one to a
standstill rather than asking anyone to compare speeds.

The angle demanded is worked out from the response, not guessed. Base thrust
stays put: brushed motors need more duty to break away from rest than to keep
turning, so lowering it to stop one motor stops all four instead.

It also reads the frame geometry from the same measurement. In "+" the arms
point front, back, left and right, so pitch is one motor against one and the
other two barely stir. In "X" the arms point at the diagonals, so pitch moves
all four. Ranking the responses separates them: the third-strongest motor is
near silent on a plus frame and as loud as the first on an X.

That is worth knowing before trimming anything. A mixer built for plus flying
an X frame still flies -- it answers a pitch command with a 45 degree diagonal,
which reads as untrimmable drift rather than as a configuration error.

Then, whichever the frame:

  * the motor (or pair) that stops lets its side drop
  * the drone then accelerates toward that side

Which arm that turns out to be is a fact about how the IMU is mounted, not
about the paint on the frame. If it is the left or right arm, then what cflib
calls pitch moves this drone sideways, and a front-back drift is a roll
problem -- worth knowing before trimming the wrong axis.

SAFETY: the propellers spin. Put the drone on a flat surface, hands clear. It
cannot lift at this thrust but it may skitter, so hold it by the battery.

If frame.json says which arm each motor is on, the arm that stopped is named
from the motor that reached zero instead of asking you to spot it.

  uv run motorcheck.py
"""
from __future__ import annotations

from typing import NamedTuple

import cfenv
from flight import DT, Interruptible, load_frame, stop_motors, ticks
from signals import best_fit

PROBE_PITCH = 20.0       # degrees; deliberately unreachable, to saturate output
PHASE_SECONDS = 0.8
CYCLES = 3
HOLD_SECONDS = 4.0       # steady lean, long enough to watch the props
DEFAULT_THRUST = 15000   # high enough that every motor reliably starts
MIN_CORRELATION = 0.5

# The losing motor floors at (thrust - differential). Getting it to zero by
# lowering the base thrust starves every motor at once: brushed motors need
# more duty to break away from rest than to keep turning, so below roughly
# 10000 none of them start and there is nothing to watch. Raise the commanded
# angle instead -- the differential grows with the error, so the losing motor
# reaches zero while the others merely spin faster.
PITCH_MARGIN = 1.3       # overshoot the angle that would just reach zero
MAX_PROBE_PITCH = 70.0   # beyond this the firmware clamps the setpoint anyway
MAX_ATTEMPTS = 3

MOTORS: tuple[str, ...] = ("motor.m1", "motor.m2", "motor.m3", "motor.m4")
VARIABLES = dict.fromkeys(MOTORS, "int32_t")

# Frame geometry, read off how many motors answer a single-axis command.
#
# Plus puts one motor at the front and one at the back, so pitch moves exactly
# two of them and the other two barely stir. X puts two motors at each end, so
# pitch moves all four about equally. Ranking the gains separates the cases
# cleanly: compare the third-strongest against the strongest, which is near
# zero on a plus frame and near one on an X.
#
# This matters more than it sounds. A mixer built for plus flying an X frame
# still flies -- it just answers a pitch command with a 45 degree diagonal,
# which reads as untrimmable drift rather than as a configuration error.
PLUS_RATIO = 0.25        # third motor this quiet means only two are involved
X_RATIO = 0.50           # third motor this loud means all four are
FRAME_NAMES = {"plus": "PLUS (+)", "x": "X"}

ARM_NAMES = {"f": "FRONT", "b": "BACK", "l": "LEFT", "r": "RIGHT"}
OPPOSITES = {"f": "b", "b": "f", "l": "r", "r": "l"}


class MotorResponse(NamedTuple):
    """How one motor answered the commanded angle in step 1."""

    gain: float          # motor output per degree commanded, from the lag-corrected fit
    correlation: float
    low: float           # output range it covered
    high: float


Report = dict[str, MotorResponse]


class Frame(NamedTuple):
    kind: str | None     # "plus", "x", or None when the reading cannot be trusted
    ranked: list[str]    # motors by response, strongest first
    ratio: float         # third-strongest response over the strongest

    @property
    def movers(self) -> int:
        """Motors at each end of the axis: one on a plus frame, a pair on an X."""
        return 1 if self.kind == "plus" else 2


def detect_frame(report: Report) -> Frame:
    """Read the frame geometry off the ranked responses."""
    ranked = sorted(MOTORS, key=lambda m: abs(report[m].gain), reverse=True)
    strongest = abs(report[ranked[0]].gain)
    if strongest <= 0:
        return Frame(None, ranked, 0.0)

    ratio = abs(report[ranked[2]].gain) / strongest
    if ratio < PLUS_RATIO:
        return Frame("plus", ranked, ratio)
    if ratio > X_RATIO:
        return Frame("x", ranked, ratio)
    return Frame(None, ranked, ratio)


def axis_ends(report: Report, movers: int) -> tuple[list[str], list[str]]:
    """(motors driven down by a +angle, motors driven down by a -angle)."""
    by_gain = sorted(MOTORS, key=lambda m: report[m].gain)
    return by_gain[:movers], by_gain[-movers:]


def drive(scf, thrust: int, schedule, axis: str = "pitch"
          ) -> tuple[list, dict[str, list]]:
    """Command each (angle, duration) in turn, recording motor outputs."""
    cf = scf.cf
    commands: list[tuple[float, float]] = []
    series: dict[str, list[tuple[float, float]]] = {m: [] for m in MOTORS}
    taken = 0

    with cfenv.record_log(scf, VARIABLES, period_ms=50) as raw, \
            Interruptible() as interrupt:
        try:
            cf.commander.send_setpoint(0, 0, 0, 0)
            for angle, duration in schedule:
                for _frac, now in ticks(duration):
                    if interrupt.requested:
                        raise KeyboardInterrupt
                    roll, pitch = (angle, 0) if axis == "roll" else (0, angle)
                    cf.commander.send_setpoint(roll, pitch, 0, thrust)
                    commands.append((now, angle))
                    while taken < len(raw):
                        for motor in MOTORS:
                            series[motor].append((now, float(raw[taken][motor])))
                        taken += 1
        finally:
            stop_motors(cf, from_thrust=thrust, dt=DT)

    return commands, series


def summarise(commands, series) -> Report:
    """Per-motor gain against the commanded angle, plus the range it covered."""
    report = {}
    for motor, samples in series.items():
        if len(samples) < 8:
            continue
        _lag, gain, correlation = best_fit(commands, samples)
        values = [value for _t, value in samples]
        report[motor] = MotorResponse(gain, correlation, min(values), max(values))
    return report


def probe_axis(scf, thrust: int, pitch: float, axis: str) -> Report | None:
    """Step 1: alternate the angle, and show how each motor answered."""
    print(f"\nStep 1: alternating {pitch:+.0f} deg of {axis}, {CYCLES} cycles ...")
    schedule = [(+pitch, PHASE_SECONDS), (-pitch, PHASE_SECONDS)] * CYCLES
    report = summarise(*drive(scf, thrust, schedule, axis))

    if len(report) < 4:
        print("Not all motors reported. Power-cycle and retry.")
        return None

    print("\n  motor      gain   correlation      range")
    for motor in MOTORS:
        row = report[motor]
        print(f"  {motor}  {row.gain:+8.1f}     {row.correlation:+.2f}"
              f"      {row.low:.0f} - {row.high:.0f}")

    responding = [m for m in MOTORS if abs(report[m].correlation) >= MIN_CORRELATION]
    if len(responding) < 2:
        print("\nThe motors do not track the command. Raise --thrust so the\n"
              "controller has room to push, and retry.")
        return None
    return report


def describe_frame(report: Report, axis: str) -> Frame | None:
    """Say what the responses reveal about the frame; None when they do not."""
    frame = detect_frame(report)
    other = "roll" if axis == "pitch" else "pitch"
    third = abs(report[frame.ranked[2]].gain)
    first = abs(report[frame.ranked[0]].gain)

    print(f"\nFrame: {FRAME_NAMES.get(frame.kind, 'UNCLEAR')}")
    print(f"  third-strongest motor moves {frame.ratio * 100:.0f}% as much as the "
          f"strongest ({third:.0f} against {first:.0f})")

    if frame.kind is None:
        print("\nThat is between the two patterns, so the frame cannot be\n"
              "read from it. Raise --pitch for a cleaner separation, and\n"
              "check the drone is sitting flat and still.")
        return None
    if frame.kind == "plus":
        print(f"  two motors answer {axis}, so the arms point front, back,\n"
              f"  left and right. {frame.ranked[0]} and {frame.ranked[1]} are the {axis}\n"
              f"  axis; {frame.ranked[2]} and {frame.ranked[3]} are {other}.")
    else:
        print(f"  all four motors answer {axis}, so the arms point at the\n"
              f"  diagonals. Each end of the {axis} axis is a PAIR.")
    return frame


def hold_angle(report: Report, thrust: int, pitch: float, watched: list[str],
               noun: str) -> float:
    """The angle to hold in step 2, predicted from step 1 to bottom the motors out.

    Step 1 already showed how far the differential moves per degree, so start
    from an angle predicted to reach zero rather than from the probing angle,
    which demonstrably does not.
    """
    step1_reach = thrust - min(report[m].low for m in watched)
    scale = thrust / max(1.0, step1_reach)
    angle = min(MAX_PROBE_PITCH, pitch * scale * PITCH_MARGIN)
    print(f"Step 1 moved the motors {step1_reach:.0f} of the {thrust} "
          f"needed to reach zero,\nso holding {angle:.0f} deg rather "
          f"than {pitch:.0f}. Base thrust stays at {thrust} so "
          f"every\nmotor keeps turning; only the losing {noun} should stop.\n")
    return angle


def stop_end(scf, thrust: int, axis: str, angle: float, motors: list[str]) -> float:
    """Hold `angle` until `motors` stop, demanding more each time they do not.

    Returns the lowest output the slowest of them reached: zero means it stopped.
    """
    sign = 1.0 if angle > 0 else -1.0
    stops = " and ".join(motors)
    plural = len(motors) > 1
    attempt = 0
    while True:
        attempt += 1
        print(f"  Holding {angle:+.0f} deg for {HOLD_SECONDS:.0f}s -- "
              f"watch for the propeller{'s' if plural else ''} that STOP{'' if plural else 'S'}.")
        input("  Press Enter when ready: ")
        _c, held = drive(scf, thrust, [(angle, HOLD_SECONDS)], axis)
        worst = max(min((v for _t, v in held[m]), default=0.0) for m in motors)
        if worst <= 0:
            print(f"  ({stops} reached zero -- "
                  f"{'those propellers' if plural else 'that propeller'} stopped.)")
            return worst

        harder = min(MAX_PROBE_PITCH,
                     abs(angle) * thrust / max(1.0, thrust - worst) * PITCH_MARGIN)
        if harder <= abs(angle) * 1.05 or attempt == MAX_ATTEMPTS:
            print(f"  ({stops} bottomed out at {worst:.0f} and will not "
                  f"reach zero.\n   Watch for the slowest instead.)")
            return worst

        print(f"  ({stops} only reached {worst:.0f}, still spinning. "
              f"Demanding\n   {harder:.0f} deg instead -- watch again.)")
        angle = sign * harder


def which_arm(frame: Frame, layout: dict[str, str] | None, motors: list[str],
              worst: float) -> str:
    """The arm that stopped, from frame.json when it can say, else from you."""
    if layout and len(motors) == 1 and worst <= 0:
        arm = layout[motors[0]]
        print(f"  (frame.json: {motors[0]} is the {arm} arm.)")
        return arm[0]
    prompt = ("  Which arm stopped?  " if frame.kind == "plus"
              else "  Which SIDE stopped (the two adjacent arms)?  ")
    return input(prompt + "[f] front  [b] back  [l] left  [r] right: ").strip().lower()[:1]


def conclude(axis: str, first: str, second: str) -> None:
    """Turn the two arm letters into the answer, and what it means for trim."""
    names = ARM_NAMES
    if OPPOSITES[first] != second:
        print("\nThose two arms are not opposite each other, and the pitch\n"
              "pair has to sit across the frame. One reading was misread --\n"
              "run it again and watch for the propeller that stops.")
        return

    print("\nConsistent: the two arms are opposite, as the pitch axis must be.")

    # Whichever arm drops on a positive pitch command is the direction a
    # positive command sends the drone. That is the whole answer; whether
    # it matches the arm you call the front is a separate question.
    print(f"\n  positive {axis}  ->  drops the {names[first]} arm  ->  "
          f"drone accelerates {names[first]}")
    print(f"  negative {axis}  ->  drops the {names[second]} arm  ->  "
          f"drone accelerates {names[second]}")

    if first in ("l", "r"):
        print(f"\nSo what cflib calls pitch moves this drone along its\n"
              f"{names[first]}-{names[second]} axis, not front-back. The IMU is\n"
              f"mounted 90 degrees from the arm you call the front, which is\n"
              f"normal -- the frame is symmetric and the firmware picks the\n"
              f"axis, not the paint.\n")
        print("Two consequences worth knowing:")
        print(f"  * fly with the {names[first]} arm leading and the controls\n"
              f"    behave conventionally, pitch forward and roll sideways")
        print("  * a drift along your front-back axis is a ROLL problem on\n"
              "    this drone, so pitch trim was never going to fix it")
        print("\nThat would explain trimming pitch making no difference.")
    else:
        print(f"\nSo a positive pitch command flies the drone {names[first]}.")
        if first == "f":
            print("A backward drift is corrected by INCREASING pitch trim.")
            print('\n  hoptest CORRECTIONS should read  "b": ("pitch", +1)')
        else:
            print("A backward drift is corrected by DECREASING pitch trim.")
            print('\n  hoptest CORRECTIONS should read  "b": ("pitch", -1)')

    print("\nTell me the result and I will set it from this measurement.")


def run(
    thrust: int = DEFAULT_THRUST,
    pitch: float = PROBE_PITCH,
    axis: str = "pitch",
    uri: str | None = None,
) -> None:
    """Find an axis's sign on the ground, by which motor stops."""
    if axis not in ("pitch", "roll"):
        raise SystemExit("axis must be pitch or roll")

    with cfenv.session(uri) as scf:
        print("Connected.\n")

        print("The propellers will spin, at a thrust far too low to lift.")
        print("Put the drone on a flat surface, props clear, hands away.")
        print("Ctrl-C stops the motors at any point.\n")
        if input("Type 'go' to start: ").strip().lower() != "go":
            print("Nothing done.")
            return

        report = probe_axis(scf, thrust, pitch, axis)
        if report is None:
            return
        frame = describe_frame(report, axis)
        if frame is None:
            return

        low_on_plus, low_on_minus = axis_ends(report, frame.movers)
        if report[low_on_plus[0]].gain * report[low_on_minus[0]].gain > 0:
            print("\nThe motors all moved the same way, which is thrust rather\n"
                  f"than {axis}. Raise --{axis} or --thrust and retry.")
            return

        # Step 2: stop each end of the axis in turn. Which propeller STOPS is
        # something you can actually see; which is merely slower is not.
        # Asking about both ends cross-checks the answer, since the pitch pair
        # must sit across the frame from one another.
        noun = "arm" if frame.kind == "plus" else "pair of arms"
        print(f"\nStep 2: identifying the two ends of the {axis} axis.")
        print("Answers are relative to the drone's own front, as you know it.\n")
        angle = hold_angle(report, thrust, pitch, low_on_plus + low_on_minus, noun)

        layout = load_frame()
        answers = []
        for sign, motors in ((+1.0, low_on_plus), (-1.0, low_on_minus)):
            worst = stop_end(scf, thrust, axis, sign * angle, motors)
            answer = which_arm(frame, layout, motors, worst)
            if answer not in ARM_NAMES:
                print("\n  Not a recognised answer. Re-run and watch again.")
                return
            plural = len(motors) > 1
            print(f"  {' and '.join(motors)} {'are' if plural else 'is'} "
                  f"the {ARM_NAMES[answer]} {'arms' if plural else 'arm'}\n")
            answers.append(answer)

        conclude(axis, *answers)


if __name__ == "__main__":
    cfenv.cli(run)
