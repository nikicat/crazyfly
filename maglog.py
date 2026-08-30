#!/usr/bin/env python3
"""Record the magnetometer to a CSV, with attitude, thrust and battery beside it.

  uv run cf.py mag                       30 s into mag.csv
  uv run cf.py mag --seconds 60 --out spin.csv
  uv run cf.py mag --seconds 0 --out /dev/stdout | ...   stream until Ctrl-C

Rows are written as they arrive and messages go to stderr, so the file is
usable however the run ends.

For a hard-iron calibration, turn the drone slowly through every orientation
while it records; the offset per axis is then (min + max) / 2 from the summary
printed at the end, and --save keeps it in mag.json for teleop's heading.
Motor current bends the field too, which is why thrust is in the file: compare
readings at rest and under power before trusting a heading.
"""
from __future__ import annotations

import csv
import statistics
import sys
import time
from pathlib import Path

import cfenv
from flight import MAG_FILE, save_mag_offset

# One CRTP log packet carries 26 bytes: the field as floats, the rest as FP16.
VARIABLES = {
    "mag.x": "float", "mag.y": "float", "mag.z": "float",
    "stabilizer.roll": "FP16", "stabilizer.pitch": "FP16", "stabilizer.yaw": "FP16",
    "stabilizer.thrust": "FP16", "pm.vbat": "FP16",
}
PERIOD_MS = 50
MIN_SWING = 0.5          # gauss; a full turn swings each axis by about twice Earth's field


def say(*args) -> None:
    """Messages go to stderr, keeping stdout for the data."""
    print(*args, file=sys.stderr, flush=True)


def summarise(field: dict[str, list[float]], save: bool) -> None:
    """Per-axis range and hard-iron offset; saved to mag.json when the turn was full."""
    count = len(field["mag.x"])
    say(f"{count} samples written.")
    if not count:
        return
    for axis, values in field.items():
        say(f"  {axis}: min {min(values):+8.3f}  max {max(values):+8.3f}  "
            f"mean {statistics.fmean(values):+8.3f}  "
            f"hard-iron offset {(min(values) + max(values)) / 2:+8.3f}")
    if not save:
        return
    swing = min(max(v) - min(v) for v in field.values())
    if swing < MIN_SWING:
        say(f"Not saved: an axis swung only {swing:.2f} G, so the drone was not "
            f"turned through every orientation. Need {MIN_SWING} G on each.")
        return
    save_mag_offset(*((min(v) + max(v)) / 2 for v in field.values()))
    say(f"Offset saved to {MAG_FILE.name}; teleop will show a heading.")


def run(seconds: float = 30.0, out: Path = Path("mag.csv"), save: bool = False,
        uri: str | None = None) -> None:
    """Record the magnetometer to a CSV, with attitude, thrust and battery."""
    field: dict[str, list[float]] = {axis: [] for axis in ("mag.x", "mag.y", "mag.z")}
    with cfenv.session(uri) as scf, out.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["t", *VARIABLES])
        writer.writeheader()
        f.flush()
        start = time.time()
        say(f"Recording {f'{seconds:.0f} s' if seconds > 0 else 'until Ctrl-C'} "
            f"at {1000 // PERIOD_MS} Hz into {out} ...")

        def collect(data) -> bool:
            writer.writerow({"t": round(time.time() - start, 3), **data})
            f.flush()
            for axis in field:
                field[axis].append(data[axis])
            return 0 < seconds <= time.time() - start

        try:
            cfenv.stream_log(scf, VARIABLES, collect, period_ms=PERIOD_MS,
                             timeout=seconds + 5 if seconds > 0 else None)
        finally:
            summarise(field, save)


if __name__ == "__main__":
    cfenv.cli(run)
