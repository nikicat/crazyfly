#!/usr/bin/env python3
"""Record the magnetometer to a CSV, with attitude, thrust and battery beside it.

  uv run cf.py mag                       30 s into mag.csv
  uv run cf.py mag --seconds 60 --out spin.csv
  uv run cf.py mag --seconds 0 --out /dev/stdout | ...   stream until Ctrl-C

Rows are written as they arrive and messages go to stderr, so the file is
usable however the run ends.

For a hard-iron calibration, turn the drone slowly through every orientation
while it records; the offset per axis is then (min + max) / 2 from the summary
printed at the end. Motor current bends the field too, which is why thrust is
in the file: compare readings at rest and under power before trusting a heading.
"""
from __future__ import annotations

import csv
import statistics
import sys
import time
from pathlib import Path

import typer

import cfenv

# One CRTP log packet carries 26 bytes: the field as floats, the rest as FP16.
VARIABLES = {
    "mag.x": "float", "mag.y": "float", "mag.z": "float",
    "stabilizer.roll": "FP16", "stabilizer.pitch": "FP16", "stabilizer.yaw": "FP16",
    "stabilizer.thrust": "FP16", "pm.vbat": "FP16",
}
PERIOD_MS = 50


def run(seconds: float = 30.0, out: Path = Path("mag.csv"), uri: str | None = None) -> None:
    """Record the magnetometer to a CSV, with attitude, thrust and battery."""
    cfenv.init()
    uri = cfenv.resolve_uri(uri)
    say = lambda *a: print(*a, file=sys.stderr, flush=True)     # noqa: E731 - keep stdout for data

    say(f"Connecting to {uri} ...")
    field: dict[str, list[float]] = {axis: [] for axis in ("mag.x", "mag.y", "mag.z")}
    with cfenv.connect(uri) as scf, out.open("w", newline="") as f:
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
            count = len(field["mag.x"])
            say(f"{count} samples written.")
            if count:
                for axis, values in field.items():
                    say(f"  {axis}: min {min(values):+8.3f}  max {max(values):+8.3f}  "
                        f"mean {statistics.fmean(values):+8.3f}  "
                        f"hard-iron offset {(min(values) + max(values)) / 2:+8.3f}")


if __name__ == "__main__":
    cfenv.run(lambda: typer.run(run))
