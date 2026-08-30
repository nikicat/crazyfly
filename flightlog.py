#!/usr/bin/env python3
"""Record a flight to a CSV as it happens, and render it as an interactive page.

teleop records every flight into data/flights/<stamp>.csv and writes the page
beside it on exit; this renders one again, the newest by default:

  uv run cf.py plot
  uv run cf.py plot data/flights/2026-08-30T21-34-12.csv

Rows come from several streams -- `src` names which -- each at its own rate,
so every row carries its own `t` (seconds since the recording started) and
leaves the other streams' columns blank. Log rows also carry `ts`, the
firmware's millisecond clock, for alignment finer than the tick they were
drained on. Rows are flushed as they are written, so the file is whole
however the flight ends.

The page is uPlot -- the engine behind Grafana's time-series panel -- inlined
from the vendored uplot.min.js/css, so it works offline: one cursor across
every panel with the legend reading the values under it, drag to zoom them all
together, double-click to zoom out, click a legend entry to hide a series.
"""
from __future__ import annotations

import csv
import json
import sys
import time
from collections import deque
from pathlib import Path

import typer

from flight import DATA, charge, rest_voltage

FLIGHTS = DATA / "flights"
HERE = Path(__file__).parent
TEMPLATE = HERE / "flightlog.html"
UPLOT_JS = HERE / "uplot.min.js"
UPLOT_CSS = HERE / "uplot.min.css"


def new_path() -> Path:
    return FLIGHTS / time.strftime("%Y-%m-%dT%H-%M-%S.csv")


def newest() -> Path:
    recordings = sorted(FLIGHTS.glob("*.csv"), key=lambda p: p.stat().st_mtime)
    if not recordings:
        sys.exit(f"No recordings in {FLIGHTS}; fly with teleop first.")
    return recordings[-1]


def tidy(value):
    """A CSV cell: floats to four places, bools as 0/1, None blank."""
    if value is None:
        return ""
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, float):
        return round(value, 4)
    return value


class Recorder:
    """One flight's CSV, written a row at a time as the flight happens."""

    def __init__(self, path: Path, columns: list[str]) -> None:
        self.path = path
        self.rows = 0
        path.parent.mkdir(parents=True, exist_ok=True)
        self._file = path.open("w", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=["t", "src", "ts", *columns],
                                      restval="")
        self._writer.writeheader()
        self._start = time.time()

    def write(self, src: str, row: dict) -> None:
        self._writer.writerow({"t": round(time.time() - self._start, 3), "src": src,
                               **{name: tidy(value) for name, value in row.items()}})
        self._file.flush()
        self.rows += 1

    def close(self) -> None:
        self._file.close()

    def __enter__(self) -> Recorder:
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def load(path: Path) -> tuple[dict[str, list], dict[str, list[str]]]:
    """The CSV as columns for the page: blanks are null, strings become categories.

    A column of names -- the flip phase -- is plotted as a step series of
    1-based indices into its sorted category list, which the page's legend
    maps back to the name.
    """
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        names = [name for name in reader.fieldnames or [] if name != "src"]
    columns: dict[str, list] = {}
    categories: dict[str, list[str]] = {}
    for name in names:
        values: list = []
        for row in rows:
            cell = row[name]
            if cell == "":
                values.append(None)
                continue
            try:
                values.append(float(cell))
            except ValueError:
                values.append(cell)
        if any(isinstance(v, str) for v in values):
            categories[name] = sorted({str(v) for v in values if v is not None})
            values = [None if v is None else categories[name].index(str(v)) + 1
                      for v in values]
        columns[name] = values
    return columns, categories


def add_charge(columns: dict[str, list]) -> None:
    """A percent-charge column derived from the log, so old recordings get it too.

    The same estimate as teleop's status line: each sample sag-corrected by
    its own thrust state, then smoothed over the last 20 log samples.
    """
    vbats = columns.get("pm.vbat")
    if not vbats or not any(v is not None for v in vbats):
        return
    thrusts = columns.get("stabilizer.thrust") or [None] * len(vbats)
    recent: deque[float] = deque(maxlen=20)
    series: list[float | None] = []
    for vbat, thrust in zip(vbats, thrusts, strict=True):
        if vbat is None:
            series.append(None)
            continue
        recent.append(rest_voltage(vbat, bool(thrust)))
        series.append(round(charge(sum(recent) / len(recent)), 1))
    columns["charge"] = series


def unwrap_headings(columns: dict[str, list]) -> None:
    """Plot hdg continuously past the 0/360 seam, and ref_hdg in its revolution.

    A flight that crosses the seam draws a full-height jump for a one-degree
    turn; adding +-360 at each crossing keeps the line continuous (the page
    shows the compass value mod 360 in the legend). ref_hdg is snapped to
    whichever revolution puts it nearest the heading it is steering.
    """
    hdgs, refs = columns.get("hdg"), columns.get("ref_hdg")
    if not hdgs:
        return
    offset, previous = 0.0, None
    for i, value in enumerate(hdgs):
        if value is not None:
            if previous is not None:
                step = value - previous
                offset += -360.0 if step > 180 else 360.0 if step < -180 else 0.0
            previous = value
            hdgs[i] = value + offset
        gone = previous + offset if previous is not None else None
        anchor = hdgs[i] if value is not None else gone
        if refs and refs[i] is not None and anchor is not None:
            refs[i] += 360.0 * round((anchor - refs[i]) / 360.0)


def render(path: Path) -> Path:
    """Write the page for `path` beside it, and return where."""
    columns, categories = load(path)
    add_charge(columns)
    unwrap_headings(columns)
    payload = {"name": path.stem, "columns": columns, "cats": categories}
    page = (TEMPLATE.read_text()
            .replace("__DATA__", json.dumps(payload).replace("</", "<\\/"))
            .replace("__CSS__", UPLOT_CSS.read_text())
            .replace("__JS__", UPLOT_JS.read_text()))
    out = path.with_suffix(".html")
    out.write_text(page)
    return out


def run(path: Path | None = None) -> None:
    """Render a flight recording as an interactive page; the newest by default."""
    path = newest() if path is None else path
    print(f"{path} -> {render(path)}")


if __name__ == "__main__":
    typer.run(run)
