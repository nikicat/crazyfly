"""flightlog: the recording's shape, and the page rendered from it."""
from __future__ import annotations

import csv
import json
import re

import pytest

import flight
import flightlog


def test_recording_and_page(tmp_path):
    """Streams share one file, each row its own t and the others' columns blank;
    the page gets the columns as JSON with blanks null, names as categories,
    and uPlot inlined so it stands alone."""
    path = tmp_path / "f.csv"
    with flightlog.Recorder(path, ["roll", "flip", "pm.vbat"]) as recorder:
        recorder.write("cmd", {"roll": 1.23456, "flip": None})
        recorder.write("log", {"ts": 7, "pm.vbat": 3.9})
        recorder.write("cmd", {"roll": True, "flip": "spin"})
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    assert [r["src"] for r in rows] == ["cmd", "log", "cmd"]
    assert rows[0]["roll"] == "1.2346" and rows[0]["pm.vbat"] == "" and rows[0]["ts"] == ""
    assert rows[1]["ts"] == "7" and rows[2]["roll"] == "1" and rows[2]["flip"] == "spin"

    page = flightlog.render(path)
    assert page == path.with_suffix(".html")
    html = page.read_text()
    assert "uPlot (v" in html and "__DATA__" not in html and "__JS__" not in html
    data = json.loads(re.search(r"const DATA = (.*?);\n", html).group(1))
    assert data["columns"]["roll"] == [1.2346, None, 1]
    assert data["columns"]["flip"] == [None, None, 1] and data["cats"] == {"flip": ["spin"]}
    assert "src" not in data["columns"] and len(data["columns"]["t"]) == 3


def test_render_derives_charge_and_unwraps_headings(tmp_path):
    """The page gets a charge column from the log (sag-corrected when the
    firmware drives the motors) and headings continuous across the 0/360 seam,
    with ref_hdg snapped into the same revolution."""
    path = tmp_path / "f.csv"
    with flightlog.Recorder(path, ["hdg", "ref_hdg", "pm.vbat", "stabilizer.thrust"]) as rec:
        rec.write("log", {"pm.vbat": 3.75, "stabilizer.thrust": 0})
        rec.write("log", {"pm.vbat": 3.75 - flight.BATTERY_SAG, "stabilizer.thrust": 40000})
        for hdg in (350.0, 358.0, 2.0, 10.0, 350.0):
            rec.write("cmd", {"hdg": hdg, "ref_hdg": 5.0})
    html = flightlog.render(path).read_text()
    data = json.loads(re.search(r"const DATA = (.*?);\n", html).group(1))

    assert data["columns"]["charge"][0] == pytest.approx(flight.charge(3.75, False), abs=0.1)
    # smoothing mixes both samples; the sag is added back while airborne
    smoothed = (3.75 + 3.75 - flight.BATTERY_SAG) / 2
    assert data["columns"]["charge"][1] == pytest.approx(flight.charge(smoothed, True), abs=0.1)
    assert data["columns"]["hdg"][2:] == [350.0, 358.0, 362.0, 370.0, 350.0]
    assert data["columns"]["ref_hdg"][2:] == [365.0] * 5      # one revolution up, with hdg
