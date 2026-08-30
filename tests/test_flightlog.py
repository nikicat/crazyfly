"""flightlog: the recording's shape, and the page rendered from it."""
from __future__ import annotations

import csv
import json
import re

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
