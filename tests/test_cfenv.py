"""cfenv: telemetry must never hang."""
from __future__ import annotations

import time

import pytest

import cfenv


class FakeLogConfig:
    """Stands in for cflib's LogConfig; never delivers data."""

    def __init__(self, name=None, period_in_ms=100):
        self.valid = True
        self.data_received_cb = _CallbackList()
        self.error_cb = _CallbackList()
        self.started = False

    def add_variable(self, name, ctype=None):
        pass

    def start(self):
        self.started = True

    def stop(self):
        self.started = False

    def delete(self):
        pass


class _CallbackList:
    def __init__(self):
        self.callbacks = []

    def add_callback(self, cb):
        self.callbacks.append(cb)


class _FakeLog:
    def __init__(self):
        self.toc = type("T", (), {"toc": {"pm": {}, "stabilizer": {}}})()

    def add_config(self, config):
        pass


class _FakeSyncCf:
    def __init__(self):
        self.cf = type("C", (), {"log": _FakeLog()})()


def test_stream_log_times_out_instead_of_hanging(monkeypatch):
    """Regression: the firmware can accept and start a log block and then send
    nothing. SyncLogger's iterator blocks forever on that, so every caller hung
    until Ctrl-C."""
    monkeypatch.setattr(cfenv, "LogConfig", FakeLogConfig)
    started = time.monotonic()

    with pytest.raises(cfenv.LinkLost) as excinfo:
        cfenv.stream_log(_FakeSyncCf(), {"pm.vbat": "float"},
                         lambda _data: True, timeout=0.5)

    assert time.monotonic() - started < 5, "stream_log did not return promptly"
    assert "no telemetry" in str(excinfo.value).lower()
