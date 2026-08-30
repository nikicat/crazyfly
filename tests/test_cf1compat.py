"""cf1compat: the protocol version request gets a deadline."""
from __future__ import annotations

import time

from cflib.crazyflie.platformservice import PlatformService

import cf1compat


class FakeCf:
    def __init__(self):
        self.sent = []

    def add_port_callback(self, port, cb):
        pass

    def send_packet(self, pk):
        self.sent.append(pk)


class Packet:
    def __init__(self, channel, data):
        self.channel, self.data = channel, data


def test_unanswered_version_request_falls_back_to_legacy(monkeypatch):
    """2017.06 on a Crazyflie 1.0 says the magic string but has no platform
    service to answer the version request, which used to hang open_link()."""
    monkeypatch.setattr(cf1compat, "VERSION_TIMEOUT", 0.05)
    cf1compat.apply()

    done = []
    svc = PlatformService(FakeCf())
    svc.fetch_platform_informations(lambda: done.append(svc.get_protocol_version()))
    svc._crt_service_callback(Packet(1, cf1compat.MAGIC + b"\0" * 12))
    assert len(svc._cf.sent) == 2                 # SOURCE request, then version request
    time.sleep(0.2)
    assert done == [-1]                           # deadline passed: legacy path, once

    # Answered in time: the real version wins and the deadline is inert.
    done.clear()
    svc = PlatformService(FakeCf())
    svc.fetch_platform_informations(lambda: done.append(svc.get_protocol_version()))
    svc._crt_service_callback(Packet(1, cf1compat.MAGIC + b"\0" * 12))
    svc._platform_callback(Packet(1, bytes([0, 3])))
    time.sleep(0.2)
    assert done == [3]
