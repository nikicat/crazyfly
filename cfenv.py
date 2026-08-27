"""Shared helpers: driver init and URI resolution.

The URI comes from $CF_URI when set, otherwise from a scan. Scanning on every
run is slow at 250K, so pin CF_URI once you know your drone's link.
"""
from __future__ import annotations

import logging
import os
import sys
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.drivers.crazyradio import Crazyradio
from usb.core import USBError

import cf1compat

DEFAULT_URI = "radio://0/40/250K"

CONNECT_TIMEOUT = 20.0


class ConnectTimeout(Exception):
    pass


class LinkLost(Exception):
    """The drone stopped answering: powered off, out of range, or flat."""


NO_TELEMETRY = """\
Connected, but no telemetry came back.

The drone stopped answering mid-session. It powers itself off on an idle
timeout, so that is the usual cause; a low battery or being out of range will
do the same. Switch it back on and check it with:

    .venv/bin/python linkcheck.py"""


class TimedSyncCrazyflie(SyncCrazyflie):
    """SyncCrazyflie that gives up instead of blocking forever.

    Upstream open_link() waits on an Event with no timeout, so a drone whose
    radio acks but whose main firmware never answers leaves the caller hung
    indefinitely. That is a common enough failure -- a drone that is off or
    unflashed still acks -- to be worth handling explicitly.
    """

    def __init__(self, link_uri, cf=None, timeout: float = CONNECT_TIMEOUT):
        super().__init__(link_uri, cf=cf)
        self._timeout = timeout

    def open_link(self):
        if self.is_link_open():
            raise Exception("Link already open")

        self._add_callbacks()
        self._connect_event = Event()
        self._params_updated_event.clear()
        self.cf.open_link(self._link_uri)
        completed = self._connect_event.wait(self._timeout)
        self._connect_event = None

        if not completed:
            self._remove_callbacks()
            self._params_updated_event.clear()
            self.cf.close_link()
            raise ConnectTimeout(
                f"No response from {self._link_uri} within {self._timeout:.0f} s.\n"
                "The radio link is up but the firmware never completed the\n"
                "handshake. Run `python linkcheck.py` to see which layer is alive."
            )

        if not self._is_link_open:
            self._remove_callbacks()
            self._params_updated_event.clear()
            self.cf.close_link()
            message = self._error_message or "connection failed"
            if "packets lost" in message.lower():
                raise LinkLost(
                    f"Lost the link to {self._link_uri} while connecting.\n\n"
                    "The drone powers itself off on an idle timeout, which is the\n"
                    "usual cause. A low battery or being out of range does the\n"
                    "same. Switch it on and check it with:\n\n"
                    "    .venv/bin/python linkcheck.py"
                )
            raise LinkLost(message)


def _check_radio_free(uri: str) -> None:
    """Fail early and clearly if another process holds the dongle.

    cflib opens the radio on a worker thread, so a USBError there never reaches
    the caller -- it prints a traceback from the thread and the connection just
    times out twenty seconds later. Claiming the dongle briefly here surfaces
    the real reason on the main thread instead.
    """
    if not uri.startswith("radio://"):
        return
    try:
        radio = Crazyradio()
    except USBError as err:
        if err.errno == 16:
            sys.exit(BUSY_MESSAGE)
        raise
    else:
        radio.close()


def connect(uri: str, timeout: float = CONNECT_TIMEOUT) -> TimedSyncCrazyflie:
    """Open a link, caching the log/param TOC so reconnects are fast."""
    _check_radio_free(uri)
    return TimedSyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache"), timeout=timeout)


def sample_series(scf, variables: dict[str, str], count: int,
                  period_ms: int = 100) -> dict[str, list[float]]:
    """Collect `count` samples of each log variable.

    Raises LinkLost rather than returning empty lists: callers averaged the
    result, so a drone that stopped answering surfaced as a division by zero
    or an empty-statistics error instead of the real cause.
    """
    config = LogConfig(name="sample", period_in_ms=period_ms)
    for name, ctype in variables.items():
        config.add_variable(name, ctype)

    series: dict[str, list[float]] = {name: [] for name in variables}
    first = next(iter(variables))
    with SyncLogger(scf, config) as logger:
        for _, data, _ in logger:
            for name in variables:
                series[name].append(data[name])
            if len(series[first]) >= count:
                break

    if not series[first]:
        raise LinkLost(NO_TELEMETRY)
    return series


BUSY_MESSAGE = """\
The Crazyradio is already in use by another process.

Only one program can hold the dongle at a time, so a script left sitting at a
prompt keeps every other one locked out. Find it with:

    pgrep -af '[.]venv/bin/python'
    fuser -v /dev/bus/usb/*/*

If it is an interactive script such as hoptest.py, switch to its terminal and
quit it properly -- that saves your trim. Killing it discards any trim you
adjusted in that session."""


def run(entry) -> None:
    """Run a script entry point, turning the usual failures into plain messages.

    Without this each one surfaces as a traceback: a busy dongle as a bare
    USBError 16, which says nothing about what to do next.
    """
    try:
        entry()
    except (ConnectTimeout, LinkLost) as err:
        sys.exit(str(err))
    except USBError as err:
        if err.errno == 16:      # EBUSY
            sys.exit(BUSY_MESSAGE)
        if err.errno == 13:      # EACCES
            sys.exit("Permission denied opening the Crazyradio. The udev rule "
                     "in\n/etc/udev/rules.d/99-bitcraze.rules may be missing.")
        raise
    except KeyboardInterrupt:
        sys.exit("\nInterrupted.")


def init(quiet: bool = True) -> None:
    logging.basicConfig(level=logging.ERROR if quiet else logging.INFO)
    cf1compat.apply()
    cflib.crtp.init_drivers()


def resolve_uri(argv_uri: str | None = None) -> str:
    """Pick a URI: explicit argument, then $CF_URI, then scan for one."""
    if argv_uri:
        return argv_uri
    env = os.environ.get("CF_URI")
    if env:
        return env

    print("No CF_URI set, scanning...", file=sys.stderr)
    found = cflib.crtp.scan_interfaces()
    if not found:
        sys.exit("No Crazyflie found. Is it powered on? Try: python linkcheck.py")
    uri = found[0][0]
    print(f"Using {uri} (set CF_URI to skip this scan)", file=sys.stderr)
    return uri
