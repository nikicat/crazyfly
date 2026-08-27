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

The drone accepted the log block and then sent no data. Its control channel
still answers, so this is the log task wedged rather than a link problem --
switching the drone off and on again clears it.

If that does not help, the drone may have powered off on its idle timeout, or
the battery may be too low. Check with:

    uv run linkcheck.py"""


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
                "handshake. Run `uv run linkcheck.py` to see which layer is alive."
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
                    "    uv run linkcheck.py"
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


def stream_log(scf, variables: dict[str, str], handler, timeout: float,
               period_ms: int = 100) -> None:
    """Feed each log sample to `handler` until it returns True or time runs out.

    Built on log callbacks rather than SyncLogger because SyncLogger's iterator
    blocks forever. The firmware can accept and start a log block and then send
    no data at all -- its control channel answers while the log task is wedged
    -- and that left every caller hung with no way out but Ctrl-C.
    """
    config = LogConfig(name="stream", period_in_ms=period_ms)
    for name, ctype in variables.items():
        config.add_variable(name, ctype)

    done = Event()
    errors: list[str] = []
    received = [False]

    def on_data(_timestamp, data, _config) -> None:
        received[0] = True
        if handler(data):
            done.set()

    def on_error(_config, message) -> None:
        errors.append(str(message))
        done.set()

    config.data_received_cb.add_callback(on_data)
    config.error_cb.add_callback(on_error)

    scf.cf.log.add_config(config)
    if not config.valid:
        raise LinkLost(
            f"The drone does not offer all of {', '.join(variables)}.\n"
            "Its log table of contents lists: "
            f"{', '.join(sorted(scf.cf.log.toc.toc))}"
        )

    config.start()
    try:
        finished = done.wait(timeout)
    finally:
        try:
            config.stop()
            config.delete()
        except Exception:  # noqa: BLE001 - cleanup must not mask the real error
            pass

    if errors:
        raise LinkLost(f"The drone rejected the log block: {errors[0]}")
    if not received[0]:
        raise LinkLost(NO_TELEMETRY)
    if not finished:
        raise LinkLost(
            f"Telemetry stopped partway through (waited {timeout:.0f} s).\n"
            "The link is up but the drone stopped sending. Power-cycle it."
        )


def sample_series(scf, variables: dict[str, str], count: int,
                  period_ms: int = 100) -> dict[str, list[float]]:
    """Collect `count` samples of each log variable."""
    series: dict[str, list[float]] = {name: [] for name in variables}
    first = next(iter(variables))

    def collect(data) -> bool:
        for name in variables:
            series[name].append(data[name])
        return len(series[first]) >= count

    # Generous: three times the nominal duration, and never less than 5 s.
    timeout = max(5.0, count * period_ms / 1000 * 3)
    stream_log(scf, variables, collect, timeout=timeout, period_ms=period_ms)
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
