#!/usr/bin/env python3
"""Low-level radio diagnostic.

Talks straight to the Crazyradio dongle, bypassing cflib's link layer, and
reports which layer of the stack is actually alive:

  * nRF acknowledge      -- something is powered and listening on that link
  * CRTP link echo       -- the radio MCU (nRF51) is running
  * CRTP services        -- the main MCU (STM32) is running the firmware

A drone that acks and echoes but answers no services has a live radio and a
main MCU that is off, hung, or unflashed.
"""
from __future__ import annotations

from cflib.drivers.crazyradio import Crazyradio

import cfenv

DATARATES = {
    "250K": Crazyradio.DR_250KPS,
    "1M": Crazyradio.DR_1MPS,
    "2M": Crazyradio.DR_2MPS,
}

# Idle filler the firmware sends when it has nothing queued.
IDLE = (0xF3, 0xF7)


def hdr(port: int, channel: int) -> int:
    """CRTP header byte: port in the high nibble, reserved bits set, channel low."""
    return ((port & 0x0F) << 4) | 0x0C | (channel & 0x03)


def poll(cr: Crazyradio) -> bytes:
    """Send a null packet and return whatever ack payload comes back."""
    ack = cr.send_packet((0xFF,))
    return bytes(ack.data) if (ack and ack.data) else b""


def drain(cr: Crazyradio, rounds: int = 12) -> None:
    """Flush replies still queued from an earlier request.

    Without this a slow reply lands during the *next* probe's polling window
    and gets credited to the wrong request, which makes results flap between
    runs.
    """
    for _ in range(rounds):
        poll(cr)


def probe(cr: Crazyradio, payload: tuple[int, ...], accept,
          polls: int = 25, attempts: int = 3) -> bytes | None:
    """Send a request and return the first later ack payload `accept` likes.

    The Crazyflie answers in a later ack payload, not the one for the request
    itself, so we poll -- and `accept` decides what counts, otherwise late
    traffic from a previous probe looks like a response.
    """
    for _ in range(attempts):
        drain(cr)
        got = bytes(cr.send_packet(payload).data or b"")
        for _ in range(polls):
            if accept(got):
                return got
            got = poll(cr)
    return None


def on_port(port: int):
    """Accept a real reply on `port`: not idle filler, not another port's traffic."""
    return lambda got: bool(got) and got[0] not in IDLE and (got[0] >> 4) == port


def probe_echo(cr: Crazyradio) -> bool:
    """Link-service echo. Validated by exact payload, since idle filler
    shares port 15 and would otherwise pass a port-only check."""
    payload = (hdr(15, 0), 0xAA, 0xBB)
    return probe(cr, payload, lambda got: got == bytes(payload), polls=15) is not None


def check(cr: Crazyradio, channel: int, datarate: str, address: int) -> None:
    addr = tuple((address >> (8 * i)) & 0xFF for i in reversed(range(5)))
    cr.set_channel(channel)
    cr.set_data_rate(DATARATES[datarate])
    cr.set_address(addr)
    cr.set_arc(10)

    addr_s = "".join(f"{b:02X}" for b in addr)
    print(f"link: radio://0/{channel}/{datarate}/{addr_s}\n")

    ack = cr.send_packet((0xFF,))
    if not (ack and ack.ack):
        print("  nRF ack        : NO  -- nothing is listening on this link")
        print("\nNothing responded. The drone is off, out of range, or on another link.")
        return
    print("  nRF ack        : yes -- a device is powered and listening")

    radio_ok = probe_echo(cr)
    print(f"  CRTP link echo : {'yes -- CRTP link layer is running' if radio_ok else 'NO'}")

    # The TOC command changed between firmware generations: 0x01 is the
    # original (Crazyflie 1.0), 0x03 the v2 used by the 2.x. Ask for both --
    # checking only v2 makes a perfectly healthy 1.0 look dead.
    alive: list[str] = []
    generations = set()
    for port, name in ((2, "param TOC"), (5, "log TOC")):
        for cmd, version in ((0x03, "v2"), (0x01, "v1")):
            reply = probe(cr, (hdr(port, 0), cmd), on_port(port))
            if reply:
                count = reply[2] if version == "v1" else int.from_bytes(reply[2:4], "little")
                alive.append(f"{name} {version} ({count} items)")
                generations.add(version)
                break
    print(f"  CRTP services  : {', '.join(alive) if alive else 'NONE answered'}")

    # Only the 2.x implements the platform service; its absence is normal on a 1.0.
    platform = probe(cr, (hdr(13, 1), 0x00), on_port(13))
    print(f"  platform svc   : {'yes' if platform else 'no (normal on a Crazyflie 1.0)'}")

    print()
    if alive and "v1" in generations:
        print(
            "Main firmware is running -- this is a Crazyflie 1.0 (legacy CRTP).\n"
            "It answers the v1 TOC only, so stock cflib cannot connect to it.\n"
            "The scripts here load cf1compat, which handles that. Try info.py."
        )
    elif alive:
        print("Main firmware is running. The drone is ready to fly.")
    elif radio_ok:
        print(
            "Radio link layer answers but no CRTP service does.\n"
            "On a Crazyflie 2.x this means the STM32 is off, hung or unflashed:\n"
            "power-cycle it, check the battery, then reflash if it stays silent."
        )
    else:
        print("Device acks at the radio layer but does not speak CRTP.")


def run(
    channel: int = 40,
    datarate: str = "250K",
    address: str = "E7E7E7E7E7",
) -> None:
    """Low-level radio diagnostic. Needs no working firmware."""
    if datarate not in DATARATES:
        raise SystemExit(f"datarate must be one of {', '.join(sorted(DATARATES))}")

    radio = Crazyradio()
    print(f"Crazyradio firmware {radio.version}\n")
    try:
        check(radio, channel, datarate, int(address, 16))
    finally:
        radio.close()


if __name__ == "__main__":
    cfenv.cli(run)
