#!/usr/bin/env python3
"""One entry point for every tool here.

    uv run cf.py --help
    uv run cf.py info
    uv run cf.py hop --thrust 42000 --reset-trim

Each command is the `run()` of the module named beside it, so the options are
the same either way and `uv run hoptest.py --thrust 42000` still works. The
commands are grouped below in the order you would normally reach for them:
find the drone, look at it, fly it, then work out why it drifts.
"""
from __future__ import annotations

import typer

import bootcheck
import cfenv
import flash
import flightcheck
import hover
import info
import linkcheck
import motorcheck
import orient
import scan
import teleop
import trimcheck
from hoptest import run as hop_run

app = typer.Typer(
    help=__doc__,
    no_args_is_help=True,
    add_completion=False,
    rich_markup_mode="rich",
)

# --- finding it -----------------------------------------------------------
app.command("scan", help=scan.run.__doc__)(scan.run)
app.command("link", help=linkcheck.run.__doc__)(linkcheck.run)
app.command("boot", help=bootcheck.run.__doc__)(bootcheck.run)
app.command("flash", help=flash.run.__doc__)(flash.run)

# --- looking at it --------------------------------------------------------
app.command("info", help=info.run.__doc__)(info.run)

# --- flying it ------------------------------------------------------------
app.command("teleop", help=teleop.run.__doc__)(teleop.run)
app.command("hover", help=hover.run.__doc__)(hover.run)

# --- working out the drift ------------------------------------------------
app.command("trim", help=trimcheck.run.__doc__)(trimcheck.run)
app.command("hop", help=hop_run.__doc__)(hop_run)
app.command("orient", help=orient.run.__doc__)(orient.run)
app.command("motors", help=motorcheck.run.__doc__)(motorcheck.run)
app.command("airborne", help=flightcheck.run.__doc__)(flightcheck.run)


def main() -> None:
    # Route through cfenv.run so a busy dongle or a drone that stopped
    # answering reports as a sentence rather than a traceback, exactly as it
    # does when a script is invoked directly.
    cfenv.run(app)


if __name__ == "__main__":
    main()
