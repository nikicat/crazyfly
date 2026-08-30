"""cf.py: the command router and direct invocation cannot drift apart."""
from __future__ import annotations

import cf
import hoptest
import info


def test_router_exposes_every_tool():
    """Each script's run() must be reachable through cf.py."""
    registered = {command.name for command in cf.app.registered_commands}
    expected = {"scan", "link", "boot", "flash", "info", "mag", "teleop", "hover",
                "trim", "hop", "orient", "motors", "spin", "airborne"}
    assert expected <= registered


def test_router_commands_are_the_modules_own_run():
    """The router wraps run() rather than reimplementing the options, so a
    changed default cannot mean two different things."""
    by_name = {c.name: c.callback for c in cf.app.registered_commands}
    assert by_name["hop"] is hoptest.run
    assert by_name["info"] is info.run


def test_every_command_has_help_text():
    for command in cf.app.registered_commands:
        assert command.help, f"{command.name} has no help"
