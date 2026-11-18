"""Tests for ``xgo_robot.remote_command`` validation."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import pytest

from pi_xgo_bridge.remote_command import (
    CommandWatchdog,
    ParsedCommand,
    apply_parsed_command,
    parse_remote_command,
)


class FakeDog:
    """Record the last gait call for assertions."""

    def __init__(self) -> None:
        self.calls: list[tuple[str, float]] = []
        self.stopped = False

    def stop(self) -> None:
        self.stopped = True

    def move(self, axis: str, value: float) -> None:
        self.calls.append((axis, value))

    def turn(self, value: float) -> None:
        self.calls.append(("yaw", value))


def test_stop_command() -> None:
    parsed = parse_remote_command({"walk": 0, "yaw": 0})
    assert parsed.stop
    dog = FakeDog()
    apply_parsed_command(dog, parsed)
    assert dog.stopped
    assert not dog.calls


def test_move_then_turn() -> None:
    parsed = parse_remote_command({"walk": 8, "yaw": 10})
    assert parsed == ParsedCommand(stop=False, walk=8.0, yaw=10.0)
    dog = FakeDog()
    apply_parsed_command(dog, parsed)
    assert dog.calls == [("x", 8.0), ("yaw", 10.0)]


def test_reject_out_of_range() -> None:
    # walk/yaw are xgolib speed values: XGO Lite limits are 25 and 100
    with pytest.raises(ValueError):
        parse_remote_command({"walk": 26, "yaw": 0})
    with pytest.raises(ValueError):
        parse_remote_command({"walk": 0, "yaw": 101})
    assert parse_remote_command({"walk": -25, "yaw": 100}).walk == -25


class Clock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


def make_watchdog():
    dog, clock = FakeDog(), Clock()
    return dog, clock, CommandWatchdog(lambda fn: fn(dog), timeout=1.0, clock=clock)


def test_watchdog_stops_dog_when_commands_stop() -> None:
    dog, clock, wd = make_watchdog()
    wd.command(parse_remote_command({"walk": 8, "yaw": 0}))
    clock.t = 0.9
    assert not wd.check() and not dog.stopped
    clock.t = 1.2
    assert wd.check() and dog.stopped
    assert wd.tripped == 1
    assert not wd.check()            # only once


def test_watchdog_kept_alive_by_resent_commands() -> None:
    dog, clock, wd = make_watchdog()
    for n in range(20):
        clock.t = n * 0.25
        wd.command(parse_remote_command({"walk": 8, "yaw": 0}))
        assert not wd.check()
    assert not dog.stopped


def test_watchdog_ignores_a_standing_dog() -> None:
    dog, clock, wd = make_watchdog()
    wd.command(parse_remote_command({"walk": 0, "yaw": 0}))
    dog.stopped = False
    clock.t = 10
    assert not wd.check() and not dog.stopped


def test_watchdog_close_stops_moving_dog() -> None:
    dog, clock, wd = make_watchdog()
    wd.command(parse_remote_command({"walk": 0, "yaw": 20}))
    wd.close()
    assert dog.stopped
