"""Tests for ``xgo_robot.remote_command`` validation."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import pytest

from pi_xgo_bridge.remote_command import ParsedCommand, apply_parsed_command, parse_remote_command


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
    parsed = parse_remote_command({"walk": 40, "yaw": 10})
    assert parsed == ParsedCommand(stop=False, walk=40.0, yaw=10.0)
    dog = FakeDog()
    apply_parsed_command(dog, parsed)
    assert dog.calls == [("x", 40.0), ("yaw", 10.0)]


def test_reject_out_of_range() -> None:
    with pytest.raises(ValueError):
        parse_remote_command({"walk": 500, "yaw": 0})
