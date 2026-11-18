"""
Parse laptop gait commands into stop or walk/yaw for the motion board.

Purpose:
    The bridge laptop POSTs small JSON bodies to ``POST /command``. This module
    validates that JSON without importing ``xgolib``. The HTTP server runs the
    result through ``telemetry.command`` on the Pi.

Key terms:
    - Stop: Both ``walk`` and ``yaw`` are near zero, matching ``ObjectChaser`` when
      the dog should stand still.
    - Walk and yaw: Forward millimeters on the X axis and yaw degrees per second,
      the same two numbers ``chase.py`` sends.

Exports:
    - ParsedCommand: Either stop or a walk/yaw pair.
    - parse_remote_command: Turn a JSON object into ``ParsedCommand`` or an error.

"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class ParsedCommand:
    """Validated gait intent from the bridge laptop.

    :param stop: True when both channels should command a full stop.
    :param walk: Forward millimeters when ``stop`` is False.
    :param yaw: Yaw degrees per second when ``stop`` is False.
    """

    stop: bool
    walk: float = 0.0
    yaw: float = 0.0


def _as_float(value: Any, field: str) -> float:
    """Coerce a JSON value to float or raise ValueError.

    :param value: Raw JSON field.
    :param field: Name for error messages.
    :returns: Floating gait channel.
    """

    if isinstance(value, bool):
        raise ValueError(f"{field} must be a number, not a boolean.")
    if isinstance(value, (int, float)):
        return float(value)
    raise ValueError(f"{field} must be a number.")


def parse_remote_command(body: dict[str, Any]) -> ParsedCommand:
    """Validate ``{"walk": mm, "yaw": deg/s}`` from the bridge.

    :param body: Decoded JSON object.
    :returns: ``ParsedCommand`` ready for ``telemetry.command``.
    :raises ValueError: When the body is missing fields or out of range.
    """

    if not isinstance(body, dict):
        raise ValueError("Body must be a JSON object.")
    if "walk" not in body or "yaw" not in body:
        raise ValueError('Body must include both "walk" and "yaw".')

    walk = _as_float(body["walk"], "walk")
    yaw = _as_float(body["yaw"], "yaw")

    if abs(walk) > 400.0:
        raise ValueError("walk out of allowed range (±400 mm).")
    if abs(yaw) > 120.0:
        raise ValueError("yaw out of allowed range (±120 deg/s).")

    if abs(walk) < 0.5 and abs(yaw) < 0.5:
        return ParsedCommand(stop=True)

    return ParsedCommand(stop=False, walk=walk, yaw=yaw)


def apply_parsed_command(dog: object, parsed: ParsedCommand) -> None:
    """Mirror ``ObjectChaser._apply`` for one remote command.

    :param dog: Live ``XGO`` instance.
    :param parsed: Output of ``parse_remote_command``.
    :returns: None.
    """

    if parsed.stop:
        dog.stop()  # type: ignore[attr-defined]
        return
    dog.move("x", parsed.walk)  # type: ignore[attr-defined]
    dog.turn(parsed.yaw)  # type: ignore[attr-defined]
