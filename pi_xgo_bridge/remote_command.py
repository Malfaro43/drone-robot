"""
Parse laptop gait commands and stop the dog if the laptop goes quiet.

Purpose:
    The bridge laptop POSTs small JSON bodies to ``POST /command``. This module
    validates that JSON without importing ``xgolib``, and ``CommandWatchdog``
    applies it to the dog. A walk or turn command keeps running on the motion
    board until something replaces it, so if the laptop crashes, sleeps or
    loses the network mid-walk, the dog would keep going. The watchdog stops it
    when no command has arrived for ``timeout`` seconds (the teleop receiver
    does the same with 0.5 s). The laptop must therefore resend its command
    several times a second while the dog moves.

Key terms:
    - Stop: Both ``walk`` and ``yaw`` are near zero, matching ``ObjectChaser`` when
      the dog should stand still.
    - Walk and yaw: xgolib speed values, not distances. ``walk`` goes to
      ``move('x', walk)`` (XGO Lite range -25..25; ``chase.py`` uses 8) and
      ``yaw`` to ``turn(yaw)`` (Lite range -100..100, positive turns left; about
      19 deg/s at 20 and 32 deg/s at 40 measured on our dog).

Exports:
    - ParsedCommand: Either stop or a walk/yaw pair.
    - parse_remote_command: Turn a JSON object into ``ParsedCommand`` or an error.
    - apply_parsed_command: Send one ``ParsedCommand`` to a live ``XGO``.
    - CommandWatchdog: Apply commands and stop the dog when they stop coming.

Python 3.9 compatible (the Pi runs 3.9.2).
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

WALK_MAX = 25.0     # xgolib VX limit for the XGO Lite
YAW_MAX = 100.0     # xgolib VYAW limit for the XGO Lite
WATCHDOG_S = 1.0    # stop the dog if no command arrives for this long


@dataclass(frozen=True)
class ParsedCommand:
    """Validated gait intent from the bridge laptop.

    :param stop: True when both channels should command a full stop.
    :param walk: Forward speed value (xgolib units) when ``stop`` is False.
    :param yaw: Turn speed value (xgolib units, positive = left) when ``stop`` is False.
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
    """Validate ``{"walk": speed, "yaw": speed}`` from the bridge.

    :param body: Decoded JSON object.
    :returns: ``ParsedCommand`` ready for ``apply_parsed_command``.
    :raises ValueError: When the body is missing fields or out of range.
    """

    if not isinstance(body, dict):
        raise ValueError("Body must be a JSON object.")
    if "walk" not in body or "yaw" not in body:
        raise ValueError('Body must include both "walk" and "yaw".')

    walk = _as_float(body["walk"], "walk")
    yaw = _as_float(body["yaw"], "yaw")

    if abs(walk) > WALK_MAX:
        raise ValueError(f"walk out of range (±{WALK_MAX:g}, xgolib speed units).")
    if abs(yaw) > YAW_MAX:
        raise ValueError(f"yaw out of range (±{YAW_MAX:g}, xgolib speed units).")

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


class CommandWatchdog:
    """Apply remote commands; stop the dog when they stop arriving.

    :param run: Calls ``fn(dog)`` with the live dog, e.g. ``telemetry.command``
        (which holds the serial lock).
    :param timeout: Seconds without a command before a moving dog is stopped.
    :param clock: Monotonic clock (injectable for tests).
    """

    def __init__(self, run: Callable[[Callable[[object], None]], object],
                 timeout: float = WATCHDOG_S, clock: Callable[[], float] = time.monotonic) -> None:
        self._run = run
        self.timeout = timeout
        self._clock = clock
        self._lock = threading.Lock()
        self._last = clock()
        self.moving = False
        self.tripped = 0           # how many times the watchdog had to stop the dog
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def command(self, parsed: ParsedCommand) -> None:
        """Apply one command from the laptop and restart the timeout."""

        with self._lock:
            self._run(lambda dog: apply_parsed_command(dog, parsed))
            self._last = self._clock()
            self.moving = not parsed.stop

    def check(self) -> bool:
        """One watchdog tick. Returns True if it just stopped the dog."""

        with self._lock:
            if not self.moving or self._clock() - self._last <= self.timeout:
                return False
            self._run(lambda dog: dog.stop())  # type: ignore[attr-defined]
            self.moving = False
            self.tripped += 1
            return True

    def start(self, period: float = 0.1) -> "CommandWatchdog":
        """Run ``check`` in a background thread every ``period`` seconds."""

        def loop() -> None:
            while not self._stop_event.wait(period):
                try:
                    self.check()
                except Exception:  # keep guarding even if one stop() fails
                    pass

        self._thread = threading.Thread(target=loop, name="xgo-command-watchdog", daemon=True)
        self._thread.start()
        return self

    def close(self) -> None:
        """Stop the background thread (the dog is stopped first if moving)."""

        with self._lock:
            if self.moving:
                self._run(lambda dog: dog.stop())  # type: ignore[attr-defined]
                self.moving = False
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
