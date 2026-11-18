"""
Read and write the Tello over the drone's Wi-Fi.

Purpose:
    Wrap ``djitellopy`` so the bridge sees the same ``read`` / ``write`` shape
    as the XGO link. ``read`` returns telemetry dicts. ``write`` sends one SDK
    text command and returns the drone's reply string.

Key terms:
    - SDK command: A short text phrase such as ``battery?`` the firmware already
      understands over UDP.
    - One controller: Only one program may hold the Tello session; this link
      conflicts with ``tello_camera_stream.py``.

Exports:
    - TelloLink: Connected drone handle.

"""

from __future__ import annotations

import threading
from datetime import datetime
from typing import Any


def _finite(value: Any) -> float | None:
    """Return a float when ``value`` is numeric, else None.

    :param value: Raw reading from the drone.
    :returns: Finite float or None.
    """

    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    if out != out:  # NaN
        return None
    return out


class TelloLink:
    """Live connection to one Tello / RoboMaster TT aircraft.

    :param host: Unused placeholder for API symmetry (Tello is always local UDP).
    """

    def __init__(self, host: str = "192.168.10.1") -> None:
        self._host = host
        self._drone: Any = None
        self._lock = threading.Lock()
        self._open = False

    def open(self) -> None:
        """Connect and command the drone SDK session.

        :raises RuntimeError: When ``djitellopy`` is missing or connect fails.
        """

        if self._open:
            return
        try:
            from djitellopy import Tello
        except ImportError as exc:
            raise RuntimeError("Install djitellopy on the bridge laptop: pip install djitellopy") from exc

        drone = Tello()
        drone.connect()
        drone.set_speed(30)
        self._drone = drone
        self._open = True

    def close(self) -> None:
        """End the SDK session.

        :returns: None.
        """

        with self._lock:
            if self._drone is not None:
                try:
                    self._drone.end()
                except Exception:
                    pass
            self._drone = None
            self._open = False

    def read(self) -> dict[str, Any]:
        """Return battery, attitude, height, and related fields.

        :returns: JSON-serializable dict with ``ok`` and ``telemetry``.
        """

        with self._lock:
            if not self._open or self._drone is None:
                return {"ok": False, "error": "Tello not connected. Call open() first."}
            d = self._drone

        def safe(call: object, default: Any = None) -> Any:
            try:
                return call()  # type: ignore[misc, operator]
            except Exception:
                return default

        battery = safe(lambda: int(d.get_battery()), -1)
        if battery < 0:
            battery = None

        payload = {
            "ok": True,
            "mode": "tello",
            "updated_at": datetime.now().isoformat(timespec="seconds"),
            "telemetry": {
                "battery_percent": battery,
                "height_cm": _finite(safe(lambda: d.get_height())),
                "baro_m": _finite(safe(lambda: d.get_barometer())),
                "wifi_snr": _finite(safe(lambda: d.query_wifi_signal_noise_ratio())),
                "imu": {
                    "rotation": {
                        "roll": _finite(safe(lambda: d.get_roll())),
                        "pitch": _finite(safe(lambda: d.get_pitch())),
                        "yaw": _finite(safe(lambda: d.get_yaw())),
                    },
                },
            },
        }
        return payload

    def write(self, command: str) -> dict[str, Any]:
        """Send one SDK command string and return the firmware reply.

        :param command: Text command such as ``battery?``.
        :returns: Dict with ``ok`` and ``response``.
        """

        command = (command or "").strip()
        if not command:
            return {"ok": False, "error": "Command must be non-empty."}

        with self._lock:
            if not self._open or self._drone is None:
                return {"ok": False, "error": "Tello not connected. Call open() first."}
            d = self._drone
            try:
                if command.endswith("?"):
                    response = d.send_read_command(command)
                else:
                    response = d.send_command_with_return(command)
            except Exception as exc:
                return {"ok": False, "error": str(exc), "command": command}

        return {"ok": True, "command": command, "response": str(response)}
