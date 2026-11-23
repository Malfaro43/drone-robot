"""
Read and write the XGO over Tailscale HTTP.

Purpose:
    The dog's motors live on the Pi serial port, so the laptop calls the Pi's
    existing HTTP server. ``read`` uses ``GET /stats``. ``write`` posts walk and
    yaw to ``POST /command``.

    ``walk`` and ``yaw`` are xgolib **speed values**, not a distance or an angle:
    the dog keeps moving at that speed until the next command. XGO Lite ranges
    are walk -25..25 (``chase.py`` uses 8) and yaw -100..100, positive = left
    (about 19 deg/s at 20 on our dog). The Pi stops the dog if no command
    arrives for 1 s (``pi_xgo_bridge.CommandWatchdog``), so while the dog moves,
    resend the command several times a second.

Key terms:
    - pi_url: Base URL such as ``http://100.x.x.x:8080`` (no trailing path).
    - Stats JSON: The same payload the live browser panel uses.

Exports:
    - XgoLink: HTTP client for one Pi base URL.

"""

from __future__ import annotations

import json
import urllib.error
import urllib.request
from typing import Any

WALK_MAX = 25.0   # XGO Lite xgolib speed limits (same as pi_xgo_bridge.remote_command)
YAW_MAX = 100.0


class XgoLink:
    """HTTP link to one XGO live server on the Pi.

    :param pi_url: Scheme, host, and port for the robot process (no path).
    :param timeout_s: Per-request timeout in seconds. Kept short: the Pi stops the dog
        after 1 s without a command anyway, so a long wait only delays noticing.
    """

    def __init__(self, pi_url: str, timeout_s: float = 3.0) -> None:
        self._base = pi_url.rstrip("/")
        self._timeout = timeout_s

    def read(self) -> dict[str, Any]:
        """Fetch ``GET /stats`` from the Pi.

        :returns: Parsed JSON dict.
        """

        url = f"{self._base}/stats"
        try:
            with urllib.request.urlopen(url, timeout=self._timeout) as resp:
                raw = resp.read().decode("utf-8")
        except urllib.error.URLError as exc:
            return {
                "ok": False,
                "error": (
                    f"Could not reach {url}. Is python3 main.py stream running on the Pi? "
                    f"({exc})"
                ),
            }
        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            return {"ok": False, "error": "Pi returned non-JSON from /stats."}

    def stop(self) -> dict[str, Any]:
        """Stop the dog (``walk=0, yaw=0``).

        :returns: The Pi's reply.
        """

        return self.write(walk=0.0, yaw=0.0)

    def write(self, walk: float = 0.0, yaw: float = 0.0) -> dict[str, Any]:
        """Post ``{"walk": speed, "yaw": speed}`` to ``POST /command``.

        :param walk: Forward speed value, -25..25 (xgolib ``move('x')`` units).
        :param yaw: Turn speed value, -100..100, positive = left (xgolib ``turn``).
        :returns: Parsed JSON response from the Pi, or ``{"ok": False, ...}``.
        """

        if abs(walk) > WALK_MAX or abs(yaw) > YAW_MAX:
            return {"ok": False, "error": f"walk must be within ±{WALK_MAX:g} and yaw within "
                                          f"±{YAW_MAX:g} (xgolib speed values); not sent."}

        url = f"{self._base}/command"
        body = json.dumps({"walk": walk, "yaw": yaw}).encode("utf-8")
        request = urllib.request.Request(
            url,
            data=body,
            method="POST",
            headers={"Content-Type": "application/json; charset=utf-8"},
        )
        try:
            with urllib.request.urlopen(request, timeout=self._timeout) as resp:
                raw = resp.read().decode("utf-8")
        except urllib.error.HTTPError as exc:
            try:
                detail = exc.read().decode("utf-8")
            except Exception:
                detail = exc.reason
            return {"ok": False, "error": detail, "status": exc.code}
        except urllib.error.URLError as exc:
            return {"ok": False, "error": str(exc)}

        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            return {"ok": True, "raw": raw}
