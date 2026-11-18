"""
Read and write the XGO over Tailscale HTTP.

Purpose:
    The dog's motors live on the Pi serial port, so the laptop calls the Pi's
    existing HTTP server. ``read`` uses ``GET /stats``. ``write`` posts walk and
    yaw to ``POST /command``.

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


class XgoLink:
    """HTTP link to one XGO live server on the Pi.

    :param pi_url: Scheme, host, and port for the robot process (no path).
    :param timeout_s: Per-request timeout in seconds.
    """

    def __init__(self, pi_url: str, timeout_s: float = 8.0) -> None:
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

    def write(self, walk: float = 0.0, yaw: float = 0.0) -> dict[str, Any]:
        """Post ``{"walk": mm, "yaw": deg/s}`` to ``POST /command``.

        :param walk: Forward millimeters on the X axis.
        :param yaw: Yaw degrees per second.
        :returns: Parsed JSON response from the Pi.
        """

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
