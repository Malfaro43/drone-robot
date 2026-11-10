"""
Local HTTP camera stream with a top-right telemetry panel (XGO-style layout).

Purpose:
    Runs on your laptop after you join the drone's Wi‑Fi. Serves a browser page at
    ``http://<this-host>:8080/`` with a full-viewport MJPEG feed (``/stream``) and
    JSON telemetry (``/stats``) polled by the panel in the top-right corner—same
    visual pattern as the XGO live page: dark stage, ``object-fit: contain`` video,
    frosted stats card.

    The default backend is a DJI Tello–protocol drone (RoboMaster TT / Tello Edu
    class) via ``djitellopy``. For development without the aircraft, use
    ``--camera 0`` (or another OpenCV index / video path).

Key terms:
    - MJPEG: multipart JPEG over HTTP; browsers show it as a live image.
    - LiveHub: holds only the newest JPEG so slow clients skip frames instead of
      queuing behind real time.

CLI:
    python tello_camera_stream.py
    python tello_camera_stream.py --port 9000 --camera 0
    python tello_camera_stream.py --wifi-interval 5   # fewer wifi? polls; use 0 for every tick

Dependencies:
    pip install opencv-python djitellopy
    (``djitellopy`` is only needed for drone mode; omit for webcam-only use.)

:author: Course helper script (standalone).
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import numbers
import sys
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Optional

# --- constants (aligned with XGO http_stream / models) ---------------------------------

JPEG_QUALITY = 80
MJPEG_BOUNDARY = b"frame"
DEFAULT_BIND = "0.0.0.0"
DEFAULT_PORT = 8080
DEFAULT_TELEMETRY_HZ = 4.0
# ``wifi?`` each telemetry tick (same pacing as earlier builds). Set >0 via CLI to throttle.
DEFAULT_WIFI_REFRESH_S = 0.0

# --- HTML: same layout/CSS vocabulary as XGO ``PAGE_HTML``; content tuned for Tello -------

PAGE_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Tello camera</title>
  <style>
    body { margin: 0; background: #111; color: #eee; font-family: system-ui, sans-serif; }
    .stage { position: relative; width: 100vw; height: 100vh; overflow: hidden; background: #000; }
    .feed { display: block; width: 100%; height: 100%; object-fit: contain; position: relative; z-index: 1; }
    #stats-panel {
      position: absolute; top: 12px; right: 12px; width: min(340px, 42vw);
      max-height: calc(100vh - 24px); overflow: auto; z-index: 1000;
      background: rgba(10, 12, 18, 0.92); border: 2px solid rgba(120, 200, 255, 0.55);
      border-radius: 10px; padding: 12px 14px; font-size: 13px; line-height: 1.35;
      box-shadow: 0 8px 28px rgba(0,0,0,0.45); backdrop-filter: blur(6px);
    }
    #stats-panel h2 { margin: 0 0 8px; font-size: 15px; font-weight: 600; }
    #stats-panel h3 { margin: 12px 0 6px; font-size: 12px; text-transform: uppercase; letter-spacing: 0.04em; color: #9ab; }
    .stat-row { display: flex; justify-content: space-between; gap: 8px; padding: 2px 0; }
    .stat-row span:last-child { font-variant-numeric: tabular-nums; color: #dfe; }
    .battery-bar { height: 8px; border-radius: 4px; background: #333; margin-top: 6px; overflow: hidden; }
    .battery-fill { height: 100%; background: linear-gradient(90deg, #3d8, #6c6); width: 0%; transition: width 0.3s; }
    .servo-list { max-height: 220px; overflow: auto; padding-right: 4px; }
    .warn { color: #f88; font-size: 12px; margin-top: 8px; }
  </style>
</head>
<body>
  <div class="stage">
    <img class="feed" src="/stream" alt="Tello camera">
    <aside id="stats-panel" aria-live="polite">
      <h2>Drone stats</h2>
      <div id="stats-body">Connecting to telemetry…</div>
    </aside>
  </div>
  <script>
    function fmt(n, digits) {
      if (n === null || n === undefined || Number.isNaN(n)) return "—";
      return Number(n).toFixed(digits);
    }
    function row(label, value) {
      return '<div class="stat-row"><span>' + label + '</span><span>' + value + '</span></div>';
    }
    function renderStats(data) {
      const root = document.getElementById("stats-body");
      if (!data.ok) {
        root.innerHTML = '<p class="warn">' + (data.error || "Telemetry unavailable") + '</p>';
        return;
      }
      const mode = data.mode || "unknown";
      if (mode === "webcam") {
        const w = data.stream || {};
        root.innerHTML =
          '<h3>Camera</h3>' +
          row("Source", w.source || "—") +
          row("Resolution", (w.width || "—") + " × " + (w.height || "—")) +
          row("FPS (approx)", fmt(w.fps, 1)) +
          row("Frames", String(w.frames ?? "—")) +
          '<h3>Host</h3>' +
          row("Uptime s", fmt(data.uptime_s, 1)) +
          row("Updated", data.updated_at || "—");
        return;
      }
      const t = data.telemetry || {};
      const bat = t.battery_percent;
      const imu = t.imu || {};
      const acc = imu.acceleration || {};
      const rot = imu.rotation || {};
      const gyro = imu.gyroscope || {};
      const vel = t.velocity || {};
      root.innerHTML =
        '<h3>Battery</h3>' +
        row("Charge", (bat != null ? bat : "—") + (bat != null ? "%" : "")) +
        '<div class="battery-bar"><div class="battery-fill" style="width:' +
          (bat != null ? Math.max(0, Math.min(100, bat)) : 0) + '%"></div></div>' +
        '<h3>State</h3>' +
        row("Height cm", fmt(t.height_cm, 1)) +
        row("Baro m", fmt(t.baro_m, 2)) +
        row("Distance cm", fmt(t.tof_cm, 1)) +
        row("Flight time", fmt(t.flight_time_s, 1) + " s") +
        '<h3>Attitude (°)</h3>' +
        row("Pitch", fmt(rot.pitch, 2)) +
        row("Roll", fmt(rot.roll, 2)) +
        row("Yaw", fmt(rot.yaw, 2)) +
        '<h3>Velocity (cm/s)</h3>' +
        row("X", fmt(vel.x, 2)) +
        row("Y", fmt(vel.y, 2)) +
        row("Z", fmt(vel.z, 2)) +
        '<h3>Acceleration (cm/s²)</h3>' +
        row("X", fmt(acc.x, 2)) +
        row("Y", fmt(acc.y, 2)) +
        row("Z", fmt(acc.z, 2)) +
        '<h3>Gyro (°/s)</h3>' +
        row("X", fmt(gyro.x, 2)) +
        row("Y", fmt(gyro.y, 2)) +
        row("Z", fmt(gyro.z, 2)) +
        '<h3>Environment</h3>' +
        row("Temp °C", t.temp_c != null ? fmt(t.temp_c, 1) : "—") +
        row("Wi‑Fi SNR", t.wifi_snr != null ? fmt(t.wifi_snr, 1) : "—") +
        '<h3>Host</h3>' +
        row("Uptime s", fmt(data.uptime_s, 1)) +
        row("Updated", data.updated_at || "—");
    }
    async function refreshStats() {
      try {
        const response = await fetch("/stats", { cache: "no-store" });
        const text = await response.text();
        if (!response.ok) {
          renderStats({
            ok: false,
            error: "/stats HTTP " + response.status + ": " + text.slice(0, 200)
          });
          return;
        }
        let data;
        try {
          data = JSON.parse(text);
        } catch (parseErr) {
          renderStats({
            ok: false,
            error: "Bad JSON from /stats (invalid JSON breaks the panel; often NaN from the server). Snippet: "
              + text.slice(0, 160)
          });
          return;
        }
        renderStats(data);
      } catch (err) {
        renderStats({ ok: false, error: "Could not load /stats: " + err });
      }
    }
    refreshStats();
    setInterval(refreshStats, 400);
  </script>
</body>
</html>
"""


class LiveHub:
    """Mailbox that stores only the newest camera JPEG for MJPEG clients.

    :param: none
    """

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._jpeg: bytes | None = None
        self._sequence = 0

    def publish(self, frame: object) -> None:
        """Replace the stored JPEG with a compressed copy of ``frame`` (BGR ndarray).

        :param frame: BGR image from OpenCV or ``djitellopy`` frame reader.
        :returns: None
        """

        import cv2  # local import so ``--help`` works without OpenCV

        encoded, buffer = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
        )
        if not encoded:
            return
        with self._condition:
            self._jpeg = buffer.tobytes()
            self._sequence += 1
            self._condition.notify_all()

    def wake(self) -> None:
        """Notify waiters (e.g. so streaming threads can exit).

        :returns: None
        """

        with self._condition:
            self._condition.notify_all()

    def wait_next(self, last_sequence: int, timeout: float = 1.0) -> tuple[int, bytes | None]:
        """Block until a JPEG newer than ``last_sequence`` arrives or timeout.

        :param last_sequence: Last sequence the caller already sent (0 = none yet).
        :param timeout: Seconds to wait before returning the same sequence.
        :returns: ``(sequence, jpeg_bytes)``; jpeg is None until the first ``publish``.
        """

        with self._condition:
            if self._sequence == last_sequence:
                self._condition.wait(timeout=timeout)
            return self._sequence, self._jpeg


class StreamHandler(BaseHTTPRequestHandler):
    """Serve ``/``, ``/stream`` (MJPEG), and ``/stats`` (JSON).

    Class attributes are set by ``start_http_server`` before requests arrive.

    :param: inherited from ``BaseHTTPRequestHandler``
    """

    hub: LiveHub | None = None
    stop: threading.Event | None = None
    telemetry: Any = None

    def do_GET(self) -> None:
        """Route GET requests to HTML, MJPEG, or JSON.

        :returns: None
        """

        path = self.path.split("?", 1)[0]
        if path == "/stream":
            self._write_stream()
            return
        if path == "/stats":
            self._write_stats()
            return
        body = PAGE_HTML.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _write_stats(self) -> None:
        """Write JSON from the telemetry poller's ``snapshot()`` method.

        :returns: None
        """

        if self.telemetry is None:
            payload = {"ok": False, "error": "Telemetry not initialized."}
        else:
            payload = self.telemetry.snapshot()
        try:
            body = json.dumps(_json_safe(payload), allow_nan=False).encode("utf-8")
        except (TypeError, ValueError) as exc:
            err = {"ok": False, "error": f"Stats serialization failed: {exc}"}
            body = json.dumps(err).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _write_stream(self) -> None:
        """Push multipart JPEG until disconnect or ``stop`` is set.

        :returns: None
        """

        if self.hub is None or self.stop is None:
            self.send_error(503, "Camera hub is not ready")
            return
        self.send_response(200)
        self.send_header(
            "Content-Type",
            f"multipart/x-mixed-replace; boundary={MJPEG_BOUNDARY.decode('ascii')}",
        )
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        sequence = 0
        while not self.stop.is_set():
            sequence, jpeg = self._next_jpeg(sequence)
            if jpeg is None:
                continue
            try:
                self.wfile.write(b"--" + MJPEG_BOUNDARY + b"\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
                break

    def _next_jpeg(self, sequence: int) -> tuple[int, bytes | None]:
        """Return the next JPEG after ``sequence``, or ``(sequence, None)`` on timeout.

        :param sequence: Last sent sequence number for this client.
        :returns: Updated sequence and bytes, or unchanged sequence and None.
        """

        assert self.hub is not None
        new_sequence, jpeg = self.hub.wait_next(sequence)
        if new_sequence == sequence:
            return sequence, None
        return new_sequence, jpeg

    def log_message(self, format: str, *args: object) -> None:
        """Prefix HTTP access logs for clarity.

        :param format: ``printf``-style format string.
        :param args: Values for the format string.
        :returns: None
        """

        sys.stderr.write("[tello-stream] " + (format % args) + "\n")


class CameraHTTPServer(ThreadingHTTPServer):
    """Threaded HTTP server with SO_REUSEADDR for quick restarts."""

    allow_reuse_address = True
    daemon_threads = True


def start_http_server(
    bind: str,
    port: int,
    hub: LiveHub,
    stop: threading.Event,
    telemetry: Any,
) -> CameraHTTPServer:
    """Start the browser server on a background thread.

    :param bind: Listen address (use ``0.0.0.0`` for all interfaces).
    :param port: TCP port (default 8080).
    :param hub: ``LiveHub`` the camera loop publishes JPEGs into.
    :param stop: Event set on shutdown so MJPEG handlers exit.
    :param telemetry: Object with ``snapshot()`` returning a JSON-serializable dict.
    :returns: Running ``CameraHTTPServer`` instance.
    :raises RuntimeError: If the port cannot be bound.
    """

    StreamHandler.hub = hub
    StreamHandler.stop = stop
    StreamHandler.telemetry = telemetry
    try:
        server = CameraHTTPServer((bind, port), StreamHandler)
    except OSError as exc:
        raise RuntimeError(
            f"Could not listen on {bind}:{port} ({exc}). "
            "Another process may be using the port."
        ) from exc
    thread = threading.Thread(target=server.serve_forever, name="tello-http", daemon=True)
    thread.start()
    return server


def _safe_call(fn: Callable[[], Any], default: Any = None) -> Any:
    """Call ``fn`` and return ``default`` on any exception.

    :param fn: Zero-argument callable (often a drone getter).
    :param default: Value used when ``fn`` raises.
    :returns: Result of ``fn()`` or ``default``.
    """

    try:
        return fn()
    except Exception:
        return default


def _finite_or_none(value: Any) -> float | None:
    """Return ``value`` as a finite float, or None if missing or non-finite (NaN/inf).

    JSON has no NaN; browsers reject it, so stats must use null for unknown sensors.

    :param value: Raw number or string from the drone SDK.
    :returns: Finite float, or None.
    """

    if value is None:
        return None
    try:
        if hasattr(value, "item") and callable(getattr(value, "item", None)):
            value = value.item()
        x = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(x) or math.isinf(x):
        return None
    return x


def _json_safe(obj: Any) -> Any:
    """Recursively convert telemetry to plain JSON types (no NaN, no numpy scalars).

    ``djitellopy`` sometimes returns ``numpy`` scalar types. Those are not JSON-serializable
    and can make ``/stats`` invalid; the browser then fails ``JSON.parse`` and the panel
    stays blank.

    :param obj: Dict/list or leaf value from a telemetry snapshot.
    :returns: Structure safe for ``json.dumps(..., allow_nan=False)``.
    """

    if obj is None:
        return None
    if isinstance(obj, bool):
        return obj
    if isinstance(obj, dict):
        return {str(k): _json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_json_safe(v) for v in obj]
    if isinstance(obj, str):
        return obj
    if isinstance(obj, numbers.Integral):
        return int(obj)
    if isinstance(obj, numbers.Real):
        x = float(obj)
        if math.isnan(x) or math.isinf(x):
            return None
        return x
    if hasattr(obj, "item") and callable(getattr(obj, "item", None)):
        try:
            return _json_safe(obj.item())
        except (AttributeError, TypeError, ValueError):
            return None
    return None


class TelloTelemetryPoller:
    """Background polling of ``djitellopy.Tello`` state for the stats panel.

    :param drone: Connected ``Tello`` instance with video stream already started.
    :param poll_hz: Samples per second (keep modest to avoid flooding the drone).
    :param wifi_refresh_s: Minimum seconds between ``wifi?`` queries (each query is a UDP round-trip and logs at INFO in djitellopy).
    """

    def __init__(
        self,
        drone: object,
        poll_hz: float = DEFAULT_TELEMETRY_HZ,
        wifi_refresh_s: float = DEFAULT_WIFI_REFRESH_S,
    ) -> None:
        self._drone = drone
        self._interval = 1.0 / max(poll_hz, 0.25)
        self._wifi_refresh_s = max(wifi_refresh_s, 0.0)
        self._last_wifi_mono = -1e9
        self._wifi_snr_cache: Any = None
        self._lock = threading.Lock()
        self._start = time.monotonic()
        self._snapshot: dict[str, Any] = {"ok": False, "error": "Starting…"}
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="tello-telemetry", daemon=True)
        self._thread.start()

    def snapshot(self) -> dict[str, Any]:
        """Thread-safe copy of the latest telemetry for ``GET /stats``.

        :returns: JSON-serializable dict including ``ok``, ``mode``, and fields used by PAGE_HTML.
        """

        with self._lock:
            return json.loads(json.dumps(_json_safe(self._snapshot), allow_nan=False))

    def close(self) -> None:
        """Stop the poller thread.

        :returns: None
        """

        self._stop.set()
        self._thread.join(timeout=2.0)

    def _loop(self) -> None:
        """Repeatedly query the drone and refresh ``_snapshot``.

        :returns: None
        """

        d = self._drone
        while not self._stop.is_set():
            try:
                battery = int(_safe_call(lambda: d.get_battery(), -1))
                if battery < 0:
                    battery = None

                pitch = _finite_or_none(_safe_call(lambda: d.get_pitch(), None))
                roll = _finite_or_none(_safe_call(lambda: d.get_roll(), None))
                yaw = _finite_or_none(_safe_call(lambda: d.get_yaw(), None))

                ax = _finite_or_none(_safe_call(lambda: d.get_acceleration_x(), None))
                ay = _finite_or_none(_safe_call(lambda: d.get_acceleration_y(), None))
                az = _finite_or_none(_safe_call(lambda: d.get_acceleration_z(), None))

                gx = _finite_or_none(_safe_call(lambda: d.get_angular_velocity_x(), None))
                gy = _finite_or_none(_safe_call(lambda: d.get_angular_velocity_y(), None))
                gz = _finite_or_none(_safe_call(lambda: d.get_angular_velocity_z(), None))

                vx = _finite_or_none(_safe_call(lambda: d.get_speed_x(), None))
                vy = _finite_or_none(_safe_call(lambda: d.get_speed_y(), None))
                vz = _finite_or_none(_safe_call(lambda: d.get_speed_z(), None))

                height = _safe_call(lambda: d.get_height(), None)
                baro = _safe_call(lambda: d.get_barometer(), None)
                tof = _safe_call(lambda: d.get_distance_tof(), None)
                flight_time = _safe_call(lambda: d.get_flight_time(), None)

                temp = _safe_call(lambda: d.get_temperature(), None)
                if temp is None:
                    t_lo = _safe_call(lambda: d.get_lowest_temperature(), None)
                    t_hi = _safe_call(lambda: d.get_highest_temperature(), None)
                    if t_lo is not None and t_hi is not None:
                        temp = (float(t_lo) + float(t_hi)) / 2.0
                    elif t_lo is not None:
                        temp = float(t_lo)
                    elif t_hi is not None:
                        temp = float(t_hi)

                now_mono = time.monotonic()
                if self._wifi_refresh_s <= 0.0 or now_mono - self._last_wifi_mono >= self._wifi_refresh_s:
                    self._wifi_snr_cache = _safe_call(lambda: d.query_wifi_signal_noise_ratio(), None)
                    self._last_wifi_mono = now_mono
                wifi = self._wifi_snr_cache

                payload = {
                    "ok": True,
                    "mode": "tello",
                    "uptime_s": time.monotonic() - self._start,
                    "updated_at": datetime.now().isoformat(timespec="seconds"),
                    "telemetry": {
                        "battery_percent": battery,
                        "height_cm": _finite_or_none(height),
                        "baro_m": _finite_or_none(baro),
                        "tof_cm": _finite_or_none(tof),
                        "flight_time_s": _finite_or_none(flight_time),
                        "temp_c": _finite_or_none(temp),
                        "wifi_snr": _finite_or_none(wifi),
                        "imu": {
                            "acceleration": {"x": ax, "y": ay, "z": az},
                            "gyroscope": {"x": gx, "y": gy, "z": gz},
                            "rotation": {"roll": roll, "pitch": pitch, "yaw": yaw},
                        },
                        "velocity": {"x": vx, "y": vy, "z": vz},
                    },
                }
                with self._lock:
                    self._snapshot = payload
            except Exception as exc:
                with self._lock:
                    self._snapshot = {
                        "ok": False,
                        "mode": "tello",
                        "error": f"Telemetry read failed: {exc}",
                    }
            time.sleep(self._interval)


class WebcamTelemetryPoller:
    """Lightweight stats when using a local camera instead of a drone.

    :param source_label: Human-readable source string for the panel.
    :param dims_fn: Callable returning ``(width, height)`` for the current frame.
    :param frames_counter: Callable returning total frames published so far.
    :param fps_ema: Smoothed frames-per-second estimate updated by the capture loop.
    """

    def __init__(
        self,
        source_label: str,
        dims_fn: Callable[[], tuple[int, int]],
        frames_counter: Callable[[], int],
        fps_ema: list[float],
    ) -> None:
        self._source_label = source_label
        self._dims_fn = dims_fn
        self._frames_counter = frames_counter
        self._fps_ema = fps_ema
        self._start = time.monotonic()
        self._snapshot: dict[str, Any] = {"ok": False, "error": "Starting…", "mode": "webcam"}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="webcam-telemetry", daemon=True)
        self._thread.start()

    def snapshot(self) -> dict[str, Any]:
        """Return webcam-oriented stats for ``GET /stats``.

        :returns: Dict with ``ok``, ``mode`` ``webcam``, and ``stream`` metadata.
        """

        with self._lock:
            return json.loads(json.dumps(_json_safe(self._snapshot), allow_nan=False))

    def close(self) -> None:
        """Stop the background refresh thread.

        :returns: None
        """

        self._stop.set()
        self._thread.join(timeout=2.0)

    def _loop(self) -> None:
        """Refresh the cached snapshot a few times per second.

        :returns: None
        """

        while not self._stop.is_set():
            w, h = self._dims_fn()
            with self._lock:
                self._snapshot = {
                    "ok": True,
                    "mode": "webcam",
                    "uptime_s": time.monotonic() - self._start,
                    "updated_at": datetime.now().isoformat(timespec="seconds"),
                    "stream": {
                        "source": self._source_label,
                        "width": w,
                        "height": h,
                        "fps": self._fps_ema[0] if self._fps_ema else 0.0,
                        "frames": self._frames_counter(),
                    },
                }
            time.sleep(0.25)


def _configure_djitellopy_logging(verbose: bool = False) -> None:
    """Set djitellopy log levels so routine command traffic does not flood stderr.

    :param verbose: If True, keep INFO on djitellopy loggers; if False, use WARNING.
    :returns: None
    """

    level = logging.INFO if verbose else logging.WARNING
    for name in ("djitellopy", "djitellopy.tello"):
        logging.getLogger(name).setLevel(level)


def _run_tello(
    bind: str,
    port: int,
    stop: threading.Event,
    hub: LiveHub,
    verbose: bool = False,
    wifi_refresh_s: float = DEFAULT_WIFI_REFRESH_S,
) -> None:
    """Connect to a Tello, start video, publish frames to ``hub``, and block until ``stop``.

    :param bind: HTTP bind address (informational; HTTP server started by caller).
    :param port: HTTP port (informational).
    :param stop: Set externally to end the capture loop.
    :param hub: JPEG mailbox for MJPEG clients.
    :param verbose: Pass True to print per-command djitellopy INFO logs (``wifi?``, etc.).
    :param wifi_refresh_s: Minimum seconds between ``wifi?`` queries; ``0`` means every poll (original pacing).
    :returns: None
    """

    try:
        from djitellopy import Tello
    except ImportError as exc:
        raise SystemExit(
            "djitellopy is required for drone mode. Install with: pip install djitellopy"
        ) from exc

    _configure_djitellopy_logging(verbose=verbose)

    import cv2

    print("Connecting to Tello (join the drone Wi‑Fi first: usually TELLO-…)…")
    drone = Tello()
    drone.connect()
    drone.streamon()
    reader = drone.get_frame_read()

    poller = TelloTelemetryPoller(drone, wifi_refresh_s=wifi_refresh_s)
    server = start_http_server(bind, port, hub, stop, poller)

    print(f"Open http://127.0.0.1:{port}/ in a browser (or http://<your-LAN-IP>:{port}/).")
    print("Ctrl+C to stop.")

    try:
        while not stop.is_set():
            frame = reader.frame
            if frame is None or (hasattr(frame, "size") and frame.size == 0):
                time.sleep(0.01)
                continue
            hub.publish(frame)
    finally:
        stop.set()
        hub.wake()
        poller.close()
        _safe_call(lambda: drone.streamoff(), None)
        drone.end()
        server.shutdown()


def _run_webcam(
    bind: str,
    port: int,
    stop: threading.Event,
    hub: LiveHub,
    source: str,
) -> None:
    """Capture from OpenCV ``VideoCapture`` and serve the same HTTP UI.

    :param bind: HTTP listen address.
    :param port: HTTP port.
    :param stop: Event to end capture.
    :param hub: JPEG mailbox.
    :param source: Camera index or path passed to ``cv2.VideoCapture``.
    :returns: None
    """

    import cv2

    cap = cv2.VideoCapture(int(source) if source.isdigit() else source)
    if not cap.isOpened():
        raise SystemExit(f"Could not open camera source {source!r}")

    frames = 0
    dims_cache: list[int] = [0, 0]
    fps_ema: list[float] = [0.0]
    last_t = time.monotonic()
    frame_interval: list[float] = [0.0]

    def dims() -> tuple[int, int]:
        return int(dims_cache[0]), int(dims_cache[1])

    def nframes() -> int:
        return frames

    label = f"webcam:{source}"
    poller = WebcamTelemetryPoller(label, dims, nframes, fps_ema)
    server = start_http_server(bind, port, hub, stop, poller)

    print(f"Webcam streaming. Open http://127.0.0.1:{port}/")
    print("Ctrl+C to stop.")

    try:
        while not stop.is_set():
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.02)
                continue
            frames += 1
            h, w = frame.shape[:2]
            dims_cache[0], dims_cache[1] = w, h
            now = time.monotonic()
            dt = now - last_t
            last_t = now
            if dt > 0:
                inst_fps = 1.0 / dt
                fps_ema[0] = fps_ema[0] * 0.85 + inst_fps * 0.15
            hub.publish(frame)
    finally:
        stop.set()
        hub.wake()
        poller.close()
        cap.release()
        server.shutdown()


def main(argv: list[str] | None = None) -> int:
    """Parse CLI flags and run the HTTP camera + telemetry server.

    :param argv: Optional argument list (defaults to ``sys.argv[1:]``).
    :returns: Process exit code (0 on clean shutdown).
    """

    parser = argparse.ArgumentParser(
        description="Tello / RoboMaster TT camera stream with XGO-style stats panel.",
    )
    parser.add_argument("--bind", default=DEFAULT_BIND, help="HTTP bind address (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="HTTP port (default: 8080)")
    parser.add_argument(
        "--camera",
        default=None,
        metavar="SRC",
        help="Use a local OpenCV camera instead of the drone: index (0) or video path. "
        "If omitted, djitellopy + Tello is used.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Show djitellopy INFO logs (every UDP command such as wifi?).",
    )
    parser.add_argument(
        "--wifi-interval",
        type=float,
        default=DEFAULT_WIFI_REFRESH_S,
        metavar="SEC",
        help="Minimum seconds between wifi? queries (0 = every telemetry poll, original pacing; default: 0).",
    )
    args = parser.parse_args(argv)

    hub = LiveHub()
    stop = threading.Event()

    def handle_sigint(*_a: object) -> None:
        print("\nStopping…", file=sys.stderr)
        stop.set()
        hub.wake()

    try:
        import signal

        signal.signal(signal.SIGINT, handle_sigint)
        if sys.platform != "win32":
            signal.signal(signal.SIGTERM, handle_sigint)
    except Exception:
        pass

    try:
        if args.camera is not None:
            _run_webcam(args.bind, args.port, stop, hub, args.camera)
        else:
            _run_tello(
                args.bind,
                args.port,
                stop,
                hub,
                verbose=args.verbose,
                wifi_refresh_s=args.wifi_interval,
            )
    except KeyboardInterrupt:
        stop.set()
        hub.wake()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
