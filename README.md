# drone-robot

Drone robot project.

## Tello camera stream (local MJPEG + stats panel)

Single-file HTTP server with an XGO-style layout: full-viewport camera feed and a top-right telemetry card (`/` → HTML, `/stream` → MJPEG, `/stats` → JSON).

Install dependencies **while you still have internet** (the drone hotspot usually has no PyPI access):

```bash
pip install -r requirements.txt
```

Join the Tello Wi‑Fi, then:

```bash
python tello_camera_stream.py
```

Open `http://127.0.0.1:8080/` in a browser.

- Webcam / testing: `python tello_camera_stream.py --camera 0`
- Verbose djitellopy UDP logs: `--verbose`
- Throttle `wifi?` polls: `--wifi-interval 5` (seconds; `0` = every telemetry tick)
