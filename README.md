# drone-robot

Computer-vision drone project on a DJI RoboMaster TT (Tello Talent), driven from a laptop with [djitellopy](https://github.com/damiafuentes/DJITelloPy).

## Setup (macOS)

```bash
/opt/homebrew/bin/python3.13 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## Connecting

1. Set the switch on the TT's expansion board to direct (AP) mode and power on the drone.
2. Join the drone's Wi-Fi (`RMTT-xxxxxx`) from the laptop. It has no internet.
3. Check the link without flying:

```bash
.venv/bin/python scripts/up_down.py --check
```

## First flight: straight up and down

```bash
.venv/bin/python scripts/up_down.py                       # takeoff (~80 cm), land immediately
.venv/bin/python scripts/up_down.py --climb 20 --hover 1  # takeoff, +20 cm, hover 1 s, -20 cm, land
```

The ~80 cm takeoff height is set by the drone's firmware and can't be lowered from the SDK. `--climb` is capped at 50 cm.
Add `--ignore-temp` to fly even when the drone reports 90 C or more.

Press Ctrl+C at any point to land.

Before flying:
- Clear at least 2 m around the drone and check there's room overhead (it climbs to ~80 cm on takeoff).
- The drone holds position with its downward camera and sensor. Give it a well-lit floor with some texture (not glossy or plain carpet), or it will drift sideways even though the script only sends vertical commands.
- Keep the battery above 25% (the script refuses below that).

## Tello camera stream (local MJPEG + stats panel)

Single-file HTTP server with an XGO-style layout: full-viewport camera feed and a top-right telemetry card (`/` → HTML, `/stream` → MJPEG, `/stats` → JSON).

Install dependencies **while you still have internet** (the drone hotspot has no PyPI access), using the setup step above. Then join the drone's Wi-Fi and run:

```bash
.venv/bin/python tello_camera_stream.py
```

Open `http://127.0.0.1:8080/` in a browser.

- Webcam / testing: `python tello_camera_stream.py --camera 0`
- Verbose djitellopy UDP logs: `--verbose`
- Throttle `wifi?` polls: `--wifi-interval 5` (seconds; `0` = every telemetry tick)
