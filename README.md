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

## Tello + XGO bridge (`tello_xgo_bridge`)

Laptop package that talks to **both** the Tello and the XGO Pi with the same `read` / `write` API. See `tello_xgo_bridge/README.md` for hardware (Wi‑Fi + USB Ethernet) and routing.

```bash
pip install -r requirements.txt
python -m tello_xgo_bridge routes apply   # Administrator / sudo after joining Tello Wi‑Fi
```

```python
from tello_xgo_bridge import TelloXgoBridge
bridge = TelloXgoBridge(pi_url="http://100.x.x.x:8080")
bridge.open()
bridge.tello.read()
bridge.xgo.write(walk=0, yaw=0)
bridge.close()
```

Pi gait over the network: copy `pi_xgo_bridge/remote_command.py` into your Pi `xgo_robot` folder and merge the `POST /command` handler into `http_stream.py` (see `pi_xgo_bridge/README.md`).

Tests: `python -m pytest tello_xgo_bridge/tests -q -p no:asyncio -p no:anyio`
