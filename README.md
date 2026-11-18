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

## Find a pad and map the dog's route (`scripts/search/`)

The drone takes off from mission pad A beside the dog and sweeps the floor in lanes until it sees the target mission pad (with a second pass between the lanes if it missed it). On the way it marks every 20 cm square as free or blocked using its downward distance sensor (median of 3 readings; `--obstacle-signal tof+baro` also uses the barometer, in case the drone climbs over boxes by itself). It then flies the dog's route home, keeping half the dog's width + `--margin` + half a square away from anything blocked, re-planning if a square turns out blocked. Over pad A it measures how far its tracking drifted and corrects the whole map. Then, while the battery allows, it goes back out to read the unread squares beside the route, so the dog's whole body width is on floor the drone has read, and corrects again over pad A before landing. Below 20% battery it stops and comes home; below 10% it lands where it is. Only use a route the map marks `route_checked: true`. The map and route for the dog are printed and saved to `--out` (`map.json`). Sending them to the dog comes later, through the link module.

Lay-out:
- Pad A goes beside the dog, with its rocket pointing the way the dog faces. That is the map's +x; +y is left.
- Every other pad (target, checkpoints) also has its rocket pointing the same way.
- Put the drone on pad A, nose along the rocket. Give it a well-lit, textured floor.

```bash
.venv/bin/python scripts/search/find_pad.py --check    # no flight: live pad ID/offset, floor distance, yaw
.venv/bin/python scripts/search/find_pad.py --target 2 --forward 200 --left 60 --right 60 --dog-width 30
.venv/bin/python scripts/search/find_pad.py ... --pad 3=100,0   # extra pad at a measured spot, fixes drift
.venv/bin/python -m unittest discover -s scripts/search
```

To watch a run without the drone, `scripts/search/simulate.py` flies the same mission code against a simulated drone in a made-up room and opens a replay page (`sim.html`). It shows where the drone really is versus where it thinks it is, the squares it marks, the dog's route, and the squares beside the route it never checked.

```bash
.venv/bin/python scripts/search/simulate.py --list                 # room, box, climb, drift, overshoot
.venv/bin/python scripts/search/simulate.py --scenario drift
.venv/bin/python scripts/search/simulate.py --scenario random --seed 7
.venv/bin/python scripts/search/simulate.py --scenario climb --signal tof   # box missed without the barometer
```

The simulated drone gets noisy and occasionally garbage distance readings, missed pad detections, a creeping heading and an 11-minute battery unless you pass `--ideal`. Those numbers are guesses (`REALISM` in `simulate.py`); replace them with measurements from the real drone.

Not yet confirmed on the drone (check these before trusting a full run):
- Which way the pad x/y offsets and `go` y-axis point (`PAD_X_SIGN`/`PAD_Y_SIGN` in `tracker.py`, `GO_Y_SIGN` in `find_pad.py`). Use `--check` and slide the drone over a pad by hand.
- Whether the drone climbs by itself over a box, which would hide the box from the distance sensor. `map.json` logs distance, height and barometer at every step for this.
- How much floor the pad camera sees, which sets `--lanes` (default 40 cm).

Pad A only catches drift that doesn't cancel out on the way home. If every move overshoots by the same fraction, the overshoot cancels on the return and only checkpoint pads (`--pad`) can catch it.

## Whole mission (`scripts/run_mission.py`)

Drone search and route check, then the dog walks the route, all through `tello_xgo_bridge`. It checks the dog answers before flying, stops if the route isn't fully checked, and asks you to lift the drone off pad A before the dog moves.

```bash
.venv/bin/python scripts/run_mission.py --sim                     # simulated drone and dog
.venv/bin/python scripts/run_mission.py --pi http://100.74.30.90:8080 --target 2 \
    --forward 200 --left 80 --right 60 --dog-width 30 --walk-speed-cm 6.5
```

The dog's walking speed (`--walk-speed-cm`, cm/s at walk 8) has not been measured yet. See `scripts/dog/README.md`.

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
