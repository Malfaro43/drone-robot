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

The drone takes off from mission pad A beside the dog and sweeps the floor in lanes until it sees the target mission pad. On the way it marks every 20 cm square as free or blocked using its downward distance sensor. It then works out the dog's route home, keeping half the dog's width plus `--margin` away from anything blocked, and flies that route to check each square, re-planning if one turns out blocked. Back over pad A it measures how far its tracking drifted and corrects the whole map, then lands. The map and route for the dog are printed and saved to `--out` (`map.json`). Sending them to the dog comes later, through the link module.

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
.venv/bin/python scripts/search/simulate.py --list                 # room, box, drift, overshoot
.venv/bin/python scripts/search/simulate.py --scenario drift
.venv/bin/python scripts/search/simulate.py --scenario random --seed 7
```

Not yet confirmed on the drone (check these before trusting a full run):
- Which way the pad x/y offsets and `go` y-axis point (`PAD_X_SIGN`/`PAD_Y_SIGN` in `tracker.py`, `GO_Y_SIGN` in `find_pad.py`). Use `--check` and slide the drone over a pad by hand.
- Whether the drone climbs by itself over a box, which would hide the box from the distance sensor. `map.json` logs distance, height and barometer at every step for this.
- How much floor the pad camera sees, which sets `--lanes` (default 40 cm).

Pad A only catches drift that doesn't cancel out on the way home. If every move overshoots by the same fraction, the overshoot cancels on the return and only checkpoint pads (`--pad`) can catch it.
