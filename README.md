# drone-robot

A DJI RoboMaster TT (Tello Talent) drone and an XGO Lite robot dog working together, both driven from one laptop. The drone takes off beside the dog, searches the floor for a target mission pad, maps which 20 cm squares are free or blocked, and checks a route wide enough for the dog. Then the dog walks that route to the target.

Everything can run in simulation; the real-hardware steps are in [`docs/HARDWARE_DAY.md`](docs/HARDWARE_DAY.md).

## How it fits together

```
laptop ── run_mission.py
           ├── drone (Wi-Fi, djitellopy) ── mission.py: search, map, check the route, land on pad A
           │                                 └─ writes the map + dog route (map.json)
           └── dog (Tailscale HTTP to the Pi) ── follow_route.py: walk the route by gyro + timing
                 both through tello_xgo_bridge (one session, command lock, dog watchdog)
```

- **Map frame:** origin = centre of mission pad A (beside the dog), +x = the way pad A's rocket points (the dog faces that way), +y = left, in cm.
- **Safety rules the code enforces:** the dog only walks a route the drone marked `route_checked` (every square under the dog's body, plus a margin, read while the drone was stopped); the Pi stops the dog if the laptop goes quiet for 1 s; every flight lands in a `finally` and on Ctrl+C.

## Repository map

| Path | What it is |
|---|---|
| `scripts/run_mission.py` | Whole mission: drone search, then dog walk, through the bridge (`--sim` for simulation) |
| `scripts/search/mission.py` | The drone's mission (search, floor map, route checks, drift correction, battery rules) |
| `scripts/search/find_pad.py` | Command line for the drone mission alone, plus `--check` (live sensor readout, no flight) |
| `scripts/search/tracker.py`, `floormap.py`, `sweep.py` | Position tracking, the free/blocked grid and dog routes, search patterns |
| `scripts/search/simulate.py`, `sim_drone.py`, `sim_template.html` | Simulated drone and the replay page |
| `scripts/search/stress.py` | Runs the mission in hundreds of random rooms and scores it against the real layout |
| `scripts/dog/follow_route.py`, `sim_dog.py` | Walks the dog along a route from the laptop; simulated dog |
| `scripts/calibrate.py`, `calibration.py` | Guided measurements saved to `calibration.json` |
| `scripts/up_down.py`, `flight_common.py` | First vertical-only test flight; shared preflight checks |
| `tello_xgo_bridge/` | Laptop bridge: one object for the drone (`.tello`) and the dog (`.xgo`) |
| `pi_xgo_bridge/` | Pi-side command parser and watchdog (a copy lives in the XGO repo's `scripts/xgo_robot/`) |
| `tello_camera_stream.py` | Drone camera in a browser with a telemetry panel |
| `docs/HARDWARE_DAY.md` | Runbook for the first session with the real robots |

## Setup (macOS)

```bash
/opt/homebrew/bin/python3.13 -m venv .venv
.venv/bin/pip install -r requirements-dev.txt     # runtime + test tools
```

Install while you have internet: the drone's Wi-Fi has none.

## Tests

```bash
.venv/bin/python -m pytest          # every suite, ~3 s, no hardware
```

GitHub runs the same on every push (`.github/workflows/tests.yml`), plus the Pi-side command code on Python 3.9, the version on the dog.

## Try it without hardware

```bash
.venv/bin/python scripts/run_mission.py --sim                       # drone search + dog walk, simulated
.venv/bin/python scripts/search/simulate.py --scenario box           # replay page of a drone mission
.venv/bin/python scripts/search/simulate.py --list                   # room, box, climb, drift, overshoot
.venv/bin/python scripts/search/simulate.py --scenario random --seed 7
.venv/bin/python scripts/search/stress.py --rooms 500                # score over 500 random rooms
```

The replay page shows where the drone really is against where it thinks it is, the squares it marks (free, probably free, blocked), the dog's route, and any squares beside the route it never checked. The simulated drone has noisy and occasionally garbage readings, missed pad detections, a creeping heading, drift and an 11-minute battery unless you pass `--ideal`; those numbers are guesses (`REALISM` in `simulate.py`) to be replaced with measurements.

## With the hardware

Follow [`docs/HARDWARE_DAY.md`](docs/HARDWARE_DAY.md) the first time. In short:

1. **Connect.** Drone expansion-board switch to direct (AP) mode, join its `RMTT-…` Wi-Fi, run from **Apple Terminal** (Ghostty doesn't get the drone's replies). For the dog at the same time, plug in USB Ethernet and run `.venv/bin/python -m tello_xgo_bridge routes` (see `tello_xgo_bridge/README.md`).
2. **Calibrate** (`scripts/calibrate.py`, below). Don't fly a search before `drone-axes`: the map could come out mirrored.
3. **Lay out:** pad A beside the dog, rocket pointing the way the dog faces; every other pad with its rocket the same way; drone on pad A, nose along the rocket; a well-lit, textured floor.
4. **Run:**

```bash
.venv/bin/python scripts/run_mission.py --pi http://100.74.30.90:8080 --target 2 \
    --forward 200 --left 80 --right 60 --dog-width 30
```

Or the halves on their own: `scripts/search/find_pad.py` (drone only, writes `map.json`) and `scripts/dog/follow_route.py map.json --pi …` (dog only).

### First flight: straight up and down

```bash
.venv/bin/python scripts/up_down.py --check                  # connect + battery only
.venv/bin/python scripts/up_down.py                          # takeoff (~80 cm), land immediately
.venv/bin/python scripts/up_down.py --climb 20 --hover 1     # +20 cm, hover 1 s, back down, land
```

The ~80 cm takeoff height is set by the firmware. `--climb` is capped at 50 cm; it refuses below 25% battery or at 90 °C (`--ignore-temp`). Clear 2 m around the drone; Ctrl+C lands.

### Calibration

```bash
.venv/bin/python scripts/calibrate.py show                     # what's measured, what's a guess
.venv/bin/python scripts/calibrate.py drone-axes               # NO flight: hold the drone ~50 cm over pad A, move it by hand
.venv/bin/python scripts/calibrate.py drone-hover              # short flight over pad A (+ optional box test)
.venv/bin/python scripts/calibrate.py dog --pi http://100.74.30.90:8080   # turns, then walks ~50 cm; you tape-measure it
```

Results go to `calibration.json` (commit it, so everyone uses the same numbers). The real-hardware scripts read it and warn about anything still unmeasured; the simulation and tests don't.

## The drone mission in detail

1. Take off over pad A, take the floor-distance and barometer baselines.
2. **Search** in lanes (`--lanes`, default 40 cm apart). Straight runs are flown as single moves of up to 1 m (`--glide`, default 5 squares; `--glide 1` stops every 20 cm) while the drone's state stream is recorded; floor readings taken on the way count as *probably free* or *blocked*. Long moves still stop on pad A. If the target is missed, a second pass runs between the lanes, if the battery allows.
3. **Centre over the target** (flying back if it was glimpsed mid-move).
4. **Fly the dog's route home**, checking every square while stopped and re-planning if one is blocked. Routes keep half the dog's width + `--margin` + half a square from anything blocked, and from the edge of the search area.
5. **Correct the drift over pad A** (spiralling out to find it if needed); the correction is spread back over the flight by distance flown, so the target and the map move too.
6. **Check beside the route**: read the unread or only-probably-free squares within the dog's clearance, going back to pad A after every 5 m for a fresh correction.
7. Land and write `map.json`: target position, the dog's route (`route_checked` true only if every square under the dog's body was read while stopped), the grid, the drift measured.

**Obstacles:** the downward distance sensor (median of 3 readings) marks a square blocked if it reads much shorter (a box) or much longer (a step down) than the baseline. If the drone turns out to climb over boxes by itself (the `drone-hover` box test), `--obstacle-signal tof+baro` also watches the barometer; calibration switches it on automatically.

**Battery:** the mission turns home when the battery only just covers the trip back (measured use per second × the trip's length, ×1.5); below 10% it lands where it is.

**Drift:** pad A corrects drift that doesn't cancel out on the way home. If every move overshoots by the same fraction, the overshoot cancels on the return and only extra pads at measured spots (`--pad 3=100,0`) catch it.

## Tello camera stream

`tello_camera_stream.py` serves the drone camera with a telemetry card (`/` page, `/stream` MJPEG, `/stats` JSON) at `http://127.0.0.1:8080/`. Run it on its own: only one program may control the drone at a time.

```bash
.venv/bin/python tello_camera_stream.py                 # --camera 0 for a webcam, --verbose for UDP logs
```

## Tello + XGO bridge

`tello_xgo_bridge` gives one laptop object for both robots. See [`tello_xgo_bridge/README.md`](tello_xgo_bridge/README.md) for the Wi-Fi + Ethernet routing.

```python
from tello_xgo_bridge import TelloXgoBridge
with TelloXgoBridge(pi_url="http://100.x.x.x:8080") as bridge:   # always stops the dog at the end
    bridge.tello.read()                   # drone telemetry (state stream only)
    bridge.tello.drone                    # the djitellopy.Tello, commands serialised
    bridge.xgo.read()                     # dog stats incl. gyro heading
    bridge.xgo.write(walk=8, yaw=0)       # xgolib speed values (Lite: ±25 / ±100); resend often
```

On the Pi, the XGO repo's `scripts/xgo_robot/` serves `POST /command` with a 1 s watchdog: `cd scripts/xgo_robot && python3 main.py stream`.

## Not yet confirmed on the real robots

- Sign conventions (pad x/y, heading, `go` sideways direction): `calibrate.py drone-axes` and `drone-hover`.
- How far from a pad the drone still sees it (sets lane spacing), and whether it climbs over boxes.
- How accurately readings taken mid-move can be placed: the code assumes steady speed through each move and doesn't trust mid-move "free" readings for the dog because of it.
- The dog's walking speed and smallest working turn command: `calibrate.py dog`.
- Battery life and drift rates: the simulation's numbers are guesses.
