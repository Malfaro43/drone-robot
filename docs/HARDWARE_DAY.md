# First hardware session: runbook

Everything up to now has only run in simulation. This is the order for the first session with the real drone and dog: each step needs the previous one, and each has a pass/fail check. Stop at the first failure; the later steps rely on it. Commands run from `drone-robot/` on the laptop unless they say "on the Pi".

Bring: pad A and the target pad (mission pads), a tape measure, a box at least 15 cm tall, a USB Ethernet adapter + cable to a live jack, charged drone and dog batteries.

## 0. Before the session (no robots)

| Check | How | Pass |
|---|---|---|
| Laptop code is current | `git pull` in the umbrella repo, then `git submodule update` | No errors |
| All tests pass | `.venv/bin/python -m unittest discover -s scripts/search`, `-s scripts/dog`, `-s scripts -p 'test_calibrate.py'`, `.venv/bin/python -m pytest tello_xgo_bridge/tests -q -p no:asyncio -p no:anyio` | All OK |
| What's still a guess | `.venv/bin/python scripts/calibrate.py show` | Lists 11 guesses (normal before calibrating) |

## 1. Update the Pi (dog powered, standing still)

The dog needs the new `POST /command` handler with its 1 s watchdog (XGO repo, `scripts/xgo_robot/`).

```bash
ssh -o ConnectTimeout=8 pi@100.74.30.90
cd ~/project/xgo_robot_kit_playing_fetch
git status --short --branch          # on the Pi
```

- **Clean and only behind:** `git pull --ff-only origin main`.
- **Local changes or ahead of origin:** stop. As of 2026-10-30 this checkout held an unpushed commit and the robot ran code from `~/project/deliver-test`. Don't reset or force anything; sort out with whoever owns those changes first.
- Check `df -h /` first: the Pi's disk has filled up before, and `git pull` fails with "No space left".

Then start the server with telemetry on, from inside the package folder (`cd scripts/xgo_robot && python3 main.py stream`), and leave that terminal open. Before you do:

- **The serial port must be free.** The vendor boot menu normally holds `/dev/ttyAMA0`; two programs on it corrupt each other's replies. Check with `sudo fuser -v /dev/ttyAMA0` and `ps -eo pid,user,args | grep -E '[p]ython.*main.py'`. Stopping the menu (`sudo pkill -f '[m]ain.py'`) also stops anything else called `main.py`, so check the list first. See the XGO repo's `AGENTS.md` §5.
- **Starting it moves the dog.** Telemetry opens the dog with xgolib, which resets it and stands it up. Have it on the floor with room around it.

| Check | Pass |
|---|---|
| `curl http://100.74.30.90:8080/stats` from the laptop | JSON with `"ok": true` and an `imu.rotation.yaw` |
| Lift the dog so its feet are off the ground, then send one command and nothing else: `curl -X POST -d '{"walk": 8, "yaw": 0}' http://100.74.30.90:8080/command` | Legs move, then **stop by themselves about 1 s later** (the watchdog) |
| `curl -X POST -d '{"walk": 40, "yaw": 0}' http://100.74.30.90:8080/command` | Refused with a 400 error; nothing moves |

If the legs don't stop by themselves, **stop here**: the dog would keep walking whenever the laptop drops out.

## 2. Network: drone and dog at the same time

Join the drone's `RMTT-…` Wi-Fi, plug in USB Ethernet, open **Apple Terminal** (Ghostty can't receive the drone's replies).

| Check | How | Pass |
|---|---|---|
| Routes split | `.venv/bin/python -m tello_xgo_bridge routes` (then `routes apply` with sudo if it says so) | Tello on Wi-Fi, uplink on Ethernet |
| Both answer | `.venv/bin/python scripts/search/find_pad.py --check` in one window, `curl …/stats` in another | Live drone readings and dog JSON at the same time |

## 3. Drone calibration, no flying: `drone-axes`

```bash
.venv/bin/python scripts/calibrate.py drone-axes
```

Motors off. Hold the drone level about **50 cm above pad A** (the camera can't see a pad from closer than about 30 cm), nose along the pad's rocket. It asks you to hold it over the centre, move it ~15 cm forward, ~15 cm left, then turn it ~45° clockwise.

Pass: it saves `pad_x_sign`, `pad_y_sign`, `yaw_cw_positive`. If it says the pad isn't seen, hold it higher and steadier. **Don't fly a search until this has passed**: until then the map may come out mirrored.

## 4. Drone calibration, short flight: `drone-hover`

Clear 1 m around pad A. Put the box ready but out of the way.

```bash
.venv/bin/python scripts/calibrate.py drone-hover
```

It takes off over pad A, measures sensor noise, moves 20 cm sideways and back, flies up to 60 cm forward to find where it loses the pad, and offers the box test: put the box ~60 cm in front of pad A, in line with the rocket, when it asks.

Pass: it saves `go_y_sign`, `pad_range_cm`, `tof_noise_cm`, `baro_noise_cm`, and (with the box) `auto_climb`. Watch it the whole time; Ctrl+C lands.

- If `pad_range_cm` is 10 or less: the drone barely sees pads, so lanes 40 cm apart will miss the target. Use `--lanes 20`.
- If `auto_climb` is true: the distance sensor can't see boxes; the mission switches to the barometer automatically.

## 5. Dog calibration: `dog`

Clear 1 m around the dog, on the floor it will walk on. Stay by it.

```bash
.venv/bin/python scripts/calibrate.py dog --pi http://100.74.30.90:8080
```

It stands still 5 s (gyro drift), turns a few times, then walks straight ~50 cm. Mark the front feet when it asks, and type the distance it walked in cm.

Pass: it saves `walk_cm_per_s`, `turn_deg_per_s`, `min_turn`. If it refuses because the gyro drifts too much, restart `main.py stream` on the Pi and try again.

Check with `calibrate.py show` that the drone and dog values now say "measured". **Commit `calibration.json`** so everyone uses the same numbers.

## 6. First search: small, no boxes

Pad A beside the dog's starting spot, rocket pointing the way the dog faces; target pad ~1 m ahead. Clear the area.

```bash
.venv/bin/python scripts/search/find_pad.py --target 2 --forward 120 --left 40 --right 40 \
    --dog-width 30 --glide 1 --out first.json
```

`--glide 1` stops every 20 cm: slower, but simplest to watch. Pass:
- the reported target position is within ~10 cm of a tape measurement;
- `drift_at_home_cm` is small (under ~10 cm);
- it lands on pad A.

Then the same with the default `--glide 5` (long moves). Compare the time taken and the target position.

## 7. First dog walk on a mapped route

Use `first.json` from step 6 (`route_checked` must be true). Lift the drone off pad A. The dog stands beside pad A (default `--start 0,30`: 30 cm to its left), facing the rocket direction.

```bash
.venv/bin/python scripts/dog/follow_route.py first.json --pi http://100.74.30.90:8080 --start 0,30
```

Pass: the dog ends within ~20 cm of the target pad. Note how far off it was; that's the number the next stage (final approach with the dog's camera) has to close.

## 8. Whole mission with one box

Put the box between pad A and the target, leaving a gap of at least 70 cm on one side (dog 30 cm + 10 cm margin each side + half a square).

```bash
.venv/bin/python scripts/run_mission.py --pi http://100.74.30.90:8080 --target 2 \
    --forward 200 --left 80 --right 60 --dog-width 30
```

Pass: the route goes round the box, `route_checked` is true, and the dog walks it without touching the box.

## What to write down

For each step: pass/fail, the numbers it printed, battery at the start and end, and anything surprising. Especially useful for the simulation: how long one 20 cm `go` and one 100 cm `go` actually take, battery % used per minute of flight, and how far the drone drifts over a minute of hovering. These replace guesses in `scripts/search/simulate.py` (`REALISM`).

## Stop immediately if

- the dog keeps moving after the laptop stops sending (step 1);
- the drone drifts more than ~30 cm while hovering over pad A;
- any map comes out mirrored (the target reported on the wrong side);
- battery is below 25% on either robot.
