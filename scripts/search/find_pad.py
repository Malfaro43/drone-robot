"""Search the floor for a target mission pad, map free/blocked floor, fly the dog's route home.

Setup: pad A (--origin) on the floor beside the dog, rocket pointing the way the dog faces.
That pad is the map origin; +x = forward, +y = left. Place every other pad with its rocket
pointing the same way. Put the drone on pad A, nose along the rocket.

Flight:
  1. take off over pad A, take the floor-distance baseline;
  2. lawnmower sweep of the search rectangle in 20 cm steps, reading the pad ID and the
     downward distance sensor after every step; stop when the --target pad shows up;
  3. centre over the target to pin down its position;
  4. plan the dog's route home (keeps clear of blocked squares by half the dog's width plus
     --margin) and fly it, re-planning if a square on it turns out blocked;
  5. over pad A again: measure the tracking drift, correct the whole log with it, land;
  6. send the map + route for the dog (for now: printed and saved to --out).

    .venv/bin/python scripts/search/find_pad.py --check      # no flight: live pad/sensor readout
    .venv/bin/python scripts/search/find_pad.py --target 2 --forward 200 --left 60 --right 60 \
        --dog-width 30

Run from Apple Terminal on the drone's Wi-Fi. Ctrl+C lands at any point.
"""

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # scripts/: flight_common, calibration

from flight_common import countdown, preflight  # noqa: E402

# The mission itself lives in mission.py; these names are re-exported for older imports.
from mission import CELL, MAX_GLIDE, Mission, MissionError  # noqa: E402, F401

MAX_SPAN_CM = 600     # refuse search areas bigger than this on either axis

# ---- CLI ---------------------------------------------------------------------------
def pad_spec(text: str) -> tuple:
    try:
        pid, xy = text.split("=")
        x, y = xy.split(",")
        return int(pid), (float(x), float(y))
    except ValueError:
        raise argparse.ArgumentTypeError("use ID=X,Y in cm, e.g. 3=100,-40") from None


def live_check(tello) -> int:
    tello.enable_mission_pads()
    tello.set_mission_pad_detection_direction(0)
    print("Not flying. Live readings every 0.5 s, Ctrl+C to stop.")
    print("Slide the drone forward over a pad: x should grow. Slide it left: y should grow.")
    try:
        while True:
            print(f"pad={tello.get_mission_pad_id():>2}  x={tello.get_mission_pad_distance_x():>4}"
                  f"  y={tello.get_mission_pad_distance_y():>4}  z={tello.get_mission_pad_distance_z():>4}"
                  f"  tof={tello.get_distance_tof():>4}  h={tello.get_height():>4}"
                  f"  baro={tello.get_barometer():>7.0f}  yaw={tello.get_yaw():>4}"
                  f"  battery={tello.get_battery()}%")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print()
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--check", action="store_true", help="no flight: print pad and sensor readings live")
    p.add_argument("--target", type=int, help="ID (1-8) of the pad to find")
    p.add_argument("--origin", type=int, default=1, help="ID of pad A, beside the dog (default 1)")
    p.add_argument("--pad", type=pad_spec, action="append", default=[],
                   help="extra pad at a known spot for drift fixes, ID=X,Y cm (repeatable)")
    p.add_argument("--forward", type=float, help="search this many cm ahead of pad A")
    p.add_argument("--back", type=float, default=0, help="... and this many cm behind it (default 0)")
    p.add_argument("--left", type=float, help="... this many cm to its left")
    p.add_argument("--right", type=float, help="... and this many cm to its right")
    p.add_argument("--dog-width", type=float, help="measured width of the dog in cm")
    p.add_argument("--margin", type=float, default=10, help="extra clearance each side, cm")
    p.add_argument("--lanes", type=float, default=40, help="search lane spacing, cm (default 40)")
    p.add_argument("--speed", type=int, default=30, help="cm/s (10-100, default 30)")
    p.add_argument("--glide", type=int, default=MAX_GLIDE,
                   help=f"longest search move in 20 cm squares, reading the floor on the way "
                        f"(default {MAX_GLIDE}; 1 = stop every 20 cm, slower but simpler)")
    p.add_argument("--climb", type=int, default=0, help="cm to climb above takeoff height (0 or 20-40)")
    p.add_argument("--obstacle-cm", type=float, default=15,
                   help="floor reading this much SHORTER than baseline = something there")
    p.add_argument("--drop-cm", type=float, default=25,
                   help="floor reading this much LONGER than baseline = a drop")
    p.add_argument("--obstacle-signal", choices=["tof", "tof+baro"],
                   help="also use the barometer, in case the drone climbs over boxes by itself "
                        "(default: tof+baro if calibration found it does, else tof)")
    p.add_argument("--out", default="map.json", help="where to save the map and flight log")
    p.add_argument("--countdown", type=int, default=5)
    p.add_argument("--ignore-temp", action="store_true")
    args = p.parse_args(argv)

    if not args.check:
        missing = [n for n in ("target", "forward", "left", "right", "dog_width")
                   if getattr(args, n) is None]
        if missing:
            p.error("flying needs " + ", ".join("--" + n.replace("_", "-") for n in missing))
        for n in ("forward", "back", "left", "right"):
            if getattr(args, n) < 0:
                p.error(f"--{n} can't be negative")
        if args.forward + args.back > MAX_SPAN_CM or args.left + args.right > MAX_SPAN_CM:
            p.error(f"the search area is limited to {MAX_SPAN_CM} cm on each side")
        if not 1 <= args.target <= 8:
            p.error("--target must be 1-8")
        if args.climb and not 20 <= args.climb <= 40:
            p.error("--climb must be 0 or 20-40 (pads are only seen up to ~120 cm)")
        if not 10 <= args.speed <= 100:
            p.error("--speed must be 10-100")

    import calibration
    cal = calibration.load()
    missing = calibration.apply_to_flight(cal)
    if missing:
        print("WARNING: not yet calibrated (run scripts/calibrate.py drone-axes / drone-hover): "
              + ", ".join(missing))
    if args.obstacle_signal is None:
        args.obstacle_signal = ("tof+baro" if calibration.value(cal, "drone", "auto_climb")
                                else "tof")

    from djitellopy import Tello
    tello = Tello()
    print("Connecting to drone at 192.168.10.1 ...")
    tello.connect()
    try:
        if args.check:
            return live_check(tello)

        mission = Mission(tello, target=args.target, origin=args.origin, pads=dict(args.pad),
                          x_range=(-args.back, args.forward), y_range=(-args.right, args.left),
                          dog_width=args.dog_width,
                          margin=args.margin, lane_cm=args.lanes, speed=args.speed,
                          climb=args.climb, obstacle_cm=args.obstacle_cm, drop_cm=args.drop_cm,
                          obstacle_signal=args.obstacle_signal, max_glide=args.glide)
        problem = preflight(tello, ignore_temp=args.ignore_temp)
        if problem:
            print(problem)
            return 1
        if not countdown(args.countdown):
            return 130
        try:
            msg = mission.run()
            status = 0 if msg["target"] else 2
        except KeyboardInterrupt:
            print("\nCtrl+C: landed.")
            msg, status = mission.report(), 130
        except MissionError as exc:
            print(f"Mission stopped: {exc}. Landed.")
            msg, status = mission.report(), 1
        except Exception as exc:
            print(f"Error during flight: {exc!r}. Landed.")
            msg, status = mission.report(), 1
        Path(args.out).write_text(json.dumps({"map": msg, "flight_log": mission.raw}, indent=1) + "\n")
        print(f"Saved {args.out}")
        return status
    finally:
        tello.end()


if __name__ == "__main__":
    sys.exit(main())
