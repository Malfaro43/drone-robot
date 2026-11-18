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
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # scripts/ for flight_common

from flight_common import countdown, land_now, preflight  # noqa: E402

from floormap import BLOCKED, FREE, FloorMap, classify, waypoints  # noqa: E402
from sweep import _walk, lawnmower, snap, spiral  # noqa: E402
from tracker import Tracker, yaw_correction  # noqa: E402

CELL = 20             # cm per grid square = SDK minimum move
MAX_SPAN_CM = 600     # refuse search areas bigger than this on either axis
HOME_SEARCH_RINGS = 2  # if pad A isn't seen on return, spiral up to 2 squares (40 cm) out
GO_Y_SIGN = 1         # +1 if `go 0 20 0` moves the drone LEFT. UNVERIFIED: check in test (b).


class MissionError(Exception):
    pass


def default_send(msg: dict) -> None:
    """Stand-in for the dog link: print a summary. Replace with the link module's send."""
    t = msg["target"]
    where = "not found" if t is None else f"at x={t['x']:.0f} y={t['y']:.0f} cm"
    print(f"-> dog: target pad {msg['target_pad']} {where}; route with "
          f"{len(msg['route'])} waypoints (checked by drone: {msg['route_checked']})")


class Mission:
    def __init__(self, drone, *, target, x_range, y_range, dog_width, origin=1, pads=None,
                 lane_cm=40, speed=30, climb=0, margin=10, obstacle_cm=15, drop_cm=25,
                 settle=0.5, min_battery=20, yaw_limit=10, max_replans=10,
                 send=default_send, say=print, sleep=time.sleep):
        self.d = drone
        self.target, self.origin = target, origin
        self.i_range, self.j_range = snap(*x_range, CELL), snap(*y_range, CELL)
        self.home = (0, 0)
        if not (self.i_range[0] <= 0 <= self.i_range[1] and self.j_range[0] <= 0 <= self.j_range[1]):
            raise ValueError("the search area must include pad A at (0, 0)")
        known = {origin: (0, 0), **(pads or {})}
        if target in known:
            raise ValueError("the target pad can't also be a known reference pad")
        self.radius = dog_width / 2 + margin
        self.lane_cells = max(1, round(lane_cm / CELL))
        self.speed, self.climb, self.settle = speed, climb, settle
        self.obstacle_cm, self.drop_cm = obstacle_cm, drop_cm
        self.min_battery, self.yaw_limit, self.max_replans = min_battery, yaw_limit, max_replans
        self.send, self.say, self.sleep = send, say, sleep

        self.tracker = Tracker(known)
        self.map = FloorMap(self.i_range, self.j_range, CELL)
        self.cell = (0, 0)       # commanded grid cell
        self.readings = []       # (log index, state)
        self.raw = []            # per-step sensor log for calibration
        self.found = None        # (log index, raw x, raw y) of the target sighting
        self.baseline = None
        self.yaw0 = None
        self.drift = None
        self.flying = False

    # ---- sensing -------------------------------------------------------------------
    def pad(self):
        mid = self.d.get_mission_pad_id()
        if mid is None or mid <= 0:
            return None
        return mid, self.d.get_mission_pad_distance_x(), self.d.get_mission_pad_distance_y()

    def rebuild_map(self):
        self.map = FloorMap(self.i_range, self.j_range, CELL)
        for idx, state in self.readings:
            self.map.mark(self.map.cell_of(*self.tracker.points[idx]), state)

    def observe(self) -> str:
        """Read everything after a step. Returns the floor state under the drone."""
        self.sleep(self.settle)
        idx = self.tracker.index
        pad = self.pad()
        if pad:
            mid, px, py = pad
            if self.tracker.pad_fix(mid, px, py) is not None:
                self.rebuild_map()
                self.cell = self.map.cell_of(*self.tracker.pos)
            elif mid == self.target and self.found is None:
                self.found = (idx, px, py)
                tx, ty = self.tracker.place(idx, px, py)
                self.say(f"  target pad {mid} seen, about x={tx:.0f} y={ty:.0f} cm")
        tof = self.d.get_distance_tof()
        state = classify(tof, self.baseline, self.obstacle_cm, self.drop_cm)
        self.readings.append((idx, state))
        self.map.mark(self.map.cell_of(*self.tracker.pos), state)
        self.raw.append({"i": idx, "pos": [round(v, 1) for v in self.tracker.pos],
                         "tof": tof, "h": self.d.get_height(), "baro": self.d.get_barometer(),
                         "pad": pad, "state": state})

        battery = self.d.get_battery()
        if battery < self.min_battery:
            raise MissionError(f"battery {battery}% below {self.min_battery}%")
        turn = yaw_correction(self.yaw0, self.d.get_yaw(), self.yaw_limit)
        if turn:
            way, deg = turn
            self.say(f"  heading drifted, turning {way} {deg} deg")
            (self.d.rotate_clockwise if way == "cw" else self.d.rotate_counter_clockwise)(deg)
        return state

    # ---- moving --------------------------------------------------------------------
    def step(self, d) -> str:
        nxt = (self.cell[0] + d[0], self.cell[1] + d[1])
        if not self.map.inside(nxt):
            raise MissionError(f"step to cell {nxt} would leave the search area")
        dx, dy = d[0] * CELL, d[1] * CELL
        self.d.go_xyz_speed(dx, GO_Y_SIGN * dy, 0, self.speed)
        self.tracker.moved(dx, dy)
        self.cell = nxt
        return self.observe()

    def walk_cells(self, path) -> str:
        """Fly a cell path. Stops early with "blocked" (a square on it reads blocked) or
        "shifted" (a pad fix moved our square, so the rest of the path no longer lines up)."""
        for c in path[1:]:
            state = self.step((c[0] - self.cell[0], c[1] - self.cell[1]))
            if self.cell != c:
                return "shifted"
            if state == BLOCKED and c != self.home:
                return "blocked"
        return "ok"

    def centre_over_target(self):
        idx, px, py = self.found
        try:
            self.d.go_xyz_speed_mid(0, 0, max(30, int(self.baseline)), self.speed, self.target)
        except Exception as exc:
            self.say(f"  couldn't centre over the target ({exc!r}); using the first sighting")
            return
        ox, oy = Tracker.pad_offset(px, py)
        self.tracker.moved(-ox, -oy)
        pad = self.pad()
        if pad and pad[0] == self.target:
            self.found = (self.tracker.index, pad[1], pad[2])
        self.cell = self.map.cell_of(*self.tracker.pos)

    def fly_home(self) -> None:
        """Fly the dog's route from here (the target) to pad A, checking every square.

        Each re-plan is a whole dog route from the target. The drone rejoins it at the last
        square of the route's already-confirmed start and flies the rest, so by the end
        every square of the final route has been flown over."""
        start = self.cell
        for _ in range(self.max_replans + 1):
            if self.cell == self.home:
                return
            plan = self.map.route(start, self.home, self.radius, allow_unknown=True)
            if plan is None:
                self.say("  no safe route for the dog; flying straight home")
                break
            k = 0
            while k + 1 < len(plan) and self.map.get(plan[k + 1]) == FREE:
                k += 1
            join = plan[k]
            if self.cell != join:
                back = self.map.route(self.cell, join, self.radius, allow_unknown=True,
                                      cross_clearance=True)
                if back is None or self.walk_cells(back) != "ok":
                    continue
            result = self.walk_cells(plan[k:])
            if result == "ok":
                return
            if result == "blocked":
                self.say(f"  square {self.cell} on the route is blocked; re-planning")
        else:
            self.say("  too many re-plans; flying straight home")
        for d in _walk(self.cell, self.home):
            self.step(d)

    def _fixed_on_origin_here(self) -> bool:
        last = self.tracker.fixes[-1] if self.tracker.fixes else None
        return bool(last and last[0] == self.tracker.index and last[1] == self.origin)

    def arrive_home(self) -> None:
        pad = self.pad()
        if not self._fixed_on_origin_here() and not (pad and pad[0] == self.origin):
            self.say("  pad A not under the drone; spiralling out to find it")
            for s in spiral(HOME_SEARCH_RINGS):
                if self.map.inside((self.cell[0] + s[0], self.cell[1] + s[1])):
                    self.step(s)
                    if self._fixed_on_origin_here():
                        break
        last = self.tracker.fixes[-1] if self.tracker.fixes else None
        pad = self.pad()
        if self._fixed_on_origin_here():
            ex, ey = last[2]  # already fixed when we stepped onto pad A
        elif pad and pad[0] == self.origin:
            ex, ey = self.tracker.pad_fix(*pad)
        else:
            ex = ey = None
        if ex is not None:
            self.drift = [round(ex, 1), round(ey, 1)]
            self.rebuild_map()
            self.say(f"  over pad A: tracking was off by x={ex:.0f} y={ey:.0f} cm; log corrected")
            try:
                self.d.go_xyz_speed_mid(0, 0, max(30, int(self.baseline)), self.speed, self.origin)
            except Exception as exc:
                self.say(f"  couldn't centre over pad A ({exc!r})")
            else:
                ax, ay = self.tracker.known_pads[self.origin]
                x, y = self.tracker.pos
                self.tracker.moved(ax - x, ay - y)
        else:
            self.say("  pad A not seen on return: drift unknown, map not corrected")

    # ---- the mission ---------------------------------------------------------------
    def run(self) -> dict:
        d = self.d
        d.enable_mission_pads()
        d.set_mission_pad_detection_direction(0)
        try:
            self.say("Taking off over pad A ...")
            self.flying = True  # before takeoff(): it blocks while already airborne
            d.takeoff()
            if self.climb:
                d.move_up(self.climb)
            self.yaw0 = d.get_yaw()
            pad = self.pad()
            if not pad or pad[0] != self.origin:
                raise MissionError(f"pad {self.origin} not under the drone after takeoff (saw {pad})")
            self.tracker.pad_fix(*pad)
            self.cell = self.map.cell_of(*self.tracker.pos)
            self.baseline = statistics.median(d.get_distance_tof() for _ in range(3))
            self.say(f"  floor baseline {self.baseline:.0f} cm")
            self.observe()

            self.say("Searching ...")
            for s in lawnmower(self.cell, self.i_range, self.j_range, self.lane_cells):
                if not self.map.inside((self.cell[0] + s[0], self.cell[1] + s[1])):
                    continue  # a pad fix shifted us; skip steps that would leave the area
                self.step(s)
                if self.found:
                    break
            if self.found:
                self.centre_over_target()
            else:
                self.say("  target not found in the area")

            self.say("Flying the dog's route home ...")
            self.fly_home()
            self.arrive_home()
            self.say("Landing.")
            d.land()
            self.flying = False
        finally:
            if self.flying:
                land_now(d)
                self.flying = False

        msg = self.report()
        self.send(msg)
        return msg

    def report(self) -> dict:
        target = target_cell = None
        if self.found:
            tx, ty = self.tracker.place(*self.found)
            target = {"x": round(tx, 1), "y": round(ty, 1)}
            target_cell = self.map.cell_of(tx, ty)
        route, checked, beside = [], False, []
        if target_cell is not None:
            path = self.map.route(self.home, target_cell, self.radius, allow_unknown=False)
            checked = path is not None
            if path is None:
                path = self.map.route(self.home, target_cell, self.radius, allow_unknown=True)
            route = waypoints(path or [], CELL)
            beside = [list(self.map.center(c)) for c in
                      self.map.unchecked_beside(path or [], self.radius)]
        return {
            "type": "map", "units": "cm", "cell": CELL,
            "frame": "origin=pad A centre, +x=pad A rocket direction, +y=left",
            "target_pad": self.target, "target": target,
            "route": route, "route_checked": checked,
            "unchecked_beside_route": beside,
            "dog_clearance_cm": self.radius,
            "grid": {"origin": [self.i_range[0] * CELL, self.j_range[0] * CELL],
                     "rows": self.map.rows(),
                     "legend": {FREE: "free", BLOCKED: "blocked", "?": "unknown"},
                     "index": "rows[a][b] is the square at x=origin[0]+cell*a, y=origin[1]+cell*b"},
            "drift_at_home_cm": self.drift,
        }


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
    p.add_argument("--climb", type=int, default=0, help="cm to climb above takeoff height (0 or 20-40)")
    p.add_argument("--obstacle-cm", type=float, default=15,
                   help="floor reading this much SHORTER than baseline = something there")
    p.add_argument("--drop-cm", type=float, default=25,
                   help="floor reading this much LONGER than baseline = a drop")
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
                          climb=args.climb, obstacle_cm=args.obstacle_cm, drop_cm=args.drop_cm)
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
