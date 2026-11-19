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

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # scripts/: flight_common, calibration

from flight_common import countdown, land_now, preflight  # noqa: E402

from floormap import BLOCKED, FREE, UNKNOWN, FloorMap, classify, waypoints  # noqa: E402
from sweep import _walk, lawnmower, snap, spiral  # noqa: E402
from tracker import Tracker, heading_error, yaw_correction  # noqa: E402

CELL = 20             # cm per grid square = SDK minimum move
MAX_SPAN_CM = 600     # refuse search areas bigger than this on either axis
HOME_SEARCH_RINGS = 3  # if pad A isn't seen on return, spiral up to 3 squares (60 cm) out
CRITICAL_BATTERY = 10  # % below which the mission lands wherever it is
SIDE_CHECK_RESERVE = 15  # % battery above the minimum needed to start a round of side checks
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
                 obstacle_signal="tof", tof_samples=3, pad_tries=2, side_rounds=3,
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
        # Each square is judged by one reading at its centre, so an obstacle edge can be up
        # to half a square closer than the map shows: add that to the clearance.
        self.radius = dog_width / 2 + margin + CELL / 2
        self.lane_cells = max(1, round(lane_cm / CELL))
        self.speed, self.climb, self.settle = speed, climb, settle
        self.obstacle_cm, self.drop_cm = obstacle_cm, drop_cm
        self.min_battery, self.yaw_limit, self.max_replans = min_battery, yaw_limit, max_replans
        self.send, self.say, self.sleep = send, say, sleep
        if obstacle_signal not in ("tof", "tof+baro"):
            raise ValueError("obstacle_signal must be 'tof' or 'tof+baro'")
        self.obstacle_signal, self.tof_samples = obstacle_signal, tof_samples
        self.pad_tries, self.side_rounds = pad_tries, side_rounds
        self.baro0 = None
        self.low_battery = False  # time to stop searching/checking and go home
        self.steps = 0            # moves so far, to learn battery use per move
        self.battery0 = None

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
    def pad(self, tries=None):
        """(id, raw x, raw y) of the pad under the drone, or None. Pad detection misses
        now and then, so it looks a few times before giving up."""
        for n in range(tries or self.pad_tries):
            if n:
                self.sleep(0.1)
            mid = self.d.get_mission_pad_id()
            if mid is not None and mid > 0:
                return mid, self.d.get_mission_pad_distance_x(), self.d.get_mission_pad_distance_y()
        return None

    def read_floor(self):
        """Median of a few downward readings (one bad reading can't block a square for good).
        Returns (state or None, tof, baro)."""
        vals = []
        for n in range(self.tof_samples):
            if n:
                self.sleep(0.1)
            vals.append(self.d.get_distance_tof())
        good = [v for v in vals if v is not None and 0 < v < 600]
        tof = statistics.median(good) if good else None
        state = classify(tof, self.baseline, self.obstacle_cm, self.drop_cm)
        baro = statistics.median(self.d.get_barometer() for _ in range(3))
        if state == FREE and self.obstacle_signal == "tof+baro" and self.baro0 is not None:
            # the drone may climb over a box by itself, keeping the distance reading
            # unchanged; the barometer still sees the climb
            if baro - self.baro0 > self.obstacle_cm or baro - self.baro0 < -self.drop_cm:
                state = BLOCKED
        return state, tof, baro

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
        state, tof, baro = self.read_floor()
        if state is not None:
            self.readings.append((idx, state))
            self.map.mark(self.map.cell_of(*self.tracker.pos), state)
        self.raw.append({"i": idx, "pos": [round(v, 1) for v in self.tracker.pos],
                         "tof": tof, "h": self.d.get_height(), "baro": baro,
                         "pad": pad, "state": state})

        battery = self.d.get_battery()
        if battery < CRITICAL_BATTERY:
            raise MissionError(f"battery {battery}%: landing here")
        if not self.low_battery and battery < self.home_threshold(battery):
            self.low_battery = True
            self.say(f"  battery {battery}%: just enough left to get home; heading home")
        turn = yaw_correction(self.yaw0, self.d.get_yaw(), self.yaw_limit)
        if turn:
            way, deg = turn
            self.say(f"  heading drifted, turning {way} {deg} deg")
            (self.d.rotate_clockwise if way == "cw" else self.d.rotate_counter_clockwise)(deg)
        return state

    def home_threshold(self, battery: float) -> float:
        """Battery % below which we must head home: what the trip home costs from here
        (measured % per move so far, x1.5, plus a few moves to find pad A), on top of
        the critical level, and never below min_battery."""
        if self.battery0 is None or self.steps < 10:
            per_move = 0.5                     # until we've measured it
        else:
            per_move = max(0.2, (self.battery0 - battery) / self.steps)
        moves_home = abs(self.cell[0]) + abs(self.cell[1]) + 6
        return max(self.min_battery, CRITICAL_BATTERY + 3 + 1.5 * per_move * moves_home)

    # ---- moving --------------------------------------------------------------------
    def step(self, d) -> str:
        nxt = (self.cell[0] + d[0], self.cell[1] + d[1])
        (i0, i1), (j0, j1) = self.i_range, self.j_range
        # One square of slack: a drift correction can put our estimate just outside the
        # edge, and stepping along (or back in from) there is fine. Further is not.
        if not (i0 - 1 <= nxt[0] <= i1 + 1 and j0 - 1 <= nxt[1] <= j1 + 1):
            raise MissionError(f"step to cell {nxt} would leave the search area")
        dx, dy = d[0] * CELL, d[1] * CELL
        yaw_err = heading_error(self.yaw0, self.d.get_yaw())
        self.d.go_xyz_speed(dx, GO_Y_SIGN * dy, 0, self.speed)
        self.tracker.moved(dx, dy, yaw_err)
        self.steps += 1
        # Our square is wherever the estimate says, which can differ from the commanded
        # square once heading corrections add up (readings are filed under the estimate).
        self.cell = self.map.cell_of(*self.tracker.pos)
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
        start = self.target_cell() or self.cell
        for _ in range(self.max_replans + 1):
            if self.cell == self.home:
                return
            plan = self.map.route(start, self.home, self.radius, allow_unknown=True, edges=True)
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

    def fly_to(self, goal) -> bool:
        """Drone-only flight to a cell (may cross safety gaps; doesn't stop on blocked)."""
        for _ in range(self.max_replans + 1):
            if self.cell == goal:
                return True
            path = self.map.route(self.cell, goal, self.radius, allow_unknown=True,
                                  cross_clearance=True)
            if path is None:
                for d in _walk(self.cell, goal):
                    self.step(d)
                continue
            for c in path[1:]:
                self.step((c[0] - self.cell[0], c[1] - self.cell[1]))
                if self.cell != c:
                    break  # a pad fix shifted us: re-plan
        return self.cell == goal

    def target_cell(self):
        return self.map.cell_of(*self.tracker.place(*self.found)) if self.found else None

    def dog_route(self):
        """The route the dog would get right now (every square it passes has been read)."""
        goal = self.target_cell()
        return None if goal is None else self.map.route(self.home, goal, self.radius, strict=True)

    def check_sides(self) -> bool:
        """One pass over the unread squares beside the best dog route (nearest first).
        False if there's no point trying again (no route at all, or battery too low)."""
        plan = self.map.route(self.home, self.target_cell(), self.radius, allow_unknown=True,
                              edges=True)
        if plan is None:
            self.say("  no route for the dog to check")
            return False
        left = set(self.map.unchecked_beside(plan, self.radius))
        self.say(f"  checking {len(left)} squares beside the dog's route")
        while left:
            # Keep enough battery to get back to pad A from here (~1% per 3 squares, doubled)
            far = abs(self.cell[0]) + abs(self.cell[1])
            if self.low_battery or self.d.get_battery() < self.min_battery + 3 + 2 * far // 3:
                self.say(f"  battery low: stopping the side checks with {len(left)} left")
                return False
            c = min(left, key=lambda t: abs(t[0] - self.cell[0]) + abs(t[1] - self.cell[1]))
            left.discard(c)
            if self.map.get(c) == UNKNOWN:
                self.fly_to(c)
        return True

    def _fixed_on_origin_here(self) -> bool:
        last = self.tracker.fixes[-1] if self.tracker.fixes else None
        return bool(last and last[0] == self.tracker.index and last[1] == self.origin)

    def arrive_home(self) -> None:
        pad = self.pad(tries=5)
        if not self._fixed_on_origin_here() and not (pad and pad[0] == self.origin):
            self.say("  pad A not under the drone; spiralling out to find it")
            for s in spiral(HOME_SEARCH_RINGS):
                if self.d.get_battery() <= CRITICAL_BATTERY + 2:
                    self.say("  battery nearly flat: landing here, near pad A")
                    break
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
            if self.drift is None:  # report the first measurement, after the main flight
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
            self.battery0 = d.get_battery()
            pad = self.pad(tries=5)
            if not pad or pad[0] != self.origin:
                raise MissionError(f"pad {self.origin} not under the drone after takeoff (saw {pad})")
            self.tracker.pad_fix(*pad)
            self.cell = self.map.cell_of(*self.tracker.pos)
            good = [v for v in (d.get_distance_tof() for _ in range(5)) if 0 < v < 600]
            if not good:
                raise MissionError("no valid floor distance reading after takeoff")
            self.baseline = statistics.median(good)
            self.baro0 = statistics.median(d.get_barometer() for _ in range(5))
            self.say(f"  floor baseline {self.baseline:.0f} cm")
            self.observe()

            self.say("Searching ...")
            for s in lawnmower(self.cell, self.i_range, self.j_range, self.lane_cells):
                if not self.map.inside((self.cell[0] + s[0], self.cell[1] + s[1])):
                    continue  # a pad fix shifted us; skip steps that would leave the area
                self.step(s)
                if self.found or self.low_battery:
                    break
            if not self.found and self.lane_cells > 1:
                # Pads are only seen within ~25 cm, so with drift the drone can slip past
                # one between lanes. Second pass down the gaps between the first lanes.
                battery = d.get_battery()
                j0, j1 = self.j_range
                gaps = (min(j0 + self.lane_cells // 2, j1), j1)
                second = lawnmower(self.cell, self.i_range, gaps, self.lane_cells)
                per_move = max(0.2, (self.battery0 - battery) / max(1, self.steps))
                if self.low_battery or battery < self.home_threshold(battery) + per_move * len(second):
                    self.say(f"  target not found; battery {battery}% too low for a second pass")
                else:
                    self.say("  target not found; searching between the lanes")
                    for s in second:
                        if not self.map.inside((self.cell[0] + s[0], self.cell[1] + s[1])):
                            continue
                        self.step(s)
                        if self.found or self.low_battery:
                            break
            if self.found:
                self.centre_over_target()
            else:
                self.say("  target not found in the area")

            self.say("Flying the dog's route home ...")
            self.fly_home()
            self.arrive_home()  # corrects the drift so far, target position included
            # Until the dog's route is fully read: from pad A, check the unread squares beside
            # the best route, then come back so pad A corrects the drift of that part too.
            for _ in range(self.side_rounds if self.found else 0):
                if self.dog_route() is not None:
                    break
                battery = d.get_battery()
                if battery < self.min_battery + SIDE_CHECK_RESERVE:
                    self.say(f"  battery {battery}%: not enough to check beside the route")
                    break
                self.say("Checking beside the dog's route ...")
                go_on = self.check_sides()
                if self.cell != self.home:
                    self.say("Flying home ...")
                    self.fly_to(self.home)
                    self.arrive_home()
                if not go_on:
                    break
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
            path = self.map.route(self.home, target_cell, self.radius, strict=True)
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
                          obstacle_signal=args.obstacle_signal)
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
