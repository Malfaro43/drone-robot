"""The drone's search mission, without the command line (that's find_pad.py).

Mission(drone, ...).run() takes off from pad A, searches the floor for the target pad,
maps free/blocked 20 cm squares, checks the dog's route (and the squares beside it), flies
home correcting its drift over pad A, lands, and returns the map message for the dog.
`drone` is a djitellopy.Tello (real), sim_drone.FakeTello (simulation), or the bridge's
`bridge.tello.drone`.

Building blocks: tracker.py (where the drone is), floormap.py (the squares and routes),
sweep.py (search pattern). See find_pad.py's docstring for the flight sequence.
"""

import statistics
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # scripts/: flight_common

from flight_common import land_now  # noqa: E402

from floormap import BLOCKED, FREE, LIKELY, UNKNOWN, FloorMap, classify, waypoints  # noqa: E402
from sweep import lawnmower, snap, spiral, walk  # noqa: E402
from tracker import Tracker, heading_error, yaw_correction  # noqa: E402

CELL = 20             # cm per grid square = SDK minimum move
MAX_GLIDE = 5          # longest single move while searching, in squares (100 cm)
SAMPLE_PERIOD = 0.1    # s between state samples while a move is under way
SPIRAL_RESERVE = 5     # % battery kept back for spiralling to find pad A at the end
REFIX_CELLS = 25       # after this many squares (5 m) without a pad fix, go back to pad A
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
                 max_glide=MAX_GLIDE,
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
        self.max_glide = max(1, max_glide)
        self.sighting = None      # (x, y) where the drone was when it glimpsed the target mid-move
        self.baro0 = None
        self.low_battery = False  # time to stop searching/checking and go home
        self.steps = 0            # moves so far, to learn battery use per move
        self.since_fix = 0        # squares flown since the last pad correction
        self.t_start = None       # takeoff time, to measure battery use per second
        self.step_time = None     # running average of how long a 20 cm step takes
        self.refixing = False
        self.battery0 = None

        self.tracker = Tracker(known)
        self.map = FloorMap(self.i_range, self.j_range, CELL)
        self.cell = (0, 0)       # commanded grid cell
        self.readings = []       # (log index, fraction along the move into it, state)
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

    def at(self, idx: int, f: float) -> tuple:
        """Position a fraction f of the way through the move that ended at log point idx."""
        x1, y1 = self.tracker.points[idx]
        if f >= 1 or idx == 0:
            return x1, y1
        x0, y0 = self.tracker.points[idx - 1]
        return x0 + (x1 - x0) * f, y0 + (y1 - y0) * f

    def rebuild_map(self):
        self.map = FloorMap(self.i_range, self.j_range, CELL)
        for idx, f, state in self.readings:
            self.map.mark(self.map.cell_of(*self.at(idx, f)), state)

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
                self.since_fix = 0
            elif mid == self.target and self.found is None:
                self.found = (idx, px, py)
                tx, ty = self.tracker.place(idx, px, py)
                self.say(f"  target pad {mid} seen, about x={tx:.0f} y={ty:.0f} cm")
        state, tof, baro = self.read_floor()
        if state is not None:
            self.readings.append((idx, 1.0, state))
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
        """Battery % below which we must head home: what the trip home costs from here,
        x1.5, plus a few steps to find pad A, on top of the critical level, never below
        min_battery. The trip home is flown in 20 cm steps, so its cost is (battery used
        per second so far) x (measured time per 20 cm step) x (steps home). Long search
        moves are much cheaper per square, so a per-square average would undercount it."""
        clock = getattr(self.d, "clock", time.monotonic)
        elapsed = clock() - self.t_start if self.t_start is not None else 0
        if self.battery0 is None or elapsed < 30 or self.step_time is None:
            per_step = 0.5                     # until we've measured it
        else:
            per_step = max(0.2, (self.battery0 - battery) / elapsed * self.step_time)
        moves_home = abs(self.cell[0]) + abs(self.cell[1]) + 6
        return max(self.min_battery, CRITICAL_BATTERY + 3 + 1.5 * per_step * moves_home)

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
        clock = getattr(self.d, "clock", time.monotonic)
        t0 = clock()
        self.d.go_xyz_speed(dx, GO_Y_SIGN * dy, 0, self.speed)
        self.tracker.moved(dx, dy, yaw_err)
        self.steps += 1
        self.since_fix += 1
        took = clock() - t0 + self.settle + 0.5   # + the stop to observe
        self.step_time = took if self.step_time is None else 0.8 * self.step_time + 0.2 * took
        # Our square is wherever the estimate says, which can differ from the commanded
        # square once heading corrections add up (readings are filed under the estimate).
        self.cell = self.map.cell_of(*self.tracker.pos)
        return self.observe()

    # ---- long moves with readings along the way ----------------------------------------
    def sampled_go(self, dx: float, dy: float) -> list:
        """One `go`, recording the drone's state stream while it flies. Returns the samples,
        each with "f": how far through the move (by time) it was taken."""
        samples = []
        clock = getattr(self.d, "clock", time.monotonic)
        t0 = clock()
        if hasattr(self.d, "sample_hook"):          # simulated drone streams synchronously
            self.d.sample_hook = samples.append
            try:
                self.d.go_xyz_speed(dx, dy, 0, self.speed)
            finally:
                self.d.sample_hook = None
        else:                                       # real drone: poll the state stream
            done = threading.Event()

            def poll():
                while not done.wait(SAMPLE_PERIOD):
                    try:
                        mid = self.d.get_mission_pad_id()
                        pad = None if mid is None or mid <= 0 else (
                            mid, self.d.get_mission_pad_distance_x(),
                            self.d.get_mission_pad_distance_y())
                        samples.append({"t": clock(), "tof": self.d.get_distance_tof(),
                                        "baro": self.d.get_barometer(), "pad": pad})
                    except Exception:
                        pass
            th = threading.Thread(target=poll, daemon=True)
            th.start()
            try:
                self.d.go_xyz_speed(dx, dy, 0, self.speed)
            finally:
                done.set()
                th.join(timeout=1)
        t1 = clock()
        for smp in samples:
            # Assumes steady speed through the move. The real drone eases in and out, so
            # mid-move readings can be filed a little off; the stress test measures the cost.
            smp["f"] = min(1.0, max(0.0, (smp["t"] - t0) / max(t1 - t0, 1e-6)))
        return samples

    def glide(self, d, n: int) -> str:
        """Fly n squares in direction d in one move, reading the floor along the way, then
        stop and observe as usual. Returns the floor state at the end."""
        end = (self.cell[0] + d[0] * n, self.cell[1] + d[1] * n)
        (i0, i1), (j0, j1) = self.i_range, self.j_range
        if not (i0 - 1 <= end[0] <= i1 + 1 and j0 - 1 <= end[1] <= j1 + 1):
            raise MissionError(f"move to cell {end} would leave the search area")
        dx, dy = d[0] * CELL * n, d[1] * CELL * n
        yaw_err = heading_error(self.yaw0, self.d.get_yaw())
        samples = self.sampled_go(dx, GO_Y_SIGN * dy)
        self.tracker.moved(dx, dy, yaw_err)
        self.steps += n
        self.since_fix += n
        idx = self.tracker.index
        # (A known pad seen mid-move is NOT used to correct the position: where exactly we
        # were at that instant is the uncertain part. sweep() stops on known pads instead.)
        cells = {}
        for smp in samples:
            pos = self.at(idx, smp["f"])
            c = self.map.cell_of(*pos)
            if c != self.map.cell_of(*self.at(idx, 1.0)):    # the end square: observe() reads it
                cells.setdefault(c, []).append(smp)
            pad = smp.get("pad")
            if pad and pad[0] == self.target and self.found is None and self.sighting is None:
                self.sighting = pos
                self.say(f"  glimpsed target pad {pad[0]} mid-move near "
                         f"x={pos[0]:.0f} y={pos[1]:.0f} cm")
        for c, group in cells.items():
            good = [g["tof"] for g in group if g["tof"] is not None and 0 < g["tof"] < 600]
            tof = statistics.median(good) if good else None
            state = classify(tof, self.baseline, self.obstacle_cm, self.drop_cm)
            if state == FREE and self.obstacle_signal == "tof+baro" and self.baro0 is not None:
                baro = statistics.median(g["baro"] for g in group)
                if baro - self.baro0 > self.obstacle_cm or baro - self.baro0 < -self.drop_cm:
                    state = BLOCKED
            if state == FREE:
                state = LIKELY      # mid-move: guides the search, a stop must confirm it
            if state is not None:
                f = statistics.median(g["f"] for g in group)
                self.readings.append((idx, f, state))
                self.map.mark(self.map.cell_of(*self.at(idx, f)), state)
        self.cell = self.map.cell_of(*self.tracker.pos)
        state = self.observe()
        if self.sighting is not None and self.found is None:
            # flew past it: go back to where it was glimpsed (observe() picks it up there)
            back = self.map.cell_of(*self.sighting)
            self.sighting = None
            self.say("  going back to the target")
            self.fly_to(back)
            for s in spiral(1):
                if self.found:
                    break
                if self.map.inside((self.cell[0] + s[0], self.cell[1] + s[1])):
                    self.step(s)
        return state

    def sweep(self, steps) -> None:
        """Fly a list of unit search steps as straight runs of up to max_glide squares,
        skipping anything that would leave the area. Stops when the target is found or the
        battery says go home."""
        pads = {self.map.cell_of(*p) for p in self.tracker.known_pads.values()}
        k = 0
        while k < len(steps) and not (self.found or self.low_battery):
            d = steps[k]
            n = 1
            while k + n < len(steps) and steps[k + n] == d and n < self.max_glide:
                # stop on a known pad's square rather than fly over it: a stopped pad
                # reading is what keeps the position estimate honest
                if (self.cell[0] + d[0] * n, self.cell[1] + d[1] * n) in pads:
                    break
                n += 1
            k += n
            # keep the run inside the area (a pad fix may have shifted us)
            while n and not self.map.inside((self.cell[0] + d[0] * n, self.cell[1] + d[1] * n)):
                n -= 1
            if n == 1:
                self.step(d)
            elif n > 1:
                self.glide(d, n)

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
        for d in walk(self.cell, self.home):
            self.step(d)

    def fly_to(self, goal) -> bool:
        """Drone-only flight to a cell (may cross safety gaps; doesn't stop on blocked)."""
        for _ in range(self.max_replans + 1):
            if self.cell == goal:
                return True
            path = self.map.route(self.cell, goal, self.radius, allow_unknown=True,
                                  cross_clearance=True)
            if path is None:
                for d in walk(self.cell, goal):
                    self.step(d)
                continue
            for c in path[1:]:
                self.step((c[0] - self.cell[0], c[1] - self.cell[1]))
                if self.cell != c:
                    break  # a pad fix shifted us: re-plan
        return self.cell == goal

    def refix_if_due(self) -> None:
        """Drift grows with distance flown; pad A is the only sure way to remove it. After
        REFIX_CELLS squares without a correction, fly back to pad A, correct, and return.
        (Without this, a long stretch of checking can drift further than the spiral that
        looks for pad A at the end can reach.)"""
        if self.since_fix < REFIX_CELLS or self.refixing or self.low_battery:
            return
        self.refixing = True
        try:
            back_to = self.cell
            self.say(f"  {self.since_fix * CELL / 100:.0f} m since the last correction: "
                     f"checking in at pad A")
            self.fly_to(self.home)
            self.arrive_home()
            self.since_fix = 0
            if not self.low_battery:
                self.fly_to(back_to)
        finally:
            self.refixing = False

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
            # Keep enough battery to get back to pad A from here (measured cost per move),
            # plus a reserve for spiralling to find it
            battery = self.d.get_battery()
            if self.low_battery or battery < self.home_threshold(battery) + SPIRAL_RESERVE:
                self.say(f"  battery low: stopping the side checks with {len(left)} left")
                return False
            self.refix_if_due()
            c = min(left, key=lambda t: abs(t[0] - self.cell[0]) + abs(t[1] - self.cell[1]))
            left.discard(c)
            if self.map.get(c) in (UNKNOWN, LIKELY):
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
            self.t_start = getattr(d, "clock", time.monotonic)()
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
            self.sweep(lawnmower(self.cell, self.i_range, self.j_range, self.lane_cells))
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
                    self.sweep(second)
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
                     "legend": {FREE: "free", BLOCKED: "blocked", UNKNOWN: "unknown",
                                LIKELY: "probably free (seen mid-move, not confirmed)"},
                     "index": "rows[a][b] is the square at x=origin[0]+cell*a, y=origin[1]+cell*b"},
            "drift_at_home_cm": self.drift,
        }
