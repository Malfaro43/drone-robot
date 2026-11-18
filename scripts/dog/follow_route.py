"""Walk the XGO dog along the route the drone mapped, steering it from the laptop.

The dog only takes speed commands (tello_xgo_bridge: `xgo.write(walk, yaw)`; walk is an
xgolib forward speed value, yaw a turn speed, positive = left) and reports its fused
heading (`xgo.read()["imu"]["rotation"]["yaw"]`). So the laptop drives each leg of the
route itself:

  1. turn on the spot until the gyro says the dog faces the next waypoint;
  2. walk for distance / calibrated speed (+ the dog's start-up delay), nudging the turn
     speed to hold the heading;
  3. stop, and track where the dog should now be.

Before moving, a short test turn learns which way the gyro counts (it isn't documented,
and the dog's own alignment code learns it the same way).

Map frame = the drone's: origin pad A, +x the way pad A's rocket points, +y left. The dog
starts at --start (cm), facing +x.

    .venv/bin/python scripts/dog/follow_route.py map.json --pi http://100.74.30.90:8080 \
        --start 0,30 --walk-speed-cm 6.5
    .venv/bin/python scripts/dog/follow_route.py map.json --sim         # simulated dog

Safety: refuses a route the drone didn't fully check; stops the dog on Ctrl+C, on any
error, if the heading can't be read, or when the time limit runs out. The Pi keeps the
last command running if the laptop vanishes, so stay by the dog (see README).
"""

import argparse
import json
import math
import sys
import time

WALK = 8                 # xgolib walk speed value (chase.py's default; Lite max 25)
TURN = 16                # xgolib turn speed value (chase.py's default; Lite max 100)
START_DELAY = 0.6        # s before the legs respond to a new command (measured 0.5-0.7)
TICK = 0.25              # s between heading checks
HEADING_TOL = 6.0        # deg: close enough to start walking
HOLD_GAIN = 0.8          # turn speed per degree of heading error while walking
HOLD_CAP = 8.0           # max turn speed while walking (chase.py's arc cap)
MIN_TURN = 6.0           # turn speeds below this may not move the legs (assumed, unmeasured)
BIAS_SECONDS = 5.0       # stand still this long to measure gyro drift before moving
MAX_BIAS = 4.0           # deg/s: refuse to walk on a gyro drifting faster than this
CROSS_GAIN = 1.5         # deg of heading correction per cm the dog has drifted off the line


def wrap(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


class FollowError(Exception):
    pass


class RouteFollower:
    def __init__(self, xgo, walk_cm_per_s: float, start=(0.0, 0.0), walk=WALK, turn=TURN,
                 max_seconds=180.0, clock=time.monotonic, sleep=time.sleep, say=print):
        """xgo: anything with read() -> stats dict and write(walk=, yaw=) -> dict,
        i.e. tello_xgo_bridge's XgoLink or sim_dog.FakeXgo."""
        self.xgo, self.cm_s = xgo, walk_cm_per_s
        self.walk, self.turn = walk, turn
        self.x, self.y = float(start[0]), float(start[1])
        self.max_seconds, self.clock, self.sleep, self.say = max_seconds, clock, sleep, say
        self.sign = None       # +1 if the gyro yaw grows when the dog turns left
        self.bias = 0.0        # deg/s the gyro reading drifts while the dog stands still
        self.yaw0 = None
        self.t0 = None
        self.path = [(self.x, self.y)]

    # ---- link ----------------------------------------------------------------------
    def measure_bias(self) -> None:
        """Stand still and see how fast the gyro reading creeps (the dog's own alignment
        code does the same). Subtracted from every later reading."""
        # A straight-line fit through ~20 readings, not just first vs last: any error in
        # this rate grows into a heading error over the whole walk.
        ts, ys = [], []
        end = self.clock() + BIAS_SECONDS
        while self.clock() < end:
            ts.append(self.clock())
            y = self._read_yaw()
            ys.append(y if not ys else ys[-1] + wrap(y - ys[-1]))   # unwrap
            self.sleep(TICK)
        mt, my = sum(ts) / len(ts), sum(ys) / len(ys)
        self.bias = (sum((t - mt) * (y - my) for t, y in zip(ts, ys))
                     / sum((t - mt) ** 2 for t in ts))
        if abs(self.bias) > MAX_BIAS:
            raise FollowError(f"the gyro drifts {self.bias:.1f} deg/s while standing still; "
                              f"too unreliable to steer by (restart the dog's program?)")
        self.say(f"  gyro drift {self.bias:+.2f} deg/s (subtracted)")

    def raw_yaw(self) -> float:
        """Gyro heading with the measured drift removed."""
        return self._read_yaw() - self.bias * self.clock()

    def _read_yaw(self) -> float:
        stats = self.xgo.read()
        if not stats.get("ok"):
            raise FollowError(f"can't read the dog: {stats.get('error', stats)}")
        try:
            return float(stats["imu"]["rotation"]["yaw"])
        except (KeyError, TypeError, ValueError):
            raise FollowError("the dog's stats have no heading (imu.rotation.yaw)") from None

    def drive(self, walk: float, yaw: float) -> None:
        if self.t0 is not None and self.clock() - self.t0 > self.max_seconds:
            raise FollowError(f"time limit ({self.max_seconds:.0f} s) reached")
        reply = self.xgo.write(walk=round(walk, 1), yaw=round(yaw, 1))
        if isinstance(reply, dict) and reply.get("ok") is False:
            raise FollowError(f"the dog refused a command: {reply.get('error', reply)}")

    def stop(self) -> None:
        self.xgo.write(walk=0, yaw=0)

    def hold(self, walk: float, yaw: float, seconds: float) -> None:
        """Keep a command running for `seconds`, resending it every TICK: the Pi stops
        the dog if no command arrives for 1 s (pi_xgo_bridge CommandWatchdog)."""
        end = self.clock() + seconds
        while True:
            self.drive(walk, yaw)
            left = end - self.clock()
            if left <= 0:
                return
            self.sleep(min(TICK, left))

    # ---- heading -------------------------------------------------------------------
    def learn_sign(self) -> None:
        """Short left turn: which way does the gyro count?"""
        before = self.raw_yaw()
        self.hold(0, self.turn, START_DELAY + 1.0)
        self.stop()
        self.sleep(0.5)
        change = wrap(self.raw_yaw() - before)
        if abs(change) < 3:
            raise FollowError(f"test turn moved the gyro only {change:.1f} deg; is the dog "
                              f"standing and taking commands?")
        self.sign = 1 if change > 0 else -1
        self.yaw0 = before          # heading 0 (map +x) = the way the dog faced at the start
        self.say(f"  gyro counts {'left' if self.sign > 0 else 'right'} as positive; "
                 f"test turn {abs(change):.0f} deg")

    def heading(self) -> float:
        """Map heading in degrees, counter-clockwise from +x."""
        return wrap(self.sign * (self.raw_yaw() - self.yaw0))

    def turn_to(self, target: float, timeout: float = 20.0) -> None:
        end = self.clock() + timeout
        while True:
            err = wrap(target - self.heading())
            if abs(err) <= HEADING_TOL:
                self.stop()
                return
            if self.clock() > end:
                raise FollowError(f"couldn't turn to {target:.0f} deg ({err:.0f} deg off)")
            speed = max(MIN_TURN, min(self.turn, abs(err)))
            # positive turn = left = ccw
            self.hold(0, math.copysign(speed, err), START_DELAY if abs(err) > 30 else TICK)

    # ---- walking -------------------------------------------------------------------
    def walk_to(self, gx: float, gy: float) -> None:
        dist = math.hypot(gx - self.x, gy - self.y)
        if dist < 3:
            return
        target = math.degrees(math.atan2(gy - self.y, gx - self.x))
        self.turn_to(target)
        moving = dist / self.cm_s
        start = self.clock()
        end = start + START_DELAY + moving
        along = side = 0.0           # cm along the line / to its left, from the gyro
        last = start
        while self.clock() < end:
            h = self.heading()
            now = self.clock()
            if now - start > START_DELAY:     # legs are moving: integrate the drift
                d = self.cm_s * (now - max(last, start + START_DELAY))
                along += d * math.cos(math.radians(h - target))
                side += d * math.sin(math.radians(h - target))
            last = now
            # steer back onto the line, not just parallel to it
            err = wrap(target - h) - CROSS_GAIN * side
            cmd = max(-HOLD_CAP, min(HOLD_CAP, HOLD_GAIN * err))
            if 1.0 <= abs(cmd) < MIN_TURN:
                cmd = math.copysign(MIN_TURN, cmd)
            elif abs(cmd) < 1.0:
                cmd = 0.0
            self.drive(self.walk, cmd)
            self.sleep(min(TICK, max(0.0, end - self.clock())))
        self.stop()
        # dead reckoning: the distance we timed, corrected by the drift the gyro saw
        t = math.radians(target)
        self.x += along * math.cos(t) - side * math.sin(t) if along else dist * math.cos(t)
        self.y += along * math.sin(t) + side * math.cos(t) if along else dist * math.sin(t)
        self.path.append((self.x, self.y))

    def follow(self, waypoints) -> None:
        self.t0 = self.clock()
        try:
            self.say("Measuring gyro drift (standing still) ...")
            self.measure_bias()
            self.say("Learning the gyro direction (short left turn) ...")
            self.learn_sign()
            self.turn_to(0.0)
            for n, (wx, wy) in enumerate(waypoints, 1):
                self.say(f"  waypoint {n}/{len(waypoints)}: ({wx:.0f}, {wy:.0f}) cm")
                self.walk_to(wx, wy)
        finally:
            self.stop()


def load_route(path: str, allow_unchecked: bool = False):
    with open(path) as fh:
        msg = json.load(fh)
    msg = msg.get("map", msg)            # find_pad --out wraps it as {"map": ..., ...}
    if not msg.get("target"):
        raise FollowError("the map has no target")
    if not msg.get("route"):
        raise FollowError("the map has no route for the dog")
    if not msg.get("route_checked") and not allow_unchecked:
        raise FollowError("the drone didn't fully check this route (route_checked false); "
                          "refusing. --allow-unchecked overrides, at your own risk")
    return msg


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("map", help="map.json written by find_pad.py")
    p.add_argument("--pi", help="dog's web server, e.g. http://100.74.30.90:8080")
    p.add_argument("--sim", action="store_true", help="drive a simulated dog instead")
    p.add_argument("--start", default="0,30",
                   help="dog's starting spot x,y cm from pad A (default 0,30: beside it, "
                        "on its left), facing +x")
    p.add_argument("--walk-speed-cm", type=float,
                   help=f"measured cm/s at walk value {WALK}; required for a real dog")
    p.add_argument("--max-seconds", type=float, default=180)
    p.add_argument("--allow-unchecked", action="store_true")
    args = p.parse_args(argv)

    try:
        msg = load_route(args.map, args.allow_unchecked)
    except (FollowError, OSError, ValueError) as exc:
        print(exc)
        return 1
    sx, sy = (float(v) for v in args.start.split(","))
    route = [tuple(w) for w in msg["route"]]

    if args.sim:
        sys.path.insert(0, __file__.rsplit("/", 1)[0])
        from sim_dog import FakeXgo
        dog = FakeXgo(start=(sx, sy))
        follower = RouteFollower(dog, args.walk_speed_cm or dog.cm_per_s(WALK), start=(sx, sy),
                                 clock=dog.clock, sleep=dog.sleep, max_seconds=args.max_seconds)
    else:
        if not args.pi or not args.walk_speed_cm:
            p.error("a real dog needs --pi and --walk-speed-cm (measure it first)")
        sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[2]))
        from tello_xgo_bridge.xgo_link import XgoLink
        dog = XgoLink(args.pi)
        follower = RouteFollower(dog, args.walk_speed_cm, start=(sx, sy),
                                 max_seconds=args.max_seconds)

    t = msg["target"]
    print(f"Route to target pad {msg['target_pad']} at ({t['x']:.0f}, {t['y']:.0f}) cm: "
          f"{len(route)} waypoints")
    try:
        follower.follow(route)
    except KeyboardInterrupt:
        print("\nCtrl+C: dog stopped.")
        return 130
    except FollowError as exc:
        print(f"Stopped: {exc}")
        return 1
    print(f"Done. Dog should be at ({follower.x:.0f}, {follower.y:.0f}) cm.")
    if args.sim:
        print(f"Simulated dog is really at ({dog.x:.0f}, {dog.y:.0f}) cm "
              f"({math.hypot(dog.x - t['x'], dog.y - t['y']):.0f} cm from the target).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
