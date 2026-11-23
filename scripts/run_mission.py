"""Whole mission through tello_xgo_bridge: the drone finds the target and maps a safe route,
then the dog walks it.

  1. Check the dog answers (before anything flies).
  2. Drone: take off from pad A, search for the target pad, map free/blocked floor, check
     the dog's route, land on pad A (scripts/search/mission.py's Mission, driven through
     bridge.tello.drone).
  3. Save the map. Stop here unless the route was fully checked.
  4. You lift the drone off pad A and confirm the dog's path is clear.
  5. Dog: walk the route from --dog-start (scripts/dog/follow_route.py through bridge.xgo).

    .venv/bin/python scripts/run_mission.py --sim                       # simulated drone + dog
    .venv/bin/python scripts/run_mission.py --pi http://100.74.30.90:8080 --target 2 \
        --forward 200 --left 80 --right 60 --dog-width 30 --walk-speed-cm 6.5

Setup (see README): pad A beside the dog, rocket pointing the way the dog faces; drone on
pad A; laptop on the drone's Wi-Fi with a second connection to reach the dog (Tailscale);
on the Pi, `python3 main.py stream` with the /command handler. Run from Apple Terminal.
Ctrl+C lands the drone / stops the dog at any point.
"""

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE / "search"), str(HERE / "dog"), str(HERE), str(HERE.parent)]

import calibration  # noqa: E402
import follow_route  # noqa: E402
from mission import Mission, MissionError  # noqa: E402
from flight_common import countdown, preflight  # noqa: E402
from follow_route import FollowError, RouteFollower  # noqa: E402


def run_sim(args) -> int:
    import simulate
    from sim_dog import FakeXgo
    from sim_drone import FakeTello

    sc = simulate.SCENARIOS[args.scenario]
    a = sc["area"]
    drone = FakeTello({k: tuple(v) for k, v in sc["pads"].items()}, boxes=sc["boxes"],
                      move_error=sc["move_error"], bias=sc["bias"], battery=90,
                      **simulate.REALISM, **sc.get("drone", {}))
    print(f"Simulated room '{args.scenario}': {sc['about']}")
    mission = Mission(drone, target=2, x_range=(-a["back"], a["forward"]),
                      y_range=(-a["right"], a["left"]), dog_width=args.dog_width,
                      margin=args.margin, lane_cm=sc["lanes"], pads=sc.get("known", {}),
                      obstacle_signal=sc.get("signal", "tof"), sleep=drone.sleep,
                      say=print, send=lambda m: None)
    msg = mission.run()
    save(args.out, msg, mission.raw)
    if not ready_for_dog(msg):
        return 2
    start = parse_xy(args.dog_start)
    dog = FakeXgo(start=start, speed_error=0.05, creep=0.5, gyro_drift=0.5)
    follower = RouteFollower(dog, dog.cm_per_s(8), start=start, clock=dog.clock,
                             sleep=dog.sleep)
    follower.follow(route_from(msg, start))
    t = msg["target"]
    tx, ty = sc["pads"][2]
    print(f"Dog thinks it's at ({follower.x:.0f}, {follower.y:.0f}); really at "
          f"({dog.x:.0f}, {dog.y:.0f}), {((dog.x - tx) ** 2 + (dog.y - ty) ** 2) ** .5:.0f} cm "
          f"from the real target (mapped at {t['x']:.0f}, {t['y']:.0f}).")
    return 0


def run_real(args) -> int:
    from tello_xgo_bridge import TelloXgoBridge

    missing = calibration.apply_to_flight(args.cal)
    follow_route.MIN_TURN = calibration.value(args.cal, "dog", "min_turn")
    unmeasured = calibration.unmeasured(args.cal)
    if unmeasured:
        print("WARNING: still guesses (run scripts/calibrate.py): " + ", ".join(unmeasured))
    if missing:
        answer = input("The drone's directions aren't calibrated, so the map may be mirrored. "
                       "Type 'fly anyway' to continue: ")
        if answer.strip().lower() != "fly anyway":
            return 1
    bridge = TelloXgoBridge(pi_url=args.pi)
    stats = bridge.xgo.read()
    if not stats.get("ok"):
        print(f"The dog doesn't answer ({stats.get('error')}). Not flying.")
        return 1
    print(f"Dog answers: battery {stats.get('battery_percent')}%.")
    print("Connecting to the drone ...")
    bridge.open()
    try:
        drone = bridge.tello.drone
        mission = Mission(drone, target=args.target, origin=args.origin,
                          x_range=(-args.back, args.forward), y_range=(-args.right, args.left),
                          dog_width=args.dog_width, margin=args.margin, lane_cm=args.lanes,
                          obstacle_signal=args.obstacle_signal, max_glide=args.glide)
        problem = preflight(drone, ignore_temp=args.ignore_temp)
        if problem:
            print(problem)
            return 1
        if not countdown(args.countdown):
            return 130
        try:
            msg = mission.run()
        except (MissionError, KeyboardInterrupt) as exc:
            print(f"Drone mission stopped ({exc or 'Ctrl+C'}); landed. Not walking the dog.")
            save(args.out, mission.report(), mission.raw)
            return 1
        save(args.out, msg, mission.raw)
        if not ready_for_dog(msg):
            return 2

        start = parse_xy(args.dog_start)
        answer = input("Drone has landed on pad A. Lift it off the pad, check the dog's path "
                       "is clear and stay by the dog. Type 'go' to walk the dog: ")
        if answer.strip().lower() != "go":
            print("Not walking the dog.")
            return 0
        follower = RouteFollower(bridge.xgo, args.walk_speed_cm, start=start,
                                 max_seconds=args.dog_max_seconds)
        try:
            follower.follow(route_from(msg, start))
        except KeyboardInterrupt:
            print("\nCtrl+C: dog stopped.")
            return 130
        except FollowError as exc:
            print(f"Dog stopped: {exc}")
            return 1
        print(f"Dog should now be at the target ({follower.x:.0f}, {follower.y:.0f} cm).")
        return 0
    finally:
        bridge.close()


def route_from(msg: dict, start) -> list:
    """The dog's waypoints. The route starts at pad A, where the drone just landed; the
    dog stands beside it, so it heads straight for the second waypoint."""
    route = [tuple(p) for p in msg["route"]]
    if len(route) > 1 and route[0] == (0, 0) and tuple(start) != (0, 0):
        route = route[1:]
    return route


def ready_for_dog(msg: dict) -> bool:
    if not msg.get("target"):
        print("Target not found: nothing for the dog to do.")
        return False
    if not msg.get("route_checked"):
        print("The drone couldn't fully check a route for the dog; not walking it. "
              "See the map file for what's unchecked.")
        return False
    t = msg["target"]
    print(f"Target at ({t['x']:.0f}, {t['y']:.0f}) cm; checked route with "
          f"{len(msg['route'])} waypoints.")
    return True


def save(path: str, msg: dict, raw: list) -> None:
    Path(path).write_text(json.dumps({"map": msg, "flight_log": raw}, indent=1) + "\n")
    print(f"Map saved to {path}")


def parse_xy(text: str) -> tuple:
    x, y = (float(v) for v in text.split(","))
    return x, y


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--sim", action="store_true", help="simulated drone and dog")
    p.add_argument("--scenario", default="box", help="simulated room (see simulate.py --list)")
    p.add_argument("--pi", help="dog's web server, e.g. http://100.74.30.90:8080")
    p.add_argument("--target", type=int, help="target pad ID (1-8)")
    p.add_argument("--origin", type=int, default=1, help="pad A's ID (default 1)")
    p.add_argument("--forward", type=float)
    p.add_argument("--back", type=float, default=0)
    p.add_argument("--left", type=float)
    p.add_argument("--right", type=float)
    p.add_argument("--dog-width", type=float, default=30)
    p.add_argument("--margin", type=float, default=10)
    p.add_argument("--lanes", type=float, default=40)
    p.add_argument("--glide", type=int, default=5,
                   help="longest search move in 20 cm squares (1 = stop every 20 cm)")
    p.add_argument("--obstacle-signal", choices=["tof", "tof+baro"],
                   help="default: tof+baro if calibration found the drone climbs over boxes")
    p.add_argument("--dog-start", default="0,30",
                   help="where the dog stands, x,y cm from pad A, facing +x (default 0,30)")
    p.add_argument("--walk-speed-cm", type=float,
                   help="dog speed, cm/s at walk 8 (default: calibration.json)")
    p.add_argument("--dog-max-seconds", type=float, default=180)
    p.add_argument("--out", default="map.json")
    p.add_argument("--countdown", type=int, default=5)
    p.add_argument("--ignore-temp", action="store_true")
    args = p.parse_args(argv)

    if args.sim:
        args.obstacle_signal = args.obstacle_signal or "tof"
        return run_sim(args)
    args.cal = calibration.load()
    if args.walk_speed_cm is None:
        args.walk_speed_cm = calibration.value(args.cal, "dog", "walk_cm_per_s")
    if args.obstacle_signal is None:
        args.obstacle_signal = ("tof+baro" if calibration.value(args.cal, "drone", "auto_climb")
                                else "tof")
    missing = [n for n in ("pi", "target", "forward", "left", "right", "walk_speed_cm")
               if getattr(args, n) is None]
    if missing:
        p.error("a real run needs " + ", ".join("--" + n.replace("_", "-") for n in missing))
    return run_real(args)


if __name__ == "__main__":
    sys.exit(main())
