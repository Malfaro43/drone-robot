"""Guided calibration: measure the numbers the flight and walking code still guess, and
save them to calibration.json (see scripts/calibration.py).

  drone-axes   NO FLIGHT. Hold the drone about 50 cm above pad A and move it by hand:
               which way the pad x/y readings and the heading count.
  drone-hover  SHORT FLIGHT over pad A (asks you to confirm a clear space first): which way
               `go 0 20 0` moves, how far from a pad it still sees it, sensor noise, and
               (optional, with a box) whether it climbs over boxes by itself.
  dog          Through tello_xgo_bridge: turn rate, smallest turn that moves the legs, and
               walking speed (you measure the distance with a tape).
  show         Print what's measured and what's still a guess.

    .venv/bin/python scripts/calibrate.py drone-axes
    .venv/bin/python scripts/calibrate.py drone-hover
    .venv/bin/python scripts/calibrate.py dog --pi http://100.74.30.90:8080
    .venv/bin/python scripts/calibrate.py show

Run drone steps from Apple Terminal on the drone's Wi-Fi. Ctrl+C lands / stops the dog.
"""

import argparse
import statistics
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE / "search"), str(HERE / "dog"), str(HERE.parent)]

import calibration  # noqa: E402

MOVE_CM = 15        # how far to move the drone by hand in drone-axes
MIN_SEEN_CM = 5     # a hand move smaller than this on the pad readings isn't trusted


class CalibrationError(Exception):
    pass


def sign(v: float) -> int:
    return 1 if v > 0 else -1


# ---- drone-axes (no flight) ---------------------------------------------------------
def read_pad(drone, tries=10, sleep=time.sleep):
    """Median pad reading (id, x, y) over a few samples, or None if no pad is seen."""
    seen = []
    for _ in range(tries):
        mid = drone.get_mission_pad_id()
        if mid is not None and mid > 0:
            seen.append((mid, drone.get_mission_pad_distance_x(), drone.get_mission_pad_distance_y()))
        sleep(0.1)
    if len(seen) < tries // 3:
        return None
    mid = statistics.mode(s[0] for s in seen)
    return (mid, statistics.median(s[1] for s in seen if s[0] == mid),
            statistics.median(s[2] for s in seen if s[0] == mid))


def drone_axes(drone, cal, ask=input, say=print, sleep=time.sleep, path=calibration.PATH):
    drone.enable_mission_pads()
    drone.set_mission_pad_detection_direction(0)

    def reading(prompt):
        ask(prompt + " Press Enter when ready.")
        r = read_pad(drone, sleep=sleep)
        if r is None:
            raise CalibrationError("the drone can't see a pad. Hold it level about 50 cm above "
                                   "the pad (the camera needs at least ~30 cm).")
        return r

    say("Motors stay off. Hold the drone level about 50 cm above pad A, its nose pointing "
        "where the pad's rocket points.")
    _, x0, y0 = reading("Hold it right above the pad's centre.")
    yaw0 = drone.get_yaw()
    _, x1, y1 = reading(f"Move it about {MOVE_CM} cm FORWARD (the way the rocket points), "
                        f"same height.")
    _, x2, y2 = reading(f"Now move it back over the centre, then about {MOVE_CM} cm to its "
                        f"LEFT (the drone's left), same height.")
    fwd, side = x1 - x0, y2 - y0
    for name, moved, other in (("forward", fwd, y1 - y0), ("left", side, x2 - x0)):
        if abs(moved) < MIN_SEEN_CM or abs(moved) < abs(other):
            raise CalibrationError(f"the {name} move barely showed in the readings "
                                   f"({moved:+.0f} cm vs {other:+.0f} cm sideways); try again")
    ask("Back over the centre, then turn the drone about 45 degrees CLOCKWISE (seen from "
        "above), same spot. Press Enter when ready.")
    turned = ((drone.get_yaw() - yaw0 + 180) % 360) - 180
    if abs(turned) < 15:
        raise CalibrationError(f"the heading only changed {turned:+.0f} degrees; turn it more")

    results = {"pad_x_sign": sign(fwd), "pad_y_sign": sign(side), "yaw_cw_positive": turned > 0}
    for k, v in results.items():
        calibration.record(cal, "drone", k, v, path)
    say(f"Saved: pad x {'grows' if fwd > 0 else 'shrinks'} going forward ({fwd:+.0f} cm), "
        f"pad y {'grows' if side > 0 else 'shrinks'} going left ({side:+.0f} cm), heading "
        f"{'grows' if turned > 0 else 'shrinks'} turning clockwise ({turned:+.0f} deg).")
    return results


# ---- drone-hover (short flight) -------------------------------------------------------
def drone_hover(drone, cal, ask=input, say=print, sleep=time.sleep, path=calibration.PATH,
                origin=1, speed=20):
    from flight_common import land_now

    if not cal["drone"]["pad_y_sign"]["measured"]:
        raise CalibrationError("run drone-axes first (this step relies on its pad y direction)")
    if ask("The drone will take off over pad A, move up to 60 cm forward and 20 cm left, and "
           "land. Is there 1 m of clear space around it? Type 'yes': ").strip().lower() != "yes":
        say("Not flying.")
        return None
    drone.enable_mission_pads()
    drone.set_mission_pad_detection_direction(0)
    out = {}
    flying = True
    try:
        drone.takeoff()
        start = read_pad(drone, sleep=sleep)
        if not start or start[0] != origin:
            raise CalibrationError(f"pad {origin} not under the drone after takeoff (saw {start})")

        # sensor noise while hovering
        tofs, baros = [], []
        for _ in range(20):
            tofs.append(drone.get_distance_tof())
            baros.append(drone.get_barometer())
            sleep(0.1)
        out["tof_noise_cm"] = round(statistics.pstdev(t for t in tofs if 0 < t < 600), 1)
        out["baro_noise_cm"] = round(statistics.pstdev(baros), 1)

        # which way does `go 0 20 0` go?
        drone.go_xyz_speed(0, 20, 0, speed)
        after = read_pad(drone, sleep=sleep)
        if not after:
            raise CalibrationError("lost the pad after moving 20 cm sideways")
        moved_left = (after[2] - start[2]) * calibration.value(cal, "drone", "pad_y_sign")
        out["go_y_sign"] = sign(moved_left)
        drone.go_xyz_speed(0, -20, 0, speed)

        # how far forward is the pad still seen?
        seen = moved = 0
        for dist in (20, 40, 60):
            drone.go_xyz_speed(20, 0, 0, speed)
            moved = dist
            if read_pad(drone, sleep=sleep):
                seen = dist
            else:
                break
        out["pad_range_cm"] = seen + 10       # lost somewhere between seen and seen + 20
        drone.go_xyz_speed(-moved, 0, 0, speed)

        # optional box test
        if ask("Optional: put a box at least 15 cm tall about 60 cm in front of pad A "
               "(in line with the rocket). Done? Type 'yes' to test, Enter to skip: "
               ).strip().lower() == "yes":
            base_tof, base_baro = statistics.median(tofs), statistics.median(baros)
            drops, rises = [], []
            for _ in range(5):
                drone.go_xyz_speed(20, 0, 0, speed)
                sleep(0.5)
                drops.append(base_tof - drone.get_distance_tof())
                rises.append(statistics.median(drone.get_barometer() for _ in range(5)) - base_baro)
            drone.go_xyz_speed(-100, 0, 0, speed)
            if max(drops) >= 10:
                out["auto_climb"] = False
            elif max(rises) >= 10:
                out["auto_climb"] = True
            else:
                say("  Box test inconclusive (neither sensor changed by 10 cm); not saved.")
            say(f"  Box test: distance reading fell up to {max(drops):.0f} cm, barometer rose "
                f"up to {max(rises):.0f} cm.")

        drone.land()
        flying = False
    finally:
        if flying:
            land_now(drone)
    for k, v in out.items():
        calibration.record(cal, "drone", k, v, path)
    say("Saved: " + ", ".join(f"{k} = {v}" for k, v in out.items()))
    return out


# ---- dog (through the bridge) ---------------------------------------------------------
def dog(xgo, cal, ask=input, say=print, clock=time.monotonic, sleep=time.sleep,
        path=calibration.PATH, walk_seconds=6.0):
    from follow_route import START_DELAY, RouteFollower, wrap

    stats = xgo.read()
    if not stats.get("ok"):
        raise CalibrationError(f"can't reach the dog: {stats.get('error')}")
    if ask("The dog will turn on the spot a few times, then walk straight about 50 cm. Clear "
           "1 m around it and stay by it. Type 'yes': ").strip().lower() != "yes":
        say("Not moving the dog.")
        return None
    f = RouteFollower(xgo, walk_cm_per_s=1.0, clock=clock, sleep=sleep, say=say)
    out = {}
    try:
        f.measure_bias()
        f.learn_sign()

        def turned(turn_value, seconds):
            h0 = f.heading()
            f.hold(0, turn_value, seconds)
            f.stop()
            sleep(0.5)
            return wrap(f.heading() - h0)

        out["turn_deg_per_s"] = round(turned(16, START_DELAY + 3.0) / 3.0, 1)
        for v in (2, 4, 6, 8, 10, 12):
            if abs(turned(v, START_DELAY + 2.0)) >= 4:
                out["min_turn"] = float(v)
                break
        f.turn_to(0.0)

        ask("Mark where the dog's front feet are now. It will walk straight ahead. "
            "Press Enter to start.")
        f.hold(f.walk, 0, START_DELAY + walk_seconds)
        f.stop()
        cm = float(ask(f"How many cm did it walk (front feet, start to end)? "))
        out["walk_cm_per_s"] = round(cm / walk_seconds, 2)
    finally:
        f.stop()
    for k, v in out.items():
        calibration.record(cal, "dog", k, v, path)
    say("Saved: " + ", ".join(f"{k} = {v}" for k, v in out.items()))
    return out


def show(cal, say=print):
    for robot, keys in cal.items():
        say(robot)
        for k, v in keys.items():
            when = f"measured {v['measured']}" if v["measured"] else "GUESS (not measured)"
            say(f"  {k:16} {str(v['value']):8} {when}")


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("step", choices=["drone-axes", "drone-hover", "dog", "show"])
    p.add_argument("--pi", help="dog's web server, for the dog step")
    args = p.parse_args(argv)
    cal = calibration.load()
    if args.step == "show":
        show(cal)
        return 0
    try:
        if args.step == "dog":
            if not args.pi:
                p.error("the dog step needs --pi http://<dog>:8080")
            from tello_xgo_bridge.xgo_link import XgoLink
            dog(XgoLink(args.pi), cal)
        else:
            from djitellopy import Tello
            drone = Tello()
            print("Connecting to the drone ...")
            drone.connect()
            try:
                (drone_axes if args.step == "drone-axes" else drone_hover)(drone, cal)
            finally:
                drone.end()
    except CalibrationError as exc:
        print(f"Calibration stopped: {exc}")
        return 1
    except KeyboardInterrupt:
        print("\nStopped.")
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
