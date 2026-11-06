"""Vertical-only test flight for the RoboMaster TT (Tello Talent).

Take off (the drone rises to ~80 cm on its own) and land again right away.
Optionally climb a little higher and/or hover first.
No forward/back/sideways commands are sent at any point.

Usage (Mac joined to the drone's TELLO-xxxxxx / RMTT-xxxxxx Wi-Fi, run from
Apple Terminal: Ghostty lacks macOS Local Network permission and gets no replies):
    .venv/bin/python scripts/up_down.py --check     # connect + battery only, no flight
    .venv/bin/python scripts/up_down.py             # fly: take off, land immediately
    .venv/bin/python scripts/up_down.py --climb 20 --hover 1

Ctrl+C at any time during flight sends a normal land command.
"""

import argparse
import sys
import time

from djitellopy import Tello

MIN_BATTERY = 25      # percent; the drone itself refuses takeoff below ~10%
MAX_CLIMB_CM = 50     # keep indoor tests low; SDK allows 20-500
MIN_CLIMB_CM = 20     # SDK minimum for move_up/move_down
MAX_TEMP_C = 90       # refuse takeoff if the drone is this hot (it heats up idling on the ground)


def status(tello: Tello, label: str) -> None:
    print(f"[{label}] battery={tello.get_battery()}%  "
          f"height={tello.get_height()}cm  tof={tello.get_distance_tof()}cm  "
          f"temp={tello.get_highest_temperature()}C")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true",
                        help="connect and print status only, do not fly")
    parser.add_argument("--climb", type=int, default=0,
                        help=f"extra cm to climb after takeoff ({MIN_CLIMB_CM}-{MAX_CLIMB_CM}, 0 = skip)")
    parser.add_argument("--hover", type=float, default=0.0,
                        help="seconds to hover at the top")
    parser.add_argument("--countdown", type=int, default=5,
                        help="seconds to wait before takeoff (Ctrl+C cancels)")
    parser.add_argument("--ignore-temp", action="store_true",
                        help=f"fly even if the drone reports {MAX_TEMP_C}C or more")
    args = parser.parse_args()

    if args.climb != 0 and not MIN_CLIMB_CM <= args.climb <= MAX_CLIMB_CM:
        parser.error(f"--climb must be 0 or between {MIN_CLIMB_CM} and {MAX_CLIMB_CM}")

    tello = Tello()
    print("Connecting to drone at 192.168.10.1 ...")
    tello.connect()
    status(tello, "connected")

    if args.check:
        print("Check only, not flying.")
        tello.end()
        return 0

    battery = tello.get_battery()
    if battery < MIN_BATTERY:
        print(f"Battery {battery}% is below {MIN_BATTERY}%, charge before flying.")
        tello.end()
        return 1

    temp = tello.get_highest_temperature()
    if temp >= MAX_TEMP_C and args.ignore_temp:
        print(f"WARNING: drone is at {temp}C (limit {MAX_TEMP_C}C), flying anyway (--ignore-temp).")
    elif temp >= MAX_TEMP_C:
        print(f"Drone is at {temp}C (limit {MAX_TEMP_C}C). Power it off to cool before flying.")
        tello.end()
        return 1

    try:
        for remaining in range(args.countdown, 0, -1):
            print(f"Taking off in {remaining}... (Ctrl+C to cancel)")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nCancelled before takeoff. Nothing flew.")
        tello.end()
        return 130

    flying = False
    try:
        print("Taking off (drone climbs to ~80 cm on its own) ...")
        # Mark as flying *before* takeoff(): the call blocks until the drone
        # reports it's done (up to 20 s), and it's already airborne by then.
        # A Ctrl+C or timeout during that wait must still trigger a land.
        flying = True
        tello.takeoff()
        status(tello, "after takeoff")

        if args.climb:
            print(f"Climbing {args.climb} cm ...")
            tello.move_up(args.climb)
            status(tello, "top")

        if args.hover > 0:
            print(f"Hovering {args.hover}s ...")
            time.sleep(args.hover)

        if args.climb:
            print(f"Descending {args.climb} cm ...")
            tello.move_down(args.climb)
            status(tello, "back down")

        print("Landing ...")
        tello.land()
        flying = False
        print("Landed.")
        return 0
    except KeyboardInterrupt:
        print("\nCtrl+C: landing now.")
        return 130
    except Exception as exc:  # any SDK error/timeout mid-flight: get it on the ground
        print(f"Error during flight: {exc!r}. Landing.")
        return 1
    finally:
        if flying:
            try:
                tello.land()
            except Exception as exc:
                print(f"Land command failed: {exc!r}. Grab the drone or use the "
                      f"power button; 'tello.emergency()' cuts motors instantly.")
        tello.end()


if __name__ == "__main__":
    sys.exit(main())
