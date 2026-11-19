"""Run the mission in many random simulated rooms and score it against the real layout.

The mission only knows what its sensors told it; the simulator knows where every box
really is. For each room this records whether the target was found and how far off,
whether the dog's route was marked checked, the closest that route comes to a real
obstacle, and the battery left. The numbers that matter most:

  unsafe      a route marked checked that comes within the dog's half-width of a real
              obstacle (the dog would touch it). Must be 0.
  in margin   a checked route closer than half-width + margin (no collision, less room
              than asked for).

    .venv/bin/python scripts/search/stress.py                  # 300 rooms
    .venv/bin/python scripts/search/stress.py --rooms 1000 --show-worst 5
"""

import argparse
import math
import statistics
import sys

import simulate

DOG_WIDTH, MARGIN = 30.0, 10.0


def clearance(route, boxes) -> float:
    """Smallest distance (cm) from the route's centre line to any real obstacle."""
    best = math.inf
    for (ax, ay), (bx, by) in zip(route, route[1:]):
        n = max(1, int(math.hypot(bx - ax, by - ay) / 2))   # every 2 cm
        for k in range(n + 1):
            x, y = ax + (bx - ax) * k / n, ay + (by - ay) * k / n
            for x0, x1, y0, y1, h in boxes:
                if h == 0:
                    continue
                best = min(best, math.hypot(max(x0 - x, 0, x - x1), max(y0 - y, 0, y - y1)))
    return best


def evaluate(seed: int, dog_width=DOG_WIDTH, margin=MARGIN, ideal=False) -> dict:
    sc = simulate.random_scenario(seed)
    data = simulate.run(f"seed {seed}", sc, dog_width, margin, ideal=ideal, seed=seed)
    msg, last = data["map"], data["frames"][-1]
    tx, ty = sc["pads"][2]
    t = msg["target"]
    return {
        "seed": seed,
        "found": t is not None,
        "target_err": math.hypot(t["x"] - tx, t["y"] - ty) if t else None,
        "checked": msg["route_checked"],
        "clearance": clearance(msg["route"], sc["boxes"]) if msg["route"] else None,
        "battery": last["battery"],
        "minutes": last["t"] / 60,
        "stopped": data["error"],
    }


def summarise(results, dog_width=DOG_WIDTH, margin=MARGIN) -> dict:
    half = dog_width / 2
    checked = [r for r in results if r["checked"]]
    errs = [r["target_err"] for r in results if r["found"]]
    return {
        "rooms": len(results),
        "found": sum(r["found"] for r in results),
        "checked": len(checked),
        "unsafe": [r for r in checked if r["clearance"] < half],
        "in_margin": [r for r in checked if half <= r["clearance"] < half + margin],
        "unchecked_would_collide": sum(1 for r in results if not r["checked"]
                                       and r["clearance"] is not None
                                       and r["clearance"] < half),
        "target_err_median": statistics.median(errs) if errs else None,
        "target_err_max": max(errs) if errs else None,
        "low_battery_end": sum(r["battery"] < 20 for r in results),
        "landed_early": sum(bool(r["stopped"]) for r in results),
        "minutes_max": max(r["minutes"] for r in results),
    }


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--rooms", type=int, default=300)
    p.add_argument("--first-seed", type=int, default=1)
    p.add_argument("--ideal", action="store_true", help="perfect sensors")
    p.add_argument("--show-worst", type=int, default=3)
    args = p.parse_args(argv)

    results = [evaluate(s, ideal=args.ideal)
               for s in range(args.first_seed, args.first_seed + args.rooms)]
    s = summarise(results)
    n = s["rooms"]
    print(f"{n} random rooms ({'ideal' if args.ideal else 'realistic'} sensors), "
          f"dog {DOG_WIDTH:.0f} cm wide, margin {MARGIN:.0f} cm")
    print(f"  target found          {s['found']}/{n}; error median {s['target_err_median']:.0f} cm, "
          f"worst {s['target_err_max']:.0f} cm")
    print(f"  route marked checked  {s['checked']}/{n}")
    print(f"  UNSAFE checked routes {len(s['unsafe'])}  (dog would touch an obstacle)")
    print(f"  checked, inside margin {len(s['in_margin'])}  (no contact, less room than asked)")
    print(f"  unchecked routes that would collide {s['unchecked_would_collide']} "
          f"(refused by the dog, as they should be)")
    print(f"  ended below 20% battery {s['low_battery_end']}; landed away from pad A "
          f"{s['landed_early']}; longest flight {s['minutes_max']:.1f} min")
    worst = sorted((r for r in results if r["checked"]), key=lambda r: r["clearance"])
    for r in worst[:args.show_worst]:
        print(f"  closest checked route: seed {r['seed']}, {r['clearance']:.1f} cm from an obstacle")
    return 1 if s["unsafe"] else 0


if __name__ == "__main__":
    sys.exit(main())
