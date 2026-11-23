"""Run the find-pad mission on a simulated drone and write a replay page you open in a browser.

The real mission code (find_pad.Mission) runs unchanged against sim_drone.FakeTello, in a
made-up room with pads, boxes and tracking errors. Every step is recorded: where the drone
really is, where it thinks it is, and what it has marked on the floor map. The page plays
that back from above, then walks the dog along the route it produced.

    .venv/bin/python scripts/search/simulate.py                    # default room, opens sim.html
    .venv/bin/python scripts/search/simulate.py --scenario drift
    .venv/bin/python scripts/search/simulate.py --scenario random --seed 7
    .venv/bin/python scripts/search/simulate.py --list

No hardware, no network.
"""

import argparse
import json
import random
import subprocess
import sys
from pathlib import Path

from mission import CELL, Mission, MissionError
from sim_drone import FakeTello

HERE = Path(__file__).resolve().parent

# Real-world trouble every scenario gets unless --ideal: noisy and occasionally garbage
# distance readings, pads missed now and then, a slowly creeping heading, a noisy
# barometer, and a battery that lasts about 11 minutes of flight. These are guesses,
# to be replaced with numbers measured on the real drone.
REALISM = dict(tof_noise=2.0, tof_glitch=0.03, pad_miss=0.15, yaw_drift=0.5,
               baro_noise=5.0, drain=100 / (11 * 60))
START_BATTERY = 90

# Each scenario: area (cm from pad A), pads {id: (x, y)}, boxes (x0, x1, y0, y1, height cm;
# a negative height is a drop such as a step down), errors, and which pads the mission
# knows about beforehand (`known`, besides pad A). Optional: drone (extra FakeTello
# options) and signal (the mission's obstacle_signal).
SCENARIOS = {
    "room": dict(
        about="A room with two boxes and a step down; the drone drifts a little to the right. "
              "The route home runs beside a box the drone never flew over: the amber squares "
              "next to the route were never checked.",
        area=dict(forward=300, back=0, left=120, right=120),
        pads={1: (0, 0), 2: (260, -80)},
        boxes=[(90, 140, -100, -20, 30), (180, 220, 20, 120, 40), (40, 80, 80, 120, -40)],
        move_error=(0.02, 0.0), bias=(0.0, -0.4), lanes=40),
    "box": dict(
        about="One long box between pad A and the target, with a gap on the left.",
        area=dict(forward=220, back=0, left=120, right=80),
        pads={1: (0, 0), 2: (180, 0)},
        boxes=[(70, 110, -90, 20, 30)],
        move_error=(0.0, 0.0), bias=(0.0, 0.0), lanes=20),
    "climb": dict(
        about="The drone climbs over boxes by itself, so its distance reading never changes. "
              "Only the barometer shows the box. Try --signal tof to see the box missed.",
        area=dict(forward=220, back=0, left=120, right=80),
        pads={1: (0, 0), 2: (180, 0)},
        boxes=[(70, 110, -90, 20, 30)],
        move_error=(0.0, 0.0), bias=(0.0, 0.0), lanes=20,
        drone=dict(auto_climb=True), signal="tof+baro"),
    "drift": dict(
        about="Open floor, but the drone slides 1 cm right on every move (a draught). "
              "It comes home off pad A, spirals to find it, and corrects the map.",
        area=dict(forward=200, back=0, left=60, right=60),
        pads={1: (0, 0), 2: (160, 40)},
        boxes=[], move_error=(0.0, 0.0), bias=(0.0, -1.0), lanes=40),
    "overshoot": dict(
        about="Every move overshoots by 10%. That cancels on the way home, so pad A can't see "
              "it; the checkpoint pad 3 at x=100 is what catches it.",
        area=dict(forward=220, back=0, left=60, right=60),
        pads={1: (0, 0), 3: (100, 0), 2: (180, 40)}, known={3: (100, 0)},
        boxes=[], move_error=(0.10, 0.0), bias=(0.0, 0.0), lanes=20),
}


def random_scenario(seed: int) -> dict:
    rng = random.Random(seed)
    fwd, side = rng.choice([200, 240, 300]), rng.choice([80, 100, 120])
    boxes = []
    for _ in range(rng.randint(1, 3)):
        w, l = rng.choice([40, 60]), rng.choice([40, 60, 100])
        x0 = rng.randrange(60, fwd - w, 20)
        y0 = rng.randrange(-side, side - l + 1, 20)
        boxes.append((x0, x0 + w, y0, y0 + l, rng.choice([20, 30, 40])))

    def clear(x, y):
        return all(not (x0 - 30 <= x <= x1 + 30 and y0 - 30 <= y <= y1 + 30)
                   for x0, x1, y0, y1, _ in boxes)
    for _ in range(200):
        tx, ty = rng.randrange(fwd // 2, fwd + 1, 20), rng.randrange(-side, side + 1, 20)
        if clear(tx, ty):
            break
    return dict(about=f"Random room, seed {seed}.",
                area=dict(forward=fwd, back=0, left=side, right=side),
                pads={1: (0, 0), 2: (tx, ty)}, boxes=boxes,
                move_error=(round(rng.uniform(-0.04, 0.04), 3), 0.0),
                bias=(0.0, round(rng.uniform(-0.8, 0.8), 2)), lanes=40)


class RecordingMission(Mission):
    """The real mission, plus a snapshot after every observation."""

    PHASES = (("Taking off", "Take off"), ("Searching", "Search"),
              ("target pad", "Target found"), ("Checking beside", "Check beside the route"),
              ("route home", "Fly the dog's route home"),
              ("spiralling", "Look for pad A"), ("tracking was off", "Correct the map"),
              ("Landing", "Landed"))

    def __init__(self, drone, **kw):
        self.frames, self._said, self.phase = [], [], "Take off"
        super().__init__(drone, say=self._say, sleep=drone.sleep, send=lambda m: None, **kw)

    # Messages that announce what happens next; everything else said between steps is
    # about the step that just happened, so it goes on that step's frame.
    ANNOUNCE = ("Taking off", "Searching", "Flying the dog's route home", "Landing")

    def _say(self, text):
        text = text.strip()
        between_steps = not getattr(self, "_observing", False)
        if self.frames and between_steps and not text.startswith(self.ANNOUNCE):
            self.frames[-1]["log"].append(text)
        else:
            self._said.append(text)
        for key, phase in self.PHASES:
            if key in text:
                self.phase = phase

    def observe(self):
        self._observing = True
        try:
            state = super().observe()
        finally:
            self._observing = False
        self.snap()
        return state

    def centre_over_target(self):
        super().centre_over_target()
        self.phase = "Centre over target"
        self._said.append("centred over the target pad")
        self.snap()

    def snap(self):
        est = self.tracker.pos
        target = None
        if self.found:
            target = [round(v, 1) for v in self.tracker.place(*self.found)]
        self.frames.append({
            "phase": self.phase,
            "true": [round(self.d.x, 1), round(self.d.y, 1)],
            "est": [round(est[0], 1), round(est[1], 1)],
            "grid": self.map.rows(),
            "target": target,
            "log": self._said,
            "t": round(self.d.t, 1),
            "battery": self.d.get_battery(),
        })
        self._said = []


def run(name: str, sc: dict, dog_width: float, margin: float, ideal: bool = False,
        signal: str = None, seed: int = 0, **mission_kw) -> dict:
    a = sc["area"]
    trouble = {} if ideal else dict(REALISM)
    trouble.update(sc.get("drone", {}))
    drone = FakeTello({k: tuple(v) for k, v in sc["pads"].items()}, boxes=sc["boxes"],
                      move_error=sc["move_error"], bias=sc["bias"], battery=START_BATTERY,
                      seed=seed, **trouble)
    signal = signal or sc.get("signal", "tof")
    m = RecordingMission(drone, target=2, x_range=(-a["back"], a["forward"]),
                         y_range=(-a["right"], a["left"]), dog_width=dog_width, margin=margin,
                         lane_cm=sc["lanes"], pads=sc.get("known", {}), obstacle_signal=signal,
                         **mission_kw)
    error = None
    try:
        msg = m.run()
    except MissionError as exc:
        error, msg = str(exc), m.report()
    m.phase = "Landed"
    m.snap()
    return {
        "name": name, "about": sc["about"], "cell": CELL, "error": error,
        "area": {"i": list(m.i_range), "j": list(m.j_range)},
        "pads": {str(k): list(v) for k, v in sc["pads"].items()},
        "origin_pad": 1, "target_pad": 2,
        "known_pads": [str(k) for k in sc.get("known", {})],
        "boxes": [list(b) for b in sc["boxes"]],
        "errors": {"move_error": sc["move_error"], "bias": sc["bias"], **trouble},
        "signal": signal, "ideal": ideal,
        "dog_width": dog_width, "clearance": m.radius,
        "frames": m.frames,
        "corrected": [[round(x, 1), round(y, 1)] for x, y in m.tracker.points],
        "map": msg,
    }


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--scenario", default="room", choices=[*SCENARIOS, "random"])
    p.add_argument("--seed", type=int, default=1, help="for --scenario random")
    p.add_argument("--ideal", action="store_true",
                   help="perfect sensors and battery (only the scenario's own errors)")
    p.add_argument("--signal", choices=["tof", "tof+baro"],
                   help="how the drone spots boxes (default: the scenario's choice, else tof)")
    p.add_argument("--dog-width", type=float, default=30)
    p.add_argument("--margin", type=float, default=10)
    p.add_argument("--out", default="sim.html")
    p.add_argument("--no-open", action="store_true", help="don't open the page")
    p.add_argument("--list", action="store_true", help="list scenarios and exit")
    args = p.parse_args(argv)

    if args.list:
        for k, v in SCENARIOS.items():
            print(f"{k:10} {v['about']}")
        print(f"{'random':10} A random room; pick it with --seed.")
        return 0

    sc = random_scenario(args.seed) if args.scenario == "random" else SCENARIOS[args.scenario]
    name = f"random (seed {args.seed})" if args.scenario == "random" else args.scenario
    data = run(name, sc, args.dog_width, args.margin, ideal=args.ideal, signal=args.signal,
               seed=args.seed)

    page = (HERE / "sim_template.html").read_text()
    page = page.replace("/*SIM_DATA*/null", json.dumps(data, separators=(",", ":")))
    out = Path(args.out)
    out.write_text(page)

    t, msg = data["map"]["target"], data["map"]
    true_t = sc["pads"][2]
    print(f"{name}: {len(data['frames'])} steps")
    if t:
        err = ((t["x"] - true_t[0]) ** 2 + (t["y"] - true_t[1]) ** 2) ** 0.5
        print(f"  target at x={t['x']:.0f} y={t['y']:.0f} cm (really {true_t[0]}, {true_t[1]}; "
              f"off by {err:.0f} cm)")
    else:
        print("  target not found")
    print(f"  dog route: {len(msg['route'])} waypoints, checked by drone: {msg['route_checked']}")
    print(f"  drift at home: {msg['drift_at_home_cm']}")
    last = data["frames"][-1]
    print(f"  flight time {last['t'] / 60:.1f} min, battery {START_BATTERY}% -> {last['battery']}%"
          + ("" if not args.ideal else " (ideal: no drain)"))
    if data["error"]:
        print(f"  mission stopped: {data['error']}")
    print(f"Wrote {out.resolve()}")
    if not args.no_open and sys.platform == "darwin":
        subprocess.run(["open", str(out)], check=False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
