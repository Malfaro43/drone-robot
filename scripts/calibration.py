"""Measured robot settings, shared by the flight, walking and calibration scripts.

One file, `calibration.json` at the top of drone-robot, written by scripts/calibrate.py
and committed so everyone flies with the same measured numbers. Every value says whether
it was measured (and when) or is still the built-in guess.

The real-hardware scripts (find_pad.py, follow_route.py, run_mission.py) apply it. The
simulation and tests don't: they use the built-in defaults, so a measurement can't
silently change test results.
"""

import json
from datetime import date
from pathlib import Path

PATH = Path(__file__).resolve().parents[1] / "calibration.json"

# name: (default, what it is)
DEFAULTS = {
    "drone": {
        "pad_x_sign": (1, "+1 if the pad 'x' reading grows as the drone moves forward (pad rocket direction)"),
        "pad_y_sign": (1, "+1 if the pad 'y' reading grows as the drone moves left"),
        "yaw_cw_positive": (True, "drone yaw reading grows when it turns clockwise (seen from above)"),
        "go_y_sign": (1, "+1 if `go 0 20 0` moves the drone left"),
        "pad_range_cm": (25, "how far from a pad's centre the drone still detects it, at flight height"),
        "auto_climb": (None, "does the drone rise by itself over a box (hiding it from the distance sensor)?"),
        "tof_noise_cm": (2.0, "standard deviation of the downward distance reading while hovering"),
        "baro_noise_cm": (5.0, "standard deviation of the barometer reading while hovering"),
    },
    "dog": {
        "walk_cm_per_s": (None, "forward speed at walk value 8"),
        "min_turn": (6.0, "smallest turn value that actually turns the dog"),
        "turn_deg_per_s": (None, "turn rate at turn value 16"),
    },
}


def load(path: Path = PATH) -> dict:
    """{"drone": {...}, "dog": {...}} with every key present; each value is
    {"value": v, "measured": "YYYY-MM-DD" or None}."""
    stored = {}
    if Path(path).exists():
        stored = json.loads(Path(path).read_text())
    out = {}
    for robot, keys in DEFAULTS.items():
        out[robot] = {}
        for key, (default, _) in keys.items():
            entry = stored.get(robot, {}).get(key)
            out[robot][key] = entry if isinstance(entry, dict) else {"value": default,
                                                                     "measured": None}
    return out


def value(cal: dict, robot: str, key: str):
    return cal[robot][key]["value"]


def record(cal: dict, robot: str, key: str, val, path: Path = PATH, note: str = "") -> None:
    """Store a measured value and save the file straight away."""
    cal[robot][key] = {"value": val, "measured": date.today().isoformat()}
    if note:
        cal[robot][key]["note"] = note
    save(cal, path)


def save(cal: dict, path: Path = PATH) -> None:
    doc = {"_about": "Measured robot settings. Written by scripts/calibrate.py; "
                     "'measured': null means it's still the built-in guess."}
    for robot, keys in cal.items():
        doc[robot] = {k: {**v, "what": DEFAULTS[robot][k][1]} for k, v in keys.items()}
    Path(path).write_text(json.dumps(doc, indent=2) + "\n")


def apply_to_flight(cal: dict) -> list:
    """Point the flight code's sign conventions at the measured values. Returns the
    names still unmeasured (worth warning about)."""
    import find_pad
    import tracker
    tracker.PAD_X_SIGN = value(cal, "drone", "pad_x_sign")
    tracker.PAD_Y_SIGN = value(cal, "drone", "pad_y_sign")
    tracker.YAW_CW_POSITIVE = value(cal, "drone", "yaw_cw_positive")
    find_pad.GO_Y_SIGN = value(cal, "drone", "go_y_sign")
    return [k for k in ("pad_x_sign", "pad_y_sign", "yaw_cw_positive", "go_y_sign")
            if not cal["drone"][k]["measured"]]


def unmeasured(cal: dict) -> list:
    return [f"{robot}.{k}" for robot, keys in cal.items() for k, v in keys.items()
            if not v["measured"]]
