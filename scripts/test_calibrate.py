"""Offline tests for calibration.py and calibrate.py (simulated drone and dog, scripted answers):

    .venv/bin/python -m unittest discover -s scripts -p 'test_calibrate.py'
"""

import math
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE / "search"), str(HERE / "dog")]

import calibrate  # noqa: E402
import calibration  # noqa: E402
from sim_dog import FakeXgo  # noqa: E402
from sim_drone import FakeTello  # noqa: E402


class MirroredTello(FakeTello):
    """A drone whose pad x/y readings and heading count the other way."""

    def get_mission_pad_distance_x(self):
        v = super().get_mission_pad_distance_x()
        return v if v == -100 else -v

    def get_mission_pad_distance_y(self):
        v = super().get_mission_pad_distance_y()
        return v if v == -100 else -v

    def get_yaw(self):
        return -super().get_yaw()


def tmp_cal():
    path = Path(tempfile.mkdtemp()) / "calibration.json"
    return calibration.load(path), path


class CalibrationFileTests(unittest.TestCase):
    def test_defaults_then_record_and_reload(self):
        cal, path = tmp_cal()
        self.assertIsNone(cal["dog"]["walk_cm_per_s"]["measured"])
        calibration.record(cal, "dog", "walk_cm_per_s", 6.5, path)
        again = calibration.load(path)
        self.assertEqual(calibration.value(again, "dog", "walk_cm_per_s"), 6.5)
        self.assertTrue(again["dog"]["walk_cm_per_s"]["measured"])
        self.assertIn("drone.pad_x_sign", calibration.unmeasured(again))

    def test_apply_to_flight_sets_conventions(self):
        import mission
        import tracker
        cal, path = tmp_cal()
        calibration.record(cal, "drone", "pad_x_sign", -1, path)
        calibration.record(cal, "drone", "go_y_sign", -1, path)
        try:
            missing = calibration.apply_to_flight(cal)
            self.assertEqual((tracker.PAD_X_SIGN, mission.GO_Y_SIGN), (-1, -1))
            self.assertIn("pad_y_sign", missing)
        finally:
            tracker.PAD_X_SIGN = mission.GO_Y_SIGN = 1


def hand_mover(drone):
    """Scripted person for drone-axes: does what each prompt asks to the fake drone."""
    def ask(prompt):
        p = prompt.lower()
        if "centre" in p and "forward" not in p and "left" not in p and "clockwise" not in p:
            drone.x, drone.y = 0.0, 0.0
        elif "forward" in p:
            drone.x, drone.y = 15.0, 0.0
        elif "left" in p:
            drone.x, drone.y = 0.0, 15.0
        elif "clockwise" in p:
            drone.x, drone.y = 0.0, 0.0
            drone.yaw += 45                 # FakeTello: yaw grows clockwise
        return ""
    return ask


class DroneAxesTests(unittest.TestCase):
    def run_axes(self, drone):
        cal, path = tmp_cal()
        return calibrate.drone_axes(drone, cal, ask=hand_mover(drone), say=lambda *a: None,
                                    sleep=lambda s: None, path=path), path

    def test_standard_drone(self):
        res, path = self.run_axes(FakeTello({1: (0, 0)}))
        self.assertEqual(res, {"pad_x_sign": 1, "pad_y_sign": 1, "yaw_cw_positive": True})
        self.assertTrue(calibration.load(path)["drone"]["pad_x_sign"]["measured"])

    def test_mirrored_drone(self):
        res, _ = self.run_axes(MirroredTello({1: (0, 0)}))
        self.assertEqual(res, {"pad_x_sign": -1, "pad_y_sign": -1, "yaw_cw_positive": False})

    def test_no_pad_in_view(self):
        drone = FakeTello({1: (500, 500)})
        with self.assertRaises(calibrate.CalibrationError):
            self.run_axes(drone)


class DroneHoverTests(unittest.TestCase):
    def hover(self, drone, box_answer=""):
        cal, path = tmp_cal()
        calibration.record(cal, "drone", "pad_y_sign", 1, path)
        answers = iter(["yes", box_answer])
        res = calibrate.drone_hover(drone, cal, ask=lambda p: next(answers),
                                    say=lambda *a: None, sleep=drone.sleep, path=path)
        return res

    def test_hover_measurements_and_lands(self):
        drone = FakeTello({1: (0, 0)}, tof_noise=2.0, baro_noise=5.0, seed=2)
        res = self.hover(drone)
        self.assertEqual(res["go_y_sign"], 1)
        self.assertEqual(res["pad_range_cm"], 30)          # sim detects within 25 cm
        self.assertAlmostEqual(res["tof_noise_cm"], 2.0, delta=1.0)
        self.assertTrue(drone.landed)
        self.assertLess(math.hypot(drone.x, drone.y), 1)    # came back over pad A

    def test_box_reveals_auto_climb(self):
        box = [(50, 90, -30, 30, 30)]
        climbing = self.hover(FakeTello({1: (0, 0)}, boxes=box, auto_climb=True), "yes")
        normal = self.hover(FakeTello({1: (0, 0)}, boxes=box), "yes")
        self.assertIs(climbing["auto_climb"], True)
        self.assertIs(normal["auto_climb"], False)

    def test_refuses_without_clear_space(self):
        cal, path = tmp_cal()
        calibration.record(cal, "drone", "pad_y_sign", 1, path)
        drone = FakeTello({1: (0, 0)})
        self.assertIsNone(calibrate.drone_hover(drone, cal, ask=lambda p: "no",
                                                say=lambda *a: None, path=path))
        self.assertFalse(drone.airborne or drone.landed)

    def test_needs_axes_first(self):
        cal, path = tmp_cal()
        with self.assertRaises(calibrate.CalibrationError):
            calibrate.drone_hover(FakeTello({1: (0, 0)}), cal, ask=lambda p: "yes",
                                  say=lambda *a: None, path=path)


class DogTests(unittest.TestCase):
    def test_measures_speed_turn_and_min_turn(self):
        dog = FakeXgo(speed_error=0.25)               # real dog 25% faster than the sim's base
        cal, path = tmp_cal()
        start = {}

        def ask(prompt):
            if prompt.startswith("Mark"):
                start["xy"] = (dog.x, dog.y)
                return ""
            if prompt.startswith("How many"):
                return f"{math.hypot(dog.x - start['xy'][0], dog.y - start['xy'][1]):.0f}"
            return "yes"
        res = calibrate.dog(dog, cal, ask=ask, say=lambda *a: None, clock=dog.clock,
                            sleep=dog.sleep, path=path)
        self.assertAlmostEqual(res["walk_cm_per_s"], dog.cm_per_s(8) * 1.25, delta=0.4)
        self.assertAlmostEqual(res["turn_deg_per_s"], dog.deg_per_s(16), delta=2)
        self.assertEqual(res["min_turn"], 6.0)          # sim: nothing below 5
        self.assertFalse(dog.moving)
        self.assertEqual(dog.tripped, 0)                # kept the Pi watchdog fed

    def test_refuses_without_ok(self):
        dog = FakeXgo()
        cal, path = tmp_cal()
        self.assertIsNone(calibrate.dog(dog, cal, ask=lambda p: "no", say=lambda *a: None,
                                        clock=dog.clock, sleep=dog.sleep, path=path))
        self.assertFalse(dog.writes)


if __name__ == "__main__":
    unittest.main()
