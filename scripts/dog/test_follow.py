"""Offline tests for the dog route follower (simulated dog, no hardware):

    .venv/bin/python -m unittest discover -s scripts/dog
"""

import json
import math
import tempfile
import unittest

from follow_route import FollowError, RouteFollower, load_route, wrap
from sim_dog import FakeXgo

L_ROUTE = [(0, 0), (100, 0), (100, 60)]


def run(route=L_ROUTE, start=(0, 0), calibrated=None, **dog_kw):
    dog = FakeXgo(start=start, **dog_kw)
    f = RouteFollower(dog, calibrated or dog.cm_per_s(8), start=start, clock=dog.clock,
                      sleep=dog.sleep, say=lambda *a: None)
    f.follow(route)
    return dog, f


class FollowTests(unittest.TestCase):
    def test_l_shaped_route(self):
        dog, f = run()
        self.assertLess(math.hypot(dog.x - 100, dog.y - 60), 10)
        self.assertFalse(dog.moving)

    def test_either_gyro_sign(self):
        for sign in (1, -1):
            dog, f = run(gyro_sign=sign)
            self.assertEqual(f.sign, sign)
            self.assertLess(math.hypot(dog.x - 100, dog.y - 60), 10)

    def test_heading_hold_fights_veer(self):
        dog, _ = run(route=[(0, 0), (150, 0)], creep=3.0)   # veers 3 deg/s left
        self.assertLess(abs(dog.y), 10)
        self.assertLess(abs(dog.x - 150), 10)

    def test_wrong_speed_calibration_shows_up(self):
        # real dog 20% faster than calibrated: overshoots by ~20% per leg
        dog, f = run(route=[(0, 0), (100, 0)], speed_error=0.2)
        self.assertAlmostEqual(dog.x, 120, delta=8)
        self.assertAlmostEqual(f.x, 100, delta=3)     # the follower can't tell

    def test_turn_in_both_directions(self):
        dog, _ = run(route=[(0, 0), (0, 60), (0, 0), (0, -60)])
        self.assertLess(math.hypot(dog.x, dog.y + 60), 12)

    def test_lost_link_stops_dog(self):
        dog = FakeXgo(fail_after=6.0)
        f = RouteFollower(dog, dog.cm_per_s(8), clock=dog.clock, sleep=dog.sleep,
                          say=lambda *a: None)
        with self.assertRaises(FollowError):
            f.follow([(0, 0), (300, 0)])
        self.assertFalse(dog.moving)

    def test_ctrl_c_stops_dog(self):
        dog = FakeXgo()
        real = dog.sleep

        def sleep(s):
            if dog.t > 5:
                raise KeyboardInterrupt
            real(s)
        f = RouteFollower(dog, dog.cm_per_s(8), clock=dog.clock, sleep=sleep,
                          say=lambda *a: None)
        with self.assertRaises(KeyboardInterrupt):
            f.follow(L_ROUTE)
        self.assertFalse(dog.moving)

    def test_time_limit_stops_dog(self):
        dog = FakeXgo()
        f = RouteFollower(dog, dog.cm_per_s(8), clock=dog.clock, sleep=dog.sleep,
                          say=lambda *a: None, max_seconds=10)
        with self.assertRaises(FollowError):
            f.follow([(0, 0), (500, 0)])
        self.assertFalse(dog.moving)

    def test_dead_gyro_refuses_to_walk(self):
        dog = FakeXgo()
        dog.deg_per_s = staticmethod(lambda turn: 0.0)   # legs not taking turn commands
        f = RouteFollower(dog, dog.cm_per_s(8), clock=dog.clock, sleep=dog.sleep,
                          say=lambda *a: None)
        with self.assertRaises(FollowError):
            f.follow(L_ROUTE)
        self.assertFalse(dog.moving)
        self.assertLess(math.hypot(dog.x, dog.y), 1)

    def test_gyro_drift_is_measured_and_removed(self):
        dog, f = run(route=[(0, 0), (100, 0), (100, 60)], gyro_drift=2.0)
        self.assertAlmostEqual(f.bias, 2.0, delta=0.3)
        self.assertLess(math.hypot(dog.x - 100, dog.y - 60), 12)

    def test_refuses_badly_drifting_gyro(self):
        dog = FakeXgo(gyro_drift=7.0)
        f = RouteFollower(dog, dog.cm_per_s(8), clock=dog.clock, sleep=dog.sleep,
                          say=lambda *a: None)
        with self.assertRaises(FollowError):
            f.follow(L_ROUTE)
        self.assertFalse(dog.moving)
        self.assertLess(math.hypot(dog.x, dog.y), 1)    # never moved

    def test_never_trips_the_pi_watchdog(self):
        for route in (L_ROUTE, [(0, 0), (0, 60), (0, 0), (0, -60)]):
            dog, _ = run(route=route)
            self.assertEqual(dog.tripped, 0)

    def test_sim_watchdog_stops_a_silent_laptop(self):
        dog = FakeXgo()
        dog.write(walk=8, yaw=0)
        dog.sleep(3.0)
        self.assertFalse(dog.moving)
        self.assertEqual(dog.tripped, 1)
        self.assertLess(dog.x, 0.8 * 8 * 1.2)    # walked ~0.5 s, not 3 s

    def test_wrap(self):
        self.assertEqual(wrap(190), -170)
        self.assertEqual(wrap(-190), 170)


class LoadRouteTests(unittest.TestCase):
    def write(self, msg):
        f = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
        json.dump({"map": msg, "flight_log": []}, f)
        f.close()
        return f.name

    def test_refuses_unchecked_route(self):
        path = self.write({"target": {"x": 1, "y": 2}, "route": [[0, 0], [20, 0]],
                           "route_checked": False})
        with self.assertRaises(FollowError):
            load_route(path)
        self.assertTrue(load_route(path, allow_unchecked=True)["route"])

    def test_refuses_missing_target(self):
        path = self.write({"target": None, "route": [], "route_checked": False})
        with self.assertRaises(FollowError):
            load_route(path, allow_unchecked=True)


if __name__ == "__main__":
    unittest.main()
