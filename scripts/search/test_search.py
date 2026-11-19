"""Offline tests for the search mission. No drone needed:

    .venv/bin/python -m unittest discover -s scripts/search
"""

import io
import math
import unittest
from contextlib import redirect_stderr

import find_pad
from find_pad import CELL, Mission, MissionError
from floormap import BLOCKED, FREE, FloorMap, classify, waypoints
from sweep import lawnmower, snap, spiral
from sim_drone import FakeTello
from tracker import Tracker, wrap_degrees, yaw_correction

def make(drone, **kw):
    args = dict(target=2, x_range=(0, 200), y_range=(-60, 60), dog_width=20, margin=10,
                settle=0, sleep=lambda s: None, say=lambda *a: None, send=lambda m: None)
    args.update(kw)
    return Mission(drone, **args)


class TrackerTests(unittest.TestCase):
    def test_moves_add_up(self):
        t = Tracker({1: (0, 0)})
        t.moved(20, 0)
        t.moved(0, -40)
        self.assertEqual(t.pos, (20, -40))

    def test_pad_fix_spreads_error_back(self):
        t = Tracker({1: (0, 0)})
        for _ in range(4):
            t.moved(20, 0)        # thinks it's at x=80
        err = t.pad_fix(1, 0, 0)  # but it's actually over pad 1 at x=0
        self.assertEqual(err, (80, 0))
        self.assertEqual(t.pos, (0, 0))
        self.assertEqual([p[0] for p in t.points], [0, 0, 0, 0, 0])

    def test_pad_fix_linear_spread(self):
        t = Tracker({1: (0, 0), 3: (100, 0)})
        for _ in range(4):
            t.moved(20, 0)        # thinks x=80, is really at pad 3 (x=100)
        t.pad_fix(3, 0, 0)
        self.assertEqual([p[0] for p in t.points], [0, 25, 50, 75, 100])

    def test_unknown_pad_doesnt_fix(self):
        t = Tracker({1: (0, 0)})
        t.moved(20, 0)
        self.assertIsNone(t.pad_fix(5, 0, 0))
        self.assertEqual(t.pos, (20, 0))

    def test_place_pad_from_offset(self):
        t = Tracker({1: (0, 0)})
        t.moved(100, 40)
        self.assertEqual(t.place(1, 10, -5), (90, 45))  # drone is 10 ahead, 5 right of it

    def test_yaw(self):
        self.assertEqual(wrap_degrees(350), -10)
        self.assertIsNone(yaw_correction(0, 8))
        self.assertEqual(yaw_correction(0, 15), ("ccw", 15))
        self.assertEqual(yaw_correction(170, -170), ("ccw", 20))


class SweepTests(unittest.TestCase):
    def test_snap(self):
        self.assertEqual(snap(-60, 60, 20), (-3, 3))
        self.assertEqual(snap(-50, 50, 20), (-2, 2))

    def test_lawnmower_covers_lanes_with_unit_steps(self):
        steps = lawnmower((0, 0), (0, 5), (-2, 2), 2)
        pos, seen = (0, 0), {(0, 0)}
        for d in steps:
            self.assertEqual(abs(d[0]) + abs(d[1]), 1)
            pos = (pos[0] + d[0], pos[1] + d[1])
            seen.add(pos)
            self.assertTrue(0 <= pos[0] <= 5 and -2 <= pos[1] <= 2)
        for j in (-2, 0, 2):
            for i in range(6):
                self.assertIn((i, j), seen)

    def test_spiral_visits_every_cell_once(self):
        pos, seen = (0, 0), {(0, 0)}
        for d in spiral(2):
            pos = (pos[0] + d[0], pos[1] + d[1])
            self.assertNotIn(pos, seen)
            seen.add(pos)
        self.assertEqual(seen, {(i, j) for i in range(-2, 3) for j in range(-2, 3)})

    def test_far_lane_always_included(self):
        steps = lawnmower((0, 0), (0, 2), (0, 3), 2)  # lanes 0, 2 and the edge 3
        pos, seen = (0, 0), set()
        for d in steps:
            pos = (pos[0] + d[0], pos[1] + d[1])
            seen.add(pos)
        self.assertIn((2, 3), seen)


class FloorMapTests(unittest.TestCase):
    def test_classify(self):
        self.assertEqual(classify(80, 80, 15, 25), FREE)
        self.assertEqual(classify(50, 80, 15, 25), BLOCKED)   # box
        self.assertEqual(classify(120, 80, 15, 25), BLOCKED)  # drop
        self.assertIsNone(classify(6553, 80, 15, 25))           # no reading: leave unknown

    def test_blocked_sticks(self):
        m = FloorMap((0, 3), (0, 3))
        m.mark((1, 1), BLOCKED)
        m.mark((1, 1), FREE)
        self.assertEqual(m.get((1, 1)), BLOCKED)

    def test_route_goes_round_obstacle_with_clearance(self):
        m = FloorMap((0, 10), (-5, 5))
        for i in range(11):
            for j in range(-5, 6):
                m.mark((i, j), FREE)
        for j in range(-5, 3):      # wall across x=5, gap at the left edge
            m.cells[(5, j)] = BLOCKED
        path = m.route((0, 0), (10, 0), radius_cm=25, allow_unknown=False)
        self.assertIsNotNone(path)
        for c in path:
            for j in range(-5, 3):
                self.assertGreater(math.hypot(c[0] - 5, c[1] - j) * 20, 25)

    def test_no_route(self):
        m = FloorMap((0, 4), (0, 0))
        for i in range(5):
            m.mark((i, 0), FREE)
        m.cells[(2, 0)] = BLOCKED
        self.assertIsNone(m.route((0, 0), (4, 0), radius_cm=0))

    def test_unknown_only_when_allowed(self):
        m = FloorMap((0, 2), (0, 0))
        m.mark((0, 0), FREE)
        self.assertIsNone(m.route((0, 0), (2, 0), 0, allow_unknown=False))
        self.assertEqual(len(m.route((0, 0), (2, 0), 0, allow_unknown=True)), 3)

    def test_unchecked_beside(self):
        m = FloorMap((0, 4), (-2, 2))
        path = [(i, 0) for i in range(5)]
        for c in path:
            m.mark(c, FREE)
        m.mark((2, 1), FREE)
        beside = m.unchecked_beside(path, radius_cm=25)
        self.assertIn((2, -1), beside)
        self.assertNotIn((2, 1), beside)   # read as free
        self.assertNotIn((2, 2), beside)   # 40 cm away: outside the clearance
        self.assertNotIn((2, 0), beside)   # on the route

    def test_waypoints_keep_turns(self):
        cells = [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)]
        self.assertEqual(waypoints(cells, 20), [[0, 0], [40, 0], [40, 40]])


class MissionTests(unittest.TestCase):
    def test_finds_target_and_flies_home(self):
        drone = FakeTello({1: (0, 0), 2: (160, -40)})
        msg = make(drone).run()
        self.assertTrue(drone.landed)
        self.assertAlmostEqual(msg["target"]["x"], 160, delta=1)
        self.assertAlmostEqual(msg["target"]["y"], -40, delta=1)
        self.assertTrue(msg["route_checked"])
        self.assertEqual(msg["route"][0], [0, 0])
        self.assertLess(math.hypot(drone.x, drone.y), 1)  # back over pad A

    def test_route_avoids_box(self):
        # box across the middle, gap on the left (+y) side
        box = (70, 110, -70, 20, 30)
        drone = FakeTello({1: (0, 0), 2: (180, 0)}, boxes=[box])
        msg = make(drone, lane_cm=20, y_range=(-60, 120)).run()  # 100 cm gap on the left
        self.assertTrue(msg["route_checked"])
        self.assertEqual(msg["unchecked_beside_route"], [])
        rows = msg["grid"]["rows"]
        self.assertIn(BLOCKED, "".join(rows))
        # the dog's body (10 cm each side of the route) never touches the box
        pts = msg["route"]
        for (ax, ay), (bx, by) in zip(pts, pts[1:]):
            n = int(max(abs(bx - ax), abs(by - ay)) / CELL)
            for k in range(n + 1):
                x, y = ax + (bx - ax) * k / max(n, 1), ay + (by - ay) * k / max(n, 1)
                gap = math.hypot(max(70 - x, 0, x - 110), max(-70 - y, 0, y - 20))
                self.assertGreater(gap, 10, f"dog would touch the box at {x},{y}")

    def test_gap_too_tight_is_not_called_safe(self):
        # 40 cm gap between the box and the area edge = dog (20) + margin (10) each side:
        # no room to spare, so the route must not be reported as checked
        box = (70, 110, -70, 20, 30)
        drone = FakeTello({1: (0, 0), 2: (180, 0)}, boxes=[box])
        msg = make(drone, lane_cm=20).run()
        self.assertFalse(msg["route_checked"])

    def test_steady_drift_measured_and_corrected(self):
        # the drone slides 1 cm right on every move (e.g. a draught)
        drone = FakeTello({1: (0, 0), 2: (160, 40)}, bias=(0.0, -1.0))
        msg = make(drone).run()
        dx, dy = msg["drift_at_home_cm"]
        self.assertGreater(abs(dy), 10)                   # pad A shows the drift...
        self.assertLess(abs(msg["target"]["y"] - 40), 6)  # ...and the target is corrected
        self.assertLess(math.hypot(drone.x, drone.y), 1)  # centred over pad A before landing

    def test_checkpoint_pad_keeps_target_accurate(self):
        # 10% overshoot in x on every move; a known pad at x=100 corrects it mid-flight
        pads = {1: (0, 0), 3: (100, 0), 2: (180, 40)}
        drift_only = FakeTello(dict(pads), move_error=(0.10, 0.0))
        with_pad = FakeTello(dict(pads), move_error=(0.10, 0.0))
        a = make(drift_only, lane_cm=20).run()
        b = make(with_pad, lane_cm=20, pads={3: (100, 0)}).run()
        err_a = math.hypot(a["target"]["x"] - 180, a["target"]["y"] - 40)
        err_b = math.hypot(b["target"]["x"] - 180, b["target"]["y"] - 40)
        # Overshoot cancels on the way home, so pad A alone can't see it (err_a ~16 cm);
        # the checkpoint leaves only the 10% of the last 80 cm (~8 cm).
        self.assertGreater(err_a, 12)
        self.assertLess(err_b, 10)
        self.assertTrue(with_pad.landed)
        self.assertLess(math.hypot(with_pad.x, with_pad.y), 1)

    def test_heading_drift_is_corrected(self):
        drone = FakeTello({1: (0, 0), 2: (160, -40)}, yaw_drift=3.0)
        msg = make(drone).run()
        self.assertLessEqual(abs(drone.yaw), 12)            # never wanders past the limit
        self.assertLess(math.hypot(msg["target"]["x"] - 160, msg["target"]["y"] + 40), 15)
        self.assertTrue(drone.landed)

    def test_noisy_sensors_and_missed_pads(self):
        drone = FakeTello({1: (0, 0), 2: (160, -40)}, tof_noise=3, tof_glitch=0.05,
                          pad_miss=0.2, seed=3)
        msg = make(drone).run()
        self.assertIsNotNone(msg["target"])
        self.assertNotIn(BLOCKED, "".join(msg["grid"]["rows"]))  # open floor: no false boxes
        self.assertTrue(drone.landed)

    def test_auto_climb_hides_box_from_distance_sensor(self):
        box = (70, 110, -70, 20, 30)
        kw = dict(boxes=[box], auto_climb=True, baro_noise=3, seed=1)
        tof_only = make(FakeTello({1: (0, 0), 2: (180, 0)}, **kw), lane_cm=20).run()
        with_baro = make(FakeTello({1: (0, 0), 2: (180, 0)}, **kw), lane_cm=20,
                         obstacle_signal="tof+baro").run()
        self.assertNotIn(BLOCKED, "".join(tof_only["grid"]["rows"]))   # box missed
        self.assertIn(BLOCKED, "".join(with_baro["grid"]["rows"]))     # box found

    def test_low_battery_mid_search_flies_home(self):
        # drops below 20% partway through the search: stop searching, land on pad A
        drone = FakeTello({1: (0, 0), 2: (180, 40)}, battery=28, drain=0.1)
        make(drone, lane_cm=20, sleep=drone.sleep).run()
        self.assertTrue(drone.landed)
        self.assertLess(math.hypot(drone.x, drone.y), 1)
        self.assertGreaterEqual(drone.get_battery(), 10)

    def test_target_missing(self):
        drone = FakeTello({1: (0, 0)})
        msg = make(drone).run()
        self.assertIsNone(msg["target"])
        self.assertTrue(drone.landed)

    def test_critical_battery_lands_on_the_spot(self):
        drone = FakeTello({1: (0, 0), 2: (160, -40)}, battery=9)
        with self.assertRaises(MissionError):
            make(drone).run()
        self.assertTrue(drone.landed)

    def test_lands_on_ctrl_c(self):
        drone = FakeTello({1: (0, 0), 2: (160, -40)})
        real = drone.go_xyz_speed

        def boom(*a):
            if len(drone.calls) >= 3:
                raise KeyboardInterrupt
            real(*a)
        drone.go_xyz_speed = boom
        with self.assertRaises(KeyboardInterrupt):
            make(drone).run()
        self.assertTrue(drone.landed)

    def test_refuses_takeoff_off_pad(self):
        drone = FakeTello({1: (0, 0), 2: (160, -40)}, start=(80, 0))
        with self.assertRaises(MissionError):
            make(drone).run()
        self.assertTrue(drone.landed)

    def test_stays_in_area(self):
        drone = FakeTello({1: (0, 0), 2: (160, -40)})
        m = make(drone)
        m.run()
        for x, y in m.tracker.points:
            self.assertTrue(-20 <= x <= 220 and -80 <= y <= 80, (x, y))

    def test_area_must_include_pad_a(self):
        with self.assertRaises(ValueError):
            make(FakeTello({1: (0, 0)}), x_range=(40, 200))

    def test_cli_requires_flight_args(self):
        with self.assertRaises(SystemExit), redirect_stderr(io.StringIO()):
            find_pad.main(["--target", "2"])

    def test_cli_rejects_negative_distance(self):
        with self.assertRaises(SystemExit), redirect_stderr(io.StringIO()):
            find_pad.main(["--target", "2", "--forward", "200", "--left", "60",
                           "--right", "-60", "--dog-width", "30"])


class StressTests(unittest.TestCase):
    """150 random rooms with realistic sensors, scored against the real layout
    (stress.py runs more and prints the details)."""

    @classmethod
    def setUpClass(cls):
        import stress
        cls.s = stress.summarise([stress.evaluate(k) for k in range(1, 151)])

    def test_no_checked_route_touches_an_obstacle(self):
        self.assertEqual([r["seed"] for r in self.s["unsafe"]], [])

    def test_rarely_lands_away_from_pad_a(self):
        self.assertLessEqual(self.s["landed_early"], 0.03 * self.s["rooms"])

    def test_found_targets_are_accurate(self):
        self.assertLess(self.s["target_err_median"], 8)


if __name__ == "__main__":
    unittest.main()
