"""Simulated RoboMaster TT for tests and the replay simulation. No hardware.

Knows the true position, pads on the floor and boxes (x0, x1, y0, y1, height cm; a
negative height is a drop such as a step down). By default it is ideal; each kind of
real-world trouble can be switched on:

  move_error   fraction added to every move (0.05 = 5% overshoot)
  bias         cm added to every move whatever its direction (a draught)
  yaw_drift    degrees the heading creeps per move
  tof_noise    random error (cm, standard deviation) on the downward distance reading
  tof_glitch   chance a distance reading is garbage (out-of-range value)
  pad_miss     chance a pad under the drone isn't detected on one reading
  auto_climb   the drone rises over boxes by itself, so the distance reading stays at
               flight height and only the barometer shows the box
  baro_noise   random error (cm) on the barometer
  drain        battery % used per second of flight; `t` is the simulated clock (s)
"""

import math
import random

FLY_H = 80
BARO_GROUND = 10000   # cm, arbitrary absolute altitude of the floor
TOF_OUT_OF_RANGE = 6553


class FakeTello:
    def __init__(self, pads, boxes=(), move_error=(0.0, 0.0), bias=(0.0, 0.0),
                 start=(0.0, 0.0), pad_range=25, battery=80, *, yaw_drift=0.0,
                 tof_noise=0.0, tof_glitch=0.0, pad_miss=0.0, auto_climb=False,
                 baro_noise=0.0, drain=0.0, seed=0):
        self.pads, self.boxes = pads, list(boxes)
        self.move_error, self.bias, self.yaw_drift = move_error, bias, yaw_drift
        self.x, self.y = start
        self.pad_range, self.battery0, self.drain = pad_range, battery, drain
        self.tof_noise, self.tof_glitch, self.pad_miss = tof_noise, tof_glitch, pad_miss
        self.auto_climb, self.baro_noise = auto_climb, baro_noise
        self.rng = random.Random(seed)
        self.yaw = 0.0
        self.t = 0.0
        self.airborne = self.landed = False
        self.calls = []

    # ---- world -------------------------------------------------------------------
    def _box_under(self):
        for x0, x1, y0, y1, h in self.boxes:
            if x0 <= self.x <= x1 and y0 <= self.y <= y1:
                return h
        return 0

    def altitude(self):
        """True height above the floor's level (cm)."""
        if not self.airborne:
            return 0
        h = self._box_under()
        return FLY_H + h if self.auto_climb and h > 0 else FLY_H

    def _pad(self):
        best = None
        for pid, (px, py) in self.pads.items():
            dist = math.hypot(self.x - px, self.y - py)
            if dist <= self.pad_range and (best is None or dist < best[0]):
                best = (dist, pid)
        return None if best is None else best[1]

    # ---- sensors -----------------------------------------------------------------
    def get_mission_pad_id(self):
        pid = self._pad()
        if pid is None or self.rng.random() < self.pad_miss:
            self._seen = None
            return -1
        self._seen = pid
        return pid

    def _seen_pad(self):
        return getattr(self, "_seen", None) or self._pad()

    def get_mission_pad_distance_x(self):
        pid = self._seen_pad()
        return -100 if pid is None else round(self.x - self.pads[pid][0])

    def get_mission_pad_distance_y(self):
        pid = self._seen_pad()
        return -100 if pid is None else round(self.y - self.pads[pid][1])

    def get_distance_tof(self):
        if self.rng.random() < self.tof_glitch:
            return TOF_OUT_OF_RANGE
        true = self.altitude() - self._box_under()
        return max(1, round(true + self.rng.gauss(0, self.tof_noise)))

    def get_height(self):
        return round(self.altitude())

    def get_barometer(self):
        return BARO_GROUND + self.altitude() + self.rng.gauss(0, self.baro_noise)

    def get_yaw(self):
        return round(self.yaw)

    def get_battery(self):
        return max(0, int(self.battery0 - self.drain * self.t))

    def get_highest_temperature(self):
        return 50

    # ---- commands (each costs simulated time) --------------------------------------
    def sleep(self, s):
        self.t += s

    def enable_mission_pads(self):
        pass

    def set_mission_pad_detection_direction(self, d):
        pass

    def takeoff(self):
        self.airborne = True
        self.t += 6

    def land(self):
        self.airborne, self.landed = False, True
        self.t += 5

    def move_up(self, cm):
        self.t += 2

    def rotate_clockwise(self, deg):      # yaw grows clockwise (tracker.YAW_CW_POSITIVE)
        self.yaw += deg
        self.t += 1.5

    def rotate_counter_clockwise(self, deg):
        self.yaw -= deg
        self.t += 1.5

    def go_xyz_speed(self, x, y, z, speed):
        assert self.airborne, "moved while on the ground"
        assert max(abs(x), abs(y), abs(z)) >= 20, "SDK rejects moves all inside +-20"
        self.calls.append((x, y))
        # `go` is in the drone's own frame: if it has turned clockwise by yaw degrees,
        # its "forward" points that much to the right of the map's +x
        bx, by = x * (1 + self.move_error[0]), y * (1 + self.move_error[1])
        a = math.radians(self.yaw)
        self.x += bx * math.cos(a) + by * math.sin(a) + self.bias[0]
        self.y += -bx * math.sin(a) + by * math.cos(a) + self.bias[1]
        self.yaw += self.yaw_drift
        self.t += math.hypot(x, y) / speed + 1.5

    def go_xyz_speed_mid(self, x, y, z, speed, mid):
        if self._pad() != mid:
            raise RuntimeError("pad not in view")
        self.t += math.hypot(self.x - self.pads[mid][0] - x, self.y - self.pads[mid][1] - y) / speed + 2
        self.x, self.y = self.pads[mid][0] + x, self.pads[mid][1] + y
