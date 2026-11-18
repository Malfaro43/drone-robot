"""Simulated RoboMaster TT for tests and the replay simulation. No hardware.

Knows the true position, pads on the floor, boxes (shorten the downward distance
reading) and per-move errors: `move_error` (fraction of each move, e.g. 0.05 overshoot)
and `bias` (cm added to every move, e.g. a draught).
"""

import math

FLY_H = 80


class FakeTello:
    """Simulated drone: true position, pads on the floor, boxes, and per-move error."""

    def __init__(self, pads, boxes=(), move_error=(0.0, 0.0), bias=(0.0, 0.0),
                 start=(0.0, 0.0), pad_range=25, battery=80):
        self.pads, self.boxes = pads, list(boxes)
        self.move_error = move_error            # fraction added to every x/y move
        self.bias = bias                        # cm added to every move, whatever its direction
        self.x, self.y = start
        self.pad_range, self.battery = pad_range, battery
        self.yaw = 0
        self.airborne = self.landed = False
        self.calls = []

    # pads / sensors
    def _pad(self):
        best = None
        for pid, (px, py) in self.pads.items():
            dist = math.hypot(self.x - px, self.y - py)
            if dist <= self.pad_range and (best is None or dist < best[0]):
                best = (dist, pid)
        return None if best is None else best[1]

    def get_mission_pad_id(self):
        pid = self._pad()
        return -1 if pid is None else pid

    def get_mission_pad_distance_x(self):
        pid = self._pad()
        return -100 if pid is None else round(self.x - self.pads[pid][0])

    def get_mission_pad_distance_y(self):
        pid = self._pad()
        return -100 if pid is None else round(self.y - self.pads[pid][1])

    def get_distance_tof(self):
        for x0, x1, y0, y1, h in self.boxes:
            if x0 <= self.x <= x1 and y0 <= self.y <= y1:
                return FLY_H - h
        return FLY_H

    def get_height(self):
        return FLY_H

    def get_barometer(self):
        return 10000

    def get_yaw(self):
        return self.yaw

    def get_battery(self):
        return self.battery

    def get_highest_temperature(self):
        return 50

    # commands
    def enable_mission_pads(self):
        pass

    def set_mission_pad_detection_direction(self, d):
        pass

    def takeoff(self):
        self.airborne = True

    def land(self):
        self.airborne, self.landed = False, True

    def move_up(self, cm):
        pass

    def rotate_clockwise(self, deg):
        self.yaw -= deg

    def rotate_counter_clockwise(self, deg):
        self.yaw += deg

    def go_xyz_speed(self, x, y, z, speed):
        assert self.airborne, "moved while on the ground"
        assert max(abs(x), abs(y), abs(z)) >= 20, "SDK rejects moves all inside +-20"
        self.calls.append((x, y))
        self.x += x * (1 + self.move_error[0]) + self.bias[0]
        self.y += y * (1 + self.move_error[1]) + self.bias[1]

    def go_xyz_speed_mid(self, x, y, z, speed, mid):
        if self._pad() != mid:
            raise RuntimeError("pad not in view")
        self.x, self.y = self.pads[mid][0] + x, self.pads[mid][1] + y
