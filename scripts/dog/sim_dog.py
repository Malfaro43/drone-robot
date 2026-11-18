"""Simulated XGO Lite behind the tello_xgo_bridge interface: read() / write(walk=, yaw=).

Knows its true position and heading. Built from what's been measured on the real dog
(xgo-robot docs/PROGRESS.md) plus guesses, all adjustable:

  start_delay   legs don't respond for ~0.6 s after a stopped dog gets a command (measured)
  turn rate     ~19 deg/s at turn value 20, ~32 deg/s at 40 (measured); nothing below 5
  walk speed    cm/s per walk value: NOT MEASURED, a guess (0.8 cm/s per unit)
  speed_error   real speed differs from the calibrated one by this fraction
  creep         deg/s the dog veers while walking straight (a guess)
  gyro_sign     +1 if the reported yaw grows turning left; the real sign isn't documented
  gyro_drift    deg/s the reported yaw drifts (0 normally; 7-19 seen in bad conditions)
  watchdog      the Pi stops a moving dog when no command arrives for this long
                (pi_xgo_bridge CommandWatchdog); `tripped` counts how often it did
"""

import math
import random

CM_PER_WALK_UNIT = 0.8


class FakeXgo:
    def __init__(self, start=(0.0, 0.0), heading=0.0, start_delay=0.6, speed_error=0.0,
                 creep=0.0, gyro_sign=-1, gyro_drift=0.0, gyro_noise=0.3, seed=0,
                 fail_after=None, watchdog=1.0):
        self.x, self.y = float(start[0]), float(start[1])
        self.h = float(heading)                      # true heading, deg ccw from +x
        self.start_delay, self.speed_error, self.creep = start_delay, speed_error, creep
        self.gyro_sign, self.gyro_drift, self.gyro_noise = gyro_sign, gyro_drift, gyro_noise
        self.rng = random.Random(seed)
        self.t = 0.0
        self.cmd = (0.0, 0.0)
        self.live_at = 0.0           # when the current command starts moving the legs
        self.fail_after = fail_after  # reads fail after this many seconds (lost link)
        self.writes = []
        self.watchdog, self.last_write, self.tripped = watchdog, 0.0, 0
        self.trace = [(0.0, self.x, self.y, self.h)]

    # ---- the dog's physics -----------------------------------------------------------
    @staticmethod
    def cm_per_s(walk: float) -> float:
        return CM_PER_WALK_UNIT * min(abs(walk), 25)

    @staticmethod
    def deg_per_s(turn: float) -> float:
        v = min(abs(turn), 100)
        if v < 5:
            return 0.0
        return v * 0.95 if v <= 20 else 19 + (v - 20) * 0.65

    def _advance(self, dt: float) -> None:
        walk, turn = self.cmd
        while dt > 1e-9:
            step = min(dt, 0.05)
            if (self.watchdog and self.cmd != (0.0, 0.0)
                    and self.t - self.last_write > self.watchdog):
                self.cmd = (0.0, 0.0)          # the Pi's watchdog stops the dog
                self.tripped += 1
                walk, turn = self.cmd
            if self.t >= self.live_at:
                rate = math.copysign(self.deg_per_s(turn), turn)
                if walk:
                    rate += self.creep
                self.h += rate * step
                v = math.copysign(self.cm_per_s(walk), walk) * (1 + self.speed_error)
                self.x += v * step * math.cos(math.radians(self.h))
                self.y += v * step * math.sin(math.radians(self.h))
            self.t += step
            dt -= step
        self.trace.append((round(self.t, 2), self.x, self.y, self.h))

    # ---- clock (the follower uses these instead of real time) ------------------------
    def clock(self) -> float:
        return self.t

    def sleep(self, s: float) -> None:
        self._advance(s)

    # ---- the bridge interface ---------------------------------------------------------
    def read(self) -> dict:
        self._advance(0.05)  # an HTTP round trip
        if self.fail_after is not None and self.t > self.fail_after:
            return {"ok": False, "error": "Could not reach http://sim/stats"}
        yaw = self.gyro_sign * self.h + self.gyro_drift * self.t + self.rng.gauss(0, self.gyro_noise)
        return {"ok": True, "battery_percent": 80,
                "imu": {"rotation": {"roll": 0.0, "pitch": 0.0, "yaw": yaw}}}

    def write(self, walk: float = 0.0, yaw: float = 0.0) -> dict:
        self._advance(0.05)
        stopped = self.cmd == (0.0, 0.0)
        self.cmd = (float(walk), float(yaw))
        self.last_write = self.t
        if stopped and self.cmd != (0.0, 0.0):
            self.live_at = self.t + self.start_delay
        self.writes.append((round(self.t, 2), self.cmd))
        return {"ok": True}

    @property
    def moving(self) -> bool:
        return self.cmd != (0.0, 0.0)
