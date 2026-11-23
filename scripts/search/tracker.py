"""Where the drone is, in map coordinates (cm). Pure logic, no drone import.

Map frame: origin = centre of the start pad (pad A), +x = the way pad A's rocket points
(forward), +y = left. The drone never rotates during a mission, so every `go dx dy 0`
is already in this frame and positions just add up (dead reckoning).

When a pad with a known map position is under the drone, its reported offset gives the
true position. The gap between that and the dead-reckoned estimate is spread back over
the steps since the previous fix, so the whole flight log (and everything placed from
it: floor readings, the target) gets corrected, not only the current point.

Sign conventions are UNVERIFIED on hardware. Confirm them with `find_pad.py --check`
(slide the drone over a pad by hand) and flip these constants if needed.
"""

import math

# Drone offset from a pad, as reported in the state packet ('x', 'y'), multiplied by
# these to get map-frame cm. Assumes every pad is laid with its rocket pointing +x.
PAD_X_SIGN = 1
PAD_Y_SIGN = 1

# True if the drone's yaw reading grows when it turns clockwise (seen from above).
YAW_CW_POSITIVE = True


def wrap_degrees(a: float) -> float:
    """Wrap an angle to (-180, 180]."""
    a = (a + 180.0) % 360.0 - 180.0
    return 180.0 if a == -180.0 else a


class Tracker:
    def __init__(self, known_pads: dict):
        """known_pads: {pad_id: (x, y)} map positions of pads we can correct against."""
        self.known_pads = {int(k): (float(v[0]), float(v[1])) for k, v in known_pads.items()}
        self.points = [(0.0, 0.0)]   # estimated position after every step
        self.sources = ["start"]     # "move" or "pad <id>" per point
        self._last_fix = 0           # index of the last point fixed by a pad
        self.fixes = []              # (index, pad_id, (err_x, err_y)) for the report

    @property
    def index(self) -> int:
        return len(self.points) - 1

    @property
    def pos(self) -> tuple:
        return self.points[-1]

    def moved(self, dx: float, dy: float, yaw_err: float = 0.0) -> int:
        """Record a commanded move. yaw_err is how far the drone has turned clockwise since
        takeoff (degrees, from heading_error()): `go` moves are in the drone's own frame,
        so a turned drone moves at that angle. Returns the new point's index."""
        if yaw_err:
            a = math.radians(yaw_err)
            dx, dy = dx * math.cos(a) + dy * math.sin(a), -dx * math.sin(a) + dy * math.cos(a)
        x, y = self.points[-1]
        self.points.append((x + dx, y + dy))
        self.sources.append("move")
        return self.index

    @staticmethod
    def pad_offset(raw_x: float, raw_y: float) -> tuple:
        """Drone offset from a pad in map-frame cm, from the raw state values."""
        return PAD_X_SIGN * raw_x, PAD_Y_SIGN * raw_y

    def pad_fix(self, pad_id: int, raw_x: float, raw_y: float):
        """A known pad is under the drone: correct the log. Returns the error (ex, ey) that
        was removed, or None if the pad's position isn't known."""
        if pad_id not in self.known_pads:
            return None
        px, py = self.known_pads[pad_id]
        ox, oy = self.pad_offset(raw_x, raw_y)
        ex, ey = self.points[-1][0] - (px + ox), self.points[-1][1] - (py + oy)
        return self.correct(ex, ey, pad_id)

    def correct(self, ex: float, ey: float, label) -> tuple:
        """Remove a tracking error (ex, ey) found at the current point, spreading it back
        over the points since the last fix. `label` goes in the fix log (pad_fix passes the
        pad id)."""
        start, end = self._last_fix, self.index
        span = end - start
        # Drift grows with distance flown, not with the number of moves (a move may be
        # 20 cm or 1 m), so each point takes its share by distance flown since the last fix.
        run = [0.0]
        for i in range(start + 1, end + 1):
            (ax, ay), (bx, by) = self.points[i - 1], self.points[i]
            run.append(run[-1] + math.hypot(bx - ax, by - ay))
        total = run[-1]
        for i in range(start + 1, end + 1):
            frac = run[i - start] / total if total > 0 else (i - start) / span
            x, y = self.points[i]
            self.points[i] = (x - ex * frac, y - ey * frac)
        if span == 0:  # fixed twice at the same point: just overwrite it
            x, y = self.points[end]
            self.points[end] = (x - ex, y - ey)
        self.sources[end] = f"pad {label}"
        self._last_fix = end
        self.fixes.append((end, label, (ex, ey)))
        return ex, ey

    def place(self, index: int, raw_x: float, raw_y: float) -> tuple:
        """Map position of a pad seen at log point `index` with the given raw offset."""
        ox, oy = self.pad_offset(raw_x, raw_y)
        x, y = self.points[index]
        return x - ox, y - oy


def heading_error(yaw0: float, yaw: float) -> float:
    """Degrees the drone has turned clockwise since takeoff."""
    err = wrap_degrees(yaw - yaw0)
    return err if YAW_CW_POSITIVE else -err


def yaw_correction(yaw0: float, yaw: float, limit: float = 10.0):
    """How to turn back to the takeoff heading: ("cw"|"ccw", degrees) or None if within limit."""
    err = wrap_degrees(yaw - yaw0)
    if abs(err) <= limit:
        return None
    turned_cw = (err > 0) == YAW_CW_POSITIVE
    return ("ccw" if turned_cw else "cw"), int(round(abs(err)))
