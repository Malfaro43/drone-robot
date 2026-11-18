"""Floor grid (free / blocked / unknown) and the dog's route through it. Pure logic.

A reading from the drone's downward distance sensor says what's under one spot:
  - about the flight-height baseline -> open floor (free);
  - much shorter -> something under the drone (box, chair, table) -> blocked;
  - much longer -> a drop (step, stairs) -> blocked;
  - no valid reading -> nothing is marked (the square stays unknown).
Blocked wins: one blocked reading in a cell keeps it blocked, so callers should pass a
filtered reading (find_pad takes the median of a few).

Routes avoid every square within the dog's half-width plus a safety gap of a blocked
square. While flying, dog routes may cross unknown squares (at a cost) because the drone
checks each square as it flies them. To get back onto a route, the drone alone may also
cross safety gaps (cross_clearance). The route sent to the dog uses free squares outside
every safety gap only, and "strict" routes also keep the dog's clearance away from unknown
squares and from the edge of the search area: everything the dog's body passes over has
been read by the drone.
"""

import heapq
import math

FREE, BLOCKED, UNKNOWN = ".", "#", "?"
TOF_INVALID = 600  # cm; readings at or above this are "no reading"
UNKNOWN_COST = 3.0
CLEARANCE_COST = 50.0  # in-flight only: crossing a square too close to an obstacle for the dog


def classify(tof: float, baseline: float, obstacle_cm: float, drop_cm: float):
    """FREE, BLOCKED, or None when there is no valid reading."""
    if tof is None or tof <= 0 or tof >= TOF_INVALID:
        return None
    if tof < baseline - obstacle_cm or tof > baseline + drop_cm:
        return BLOCKED
    return FREE


class FloorMap:
    def __init__(self, i_range, j_range, cell: int = 20):
        self.i_range, self.j_range, self.cell = i_range, j_range, cell
        self.cells = {}

    def cell_of(self, x: float, y: float) -> tuple:
        return round(x / self.cell), round(y / self.cell)

    def center(self, c) -> tuple:
        return c[0] * self.cell, c[1] * self.cell

    def inside(self, c) -> bool:
        return (self.i_range[0] <= c[0] <= self.i_range[1]
                and self.j_range[0] <= c[1] <= self.j_range[1])

    def mark(self, c, state: str) -> None:
        if not self.inside(c):
            return
        if self.cells.get(c) == BLOCKED:
            return
        self.cells[c] = state

    def get(self, c) -> str:
        return self.cells.get(c, UNKNOWN)

    def no_go(self, radius_cm: float, strict: bool = False, edges: bool = False) -> set:
        """Cells too close to a blocked cell for the dog. With edges, also too close to the
        outside of the search area (which the drone can never check). With strict, also
        too close to an unknown cell."""
        r = int(math.ceil(radius_cm / self.cell))
        sources = [c for c, s in self.cells.items() if s == BLOCKED]
        if strict or edges:
            (i0, i1), (j0, j1) = self.i_range, self.j_range
            sources += [(i, j) for i in range(i0 - r, i1 + r + 1) for j in range(j0 - r, j1 + r + 1)
                        if not self.inside((i, j)) or (strict and self.get((i, j)) == UNKNOWN)]
        out = set()
        for bi, bj in sources:
            for di in range(-r, r + 1):
                for dj in range(-r, r + 1):
                    if math.hypot(di, dj) * self.cell <= radius_cm:
                        out.add((bi + di, bj + dj))
        return out

    def route(self, start, goal, radius_cm: float, allow_unknown: bool = True,
              cross_clearance: bool = False, strict: bool = False, edges: bool = False):
        """Shortest 4-connected route of cells from start to goal, or None.
        Start and goal are always allowed (the drone may be over a blocked cell).
        cross_clearance lets the drone (never the dog) pass through safety gaps.
        strict is for the dog: free cells whose whole clearance has been read.
        edges keeps the clearance inside the search area (for planning dog routes)."""
        if strict:
            allow_unknown = cross_clearance = False
        blocked = self.no_go(radius_cm, strict=strict, edges=edges)
        if strict or edges:
            # Around the start (where the dog already stands, beside pad A at the edge of
            # the area) and the goal, only real obstacles count, not unread squares.
            near_ends = {c for c in blocked
                         if min(math.dist(c, start), math.dist(c, goal)) * self.cell <= radius_cm}
            blocked -= near_ends - self.no_go(radius_cm)

        def ok(c):
            if c in (start, goal):
                return True
            if not self.inside(c) or self.get(c) == BLOCKED:
                return False
            if c in blocked:
                return cross_clearance
            return allow_unknown or self.get(c) == FREE

        def cost(c):
            if c in blocked:
                return CLEARANCE_COST
            return UNKNOWN_COST if self.get(c) == UNKNOWN else 1.0

        def h(c):
            return abs(c[0] - goal[0]) + abs(c[1] - goal[1])

        frontier = [(h(start), 0.0, start)]
        came, best = {start: None}, {start: 0.0}
        while frontier:
            _, g, c = heapq.heappop(frontier)
            if c == goal:
                path = []
                while c is not None:
                    path.append(c)
                    c = came[c]
                return path[::-1]
            if g > best[c]:
                continue
            for d in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                n = (c[0] + d[0], c[1] + d[1])
                if not ok(n):
                    continue
                ng = g + cost(n)
                if ng < best.get(n, math.inf):
                    best[n], came[n] = ng, c
                    heapq.heappush(frontier, (ng + h(n), ng, n))
        return None

    def unchecked_beside(self, path, radius_cm: float) -> list:
        """Unknown cells within the dog's clearance of a path: places the dog's body would
        pass that the drone never read (it only reads the one spot under itself)."""
        r = int(math.ceil(radius_cm / self.cell))
        out = set()
        for ci, cj in path:
            for di in range(-r, r + 1):
                for dj in range(-r, r + 1):
                    c = (ci + di, cj + dj)
                    if (math.hypot(di, dj) * self.cell <= radius_cm and self.inside(c)
                            and self.get(c) == UNKNOWN):
                        out.add(c)
        return sorted(out)

    def rows(self) -> list:
        """rows[a][b] is the cell at i = i_range[0] + a, j = j_range[0] + b."""
        return ["".join(self.get((i, j)) for j in range(self.j_range[0], self.j_range[1] + 1))
                for i in range(self.i_range[0], self.i_range[1] + 1)]


def waypoints(cells, cell_cm: int) -> list:
    """Cell path -> cm points, keeping only the start, turns and the end."""
    if not cells:
        return []
    keep = [cells[0]]
    for a, b, c in zip(cells, cells[1:], cells[2:]):
        if (b[0] - a[0], b[1] - a[1]) != (c[0] - b[0], c[1] - b[1]):
            keep.append(b)
    if len(cells) > 1:
        keep.append(cells[-1])
    return [[p[0] * cell_cm, p[1] * cell_cm] for p in keep]
