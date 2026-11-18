"""Lawnmower search pattern as unit steps on the 20 cm grid. Pure logic.

Positions are grid cells (i, j): x = i * cell, y = j * cell. Every step moves exactly
one cell along one axis, so each `go` is 20 cm (the SDK minimum) and every cell the
drone passes over gets a floor reading.
"""


def snap(lo: float, hi: float, cell: int) -> tuple:
    """Shrink a cm range inwards to whole cells: (first cell, last cell)."""
    import math
    a, b = math.ceil(lo / cell), math.floor(hi / cell)
    if a > b:
        raise ValueError(f"range {lo}..{hi} cm holds no {cell} cm cell")
    return a, b


def _walk(start, goal):
    """Unit steps from cell start to goal: x first, then y."""
    (i, j), (gi, gj) = start, goal
    steps = []
    while i != gi:
        d = 1 if gi > i else -1
        steps.append((d, 0))
        i += d
    while j != gj:
        d = 1 if gj > j else -1
        steps.append((0, d))
        j += d
    return steps


def lawnmower(start, i_range, j_range, lane_cells: int):
    """Unit steps covering the rectangle in lanes along x, `lane_cells` apart in y.

    Starts by walking from `start` to the lane end nearest it. The far lane is always
    included, so the edge of the area is covered even if the width isn't a multiple of
    the lane spacing.
    """
    if lane_cells < 1:
        raise ValueError("lane spacing must be at least one cell")
    (i0, i1), (j0, j1) = i_range, j_range
    lanes = list(range(j0, j1 + 1, lane_cells))
    if lanes[-1] != j1:
        lanes.append(j1)
    si, sj = start
    # sweep lanes starting from the side nearest the start
    if abs(sj - j1) < abs(sj - j0):
        lanes.reverse()
    begin_low = abs(si - i0) <= abs(si - i1)
    steps, pos = [], (si, sj)
    for n, j in enumerate(lanes):
        low_first = begin_low if n % 2 == 0 else not begin_low
        a, b = (i0, i1) if low_first else (i1, i0)
        steps += _walk(pos, (a, j))
        steps += _walk((a, j), (b, j))
        pos = (b, j)
    return steps


def spiral(rings: int):
    """Unit steps of a square spiral outwards from the current cell, `rings` cells out."""
    steps, (di, dj), run = [], (1, 0), 1
    while True:
        for _ in range(2):
            for _ in range(run):
                steps.append((di, dj))
            di, dj = -dj, di  # turn left
        run += 1
        if run > 2 * rings + 1:
            return steps[:(2 * rings + 1) ** 2 - 1]


def in_bounds(cell, i_range, j_range) -> bool:
    return i_range[0] <= cell[0] <= i_range[1] and j_range[0] <= cell[1] <= j_range[1]
