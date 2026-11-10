"""A small configuration-space lattice planner for rigid units (single modules or coupled assemblies).

State: (grid cell, heading in {0, 90, 180, 270} deg). Moves: forward / reverse along the heading, strafe sideways
(only for units with a lateral capability), rotate by 90 deg in place where the unit's circumscribed disk is free.
The free space of each heading is the exact Minkowski sum of the axis-aligned module rectangles with the obstacles'
axis-aligned bounding boxes (a yawed obstacle is replaced by its AABB: conservative). Costs are seconds from the
unit's speed limits. Cost-to-go comes from vectorised value iteration (numpy), the path from greedy descent.

This is the stand-in planner for the fastsim fleet domain. It is not CRANE's lattice planner
(Crane.Fleet.Autonomy LatticePlanner) and plans Manhattan paths only.
"""
import math

import numpy as np

HEAD = [(1, 0), (0, 1), (-1, 0), (0, -1)]   # (di, dj) per heading index: +x, +y, -x, -y
INF = np.inf


def rot_rect(rect, h):
    """(ox, oy, hx, hy) of a module rectangle in the unit frame, after rotating the unit by h * 90 deg."""
    ox, oy, hx, hy = rect
    for _ in range(h % 4):
        ox, oy, hx, hy = -oy, ox, hy, hx
    return ox, oy, hx, hy


class Grid:
    def __init__(self, size_xy, boxes, res=0.05, align=None):
        """boxes: [(x0, y0, x1, y1)] AABBs of everything that blocks driving. ``align``: a point (the goal) that
        falls exactly on a cell centre (the lattice is shifted by less than one cell)."""
        self.res = float(res)
        self.X, self.Y = float(size_xy[0]), float(size_xy[1])
        ox = oy = 0.0
        if align is not None:
            ox = (align[0] - 0.5 * res) % res
            oy = (align[1] - 0.5 * res) % res
        self.ox, self.oy = ox, oy
        self.ni, self.nj = int((self.X - ox) / res), int((self.Y - oy) / res)
        self.xs = ox + (np.arange(self.ni) + 0.5) * res
        self.ys = oy + (np.arange(self.nj) + 0.5) * res
        self.boxes = list(boxes)

    def cell(self, x, y):
        return (min(self.ni - 1, max(0, int((x - self.ox) / self.res))),
                min(self.nj - 1, max(0, int((y - self.oy) / self.res))))

    def centre(self, i, j):
        return (float(self.xs[i]), float(self.ys[j]))

    def _fill(self, occ, x0, y0, x1, y1):
        i0, i1 = np.searchsorted(self.xs, x0, "left"), np.searchsorted(self.xs, x1, "right")
        j0, j1 = np.searchsorted(self.ys, y0, "left"), np.searchsorted(self.ys, y1, "right")
        if i1 > i0 and j1 > j0:
            occ[i0:i1, j0:j1] = True

    def free(self, rects, h, margin, extra=()):
        """Cells where the unit (module rects, unit frame) at heading h clears every box by `margin` and stays in the arena."""
        occ = np.zeros((self.ni, self.nj), bool)
        for rect in rects:
            ox, oy, hx, hy = rot_rect(rect, h)
            # stay inside the arena
            self._fill(occ, -1e9, -1e9, hx - ox + margin - 1e-9, 1e9)
            self._fill(occ, self.X - hx - ox - margin + 1e-9, -1e9, 1e9, 1e9)
            self._fill(occ, -1e9, -1e9, 1e9, hy - oy + margin - 1e-9)
            self._fill(occ, -1e9, self.Y - hy - oy - margin + 1e-9, 1e9, 1e9)
            for (x0, y0, x1, y1) in list(self.boxes) + list(extra):
                self._fill(occ, x0 - ox - hx - margin, y0 - oy - hy - margin, x1 - ox + hx + margin, y1 - oy + hy + margin)
        return ~occ

    def disk_free(self, radius, extra=()):
        occ = np.zeros((self.ni, self.nj), bool)
        X, Y = np.meshgrid(self.xs, self.ys, indexing="ij")
        occ |= (X < radius) | (X > self.X - radius) | (Y < radius) | (Y > self.Y - radius)
        for (x0, y0, x1, y1) in list(self.boxes) + list(extra):
            dx = np.maximum(np.maximum(x0 - X, X - x1), 0)
            dy = np.maximum(np.maximum(y0 - Y, Y - y1), 0)
            occ |= dx * dx + dy * dy < radius * radius
        return ~occ


def _shift(a, di, dj, fill=INF):
    """b[i, j] = a[i + di, j + dj] (fill outside)."""
    b = np.full_like(a, fill)
    ni, nj = a.shape

    def sl(d, n):  # (target slice of b, source slice of a)
        return (slice(0, n - d), slice(d, n)) if d >= 0 else (slice(-d, n), slice(0, n + d))
    (bi, ai), (bj, aj) = sl(di, ni), sl(dj, nj)
    b[bi, bj] = a[ai, aj]
    return b


class Unitshape:
    def __init__(self, rects, circumradius, lateral, vmax, margin=0.03):
        self.rects, self.R, self.lateral, self.vmax, self.margin = list(rects), float(circumradius), bool(lateral), vmax, margin


def plan(grid, shape, start, goal, goal_yaw=None, extra=(), max_iter=4000, goal_tol_cells=None):
    """The exact goal cell first; a free cell within one cell of it when the exact one is blocked."""
    if goal_tol_cells is None:
        segs, info = _plan(grid, shape, start, goal, goal_yaw, extra, max_iter, 0)
        if segs is None and info == "goal_blocked":
            return _plan(grid, shape, start, goal, goal_yaw, extra, max_iter, 1)
        return segs, info
    return _plan(grid, shape, start, goal, goal_yaw, extra, max_iter, goal_tol_cells)


def _plan(grid, shape, start, goal, goal_yaw, extra, max_iter, goal_tol_cells):
    """A route from start (x, y, yaw) to goal (x, y): a list of segments {mode, x, y, yaw} with mode in drive |
    reverse | strafe | rotate, plus its cost (s). Returns (None, reason) when no lattice path exists."""
    res = grid.res
    free = [grid.free(shape.rects, h, shape.margin, extra) for h in range(4)]
    rot = grid.disk_free(shape.R + shape.margin, extra)
    vf = max(shape.vmax.get("fwd", 0.0), 1e-6)
    vb = max(shape.vmax.get("back", 0.0), 1e-6)
    vl = max(min(shape.vmax.get("left", 0.0), shape.vmax.get("right", 0.0)), 1e-6)
    wz = max(shape.vmax.get("ccw", 0.0), 1e-6)
    if max(shape.vmax.get("fwd", 0), shape.vmax.get("back", 0)) <= 1e-6:
        return None, "immobile"
    cf, cb, cs = res / vf, 1.5 * res / vb, 1.15 * res / vl
    cr = (math.pi / 2) / wz + 0.5
    gi, gj = grid.cell(goal[0], goal[1])
    D = np.full((4, grid.ni, grid.nj), INF)
    heads = range(4) if goal_yaw is None else [int(round(goal_yaw / (math.pi / 2))) % 4]
    for h in heads:
        sl = (h, slice(max(0, gi - goal_tol_cells), gi + goal_tol_cells + 1), slice(max(0, gj - goal_tol_cells), gj + goal_tol_cells + 1))
        D[sl] = np.where(free[h][sl[1], sl[2]], 0.0, INF)
    if not np.isfinite(D).any():
        return None, "goal_blocked"
    for _ in range(max_iter):
        old = D.copy()
        for h in range(4):
            di, dj = HEAD[h]
            cand = [cf + _shift(D[h], di, dj), cb + _shift(D[h], -di, -dj)]
            if shape.lateral:
                li, lj = HEAD[(h + 1) % 4]
                cand += [cs + _shift(D[h], li, lj), cs + _shift(D[h], -li, -lj)]
            best = np.minimum.reduce(cand)
            best = np.where(free[h], best, INF)
            rotc = np.minimum.reduce([cr + D[(h + 1) % 4], cr + D[(h + 3) % 4], 2 * cr + D[(h + 2) % 4]])
            rotc = np.where(rot & free[h], rotc, INF)
            D[h] = np.minimum(D[h], np.minimum(best, rotc))
        if np.array_equal(old, D):
            break
    si, sj = grid.cell(start[0], start[1])
    sh = int(round(start[2] / (math.pi / 2))) % 4
    # the start cell may be (marginally) inside the inflated space: accept the best free neighbour
    if not np.isfinite(D[sh, si, sj]):
        best = None
        for h in ([sh] + [k for k in range(4) if k != sh]):
            for a in range(-2, 3):
                for b in range(-2, 3):
                    i, j = si + a, sj + b
                    if 0 <= i < grid.ni and 0 <= j < grid.nj and np.isfinite(D[h, i, j]):
                        c = D[h, i, j] + (0 if h == sh else cr) + 0.01 * (abs(a) + abs(b))
                        if best is None or c < best[0]:
                            best = (c, h, i, j)
        if best is None:
            return None, "no_path"
        _, sh, si, sj = best
    cost = float(D[sh, si, sj])
    moves = []
    h, i, j = sh, si, sj
    for _ in range(grid.ni * grid.nj):
        if D[h, i, j] <= 0.0:
            break
        opts = []
        di, dj = HEAD[h]
        for mode, (a, b), c in (("drive", (di, dj), cf), ("reverse", (-di, -dj), cb)):
            if 0 <= i + a < grid.ni and 0 <= j + b < grid.nj:
                opts.append((c + D[h, i + a, j + b], mode, h, i + a, j + b))
        if shape.lateral:
            li, lj = HEAD[(h + 1) % 4]
            for a, b in ((li, lj), (-li, -lj)):
                if 0 <= i + a < grid.ni and 0 <= j + b < grid.nj:
                    opts.append((cs + D[h, i + a, j + b], "strafe", h, i + a, j + b))
        if rot[i, j]:
            for dh, c in ((1, cr), (3, cr), (2, 2 * cr)):
                opts.append((c + D[(h + dh) % 4, i, j], "rotate", (h + dh) % 4, i, j))
        cmin = min(o[0] for o in opts)
        if not math.isfinite(cmin) or cmin > D[h, i, j] + 1e-6:
            return None, "descent_failed"
        last = moves[-1] if moves else None
        tied = [o for o in opts if o[0] <= cmin + 1e-6]
        # tie-break: keep the previous move (straight runs, no staircases)
        same = [o for o in tied if last and o[1] == last[0] and (o[3] - i, o[4] - j) == last[4]]
        c, mode, h2, i2, j2 = (same or tied)[0]
        moves.append((mode, h2, i2, j2, (i2 - i, j2 - j)))
        h, i, j = h2, i2, j2
    # compress runs of the same move into segments
    segs = []
    for mode, h2, i2, j2, _ in moves:
        x, y = grid.centre(i2, j2)
        yaw = h2 * math.pi / 2
        if segs and segs[-1]["mode"] == mode and mode != "rotate":
            segs[-1].update(x=x, y=y)
        else:
            segs.append({"mode": mode, "x": x, "y": y, "yaw": _wrap(yaw)})
    for s in segs:
        if s["mode"] != "rotate":
            s["yaw"] = _wrap(s["yaw"])
    gx, gy = goal[0], goal[1]
    last_yaw = segs[-1]["yaw"] if segs else _wrap(sh * math.pi / 2)
    segs.append({"mode": "settle", "x": gx, "y": gy, "yaw": last_yaw if goal_yaw is None else _wrap(goal_yaw)})
    return segs, cost


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))
