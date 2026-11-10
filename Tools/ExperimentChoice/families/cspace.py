"""Brute-force configuration-space tools for composite footprints (EXPLORATORY).

A footprint is a list of PARTS, each an axis-aligned rectangle in the body frame whose origin is
the rotation centre: (cx, cy, lx, ly) = centre and full sizes in metres, x forward, y left.

* ``hits`` / ``clearance``: exact static collision (separating axes, from geometry.py) and exact
  distance between the footprint and axis-aligned boxes.
* ``Lattice``: free/occupied table on an (x, y, theta) lattice and flood-fill reachability,
  optionally restricted to forward-only driving. This is the brute-force check the analytic
  feasibility statements are tested against. Every node is an exact static test; moves between
  adjacent nodes are NOT swept, and a passage needs about one cell / one heading step of slack to
  be seen. So "reached" is strong evidence of a path and "not reached" is weak evidence of none.
* 2-D helpers for the feature code: clearance field, flood fill, 8-connected shortest route.
"""
from __future__ import annotations

import math

import numpy as np

from . import _PARENT  # noqa: F401  (path shim)
import geometry as G


def part_centres(x, y, th, part):
    cx, cy = part[0], part[1]
    c, s = np.cos(th), np.sin(th)
    return x + c * cx - s * cy, y + s * cx + c * cy


def _b1(x, y, th):
    x, y, th = np.broadcast_arrays(np.atleast_1d(np.asarray(x, float)), np.atleast_1d(np.asarray(y, float)),
                                   np.atleast_1d(np.asarray(th, float)))
    return x.ravel(), y.ravel(), th.ravel()


def hits(x, y, th, parts, boxes, tol=1e-9, chunk=20000):
    """True per pose if any part overlaps any box (touching is free). Exact."""
    x, y, th = _b1(x, y, th)
    boxes = np.asarray(boxes, float).reshape(-1, 4)
    out = np.zeros(x.size, bool)
    for k in range(0, x.size, chunk):
        sl = slice(k, k + chunk)
        for part in parts:
            px, py = part_centres(x[sl], y[sl], th[sl], part)
            out[sl] |= G.rect_hits_boxes(px, py, th[sl], part[2], part[3], boxes, 0.0, tol)
    return out


def clearance(x, y, th, parts, boxes, chunk=4000):
    """Distance from the footprint to the nearest box, 0 where they overlap. Exact.

    For disjoint convex polygons the closest pair contains a vertex of one of them, so the
    distance is the smaller of (footprint corners to boxes) and (box corners to footprint parts).
    """
    x, y, th = _b1(x, y, th)
    boxes = np.asarray(boxes, float).reshape(-1, 4)
    bcx = np.stack([boxes[:, 0], boxes[:, 1], boxes[:, 1], boxes[:, 0]], 1)     # (M, 4)
    bcy = np.stack([boxes[:, 2], boxes[:, 2], boxes[:, 3], boxes[:, 3]], 1)
    out = np.full(x.size, np.inf)
    for k in range(0, x.size, chunk):
        sl = slice(k, k + chunk)
        best = np.full(x[sl].size, np.inf)
        for part in parts:
            px, py = part_centres(x[sl], y[sl], th[sl], part)
            cor = G.rect_corners(px, py, th[sl], part[2], part[3])               # (N, 4, 2)
            X, Y = cor[..., 0][..., None], cor[..., 1][..., None]                # (N, 4, 1)
            dx = np.maximum(np.maximum(boxes[:, 0] - X, X - boxes[:, 1]), 0.0)
            dy = np.maximum(np.maximum(boxes[:, 2] - Y, Y - boxes[:, 3]), 0.0)
            d1 = np.hypot(dx, dy).min(axis=(1, 2))
            c, s = np.cos(th[sl])[:, None, None], np.sin(th[sl])[:, None, None]
            rx, ry = bcx[None] - px[:, None, None], bcy[None] - py[:, None, None]
            u, n = rx * c + ry * s, -rx * s + ry * c
            d2 = np.hypot(np.maximum(np.abs(u) - part[2] / 2, 0.0),
                          np.maximum(np.abs(n) - part[3] / 2, 0.0)).min(axis=(1, 2))
            best = np.minimum(best, np.minimum(d1, d2))
        out[sl] = best
    out[hits(x, y, th, parts, boxes)] = 0.0
    return out


def extents(th, parts):
    """(xmin, xmax, ymin, ymax) of the footprint about the rotation centre at heading(s) th."""
    th = np.atleast_1d(np.asarray(th, float))
    lo_x = np.full(th.shape, np.inf); hi_x = -lo_x.copy(); lo_y = lo_x.copy(); hi_y = hi_x.copy()
    for part in parts:
        cor = G.rect_corners(*part_centres(0.0, 0.0, th, part), th, part[2], part[3])
        lo_x = np.minimum(lo_x, cor[..., 0].min(-1)); hi_x = np.maximum(hi_x, cor[..., 0].max(-1))
        lo_y = np.minimum(lo_y, cor[..., 1].min(-1)); hi_y = np.maximum(hi_y, cor[..., 1].max(-1))
    return lo_x, hi_x, lo_y, hi_y


def circumradius(parts):
    """Largest distance from the rotation centre to a footprint corner (in-place pivot disc)."""
    return max(math.hypot(abs(p[0]) + p[2] / 2, abs(p[1]) + p[3] / 2) for p in parts)


def swept_free(xs, ys, ths, parts, boxes):
    """True if every sampled pose of a path is collision free (dense sampling is the caller's job)."""
    return not bool(hits(xs, ys, ths, parts, boxes).any())


# --------------------------------------------------------------------------- flood fill

def _scan(reach, free, axis, periodic=False):
    r, f = np.moveaxis(reach, axis, 0), np.moveaxis(free, axis, 0)       # views
    n = r.shape[0]
    for _ in range(2 if periodic else 1):
        for i in range(1, n):
            r[i] |= r[i - 1] & f[i]
        if periodic:
            r[0] |= r[n - 1] & f[0]
        for i in range(n - 2, -1, -1):
            r[i] |= r[i + 1] & f[i]
        if periodic:
            r[n - 1] |= r[0] & f[n - 1]


def flood(free, seed, periodic_axis=None):
    """Connected component of ``seed`` in ``free`` (axis-neighbour connectivity), by line sweeps."""
    reach = np.zeros_like(free)
    if not free[seed]:
        return reach
    reach[seed] = True
    count = 1
    while True:
        for ax in range(free.ndim):
            _scan(reach, free, ax, periodic=(ax == periodic_axis))
        new = int(reach.sum())
        if new == count:
            return reach
        count = new


class Lattice:
    """(theta, y, x) lattice through the start pose. free[k, iy, ix] is an exact static test."""

    def __init__(self, boxes, parts, start, res=0.05, n_theta=72, bounds=None):
        self.boxes = np.asarray(boxes, float).reshape(-1, 4)
        self.parts = list(parts)
        self.res, self.n_theta = float(res), int(n_theta)
        if bounds is None:
            bounds = (self.boxes[:, 0].min(), self.boxes[:, 1].max(), self.boxes[:, 2].min(), self.boxes[:, 3].max())
        sx, sy, sth = start
        k0, k1 = math.ceil((bounds[0] - sx) / res), math.floor((bounds[1] - sx) / res)
        j0, j1 = math.ceil((bounds[2] - sy) / res), math.floor((bounds[3] - sy) / res)
        self.xs = sx + res * np.arange(k0, k1 + 1)
        self.ys = sy + res * np.arange(j0, j1 + 1)
        self.ths = sth + 2 * math.pi * np.arange(n_theta) / n_theta
        self.start_idx = (0, -j0, -k0)
        X, Y = np.meshgrid(self.xs, self.ys)
        self.free = np.empty((n_theta, self.ys.size, self.xs.size), bool)
        for k, th in enumerate(self.ths):
            self.free[k] = ~hits(X.ravel(), Y.ravel(), th, self.parts, self.boxes).reshape(X.shape)

    def reach(self, forward_only=False, cone=math.radians(22.5)):
        """Nodes connected to the start. forward_only: translation only to the 8-neighbours whose
        direction lies within ``cone`` of the heading (a relaxation of 'never reverses')."""
        if not forward_only:
            return flood(self.free, self.start_idx, periodic_axis=0)
        free = self.free
        reach = np.zeros_like(free)
        if not free[self.start_idx]:
            return reach
        reach[self.start_idx] = True
        moves = []
        for dx, dy in ((1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)):
            ang = math.atan2(dy, dx)
            ok = np.abs(G.wrap(self.ths - ang)) <= cone + 1e-9
            if ok.any():
                moves.append((dx, dy, ok[:, None, None]))
        ny, nx = free.shape[1:]
        count = 1
        while True:
            new = reach.copy()
            new |= np.roll(reach, 1, axis=0) | np.roll(reach, -1, axis=0)
            for dx, dy, ok in moves:
                src = reach & ok
                dst_y = slice(max(dy, 0), ny + min(dy, 0)); src_y = slice(max(-dy, 0), ny + min(-dy, 0))
                dst_x = slice(max(dx, 0), nx + min(dx, 0)); src_x = slice(max(-dx, 0), nx + min(-dx, 0))
                new[:, dst_y, dst_x] |= src[:, src_y, src_x]
            new &= free
            c = int(new.sum())
            reach = new
            if c == count:
                return reach
            count = c

    def near(self, reach, xy, tol):
        """True if a reached node (any heading) lies within ``tol`` of the point."""
        X, Y = np.meshgrid(self.xs, self.ys)
        m = np.hypot(X - xy[0], Y - xy[1]) <= tol
        return bool(reach[:, m].any())

    def headings_reached(self, reach):
        return self.ths[reach.any(axis=(1, 2))]


def reachable(course, parts, res=0.05, n_theta=72, tol=None, forward_only=False):
    """Brute-force verdict for one course: does the lattice connect start to goal?"""
    lat = Lattice(course.boxes(), parts, course.start, res=res, n_theta=n_theta)
    r = lat.reach(forward_only=forward_only)
    return lat.near(r, course.goal, tol if tol is not None else 0.25)


# --------------------------------------------------------------------------- 2-D helpers

def clearance_field(boxes, xs, ys):
    """Distance from each grid point to the nearest box (0 inside a box). Exact per point."""
    X, Y = np.meshgrid(xs, ys)
    out = np.full(X.shape, np.inf)
    for xmin, xmax, ymin, ymax in np.asarray(boxes, float).reshape(-1, 4):
        dx = np.maximum(np.maximum(xmin - X, X - xmax), 0.0)
        dy = np.maximum(np.maximum(ymin - Y, Y - ymax), 0.0)
        np.minimum(out, np.hypot(dx, dy), out=out)
    return out


_AX = ((1, 0), (-1, 0), (0, 1), (0, -1))
_DG = ((1, 1), (1, -1), (-1, 1), (-1, -1))


def _shift_or(dst, src, dy, dx):
    ny, nx = src.shape
    dst[max(dy, 0):ny + min(dy, 0), max(dx, 0):nx + min(dx, 0)] |= \
        src[max(-dy, 0):ny + min(-dy, 0), max(-dx, 0):nx + min(-dx, 0)]


def grid_route(free, s, g):
    """Shortest 8-connected route from cell s to cell g (costs 2 axis / 3 diagonal, Dial's method).
    Returns an (n, 2) int array of (iy, ix) or None."""
    if not (free[s] and free[g]):
        return None
    dist = np.full(free.shape, -1, np.int32)
    dist[s] = 0
    first = np.zeros_like(free)
    first[s] = True
    fronts = {0: first}
    t, idle = 0, 0
    while dist[g] < 0:
        t += 1
        cand = np.zeros_like(free)
        f2, f3 = fronts.get(t - 2), fronts.get(t - 3)
        if f2 is not None:
            for dy, dx in _AX:
                _shift_or(cand, f2, dy, dx)
        if f3 is not None:
            for dy, dx in _DG:
                _shift_or(cand, f3, dy, dx)
        cand &= free
        cand &= dist < 0
        if cand.any():
            dist[cand] = t
            fronts[t] = cand
            idle = 0
        else:
            idle += 1
            if idle > 3:
                return None
        fronts.pop(t - 3, None)
    path = [g]
    cur = g
    ny, nx = free.shape
    while cur != s:
        d = dist[cur]
        for (dy, dx), cost in [(m, 2) for m in _AX] + [(m, 3) for m in _DG]:
            q = (cur[0] - dy, cur[1] - dx)
            if 0 <= q[0] < ny and 0 <= q[1] < nx and dist[q] >= 0 and dist[q] == d - cost:
                cur = q
                break
        else:                                             # cannot happen on a consistent table
            return None
        path.append(cur)
    return np.array(path[::-1], int)


def line_free(free, a, b):
    """All cells under the straight segment between cells a and b are free (sampled at half a cell)."""
    n = int(2 * max(abs(b[0] - a[0]), abs(b[1] - a[1]))) + 1
    iy = np.rint(np.linspace(a[0], b[0], n + 1)).astype(int)
    ix = np.rint(np.linspace(a[1], b[1], n + 1)).astype(int)
    return bool(free[iy, ix].all())


def pull_string(free, path):
    """Greedy line-of-sight shortcutting of a cell path. Returns the kept cells (n, 2)."""
    keep = [0]
    i = 0
    n = len(path)
    while i < n - 1:
        j = i + 1
        while j + 1 < n and line_free(free, path[i], path[j + 1]):
            j += 1
        keep.append(j)
        i = j
    return path[keep]
