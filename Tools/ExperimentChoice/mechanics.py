"""Mechanics of a rectangular footprint in an approach bay and a doorway (EXPLORATORY).

Formulas of the plan's section 2, plus the independent numerical checks used to test them:

* closed forms: h(theta), rho, alpha_max(d), four-flow shift delta(alpha), its maximiser,
  cycle counts, small-slack forms, the exact per-stretch bound (new here), door heading
  tolerances, door visibility;
* brute force: footprint-corner feasibility in a strip, swept four-flow poses;
* numerical planners that minimise direction reversals for a sideways shift e:
  ``plan_min_reversals`` (label-setting dynamic programme on an (x, theta) lattice, y carried
  as a continuous max-label; uses only brute-force corner feasibility) and
  ``lattice_bfs_min_reversals`` (a plain reachability search on a true (x, y, theta) lattice,
  used to check the dynamic programme's dominance argument);
* ``features(course, robot)`` and ``certificate(course, robot)``.

Counting convention (it matters; the plan mixed two of them):
  a STRETCH is a maximal piece of motion with one sign of longitudinal velocity; in-place
  rotations do not end a stretch. REVERSALS = stretches - 1. One four-flow CYCLE (rotate
  +alpha, drive, rotate -alpha, reverse) is two stretches; n cycles need 2n - 1 stretches
  (the last reverse is not needed) and 2n - 2 reversals.

Frame and footprint conventions: see geometry.py. numpy only.
"""
from __future__ import annotations

import math

import numpy as np

try:
    from . import geometry as G
except ImportError:  # flat import (tests, CLI)
    import os
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import geometry as G

INF = float("inf")


# --------------------------------------------------------------------------- closed forms

def h(theta, L, W):
    """Extent of the footprint along the bay depth at heading theta (support function; exact)."""
    return L * np.abs(np.cos(theta)) + W * np.abs(np.sin(theta))


def rho(L, W):
    """Half diagonal. 2*rho is the largest value of h, reached at theta = atan(W/L)."""
    return 0.5 * math.hypot(L, W)


def regime(d, L, W):
    """'blocked' (d < L), 'tight' (L <= d < 2 rho) or 'roomy' (d >= 2 rho)."""
    if d < L:
        return "blocked"
    return "roomy" if d >= 2 * rho(L, W) else "tight"


def alpha_max(d, L, W):
    """Largest |heading| any motion can reach in a strip of depth d, starting from heading 0.

    For L <= d < 2 rho: atan(W/L) - arccos(d / 2 rho) (exact). The confinement is on the
    heading itself (h(theta) <= d must hold wherever the robot is), so it applies to every
    maneuver, not only in-place rotation, and does not depend on the axle offset.
    Returns pi/2 for d >= 2 rho (unconfined) and nan for d < L.
    """
    if d < L:
        return float("nan")
    r2 = 2 * rho(L, W)
    if d >= r2:
        return math.pi / 2
    return math.atan2(W, L) - math.acos(d / r2)


def four_flow_delta(alpha, d, L, W):
    """Net sideways shift of one four-flow cycle at angle alpha: (d - h(alpha)) tan(alpha), exact.

    The cycle: at the rear-most position that admits heading alpha, rotate +alpha, drive
    s = (d - h(alpha)) / cos(alpha), rotate -alpha, reverse s cos(alpha). Independent of the
    axle offset (the travel range of the rotation centre at heading alpha is d - h(alpha)).
    """
    alpha = np.asarray(alpha, float)
    return np.maximum(d - h(alpha, L, W), 0.0) * np.tan(alpha)


def four_flow_best(d, L, W):
    """Numerical maximiser of delta(alpha) on [0, alpha_max]: returns (alpha*, delta*).

    Roomy bay: (pi/2, inf) -- a 90 degree turn shifts any distance. Blocked: (0, 0).
    """
    reg = regime(d, L, W)
    if reg == "blocked":
        return 0.0, 0.0
    if reg == "roomy":
        return math.pi / 2, INF
    am = alpha_max(d, L, W)
    if am <= 0:
        return 0.0, 0.0
    grid = np.linspace(0.0, am, 4001)
    k = int(np.argmax(four_flow_delta(grid, d, L, W)))
    lo, hi = grid[max(k - 1, 0)], grid[min(k + 1, grid.size - 1)]
    gr = (math.sqrt(5) - 1) / 2
    for _ in range(80):                                   # golden section on the bracket
        c, e_ = hi - gr * (hi - lo), lo + gr * (hi - lo)
        if four_flow_delta(c, d, L, W) > four_flow_delta(e_, d, L, W):
            hi = e_
        else:
            lo = c
    a = 0.5 * (lo + hi)
    return a, float(four_flow_delta(a, d, L, W))


def four_flow_cycles(e, d, L, W):
    """Cycles of the best four-flow needed for an offset e (exact geometry, integer; inf if impossible)."""
    e = abs(e)
    if e == 0:
        return 0
    _, dstar = four_flow_best(d, L, W)
    if dstar == INF:
        return 0
    if dstar <= 0:
        return INF
    return int(math.ceil(e / dstar - 1e-12))


def four_flow_reversals(cycles):
    """Direction reversals of n four-flow cycles: 2n - 2 (see the counting convention)."""
    return INF if cycles == INF else max(0, 2 * cycles - 2)


# first-order (small slack eps = d - L) forms -- NOT exact
def alpha_max_small(eps, W):
    return eps / W


def alpha_star_small(eps, W):
    return eps / (2 * W)


def delta_star_small(eps, W):
    return eps ** 2 / (4 * W)


def four_flow_cycles_small(e, eps, W):
    return INF if eps <= 0 else 4 * W * abs(e) / eps ** 2


def lower_bound_small(e, eps, W):
    """The plan's claimed bound 2 W e / eps^2. Its unit is STRETCHES (reversals + 1), first order."""
    return INF if eps <= 0 else 2 * W * abs(e) / eps ** 2


def stretch_gain_exact(d, L, W):
    """Largest sideways shift of the rotation centre on ONE stretch, exact (derived here).

    On a stretch dy = tan(theta) dx and the heading at depth x is at most theta_max(x), the
    heading at which the footprint touches the nearer wall. Layer-cake integration gives

        Delta(d) = int_0^{alpha_max} sec^2(theta) (d - h(theta)) dtheta
                 = W (1 - cos a) - L (asinh(tan a) - sin a),   a = alpha_max(d).

    The path theta(x) = theta_max(x) attains it (in the closure: the footprint slides along a
    wall), so the minimum number of stretches for an offset e is ceil(e / Delta) and
    Delta ~ eps^2 / 2W (1 + (L/W)(eps/W)/3 + ...) > eps^2 / 2W. The width d - h(theta) of the
    admissible depth interval does not depend on the axle offset, so neither does Delta as
    long as those intervals are nested (alpha_max <= atan(W / (L + 2|a|))).
    """
    reg = regime(d, L, W)
    if reg == "blocked":
        return 0.0
    if reg == "roomy":
        return INF
    a = alpha_max(d, L, W)
    return W * (1 - math.cos(a)) - L * (math.asinh(math.tan(a)) - math.sin(a))


def min_stretches_exact(e, d, L, W, start="rear"):
    """Minimum stretches for an offset e in a strip (exact, centre or offset rotation centre).

    start='rear' (or 'front'): the robot starts touching a wall, every stretch is a full one.
    start='centre': the first stretch only covers half the depth range and gains Delta / 2.
    Returns 0 for e = 0, 1 for a roomy bay, inf when blocked or eps = 0.
    """
    e = abs(e)
    if e == 0:
        return 0
    D = stretch_gain_exact(d, L, W)
    if D == INF:
        return 1
    if D <= 0:
        return INF
    if start == "centre":
        return 1 if e <= D / 2 + 1e-12 else 1 + int(math.ceil((e - D / 2) / D - 1e-12))
    return int(math.ceil(e / D - 1e-12))


def min_reversals_exact(e, d, L, W, start="rear"):
    n = min_stretches_exact(e, d, L, W, start)
    return INF if n == INF else max(0, n - 1)


# --------------------------------------------------------------------------- brute force

def footprint_in_strip(x, th, d, L, W, a=0.0, tol=1e-12):
    """Brute-force feasibility: every footprint corner inside 0 <= x <= d. Uses corners only.

    Memory-lean (running min/max over the four corners, no (..., 4, 2) array).
    """
    x = np.asarray(x, float)
    th = np.asarray(th, float)
    c, s = np.cos(th), np.sin(th)
    lo = hi = None
    for bx, by in ((L / 2 + a, W / 2), (L / 2 + a, -W / 2), (-(L / 2 - a), -W / 2), (-(L / 2 - a), W / 2)):
        cx = c * bx - s * by
        lo = cx if lo is None else np.minimum(lo, cx)
        hi = cx if hi is None else np.maximum(hi, cx)
    return (x + lo >= -tol) & (x + hi <= d + tol)


def feasibility_table(xs, thetas, d, L, W, a=0.0, chunk=64):
    """Bool table F[ix, jtheta] of footprint_in_strip, built in row chunks to bound memory."""
    F = np.empty((xs.size, thetas.size), bool)
    for k in range(0, xs.size, chunk):
        F[k:k + chunk] = footprint_in_strip(xs[k:k + chunk, None], thetas[None, :], d, L, W, a)
    return F


MAX_TABLE_CELLS = 4_000_000        # lattice size guard (memory cap of the shared host)


def four_flow_poses(alpha, d, L, W, a=0.0, n=400, stroke_scale=1.0):
    """Densely sampled poses (x, y, theta) of one four-flow cycle, for swept-footprint checks."""
    x0 = (L / 2 - a) * math.cos(alpha) + (W / 2) * math.sin(alpha)      # rear-most centre for heading alpha
    s = stroke_scale * (d - float(h(alpha, L, W))) / math.cos(alpha)
    u = np.linspace(0.0, 1.0, n)
    one = np.ones(n)
    segs = [
        (x0 * one, 0.0 * one, alpha * u),                                               # rotate +alpha
        (x0 + s * math.cos(alpha) * u, s * math.sin(alpha) * u, alpha * one),            # drive
        ((x0 + s * math.cos(alpha)) * one, s * math.sin(alpha) * one, alpha * (1 - u)),  # rotate -alpha
        (x0 + s * math.cos(alpha) * (1 - u), s * math.sin(alpha) * one, 0.0 * one),      # reverse
    ]
    return tuple(np.concatenate([sg[k] for sg in segs]) for k in range(3))


def swept_free_in_strip(xs, ths, d, L, W, a=0.0, tol=1e-9):
    """True if every sampled pose keeps the footprint inside the strip."""
    return bool(np.all(footprint_in_strip(xs, ths, d, L, W, a, tol)))


def brute_alpha_max(d, L, W, n=200001):
    """Largest heading reachable by in-place rotation at the strip centre, by corner sampling."""
    th = np.linspace(0.0, math.pi / 2, n)
    ok = footprint_in_strip(d / 2, th, d, L, W)
    bad = np.flatnonzero(~ok)
    return math.pi / 2 if bad.size == 0 else float(th[max(bad[0] - 1, 0)])


# --------------------------------------------------------------------------- numerical planners

def _x_lattice(d, L, W, a, nx):
    return np.linspace(L / 2 - a, d - L / 2 - a, nx)


def _start_index(xs, d, a, x_start):
    if x_start == "rear":
        return 0
    if x_start == "front":
        return xs.size - 1
    target = d / 2 - a if x_start == "centre" else float(x_start)
    return int(np.argmin(np.abs(xs - target)))


def _components(frow):
    """Contiguous True runs of a 1-D bool array as (lo, hi) slices."""
    idx = np.flatnonzero(np.diff(np.concatenate([[False], frow, [False]]).astype(np.int8)))
    return [(int(idx[k]), int(idx[k + 1])) for k in range(0, idx.size, 2)]


def _stretch_general(V, F, comps, dx, tans, direction, arcs):
    """One stretch of the label-setting DP: V[ix, j] = largest y reachable at (x_ix, theta_j)."""
    V = V.copy()
    nx = V.shape[0]
    tmid = np.tan(0.5 * (np.arctan(tans[:-1]) + np.arctan(tans[1:])))
    order = range(nx) if direction > 0 else range(nx - 1, -1, -1)
    for i in order:
        for lo, hi in comps[i]:                       # in-place rotation: free within a feasible run
            V[i, lo:hi] = V[i, lo:hi].max()
        k = i + direction
        if not 0 <= k < nx:
            continue
        with np.errstate(invalid="ignore"):
            cand = np.where(F[i] & F[k], V[i] + direction * dx * tans, -np.inf)     # straight drive
            V[k] = np.maximum(V[k], cand)
            if arcs:                                  # drive while turning one heading step
                up = np.where(F[i, :-1] & F[k, 1:], V[i, :-1] + direction * dx * tmid, -np.inf)
                V[k, 1:] = np.maximum(V[k, 1:], up)
                dn = np.where(F[i, 1:] & F[k, :-1], V[i, 1:] + direction * dx * tmid, -np.inf)
                V[k, :-1] = np.maximum(V[k, :-1], dn)
    for i in range(nx):
        for lo, hi in comps[i]:
            V[i, lo:hi] = V[i, lo:hi].max()
    return V


def _layers(e, step_f, step_b, V0, goal_value, max_stretches):
    """Alternate forward and backward stretches until the goal label reaches e."""
    Vf, Vb = step_f(V0), step_b(V0)
    history = []
    for n in range(1, max_stretches + 1):
        best = max(goal_value(Vf), goal_value(Vb))
        history.append(best)
        if best >= e - 1e-9:
            return n, history
        if n > 2 and history[-1] <= history[-3] + 1e-15:
            break                                     # no progress over a full cycle: unreachable
        Vf, Vb = step_f(Vb), step_b(Vf)
    return INF, history


def plan_min_reversals(e, d, L, W, a=0.0, nx=401, ntheta=1201, x_start="rear", method="auto",
                       arcs=True, xs=None, thetas=None, max_stretches=200000):
    """Minimum direction reversals for a sideways shift e in a strip of depth d (numerical).

    Search space: poses (x, y, theta) of a pivot-capable differential-drive rectangle.
    Primitives: drive forward/back along the heading, rotate in place, and (general method)
    drive while turning one heading step. Feasibility comes from footprint corners only.
    Because the strip is invariant in y and only the shift matters, the label kept per
    lattice node (x, theta, stretch count) is the largest y reached; a larger y dominates.
    ``lattice_bfs_min_reversals`` checks that dominance on an exact lattice.

    method 'fast' uses the fact that, when the feasible headings at each x form one run
    containing 0, rotation makes all of them equivalent, so a stretch is a prefix-maximum;
    'general' loops over nodes and handles any feasibility table; 'auto' picks.
    The lattice is an inner approximation, so the count can only err upward (by discretisation).

    Returns dict(stretches, reversals, history, gain_per_stretch, method).
    """
    e = abs(e)
    if e == 0:
        return {"stretches": 0, "reversals": 0, "history": [], "gain_per_stretch": float("nan"), "method": "trivial"}
    if d < L:
        return {"stretches": INF, "reversals": INF, "history": [], "gain_per_stretch": 0.0, "method": "blocked"}
    xs = _x_lattice(d, L, W, a, nx) if xs is None else np.asarray(xs, float)
    nx = xs.size
    dx = (xs[-1] - xs[0]) / max(nx - 1, 1)
    i0 = _start_index(xs, d, a, x_start)
    if thetas is None:
        # Headings are confined to the feasible run that contains 0 (heading is continuous and
        # h(theta) <= d must hold throughout), so the lattice only covers that run.
        probe = xs[::max(1, nx // 60)]
        coarse = np.linspace(0.0, math.pi / 2, 2001)
        fc = np.array([footprint_in_strip(probe, t_, d, L, W, a).any() for t_ in coarse])
        bad = np.flatnonzero(~fc)
        if bad.size == 0:
            top = math.pi / 2 - 1e-3
        else:
            lo_, hi_ = coarse[bad[0] - 1], coarse[bad[0]]
            for _ in range(40):
                mid = 0.5 * (lo_ + hi_)
                if footprint_in_strip(xs, mid, d, L, W, a).any():
                    lo_ = mid
                else:
                    hi_ = mid
            top = lo_
        pos = np.linspace(0.0, top, ntheta)
        thetas = np.concatenate([-pos[:0:-1], pos])
    thetas = np.asarray(thetas, float)
    nth = thetas.size
    j0 = int(np.argmin(np.abs(thetas)))
    if nx * nth > MAX_TABLE_CELLS:
        raise MemoryError("lattice %d x %d exceeds MAX_TABLE_CELLS; use a coarser lattice" % (nx, nth))
    F = feasibility_table(xs, thetas, d, L, W, a)
    tans = np.tan(thetas)

    single = all(len(_components(F[i])) == 1 and F[i, j0] for i in range(nx))
    sym = nth % 2 == 1 and j0 == nth // 2 and np.allclose(thetas, -thetas[::-1]) and bool(np.all(F == F[:, ::-1]))
    use_fast = method == "fast" or (method == "auto" and single and sym)
    if use_fast:
        if not (single and sym):
            raise ValueError("fast method needs one feasible heading run per x and a symmetric lattice")
        jmax = F[:, j0:].sum(axis=1) - 1                              # feasible prefix on theta >= 0
        common = np.minimum(jmax[:-1], jmax[1:])
        S = np.concatenate([[0.0], np.cumsum(dx * tans[j0 + common])])
        m0 = np.full(nx, -np.inf)
        m0[i0] = 0.0

        def step_f(m):
            return S + np.maximum.accumulate(m - S)

        def step_b(m):
            return -S + np.maximum.accumulate((m + S)[::-1])[::-1]

        n, hist = _layers(e, step_f, step_b, m0, lambda m: float(m.max()), max_stretches)
        used = "fast"
    else:
        comps = [_components(F[i]) for i in range(nx)]
        V0 = np.full((nx, nth), -np.inf)
        V0[i0, j0] = 0.0

        def step_f(V):
            return _stretch_general(V, F, comps, dx, tans, +1, arcs)

        def step_b(V):
            return _stretch_general(V, F, comps, dx, tans, -1, arcs)

        n, hist = _layers(e, step_f, step_b, V0, lambda V: float(V[:, j0].max()), max_stretches)
        used = "general"
    inc = np.diff(hist) if len(hist) > 1 else np.array([hist[0] if hist else float("nan")])
    gain = float(np.max(inc)) if inc.size else float("nan")
    return {"stretches": n, "reversals": INF if n == INF else n - 1, "history": hist,
            "gain_per_stretch": gain, "method": used}


def numeric_stretch_gain(d, L, W, a=0.0, nx=801, ntheta=2401):
    """Largest one-stretch shift from the lattice planner (rear start, one stretch)."""
    r = plan_min_reversals(1e9, d, L, W, a=a, nx=nx, ntheta=ntheta, x_start="rear", max_stretches=1)
    return r["history"][0]


def lattice_bfs_min_reversals(e, d, L, W, a=0.0, nx=41, kappa=0.05, x_start="rear", max_stretches=400):
    """Plain reachability search on a true (x, y, theta) lattice, minimising reversals.

    Headings satisfy tan(theta_j) = j * kappa, so one drive step dx moves y by exactly j cells
    of size dy = kappa * dx: the lattice is exact, with no rounding. Primitives: drive one x
    step forward/back along the heading, rotate one heading step in place. y is clamped at
    the goal and motion below y = 0 is dropped. Returns dict(stretches, reversals, xs, thetas)
    so the dynamic programme can be run on the identical lattice.
    """
    e = abs(e)
    xs = _x_lattice(d, L, W, a, nx)
    dx = (xs[-1] - xs[0]) / (nx - 1)
    dy = kappa * dx
    J = 1
    while footprint_in_strip(xs, math.atan((J + 1) * kappa), d, L, W, a).any() and (J + 1) * kappa < 50:
        J += 1
    js = np.arange(-J, J + 1)
    thetas = np.arctan(js * kappa)
    F = feasibility_table(xs, thetas, d, L, W, a)
    comps = [_components(F[i]) for i in range(nx)]
    top = int(math.ceil(e / dy - 1e-9))
    if nx * (top + 1) * js.size > MAX_TABLE_CELLS:
        raise MemoryError("(x, y, theta) lattice too large; use a smaller e, a coarser nx or a larger kappa")
    i0 = _start_index(xs, d, a, x_start)
    R0 = np.zeros((nx, top + 1, js.size), bool)
    R0[i0, 0, J] = True

    def stretch(R, direction):
        R = R.copy()
        order = range(nx) if direction > 0 else range(nx - 1, -1, -1)
        for i in order:
            for lo, hi in comps[i]:
                R[i, :, lo:hi] |= R[i, :, lo:hi].any(axis=1, keepdims=True)
            k = i + direction
            if not 0 <= k < nx:
                continue
            for jj in np.flatnonzero(F[i] & F[k]):
                sh = direction * int(js[jj])
                col = R[i, :, jj]
                if sh == 0:
                    R[k, :, jj] |= col
                elif sh > 0:
                    R[k, sh:, jj] |= col[:-sh] if sh <= top else False
                    if col[max(top - sh, 0):].any():
                        R[k, top, jj] = True                      # clamp at the goal line
                else:
                    R[k, :sh, jj] |= col[-sh:]
        for i in range(nx):
            for lo, hi in comps[i]:
                R[i, :, lo:hi] |= R[i, :, lo:hi].any(axis=1, keepdims=True)
        return R

    Rf, Rb = stretch(R0, +1), stretch(R0, -1)
    n_found = INF
    for n in range(1, max_stretches + 1):
        if Rf[:, top, J].any() or Rb[:, top, J].any():
            n_found = n
            break
        Rf, Rb = stretch(Rb, +1), stretch(Rf, -1)
    return {"stretches": n_found, "reversals": INF if n_found == INF else n_found - 1,
            "xs": xs, "thetas": thetas, "dy": dy}


# --------------------------------------------------------------------------- door passage

def door_effective_width(theta, t, L, W):
    """Largest lateral extent of (footprint at heading theta) cut by a wall slab of thickness t.

    theta is measured from the passage axis. The footprint can stand in the opening at some
    lateral position iff this is <= the opening width, for every depth of the footprint
    relative to the wall. Exact: the extent is piecewise linear in the slab position, so it
    is evaluated at the breakpoints (slab faces on footprint vertices).
    """
    cor = G.rect_corners(0.0, 0.0, theta, L, W)                        # (4, 2), convex, ordered
    vx, vy = cor[:, 0], cor[:, 1]
    xmin, xmax = vx.min(), vx.max()

    def yrange(x):
        lo, hi = INF, -INF
        for k in range(4):
            x1, y1, x2, y2 = vx[k], vy[k], vx[(k + 1) % 4], vy[(k + 1) % 4]
            if min(x1, x2) - 1e-12 <= x <= max(x1, x2) + 1e-12:
                yv = [y1, y2] if abs(x2 - x1) < 1e-12 else [y1 + (y2 - y1) * (x - x1) / (x2 - x1)]
                lo, hi = min(lo, *yv), max(hi, *yv)
        return lo, hi

    best = 0.0
    for x0 in set([float(v) for v in vx] + [float(v) - t for v in vx]):
        a_, b_ = max(x0, xmin), min(x0 + t, xmax)
        if b_ < a_ - 1e-12:
            continue
        pts = [a_, b_] + [float(v) for v in vx if a_ <= v <= b_]
        los, his = zip(*(yrange(p) for p in pts))
        best = max(best, max(his) - min(los))
    return best


def door_heading_tolerance(w, t, L, W):
    """Largest |heading error| at which the footprint can still stand in the opening (exact, static).

    'Stand in the opening' means: for every depth of the robot relative to the wall there is a
    lateral position without collision. Returns 0 if W >= w, pi/2 if every heading is admissible.
    """
    if W >= w:
        return 0.0
    grid = np.linspace(0.0, math.pi / 2, 721)
    ok = np.array([door_effective_width(th, t, L, W) <= w for th in grid])
    bad = np.flatnonzero(~ok)
    if bad.size == 0:
        return math.pi / 2
    lo, hi = grid[bad[0] - 1], grid[bad[0]]
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if door_effective_width(mid, t, L, W) <= w:
            lo = mid
        else:
            hi = mid
    return float(lo)


def door_drive_through_tolerance(w, t, W):
    """Largest constant heading error at which a straight drive passes the wall: W + t sin = w cos (exact).

    The body sweeps a band of width W along its heading; inside the slab that band spans
    W / cos(theta) + t tan(theta) laterally. Independent of L. Equals the static tolerance when
    the wall is thinner than the part of the body with two parallel sides in the slab.
    """
    if W >= w:
        return 0.0
    return math.acos(W / math.hypot(w, t)) - math.atan2(t, w)


def door_heading_tolerance_small(w, L, W):
    """The plan's small-angle form (w - W) / L. Valid only for a wall at least about L thick."""
    return max(w - W, 0.0) / L


def door_heading_tolerance_thin_small(w, t, W):
    """Small-angle form for a thin wall, (w - W) / t (first order; diverges as t -> 0)."""
    return INF if t <= 0 else max(w - W, 0.0) / t


# --------------------------------------------------------------------------- visibility

def door_visibility(px, py, d, t, e, w, boxes=None, n_beams=720, max_range=12.0, min_returns=3):
    """What a 2-D lidar at (px, py) in the bay sees of the door opening.

    Analytic: the angle subtended by the near edges, and by the rays that clear BOTH faces of
    the wall (near and far edges); the first-order form w r / (r^2 + y^2). Ray check: beams at
    the lidar's angular resolution cast against the course boxes; a beam 'passes' if its first
    hit lies beyond the far face of the door wall. far_visible = passes >= min_returns.
    """
    r = d - px
    if r <= 0:
        raise ValueError("lidar must be on the bay side of the door wall")
    lo_n, hi_n = math.atan2(e - w / 2 - py, r), math.atan2(e + w / 2 - py, r)
    lo_f, hi_f = math.atan2(e - w / 2 - py, r + t), math.atan2(e + w / 2 - py, r + t)
    near = hi_n - lo_n
    through = max(0.0, min(hi_n, hi_f) - max(lo_n, lo_f))
    yy = e - py
    if boxes is None:
        big = 50.0
        boxes = np.array([[d, d + t, -big, e - w / 2], [d, d + t, e + w / 2, big]])
    ang = np.arange(n_beams) * (2 * math.pi / n_beams)
    rng = G.raycast(np.array([[px, py]]), ang, boxes, max_range)[0]
    hx = px + rng * np.cos(ang)
    passes = int(np.sum((hx > d + t + 1e-6) & (np.cos(ang) > 0)))
    res = 2 * math.pi / n_beams
    return {
        "near_width_rad": near,
        "through_width_rad": through,
        "first_order_width_rad": w * r / (r * r + yy * yy),
        "beams_through": passes,
        "angular_resolution_rad": res,
        "visibility_ratio": through / (min_returns * res),
        "far_visible": passes >= min_returns,
    }


# --------------------------------------------------------------------------- features, certificate

def certificate(course, robot, drive=None, margin=0.0):
    """Geometric certificate {'verdict': fits | does_not_fit | undetermined, 'reason': ...}.

    'fits' means a collision-free maneuver of the stated kind exists for the footprint with
    ``margin`` metres to spare. It says nothing about whether the robot can discover or
    complete it. The tight-bay argument treats the bay as a plain strip; it ignores the extra
    room the door opening itself offers to the nose (which can only help).
    """
    drive = drive or robot.drive
    L, W = robot.L, robot.W
    w, e, d, B = course.w, abs(course.e), course.d, course.B
    out = {"drive": drive, "margin": margin}

    def res(verdict, reason, **kw):
        out.update(verdict=verdict, reason=reason, **kw)
        return out

    if abs(course.e) + w / 2 > B / 2 + 1e-9:
        return res("undetermined", "door opening is not inside the bay width (malformed course)")
    if abs(course.start[2]) > 1e-9 or abs(course.start[1]) > 1e-9:
        return res("undetermined", "start pose is not on the bay axis at heading 0; formulas assume it is")
    if d < L:
        return res("does_not_fit", "bay depth d=%.3f < footprint length L=%.3f: the start pose collides" % (d, L))
    if min(L, W) >= w:
        return res("does_not_fit", "door width w=%.3f <= smallest footprint dimension %.3f" % (w, min(L, W)))
    two_rho = 2 * rho(L, W)
    side_room = B / 2 - e                                    # from the door axis to the nearer side wall

    if drive == "mecanum":
        if W < w - 2 * margin and d >= L + 2 * margin and side_room >= W / 2 + margin:
            return res("fits", "translate sideways at heading 0 (needs d >= L and w > W), then drive through")
        if W >= w and L < w - 2 * margin and d >= two_rho + 2 * margin and side_room >= rho(L, W) + margin:
            return res("fits", "rotate 90 deg (d >= 2 rho) and pass the door sideways (L < w)")
        if W >= w and d < two_rho:
            return res("undetermined", "W >= w needs a 90 deg turn but d < 2 rho; partial turns in the opening not analysed")
        return res("undetermined", "within margin %.3f of a limit" % margin)

    if drive != "differential":
        return res("undetermined", "unknown drive type %r" % drive)
    if W >= w:
        return res("does_not_fit", "differential drive passes the door along its heading and W=%.3f >= w=%.3f" % (W, w))
    if w - W <= 2 * margin:
        return res("undetermined", "door clearance w - W = %.3f is within margin" % (w - W))
    if e == 0:
        return res("fits", "no offset: drive straight through", min_reversals=0, four_flow_cycles=0)
    if d >= two_rho + 2 * margin:
        if side_room >= rho(L, W) + margin:
            return res("fits", "roomy bay (d >= 2 rho): turn 90 deg, drive e, turn back; no reversal",
                       min_reversals=0, four_flow_cycles=0)
        return res("undetermined", "roomy bay but the door axis is closer than rho to a side wall")
    dd = d - 2 * margin                                      # tight: shimmy inside an effective strip
    if dd <= L:
        if margin == 0 and d == L:
            return res("does_not_fit", "zero slack (d = L): no heading change and no sideways shift possible")
        return res("undetermined", "slack d - L = %.3f is within margin" % (d - L))
    if side_room < W / 2 + L * math.sin(alpha_max(dd, L, W)) / 2 + margin:
        return res("undetermined", "tight bay and the door axis is too close to a side wall for the shimmy")
    cyc = four_flow_cycles(e, dd, L, W)
    return res("fits", "tight bay (L < d < 2 rho): shimmy; at least %s reversals, %s four-flow cycles"
               % (min_reversals_exact(e, dd, L, W, "centre"), cyc),
               min_reversals=min_reversals_exact(e, dd, L, W, "centre"), four_flow_cycles=cyc)


def certificates_all_drives(course, robot, margin=0.0):
    return {dr: certificate(course, robot, drive=dr, margin=margin) for dr in ("differential", "mecanum")}


def features(course, robot, margin=0.0):
    """Dimensionless mechanics features of (course, robot) plus the certificate for the robot's drive."""
    L, W = robot.L, robot.W
    w, e, d, t = course.w, abs(course.e), course.d, course.t
    eps = d - L
    reg = regime(d, L, W)
    if robot.drive == "mecanum":
        cyc_exact, min_rev = 0, 0
    else:
        cyc_exact = four_flow_cycles(e, d, L, W)
        min_rev = min_reversals_exact(e, d, L, W, "centre")
    sx, sy = course.start[0], course.start[1]
    lid = robot.lidar
    vis = door_visibility(sx, sy, d, t, course.e, w, boxes=course.boxes(), n_beams=lid["n_beams"],
                          max_range=lid["max_range"])
    return {
        "label": "exploratory",
        "robot": robot.name,
        "drive": robot.drive,
        "regime": reg,
        "w_over_W": w / W,
        "e_over_W": e / W,
        "eps_over_W": eps / W,
        "d_over_2rho": d / (2 * rho(L, W)),
        "t_over_L": t / L,
        "predicted_cycles": cyc_exact,                                   # exact four-flow geometry
        "predicted_cycles_first_order": 0.0 if (reg == "roomy" or robot.drive == "mecanum" or e == 0)
        else four_flow_cycles_small(e, eps, W),                          # 4 W e / eps^2
        "min_reversals_exact": min_rev,                                  # strip bound, centre start
        "lower_bound_first_order": 0.0 if (reg == "roomy" or robot.drive == "mecanum" or e == 0)
        else lower_bound_small(e, eps, W),                               # 2 W e / eps^2 (stretches)
        "heading_tolerance": door_heading_tolerance(w, t, L, W),         # exact, static
        "heading_tolerance_drive_through": door_drive_through_tolerance(w, t, W),
        "heading_tolerance_small_angle": door_heading_tolerance_small(w, L, W),   # (w - W) / L
        "braking_over_d": robot.v_max ** 2 / (2 * robot.acc) / d,
        "visibility": vis["visibility_ratio"],
        "door_far_side_visible": bool(vis["far_visible"]),
        "door_beams_through": vis["beams_through"],
        "certificate": certificate(course, robot, margin=margin),
    }
