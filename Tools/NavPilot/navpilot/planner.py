# SPDX-License-Identifier: GPL-3.0-or-later
"""Conventional baseline: log-odds occupancy map + inflation + ray-primitive receding-horizon planner with a
clearance-limited speed from the stopping rule

    d_free >= v * tau + v^2 / (2 a_brake) + margin        (all in the planner's belief)

tau and a_brake are the planner's *nominal* beliefs (pipeline p50 latency, 0.8 x tilt-limited deceleration); the
baseline does not know the compute-stall tail. Unknown cells count as blocked; occupied cells are inflated by
r_body + inflate_extra. Free distance along a heading is the distance to the first blocked sample.
"""
import math

import numpy as np

PLANNER = {"rate_hz": 20.0, "tau_belief_s": 0.10, "brake_frac": 0.8, "margin_m": 0.5, "inflate_extra_m": 0.10,
           "horizon_m": 14.0, "n_headings": 48, "window_m": 15.0, "w_turn": 0.35, "w_free": 0.6,
           "stuck_s": 6.0}
# 2.5D column map standing in for a 3D voxel map projected over the 0.7 m flight slab: a missed ray in the slab
# crosses an obstacle's voxel layer with probability ~ voxel/slab = 0.1/0.7, so the per-column miss update is the
# OctoMap-style -0.4 scaled by ~0.25 (rounded up, conservative toward erosion). Clamp min -1.0 (two hits recover).
MAP = {"res": 0.1, "l_hit": 0.85, "l_miss": -0.1, "l_min": -1.0, "l_max": 3.5, "l_occ": 0.4}


def stop_distance(v, tau, a, margin):
    return v * tau + v * v / (2.0 * a) + margin


def v_from_free(d_free, tau, a, margin):
    """Largest v >= 0 with stop_distance(v) <= d_free (0 if none)."""
    d = np.asarray(d_free, float) - margin
    v = a * (-tau + np.sqrt(tau * tau + 2.0 * np.maximum(d, 0.0) / a))
    return np.where(d > 0, v, 0.0)


class OccMap:
    def __init__(self, bounds, half_width, cfg=MAP):
        x0, x1, y0, y1 = bounds
        self.cfg = cfg
        self.res = cfg["res"]
        self.x0, self.y0 = x0, -half_width
        self.nx = int(math.ceil((x1 - x0) / self.res))
        self.ny = int(math.ceil(2 * half_width / self.res))
        self.L = np.zeros((self.nx, self.ny), np.float32)
        self.obs = np.zeros((self.nx, self.ny), bool)
        self.first_occ = np.full((self.nx, self.ny), np.inf, np.float32)

    def cell(self, x, y):
        return ((np.asarray(x) - self.x0) / self.res).astype(np.int64), ((np.asarray(y) - self.y0) / self.res).astype(np.int64)

    def integrate(self, fr, t):
        c = self.cfg
        cosa, sina = np.cos(fr.angles), np.sin(fr.angles)
        carve = np.minimum(fr.carve, 12.0)
        steps = np.arange(0.0, 12.0, 0.8 * self.res)
        msk = steps[None, :] < carve[:, None]
        px = fr.ox + steps[None, :] * cosa[:, None]
        py = fr.oy + steps[None, :] * sina[:, None]
        i, j = self.cell(px[msk], py[msk])
        ok = (i >= 0) & (i < self.nx) & (j >= 0) & (j < self.ny)
        i, j = i[ok], j[ok]
        self.L[i, j] += c["l_miss"]           # fancy-index += applies once per cell per frame
        self.obs[i, j] = True
        hx = fr.ox + fr.ranges[fr.hit] * cosa[fr.hit]
        hy = fr.oy + fr.ranges[fr.hit] * sina[fr.hit]
        i, j = self.cell(hx, hy)
        ok = (i >= 0) & (i < self.nx) & (j >= 0) & (j < self.ny)
        i, j = i[ok], j[ok]
        self.L[i, j] += c["l_hit"]
        self.obs[i, j] = True
        np.clip(self.L, c["l_min"], c["l_max"], out=self.L)
        newly = (self.L[i, j] > c["l_occ"]) & np.isinf(self.first_occ[i, j])
        self.first_occ[i[newly], j[newly]] = t

    def first_occupied_near(self, xs, ys, radius=0.15):
        """Earliest time any cell within `radius` of the given points became occupied (inf if never)."""
        k = int(math.ceil(radius / self.res))
        i, j = self.cell(xs, ys)
        best = math.inf
        for a in range(-k, k + 1):
            for b in range(-k, k + 1):
                ii, jj = i + a, j + b
                ok = (ii >= 0) & (ii < self.nx) & (jj >= 0) & (jj < self.ny)
                if ok.any():
                    best = min(best, float(self.first_occ[ii[ok], jj[ok]].min()))
        return best

    def occupied_now_near(self, xs, ys, radius=0.15):
        k = int(math.ceil(radius / self.res))
        i, j = self.cell(xs, ys)
        for a in range(-k, k + 1):
            for b in range(-k, k + 1):
                ii, jj = i + a, j + b
                ok = (ii >= 0) & (ii < self.nx) & (jj >= 0) & (jj < self.ny)
                if ok.any() and (self.L[ii[ok], jj[ok]] > self.cfg["l_occ"]).any():
                    return True
        return False


_DISK_CACHE = {}


def _disk(k):
    if k not in _DISK_CACHE:
        a = np.arange(-k, k + 1)
        _DISK_CACHE[k] = [(int(u), int(v)) for u in a for v in a if u * u + v * v <= k * k]
    return _DISK_CACHE[k]


def blocked_window(m, x, y, r_infl, window):
    """Inflated-occupied | unknown, on a square window around (x, y). Outside the corridor = blocked."""
    res = m.res
    w = int(window / res)
    ci, cj = int((x - m.x0) / res), int((y - m.y0) / res)
    i0, j0 = ci - w, cj - w
    n = 2 * w + 1
    k = int(math.ceil(r_infl / res))
    occ = np.ones((n + 2 * k, n + 2 * k), bool)
    unk = np.ones((n, n), bool)
    a0, a1 = max(i0 - k, 0), min(i0 + n + k, m.nx)
    b0, b1 = max(j0 - k, 0), min(j0 + n + k, m.ny)
    if a1 > a0 and b1 > b0:
        occ[a0 - (i0 - k):a1 - (i0 - k), b0 - (j0 - k):b1 - (j0 - k)] = m.L[a0:a1, b0:b1] > m.cfg["l_occ"]
    a0, a1 = max(i0, 0), min(i0 + n, m.nx)
    b0, b1 = max(j0, 0), min(j0 + n, m.ny)
    if a1 > a0 and b1 > b0:
        unk[a0 - i0:a1 - i0, b0 - j0:b1 - j0] = ~m.obs[a0:a1, b0:b1]
    infl = np.zeros((n, n), bool)
    for u, v in _disk(k):
        infl |= occ[k + u:k + u + n, k + v:k + v + n]
    raw = occ[k:k + n, k:k + n]
    return infl, unk, raw, i0, j0


def _lookup_blocked(m, win, px, py, x, y):
    infl, unk, raw, i0, j0 = win
    n = infl.shape[0]
    ii = ((px - m.x0) / m.res).astype(np.int64) - i0
    jj = ((py - m.y0) / m.res).astype(np.int64) - j0
    inside = (ii >= 0) & (ii < n) & (jj >= 0) & (jj < n)
    ii, jj = np.clip(ii, 0, n - 1), np.clip(jj, 0, n - 1)
    ci = min(max(int((x - m.x0) / m.res) - i0, 0), n - 1)
    cj = min(max(int((y - m.y0) / m.res) - j0, 0), n - 1)
    bl = infl[ii, jj]
    if infl[ci, cj]:
        # Already inside an inflation blob: near the start, a sample is blocked only if it is no farther from the
        # nearest occupied cell than the vehicle is now (escape allowed, approach forbidden).
        k = 10
        a0, b0 = max(ci - k, 0), max(cj - k, 0)
        oi, oj = np.nonzero(raw[a0:ci + k + 1, b0:cj + k + 1])
        if len(oi):
            ox = m.x0 + (oi + a0 + i0 + 0.5) * m.res
            oy = m.y0 + (oj + b0 + j0 + 0.5) * m.res
            d0 = float(np.min(np.hypot(ox - x, oy - y)))
            near = np.hypot(px - x, py - y) < 0.6
            if near.any():
                qx, qy = px[near], py[near]
                dq = np.min(np.hypot(qx[:, None] - ox[None, :], qy[:, None] - oy[None, :]), axis=1)
                bl = bl.copy()
                bl[near] = dq < d0 - 0.01
    return bl | unk[ii, jj] | ~inside


def free_distances(m, x, y, thetas, r_infl, horizon, window):
    """Straight-ray free distance (used for the velocity-direction check)."""
    win = blocked_window(m, x, y, r_infl, window)
    s = np.arange(0.1, horizon + 1e-9, 0.1)
    px = x + s[None, :] * np.cos(thetas)[:, None]
    py = y + s[None, :] * np.sin(thetas)[:, None]
    blocked = _lookup_blocked(m, win, px, py, x, y)
    first = np.argmax(blocked, axis=1)
    return np.maximum(np.where(blocked.any(axis=1), s[first] - 0.1, horizon), 0.0)


def primitive_paths(x, y, vx, vy, thetas, speeds, veh, horizon, n_steps=60):
    """Closed-loop forward simulation of the velocity P loop (tau_track, |a| <= a_h_max) for every
    (heading, speed) command. Returns positions [K, n_steps] and cumulative arc length [K, n_steps]."""
    th = np.repeat(thetas, len(speeds))
    sp = np.tile(speeds, len(thetas))
    T = np.clip(horizon / np.maximum(sp, 0.5), 2.0, 8.0)
    dt = T / n_steps
    cx, cy = sp * np.cos(th), sp * np.sin(th)
    K = len(th)
    px, py = np.full(K, float(x)), np.full(K, float(y))
    ux, uy = np.full(K, float(vx)), np.full(K, float(vy))
    PX, PY, S = np.empty((K, n_steps)), np.empty((K, n_steps)), np.empty((K, n_steps))
    arc = np.zeros(K)
    tau, amax = veh["tau_track"], veh["a_h_max"]
    for k in range(n_steps):
        ax, ay = (cx - ux) / tau, (cy - uy) / tau
        an = np.hypot(ax, ay)
        sc = np.minimum(1.0, amax / np.maximum(an, 1e-9))
        ux += ax * sc * dt
        uy += ay * sc * dt
        dx, dy = ux * dt, uy * dt
        px += dx
        py += dy
        arc += np.hypot(dx, dy)
        PX[:, k], PY[:, k], S[:, k] = px, py, arc
    return th, sp, PX, PY, S


def plan(m, x, y, vx, vy, goal, v_limit, veh, cfg=PLANNER):
    """Sample (heading, speed) commands, forward-simulate each through the tracking loop, and keep only those
    satisfying the stopping rule in belief:

        stop_distance(v_rule, tau_belief, a_belief, margin) <= d_free(path),  v_rule = max(v_cmd, |v_now|)

    where d_free(path) is the arc length to the first blocked (inflated-occupied or unknown) sample, extended
    straight to the horizon if the simulated path ends free. If none is feasible: v_cmd = 0 (brake/hover).
    Invariant (tested): v_cmd == 0 or stop_distance(max(v_cmd, |v|)) <= d_free_cmd.
    """
    tau = cfg["tau_belief_s"]
    a = cfg["brake_frac"] * min(veh.get("a_h_tilt", veh["a_h_max"]), veh["a_h_max"])
    margin = cfg["margin_m"]
    r_infl = veh["r_body"] + cfg["inflate_extra_m"]
    H = cfg["horizon_m"]
    gth = math.atan2(goal[1] - y, goal[0] - x)
    thetas = gth + np.linspace(-math.pi, math.pi, cfg["n_headings"], endpoint=False)
    speeds = v_limit * np.array([1.0, 0.75, 0.5, 0.3, 0.15])
    sp = math.hypot(vx, vy)
    vth = math.atan2(vy, vx) if sp > 0.2 else gth
    win = blocked_window(m, x, y, r_infl, cfg["window_m"])
    th, spd, PX, PY, S = primitive_paths(x, y, vx, vy, thetas, speeds, veh, H)
    blocked = _lookup_blocked(m, win, PX, PY, x, y)
    anyb = blocked.any(axis=1)
    first = np.argmax(blocked, axis=1)
    rows = np.arange(len(th))
    s_prev = np.where(first > 0, S[rows, np.maximum(first - 1, 0)], 0.0)
    # free path that ends before the horizon: extend straight along the commanded heading
    end_free = ~anyb & (S[:, -1] < H)
    if end_free.any():
        ext = np.arange(0.2, H + 1e-9, 0.2)
        ex = PX[end_free, -1][:, None] + ext[None, :] * np.cos(th[end_free])[:, None]
        ey = PY[end_free, -1][:, None] + ext[None, :] * np.sin(th[end_free])[:, None]
        eb = _lookup_blocked(m, win, ex, ey, x, y)
        ef = np.where(eb.any(axis=1), ext[np.argmax(eb, axis=1)] - 0.2, H)
    d = np.where(anyb, s_prev, H)
    if end_free.any():
        d[end_free] = np.minimum(S[end_free, -1] + ef, H)
    d = np.maximum(d, 0.0)
    v_rule = np.maximum(spd, sp)
    feas = stop_distance(v_rule, tau, a, margin) <= d
    dg = np.abs((th - gth + math.pi) % (2 * math.pi) - math.pi)
    dv = np.abs((th - vth + math.pi) % (2 * math.pi) - math.pi)
    score = (spd * np.cos(dg) + 0.15 * v_limit * np.cos(dg) - cfg["w_turn"] * dv * sp
             + cfg["w_free"] * np.minimum(d, H) / H)
    d_vel = float(free_distances(m, x, y, np.array([vth]), r_infl, H, cfg["window_m"])[0])
    emergency = sp > 0.3 and stop_distance(sp, tau, a, margin) > d_vel
    if feas.any():
        k = int(np.argmax(np.where(feas, score, -np.inf)))
        v_cmd = float(spd[k])
    else:
        # nothing satisfies the rule: brake to hover and face the most open heading toward the goal
        k = int(np.argmax(np.minimum(d, 3.0) + 0.5 * np.cos(dg)))
        v_cmd = 0.0
    return {"theta": float(th[k]), "v_cmd": v_cmd, "d_free_cmd": float(d[k]), "v_rule": float(max(v_cmd, sp)) if v_cmd > 0 else 0.0,
            "thetas": th, "speeds": spd, "d_free": d, "feasible": feas, "emergency": bool(emergency), "tau": tau,
            "a_brake": a, "margin": margin, "d_free_vel": d_vel}
