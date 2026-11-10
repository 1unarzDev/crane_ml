# SPDX-License-Identifier: GPL-3.0-or-later
"""Shared machinery for the experiment-2 navigation stacks (REACTIVE, MAP-RHP, HYBRID-CONF, HYBRID-FIXED).

Every stack uses the same pieces here, so the comparison only varies what the stack *believes* and how it adapts:
  * brake_table(): stopping distance from a closed-loop simulation of the plant (P velocity loop tau_track,
    jerk limit, tilt limit, drag), so the stopping rule includes controller lag and jerk (N1 failure 2).
  * StopRule: d_free >= v * tau_age + D_brake_sim(v) + margin, with tau_age = rolling p99 of the measured
    perception-to-command age (N1 failure 1). The stall watchdog lives in the episode (arbiter) loop.
  * TimedOccMap: the N1 log-odds map with registration offset (local estimate) and per-cell last-touch time, so a
    stack can forget evidence older than a memory horizon.
  * compose(): belief = accumulated (memory-masked) + w_prior * prior (shifted by the believed alignment) or an
    ephemeral current-frame map. Unknown = blocked unless the prior (w_prior > PRIOR_OBS_MIN) claims knowledge.
  * cost_to_go(): 8-connected wavefront on a 0.5 m grid over the whole corridor (unknown traversable, optimistic).
  * evaluate(): the N1 primitive set (48 headings x 5 speeds forward-simulated through the tracking loop) with the
    stopping rule above; scoring by cost-to-go progress (map stacks) or goal heading (reactive).
Velocity commands (m/s) and accelerations (m/s^2) are separate quantities throughout; no force gains appear here.
"""
import math

import numpy as np

from navpilot.planner import MAP, _disk, primitive_paths
from navpilot.vehicle import Vehicle

PRIOR_OBS_MIN = 0.05
_BRAKE = {}


def brake_table(veh, vmax=14.0, dv=0.25):
    """Distance (m) to stop from steady speed v under a zero-velocity command, simulated through the plant."""
    key = (veh["tau_track"], veh["jerk_max"], veh["a_h_max"])
    if key in _BRAKE:
        return _BRAKE[key]
    vs = np.arange(0.0, vmax + 1e-9, dv)
    ds = np.zeros_like(vs)
    dt = 0.005
    for k, v0 in enumerate(vs):
        if v0 <= 0:
            continue
        p = Vehicle(veh, 0.0, 0.0, 0.0, wind="calm")
        # settle at cruise (steady thrust holding drag) before braking
        p.vx = v0
        for _ in range(200):
            p.step(dt, v0, 0.0, 0.0, 0.0, 0.0)
        x0 = p.x
        for _ in range(4000):
            p.step(dt, 0.0, 0.0, 0.0, 0.0, 0.0)
            if p.vx < 0.05:
                break
        ds[k] = p.x - x0
    _BRAKE[key] = (vs, ds)
    return vs, ds


class StopRule:
    """Stopping rule with measured-age tau and simulated braking (lag + jerk + tilt + drag)."""

    def __init__(self, veh, tau_prior=0.12, margin=0.5, window=200, q=0.99):
        self.vs, self.ds = brake_table(veh)
        self.tau = tau_prior
        self.tau_prior = tau_prior
        self.margin = margin
        self.ages = []
        self.window, self.q = window, q

    def observe_age(self, age):
        self.ages.append(age)
        if len(self.ages) > self.window:
            self.ages = self.ages[-self.window:]
        if len(self.ages) >= 20:
            self.tau = max(self.tau_prior * 0.5, float(np.quantile(self.ages, self.q)))

    def dist(self, v, extra_margin=0.0):
        v = np.asarray(v, float)
        return v * self.tau + np.interp(v, self.vs, self.ds) + self.margin + extra_margin


class TimedOccMap:
    """N1 log-odds column map + last-touch time, integrated at the *estimated* pose (registration offset)."""

    def __init__(self, bounds, half_width, cfg=MAP):
        x0, x1, y0, y1 = bounds
        self.cfg, self.res = cfg, cfg["res"]
        self.x0, self.y0 = x0, -half_width
        self.nx = int(math.ceil((x1 - x0) / self.res))
        self.ny = int(math.ceil(2 * half_width / self.res))
        self.L = np.zeros((self.nx, self.ny), np.float32)
        self.obs = np.zeros((self.nx, self.ny), bool)
        self.last = np.full((self.nx, self.ny), -np.inf, np.float32)

    def clear(self):
        self.L.fill(0.0)
        self.obs.fill(False)
        self.last.fill(-np.inf)

    def integrate(self, fr, t, ex=0.0, ey=0.0):
        c = self.cfg
        ox, oy = fr.ox + ex, fr.oy + ey
        cosa, sina = np.cos(fr.angles), np.sin(fr.angles)
        carve = np.minimum(fr.carve, 12.0)
        steps = np.arange(0.0, 12.0, 0.8 * self.res)
        msk = steps[None, :] < carve[:, None]
        px = ox + steps[None, :] * cosa[:, None]
        py = oy + steps[None, :] * sina[:, None]
        i = ((px[msk] - self.x0) / self.res).astype(np.int64)
        j = ((py[msk] - self.y0) / self.res).astype(np.int64)
        ok = (i >= 0) & (i < self.nx) & (j >= 0) & (j < self.ny)
        i, j = i[ok], j[ok]
        self.L[i, j] += c["l_miss"]
        self.obs[i, j] = True
        self.last[i, j] = t
        self.L[i, j] = np.maximum(self.L[i, j], c["l_min"])
        hx = ox + fr.ranges[fr.hit] * cosa[fr.hit]
        hy = oy + fr.ranges[fr.hit] * sina[fr.hit]
        i = ((hx - self.x0) / self.res).astype(np.int64)
        j = ((hy - self.y0) / self.res).astype(np.int64)
        ok = (i >= 0) & (i < self.nx) & (j >= 0) & (j < self.ny)
        i, j = i[ok], j[ok]
        self.L[i, j] += c["l_hit"]
        self.obs[i, j] = True
        self.last[i, j] = t
        self.L[i, j] = np.clip(self.L[i, j], c["l_min"], c["l_max"])
        return hx, hy


def _slice(a, i0, j0, n, fill, nj=None):
    """a[i0:i0+n, j0:j0+nj] with out-of-range filled."""
    nj = n if nj is None else nj
    out = np.full((n, nj), fill, a.dtype)
    a0, a1 = max(i0, 0), min(i0 + n, a.shape[0])
    b0, b1 = max(j0, 0), min(j0 + nj, a.shape[1])
    if a1 > a0 and b1 > b0:
        out[a0 - i0:a1 - i0, b0 - j0:b1 - j0] = a[a0:a1, b0:b1]
    return out


class Belief:
    """What a stack plans on, for one cycle: sources and weights."""
    __slots__ = ("acc", "mem_s", "eph", "prior_L", "prior_obs", "w_prior", "shift", "t")

    def __init__(self, acc=None, mem_s=np.inf, eph=None, prior_L=None, prior_obs=None, w_prior=0.0, shift=(0, 0), t=0.0):
        self.acc, self.mem_s, self.eph = acc, mem_s, eph
        self.prior_L, self.prior_obs, self.w_prior, self.shift, self.t = prior_L, prior_obs, w_prior, shift, t


def _compose_region(b, i0, j0, n, nj=None):
    """Combined log-odds and observed mask on the region [i0, i0+n) x [j0, j0+nj) (square if nj is None)."""
    nj = n if nj is None else nj
    L = np.zeros((n, nj), np.float32)
    obs = np.zeros((n, nj), bool)
    if b.acc is not None:
        La = _slice(b.acc.L, i0, j0, n, 0.0, nj)
        oa = _slice(b.acc.obs, i0, j0, n, False, nj)
        if np.isfinite(b.mem_s):
            fresh = _slice(b.acc.last, i0, j0, n, -np.inf, nj) >= b.t - b.mem_s
            La = np.where(fresh, La, 0.0)
            oa &= fresh
        L += La
        obs |= oa
    if b.eph is not None:
        L += _slice(b.eph.L, i0, j0, n, 0.0, nj)
        obs |= _slice(b.eph.obs, i0, j0, n, False, nj)
    if b.prior_L is not None and b.w_prior > 0.0:
        si, sj = b.shift
        L += b.w_prior * _slice(b.prior_L, i0 - si, j0 - sj, n, 0.0, nj)
        if b.w_prior > PRIOR_OBS_MIN:
            obs |= _slice(b.prior_obs, i0 - si, j0 - sj, n, False, nj)
    return L, obs


def window(b, m, x, y, r_infl, win_m, unknown_free=False):
    """(inflated-occupied, unknown, raw-occupied, i0, j0) around (x, y), as navpilot.planner.blocked_window.
    unknown_free=True: optimistic belief (unknown cells traversable)."""
    res = m.res
    w = int(win_m / res)
    ci, cj = int((x - m.x0) / res), int((y - m.y0) / res)
    i0, j0 = ci - w, cj - w
    n = 2 * w + 1
    k = int(math.ceil(r_infl / res))
    L, obs = _compose_region(b, i0 - k, j0 - k, n + 2 * k)
    occ = L > m.cfg["l_occ"]
    # outside the map = blocked
    inside = np.zeros_like(occ)
    a0, a1 = max(i0 - k, 0) - (i0 - k), min(i0 + n + k, m.nx) - (i0 - k)
    b0, b1 = max(j0 - k, 0) - (j0 - k), min(j0 + n + k, m.ny) - (j0 - k)
    if a1 > a0 and b1 > b0:
        inside[a0:a1, b0:b1] = True
    occ |= ~inside
    unk = np.zeros((n, n), bool) if unknown_free else ~obs[k:k + n, k:k + n]
    infl = np.zeros((n, n), bool)
    for u, v in _disk(k):
        infl |= occ[k + u:k + u + n, k + v:k + v + n]
    return infl, unk, occ[k:k + n, k:k + n], i0, j0


def lookup_blocked(m, win, px, py, x, y):
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
        # inside an inflation blob: near the start, block only samples that approach the nearest occupied cell
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


# ------------------------------------------------------------------ cost-to-go
CTG_RES = 0.5


def cost_to_go(b, m, goal_x):
    """Wavefront cost-to-go (m) to the goal line x >= goal_x on a 0.5 m grid; blocked = any occupied fine cell."""
    f = int(round(CTG_RES / m.res))
    nx, ny = m.nx // f, m.ny // f
    L, _ = _compose_region(b, 0, 0, m.nx, m.ny)
    occ = (L[:nx * f, :ny * f] > m.cfg["l_occ"]).reshape(nx, f, ny, f).any(axis=(1, 3))
    free = ~occ
    C = np.full((nx, ny), np.inf)
    gi = int((goal_x - m.x0) / CTG_RES)
    C[min(gi, nx - 1):, :] = 0.0
    C[~free] = np.inf
    s2 = math.sqrt(2.0) * CTG_RES
    shifts = [(1, 0, CTG_RES), (-1, 0, CTG_RES), (0, 1, CTG_RES), (0, -1, CTG_RES),
              (1, 1, s2), (1, -1, s2), (-1, 1, s2), (-1, -1, s2)]
    P = np.full((nx + 2, ny + 2), np.inf)
    for _ in range(4 * (nx + ny)):
        P[1:-1, 1:-1] = C
        Cn = C.copy()
        for di, dj, c in shifts:
            np.minimum(Cn, P[1 + di:1 + di + nx, 1 + dj:1 + dj + ny] + c, out=Cn)
        Cn[~free] = np.inf
        Cn[min(gi, nx - 1):, :][free[min(gi, nx - 1):, :]] = 0.0
        if np.array_equal(Cn, C):
            break
        C = Cn
    return {"C": C, "x0": m.x0, "y0": m.y0, "res": CTG_RES}


def ctg_lookup(ctg, px, py):
    C = ctg["C"]
    i = np.clip(((np.asarray(px) - ctg["x0"]) / ctg["res"]).astype(np.int64), 0, C.shape[0] - 1)
    j = np.clip(((np.asarray(py) - ctg["y0"]) / ctg["res"]).astype(np.int64), 0, C.shape[1] - 1)
    return C[i, j]


def _ctg_soft(ctg, px, py):
    """Cost-to-go with blocked cells replaced by (neighbour min + 1 m), so a sample inside a coarse blocked cell
    next to free space still ranks sensibly; truly enclosed samples get a large finite value."""
    C = ctg["C"]
    if "Cs" not in ctg:
        Cp = np.pad(C, 1, constant_values=np.inf)
        nb = np.min(np.stack([Cp[1 + a:1 + a + C.shape[0], 1 + b:1 + b + C.shape[1]]
                              for a in (-1, 0, 1) for b in (-1, 0, 1)]), axis=0)
        Cs = np.where(np.isfinite(C), C, nb + 1.0)
        fin = np.isfinite(Cs)
        big = (Cs[fin].max() if fin.any() else 0.0) + 50.0
        ctg["Cs"] = np.where(fin, Cs, big)
    i = np.clip(((np.asarray(px) - ctg["x0"]) / ctg["res"]).astype(np.int64), 0, C.shape[0] - 1)
    j = np.clip(((np.asarray(py) - ctg["y0"]) / ctg["res"]).astype(np.int64), 0, C.shape[1] - 1)
    return ctg["Cs"][i, j]


# ------------------------------------------------------------------ primitive evaluation
def _free_along(m, win, th, PX, PY, S, H, x, y):
    blocked = lookup_blocked(m, win, PX, PY, x, y)
    anyb = blocked.any(axis=1)
    first = np.argmax(blocked, axis=1)
    rows = np.arange(len(th))
    s_prev = np.where(first > 0, S[rows, np.maximum(first - 1, 0)], 0.0)
    end_free = ~anyb & (S[:, -1] < H)
    d = np.where(anyb, s_prev, H)
    if end_free.any():
        ext = np.arange(0.2, H + 1e-9, 0.2)
        ex = PX[end_free, -1][:, None] + ext[None, :] * np.cos(th[end_free])[:, None]
        ey = PY[end_free, -1][:, None] + ext[None, :] * np.sin(th[end_free])[:, None]
        eb = lookup_blocked(m, win, ex, ey, x, y)
        ef = np.where(eb.any(axis=1), ext[np.argmax(eb, axis=1)] - 0.2, H)
        d[end_free] = np.minimum(S[end_free, -1] + ef, H)
    return np.maximum(d, 0.0)


def evaluate(b, m, x, y, vx, vy, goal, v_cap, veh, rule, horizon=14.0, win_m=15.0, inflate_extra=0.15,
             ctg=None, extra_margin=0.0, w_turn=0.35, w_free=0.6, n_headings=48, s_eval=4.0,
             b_safe=None, optimistic=False):
    """Choose (heading, speed). Feasible iff rule.dist(max(v_cmd, |v|)) <= d_free(path) in the belief.
    Score: progress rate g (cost-to-go decrease per metre at s_eval along the path, else cos(goal angle)).
    FASTER/SUPER-style option: optimistic=True scores primitives on the belief with unknown = free, and b_safe
    (fresh observations only, unknown = blocked) supplies the feasibility distance, i.e. every committed command
    keeps a backup stop (braking along the committed primitive) inside known-free space."""
    r_infl = veh["r_body"] + inflate_extra
    H = horizon
    gth = math.atan2(goal[1] - y, goal[0] - x)
    thetas = gth + np.linspace(-math.pi, math.pi, n_headings, endpoint=False)
    speeds = max(v_cap, 0.3) * np.array([1.0, 0.75, 0.5, 0.3, 0.15])
    sp = math.hypot(vx, vy)
    vth = math.atan2(vy, vx) if sp > 0.2 else gth
    win = window(b, m, x, y, r_infl, max(win_m, H + 1.0), unknown_free=optimistic)
    th, spd, PX, PY, S = primitive_paths(x, y, vx, vy, thetas, speeds, veh, H)
    rows = np.arange(len(th))
    d = _free_along(m, win, th, PX, PY, S, H, x, y)
    d_feas = d
    if b_safe is not None:
        d_feas = np.minimum(d, _free_along(m, window(b_safe, m, x, y, r_infl, max(win_m, H + 1.0)), th, PX, PY, S, H, x, y))
    v_rule = np.maximum(spd, sp)
    feas = rule.dist(v_rule, extra_margin) <= d_feas
    dv = np.abs((th - vth + math.pi) % (2 * math.pi) - math.pi)
    if ctg is not None:
        se = np.minimum(np.minimum(d, s_eval), S[:, -1])
        se = np.maximum(se, 0.3)
        # index of the first sample with arc >= se, per row
        kk = np.argmax(S >= se[:, None], axis=1)
        qx, qy = PX[rows, kk], PY[rows, kk]
        c0 = float(_ctg_soft(ctg, x, y))
        g = np.clip((c0 - _ctg_soft(ctg, qx, qy)) / np.maximum(S[rows, kk], 0.3), -1.0, 1.0)
    else:
        dg = np.abs((th - gth + math.pi) % (2 * math.pi) - math.pi)
        g = np.cos(dg)
    score = spd * g + 0.15 * v_cap * g - w_turn * dv * sp + w_free * np.minimum(d, H) / H
    if feas.any():
        k = int(np.argmax(np.where(feas, score, -np.inf)))
        v_cmd = float(spd[k])
    else:
        k = int(np.argmax(np.minimum(d, 3.0) + 0.5 * g))
        v_cmd = 0.0
    return {"theta": float(th[k]), "v_cmd": v_cmd, "d_free_cmd": float(d_feas[k]), "feasible_any": bool(feas.any()),
            "v_rule": float(max(v_cmd, sp)) if v_cmd > 0 else 0.0, "g": float(g[k])}
