# SPDX-License-Identifier: GPL-3.0-or-later
"""Experiment-3 stacks ("max safe speed"). New file: the E2-frozen modules (core.py, policies.py) are imported, never
edited. One configurable stack, AdvStack, whose feature flags define every E3 method:

  belief       "short": fresh accumulated map with a short memory (reactive, no prior, goal-heading score);
               "map": fresh accumulated map + prior map + cost-to-go score.
  optimistic   score on unknown = free (+ prior), but a command is feasible only if braking along it stops inside
               space the fresh map observed free (FASTER/SUPER-style backup stop). False: unknown = blocked.
  prior        "full": prior at w = 1, shifted by the believed alignment (E2 OPT_BACKUP);
               "ua": w = c_align (continuous) and prior obstacles dilated by z * sigma_align (uncertainty-aware).
  mem          "inf" or "drift": fresh-map memory horizon from the drift budget, mem = (tol / sigma_loc)^2 / v.
  latcomp      plan from the state predicted at command application (median compute time, current command
               through the tracking loop); the stopping-rule tau then drops the compensated median (no double count).
  tighten      constraint tightening along the primitive: extra clearance k0 + ks * s (s = arc length, m), in
               0.1 m tiers, beyond 0.6 m from the start.
  pas          perception-aware backup set (the E3 "active perception" method): a fresh-free cell counts as free for
               the backup stop only if it was observed free from within R_conf (range at which a thin branch would
               have been detected) OR lies farther than d_struct from any mapped structure (branches grow from
               stems/deadfall). This schedules speed so that braking ends in close-range-confirmed space.
  look         yaw toward the committed path at the stopping distance (camera looks where the vehicle will be).

All stacks keep the E2 safety layer: StopRule (p99 measured age tau + simulated lag/jerk braking) and the episode's
stall watchdog. Velocities (m/s) and accelerations (m/s^2) only; no force gains.
"""
import math

import numpy as np

from .core import Belief, StopRule, _ctg_soft, _slice, cost_to_go, lookup_blocked, primitive_paths, window
from .policies import CONF_DEFAULT, confidences

ADV_DEFAULTS = {
    "belief": "map", "optimistic": True, "react_mem": 1.5, "H": 14.0,
    "prior": "full", "ua_z": 1.5, "ua_kmax": 10, "ua_wmin": 0.0,
    "mem": "inf", "drift_tol": 0.15, "mem_min": 0.3,
    "latcomp": False, "tighten_k0": 0.0, "tighten_ks": 0.0,
    "pas": False, "R_conf": 4.0, "d_struct": 1.5,
    "look": False,
    "margin": 0.5, "inflate_extra": 0.15, "w_free": 0.6, "w_turn": 0.35, "n_headings": 48,
}

# E3 methods (feature sets). Tuned parameters are layered on top per method (results/raw/e3_tuned.json).
METHODS = {
    "opt_backup_t": {},                                                     # strongest baseline (SUPER/FASTER-style)
    "reactive_plus": {"belief": "short", "H": 10.0, "latcomp": True},      # stronger reactive
    "mpc_lc": {"latcomp": True, "tighten_k0": 0.05, "tighten_ks": 0.01},  # latency-comp. + tightening
    "ua_map": {"prior": "ua", "mem": "drift"},                             # uncertainty-aware map reliance
    "pas_opt": {"pas": True, "look": True},                                # active perception on the baseline
    "pas_full": {"prior": "ua", "mem": "drift", "latcomp": True, "tighten_k0": 0.05, "tighten_ks": 0.01,
                 "pas": True, "look": True},                               # everything
}


class AdvRule(StopRule):
    """StopRule whose tau drops the latency-compensated part (sub, s)."""

    def __init__(self, veh, margin=0.5):
        super().__init__(veh, margin=margin)
        self.sub = 0.0

    def dist(self, v, extra_margin=0.0):
        v = np.asarray(v, float)
        tau = max(self.tau - self.sub, 0.06)
        return v * tau + np.interp(v, self.vs, self.ds) + self.margin + extra_margin


def _dilate(a, k):
    """Square (Chebyshev) dilation by k cells via repeated 3x3 max."""
    out = a.copy()
    for _ in range(int(k)):
        P = np.pad(out, 1)
        n = P[1:-1, 1:-1].copy()
        for di in (0, 1, 2):
            for dj in (0, 1, 2):
                n |= P[di:di + out.shape[0], dj:dj + out.shape[1]]
        out = n
    return out


def _dilate_coarse(a, k_fine, f=5):
    """Fast approximate dilation by k_fine cells: max-pool by f, dilate ceil(k_fine / f), upsample."""
    n0, n1 = a.shape
    m0, m1 = -(-n0 // f), -(-n1 // f)
    P = np.zeros((m0 * f, m1 * f), bool)
    P[:n0, :n1] = a
    C = P.reshape(m0, f, m1, f).any(axis=(1, 3))
    C = _dilate(C, int(math.ceil(k_fine / f)))
    return np.repeat(np.repeat(C, f, axis=0), f, axis=1)[:n0, :n1]


def _blocked_t(m, win, tiers, px, py, s, x, y, k0, ks):
    """lookup_blocked plus tiered tightening: sample blocked if inside tier(k0 + ks*s) inflation beyond 0.6 m."""
    bl = lookup_blocked(m, win, px, py, x, y)
    if tiers is None:
        return bl
    i0, j0 = win[3], win[4]
    n = tiers.shape[1]
    ii = np.clip(((px - m.x0) / m.res).astype(np.int64) - i0, 0, n - 1)
    jj = np.clip(((py - m.y0) / m.res).astype(np.int64) - j0, 0, n - 1)
    tier = np.clip(np.floor((k0 + ks * s) / m.res + 0.5).astype(np.int64), 0, tiers.shape[0] - 1)
    return bl | (tiers[tier, ii, jj] & (s > 0.6))


def _free_along_t(m, win, tiers, th, PX, PY, S, H, x, y, k0=0.0, ks=0.0):
    blocked = _blocked_t(m, win, tiers, PX, PY, S, x, y, k0, ks)
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
        es = S[end_free, -1][:, None] + ext[None, :]
        eb = _blocked_t(m, win, tiers, ex, ey, es, x, y, k0, ks)
        ef = np.where(eb.any(axis=1), ext[np.argmax(eb, axis=1)] - 0.2, H)
        d[end_free] = np.minimum(S[end_free, -1] + ef, H)
    return np.maximum(d, 0.0)


def _tiers(win, kmax):
    infl = win[0]
    T = [infl]
    for _ in range(kmax):
        T.append(_dilate(T[-1], 1))
    return np.stack(T)


class AdvStack:
    def __init__(self, name, veh, params=None):
        self.name, self.veh = name, veh
        self.p = dict(ADV_DEFAULTS)
        self.p.update(METHODS.get(name, {}))
        self.p.update(params or {})
        self.rule = AdvRule(veh, margin=self.p["margin"])
        self.watchdog = True
        self.ctg, self.ctg_t, self._ctg_key = None, -1e9, None
        self.mode, self.mode_switches, self.fallback_cycles = "map", 0, 0
        self.comps = []
        self._pd = {}
        self.trace = {"w_prior": 0.0, "v_cap": 0.0, "H": 0.0, "n": 0, "untrusted_cut": 0}

    def observe_compute(self, c):
        self.comps.append(c)
        if len(self.comps) > 100:
            self.comps = self.comps[-100:]

    def _prior(self, pL, k):
        if k <= 0:
            return pL
        if k not in self._pd:
            occ = _dilate(pL > 0.5, k)
            self._pd[k] = np.where(occ, np.float32(2.0), pL).astype(np.float32)
        return self._pd[k]

    def _predict(self, x, y, vx, vy, cmd, c):
        tau, amax = self.veh["tau_track"], self.veh["a_h_max"]
        n = 8
        dt = c / n
        for _ in range(n):
            ax, ay = (cmd[0] - vx) / tau, (cmd[1] - vy) / tau
            an = math.hypot(ax, ay)
            if an > amax:
                ax, ay = ax * amax / an, ay * amax / an
            vx += ax * dt
            vy += ay * dt
            x += vx * dt
            y += vy * dt
        return x, y, vx, vy

    def cycle(self, t, x, y, vx, vy, goal, v_lim, acc, eph, prior, shift, sig, v_cap_ext=math.inf, cmd=(0.0, 0.0)):
        p = self.p
        pL, pO = prior if prior is not None else (None, None)
        sp0 = math.hypot(vx, vy)
        # memory horizon of the fresh map
        if p["mem"] == "drift":
            mem = max(p["mem_min"], (p["drift_tol"] / max(sig["sigma_loc_rate"], 1e-3)) ** 2 / max(sp0, 1.0))
        else:
            mem = math.inf
        if p["belief"] == "short":
            mem = min(mem, p["react_mem"])
        w, kd = 0.0, 0
        if p["belief"] == "map" and pL is not None:
            if p["prior"] == "ua":
                ca, _, _ = confidences(sig, CONF_DEFAULT)
                w = max(ca, p["ua_wmin"])
                s_eff = sig["sigma_align_rep"] + CONF_DEFAULT["age_growth"] * sig["rep_age"]
                kd = min(int(math.ceil(p["ua_z"] * s_eff / acc.res)), int(p["ua_kmax"]))
                pl = self._prior(pL, kd)
            else:
                w, pl = 1.0, pL
            b = Belief(acc=acc, mem_s=mem, prior_L=pl if w > 0 else None, prior_obs=pO, w_prior=w, shift=shift, t=t)
        else:
            b = Belief(acc=acc, mem_s=mem, t=t)
        b_safe = Belief(acc=acc, mem_s=mem, t=t)
        # latency compensation
        if p["latcomp"]:
            c_hat = float(np.median(self.comps)) if len(self.comps) >= 5 else 0.03
            x, y, vx, vy = self._predict(x, y, vx, vy, cmd, c_hat)
            self.rule.sub = c_hat
        v_cap = min(v_lim, v_cap_ext)
        ctg = None
        if p["belief"] == "map":
            key = (round(w, 1), kd)
            if self.ctg is None or t - self.ctg_t >= 0.5 or key != self._ctg_key:
                self.ctg, self.ctg_t, self._ctg_key = cost_to_go(b, acc, goal[0]), t, key
            ctg = self.ctg
        out = self._evaluate(b, b_safe, acc, x, y, vx, vy, goal, v_cap, ctg)
        out["fallback"] = False
        tr = self.trace
        tr["w_prior"] += w
        tr["v_cap"] += v_cap
        tr["H"] += p["H"]
        tr["n"] += 1
        return out

    def _untrusted(self, m, win):
        """Cells (window coords) observed free but neither close-range-confirmed nor far from structure."""
        p = self.p
        infl, unk, raw, i0, j0 = win
        n = infl.shape[0]
        ks = int(math.ceil(p["d_struct"] / m.res))
        occ = _slice(m.L, i0 - ks, j0 - ks, n + 2 * ks, 0.0) > m.cfg["l_occ"]
        near = _dilate_coarse(occ, ks)[ks:ks + n, ks:ks + n]
        rmin = _slice(m.rmin, i0, j0, n, np.inf)
        return near & (rmin > p["R_conf"])

    def _evaluate(self, b, b_safe, m, x, y, vx, vy, goal, v_cap, ctg):
        p, veh, rule = self.p, self.veh, self.rule
        H = p["H"]
        r_infl = veh["r_body"] + p["inflate_extra"]
        gth = math.atan2(goal[1] - y, goal[0] - x)
        thetas = gth + np.linspace(-math.pi, math.pi, int(p["n_headings"]), endpoint=False)
        speeds = max(v_cap, 0.3) * np.array([1.0, 0.75, 0.5, 0.3, 0.15])
        sp = math.hypot(vx, vy)
        vth = math.atan2(vy, vx) if sp > 0.2 else gth
        wm = max(15.0, H + 1.0)
        th, spd, PX, PY, S = primitive_paths(x, y, vx, vy, thetas, speeds, veh, H)
        rows = np.arange(len(th))
        k0, ksl = p["tighten_k0"], p["tighten_ks"]
        tight = k0 > 0 or ksl > 0
        kmax = int(math.ceil((k0 + ksl * (2 * H)) / m.res)) if tight else 0
        kmax = min(kmax, 4)
        if p["optimistic"]:
            win_s = window(b, m, x, y, r_infl, wm, unknown_free=True)
            d = _free_along_t(m, win_s, None, th, PX, PY, S, H, x, y)
            win_f = window(b_safe, m, x, y, r_infl, wm)
        else:
            win_f = window(b, m, x, y, r_infl, wm)
            d = None
        win_f0 = win_f
        if p["pas"]:
            u = self._untrusted(m, win_f)
            win_f = (win_f[0], win_f[1] | u, win_f[2], win_f[3], win_f[4])
        tiers = _tiers(win_f, kmax) if tight and kmax > 0 else None
        d_f = _free_along_t(m, win_f, tiers, th, PX, PY, S, H, x, y, k0, ksl)
        if d is None:
            d = _free_along_t(m, win_f0, None, th, PX, PY, S, H, x, y) if (p["pas"] or tiers is not None) else d_f
        d_feas = np.minimum(d, d_f)
        v_rule = np.maximum(spd, sp)
        feas = rule.dist(v_rule) <= d_feas
        dv = np.abs((th - vth + math.pi) % (2 * math.pi) - math.pi)
        if ctg is not None:
            se = np.maximum(np.minimum(np.minimum(d, 4.0), S[:, -1]), 0.3)
            kk = np.argmax(S >= se[:, None], axis=1)
            qx, qy = PX[rows, kk], PY[rows, kk]
            c0 = float(_ctg_soft(ctg, x, y))
            g = np.clip((c0 - _ctg_soft(ctg, qx, qy)) / np.maximum(S[rows, kk], 0.3), -1.0, 1.0)
        else:
            dg = np.abs((th - gth + math.pi) % (2 * math.pi) - math.pi)
            g = np.cos(dg)
        score = spd * g + 0.15 * v_cap * g - p["w_turn"] * dv * sp + p["w_free"] * np.minimum(d, H) / H
        if feas.any():
            k = int(np.argmax(np.where(feas, score, -np.inf)))
            v_cmd = float(spd[k])
        else:
            k = int(np.argmax(np.minimum(d, 3.0) + 0.5 * g))
            v_cmd = 0.0
        yaw = float(th[k])
        if p["look"]:
            s_look = float(np.clip(rule.dist(max(v_cmd, sp)) + 1.0, 1.0, S[k, -1]))
            j = int(np.argmax(S[k] >= s_look - 1e-9))
            if math.hypot(PX[k, j] - x, PY[k, j] - y) > 0.5:
                yaw = math.atan2(PY[k, j] - y, PX[k, j] - x)
        if p["pas"] and feas.any():
            self.trace["untrusted_cut"] += int(d_f[k] < d[k] - 0.5)
        return {"theta": float(th[k]), "v_cmd": v_cmd, "d_free_cmd": float(d_feas[k]), "feasible_any": bool(feas.any()),
                "v_rule": float(max(v_cmd, sp)) if v_cmd > 0 else 0.0, "g": float(g[k]), "yaw": yaw}


from .core import TimedOccMap  # noqa: E402


class RangeOccMap(TimedOccMap):
    """TimedOccMap + rmin: the closest range (m) from which each cell was ever observed free (inf if never).
    Identical L / obs / last updates to TimedOccMap, so stacks that ignore rmin behave exactly as on E2's map."""

    def __init__(self, bounds, half_width, cfg=None):
        if cfg is None:
            super().__init__(bounds, half_width)
        else:
            super().__init__(bounds, half_width, cfg)
        self.rmin = np.full((self.nx, self.ny), np.inf, np.float32)

    def clear(self):
        super().clear()
        self.rmin.fill(np.inf)

    def integrate(self, fr, t, ex=0.0, ey=0.0):
        hx, hy = super().integrate(fr, t, ex, ey)
        ox, oy = fr.ox + ex, fr.oy + ey
        cosa, sina = np.cos(fr.angles), np.sin(fr.angles)
        carve = np.minimum(fr.carve, 12.0)
        steps = np.arange(0.0, 12.0, 0.8 * self.res)
        msk = steps[None, :] < carve[:, None]
        sx = np.broadcast_to(steps[None, :], msk.shape)[msk]
        px = (ox + steps[None, :] * cosa[:, None])[msk]
        py = (oy + steps[None, :] * sina[:, None])[msk]
        i = ((px - self.x0) / self.res).astype(np.int64)
        j = ((py - self.y0) / self.res).astype(np.int64)
        ok = (i >= 0) & (i < self.nx) & (j >= 0) & (j < self.ny)
        i, j, sx = i[ok], j[ok], sx[ok].astype(np.float32)
        self.rmin[i, j] = np.minimum(self.rmin[i, j], sx)
        return hx, hy
