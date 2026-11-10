# SPDX-License-Identifier: GPL-3.0-or-later
"""D3 compute-aware stacks: a joint (speed, compute-level) policy that uses the measured latency tail.

Compute levels c in {0, 1, 2} trade latency for free-space quality (LEVELS). tau(c): the planner body time scales with
the work done (primitive headings, map frames integrated per cycle), the Pareto stall part (GPU/memory contention,
the part that drives the p99) scales less, and the person-detector rate (GPU share rho) drops with c. q(c): n_headings
and frames integrated really change the planner (fewer candidate headings, less map evidence), so quality loss is
measured by closed-loop episodes, not assumed; the extra clearance e_c is an additional prior margin [U, tuned on DEV].
Grounding and honest limits: results/D3_dev.md section "Compute-level model".

Policies (AdvStack feature set + governor; all keep the E2/E3 safety layer, i.e. the stall watchdog in the episode):
  clear_only   opt_backup_t with tau = 0 (clearance + braking envelope only; ignores measured latency)
  cc_opt       analytic chance-constrained governor on opt_backup_t: choose (command, level) so that
               P(d_free < v tau + D_brake(v) + m + e_c) <= delta, tau = (1-delta) quantile of (frame staleness + F_c)
               with F_c the empirical per-level compute distribution (cold start: level-0 empirical scaled)
  cc_pas       the same on pas_full's belief/safety features
  lp_pas       learned level/delta policy (softmax heads over 5 features; weights from CEM, experiments/d3.py train)
  fix_l1/fix_l2  pas_full pinned to level 1 / 2 (compute-only ablations)
Velocities (m/s) and accelerations (m/s^2) only; no force gains here.
"""
import math
from collections import deque

import numpy as np

from navpilot import sensors as S

from .advanced import ADV_DEFAULTS, METHODS, AdvStack, AdvRule, _dilate_coarse  # noqa: F401
from .advanced import _free_along_t, _tiers
from .core import _ctg_soft, lookup_blocked, primitive_paths, window  # noqa: F401

# s_body: planner body-time multiplier; s_stall: Pareto stall multiplier; frames: map frames integrated per sensor;
# det_scale: detector rate multiplier (GPU share rho).  s_body from a wall-time microbench of CCStack.cycle (fix_l0/l1/l2 on S0, 2 layouts, 600 cycles each,
# loaded host: mean 12.5 / 14.9 / 8.3 ms; level 2 = 0.66x measured, level 1 interpolated because its measurement was noise); s_stall assumed [U].
LEVELS = [
    {"nh": 48, "frames": 3, "s_body": 1.00, "s_stall": 1.00, "det_scale": 1.00},
    {"nh": 24, "frames": 2, "s_body": 0.83, "s_stall": 0.80, "det_scale": 0.50},
    {"nh": 12, "frames": 1, "s_body": 0.66, "s_stall": 0.60, "det_scale": 0.25},
]

D3_METHODS = {
    "clear_only": {"base": "opt_backup_t", "gov": "clear"},
    "cc_opt": {"base": "opt_backup_t", "gov": "cc"},
    "cc_pas": {"base": "pas_full", "gov": "cc"},
    "lp_pas": {"base": "pas_full", "gov": "lp"},
    "p99_nb": {"base": "opt_backup_t", "gov": "rule", "backup": False},   # p99 rule, no known-free backup (optimistic belief only)
    "cc_nb": {"base": "opt_backup_t", "gov": "cc", "backup": False},
    "rr_pas": {"base": "pas_full", "gov": "rr"},                          # RoboRun-style: environment-driven level, p99 speed cap
    "fix_l0": {"base": "pas_full", "gov": "fix0"},
    "fix_l1": {"base": "pas_full", "gov": "fix1"},
    "fix_l2": {"base": "pas_full", "gov": "fix2"},
}
D3_DEFAULTS = {"delta": 0.01, "e_q": [0.0, 0.3, 0.8], "lam": [0.0, 0.05, 0.12], "hold": 0, "w": None}
DELTAS = [0.1, 0.01, 0.001]
N_FEAT = 5


def level_compute(tail, lev, u1, u2, u3):
    """Compute time of a planner cycle at level `lev`: scaled body + scaled stall (== S.compute_time at level 0)."""
    c = S.LATENCY_TAILS[tail]
    z = math.sqrt(2.0) * S._erfinv(2.0 * u1 - 1.0)
    body = c["median_s"] * math.exp(c["sigma"] * z)
    stall = c["stall_min_s"] * max(u3, 1e-12) ** (-1.0 / c["alpha"]) if u2 < c["p_stall"] else 0.0
    L = LEVELS[lev]
    return min(L["s_body"] * body + L["s_stall"] * stall, 3.0) if lev else min(body + stall, 3.0)


class _ClearRule(AdvRule):
    def observe_age(self, age):
        pass

    def dist(self, v, extra_margin=0.0):
        v = np.asarray(v, float)
        return np.interp(v, self.vs, self.ds) + self.margin + extra_margin


class CCStack(AdvStack):
    def __init__(self, name, veh, params=None):
        spec = D3_METHODS[name]
        pr = dict(params or {})
        self.g = dict(D3_DEFAULTS)
        for k in list(pr):
            if k in D3_DEFAULTS:
                self.g[k] = pr.pop(k)
        super().__init__(spec["base"], veh, pr)
        self.name = name
        self.gov = spec["gov"]
        self.backup = spec.get("backup", True)
        if self.gov == "clear":
            self.rule = _ClearRule(veh, margin=self.p["margin"])
        self.level = {"fix0": 0, "fix1": 1, "fix2": 2}.get(self.gov, 0)
        self.stale = deque(maxlen=100)
        self.cl = [deque(maxlen=100) for _ in LEVELS]
        self.rng = np.random.default_rng(7)
        self.n_dec = 0

    # ---- measured latency distribution F_c
    def observe_compute(self, c, lev=0):
        self.comps.append(c)
        self.cl[lev].append(c)

    def observe_age_split(self, age, c):
        self.stale.append(max(age - c, 0.0))

    def _comp_samples(self, lev):
        own = self.cl[lev]
        if len(own) >= 30:
            return np.asarray(own)
        base = np.asarray(self.cl[0]) if len(self.cl[0]) >= 10 else np.array([0.03, 0.035, 0.04, 0.06, 0.1])
        if lev == 0:
            return base
        L = LEVELS[lev]
        q90 = np.quantile(base, 0.9)
        return np.where(base > q90, base * L["s_stall"], base * L["s_body"])

    def tau(self, lev, delta):
        st = np.asarray(self.stale) if len(self.stale) >= 10 else np.array([0.06, 0.08, 0.1, 0.12])
        cs = self._comp_samples(lev)
        if len(st) > 40:
            st = self.rng.choice(st, 40, replace=False)
        if len(cs) > 40:
            cs = np.sort(cs)[np.linspace(0, len(cs) - 1, 40).astype(int)]
        return float(np.quantile((st[:, None] + cs[None, :]).ravel(), 1.0 - delta))

    def cycle(self, *a, **k):
        self.p["n_headings"] = LEVELS[self.level]["nh"]
        return super().cycle(*a, **k)

    def _dist_level(self, v, lev, delta):
        sub = getattr(self.rule, "sub", 0.0)
        tau = max(self.tau(lev, delta) - sub, 0.06)
        return v * tau + np.interp(v, self.rule.vs, self.rule.ds) + self.rule.margin + self.g["e_q"][lev]

    # ---- policy evaluation (a copy of AdvStack._evaluate up to d_feas; command/level choice differs)
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
        th, spd, PX, PY, S_ = primitive_paths(x, y, vx, vy, thetas, speeds, veh, H)
        rows = np.arange(len(th))
        k0, ksl = p["tighten_k0"], p["tighten_ks"]
        tight = k0 > 0 or ksl > 0
        kmax = min(int(math.ceil((k0 + ksl * (2 * H)) / m.res)) if tight else 0, 4)
        if p["optimistic"]:
            win_s = window(b, m, x, y, r_infl, wm, unknown_free=True)
            d = _free_along_t(m, win_s, None, th, PX, PY, S_, H, x, y)
            win_f = window(b_safe, m, x, y, r_infl, wm)
        else:
            win_f = window(b, m, x, y, r_infl, wm)
            d = None
        win_f0 = win_f
        if p["pas"]:
            u = self._untrusted(m, win_f)
            win_f = (win_f[0], win_f[1] | u, win_f[2], win_f[3], win_f[4])
        tiers = _tiers(win_f, kmax) if tight and kmax > 0 else None
        d_f = _free_along_t(m, win_f, tiers, th, PX, PY, S_, H, x, y, k0, ksl)
        if d is None:
            d = _free_along_t(m, win_f0, None, th, PX, PY, S_, H, x, y) if (p["pas"] or tiers is not None) else d_f
        d_feas = np.minimum(d, d_f) if self.backup else d.copy()
        v_rule = np.maximum(spd, sp)
        dv = np.abs((th - vth + math.pi) % (2 * math.pi) - math.pi)
        if ctg is not None:
            se = np.maximum(np.minimum(np.minimum(d, 4.0), S_[:, -1]), 0.3)
            kk = np.argmax(S_ >= se[:, None], axis=1)
            qx, qy = PX[rows, kk], PY[rows, kk]
            c0 = float(_ctg_soft(ctg, x, y))
            g = np.clip((c0 - _ctg_soft(ctg, qx, qy)) / np.maximum(S_[rows, kk], 0.3), -1.0, 1.0)
        else:
            dg = np.abs((th - gth + math.pi) % (2 * math.pi) - math.pi)
            g = np.cos(dg)
        score = spd * g + 0.15 * v_cap * g - p["w_turn"] * dv * sp + p["w_free"] * np.minimum(d, H) / H
        # ---- per-level feasibility and value
        g_ = self.g
        if self.gov == "clear":
            feas_l = [rule.dist(v_rule) <= d_feas] * 3
            lev_next, delta = 0, 0.01
        elif self.gov == "rule":
            feas_l = [rule.dist(v_rule) <= d_feas] * 3
            lev_next, delta = 0, 0.01
        elif self.gov == "rr":
            # RoboRun-style: pick the compute level from the environment (clutter = share of headings with < 6 m
            # of free path), then apply the p99 speed cap at that level's latency tail; speed does not pick the level
            lev_next = 0 if float(np.mean(d_feas < 6.0)) > 0.3 else 2
            delta = 0.01
            feas_l = [self._dist_level(v_rule, lv, delta) <= d_feas for lv in range(3)]
        elif self.gov.startswith("fix"):
            lev_next, delta = self.level, g_["delta"]
            feas_l = [self._dist_level(v_rule, lv, delta) <= d_feas for lv in range(3)]
        else:
            feas_l, vals, delta = [], [], g_["delta"]
            feas0 = None
            if self.gov == "lp":
                f0 = self._dist_level(v_rule, 0, 0.01) <= d_feas
                phi = np.array([1.0, float(f0.mean()), float(min(d_feas.max(), H) / H), self.tau(0, 0.01), float(self.level) / 2.0])
                w = np.asarray(g_["w"], float).reshape(2, 3, N_FEAT) if g_["w"] is not None else np.zeros((2, 3, N_FEAT))
                lg = w[0] @ phi
                lg[0] += 0.5   # default prior: level 0
                lev_pick = int(np.argmax(lg))
                dl = w[1] @ phi
                dl[1] += 0.5
                delta = DELTAS[int(np.argmax(dl))]
            for lv in range(3):
                f = self._dist_level(v_rule, lv, delta) <= d_feas
                feas_l.append(f)
                vals.append(float(np.max(np.where(f, score, -np.inf))) - g_["lam"][lv] if f.any() else -1e9)
            if self.gov == "lp":
                lev_next = lev_pick
            else:
                lev_next = int(np.argmax(vals))
                if max(vals) <= -1e8:
                    lev_next = 2      # nothing feasible anywhere: shortest latency to re-plan
        feas = feas_l[lev_next]
        if feas.any():
            k = int(np.argmax(np.where(feas, score, -np.inf)))
            v_cmd = float(spd[k])
        else:
            k = int(np.argmax(np.minimum(d, 3.0) + 0.5 * g))
            v_cmd = 0.0
        yaw = float(th[k])
        if p["look"]:
            s_look = float(np.clip(rule.dist(max(v_cmd, sp)) + 1.0, 1.0, S_[k, -1]))
            j = int(np.argmax(S_[k] >= s_look - 1e-9))
            if math.hypot(PX[k, j] - x, PY[k, j] - y) > 0.5:
                yaw = math.atan2(PY[k, j] - y, PX[k, j] - x)
        if p["pas"] and feas.any():
            self.trace["untrusted_cut"] += int(d_f[k] < d[k] - 0.5)
        self.level = lev_next
        self.n_dec += 1
        return {"theta": float(th[k]), "v_cmd": v_cmd, "d_free_cmd": float(d_feas[k]), "feasible_any": bool(feas.any()),
                "v_rule": float(max(v_cmd, sp)) if v_cmd > 0 else 0.0, "g": float(g[k]), "yaw": yaw,
                "level_next": lev_next}

