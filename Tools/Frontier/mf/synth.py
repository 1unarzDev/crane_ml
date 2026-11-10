# SPDX-License-Identifier: GPL-3.0-or-later
"""Synthetic-but-calibrated multi-fidelity tables for sanity-checking selectors (label: SYNTHETIC-CALIBRATED).

Backbone (REAL): per (stack, speed) collision probability p1 and achieved progress a1 of NavPilot E3 held-out (L1), pooled over
families and layouts, linearly interpolated in speed. Everything marked SYNTH is an assumption: the margin knob (3 levels),
the L2 transfer model, and the noise scale. The L1->L2 link is NOT measured (no paired data), so scenarios span it.

Per-run utility (higher is better, m/s-equivalent): y = a + N(0, sd_a) if no collision, else -C (C = 8, SYNTH: a collision costs
8 m/s of progress, so the optimum is an interior speed near E3's 5% collision knee). True mean f(x) = (1 - p) a - C p.  L2 truth: logit p2 = rho * logit p1 + beta + delta(x); a2 = kappa * a1.
delta(x) = stack_offset[s] * g(v) + idio(x) * g(v), g(v) = speed gain: grows above `v0` (speed-divergent) or constant.
The level calibration beta: observed NavPilot-vs-.NET collision level gap at ~7 m/s (L2/L1 = 0.62, CIs wide; RETRO.md section 3).
"""
import numpy as np

STACKS = ["opt_backup_t", "reactive_plus", "mpc_lc", "ua_map", "pas_full"]
SPEEDS = [2.5, 3.25, 4.0, 4.75, 5.5, 6.25, 7.0, 8.0, 9.0]
MARGINS = [0, 1, 2]

SCENARIOS = {
    # name: rho, sigma_stack, sigma_idio, divergence start v0 (None = constant over speed)
    "faithful":       dict(rho=1.0, s_stack=0.15, s_idio=0.15, v0=None),
    "moderate":       dict(rho=0.8, s_stack=0.50, s_idio=0.40, v0=None),
    "speed_divergent": dict(rho=0.8, s_stack=0.90, s_idio=0.70, v0=5.0),   # H1: ordering degrades at high speed
    "unfaithful":     dict(rho=0.4, s_stack=1.20, s_idio=1.00, v0=None),
}


def logit(p):
    p = np.clip(p, 1e-3, 1 - 1e-3)
    return np.log(p / (1 - p))


def sigmoid(z):
    return 1 / (1 + np.exp(-z))


def e3_backbone(e3_rows):
    """Real E3 cell stats: dict[(stack, speed)] = (p_collision, a_progress_noncollision, sd_a)."""
    out = {}
    for s in STACKS:
        for v in sorted({r["speed"] for r in e3_rows}):
            rr = [r for r in e3_rows if r["config"] == s and r["speed"] == v]
            ok = [r["progress"] for r in rr if not r["collision"]]
            out[(s, v)] = (float(np.mean([r["collision"] for r in rr])), float(np.mean(ok)) if ok else 0.0,
                           float(np.std(ok)) if len(ok) > 1 else 1.0)
    return out


def build_configs(bb):
    vs = sorted({v for (_, v) in bb})
    cfgs, p1, a1, sd = [], [], [], []
    for si, s in enumerate(STACKS):
        pv = [bb[(s, v)][0] for v in vs]
        av = [bb[(s, v)][1] for v in vs]
        for v in SPEEDS:
            for m in MARGINS:
                p = np.interp(v, vs, pv)
                a = np.interp(v, vs, av)
                lp = logit(max(p, 0.004)) - 0.4 * m               # SYNTH margin effect
                cfgs.append((s, v, m))
                p1.append(float(sigmoid(lp)))
                a1.append(float(a * (1 - 0.04 * m)))
                sd.append(float(np.mean([bb[(s, vv)][2] for vv in vs])))
    return cfgs, np.array(p1), np.array(a1), np.array(sd)


def features(cfgs):
    """Simple config features for the Bayesian linear model (no knowledge of the scenario's deltas)."""
    X = []
    for s, v, m in cfgs:
        oh = np.zeros(len(STACKS))
        oh[STACKS.index(s)] = 1
        z = (v - 5.5) / 3.0
        X.append(np.concatenate([oh, [z, z * z, m, z * m], oh * z]))
    return np.array(X)


def build_truth(cfgs, p1, a1, scenario, seed, level_beta=-0.35):
    sc = SCENARIOS[scenario]
    rng = np.random.default_rng([seed, 0x51])
    off = rng.normal(0, sc["s_stack"], len(STACKS))
    v = np.array([c[1] for c in cfgs])
    g = np.ones_like(v) if sc["v0"] is None else np.clip((v - sc["v0"]) / 3.0, 0.0, 1.2) + 0.05
    stack_idx = np.array([STACKS.index(c[0]) for c in cfgs])
    delta = off[stack_idx] * g + rng.normal(0, sc["s_idio"], len(v)) * g
    p2 = sigmoid(sc["rho"] * logit(p1) + level_beta + delta)
    a2 = 0.9 * a1
    return p2, a2


def kendall(a, b):
    a, b = np.asarray(a), np.asarray(b)
    da, db = np.sign(a[:, None] - a[None, :]), np.sign(b[:, None] - b[None, :])
    iu = np.triu_indices(len(a), 1)
    c = (da * db)[iu]
    den = np.sqrt((da[iu] != 0).sum() * (db[iu] != 0).sum())
    return float(c.sum() / den) if den else float("nan")


C_COLL = 8.0


def utility(p, a):
    return (1 - p) * a - C_COLL * p


class Table:
    """Cached outcomes: pools[(i, fid)] arrays drawn once per replication; truth means for regret."""

    def __init__(self, cfgs, p1, a1, sd1, p2, a2, pool_l1, pool_l2, seed, noise_scale=1.0):
        self.cfgs, self.N = cfgs, len(cfgs)
        self.f1, self.f2 = utility(p1, a1), utility(p2, a2)
        rng = np.random.default_rng([seed, 0xB0B])
        self.pools = {}
        for i in range(self.N):
            for fid, p, a, n in (("L1", p1, a1, pool_l1), ("L2", p2, a2, pool_l2)):
                coll = rng.random(n) < p[i]
                self.pools[(i, fid)] = np.where(coll, -C_COLL, a[i] + rng.normal(0, sd1[i] * noise_scale, n))
        self.features = features(cfgs)
