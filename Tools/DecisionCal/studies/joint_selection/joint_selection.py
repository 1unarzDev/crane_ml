"""Joint selection of calibration probes and simulation queries (EXPLORATORY; offline; label HSUR).

Reads ONLY stored outputs of the probe-value pilot run without the thrust snap
(~/.cache/decisioncal/probe-value-3131-nosnap: evaluation seed 3131, design seed 3137, both
development seeds). Simulates no boat, flies no mission, touches no frozen M1-M6 output.
Formulation, caveats and results: README.md here.

Two offline instances:

  T  policies a-f at equal total cost on the stored boats. Hypotheses = delay grid x lag grid (21 cells).
     Probe actions are the 7 stored design packages; a probe's outcome is the boat's real stored MAP cell;
     a simulation query (k, cell) returns one of the boat's real stored twin scores in that cell (other
     boats' scores where the boat has no particle there); regret is the boat's real hidden-boat regret. The
     planner's model (cell prior, probe channels, prior score table) is built from the OTHER boats.
  R  the stored boats with their real fits and no emulation: fixed designs, Monte-Carlo (particle-count)
     uncertainty, successive rejection, and stop rules 60 s -> 180 s -> 624 s by decision value (twin EVPI)
     versus posterior entropy, thresholds chosen leave-one-out.

    cd /home/lunarz/worktrees/decisioncal && ulimit -v 1500000 && nice -n 19 python3 -I -W ignore \\
        Tools/DecisionCal/studies/joint_selection/joint_selection.py [--episodes 3] [--minutes 9] [--out DIR]
"""

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

os.environ.setdefault('OMP_NUM_THREADS', '1')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
sys.dont_write_bytecode = True
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
SRC = Path('~/.cache/decisioncal/probe-value-3131-nosnap').expanduser()
ALLOWED_SEEDS = {777, 4242, 5151, 5252, 5353, 6262, 7373, 8484, 9595, 3131, 4141}
GD = [0.0, 0.08, 0.16, 0.25, 0.35, 0.47, 0.62]         # twin.GRID['delay_s']
GT = [0.0, 0.2, 0.45]                                  # twin.GRID['tau_s']
ND, NT = len(GD), len(GT)
H = ND * NT                                            # hypothesis cells, index = i_delay * NT + i_lag
CAP, J_FAIL = 60.0, 300.0
LIBS = ('v2', 'v3')
ATOMS = ('pulse@60', 'reversal@60', 'task@60', 'pulse@180', 'reversal@180', 'task@180', 'dopt@180')
REF = 'full@624'
ENV_CPU_S = 0.0175                                     # measured mission cost per env (probe_value README)
N_DEPLOY = 10                                          # missions the chosen controller will fly (exchange rate)
EPS_ABS = 0.02                                         # s of regret: below this no action is worth taking
BUDGETS = (60, 120, 240)
# boat-second equivalents per simulation query (see README "exchange rate")
REGIMES = {'numpy-online': round(ENV_CPU_S / 3, 4), 'costly-sim': 1.0}


# ------------------------------------------------------------------------------------------------ data
def load():
    man = json.loads((SRC / 'manifest.json').read_text())
    if man['seed'] not in ALLOWED_SEEDS or not man.get('exploratory'):
        raise SystemExit('source is not an exploratory development-seed run; refused')
    rows = []
    for x in (SRC / 'plants.jsonl').read_text().splitlines():
        try:                                           # the pilot may still be appending: skip a partial last line
            rows.append(json.loads(x))
        except json.JSONDecodeError:
            pass
    for r in rows:
        if r['seed'] not in ALLOWED_SEEDS:
            raise SystemExit(f'seed {r["seed"]} is not a development seed; refused')
    rows.sort(key=lambda r: r['id'])
    return rows, man


def cell_of(d, t):
    return GD.index(round(float(d), 5)) * NT + GT.index(round(float(t), 5))


def regret_c(J_h, sel):
    """score.cross_fit_regret capped at 60 s (same rule as the probe-value pilot)."""
    J_h = np.where(np.isfinite(J_h), J_h, J_FAIL)
    K = J_h.shape[1]
    A, B = np.arange(0, K // 2), np.arange(K // 2, 2 * (K // 2))
    out = []
    for a, b in ((A, B), (B, A)):
        o = int(np.argmin(J_h[:, a].mean(1)))
        out.append(J_h[sel, b].mean() - J_h[o, b].mean())
    return float(min(np.mean(out), CAP))


def boot(x, n=4000, seed=0):
    x = np.asarray(x, float)
    idx = np.random.default_rng(seed).integers(0, len(x), (n, len(x)))
    return [float(v) for v in np.quantile(x[idx].mean(1), [0.05, 0.95])]


# ------------------------------------------------------------------------------------------------ instance T
def build_T(rows, lib):
    """Bank of stored particle score columns by (delay, lag) cell, the channels and the cell prior."""
    cells, cols, boat = [], [], []
    for ib, r in enumerate(rows):
        for a, v in r['arms'].items():
            if v['same_as'] or a.startswith('stopwatch') or a == 'rigid@624':
                continue                               # assumed-delay / no-timing arms are not posterior draws
            p = v['particles']
            J = np.asarray(r['libs'][lib]['arms'][a]['J_twin'], float)       # [C, M]
            for m in range(J.shape[1]):
                cells.append(cell_of(p['delay_s'][m], p['tau_s'][m]))
                boat.append(ib)
                cols.append(J[:, m])
    cells, bank, boat = np.array(cells), np.minimum(np.array(cols), J_FAIL), np.array(boat)   # bank [N, C]
    idx = [np.flatnonzero(cells == h) for h in range(H)]
    C = bank.shape[1]
    mu = np.stack([bank[i].mean(0) for i in idx], 1)                         # [C, H] exact cell means
    kstar = mu.argmin(0)
    R = np.stack([np.minimum(bank[idx[h]] - bank[idx[h]][:, [kstar[h]]], CAP).mean(0) for h in range(H)], 1)
    # planner's prior on a NEW boat's table: pooled cell means of the other boats, with the between-boat
    # variance of a boat's own cell mean (v0) and the within-boat-cell variance (sig2), per controller
    dev, within = [], []
    for ib in range(len(rows)):
        for h in range(H):
            j = np.flatnonzero((boat == ib) & (cells == h))
            if len(j) >= 4:
                dev.append((bank[j].mean(0) - mu[:, h]) ** 2 - bank[j].var(0, ddof=1) / len(j))
                within.append(bank[j].var(0, ddof=1))
    sig2 = np.mean(within, 0) + 1.0
    m0, v0 = mu, np.maximum(np.mean(dev, 0), 1.0)
    # channels: MAP cell of the short design given the 624 s MAP cell of the same boat
    ref = np.array([cell_of(r['arms'][REF]['fit']['map']['delay_s'], r['arms'][REF]['fit']['map']['tau_s']) for r in rows])
    prior = np.bincount(ref, minlength=H) + 0.5
    prior = prior / prior.sum()
    edges = np.concatenate([[-np.inf], (np.array(GD[1:]) + np.array(GD[:-1])) / 2, [np.inf]])
    chan, cost, segs, resid = {}, {}, {}, {}
    for a in ATOMS:
        y = np.array([cell_of(r['arms'][a]['fit']['map']['delay_s'], r['arms'][a]['fit']['map']['tau_s']) for r in rows])
        e = np.array(GD)[y // NT] - np.array(GD)[ref // NT]
        b, s = float(e.mean()), float(max(e.std(ddof=1), 0.04))
        Pd = np.stack([np.diff(_Phi((edges - (g + b)) / s)) for g in GD])    # [true delay, observed delay]
        Pl = np.ones((NT, NT))
        for i, j in zip(ref % NT, y % NT):
            Pl[i, j] += 1
        Pl /= Pl.sum(1, keepdims=True)
        P = np.einsum('ab,cd->acbd', Pd, Pl).reshape(H, H)
        chan[a] = P / P.sum(1, keepdims=True)
        cost[a] = float(rows[0]['arms'][a]['seconds'])
        segs[a] = {tuple(k) for k in rows[0]['arms'][a]['design']}
        resid[a] = e
        chan[a + ':meta'] = {'delay_bias': b, 'delay_sd': s, 'lag_hit': float(np.mean(ref % NT == y % NT)),
                             'delay_hit': float(np.mean(ref // NT == y // NT))}
    compat = {a: {b for b in ATOMS if b != a and not (segs[a] & segs[b])} for a in ATOMS}
    return {'bank': bank, 'idx': idx, 'cells': cells, 'boat': boat, 'mu': mu, 'R': R, 'sig2': sig2, 'm0': m0, 'v0': v0, 'prior': prior,
            'chan': chan, 'cost': cost, 'compat': compat, 'resid': resid, 'C': C,
            'occupancy': [int(len(i)) for i in idx]}


def _erf(x):
    s = np.sign(x)
    x = np.abs(x)
    t = 1.0 / (1.0 + 0.3275911 * x)
    y = 1 - (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t - 0.284496736) * t + 0.254829592) * t * np.exp(-x * x)
    return s * y


def _Phi(x):
    return 0.5 * (1 + _erf(np.asarray(x, float) / math.sqrt(2)))


def _f(z):
    return np.exp(-0.5 * z * z) / math.sqrt(2 * math.pi) - z * _Phi(-z)


def entropy(w):
    w = w[w > 0]
    return float(-(w * np.log(w)).sum())


class Episode:
    """One stored pilot boat: its real probe outcomes (stored MAP cells), its own stored twin scores as the
    simulation stream, its hidden-boat regret; and the belief state of a planner whose model (cell prior,
    channels, prior score table) comes from the OTHER boats."""

    def __init__(self, T, own, q, seed):
        self.T, self.own, self.q = T, own, q
        self.y = own['y']
        self.rng = np.random.default_rng(seed)
        self.held, self.cost, self.trace = [], 0.0, []
        self.w = T['prior'].copy()
        self.n = np.zeros((T['C'], H))
        self.s = np.zeros((T['C'], H))

    # -- actions
    def probe(self, a):
        self.held.append(a)
        self.cost += self.T['cost'][a]
        w = self.w * self.T['chan'][a][:, self.y[a]]
        self.w = w / w.sum()
        self.trace.append(a)

    def sim(self, k, h, r=1):
        src = self.own if len(self.own['idx'][h]) else self.T        # no own particle in the cell: pooled others
        j = src['idx'][h][self.rng.integers(0, len(src['idx'][h]), r)]
        x = src['bank'][j, k]
        self.n[k, h] += r
        self.s[k, h] += x.sum()
        self.cost += r * self.q
        if not self.trace or self.trace[-1] != 'sim':
            self.trace.append('sim')
        return x

    def options(self, budget):
        out = []
        for a in ATOMS:
            if a in self.held or self.cost + self.T['cost'][a] > budget + 1e-9:
                continue
            if all(a in self.T['compat'][b] for b in self.held):
                out.append(a)
        return out

    # -- belief
    def post(self):
        T = self.T
        prec = 1 / T['v0'][:, None] + self.n / T['sig2'][:, None]
        return (T['m0'] / T['v0'][:, None] + self.s / T['sig2'][:, None]) / prec, 1 / prec

    def regret(self, k):
        return regret_c(self.own['J_h'], k)


def kg_table(L, w, var, n, sig2, q, rs, room):
    """Knowledge-gradient value of r more draws of (k, h) for each batch size r: value [..., R, C, H] and the
    best (value, cost) by value per cost. L [..., C], w [..., H]; var, n [C, H]."""
    srt = np.sort(L, -1)
    other = np.where(L <= srt[..., :1], srt[..., 1:2], srt[..., :1])         # best of the others
    delta = np.abs(L - other)[..., None]                                     # [..., C, 1]
    best = (np.zeros(L.shape[:-1]), np.ones(L.shape[:-1]), np.full(L.shape[:-1] + (3,), -1))
    ratio_best = np.full(L.shape[:-1], -1.0)
    for r in rs:
        if r * q > room + 1e-12:
            continue
        st = np.sqrt(np.maximum(var - 1 / (1 / var + r / sig2[:, None]), 0))     # [C, H]
        sc = w[..., None, :] * st                                            # [..., C, H]
        v = np.where(sc > 1e-12, sc * _f(delta / np.maximum(sc, 1e-12)), 0.0)
        flat = v.reshape(v.shape[:-2] + (-1,))
        j = flat.argmax(-1)
        val = np.take_along_axis(flat, j[..., None], -1)[..., 0]
        better = val / (r * q) > ratio_best
        ratio_best = np.where(better, val / (r * q), ratio_best)
        best = (np.where(better, val, best[0]), np.where(better, r * q, best[1]),
                np.where(better[..., None], np.stack([np.full(j.shape, r), j // H, j % H], -1), best[2]))
    return best                                                              # value, cost, (r, k, h)


def evsi(mb, w, P):
    """Expected reduction of min_k E[J_k] from one probe channel P [H, Y] (exact)."""
    W = w[:, None] * P
    return float((mb @ w).min() - (mb @ W).min(0).sum()), W


def info_gain(w, P):
    W = w[:, None] * P
    py = W.sum(0)
    return entropy(w) - float(sum(p * entropy(W[:, y] / p) for y, p in enumerate(py) if p > 1e-12))


def run_joint(ep, budget, lookahead=False, n_deploy=None, max_steps=120):
    """(b) greedy by expected regret reduction per boat-second equivalent; (c) adds a two-step lookahead
    (probe then best simulation batch; simulation batch then best probe)."""
    T, q = ep.T, ep.q
    rs = (16, 64, 256) if q < 0.01 else (1, 4, 16)
    for _ in range(max_steps):
        room = budget - ep.cost
        mb, var = ep.post()
        L = mb @ ep.w
        sv, sc, sa = kg_table(L, ep.w, var, ep.n, T['sig2'], q, rs, room)
        cands = []                                                           # (ratio, value, kind, action)
        if sa[0] >= 0:
            cands.append((float(sv / sc), float(sv), 'sim', tuple(int(x) for x in sa)))
        opts = ep.options(budget)
        for a in opts:
            v, W = evsi(mb, ep.w, T['chan'][a])
            c = T['cost'][a]
            cands.append((v / c, v, 'probe', a))
            if lookahead and room - c > q:
                py = W.sum(0)
                ok = py > 1e-9
                Wn = (W[:, ok] / py[ok]).T                                   # [Y, H]
                v2, c2, _ = kg_table(Wn @ mb.T, Wn, var, ep.n, T['sig2'], q, rs, room - c)
                v2t, c2t = float((py[ok] * v2).sum()), float((py[ok] * c2).sum())
                cands.append(((v + v2t) / (c + c2t), v + v2t, 'probe', a))
        if lookahead and sa[0] >= 0 and opts:
            r, k, h = (int(x) for x in sa)
            st = math.sqrt(max(var[k, h] - 1 / (1 / var[k, h] + r / T['sig2'][k]), 0))
            for a in opts:
                c = T['cost'][a]
                if room - r * q < c:
                    continue
                v2 = 0.0
                for z, wz in ((-math.sqrt(3), 1 / 6), (0.0, 2 / 3), (math.sqrt(3), 1 / 6)):
                    mb2 = mb.copy()
                    mb2[k, h] += z * st
                    v2 += wz * evsi(mb2, ep.w, T['chan'][a])[0]
                cands.append(((float(sv) + v2) / (r * q + c), float(sv) + v2, 'sim', (r, k, h)))
        cands = [c for c in cands if c[1] >= EPS_ABS and not (n_deploy and c[0] < 1.0 / n_deploy)]
        if not cands:                                   # nothing left that is worth its cost
            break
        ratio, val, kind, act = max(cands, key=lambda x: x[0])
        if kind == 'probe':
            ep.probe(act)
        else:
            ep.sim(act[1], act[2], act[0])
    mb, _ = ep.post()
    return int((mb @ ep.w).argmin())


def pipeline_sims(ep, budget, cell_fixed=None, cap=4096):
    """Today's simulation stage: particles drawn from the current belief, every controller on each; the pick
    is the lowest plain sample mean (no model). With fewer than 16 affordable queries: random controllers."""
    C = ep.T['C']
    nq = int(min((budget - ep.cost) / ep.q + 1e-9, cap))
    tot, cnt = np.zeros(C), np.zeros(C)
    if nq >= C:
        for _ in range(nq // C):
            h = cell_fixed if cell_fixed is not None else int(ep.rng.choice(H, p=ep.w))
            for k in range(C):
                tot[k] += ep.sim(k, h)[0]
                cnt[k] += 1
    else:
        for k in ep.rng.permutation(C)[:nq]:
            h = cell_fixed if cell_fixed is not None else int(ep.rng.choice(H, p=ep.w))
            tot[k] += ep.sim(int(k), h)[0]
            cnt[k] += 1
    if cnt.sum() == 0:
        return int((ep.T['m0'] @ ep.w).argmin())
    return int(np.where(cnt > 0, tot / np.maximum(cnt, 1), np.inf).argmin())


def fixed_atom(budget, fam='pulse'):
    return f'{fam}@180' if budget >= 180 else f'{fam}@60'


def run_fixed(ep, budget, fam='pulse'):                  # (a) today's pipeline
    a = fixed_atom(budget, fam)
    if ep.T['cost'][a] <= budget:
        ep.probe(a)
    return pipeline_sims(ep, budget)


def run_nominal(ep, budget):                             # (e) no probe, nominal twin (delay 0, lag 0), 16 particles
    return pipeline_sims(ep, budget, cell_fixed=cell_of(0.0, 0.0), cap=16 * ep.T['C'])


def run_entropy(ep, budget):                             # (d) probes by information gain per second, then (a)'s sims
    reserve = min(16 * ep.T['C'] * ep.q, budget / 8)
    while True:
        opts = [a for a in ep.options(budget - reserve)]
        if not opts:
            break
        ig = {a: info_gain(ep.w, ep.T['chan'][a]) for a in opts}
        a = max(opts, key=lambda x: ig[x] / ep.T['cost'][x])
        if ig[a] < 0.05:
            break
        ep.probe(a)
    return pipeline_sims(ep, budget)


def run_reject(ep, budget, max_rounds=64):               # (f) fixed probe, successive rejection of controllers
    a = fixed_atom(budget)
    if ep.T['cost'][a] <= budget:
        ep.probe(a)
    C = ep.T['C']
    alive = np.ones(C, bool)
    xs = [[] for _ in range(C)]
    for rd in range(max_rounds):
        if alive.sum() <= 1 or ep.cost + alive.sum() * ep.q > budget + 1e-9:
            break
        h = int(ep.rng.choice(H, p=ep.w))
        for k in np.flatnonzero(alive):
            xs[k].append(ep.sim(int(k), h)[0])
        if rd >= 2:
            m = np.array([np.mean(x) if x else np.inf for x in xs])
            se = np.array([np.std(x, ddof=1) / math.sqrt(len(x)) if len(x) > 1 else np.inf for x in xs])
            b = int(np.where(alive, m, np.inf).argmin())
            alive &= ~(m - m[b] > 2 * np.sqrt(se ** 2 + se[b] ** 2))
    m = np.array([np.mean(x) if x else np.inf for x in xs])
    if not np.isfinite(m).any():
        return int((ep.T['m0'] @ ep.w).argmin())
    return int(np.where(alive, m, np.inf).argmin())


POLICIES = {'a_fixed_then_all': run_fixed, 'a2_fixed_reversal': lambda ep, B: run_fixed(ep, B, 'reversal'),
            'b_greedy_joint': run_joint,
            'b_stop_N10': lambda ep, B: run_joint(ep, B, n_deploy=N_DEPLOY),
            'c_lookahead2': lambda ep, B: run_joint(ep, B, lookahead=True), 'd_entropy_probes': run_entropy,
            'e_nominal_only': run_nominal, 'f_fixed_reject': run_reject}


def exact_T(T):
    """Exact (no sampling) quantities of the instance with the score table known."""
    mu, R, w = T['mu'], T['R'], T['prior']
    k0 = int((mu @ w).argmin())
    out = {'prior_regret': float(R[k0] @ w), 'prior_pick': k0}
    # expected value of perfect information on one axis (partial EVPI), in capped-regret seconds
    Wd = w.reshape(ND, NT)
    for name, ax in (('delay', 1), ('lag', 0)):
        tot = 0.0
        groups = [np.arange(H).reshape(ND, NT)[i] for i in range(ND)] if ax == 1 else \
                 [np.arange(H).reshape(ND, NT)[:, j] for j in range(NT)]
        for g in groups:
            k = int((mu[:, g] @ w[g]).argmin())
            tot += float(R[k, g] @ w[g])
        out[f'regret_knowing_{name}'] = tot
    out['regret_knowing_cell'] = float(sum(w[h] * R[mu[:, h].argmin(), h] for h in range(H)))
    # the cliff from the stored matrices: act as if the delay were one / two grid levels lower / higher
    for sh in (-2, -1, 1, 2):
        tot, wt = 0.0, 0.0
        for h in range(H):
            i, j = divmod(h, NT)
            if 0 <= i + sh < ND:
                tot += w[h] * R[mu[:, (i + sh) * NT + j].argmin(), h]
                wt += w[h]
        out[f'regret_delay_shift_{sh:+d}_levels'] = tot / wt
    # one-probe designs with the table known: EVSI and information gain per package
    out['packages'] = {}
    for a in ATOMS:
        v, W = evsi(mu, w, T['chan'][a])
        k_y = (mu @ W).argmin(0)
        out['packages'][a] = {'seconds': T['cost'][a], 'regret_after': float(sum((R[k_y[y]] * W[:, y]).sum() for y in range(H))),
                              'evsi_J': v, 'info_gain_nats': info_gain(w, T['chan'][a]),
                              'info_gain_delay_nats': info_gain_axis(w, T['chan'][a], 'delay'),
                              'info_gain_lag_nats': info_gain_axis(w, T['chan'][a], 'lag'), **T['chan'][a + ':meta']}
    return out


def info_gain_axis(w, P, axis):
    """Mutual information between the probe outcome and one axis of the cell."""
    W = (w[:, None] * P).reshape(ND, NT, H)
    Wa = W.sum(1) if axis == 'delay' else W.sum(0)                           # [levels, Y]
    pa, py = Wa.sum(1), Wa.sum(0)
    m = Wa > 1e-15
    return float((Wa[m] * np.log(Wa[m] / (pa[:, None] * py[None])[m])).sum())


def own_of(rows, ib, lib, T_all):
    j = np.flatnonzero(T_all['boat'] == ib)
    r = rows[ib]
    return {'bank': T_all['bank'], 'idx': [j[T_all['cells'][j] == h] for h in range(H)],
            'J_h': np.asarray(r['libs'][lib]['J_h'], float),
            'y': {a: cell_of(r['arms'][a]['fit']['map']['delay_s'], r['arms'][a]['fit']['map']['tau_s']) for a in ATOMS}}


def run_T(rows, n_rep, deadline):
    out, acc, models, tpol = {}, {}, {}, {p: 0.0 for p in POLICIES}
    for lib in LIBS:
        T = build_T(rows, lib)
        models[lib] = (T, [build_T(rows[:i] + rows[i + 1:], lib) for i in range(len(rows))])
        out[lib] = {'exact': exact_T(T), 'occupancy': T['occupancy'], 'prior': [round(float(x), 4) for x in T['prior']],
                    'compat': {a: sorted(T['compat'][a]) for a in ATOMS},
                    'own_cells_per_boat': float(np.mean([sum(len(x) > 0 for x in own_of(rows, i, lib, T)['idx']) for i in range(len(rows))])),
                    'resid_corr_pulse60_reversal60': float(np.corrcoef(T['resid']['pulse@60'], T['resid']['reversal@60'])[0, 1]),
                    'resid_corr_pulse180_reversal180': float(np.corrcoef(T['resid']['pulse@180'], T['resid']['reversal@180'])[0, 1]),
                    'regimes': {}}
    done = 0
    for rp in range(n_rep):
        if time.time() > deadline:
            break
        for lib in LIBS:
            T, loo = models[lib]
            for reg, q in REGIMES.items():
                for B in BUDGETS:
                    for ib in range(len(rows)):
                        own = own_of(rows, ib, lib, T)
                        for p, fn in POLICIES.items():
                            ep = Episode(loo[ib], own, q, [20000 + rp, ib])
                            tp = time.time()
                            k = fn(ep, B)
                            tpol[p] += time.time() - tp
                            d = acc.setdefault((lib, reg, B, p), {x: np.zeros((n_rep, len(rows))) for x in
                                                                 ('regret', 'cost', 'n_sim', 'n_probe')} | {'first': [], 'first_probe': []})
                            d['regret'][rp, ib], d['cost'][rp, ib] = ep.regret(k), ep.cost
                            d['n_sim'][rp, ib], d['n_probe'][rp, ib] = ep.n.sum(), len(ep.held)
                            d['first'].append(ep.trace[0] if ep.trace else 'none')
                            d['first_probe'].append(ep.held[0] if ep.held else 'none')
        done += 1
    cnt = lambda xs: {u: xs.count(u) for u in sorted(set(xs))}
    out['policy_cpu_s'] = tpol
    for (lib, reg, B, p), d in acc.items():
        r = d['regret'][:done].mean(0)                                       # per boat, mean over repetitions
        ra = acc[(lib, reg, B, 'a_fixed_then_all')]['regret'][:done].mean(0)
        ra2 = acc[(lib, reg, B, 'a2_fixed_reversal')]['regret'][:done].mean(0)
        c = d['cost'][:done].mean(0)
        tab = out[lib]['regimes'].setdefault(reg, {'q_boat_s_per_query': REGIMES[reg], 'budgets': {}})['budgets'].setdefault(B, {})
        tab[p] = {'n': len(r), 'reps': done, 'regret': float(r.mean()), 'ci90': boot(r), 'cost': float(c.mean()),
                  'n_sim': float(d['n_sim'][:done].mean()), 'n_probe': float(d['n_probe'][:done].mean()),
                  'diff_vs_a': float((r - ra).mean()), 'diff_ci90': boot(r - ra),
                  'diff_vs_a2': float((r - ra2).mean()), 'diff_a2_ci90': boot(r - ra2),
                  'per_boat_regret': [round(float(x), 3) for x in r],
                  'rep_sd_of_mean': float(d['regret'][:done].mean(1).std(ddof=1)) if done > 1 else None,
                  'net': float(r.mean() + c.mean() / N_DEPLOY),
                  'first_action': cnt(d['first']), 'first_probe': cnt(d['first_probe'])}
    return out


# ------------------------------------------------------------------------------------------------ instance R
def twin_stats(J):
    """Pick, the twin's own expected regret of its pick (EVPI, capped per particle) from a [C, m] matrix."""
    J = np.minimum(np.where(np.isfinite(J), J, J_FAIL), J_FAIL)
    sel = int(J.mean(1).argmin())
    return sel, float(np.minimum(J[sel] - J.min(0), CAP).mean())


def reject_R(J, min_rounds=3):
    """Successive rejection over particles in stored order: every surviving controller is flown on the next
    particle; a controller leaves when its paired mean gap to the leader exceeds 2 standard errors."""
    J = np.minimum(np.where(np.isfinite(J), J, J_FAIL), J_FAIL)
    C, M = J.shape
    alive, used = np.ones(C, bool), 0
    for m in range(M):
        used += int(alive.sum())
        if m + 1 >= min_rounds:
            mean = J[:, :m + 1].mean(1)
            b = int(np.where(alive, mean, np.inf).argmin())
            d = J[:, :m + 1] - J[b, :m + 1]
            se = d.std(1, ddof=1) / math.sqrt(m + 1)
            alive &= ~(d.mean(1) > 2 * se + 1e-9)
            alive[b] = True
            if alive.sum() == 1:
                M = m + 1
                break
    mean = J[:, :M].mean(1)
    return int(np.where(alive, mean, np.inf).argmin()), used


def run_R(rows):
    out = {'n_boats': len(rows), 'strata': {s: sum(r['stratum'] == s for r in rows) for s in sorted({r['stratum'] for r in rows})}}
    chain = ('pulse@60', 'pulse@180', 'full@624')
    secs = {a: float(rows[0]['arms'][a]['seconds']) for a in rows[0]['arms']}
    for lib in LIBS:
        res = {}
        Jh = [np.asarray(r['libs'][lib]['J_h'], float) for r in rows]
        Jt = lambda r, a: np.asarray(r['libs'][lib]['arms'][a]['J_twin'], float)
        # fixed designs (stored picks, all 256 simulations)
        res['fixed'] = {}
        for a in ('nominal', 'passive@60', 'pulse@60', 'reversal@60', 'task@60', 'pulse@180', 'reversal@180',
                  'task@180', 'dopt@180', 'full@624'):
            x = [r['libs'][lib]['arms'][a]['regret_c'] for r in rows]
            res['fixed'][a] = {'seconds': secs[a], 'regret': float(np.mean(x)), 'ci90': boot(x),
                               'dock': float(np.mean([r['libs'][lib]['arms'][a]['dock'] for r in rows]))}
        # Monte-Carlo uncertainty: the pick from the first m particles only
        res['particles'] = {}
        for a in ('pulse@60', 'pulse@180', 'full@624'):
            res['particles'][a] = {}
            for m in (1, 2, 4, 8, 16):
                x = [regret_c(Jh[i], twin_stats(Jt(r, a)[:, :m])[0]) for i, r in enumerate(rows)]
                res['particles'][a][m] = {'queries': 16 * m, 'regret': float(np.mean(x)), 'ci90': boot(x)}
        # (f) successive rejection on the stored matrices
        res['reject'] = {}
        for a in ('pulse@60', 'pulse@180', 'full@624'):
            pk = [reject_R(Jt(r, a)) for r in rows]
            x = np.array([regret_c(Jh[i], k) for i, (k, _) in enumerate(pk)])
            full = np.array([r['libs'][lib]['arms'][a]['regret_c'] for r in rows])
            res['reject'][a] = {'regret': float(x.mean()), 'queries': float(np.mean([u for _, u in pk])),
                                'diff_vs_all256': float((x - full).mean()), 'diff_ci90': boot(x - full),
                                'same_pick': float(np.mean([k == r['libs'][lib]['arms'][a]['sel'] for (k, _), r in zip(pk, rows)]))}
        # sequential stop rules along pulse@60 -> pulse@180 -> full@624
        stat = {'evpi': np.array([[twin_stats(Jt(r, a))[1] for a in chain] for r in rows]),
                'entropy': np.array([[r['arms'][a]['timing']['H_timing'] for a in chain] for r in rows]),
                'random': np.random.default_rng(7).uniform(0, 1, (len(rows), 3))}
        reg = np.array([[r['libs'][lib]['arms'][a]['regret_c'] for a in chain] for r in rows])
        sc = np.array([secs[a] for a in chain])

        def stop_stage(s, thr):
            st = np.zeros(len(rows), int)
            for j in (0, 1):
                st = np.where((st == j) & (s[:, j] > thr), j + 1, st)
            return st
        res['sequential'] = {}
        for ND_ in (10, 100):
          seq = res['sequential'][f'N{ND_}'] = {}
          for name, s in stat.items():
              thrs = np.unique(np.concatenate([[-1e9], np.quantile(s[:, :2], np.linspace(0, 1, 21)), [1e9]]))
              curve = []
              for t in thrs:
                  st = stop_stage(s, t)
                  curve.append([float(t), float(sc[st].mean()), float(reg[np.arange(len(rows)), st].mean())])
              # leave-one-out threshold for the net objective regret + seconds / ND_
              net, used, stops = [], [], []
              for i in range(len(rows)):
                  keep = np.arange(len(rows)) != i
                  best = min(thrs, key=lambda t: (reg[keep, stop_stage(s, t)[keep]] + sc[stop_stage(s, t)[keep]] / ND_).mean())
                  j = int(stop_stage(s[[i]], best)[0])
                  net.append(reg[i, j])
                  used.append(sc[j])
                  stops.append(j)
              seq[name] = {'curve_thr_seconds_regret': curve, 'loo_regret': float(np.mean(net)),
                                         'loo_regret_ci90': boot(net), 'loo_seconds': float(np.mean(used)),
                                         'loo_net': float(np.mean(net) + np.mean(used) / ND_),
                                         'loo_stage_counts': [int(np.sum(np.array(stops) == j)) for j in range(3)],
                                         '_reg': net, '_sec': used}
          for j, a in enumerate(chain):
              seq[f'always_{a}'] = {'loo_regret': float(reg[:, j].mean()), 'loo_seconds': float(sc[j]),
                                                  'loo_net': float(reg[:, j].mean() + sc[j] / ND_)}
          ev, en = seq['evpi'], seq['entropy']
          netv = lambda d_: np.array(d_['_reg']) + np.array(d_['_sec']) / ND_
          for other, vals in (('random', netv(seq['random'])), ('always_pulse@180', reg[:, 1] + sc[1] / ND_),
                              ('always_pulse@60', reg[:, 0] + sc[0] / ND_)):
              dd = netv(ev) - vals
              seq[f'evpi_minus_{other}_net'] = {'mean': float(dd.mean()), 'ci90': boot(dd)}
          dn = (np.array(ev.pop('_reg')) + np.array(ev.pop('_sec')) / ND_) - (np.array(en.pop('_reg')) + np.array(en.pop('_sec')) / ND_)
          seq['random'].pop('_reg'), seq['random'].pop('_sec')
          seq['evpi_minus_entropy_net'] = {'mean': float(dn.mean()), 'ci90': boot(dn)}
        # asymmetry at 60 s: regret and the twin's own EVPI by the sign of the carried-delay error
        err = np.array([r['arms']['pulse@60']['delay_err'] for r in rows])
        bins = {'under (<-0.05 s)': err < -0.05, 'near': np.abs(err) <= 0.05, 'over (>+0.05 s)': err > 0.05}
        res['asymmetry_pulse60'] = {b: {'n': int(m.sum()), 'regret': float(reg[m, 0].mean()) if m.any() else None,
                                        'twin_evpi': float(stat['evpi'][m, 0].mean()) if m.any() else None,
                                        'H_timing': float(stat['entropy'][m, 0].mean()) if m.any() else None,
                                        'gain_from_180': float((reg[m, 0] - reg[m, 1]).mean()) if m.any() else None,
                                        'gain_from_624': float((reg[m, 0] - reg[m, 2]).mean()) if m.any() else None}
                                    for b, m in bins.items()}
        res['corr_regret60'] = {'twin_evpi': float(np.corrcoef(stat['evpi'][:, 0], reg[:, 0])[0, 1]),
                                'H_timing': float(np.corrcoef(stat['entropy'][:, 0], reg[:, 0])[0, 1])}
        out[lib] = res
    return out


# ------------------------------------------------------------------------------------------------ report
def report(res):
    print(f'== joint selection, offline; source {SRC.name} ({res["R"]["n_boats"]} boats {res["R"]["strata"]}); EXPLORATORY, HSUR')
    print('policy cpu s', res['T'].pop('policy_cpu_s'))
    for lib in LIBS:
        ex = res['T'][lib]['exact']
        print(f'\n-- instance T, library {lib} (score table known): capped regret, s')
        print(f'   (pooled table, all boats) prior pick {ex["prior_regret"]:.2f} | knowing delay {ex["regret_knowing_delay"]:.2f} | knowing lag '
              f'{ex["regret_knowing_lag"]:.2f} | knowing the cell {ex["regret_knowing_cell"]:.2f}')
        print('   acting on a delay shifted by levels: ' + '  '.join(f'{k[19:]}: {v:.1f}' for k, v in ex.items() if k.startswith('regret_delay_shift')))
        for a, v in ex['packages'].items():
            print(f'   {a:13s} {v["seconds"]:4.0f}s regret after {v["regret_after"]:5.2f}  EVSI/min {60 * v["evsi_J"] / v["seconds"]:5.2f}  '
                  f'IG {v["info_gain_nats"]:.2f} nats (delay {v["info_gain_delay_nats"]:.2f}, lag {v["info_gain_lag_nats"]:.2f})  '
                  f'delay sd {v["delay_sd"]:.3f} bias {v["delay_bias"]:+.3f}')
        for reg, rr in res['T'][lib]['regimes'].items():
            print(f'   regime {reg} ({rr["q_boat_s_per_query"]} boat-s per query)')
            for B, tab in rr['budgets'].items():
                for p, v in tab.items():
                    fp = max(v['first_probe'], key=v['first_probe'].get)
                    print(f'     B={B:>3} {p:18s} regret {v["regret"]:5.1f} [{v["ci90"][0]:4.1f},{v["ci90"][1]:4.1f}] cost {v["cost"]:6.1f} probes {v["n_probe"]:.2f} '
                          f'sims {v["n_sim"]:6.0f} vs a {v["diff_vs_a"]:+5.1f} [{v["diff_ci90"][0]:+5.1f},{v["diff_ci90"][1]:+5.1f}] vs a2 {v["diff_vs_a2"]:+5.1f} [{v["diff_a2_ci90"][0]:+5.1f},{v["diff_a2_ci90"][1]:+5.1f}] net {v["net"]:5.1f} '
                          f'first {max(v["first_action"], key=v["first_action"].get)} / {json.dumps(v["first_probe"])}')
    for lib in LIBS:
        R = res['R'][lib]
        print(f'\n-- instance R, library {lib}: stored boats')
        print('   fixed: ' + '  '.join(f'{a} {v["regret"]:.1f} [{v["ci90"][0]:.1f},{v["ci90"][1]:.1f}]' for a, v in R['fixed'].items()))
        for a, v in R['particles'].items():
            print(f'   particles {a}: ' + '  '.join(f'm={m}: {x["regret"]:.1f}' for m, x in v.items()))
        for a, v in R['reject'].items():
            print(f'   reject {a}: regret {v["regret"]:.1f} with {v["queries"]:.0f}/256 queries; vs all {v["diff_vs_all256"]:+.2f} '
                  f'[{v["diff_ci90"][0]:+.2f},{v["diff_ci90"][1]:+.2f}]; same pick {v["same_pick"]:.2f}')
        for nd, seq in R['sequential'].items():
            for k, v in seq.items():
                if 'loo_regret' in v:
                    print(f'   {nd} stop rule {k:16s} regret {v["loo_regret"]:5.1f} seconds {v["loo_seconds"]:5.0f} net {v["loo_net"]:5.1f} {v.get("loo_stage_counts", "")}')
            for k, v in seq.items():
                if k.startswith('evpi_minus'):
                    print(f'   {nd} {k} (paired): {v["mean"]:+.1f} [{v["ci90"][0]:+.1f},{v["ci90"][1]:+.1f}]')
        for b, v in R['asymmetry_pulse60'].items():
            print(f'   pulse@60 carried delay {b:17s} n={v["n"]:2d} regret {v["regret"]:5.1f} twin EVPI {v["twin_evpi"]:4.1f} H {v["H_timing"]:.2f} '
                  f'gain from 180 s {v["gain_from_180"]:+5.1f} from 624 s {v["gain_from_624"]:+5.1f}')
        print(f'   corr with realised regret at 60 s: {R["corr_regret60"]}')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--episodes', type=int, default=3, help='simulation-stream repetitions per boat')
    ap.add_argument('--minutes', type=float, default=18.0, help='wall-clock cap for instance T')
    ap.add_argument('--boats', type=int, default=0, help='debug: every n-th boat only')
    ap.add_argument('--out', default=str(HERE / 'out'))
    a = ap.parse_args(argv)
    t0 = time.time()
    rows, man = load()
    rows = rows[::a.boats] if a.boats else rows
    res = {'source': str(SRC), 'source_manifest_seed': man['seed'], 'design_seed': man['design_seed'],
           'boat_ids': [r['id'] for r in rows], 'snap': 'plant.SNAP_FRAC = 0 (no thrust snap)',
           'N_DEPLOY': N_DEPLOY, 'regimes': REGIMES, 'budgets': BUDGETS, 'R': run_R(rows)}
    res['T'] = run_T(rows, a.episodes, t0 + 60 * a.minutes)
    res['elapsed_s'] = time.time() - t0
    out = Path(a.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    (out / 'results.json').write_text(json.dumps(res, indent=1, sort_keys=True, default=float))
    report(res)
    print(f'\n({res["elapsed_s"]:.0f} s)')


if __name__ == '__main__':
    main()
