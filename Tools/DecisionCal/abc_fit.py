"""Sample-based (ABC / BayesSim-style) calibration of the twin from the shared open-loop probe log.

A second, independent estimator next to ``twin.fit`` (integral-regression NIG blocks + grid evidence):
no regression, no velocity channel. Proposals are drawn from the ``twin.BROAD`` ranges for exactly the
parameters an arm frees (``twin.FREE``), each proposal is simulated open loop on all 24 probe segments
with ``plant.Plant`` (sensor noise off, the segment's drift current, the segment's commands) and the
simulated *measurements* are compared with the logged ones: position relative to the start of the fit
window and heading change, sampled on the segment grid over ``meta['fit_window']``. Odometry latency
(``odom_frac * delay_s``) is applied as a measurement delay, as in the log. The SG velocities are not used.

Weights. Per proposal, D = sum_segments f(a), a = rms_pos^2 / SIGMA_POS^2 + rms_head^2 / SIGMA_HEAD^2 and
f(a) = a / (1 + a / SEG_CAP) (soft cap: at full thrust the boat is yaw-unstable, so one degree of initial heading or
current error gives a metre of divergence even for the true parameters; such segments must not dominate); the
weight is exp(-D / (2 T)). T >= 1 is the smallest tempering scale (bisection in log T) for which the
effective sample size 1 / sum(w^2) of the proposals is >= ess_target = min(max(8, n / 4), n_prop / 2); T = 1
is the untempered Gaussian likelihood. Then ``n`` particles are resampled with replacement (seeded).
SIGMA_POS/SIGMA_HEAD are the nominal measurement noise levels (pos_sd, yaw_sd of twin.SENSORS, doubled
because both the data and the reference sample carry noise, rounded to 0.1 m / 0.02 rad).
Parameters an arm does not free stay at twin.NOMINAL; odom_frac ~ U(0, 1) if 'delay_s' is free, else 0.

Budget: n_prop proposals x 24 segments open-loop simulations (default 512 x 24 = 12288), identical for
every arm, each run to the end of the longest fit window. Not defined for 'nominal', 'nominal_d100', 'broad'
(nothing to fit); use twin.fit for them.

Note: this file is named abc.py; import it by path or with a package-qualified name, ``import abc`` inside
a plain interpreter gives the standard library module.
"""

import warnings

import numpy as np

import _paths  # noqa: F401
import twin
from plant import DT, Plant, PlantSpec

SIGMA_POS, SIGMA_HEAD = 0.1, 0.02
SEG_CAP = 50.0            # soft cap of one segment's chi^2: a/(1 + a/CAP); yaw-unstable segments are chaotic
T0 = 1.0                  # run time of segment t = 0 (probes.generate: the grid starts at 1.0 s)
CHUNK = 48                # proposals per simulated batch (x 24 segments)
NO_FIT = ('nominal', 'nominal_d100', 'broad')


def _free_names(arm):
    grid_free, nig_free, _ = twin.FREE[arm]
    names = list(grid_free)
    for g in nig_free:
        if g == 'm':
            names += [b[0] for b in twin.BLOCKS.values()]
        elif g == 'd':
            names += [k for b in twin.BLOCKS.values() for k in b[1:]]
    return names, 'delay_s' in grid_free


def propose(arm, n_prop, rng):
    """Particle dict [n_prop] drawn from twin.BROAD for the arm's free parameters (others at NOMINAL)."""
    names, timing = _free_names(arm)
    p = twin.nominal_particles(n_prop)
    for k in twin.NOMINAL:                       # fixed key order -> deterministic stream
        if k in names:
            if k in twin.GRID:
                p[k] = rng.uniform(*twin.BROAD[k], n_prop)
            else:
                p[k] = twin.NOMINAL[k] * rng.uniform(*twin.BROAD['m' if k in ('m11', 'm22', 'm66') else 'd'], n_prop)
        elif k == 'odom_frac' and timing:
            p[k] = rng.uniform(*twin.BROAD['odom_frac'], n_prop)
    return p


class _Data:
    """Per-segment measurement targets, commands and sampling indices."""

    def __init__(self, evidence):
        segs = evidence.segments
        self.S = len(segs)
        self.k_end = 0
        self.q = []
        self.win, self.pos, self.psi = [], [], []
        self.psi0 = np.array([np.unwrap(s.x[:3, 2]).mean() for s in segs])
        self.cur = np.array([s.meta['current'] for s in segs], float)
        for s in segs:
            w0, w1 = s.meta['fit_window']
            j = np.flatnonzero((s.t >= w0 - 1e-9) & (s.t <= w1 + 1e-9))
            ref = slice(0, 3)
            p = s.x[j, :2]
            h = np.unwrap(s.x[j, 2])
            self.pos.append(p - p[ref].mean(0))
            self.psi.append(h - h[ref].mean())
            self.win.append(np.round((s.t[j] + T0) / DT).astype(int))       # tick of every compared sample
            self.q.append(s.u)
        self.dt_grid = float(np.median(np.diff(segs[0].t)))
        self.n_ticks = int(max(w.max() for w in self.win))
        self.k0 = int(min(w.min() for w in self.win))

    def command(self, k):
        """q [S, 4] applied by the plant at tick k (time k DT): ZOH of the logged commands, neutral before the log."""
        out = np.zeros((self.S, 4))
        for i, u in enumerate(self.q):
            if k * DT >= T0 - 1e-9:
                out[i] = u[min(int(np.floor((k * DT - T0) / self.dt_grid + 1e-6)), len(u) - 1)]
        return out


def _simulate(data, part, idx):
    """Measured (pos, heading) of proposals ``idx`` on every segment, at ticks win_i:
    list over segments of (pos [P, n_i, 2], psi [P, n_i])."""
    P, S = len(idx), data.S
    sub = {k: v[idx] for k, v in part.items()}
    quiet = {k: 0.0 for k in twin.SENSORS}
    spec = twin.to_spec(sub, sensors=quiet)
    spec = spec.repeat(S)                                          # env = proposal * S + segment
    spec = spec.with_(current=np.tile(data.cur, (P, 1)))
    st0 = np.zeros((P * S, 6))
    st0[:, 2] = np.tile(data.psi0, P)
    plant = Plant(spec, np.arange(P * S, dtype=np.uint64), state0=st0, t_max=data.n_ticks * DT)
    rec_ticks = np.unique(np.concatenate(data.win))
    pos = np.full((len(rec_ticks), P * S, 3), np.nan)
    where = {int(k): i for i, k in enumerate(rec_ticks)}
    with warnings.catch_warnings(), np.errstate(all='ignore'):
        warnings.simplefilter('ignore')
        for k in range(1, data.n_ticks + 1):
            plant.step(np.tile(data.command(k - 1), (P, 1)))
            if k in where:
                pos[where[k]] = plant.delayed[:, :3]
    pos = pos.reshape(len(rec_ticks), P, S, 3)
    out = []
    for i, w in enumerate(data.win):
        r = np.searchsorted(rec_ticks, w)
        x = pos[r, :, i]                                           # [n_i, P, 3]
        h = np.unwrap(x[..., 2], axis=0)
        out.append((x[..., :2].transpose(1, 0, 2), h.T))
    return out


def discrepancy(data, part, n_prop):
    """(D, rms_pos [n_prop, S], rms_head [n_prop, S]) of every proposal."""
    rp = np.empty((n_prop, data.S))
    rh = np.empty((n_prop, data.S))
    for c in range(0, n_prop, CHUNK):
        idx = np.arange(c, min(c + CHUNK, n_prop))
        sim = _simulate(data, part, idx)
        for i, (p, h) in enumerate(sim):
            p = p - p[:, :3].mean(1, keepdims=True)
            h = h - h[:, :3].mean(1, keepdims=True)
            rp[idx, i] = np.sqrt(((p - data.pos[i][None]) ** 2).sum(-1).mean(1))
            rh[idx, i] = np.sqrt(((h - data.psi[i][None]) ** 2).mean(1))
    rp, rh = np.where(np.isfinite(rp), rp, 1e6), np.where(np.isfinite(rh), rh, 1e6)
    a = rp ** 2 / SIGMA_POS ** 2 + rh ** 2 / SIGMA_HEAD ** 2
    return (a / (1.0 + a / SEG_CAP)).sum(1), rp, rh


def _weights(D, T):
    w = np.exp(-(D - D.min()) / (2.0 * T))
    return w / w.sum()


def temper(D, ess_target):
    """Smallest T >= 1 with ESS(exp(-D / 2T)) >= ess_target (bisection in log T)."""
    ess = lambda T: 1.0 / np.sum(_weights(D, T) ** 2)
    if ess(1.0) >= ess_target:
        return 1.0
    lo, hi = 0.0, np.log(max(float(D.max() - D.min()), 2.0)) + 5.0
    while ess(np.exp(hi)) < ess_target and hi < 60:
        hi += 5.0
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if ess(np.exp(mid)) < ess_target else (lo, mid)
    return float(np.exp(hi))


def fit(evidence, arm, n, seed, n_prop=512):
    """ABC posterior particles (dict over every twin.NOMINAL key, arrays [n]) and a report."""
    if arm in NO_FIT or arm not in twin.FREE:
        raise ValueError(f'abc.fit: arm {arm!r} has no free parameters to fit')
    rng = np.random.default_rng(seed)
    part = propose(arm, n_prop, rng)
    data = _Data(evidence)
    D, rp, rh = discrepancy(data, part, n_prop)
    ess_target = min(max(8.0, n / 4.0), n_prop / 2.0)
    T = temper(D, ess_target)
    w = _weights(D, T)
    pick = rng.choice(n_prop, size=n, p=w)
    out = {k: np.asarray(v, float)[pick] for k, v in part.items()}
    best = int(np.argmin(D))
    err = np.median(rp, 1)                                  # per proposal: median over segments of RMS position error
    report = {'arm': arm, 'ess': float(1.0 / np.sum(w ** 2)), 'ess_target': ess_target, 'temper': T,
              'n_prop': n_prop, 'n_sims': n_prop * data.S, 'best': {k: float(v[best]) for k, v in part.items()},
              'best_D': float(D[best]), 'prior_median_pos_err': float(np.median(err)),
              'post_median_pos_err': float(np.median(err[pick])), 'free': _free_names(arm)[0]}
    return out, report
