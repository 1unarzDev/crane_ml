"""Twin family, the channel ablations and their fits from one shared probe log.

Family (a strict sub-family of the hidden boats, see ``plant.py``): plantcard ``Fossen3``
parameters (effective m_jj, d1, d2 per axis; gauge k_ref fixed) plus a thruster bank with lumped
delay, one lag ``tau_s`` (up = down), a reversal hold ``rev_hold_s``, a reverse/forward ratio and a
deadband, identical thrusters, gamma = 2. The lumped delay is split into dead time and odometry
latency by ``odom_frac``, which no stamp-aligned log identifies (ID-07): fitted twins draw it from
its prior.

Identification is plantcard's integral regression (``Fossen3._base`` rows, NIG blocks, a grid
posterior from the closed-form block evidence), vectorised over the grid: the regressors Phi do
not depend on the grid point, only the integrated thrust y does, so every grid point shares one
posterior precision and the evidence is one batched solve.

Channel ablations (identical log and particle count for every arm; a channel an arm does not
fit stays at the prior card's nominal value):

| arm       | grid axes free             | NIG                       | rows   |
| nominal   | -                          | -                         | -      |
| broad     | - (wide uniform draws)     | - (wide uniform draws)    | -      |
| steady    | rev_ratio, deadband_q      | d1, d2 (m pinned)         | steady |
| timing    | delay_s                    | - (evidence only)         | all    |
| transient | tau_s, rev_hold_s          | m (d1, d2 pinned)         | all    |
| joint     | all                        | all                       | all    |
"""

import itertools
import json
import math
from pathlib import Path

import numpy as np

import _paths
from plant import B as B_GEOM, DT, N_THR, PlantSpec, ThrusterState, static_thrust
from plantcard.blocks.nig import NIG
from plantcard.marine.fieldtrials import sg_velocity
from plantcard.marine.fossen3 import Fossen3, profile_config, profile_params

PROFILE = _paths.CRANE_ROOT / 'Tools' / 'Characterization' / 'profiles' / 'characterized.json'
GRID = {'delay_s': [0.0, 0.08, 0.16, 0.25, 0.35, 0.47, 0.62], 'tau_s': [0.0, 0.2, 0.45],
        'rev_hold_s': [0.0, 0.2, 0.5], 'rev_ratio': [0.6, 0.75, 0.9, 1.0], 'deadband_q': [0.0, 0.04]}
BLOCKS = {'surge': ['m11', 'd1_u', 'd2_u'], 'sway': ['m22', 'd1_v', 'd2_v'], 'yaw': ['m66', 'd1_r', 'd2_r']}
# lower bounds on effective inertia: half the rigid-body value (effective = physical / thrust scale,
# so it can only fall below the rigid body if thrust were stronger than the profile's). Damping >= 0.
# M0 v1 used plantcard's (5, 5, 0.2): yaw draws near 0.2 kg m^2 made the twin stiff at dt = 0.02 and
# 3 of 48 boats had diverging particles (amendment A1, 2026-11-09).
BOUNDS = {'m11': 15.0, 'm22': 15.0, 'm66': 2.05}
REL_STD, NOISE_STD = 0.5, 2.0                         # plantcard seed prior strength (NLC)
EXCITES = Fossen3.EXCITES
STEADY_DNU = 0.05                                     # |delta nu| over a 1 s window counted as steady
ARMS = ('nominal', 'nominal_d100', 'broad', 'steady', 'timing', 'transient', 'joint',
        'joint-steady', 'joint-timing', 'joint-transient', 'rigid', 'rigid+delay', 'rigid+lag')
# (grid axes free, NIG groups free, rows). 'steady' rows = windows with |delta nu| < STEADY_DNU.
#
# Ladder arms (M1, after REVIEW_verifier_wave1): the hidden boats differ from the prior card mostly in
# thrust scale, i.e. in effective inertia and damping, so an arm that pins inertia at the prior is too
# light and misattributes that to delay or lag. The ladder always fits the rigid body:
#   steady -> rigid (steady + inertia) -> rigid+delay | rigid+lag -> joint
# 'rigid+delay' vs 'rigid' is the test of "delay calibrated without the lag channel".
# Add-one arms from M0b (steady/timing/transient: every other channel at the prior) and the M0b
# leave-one-out arms are kept under their old definitions for replication only.
# nominal_d100 is the prior card with an engineering-guess 0.1 s delay (sensitivity arm).
STEADY_G, TIMING_G, TRANS_G = ('rev_ratio', 'deadband_q'), ('delay_s',), ('tau_s', 'rev_hold_s')
FREE = {'nominal': ((), (), 'all'), 'nominal_d100': ((), (), 'all'), 'broad': ((), (), 'all'),
        'steady': (STEADY_G, ('d',), 'steady'), 'timing': (TIMING_G, (), 'all'),
        'transient': (TRANS_G, ('m',), 'all'), 'joint': (tuple(GRID), ('m', 'd'), 'all'),
        'joint-steady': (TIMING_G + TRANS_G, ('m',), 'all'), 'joint-timing': (STEADY_G + TRANS_G, ('m', 'd'), 'all'),
        'joint-transient': (STEADY_G + TIMING_G, ('d',), 'all'),
        'rigid': (STEADY_G, ('m', 'd'), 'all'), 'rigid+delay': (STEADY_G + TIMING_G, ('m', 'd'), 'all'),
        'rigid+lag': (STEADY_G + TRANS_G, ('m', 'd'), 'all')}
REF_PASSES = 3            # refits with the coupling terms at the previous pass's fitted masses
BROAD = {'m': (0.7, 3.5), 'd': (0.5, 3.0), 'delay_s': (0.0, 0.6), 'tau_s': (0.0, 0.5), 'rev_hold_s': (0.0, 0.5),
         'rev_ratio': (0.6, 1.0), 'deadband_q': (0.0, 0.06), 'odom_frac': (0.0, 1.0)}
SENSORS = {'pos_sd': 0.05, 'vel_sd': 0.03, 'yaw_sd': 0.01, 'gyro_sd': 0.01}   # assumed by every twin


def _profile():
    prof = json.loads(Path(PROFILE).read_text())
    return profile_config(prof), profile_params(prof)


CONFIG, _P = _profile()
NOMINAL = {**{k: _P[k] for b in BLOCKS.values() for k in b}, 'delay_s': _P['delay_s'], 'tau_s': 0.0,
           'rev_hold_s': 0.0, 'rev_ratio': _P['rev_ratio'], 'deadband_q': _P['deadband_q'], 'odom_frac': 0.0}
for _k in GRID:
    assert NOMINAL[_k] in GRID[_k], f'nominal {_k}={NOMINAL[_k]} must be a grid point'
K_REF = CONFIG['k_ref']


def k_split(rev_ratio):
    """plantcard gauge: (k_fwd + k_rev)/2 = k_ref, k_rev = rev_ratio k_fwd."""
    k_fwd = 2.0 * K_REF / (1.0 + np.asarray(rev_ratio, float))
    return k_fwd, np.asarray(rev_ratio) * k_fwd


def to_spec(p, current=(0.0, 0.0), current_sd=0.0, sensors=SENSORS, rng=None):
    """PlantSpec of M twin particles (dict of arrays). Effective masses map onto nlc's
    (mass, a_jj) with the rigid-body mass fixed, as in ``Fossen3.rhs_wrench``."""
    m = len(p['m11'])
    rng = rng or np.random.default_rng(0)
    k_fwd, k_rev = k_split(p['rev_ratio'])
    m_rb, izz = CONFIG['m_rb'], CONFIG['izz_rb']
    frac = np.asarray(p.get('odom_frac', np.zeros(m)))
    cur = np.asarray(current, float)[None, :] + current_sd * rng.standard_normal((m, 2))
    return PlantSpec.build(
        m, mass=m_rb, izz=izz, a11=p['m11'] - m_rb, a22=p['m22'] - m_rb, a26=CONFIG['a26'], a66=p['m66'] - izz,
        d1=np.stack([p['d1_u'], p['d1_v'], p['d1_r']], -1), d2=np.stack([p['d2_u'], p['d2_v'], p['d2_r']], -1),
        k_fwd=np.repeat(k_fwd[:, None], N_THR, 1), k_rev=np.repeat(k_rev[:, None], N_THR, 1), gamma=2.0,
        deadband=np.repeat(np.asarray(p['deadband_q'])[:, None], N_THR, 1),
        dead_time=(1 - frac) * p['delay_s'], odom_latency=frac * p['delay_s'],
        tau_up=p['tau_s'], tau_down=p['tau_s'], rev_hold=p['rev_hold_s'], slew=np.inf,
        current=cur, **{k: v for k, v in sensors.items()})


def nominal_particles(m):
    return {k: np.full(m, float(v)) for k, v in NOMINAL.items()}


# ---------------------------------------------------------------------------------------------
# regression cache: one pass over the log for every grid point
# ---------------------------------------------------------------------------------------------

class Regression:
    """Rows of every segment (Phi per block, grid-independent) and the integrated-thrust targets y
    at every grid point ([G, rows] per block), plus the steady-window mask."""

    def __init__(self, evidence, window_s=1.0, step_s=0.2, ref=None):
        self.fam = Fossen3(CONFIG)
        self.ref = dict(ref or NOMINAL)
        segs = evidence.segments
        dt = float(np.median(np.diff(segs[0].t)))
        sub = int(round(dt / DT))
        if abs(sub * DT - dt) > 1e-9:
            raise ValueError('segment grid must be a whole number of plant ticks')
        self.axes = GRID
        self.points = list(itertools.product(*GRID.values()))
        self.shape = tuple(len(v) for v in GRID.values())
        self._win = (window_s, step_s, sub)
        self.segs = segs
        T = max(len(s.t) for s in segs)
        nf = (T - 1) * sub + 1
        U = np.zeros((len(segs), T, N_THR))
        for i, s in enumerate(segs):
            U[i, :len(s.t)] = s.u
            U[i, len(s.t):] = s.u[-1]
        # the measured velocity is a Savitzky-Golay derivative of 5 Hz GPS (and r a box-smoothed gyro),
        # so the integrated-thrust target goes through the same linear operators: the impulse I = int tau
        # is replaced by S[I], with S the identical SG operator applied to D = int I at the GPS sample
        # times (exact for the linear terms; the d2 and Coriolis terms stay approximate). Review B1.
        gps_t = [self._gps_times(s) for s in segs]
        gyro_t = [s.meta.get('gyro_t') for s in segs]
        cum = self._cum_thrust(U, dt, sub, nf, segs, gps_t, gyro_t)   # [S, G, T, 3] at sample times
        self.smoothed = all(g is not None for g in gps_t)
        self.cum = cum
        self.blocks = self._build(self.ref)
        self.n_rows = {b: len(v[0]) for b, v in self.blocks.items()}
        self.evidence_sha = evidence.provenance.get('sha256', '')

    def _build(self, ref):
        """Rows per block for coupling terms (Coriolis, added mass) evaluated at the masses in ``ref``.
        The integrated thrust ``cum`` does not depend on ref, so this is cheap."""
        segs, cum = self.segs, self.cum
        Ys, Ps = {b: [] for b in BLOCKS}, {b: [] for b in BLOCKS}
        for i, s in enumerate(segs):
            s.meta.pop('_cache', None)
            bb = self.fam._base(s, ref, *self._win)
            k1, k2 = bb['k1'], bb['k2']
            y = cum[i][:, k2] - cum[i][:, k1] - bb['icorr'][None]        # [G, R, 3]
            wrench = static_thrust(s.u, K_REF, K_REF, 2.0, 0.0) @ B_GEOM.T
            excited = np.abs(wrench).max(0) >= np.asarray(EXCITES)
            st = np.all(np.abs(bb['dnu']) < STEADY_DNU, axis=1)
            for j, b in enumerate(BLOCKS):
                if not excited[j]:
                    continue
                Ps[b].append((np.stack([bb['dnu'][:, j], bb['ir'][:, j], bb['iq'][:, j]], -1), st))
                Ys[b].append(y[:, :, j])
        blocks = {}
        for b in BLOCKS:
            if not Ps[b]:
                raise ValueError(f'no segment excites the {b} block')
            blocks[b] = (np.concatenate([p for p, _ in Ps[b]]), np.concatenate(Ys[b], axis=1),
                         np.concatenate([m for _, m in Ps[b]]))
        return blocks

    def with_ref(self, ref):
        """A view of this regression with the coupling terms at other reference masses."""
        import copy
        r = copy.copy(self)
        r.ref = dict(ref)
        r.blocks = self._build(r.ref)
        return r

    @staticmethod
    def _gps_times(seg):
        if 'gps_t' in seg.meta:
            return np.asarray(seg.meta['gps_t'], float)
        if 'gps' in seg.meta:                       # plantcard fieldtrials: (t - t0, positions)
            return np.asarray(seg.meta['gps'][0], float)
        return None

    def _cum_thrust(self, U, dt, sub, nf, segs, gps_t, gyro_t):
        """Integrated twin thrust at every grid point, as the measurement pipeline sees it: [S, G, T, 3].
        Thrust is held over each plant tick, so the impulse I is exact; D = int I is exact by trapezoid.
        x/y: SG derivative of D at the GPS times (the operator that made the velocities); N: the gyro's
        5-sample box over I at the gyro times. Without GPS/gyro times a channel falls back to raw I."""
        S, T, _ = U.shape
        G = len(self.points)
        P = np.array(self.points)                                       # [G, 5]
        delay, tau, hold, rev, db = (P[:, j] for j in range(5))
        k_fwd, k_rev = k_split(rev)
        state = ThrusterState((S, G), np.broadcast_to(np.maximum(k_fwd, k_rev), (S, G)), DT,
                              np.broadcast_to(tau, (S, G)), np.broadcast_to(tau, (S, G)),
                              np.broadcast_to(hold, (S, G)), np.full((S, G), np.inf))
        h = DT
        to_k = lambda t: np.clip(np.round(np.asarray(t) / DT).astype(int), 0, nf - 1)
        gk = np.unique(np.concatenate([to_k(g) for g in gps_t if g is not None] or [np.zeros(0, int)]))
        rk = np.unique(np.concatenate([to_k(g) for g in gyro_t if g is not None] or [np.zeros(0, int)]))
        gpos, rpos = {k: i for i, k in enumerate(gk)}, {k: i for i, k in enumerate(rk)}
        raw = np.zeros((S, G, T, 3))
        Dg = np.zeros((S, G, len(gk), 2))
        Ir = np.zeros((S, G, len(rk)))
        acc = np.zeros((S, G, 3))
        D = np.zeros((S, G, 2))
        dticks = np.round(delay / DT).astype(int)                        # [G]
        kf_ = k_fwd[None, :, None]
        kr_ = k_rev[None, :, None]
        db_ = db[None, :, None]
        gam = np.full((S, G), 2.0)
        for k in range(1, nf):
            src = np.clip(k - 1 - dticks, 0, None) // sub                 # command index applied over [k-1, k)
            q = U[:, np.minimum(src, T - 1)]                              # [S, G, 4]
            q = np.where((k - 1 - dticks)[None, :, None] < 0, U[:, :1], q)
            f = static_thrust(q, kf_, kr_, gam, db_)
            W = state.step(f) @ B_GEOM.T
            prev = acc[..., :2]
            acc = acc + W * h
            D = D + 0.5 * (prev + acc[..., :2]) * h
            if k % sub == 0:
                raw[:, :, k // sub] = acc
            if k in gpos:
                Dg[:, :, gpos[k]] = D
            if k in rpos:
                Ir[:, :, rpos[k]] = acc[..., 2]
        out = raw.copy()
        for i, s in enumerate(segs):
            n = len(s.t)
            if gps_t[i] is not None:
                tg = np.asarray(gps_t[i], float)
                Wsg = sg_velocity(tg, np.eye(len(tg)), s.t)               # [T_i, n_gps]: v = Wsg @ p
                ok = np.isfinite(Wsg).all(1)
                cols = np.array([gpos[k] for k in to_k(tg)])
                Dseg = np.where((tg < 0)[None, :, None], 0.0, Dg[i][:, cols])   # before t = 0: no thrust
                sm = np.einsum('tn,gnc->gtc', np.nan_to_num(Wsg), Dseg)
                out[i, :, :n, :2] = np.where(ok[None, :, None], sm, raw[i, :, :n, :2])
            if gyro_t[i] is not None:
                tr = np.asarray(gyro_t[i], float)
                cols = np.array([rpos[k] for k in to_k(tr)])
                Iseg = np.where((tr < 0)[None, :], 0.0, Ir[i][:, cols])
                box = np.apply_along_axis(lambda v: np.convolve(v, np.ones(5) / 5, 'same'), 1, Iseg)
                out[i, :, :n, 2] = np.stack([np.interp(s.t, tr, b) for b in box])
        return out

    def index(self, fixed):
        """Flat grid indices whose fixed axes equal the given values."""
        names = list(GRID)
        keep = [i for i, pt in enumerate(self.points)
                if all(abs(pt[names.index(k)] - v) < 1e-12 for k, v in fixed.items())]
        return np.asarray(keep)


# ---------------------------------------------------------------------------------------------
# batched NIG evidence
# ---------------------------------------------------------------------------------------------

def nig_prior(names):
    return NIG.prior([NOMINAL[k] for k in names], REL_STD, NOISE_STD)


def batched_posterior(prior, Phi, Y, pinned):
    """Posterior over the free columns for every row of Y ([G, R]) and the log evidence [G].
    ``pinned``: {column: value}, conditioned out of the prior (plantcard BlockFit semantics)."""
    d = len(prior.mu)
    free = [j for j in range(d) if j not in pinned]
    mu0, L0 = prior.mu, prior.Lam
    if pinned:
        P = sorted(pinned)
        dv = np.array([pinned[j] for j in P]) - mu0[P]
        Lff = L0[np.ix_(free, free)]
        mu0 = mu0[free] - np.linalg.solve(Lff, L0[np.ix_(free, P)] @ dv)
        L0 = Lff
        Y = Y - (Phi[:, P] @ np.array([pinned[j] for j in P]))[None, :]
        Phi = Phi[:, free]
    a0, b0 = prior.a, prior.b
    R = Phi.shape[0]
    Ln = L0 + Phi.T @ Phi
    rhs = (L0 @ mu0)[None, :] + Y @ Phi                                    # [G, f]
    mun = np.linalg.solve(Ln, rhs.T).T
    an = a0 + R / 2
    bn = b0 + 0.5 * ((Y * Y).sum(1) + mu0 @ L0 @ mu0 - np.einsum('gi,ij,gj->g', mun, Ln, mun))
    bn = np.maximum(bn, 1e-12)
    _, ld0 = np.linalg.slogdet(L0)
    _, ldn = np.linalg.slogdet(Ln)
    logev = (-0.5 * R * math.log(2 * math.pi) + 0.5 * (ld0 - ldn) + a0 * math.log(b0) - an * np.log(bn)
             + math.lgamma(an) - math.lgamma(a0))
    return {'free': free, 'mu': mun, 'Lam': Ln, 'a': an, 'b': bn, 'logev': logev}


def fit(reg, arm, n, seed, extra_fixed=None):
    """Particles (dict name -> [n]) of one ablation arm, and a fit report."""
    rng = np.random.default_rng(seed)
    if arm == 'nominal':
        return nominal_particles(n), {'arm': arm}
    if arm == 'nominal_d100':
        p = nominal_particles(n)
        p['delay_s'][:] = 0.1
        return p, {'arm': arm}
    if arm == 'broad':
        p = {}
        for b, names in BLOCKS.items():
            p[names[0]] = NOMINAL[names[0]] * rng.uniform(*BROAD['m'], n)
            for k in names[1:]:
                p[k] = NOMINAL[k] * rng.uniform(*BROAD['d'], n)
        for k in GRID:
            p[k] = rng.uniform(*BROAD[k], n)
        p['odom_frac'] = rng.uniform(*BROAD['odom_frac'], n)
        return p, {'arm': arm}
    if '@d' in arm:                         # e.g. 'rigid@d0.25': the base arm with the delay ASSUMED, not fitted
        base, d = arm.split('@d')
        extra_fixed = dict(extra_fixed or {}, delay_s=float(d))
        arm_key = base
    else:
        arm_key = arm
    grid_free, nig_free, rows = FREE[arm_key]
    fixed = {k: NOMINAL[k] for k in GRID if k not in grid_free}
    fixed.update(extra_fixed or {})
    gidx = reg.index(fixed)
    passes = REF_PASSES if 'm' in nig_free else 1
    for it in range(passes):
        logp = np.zeros(len(gidx))
        posts = {}
        for b, names in BLOCKS.items():
            Phi, Y, steady = reg.blocks[b]
            if rows == 'steady':
                Phi, Y = Phi[steady], Y[:, steady]
            Y = Y[gidx]
            pinned = {}
            if 'm' not in nig_free:
                pinned[0] = NOMINAL[names[0]]
            if 'd' not in nig_free:
                pinned[1], pinned[2] = NOMINAL[names[1]], NOMINAL[names[2]]
            post = batched_posterior(nig_prior(names), Phi, Y, pinned)
            posts[b] = (post, pinned)
            logp += post['logev']
        if it + 1 < passes:                   # coupling terms at this pass's MAP masses (plantcard's pass 2)
            g = int(np.argmax(logp))
            ref = dict(reg.ref)
            for b, names in BLOCKS.items():
                post, pinned = posts[b]
                if 0 in post['free']:
                    ref[names[0]] = float(max(post['mu'][g][post['free'].index(0)], BOUNDS[names[0]]))
            reg = reg.with_ref(ref)
    w = np.exp(logp - logp.max())
    w /= w.sum()
    names_g = list(GRID)
    pick = rng.choice(len(gidx), size=n, p=w)
    p = {k: np.array([reg.points[gidx[i]][names_g.index(k)] for i in pick]) for k in GRID}
    clipped = {}
    for b, names in BLOCKS.items():
        post, pinned = posts[b]
        free = post['free']
        th = np.empty((n, 3))
        for j, v in pinned.items():
            th[:, j] = v
        if free:
            cov_unit = np.linalg.inv(post['Lam'])
            Lc = np.linalg.cholesky(cov_unit)
            s2 = post['b'][pick] / rng.gamma(post['a'], 1.0, n)
            z = rng.standard_normal((n, len(free)))
            th[:, free] = post['mu'][pick] + np.sqrt(s2)[:, None] * (z @ Lc.T)
        clipped[b] = float(np.mean((th[:, 0] < BOUNDS[names[0]]) | (th[:, 1:] < 0).any(1)))
        th[:, 0] = np.maximum(th[:, 0], BOUNDS[names[0]])
        th[:, 1:] = np.maximum(th[:, 1:], 0.0)
        for j, k in enumerate(names):
            p[k] = th[:, j]
    p['odom_frac'] = rng.uniform(0.0, 1.0, n) if 'delay_s' in grid_free else np.zeros(n)
    i_map = gidx[int(np.argmax(logp))]
    report = {'arm': arm, 'grid_points': int(len(gidx)), 'map': dict(zip(names_g, reg.points[i_map])),
              'grid_entropy': float(-(w[w > 0] * np.log(w[w > 0])).sum()),
              'block_map': {b: [float(v) for v in posts[b][0]['mu'][int(np.argmax(logp))]] for b in BLOCKS},
              'rows': {b: int((reg.blocks[b][2].sum() if rows == 'steady' else len(reg.blocks[b][0]))) for b in BLOCKS},
              'frac_clipped': clipped}
    return p, report


def fit_all(evidence, n, seed, arms=ARMS):
    """Every arm from the same evidence (one shared regression pass)."""
    reg = Regression(evidence)
    out = {}
    for i, arm in enumerate(arms):
        out[arm] = fit(reg, arm, n, seed + 7919 * i)
    return out, reg
