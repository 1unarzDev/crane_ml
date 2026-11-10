"""Model-free baseline selector: probe-log features -> controller, trained on development boats.

Purpose. A reviewer asks "why a physics twin at all?". This is the strongest simple answer we can
give to the opposite question: given the SAME 624 s open-loop probe log (probes.generate), a selector
with NO physical model maps hand-built log features directly to one of the 16 controllers of a library
(v2 hand-placed, v3 random), using supervised data from development boats. Compare its regret / dock
rate with the twin arms on the same metrics (score.cross_fit_regret, regret capped at 60 s, dock rate
of the selected controller on the hidden boat).

ADVANTAGE THE BASELINE HAS OVER THE TWIN ARMS (state this whenever the numbers are quoted): the
twin arms use zero development boats and no controller-score labels; this baseline is trained on N
development boats (default 200) whose full mission score matrix J_h [C, K=4] was computed by flying every
library member on the hidden plant. It is a supervised learner with labels the twin never sees. It also
cannot be applied to a new library or mission without regenerating that training set, whereas a twin
can. Conversely the twin arms are not trained on the hidden-boat family, so a twin win over this baseline
at N=200 is a strong statement, and a baseline win is "labelled data beat physics" only for these libraries.

Leakage controls.
* Features are computed ONLY from the Evidence returned by probes.generate (Segment.t, .x, .u, .name):
  measured GPS/heading/gyro-derived signals and the commanded thruster efforts. Segment.meta (which
  holds only log-derived drift) is not read; the raw ``truth`` returned by probes.generate and every
  PlantSpec field are never touched by feature code. Segment names (the public protocol) are used to
  know which command was applied, exactly as the twin fitters do.
* Standardization (median / IQR, imputation of missing features by the median, clipping to +-5) is
  fitted on training boats only and applied unchanged to test boats. Ridge penalties and the kNN k are
  chosen by leave-one-out on the training set (the scaler is fitted on all training boats, which only
  touches training data).
* Training seed (default 7373, a development seed) and test seed (default 8484) are different, and the
  seeds of every scored study (20261109..20261114) are refused; probe, mission and noise seeds are
  derived from (seed, boat id, tag) so test boats never share a stream with training boats.
* Strata labels are used only to report results, never as features. The ``stratum-fixed`` row is a
  REFERENCE that uses training stratum labels (not available to a real selector) and is not a baseline
  claim; ``hindsight-fixed`` picks the best constant on the TEST boats and is an optimistic reference.
* Learning-curve subsets are per-stratum prefixes of the training set, so each size keeps the strata
  balanced (sizes not divisible by 4 give the first strata one extra boat).
* Resuming requires identical (seed, n); sample_hidden is not prefix-stable in n, so ids encode the
  per-stratum count.

Features (14, all from the log; NaN when a response is below 6 noise SD, imputed by the training median):
see FEATURES / extract_features.

    python3 -I Tools/DecisionCal/baseline_selector.py generate --out DIR --n 200 --procs 4
    python3 -I Tools/DecisionCal/baseline_selector.py evaluate --train DIR/train.npz --test-dir DIR_TEST \
        --n 80 --procs 4 --json DIR_TEST/eval.json
    python3 -I Tools/DecisionCal/baseline_selector.py smoke --out DIR       # 6 train + 4 test, 1 process

Pure numpy; run niced with <= 4 processes.
"""

import argparse
import hashlib
import json
import os
import sys
import time
from multiprocessing import get_context
from pathlib import Path

os.environ.setdefault('OMP_NUM_THREADS', '1')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import _paths  # noqa: E402,F401
import hidden  # noqa: E402
import mission  # noqa: E402
import probes  # noqa: E402
import score  # noqa: E402
from library_v2 import LIBRARY_V2  # noqa: E402
from library_v3 import LIBRARY_V3  # noqa: E402

LIBS = {'v2': LIBRARY_V2, 'v3': LIBRARY_V3}
LIB_NAMES = tuple(LIBS)
K = 4
CAP = 60.0
DEV_SEED, TEST_SEED = 7373, 8484
FORBIDDEN = range(20261109, 20261115)          # seeds of scored studies M0..M4
FEATURES = ('delay_yaw', 'delay_surge', 'rise_yaw', 'rise_surge', 'speed_hi', 'speed_lin', 'yaw_gain',
            'rev_surge_t', 'rev_yaw_t', 'coast_surge_t50', 'coast_yaw_t50', 'asym_surge', 'drift', 'noise_u')
NF = len(FEATURES)
REV_CAP, COAST_CAP = 6.0, 12.0


def _seed(*parts):
    h = hashlib.sha256('/'.join(str(p) for p in parts).encode()).digest()
    return int.from_bytes(h[:7], 'little')


def check_seed(seed, other=None):
    if seed in FORBIDDEN:
        raise SystemExit(f'seed {seed} belongs to a scored study; refused')
    if other is not None and seed == other:
        raise SystemExit('training and test seeds must differ')


# ----------------------------------------------------------------------------------------- features
def _smooth3(y):
    return np.convolve(np.pad(y, 1, mode='edge'), np.ones(3) / 3, 'valid')


def _cross(t, y, thr, start):
    """Interpolated time of the first sustained (2 samples) upward crossing of thr at index >= start."""
    for i in range(max(start, 1), len(y) - 1):
        if y[i] >= thr and y[i + 1] >= thr:
            lo = y[i - 1]
            f = 0.0 if y[i] == lo else np.clip((thr - lo) / (y[i] - lo), 0.0, 1.0)
            return float(t[i - 1] + f * (t[i] - t[i - 1]))
    return np.nan


def _channel(seg, col):
    """Baseline-removed, sign-aligned (to the response) smoothed channel of one segment, plus timings."""
    t, U = np.asarray(seg.t), np.asarray(seg.u)
    on_idx = int(np.argmax(np.abs(U).sum(1) > 1e-9))
    on = float(t[on_idx])
    act = np.abs(U).sum(1) > 1e-9
    pe = float(t[np.flatnonzero(act)[-1]] + (t[1] - t[0]))
    u0 = U[on_idx]
    flip = np.flatnonzero((U @ u0 < -1e-9) & (t > on))
    t_flip = float(t[flip[0]]) if len(flip) else None
    p1 = t_flip if t_flip is not None else pe
    base = (t >= 5.0) & (t < on - 0.3)
    y = np.asarray(seg.x)[:, col]
    A = np.stack([np.ones(base.sum()), t[base] - on], 1)
    coef, *_ = np.linalg.lstsq(A, y[base], rcond=None)
    sd = float(np.std(y[base] - A @ coef))
    y0 = float(np.median(y[base]))
    ys = _smooth3(y - y0)
    win = (t >= on + 0.6 * (p1 - on)) & (t <= p1 + 0.3)
    sgn = 1.0 if np.median(ys[win]) >= 0 else -1.0
    return {'t': t, 'y': sgn * ys, 'on': on, 'pe': pe, 'flip': t_flip, 'sd': sd, 'sgn': sgn}


def _level(c, t_end):
    m = (c['t'] >= c['on']) & (c['t'] <= t_end)
    return float(c['y'][m].max())


def _step(c):
    """delay (10% crossing minus onset), 10-90% rise time, peak (response units)."""
    level = _level(c, (c['flip'] if c['flip'] is not None else c['pe']) + 1.5)
    out = {'peak': max(level, 0.0), 'delay': np.nan, 'rise': np.nan}
    if level < 6 * c['sd'] or level <= 0:
        return out
    thr = max(0.1 * level, 3 * c['sd'])
    if thr > 0.5 * level:
        return out
    t, y = c['t'], c['y']
    s = int(np.searchsorted(t, c['on'] - 1e-9))
    t10, t90 = _cross(t, y, thr, s), _cross(t, y, 0.9 * level, s)
    out['delay'] = t10 - c['on'] if np.isfinite(t10) else np.nan
    out['rise'] = t90 - t10 if np.isfinite(t10) and np.isfinite(t90) else np.nan
    return out


def _coast_t50(c):
    """Time from the response peak to half the peak (COAST_CAP if the half level is not reached)."""
    t, y = c['t'], c['y']
    m = (t >= c['on']) & (t <= c['pe'] + 3.0)
    idx = np.flatnonzero(m)
    k = idx[int(np.argmax(y[idx]))]
    level = y[k]
    if level < 6 * c['sd']:
        return np.nan
    after = np.flatnonzero((np.arange(len(y)) > k) & (y <= 0.5 * level))
    return float(t[after[0]] - t[k]) if len(after) else COAST_CAP


def _reversal_t(c):
    """Time from the command sign flip to the response sign flip (REV_CAP if it never flips)."""
    if c['flip'] is None:
        return np.nan
    t, y = c['t'], c['y']
    if _level(c, c['flip']) < 6 * c['sd']:
        return np.nan
    f = int(np.searchsorted(t, c['flip'] - 1e-9))
    for i in range(f, len(y) - 1):
        if y[i] <= 0 and y[i + 1] <= 0:
            return min(float(t[i] - c['flip']), REV_CAP)
    return REV_CAP


def _eff(name):
    return abs(float(name[-4:]))


def _nm(x):
    x = [v for v in x if np.isfinite(v)]
    return float(np.mean(x)) if x else np.nan


def extract_features(evidence):
    """The 14 features of FEATURES from a probes.generate Evidence. Log only; see module docstring."""
    S = {s.name: s for s in evidence.segments}
    U_COL, R_COL = 3, 5
    yaw = [_channel(S[n], R_COL) for n in ('yaw+0.3', 'yaw-0.3')]
    sur = {n: _channel(S[n], U_COL) for n in ('surge+0.3', 'surge-0.3', 'surge+0.6', 'surge-0.6', 'surge+0.9', 'surge-0.9')}
    st_y = [_step(c) for c in yaw]
    st_s = {n: _step(c) for n, c in sur.items()}
    hi = [st_s[n]['peak'] / 0.9 for n in ('surge+0.9', 'surge-0.9')]
    lo = [st_s[n]['peak'] / 0.3 for n in ('surge+0.3', 'surge-0.3')]
    speed_hi = _nm(hi)
    coast_s = [_coast_t50(sur[n]) for n in ('surge+0.9', 'surge-0.9')] + [_coast_t50(_channel(S['coast_surge'], U_COL))]
    p_f, p_r = st_s['surge+0.9']['peak'], st_s['surge-0.9']['peak']
    drift_v, noise = [], []
    for s in evidence.segments:
        t = np.asarray(s.t)
        b = (t >= 5.0) & (t < 10.7)
        A = np.stack([np.ones(b.sum()), t[b]], 1)
        P = np.asarray(s.x)[b][:, :2]
        drift_v.append(np.linalg.lstsq(A, P, rcond=None)[0][1])
        yb = np.asarray(s.x)[b][:, U_COL]
        noise.append(np.std(yb - A @ np.linalg.lstsq(A, yb, rcond=None)[0]))
    f = {
        'delay_yaw': _nm([x['delay'] for x in st_y]),
        'delay_surge': _nm([st_s[n]['delay'] for n in ('surge+0.6', 'surge-0.6', 'surge+0.9', 'surge-0.9')]),
        'rise_yaw': _nm([x['rise'] for x in st_y]),
        'rise_surge': _nm([st_s[n]['rise'] for n in ('surge+0.6', 'surge-0.6', 'surge+0.9', 'surge-0.9')]),
        'speed_hi': speed_hi,
        'speed_lin': _nm(lo) / speed_hi if speed_hi and np.isfinite(speed_hi) and speed_hi > 0 else np.nan,
        'yaw_gain': _nm([x['peak'] / 0.3 for x in st_y]),
        'rev_surge_t': _reversal_t(_channel(S['rev_surge'], U_COL)),
        'rev_yaw_t': _reversal_t(_channel(S['rev_yaw'], R_COL)),
        'coast_surge_t50': _nm(coast_s),
        'coast_yaw_t50': _nm([_coast_t50(c) for c in yaw]),
        'asym_surge': float(np.log(max(p_r, 1e-3) / max(p_f, 1e-3))),
        'drift': float(np.linalg.norm(np.mean(drift_v, 0))),
        'noise_u': float(np.median(noise)),
    }
    return np.array([f[k] for k in FEATURES], float)


# --------------------------------------------------------------------------------------- generation
def strata_counts(n):
    q, r = divmod(n, len(hidden.STRATA))
    return {st: q + (1 if i < r else 0) for i, st in enumerate(hidden.STRATA)}


def evaluate_boat(job):
    """Probe log -> features, and J_h [L, C, K] + docked [L, C] on the hidden boat (as study_m1.evaluate)."""
    t0 = time.time()
    row, pid, seed = job['spec'], job['id'], job['seed']
    ev, _raw = probes.generate(row, _seed(seed, pid, 'probe'))
    feats = extract_features(ev)
    t1 = time.time()
    J, D = [], []
    for lib in LIBS.values():
        C = len(lib)
        sp = row.repeat(C * K)
        seeds = np.tile([_seed(seed, pid, 'hidden', k) for k in range(K)], C).astype(np.uint64)
        met, _ = mission.run(sp, np.repeat(np.arange(C), K), seeds, lib=lib)
        J.append(score.finite_scores(met['J']).reshape(C, K))
        D.append(np.asarray(met['docked'], float).reshape(C, K).mean(1))
    return {'id': pid, 'stratum': job['stratum'], 'idx': job['idx'], 'features': feats.tolist(),
            'J_h': [j.tolist() for j in J], 'docked': [d.tolist() for d in D],
            'wall_probe_s': t1 - t0, 'wall_mission_s': time.time() - t1}


def build(out, seed, n, procs, label):
    """Generate (resumably) n boats at seed into out/boats.jsonl; write and return out/data.npz arrays."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    cnt = strata_counts(n)
    man = {'seed': seed, 'n': n, 'K': K, 'features': FEATURES, 'libs': {k: [c.label() for c in v] for k, v in LIBS.items()},
           'hidden_ranges_sha256': hidden.ranges_sha(), 'label': label, 'J_weights': mission.J_WEIGHTS}
    mp = out / 'manifest.json'
    if mp.exists():
        old = json.loads(mp.read_text())
        if (old['seed'], old['n']) != (seed, n):
            raise SystemExit(f'{mp} was made with seed/n {old["seed"]}/{old["n"]}; resume needs identical values')
    mp.write_text(json.dumps(man, indent=1, sort_keys=True))
    jobs = []
    for st in hidden.STRATA:
        m = cnt[st]
        if m == 0:
            continue
        spec = hidden.sample_hidden(m, seed, st)       # NOT prefix-stable in m: ids carry m
        for i in range(m):
            jobs.append({'spec': spec.take([i]), 'id': f'{st}-{seed}-n{m}-{i:03d}', 'stratum': st, 'idx': i, 'seed': seed})
    done = {}
    jp = out / 'boats.jsonl'
    if jp.exists():
        for line in jp.read_text().splitlines():
            r = json.loads(line)
            done[r['id']] = r
    todo = [j for j in jobs if j['id'] not in done]
    t0 = time.time()
    if todo:
        with get_context('fork').Pool(min(procs, len(todo))) as pool, open(jp, 'a') as fp:
            for k, r in enumerate(pool.imap_unordered(evaluate_boat, todo)):
                done[r['id']] = r
                fp.write(json.dumps(r, sort_keys=True) + '\n')
                fp.flush()
                print(f'[{label} {k + 1}/{len(todo)}] {r["id"]} probe {r["wall_probe_s"]:.0f}s mission {r["wall_mission_s"]:.0f}s '
                      f'({time.time() - t0:.0f}s)', flush=True)
    rows = [done[j['id']] for j in jobs]
    data = {'X': np.array([r['features'] for r in rows], float), 'J_h': np.array([r['J_h'] for r in rows], float),
            'docked': np.array([r['docked'] for r in rows], float), 'stratum': np.array([r['stratum'] for r in rows]),
            'idx': np.array([r['idx'] for r in rows]), 'ids': np.array([r['id'] for r in rows]),
            'features': np.array(FEATURES), 'libs': np.array(LIB_NAMES), 'seed': np.array(seed)}
    np.savez(out / 'data.npz' if label != 'train' else out / 'train.npz', **data)
    return data


def load(path):
    z = np.load(path, allow_pickle=False)
    return {k: z[k] for k in z.files}


def subset(data, size):
    """Per-stratum prefix subset of about `size` boats (strata balanced)."""
    cnt = strata_counts(size)
    keep = np.array([i < cnt.get(st, 0) for st, i in zip(data['stratum'], data['idx'])])
    return {k: (v[keep] if getattr(v, 'ndim', 0) and len(v) == len(keep) else v) for k, v in data.items()}


# --------------------------------------------------------------------------------------- selectors
class Scaler:
    """Median imputation, median/IQR scaling, clip to +-5. Fitted on training features only."""

    def fit(self, X):
        self.med = np.nanmedian(np.where(np.isfinite(X), X, np.nan), 0)
        self.med = np.where(np.isfinite(self.med), self.med, 0.0)
        q = np.nanpercentile(np.where(np.isfinite(X), X, np.nan), [25, 75], axis=0)
        s = (q[1] - q[0]) / 1.349
        sd = np.nanstd(np.where(np.isfinite(X), X, np.nan), 0)
        self.scale = np.where(np.isfinite(s) & (s > 1e-9), s, np.where(np.isfinite(sd) & (sd > 1e-9), sd, 1.0))
        return self

    def __call__(self, X):
        X = np.where(np.isfinite(X), X, self.med)
        return np.clip((X - self.med) / self.scale, -5.0, 5.0)


def _regret_of(Jm, sel):
    return float(np.mean(Jm[np.arange(len(Jm)), sel] - Jm.min(1)))


def _capped_regret_of(Jm, sel):
    return float(np.mean(np.minimum(Jm[np.arange(len(Jm)), sel] - Jm.min(1), CAP)))


class Constant:
    name = 'best-fixed'

    def fit(self, X, Jm, strata=None):
        self.c = int(np.argmin(Jm.mean(0)))
        return self

    def select(self, X):
        return np.full(len(X), self.c)


class StratumFixed:
    """REFERENCE ONLY: best constant per training stratum, applied with the TRUE test stratum."""
    name = 'stratum-fixed(ref)'

    def fit(self, X, Jm, strata):
        self.best = {s: int(np.argmin(Jm[strata == s].mean(0))) for s in np.unique(strata)}
        return self

    def select(self, X, strata):
        return np.array([self.best.get(s, 0) for s in strata])


class KNN:
    name = 'knn'

    def fit(self, X, Jm, strata=None, ks=(1, 2, 3, 5, 8, 12, 20, 30)):
        self.sc = Scaler().fit(X)
        self.Z, self.Jm = self.sc(X), Jm
        n = len(X)
        D = np.linalg.norm(self.Z[:, None] - self.Z[None], axis=2)
        np.fill_diagonal(D, np.inf)
        order = np.argsort(D, 1)
        best = None
        self.loo = {}
        for k in [k for k in ks if k <= n - 1] or [1]:
            if n < 2:
                break
            nb = order[:, :k]
            sel = Jm[nb].mean(1).argmin(1)
            r = _regret_of(Jm, sel)
            self.loo[k] = r
            if best is None or r < best[0] - 1e-12:
                best = (r, k)
        self.k = best[1] if best else 1
        return self

    def select(self, X):
        Z = self.sc(X)
        D = np.linalg.norm(Z[:, None] - self.Z[None], axis=2)
        nb = np.argsort(D, 1)[:, :min(self.k, len(self.Z))]
        return self.Jm[nb].mean(1).argmin(1)


def _quad(Z):
    iu = np.triu_indices(Z.shape[1])
    return np.concatenate([Z, Z[:, iu[0]] * Z[:, iu[1]]], 1)


class Ridge:
    """Ridge per controller (one multi-output solve) predicting mean J; lambda by closed-form LOO on the
    regret of argmin(prediction). quad=True adds all squares and pairwise products of the scaled features."""

    def __init__(self, quad=False):
        self.quad = quad
        self.name = 'ridge-quad' if quad else 'ridge'

    def _phi(self, X):
        Z = self.sc(X)
        F = _quad(Z) if self.quad else Z
        return (F - self.fm) / self.fs

    def fit(self, X, Jm, strata=None, lams=tuple(10.0 ** np.arange(-2, 4.5, 0.5))):
        self.sc = Scaler().fit(X)
        Z = self.sc(X)
        F = _quad(Z) if self.quad else Z
        self.fm, self.fs = F.mean(0), np.where(F.std(0) > 1e-9, F.std(0), 1.0)
        P = (F - self.fm) / self.fs
        n = len(P)
        self.ym = Jm.mean(0)
        Y = Jm - self.ym
        U, s, Vt = np.linalg.svd(P, full_matrices=False)
        best = None
        self.loo = {}
        for lam in lams:
            d = s / (s ** 2 + lam)
            W = Vt.T @ (d[:, None] * (U.T @ Y))
            h = np.einsum('ij,j,ij->i', U, s ** 2 / (s ** 2 + lam), U) + 1.0 / n
            res = Y - P @ W
            loo_pred = Jm - res / (1 - h)[:, None]           # leave-one-out prediction of J (incl. intercept)
            r = _regret_of(Jm, loo_pred.argmin(1)) if n > 2 else 0.0
            self.loo[float(lam)] = r
            if best is None or r < best[0] - 1e-12:
                best = (r, lam, W)
        self.lam, self.W = best[1], best[2]
        return self

    def select(self, X):
        return (self._phi(X) @ self.W + self.ym).argmin(1)


def make_selectors():
    return [Constant(), KNN(), Ridge(False), Ridge(True)]


def train_selectors(data, li):
    """Fit all selectors for library index li on a (sub)set; target = mean_K J_h. Returns dict name -> fitted."""
    Jm = data['J_h'][:, li].mean(2)
    out = {}
    for s in make_selectors() + [StratumFixed()]:
        out[s.name] = s.fit(data['X'], Jm, data['stratum'])
    return out


# --------------------------------------------------------------------------------------- evaluation
def _ci(x, rng, n_boot=2000):
    x = np.asarray(x, float)
    idx = rng.integers(0, len(x), (n_boot, len(x)))
    lo, hi = np.quantile(x[idx].mean(1), [0.05, 0.95])
    return float(x.mean()), float(lo), float(hi)


def score_selection(test, li, sel):
    """Per test boat: cross-fit regret, capped regret, dock rate of the selected controller."""
    J = test['J_h'][:, li]
    reg = np.array([score.cross_fit_regret(J[i], int(sel[i])) for i in range(len(sel))])
    dock = test['docked'][np.arange(len(sel)), li, sel]
    return reg, np.minimum(reg, CAP), dock


def apply_selectors(sels, test, li):
    out = {}
    for name, m in sels.items():
        out[name] = m.select(test['X'], test['stratum']) if name.startswith('stratum-fixed') else m.select(test['X'])
    Jm = test['J_h'][:, li].mean(2)
    out['hindsight-fixed(ref)'] = np.full(len(Jm), int(np.argmin(Jm.mean(0))))
    return out


def evaluate(train, test, sizes, seed=0):
    rng = np.random.default_rng(seed)
    res = {'n_train': int(len(train['X'])), 'n_test': int(len(test['X'])), 'libs': {}, 'learning_curve': {}}
    groups = [(st, test['stratum'] == st) for st in hidden.STRATA if (test['stratum'] == st).any()] + [('all', np.ones(len(test['X']), bool))]
    for li, ln in enumerate(LIB_NAMES):
        sels = train_selectors(train, li)
        picks = apply_selectors(sels, test, li)
        tab = {}
        for name, sel in picks.items():
            reg, capr, dock = score_selection(test, li, sel)
            tab[name] = {g: {'n': int(m.sum()), 'regret_cf': _ci(reg[m], rng), 'regret_cf_capped60': _ci(capr[m], rng),
                             'dock_rate': _ci(dock[m], rng)} for g, m in groups}
            tab[name]['n_distinct_selected'] = int(len(set(sel.tolist())))
        res['libs'][ln] = {'table': tab, 'knn_k': int(sels['knn'].k), 'ridge_lambda': float(sels['ridge'].lam),
                           'ridge_quad_lambda': float(sels['ridge-quad'].lam),
                           'knn_loo': {str(k): v for k, v in sels['knn'].loo.items()},
                           'oracle_top_share_test': float(np.bincount(test['J_h'][:, li].mean(2).argmin(1)).max() / len(test['X']))}
        lc = {}
        for sz in sizes:
            if sz > len(train['X']):
                continue
            sub = subset(train, sz)
            ss = apply_selectors(train_selectors(sub, li), test, li)
            lc[str(sz)] = {name: {'n_train': int(len(sub['X'])),
                                  'regret_cf_capped60': _ci(score_selection(test, li, sel)[1], rng),
                                  'dock_rate': _ci(score_selection(test, li, sel)[2], rng)} for name, sel in ss.items()}
        res['learning_curve'][ln] = lc
    return res


def fmt(x, p=1):
    return f'{x[0]:.{p}f} [{x[1]:.{p}f},{x[2]:.{p}f}]'


def print_eval(res):
    print(f'baseline selector: n_train={res["n_train"]} n_test={res["n_test"]} (twin arms use 0 training boats)')
    for ln, body in res['libs'].items():
        print(f'== library {ln}: knn k={body["knn_k"]}, ridge lambda={body["ridge_lambda"]:g}, '
              f'ridge-quad lambda={body["ridge_quad_lambda"]:g}, test oracle top share {body["oracle_top_share_test"]:.2f}')
        print('   cell = capped(60 s) cross-fit regret [90% CI] / uncapped regret / dock rate')
        for name, row in body['table'].items():
            cells = []
            for g, v in row.items():
                if g == 'n_distinct_selected':
                    continue
                cells.append(f'{g}(n={v["n"]}): {fmt(v["regret_cf_capped60"])} / {v["regret_cf"][0]:.1f} / {v["dock_rate"][0]:.2f}')
            print(f'  {name:22s} ' + ' | '.join(cells))
    for ln, lc in res['learning_curve'].items():
        print(f'== learning curve, library {ln} (all test strata; capped regret [90% CI] / dock rate)')
        for sz, d in lc.items():
            print(f'  N={sz:>4s} ' + ' | '.join(f'{n}: {fmt(v["regret_cf_capped60"])} / {v["dock_rate"][0]:.2f}'
                                                 for n, v in d.items() if n in ('best-fixed', 'knn', 'ridge', 'ridge-quad')))


# ------------------------------------------------------------------------------------------- CLI
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    g = sub.add_parser('generate', help='development boats -> features + J_h (resumable)')
    g.add_argument('--out', required=True)
    g.add_argument('--seed', type=int, default=DEV_SEED)
    g.add_argument('--n', type=int, default=200)
    g.add_argument('--procs', type=int, default=4)
    e = sub.add_parser('evaluate', help='fresh test boats, trained selectors, regret / dock tables, learning curve')
    e.add_argument('--train', required=True, help='train.npz from generate')
    e.add_argument('--test-dir', required=True)
    e.add_argument('--seed', type=int, default=TEST_SEED)
    e.add_argument('--n', type=int, default=80)
    e.add_argument('--procs', type=int, default=4)
    e.add_argument('--sizes', default='25,50,100,200')
    e.add_argument('--json', default=None)
    s = sub.add_parser('smoke', help='6 training + 4 test boats, 1 process')
    s.add_argument('--out', required=True)
    s.add_argument('--procs', type=int, default=1)
    a = ap.parse_args(argv)
    if getattr(a, 'procs', 1) > 4:
        raise SystemExit('innovation compute policy: at most 4 processes')
    if a.cmd == 'generate':
        check_seed(a.seed)
        d = build(a.out, a.seed, a.n, a.procs, 'train')
        print(f'wrote {a.out}/train.npz  X {d["X"].shape}  J_h {d["J_h"].shape}  missing features: '
              f'{int((~np.isfinite(d["X"])).sum())}')
    elif a.cmd == 'evaluate':
        tr = load(a.train)
        check_seed(a.seed, int(tr['seed']))
        te = build(a.test_dir, a.seed, a.n, a.procs, 'test')
        res = evaluate(tr, te, [int(x) for x in a.sizes.split(',')])
        print_eval(res)
        if a.json:
            Path(a.json).write_text(json.dumps(res, indent=1, sort_keys=True))
    else:
        out = Path(a.out)
        tr = build(out / 'train', DEV_SEED, 6, a.procs, 'train')
        te = build(out / 'test', TEST_SEED, 4, a.procs, 'test')
        print_eval(evaluate(tr, te, [3, 6]))


if __name__ == '__main__':
    main()
