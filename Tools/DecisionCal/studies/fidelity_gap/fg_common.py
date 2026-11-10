"""EXPLORATORY. Shared helpers of the fidelity-gap study: table loading, tiny numpy models (ridge with
unpenalised columns, logistic ridge, an isotropic-RBF GP), grouped nested cross-validation, exact intervals
and rank statistics. numpy + stdlib only."""

import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
BOATS = ('steady', 'timing', 'transient', 'mixed')
LAMS = (0.1, 1.0, 10.0, 100.0, 1000.0)

RAW_BOAT = ('b_dead_time', 'b_odom_latency', 'b_lag', 'b_deadband', 'b_rev_hold', 'b_inertia_ratio',
            'b_d_surge_ratio', 'b_d_sway_ratio', 'b_d_yaw_ratio', 'b_thrust_scale', 'b_wind', 'b_gust_sigma',
            'b_current')
RAW_CTRL = ('c_turn_alat', 'c_brake', 'c_cruise', 'c_ff', 'c_latcomp', 'c_dock_const', 'c_dock_sqrt',
            'c_dock_two_stage', 'c_dock_creep')
# the mechanism-indexed interaction features (build_table.interaction_features); fixed before any fit
MECH = ('x_no_dock_phase', 'x_t_in_window', 'x_t_slow_in_window', 'x_frac_slow_in_window', 'x_hold_breaks',
        'x_hold_margin', 'x_speed_p90_window', 'x_dwell_before_hold', 'x_rev_per_s', 'x_rev_per_lag',
        'x_rev_hold_duty', 'x_frac_cmd_in_deadband', 'x_frac_cmd_below_snap', 'x_frac_cmd_dead_any',
        'x_cmd_min_over_deadband', 'x_cmd_med', 'x_lat_force_used', 'x_lat_force_margin', 'x_dist_speed_over_hold',
        'x_knife_edge', 'x_current_over_hold')


def load_table():
    rows = json.loads((OUT / 'paired_table.json').read_text())
    T = {}
    for k in rows[0]:
        v = [r[k] for r in rows]
        T[k] = np.array(v) if isinstance(v[0], str) else np.array(v, float)
    T['boat_id'] = np.array([BOATS.index(b) for b in T['boat']])
    T['block'] = T['boat_id'] * 10 + T['seed'].astype(int)          # (boat, seed) block id
    T['controller'] = T['controller'].astype(int)
    return T


def cols(T, names, prefix=''):
    return np.stack([T[(prefix + n) if n.startswith('x_') else n] for n in names], 1)


# ------------------------------------------------------------------ models

class Standardizer:
    def __init__(self, X):
        self.mu = X.mean(0)
        sd = X.std(0)
        self.sd = np.where(sd > 1e-12, sd, 1.0)
        self.keep = sd > 1e-12

    def __call__(self, X):
        return ((X - self.mu) / self.sd) * self.keep


def ridge_fit(Xp, Xu, y, lam):
    """Least squares with ridge on the columns of Xp (already standardised) and none on Xu (+ intercept).
    Returns a predictor."""
    n = len(y)
    U_ = np.column_stack([np.ones(n)] + ([Xu] if Xu is not None and Xu.shape[1] else []))
    A = np.column_stack([U_, Xp])
    P = np.zeros(A.shape[1])
    P[U_.shape[1]:] = lam
    w = np.linalg.solve(A.T @ A + np.diag(P) + 1e-9 * np.eye(A.shape[1]), A.T @ y)
    ku = U_.shape[1]

    def predict(Xp2, Xu2):
        U2 = np.column_stack([np.ones(len(Xp2))] + ([Xu2] if Xu2 is not None and Xu2.shape[1] else []))
        return U2 @ w[:ku] + Xp2 @ w[ku:]
    predict.w = w
    return predict


def logit_fit(X, y, lam, iters=15):
    """Logistic regression, ridge lam on all columns of X (standardised), free intercept; damped IRLS."""
    n, p = X.shape
    A = np.column_stack([np.ones(n), X])
    P = np.full(p + 1, lam)
    P[0] = 1e-6
    w = np.zeros(p + 1)
    pbar = min(max(y.mean(), 1e-3), 1 - 1e-3)
    w[0] = math.log(pbar / (1 - pbar))
    for _ in range(iters):
        z = np.clip(A @ w, -30, 30)
        mu = 1 / (1 + np.exp(-z))
        W = np.maximum(mu * (1 - mu), 1e-6)
        g = A.T @ (mu - y) + P * w
        H = (A * W[:, None]).T @ A + np.diag(P)
        step = np.linalg.solve(H, g)
        w = w - step
        if np.abs(step).max() < 1e-8:
            break

    def predict(X2):
        return 1 / (1 + np.exp(-np.clip(np.column_stack([np.ones(len(X2)), X2]) @ w, -30, 30)))
    predict.w = w
    return predict


def gp_fit(X, r, ells=(0.5, 1.0, 2.0, 4.0, 8.0), noise_fracs=(0.05, 0.2, 0.5, 0.9)):
    """Zero-mean GP on standardised X for the residual r; isotropic RBF; lengthscale (in units of sqrt(dim))
    and noise share chosen by marginal likelihood on a grid; total variance = var(r). Returns predictor of
    (mean, sd)."""
    n, d = X.shape
    D2 = ((X[:, None, :] - X[None, :, :]) ** 2).sum(-1) / max(d, 1)
    v = max(float(r.var()), 1e-9)
    best = None
    for ell in ells:
        K0 = np.exp(-0.5 * D2 / ell ** 2)
        for nf in noise_fracs:
            K = v * (1 - nf) * K0 + v * nf * np.eye(n)
            try:
                L = np.linalg.cholesky(K + 1e-9 * v * np.eye(n))
            except np.linalg.LinAlgError:
                continue
            a = np.linalg.solve(L.T, np.linalg.solve(L, r))
            ml = -0.5 * r @ a - np.log(np.diag(L)).sum()
            if best is None or ml > best[0]:
                best = (ml, ell, nf, L, a)
    _, ell, nf, L, a = best

    def predict(X2):
        d2 = ((X2[:, None, :] - X[None, :, :]) ** 2).sum(-1) / max(d, 1)
        Ks = v * (1 - nf) * np.exp(-0.5 * d2 / ell ** 2)
        mean = Ks @ a
        V = np.linalg.solve(L, Ks.T)
        var = v - (V ** 2).sum(0)
        return mean, np.sqrt(np.maximum(var, 1e-12))
    predict.hyper = {'ell': ell, 'noise_frac': nf}
    return predict


def group_folds(groups):
    for g in np.unique(groups):
        yield g, np.where(groups != g)[0], np.where(groups == g)[0]


def choose_lam(make, loss, groups_tr, n_tr, lams=LAMS, max_folds=6):
    """Inner grouped CV on the training rows only. ``make(lam, tr_idx)`` -> predict(te_idx) (indices local to
    the training set); ``loss(pred, te_idx)`` -> summed loss. Returns (best lam, out-of-fold residual info)."""
    ug = np.unique(groups_tr)
    if len(ug) < 2:
        return lams[len(lams) // 2]
    if len(ug) > max_folds:                       # merge groups into max_folds folds, deterministic
        fold = {g: i % max_folds for i, g in enumerate(ug)}
        gi = np.array([fold[g] for g in groups_tr])
    else:
        gi = groups_tr
    tot = []
    for lam in lams:
        s = 0.0
        for _, tr, te in group_folds(gi):
            s += loss(make(lam, tr)(te), te)
        tot.append(s)
    return lams[int(np.argmin(tot))]


# ------------------------------------------------------------------ statistics

def clopper_pearson(k, n, alpha=0.05):
    """Exact two-sided binomial interval."""
    if n == 0:
        return (0.0, 1.0)

    def cdf(p, kk):                                # P(X <= kk)
        return sum(math.comb(n, i) * p ** i * (1 - p) ** (n - i) for i in range(0, kk + 1))

    # lower: P(X >= k | p) = alpha/2 ; upper: P(X <= k | p) = alpha/2
    if k == 0:
        lower = 0.0
    else:
        lo, hi = 0.0, 1.0
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if 1 - cdf(mid, k - 1) < alpha / 2:
                lo = mid
            else:
                hi = mid
        lower = 0.5 * (lo + hi)
    if k == n:
        upper = 1.0
    else:
        lo, hi = 0.0, 1.0
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if cdf(mid, k) > alpha / 2:
                lo = mid
            else:
                hi = mid
        upper = 0.5 * (lo + hi)
    return (lower, upper)


def rankdata(a):
    a = np.asarray(a, float)
    order = np.argsort(a, kind='stable')
    r = np.empty(len(a))
    i = 0
    while i < len(a):
        j = i
        while j + 1 < len(a) and a[order[j + 1]] == a[order[i]]:
            j += 1
        r[order[i:j + 1]] = 0.5 * (i + j) + 1
        i = j + 1
    return r


def spearman(a, b):
    ra, rb = rankdata(a), rankdata(b)
    if ra.std() == 0 or rb.std() == 0:
        return float('nan')
    return float(np.corrcoef(ra, rb)[0, 1])


def auc(score, label):
    """P(score of a positive > score of a negative) with ties 1/2; nan when one class is empty."""
    label = np.asarray(label, bool)
    n1, n0 = int(label.sum()), int((~label).sum())
    if n1 == 0 or n0 == 0:
        return float('nan')
    r = rankdata(score)
    return float((r[label].sum() - n1 * (n1 + 1) / 2) / (n1 * n0))


def brier(p, y):
    return float(np.mean((np.asarray(p) - np.asarray(y)) ** 2))


def sign_flip_exact(d):
    """Exact two-sided sign-flip p of mean(d) != 0 over len(d) <= 16 units; smallest attainable 2 / 2^n."""
    d = np.asarray(d, float)
    n = len(d)
    signs = ((np.arange(2 ** n)[:, None] >> np.arange(n)) & 1) * 2 - 1
    return float(np.mean(np.abs((signs * d).mean(1)) >= abs(d.mean()) - 1e-12))


def within_block_perm(block, rng):
    """A permutation of row indices that shuffles rows inside each block only."""
    idx = np.arange(len(block))
    out = idx.copy()
    for b in np.unique(block):
        m = idx[block == b]
        out[m] = rng.permutation(m)
    return out


def jdump(obj, path):
    def conv(o):
        if isinstance(o, (np.floating, np.integer)):
            return o.item()
        if isinstance(o, np.ndarray):
            return o.tolist()
        if isinstance(o, np.bool_):
            return bool(o)
        raise TypeError(type(o))
    Path(path).write_text(json.dumps(obj, indent=1, default=conv))
