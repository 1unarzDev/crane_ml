"""Held-out prediction coverage: what "the card is right" means (design sec. 5.6).

Event ``pos_err_max@H``: replay the executed inputs from the measured state through
``card.rollout`` (one particle and one disturbance draw per rollout) and take, over the window
[t_event, t_event + H], the max distance of the real position to the rollouts' mean path. The
raw set radius is the (1 - alpha) quantile of the same max distance for the rollouts themselves;
the normalised score is s = e_real / radius (s <= 1: covered by the raw set).

Sessions are the replication units: each session is predicted by a card fitted without it
(leave-one-session-out). Conformal sets use the cross-conformal multiplier of the other
sessions' out-of-fold scores, so a biased card means wider sets, not invalid ones. Bounds:
Clopper-Pearson on segments and a session (cluster) bootstrap, both one-sided 95%.
"""

import math

import numpy as np

from plantcard.card import Card
from plantcard.update import propose

HORIZONS = (2.0, 5.0, 10.0)


def spread_scores(tr, pos):
    """Mean path [T, 2] of rollouts [n, T, nx] and each rollout's max distance to it [n]."""
    P = tr[:, :, list(pos)]
    mean = P.mean(0)
    return mean, np.sqrt(((P - mean) ** 2).sum(-1)).max(1)


def crps_samples(x, y):
    """Sample CRPS of draws x [n] against the observation y."""
    x = np.sort(np.asarray(x, float))
    n = len(x)
    return float(np.mean(np.abs(x - y)) - (2 * np.arange(1, n + 1) - n - 1) @ x / n ** 2)


def score_segment(card, seg, horizons=HORIZONS, alphas=(0.1,), n=128, seed=0, pre_s=None, disturbance=True):
    """Scores of one segment for each horizon (one rollout set over the longest horizon)."""
    fam = card.family
    t = seg.t
    if pre_s is None:
        d = card.particles.get(fam.delay_param) if fam.delay_param else None
        pre_s = float(np.max(d)) + 0.1 if d is not None else 0.0
    k_ev = int(np.searchsorted(t, seg.t_event))
    k0 = max(0, int(np.searchsorted(t, seg.t_event - pre_s)))
    k1 = min(len(t), int(np.searchsorted(t, seg.t_event + max(horizons), side='right')))
    x0 = seg.x[k0].copy()
    if fam.delay_param:   # causal velocity estimate: trailing mean (no look-ahead)
        lo = max(0, k0 - 4)
        x0[3:] = seg.x[lo:k0 + 1, 3:].mean(0)
    tr = card.rollout(x0, t[k0:k1], seg.u[k0:k1], n=n, seed=seed, seg=seg, disturbance=disturbance)
    real = seg.x[k0:k1][:, list(fam.position)]
    out = []
    for H in horizons:
        kh = int(np.searchsorted(t[k0:k1], seg.t_event + H, side='right'))
        if t[k0 + kh - 1] < seg.t_event + H - 0.2:
            continue                                    # the segment is shorter than H
        sub = slice(k_ev - k0, kh)
        mean, e = spread_scores(tr[:, sub], fam.position)
        err = np.sqrt(((real[sub] - mean) ** 2).sum(-1))
        e_real = float(err.max())
        final = tr[:, kh - 1][:, list(fam.position)]
        row = {'segment': seg.name, 'session': seg.session, 'H': H, 'e_real': e_real,
               'e_mean_path_final': float(err[-1]),
               'crps_final': float(np.mean([crps_samples(final[:, j], real[kh - 1, j]) for j in range(final.shape[1])]))}
        if fam.heading is not None:
            dpsi = tr[:, sub, fam.heading] - seg.x[k0:k1][sub, fam.heading]
            row['heading_err_max_deg'] = float(np.degrees(np.abs(np.median(dpsi, 0)).max()))
        for a in alphas:
            q = float(np.quantile(e, 1 - a))
            row[f'radius@{a}'] = q
            row[f's@{a}'] = e_real / max(q, 1e-9)
        out.append(row)
    return out


def clopper_pearson_lower(k, n, conf=0.95):
    """One-sided lower bound p_L with P(X >= k | n, p_L) = 1 - conf."""
    if k <= 0:
        return 0.0

    def tail(p):
        return sum(math.comb(n, i) * p ** i * (1 - p) ** (n - i) for i in range(k, n + 1))
    lo, hi = 0.0, 1.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if tail(mid) < 1 - conf:
            lo = mid
        else:
            hi = mid
    return lo


def conformal_quantile(scores, alpha):
    s = np.sort(np.asarray(scores, float))
    n = len(s)
    k = math.ceil((n + 1) * (1 - alpha))
    return float('inf') if k > n else float(s[k - 1])


def cluster_bootstrap_lower(covered, sessions, conf=0.95, B=4000, seed=0):
    rng = np.random.default_rng(seed)
    covered, sessions = np.asarray(covered, float), np.asarray(sessions)
    units = sorted(set(sessions))
    sums = np.array([covered[sessions == u].sum() for u in units])
    cnts = np.array([(sessions == u).sum() for u in units])
    idx = rng.integers(0, len(units), (B, len(units)))
    return float(np.quantile(sums[idx].sum(1) / cnts[idx].sum(1), 1 - conf))


def summarize(rows, alpha=0.1, H=5.0):
    """Coverage of one (event, horizon, alpha) over out-of-fold rows."""
    R = [r for r in rows if r['H'] == H]
    if not R:
        return None
    s = np.array([r[f's@{alpha}'] for r in R])
    sess = np.array([r['session'] for r in R])
    raw = s <= 1.0
    mult = np.array([conformal_quantile(s[sess != r['session']], alpha) for r in R])
    conf_cov = s <= mult
    radius = np.array([r[f'radius@{alpha}'] for r in R])
    k, n = int(conf_cov.sum()), len(R)
    finite = np.isfinite(mult)
    return {
        'event': f'pos_err_max@H{H:g}s', 'alpha': alpha, 'units': 'session', 'n_units': len(set(sess)),
        'n_segments': n,
        'raw': {'covered_frac': float(raw.mean()), 'lcb95_cp': clopper_pearson_lower(int(raw.sum()), n),
                'width_median_m': float(np.median(radius)), 'label': 'uncalibrated particle spread'},
        'covered_frac': k / n, 'lcb95_cp': clopper_pearson_lower(k, n),
        'lcb95_cluster_bootstrap': cluster_bootstrap_lower(conf_cov, sess),
        'width_median_m': float(np.median((radius * mult)[finite])) if finite.any() else float('inf'),
        'conformal_multiplier': conformal_quantile(s, alpha),
        'crps_final_m': float(np.mean([r['crps_final'] for r in R])),
        'mean_path_err_max_median_m': float(np.median([r['e_real'] for r in R])),
        'heading_err_max_median_deg': float(np.median([r.get('heading_err_max_deg', np.nan) for r in R])),
    }


def cross_validate(parent, evidence, *, horizons=HORIZONS, alphas=(0.1,), n=128, lam=1.0,
                   n_particles=512, seed=7, log=print):
    """Leave-one-session-out: (child rows, parent rows). The parent is scored on every segment
    BEFORE any fit (prequential); each child only on the session it never saw."""
    eligible = [s for s in evidence.segments if s.t_event is not None]
    parent_rows, child_rows = [], []
    for seg in eligible:
        parent_rows += score_segment(parent, seg, horizons, alphas, n=n, seed=seed)
    for b in evidence.sessions():
        train, test = evidence.split([b])
        test = [s for s in test.segments if s.t_event is not None]
        if not test or not train.segments:
            continue
        doc, parts, support, _ = propose(parent, train, lam=lam, n_particles=n_particles, seed=seed)
        child = Card(doc, parts, support)
        for seg in test:
            child_rows += score_segment(child, seg, horizons, alphas, n=n, seed=seed)
        log(f'  fold {b}: {len(test)} held-out segments, delay {doc["nominal"]["delay_s"] if "delay_s" in doc["nominal"] else "-"}')
    return child_rows, parent_rows


def promotion(child_summary, parent_summary, alpha=0.1, slack=0.05):
    """The frozen policy (design 5.6): LCB >= 1 - alpha - slack, width <= 1.1 x parent's, CRPS no
    worse than the parent's. A human still accepts (``accepted_by``)."""
    c, p = child_summary, parent_summary
    checks = {
        'coverage_lcb': c['lcb95_cp'] >= 1 - alpha - slack,
        'width_vs_parent': c['width_median_m'] <= 1.1 * p['width_median_m'],
        'crps_vs_parent': c['crps_final_m'] <= p['crps_final_m'],
    }
    return {'passed': all(checks.values()), 'checks': checks, 'target_lcb': 1 - alpha - slack}
