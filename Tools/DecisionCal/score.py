"""Selection regret, noise band, coverage and the M0 kill verdict (pre-registered 2026-11-09).

Per hidden boat h: J_h [C, K] (every controller, K repeats with common random numbers) and, per
arm a, J_a [C, M] (every controller on M twin particles). Arm a selects argmin_c mean_m J_a[c].

* regret (cross-fit): split the K repeats into halves A, B; the oracle is the best controller on
  A, both are scored on B, and the two splits are averaged. This removes the optimism of the
  min-of-noisy-means oracle, so regret can be slightly negative.
* raw regret: mean_K J_h[sel] - min_c mean_K J_h[c] (biased up; reported alongside).
* noise band: 2 SD_K(J_h[oracle]) / sqrt(K/2) -- the resolution of a half-split comparison.
* coverage: the hidden mean of the selected controller inside the arm's 10-90% particle interval.
* kill: H1 is dead if, in EVERY stratum, the nominal arm's regret is within the noise band on
  >= 80% of boats.
"""

import numpy as np


J_FAIL = 300.0          # a non-finite particle score counts as worse than any finite mission (review M2)
MARGIN_S = 2.0          # practical-equivalence margin: ~5% of a 40 s mission (amendment M0b)


def finite_scores(J_twin):
    J = np.asarray(J_twin, float)
    return np.where(np.isfinite(J), J, J_FAIL)


def selection(J_twin):
    return int(np.argmin(finite_scores(J_twin).mean(axis=1)))


def cross_fit_regret(J_h, sel):
    K = J_h.shape[1]
    A, B = np.arange(0, K // 2), np.arange(K // 2, 2 * (K // 2))
    out = []
    for a, b in ((A, B), (B, A)):
        oracle = int(np.argmin(J_h[:, a].mean(1)))
        out.append(J_h[sel, b].mean() - J_h[oracle, b].mean())
    return float(np.mean(out))


J_ABSTAIN = 180.0       # a mission that never docks scores >= 120 + 60


def plant_scores(J_h, J_arms, default_arm='nominal'):
    """Per-arm scores for one hidden boat, under two decision rules:

    * R0 argmin: select argmin_c mean_m J_arm[c]  (fields sel, regret);
    * R1 abstain: if the twin predicts that even its selected controller fails to dock on at least half
      of its particles (median_m J_arm[sel] >= J_ABSTAIN), the twin is not ranking controllers; fall
      back to the default arm's selection (fields sel_abst, regret_abst, abstained)."""
    J_h = finite_scores(J_h)
    K = J_h.shape[1]
    mean_h = J_h.mean(1)
    oracle = int(np.argmin(mean_h))
    band = float(2 * J_h[oracle].std(ddof=1) / np.sqrt(max(K // 2, 1))) if K > 1 else 0.0
    out = {'oracle': oracle, 'band': band, 'J_oracle': float(mean_h[oracle]), 'arms': {}}
    default_sel = selection(J_arms[default_arm]) if default_arm in J_arms else None
    for arm, Jt in J_arms.items():
        sel = selection(Jt)
        Jf = finite_scores(Jt)
        lo, hi = np.quantile(Jf[sel], [0.1, 0.9])
        abstained = bool(default_sel is not None and np.median(Jf[sel]) >= J_ABSTAIN)
        sel_a = default_sel if abstained else sel
        reg = cross_fit_regret(J_h, sel)
        out['arms'][arm] = {'sel': sel, 'regret': reg,
                            'regret_raw': float(mean_h[sel] - mean_h[oracle]),
                            'within_band': bool(reg <= max(band, MARGIN_S)),
                            'finite_frac': float(np.isfinite(np.asarray(Jt, float)).mean()),
                            'abstained': abstained, 'sel_abst': sel_a, 'regret_abst': cross_fit_regret(J_h, sel_a),
                            'pred_dock_frac_sel': float((Jf[sel] < J_ABSTAIN).mean()),
                            'pred_dock_frac_any': float((Jf < J_ABSTAIN).any(0).mean()),
                            'covered': bool(lo <= mean_h[sel] <= hi),
                            'pred_J': float(Jf[sel].mean()), 'hidden_J': float(mean_h[sel]),
                            'rank_corr': spearman(Jf.mean(1), mean_h)}
    return out


def spearman(a, b):
    ra, rb = np.argsort(np.argsort(a)), np.argsort(np.argsort(b))
    if ra.std() == 0 or rb.std() == 0:
        return float('nan')
    return float(np.corrcoef(ra, rb)[0, 1])


def sign_flip_p(d, n_perm=20000, seed=0):
    """Two-sided paired sign-flip permutation p-value for mean(d) = 0 (exact for n <= 16)."""
    d = np.asarray(d, float)
    n = len(d)
    obs = abs(d.mean())
    if n <= 16:
        signs = ((np.arange(2 ** n)[:, None] >> np.arange(n)) & 1) * 2 - 1
    else:
        signs = np.random.default_rng(seed).choice([-1, 1], size=(n_perm, n))
    return float(np.mean(np.abs((signs * d).mean(1)) >= obs - 1e-12))


def holm(pvals):
    """Holm step-down adjusted p-values (dict key -> p)."""
    keys = sorted(pvals, key=pvals.get)
    m, out, run = len(keys), {}, 0.0
    for i, k in enumerate(keys):
        run = max(run, min(1.0, (m - i) * pvals[k]))
        out[k] = run
    return out


def kendall_tau(x, y):
    c = d = 0
    for i in range(len(x)):
        for j in range(i + 1, len(x)):
            p = (x[i] - x[j]) * (y[i] - y[j])
            c += p > 0
            d += p < 0
    return (c - d) / max(c + d, 1)


def hypotheses(rows, arms, alpha=0.1):
    """H1: per (stratum, arm != nominal), paired gain over nominal, sign-flip p, Holm over the whole
    family. Supported if some calibrated arm has mean gain > 0 with Holm p < alpha. H2: Kendall tau
    between per-arm median open-loop trajectory error and mean regret (pooled), supported if < 0.5."""
    strata = sorted({r['stratum'] for r in rows})
    p, gain = {}, {}
    for st in strata:
        R = [r for r in rows if r['stratum'] == st]
        nom = np.array([r['scores']['arms']['nominal']['regret'] for r in R])
        for arm in arms:
            if arm == 'nominal':
                continue
            d = nom - np.array([r['scores']['arms'][arm]['regret'] for r in R])
            p[(st, arm)], gain[(st, arm)] = sign_flip_p(d), float(d.mean())
    adj = holm(p)
    calibrated = [a for a in arms if a not in ('nominal', 'nominal_d100', 'broad')]
    wins = [f'{st}:{a}' for (st, a), q in adj.items() if a in calibrated and q < alpha and gain[(st, a)] > 0]
    harms = [f'{st}:{a}' for (st, a), q in adj.items() if q < alpha and gain[(st, a)] < 0]
    traj = [float(np.nanmedian([np.nan if r['traj_rmse'][a] is None else r['traj_rmse'][a] for r in rows])) for a in arms]
    reg = [float(np.mean([r['scores']['arms'][a]['regret'] for r in rows])) for a in arms]
    tau = kendall_tau(traj, reg)
    return {'H1': {'supported': bool(wins), 'wins': wins, 'harms': harms, 'alpha': alpha,
                   'tests': {f'{st}:{a}': {'gain': gain[(st, a)], 'p': p[(st, a)], 'p_holm': adj[(st, a)]}
                             for (st, a) in p}},
            'H2': {'supported': bool(tau < 0.5), 'kendall_tau': tau, 'traj_median': dict(zip(arms, traj)),
                   'mean_regret': dict(zip(arms, reg))}}


def summarize(rows, arms, n_boot=2000, seed=0):
    """Per stratum and arm: median/mean regret with a bootstrap CI over boats, P(within band),
    worst decile, coverage, rank correlation; the nominal-relative paired improvement; the kill
    verdict."""
    rng = np.random.default_rng(seed)
    strata = sorted({r['stratum'] for r in rows}) + ['all']
    table = {}
    for st in strata:
        R = [r for r in rows if st == 'all' or r['stratum'] == st]
        if not R:
            continue
        table[st] = {'n': len(R)}
        nom = np.array([r['scores']['arms']['nominal']['regret'] for r in R])
        for arm in arms:
            reg = np.array([r['scores']['arms'][arm]['regret'] for r in R])
            wb = np.array([r['scores']['arms'][arm]['within_band'] for r in R])
            cov = np.array([r['scores']['arms'][arm]['covered'] for r in R])
            rc = np.array([r['scores']['arms'][arm]['rank_corr'] for r in R])
            idx = rng.integers(0, len(R), (n_boot, len(R)))
            diff = nom - reg
            table[st][arm] = {
                'median_regret': float(np.median(reg)), 'mean_regret': float(reg.mean()),
                'mean_regret_ci90': [float(x) for x in np.quantile(reg[idx].mean(1), [0.05, 0.95])],
                'worst_decile': float(np.quantile(reg, 0.9)), 'p_within_band': float(wb.mean()),
                'coverage': float(cov.mean()), 'rank_corr_median': float(np.nanmedian(rc)),
                'gain_vs_nominal': float(diff.mean()),
                'gain_vs_nominal_ci90': [float(x) for x in np.quantile(diff[idx].mean(1), [0.05, 0.95])]}
    per = {st: table[st]['nominal']['p_within_band'] for st in table if st != 'all'}
    dead = bool(per) and all(v >= 0.8 for v in per.values())
    return {'table': table, 'kill': {'dead': dead, 'nominal_p_within_band': per,
                                     'rule': '>=0.8 in every stratum, within = regret <= max(band, 2 s)'},
            'hypotheses': hypotheses(rows, arms)}
