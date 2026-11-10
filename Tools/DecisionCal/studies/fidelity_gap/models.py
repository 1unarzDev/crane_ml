#!/usr/bin/env python3
"""EXPLORATORY. Step 2 of the fidelity-gap study: discrepancy models between the fast layer (NumPy) and the
integrated layer (Unity), validated leave-one-boat-out (LOBO, 4 folds) and leave-one-controller-out (LOCO,
16 folds). Reads out/paired_table.json only; no simulation.

Predict J_unity from the fast layer:
  a  face value            J_u ~ J_f
  b  constant bias         J_u ~ J_f + mean(train J_u - J_f)        (b_med: median)
  c  Kennedy-O'Hagan style J_u ~ rho J_f + delta(raw boat + controller parameters)
       c_ridge: delta linear, ridge;   c_gp: delta a zero-mean isotropic-RBF GP on the residual of rho J_f + b
  d  mechanism-indexed     J_u ~ rho J_f + delta(interaction features of the fast trajectory), ridge
       d_hurdle: P(Unity fails | fast docked, features) x failure score + (1 - P) x docked-time model
       cd: raw parameters and interaction features together
  e  classifiers           e_dock: the layers disagree on dock / no-dock (per mission)
                           e_pair: the fast layer's pick of a controller pair is wrong on Unity (per pair)
All regularisation is chosen by grouped CV inside the training folds only (nested).

    ulimit -v 1500000; nice -n 19 python3 -I Tools/DecisionCal/studies/fidelity_gap/models.py
"""

import itertools
import sys
from pathlib import Path

import numpy as np

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))

import fg_common as F  # noqa: E402

N_PERM = {'lobo': 100, 'loco': 30}   # permutations per validation scheme (compute budget)
N_PERM_PAIR = {'lobo': 100, 'loco': 0}
MARGIN_S = 2.0            # score.MARGIN_S: a pick more than 2 s worse on Unity is "wrong"
SEED = 20261110


# ------------------------------------------------------------------ regression models (predict J_u)

def _ridge_model(Xtr, ftr, ytr, gtr, Xte, fte):
    """rho J_f (unpenalised, with intercept) + ridge on standardised X; lambda by inner grouped CV."""
    S = F.Standardizer(Xtr)
    Ztr, Zte = S(Xtr), S(Xte)

    def make(lam, tr):
        p = F.ridge_fit(Ztr[tr], ftr[tr, None], ytr[tr], lam)
        return lambda te: p(Ztr[te], ftr[te, None])
    lam = F.choose_lam(make, lambda pred, te: float(np.abs(pred - ytr[te]).sum()), gtr, len(ytr))
    p = F.ridge_fit(Ztr, ftr[:, None], ytr, lam)
    return p(Zte, fte[:, None]), {'lam': lam}


def _gp_model(Xtr, ftr, ytr, gtr, Xte, fte):
    A = np.column_stack([np.ones(len(ftr)), ftr])
    w = np.linalg.lstsq(A, ytr, rcond=None)[0]
    S = F.Standardizer(Xtr)
    gp = F.gp_fit(S(Xtr), ytr - A @ w)
    mean, sd = gp(S(Xte))
    return w[0] + w[1] * fte + mean, {'sd': sd, **gp.hyper, 'rho': float(w[1])}


def _hurdle_model(Xtr, ftr, ytr, gtr, Xte, fte, dtr_f, dte_f, dtr_u):
    """Two-part: Unity dock probability from [fast docked, features]; docked-time model = J_f + bias when the
    fast run docked, else the training mean of Unity-docked scores; failure score = training mean of
    Unity-failed scores (180 = the score's floor for a failed dock when the training folds hold none)."""
    S = F.Standardizer(np.column_stack([dtr_f, Xtr]))
    Ztr, Zte = S(np.column_stack([dtr_f, Xtr])), S(np.column_stack([dte_f, Xte]))
    fail = 1.0 - dtr_u
    if fail.sum() == 0 or fail.sum() == len(fail):
        p_fail = np.full(len(fte), fail.mean())
        lam = None
    else:
        def make(lam, tr):
            p = F.logit_fit(Ztr[tr], fail[tr], lam)
            return lambda te: p(Ztr[te])

        def nll(pred, te):
            pred = np.clip(pred, 1e-6, 1 - 1e-6)
            return float(-(fail[te] * np.log(pred) + (1 - fail[te]) * np.log(1 - pred)).sum())
        lam = F.choose_lam(make, nll, gtr, len(ytr))
        p_fail = F.logit_fit(Ztr, fail, lam)(Zte)
    both = (dtr_f == 1) & (dtr_u == 1)
    b_dock = float((ytr - ftr)[both].mean()) if both.any() else 0.0
    j_dock_mean = float(ytr[dtr_u == 1].mean()) if (dtr_u == 1).any() else float(ytr.mean())
    j_fail = float(ytr[dtr_u == 0].mean()) if (dtr_u == 0).any() else 180.0
    y_dock = np.where(dte_f == 1, fte + b_dock, j_dock_mean)
    return (1 - p_fail) * y_dock + p_fail * j_fail, {'lam': lam, 'p_fail': p_fail}


def regression_cv(T, fast, groups, perm=None):
    """Out-of-fold predictions of every model. ``perm``: row permutation applied to the interaction features
    only (null for 'the features carry information')."""
    f, y = T[f'{fast}_J'], T['u_J']
    df, du = T[f'{fast}_docked'], T['u_docked']
    Xraw = np.column_stack([F.cols(T, F.RAW_BOAT), F.cols(T, F.RAW_CTRL)])
    Xm = F.cols(T, F.MECH, f'{fast}_')
    if perm is not None:
        Xm = Xm[perm]
    n = len(y)
    names = ('a', 'b', 'b_med', 'c_ridge', 'c_gp', 'd_ridge', 'd_hurdle', 'cd_ridge') if perm is None else ('d_ridge', 'd_hurdle')
    P = {k: np.full(n, np.nan) for k in names}
    extra = {'c_gp_sd': np.full(n, np.nan), 'ridge_q90': {k: np.full(n, np.nan) for k in ('b', 'c_ridge', 'd_ridge')}}
    inner_groups = T['boat_id'] if groups is T['boat_id'] else T['controller']
    for g, tr, te in F.group_folds(groups):
        gtr = inner_groups[tr]
        if perm is None:
            P['a'][te] = f[te]
            P['b'][te] = f[te] + (y[tr] - f[tr]).mean()
            P['b_med'][te] = f[te] + np.median(y[tr] - f[tr])
            P['c_ridge'][te], _ = _ridge_model(Xraw[tr], f[tr], y[tr], gtr, Xraw[te], f[te])
            P['c_gp'][te], info = _gp_model(Xraw[tr], f[tr], y[tr], gtr, Xraw[te], f[te])
            extra['c_gp_sd'][te] = info['sd']
            P['cd_ridge'][te], _ = _ridge_model(np.column_stack([Xraw, Xm])[tr], f[tr], y[tr], gtr,
                                                np.column_stack([Xraw, Xm])[te], f[te])
            # 90 % interval half-width from inner out-of-fold absolute residuals (training folds only)
            for k, X in (('b', None), ('c_ridge', Xraw), ('d_ridge', Xm)):
                res = np.full(len(tr), np.nan)
                for _, itr, ite in F.group_folds(gtr):
                    if X is None:
                        res[ite] = y[tr][ite] - (f[tr][ite] + (y[tr][itr] - f[tr][itr]).mean())
                    else:
                        pr, _ = _ridge_model(X[tr][itr], f[tr][itr], y[tr][itr], gtr[itr], X[tr][ite], f[tr][ite])
                        res[ite] = y[tr][ite] - pr
                extra['ridge_q90'][k][te] = np.quantile(np.abs(res), 0.9)
        P['d_ridge'][te], _ = _ridge_model(Xm[tr], f[tr], y[tr], gtr, Xm[te], f[te])
        P['d_hurdle'][te], _ = _hurdle_model(Xm[tr], f[tr], y[tr], gtr, Xm[te], f[te], df[tr], df[te], du[tr])
    return P, extra


def score_regression(T, P):
    y = T['u_J']
    out = {}
    for k, p in P.items():
        e = np.abs(p - y)
        rho = [F.spearman(p[T['block'] == b], y[T['block'] == b]) for b in np.unique(T['block'])]
        out[k] = {'mae': float(e.mean()), 'rmse': float(np.sqrt(((p - y) ** 2).mean())),
                  'mae_by_boat': {b: float(e[T['boat'] == b].mean()) for b in F.BOATS},
                  'mae_by_block': [float(e[T['block'] == b].mean()) for b in np.unique(T['block'])],
                  'median_abs': float(np.median(e)), 'within_block_spearman_mean': float(np.nanmean(rho))}
    return out


def compare_to(T, sc, base, cand):
    """Block-level paired difference in MAE (cand - base): exact sign-flip p over the 8 (boat, seed) blocks and
    the count of boats where cand is better. 8 blocks are not independent (3 per boat on two boats)."""
    d = np.array(sc[cand]['mae_by_block']) - np.array(sc[base]['mae_by_block'])
    boats_better = sum(sc[cand]['mae_by_boat'][b] < sc[base]['mae_by_boat'][b] for b in F.BOATS)
    return {'mae_diff': float(sc[cand]['mae'] - sc[base]['mae']), 'p_signflip_blocks': F.sign_flip_exact(d),
            'blocks_better': int((d < 0).sum()), 'boats_better_of_4': int(boats_better)}


# ------------------------------------------------------------------ classifiers

def _logit_cv(X, lab, groups, inner_groups):
    """Out-of-fold probabilities; base-rate prediction when a training fold has one class only."""
    n = len(lab)
    p = np.full(n, np.nan)
    p0 = np.full(n, np.nan)
    degenerate = []
    for g, tr, te in F.group_folds(groups):
        ytr = lab[tr]
        p0[te] = ytr.mean()
        if ytr.sum() == 0 or ytr.sum() == len(ytr):
            p[te] = ytr.mean()
            degenerate.append(g)
            continue
        S = F.Standardizer(X[tr])
        Ztr, Zte = S(X[tr]), S(X[te])

        def make(lam, itr):
            m = F.logit_fit(Ztr[itr], ytr[itr], lam)
            return lambda ite: m(Ztr[ite])

        def nll(pred, ite):
            pred = np.clip(pred, 1e-6, 1 - 1e-6)
            return float(-(ytr[ite] * np.log(pred) + (1 - ytr[ite]) * np.log(1 - pred)).sum())
        lam = F.choose_lam(make, nll, inner_groups[tr], len(tr))
        p[te] = F.logit_fit(Ztr, ytr, lam)(Zte)
    return p, p0, degenerate


def reliability(p, lab, edges=(0.0, 0.1, 0.3, 1.0001)):
    out = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (p >= lo) & (p < hi)
        k, n = int(lab[m].sum()), int(m.sum())
        out.append({'bin': [lo, min(hi, 1.0)], 'n': n, 'mean_p': float(p[m].mean()) if n else None,
                    'observed': (k / n) if n else None, 'cp95': list(F.clopper_pearson(k, n)) if n else None})
    return out


def score_classifier(p, p0, lab, strata):
    """Brier vs the training base rate, pooled AUC, and AUC inside each stratum that has both classes."""
    by = {}
    for s in np.unique(strata):
        m = strata == s
        by[str(s)] = {'n': int(m.sum()), 'pos': int(lab[m].sum()), 'auc': F.auc(p[m], lab[m])}
    return {'n': len(lab), 'pos': int(lab.sum()), 'base_rate_cp95': list(F.clopper_pearson(int(lab.sum()), len(lab))),
            'brier': F.brier(p, lab), 'brier_base_rate': F.brier(p0, lab),
            'brier_skill': 1 - F.brier(p, lab) / max(F.brier(p0, lab), 1e-12), 'auc': F.auc(p, lab),
            'by_stratum': by, 'reliability': reliability(p, lab)}


def dock_classifier(T, fast, groups, perm=None):
    lab = (T['u_docked'] != T[f'{fast}_docked']).astype(float)
    Xm = F.cols(T, F.MECH, f'{fast}_')
    if perm is not None:
        Xm = Xm[perm]
    X = np.column_stack([T[f'{fast}_docked'], Xm])
    inner = T['boat_id'] if groups is T['boat_id'] else T['controller']
    p, p0, deg = _logit_cv(X, lab, groups, inner)
    return p, p0, lab, deg


PAIR_MECH = ('x_frac_slow_in_window', 'x_hold_breaks', 'x_hold_margin', 'x_speed_p90_window', 'x_dwell_before_hold',
             'x_rev_per_s', 'x_rev_per_lag', 'x_rev_hold_duty', 'x_frac_cmd_in_deadband', 'x_frac_cmd_below_snap',
             'x_cmd_min_over_deadband', 'x_lat_force_used', 'x_knife_edge')


def pair_table(T, fast, perm=None):
    """One row per controller pair per (boat, seed) block (120 x 8 = 960). The fast layer's pick is the lower
    J_f; label wrong = the pick is more than MARGIN_S worse than the other on Unity (same block)."""
    Xm = F.cols(T, PAIR_MECH, f'{fast}_')
    if perm is not None:
        Xm = Xm[perm]
    f, y, df = T[f'{fast}_J'], T['u_J'], T[f'{fast}_docked']
    rows = {k: [] for k in ('block', 'boat_id', 'pick', 'other', 'gap', 'wrong', 'flip', 'regret', 'i_pick', 'i_other')}
    feats = []
    for b in np.unique(T['block']):
        idx = np.where(T['block'] == b)[0]
        for i, j in itertools.combinations(idx, 2):
            pk, ot = (i, j) if f[i] <= f[j] else (j, i)
            reg = max(y[pk] - y[ot], 0.0)
            rows['block'].append(b)
            rows['boat_id'].append(T['boat_id'][i])
            rows['pick'].append(T['controller'][pk])
            rows['other'].append(T['controller'][ot])
            rows['gap'].append(f[ot] - f[pk])
            rows['regret'].append(reg)
            rows['wrong'].append(float(reg > MARGIN_S))
            rows['flip'].append(float(y[pk] > y[ot]))
            rows['i_pick'].append(pk)
            rows['i_other'].append(ot)
            feats.append(np.concatenate([[df[pk], df[ot]], Xm[pk], Xm[ot]]))
    R = {k: np.array(v) for k, v in rows.items()}
    R['log_gap'] = np.log1p(R['gap'])
    R['X_mech'] = np.array(feats)
    return R


def pair_classifier(R, scheme, with_mech=True):
    """LOBO: folds are boats. LOCO: for each controller c, test on the pairs that involve c, train on the pairs
    that do not (a pair is tested twice, once per member; probabilities averaged)."""
    lab = R['wrong']
    X = np.column_stack([R['log_gap'], R['X_mech']]) if with_mech else R['log_gap'][:, None]
    if scheme == 'lobo':
        p, p0, deg = _logit_cv(X, lab, R['boat_id'], R['boat_id'])
        return p, p0
    n = len(lab)
    acc, acc0, cnt = np.zeros(n), np.zeros(n), np.zeros(n)
    for c in range(16):
        te = np.where((R['pick'] == c) | (R['other'] == c))[0]
        tr = np.where((R['pick'] != c) & (R['other'] != c))[0]
        S = F.Standardizer(X[tr])
        Ztr, ytr = S(X[tr]), lab[tr]

        def make(lam, itr):
            m = F.logit_fit(Ztr[itr], ytr[itr], lam)
            return lambda ite: m(Ztr[ite])

        def nll(pred, ite):
            pred = np.clip(pred, 1e-6, 1 - 1e-6)
            return float(-(ytr[ite] * np.log(pred) + (1 - ytr[ite]) * np.log(1 - pred)).sum())
        lam = F.choose_lam(make, nll, R['boat_id'][tr], len(tr))
        acc[te] += F.logit_fit(Ztr, ytr, lam)(S(X[te]))
        acc0[te] += ytr.mean()
        cnt[te] += 1
    return acc / cnt, acc0 / cnt


# ------------------------------------------------------------------ driver

def univariate(T, fast, rng):
    """Descriptive, NOT validated: Spearman of each interaction feature with J_u - J_f, with a within-block
    permutation p (the feature shuffled among the 16 controllers of each (boat, seed) block) and Holm."""
    d = T['u_J'] - T[f'{fast}_J']
    rows = []
    perms = [F.within_block_perm(T['block'], rng) for _ in range(2000)]
    for name in F.MECH:
        x = T[f'{fast}_{name}']
        if x.std() == 0:
            continue
        # block-centred ranks: association inside blocks only
        def stat(xx):
            return float(np.mean([F.spearman(xx[T['block'] == b], d[T['block'] == b]) if xx[T['block'] == b].std() > 0 else 0.0
                                  for b in np.unique(T['block'])]))
        s0 = stat(x)
        null = np.array([stat(x[p]) for p in perms[:500]])
        p = float((1 + (np.abs(null) >= abs(s0) - 1e-12).sum()) / (1 + len(null)))
        rows.append({'feature': name, 'pooled_spearman': F.spearman(x, d), 'within_block_spearman': s0, 'p_perm': p})
    order = np.argsort([r['p_perm'] for r in rows])
    m = len(rows)
    run = 0.0
    for rank, i in enumerate(order):
        run = max(run, min(1.0, rows[i]['p_perm'] * (m - rank)))
        rows[i]['p_holm'] = run
    return rows


def main():
    T = F.load_table()
    rng = np.random.default_rng(SEED)
    res = {'n': {'missions': len(T['u_J']), 'boats': 4, 'controllers': 16, 'blocks': 8,
                 'per_boat': {b: int((T['boat'] == b).sum()) for b in F.BOATS}}}
    for fast in ('rep', 'hid'):
        R_ = {}
        d = T['u_J'] - T[f'{fast}_J']
        dis = T['u_docked'] != T[f'{fast}_docked']
        R_['describe'] = {
            'dJ_mean': float(d.mean()), 'dJ_median': float(np.median(d)), 'dJ_abs_median': float(np.median(np.abs(d))),
            'dJ_by_boat': {b: {'mean': float(d[T['boat'] == b].mean()), 'median': float(np.median(d[T['boat'] == b])),
                               'abs_median': float(np.median(np.abs(d[T['boat'] == b])))} for b in F.BOATS},
            'dock_disagree': {b: {'k': int(dis[T['boat'] == b].sum()), 'n': int((T['boat'] == b).sum()),
                                  'cp95': list(F.clopper_pearson(int(dis[T['boat'] == b].sum()), int((T['boat'] == b).sum())))}
                              for b in F.BOATS},
            'dock_table': {f'u{int(u)}_f{int(v)}': int(((T['u_docked'] == u) & (T[f'{fast}_docked'] == v)).sum())
                           for u in (0, 1) for v in (0, 1)},
            'dJ_abs_median_both_docked': float(np.median(np.abs(d[(T['u_docked'] == 1) & (T[f'{fast}_docked'] == 1)]))),
            'n_both_docked': int(((T['u_docked'] == 1) & (T[f'{fast}_docked'] == 1)).sum())}
        R_['univariate'] = univariate(T, fast, rng)
        for scheme, groups in (('lobo', T['boat_id']), ('loco', T['controller'])):
            P, extra = regression_cv(T, fast, groups)
            sc = score_regression(T, P)
            y = T['u_J']
            cov = {}
            for k in ('b', 'c_ridge', 'd_ridge'):
                inside = np.abs(P[k] - y) <= extra['ridge_q90'][k]
                cov[k] = {'nominal': 0.9, 'covered': int(inside.sum()), 'n': len(y),
                          'cp95': list(F.clopper_pearson(int(inside.sum()), len(y))),
                          'by_boat': {b: float(inside[T['boat'] == b].mean()) for b in F.BOATS}}
            inside = np.abs(P['c_gp'] - y) <= 1.645 * extra['c_gp_sd']
            cov['c_gp'] = {'nominal': 0.9, 'covered': int(inside.sum()), 'n': len(y),
                           'cp95': list(F.clopper_pearson(int(inside.sum()), len(y))),
                           'by_boat': {b: float(inside[T['boat'] == b].mean()) for b in F.BOATS}}
            comp = {f'{c}_vs_{b}': compare_to(T, sc, b, c) for b in ('a', 'b') for c in ('b', 'c_ridge', 'c_gp', 'd_ridge', 'd_hurdle', 'cd_ridge') if c != b}
            # null for (d): interaction features shuffled inside each block
            null = {'d_ridge': [], 'd_hurdle': []}
            for _ in range(N_PERM[scheme]):
                Pp, _e = regression_cv(T, fast, groups, perm=F.within_block_perm(T['block'], rng))
                for k in null:
                    null[k].append(float(np.abs(Pp[k] - y).mean()))
            perm_p = {k: {'mae': sc[k]['mae'], 'null_mae_median': float(np.median(v)), 'null_mae_q05': float(np.quantile(v, 0.05)),
                          'p_perm_features': float((1 + (np.array(v) <= sc[k]['mae'] + 1e-12).sum()) / (1 + N_PERM[scheme])), 'n_perm': N_PERM[scheme]}
                      for k, v in null.items()}
            # (e) dock disagreement
            p, p0, lab, deg = dock_classifier(T, fast, groups)
            e_dock = score_classifier(p, p0, lab, T['boat'])
            e_dock['degenerate_training_folds'] = [int(g) for g in deg]
            nulls = []
            for _ in range(N_PERM[scheme]):
                pp, _, _, _ = dock_classifier(T, fast, groups, perm=F.within_block_perm(T['block'], rng))
                nulls.append(F.auc(pp, lab))
            nulls = np.array(nulls)
            e_dock['auc_perm'] = {'null_median': float(np.nanmedian(nulls)),
                                  'p': float((1 + (nulls >= e_dock['auc'] - 1e-12).sum()) / (1 + N_PERM[scheme])) if not np.isnan(e_dock['auc']) else None,
                                  'n_perm': N_PERM[scheme]}
            # (e) pairwise
            Rp = pair_table(T, fast)
            pm, p0m = pair_classifier(Rp, scheme, True)
            pg, p0g = pair_classifier(Rp, scheme, False)
            e_pair = {'mech+gap': score_classifier(pm, p0m, Rp['wrong'], np.array(F.BOATS)[Rp['boat_id'].astype(int)]),
                      'gap_only': score_classifier(pg, p0g, Rp['wrong'], np.array(F.BOATS)[Rp['boat_id'].astype(int)]),
                      'raw_gap_auc': F.auc(-Rp['gap'], Rp['wrong']),
                      'n_pairs': len(Rp['wrong']), 'wrong': int(Rp['wrong'].sum()), 'strict_flip': int(Rp['flip'].sum()),
                      'wrong_by_boat': {b: {'k': int(Rp['wrong'][Rp['boat_id'] == i].sum()), 'n': int((Rp['boat_id'] == i).sum()),
                                            'mean_regret': float(Rp['regret'][Rp['boat_id'] == i].mean())}
                                        for i, b in enumerate(F.BOATS)}}
            n_perm_pair = N_PERM_PAIR[scheme]
            nulls, nullb = [], []
            for _ in range(n_perm_pair):
                Rn = pair_table(T, fast, perm=F.within_block_perm(T['block'], rng))
                pn, _ = pair_classifier(Rn, scheme, True)
                nulls.append(F.auc(pn, Rp['wrong']))
                nullb.append(F.brier(pn, Rp['wrong']))
            e_pair['perm_mech_features'] = None if not n_perm_pair else {
                'n_perm': n_perm_pair, 'null_auc_median': float(np.median(nulls)), 'null_auc_q95': float(np.quantile(nulls, 0.95)),
                'p_auc': float((1 + (np.array(nulls) >= e_pair['mech+gap']['auc'] - 1e-12).sum()) / (1 + n_perm_pair)),
                'p_brier': float((1 + (np.array(nullb) <= e_pair['mech+gap']['brier'] + 1e-12).sum()) / (1 + n_perm_pair))}
            R_[scheme] = {'regression': sc, 'coverage90': cov, 'comparisons': comp, 'perm_features': perm_p,
                          'e_dock': e_dock, 'e_pair': e_pair,
                          'oof': {'J_pred': {k: v for k, v in P.items()}, 'p_dock_disagree': p, 'p_pair_wrong_mech': pm,
                                  'p_pair_wrong_gap': pg}}
            print(f'[{fast} {scheme}] done', file=sys.stderr, flush=True)
        res[fast] = R_
    F.jdump(res, F.OUT / 'results_models.json')
    report(res)


def report(res):
    L = []
    for fast in ('rep', 'hid'):
        r = res[fast]
        L.append(f'=== fast layer = NumPy {"representable" if fast == "rep" else "full hidden"} boat  (EXPLORATORY; n = 128 missions, 4 boats, 8 blocks) ===')
        d = r['describe']
        L.append(f"J_u - J_f: mean {d['dJ_mean']:+.1f}, median {d['dJ_median']:+.1f}, median |.| {d['dJ_abs_median']:.1f} s; both docked (n {d['n_both_docked']}): median |.| {d['dJ_abs_median_both_docked']:.1f} s; dock table {d['dock_table']}")
        L.append('dock disagreement by boat: ' + '; '.join(f"{b} {v['k']}/{v['n']} [{v['cp95'][0]:.2f},{v['cp95'][1]:.2f}]" for b, v in d['dock_disagree'].items()))
        for scheme in ('lobo', 'loco'):
            s = r[scheme]
            L.append(f'-- {scheme.upper()} : held-out prediction of J_unity')
            L.append(f'{"model":10s} {"MAE":>7s} {"RMSE":>7s} {"med|e|":>7s} {"rho_blk":>7s}  MAE by boat (steady timing transient mixed)   vs b: dMAE  p(8 blocks)  boats better')
            for k, v in s['regression'].items():
                c = s['comparisons'].get(f'{k}_vs_b')
                cs = f"{c['mae_diff']:+7.2f}  {c['p_signflip_blocks']:.3f}  {c['boats_better_of_4']}/4" if c else ''
                L.append(f"{k:10s} {v['mae']:7.2f} {v['rmse']:7.2f} {v['median_abs']:7.2f} {v['within_block_spearman_mean']:7.2f}  "
                         + ' '.join(f"{v['mae_by_boat'][b]:7.2f}" for b in F.BOATS) + '   ' + cs)
            for k, v in s['perm_features'].items():
                L.append(f"   {k}: MAE {v['mae']:.2f} vs features shuffled within block median {v['null_mae_median']:.2f} (5% {v['null_mae_q05']:.2f}); p = {v['p_perm_features']:.3f}")
            L.append('   90% interval coverage: ' + '; '.join(f"{k} {v['covered']}/{v['n']} [{v['cp95'][0]:.2f},{v['cp95'][1]:.2f}] by boat " + '/'.join(f"{v['by_boat'][b]:.2f}" for b in F.BOATS) for k, v in s['coverage90'].items()))
            e = s['e_dock']
            L.append(f"   e_dock: {e['pos']}/{e['n']} disagree; Brier {e['brier']:.4f} vs base-rate {e['brier_base_rate']:.4f} (skill {e['brier_skill']:+.2f}); AUC {e['auc']:.2f} "
                     f"(perm null median {e['auc_perm']['null_median']:.2f}, p {e['auc_perm']['p']}); folds with a one-class training set: {e['degenerate_training_folds']}")
            L.append('      AUC within boat: ' + '; '.join(f"{b} {v['auc']:.2f} ({v['pos']}/{v['n']})" for b, v in e['by_stratum'].items()))
            L.append('      reliability: ' + '; '.join(f"p in [{x['bin'][0]:.1f},{x['bin'][1]:.1f}) n {x['n']} obs {x['observed'] if x['observed'] is None else round(x['observed'], 2)}" for x in e['reliability']))
            e = s['e_pair']
            L.append(f"   e_pair: {e['wrong']}/{e['n_pairs']} fast picks wrong by > 2 s on Unity (strict flips {e['strict_flip']}); by boat "
                     + '; '.join(f"{b} {v['k']}/{v['n']} (mean regret {v['mean_regret']:.1f})" for b, v in e['wrong_by_boat'].items()))
            for k in ('gap_only', 'mech+gap'):
                v = e[k]
                L.append(f"      {k:9s}: AUC {v['auc']:.3f}, Brier {v['brier']:.4f} vs base {v['brier_base_rate']:.4f} (skill {v['brier_skill']:+.2f}); within boat AUC "
                         + '; '.join(f"{b} {x['auc']:.2f}" for b, x in v['by_stratum'].items()))
                L.append('         reliability: ' + '; '.join(f"[{x['bin'][0]:.1f},{x['bin'][1]:.1f}) n {x['n']} obs {x['observed'] if x['observed'] is None else round(x['observed'], 2)}" for x in v['reliability']))
            pm = e['perm_mech_features'] or {'n_perm': 0, 'null_auc_median': float('nan'), 'null_auc_q95': float('nan'), 'p_auc': float('nan'), 'p_brier': float('nan')}
            L.append(f"      mechanism features shuffled within block ({pm['n_perm']} perms): null AUC median {pm['null_auc_median']:.3f} (95% {pm['null_auc_q95']:.3f}); p_auc {pm['p_auc']:.3f}, p_brier {pm['p_brier']:.3f}; raw -gap AUC {e['raw_gap_auc']:.3f}")
        L.append('-- univariate (descriptive, within-block Spearman with J_u - J_f; 500 within-block permutations; Holm)')
        for u in sorted(r['univariate'], key=lambda u: u['p_perm']):
            L.append(f"   {u['feature']:26s} pooled {u['pooled_spearman']:+.2f}  within-block {u['within_block_spearman']:+.2f}  p {u['p_perm']:.3f}  holm {u['p_holm']:.3f}")
        L.append('')
    txt = '\n'.join(L)
    (F.OUT / 'results_models.txt').write_text(txt + '\n')
    print(txt)


if __name__ == '__main__':
    main()
