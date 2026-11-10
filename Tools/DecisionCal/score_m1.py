"""M1 analysis (PREREG_M1.md): the pre-registered contrasts per (library, fitter) family, the
no-twin fixed-controller baselines, the lag dose-response and descriptive tables.

gain(A over B) = regret(B) - regret(A) per boat (positive: A selects better). Tests are paired
sign-flip permutation tests (score.sign_flip_p), Holm-adjusted within a family of six.
"""

import numpy as np

import score

MAIN = ('steady', 'transient', 'timing', 'mixed')
FAMILIES = {'v1/reg': ('v1', ''), 'v2/reg': ('v2', ''), 'v1/abc': ('v1', 'abc:'), 'v2/abc': ('v2', 'abc:')}
PRIMARY = 'v1/reg'
ALPHA = 0.10


def _reg(r, lib, arm, rule='R0'):
    return r['libs'][lib]['scores']['arms'][arm]['regret' if rule == 'R0' else 'regret_abst']


def fixed_regrets(rows, lib):
    """Per boat: cross-fit regret of every fixed controller [n, C], and of the leave-one-boat-out best
    fixed controller (chosen on the mean hidden J of the OTHER main-strata boats; no stratum knowledge)."""
    J = np.array([np.asarray(r['libs'][lib]['J_h'], float) for r in rows])          # [n, C, K]
    per = np.array([[score.cross_fit_regret(J[i], c) for c in range(J.shape[1])] for i in range(len(rows))])
    mean_c = J.mean(2)                                                                 # [n, C]
    main = np.array([r['stratum'] in MAIN for r in rows])
    loo, pick = np.empty(len(rows)), np.empty(len(rows), int)
    for i in range(len(rows)):
        m = main.copy()
        m[i] = False
        pick[i] = int(np.argmin(mean_c[m].mean(0)))
        loo[i] = per[i, pick[i]]
    return per, loo, pick


def _test(d):
    d = np.asarray(d, float)
    idx = np.random.default_rng(0).integers(0, len(d), (4000, len(d)))
    return {'n': int(len(d)), 'mean_gain': float(d.mean()), 'median_gain': float(np.median(d)),
            'ci90': [float(x) for x in np.quantile(d[idx].mean(1), [0.05, 0.95])],
            'frac_pos': float((d > 2.0).mean()), 'frac_neg': float((d < -2.0).mean()), 'p': score.sign_flip_p(d)}


def lag_slopes(rows, lib, arm, ref_arm):
    """Per base boat: OLS slope (s of regret per s of lag) of harm = regret(arm) - regret(ref_arm)
    against the lag level, and the slope of the arm's fitted delay against the lag level."""
    out_h, out_d = [], []
    bases = sorted({r['lag_base'] for r in rows if r['stratum'] == 'lagsweep'})
    for b in bases:
        R = sorted((r for r in rows if r['stratum'] == 'lagsweep' and r['lag_base'] == b), key=lambda r: r['lag_level'])
        x = np.array([r['lag_level'] for r in R])
        h = np.array([_reg(r, lib, arm) - _reg(r, lib, ref_arm) for r in R])
        out_h.append(float(np.polyfit(x, h, 1)[0]))
        f = [r['fits'][arm] for r in R]
        dly = np.array([(q.get('map') or q.get('best') or {}).get('delay_s', np.nan) for q in f], float)
        out_d.append(float(np.polyfit(x, dly, 1)[0]) if np.isfinite(dly).all() else float('nan'))
    return np.array(out_h), np.array(out_d)


def family(rows, lib, pre):
    """The six pre-registered tests for one (library, fitter). ``pre`` = '' (regression) or 'abc:'."""
    S = lambda st: [r for r in rows if r['stratum'] == st]
    g = lambda R, a, b, ra='R0', rb='R0': np.array([_reg(r, lib, b, rb) - _reg(r, lib, a, ra) for r in R])
    tests = {}
    if not pre:
        tests['T1 transient: delay-only (M0b arm) vs nominal, argmin (pred. harm, replication)'] = \
            _test(g(S('transient'), 'timing', 'nominal'))
        tests['T3 transient: delay-only (M0b arm), abstain rule vs argmin (pred. gain)'] = \
            _test(g(S('transient'), 'timing', 'timing', 'R1', 'R0'))
    tests['T2 transient: rigid+delay vs rigid, argmin (pred. harm: delay without lag)'] = \
        _test(g(S('transient'), pre + 'rigid+delay', pre + 'rigid'))
    tests['T4 timing: joint vs nominal, abstain rule (pred. gain)'] = \
        _test(g(S('timing'), pre + 'joint', 'nominal', 'R1', 'R1'))
    main_rows = [r for r in rows if r['stratum'] in MAIN]
    _, loo, _ = fixed_regrets(rows, lib)
    loo_main = np.array([loo[i] for i, r in enumerate(rows) if r['stratum'] in MAIN])
    joint = np.array([_reg(r, lib, pre + 'joint', 'R1') for r in main_rows])
    tests['T5 pooled: joint (abstain rule) vs best fixed controller, leave-one-out (pred. gain)'] = _test(loo_main - joint)
    sl_h, sl_d = lag_slopes(rows, lib, pre + 'rigid+delay', pre + 'rigid')
    if len(sl_h):
        t6 = _test(sl_h)
        t6['fitted_delay_slope_mean'] = float(np.nanmean(sl_d))
        tests['T6 lagsweep: slope of (rigid+delay - rigid) regret vs true lag (pred. > 0)'] = t6
    adj = score.holm({k: v['p'] for k, v in tests.items()})
    for k in tests:
        tests[k]['p_holm'] = adj[k]
        tests[k]['significant'] = bool(adj[k] < ALPHA)
    extra = {'transient: rigid+lag vs rigid (pred. >= 0)': _test(g(S('transient'), pre + 'rigid+lag', pre + 'rigid')) if not pre else None,
             'transient: joint vs rigid': _test(g(S('transient'), pre + 'joint', pre + 'rigid')),
             'transient: rigid+delay vs rigid, abstain rule': _test(g(S('transient'), pre + 'rigid+delay', pre + 'rigid', 'R1', 'R1')),
             'timing: rigid+delay vs rigid': _test(g(S('timing'), pre + 'rigid+delay', pre + 'rigid')),
             'timing: joint vs rigid': _test(g(S('timing'), pre + 'joint', pre + 'rigid')),
             'mixed: joint vs nominal, abstain rule': _test(g(S('mixed'), pre + 'joint', 'nominal', 'R1', 'R1')),
             'pooled: rigid vs nominal': _test(g(main_rows, pre + 'rigid', 'nominal')),
             'pooled: joint vs rigid': _test(g(main_rows, pre + 'joint', pre + 'rigid'))}
    return {'tests': tests, 'descriptive': {k: v for k, v in extra.items() if v is not None}}


def table(rows, lib, arms):
    per, loo, pick = fixed_regrets(rows, lib)
    C = per.shape[1]
    out = {}
    for st in MAIN + ('lagsweep', 'main-pooled'):
        ix = [i for i, r in enumerate(rows) if (r['stratum'] == st or (st == 'main-pooled' and r['stratum'] in MAIN))]
        if not ix:
            continue
        R = [rows[i] for i in ix]
        t = {'n': len(R)}
        for a in arms:
            x = np.array([_reg(r, lib, a) for r in R])
            xa = np.array([_reg(r, lib, a, 'R1') for r in R])
            sels = [r['libs'][lib]['scores']['arms'][a]['sel'] for r in R]
            t[a] = {'mean_regret': float(x.mean()), 'median_regret': float(np.median(x)),
                    'p_regret_gt2': float((x > 2.0).mean()), 'n_distinct_selected': len(set(sels)),
                    'mean_regret_abst': float(xa.mean()), 'p_regret_gt2_abst': float((xa > 2.0).mean()),
                    'abstain_rate': float(np.mean([r['libs'][lib]['scores']['arms'][a]['abstained'] for r in R]))}
        t['fixed:loo'] = {'mean_regret': float(loo[ix].mean()), 'median_regret': float(np.median(loo[ix])),
                          'p_regret_gt2': float((loo[ix] > 2.0).mean()), 'n_distinct_selected': len(set(pick[ix]))}
        best = int(np.argmin(per[ix].mean(0)))
        t['fixed:best-in-stratum (in-sample, needs stratum knowledge)'] = {
            'controller': best, 'mean_regret': float(per[ix, best].mean()), 'median_regret': float(np.median(per[ix, best])),
            'p_regret_gt2': float((per[ix, best] > 2.0).mean()), 'n_distinct_selected': 1}
        orc = [r['libs'][lib]['scores']['oracle'] for r in R]
        t['oracle_top_share'] = float(max(orc.count(c) for c in range(C)) / len(orc))
        t['oracle_distinct'] = len(set(orc))
        out[st] = t
    return out


def h2(rows, lib, arms):
    """Within-boat Spearman between an arm's open-loop trajectory error and its regret (across arms)."""
    rho = []
    for r in rows:
        if r['stratum'] not in MAIN:
            continue
        tr = [r['traj_rmse'][a] for a in arms]
        if any(v is None for v in tr):
            continue
        rho.append(score.spearman(np.array(tr), np.array([_reg(r, lib, a) for a in arms])))
    rho = np.array([x for x in rho if np.isfinite(x)])
    if not len(rho):
        return {'n': 0}
    idx = np.random.default_rng(1).integers(0, len(rho), (4000, len(rho)))
    return {'n': int(len(rho)), 'mean_rho': float(rho.mean()), 'ci90': [float(x) for x in np.quantile(rho[idx].mean(1), [0.05, 0.95])]}


def summarize(rows):
    arms = list(rows[0]['libs']['v1']['scores']['arms'])
    out = {'n_rows': len(rows), 'families': {}, 'tables': {}, 'h2': {}}
    for name, (lib, pre) in FAMILIES.items():
        out['families'][name] = family(rows, lib, pre)
    for lib in ('v1', 'v2'):
        out['tables'][lib] = table(rows, lib, arms)
        out['h2'][lib] = h2(rows, lib, [a for a in arms if not a.startswith('abc:')])
    prim = out['families'][PRIMARY]['tests']
    out['primary'] = {k: {'mean_gain': v['mean_gain'], 'p_holm': v['p_holm'], 'significant': v['significant']}
                      for k, v in prim.items()}
    return out


def print_summary(s):
    for fam, body in s['families'].items():
        print(f'== family {fam}' + ('  (PRIMARY)' if fam == PRIMARY else ''))
        for k, v in {**body['tests'], **body['descriptive']}.items():
            holm = f' holm {v["p_holm"]:.4f}{" *" if v.get("significant") else ""}' if 'p_holm' in v else ''
            print(f'  {k[:66]:66s} n={v["n"]:3d} gain {v["mean_gain"]:7.1f} ci90 [{v["ci90"][0]:.1f},{v["ci90"][1]:.1f}] '
                  f'med {v["median_gain"]:6.1f} p {v["p"]:.4f}{holm}')
    for lib, tab in s['tables'].items():
        print(f'== mean regret argmin (s) / mean regret abstain rule (s) / abstain rate, library {lib}')
        for st, t in tab.items():
            cells = ' '.join(f'{a[:15]}={v["mean_regret"]:.1f}/{v.get("mean_regret_abst", v["mean_regret"]):.1f}/{v.get("abstain_rate", 0):.2f}' for a, v in t.items()
                             if isinstance(v, dict) and 'mean_regret' in v and not a.startswith('fixed:best'))
            print(f'  {st:11s} n={t["n"]:3d} oracle top share {t["oracle_top_share"]:.2f} ({t["oracle_distinct"]} distinct) {cells}')
    print('H2 within-boat rho(traj error, regret):', s['h2'])
