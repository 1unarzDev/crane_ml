"""Q3 (twin optimism), Q4 (dose response), Q5 (single assumed level vs fitted), Q6 (value of information bounds).
Works on any run with fixed-delay arms named 'rigid@d<x>' (levels autodetected; 'rigid' = level 0)."""
import sys
sys.path.insert(0, __file__.rsplit('/', 1)[0])
from thr_common import *

def bci(x, nb=3000, seed=0, fn=np.mean):
    x = np.asarray(x, float)
    if len(x) < 2: return (np.nan, np.nan)
    idx = np.random.default_rng(seed).integers(0, len(x), (nb, len(x)))
    return tuple(np.percentile(fn(x[idx], axis=1), [5, 95]))

def levels_of(row):
    L = {0.0: 'rigid'}
    for a in row['fits']:
        if '@d' in a: L[float(a.split('@d')[1])] = a
    return dict(sorted(L.items()))

def get(r, lib, arm, key='regret'): return r['libs'][lib]['scores']['arms'][arm][key]

# ------------------------------------------------------------------ Q3
def q3(rows, run, out, thr=120.0):
    out(f'##### Q3 twin optimism ({run}); pred = mean over twin particles of J (non-finite -> 300); hid = mean over K hidden repeats; thr for predicted failure = mean J >= {thr:.0f}')
    for lib in ('v2', 'v3'):
        for st in sorted({r['stratum'] for r in rows}):
            R = [r for r in rows if r['stratum'] == st]
            arms = list(R[0]['libs'][lib]['J_twin_mean'])
            out(f'-- {lib} {st} (n={len(R)} boats)')
            out('   arm            | at ORACLE: pred-hid mean [90%] | share opt>10 / pess>10 | at PICK: pred-hid mean [90%] | opt>10/pess>10 | mean ctrl-rank corr | all ctrls: P(pred fail|hid ok) P(pred ok|hid fail) | bias on ctrls ok in both')
            for a in arms:
                go, gp, rc = [], [], []; fa = [0, 0]; mi = [0, 0]; bias = []
                for r in R:
                    Ld = r['libs'][lib]; Jt = np.array(Ld['J_twin_mean'][a]); Jh = np.array(Ld['J_h']).mean(1); dk = np.array(Ld['hidden_docked'])
                    o = Ld['scores']['oracle']; go.append(Jt[o] - Jh[o])
                    s = Ld['scores']['arms'].get(a)
                    if s is not None: gp.append(s['pred_J'] - s['hidden_J']); rc.append(s['rank_corr'])
                    else:
                        sel = int(np.argmin(Jt)); gp.append(Jt[sel] - Jh[sel])
                    hok, pok = dk >= 0.5, Jt < thr
                    fa[0] += np.sum(hok & ~pok); fa[1] += hok.sum(); mi[0] += np.sum(~hok & pok); mi[1] += (~hok).sum()
                    bias += list((Jt - Jh)[hok & pok])
                go, gp = np.array(go), np.array(gp)
                out(f'   {a:14s} | {go.mean():+7.1f} [{bci(go)[0]:+6.1f},{bci(go)[1]:+6.1f}] | {np.mean(go < -10):.2f} / {np.mean(go > 10):.2f} | '
                    f'{gp.mean():+7.1f} [{bci(gp)[0]:+6.1f},{bci(gp)[1]:+6.1f}] | {np.mean(gp < -10):.2f}/{np.mean(gp > 10):.2f} | {np.nanmean(rc) if rc else float("nan"):.2f} | '
                    f'{fa[0] / max(fa[1], 1):.2f} ({fa[1]})  {mi[0] / max(mi[1], 1):.2f} ({mi[1]}) | {np.mean(bias):+.1f}')

# ------------------------------------------------------------------ Q4
def first_stable_good(goods):
    """index of first level such that it and all higher levels are good; None if top level bad."""
    n = len(goods)
    if not goods[-1]: return None
    i = n - 1
    while i > 0 and goods[i - 1]: i -= 1
    return i

def spearman(a, b):
    ra, rb = np.argsort(np.argsort(a)).astype(float), np.argsort(np.argsort(b)).astype(float)
    return float(np.corrcoef(ra, rb)[0, 1])

def theil_sen(x, y):
    """OLS slope (name kept); Theil-Sen is degenerate here because d_min takes 4-5 distinct values."""
    xc = x - x.mean(); return float((xc * (y - y.mean())).sum() / max((xc ** 2).sum(), 1e-12))

def partial_spearman(x, y, z):
    rx, ry, rz = (np.argsort(np.argsort(v)).astype(float) for v in (x, y, z))
    A = np.column_stack([np.ones(len(z)), rz])
    ex = rx - A @ np.linalg.lstsq(A, rx, rcond=None)[0]; ey = ry - A @ np.linalg.lstsq(A, ry, rcond=None)[0]
    return float(np.corrcoef(ex, ey)[0, 1])

def q4(rows, run, out, stratum, covs, lib, good_def='reg10', cens_val=None):
    R = [r for r in rows if r['stratum'] == stratum]
    lv = levels_of(R[0]); keys = list(lv); top = keys[-1]
    cens_val = cens_val if cens_val is not None else top + 0.1
    dmin, X, nonmono, cens = [], {c: [] for c in covs}, 0, 0
    for r in R:
        g = []
        for l, a in lv.items():
            s = r['libs'][lib]['scores']['arms'][a]
            g.append(s['regret'] <= 10 if good_def == 'reg10' else r['libs'][lib]['hidden_docked'][s['sel']] >= 0.5)
        i = first_stable_good(g)
        nonmono += int(any((not g[k]) and g[k - 1] for k in range(1, len(g))) or (not g[0] and False))
        if i is None: v = cens_val; cens += 1
        elif i == 0: v = 0.0
        else: v = 0.5 * (keys[i] + keys[i - 1])          # midpoint of the (last bad, first good] bracket
        dmin.append(v)
        for c in covs: X[c].append(r['spec']['dead_time'] + r['spec']['odom_latency'] if c == 'L1' else r['spec'][c])
    dmin = np.array(dmin)
    out(f'  [{run} {lib} {stratum}, good={good_def}] (censored imputed at {cens_val:g}) levels {keys}; min-stable-good level distribution: ' +
        ', '.join(f'<= {k:g}: {np.mean(dmin <= k + 1e-9):.2f}' for k in keys) + f'; right-censored (top level {top:g} bad): {cens}/{len(R)}; non-monotone boats: {nonmono}')
    for c in covs:
        x = np.array(X[c]); n = len(x)
        rho = spearman(x, dmin); sl = theil_sen(x, dmin)
        rng = np.random.default_rng(1); bs = []
        for _ in range(1000):
            ix = rng.integers(0, n, n)
            if len(set(x[ix])) > 2:
                bs.append((spearman(x[ix], dmin[ix]), theil_sen(x[ix], dmin[ix])))
        bs = np.array(bs)
        kz = np.array([np.mean(r['spec']['k_fwd']) for r in R])
        pcs = partial_spearman(x, dmin, kz)
        out(f'      {c:9s} partial Spearman given thrust gain {pcs:+.2f};  Spearman {rho:+.2f} [{np.percentile(bs[:, 0], 5):+.2f},{np.percentile(bs[:, 0], 95):+.2f}]  OLS slope of d_min per unit {c}: {sl:+.2f} [{np.percentile(bs[:, 1], 5):+.2f},{np.percentile(bs[:, 1], 95):+.2f}]')
    if 'L1' in covs:
        x = np.array(X['L1']); q = np.quantile(x, [1 / 3, 2 / 3]); g = np.digitize(x, q)
        out('      by L tercile (cuts %.2f,%.2f): mean d_min (censored at %.2f) = ' % (q[0], q[1], cens_val) + ', '.join(f'{dmin[g == k].mean():.3f} (n={np.sum(g == k)}, L mean {x[g == k].mean():.2f})' for k in range(3)))

# ------------------------------------------------------------------ Q5
def q5(rows, run, out, strata=('timing', 'transient')):
    out(f'##### Q5 fixed assumed level vs fitted delay ({run}); capped regret (60 s), gain = regret(level) - regret(fitted) >0 means fitted better; w/l by >2 s')
    for lib in ('v2', 'v3'):
        for st in strata:
            R = [r for r in rows if r['stratum'] == st]
            if not R: continue
            lv = levels_of(R[0]); ref = 'rigid+delay'
            out(f'-- {lib} {st} (n={len(R)})  arm: mean capped regret / uncapped / dock | gain over fitted delay [90%] w/l | same pick as fitted')
            for l, a in list(lv.items()) + [(None, 'rigid+delay'), (None, 'rigid+lag'), (None, 'joint')]:
                if a not in R[0]['libs'][lib]['scores']['arms']: continue
                x = np.array([get(r, lib, a) for r in R]); xr = np.array([get(r, lib, ref) for r in R])
                d = np.minimum(x, CAP) - np.minimum(xr, CAP)
                dock = np.mean([r['libs'][lib]['hidden_docked'][get(r, lib, a, 'sel')] for r in R])
                same = np.mean([get(r, lib, a, 'sel') == get(r, lib, ref, 'sel') for r in R])
                out(f'   {a:12s} {np.minimum(x, CAP).mean():5.1f} / {x.mean():5.1f} / {dock:.2f} | {d.mean():+6.1f} [{bci(d)[0]:+6.1f},{bci(d)[1]:+6.1f}] {np.sum(d > 2)}/{np.sum(d < -2)} | {same:.2f}')
    return None

def q5_fail(rows, run, out, level_arm, strata=('timing', 'transient')):
    out(f'##### Q5b boats where {level_arm} is clearly worse than fitted delay (capped regret gap > 10 s) vs the rest ({run})')
    for lib in ('v2', 'v3'):
        for st in strata:
            R = [r for r in rows if r['stratum'] == st and level_arm in r['libs'][lib]['scores']['arms']]
            if not R: continue
            gap = np.array([min(get(r, lib, level_arm), CAP) - min(get(r, lib, 'rigid+delay'), CAP) for r in R])
            bad = gap > 10
            lvl = float(level_arm.split('@d')[1])
            rg = lambda f: np.array([f(r) for r in R])
            L1 = rg(lambda r: r['spec']['dead_time'] + r['spec']['odom_latency']); fd = rg(lambda r: r['fits']['rigid+delay']['map']['delay_s'])
            out(f'-- {lib} {st}: {bad.sum()} of {len(R)} boats; fitted-better by >10 s')
            for nm, v in (('true L=dead+odom', L1), ('fitted delay', fd), ('fitted - L', fd - L1), (f'L - {lvl:g}', L1 - lvl), ('tau_up', rg(lambda r: r['spec']['tau_up'])),
                          ('rev_hold', rg(lambda r: r['spec']['rev_hold'])), ('thrust kmean', rg(lambda r: np.mean(r['spec']['k_fwd']))), ('slew', rg(lambda r: r['spec']['slew']))):
                if bad.sum() >= 2 and (~bad).sum() >= 2:
                    out(f'     {nm:18s} fail-boats {v[bad].mean():+.3f} [{bci(v[bad])[0]:+.3f},{bci(v[bad])[1]:+.3f}]   others {v[~bad].mean():+.3f} [{bci(v[~bad])[0]:+.3f},{bci(v[~bad])[1]:+.3f}]')
            if st == 'timing':
                out(f'     L > {lvl:g}: {np.sum(bad & (L1 > lvl))} of the {bad.sum()} fail-boats; among boats with L > {lvl:g} ({np.sum(L1 > lvl)}): {np.mean(bad[L1 > lvl]) if np.any(L1 > lvl) else float("nan"):.2f} fail; with L <= {lvl:g}: {np.mean(bad[L1 <= lvl]):.2f}')

# ------------------------------------------------------------------ Q6
def q6(rows, run, out, strata=('timing', 'transient'), pool_strata=None):
    pool_strata = pool_strata or strata
    out(f'##### Q6 value of information bounds ({run}); capped regret; "best fixed" is chosen to minimise the mean over strata {pool_strata} (stratum-agnostic), per library')
    for lib in ('v2', 'v3'):
        R_all = [r for r in rows if r['stratum'] in pool_strata]
        lv = levels_of(R_all[0]); names = list(lv.values())
        pooled = {a: np.mean([min(get(r, lib, a), CAP) for r in R_all]) for a in names}
        bestfix = min(pooled, key=pooled.get)
        out(f'-- {lib}: pooled mean capped regret by fixed level: ' + ', '.join(f'{a} {v:.1f}' for a, v in pooled.items()) + f'  => best fixed = {bestfix}')
        for st in strata:
            R = [r for r in rows if r['stratum'] == st]
            if not R: continue
            reg = lambda a: np.array([min(get(r, lib, a), CAP) for r in R])
            L1 = np.array([r['spec']['dead_time'] + r['spec']['odom_latency'] for r in R])
            ks = list(lv); lvarr = np.array(ks)
            def grid_arm(margin, mode):
                out_ = []
                for r, l1 in zip(R, L1):
                    if mode == 'ge':
                        ok = lvarr[lvarr >= l1 + margin - 1e-9]; k = ok[0] if len(ok) else ks[-1]
                    else:
                        k = lvarr[np.argmin(np.abs(lvarr - (l1 + margin)))]
                    out_.append(min(get(r, lib, lv[k]), CAP))
                return np.array(out_)
            Rfix = np.array([[min(get(r, lib, a), CAP) for a in names] for r in R])
            hind_fix = Rfix.min(1)
            anyarms = [a for a in R[0]['libs'][lib]['scores']['arms'] if a != 'nominal']
            hind_any = np.array([min(min(get(r, lib, a), CAP) for a in anyarms) for r in R])
            fit = reg('rigid+delay'); bf = reg(bestfix)
            rows_ = [('best fixed level (%s)' % bestfix, bf), ('fitted delay (rigid+delay)', fit),
                     ('exact-L rule: smallest level >= L', grid_arm(0.0, 'ge')), ('exact-L rule: nearest level to L', grid_arm(0.0, 'nn')),
                     ('exact-L rule: nearest level to L+0.10', grid_arm(0.10, 'nn')),
                     ('hindsight best fixed level per boat (biased low)', hind_fix), ('hindsight best of all arms (biased low)', hind_any)]
            out(f'   {st} (n={len(R)}): mean capped regret [90% CI]; VoI vs best-fixed = regret(best fixed) - regret(arm), [90% CI]; share of boats with VoI > 5 s')
            for nm, x in rows_:
                v = bf - x
                out(f'      {nm:50s} {x.mean():5.1f} [{bci(x)[0]:5.1f},{bci(x)[1]:5.1f}]   VoI {v.mean():+6.1f} [{bci(v)[0]:+6.1f},{bci(v)[1]:+6.1f}]  >5s: {np.mean(v > 5):.2f}')

def main(out=print, run='m3-20261113', label='M3'):
    rows = load(run)
    q3(rows, label, out)
    for lib in ('v2', 'v3'):
        out(f'##### Q4 dose response ({label})')
        for gd in ('reg10', 'dock'):
            q4(rows, label, out, 'timing', ('L1', 'dead_time', 'odom_latency', 'tau_up', 'slew', 'gamma'), lib, gd)
            q4(rows, label, out, 'transient', ('tau_up', 'tau_down', 'rev_hold', 'mass'), lib, gd)
    q5(rows, label, out)
    q5_fail(rows, label, out, 'rigid@d0.35')
    q6(rows, label, out)
if __name__ == '__main__':
    main()
