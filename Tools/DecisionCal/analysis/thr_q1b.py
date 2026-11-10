"""Q1b: exogenous (assumed-delay) arms only: where is the threshold in d as a function of true L, and of covariates."""
import sys
sys.path.insert(0, __file__.rsplit('/', 1)[0])
from thr_common import *
from thr_q1 import logit_fit, boat_groups

def fixed_recs(rows, stratum, lib, with_zero=True):
    R = []
    for r in records([x for x in rows if x['stratum'] == stratum], (lib,)):
        if r['kind'] == 'fixed' or (with_zero and r['arm'] == 'rigid'): R.append(r)
    return R

def nll(X, y, w):
    z = X @ w; return float(np.sum(np.logaddexp(0, z) - y * z))

def line_model(R, how='reg10', lam=0.05):
    d, L = arr(R, 'd') / 0.1, arr(R, 'L1') / 0.1
    y = (arr(R, 'reg') > 10).astype(float) if how == 'reg10' else (arr(R, 'dock') < 0.5).astype(float)
    # good probability increases with d: model P(good)
    g = 1 - y
    X = np.column_stack([np.ones(len(d)), d, L])
    w = logit_fit(X, g, lam)
    return w, X, g

def summarize_line(R, label, out, how, nb=400):
    G = boat_groups(R); rng = np.random.default_rng(5)
    w, X, g = line_model(R, how)
    # thresholds
    def stats(w):
        b, c = w[1], w[2]
        return [-c / b, -(w[0] + c * 3.0) / b * 0.1]  # slope of required d on L ; required d (s) at L=0.30
    pt = stats(w)
    # null models
    wx = logit_fit(np.column_stack([np.ones(len(g)), X[:, 1] - X[:, 2]]), g, 0.05)
    nll_full = nll(X, g, w); nll_ex = nll(np.column_stack([np.ones(len(g)), X[:, 1] - X[:, 2]]), g, wx)
    wd = logit_fit(np.column_stack([np.ones(len(g)), X[:, 1]]), g, 0.05)
    nll_d = nll(np.column_stack([np.ones(len(g)), X[:, 1]]), g, wd)
    bs = []
    for _ in range(nb):
        pick = rng.integers(0, len(G), len(G)); idx = np.concatenate([G[p] for p in pick])
        try:
            wb = logit_fit(X[idx], g[idx], 0.05); bs.append(stats(wb))
        except Exception: pass
    bs = np.array(bs); ci = lambda j: np.percentile(bs[:, j], [5, 95])
    x50 = -(w[0] + w[2] * 3.0) / w[1] * 0.1
    out(f'  {label} good=NOT({how}) n={len(g)} picks, {len(G)} boats: required d at L=0.30: {x50:.3f} s [{ci(1)[0]:.3f},{ci(1)[1]:.3f}]; '
        f'slope dd*/dL = {pt[0]:.2f} [{ci(0)[0]:.2f},{ci(0)[1]:.2f}]; -2logL: d-only {2 * nll_d:.1f}, (d-L) only {2 * nll_ex:.1f}, d+L {2 * nll_full:.1f}')
    out(f'      logit(good) = {w[0]:+.2f} {w[1]:+.2f}*d/0.1s {w[2]:+.2f}*L/0.1s  (width of transition: 1/b = {0.1 / w[1]:.3f} s per logit unit)')


def best_split(t, good):
    """threshold c0 minimising misclassification of good = (t >= c0)."""
    o = np.argsort(t); ts, gs = t[o], good[o]; n = len(ts)
    # predicting bad for first i, good for rest: errors = goods in first i + bads in rest
    cg = np.concatenate([[0], np.cumsum(gs)]); cb = np.concatenate([[0], np.cumsum(1 - gs)])
    err = cg + (cb[-1] - cb)
    ok = np.concatenate([[True], ts[1:] != ts[:-1], [True]])          # split only between distinct values
    err = np.where(ok, err, 1e9); i = int(np.argmin(err))
    c0 = -1e9 if i == 0 else (1e9 if i == n else 0.5 * (ts[i - 1] + ts[i]))
    return c0, err[i] / n

def maxscore(R, how, c1s=np.arange(0, 3.01, 0.1), extra=None, idx=None):
    idx = np.arange(len(R)) if idx is None else idx
    d, L = arr(R, 'd')[idx], arr(R, 'L1')[idx]
    good = (arr(R, 'reg')[idx] <= 10).astype(float) if how == 'reg10' else (arr(R, 'dock')[idx] >= 0.5).astype(float)
    prof = [best_split(d - c1 * L, good) for c1 in c1s]
    return c1s, np.array([p[0] for p in prof]), np.array([p[1] for p in prof])

def report_maxscore(R, label, out, how, nb=300):
    c1s, c0s, err = maxscore(R, how)
    G = boat_groups(R); rng = np.random.default_rng(11); best = []
    for _ in range(nb):
        pick = rng.integers(0, len(G), len(G)); idx = np.concatenate([G[p] for p in pick])
        c, c0, e = maxscore(R, how, idx=idx); j = np.where(e <= e.min() + 1e-12)[0]
        best.append(c[j].mean())
    k = int(np.argmin(err)); i0, i1 = 0, int(np.argmin(np.abs(c1s - 1)))
    lo, hi = np.percentile(best, [5, 95])
    out(f'  {label} good=NOT({how}): misclass. error with rule d >= c0 + c1*L: c1=0 (absolute d) {err[i0]:.3f} (c0={c0s[i0]:.2f}); '
        f'c1=1 (excess) {err[i1]:.3f} (c0={c0s[i1]:+.3f} => threshold d = L {c0s[i1]:+.3f}); best c1={c1s[k]:.1f} err {err[k]:.3f} (c0={c0s[k]:+.2f}), '
        f'boot 90% CI of best c1 [{lo:.1f},{hi:.1f}]  (n={len(R)})')


def _fit_rule(d, L, kz, good, form):
    """returns predict(d,L,kz)->good bool, params. forms: abs, excess, linL, linLk, ratio_k"""
    if form == 'abs':
        c0, _ = best_split(d, good); return (lambda d, L, kz: d >= c0), (c0,)
    if form == 'excess':
        c0, _ = best_split(d - L, good); return (lambda d, L, kz: d - L >= c0), (c0,)
    if form == 'linL':
        best = min((best_split(d - c1 * L, good)[1], c1) for c1 in np.arange(0, 3.01, 0.25)); c1 = best[1]
        c0, _ = best_split(d - c1 * L, good); return (lambda d, L, kz: d - c1 * L >= c0), (c1, c0)
    if form == 'excess+k':
        best = min((best_split(d - L - c2 * kz, good)[1], c2) for c2 in np.arange(0, 0.41, 0.05)); c2 = best[1]
        c0, _ = best_split(d - L - c2 * kz, good); return (lambda d, L, kz: d - L - c2 * kz >= c0), (c2, c0)
    if form == 'linL+k':
        bb = min((best_split(d - c1 * L - c2 * kz, good)[1], c1, c2) for c1 in np.arange(0, 3.01, 0.25) for c2 in np.arange(0, 0.41, 0.05))
        _, c1, c2 = bb; c0, _ = best_split(d - c1 * L - c2 * kz, good)
        return (lambda d, L, kz: d - c1 * L - c2 * kz >= c0), (c1, c2, c0)

FORMS = ('abs', 'excess', 'linL', 'excess+k', 'linL+k')

def cv_models(R, how, label, out, reps=10, folds=5):
    d, L, k = arr(R, 'd'), arr(R, 'L1'), arr(R, 'kmean'); kz = (k - k.mean()) / k.std()
    good = (arr(R, 'reg') <= 10).astype(float) if how == 'reg10' else (arr(R, 'dock') >= 0.5).astype(float)
    ids = np.array([r['id'] for r in R]); ub = np.unique(ids); rng = np.random.default_rng(13)
    errs = {f: [] for f in FORMS}
    for _ in range(reps):
        perm = rng.permutation(ub); fold = {b: i % folds for i, b in enumerate(perm)}; fi = np.array([fold[i] for i in ids])
        for f in FORMS:
            wrong = 0
            for q in range(folds):
                tr, te = fi != q, fi == q
                pred, _ = _fit_rule(d[tr], L[tr], kz[tr], good[tr], f)
                wrong += np.sum(pred(d[te], L[te], kz[te]) != (good[te] > 0.5))
            errs[f].append(wrong / len(good))
    out(f'  {label} good=NOT({how}) boat-grouped {folds}-fold CV misclassification (mean of {reps} reps): ' +
        '  '.join(f'{f} {np.mean(errs[f]):.3f}' for f in FORMS) + f'   [SD(kmean)={k.std():.1f}, mean {k.mean():.1f}]')

def L_table(R, label, out, edges=(0, 0.22, 0.30, 0.38, 1.0), arms=None):
    arms = arms or sorted({r['arm'] for r in R}, key=lambda a: np.mean([r['d'] for r in R if r['arm'] == a]))
    out(f'  {label}: cell = mean capped regret / dock rate of pick (n boats); rows = true L = dead_time + odom_latency')
    out('    L bin        n  ' + ' '.join(f'{a:>16s}' for a in arms))
    for lo, hi in zip(edges[:-1], edges[1:]):
        cells = []; nb = 0
        for a in arms:
            q = [r for r in R if r['arm'] == a and lo <= r['L1'] < hi]; nb = len(q)
            if not q: cells.append('-'); continue
            cells.append(f'{np.minimum(arr(q, "reg"), CAP).mean():5.1f}/{arr(q, "dock").mean():.2f}')
        out(f'    [{lo:.2f},{hi:.2f}) {nb:3d}  ' + ' '.join(f'{c:>16s}' for c in cells))

def covariate_shifts(R, label, out, how='reg10', covs=('tau_up', 'tau_down', 'slew', 'gamma', 'pos_sd', 'vel_sd', 'kmean', 'gust', 'rev_hold'), nb=300):
    d, L = arr(R, 'd') / 0.1, arr(R, 'L1') / 0.1
    y = (arr(R, 'reg') > 10).astype(float) if how == 'reg10' else (arr(R, 'dock') < 0.5).astype(float)
    g = 1 - y; G = boat_groups(R); rng = np.random.default_rng(7)
    out(f'  {label}: shift in required assumed delay (s) per +1 SD of covariate, controlling for d and L (logit of good; one covariate at a time; ridge); 90% boat-boot CI')
    for c in covs:
        z = arr(R, c)
        if z.std() == 0: continue
        def est(idx):
            zs = (z[idx] - z[idx].mean()) / (z[idx].std() + 1e-12)
            X = np.column_stack([np.ones(len(idx)), d[idx], L[idx], zs])
            w = logit_fit(X, g[idx], 0.2); return -w[3] / w[1] * 0.1      # required d rises when -e/b > 0
        pt = est(np.arange(len(g))); bs = []
        for _ in range(nb):
            pick = rng.integers(0, len(G), len(G)); idx = np.concatenate([G[p] for p in pick])
            try: bs.append(est(idx))
            except Exception: pass
        lo, hi = np.percentile(bs, [5, 95])
        out(f'    {c:9s} {pt:+.3f} [{lo:+.3f},{hi:+.3f}]' + ('   *' if lo > 0 or hi < 0 else ''))

def main(out=print, run='m3-20261113'):
    rows = load(run)
    for lib in ('v2', 'v3'):
        out(f'===== Q1b ({run}) library {lib}: ASSUMED-delay arms (exogenous) only, timing boats')
        R = fixed_recs(rows, 'timing', lib)
        L_table(R, 'timing', out)
        for how in ('reg10', 'fail'):
            report_maxscore(R, 'timing', out, how); summarize_line(R, 'timing', out, how); cv_models(R, how, 'timing', out)
        covariate_shifts(R, 'timing', out)
        out(f'--- transient boats (true L ~0.05 s; excess is huge, lag is the confound)')
        R2 = fixed_recs(rows, 'transient', lib)
        L_table(R2, 'transient', out, edges=(0, 0.1, 1.0))
if __name__ == '__main__':
    main()
