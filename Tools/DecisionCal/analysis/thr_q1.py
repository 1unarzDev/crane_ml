"""Q1: threshold in (carried delay - true latency); harm from over-estimation; dependence on boat covariates."""
import sys
sys.path.insert(0, __file__.rsplit('/', 1)[0])
from thr_common import *

def step_fit(x, y, lo_q=5, hi_q=95):
    """Best single split of y on x (two-mean step), returns (delta, left_mean, right_mean, sse, sse0)."""
    o = np.argsort(x); xs, ys = x[o], y[o]
    n = len(xs); cs = np.cumsum(ys); cs2 = np.cumsum(ys ** 2)
    best = None
    for i in range(max(3, int(n * lo_q / 100)), min(n - 3, int(n * hi_q / 100))):
        if xs[i] == xs[i - 1]: continue
        l, r = cs[i - 1], cs[-1] - cs[i - 1]
        sse = (cs2[i - 1] - l * l / i) + ((cs2[-1] - cs2[i - 1]) - r * r / (n - i))
        if best is None or sse < best[3]:
            best = (0.5 * (xs[i] + xs[i - 1]), l / i, r / (n - i), sse)
    sse0 = ((ys - ys.mean()) ** 2).sum()
    return best + (sse0,)

def boat_groups(recs):
    g = {}
    for i, r in enumerate(recs): g.setdefault(r['id'], []).append(i)
    return list(g.values())

def cluster_boot(recs, stat, n=500, seed=0):
    G = boat_groups(recs); rng = np.random.default_rng(seed); out = []
    for _ in range(n):
        pick = rng.integers(0, len(G), len(G))
        idx = np.concatenate([G[p] for p in pick])
        try: out.append(stat([recs[i] for i in idx]))
        except Exception: pass
    return np.array(out)

def logit_fit(X, y, lam=0.2, it=60):
    """Ridge-logistic via Newton; first column is intercept (unpenalised)."""
    w = np.zeros(X.shape[1]); P = lam * np.eye(X.shape[1]); P[0, 0] = 0
    for _ in range(it):
        p = 1 / (1 + np.exp(-np.clip(X @ w, -30, 30)))
        g = X.T @ (p - y) + P @ w
        H = (X * (p * (1 - p))[:, None]).T @ X + P + 1e-9 * np.eye(len(w))
        step = np.linalg.solve(H, g); w -= step
        if np.abs(step).max() < 1e-8: break
    return w

def pooled(rows_by_run, stratum, lib, arms=None, exclude=('rigid+lag',)):
    out = []
    for run, rows in rows_by_run.items():
        for r in records([x for x in rows if x['stratum'] == stratum], (lib,)):
            if r['arm'] in exclude or (arms and r['arm'] not in arms): continue
            r['run'] = run; out.append(r)
    return out

def bad_of(recs, how):
    if how == 'reg10': return (arr(recs, 'reg') > 10).astype(float)
    if how == 'fail': return (arr(recs, 'dock') < 0.5).astype(float)

def report_threshold(R, label, out):
    """Step-model threshold on excess vs on absolute delay, for several outcome definitions."""
    for how in ('reg10', 'fail'):
        for xv in ('excess', 'abs_d', 'excess_lag'):
            def stat(rs, xv=xv, how=how):
                d, L = arr(rs, 'd'), arr(rs, 'L1')
                x = {'excess': d - L, 'abs_d': d, 'excess_lag': d - L - arr(rs, 'tau_up')}[xv]
                y = bad_of(rs, how); b = step_fit(x, y)
                return [b[0], b[1], b[2], 1 - b[3] / b[4]]
            pt = stat(R); bs = cluster_boot(R, stat, 300)
            lo, hi = np.percentile(bs[:, 0], [5, 95]); r2lo, r2hi = np.percentile(bs[:, 3], [5, 95])
            out(f'  {label} outcome={how:6s} x={xv:10s} step at {pt[0]:+.3f} s [{lo:+.3f},{hi:+.3f}]  P(bad) below {pt[1]:.2f} above {pt[2]:.2f}  R2 {pt[3]:.2f} [{r2lo:.2f},{r2hi:.2f}]')

def harm_curve(R, label, out, edges=(-1, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.45, 1)):
    ex = arr(R, 'd') - arr(R, 'L1'); reg = arr(R, 'reg'); regc = np.minimum(reg, CAP); dock = arr(R, 'dock')
    G = boat_groups(R)
    out(f'  {label}: excess bin | n | mean capped regret [90% boat-boot CI] | median | P(reg>10) | dock of pick')
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (ex >= lo) & (ex < hi)
        if m.sum() < 3: continue
        idx = np.where(m)[0]; ids = np.array([R[i]['id'] for i in idx])
        ub = np.unique(ids); rng = np.random.default_rng(1); bm = []
        for _ in range(500):
            ch = rng.choice(ub, len(ub)); sel = np.concatenate([idx[ids == c] for c in ch]); bm.append(regc[sel].mean())
        a, b = np.percentile(bm, [5, 95])
        out(f'    [{lo:+.2f},{hi:+.2f}) n={m.sum():3d} boats={len(ub):3d} {regc[m].mean():6.1f} [{a:5.1f},{b:5.1f}] med {np.median(reg[m]):5.1f}  P>10 {(reg[m] > 10).mean():.2f} dock {dock[m].mean():.2f}')

COVS = ('tau_up', 'tau_down', 'slew', 'gamma', 'pos_sd', 'vel_sd', 'kmean', 'L1', 'gust')

def cov_shift(R, label, out, covs=COVS, nb=300):
    """logit P(bad) = a + b*(excess/0.1) + sum c_k z_k + sum e_k z_k*(excess/0.1) ; threshold shift per +1 SD = -e_k/b (in s*0.1)."""
    ex = (arr(R, 'd') - arr(R, 'L1')) / 0.1
    y = bad_of(R, 'reg10')
    Z = np.stack([arr(R, c) for c in covs], 1)
    def fit(rs_idx):
        Zs = Z[rs_idx]; mu, sd = Zs.mean(0), Zs.std(0) + 1e-12; z = (Zs - mu) / sd
        e = ex[rs_idx]
        # one covariate at a time (interaction model), slope b0 at covariate mean
        res = {}
        for k, c in enumerate(covs):
            X = np.column_stack([np.ones(len(e)), e, z[:, k], z[:, k] * e])
            w = logit_fit(X, y[rs_idx], lam=0.3)
            # threshold (50%) location in excess units: x0(z) = -(a + c z)/(b + e z)
            res[c] = (-(w[0]) / w[1], -(w[2] + 0) / w[1] if False else (-(w[0] + w[2]) / (w[1] + w[3]) - (-(w[0] - w[2]) / (w[1] - w[3]))) / 2)
        return res
    G = boat_groups(R); rng = np.random.default_rng(3)
    pt = fit(np.arange(len(R))); bs = {c: [] for c in covs}
    for _ in range(nb):
        pick = rng.integers(0, len(G), len(G)); idx = np.concatenate([G[p] for p in pick])
        try:
            f = fit(idx)
            for c in covs: bs[c].append(f[c][1])
        except Exception: pass
    out(f'  {label}: 50% threshold location x0 (s) at the covariate mean, and shift of x0 for +1 SD of covariate (s), 90% boat-boot CI')
    for c in covs:
        v = np.array(bs[c]); v = v[np.isfinite(v)]
        lo, hi = np.percentile(v, [5, 95]) if len(v) else (np.nan, np.nan)
        out(f'    {c:9s} x0={pt[c][0] * 0.1:+.3f}  shift/SD {pt[c][1] * 0.1:+.3f} [{lo * 0.1:+.3f},{hi * 0.1:+.3f}]')

def main(out=print, runs=None):
    runs = runs or {'m2': 'm2-20261112', 'm3': 'm3-20261113'}
    rows = {k: load(v) for k, v in runs.items()}
    for lib in ('v2', 'v3'):
        out(f'===== Q1 library {lib}')
        for st in ('timing', 'transient'):
            R = pooled(rows, st, lib)
            out(f'-- {st}: {len({r["id"] for r in R})} boats, {len(R)} (boat, arm) picks; arms with a carried delay (rigid=0, fitted delay/joint, assumed levels)')
            report_threshold(R, st, out)
            harm_curve(R, st, out)
            if st == 'timing': cov_shift(R, st, out)
if __name__ == '__main__':
    main()
