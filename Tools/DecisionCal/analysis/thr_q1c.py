"""Q1c: failure curve by excess and thrust tercile; over-estimation cost (hinge fit of capped regret on excess)."""
import sys
sys.path.insert(0, __file__.rsplit('/', 1)[0])
from thr_common import *
from thr_q1 import boat_groups

def fail_curve(R, label, out, edges=(-1, -0.25, -0.15, -0.05, 0.05, 0.15, 0.25, 1)):
    ex = arr(R, 'd') - arr(R, 'L1'); k = arr(R, 'kmean'); q = np.quantile(k, [1 / 3, 2 / 3]); g = np.digitize(k, q)
    fail = arr(R, 'dock') < 0.5; regc = np.minimum(arr(R, 'reg'), CAP)
    out(f'  {label}: P(pick fails to dock) / mean capped regret (n picks) by excess = d - L and thrust-gain tercile (cuts {q[0]:.0f},{q[1]:.0f}; low / mid / high)')
    for lo, hi in zip(edges[:-1], edges[1:]):
        cells = []
        for t in range(3):
            m = (ex >= lo) & (ex < hi) & (g == t)
            cells.append(f'{fail[m].mean():.2f}/{regc[m].mean():5.1f} (n={m.sum():3d})' if m.sum() >= 3 else '          -          ')
        m = (ex >= lo) & (ex < hi)
        out(f'    excess [{lo:+.2f},{hi:+.2f}): ' + ' | '.join(cells) + f'  || all: {fail[m].mean():.2f}/{regc[m].mean():5.1f} (n={m.sum()})' if m.sum() else f'    excess [{lo:+.2f},{hi:+.2f}): none')

def hinge(ex, y, e0s=np.arange(-0.1, 0.46, 0.025)):
    best = None
    for e0 in e0s:
        h = np.maximum(0, ex - e0); X = np.column_stack([np.ones(len(h)), h])
        w, res, *_ = np.linalg.lstsq(X, y, rcond=None); sse = ((y - X @ w) ** 2).sum()
        if best is None or sse < best[0]: best = (sse, e0, w[0], w[1])
    return best

def hinge_report(R, label, out, nb=400, min_ex=-1.0):
    R = [r for r in R if r['d'] - r['L1'] >= min_ex]
    ex = arr(R, 'd') - arr(R, 'L1'); y = np.minimum(arr(R, 'reg'), CAP)
    sse, e0, b0, a = hinge(ex, y)
    G = boat_groups(R); rng = np.random.default_rng(4); bs = []
    for _ in range(nb):
        pick = rng.integers(0, len(G), len(G)); idx = np.concatenate([G[p] for p in pick]); b = hinge(ex[idx], y[idx]); bs.append((b[1], b[3]))
    bs = np.array(bs)
    out(f'  {label}: capped regret ~ {b0:.1f} + {a:.0f} s/s * max(0, excess - {e0:+.3f}); knee e0 90% CI [{np.percentile(bs[:, 0], 5):+.3f},{np.percentile(bs[:, 0], 95):+.3f}], '
        f'slope 90% CI [{np.percentile(bs[:, 1], 5):.0f},{np.percentile(bs[:, 1], 95):.0f}]  (n={len(R)} picks, {len(G)} boats; picks with excess >= {min_ex:+.2f})')

from thr_q1 import step_fit

def delta_terciles(R, label, out, covs=('kmean', 'slew', 'gamma', 'tau_up', 'tau_down', 'pos_sd', 'vel_sd', 'rev_hold'), nb=300):
    """Step location of P(pick fails to dock) vs excess within terciles of each covariate (assumed-delay arms, timing boats)."""
    ex = arr(R, 'd') - arr(R, 'L1'); fail = (arr(R, 'dock') < 0.5).astype(float)
    out(f'  {label}: step location delta (s of excess) of P(fail) within covariate terciles [boat-boot 90% CI]; P(fail) below/above the step in brackets. low | mid | high tercile')
    for c in covs:
        z = arr(R, c); q = np.quantile(z, [1 / 3, 2 / 3]); g = np.digitize(z, q); cells = []
        for t in range(3):
            idx = np.where(g == t)[0]
            if len(idx) < 20: cells.append('-'); continue
            b = step_fit(ex[idx], fail[idx], 8, 92)
            ids = np.array([R[i]['id'] for i in idx]); ub = np.unique(ids); rng = np.random.default_rng(9); bs = []
            for _ in range(nb):
                ch = rng.choice(ub, len(ub)); sel = np.concatenate([idx[ids == u] for u in ch]); bs.append(step_fit(ex[sel], fail[sel], 8, 92)[0])
            cells.append(f'{b[0]:+.2f} [{np.percentile(bs, 5):+.2f},{np.percentile(bs, 95):+.2f}] ({b[1]:.2f}->{b[2]:.2f})')
        out(f'    {c:9s} ' + ' | '.join(cells))
