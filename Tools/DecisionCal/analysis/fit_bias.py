"""Fit-bias / error-compensation analysis (numpy only, stored results; features from fb_features.py).

    cd /home/lunarz/worktrees/decisioncal && nice -n 19 python3 -I -W ignore \
        Tools/DecisionCal/analysis/fit_bias.py > Tools/DecisionCal/analysis/output_fit_bias.txt
"""
import pickle
import os
import sys
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import numpy as np

D = pickle.load(open(os.path.expanduser('~/.cache/decisioncal/fitbias_features.pkl'), 'rb'))
BOATS, TRUTH = D['boats'], D['truth']
for i, b in enumerate(BOATS):
    b['i'] = i
    b['L'] = b['dead'] + b['odom']
RNG = np.random.default_rng(1234)
NB = 2000
FIT_ARMS = {'M4': ['rigid', 'rigid+delay', 'joint', 'rigid@d0.35', 'rigid@d0.47', 'rigid@d0.62'],
            'M2': ['rigid', 'rigid+delay', 'rigid+lag', 'joint']}
STRATA = {'M4': ['steady', 'transient', 'timing'], 'M2': ['transient', 'timing']}
BASE = {'rigid', 'rigid+delay', 'rigid+lag', 'joint', 'rigid@d0.35', 'rigid@d0.47', 'rigid@d0.62'}


def P(*a):
    print(*a, flush=True)


def ci(x, fn=np.median, q=(5, 95)):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if len(x) < 3:
        return (np.nan, np.nan)
    idx = RNG.integers(0, len(x), (NB, len(x)))
    v = np.median(x[idx], 1) if fn is np.median else np.array([fn(x[i]) for i in idx])
    return tuple(np.percentile(v, q))


def fmt(x, fn=np.median, d=2):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if len(x) == 0:
        return 'n/a'
    lo, hi = ci(x, fn)
    return f'{fn(x):.{d}f} [{lo:.{d}f},{hi:.{d}f}]'


def sel(ds=None, stratum=None, arm=None, boats=None):
    out = []
    for b in (boats or BOATS):
        if ds and b['ds'] != ds and ds != 'ALL':
            continue
        if stratum and b['stratum'] != stratum:
            continue
        if arm and arm not in b['fits']:
            continue
        out.append(b)
    return out


# ---------------- per-(boat, arm) scalar metrics ----------------
def metric(b, arm, name):
    f, h, t = b['fits'][arm], b['hid'], TRUTH[b['i']]
    par = f['par']
    if name in ('m11', 'm22', 'm66', 'd1_u', 'd2_u', 'd1_v', 'd2_v', 'd1_r', 'd2_r'):
        return par[name] / t[name]
    if name == 'rev':
        return par['rev_ratio'] / t['rev_ratio']
    if name == 'db_diff':
        return par['deadband_q'] - t['deadband_q']
    if name in ('surge50', 'surge100', 'yaw50', 'yaw100', 'tau_surge', 'tau_yaw'):
        return f[name] / h[name]
    if name == 'dly':          # delay carried minus true dead time + latency
        return f['delay'] - b['L']
    if name in ('lag_yaw', 'lag_surge'):   # equivalent lag the controller sees: command -> 63% of step + latency
        ax = name.split('_')[1]
        return (f['t63_' + ax] + f['lat']) - (h['t63_' + ax] + h['lat'])
    if name in ('body_yaw', 'body_surge'):  # lag minus delay: the part not attributable to delay
        ax = name.split('_')[1]
        return metric(b, arm, 'lag_' + ax) - metric(b, arm, 'dly')
    raise KeyError(name)


def col(boats, arm, name):
    return np.array([metric(b, arm, name) for b in boats])


CORE = ['m11', 'm22', 'm66', 'd1_u', 'd2_u', 'd1_r', 'd2_r', 'surge50', 'surge100', 'yaw50', 'yaw100', 'tau_surge',
        'tau_yaw', 'rev']
ADD = ['dly', 'lag_yaw', 'lag_surge', 'body_yaw', 'body_surge', 'db_diff']


# =============================================================================================
P('=' * 100)
P('Q1  fitted / true ratios (median [90% boat-bootstrap CI]); ratio < 1 = fitted twin has smaller inertia / damping / time constant')
P('    truth = study_oracle_twin.project (in-family projection); oracle-proj row = projected truth run through the same step sims')
P('    (it shows the projection floor: gamma != 2, per-thruster asymmetry, slew are dropped)')
for ds in ('M4', 'M2'):
    for st in STRATA[ds]:
        bs = sel(ds, st)
        P(f'\n--- {ds} {st} (n={len(bs)}) ---')
        for grp, names in (('inertia/damping', CORE[:7]), ('steady speeds / time constants / rev', CORE[7:])):
            P(f'  [{grp}]')
            for arm in FIT_ARMS[ds] + ['oracle-proj']:
                P(f'   {arm:12s} ' + ' | '.join(f'{n}={fmt(col(bs, arm, n))}' for n in names))
        P('  [equivalent-lag quantities, seconds; dly = carried delay - (dead+latency); lag = twin minus hidden command->63% (+ latency)]')
        for arm in FIT_ARMS[ds] + ['oracle-proj']:
            P(f'   {arm:12s} ' + ' | '.join(f'{n}={fmt(col(bs, arm, n), d=3)}' for n in ADD[:5]))
        P('  [share of boats with ratio < 1 : m11 m66 tau_surge tau_yaw lag_yaw<0 ]')
        for arm in FIT_ARMS[ds]:
            P(f'   {arm:12s} ' + ' '.join(f'{(col(bs, arm, n) < (0 if n.startswith("lag") else 1)).mean():.2f}' for n in ('m11', 'm66', 'tau_surge', 'tau_yaw', 'lag_yaw')))
        P('  [IQR of ratio m11, m66, tau_yaw]')
        for arm in FIT_ARMS[ds]:
            P(f'   {arm:12s} ' + ' '.join(f'{n}:[{np.percentile(col(bs, arm, n), 25):.2f},{np.percentile(col(bs, arm, n), 75):.2f}]' for n in ('m11', 'm66', 'tau_yaw')))

P('\n--- sway block, all datasets pooled, ratios (median) ---')
for arm in ('rigid', 'rigid+delay', 'joint'):
    bs = sel('ALL', None, arm)
    P(f'  {arm:12s} ' + ' | '.join(f'{n}={fmt(col(bs, arm, n))}' for n in ('m22', 'd1_v', 'd2_v')))

P('\n--- clipping: share of boats whose raw MAP mean is below the bound (m) / negative (d), by block (pooled M2+M4) ---')
import twin
for arm in ('rigid', 'rigid+delay', 'rigid+lag', 'joint', 'rigid@d0.47'):
    bs = sel('ALL', None, arm)
    for st in ('steady', 'transient', 'timing'):
        bb = [b for b in bs if b['stratum'] == st]
        if not bb:
            continue
        s = []
        for blk, nm in (('surge', 'm11'), ('sway', 'm22'), ('yaw', 'm66')):
            m_clip = np.mean([b['fits'][arm]['raw'][blk][0] < twin.BOUNDS[nm] for b in bb])
            d_clip = np.mean([min(b['fits'][arm]['raw'][blk][1:]) < 0 for b in bb])
            fc = np.mean([b['fits'][arm]['frac_clipped'][blk] for b in bb])
            s.append(f'{blk}: m<bound {m_clip:.2f}, d<0 {d_clip:.2f}, particle-clip {fc:.2f}')
        P(f'  {arm:12s} {st:9s} n={len(bb):3d} ' + ' ; '.join(s))

# rev ratio / deadband discrete fits
P('\n--- rev_ratio (fit/true) and deadband (fit - true) per stratum, rigid+delay and joint ---')
for st in ('steady', 'transient', 'timing'):
    for arm in ('rigid', 'rigid+delay', 'joint'):
        bs = [b for b in sel('ALL', None, arm) if b['stratum'] == st]
        P(f'  {st:9s} {arm:12s} rev ratio {fmt(col(bs, arm, "rev"))}   deadband diff {fmt(col(bs, arm, "db_diff"), d=3)}   '
          f'(true rev {np.median([b["rev_true"] for b in bs]):.2f}, true db {np.median([b["db_true"] for b in bs]):.3f})')

# ---- dependence on thrust scale, gamma, noise: within-stratum Spearman, M2+M4 pooled
P('\n--- dependence: within-stratum Spearman rho [90% CI] of metric with covariate (M2+M4 pooled; * = CI excludes 0) ---')
COV = ['s_eff', 'gamma', 'pos_sd', 'vel_sd', 'gyro_sd', 'yaw_sd', 'k_asym']


def rank_within(x, strata):
    r = np.zeros(len(x))
    for st in set(strata):
        m = np.array([s == st for s in strata])
        r[m] = np.argsort(np.argsort(x[m])) / max(m.sum() - 1, 1)
    return r


def srho(x, y, strata):
    return np.corrcoef(rank_within(x, strata), rank_within(y, strata))[0, 1]


def boot_rho(x, y, strata, nb=400):
    n = len(x)
    strata = np.array(strata)
    v = []
    for _ in range(nb):
        i = RNG.integers(0, n, n)
        v.append(srho(x[i], y[i], list(strata[i])))
    return np.percentile(v, [5, 95])


for arm in ('rigid', 'rigid+delay', 'joint'):
    bs = sel('ALL', None, arm)
    strata = [b['stratum'] for b in bs]
    P(f'  arm {arm} (n={len(bs)})')
    P('    metric      ' + ' '.join(f'{c:>20s}' for c in COV))
    for name in ('m11', 'm66', 'd1_u', 'tau_surge', 'tau_yaw', 'dly', 'lag_yaw'):
        y = col(bs, arm, name)
        cells = []
        for c in COV:
            x = np.array([b[c] for b in bs], float)
            r = srho(x, y, strata)
            lo, hi = boot_rho(x, y, strata)
            cells.append(f'{r:+.2f}[{lo:+.2f},{hi:+.2f}]' + ('*' if lo > 0 or hi < 0 else ' '))
        P(f'    {name:11s} ' + ' '.join(f'{c:>20s}' for c in cells))

# =============================================================================================
P('\n' + '=' * 100)
P('Q2  paired change of the fitted rigid body when channels are added (same boat). Entries: median ratio of ratios (A/B) [90% CI];')
P('    1.00 = no change. A rise of m11/m66/tau toward 1 would mean the extra channel de-biases the rigid body.')


def paired(ds, st, A, Bm, names):
    bs = [b for b in sel(ds, st) if A in b['fits'] and Bm in b['fits']]
    cells = []
    for n in names:
        a = np.maximum(col(bs, A, n), 1e-3)
        c = np.maximum(col(bs, Bm, n), 1e-3)
        cells.append(f'{n}={fmt(a / c)}')
    return len(bs), ' | '.join(cells)


PAIRS = {'M4': [('rigid+delay', 'rigid'), ('joint', 'rigid+delay'), ('joint', 'rigid'), ('rigid@d0.35', 'rigid'),
                ('rigid@d0.47', 'rigid@d0.35'), ('rigid@d0.62', 'rigid@d0.47'), ('rigid@d0.62', 'rigid')],
         'M2': [('rigid+delay', 'rigid'), ('rigid+lag', 'rigid'), ('joint', 'rigid+delay'), ('joint', 'rigid+lag'),
                ('joint', 'rigid')]}
for ds in ('M4', 'M2'):
    for st in STRATA[ds]:
        P(f'\n--- {ds} {st} ---')
        for A, Bm in PAIRS[ds]:
            n, s = paired(ds, st, A, Bm, ['m11', 'm66', 'd1_r', 'd2_r', 'tau_surge', 'tau_yaw', 'surge50'])
            P(f'   {A:12s}/{Bm:12s} n={n:3d} {s}')
P('\n--- paired change in the carried/equivalent lag (seconds, A - B) ---')
for ds in ('M4', 'M2'):
    for st in STRATA[ds]:
        P(f'  {ds} {st}')
        for A, Bm in PAIRS[ds][:4]:
            bs = [b for b in sel(ds, st) if A in b['fits'] and Bm in b['fits']]
            P(f'   {A:12s}-{Bm:12s} ' + ' | '.join(f'{n}={fmt(col(bs, A, n, ) - col(bs, Bm, n), d=3)}' for n in ('dly', 'lag_yaw', 'lag_surge', 'body_yaw')))
P('\n--- absolute log-error of the rigid body |ln ratio| (median over boats): does adding delay reduce it? ---')
for ds in ('M4', 'M2'):
    for st in STRATA[ds]:
        bs = sel(ds, st)
        P(f'  {ds} {st:9s} ' + ' ; '.join(f'{a}: m11 {np.median(np.abs(np.log(np.maximum(col(bs, a, "m11"), 1e-3)))):.2f} '
                                           f'm66 {np.median(np.abs(np.log(np.maximum(col(bs, a, "m66"), 1e-3)))):.2f} '
                                           f'tau_yaw {np.median(np.abs(np.log(np.maximum(col(bs, a, "tau_yaw"), 1e-3)))):.2f}' for a in FIT_ARMS[ds]))
P('\n--- does the fitted-delay arm choose the rigid body that the delay needs? corr(fitted delay, ln m11 ratio) and OLS slope, rigid+delay & joint (within stratum, M2+M4) ---')
for arm in ('rigid+delay', 'joint'):
    for st in ('steady', 'transient', 'timing'):
        bs = [b for b in sel('ALL', None, arm) if b['stratum'] == st]
        if len(bs) < 10:
            continue
        x = col(bs, arm, 'dly')
        for nm in ('m11', 'm66', 'tau_yaw'):
            y = np.log(np.maximum(col(bs, arm, nm), 1e-2))
            r = np.corrcoef(x, y)[0, 1]
            sl = np.polyfit(x, y, 1)[0]
            bl = [np.polyfit(x[i], y[i], 1)[0] for i in RNG.integers(0, len(x), (400, len(x)))]
            P(f'  {arm:12s} {st:9s} {nm:8s} r(dly, ln ratio) {r:+.2f}  slope {sl:+.2f} ln/s [{np.percentile(bl, 5):+.2f},{np.percentile(bl, 95):+.2f}]')

# =============================================================================================
P('\n' + '=' * 100)
P('Q3  equivalent sluggishness. lag = (twin command->63% step time + latency) - (hidden same); dly = carried delay - (dead+latency); body = lag - dly')
for ds in ('M4', 'M2'):
    for st in STRATA[ds]:
        bs = sel(ds, st)
        P(f'\n--- {ds} {st} (n={len(bs)}), seconds, median [90% CI]; |.| = median absolute ---')
        for arm in FIT_ARMS[ds]:
            c = {n: col(bs, arm, n) for n in ('dly', 'lag_yaw', 'lag_surge', 'body_yaw', 'body_surge')}
            P(f'   {arm:12s} dly {fmt(c["dly"], d=3)} | lag_yaw {fmt(c["lag_yaw"], d=3)} | lag_surge {fmt(c["lag_surge"], d=3)} | '
              f'body_yaw {fmt(c["body_yaw"], d=3)} | body_surge {fmt(c["body_surge"], d=3)} | '
              f'|dly| {np.median(np.abs(c["dly"])):.3f} |lag_yaw| {np.median(np.abs(c["lag_yaw"])):.3f} |lag_surge| {np.median(np.abs(c["lag_surge"])):.3f}')
        for arm in ('rigid+delay', 'joint'):
            if arm in FIT_ARMS[ds]:
                dl, bd = col(bs, arm, 'dly'), col(bs, arm, 'body_yaw')
                r = np.corrcoef(dl, bd)[0, 1]
                bl = [np.corrcoef(dl[i], bd[i])[0, 1] for i in RNG.integers(0, len(dl), (400, len(dl)))]
                sl = np.polyfit(dl, col(bs, arm, 'lag_yaw'), 1)[0]
                sb = [np.polyfit(dl[i], col(bs, arm, 'lag_yaw')[i], 1)[0] for i in RNG.integers(0, len(dl), (400, len(dl)))]
                P(f'   compensation {arm:12s}: corr(dly, body_yaw) {r:+.2f} [{np.percentile(bl, 5):+.2f},{np.percentile(bl, 95):+.2f}]  '
                  f'slope d(lag_yaw)/d(dly) {sl:+.2f} [{np.percentile(sb, 5):+.2f},{np.percentile(sb, 95):+.2f}]')

# ---------------- logistic machinery ----------------
def relu(x):
    return np.maximum(x, 0.0)


def logit_fit(X, y, ridge=1e-3, it=40):
    mu, sd = X.mean(0), X.std(0) + 1e-9
    Z = np.c_[np.ones(len(X)), (X - mu) / sd]
    w = np.zeros(Z.shape[1])
    R = ridge * np.eye(Z.shape[1])
    R[0, 0] = 0
    for _ in range(it):
        p = 1 / (1 + np.exp(-np.clip(Z @ w, -30, 30)))
        g = Z.T @ (p - y) + R @ w
        H = (Z * (p * (1 - p))[:, None]).T @ Z + R + 1e-8 * np.eye(len(w))
        step = np.linalg.solve(H, g)
        w -= step
        if np.abs(step).max() < 1e-8:
            break
    return w, mu, sd


def logit_pred(m, X):
    w, mu, sd = m
    Z = np.c_[np.ones(len(X)), (X - mu) / sd]
    return 1 / (1 + np.exp(-np.clip(Z @ w, -30, 30)))


def raw_coef(m):
    w, mu, sd = m
    b = w[1:] / sd
    return w[0] - (b * mu).sum(), b


def cv_probs(X, y, groups, folds=10, reps=5, seed=7):
    ug = np.unique(groups)
    out = np.zeros((reps, len(y)))
    for r in range(reps):
        rng = np.random.default_rng(seed + r)
        perm = rng.permutation(ug)
        fold = {g: k % folds for k, g in enumerate(perm)}
        f = np.array([fold[g] for g in groups])
        for k in range(folds):
            te = f == k
            if te.sum() == 0 or len(np.unique(y[~te])) < 2:
                out[r, te] = y[~te].mean()
                continue
            m = logit_fit(X[~te], y[~te])
            out[r, te] = logit_pred(m, X[te])
    return out


def auc(p, y):
    if len(np.unique(y)) < 2:
        return np.nan
    r = np.argsort(np.argsort(p)) + 1.0
    n1, n0 = y.sum(), len(y) - y.sum()
    return (r[y == 1].sum() - n1 * (n1 + 1) / 2) / (n1 * n0)


def per_boat_mean(vals, groups):
    ug, inv = np.unique(groups, return_inverse=True)
    s = np.bincount(inv, weights=vals)
    return s / np.bincount(inv), ug


# ---------------- observation table ----------------
OBS = []
for b in BOATS:
    for lib in ('v2', 'v3'):
        Ld = b['libs'][lib]
        for arm, f in b['fits'].items():
            if arm not in BASE:
                continue
            reg = Ld['regret'][arm]
            dock = Ld['docked'][Ld['sel'][arm]]
            OBS.append(dict(gid=b['ds'] + ':' + b['id'], ds=b['ds'], st=b['stratum'], arm=arm, lib=lib, i=b['i'],
                            reg=reg, dock=dock, fail=float(dock < 0.5), bad=float((dock < 0.5) or (reg > 10)),
                            slow=float((dock >= 0.5) and (reg > 10)),
                            d=f['delay'], L=b['L'], dly=metric(b, arm, 'dly'), lag_yaw=metric(b, arm, 'lag_yaw'),
                            lag_surge=metric(b, arm, 'lag_surge'), body_yaw=metric(b, arm, 'body_yaw'),
                            body_surge=metric(b, arm, 'body_surge'), s=np.log(b['s_eff']), assumed='@d' in arm))
P(f'\nobservations: {len(OBS)} (boat x arm x library), bad rate {np.mean([o["bad"] for o in OBS]):.2f}, '
  f'fail rate {np.mean([o["fail"] for o in OBS]):.2f}, slow rate {np.mean([o["slow"] for o in OBS]):.2f}')


def feats(obs, spec):
    cols = []
    for k in spec:
        v = np.array([o[k.lstrip('+-^')] if k[0] not in '+-^' else o[k[1:]] for o in obs], float)
        if k[0] == '+':
            cols.append(relu(v))
        elif k[0] == '-':
            cols.append(relu(-v))
        elif k[0] == '^':
            cols.append(v ** 2)
        else:
            cols.append(v)
    return np.stack(cols, 1)


MODELS = {
    'prior only': [],
    'carried delay d (abs)': ['d'],
    'd and true L (linear)': ['d', 'L'],
    'excess dly hinge': ['+dly', '-dly'],
    'lag_yaw hinge': ['+lag_yaw', '-lag_yaw'],
    'lag_surge hinge': ['+lag_surge', '-lag_surge'],
    'lag_yaw+lag_surge hinge': ['+lag_yaw', '-lag_yaw', '+lag_surge', '-lag_surge'],
    'dly hinge + lag_yaw hinge': ['+dly', '-dly', '+lag_yaw', '-lag_yaw'],
    'dly hinge + body_yaw': ['+dly', '-dly', 'body_yaw'],
    'dly hinge + log s': ['+dly', '-dly', 's'],
    'lag_yaw hinge + log s': ['+lag_yaw', '-lag_yaw', 's'],
    'dly hinge + lag_yaw hinge + log s': ['+dly', '-dly', '+lag_yaw', '-lag_yaw', 's'],
}


def run_cv(obs, outcome, label):
    y = np.array([o[outcome] for o in obs], float)
    groups = np.array([o['gid'] for o in obs])
    if y.sum() < 5 or (len(y) - y.sum()) < 5:
        P(f'  [{label}] skipped (too few of one class)')
        return
    res = {}
    for name, spec in MODELS.items():
        if not spec:
            p = np.zeros((5, len(y))) + y.mean()
            pcv = p
        else:
            pcv = cv_probs(feats(obs, spec), y, groups)
        err = ((pcv > 0.5) != (y > 0.5)).astype(float).mean(0)
        ll = -(y * np.log(np.clip(pcv, 1e-6, 1)) + (1 - y) * np.log(np.clip(1 - pcv, 1e-6, 1))).mean(0)
        res[name] = (per_boat_mean(err, groups), per_boat_mean(ll, groups), np.mean([auc(pcv[r], y) for r in range(5)]),
                     (pcv.mean(0) > 0.5))
    base = res['excess dly hinge'][0][0]
    P(f'  [{label}] n={len(y)}, boats={len(np.unique(groups))}, positive rate {y.mean():.2f}; majority error {min(y.mean(), 1 - y.mean()):.3f}')
    P(f'    {"model":38s} {"CV err":>7s} {"CV logloss":>10s} {"AUC":>5s}   err minus err(excess dly hinge) [90% boat-bootstrap CI]')
    nb = len(base)
    idx = RNG.integers(0, nb, (1000, nb))
    for name, ((e, _), (l, _), a, _) in res.items():
        d = e - base
        v = d[idx].mean(1)
        P(f'    {name:38s} {e.mean():7.3f} {l.mean():10.3f} {a:5.2f}   {d.mean():+.3f} [{np.percentile(v, 5):+.3f},{np.percentile(v, 95):+.3f}]')


P('\n--- Q3 boat-grouped 10-fold CV (5 repeats), logistic models; err = misclassification at 0.5 ---')
SUBS = {'all arms pooled (M2+M4, both libs)': lambda o: True,
        'assumed-delay arms only (M4 rigid@d*)': lambda o: o['assumed'],
        'fitted arms only (rigid, rigid+delay, rigid+lag, joint)': lambda o: not o['assumed'],
        'timing boats, all arms': lambda o: o['st'] == 'timing',
        'transient boats, all arms': lambda o: o['st'] == 'transient',
        'steady boats, all arms (M4)': lambda o: o['st'] == 'steady'}
for sname, fn in SUBS.items():
    obs = [o for o in OBS if fn(o)]
    P(f'\n== subset: {sname}')
    for oc in ('bad', 'fail'):
        run_cv(obs, oc, oc)

P('\n--- Q3 fitted coefficients (raw units), all arms pooled, outcome bad: P = sigmoid(a + sum b x) ---')
obs = OBS
y = np.array([o['bad'] for o in obs])
for name in ('excess dly hinge', 'lag_yaw hinge', 'lag_yaw+lag_surge hinge', 'dly hinge + lag_yaw hinge'):
    spec = MODELS[name]
    m = logit_fit(feats(obs, spec), y)
    a, b = raw_coef(m)
    P(f'  {name:30s} a={a:+.2f} ' + ' '.join(f'{k}:{v:+.1f}' for k, v in zip(spec, b)))

# bad rate by bins of Δlag_yaw and Δdly : all arms
P('\n--- Q3 bad / fail rate by bin (all arms pooled): rows = binned dly, columns = binned lag_yaw (seconds) ---')
bins = [-9, -0.3, -0.1, 0.0, 0.1, 0.2, 9]
dl = np.array([o['dly'] for o in OBS])
lg = np.array([o['lag_yaw'] for o in OBS])
bd = np.array([o['bad'] for o in OBS])
fl = np.array([o['fail'] for o in OBS])
hdr = ' '.join(f'({bins[j]:+.1f},{bins[j + 1]:+.1f}]'.rjust(14) for j in range(len(bins) - 1))
P('   dly \\ lag_yaw  ' + hdr + '      (cell: bad rate / n)')
for i in range(len(bins) - 1):
    cells = []
    for j in range(len(bins) - 1):
        m = (dl > bins[i]) & (dl <= bins[i + 1]) & (lg > bins[j]) & (lg <= bins[j + 1])
        cells.append(f'{bd[m].mean():.2f}/{m.sum():d}'.rjust(14) if m.sum() >= 5 else '.'.rjust(14))
    P(f'   ({bins[i]:+.1f},{bins[i + 1]:+.1f}]  ' + ' '.join(cells))
P('   same table, fail rate')
for i in range(len(bins) - 1):
    cells = []
    for j in range(len(bins) - 1):
        m = (dl > bins[i]) & (dl <= bins[i + 1]) & (lg > bins[j]) & (lg <= bins[j + 1])
        cells.append(f'{fl[m].mean():.2f}/{m.sum():d}'.rjust(14) if m.sum() >= 5 else '.'.rjust(14))
    P(f'   ({bins[i]:+.1f},{bins[i + 1]:+.1f}]  ' + ' '.join(cells))

# =============================================================================================
P('\n' + '=' * 100)
P('Q4  timing boats in M4, assumed-delay arms (0.35, 0.47, 0.62): minimum workable assumed delay = a + b L + c misfit')
T = [o for o in OBS if o['assumed'] and o['ds'] == 'M4' and o['st'] == 'timing']
for o in T:
    b = BOATS[o['i']]
    o['mis_yaw47'] = -metric(b, 'rigid@d0.47', 'body_yaw')       # body_true - body_twin at the 0.47 s twin (>0: twin too crisp)
    o['mis_surge47'] = -metric(b, 'rigid@d0.47', 'body_surge')
    o['mis_yaw'] = -o['body_yaw']                                  # at the twin's own assumed delay
    o['mis_surge'] = -o['body_surge']
    o['mis_yaw0'] = -metric(b, 'rigid', 'body_yaw')                # rigid (delay 0) fit
    o['mis_surge0'] = -metric(b, 'rigid', 'body_surge')
    o['mis_m'] = np.log(max(metric(b, 'rigid@d0.47', 'm66'), 1e-2))
    o['mis_m11'] = np.log(max(metric(b, 'rigid@d0.47', 'm11'), 1e-2))
    o['tau_s'] = 0.5 * (b['tau_up'] + b['tau_down'])
P(f'  n obs {len(T)} (60 boats x 3 levels x 2 libs); fail rate by level: ' +
  ', '.join(f'{d}: {np.mean([o["fail"] for o in T if abs(o["d"] - d) < 1e-6]):.2f}' for d in (0.35, 0.47, 0.62)) +
  '; bad rate: ' + ', '.join(f'{d}: {np.mean([o["bad"] for o in T if abs(o["d"] - d) < 1e-6]):.2f}' for d in (0.35, 0.47, 0.62)))
Q4M = {'d': ['d'], 'd - L (fixed unit slope)': ['dly'], 'd, L': ['d', 'L'], 'd, L, log s': ['d', 'L', 's'],
       'd, L, yaw misfit @0.47': ['d', 'L', 'mis_yaw47'], 'd, L, surge misfit @0.47': ['d', 'L', 'mis_surge47'],
       'd, L, both misfits @0.47': ['d', 'L', 'mis_yaw47', 'mis_surge47'],
       'd, L, yaw misfit @rigid(d=0)': ['d', 'L', 'mis_yaw0'], 'd, L, ln m66 ratio @0.47': ['d', 'L', 'mis_m'],
       'd, L, yaw misfit @own d': ['d', 'L', 'mis_yaw'],
       'd, L, yaw misfit @0.47, log s': ['d', 'L', 'mis_yaw47', 's'],
       'lag_yaw hinge (own d)': ['+lag_yaw', '-lag_yaw'], 'lag_yaw hinge + L': ['+lag_yaw', '-lag_yaw', 'L'],
       'd, L, true body lag (tau avg)': ['d', 'L', 'tau_s']}
for oc in ('fail', 'bad'):
    y = np.array([o[oc] for o in T])
    groups = np.array([o['gid'] for o in T])
    P(f'\n  outcome {oc}: positive rate {y.mean():.2f}')
    P(f'    {"model":36s} {"CV err":>7s} {"logloss":>8s} {"AUC":>5s}   coefficients (raw) ; implied d_min = P=0.5 point: median [5,95] over boats')
    for name, spec in Q4M.items():
        X = feats(T, spec)
        pcv = cv_probs(X, y, groups)
        e = ((pcv > 0.5) != (y > 0.5)).astype(float).mean(0)
        ll = -(y * np.log(np.clip(pcv, 1e-6, 1)) + (1 - y) * np.log(np.clip(1 - pcv, 1e-6, 1))).mean(0)
        m = logit_fit(X, y)
        a, bb = raw_coef(m)
        extra = ''
        if spec[0] == 'd':
            theta = -(a + X[:, 1:] @ bb[1:]) / bb[0]
            # theta per boat: average over the 3 levels' observations
            tb, _ = per_boat_mean(theta, groups)
            extra = f' d_min {np.median(tb):.2f} [{np.percentile(tb, 5):.2f},{np.percentile(tb, 95):.2f}]'
        P(f'    {name:36s} {e.mean():7.3f} {ll.mean():8.3f} {np.mean([auc(pcv[r], y) for r in range(5)]):5.2f}   '
          f'a={a:+.1f} ' + ' '.join(f'{k}:{v:+.1f}' for k, v in zip(spec, bb)) + extra)

P('\n  Bootstrap (boats resampled, 300x) of the d_min model  d_min = a + bL*L + bm*misfit  [fail outcome, both libraries stacked]:')
for name in ('d, L', 'd, L, yaw misfit @0.47', 'd, L, surge misfit @0.47', 'd, L, both misfits @0.47', 'd, L, log s'):
    spec = Q4M[name]
    y = np.array([o['fail'] for o in T])
    groups = np.array([o['gid'] for o in T])
    X = feats(T, spec)
    ug = np.unique(groups)
    res = []
    for _ in range(300):
        pick = RNG.choice(ug, len(ug))
        ii = np.concatenate([np.where(groups == g)[0] for g in pick])
        yy = y[ii]
        if yy.sum() < 3:
            continue
        m = logit_fit(X[ii], yy)
        a, bb = raw_coef(m)
        if bb[0] >= -1e-6:
            continue
        res.append([-a / bb[0]] + [-bb[k] / bb[0] for k in range(1, len(spec))] + [-1 / bb[0]])
    res = np.array(res)
    m = logit_fit(X, y)
    a, bb = raw_coef(m)
    P(f'    {name:30s} a={-a / bb[0]:+.2f} [{np.percentile(res[:, 0], 5):+.2f},{np.percentile(res[:, 0], 95):+.2f}]  ' +
      '  '.join(f'{spec[k]}: {-bb[k] / bb[0]:+.2f} [{np.percentile(res[:, k], 5):+.2f},{np.percentile(res[:, k], 95):+.2f}]' for k in range(1, len(spec))) +
      f'  width sigma(1/|b_d|)={-1 / bb[0]:.3f} s [{np.percentile(res[:, -1], 5):.3f},{np.percentile(res[:, -1], 95):.3f}]  (n boot ok {len(res)})')
P('\n  misfit summary on M4 timing boats (seconds; >0 = twin too crisp): yaw@0.47 {} ; surge@0.47 {} ; yaw@rigid(d=0) {}'.format(
    fmt(np.array([o['mis_yaw47'] for o in T[::6]]), d=3), fmt(np.array([o['mis_surge47'] for o in T[::6]]), d=3),
    fmt(np.array([o['mis_yaw0'] for o in T[::6]]), d=3)))

# =============================================================================================
P('\n' + '=' * 100)
P('Q5 extras (exploratory)')
# (a) fixed unit-slope threshold on timing boats: d_min = L + a + c*misfit, bootstrap over boats
P('\n--- (a) M4 timing, assumed-delay arms, fail outcome: logit P(fail) = alpha + beta*(d - L) + gamma*misfit  =>  d_min = L - alpha/beta - (gamma/beta) misfit ---')
y = np.array([o['fail'] for o in T])
groups = np.array([o['gid'] for o in T])
ug = np.unique(groups)
for name, extra in (('no misfit', []), ('yaw misfit @0.47', ['mis_yaw47']), ('surge misfit @0.47', ['mis_surge47']),
                    ('yaw misfit @own d', ['mis_yaw']), ('log s', ['s'])):
    X = feats(T, ['dly'] + extra)
    m = logit_fit(X, y)
    a, bb = raw_coef(m)
    res = []
    for _ in range(400):
        pick = RNG.choice(ug, len(ug))
        ii = np.concatenate([np.where(groups == g)[0] for g in pick])
        if y[ii].sum() < 3:
            continue
        a2, b2 = raw_coef(logit_fit(X[ii], y[ii]))
        if b2[0] >= -1e-6:
            continue
        res.append([-a2 / b2[0]] + [-b2[k] / b2[0] for k in range(1, len(b2))] + [-1 / b2[0]])
    res = np.array(res)
    P(f'   {name:20s} offset (d_min - L at misfit=0) {-a / bb[0]:+.3f} [{np.percentile(res[:, 0], 5):+.3f},{np.percentile(res[:, 0], 95):+.3f}]  ' +
      ' '.join(f'{extra[k - 1]} coef {-bb[k] / bb[0]:+.2f} [{np.percentile(res[:, k], 5):+.2f},{np.percentile(res[:, k], 95):+.2f}]' for k in range(1, len(bb))) +
      f'  width {-1 / bb[0]:.3f} s [{np.percentile(res[:, -1], 5):.3f},{np.percentile(res[:, -1], 95):.3f}]')
# (b) what drives the fitted-delay bias? OLS of dly on hidden latent parameters (rigid+delay and joint), per stratum, M2+M4 pooled
P('\n--- (b) OLS of carried-delay bias (dly) on hidden parameters, standardized coefficients [90% boat-bootstrap CI]; R2 ---')
for arm in ('rigid+delay', 'joint'):
    for st in ('steady', 'transient', 'timing'):
        bs = [b for b in sel('ALL', None, arm) if b['stratum'] == st]
        y_ = col(bs, arm, 'dly')
        names = ['tau_mean', 'rev_hold', 'ramp_time', 'log_s', 'gamma', 'pos_sd', 'gyro_sd', 'k_asym']
        Xm = np.array([[0.5 * (b['tau_up'] + b['tau_down']), b['rev_hold'], b['kbar'] / b['slew'], np.log(b['s_eff']), b['gamma'], b['pos_sd'], b['gyro_sd'], b['k_asym']] for b in bs])
        Z = (Xm - Xm.mean(0)) / Xm.std(0)
        A = np.c_[np.ones(len(bs)), Z]
        beta = np.linalg.lstsq(A, y_, rcond=None)[0]
        r2 = 1 - ((y_ - A @ beta) ** 2).sum() / ((y_ - y_.mean()) ** 2).sum()
        bl = np.array([np.linalg.lstsq(A[i], y_[i], rcond=None)[0] for i in RNG.integers(0, len(bs), (300, len(bs)))])
        raw_tau = np.linalg.lstsq(np.c_[np.ones(len(bs)), Xm[:, 0], Xm[:, 1], Xm[:, 2]], y_, rcond=None)[0]
        P(f'  {arm:12s} {st:9s} n={len(bs):3d} R2 {r2:.2f} intercept(bias at mean) {beta[0]:+.3f} | ' +
          ' '.join(f'{n}:{beta[k + 1]:+.3f}[{np.percentile(bl[:, k + 1], 5):+.3f},{np.percentile(bl[:, k + 1], 95):+.3f}]{"*" if np.percentile(bl[:, k + 1], 5) > 0 or np.percentile(bl[:, k + 1], 95) < 0 else ""}' for k, n in enumerate(names)))
        P(f'      raw: dly = {raw_tau[0]:+.3f} + {raw_tau[1]:.2f} * tau_mean + {raw_tau[2]:.2f} * rev_hold + {raw_tau[3]:.2f} * ramp_time  (tau_mean range {Xm[:, 0].min():.2f}-{Xm[:, 0].max():.2f}, rev_hold {Xm[:, 1].min():.2f}-{Xm[:, 1].max():.2f}, ramp_time {Xm[:, 2].min():.2f}-{Xm[:, 2].max():.2f})')
# (c) normalised delay (delay / yaw time constant) feature
for o in OBS:
    b = BOATS[o['i']]
    f, h = b['fits'][o['arm']], b['hid']
    o['dr_yaw'] = f['delay'] / f['tau_yaw'] - b['L'] / h['tau_yaw']
    o['dr_surge'] = f['delay'] / f['tau_surge'] - b['L'] / h['tau_surge']
    o['ldr'] = np.log((f['delay'] + 0.05) / (b['L'] + 0.05))
    o['lag_yaw_p'] = f['t63_yaw'] + f['lat'] - (b['fits']['oracle-proj']['t63_yaw'] + b['fits']['oracle-proj']['lat'])
MODELS = {'excess dly hinge': MODELS['excess dly hinge'],
          'dly hinge + log s': MODELS['dly hinge + log s'],
          'lag_yaw hinge (vs hidden)': ['+lag_yaw', '-lag_yaw'],
          'lag_yaw hinge (vs oracle-proj twin)': ['+lag_yaw_p', '-lag_yaw_p'],
          'dly/tau_yaw difference hinge': ['+dr_yaw', '-dr_yaw'],
          'dly/tau_surge difference hinge': ['+dr_surge', '-dr_surge'],
          'ln((d+.05)/(L+.05)) hinge': ['+ldr', '-ldr'],
          'dly hinge + dr_yaw hinge': ['+dly', '-dly', '+dr_yaw', '-dr_yaw']}
P('\n--- (c) alternative single quantities, boat-grouped CV, all arms pooled ---')
for oc in ('bad', 'fail'):
    run_cv(OBS, oc, oc)
P('\n--- (d) equivalent lag of fitted arms relative to the oracle-proj twin (removes projection floor): lag_yaw_p, seconds ---')
for ds in ('M4', 'M2'):
    for st in STRATA[ds]:
        bs = sel(ds, st)
        P(f'  {ds} {st:9s} ' + ' | '.join(f'{a}: ' + fmt(np.array([(b["fits"][a]["t63_yaw"] + b["fits"][a]["lat"]) - (b["fits"]["oracle-proj"]["t63_yaw"] + b["fits"]["oracle-proj"]["lat"]) for b in bs]), d=3) for a in FIT_ARMS[ds]))
P('\n--- (e) equivalent-delay accounting (seconds, median over boats, M2+M4 pooled by stratum): carried-delay bias, yaw total-lag bias, share of delay bias absorbed by lighter body ---')
for arm in ('rigid+delay', 'joint'):
    for st in ('steady', 'transient', 'timing'):
        bs = [b for b in sel('ALL', None, arm) if b['stratum'] == st]
        dl, lg = col(bs, arm, 'dly'), col(bs, arm, 'lag_yaw')
        lgs = col(bs, arm, 'lag_surge')
        P(f'  {arm:12s} {st:9s} n={len(bs):3d} dly {fmt(dl, d=3)}  lag_yaw {fmt(lg, d=3)}  lag_surge {fmt(lgs, d=3)}  '
          f'absorbed share of delay bias (1 - lag_yaw/dly, ratio of medians) {1 - np.median(lg) / np.median(dl):.2f}')
