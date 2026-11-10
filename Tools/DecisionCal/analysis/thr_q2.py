"""Q2: what the pick changes when the twin's delay crosses the threshold; hidden failure structure across controllers."""
import sys
sys.path.insert(0, __file__.rsplit('/', 1)[0])
from thr_common import *
from thr_q1 import logit_fit

DOCKS = ('const', 'sqrt', 'two_stage', 'sqrt@0.08', 'sqrt@0.3', 'sqrt@0.6')

def feat(lib):
    T = TABLES[lib]
    return np.array([[t[0], t[1], t[2], t[3], t[4], t[6], t[7]] for t in T], float)      # kp ki cruise turn brake latcomp ff
FN = ('kp', 'ki', 'cruise', 'turn_alat', 'brake', 'latcomp', 'ff')

def pick_shift(rows, stratum, lib, a, b, label, out, cond=None, nb=2000):
    """Per-boat paired change in pick parameters, arm a minus arm b (same boat), optional boat filter."""
    F = feat(lib); dif = []; keep = []
    for r in rows:
        if r['stratum'] != stratum or a not in r['libs'][lib]['scores']['arms'] or b not in r['libs'][lib]['scores']['arms']: continue
        if cond and not cond(r, lib): continue
        sa = r['libs'][lib]['scores']['arms'][a]['sel']; sb = r['libs'][lib]['scores']['arms'][b]['sel']
        dif.append(F[sa] - F[sb])
    dif = np.array(dif)
    if len(dif) < 3: out(f'  {label}: n={len(dif)} (too few)'); return
    rng = np.random.default_rng(0); idx = rng.integers(0, len(dif), (nb, len(dif)))
    m = dif.mean(0); ci = np.percentile(dif[idx].mean(1), [5, 95], axis=0)
    same = np.mean(np.all(dif == 0, 1))
    out(f'  {label} (n={len(dif)}, pick unchanged {same:.2f}): ' + '  '.join(f'{FN[j]} {m[j]:+.2f}[{ci[0][j]:+.2f},{ci[1][j]:+.2f}]' for j in range(len(FN))))

def pick_means(rows, stratum, lib, arms, out):
    F = feat(lib)
    out(f'  mean parameters of the picked controller ({stratum}, {lib}):  ' + ' '.join(f'{n:>8s}' for n in FN) + '   hidden dock   hidden J')
    for a in arms:
        P, dk, Jh = [], [], []
        for r in rows:
            if r['stratum'] != stratum: continue
            L = r['libs'][lib]
            s = L['scores']['oracle'] if a == 'ORACLE' else (L['scores']['arms'].get(a) or {}).get('sel')
            if s is None: continue
            P.append(F[s]); dk.append(L['hidden_docked'][s]); Jh.append(np.mean(L['J_h'][s]))
        if P: out(f'    {a:14s} ' + ' '.join(f'{x:8.2f}' for x in np.mean(P, 0)) + f'   {np.mean(dk):.2f}   {np.mean(np.minimum(Jh, 300)):.1f}')

def failure_structure(rows, stratum, lib, out, nb=300):
    """Controller-level hidden failure (docked < 0.5): univariate rates, multivariate logit with boat-cluster bootstrap."""
    F = feat(lib); X, y, bid, cont, Lb = [], [], [], [], []
    ids = sorted({r['id'] for r in rows if r['stratum'] == stratum})
    for r in rows:
        if r['stratum'] != stratum: continue
        L = r['libs'][lib]; dk = np.array(L['hidden_docked']); ct = np.array(L['hidden_contacts'])
        for c in range(len(dk)):
            X.append(F[c]); y.append(float(dk[c] < 0.5)); bid.append(r['id']); cont.append(ct[c]); Lb.append(r['spec']['dead_time'] + r['spec']['odom_latency'])
    X, y, cont, Lb = np.array(X), np.array(y), np.array(cont), np.array(Lb)
    out(f'  [{stratum}, {lib}] hidden failure (dock<0.5) rate over all controllers: {y.mean():.3f}; boats with >=1 failing controller: '
        f'{np.mean([y[np.array(bid) == i].max() for i in ids]):.2f}')
    fc = cont[y > 0.5]
    out(f'    contacts per failing mission-set (mean over K): median {np.median(fc):.1f}, share > 10: {(fc > 10).mean():.2f}; non-failing: median {np.median(cont[y < 0.5]):.2f}')
    out(f'    share of failing sets with <3 contact events (single-overshoot-like): {(fc < 3).mean():.2f}; with >=20 (repeated wall hits): {(fc >= 20).mean():.2f}')
    q3 = np.quantile(Lb, [1 / 3, 2 / 3]); gL = np.digitize(Lb, q3)
    out('    failure rate for kp=1.5 / kp<=1.0 by true-L tercile (cuts %.2f,%.2f): ' % tuple(q3) + ', '.join(
        f'{y[(gL == k) & (X[:, 0] == 1.5)].mean():.2f} / {y[(gL == k) & (X[:, 0] <= 1.0)].mean():.2f}' for k in range(3)))
    for j in (0, 5, 6):
        vals = sorted(set(X[:, j]))
        out(f'    failure rate by {FN[j]}: ' + ', '.join(f'{v:g}: {y[X[:, j] == v].mean():.2f} (n={int((X[:, j] == v).sum())})' for v in vals))
    for j in (2, 4):
        q = np.quantile(X[:, j], [1 / 3, 2 / 3]); g = np.digitize(X[:, j], q)
        out(f'    failure rate by {FN[j]} tercile (cuts {q[0]:.2f},{q[1]:.2f}): ' + ', '.join(f'{y[g == k].mean():.2f}' for k in range(3)))
    # multivariate logit on standardised features + L  (hidden failure ~ controller features); cluster bootstrap by boat
    Z = (X - X.mean(0)) / (X.std(0) + 1e-12)
    D = np.column_stack([np.ones(len(y)), Z])
    ub = sorted(set(bid)); bidv = np.array(bid)
    w = logit_fit(D, y, 0.5)
    rng = np.random.default_rng(2); bs = []
    pos = {u: np.where(bidv == u)[0] for u in ub}
    for _ in range(nb):
        ch = rng.choice(len(ub), len(ub)); idx = np.concatenate([pos[ub[c]] for c in ch])
        try: bs.append(logit_fit(D[idx], y[idx], 0.5))
        except Exception: pass
    bs = np.array(bs); lo, hi = np.percentile(bs, [5, 95], axis=0)
    out(f'    multivariate logit (log-odds per +1 SD of the controller feature; ridge 0.5): ' + '  '.join(
        f'{FN[j]} {w[j + 1]:+.2f}[{lo[j + 1]:+.2f},{hi[j + 1]:+.2f}]' for j in range(len(FN))))

def main(out=print, runs=None):
    runs = runs or {'m2': 'm2-20261112', 'm3-fixed': 'm3-20261113'}
    R2, R3 = load(runs['m2']), load(runs['m3-fixed'])
    for lib in ('v2', 'v3'):
        out(f'===== Q2 library {lib}')
        for nm, rows in (('M3', R3), ('M2', R2)):
            for st in ('timing', 'transient'):
                arms = [a for a in ('rigid', 'rigid@d0.16', 'rigid@d0.25', 'rigid@d0.35', 'rigid+delay', 'joint', 'rigid+lag', 'ORACLE') if a == 'ORACLE' or a in rows[0]['libs'][lib]['scores']['arms']]
                out(f'--- {nm} {st}')
                pick_means(rows, st, lib, arms, out)
        out('--- paired change in the pick (same boat), timing boats')
        pick_shift(R3, 'timing', lib, 'rigid@d0.35', 'rigid', 'M3 timing, @0.35 minus rigid, all boats', out)
        pick_shift(R3, 'timing', lib, 'rigid@d0.35', 'rigid', 'M3 timing, @0.35 minus rigid, boats with L<=0.35 (twin delay >= L)', out,
                   cond=lambda r, l: r['spec']['dead_time'] + r['spec']['odom_latency'] <= 0.35)
        pick_shift(R3, 'timing', lib, 'rigid@d0.35', 'rigid', 'M3 timing, @0.35 minus rigid, boats with L>0.38 (twin delay < L)', out,
                   cond=lambda r, l: r['spec']['dead_time'] + r['spec']['odom_latency'] > 0.38)
        pick_shift(R3, 'timing', lib, 'rigid@d0.35', 'rigid@d0.25', 'M3 timing, @0.35 minus @0.25', out)
        pick_shift(R3, 'timing', lib, 'rigid@d0.25', 'rigid', 'M3 timing, @0.25 minus rigid', out)
        pick_shift(R2, 'timing', lib, 'rigid+delay', 'rigid', 'M2 timing, fitted delay minus rigid', out)
        pick_shift(R3, 'transient', lib, 'rigid@d0.35', 'rigid', 'M3 transient, @0.35 minus rigid', out)
        pick_shift(R2, 'transient', lib, 'rigid+delay', 'rigid', 'M2 transient, fitted delay minus rigid', out)
        pick_shift(R2, 'transient', lib, 'rigid+lag', 'rigid', 'M2 transient, fitted lag minus rigid', out)
        out('--- hidden failure structure over all 16 controllers (both runs pooled, boats are independent draws)')
        for st in ('timing', 'transient'):
            failure_structure(R2 + R3, st, lib, out)
if __name__ == '__main__':
    main()
