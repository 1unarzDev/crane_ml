"""Post-hoc analysis of the DecisionCal M0b run (read-only; no simulation is rerun).

Usage: nice -n 19 python3 analyze_m0b.py [RUN_DIR] > out.txt
Reads plants.jsonl only. numpy only. Everything here is POST-HOC / exploratory.
"""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from score import cross_fit_regret, holm, sign_flip_p, kendall_tau  # noqa: E402
from library import LIBRARY, factor_matrix  # noqa: E402

RUN = Path(sys.argv[1] if len(sys.argv) > 1 else '/home/lunarz/.cache/decisioncal/m0b-20261109')
ROWS = [json.loads(l) for l in open(RUN / 'plants.jsonl')]
STRATA = ['steady', 'transient', 'timing', 'mixed']
ARMS = ['nominal', 'nominal_d100', 'broad', 'steady', 'timing', 'transient', 'joint',
        'joint-steady', 'joint-timing', 'joint-transient']
RNG = np.random.default_rng(12345)
BY = {st: [r for r in ROWS if r['stratum'] == st] for st in STRATA}
JH = {st: np.array([r['J_h'] for r in BY[st]], float) for st in STRATA}          # [B,16,6]
FM = factor_matrix()


def H(title):
    print('\n' + '=' * 100 + f'\n{title}\n' + '=' * 100)


def reg(st, arm, key='regret'):
    return np.array([r['scores']['arms'][arm][key] for r in BY[st]])


def fixed_regret(Jh, c):
    return np.array([cross_fit_regret(J, c) for J in Jh])


def boot_ci(x, n=4000, q=(0.05, 0.95), stat=np.mean):
    idx = RNG.integers(0, len(x), (n, len(x)))
    v = np.array([stat(x[i]) for i in idx])
    return float(np.quantile(v, q[0])), float(np.quantile(v, q[1]))


def fmt_ci(x, stat=np.mean):
    lo, hi = boot_ci(np.asarray(x, float), stat=stat)
    return f'{stat(x):7.2f} [{lo:7.2f},{hi:7.2f}]'


def qs(x):
    return ' '.join(f'{v:6.1f}' for v in np.quantile(x, [0, .1, .25, .5, .75, .9, 1]))


# ---------------------------------------------------------------------------------------------
def q0_sanity():
    H('0. SANITY: recomputed regrets and the per-stratum table')
    mx = 0
    for r in ROWS:
        J = np.array(r['J_h'], float)
        for a in ARMS:
            mx = max(mx, abs(cross_fit_regret(J, r['scores']['arms'][a]['sel']) - r['scores']['arms'][a]['regret']))
    print('max |recomputed cross-fit regret - stored| =', mx)
    print('mean regret by stratum (rows) x arm:')
    print('stratum      ' + ' '.join(f'{a[:9]:>9}' for a in ARMS))
    for st in STRATA + ['all']:
        v = [np.mean(np.concatenate([reg(s, a) for s in (STRATA if st == 'all' else [st])])) for a in ARMS]
        print(f'{st:12} ' + ' '.join(f'{x:9.2f}' for x in v))
    print('nominal selects:', {st: sorted(set(r['scores']['arms']['nominal']['sel'] for r in BY[st])) for st in STRATA})
    print('nominal_d100 selects:', {st: sorted(set(r['scores']['arms']['nominal_d100']['sel'] for r in BY[st])) for st in STRATA})


# ---------------------------------------------------------------------------------------------
def decompose(r, c):
    """Mean over K of J components for controller c on the hidden boat."""
    m = r['hidden_metrics']
    dk = np.array(m['docked'], float)[c]
    td = np.nan_to_num(np.array(m['t_dock'], float)[c], nan=0.0)
    time_ok = (dk * td).mean()                       # time of runs that docked
    fail = ((1 - dk) * 180.0).mean()                 # timeout 120 + 60 penalty
    cont = (20 * np.minimum(np.array(m['contacts'], float)[c], 3)).mean()
    gate = (30 * np.array(m['gate_miss'], float)[c]).mean()
    pose = (10 * np.minimum(np.array(m['pose_err'], float)[c], 3)).mean()
    return np.array([time_ok, fail, cont, gate, pose]), dk.mean()


def q1():
    H('1. MECHANISM of the transient-stratum harm')
    st = 'transient'
    R = BY[st]
    for arm in ['nominal', 'timing', 'joint-transient', 'joint']:
        print(f'\n--- arm {arm}, transient stratum ---')
        regs = reg(st, arm)
        sel = np.array([r['scores']['arms'][arm]['sel'] for r in R])
        orc = np.array([r['scores']['oracle'] for r in R])
        print('regret quantiles (min,10,25,50,75,90,max):', qs(regs))
        print(f'mean {regs.mean():.2f}  median {np.median(regs):.2f}  #regret>2: {(regs > 2).sum()}/16  >10: {(regs > 10).sum()}  >60: {(regs > 60).sum()}')
        print('sorted regrets:', np.round(np.sort(regs), 1))
        print('selected counts:', dict(zip(*np.unique(sel, return_counts=True))), ' oracle counts:', dict(zip(*np.unique(orc, return_counts=True))))
        # decomposition of raw regret
        comp = np.zeros(5)
        rows = []
        for r, s, o in zip(R, sel, orc):
            cs, ds = decompose(r, s)
            co, do = decompose(r, o)
            rows.append((cs - co, ds, do))
        D = np.array([x[0] for x in rows])
        dfrac = np.array([x[1] for x in rows])
        raw = D.sum(1)
        print('raw-regret components (mean over boats): time_of_docked_runs %.2f  FAIL(timeout+60) %.2f  contacts %.2f  gate %.2f  pose %.2f ; total %.2f'
              % (*D.mean(0), raw.mean()))
        anyfail = dfrac < 1
        print(f'boats where selected controller fails to dock in >=1 of 6 hidden repeats: {anyfail.sum()}/16; all 6 fail: {(dfrac == 0).sum()}')
        if anyfail.any():
            print(f'   share of total raw regret from those boats: {raw[anyfail].sum() / raw.sum():.2f}; their mean raw regret {raw[anyfail].mean():.1f}')
        print(f'   non-failing boats: mean raw regret {raw[~anyfail].mean():.2f}, max {raw[~anyfail].max():.2f}')
        print(f'   hidden dock-fraction of selected controller per boat: {np.round(dfrac, 2)}')
        # slower-but-successful: boats with no fails
        print('   per boat (sel,oracle,regret_cf,dock_frac_sel,rawΔtime,ΔFAIL): ')
        for i, (r, s, o) in enumerate(zip(R, sel, orc)):
            print(f'     {r["id"]:14} sel c{s:02d} orc c{o:02d} reg {regs[i]:7.1f} dock {dfrac[i]:.2f} dT {D[i,0]:7.1f} dF {D[i,1]:7.1f} dOther {D[i,2:].sum():6.1f}')

    print('\n--- selected-controller factor levels, transient stratum (mean over boats of level values) ---')
    names = ['turn_alat', 'brake', 'cruise', 'ff', 'latcomp', 'dock']
    for arm in ['nominal', 'timing', 'joint-transient', 'joint']:
        sel = [r['scores']['arms'][arm]['sel'] for r in R]
        print(f'{arm:16}', ' '.join(f'{n}={FM[sel, j].mean():.2f}' for j, n in enumerate(names[:5])),
              ' dock-types:', np.bincount([int(FM[s, 5]) for s in sel], minlength=4))
    sel_o = [r['scores']['oracle'] for r in R]
    print(f'{"oracle":16}', ' '.join(f'{n}={FM[sel_o, j].mean():.2f}' for j, n in enumerate(names[:5])), ' dock-types:', np.bincount([int(FM[s, 5]) for s in sel_o], minlength=4))

    print('\n--- twin belief vs hidden truth about c08 (the default/oracle) and about the chosen controller, transient stratum ---')
    for arm in ['timing', 'joint-transient', 'joint']:
        pc08 = [np.mean(np.where(np.isfinite(np.array(r['J_arms'][arm], float)[8]), np.array(r['J_arms'][arm], float)[8], 300)) for r in R]
        hc08 = [np.mean(r['J_h'][8]) for r in R]
        psel = [r['scores']['arms'][arm]['pred_J'] for r in R]
        hsel = [r['scores']['arms'][arm]['hidden_J'] for r in R]
        print(f'{arm:16} mean twin-predicted J[c08] {np.mean(pc08):6.1f} vs hidden J[c08] {np.mean(hc08):6.1f};  twin-pred J[sel] {np.mean(psel):6.1f} vs hidden J[sel] {np.mean(hsel):6.1f}')
        print('    fraction of boats where twin predicts c08 docks-fail class (pred J[c08] > 120):', np.mean(np.array(pc08) > 120))
        print('    twin-predicted J[c08]-J[sel] (belief of gain from switching) mean %.1f ; hidden J[c08]-J[sel] mean %.1f'
              % (np.mean(np.array(pc08) - np.array(psel)), np.mean(np.array(hc08) - np.array(hsel))))

    print('\n--- FITTED DELAY vs TRUE delay (dead_time + odom_latency) and tau_up/rev_hold ---')
    print('MAP delay_s grid: [0, .08, .16, .25, .35, .47, .62]. True ranges: transient dead+odom 0-0.09, timing 0.15-0.45, mixed 0.08-0.32, steady 0-0.09')
    for arm in ['timing', 'joint-transient', 'joint', 'joint-timing', 'joint-steady']:
        if arm == 'joint-timing':
            print('(joint-timing does not fit delay; skipped)')
            continue
        if arm == 'joint-steady':
            pass
        print(f'\n arm {arm}:')
        for st2 in STRATA + ['all']:
            Rs = ROWS if st2 == 'all' else BY[st2]
            fd = np.array([r['fits'][arm]['map']['delay_s'] for r in Rs])
            td = np.array([r['spec']['dead_time'] + r['spec']['odom_latency'] for r in Rs])
            tu = np.array([r['spec']['tau_up'] for r in Rs])
            td2 = np.array([r['spec']['tau_down'] for r in Rs])
            rh = np.array([r['spec']['rev_hold'] for r in Rs])
            ex = fd - td
            line = f'  {st2:10} fitted mean {fd.mean():.3f} true delay mean {td.mean():.3f} bias {ex.mean():+.3f};'
            if fd.std() > 0:
                line += (f' corr(fit,true_delay) {np.corrcoef(fd, td)[0,1]:+.2f} corr(fit,tau_up) {np.corrcoef(fd, tu)[0,1]:+.2f}'
                         f' corr(fit,rev_hold) {np.corrcoef(fd, rh)[0,1]:+.2f} corr(fit,tau_up+rev_hold) {np.corrcoef(fd, tu + rh)[0,1]:+.2f}')
            print(line)
            if st2 in ('transient', 'all') and fd.std() > 0:
                X = np.column_stack([np.ones(len(fd)), td, tu, rh])
                b, *_ = np.linalg.lstsq(X, fd, rcond=None)
                res = fd - X @ b
                r2 = 1 - res.var() / fd.var()
                print(f'      OLS fit = {b[0]:.3f} + {b[1]:.2f}*true_delay + {b[2]:.2f}*tau_up + {b[3]:.2f}*rev_hold  R2={r2:.2f}')
                print('      fitted delay values:', np.round(np.sort(fd), 2))
    print('\n true delay / tau_up / rev_hold in transient stratum (mean, min, max):')
    for k in ('dead_time', 'odom_latency', 'tau_up', 'tau_down', 'rev_hold'):
        v = np.array([r['spec'][k] for r in BY['transient']])
        print(f'   {k:13} {v.mean():.3f} {v.min():.3f} {v.max():.3f}')
    # joint-transient / timing: grid composition
    print('\n joint-transient fitted non-delay axes in transient stratum (it fits rev_ratio, deadband too):')
    for arm in ['joint-transient']:
        print('  frac_clipped sway mean', np.mean([r['fits'][arm]['frac_clipped']['sway'] for r in BY['transient']]))
    # relation of fitted delay to regret
    print('\n regret vs fitted delay, transient stratum (per boat, timing arm): rank corr')
    for arm in ['timing', 'joint-transient']:
        fd = np.array([r['fits'][arm]['map']['delay_s'] for r in BY['transient']])
        rg = reg('transient', arm)
        ra, rb = np.argsort(np.argsort(fd)), np.argsort(np.argsort(rg))
        print(f'   {arm}: spearman(fitted delay, regret) = {np.corrcoef(ra, rb)[0,1]:+.2f}')
        for v in sorted(set(fd)):
            m = fd == v
            print(f'      fitted delay {v:.2f}: n={m.sum()} mean regret {rg[m].mean():.1f}')
    # does fitted delay > threshold flip decision: the selected-controller type vs fitted delay
    print('\n Also: in timing stratum (true delay large) what do timing/joint choose vs oracle (selection agreement)')
    for arm in ['nominal', 'timing', 'joint', 'joint-transient']:
        sel = np.array([r['scores']['arms'][arm]['sel'] for r in BY['timing']])
        o = np.array([r['scores']['oracle'] for r in BY['timing']])
        print(f'   {arm:16} exact oracle match {np.mean(sel == o):.2f}; regret quantiles {qs(reg("timing", arm))}')


# ---------------------------------------------------------------------------------------------
def q2():
    H('2. LIBRARY DEGENERACY and FIXED-CONTROLLER baselines')
    for st in STRATA:
        Jh = JH[st]
        orc = np.array([r['scores']['oracle'] for r in BY[st]])
        cnt = np.bincount(orc, minlength=16)
        p = cnt / cnt.sum()
        ent = -(p[p > 0] * np.log2(p[p > 0])).sum()
        # number of controllers within 2 s of the oracle (by hidden mean)
        mj = Jh.mean(2)
        near = ((mj - mj.min(1, keepdims=True)) <= 2.0).sum(1)
        print(f'\n[{st}] distinct oracles {len(set(orc))}, counts {dict((i, int(c)) for i, c in enumerate(cnt) if c)}; top share {p.max():.2f}; entropy {ent:.2f} bits (max 4)')
        print(f'   #controllers within 2 s of the oracle per boat: mean {near.mean():.1f}, median {np.median(near)}; oracle J mean {mj.min(1).mean():.1f}; c08 J mean {mj[:,8].mean():.1f}; c00 J mean {mj[:,0].mean():.1f}')
        print(f'   hidden mean J over controllers (mean over boats): ' + ' '.join(f'c{c:02d}:{mj[:, c].mean():.0f}' for c in range(16)))
        # fixed baselines
        fr = np.array([fixed_regret(Jh, c) for c in range(16)])           # [16,B]
        means = fr.mean(1)
        best = int(np.argmin(means))
        # leave-one-boat-out choice of best fixed controller (honest)
        loo = []
        for b in range(len(Jh)):
            others = np.delete(fr, b, axis=1).mean(1)
            loo.append(fr[int(np.argmin(others)), b])
        loo = np.array(loo)
        print(f'   FIXED c08 regret {fmt_ci(fr[8])}   FIXED c00 regret {fmt_ci(fr[0])}   best-in-sample fixed c{best:02d} {fmt_ci(fr[best])}   LOBO-chosen fixed {fmt_ci(loo)}')
        print(f'   fixed regret by controller: ' + ' '.join(f'c{c:02d}:{means[c]:.1f}' for c in range(16)))
        print('   --- arm vs fixed baseline: mean regret(arm) - mean regret(fixed) paired, bootstrap 90% CI; (neg => arm better)')
        for arm in ARMS:
            ra = reg(st, arm)
            line = f'     {arm:16} regret {ra.mean():6.2f} | '
            for lab, f in (('c08', fr[8]), ('c00', fr[0]), (f'best c{best:02d}', fr[best]), ('LOBO', loo)):
                d = ra - f
                lo, hi = boot_ci(d)
                line += f'vs {lab}: {d.mean():+7.2f} [{lo:+7.2f},{hi:+7.2f}] '
            print(line)
    # pooled
    print('\n[ALL 64 boats pooled]')
    allJ = np.concatenate([JH[s] for s in STRATA])
    fr = np.array([fixed_regret(allJ, c) for c in range(16)])
    means = fr.mean(1)
    best = int(np.argmin(means))
    print(f'   fixed c08 {fmt_ci(fr[8])}  c00 {fmt_ci(fr[0])}  best fixed c{best:02d} {fmt_ci(fr[best])}')
    print('   fixed by controller: ' + ' '.join(f'c{c:02d}:{means[c]:.1f}' for c in range(16)))
    for arm in ARMS:
        ra = np.concatenate([reg(s, arm) for s in STRATA])
        d = ra - fr[best]
        d8 = ra - fr[8]
        print(f'     {arm:16} pooled regret {ra.mean():6.2f}; minus best-fixed {d.mean():+6.2f} [{boot_ci(d)[0]:+6.2f},{boot_ci(d)[1]:+6.2f}]; minus c08 {d8.mean():+6.2f} [{boot_ci(d8)[0]:+6.2f},{boot_ci(d8)[1]:+6.2f}]')
    # per-stratum oracle-fixed: how much does stratum-conditional fixed help; 'universal' best fixed from other strata
    print('\n   Transfer: best fixed chosen on OTHER three strata, evaluated on each stratum')
    for st in STRATA:
        oth = np.concatenate([JH[s] for s in STRATA if s != st])
        fro = np.array([fixed_regret(oth, c) for c in range(16)]).mean(1)
        c = int(np.argmin(fro))
        mine = fixed_regret(JH[st], c)
        print(f'     {st:10} chosen c{c:02d}; regret on {st}: {fmt_ci(mine)}')


# ---------------------------------------------------------------------------------------------
def design(levels_dock=True):
    """OLS design for the L16: intercept + 5 two-level factors (effect-coded +-1/2 sign) + 3 dock dummies."""
    cols = []
    for j in range(5):
        lv = np.unique(FM[:, j])
        cols.append(np.where(FM[:, j] == lv.max(), 0.5, -0.5))
    D = np.stack(cols, 1)
    dk = FM[:, 5].astype(int)
    dm = np.stack([(dk == k).astype(float) for k in (1, 2, 3)], 1)
    dm = dm - dm.mean(0)
    return np.column_stack([np.ones(16), D, dm]), dk


def q3():
    H('3. FACTOR MAIN EFFECTS (L16; latcomp and dock are aliased with 2-factor interactions of b0..b3)')
    names = ['turn_alat(0.12 vs 0.5)', 'brake(0.2 vs 0.6)', 'cruise(0.8 vs 1.2)', 'ff(off vs on)', 'latcomp(0 vs .15)']
    # direction: coefficient = J(level high-numeric) - J(level low-numeric)
    X, dk = design()
    print('design rank', np.linalg.matrix_rank(X), 'of', X.shape[1], 'columns, 16 runs; coef = J(high numeric level) - J(low numeric level):')
    print('turn_alat high=0.5 (wide), brake high=0.6 (reverse), cruise high=1.2, ff high=on, latcomp high=0.15, dock dummies vs const (const=0, sqrt=1, two_stage=2, creep=3)')
    Xp = np.linalg.pinv(X)
    for st in STRATA:
        mj = JH[st].mean(2)                                    # [B,16]
        cap = np.minimum(mj, 130)                              # cap failures to see typical effects
        fail = np.array([np.mean(np.array(r['hidden_metrics']['docked'], float), axis=1) for r in BY[st]])  # [B,16]
        print(f'\n[{st}]  hidden-boat J (mean over boats):  main-effect coefficients')
        for lab, Y in (('J', mj), ('dock-fail rate', 1 - fail)):
            B = (Xp @ Y.T)                                    # [9,B]
            m = B.mean(1)
            # fraction of variance of the 16 controller means explained by main effects
            Yhat = X @ B
            r2 = 1 - ((Y.T - Yhat) ** 2).sum() / ((Y.T - Y.T.mean(0)) ** 2).sum()
            se = B.std(1, ddof=1) / np.sqrt(B.shape[1])
            s = ' '.join(f'{n.split("(")[0]}={m[1 + j]:+7.2f}±{se[1 + j]:.1f}' for j, n in enumerate(names))
            s += ' dock[sqrt,2stg,creep vs const]=' + ','.join(f'{m[6 + k]:+.1f}' for k in range(3))
            print(f'   {lab:15} {s}   R2(main effects, mean over boats)={r2:.2f}')
        # per-boat sign consistency of each factor effect on J
        B = (Xp @ mj.T)
        print('   sign consistency (share of boats with coef>0): ' + ' '.join(f'{n.split("(")[0]}={np.mean(B[1 + j] > 0):.2f}' for j, n in enumerate(names)))
    print('\n--- Where the TWIN gets the factor effects wrong: (twin-predicted effect, mean over boats & particles) vs hidden effect (J) per arm, transient & timing & mixed ---')
    for st in ['transient', 'timing', 'mixed']:
        mj = JH[st].mean(2)
        Bh = (Xp @ mj.T).mean(1)
        print(f'\n[{st}] hidden effects: ' + ' '.join(f'{n.split("(")[0]}={Bh[1 + j]:+.1f}' for j, n in enumerate(names)) + ' dock ' + ','.join(f'{Bh[6 + k]:+.1f}' for k in range(3)))
        for arm in ['nominal', 'timing', 'joint-transient', 'joint', 'broad']:
            Jt = np.array([np.where(np.isfinite(np.array(r['J_arms'][arm], float)), np.array(r['J_arms'][arm], float), 300).mean(1) for r in BY[st]])
            Bt = (Xp @ Jt.T).mean(1)
            # per-boat sign agreement of effect on (hidden vs twin)
            Bth = (Xp @ Jt.T)
            Bhh = (Xp @ mj.T)
            agree = [np.mean(np.sign(Bth[1 + j]) == np.sign(Bhh[1 + j])) for j in range(5)]
            print(f'   twin {arm:16}' + ' '.join(f'{n.split("(")[0]}={Bt[1 + j]:+.1f}' for j, n in enumerate(names)) + ' | sign agree w/ hidden per boat: ' + ' '.join(f'{a:.2f}' for a in agree))
    print('\n--- Arm disagreement explained by factors: factor levels of selected controllers (share of boats with level high) ---')
    for st in ['transient', 'timing', 'mixed']:
        print(f'[{st}]')
        for arm in ['nominal', 'timing', 'joint-transient', 'joint', 'oracle']:
            sel = [r['scores']['oracle'] if arm == 'oracle' else r['scores']['arms'][arm]['sel'] for r in BY[st]]
            hi = [np.mean(FM[sel, j] == FM[:, j].max()) for j in range(5)]
            print(f'   {arm:16} ' + ' '.join(f'{n.split("(")[0]}:{h:.2f}' for n, h in zip(names, hi)))
    print('\n--- Which factor change is associated with failing to dock (hidden)? dock-fail rate by controller level, per stratum ---')
    for st in STRATA:
        fail = 1 - np.array([np.mean(np.array(r['hidden_metrics']['docked'], float), axis=1) for r in BY[st]])  # [B,16]
        fr = fail.mean(0)
        print(f'[{st}] boats-by-controller fail rate: ' + ' '.join(f'c{c:02d}:{fr[c]:.2f}' for c in range(16)))


# ---------------------------------------------------------------------------------------------
def pvals_family(getreg, strata_override=None):
    """Replicates score.hypotheses family (9 arms x 4 strata) for a given regret getter(st, arm)->array."""
    p, g = {}, {}
    for st in STRATA:
        nom = getreg(st, 'nominal')
        for a in ARMS:
            if a == 'nominal':
                continue
            d = nom - getreg(st, a)
            p[(st, a)], g[(st, a)] = sign_flip_p(d), float(d.mean())
    return p, g, holm(p)


def exact_sign_test(d):
    d = d[d != 0]
    n = len(d)
    if n == 0:
        return 1.0
    k = min((d > 0).sum(), (d < 0).sum())
    from math import comb
    return float(min(1.0, 2 * sum(comb(n, i) for i in range(k + 1)) / 2 ** n))


def q4():
    H('4. SENSITIVITY of the harm conclusion (nominal minus arm gain; NEGATIVE = harm)')
    HARM = [('transient', 'timing'), ('transient', 'joint-transient')]
    # baseline
    base = lambda st, a: reg(st, a)
    p, g, adj = pvals_family(base)
    print('baseline (replicates summary.json): ' + '; '.join(f'{s}:{a} gain {g[(s, a)]:+.1f} raw p {p[(s, a)]:.4f} Holm {adj[(s, a)]:.4f}' for s, a in HARM))
    # (a) drop-one-boat in transient stratum, recompute entire family Holm
    print('\n(a) drop each single boat (transient stratum, n=15), full 36-test Holm recomputed:')
    for s, a in HARM:
        out = []
        for drop in range(16):
            keep = [i for i in range(16) if i != drop]
            pp, gg = {}, {}
            for st in STRATA:
                for arm in ARMS:
                    if arm == 'nominal':
                        continue
                    nom, ar = reg(st, 'nominal'), reg(st, arm)
                    if st == 'transient':
                        nom, ar = nom[keep], ar[keep]
                    pp[(st, arm)] = sign_flip_p(nom - ar)
                    gg[(st, arm)] = float((nom - ar).mean())
            ad = holm(pp)
            out.append((drop, gg[(s, a)], pp[(s, a)], ad[(s, a)]))
        worst = max(out, key=lambda x: x[3])
        best = min(out, key=lambda x: x[3])
        print(f'   {s}:{a}: Holm p range over drops [{best[3]:.4f}, {worst[3]:.4f}]; worst drop = {ROWS and BY["transient"][worst[0]]["id"]} (gain {worst[1]:+.1f}, raw {worst[2]:.4f}); #drops with Holm<0.10: {sum(o[3] < 0.10 for o in out)}/16; gain range [{min(o[1] for o in out):+.1f},{max(o[1] for o in out):+.1f}]')
    # drop two boats (worst-case pairs)
    print('   drop ANY TWO boats (120 pairs), worst Holm p (family recomputed only for those two contrasts approx: Holm multiplier fixed from baseline rank):')
    for s, a in HARM:
        worst = (0, None)
        d0 = reg(s, 'nominal') - reg(s, a)
        for i in range(16):
            for j in range(i + 1, 16):
                keep = [k for k in range(16) if k not in (i, j)]
                pr = sign_flip_p(d0[keep])
                if pr > worst[0]:
                    worst = (pr, (i, j))
        print(f'   {s}:{a}: worst raw p after dropping two = {worst[0]:.4f} (boats {worst[1]}); x36 Bonferroni = {min(1, 36 * worst[0]):.3f}; x2 (smaller family) = {min(1, 2 * worst[0]):.3f}')
    # (b) cap
    for cap in (60.0, 30.0):
        capped = lambda st, a, c=cap: np.minimum(reg(st, a), c)
        p, g, adj = pvals_family(capped)
        print(f'\n(b) regret capped at +{cap:.0f} s: ' + '; '.join(f'{s}:{a} gain {g[(s, a)]:+.1f} raw p {p[(s, a)]:.4f} Holm {adj[(s, a)]:.4f}' for s, a in HARM))
        print('    all Holm<0.10 under cap:', [f'{k[0]}:{k[1]} {g[k]:+.1f}' for k in adj if adj[k] < 0.10])
    # (c) median & P(>2) & P(>10)
    print('\n(c) alternative summaries, transient stratum:')
    for s, a in HARM:
        n, ar = reg(s, 'nominal'), reg(s, a)
        d = n - ar
        print(f'   {s}:{a}: median regret nominal {np.median(n):.2f} vs arm {np.median(ar):.2f}; median paired diff {np.median(d):+.2f}; exact paired sign test p {exact_sign_test(d):.4f} (arm worse by >0 in {(d < 0).sum()}, better in {(d > 0).sum()}, ties {(d == 0).sum()})')
        for thr in (2.0, 10.0, 30.0):
            pn, pa = (n > thr).mean(), (ar > thr).mean()
            di = (n > thr).astype(float) - (ar > thr).astype(float)
            print(f'      P(regret>{thr:.0f}): nominal {pn:.2f}, arm {pa:.2f}; paired indicator sign-flip p {sign_flip_p(di):.4f}')
    # family-wise Holm for sign test (36) and indicator>2 family
    for lab, fn in (('paired sign test', lambda d: exact_sign_test(d)), ('indicator P(regret>2) sign-flip', None)):
        pp = {}
        for st in STRATA:
            n = reg(st, 'nominal')
            for arm in ARMS:
                if arm == 'nominal':
                    continue
                a = reg(st, arm)
                pp[(st, arm)] = exact_sign_test(n - a) if fn else sign_flip_p((n > 2).astype(float) - (a > 2).astype(float))
        ad = holm(pp)
        print(f'   family of 36, {lab}: ' + '; '.join(f'{s}:{a} p {pp[(s, a)]:.4f} Holm {ad[(s, a)]:.3f}' for s, a in HARM) + ' | Holm<0.10: ' + str([f'{k[0]}:{k[1]}' for k in ad if ad[k] < 0.10]))
    # (d) raw regret
    raw = lambda st, a: reg(st, a, 'regret_raw')
    p, g, adj = pvals_family(raw)
    print('\n(d) RAW (non-cross-fit) regret: ' + '; '.join(f'{s}:{a} gain {g[(s, a)]:+.1f} raw p {p[(s, a)]:.4f} Holm {adj[(s, a)]:.4f}' for s, a in HARM))
    print('    all Holm<0.10 under raw regret:', [f'{k[0]}:{k[1]} {g[k]:+.1f}' for k in adj if adj[k] < 0.10])
    print('    (cross-fit - raw) mean difference per arm in transient: ' + ', '.join(f'{a}:{(reg("transient", a) - reg("transient", a, "regret_raw")).mean():+.2f}' for a in ['nominal', 'timing', 'joint-transient']))
    # (e) also: the pooled joint-vs-nominal claim under same perturbations
    print('\n(e) pooled joint vs nominal (exploratory): ')
    n = np.concatenate([reg(s, 'nominal') for s in STRATA])
    j = np.concatenate([reg(s, 'joint') for s in STRATA])
    nr = np.concatenate([reg(s, 'nominal', 'regret_raw') for s in STRATA])
    jr = np.concatenate([reg(s, 'joint', 'regret_raw') for s in STRATA])
    for lab, d in (('cross-fit', n - j), ('raw', nr - jr), ('capped60', np.minimum(n, 60) - np.minimum(j, 60)), ('indicator>2', (n > 2).astype(float) - (j > 2).astype(float))):
        print(f'   {lab:12} mean gain {d.mean():+7.2f}  sign-flip p {sign_flip_p(d, n_perm=100000):.4f}  better {(d > 0).sum()} worse {(d < 0).sum()} tied {(d == 0).sum()}')
    print('   median regret nominal %.2f joint %.2f; drop each single boat (64): max p %.4f' %
          (np.median(n), np.median(j), max(sign_flip_p(np.delete(n - j, i), n_perm=20000) for i in range(64))))
    dd = n - j
    top = np.argsort(-dd)[:6]
    print('   joint-vs-nominal: gain from top 6 boats = %.1f of total %.1f (sum)' % (dd[top].sum(), dd.sum()))
    # same for transient harm: sum shares
    for s, a in HARM:
        d = reg(s, a) - reg(s, 'nominal')
        top = np.argsort(-d)[:6]
        print(f'   {s}:{a}: top-6 boats carry {d[top].sum():.0f} of {d.sum():.0f} s total excess regret; #boats with excess >10 s: {(d > 10).sum()}')


# ---------------------------------------------------------------------------------------------
def spearman(a, b):
    ra, rb = np.argsort(np.argsort(a)), np.argsort(np.argsort(b))
    if ra.std() == 0 or rb.std() == 0:
        return np.nan
    return float(np.corrcoef(ra, rb)[0, 1])


def q5():
    H('5. H2: trajectory error vs decision regret')
    T = {st: np.array([[r['traj_rmse'][a] for a in ARMS] for r in BY[st]], float) for st in STRATA}   # [B,10]
    G = {st: np.array([[r['scores']['arms'][a]['regret'] for a in ARMS] for r in BY[st]], float) for st in STRATA}
    allT = np.concatenate([T[s] for s in STRATA])
    allG = np.concatenate([G[s] for s in STRATA])
    tm, gm = np.median(allT, 0), allG.mean(0)
    print('Kendall tau pooled (arm median traj vs arm mean regret) = %.3f (score.py: ties dropped); ' % kendall_tau(list(tm), list(gm)))
    print(f'{"arm":16} {"med traj":>9} {"mean regret":>12}  traj rank  regret rank')
    rt, rg = np.argsort(np.argsort(tm)) + 1, np.argsort(np.argsort(gm)) + 1
    for i, a in enumerate(ARMS):
        print(f'{a:16} {tm[i]:9.2f} {gm[i]:12.2f}  {rt[i]:5d}     {rg[i]:5d}')
    print('\nPer-stratum: arm-level (median traj, mean regret) Kendall tau, best-predicting arm (min median traj) vs best-deciding (min mean regret), broad rank')
    for st in STRATA:
        tm, gm = np.median(T[st], 0), G[st].mean(0)
        order_t, order_g = np.argsort(tm), np.argsort(gm)
        rtb = int(np.where(order_t == ARMS.index('broad'))[0][0]) + 1
        rgb = int(np.where(order_g == ARMS.index('broad'))[0][0]) + 1
        print(f'[{st}] tau {kendall_tau(list(tm), list(gm)):+.2f}; best-predicting {ARMS[order_t[0]]} ({tm[order_t[0]]:.2f}), 2nd {ARMS[order_t[1]]} ({tm[order_t[1]]:.2f}); '
              f'best-deciding {ARMS[order_g[0]]} ({gm[order_g[0]]:.2f}), 2nd {ARMS[order_g[1]]} ({gm[order_g[1]]:.2f}); broad traj rank {rtb}/10, regret rank {rgb}/10 (traj {tm[ARMS.index("broad")]:.2f}, regret {gm[ARMS.index("broad")]:.2f})')
        print('     median traj: ' + ' '.join(f'{a[:6]}:{v:.2f}' for a, v in zip(ARMS, tm)))
        print('     mean regret: ' + ' '.join(f'{a[:6]}:{v:.1f}' for a, v in zip(ARMS, gm)))
        # how many boats is each arm best-trajectory / best-decision (min regret, ties split)
        wt = np.bincount(np.argmin(T[st], 1), minlength=10)
        wg = np.array([np.mean(G[st][:, i] <= G[st].min(1) + 1e-9) for i in range(10)])
        print('     per-boat: #boats arm has lowest traj: ' + ' '.join(f'{a[:6]}:{c}' for a, c in zip(ARMS, wt)) + '\n     share of boats arm attains min regret (ties incl.): ' + ' '.join(f'{a[:6]}:{c:.2f}' for a, c in zip(ARMS, wg)))
    print('\nWithin-boat (across the 10 arms) Spearman between traj_rmse and regret; mean over boats [boot 90% CI] (positive = lower traj error goes with lower regret)')
    for st in STRATA + ['all']:
        TT = np.concatenate([T[s] for s in STRATA]) if st == 'all' else T[st]
        GG = np.concatenate([G[s] for s in STRATA]) if st == 'all' else G[st]
        rho = np.array([spearman(t, g) for t, g in zip(TT, GG)])
        ok = rho[~np.isnan(rho)]
        print(f'   {st:10} mean rho {fmt_ci(ok)} (n={len(ok)} boats with variation) share rho>0: {np.mean(ok > 0):.2f}')
    print('\nWithin-arm (across boats) Spearman between traj_rmse and regret, per stratum (n=16 each; rough |rho|>0.5 ~ p<0.05)')
    print(f'{"arm":16}' + ''.join(f'{s:>10}' for s in STRATA) + f'{"all":>10}')
    for i, a in enumerate(ARMS):
        row = [spearman(T[s][:, i], G[s][:, i]) for s in STRATA] + [spearman(allT[:, i], allG[:, i])]
        print(f'{a:16}' + ''.join(f'{v:10.2f}' for v in row))
    print('\nCross-boat: does arm-minus-nominal traj improvement predict arm-minus-nominal regret improvement? Spearman over boats, per arm (pooled 64 boats; negative traj-delta=better)')
    for i, a in enumerate(ARMS):
        if a == 'nominal':
            continue
        dt = allT[:, i] - allT[:, 0]
        dg = allG[:, i] - allG[:, 0]
        print(f'   {a:16} spearman(Δtraj,Δregret) {spearman(dt, dg):+.2f}; mean Δtraj {dt.mean():+.2f} m; mean Δregret {dg.mean():+.1f} s; boats with Δtraj<0 and Δregret>2: {np.sum((dt < 0) & (dg > 2))}; Δtraj>0 and Δregret<-2: {np.sum((dt > 0) & (dg < -2))}')
    # bootstrap tau over boats
    pooled_taus = []
    for _ in range(1000):
        idx = RNG.integers(0, 64, 64)
        pooled_taus.append(kendall_tau(list(np.median(allT[idx], 0)), list(allG[idx].mean(0))))
    print('\nBootstrap (resample 64 boats) Kendall tau pooled: median %.2f, 5-95%% [%.2f, %.2f], P(tau>=0.5)=%.2f' %
          (np.median(pooled_taus), *np.quantile(pooled_taus, [0.05, 0.95]), np.mean(np.array(pooled_taus) >= 0.5)))
    # leave-nominal_d100 out (duplicate of nominal)
    keep = [i for i, a in enumerate(ARMS) if a != 'nominal_d100']
    print('Kendall tau without duplicate arm nominal_d100: %.3f' % kendall_tau(list(np.median(allT, 0)[keep]), list(allG.mean(0)[keep])))
    keep = [i for i, a in enumerate(ARMS) if a not in ('nominal_d100', 'broad')]
    print('Kendall tau without nominal_d100 and broad: %.3f' % kendall_tau(list(np.median(allT, 0)[keep]), list(allG.mean(0)[keep])))
    # traj_rmse excl joint? tau bootstrap w/ ties counted
    # IMPORTANT: timing arms' trajectory error
    print('\nTraj error medians by arm in timing stratum vs nominal (does fitting delay lower trajectory error where delay is large?):')
    for st in STRATA:
        print(f'   {st:10}: ' + ' '.join(f'{a[:8]}:{np.median(T[st][:, i]):.2f}' for i, a in enumerate(ARMS)))


# ---------------------------------------------------------------------------------------------
def power_curve(d, ns, alpha, n_sim=600, n_perm=1000, shrink=0.0, seed=0):
    rng = np.random.default_rng(seed)
    d = np.asarray(d, float)
    mu = d.mean()
    d = d - shrink * mu                                      # shrink: effect scaled by (1-shrink) by shifting the distribution
    out = {}
    for n in ns:
        rej = 0
        for _ in range(n_sim):
            x = d[rng.integers(0, len(d), n)]
            obs = abs(x.mean())
            sg = rng.choice((-1.0, 1.0), size=(n_perm, n))
            p = (1 + np.sum(np.abs((sg * x).mean(1)) >= obs - 1e-12)) / (n_perm + 1)
            rej += (p <= alpha) and (np.sign(x.mean()) == np.sign(mu))
        out[n] = rej / n_sim
    return out


def q6():
    H('6. POWER for confirmatory sign-flip test, alpha = 0.10/6 = 0.0167 two-sided (bootstrap-from-M0b paired differences)')
    alpha = 0.10 / 6
    ns = [8, 12, 16, 24, 32, 40, 48, 64, 80, 96, 128, 160]
    cont = [('timing', 'joint', 'nominal'), ('mixed', 'joint', 'nominal'),
            ('transient', 'timing', 'nominal'), ('transient', 'joint-transient', 'nominal')]
    print('d = regret(nominal) - regret(arm) per boat (positive = arm better). Resampling boats with replacement (n_sim=600, 1000 sign-flip perms). ')
    print('Scenarios: "as observed" (treats M0b mean effect as true: optimistic, winner-curse) and "effect x0.5" (distribution shifted so mean is halved).')
    for st, arm, ref in cont:
        d = reg(st, ref) - reg(st, arm)
        print(f'\n[{st}] {arm} vs {ref}: n=16 mean d {d.mean():+.1f}, sd {d.std(ddof=1):.1f}, median {np.median(d):+.1f}, #pos {(d > 0).sum()} #neg {(d < 0).sum()} #zero {(d == 0).sum()}; observed raw p {sign_flip_p(d):.4f}; |mean|/sd = {abs(d.mean()) / d.std(ddof=1):.2f}')
        for lab, sh in (('as observed', 0.0), ('effect x0.5', 0.5)):
            pc = power_curve(d, ns, alpha, shrink=sh, seed=sum(map(ord, st + arm)) % 1000)
            line = ' '.join(f'n={n}:{p:.2f}' for n, p in pc.items())
            need = next((n for n in ns if pc[n] >= 0.8), None)
            print(f'   {lab:12} {line}   -> smallest n on grid with power>=0.80: {need}')
    # sample-size of a normal-approx check
    print('\nNormal-approximation check (t-test-like) n for 80% power at alpha=0.0167: n ≈ ((z_{1-a/2}+z_{0.8})*sd/mean)^2 with z=2.39+0.84')
    for st, arm, ref in cont:
        d = reg(st, ref) - reg(st, arm)
        print(f'   [{st}] {arm}: n ≈ {((2.39 + 0.84) * d.std(ddof=1) / abs(d.mean())) ** 2:.0f}')


def q7():
    H('7. EXTRA: what makes feed-forward controllers fail (transient) and PI-only controllers fail (timing)')
    ff = np.array([c.feedforward for c in LIBRARY])
    for st in ['transient', 'timing', 'mixed']:
        R = BY[st]
        fail = 1 - np.array([np.mean(np.array(r['hidden_metrics']['docked'], float), axis=1) for r in R])       # [B,16]
        Jm = JH[st].mean(2)
        f_ff, f_pi = fail[:, ff].mean(1), fail[:, ~ff].mean(1)
        print(f'[{st}] mean dock-fail rate FF-on controllers {f_ff.mean():.2f}, PI-only {f_pi.mean():.2f};  mean J FF-on {Jm[:, ff].mean():.0f} vs PI-only {Jm[:, ~ff].mean():.0f}; boats where FF-on fails >0: {(f_ff > 0).sum()}/16, PI-only >0: {(f_pi > 0).sum()}/16')
        for k in ('tau_up', 'tau_down', 'rev_hold', 'dead_time', 'odom_latency'):
            v = np.array([r['spec'][k] for r in R])
            tot = np.array([r['spec']['dead_time'] + r['spec']['odom_latency'] for r in R])
            xs = v if k != 'dead_time' else tot
            c1 = np.corrcoef(xs, f_ff)[0, 1] if f_ff.std() > 0 else np.nan
            c2 = np.corrcoef(xs, f_pi)[0, 1] if f_pi.std() > 0 else np.nan
            print(f'     corr({k if k != "dead_time" else "dead+odom"}, FF-on fail rate) {c1:+.2f}; corr(., PI-only fail rate) {c2:+.2f}')
        dj = Jm[:, ff].mean(1) - Jm[:, ~ff].mean(1)
        print(f'     J(FF-on)-J(PI-only) by boat: quartiles {np.round(np.quantile(dj, [0, .25, .5, .75, 1]), 0)}; #boats FF-on better: {(dj < 0).sum()}')
    st = 'transient'
    R = BY[st]
    print('\n[transient] share of selections that are FF-on controllers: ' + ', '.join(f'{a}: {np.mean([ff[r["scores"]["arms"][a]["sel"]] for r in R]):.2f}' for a in ['nominal', 'timing', 'joint-transient', 'joint', 'broad', 'steady']))
    print('[transient] regret of the arm when it selects FF-on vs PI-only (mean regret, n):')
    for a in ['timing', 'joint-transient', 'joint', 'broad']:
        g = np.array([r['scores']['arms'][a]['regret'] for r in R])
        f = np.array([ff[r['scores']['arms'][a]['sel']] for r in R])
        print(f'    {a:16} FF-on: {g[f].mean() if f.any() else float("nan"):7.1f} (n={f.sum()});  PI-only: {g[~f].mean() if (~f).any() else float("nan"):7.1f} (n={(~f).sum()})')
    print('\n[transient] fitted delay vs tau_down / mean lag: timing arm')
    fd = np.array([r['fits']['timing']['map']['delay_s'] for r in R])
    for k in ('tau_up', 'tau_down'):
        v = np.array([r['spec'][k] for r in R])
        print(f'    corr(fitted delay, {k}) {np.corrcoef(fd, v)[0,1]:+.2f}')
    lag = np.array([(r['spec']['tau_up'] + r['spec']['tau_down']) / 2 for r in R])
    print(f'    corr(fitted delay, mean(tau_up,tau_down)) {np.corrcoef(fd, lag)[0,1]:+.2f}; mean fitted {fd.mean():.2f} vs mean lag {lag.mean():.2f}, mean tau_up {np.mean([r["spec"]["tau_up"] for r in R]):.2f}')
    print('\n[ALL strata] twin-predicted P(c08 fails) vs hidden: timing arm - fitted delay >= 0.35 => twin J[c08] mean:')
    for arm in ['timing', 'joint-transient']:
        fd = np.array([r['fits'][arm]['map']['delay_s'] for r in ROWS])
        pj = np.array([np.mean(np.where(np.isfinite(np.array(r['J_arms'][arm], float)[8]), np.array(r['J_arms'][arm], float)[8], 300)) for r in ROWS])
        hj = np.array([np.mean(r['J_h'][8]) for r in ROWS])
        for lo, hi in ((-1, 0.2), (0.2, 0.4), (0.4, 0.7)):
            m = (fd > lo) & (fd <= hi)
            if m.any():
                print(f'   {arm:16} fitted delay in ({lo},{hi}]: n={m.sum()}, twin J[c08] {pj[m].mean():.0f}, hidden J[c08] {hj[m].mean():.0f}')
    # pooled joint-vs-nominal tie tolerance
    n = np.concatenate([reg(s, 'nominal') for s in STRATA]); j = np.concatenate([reg(s, 'joint') for s in STRATA])
    d = n - j
    for tol in (0.0, 0.5, 1.0, 2.0):
        print(f'   joint vs nominal pooled, tie tolerance {tol}: better {(d > tol).sum()}, worse {(d < -tol).sum()}, tied {(abs(d) <= tol).sum()}')
    print('   exact paired sign test p (ties dropped):', exact_sign_test(d))


if __name__ == '__main__':
    which = sys.argv[2:] or ['0', '1', '2', '3', '4', '5', '6', '7']
    fns = {'0': q0_sanity, '1': q1, '2': q2, '3': q3, '4': q4, '5': q5, '6': q6, '7': q7}
    for w in which:
        fns[w]()
