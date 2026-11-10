"""FACTORIAL pilot: why does a lumped FITTED delay rescue selection when a near-true ASSUMED delay does not?
(development seeds only; exploratory; ORACLE sets use hidden truth, evaluator-only; label HSUR-ORACLE-DIAGNOSTIC)

Hypothesis (error compensation): the fitted delay's upward bias compensates errors elsewhere in the fitted twin.

Twin particle sets, 2 x 4 x 2 = 16, named  <body>|<delay>|<lag>:
  A body   F  the fitted 'rigid' arm's body (m, d1, d2, reverse ratio, deadband; M posterior draws)
           T  the boat's TRUE in-family body (study_oracle_twin.project; one point replicated M times)
  B delay  n  none (dead time = latency = 0)
           t  TRUE total L = dead_time + latency, TRUE split (exact override after to_spec)
           r  TRUE total L, RANDOM split (odom_frac draws of the fitted rigid+delay arm, as fitted arms do)
           f  FITTED lumped delay (the rigid+delay arm's delay_s draws, with its odom_frac draws)
  C lag    0  tau = 0, reversal hold = 0 (as in the rigid arms)
           L  TRUE mean lag (tau_up+tau_down)/2 and TRUE reversal hold
Extras (spread, candidate iv):  x:Fmed|t|0, x:Fmed|t|L (body collapsed to its per-parameter particle median),
  x:Fmed|fmed|0 (that body, delay collapsed to the fitted median, random split kept).
References exactly as returned: rigid+delay, joint (twin.fit_all), and oracle-proj (study_oracle_twin.oracle_spec).
Common random numbers: all sets share the twin noise seeds and the current draws; hidden repeats are shared.
All sets of a boat are flown in ONE mission.run call per library; scoring is score.plant_scores (argmin of the mean
over M particles, cross-fit regret; capped at 60 s in the analysis) and dock rate of the pick on the hidden repeats.

    cd /home/lunarz/worktrees/decisioncal && nice -n 19 python3 -I -W ignore \
        Tools/DecisionCal/study_factorial.py --out DIR --procs 4 [--n 24] [--seed 5252]
    ... study_factorial.py --out DIR --n 1 --smoke        # 2 boats (1 per stratum)
"""

import argparse
import itertools
import json
import sys
import time
from multiprocessing import get_context
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import _paths  # noqa: E402
import hidden  # noqa: E402
import mission  # noqa: E402
import probes  # noqa: E402
import record  # noqa: E402
import score  # noqa: E402
import twin  # noqa: E402
from plant import PlantSpec  # noqa: E402
from study import _seed  # noqa: E402
from study_oracle_twin import FIT_ARMS, K_HID, LIBS, M_TWIN, STRATA, CAP, boot_ci, project  # noqa: E402

SEED = 5252                       # development seed
FORBIDDEN = range(20261109, 20261115)
BODIES, DELAYS, LAGS = ('F', 'T'), ('n', 't', 'r', 'f'), ('0', 'L')
FACT = tuple(f'{a}|{b}|{c}' for a, b, c in itertools.product(BODIES, DELAYS, LAGS))
EXTRA = ('x:Fmed|t|0', 'x:Fmed|t|L', 'x:Fmed|fmed|0')
REFS = ('rigid+delay', 'joint', 'oracle-proj')
SETS = FACT + EXTRA + REFS
BODY_KEYS = ('m11', 'm22', 'm66', 'd1_u', 'd1_v', 'd1_r', 'd2_u', 'd2_v', 'd2_r', 'rev_ratio', 'deadband_q')


# ---------------------------------------------------------------------------------------------
# particle sets
# ---------------------------------------------------------------------------------------------

def build_sets(spec_row, fits, M=M_TWIN):
    """(particle dicts for twin.to_spec, exact dead-time/latency overrides or None) for every factorial / extra set.
    ORACLE: uses the hidden row for the T body, true delay, true lag."""
    truth, info = project(spec_row, 0.0, M)
    L = info['dead_time'] + info['odom_latency']
    frac_true = info['odom_latency'] / L if L > 0 else 0.0
    rg, rd = fits['rigid'][0], fits['rigid+delay'][0]
    med = lambda p: {k: np.full(M, float(np.median(p[k]))) for k in BODY_KEYS}
    bodies = {'F': {k: np.asarray(rg[k], float) for k in BODY_KEYS}, 'T': {k: truth[k] for k in BODY_KEYS},
              'Fmed': med(rg)}
    lag = {'0': (np.zeros(M), np.zeros(M)), 'L': (truth['tau_s'], truth['rev_hold_s'])}
    rfrac = np.asarray(rd['odom_frac'], float)
    delays = {'n': (np.zeros(M), np.zeros(M), None),
              't': (np.full(M, L), np.full(M, frac_true), (info['dead_time'], info['odom_latency'])),
              'r': (np.full(M, L), rfrac, None),
              'f': (np.asarray(rd['delay_s'], float), rfrac, None),
              'fmed': (np.full(M, float(np.median(rd['delay_s']))), rfrac, None)}
    out = {}

    def make(body, dl, lg):
        d, fr, ov = delays[dl]
        p = {**bodies[body], 'delay_s': d.copy(), 'odom_frac': fr.copy(), 'tau_s': lag[lg][0].copy(),
             'rev_hold_s': lag[lg][1].copy()}
        return p, ov
    for a, b, c in itertools.product(BODIES, DELAYS, LAGS):
        out[f'{a}|{b}|{c}'] = make(a, b, c)
    out['x:Fmed|t|0'], out['x:Fmed|t|L'], out['x:Fmed|fmed|0'] = make('Fmed', 't', '0'), make('Fmed', 't', 'L'), make('Fmed', 'fmed', '0')
    return out, info


def misfit(spec_row, fits, M=M_TWIN):
    """ORACLE. Fitted 'rigid' body (particle median) over the true in-family body, per parameter, plus summary indices.
    inertia_idx / damping_idx = mean |log ratio| over the three axes (damping: d1 and d2 pooled)."""
    truth, _ = project(spec_row, 0.0, M)
    rg = fits['rigid'][0]
    ratio = {k: float(np.median(np.asarray(rg[k], float)) / max(truth[k][0], 1e-9)) for k in BODY_KEYS}
    lg = lambda ks: float(np.mean([abs(np.log(max(ratio[k], 1e-9))) for k in ks]))
    return {'ratio': ratio, 'inertia_idx': lg(('m11', 'm22', 'm66')),
            'damping_idx': lg(('d1_u', 'd1_v', 'd1_r', 'd2_u', 'd2_v', 'd2_r')),
            'inertia_logmean': float(np.mean([np.log(ratio[k]) for k in ('m11', 'm22', 'm66')])),
            'damping_logmean': float(np.mean([np.log(max(ratio[k], 1e-9)) for k in ('d1_u', 'd1_v', 'd1_r', 'd2_u', 'd2_v', 'd2_r')]))}


def to_specs(sets, cur, cur_sd, cur_seed, M=M_TWIN):
    specs = {}
    for name, (p, ov) in sets.items():
        ts = twin.to_spec(p, current=tuple(cur), current_sd=cur_sd, rng=np.random.default_rng(cur_seed))
        if ov is not None:
            ts = ts.with_(dead_time=np.full(M, ov[0]), odom_latency=np.full(M, ov[1]))
        specs[name] = ts
    return specs


# ---------------------------------------------------------------------------------------------
# one boat
# ---------------------------------------------------------------------------------------------

def evaluate(job):
    from study_oracle_twin import oracle_spec
    spec_row, pid, stratum, seed, K, M = (job[k] for k in ('spec', 'id', 'stratum', 'seed', 'K', 'M'))
    t0 = time.time()
    ev, _raw = probes.generate(spec_row, _seed(seed, pid, 'probe'))
    fits, _reg = twin.fit_all(ev, M, _seed(seed, pid, 'fit') % (2 ** 31), FIT_ARMS)
    cur = np.mean([s.meta['current'] for s in ev.segments], axis=0)
    cur_sd = float(np.std([s.meta['current'] for s in ev.segments], axis=0).mean())
    cur_seed = _seed(seed, pid, 'cur') % (2 ** 31)
    sets, info = build_sets(spec_row, fits, M)
    twin_specs = to_specs(sets, cur, cur_sd, cur_seed, M)
    for a in ('rigid+delay', 'joint'):
        twin_specs[a] = twin.to_spec(fits[a][0], current=tuple(cur), current_sd=cur_sd, rng=np.random.default_rng(cur_seed))
    twin_specs['oracle-proj'], _ = oracle_spec(spec_row, 0.0, cur, cur_sd, cur_seed, M)
    out = {'id': pid, 'stratum': stratum, 'seed': seed, 'spec': spec_row.row(0), 'libs': {},
           'misfit': misfit(spec_row, fits, M), 'oracle_map': info,
           'fitted_delay_map': {a: fits[a][1]['map']['delay_s'] for a in FIT_ARMS},
           'fitted_delay_median': float(np.median(fits['rigid+delay'][0]['delay_s'])),
           'true_L': info['dead_time'] + info['odom_latency'],
           'rigid_nominal_timing': [float(fits['rigid'][0][k][0]) for k in ('delay_s', 'tau_s', 'rev_hold_s')]}
    t1 = time.time()
    for lname, lib in LIBS.items():
        C = len(lib)
        specs, ctrls, seeds = [], [], []
        for name in SETS:
            specs.append(twin_specs[name].repeat(C))
            ctrls.append(np.tile(np.arange(C), M))
            seeds.append(np.repeat([_seed(seed, pid, 'twin', m) for m in range(M)], C))
        specs.append(spec_row.repeat(C * K))
        ctrls.append(np.repeat(np.arange(C), K))
        seeds.append(np.tile([_seed(seed, pid, 'hidden', k) for k in range(K)], C))
        met, _ = mission.run(PlantSpec.concat(specs), np.concatenate(ctrls), np.concatenate(seeds).astype(np.uint64), lib=lib)
        J, off, J_arms = met['J'], 0, {}
        for name in SETS:
            J_arms[name] = J[off:off + M * C].reshape(M, C).T
            off += M * C
        J_h = J[off:].reshape(C, K)
        sc = score.plant_scores(J_h, J_arms, default_arm='rigid+delay')
        docked = met['docked'][off:].reshape(C, K).mean(1)
        arms = {a: {'sel': v['sel'], 'regret': v['regret'], 'regret_raw': v['regret_raw'], 'pred_J': v['pred_J'],
                    'hidden_J': v['hidden_J'], 'dock': float(docked[v['sel']])} for a, v in sc['arms'].items()}
        out['libs'][lname] = {'arms': arms, 'oracle': sc['oracle'], 'J_oracle': sc['J_oracle'], 'hidden_docked': docked.tolist(),
                              'J_h': J_h.tolist()}
    out['budget'] = {'fit_s': t1 - t0, 'mission_s': time.time() - t1, 'particles': M, 'K': K, 'sets': len(SETS)}
    return out


def build_jobs(seed, n, strata=STRATA):
    jobs = []
    for st in strata:
        spec = hidden.sample_hidden(n, seed, st)
        for i in range(n):
            jobs.append({'spec': spec.take([i]), 'id': f'fact-{st}-s{seed}-n{n}-{i:03d}', 'stratum': st, 'seed': seed,
                         'K': K_HID, 'M': M_TWIN})
    return jobs


# ---------------------------------------------------------------------------------------------
# analysis
# ---------------------------------------------------------------------------------------------

def reg(r, lib, s):
    return min(r['libs'][lib]['arms'][s]['regret'], CAP)


def dock(r, lib, s):
    return r['libs'][lib]['arms'][s]['dock']


def _ci(x, seed=0):
    m, lo, hi = boot_ci(x, 4000, seed)
    return {'mean': m, 'lo': lo, 'hi': hi}


def contrasts(R, lib):
    """Each contrast is per-boat; positive = the first named set is WORSE (higher regret) than the second, i.e. the
    second (treatment) helps: gain(base -> treat) = regret(base) - regret(treat)."""
    g = lambda base, treat: np.array([reg(r, lib, base) - reg(r, lib, treat) for r in R])
    c = {}
    cells = [(a, l) for a in BODIES for l in LAGS]
    # main effects (averaged over the other two factors)
    mean_over = lambda f, items: sum(f(*it) for it in items) / len(items)
    c['main A: true body over fitted body'] = mean_over(lambda b, d, l: g(f'F|{d}|{l}', f'T|{d}|{l}'), [(0, d, l) for d in DELAYS for l in LAGS])
    c['main C: true lag over no lag'] = mean_over(lambda a, d, l: g(f'{a}|{d}|0', f'{a}|{d}|L'), [(a, d, 0) for a in BODIES for d in DELAYS])
    for d in ('t', 'r', 'f'):
        c[f'main B: delay {d} over none'] = mean_over(lambda a, l: g(f'{a}|n|{l}', f'{a}|{d}|{l}'), cells)
    c['main B: fitted delay over true delay (t)'] = mean_over(lambda a, l: g(f'{a}|t|{l}', f'{a}|f|{l}'), cells)
    c['main B: true split (t) over random split (r)'] = mean_over(lambda a, l: g(f'{a}|r|{l}', f'{a}|t|{l}'), cells)
    # key questions
    c['Q1a: T body, no lag: true delay over none'] = g('T|n|0', 'T|t|0')
    c['Q1b: T body, true delay: true lag over none (lag still needed?)'] = g('T|t|0', 'T|t|L')
    c['Q1c: true delay, no lag: T body over F body'] = g('F|t|0', 'T|t|0')
    c['Q1d: F body: true delay over none'] = g('F|n|0', 'F|t|0')
    c['Q2a: F body + true delay: true lag over none'] = g('F|t|0', 'F|t|L')
    c['Q2b: F body + true delay + true lag vs oracle-proj (T|t|L): oracle better by'] = g('F|t|L', 'T|t|L')
    c['Q2c: F body + true lag, true delay over none'] = g('F|n|L', 'F|t|L')
    for a, l in cells:
        c[f'Q3: true split over random split [{a}|.|{l}]'] = g(f'{a}|r|{l}', f'{a}|t|{l}')
    for a, l in cells:
        c[f'Q4: fitted delay over true delay [{a}|.|{l}]'] = g(f'{a}|t|{l}', f'{a}|f|{l}')
    c['Q4 interaction A: (F - T) of fitted-over-true delay, no lag'] = g('F|t|0', 'F|f|0') - g('T|t|0', 'T|f|0')
    c['Q4 interaction A: (F - T) of fitted-over-true delay, true lag'] = g('F|t|L', 'F|f|L') - g('T|t|L', 'T|f|L')
    c['Q4 interaction C: (no lag - lag) of fitted-over-true delay, F body'] = g('F|t|0', 'F|f|0') - g('F|t|L', 'F|f|L')
    c['Q4 interaction C: (no lag - lag) of fitted-over-true delay, T body'] = g('T|t|0', 'T|f|0') - g('T|t|L', 'T|f|L')
    c['Q4 three-way: A x C of fitted-over-true delay'] = (g('F|t|0', 'F|f|0') - g('T|t|0', 'T|f|0')) - (g('F|t|L', 'F|f|L') - g('T|t|L', 'T|f|L'))
    # spread (iv)
    c['Q5a: F spread vs F median body (true delay, no lag): median body over spread'] = g('F|t|0', 'x:Fmed|t|0')
    c['Q5b: same with true lag'] = g('F|t|L', 'x:Fmed|t|L')
    c['Q5c: F|f|0 -> median body AND median fitted delay (spread removed)'] = g('F|f|0', 'x:Fmed|fmed|0')
    c['ref: rigid+delay minus F|f|0 (body from rigid+delay arm own fit vs rigid arm body)'] = g('F|f|0', 'rigid+delay')
    c['ref: oracle-proj minus T|t|L (should be ~0)'] = g('oracle-proj', 'T|t|L')
    return c


def summarize(rows):
    out = {}
    for lib in LIBS:
        for st in STRATA + ('both',):
            R = [r for r in rows if st == 'both' or r['stratum'] == st]
            if not R:
                continue
            d = {'n': len(R), 'sets': {}, 'contrasts': {}}
            for s in SETS:
                x = np.array([reg(r, lib, s) for r in R])
                xu = np.array([r['libs'][lib]['arms'][s]['regret'] for r in R])
                d['sets'][s] = {'capped': float(x.mean()), 'uncapped': float(xu.mean()), 'ci90': [_ci(x)['lo'], _ci(x)['hi']], 'dock': float(np.mean([dock(r, lib, s) for r in R])),
                                'n_distinct': len({r['libs'][lib]['arms'][s]['sel'] for r in R})}
            for k, v in contrasts(R, lib).items():
                ci = _ci(v)
                ci.update({'wins': int((v > 2).sum()), 'losses': int((v < -2).sum())})
                d['contrasts'][k] = ci
            out[f'{lib}/{st}'] = d
    out['misfit'] = misfit_summary(rows)
    return out


def misfit_summary(rows):
    res = {}
    for st in STRATA + ('both',):
        R = [r for r in rows if st == 'both' or r['stratum'] == st]
        if not R:
            continue
        mf = lambda k: np.array([r['misfit']['ratio'][k] for r in R])
        d = {'n': len(R), 'ratio_median_[q10,q90]': {}, 'frac_off_by_20pct': {}}
        for k in BODY_KEYS:
            v = mf(k)
            d['ratio_median_[q10,q90]'][k] = [float(np.median(v)), float(np.quantile(v, 0.1)), float(np.quantile(v, 0.9))]
            d['frac_off_by_20pct'][k] = float((np.abs(np.log(np.maximum(v, 1e-9))) > np.log(1.2)).mean())
        d['inertia_idx_mean'] = float(np.mean([r['misfit']['inertia_idx'] for r in R]))
        d['damping_idx_mean'] = float(np.mean([r['misfit']['damping_idx'] for r in R]))
        d['inertia_logmean'] = float(np.mean([r['misfit']['inertia_logmean'] for r in R]))
        d['damping_logmean'] = float(np.mean([r['misfit']['damping_logmean'] for r in R]))
        d['fitted_delay_median_minus_trueL'] = float(np.mean([r['fitted_delay_median'] - r['true_L'] for r in R]))
        d['corr'] = {}
        for lib in LIBS:
            for s in ('F|n|0', 'F|t|0', 'F|f|0', 'F|t|L', 'T|t|0', 'rigid+delay'):
                reg_s = [reg(r, lib, s) for r in R]
                for idx in ('inertia_idx', 'damping_idx'):
                    xs = [r['misfit'][idx] for r in R]
                    d['corr'][f'{lib} {s} regret ~ {idx} (spearman)'] = score.spearman(xs, reg_s) if len(R) > 2 else float('nan')
            # misfit tertiles for the key sets
            idx = np.array([r['misfit']['inertia_idx'] + r['misfit']['damping_idx'] for r in R])
            hi = idx >= np.median(idx)
            for s in ('F|t|0', 'F|f|0', 'F|t|L', 'T|t|0'):
                v = np.array([reg(r, lib, s) for r in R])
                d['corr'][f'{lib} {s} mean regret: low-misfit half / high-misfit half'] = [float(v[~hi].mean()) if (~hi).any() else float('nan'),
                                                                                         float(v[hi].mean()) if hi.any() else float('nan')]
        res[st] = d
    return res


def print_summary(s):
    for key, d in s.items():
        if key == 'misfit':
            continue
        print(f'\n== {key}  (n={d["n"]})  capped regret (s) / dock rate of the pick; ORACLE sets use hidden truth')
        print(f'  {"set":16s} {"capped":>7s} {"[90% CI]":>16s} {"uncap":>7s} {"dock":>5s} {"ndist":>5s}')
        for k, v in d['sets'].items():
            print(f'  {k:16s} {v["capped"]:7.1f} [{v["ci90"][0]:6.1f},{v["ci90"][1]:6.1f}] {v["uncapped"]:7.1f} {v["dock"]:5.2f} {v["n_distinct"]:5d}')
        print('  contrasts (positive = treatment lowers regret), mean [90% CI], boats better/worse by >2 s')
        for k, v in d['contrasts'].items():
            print(f'  {k:90s} {v["mean"]:6.1f} [{v["lo"]:6.1f},{v["hi"]:6.1f}] {v["wins"]}/{v["losses"]}')
    print('\n== rigid-body misfit (fitted rigid arm, particle median / true in-family value)')
    for st, d in s['misfit'].items():
        print(f' {st} n={d["n"]}: inertia |log ratio| {d["inertia_idx_mean"]:.2f}, damping {d["damping_idx_mean"]:.2f}; '
              f'mean log ratio inertia {d["inertia_logmean"]:+.2f}, damping {d["damping_logmean"]:+.2f}; '
              f'fitted delay median minus true L {d["fitted_delay_median_minus_trueL"]:+.3f} s')
        for k, v in d['ratio_median_[q10,q90]'].items():
            print(f'    {k:12s} median {v[0]:5.2f} [q10 {v[1]:5.2f}, q90 {v[2]:5.2f}]  frac >20% off {d["frac_off_by_20pct"][k]:.2f}')
        for k, v in d['corr'].items():
            print(f'    {k}: {v if isinstance(v, float) else [round(x, 1) for x in v]}')


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--seed', type=int, default=SEED, help='development seed (20261109-20261114 are refused)')
    ap.add_argument('--n', type=int, default=24, help='boats per stratum (timing, transient)')
    ap.add_argument('--procs', type=int, default=1)
    ap.add_argument('--strata', nargs='+', default=list(STRATA), choices=STRATA)
    ap.add_argument('--smoke', action='store_true', help='label only; use with --n 1')
    a = ap.parse_args(argv)
    if not 1 <= a.procs <= 4:
        raise SystemExit('innovation compute policy: 1..4 processes')
    if a.seed in FORBIDDEN:
        raise SystemExit('seeds 20261109-20261114 are confirmatory; this is a development experiment')
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    jobs = build_jobs(a.seed, a.n, a.strata)
    done = {}
    pj = out / 'plants.jsonl'
    if pj.exists():
        for line in pj.read_text().splitlines():
            r = json.loads(line)
            done[r['id']] = r
    todo = [j for j in jobs if j['id'] not in done]
    manifest = {'argv': vars(a), 'provenance': _paths.provenance(), 'sets': SETS, 'evidence_label': 'HSUR-ORACLE-DIAGNOSTIC',
                'libraries': {k: [c.label() for c in v] for k, v in LIBS.items()}, 'J_weights': mission.J_WEIGHTS,
                'started': time.strftime('%Y-%m-%dT%H:%M:%S')}
    (out / 'manifest.json').write_text(json.dumps(record._clean(manifest), indent=1, sort_keys=True))
    t0 = time.time()
    if todo:
        with get_context('fork').Pool(a.procs) as pool, open(pj, 'a') as fp:
            for k, r in enumerate(pool.imap_unordered(evaluate, todo)):
                done[r['id']] = r
                fp.write(json.dumps(record._clean(r), sort_keys=True) + '\n')
                fp.flush()
                s = r['libs']['v2']['arms']
                print(f'[{k + 1}/{len(todo)}] {r["id"]}: v2 regret F|n|0={s["F|n|0"]["regret"]:.1f} F|t|0={s["F|t|0"]["regret"]:.1f} '
                      f'F|f|0={s["F|f|0"]["regret"]:.1f} F|t|L={s["F|t|L"]["regret"]:.1f} T|t|L={s["T|t|L"]["regret"]:.1f} '
                      f'oracle-proj={s["oracle-proj"]["regret"]:.1f} ({time.time() - t0:.0f}s, boat {r["budget"]["fit_s"] + r["budget"]["mission_s"]:.0f}s)', flush=True)
    rows = [json.loads(json.dumps(record._clean(done[j['id']]))) for j in jobs if j['id'] in done]
    summ = summarize(rows)
    (out / 'summary.json').write_text(json.dumps(record._clean(summ), indent=1, sort_keys=True))
    print_summary(summ)
    return summ


if __name__ == '__main__':
    main()
