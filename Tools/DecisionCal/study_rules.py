"""Method-exploration pilot: risk-averse decision rules over deliberately widened twin posteriors.

Question: can a risk-averse rule over a widened posterior (rigid-body fit + a broad timing PRIOR,
no timing calibration) match or beat the fitted-delay twin, and does it cost anything on easy boats?

Arms (particle sets; all share one probe log and one rigid-body regression):
  rigid             twin.fit 'rigid', M=16, as is
  rigid+delay       twin.fit 'rigid+delay', M=16 (reference)
  joint             twin.fit 'joint', M=16 (reference)
  rigid*timing      the rigid fit (M=32) with delay_s ~ U[0,0.6], tau_s ~ U[0,0.5], rev_hold_s ~ U[0,0.5],
                    odom_frac ~ U[0,1] drawn per particle (independent of everything the log says)
  broad             twin.fit 'broad', M=32
Rules, all applied to the SAME flown particle scores J[c, m] (no extra simulation):
  mean | cvar25 (mean of the worst ceil(0.25 M) particles) | worst (max) | mean+sd | dockfirst
  (maximise the fraction of particles with J < 180, ties by mean).
Per (arm, rule, library): selection, cross-fit regret, capped (60 s) regret, dock rate of the selection on
the hidden boat. Full per-particle J matrices are stored so other rules can be tried offline.

    cd REPO && nice -n 19 python3 -I -W ignore Tools/DecisionCal/study_rules.py --out DIR [--smoke] [--procs 1]
    ... --analyse-only --out DIR      # re-summarise a finished/partial run

Pure numpy; at most 4 processes. Refuses the confirmatory seeds 20261109-20261114.
"""

import argparse
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
from library_v2 import LIBRARY_V2  # noqa: E402
from library_v3 import LIBRARY_V3  # noqa: E402
from plant import PlantSpec  # noqa: E402
from study import _seed  # noqa: E402

SEED = 6262
STRATA = ('steady', 'timing', 'transient')
FORBIDDEN = range(20261109, 20261115)
LIBS = {'v2': LIBRARY_V2, 'v3': LIBRARY_V3}
ARMS = ('rigid', 'rigid+delay', 'joint', 'rigid*timing', 'broad')
M_OF = {'rigid': 16, 'rigid+delay': 16, 'joint': 16, 'rigid*timing': 32, 'broad': 32}
RULES = ('mean', 'cvar25', 'worst', 'mean+sd', 'dockfirst')
RISK_RULES = RULES[1:]
PRIOR = {'delay_s': (0.0, 0.6), 'tau_s': (0.0, 0.5), 'rev_hold_s': (0.0, 0.5), 'odom_frac': (0.0, 1.0)}
CAP = 60.0
J_DOCK = score.J_ABSTAIN          # J < 180 means the mission docked
K = 6
BASE = ('rigid', 'mean')


# ---------------------------------------------------------------------------------------------
# decision rules
# ---------------------------------------------------------------------------------------------

def rule_scores(J, rule):
    """J [C, M] (non-finite -> J_FAIL). Returns a [C] array, lower is better, for every rule."""
    J = score.finite_scores(J)
    M = J.shape[1]
    if rule == 'mean':
        return J.mean(1)
    if rule == 'cvar25':
        k = max(1, int(np.ceil(0.25 * M)))
        return np.sort(J, axis=1)[:, -k:].mean(1)
    if rule == 'worst':
        return J.max(1)
    if rule == 'mean+sd':
        return J.mean(1) + J.std(1)
    if rule == 'dockfirst':
        # lexicographic (docking fraction desc, mean asc) folded into one number: the fraction is a multiple
        # of 1/M and the mean is below J_FAIL * 2, so a weight of 1e4 per particle separates the levels.
        frac_fail = (J >= J_DOCK).sum(1)
        return frac_fail * 1e4 + J.mean(1)
    raise ValueError(rule)


def select(J, rule):
    return int(np.argmin(rule_scores(J, rule)))      # ties: lowest index


# ---------------------------------------------------------------------------------------------
# particles
# ---------------------------------------------------------------------------------------------

def timing_prior_particles(base, seed):
    """Rigid-body particles with delay, tau, rev_hold, odom_frac replaced by independent prior draws."""
    rng = np.random.default_rng(seed)
    p = {k: np.array(v, float) for k, v in base.items()}
    n = len(p['m11'])
    for k, (lo, hi) in PRIOR.items():
        p[k] = rng.uniform(lo, hi, n)
    return p


def fit_arms(ev, seed_base, pid):
    reg = twin.Regression(ev)
    fits = {}
    fseed = lambda *x: _seed(seed_base, pid, 'fit', *x) % (2 ** 31)
    for arm in ('rigid', 'rigid+delay', 'joint'):
        fits[arm] = twin.fit(reg, arm, M_OF[arm], fseed(arm))
    rig32, rep = twin.fit(reg, 'rigid', M_OF['rigid*timing'], fseed('rigid32'))
    fits['rigid*timing'] = (timing_prior_particles(rig32, fseed('prior')), {'arm': 'rigid*timing', 'base': rep})
    fits['broad'] = twin.fit(reg, 'broad', M_OF['broad'], fseed('broad'))
    return fits


# ---------------------------------------------------------------------------------------------
# one boat
# ---------------------------------------------------------------------------------------------

def evaluate(job):
    spec_row, pid, stratum, seed = job['spec'], job['id'], job['stratum'], job['seed']
    t0 = time.time()
    ev, _raw = probes.generate(spec_row, _seed(seed, pid, 'probe'))
    fits = fit_arms(ev, seed, pid)
    t_fit = time.time() - t0
    cur = np.mean([s.meta['current'] for s in ev.segments], axis=0)
    cur_sd = float(np.std([s.meta['current'] for s in ev.segments], axis=0).mean())
    out = {'id': pid, 'stratum': stratum, 'seed': seed, 'spec': spec_row.row(0), 'libs': {},
           'fits': {a: fits[a][1] for a in ARMS}}
    t1 = time.time()
    for lname, lib in LIBS.items():
        C = len(lib)
        specs, ctrls, seeds = [], [], []
        for arm in ARMS:
            M = M_OF[arm]
            ts = twin.to_spec(fits[arm][0], current=tuple(cur), current_sd=cur_sd,
                              rng=np.random.default_rng(_seed(seed, pid, 'cur') % (2 ** 31)))
            specs.append(ts.repeat(C))
            ctrls.append(np.tile(np.arange(C), M))
            seeds.append(np.repeat([_seed(seed, pid, 'twin', m) for m in range(M)], C))   # common noise across arms
        specs.append(spec_row.repeat(C * K))
        ctrls.append(np.repeat(np.arange(C), K))
        seeds.append(np.tile([_seed(seed, pid, 'hidden', k) for k in range(K)], C))
        met, _ = mission.run(PlantSpec.concat(specs), np.concatenate(ctrls), np.concatenate(seeds).astype(np.uint64),
                             lib=lib)
        Jall = met['J']
        J_arms, off = {}, 0
        for arm in ARMS:
            M = M_OF[arm]
            J_arms[arm] = score.finite_scores(Jall[off:off + M * C].reshape(M, C).T)
            off += M * C
        J_h = score.finite_scores(Jall[off:].reshape(C, K))
        docked = met['docked'][off:].reshape(C, K).mean(1)
        res = {}
        for arm in ARMS:
            for rule in RULES:
                s = select(J_arms[arm], rule)
                reg = score.cross_fit_regret(J_h, s)
                res[f'{arm}|{rule}'] = {'sel': s, 'regret': reg, 'regret_cap': min(reg, CAP),
                                        'dock': float(docked[s])}
        out['libs'][lname] = {'J_h': np.round(J_h, 4).tolist(), 'hidden_docked': docked.tolist(),
                              'J_arms': {a: np.round(v, 4).tolist() for a, v in J_arms.items()},   # [C, M]
                              'oracle': int(np.argmin(J_h.mean(1))), 'res': res}
    out['wall_s'] = {'fit': t_fit, 'mission': time.time() - t1}
    return out


# ---------------------------------------------------------------------------------------------
# analysis
# ---------------------------------------------------------------------------------------------

def recompute(row, lib, arm, rule):
    """Selection/regret/dock for any (arm, rule) from the stored matrices (offline rules)."""
    L = row['libs'][lib]
    J = np.asarray(L['J_arms'][arm], float)
    s = select(J, rule)
    reg = score.cross_fit_regret(np.asarray(L['J_h'], float), s)
    return {'sel': s, 'regret': reg, 'regret_cap': min(reg, CAP), 'dock': float(L['hidden_docked'][s])}


def _vec(rows, lib, arm, rule, field='regret_cap'):
    return np.array([r['libs'][lib]['res'][f'{arm}|{rule}'][field] for r in rows], float)


def boot_ci(d, n_boot=4000, seed=0):
    d = np.asarray(d, float)
    if len(d) == 0:
        return float('nan'), float('nan'), float('nan')
    idx = np.random.default_rng(seed).integers(0, len(d), (n_boot, len(d)))
    lo, hi = np.quantile(d[idx].mean(1), [0.05, 0.95])
    return float(d.mean()), float(lo), float(hi)


def paired(rows, lib, a, b):
    """Gain of option a over option b (positive = a better): capped regret difference, dock-rate difference."""
    g = _vec(rows, lib, *b) - _vec(rows, lib, *a)
    dk = _vec(rows, lib, *a, field='dock') - _vec(rows, lib, *b, field='dock')
    return g, dk


def summarize(rows):
    out = {'n_rows': len(rows), 'cap_s': CAP, 'table': {}, 'gains': {}, 'steady_cost': {}}
    for st in list(STRATA) + ['all']:
        R = [r for r in rows if st == 'all' or r['stratum'] == st]
        if not R:
            continue
        for lib in LIBS:
            out['table'][f'{st}|{lib}'] = {
                'n': len(R), **{f'{a}|{ru}': {'regret_cap': float(_vec(R, lib, a, ru).mean()),
                                              'dock': float(_vec(R, lib, a, ru, 'dock').mean())}
                                for a in ARMS for ru in RULES}}
            comps = {'(rigid*timing,cvar25) vs (rigid+delay,mean)': (('rigid*timing', 'cvar25'), ('rigid+delay', 'mean')),
                     '(rigid*timing,cvar25) vs (rigid,mean)': (('rigid*timing', 'cvar25'), ('rigid', 'mean'))}
            for name, (a, b) in comps.items():
                g, dk = paired(R, lib, a, b)
                out['gains'][f'{st}|{lib}|{name}'] = {'n': len(R), 'regret_gain': boot_ci(g), 'dock_gain': boot_ci(dk, seed=1)}
            if st == 'steady':
                for a in ARMS:
                    for ru in RISK_RULES:
                        g, dk = paired(R, lib, (a, ru), BASE)
                        out['steady_cost'][f'{lib}|{a}|{ru}'] = {'n': len(R), 'regret_gain': boot_ci(g),
                                                                  'dock_gain': boot_ci(dk, seed=1)}
    return out


def _ci(t):
    return f'{t[0]:+6.1f} [{t[1]:+.1f}, {t[2]:+.1f}]'


def print_summary(s):
    print(f'== rules pilot, n_rows={s["n_rows"]}; cells = mean capped(60 s) regret / dock rate of selection on the hidden boat')
    for key, t in s['table'].items():
        st, lib = key.split('|')
        print(f'\n-- {st} / {lib} (n={t["n"]})')
        print(f'{"arm":14s}' + ''.join(f'{r:>14s}' for r in RULES))
        for a in ARMS:
            print(f'{a:14s}' + ''.join(f'{t[f"{a}|{r}"]["regret_cap"]:8.1f}/{t[f"{a}|{r}"]["dock"]:.2f} ' for r in RULES))
    print('\n== paired gains (positive = first option better), mean [90% bootstrap CI]; regret in s, dock in rate')
    for key, g in s['gains'].items():
        print(f'{key:78s} n={g["n"]:3d} regret {_ci(g["regret_gain"])}  dock {g["dock_gain"][0]:+.2f} '
              f'[{g["dock_gain"][1]:+.2f}, {g["dock_gain"][2]:+.2f}]')
    print('\n== steady boats: gain of each risk-averse (arm, rule) over (rigid, mean); negative = cost')
    for key, g in s['steady_cost'].items():
        print(f'{key:34s} n={g["n"]:3d} regret {_ci(g["regret_gain"])}  dock {g["dock_gain"][0]:+.2f} '
              f'[{g["dock_gain"][1]:+.2f}, {g["dock_gain"][2]:+.2f}]')


# ---------------------------------------------------------------------------------------------
# driver
# ---------------------------------------------------------------------------------------------

def build_jobs(seed, n_per, strata=STRATA):
    jobs = []
    for st in strata:
        spec = hidden.sample_hidden(n_per, seed, st)       # NB: draws depend on n_per, so n_per is part of the run key
        for i in range(n_per):
            jobs.append({'spec': spec.take([i]), 'id': f'{st}-{i:03d}', 'stratum': st, 'seed': seed})
    return jobs


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--seed', type=int, default=SEED, help='development seed (default 6262)')
    ap.add_argument('--n-per', type=int, default=20, help='boats per stratum (steady, timing, transient)')
    ap.add_argument('--procs', type=int, default=4)
    ap.add_argument('--smoke', action='store_true', help='1 boat per stratum (3 boats), overrides --n-per')
    ap.add_argument('--max-boats', type=int, default=0, help='stop after this many NEW boats (0 = all)')
    ap.add_argument('--analyse-only', action='store_true')
    a = ap.parse_args(argv)
    if not 1 <= a.procs <= 4:
        raise SystemExit('innovation compute policy: 1-4 processes')
    if a.seed in FORBIDDEN:
        raise SystemExit(f'seed {a.seed} is reserved for confirmatory runs')
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    n_per = 1 if a.smoke else a.n_per
    pj, mj = out / 'plants.jsonl', out / 'manifest.json'
    key = {'seed': a.seed, 'n_per': n_per, 'strata': STRATA, 'arms': ARMS, 'M_OF': M_OF, 'K': K}
    if mj.exists():
        old = json.loads(mj.read_text()).get('key')
        if old is not None and json.loads(json.dumps(key)) != old and not a.analyse_only:
            raise SystemExit(f'{mj} was written for a different (seed, n_per, arms): use a new --out')
    done = {}
    if pj.exists():
        for line in pj.read_text().splitlines():
            r = json.loads(line)
            done[r['id']] = r
    jobs = build_jobs(a.seed, n_per)
    todo = [j for j in jobs if j['id'] not in done]
    if a.max_boats:
        todo = todo[:a.max_boats]
    t0 = time.time()
    if not a.analyse_only:
        manifest = {'key': key, 'argv': vars(a), 'provenance': _paths.provenance(), 'hidden_ranges_sha256': hidden.ranges_sha(),
                    'libraries': {k: [c.label() for c in v] for k, v in LIBS.items()}, 'rules': RULES, 'prior': PRIOR,
                    'evidence_label': 'HSUR', 'J_weights': mission.J_WEIGHTS, 'cap_s': CAP,
                    'started': time.strftime('%Y-%m-%dT%H:%M:%S')}
        mj.write_text(json.dumps(record._clean(manifest), indent=1, sort_keys=True))
        with open(pj, 'a') as fp:
            if a.procs == 1:
                it = map(evaluate, todo)
                pool = None
            else:
                pool = get_context('fork').Pool(a.procs)
                it = pool.imap_unordered(evaluate, todo)
            for k, r in enumerate(it):
                done[r['id']] = r
                fp.write(json.dumps(record._clean(r), sort_keys=True) + '\n')
                fp.flush()
                print(f'[{k + 1}/{len(todo)}] {r["id"]} fit {r["wall_s"]["fit"]:.0f}s mission {r["wall_s"]["mission"]:.0f}s '
                      f'({time.time() - t0:.0f}s)', flush=True)
            if pool is not None:
                pool.close()
                pool.join()
    ids = {j['id'] for j in jobs}
    rows = [json.loads(json.dumps(record._clean(done[i]))) for i in sorted(done) if i in ids]
    summ = summarize(rows)
    summ['elapsed_s'] = time.time() - t0
    (out / 'summary.json').write_text(json.dumps(record._clean(summ), indent=1, sort_keys=True))
    print_summary(summ)
    return summ


if __name__ == '__main__':
    main()
