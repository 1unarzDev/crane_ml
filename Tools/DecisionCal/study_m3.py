"""M3 control driver (PREREG_M3.md): is it FITTING the delay that helps, or assuming any delay?

Reuses study_m1.evaluate with a reduced arm set (regression fitter only) and libraries v2 (frozen) and
v3 (random draws). Boats: timing 60, transient 120, seed 20261112.

    python3 -I Tools/DecisionCal/study_m2.py --out DIR
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
import record  # noqa: E402
import score  # noqa: E402
import study_m1  # noqa: E402
from library_v2 import LIBRARY_V2  # noqa: E402
from library_v3 import LIBRARY_V3  # noqa: E402

SEED = 20261113
N_PER = {'timing': 40, 'transient': 40}
ARMS = ('rigid', 'rigid+delay', 'rigid+lag', 'rigid@d0.16', 'rigid@d0.25', 'rigid@d0.35')
LIBS = {'v2': LIBRARY_V2, 'v3': LIBRARY_V3}
CAP = 60.0
ALPHA = 0.10
PRIMARY = 'v2'

# study_m1.evaluate reads these module globals (fork keeps them in the workers)
study_m1.REG_ARMS, study_m1.ABC_ARMS, study_m1.ARMS, study_m1.LIBS = ARMS, (), ARMS, LIBS


def _reg(r, lib, arm, cap=CAP):
    x = r['libs'][lib]['scores']['arms'][arm]['regret']
    return min(x, cap) if cap is not None else x


def two_sample_perm_p(a, b, n_perm=20000, seed=0):
    """Two-sided permutation p for mean(a) - mean(b) (labels permuted)."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    z = np.concatenate([a, b])
    obs = abs(a.mean() - b.mean())
    rng = np.random.default_rng(seed)
    cnt = 0
    for _ in range(n_perm):
        p = rng.permutation(len(z))
        cnt += abs(z[p[:len(a)]].mean() - z[p[len(a):]].mean()) >= obs - 1e-12
    return (cnt + 1) / (n_perm + 1)


def _stat(d):
    d = np.asarray(d, float)
    idx = np.random.default_rng(0).integers(0, len(d), (4000, len(d)))
    return {'n': int(len(d)), 'mean_gain': float(d.mean()), 'median_gain': float(np.median(d)),
            'ci90': [float(x) for x in np.quantile(d[idx].mean(1), [0.05, 0.95])],
            'wins': int((d > 2).sum()), 'losses': int((d < -2).sum()), 'p': score.sign_flip_p(d)}


def analyse(rows, lib, cap=CAP):
    S = lambda st: [r for r in rows if r['stratum'] == st]
    g = lambda R, a, b: np.array([_reg(r, lib, b, cap) - _reg(r, lib, a, cap) for r in R])   # gain of a over b
    T, Tr = S('timing'), S('transient')
    tests = {'M3-1 timing: fitted delay (rigid+delay) over assumed 0.25 s (rigid@d0.25)': _stat(g(T, 'rigid+delay', 'rigid@d0.25')),
             'M3-2 transient: fitted lag (rigid+lag) over assumed 0.25 s delay (rigid@d0.25)': _stat(g(Tr, 'rigid+lag', 'rigid@d0.25'))}
    adj = score.holm({k: v['p'] for k, v in tests.items()})
    for k in tests:
        tests[k]['p_holm'] = adj[k]
        tests[k]['significant'] = bool(adj[k] < ALPHA)
    desc = {}
    for st, R in (('timing', T), ('transient', Tr)):
        desc[st] = {}
        for a in ARMS:
            x = np.array([_reg(r, lib, a, None) for r in R])
            sel = [r['libs'][lib]['scores']['arms'][a]['sel'] for r in R]
            dock = np.array([r['libs'][lib]['hidden_docked'][s] for r, s in zip(R, sel)])
            desc[st][a] = {'mean_regret': float(x.mean()), 'mean_regret_capped': float(np.minimum(x, CAP).mean()),
                           'median_regret': float(np.median(x)), 'p_regret_gt2': float((x > 2).mean()),
                           'dock_rate_of_selected': float(dock.mean()), 'n_distinct_selected': len(set(sel))}
        for gname in ('rigid@d0.16', 'rigid@d0.25', 'rigid@d0.35'):
            desc[st][gname + ' over rigid'] = _stat(g(R, gname, 'rigid'))
            desc[st]['fitted (rigid+delay) over ' + gname] = _stat(g(R, 'rigid+delay', gname))
        orc = [r['libs'][lib]['scores']['oracle'] for r in R]
        desc[st]['oracle_top_share'] = max(orc.count(c) for c in set(orc)) / len(orc)
    return {'tests': tests, 'descriptive': desc}


def summarize(rows):
    return {'n_rows': len(rows), 'cap_s': CAP,
            'families': {lib: analyse(rows, lib) for lib in LIBS},
            'uncapped': {lib: analyse(rows, lib, None)['tests'] for lib in LIBS}}


def print_summary(s):
    for lib, body in s['families'].items():
        print(f'== library {lib}' + ('  (PRIMARY)' if lib == PRIMARY else '') + f', regret capped at {s["cap_s"]:.0f} s')
        for k, v in body['tests'].items():
            u = s['uncapped'][lib][k]
            print(f'  {k:66s} n={v["n"]:3d} gain {v["mean_gain"]:6.1f} ci90 [{v["ci90"][0]:.1f},{v["ci90"][1]:.1f}] '
                  f'w/l {v["wins"]}/{v["losses"]} p {v["p"]:.4f} holm {v["p_holm"]:.4f}{" *" if v["significant"] else ""}'
                  f'  | uncapped {u["mean_gain"]:.1f} p {u["p"]:.4f}')
        for st, d in body['descriptive'].items():
            print(f'  {st}: ' + ' '.join(f'{a}={d[a]["mean_regret"]:.1f}/{d[a]["p_regret_gt2"]:.2f}/{d[a]["dock_rate_of_selected"]:.2f}'
                                         for a in ARMS) + f'  (mean regret / P(>2 s) / dock rate; oracle top share {d["oracle_top_share"]:.2f})')


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--seed', type=int, default=SEED)
    ap.add_argument('--procs', type=int, default=4)
    ap.add_argument('--smoke', action='store_true')
    a = ap.parse_args(argv)
    if a.procs > 4:
        raise SystemExit('innovation compute policy: at most 4 processes')
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    n_per = {k: 3 for k in N_PER} if a.smoke else N_PER
    jobs = []
    for st, n in n_per.items():
        spec = hidden.sample_hidden(n, a.seed, st)
        for i in range(n):
            jobs.append({'spec': spec.take([i]), 'id': f'{st}-{i:03d}', 'stratum': st, 'seed': a.seed, 'K': 6, 'M': 16,
                         'n_prop': 0})
    done = {}
    pj = out / 'plants.jsonl'
    if pj.exists():
        for line in pj.read_text().splitlines():
            r = json.loads(line)
            done[r['id']] = r
    todo = [j for j in jobs if j['id'] not in done]
    manifest = {'argv': vars(a), 'provenance': _paths.provenance(), 'hidden_ranges_sha256': hidden.ranges_sha(),
                'arms': ARMS, 'libraries': {k: [c.label() for c in v] for k, v in LIBS.items()}, 'n_per': n_per,
                'evidence_label': 'HSUR', 'J_weights': mission.J_WEIGHTS, 'cap_s': CAP,
                'started': time.strftime('%Y-%m-%dT%H:%M:%S')}
    (out / 'manifest.json').write_text(json.dumps(record._clean(manifest), indent=1, sort_keys=True))
    t0 = time.time()
    with get_context('fork').Pool(a.procs) as pool, open(pj, 'a') as fp:
        for k, r in enumerate(pool.imap_unordered(study_m1.evaluate, todo)):
            done[r['id']] = r
            fp.write(json.dumps(record._clean(r), sort_keys=True) + '\n')
            fp.flush()
            if (k + 1) % 10 == 0 or k + 1 == len(todo):
                print(f'[{k + 1}/{len(todo)}] ({time.time() - t0:.0f}s)', flush=True)
    rows = [json.loads(json.dumps(record._clean(done[j['id']]))) for j in jobs if j['id'] in done]
    summ = summarize(rows)
    summ['elapsed_s'] = time.time() - t0
    (out / 'summary.json').write_text(json.dumps(record._clean(summ), indent=1, sort_keys=True))
    print_summary(summ)


if __name__ == '__main__':
    main()
