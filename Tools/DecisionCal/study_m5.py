"""M5 confirmatory driver (PREREG_M5.md): the excess-delay sweep.

For each hidden boat, twins that carry a delay of L + e, where L is the boat's TRUE dead time + odometry
latency (evaluator knowledge: these are ORACLE-DELAY arms, used to map the decision's sensitivity, not as a
calibration method) and e in EXCESS. Two bodies:
  fit:e   the rigid-body particles of the 'rigid+delay' fit (no lag channel), with the delay replaced by L + e;
  true:e  the in-family projection of the hidden boat (true inertia, damping, thrust shape, lag, reversal hold),
          with the delay replaced by L + e.
Reference arms: rigid, rigid+delay (fitted lumped delay), joint.

    python3 -I Tools/DecisionCal/study_m5.py --out DIR
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
from study_oracle_twin import project  # noqa: E402

SEED = 20261115
N_TIMING = 60
EXCESS = (-0.10, -0.05, 0.0, 0.05, 0.10, 0.20)
FIT_ARMS = ('rigid', 'rigid+delay', 'joint')
SWEEP = tuple(f'{b}:{e:+.2f}' for b in ('fit', 'true') for e in EXCESS)
ARMS = FIT_ARMS + SWEEP
LIBS = {'v2': LIBRARY_V2, 'v3': LIBRARY_V3}
CAP, MARGIN, ALPHA, PRIMARY = 60.0, 3.0, 0.10, 'v2'
K, M = 6, 16


def with_delay(spec, dead, lat):
    n = spec.n
    return spec.with_(dead_time=np.full(n, dead), odom_latency=np.full(n, lat))


def split(spec_row, e):
    """Dead time and latency of a twin carrying L + e (never negative): the excess goes on the dead time; a
    deficit larger than the dead time comes off the latency."""
    dead, lat = float(spec_row.dead_time[0]), float(spec_row.odom_latency[0])
    d = dead + e
    if d < 0:
        lat, d = max(lat + d, 0.0), 0.0
    return d, lat


def evaluate(job):
    spec_row, pid, seed = job['spec'], job['id'], job['seed']
    t0 = time.time()
    ev, _ = probes.generate(spec_row, _seed(seed, pid, 'probe'))
    fits, _reg = twin.fit_all(ev, M, _seed(seed, pid, 'fit') % (2 ** 31), FIT_ARMS)
    cur = np.mean([s.meta['current'] for s in ev.segments], axis=0)
    cur_sd = float(np.std([s.meta['current'] for s in ev.segments], axis=0).mean())
    mk = lambda part: twin.to_spec(part, current=tuple(cur), current_sd=cur_sd,
                                   rng=np.random.default_rng(_seed(seed, pid, 'cur') % (2 ** 31)))
    specs = {a: mk(fits[a][0]) for a in FIT_ARMS}
    body_fit = mk(fits['rigid+delay'][0])
    body_true = mk(project(spec_row, 0.0, M)[0])
    for e in EXCESS:
        d, lat = split(spec_row, e)
        specs[f'fit:{e:+.2f}'] = with_delay(body_fit, d, lat)
        specs[f'true:{e:+.2f}'] = with_delay(body_true, d, lat)
    out = {'id': pid, 'stratum': job['stratum'], 'seed': seed, 'spec': spec_row.row(0), 'libs': {},
           'L': float(spec_row.dead_time[0] + spec_row.odom_latency[0]),
           'fits': {a: fits[a][1] for a in FIT_ARMS}}
    for lname, lib in LIBS.items():
        C = len(lib)
        sp, ct, sd = [], [], []
        for arm in ARMS:
            sp.append(specs[arm].repeat(C))
            ct.append(np.tile(np.arange(C), M))
            sd.append(np.repeat([_seed(seed, pid, 'twin', m) for m in range(M)], C))
        sp.append(spec_row.repeat(C * K))
        ct.append(np.repeat(np.arange(C), K))
        sd.append(np.tile([_seed(seed, pid, 'hidden', k) for k in range(K)], C))
        met, _ = mission.run(PlantSpec.concat(sp), np.concatenate(ct), np.concatenate(sd).astype(np.uint64), lib=lib)
        J, off, J_arms = met['J'], 0, {}
        for arm in ARMS:
            J_arms[arm] = J[off:off + M * C].reshape(M, C).T
            off += M * C
        J_h = J[off:].reshape(C, K)
        out['libs'][lname] = {'J_h': J_h.tolist(), 'scores': score.plant_scores(J_h, J_arms, default_arm='rigid'),
                              'J_twin_mean': {a: score.finite_scores(v).mean(1).tolist() for a, v in J_arms.items()},
                              'hidden_docked': met['docked'][off:].reshape(C, K).mean(1).tolist()}
    out['wall_s'] = time.time() - t0
    return out


def _reg(r, lib, arm):
    return min(r['libs'][lib]['scores']['arms'][arm]['regret'], CAP)


def _dock(r, lib, arm):
    return r['libs'][lib]['hidden_docked'][r['libs'][lib]['scores']['arms'][arm]['sel']]


def _stat(d):
    d = np.asarray(d, float)
    idx = np.random.default_rng(0).integers(0, len(d), (4000, len(d)))
    lo, hi = np.quantile(d[idx].mean(1), [0.05, 0.95])
    return {'n': int(len(d)), 'mean': float(d.mean()), 'median': float(np.median(d)), 'ci90': [float(lo), float(hi)],
            'pos': int((d > 2).sum()), 'neg': int((d < -2).sum()), 'p': score.sign_flip_p(d),
            'equivalent': bool(lo > -MARGIN and hi < MARGIN)}


def analyse(rows, lib):
    d = lambda a, b: np.array([_reg(r, lib, a) - _reg(r, lib, b) for r in rows])     # regret(a) - regret(b)
    tests = {'M5-1 cliff: fitted body, regret at e=-0.10 minus e=+0.05 (pred. > 0)': _stat(d('fit:-0.10', 'fit:+0.05')),
             'M5-2 over-estimation: fitted body, regret at e=+0.20 minus e=+0.05 (pred. > 0)': _stat(d('fit:+0.20', 'fit:+0.05')),
             'M5-3 body: fitted minus true body at e=+0.10 (pred. equivalent within 3 s)': _stat(d('fit:+0.10', 'true:+0.10')),
             'M5-4 fitted lumped delay minus fitted body at e=+0.05': _stat(d('rigid+delay', 'fit:+0.05'))}
    adj = score.holm({k: v['p'] for k, v in tests.items()})
    for k in tests:
        tests[k]['p_holm'] = adj[k]
        tests[k]['significant'] = bool(adj[k] < ALPHA)
    curve = {a: {'regret': float(np.mean([_reg(r, lib, a) for r in rows])),
                 'median_regret': float(np.median([_reg(r, lib, a) for r in rows])),
                 'dock': float(np.mean([_dock(r, lib, a) for r in rows])),
                 'p_regret_gt2': float(np.mean([_reg(r, lib, a) > 2 for r in rows]))} for a in ARMS}
    extra = {'body at edge: fitted minus true at e=0': _stat(d('fit:+0.00', 'true:+0.00')),
             'body at e=+0.05': _stat(d('fit:+0.05', 'true:+0.05')),
             'fitted lumped delay minus true body at e=0 (oracle-proj)': _stat(d('rigid+delay', 'true:+0.00')),
             'joint minus true body at e=0': _stat(d('joint', 'true:+0.00')),
             'joint minus fitted lumped delay': _stat(d('joint', 'rigid+delay'))}
    return {'tests': tests, 'curve': curve, 'descriptive': extra}


def summarize(rows):
    L = np.array([r['L'] for r in rows])
    fd = np.array([r['fits']['rigid+delay']['map']['delay_s'] for r in rows])
    return {'n_rows': len(rows), 'families': {lib: analyse(rows, lib) for lib in LIBS},
            'fitted_delay_excess': {'mean': float((fd - L).mean()), 'q10': float(np.quantile(fd - L, 0.1)),
                                    'q90': float(np.quantile(fd - L, 0.9)), 'corr_with_L': float(np.corrcoef(fd, L)[0, 1])}}


def print_summary(s):
    print('fitted lumped delay minus L:', {k: round(v, 3) for k, v in s['fitted_delay_excess'].items()})
    for lib, body in s['families'].items():
        print(f'== library {lib}' + ('  (PRIMARY)' if lib == PRIMARY else '') + f', regret capped at {CAP:.0f} s, n={s["n_rows"]}')
        for k, v in {**body['tests'], **body['descriptive']}.items():
            h = f' holm {v["p_holm"]:.4f}{" *" if v["significant"] else ""}' if 'p_holm' in v else ''
            print(f'  {k[:84]:84s} {v["mean"]:+6.1f} [{v["ci90"][0]:+.1f},{v["ci90"][1]:+.1f}] +/-: {v["pos"]}/{v["neg"]} p {v["p"]:.4f}{h}'
                  + ('  EQUIV' if v['equivalent'] else ''))
        for b in ('fit', 'true'):
            print(f'  {b} body   ' + ' | '.join(f'e={e:+.2f}: {body["curve"][f"{b}:{e:+.2f}"]["regret"]:.1f}/{body["curve"][f"{b}:{e:+.2f}"]["dock"]:.2f}'
                                                for e in EXCESS))
        print('  reference  ' + ' | '.join(f'{a}: {body["curve"][a]["regret"]:.1f}/{body["curve"][a]["dock"]:.2f}' for a in FIT_ARMS))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--seed', type=int, default=SEED)
    ap.add_argument('--n', type=int, default=N_TIMING)
    ap.add_argument('--procs', type=int, default=4)
    a = ap.parse_args(argv)
    if a.procs > 4:
        raise SystemExit('innovation compute policy: at most 4 processes')
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    spec = hidden.sample_hidden(a.n, a.seed, 'timing')
    jobs = [{'spec': spec.take([i]), 'id': f'timing-{i:03d}', 'stratum': 'timing', 'seed': a.seed} for i in range(a.n)]
    done, pj = {}, out / 'plants.jsonl'
    if pj.exists():
        for line in pj.read_text().splitlines():
            r = json.loads(line)
            done[r['id']] = r
    todo = [j for j in jobs if j['id'] not in done]
    (out / 'manifest.json').write_text(json.dumps(record._clean(
        {'argv': vars(a), 'provenance': _paths.provenance(), 'hidden_ranges_sha256': hidden.ranges_sha(), 'arms': ARMS,
         'excess': EXCESS, 'evidence_label': 'HSUR', 'cap_s': CAP, 'started': time.strftime('%Y-%m-%dT%H:%M:%S')}),
        indent=1, sort_keys=True))
    t0 = time.time()
    with get_context('fork').Pool(a.procs) as pool, open(pj, 'a') as fp:
        for k, r in enumerate(pool.imap_unordered(evaluate, todo)):
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
