"""M1 confirmatory driver (see PREREG_M1.md). Differences from study.py (M0b, kept unchanged):

* two fitters on the same probe log: the integral-regression fitter (twin.fit) and the sample-based
  trajectory-matching fitter (abc_fit.fit), arms named 'abc:<arm>';
* two controller libraries (v1 = library.LIBRARY, v2 = library_v2.LIBRARY_V2), each with its own
  hidden-boat ground truth;
* a 'lagsweep' design: the same base boats (steady-stratum draws) re-run at fixed thruster-lag levels,
  for the dose-response test of the delay-absorbs-lag mechanism;
* fixed-controller (no-twin) baselines, computed in score_m1 from the hidden J matrices.

    python3 -I Tools/DecisionCal/study_m1.py --out DIR [--smoke]

Pure numpy; run niced with <= 4 processes (innovation compute policy).
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
import abc_fit  # noqa: E402
import hidden  # noqa: E402
import mission  # noqa: E402
import probes  # noqa: E402
import record  # noqa: E402
import score  # noqa: E402
import score_m1  # noqa: E402
import twin  # noqa: E402
from library import LIBRARY  # noqa: E402
from library_v2 import LIBRARY_V2  # noqa: E402
from plant import PlantSpec  # noqa: E402
from study import _seed, traj_error  # noqa: E402

LIBS = {'v1': LIBRARY, 'v2': LIBRARY_V2}
REG_ARMS = ('nominal', 'broad', 'steady', 'rigid', 'rigid+delay', 'rigid+lag', 'joint',
            'timing', 'joint-transient')              # the last two: M0b definitions, replication only
ABC_ARMS = ('rigid', 'rigid+delay', 'joint')
ARMS = REG_ARMS + tuple('abc:' + a for a in ABC_ARMS)
N_PER = {'steady': 16, 'transient': 24, 'timing': 40, 'mixed': 40}
LAG_LEVELS = (0.0, 0.1, 0.2, 0.3, 0.4, 0.5)        # s, thruster spin-up time constant
LAG_BASE = 10                                       # base boats, each run at every level
SEED = 20261110                                     # new boats: M0/M0b used 20261109, development 4242/777


def lagsweep_specs(seed, n_base=LAG_BASE, levels=LAG_LEVELS):
    """Base boats are steady-stratum draws from a separate stream; only tau_up (and tau_down = 1.2 tau_up)
    is overwritten, so dead time, latency and reversal hold stay small (< 0.05 s)."""
    base = hidden.sample_hidden(n_base, seed + 1, 'steady')
    out = []
    for b in range(n_base):
        for lv in levels:
            row = base.take([b])
            row = row.with_(tau_up=np.array([lv]), tau_down=np.array([1.2 * lv]))
            out.append((row, f'lagsweep-b{b:02d}-t{int(round(lv * 100)):02d}', b, lv))
    return out


def evaluate(job):
    spec_row, pid, stratum, seed, K, M = (job[k] for k in ('spec', 'id', 'stratum', 'seed', 'K', 'M'))
    t0 = time.time()
    ev, _raw = probes.generate(spec_row, _seed(seed, pid, 'probe'))
    fits, reg = twin.fit_all(ev, M, _seed(seed, pid, 'fit') % (2 ** 31), REG_ARMS)
    t_reg = time.time() - t0
    t1 = time.time()
    for i, a in enumerate(ABC_ARMS):
        fits['abc:' + a] = abc_fit.fit(ev, a, M, _seed(seed, pid, 'abc', a) % (2 ** 31), n_prop=job['n_prop'])
    t_abc = time.time() - t1
    cur = np.mean([s.meta['current'] for s in ev.segments], axis=0)
    cur_sd = float(np.std([s.meta['current'] for s in ev.segments], axis=0).mean())
    out = {'id': pid, 'stratum': stratum, 'seed': seed, 'spec': spec_row.row(0), 'libs': {},
           'fits': {a: fits[a][1] for a in ARMS}, 'lag_base': job.get('lag_base'), 'lag_level': job.get('lag_level')}
    t2 = time.time()
    for lname, lib in LIBS.items():
        C = len(lib)
        specs, ctrls, seeds = [], [], []
        for arm in ARMS:
            ts = twin.to_spec(fits[arm][0], current=tuple(cur), current_sd=cur_sd,
                              rng=np.random.default_rng(_seed(seed, pid, 'cur') % (2 ** 31)))
            specs.append(ts.repeat(C))
            ctrls.append(np.tile(np.arange(C), M))
            seeds.append(np.repeat([_seed(seed, pid, 'twin', m) for m in range(M)], C))
        specs.append(spec_row.repeat(C * K))
        ctrls.append(np.repeat(np.arange(C), K))
        seeds.append(np.tile([_seed(seed, pid, 'hidden', k) for k in range(K)], C))
        met, _ = mission.run(PlantSpec.concat(specs), np.concatenate(ctrls), np.concatenate(seeds).astype(np.uint64),
                             lib=lib)
        J = met['J']
        J_arms, off = {}, 0
        for arm in ARMS:
            J_arms[arm] = J[off:off + M * C].reshape(M, C).T
            off += M * C
        J_h = J[off:].reshape(C, K)
        out['libs'][lname] = {'J_h': J_h.tolist(), 'scores': score.plant_scores(J_h, J_arms),
                              'J_twin_mean': {a: score.finite_scores(v).mean(1).tolist() for a, v in J_arms.items()},
                              'hidden_docked': met['docked'][off:].reshape(C, K).mean(1).tolist(),
                              'hidden_contacts': met['contacts'][off:].reshape(C, K).mean(1).tolist()}
    t_mis = time.time() - t2
    out['traj_rmse'] = {arm: traj_error(spec_row, fits[arm][0], _seed(seed, pid, 'traj')) for arm in ARMS}
    out['budget'] = {'probe_s': probes.probe_seconds(), 'segments': len(ev.segments),
                     'evidence_sha': ev.provenance['sha256'], 'particles': M, 'abc_n_prop': job['n_prop'],
                     'reg_wall_s': t_reg, 'abc_wall_s': t_abc, 'mission_wall_s': t_mis}
    return out


def build_jobs(seed, K, M, n_prop, n_per=N_PER, n_base=LAG_BASE, levels=LAG_LEVELS):
    jobs = []
    for st, n in n_per.items():
        spec = hidden.sample_hidden(n, seed, st)
        for i in range(n):
            jobs.append({'spec': spec.take([i]), 'id': f'{st}-{i:03d}', 'stratum': st, 'seed': seed, 'K': K, 'M': M,
                         'n_prop': n_prop})
    for row, pid, b, lv in lagsweep_specs(seed, n_base, levels):
        jobs.append({'spec': row, 'id': pid, 'stratum': 'lagsweep', 'seed': seed, 'K': K, 'M': M, 'n_prop': n_prop,
                     'lag_base': b, 'lag_level': lv})
    return jobs


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--seed', type=int, default=SEED)
    ap.add_argument('--K', type=int, default=6)
    ap.add_argument('--M', type=int, default=16)
    ap.add_argument('--n-prop', type=int, default=512)
    ap.add_argument('--procs', type=int, default=4)
    ap.add_argument('--smoke', action='store_true', help='2 boats per stratum, 2 base boats x 2 lag levels')
    a = ap.parse_args(argv)
    if a.procs > 4:
        raise SystemExit('innovation compute policy: at most 4 processes')
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    if a.smoke:
        jobs = build_jobs(a.seed, a.K, a.M, a.n_prop, {k: 2 for k in N_PER}, 2, (0.0, 0.4))
    else:
        jobs = build_jobs(a.seed, a.K, a.M, a.n_prop)
    done = {}
    pj = out / 'plants.jsonl'
    if pj.exists():                                       # resume: keep finished boats
        for line in pj.read_text().splitlines():
            r = json.loads(line)
            done[r['id']] = r
    todo = [j for j in jobs if j['id'] not in done]
    manifest = {'argv': vars(a), 'provenance': _paths.provenance(), 'hidden_ranges_sha256': hidden.ranges_sha(),
                'arms': ARMS, 'libraries': {k: [c.label() for c in v] for k, v in LIBS.items()},
                'n_per': N_PER, 'lag_levels': LAG_LEVELS, 'lag_base': LAG_BASE, 'evidence_label': 'HSUR',
                'J_weights': mission.J_WEIGHTS, 'started': time.strftime('%Y-%m-%dT%H:%M:%S'), 'n_jobs': len(jobs)}
    (out / 'manifest.json').write_text(json.dumps(record._clean(manifest), indent=1, sort_keys=True))
    t0 = time.time()
    with get_context('fork').Pool(a.procs) as pool, open(pj, 'a') as fp:
        for k, r in enumerate(pool.imap_unordered(evaluate, todo)):
            done[r['id']] = r
            fp.write(json.dumps(record._clean(r), sort_keys=True) + '\n')
            fp.flush()
            s1 = r['libs']['v1']['scores']['arms']
            print(f'[{k + 1}/{len(todo)}] {r["id"]}: v1 ' + ' '.join(
                f'{a}={s1[a]["regret"]:.1f}{"!" if s1[a]["abstained"] else ""}' for a in
                ('nominal', 'rigid', 'rigid+delay', 'joint', 'timing')) + f' ({time.time() - t0:.0f}s)', flush=True)
    rows = [json.loads(json.dumps(record._clean(done[j['id']]))) for j in jobs if j['id'] in done]
    summ = score_m1.summarize(rows)
    summ['elapsed_s'] = time.time() - t0
    (out / 'summary.json').write_text(json.dumps(record._clean(summ), indent=1, sort_keys=True))
    score_m1.print_summary(summ)
    return summ


if __name__ == '__main__':
    main()
