"""M0/M1 study driver: hidden boats -> shared probe log -> six twin arms -> controller selection ->
regret on the hidden boat.

    python3 -I Tools/DecisionCal/study.py --per-stratum 12 --K 6 --M 16 --procs 4 --out DIR

Writes DIR/plants.jsonl (one row per hidden boat: J matrices, fits, scores), DIR/runs.jsonl
(dc-run/1 rows of every hidden-boat mission, label HSUR), DIR/summary.json (per-stratum table and
the kill verdict) and DIR/manifest.json (code/nlc hashes, frozen ranges sha, arguments).
Run it niced; it is pure numpy (innovation compute policy: <= 4 processes, < 2 GB).
"""

import argparse
import hashlib
import json
import os
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
from library import LIBRARY  # noqa: E402
from plant import DT, Plant, PlantSpec  # noqa: E402

C = len(LIBRARY)


def _seed(*parts):
    h = hashlib.sha256('/'.join(str(p) for p in parts).encode()).digest()
    return int.from_bytes(h[:7], 'little')


def traj_error(spec_h, particles, seed, n_part=8):
    """Open-loop prediction error of an arm on a fresh probe schedule (H2's 'mean trajectory
    error'): RMS over segments of the position error 8 s after each pulse starts, using the first
    ``n_part`` particles, against the hidden boat's truth on the same commands."""
    proto = probes.PROTOCOL
    S = len(proto)
    ticks = int(round(probes.SEG_S / DT))
    tt = np.arange(ticks) * DT
    Q = np.stack([probes.schedule(ph, tt) for _, _, ph in proto], 1)
    hs = spec_h.repeat(S)
    hp = Plant(hs, np.array([_seed(seed, 'val', i) for i in range(S)], np.uint64), t_max=probes.SEG_S)
    sub = {k: np.asarray(v)[:n_part] for k, v in particles.items()}
    ts = twin.to_spec(sub, current=tuple(np.asarray(spec_h.current)[0]))
    ts = ts.repeat(S)                                                # particle-major, then segment
    tp = Plant(ts, np.arange(ts.n, dtype=np.uint64), t_max=probes.SEG_S)
    Qt = np.broadcast_to(Q[:, None], (ticks, n_part, S, 4)).reshape(ticks, -1, 4)     # env p*S + s
    k_eval = int(round((probes.BASELINE_S + 8.0) / DT))
    k0 = int(round(probes.BASELINE_S / DT))
    for k in range(k_eval):
        hp.step(Q[k])
        tp.step(Qt[k])
        if k == k0 - 1:
            h0, t0 = hp.st[:, :2].copy(), tp.st[:, :2].copy()
    dh = hp.st[:, :2] - h0
    dt_ = (tp.st[:, :2] - t0).reshape(S, n_part, 2)
    err = np.linalg.norm(dt_ - dh[:, None, :], axis=-1)
    return float(np.sqrt((err ** 2).mean()))


def evaluate_plant(job):
    spec_row, pid, stratum, seed, K, M, arms = (job[k] for k in ('spec', 'id', 'stratum', 'seed', 'K', 'M', 'arms'))
    t0 = time.time()
    ev, _raw = probes.generate(spec_row, _seed(seed, pid, 'probe'))
    t_probe = time.time() - t0
    t1 = time.time()
    fits, reg = twin.fit_all(ev, M, _seed(seed, pid, 'fit') % (2 ** 31), arms)
    t_fit = time.time() - t1
    cur = np.mean([s.meta['current'] for s in ev.segments], axis=0)
    cur_sd = float(np.std([s.meta['current'] for s in ev.segments], axis=0).mean())
    # one batch: every arm's M particles x C controllers, then the hidden boat C x K
    specs, ctrls, seeds = [], [], []
    for a_i, arm in enumerate(arms):
        part, _ = fits[arm]
        ts = twin.to_spec(part, current=tuple(cur), current_sd=cur_sd,             # same draws for every arm (CRN)
                          rng=np.random.default_rng(_seed(seed, pid, 'cur') % (2 ** 31)))
        specs.append(ts.repeat(C))                                   # particle-major
        ctrls.append(np.tile(np.arange(C), M))
        seeds.append(np.repeat([_seed(seed, pid, 'twin', m) for m in range(M)], C))
    specs.append(spec_row.repeat(C * K))
    ctrls.append(np.repeat(np.arange(C), K))                         # controller-major
    seeds.append(np.tile([_seed(seed, pid, 'hidden', k) for k in range(K)], C))   # CRN across controllers
    spec_all = PlantSpec.concat(specs)
    ctrl_all = np.concatenate(ctrls)
    seed_all = np.concatenate(seeds).astype(np.uint64)
    t2 = time.time()
    met, _ = mission.run(spec_all, ctrl_all, seed_all)
    t_mission = time.time() - t2
    J = met['J']
    J_arms, off = {}, 0
    for arm in arms:
        J_arms[arm] = J[off:off + M * C].reshape(M, C).T
        off += M * C
    J_h = J[off:].reshape(C, K)
    hidden_met = {k: v[off:].reshape(C, K) for k, v in met.items()}
    sc = score.plant_scores(J_h, J_arms)
    traj = {arm: traj_error(spec_row, fits[arm][0], _seed(seed, pid, 'traj')) for arm in arms}
    return {'id': pid, 'stratum': stratum, 'seed': seed, 'spec': spec_row.row(0), 'J_h': J_h.tolist(),
            'J_arms': {a: v.tolist() for a, v in J_arms.items()}, 'scores': sc, 'traj_rmse': traj,
            'fits': {a: fits[a][1] for a in arms}, 'current_est': cur.tolist(), 'current_sd_est': cur_sd,
            'hidden_metrics': {k: v.tolist() for k, v in hidden_met.items()},
            'budget': {'probe_s': probes.probe_seconds(), 'segments': len(ev.segments), 'evidence_sha': ev.provenance['sha256'],
                       'particles': M, 'rows': reg.n_rows, 'fit_wall_s': t_fit, 'probe_wall_s': t_probe,
                       'mission_wall_s': t_mission}}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--per-stratum', type=int, default=12)
    ap.add_argument('--strata', default=','.join(hidden.STRATA))
    ap.add_argument('--K', type=int, default=6)
    ap.add_argument('--M', type=int, default=16)
    ap.add_argument('--seed', type=int, default=20261109)
    ap.add_argument('--procs', type=int, default=4)
    ap.add_argument('--arms', default=','.join(twin.ARMS))
    ap.add_argument('--out', required=True)
    ap.add_argument('--study', default='decisioncal-M0b')
    a = ap.parse_args(argv)
    if a.procs > 4:
        raise SystemExit('innovation compute policy: at most 4 processes')
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    arms = tuple(a.arms.split(','))
    if 'nominal' not in arms:
        raise SystemExit('the nominal arm is the regret reference')
    jobs = []
    for st in a.strata.split(','):
        spec = hidden.sample_hidden(a.per_stratum, a.seed, st)
        for i in range(a.per_stratum):
            jobs.append({'spec': spec.take([i]), 'id': f'{st}-{i:03d}', 'stratum': st, 'seed': a.seed, 'K': a.K,
                         'M': a.M, 'arms': arms})
    manifest = {'argv': vars(a), 'provenance': _paths.provenance(), 'hidden_ranges_sha256': hidden.ranges_sha(),
                'hidden_frozen': hidden.FROZEN, 'library': [c.label() for c in LIBRARY],
                'J_weights': mission.J_WEIGHTS, 'evidence_label': 'HSUR', 'started': time.strftime('%Y-%m-%dT%H:%M:%S')}
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=1, sort_keys=True))
    rows = []
    t0 = time.time()
    ctx = get_context('fork')
    with ctx.Pool(a.procs) as pool, open(out / 'plants.jsonl', 'w') as fp, open(out / 'runs.jsonl', 'w') as fr:
        for k, r in enumerate(pool.imap(evaluate_plant, jobs)):
            rows.append(r)
            fp.write(json.dumps(record._clean(r), sort_keys=True) + '\n')
            fp.flush()
            for c in range(C):
                for rep in range(a.K):
                    hm = {m: r['hidden_metrics'][m][c][rep] for m in r['hidden_metrics']}
                    record.write(fr, {
                        'schema': record.SCHEMA, 'run_id': f'{r["id"]}/c{c:02d}/k{rep}', 'study': a.study,
                        'evidence_label': 'HSUR', 'seed': rep,
                        'plant': {'id': r['id'], 'class': 'hidden', 'stratum': r['stratum'], 'split': 'eval'},
                        'controller': {'id': LIBRARY[c].label(), 'spec_sha': LIBRARY[c].sha()},
                        'termination': 'DOCKED' if hm['docked'] else 'TIMEOUT', 'metrics': hm,
                        'obs_quality': {'odom_latency_s': r['spec']['odom_latency']}, 'interventions': [],
                        'budget': r['budget'], 'code': manifest['provenance']})
            el = time.time() - t0
            print(f'[{k + 1}/{len(jobs)}] {r["id"]}: oracle c{r["scores"]["oracle"]:02d} band {r["scores"]["band"]:.1f} '
                  + ' '.join(f'{arm}={r["scores"]["arms"][arm]["regret"]:.1f}' for arm in arms)
                  + f'  ({el:.0f}s)', flush=True)
    summ = score.summarize(rows, arms)
    summ['traj_rmse_median'] = {arm: float(np.nanmedian([np.nan if r['traj_rmse'][arm] is None else r['traj_rmse'][arm]
                                                         for r in rows])) for arm in arms}
    summ['elapsed_s'] = time.time() - t0
    (out / 'summary.json').write_text(json.dumps(record._clean(summ), indent=1, sort_keys=True))
    print(json.dumps(record._clean(summ['kill']), indent=1))
    h = summ['hypotheses']
    print('H1 supported:', h['H1']['supported'], 'wins', h['H1']['wins'], 'harms', h['H1']['harms'])
    print('H2 supported:', h['H2']['supported'], 'tau', round(h['H2']['kendall_tau'], 2))
    for st, tab in summ['table'].items():
        print(f'== {st} (n={tab["n"]})')
        for arm in arms:
            t = tab[arm]
            print(f'  {arm:10s} median {t["median_regret"]:7.2f} mean {t["mean_regret"]:7.2f} '
                  f'ci90 [{t["mean_regret_ci90"][0]:.1f},{t["mean_regret_ci90"][1]:.1f}] '
                  f'p_band {t["p_within_band"]:.2f} gain_vs_nom {t["gain_vs_nominal"]:6.2f} '
                  f'cov {t["coverage"]:.2f} rho {t["rank_corr_median"]:.2f}')
    return summ


if __name__ == '__main__':
    main()
