"""ORACLE-TWIN falsification experiment (development seeds only; NOT a calibration result).

Question: is the twin FAMILY structurally optimistic about the hidden boats (H-A: even the best
in-family description mis-selects; extra pessimism rescues), or is the fitted delay simply the
accurate effective closed-loop latency (H-B)?

Arms. Every arm named ORACLE uses the hidden boat's true parameters, i.e. evaluator-only knowledge. They
are diagnostics of the model family, never calibration methods.

  rigid, rigid+delay, joint   twin.fit_all from the probe log (regression fitter), for comparison
  oracle-proj      ORACLE  in-family projection of the hidden boat (see ``project``): one particle replicated M
                           times, delay = true dead_time, odometry latency reproduced exactly
  oracle-proj+0.1  ORACLE  the same with 0.1 s extra dead time
  oracle-proj+0.2  ORACLE  the same with 0.2 s extra dead time
  oracle-full      ORACLE  the hidden boat's own PlantSpec as the twin (twin noise seeds, not the hidden repeats):
                           the ceiling. If even this mis-selects, the residual is run-to-run noise.

Per-arm current draws are shared with the fitted arms (the twin's current estimate from the probe log
plus the same current_sd noise, same rng seed) for the oracle-proj family; oracle-full keeps the
hidden boat's own current, OU, spatial field, wind and gusts.

    cd /home/lunarz/worktrees/decisioncal && nice -n 19 python3 -I -W ignore \
        Tools/DecisionCal/study_oracle_twin.py --out DIR --procs 1 --n 2     # smoke: 2 per stratum
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
from study import _seed  # noqa: E402

SEED = 5151                       # development seed
FORBIDDEN = range(20261109, 20261115)
STRATA = ('timing', 'transient')
LIBS = {'v2': LIBRARY_V2, 'v3': LIBRARY_V3}
FIT_ARMS = ('rigid', 'rigid+delay', 'joint')
EXTRAS = (0.0, 0.1, 0.2)
PROJ_ARMS = tuple('oracle-proj' + ('' if x == 0 else f'+{x:g}') for x in EXTRAS)
ARMS = FIT_ARMS + PROJ_ARMS + ('oracle-full',)
CAP = 60.0
K_HID, M_TWIN = 6, 16
GAINS = (('oracle-proj', 'rigid+delay'), ('oracle-proj+0.2', 'oracle-proj'), ('oracle-proj+0.1', 'oracle-proj'),
         ('oracle-full', 'joint'), ('oracle-full', 'oracle-proj'))      # (a, b): gain of a over b


# ---------------------------------------------------------------------------------------------
# ORACLE mapping hidden -> in-family
# ---------------------------------------------------------------------------------------------

def project(spec_row, extra_delay=0.0, M=M_TWIN):
    """ORACLE. In-family twin particles (dict of [M] arrays for twin.to_spec) and the exact dead-time /
    latency override, for a 1-row hidden PlantSpec.

    Gauge: the twin fixes the thruster scale so that the mean of 0.5 (k_fwd + k_rev) over thrusters
    is K_REF. The hidden mean is Kbar. A force law F = Kbar f_unit is rewritten as K_REF f_unit by
    dividing the whole equation of motion by c = K_REF / Kbar, so every inertia and damping is
    multiplied by c:  m_jj = c (mass + a_jj), m66 = c (izz + a66), d1, d2 -> c d1, c d2.
    Reverse ratio = mean(k_rev) / mean(k_fwd) over thrusters; deadband = mean over thrusters;
    tau_s = (tau_up + tau_down) / 2; rev_hold as is. Delay: dead_time + extra, odometry latency exact
    (applied with ``with_`` after to_spec, so no rounding through odom_frac).
    Dropped, being outside the family: gamma != 2, per-thruster differences, slew limit, OU / spatial
    current, wind and gusts, the hidden boat's sensor noise levels."""
    s = spec_row
    kf, kr = np.asarray(s.k_fwd)[0], np.asarray(s.k_rev)[0]
    kbar = float(np.mean(0.5 * (kf + kr)))
    c = twin.K_REF / kbar
    one = lambda v: np.full(M, float(v))
    p = {'m11': one(c * (s.mass[0] + s.a11[0])), 'm22': one(c * (s.mass[0] + s.a22[0])),
         'm66': one(c * (s.izz[0] + s.a66[0])), 'rev_ratio': one(kr.mean() / kf.mean()),
         'deadband_q': one(np.mean(s.deadband[0])), 'tau_s': one(0.5 * (s.tau_up[0] + s.tau_down[0])),
         'rev_hold_s': one(s.rev_hold[0])}
    for j, ax in enumerate(('u', 'v', 'r')):
        p[f'd1_{ax}'], p[f'd2_{ax}'] = one(c * s.d1[0, j]), one(c * s.d2[0, j])
    dead = float(s.dead_time[0]) + extra_delay
    lat = float(s.odom_latency[0])
    p['delay_s'] = one(dead + lat)
    p['odom_frac'] = one(lat / (dead + lat) if dead + lat > 0 else 0.0)
    return p, {'dead_time': dead, 'odom_latency': lat, 'c': c, 'kbar': kbar}


def oracle_spec(spec_row, extra_delay, cur, cur_sd, rng_seed, M=M_TWIN):
    p, info = project(spec_row, extra_delay, M)
    ts = twin.to_spec(p, current=tuple(cur), current_sd=cur_sd, rng=np.random.default_rng(rng_seed))
    ts = ts.with_(dead_time=np.full(M, info['dead_time']), odom_latency=np.full(M, info['odom_latency']))
    return ts, info


# ---------------------------------------------------------------------------------------------
# one boat
# ---------------------------------------------------------------------------------------------

def evaluate(job):
    spec_row, pid, stratum, seed, K, M = (job[k] for k in ('spec', 'id', 'stratum', 'seed', 'K', 'M'))
    t0 = time.time()
    ev, _raw = probes.generate(spec_row, _seed(seed, pid, 'probe'))
    fits, _reg = twin.fit_all(ev, M, _seed(seed, pid, 'fit') % (2 ** 31), FIT_ARMS)
    cur = np.mean([s.meta['current'] for s in ev.segments], axis=0)
    cur_sd = float(np.std([s.meta['current'] for s in ev.segments], axis=0).mean())
    cur_seed = _seed(seed, pid, 'cur') % (2 ** 31)
    twin_specs, info = {}, None
    for arm in FIT_ARMS:
        twin_specs[arm] = twin.to_spec(fits[arm][0], current=tuple(cur), current_sd=cur_sd,
                                       rng=np.random.default_rng(cur_seed))
    for arm, x in zip(PROJ_ARMS, EXTRAS):
        twin_specs[arm], info = oracle_spec(spec_row, x, cur, cur_sd, cur_seed, M)
    twin_specs['oracle-full'] = spec_row.repeat(M)
    out = {'id': pid, 'stratum': stratum, 'seed': seed, 'spec': spec_row.row(0), 'libs': {},
           'fits': {a: fits[a][1] for a in FIT_ARMS}, 'fitted_delay_map': {a: fits[a][1]['map']['delay_s'] for a in FIT_ARMS},
           'oracle_map': info, 'twin_current': [float(x) for x in cur]}
    t1 = time.time()
    for lname, lib in LIBS.items():
        C = len(lib)
        specs, ctrls, seeds = [], [], []
        for arm in ARMS:
            specs.append(twin_specs[arm].repeat(C))
            ctrls.append(np.tile(np.arange(C), M))
            seeds.append(np.repeat([_seed(seed, pid, 'twin', m) for m in range(M)], C))
        specs.append(spec_row.repeat(C * K))
        ctrls.append(np.repeat(np.arange(C), K))
        seeds.append(np.tile([_seed(seed, pid, 'hidden', k) for k in range(K)], C))
        met, _ = mission.run(_concat(specs), np.concatenate(ctrls), np.concatenate(seeds).astype(np.uint64), lib=lib)
        J, off, J_arms = met['J'], 0, {}
        for arm in ARMS:
            J_arms[arm] = J[off:off + M * C].reshape(M, C).T
            off += M * C
        J_h = J[off:].reshape(C, K)
        sc = score.plant_scores(J_h, J_arms, default_arm='rigid')
        orc = sc['oracle']
        docked = met['docked'][off:].reshape(C, K).mean(1)
        fin = {a: score.finite_scores(v) for a, v in J_arms.items()}
        out['libs'][lname] = {
            'J_h': J_h.tolist(), 'scores': sc, 'hidden_docked': docked.tolist(),
            'J_twin_mean': {a: v.mean(1).tolist() for a, v in fin.items()},
            'pred_J_oracle': {a: float(v[orc].mean()) for a, v in fin.items()},      # twin's prediction for the best controller
            'pred_J_oracle_median': {a: float(np.median(v[orc])) for a, v in fin.items()},
            'hidden_J_oracle': float(score.finite_scores(J_h)[orc].mean())}
    out['budget'] = {'fit_s': t1 - t0, 'mission_s': time.time() - t1, 'particles': M, 'K': K}
    return out


def _concat(specs):
    from plant import PlantSpec
    return PlantSpec.concat(specs)


def build_jobs(seed, n, strata=STRATA):
    jobs = []
    for st in strata:
        spec = hidden.sample_hidden(n, seed, st)
        for i in range(n):
            jobs.append({'spec': spec.take([i]), 'id': f'{st}-s{seed}-n{n}-{i:03d}', 'stratum': st, 'seed': seed,
                         'K': K_HID, 'M': M_TWIN})
    return jobs


# ---------------------------------------------------------------------------------------------
# summary
# ---------------------------------------------------------------------------------------------

def _reg(r, lib, arm, cap):
    x = r['libs'][lib]['scores']['arms'][arm]['regret']
    return min(x, cap) if cap is not None else x


def _dock(r, lib, arm):
    return r['libs'][lib]['hidden_docked'][r['libs'][lib]['scores']['arms'][arm]['sel']]


def boot_ci(d, n_boot=4000, seed=0):
    d = np.asarray(d, float)
    if len(d) == 0:
        return float('nan'), float('nan'), float('nan')
    idx = np.random.default_rng(seed).integers(0, len(d), (n_boot, len(d)))
    lo, hi = np.quantile(d[idx].mean(1), [0.05, 0.95])
    return float(d.mean()), float(lo), float(hi)


def summarize(rows):
    out = {}
    for lib in LIBS:
        for st in STRATA:
            R = [r for r in rows if r['stratum'] == st]
            if not R:
                continue
            d = {'n': len(R), 'arms': {}, 'gains': {}}
            for a in ARMS:
                x = np.array([_reg(r, lib, a, None) for r in R])
                po = np.array([r['libs'][lib]['pred_J_oracle'][a] for r in R])
                ho = np.array([r['libs'][lib]['hidden_J_oracle'] for r in R])
                d['arms'][a] = {
                    'regret': float(x.mean()), 'regret_capped': float(np.minimum(x, CAP).mean()),
                    'dock_rate': float(np.mean([_dock(r, lib, a) for r in R])),
                    'n_distinct': len({r['libs'][lib]['scores']['arms'][a]['sel'] for r in R}),
                    'pred_J_oracle': float(po.mean()), 'hidden_J_oracle': float(ho.mean()),
                    'optimism_J': float((ho - po).mean()), 'optimism_J_median': float(np.median(ho - po)),
                    'frac_pred_below_hidden': float((po < ho - 2.0).mean())}
            for a, b in GAINS:
                for tag, cap in (('capped', CAP), ('uncapped', None)):
                    g = np.array([_reg(r, lib, b, cap) - _reg(r, lib, a, cap) for r in R])
                    m, lo, hi = boot_ci(g)
                    d['gains'][f'{a} over {b} [{tag}]'] = {'mean': m, 'ci90': [lo, hi], 'wins': int((g > 2).sum()),
                                                           'losses': int((g < -2).sum())}
            out[f'{lib}/{st}'] = d
    return out


def print_summary(s):
    for key, d in s.items():
        print(f'== {key}  (n={d["n"]})  ORACLE arms use hidden truth; regret capped at {CAP:.0f} s')
        print(f'  {"arm":16s} {"regret":>7s} {"capped":>7s} {"dock":>5s} {"ndist":>5s} {"predJ*":>7s} {"hidJ*":>7s} {"optim":>7s} {"P(pred<hid-2)":>13s}')
        for a, v in d['arms'].items():
            print(f'  {a:16s} {v["regret"]:7.1f} {v["regret_capped"]:7.1f} {v["dock_rate"]:5.2f} {v["n_distinct"]:5d} '
                  f'{v["pred_J_oracle"]:7.1f} {v["hidden_J_oracle"]:7.1f} {v["optimism_J"]:7.1f} {v["frac_pred_below_hidden"]:13.2f}')
        for k, v in d['gains'].items():
            print(f'  gain {k:48s} {v["mean"]:6.1f} ci90 [{v["ci90"][0]:.1f},{v["ci90"][1]:.1f}] w/l {v["wins"]}/{v["losses"]}')


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--seed', type=int, default=SEED, help='development seed (20261109-20261114 are refused)')
    ap.add_argument('--n', type=int, default=24, help='boats per stratum (timing, transient)')
    ap.add_argument('--procs', type=int, default=1)
    ap.add_argument('--strata', nargs='+', default=list(STRATA), choices=STRATA)
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
    manifest = {'argv': vars(a), 'provenance': _paths.provenance(), 'arms': ARMS, 'evidence_label': 'HSUR-ORACLE-DIAGNOSTIC',
                'libraries': {k: [c.label() for c in v] for k, v in LIBS.items()}, 'extras': EXTRAS,
                'J_weights': mission.J_WEIGHTS, 'started': time.strftime('%Y-%m-%dT%H:%M:%S')}
    (out / 'manifest.json').write_text(json.dumps(record._clean(manifest), indent=1, sort_keys=True))
    t0 = time.time()
    if todo:
        with get_context('fork').Pool(a.procs) as pool, open(pj, 'a') as fp:
            for k, r in enumerate(pool.imap_unordered(evaluate, todo)):
                done[r['id']] = r
                fp.write(json.dumps(record._clean(r), sort_keys=True) + '\n')
                fp.flush()
                s = r['libs']['v2']['scores']['arms']
                print(f'[{k + 1}/{len(todo)}] {r["id"]}: v2 regret ' + ' '.join(f'{x}={s[x]["regret"]:.1f}' for x in ARMS)
                      + f' ({time.time() - t0:.0f}s)', flush=True)
    rows = [json.loads(json.dumps(record._clean(done[j['id']]))) for j in jobs if j['id'] in done]
    summ = summarize(rows)
    (out / 'summary.json').write_text(json.dumps(record._clean(summ), indent=1, sort_keys=True))
    print_summary(summ)
    return summ


if __name__ == '__main__':
    main()
