"""Falsifier: is the delay rescue "pessimism" or "calibration accuracy"? (probe-rate pilot, development seeds)

The standard probe log (GPS 5 Hz, velocity from a +-1 s local quadratic fit, heading 10 Hz, gyro box-smoothed)
makes the fitted lumped delay run ~0.15 s above the hidden boat's dead time + odometry latency, and that
over-estimate is what rescues controller selection (RESULTS_M2.md, REVIEW_m2_fresh.md). Here the probe log is
measured better (higher GPS rate, optionally a narrower SG window, optionally less position noise) while
everything else is held fixed: same hidden boats, same commands, same plant noise (common random numbers: a
faster GPS samples a superset of the 5 Hz ticks), same fitter arms, same libraries, same twin missions.

    pessimism, not accuracy:  better measurement -> smaller fitted delay -> WORSE selection on delay boats
    accuracy matters:         better measurement -> better or equal selection

Variants (probes.generate copied into ``generate_variant``; nothing else in the harness is edited):

    V0  standard           GPS 0.20 s (5 Hz),   SG half 1.0 s
    V1  fast GPS           GPS 0.04 s (25 Hz),  SG half 1.0 s
    V2  fast GPS, narrow   GPS 0.04 s (25 Hz),  SG half 0.3 s
    V3  fast GPS, quiet    as V1 with the probe-time position noise x0.25 (the hidden boat the missions
                           use is unchanged; only the logged GPS is quieter)

Why 25 Hz and not 20 Hz: the plant ticks at DT = 0.02 s and every sampler (probes.generate) fires on whole
ticks; 0.05 s is 2.5 ticks and would silently round to 0.04 s (as the standard gyro's GYRO_DT = 0.05 already
does: the "20 Hz" gyro in probes.py is really 25 Hz). So V1-V3 use 0.04 s explicitly.

What twin.Regression assumes (checked against twin.py):
  * the segment grid dt is a whole number of ticks (GRID_DT = 0.1 -> 5 ticks; kept, and asserted);
  * ``_cum_thrust`` rebuilds the measurement operator from seg.meta['gps_t'] / ['gyro_t'] and calls the
    module-level ``twin.sg_velocity(tg, eye, s.t)`` with the DEFAULT half window of 1.0 s, hard-coded; a
    different window in the generator would therefore be a mismatched operator. ``matched_operator(half)``
    rebinds ``twin.sg_velocity`` to ``functools.partial(sg_velocity, half=half)`` for the duration of a
    Regression build (the regressors are cached in the object, so the patch is not needed afterwards).
    twin.py is not edited; the rebinding is process-local and restored on exit.
  * the gyro box (np.ones(5)) is hard-coded in ``_cum_thrust``; the generator therefore keeps GYRO_BOX = 5
    (asserted) and the standard gyro rate in every variant.
  * the heading (10 Hz, interpolated onto the grid) is not part of the thrust-side operator, as in the
    standard pipeline; unchanged in every variant.

    cd /home/lunarz/worktrees/decisioncal && nice -n 19 python3 -I -W ignore \\
        Tools/DecisionCal/study_probe_rate.py --out DIR [--seed 9595] [--n-per 24] [--procs 4]
"""

import argparse
import contextlib
import functools
import hashlib
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
from plant import DT, Plant, PlantSpec  # noqa: E402
from plantcard.family import Evidence, Segment  # noqa: E402
from plantcard.marine.fieldtrials import sg_velocity  # noqa: E402
from study import _seed  # noqa: E402

SEED = 9595                                   # development seed; the confirmatory seeds 20261109-20261114 are forbidden
FORBIDDEN = range(20261109, 20261115)
STRATA = ('timing', 'transient')
ARMS = ('rigid', 'rigid+delay', 'rigid+lag', 'joint')
LIBS = {'v2': LIBRARY_V2, 'v3': LIBRARY_V3}
CAP = 60.0
K_HIDDEN, M_PART = 6, 16                      # as study_m2
GYRO_BOX = 5                                  # hard-coded in twin.Regression._cum_thrust

VARIANTS = {
    'V0': dict(gps_dt=0.20, sg_half=1.0, gyro_dt=probes.GYRO_DT, gyro_box=5, grid_dt=probes.GRID_DT, pos_scale=1.0),
    'V1': dict(gps_dt=0.04, sg_half=1.0, gyro_dt=probes.GYRO_DT, gyro_box=5, grid_dt=probes.GRID_DT, pos_scale=1.0),
    'V2': dict(gps_dt=0.04, sg_half=0.3, gyro_dt=probes.GYRO_DT, gyro_box=5, grid_dt=probes.GRID_DT, pos_scale=1.0),
    'V3': dict(gps_dt=0.04, sg_half=1.0, gyro_dt=probes.GYRO_DT, gyro_box=5, grid_dt=probes.GRID_DT, pos_scale=0.25),
}


# ---------------------------------------------------------------------------------------------
# parameterised probes.generate
# ---------------------------------------------------------------------------------------------

def _ticks(x, what):
    k = int(round(x / DT))
    if k < 1:
        raise ValueError(f'{what}={x} is below one plant tick')
    return k


def generate_variant(spec_row, seed, var, protocol=probes.PROTOCOL, session='probe', tier='synthetic'):
    """probes.generate with the sampling / smoothing of ``var`` (a VARIANTS entry). Identical to
    probes.generate when var == VARIANTS['V0'] (bit-exact; tested). ``pos_scale`` multiplies the hidden boat's
    pos_sd for this log only."""
    gps_dt, sg_half, gyro_dt, gyro_box, grid_dt = (var[k] for k in ('gps_dt', 'sg_half', 'gyro_dt', 'gyro_box', 'grid_dt'))
    if gyro_box != GYRO_BOX:
        raise ValueError('twin.Regression hard-codes a 5-sample gyro box; a different box would be a mismatched operator')
    sub = grid_dt / DT
    if abs(sub - round(sub)) > 1e-9:
        raise ValueError('the segment grid must be a whole number of plant ticks (twin.Regression)')
    if abs(gps_dt / DT - round(gps_dt / DT)) > 1e-9:
        raise ValueError(f'gps_dt={gps_dt} is not a whole number of plant ticks (DT={DT})')
    if var.get('pos_scale', 1.0) != 1.0:
        spec_row = spec_row.with_(pos_sd=spec_row.pos_sd * var['pos_scale'])
    BASELINE_S, SEG_S = probes.BASELINE_S, probes.SEG_S
    n = len(protocol)
    spec = spec_row.repeat(n)
    seeds = (np.uint64(seed) * np.uint64(1000003) + np.arange(n, dtype=np.uint64))
    plant = Plant(spec, seeds, t_max=SEG_S)
    ticks = int(round(SEG_S / DT))
    tt = np.arange(ticks) * DT
    Q = np.stack([probes.schedule(ph, tt) for _, _, ph in protocol], 1)
    gps_every, head_every, gyro_every = (_ticks(x, nm) for x, nm in
                                         ((gps_dt, 'gps_dt'), (probes.HEAD_DT, 'head_dt'), (gyro_dt, 'gyro_dt')))
    tg, P, th, H, tr, R = [], [], [], [], [], []
    truth = np.empty((ticks, n, 6))
    for k in range(ticks):
        plant.step(Q[k])
        truth[k] = plant.st
        pose, vel, gyro = plant.measure()
        if plant.tick % gps_every == 0:
            tg.append(plant.t)
            P.append(pose[:, :2])
        if plant.tick % head_every == 0:
            th.append(plant.t)
            H.append(pose[:, 2])
        if plant.tick % gyro_every == 0:
            tr.append(plant.t)
            R.append(gyro)
    tg, P, th, H, tr, R = map(np.asarray, (tg, P, th, H, tr, R))
    grid = np.arange(1.0, SEG_S - 1.0, grid_dt)
    segs = []
    for i, (name, group, phases) in enumerate(protocol):
        Pi = P[:, i]
        pos = np.stack([np.interp(grid, tg, Pi[:, 0]), np.interp(grid, tg, Pi[:, 1])], -1)
        psi = np.interp(grid, th, np.unwrap(H[:, i]))
        r = np.interp(grid, tr, np.convolve(R[:, i], np.ones(gyro_box) / gyro_box, 'same'))
        vw = sg_velocity(tg, Pi, grid, half=sg_half)
        c, s = np.cos(psi), np.sin(psi)
        X = np.stack([pos[:, 0], pos[:, 1], psi, c * vw[:, 0] + s * vw[:, 1], -s * vw[:, 0] + c * vw[:, 1], r], -1)
        U = probes.schedule(phases, grid)
        bw = (grid >= 6.0) & (grid < BASELINE_S - 0.2)
        k0, k1 = np.flatnonzero(bw)[[0, -1]]
        cur = tuple(float(x) for x in (pos[k1] - pos[k0]) / (grid[k1] - grid[k0]))
        pulse_end = BASELINE_S + sum(d for d, _ in phases)
        t0 = grid[0]
        h = hashlib.sha256(np.ascontiguousarray(X).tobytes() + np.ascontiguousarray(U).tobytes()).hexdigest()
        segs.append(Segment(
            session=f'{session}:{group}', tier=tier, t=grid - t0, x=X, u=U, u_kind='pwm_q',
            quality={'synthetic': True}, labels=[group, name],
            meta={'current': cur, 'current_sd': float(np.sqrt(vw[bw].var(0).mean())),
                  'fit_window': (BASELINE_S - 1.0 - t0, min(grid[-1], pulse_end + 8.0) - t0),
                  'pulse_s': pulse_end - BASELINE_S, 'flags': ['current_from_drift', 'synthetic'],
                  'gps_t': tg - t0, 'gyro_t': tr - t0, 'sg_half': sg_half},
            name=name, source_sha=h, t_event=BASELINE_S - t0))
    ev = Evidence(segs, {'kind': 'decisioncal.probe_rate', 'seed': int(seed), 'n_segments': n, 'variant': dict(var),
                         'probe_seconds': probes.probe_seconds(protocol),
                         'sha256': hashlib.sha256(''.join(s.source_sha for s in segs).encode()).hexdigest()})
    return ev, {'truth': truth, 'Q': Q, 't': tt}


@contextlib.contextmanager
def matched_operator(half):
    """Make twin.Regression's integrated-thrust operator the SG operator the log was built with."""
    orig = twin.sg_velocity
    twin.sg_velocity = functools.partial(sg_velocity, half=half)
    try:
        yield
    finally:
        twin.sg_velocity = orig


def build_regression(ev, half=None, **kw):
    """twin.Regression for a log made by generate_variant (window read from the segments' meta)."""
    half = ev.segments[0].meta.get('sg_half', 1.0) if half is None else half
    with matched_operator(half):
        return twin.Regression(ev, **kw)


# ---------------------------------------------------------------------------------------------
# per-boat evaluation
# ---------------------------------------------------------------------------------------------

def _hidden_scores(spec_row, pid, seed):
    """Hidden-boat J [C, K] and docked per library; variant independent, computed once per boat."""
    out = {}
    for lname, lib in LIBS.items():
        C = len(lib)
        met, _ = mission.run(spec_row.repeat(C * K_HIDDEN), np.repeat(np.arange(C), K_HIDDEN),
                             np.array([_seed(seed, pid, 'hidden', k) for k in range(K_HIDDEN)] * C, dtype=np.uint64),
                             lib=lib)
        out[lname] = {'J_h': met['J'].reshape(C, K_HIDDEN), 'docked': met['docked'].reshape(C, K_HIDDEN).mean(1)}
    return out


def _eval_variant(spec_row, pid, seed, vname, hid):
    var = VARIANTS[vname]
    t0 = time.time()
    ev, _ = generate_variant(spec_row, _seed(seed, pid, 'probe'), var)
    reg = build_regression(ev, var['sg_half'])
    fits = {}
    for i, arm in enumerate(ARMS):
        fits[arm] = twin.fit(reg, arm, M_PART, _seed(seed, pid, 'fit') % (2 ** 31) + 7919 * i)
    t_fit = time.time() - t0
    cur = np.mean([s.meta['current'] for s in ev.segments], axis=0)
    cur_sd = float(np.std([s.meta['current'] for s in ev.segments], axis=0).mean())
    out = {'fit': {}, 'libs': {}}
    for arm in ARMS:
        p, rep = fits[arm]
        out['fit'][arm] = {'map': rep['map'], 'delay_mean': float(p['delay_s'].mean()), 'tau_mean': float(p['tau_s'].mean()),
                           'rev_hold_mean': float(p['rev_hold_s'].mean()), 'entropy': rep['grid_entropy']}
    for lname, lib in LIBS.items():
        C = len(lib)
        specs, ctrls, seeds = [], [], []
        for arm in ARMS:
            ts = twin.to_spec(fits[arm][0], current=tuple(cur), current_sd=cur_sd,
                              rng=np.random.default_rng(_seed(seed, pid, 'cur') % (2 ** 31)))
            specs.append(ts.repeat(C))
            ctrls.append(np.tile(np.arange(C), M_PART))
            seeds.append(np.repeat([_seed(seed, pid, 'twin', m) for m in range(M_PART)], C))
        met, _ = mission.run(PlantSpec.concat(specs), np.concatenate(ctrls), np.concatenate(seeds).astype(np.uint64), lib=lib)
        J = met['J']
        rows = {}
        for a, arm in enumerate(ARMS):
            Jt = J[a * M_PART * C:(a + 1) * M_PART * C].reshape(M_PART, C).T
            sel = score.selection(Jt)
            reg_x = score.cross_fit_regret(score.finite_scores(hid[lname]['J_h']), sel)
            rows[arm] = {'sel': sel, 'regret': reg_x, 'regret_capped': min(reg_x, CAP),
                         'dock': float(hid[lname]['docked'][sel])}
        out['libs'][lname] = rows
    out['wall_s'] = {'fit': t_fit, 'total': time.time() - t0}
    return out


def evaluate(job):
    spec_row, pid, seed = job['spec'], job['id'], job['seed']
    t0 = time.time()
    hid = _hidden_scores(spec_row, pid, seed)
    t_h = time.time() - t0
    truth = {'dead_time': float(spec_row.dead_time[0]), 'odom_latency': float(spec_row.odom_latency[0]),
             'tau_up': float(spec_row.tau_up[0]), 'tau_down': float(spec_row.tau_down[0]),
             'rev_hold': float(spec_row.rev_hold[0])}
    truth['delay'] = truth['dead_time'] + truth['odom_latency']
    res = {'id': pid, 'stratum': job['stratum'], 'seed': seed, 'truth': truth, 'variants': {}, 'hidden_wall_s': t_h}
    for v in job['variants']:
        res['variants'][v] = _eval_variant(spec_row, pid, seed, v, hid)
    for lname in LIBS:
        res.setdefault('oracle', {})[lname] = int(np.argmin(score.finite_scores(hid[lname]['J_h']).mean(1)))
    res['wall_s'] = time.time() - t0
    return res


# ---------------------------------------------------------------------------------------------
# summary
# ---------------------------------------------------------------------------------------------

def _boot_ci(d, seed=0, n=4000):
    d = np.asarray(d, float)
    idx = np.random.default_rng(seed).integers(0, len(d), (n, len(d)))
    return [float(x) for x in np.quantile(d[idx].mean(1), [0.05, 0.95])]


def summarize(rows, variants):
    out = {'n_rows': len(rows), 'cap_s': CAP, 'per_variant': {}, 'paired': {}}
    for st in STRATA:
        R = [r for r in rows if r['stratum'] == st and all(v in r['variants'] for v in variants)]
        out['per_variant'][st] = {'n': len(R)}
        if not R:
            continue
        for v in variants:
            d = {'true_delay': float(np.mean([r['truth']['delay'] for r in R])),
                 'true_tau_up': float(np.mean([r['truth']['tau_up'] for r in R])),
                 'true_rev_hold': float(np.mean([r['truth']['rev_hold'] for r in R]))}
            for arm in ('rigid+delay', 'joint'):
                f = [r['variants'][v]['fit'][arm] for r in R]
                d[arm] = {'delay_mean': float(np.mean([x['delay_mean'] for x in f])),
                          'delay_bias': float(np.mean([x['delay_mean'] - r['truth']['delay'] for x, r in zip(f, R)])),
                          'delay_rmse': float(np.sqrt(np.mean([(x['delay_mean'] - r['truth']['delay']) ** 2 for x, r in zip(f, R)]))),
                          'tau_mean': float(np.mean([x['tau_mean'] for x in f])),
                          'rev_hold_mean': float(np.mean([x['rev_hold_mean'] for x in f]))}
            for lib in LIBS:
                d[lib] = {a: {'regret_capped': float(np.mean([r['variants'][v]['libs'][lib][a]['regret_capped'] for r in R])),
                              'dock': float(np.mean([r['variants'][v]['libs'][lib][a]['dock'] for r in R]))} for a in ARMS}
            out['per_variant'][st][v] = d
        for v in variants[1:]:
            for lib in LIBS:
                for a in ('rigid+delay', 'joint'):
                    # delta = Vk - V0: positive regret delta = worse selection with the better measurement
                    dr = np.array([r['variants'][v]['libs'][lib][a]['regret_capped'] - r['variants'][variants[0]]['libs'][lib][a]['regret_capped'] for r in R])
                    dk = np.array([r['variants'][v]['libs'][lib][a]['dock'] - r['variants'][variants[0]]['libs'][lib][a]['dock'] for r in R])
                    dd = np.array([r['variants'][v]['fit'][a]['delay_mean'] - r['variants'][variants[0]]['fit'][a]['delay_mean'] for r in R])
                    out['paired'][f'{st}|{lib}|{a}|{v}-{variants[0]}'] = {
                        'n': len(R), 'd_regret_capped': float(dr.mean()), 'd_regret_ci90': _boot_ci(dr),
                        'd_dock': float(dk.mean()), 'd_dock_ci90': _boot_ci(dk, 1),
                        'd_fitted_delay': float(dd.mean()), 'd_fitted_delay_ci90': _boot_ci(dd, 2)}
    return out


def print_summary(s, variants):
    print(f'== probe-rate pilot, capped regret {s["cap_s"]:.0f} s; delta = V - {variants[0]} (positive regret delta = WORSE)')
    for st, body in s['per_variant'].items():
        if body['n'] == 0:
            continue
        print(f'-- {st} (n={body["n"]})')
        for v in variants:
            d = body[v]
            print(f'  {v}: true delay {d["true_delay"]:.3f} | ' + ' | '.join(
                f'{a} delay {d[a]["delay_mean"]:.3f} (bias {d[a]["delay_bias"]:+.3f}) tau {d[a]["tau_mean"]:.2f} hold {d[a]["rev_hold_mean"]:.2f}'
                for a in ('rigid+delay', 'joint')) + f' [true tau {d["true_tau_up"]:.2f}, hold {d["true_rev_hold"]:.2f}]')
            for lib in LIBS:
                print(f'      {lib} regret/dock: ' + ' '.join(f'{a}={d[lib][a]["regret_capped"]:.1f}/{d[lib][a]["dock"]:.2f}' for a in ARMS))
    print('-- paired (Vk - V0), mean [90% bootstrap CI]')
    for k, v in s['paired'].items():
        print(f'  {k:42s} n={v["n"]:2d} d_regret {v["d_regret_capped"]:+6.1f} [{v["d_regret_ci90"][0]:+.1f},{v["d_regret_ci90"][1]:+.1f}]'
              f'  d_dock {v["d_dock"]:+.2f} [{v["d_dock_ci90"][0]:+.2f},{v["d_dock_ci90"][1]:+.2f}]'
              f'  d_delay {v["d_fitted_delay"]:+.3f} [{v["d_fitted_delay_ci90"][0]:+.3f},{v["d_fitted_delay_ci90"][1]:+.3f}]')


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--seed', type=int, default=SEED)
    ap.add_argument('--n-per', type=int, default=24, help='boats per stratum (timing, transient)')
    ap.add_argument('--variants', default='V0,V1,V2,V3')
    ap.add_argument('--procs', type=int, default=4)
    ap.add_argument('--limit', type=int, default=0, help='smoke: run at most this many boats (interleaved strata)')
    a = ap.parse_args(argv)
    if a.procs > 4:
        raise SystemExit('innovation compute policy: at most 4 processes')
    if a.seed in FORBIDDEN:
        raise SystemExit('seeds 20261109-20261114 are confirmatory; use a development seed')
    variants = a.variants.split(',')
    if variants[0] != 'V0' or any(v not in VARIANTS for v in variants):
        raise SystemExit('variants must start with V0 and be a subset of ' + ','.join(VARIANTS))
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    per = {st: hidden.sample_hidden(a.n_per, a.seed, st) for st in STRATA}
    jobs = [{'spec': per[st].take([i]), 'id': f'{st}-{i:03d}', 'stratum': st, 'seed': a.seed, 'variants': variants}
            for i in range(a.n_per) for st in STRATA]            # interleaved so a --limit run covers both strata
    if a.limit:
        jobs = jobs[:a.limit]
    done = {}
    pj = out / 'boats.jsonl'
    if pj.exists():
        for line in pj.read_text().splitlines():
            r = json.loads(line)
            if set(variants) <= set(r['variants']):
                done[r['id']] = r
    todo = [j for j in jobs if j['id'] not in done]
    (out / 'manifest.json').write_text(json.dumps(record._clean(
        {'argv': vars(a), 'provenance': _paths.provenance(), 'variants': {v: VARIANTS[v] for v in variants},
         'arms': ARMS, 'libraries': {k: [c.label() for c in v] for k, v in LIBS.items()}, 'cap_s': CAP,
         'evidence_label': 'HSUR', 'started': time.strftime('%Y-%m-%dT%H:%M:%S'), 'n_jobs': len(jobs)}), indent=1, sort_keys=True))
    t0 = time.time()
    with get_context('fork').Pool(a.procs) as pool, open(pj, 'a') as fp:
        for k, r in enumerate(pool.imap_unordered(evaluate, todo)):
            done[r['id']] = r
            fp.write(json.dumps(record._clean(r), sort_keys=True) + '\n')
            fp.flush()
            print(f'[{k + 1}/{len(todo)}] {r["id"]} true delay {r["truth"]["delay"]:.3f} | fitted rigid+delay ' + ' '.join(
                f'{v}={r["variants"][v]["fit"]["rigid+delay"]["delay_mean"]:.3f}' for v in variants) + ' | joint ' + ' '.join(
                f'{v}={r["variants"][v]["fit"]["joint"]["delay_mean"]:.3f}' for v in variants) + ' | v2 rigid+delay regret ' + ' '.join(
                f'{v}={r["variants"][v]["libs"]["v2"]["rigid+delay"]["regret_capped"]:.1f}' for v in variants) +
                f' ({r["wall_s"]:.0f}s boat, {time.time() - t0:.0f}s total)', flush=True)
    rows = [json.loads(json.dumps(record._clean(done[j['id']]))) for j in jobs if j['id'] in done]
    s = summarize(rows, variants)
    s['elapsed_s'] = time.time() - t0
    (out / 'summary.json').write_text(json.dumps(record._clean(s), indent=1, sort_keys=True))
    print_summary(s, variants)


if __name__ == '__main__':
    main()
