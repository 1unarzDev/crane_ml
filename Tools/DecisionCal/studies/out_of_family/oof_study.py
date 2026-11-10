"""Out-of-family study driver (EXPLORATORY; development seeds only; label HSUR-OOF, numpy plant only).

Does the latency result (a twin carrying less than the boat's true command-to-measurement latency picks a
controller that fails to dock) survive on boats with mechanisms the twin family cannot represent, and does a
conservative baseline that uses no timing calibration do just as well? Design and decision rules: README.md.

Modes
  diag   distance from the family only: per config (mechanism set : level), ORACLE in-family projection vs
         the boat itself flown as a twin (oracle-full) vs the hidden repeats. Tuning seed.
  pilot  every arm on delay-dominated boats (hidden 'timing' stratum + mechanisms at each distance level),
         plus hidden-only development boats (another seed) for the leave-population-out fixed controller.

    cd /home/lunarz/worktrees/decisioncal && nice -n 19 python3 -I -W ignore \\
        Tools/DecisionCal/studies/out_of_family/oof_study.py diag --out DIR --configs none:0 all:1 --n 4 --procs 1
    ... oof_study.py pilot --out DIR --procs 4                      # 4 levels x 20 boats + 4 x 20 dev boats
    ... oof_study.py pilot --out DIR --smoke --procs 1              # 1 + 1 boats on 2 levels, M=4, K=2
"""

import argparse
import json
import sys
import time
from multiprocessing import get_context
from pathlib import Path

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]))

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
from oof_plant import HI, MECHS, MechSpec, at_level, latency_stats, use_mech  # noqa: E402
from plant import PlantSpec  # noqa: E402
from study import _seed  # noqa: E402
from study_oracle_twin import project  # noqa: E402

SEED, TUNE_SEED, DEV_SEED = 4141, 4142, 4143      # evaluation / magnitude tuning / leave-population-out boats
FORBIDDEN = range(20261109, 20261117)             # confirmatory seeds M0-M6: refused
STRATUM = 'timing'
# Distance levels: lambda = fraction of every mechanism's plausible upper end (oof_plant.HI), all seven on.
# PROVISIONAL, from the smoke tuning on TUNE_SEED (README "Tuning"); re-tune with `diag --n 12` before the pilot.
LEVELS = {'none': 0.0, 'near': 0.25, 'medium': 0.5, 'far': 1.0}
N_EVAL, N_DEV = 20, 20
EXCESS = (-0.10, -0.05, 0.0, 0.05, 0.10, 0.20)
EXCESS_P90 = (-0.10, -0.05, 0.0, 0.05)          # trimmed for compute: the cliff contrast and the edge region
LDEFS = ('mean', 'p90')
EXC = {'mean': EXCESS, 'p90': EXCESS_P90}
FIT_ARMS = ('rigid', 'rigid+delay', 'joint')
BLANKET = ('rigid@d0.35', 'rigid@d0.47', 'rigid@d0.62')
ENSEMBLE = ('rigid', 'rigid@d0.16') + BLANKET     # assumed delays 0, 0.16, 0.35, 0.47, 0.62 s ('rigid' = 0 s)
ALL_FITS = FIT_ARMS + ('rigid@d0.16',) + BLANKET
SWEEP = tuple(f'fit:{L}:{e:+.2f}' for L in LDEFS for e in EXC[L])
ORACLE = ('ORACLE-proj', 'ORACLE-full')
TWIN_ARMS = ALL_FITS + SWEEP + ('ORACLE-proj',)    # flown on plain Plant; ORACLE-full is flown on MechPlant
CONSERVATIVE = BLANKET + ('fixed-LPO', 'robust')
REALIZABLE = FIT_ARMS
LIBS = {'v2': LIBRARY_V2, 'v3': LIBRARY_V3}
CAP, MARGIN, DOCK_DROP = 60.0, 3.0, 0.10           # stated before any pilot run (README "Decision rules")
K_HID, M_TWIN = 6, 16


def check_seed(*seeds):
    for s in seeds:
        if s in FORBIDDEN:
            raise SystemExit(f'seed {s} is confirmatory (20261109-20261116 are spent); this study is exploratory')
    if len(set(seeds)) != len(seeds):
        raise SystemExit(f'evaluation, tuning and development seeds must differ: {seeds}')


def parse_config(c):
    """'none:0', 'link:1', 'all:0.6', 'link+waves:0.5' -> (mechs tuple, lambda)."""
    name, lam = c.split(':')
    mechs = MECHS if name in ('all', 'none') else tuple(name.split('+'))
    return mechs, (0.0 if name == 'none' else float(lam))


def split_delay(cmd, meas, e):
    """Dead time / odometry latency of a twin carrying cmd + meas + e (excess on the dead time; a deficit
    larger than the dead time comes off the latency), as study_m5.split."""
    d = cmd + e
    lat = meas
    if d < 0:
        lat, d = max(lat + d, 0.0), 0.0
    return d, lat


def with_delay(spec, dead, lat):
    n = spec.n
    return spec.with_(dead_time=np.full(n, dead), odom_latency=np.full(n, lat))


# ---------------------------------------------------------------------------------------------
# one boat
# ---------------------------------------------------------------------------------------------

def _distance(Jp, Jf, Jh):
    """Rank correlation / mean abs error between per-controller mean scores (finite-scored)."""
    a, b, h = (score.finite_scores(x).mean(1) for x in (Jp, Jf, Jh))
    return {'rho_proj_full': score.spearman(a, b), 'mae_proj_full': float(np.abs(a - b).mean()),
            'rho_full_hidden': score.spearman(b, h), 'mae_full_hidden': float(np.abs(b - h).mean()),
            'rho_proj_hidden': score.spearman(a, h), 'mae_proj_hidden': float(np.abs(a - h).mean()),
            'same_pick_proj_full': bool(np.argmin(a) == np.argmin(b))}


def evaluate(job):
    spec_row, mech, base, seed, K, M, kind = (job[k] for k in ('spec', 'mech', 'base', 'seed', 'K', 'M', 'kind'))
    t0 = time.time()
    h_seeds = np.array([_seed(seed, base, 'hidden', k) for k in range(K)], np.uint64)
    t_seeds = np.array([_seed(seed, base, 'twin', m) for m in range(M)], np.uint64)
    lat = latency_stats(spec_row, mech, h_seeds, t_max=mission.T_MAX)
    out = {'id': job['id'], 'base': base, 'level': job['level'], 'lam': job['lam'], 'kind': kind, 'seed': seed,
           'spec': spec_row.row(0), 'mech': mech.row(0), 'latency': lat, 'libs': {}, 'budget': {}}
    if kind == 'dev':                               # hidden repeats only (leave-population-out fixed controller)
        for lname, lib in LIBS.items():
            C = len(lib)
            with use_mech(mech):
                met, _ = mission.run(spec_row.repeat(C * K), np.repeat(np.arange(C), K), np.tile(h_seeds, C), lib=lib)
            out['libs'][lname] = {'J_h': met['J'].reshape(C, K).tolist(),
                                  'hidden_docked': met['docked'].reshape(C, K).mean(1).tolist()}
        out['budget']['wall_s'] = time.time() - t0
        return out
    with use_mech(mech):
        ev, _ = probes.generate(spec_row, _seed(seed, base, 'probe'))
    cur = np.mean([s.meta['current'] for s in ev.segments], axis=0)
    cur_sd = float(np.std([s.meta['current'] for s in ev.segments], axis=0).mean())
    cur_rng = lambda: np.random.default_rng(_seed(seed, base, 'cur') % (2 ** 31))
    mk = lambda part: twin.to_spec(part, current=tuple(cur), current_sd=cur_sd, rng=cur_rng())
    specs = {}
    if kind == 'eval':
        fits, _reg = twin.fit_all(ev, M, _seed(seed, base, 'fit') % (2 ** 31), ALL_FITS)
        out['fits'] = {a: fits[a][1] for a in ALL_FITS}
        for a in ALL_FITS:
            specs[a] = mk(fits[a][0])
        body = specs['rigid+delay']
        same = abs(lat['L_p90'] - lat['L_mean']) < 1e-9          # ideal link and no estimator: p90 = mean
        out['p90_aliased'] = same
        for L in (('mean',) if same else LDEFS):
            cmd = lat['cmd_mean'] if L == 'mean' else lat['L_p90'] - lat['meas_mean']
            for e in EXC[L]:
                specs[f'fit:{L}:{e:+.2f}'] = with_delay(body, *split_delay(cmd, lat['meas_mean'], e))
    specs['ORACLE-proj'] = with_delay(mk(project(spec_row, 0.0, M)[0]), lat['cmd_mean'], lat['meas_mean'])
    arms = [a for a in TWIN_ARMS if a in specs]
    out['budget']['fit_s'] = time.time() - t0
    for lname, lib in LIBS.items():
        C = len(lib)
        t1 = time.time()
        met, _ = mission.run(PlantSpec.concat([specs[a].repeat(C) for a in arms]),
                             np.tile(np.tile(np.arange(C), M), len(arms)), np.tile(np.repeat(t_seeds, C), len(arms)),
                             lib=lib)
        J_arms = {a: met['J'][i * M * C:(i + 1) * M * C].reshape(M, C).T for i, a in enumerate(arms)}
        t2 = time.time()
        with use_mech(mech):                        # the boat itself as a twin (ORACLE-full) + the hidden repeats
            met2, _ = mission.run(spec_row.repeat(M * C + C * K),
                                  np.concatenate([np.tile(np.arange(C), M), np.repeat(np.arange(C), K)]),
                                  np.concatenate([np.repeat(t_seeds, C), np.tile(h_seeds, C)]), lib=lib)
        J_arms['ORACLE-full'] = met2['J'][:M * C].reshape(M, C).T
        if out.get('p90_aliased'):
            for e in EXCESS_P90:
                J_arms[f'fit:p90:{e:+.2f}'] = J_arms[f'fit:mean:{e:+.2f}']
        J_h = met2['J'][M * C:].reshape(C, K)
        sc = score.plant_scores(J_h, J_arms, default_arm='ORACLE-proj' if kind != 'eval' else 'rigid')
        if kind == 'eval':
            worst = np.max([score.finite_scores(J_arms[a]).mean(1) for a in ENSEMBLE], axis=0)
            sel = int(np.argmin(worst))
            sc['arms']['robust'] = {'sel': sel, 'regret': score.cross_fit_regret(score.finite_scores(J_h), sel),
                                    'worst_case_pred': worst.tolist()}
        out['libs'][lname] = {
            'J_h': J_h.tolist(), 'scores': sc, 'hidden_docked': met2['docked'][M * C:].reshape(C, K).mean(1).tolist(),
            'J_twin_mean': {a: score.finite_scores(v).mean(1).tolist() for a, v in J_arms.items()},
            'distance': _distance(J_arms['ORACLE-proj'], J_arms['ORACLE-full'], J_h)}
        if kind == 'eval':
            out['libs'][lname]['distance_joint'] = _distance(J_arms['joint'], J_arms['ORACLE-full'], J_h)
        out['budget'][lname] = {'twin_s': t2 - t1, 'mech_s': time.time() - t2, 'twin_envs': len(arms) * M * C,
                                'mech_envs': M * C + C * K}
    out['budget']['wall_s'] = time.time() - t0
    return out


# ---------------------------------------------------------------------------------------------
# jobs, running, resuming
# ---------------------------------------------------------------------------------------------

def boats(n, seed, lam, mechs, level, kind, K, M, prefix=''):
    spec = hidden.sample_hidden(n, seed, STRATUM)
    mech = at_level(n, lam, mechs, seed)
    out = []
    for i in range(n):
        base = f'{prefix}{STRATUM}-{seed}-{i:03d}'
        out.append({'spec': spec.take([i]), 'mech': mech.take([i]), 'base': base, 'id': f'{level}/{kind}/{base}',
                    'level': level, 'lam': lam, 'seed': seed, 'K': K, 'M': M, 'kind': kind})
    return out


def run_jobs(jobs, out, procs, manifest):
    out.mkdir(parents=True, exist_ok=True)
    pj = out / 'plants.jsonl'
    done = {}
    if pj.exists():
        for line in pj.read_text().splitlines():
            if line.strip():
                r = json.loads(line)
                done[r['id']] = r
    todo = [j for j in jobs if j['id'] not in done]
    (out / 'manifest.json').write_text(json.dumps(record._clean(manifest), indent=1, sort_keys=True))
    t0 = time.time()
    it = map(evaluate, todo) if procs == 1 else None
    pool = None
    if procs > 1:
        pool = get_context('fork').Pool(procs)
        it = pool.imap_unordered(evaluate, todo)
    try:
        with open(pj, 'a') as fp:
            for k, r in enumerate(it):
                done[r['id']] = r
                fp.write(json.dumps(record._clean(r), sort_keys=True) + '\n')
                fp.flush()
                print(f'[{k + 1}/{len(todo)}] {r["id"]} {r["budget"]["wall_s"]:.0f}s (elapsed {time.time() - t0:.0f}s)',
                      flush=True)
    finally:
        if pool is not None:
            pool.close()
            pool.join()
    return [json.loads(json.dumps(record._clean(done[j['id']]))) for j in jobs if j['id'] in done]


# ---------------------------------------------------------------------------------------------
# analysis
# ---------------------------------------------------------------------------------------------

def boot(d, n_boot=4000, seed=0):
    d = np.asarray(d, float)
    idx = np.random.default_rng(seed).integers(0, len(d), (n_boot, len(d)))
    lo, hi = np.quantile(d[idx].mean(1), [0.05, 0.95])
    return {'n': int(len(d)), 'mean': float(d.mean()), 'ci90': [float(lo), float(hi)], 'median': float(np.median(d)),
            'pos': int((d > 2).sum()), 'neg': int((d < -2).sum())}


def fixed_picks(dev_rows):
    """Leave-population-out fixed controller per (level, library): best mean hidden score on the dev boats."""
    out = {}
    for lev in sorted({r['level'] for r in dev_rows}):
        R = [r for r in dev_rows if r['level'] == lev]
        out[lev] = {lib: int(np.argmin(np.mean([score.finite_scores(np.array(r['libs'][lib]['J_h'])).mean(1)
                                                 for r in R], axis=0))) for lib in LIBS}
    return out


def _sel_reg(r, lib, arm, fixed):
    L = r['libs'][lib]
    if arm == 'fixed-LPO':
        sel = fixed[r['level']][lib]
        reg = score.cross_fit_regret(score.finite_scores(np.array(L['J_h'])), sel)
    else:
        sel, reg = L['scores']['arms'][arm]['sel'], L['scores']['arms'][arm]['regret']
    return sel, min(reg, CAP), L['hidden_docked'][sel]


def verdict_baseline(ci):
    lo, hi = ci
    if lo > -MARGIN and hi < MARGIN:
        return 'EQUIVALENT (baseline as good)'
    if hi < 0:
        return 'fitted delay better' + (' (> margin)' if hi < -MARGIN else '')
    if lo > 0:
        return 'baseline better'
    return 'inconclusive'


def analyse(rows, lib, fixed, ref=None):
    arms = list(FIT_ARMS) + list(CONSERVATIVE) + list(SWEEP) + list(ORACLE)
    reg = {a: np.array([_sel_reg(r, lib, a, fixed)[1] for r in rows]) for a in arms}
    dock = {a: np.array([_sel_reg(r, lib, a, fixed)[2] for r in rows]) for a in arms}
    table = {a: {**boot(reg[a]), 'dock': float(dock[a].mean())} for a in arms}
    cons_means = {a: reg[a].mean() for a in CONSERVATIVE}
    best_cons = min(cons_means, key=cons_means.get)
    d_best = boot(reg['rigid+delay'] - reg[best_cons])
    d_047 = boot(reg['rigid+delay'] - reg['rigid@d0.47'])
    d_robust = boot(reg['rigid+delay'] - reg['robust'])
    d_fixed = boot(reg['rigid+delay'] - reg['fixed-LPO'])
    cliff = {}
    for L in LDEFS:
        c = boot(reg[f'fit:{L}:-0.10'] - reg[f'fit:{L}:+0.05'])
        dd = boot(dock[f'fit:{L}:+0.05'] - dock[f'fit:{L}:-0.10'])
        edge = next((e for e in EXC[L] if dock[f'fit:{L}:{e:+.2f}'].mean() >= 0.9), None)
        cliff[L] = {'regret': c, 'dock_drop': dd, 'edge_e': edge,
                    'survives': bool(c['ci90'][0] > 0 and dd['mean'] >= DOCK_DROP)}
    dist = {k: float(np.nanmedian([r['libs'][lib]['distance'][k] for r in rows]))
            for k in rows[0]['libs'][lib]['distance']}
    dist_joint = {k: float(np.nanmedian([r['libs'][lib]['distance_joint'][k] for r in rows]))
                  for k in ('rho_proj_full', 'mae_proj_full')}
    fd = np.array([r['fits']['rigid+delay']['map']['delay_s'] for r in rows])
    Lm = np.array([r['latency']['L_mean'] for r in rows])
    Lp = np.array([r['latency']['L_p90'] for r in rows])
    out = {'n': len(rows), 'arms': table, 'best_conservative': best_cons,
           'fitted_minus_best_conservative': {**d_best, 'verdict': verdict_baseline(d_best['ci90']),
                                               'note': 'best conservative arm chosen post hoc (favours the baseline)'},
           'fitted_minus_assumed_0.47': {**d_047, 'verdict': verdict_baseline(d_047['ci90'])},
           'fitted_minus_robust': {**d_robust, 'verdict': verdict_baseline(d_robust['ci90'])},
           'fitted_minus_fixed_LPO': {**d_fixed, 'verdict': verdict_baseline(d_fixed['ci90'])},
           'fitted_minus_oracle_proj': boot(reg['rigid+delay'] - reg['ORACLE-proj']),
           'cliff': cliff, 'distance_median': dist, 'distance_joint_median': dist_joint,
           'latency': {'L_mean': float(Lm.mean()), 'L_p90': float(Lp.mean()), 'L_spec': float(np.mean([r['latency']['L_spec'] for r in rows])),
                       'fitted_minus_L_mean': float((fd - Lm).mean()), 'fitted_minus_L_p90': float((fd - Lp).mean())}}
    if ref is not None:                             # cliff relative to the in-family ('none') level, same library
        for L in LDEFS:
            r0 = ref['cliff'][L]['regret']['mean']
            c = out['cliff'][L]
            c['ratio_to_none'] = float(c['regret']['mean'] / r0) if abs(r0) > 1e-9 else None
            c['blurred'] = bool(c['ratio_to_none'] is not None and c['ratio_to_none'] < 0.5)
            e0 = ref['cliff'][L]['edge_e']
            c['edge_moved'] = bool(e0 is not None and c['edge_e'] is not None and abs(c['edge_e'] - e0) >= 0.05 - 1e-9)
    return out


def summarize(rows):
    ev = [r for r in rows if r['kind'] == 'eval']
    dev = [r for r in rows if r['kind'] == 'dev']
    fixed = fixed_picks(dev)
    levels = [lv for lv in LEVELS if any(r['level'] == lv for r in ev)]
    out = {'levels': {}, 'fixed_picks': fixed, 'n_eval': len(ev), 'n_dev': len(dev), 'margin_s': MARGIN, 'cap_s': CAP}
    for lib in LIBS:
        ref = None
        for lv in levels:
            R = [r for r in ev if r['level'] == lv]
            if lv not in fixed:
                continue
            a = analyse(R, lib, fixed, ref if lv != 'none' else None)
            if lv == 'none':
                ref = a
            out['levels'][f'{lv}/{lib}'] = a
    return out


def print_summary(s):
    show = ('rigid', 'rigid+delay', 'joint') + CONSERVATIVE + ORACLE
    for key, a in s['levels'].items():
        d = a['distance_median']
        print(f'== {key} (n={a["n"]})  distance: rho(proj,full) {d["rho_proj_full"]:.2f}  MAE {d["mae_proj_full"]:.1f} s'
              f'  | noise floor rho(full,hidden) {d["rho_full_hidden"]:.2f} MAE {d["mae_full_hidden"]:.1f}'
              f'  | joint rho {a["distance_joint_median"]["rho_proj_full"]:.2f}')
        lt = a['latency']
        print(f'   latency: L_spec {lt["L_spec"]:.3f}  L_mean {lt["L_mean"]:.3f}  L_p90 {lt["L_p90"]:.3f}  '
              f'fitted-L_mean {lt["fitted_minus_L_mean"]:+.3f}  fitted-L_p90 {lt["fitted_minus_L_p90"]:+.3f}')
        print('   regret capped / dock: ' + ' | '.join(f'{x} {a["arms"][x]["mean"]:.1f} [{a["arms"][x]["ci90"][0]:.1f},'
                                                       f'{a["arms"][x]["ci90"][1]:.1f}]/{a["arms"][x]["dock"]:.2f}' for x in show))
        for L in LDEFS:
            print(f'   sweep L_{L}: ' + ' | '.join(f'{e:+.2f}: {a["arms"][f"fit:{L}:{e:+.2f}"]["mean"]:.1f}/'
                                                  f'{a["arms"][f"fit:{L}:{e:+.2f}"]["dock"]:.2f}' for e in EXC[L]))
            c = a['cliff'][L]
            extra = '' if 'ratio_to_none' not in c else (f' ratio-to-none {c["ratio_to_none"]}' if c['ratio_to_none'] is None else
                                                          f' ratio-to-none {c["ratio_to_none"]:.2f}{" BLURRED" if c["blurred"] else ""}'
                                                          f'{" EDGE-MOVED" if c["edge_moved"] else ""}')
            print(f'   cliff L_{L} (e=-0.10 minus +0.05): {c["regret"]["mean"]:+.1f} [{c["regret"]["ci90"][0]:+.1f},'
                  f'{c["regret"]["ci90"][1]:+.1f}] dock drop {c["dock_drop"]["mean"]:+.2f}  edge e={c["edge_e"]}'
                  f'  {"SURVIVES" if c["survives"] else "does not survive"}{extra}')
        for k in ('fitted_minus_best_conservative', 'fitted_minus_assumed_0.47', 'fitted_minus_robust', 'fitted_minus_fixed_LPO'):
            v = a[k]
            nm = k + (f' ({a["best_conservative"]})' if k.endswith('best_conservative') else '')
            print(f'   {nm:52s} {v["mean"]:+6.1f} [{v["ci90"][0]:+.1f},{v["ci90"][1]:+.1f}] -> {v["verdict"]}')
        v = a['fitted_minus_oracle_proj']
        print(f'   {"fitted minus ORACLE-proj (ceiling gap)":52s} {v["mean"]:+6.1f} [{v["ci90"][0]:+.1f},{v["ci90"][1]:+.1f}]')
    print('\nDECISION TABLE (margin +-%.0f s; cliff survives = CI>0 and dock drop >= %.2f)' % (MARGIN, DOCK_DROP))
    print(f'  {"level/lib":12s} {"rho":>5s} {"cliff L_mean":>22s} {"cliff L_p90":>22s}  {"fitted - best conservative":40s}')
    for key, a in s['levels'].items():
        cm, cp = a['cliff']['mean'], a['cliff']['p90']
        b = a['fitted_minus_best_conservative']
        f = lambda c: f'{c["regret"]["mean"]:+.1f}[{c["regret"]["ci90"][0]:+.0f},{c["regret"]["ci90"][1]:+.0f}]{"Y" if c["survives"] else "n"}'
        print(f'  {key:12s} {a["distance_median"]["rho_proj_full"]:5.2f} {f(cm):>22s} {f(cp):>22s}  '
              f'{b["mean"]:+.1f} [{b["ci90"][0]:+.1f},{b["ci90"][1]:+.1f}] {b["verdict"]} vs {a["best_conservative"]}')


def summarize_diag(rows):
    out = {}
    for cfg in sorted({r['level'] for r in rows}):
        R = [r for r in rows if r['level'] == cfg]
        for lib in LIBS:
            D = [r['libs'][lib]['distance'] for r in R]
            out[f'{cfg}/{lib}'] = {'n': len(R), **{k: float(np.nanmedian([d[k] for d in D])) for k in D[0] if k != 'same_pick_proj_full'},
                                   'same_pick_proj_full': float(np.mean([d['same_pick_proj_full'] for d in D])),
                                   'proj_regret_capped': float(np.mean([min(r['libs'][lib]['scores']['arms']['ORACLE-proj']['regret'], CAP) for r in R])),
                                   'full_regret_capped': float(np.mean([min(r['libs'][lib]['scores']['arms']['ORACLE-full']['regret'], CAP) for r in R])),
                                   'L_mean': float(np.mean([r['latency']['L_mean'] for r in R])),
                                   'L_p90': float(np.mean([r['latency']['L_p90'] for r in R])),
                                   'L_spec': float(np.mean([r['latency']['L_spec'] for r in R]))}
    return out


def print_diag(s):
    print(f'{"config/lib":22s} {"n":>2s} {"rho(p,f)":>8s} {"MAE(p,f)":>8s} {"rho(f,h)":>8s} {"MAE(f,h)":>8s} {"rho(p,h)":>8s} '
          f'{"pick=":>5s} {"reg p":>6s} {"reg f":>6s} {"L_spec":>6s} {"L_mean":>6s} {"L_p90":>6s}')
    for k, v in s.items():
        print(f'{k:22s} {v["n"]:2d} {v["rho_proj_full"]:8.2f} {v["mae_proj_full"]:8.1f} {v["rho_full_hidden"]:8.2f} '
              f'{v["mae_full_hidden"]:8.1f} {v["rho_proj_hidden"]:8.2f} {v["same_pick_proj_full"]:5.2f} '
              f'{v["proj_regret_capped"]:6.1f} {v["full_regret_capped"]:6.1f} {v["L_spec"]:6.3f} {v["L_mean"]:6.3f} {v["L_p90"]:6.3f}')


# ---------------------------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=('diag', 'pilot'))
    ap.add_argument('--out', required=True)
    ap.add_argument('--seed', type=int, default=SEED, help='evaluation seed (development; 20261109-20261116 refused)')
    ap.add_argument('--tune-seed', type=int, default=TUNE_SEED)
    ap.add_argument('--dev-seed', type=int, default=DEV_SEED)
    ap.add_argument('--n', type=int, default=None, help='boats per level (pilot, default 20) or per config (diag, default 4)')
    ap.add_argument('--n-dev', type=int, default=N_DEV)
    ap.add_argument('--levels', nargs='+', default=None, choices=list(LEVELS))
    ap.add_argument('--configs', nargs='+', default=['none:0', 'all:1'], help="diag: 'mech[+mech]:lambda', 'all:x', 'none:0'")
    ap.add_argument('--procs', type=int, default=1)
    ap.add_argument('--K', type=int, default=K_HID)
    ap.add_argument('--M', type=int, default=M_TWIN)
    ap.add_argument('--mechs', nargs='+', default=['all'], choices=['all'] + list(MECHS),
                    help='pilot: mechanisms switched on at each level (default all seven)')
    ap.add_argument('--smoke', action='store_true', help='pilot: 1 eval + 1 dev boat on none/far, M=4, K=2')
    a = ap.parse_args(argv)
    if not 1 <= a.procs <= 4:
        raise SystemExit('compute policy: 1..4 processes')
    check_seed(a.seed, a.tune_seed, a.dev_seed)
    out = Path(a.out)
    man = {'argv': vars(a), 'provenance': _paths.provenance(), 'hidden_ranges_sha256': hidden.ranges_sha(),
           'mech_hi': HI, 'levels': LEVELS, 'excess': EXCESS, 'margin_s': MARGIN, 'cap_s': CAP, 'dock_drop': DOCK_DROP,
           'evidence_label': 'HSUR-OOF-EXPLORATORY', 'libraries': {k: [c.label() for c in v] for k, v in LIBS.items()},
           'started': time.strftime('%Y-%m-%dT%H:%M:%S')}
    if a.mode == 'diag':
        n = a.n or 4
        jobs = []
        for c in a.configs:
            mechs, lam = parse_config(c)
            jobs += boats(n, a.tune_seed, lam, mechs, c, 'diag', a.K, a.M)
        rows = run_jobs(jobs, out, a.procs, man)
        s = summarize_diag(rows)
        (out / 'summary.json').write_text(json.dumps(record._clean(s), indent=1, sort_keys=True))
        print_diag(s)
        return s
    if a.smoke:
        a.levels, a.K, a.M = a.levels or ['none', 'far'], 2, 4
        n, n_dev = 1, 1
    else:
        a.levels = a.levels or list(LEVELS)
        n, n_dev = a.n or N_EVAL, a.n_dev
    mechs = MECHS if 'all' in a.mechs else tuple(a.mechs)
    jobs = []
    for lv in a.levels:
        jobs += boats(n_dev, a.dev_seed, LEVELS[lv], mechs, lv, 'dev', a.K, a.M)
    for lv in a.levels:
        jobs += boats(n, a.seed, LEVELS[lv], mechs, lv, 'eval', a.K, a.M)
    rows = run_jobs(jobs, out, a.procs, man)
    s = summarize(rows)
    (out / 'summary.json').write_text(json.dumps(record._clean(s), indent=1, sort_keys=True))
    print_summary(s)
    return s


if __name__ == '__main__':
    main()
