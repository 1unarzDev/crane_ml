"""Feature table for the fit-bias analysis (numpy only; no mission simulations).

For every stored boat (M2, M4) and every fitted arm: the MAP twin (clipped as the particles are clipped),
its in-family effective parameters, and noise-free open-loop surge / yaw step responses of the twin and
of the hidden boat itself (plant.Plant, 40 s, one batch). Hidden truth in twin units comes from
study_oracle_twin.project (reused by import). Cached to ~/.cache/decisioncal/fitbias_features.pkl.

    cd /home/lunarz/worktrees/decisioncal && nice -n 19 python3 -I -W ignore \
        Tools/DecisionCal/analysis/fb_features.py
"""
import json
import os
import pickle
import sys
from pathlib import Path

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import numpy as np  # noqa: E402

import twin  # noqa: E402
import study_oracle_twin as sot  # noqa: E402
from plant import B, DT, Plant, PlantSpec  # noqa: E402

CACHE = os.path.expanduser('~/.cache/decisioncal')
RUNS = {'M4': 'm4-20261114', 'M2': 'm2-20261112'}
OUT = os.path.join(CACHE, 'fitbias_features.pkl')
T_SIM, EFF = 40.0, (0.5, 1.0)
ZERO_SENS = {'pos_sd': 0.0, 'vel_sd': 0.0, 'yaw_sd': 0.0, 'gyro_sd': 0.0}
AXES = {'surge': ('u', 0, 3), 'yaw': ('r', 2, 5)}      # name -> (suffix, B row, state index)
BLK = {'surge': ('m11', 'd1_u', 'd2_u'), 'sway': ('m22', 'd1_v', 'd2_v'), 'yaw': ('m66', 'd1_r', 'd2_r')}


def quiet(spec):
    n = spec.n
    return spec.with_(current=np.zeros((n, 2)), wind=np.zeros((n, 2)), ou_sigma=np.zeros(n),
                      spatial_amp=np.zeros(n), gust_sigma=np.zeros(n), **{k: np.zeros(n) for k in ZERO_SENS})


def fitted_particle(rec):
    """Clipped MAP twin parameters (1-element arrays) of one fit record, as twin.fit clips particles."""
    p = {}
    for b, names in BLK.items():
        m, d1, d2 = rec['block_map'][b]
        p[names[0]] = max(m, twin.BOUNDS[names[0]])
        p[names[1]], p[names[2]] = max(d1, 0.0), max(d2, 0.0)
    mp = rec['map']
    p.update(delay_s=mp['delay_s'], tau_s=mp['tau_s'], rev_hold_s=mp['rev_hold_s'], rev_ratio=mp['rev_ratio'],
             deadband_q=mp['deadband_q'])
    fitted_delay = ('@d' not in rec['arm']) and rec['arm'] in ('rigid+delay', 'joint')
    p['odom_frac'] = 0.5 if fitted_delay else 0.0
    raw = {b: list(rec['block_map'][b]) for b in BLK}
    return {k: np.array([float(v)]) for k, v in p.items()}, raw


def patterns():
    P = np.linalg.pinv(B)
    out = {}
    for ax, (_, row, _) in AXES.items():
        q = P[:, row]
        out[ax] = q / np.abs(q).max()
    return out


def run_steps(spec, pats):
    """Final value and 63% time of the step response for every env: {(axis, effort): (final, t63)}."""
    res = {}
    n = spec.n
    for ax, (_, _, si) in AXES.items():
        for eff in EFF:
            pl = Plant(spec, np.arange(n), t_max=T_SIM)
            q = np.tile(eff * pats[ax], (n, 1))
            nt = int(round(T_SIM / DT))
            tr = np.empty((nt, n))
            for k in range(nt):
                pl.step(q)
                tr[k] = pl.st[:, si]
            fin = tr[-int(2 / DT):].mean(0)
            frac = tr / np.where(fin == 0, np.nan, fin)[None]
            hit = frac >= 0.632
            first = np.where(hit.any(0), hit.argmax(0), nt - 1)
            f1 = frac[first, np.arange(n)]
            f0 = frac[np.maximum(first - 1, 0), np.arange(n)]
            w = np.where(f1 > f0, (0.632 - f0) / np.maximum(f1 - f0, 1e-12), 1.0)
            t63 = (first - 1 + np.clip(w, 0, 1) + 1) * DT
            res[(ax, eff)] = (fin, t63)
    return res


def analytic_tau(spec, ax, fin):
    """Linearised time constant m / (d1 + 2 d2 |v|) at the steady value of the 50% step."""
    j = {'surge': 0, 'yaw': 2}[ax]
    m = (spec.mass + spec.a11) if ax == 'surge' else (spec.izz + spec.a66)
    return m / (spec.d1[:, j] + 2.0 * spec.d2[:, j] * np.abs(fin))


def build():
    pats = patterns()
    boats, specs = [], []
    for ds, run in RUNS.items():
        for l in open(f'{CACHE}/{run}/plants.jsonl'):
            r = json.loads(l)
            boats.append((ds, r))
    # --- hidden
    hid = PlantSpec.concat([PlantSpec.build(1, **r['spec']) for _, r in boats])
    truth = []
    for i, (_, r) in enumerate(boats):
        p, info = sot.project(hid.take([i]), 0.0, 1)
        truth.append({**{k: float(v[0]) for k, v in p.items()}, 'c': info['c'], 'kbar': info['kbar']})
    # --- twins: one list of (boat index, arm)
    items, parts, raws = [], [], []
    for i, (ds, r) in enumerate(boats):
        for arm, rec in r['fits'].items():
            if 'map' not in rec:
                continue
            rec = dict(rec, arm=arm)
            p, raw = fitted_particle(rec)
            items.append((i, arm))
            parts.append(p)
            raws.append(raw)
    for i, (ds, r) in enumerate(boats):          # ORACLE in-family projection (diagnostic of the projection itself)
        p, info = sot.project(hid.take([i]), 0.0, 1)
        items.append((i, 'oracle-proj'))
        parts.append(p)
        raws.append(None)
    pt = {k: np.concatenate([p[k] for p in parts]) for k in parts[0]}
    tw = twin.to_spec(pt, sensors=ZERO_SENS)
    allspec = quiet(PlantSpec.concat([hid, tw]))
    nb = len(boats)
    print(f'{nb} boats, {len(items)} twins; stepping {allspec.n} envs', flush=True)
    st = run_steps(allspec, pats)
    out = {'boats': [], 'items': items, 'truth': truth, 'raw_block_map': raws}
    for i, (ds, r) in enumerate(boats):
        s = r['spec']
        out['boats'].append({
            'ds': ds, 'id': r['id'], 'stratum': r['stratum'], 'dead': s['dead_time'], 'odom': s['odom_latency'],
            'gamma': s['gamma'], 'pos_sd': s['pos_sd'], 'vel_sd': s['vel_sd'], 'yaw_sd': s['yaw_sd'],
            'gyro_sd': s['gyro_sd'], 'tau_up': s['tau_up'], 'tau_down': s['tau_down'], 'rev_hold': s['rev_hold'],
            'slew': s['slew'], 'kbar': truth[i]['kbar'], 's_eff': truth[i]['kbar'] / twin.K_REF,
            'rev_true': truth[i]['rev_ratio'], 'db_true': truth[i]['deadband_q'],
            'k_asym': float(np.std(np.array(s['k_fwd']) / np.mean(s['k_fwd']))),
            'fits': {}, 'libs': {lib: {'sel': {a: r['libs'][lib]['scores']['arms'][a]['sel'] for a in r['libs'][lib]['scores']['arms']},
                                      'regret': {a: r['libs'][lib]['scores']['arms'][a]['regret'] for a in r['libs'][lib]['scores']['arms']},
                                      'docked': r['libs'][lib]['hidden_docked']} for lib in r['libs']}})
    # hidden metrics
    def metrics(idx, spec_row_idx):
        m = {}
        for ax in AXES:
            for eff in EFF:
                fin, t63 = st[(ax, eff)]
                m[f'{ax}{int(eff * 100)}'] = float(fin[idx])
                if eff == 0.5:
                    m[f't63_{ax}'] = float(t63[idx])
            m[f'tau_{ax}'] = float(analytic_tau(allspec.take([idx]), ax, st[(ax, 0.5)][0][[idx]])[0])
        return m
    for i in range(nb):
        out['boats'][i]['hid'] = metrics(i, i)
        out['boats'][i]['hid']['lat'] = float(allspec.odom_latency[i])
        out['boats'][i]['hid']['dead'] = float(allspec.dead_time[i])
    for j, (i, arm) in enumerate(items):
        m = metrics(nb + j, nb + j)
        m['lat'] = float(allspec.odom_latency[nb + j])
        m['dead'] = float(allspec.dead_time[nb + j])
        m['delay'] = m['lat'] + m['dead']
        m['par'] = {k: float(v[0]) for k, v in parts[j].items()}
        m['raw'] = raws[j]
        m['frac_clipped'] = {'surge': 0.0, 'sway': 0.0, 'yaw': 0.0}
        out['boats'][i]['fits'][arm] = m
    for i, (ds, r) in enumerate(boats):
        for arm in out['boats'][i]['fits']:
            if arm in r['fits']:
                out['boats'][i]['fits'][arm]['frac_clipped'] = r['fits'][arm]['frac_clipped']
    return out


if __name__ == '__main__':
    d = build()
    with open(OUT, 'wb') as f:
        pickle.dump(d, f)
    print('saved', OUT)
