#!/usr/bin/env python3
"""Replay the pulses logged by lockstep_openloop.py on the NumPy plant and on the Python twin of Unity's
discrete planar model, and compare with what Unity reported (PARITY.md, tasks B and C).

Layers replayed (all from the Unity-reported state right after ``reset``, same per-tick pulses):
  np_snap      plant.Plant(representable_numpy(boat)), SNAP_FRAC = 1e-3 (the frozen harness)
  np_nosnap    the same, SNAP_FRAC = 0
  np_nosnap_noair  the same, and no air drag (NumPy applies wind drag on the boat's own motion even in still
               air; Unity attaches no wind at all when the scenario wind speed is 0) -- only for env 'none'
  twin         Tools/Characterization/plant.py::ProfilePlant (the exact twin of MarinePlanarModel) on
               profile.json + faults.json (ActuatorErrorProfile.Apply in Python) + Unity's mean wind
               (RoboBoatWind.Force, mean speed, no gust/veer, applied to M_RB only as Unity does)

    nice -n 19 python3 -I Tools/DecisionCal/parity/openloop_compare.py LABEL [LABEL ...]
"""

import copy
import json
import math
import sys
from pathlib import Path

import numpy as np

DC = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DC))
sys.path.insert(0, str(DC.parent))                  # Tools/ (Characterization)

import _paths  # noqa: E402,F401
import plant  # noqa: E402
import unity_check as U  # noqa: E402
from plant import DT, Plant  # noqa: E402
from Characterization.plant import ProfilePlant  # noqa: E402

ROOT = Path.home() / '.cache' / 'decisioncal' / 'parity' / 'unity'
AIR, CD, A_FRONT, A_SIDE, LEVER_BOW = 1.225, 1.0, 0.30, 0.40, 0.05


def load(label):
    d = ROOT / label
    meta = json.loads((d / 'session.json').read_text())
    z = np.load(d / 'sequences.npz')
    seqs = {}
    for k in z.files:
        n, f = k.split('__')
        seqs.setdefault(n, {})[f] = z[k]
    return d, meta, seqs


def apply_faults(profile, faults):
    """Python of ActuatorErrorProfile.Apply (the fields unity_config writes)."""
    p = copy.deepcopy(profile)
    for i, name in enumerate(faults['names']):
        t = next(t for t in p['thrusters'] if t['name'] == name)
        s = faults['strengths'][i]
        t['kForward'] *= s
        t['kReverse'] = t['kForward'] * faults['reverse'][i]
        t['deadbandUs'] += faults['deadband_us'][i]
        t['deadTime'] += faults['deadtime_s'][i]
        t['tauT'] = faults['spinup']
    return p


def numpy_spec(meta):
    st, idx = {'transient': ('transient', 0), 'steady': ('steady', 0), 'mixed': ('mixed', 0),
               'timing': ('timing', 0)}[meta['boat']]
    spec = U.representable_numpy(U.get_boat(4242, st, idx))
    sc = meta.get('scenario') or {'cur_speed': 0, 'cur_toward_deg': 0, 'wind_speed': 0, 'wind_toward_deg': 0}
    if meta['mode'] == 'fake':
        if meta['env'] == 'none':
            sc = dict.fromkeys(sc, 0.0)
        elif meta['env'] == 'wind':
            spec0 = spec
            sc = {'cur_speed': 0.0, 'cur_toward_deg': 0.0,
                  'wind_speed': float(np.hypot(*spec0.wind[0])),
                  'wind_toward_deg': float(np.degrees(np.arctan2(spec0.wind[0, 1], spec0.wind[0, 0])))}
    cur = sc['cur_speed'] * np.array([math.cos(math.radians(sc['cur_toward_deg'])), math.sin(math.radians(sc['cur_toward_deg']))])
    wind = sc['wind_speed'] * np.array([math.cos(math.radians(sc['wind_toward_deg'])), math.sin(math.radians(sc['wind_toward_deg']))])
    return spec.with_(current=cur[None], wind=wind[None]), cur, wind


def replay_numpy(spec, state0, q, snap, air=True):
    old, old_w = plant.SNAP_FRAC, plant.wind_wrench
    plant.SNAP_FRAC = 1e-3 if snap else 0.0
    if not air:
        plant.wind_wrench = lambda P, st, w, n: np.zeros((n, 3))
    try:
        p = Plant(spec, np.array([1], np.uint64), state0=state0[None], t_max=len(q) * DT + 1)
        out, T = [state0.copy()], [np.zeros(4)]
        for k in range(len(q)):
            p.step(q[k][None])
            out.append(p.st[0].copy())
            T.append(p.thr.T[0].copy())
    finally:
        plant.SNAP_FRAC, plant.wind_wrench = old, old_w
    return np.array(out), np.array(T)


def replay_twin(profile, state0, pulses, cur, wind):
    tw = ProfilePlant(profile, DT, current=tuple(cur))
    x, y, psi, u, v, r = state0
    c, s = math.cos(psi), math.sin(psi)
    tw.reset((x, y, psi, c * u - s * v, s * u + c * v, r))
    m, iz = profile['mass'], profile['izz']
    out, T = [np.array(state0, float)], [np.zeros(4)]
    ws = float(np.hypot(*wind))
    for k in range(len(pulses)):
        x, y, psi, vx, vy, r = tw.state
        F = np.zeros(2)
        if ws > 0:                                   # RoboBoatWind.Force at the mean wind (no gust, no veer)
            rel = np.asarray(wind) - np.array([vx, vy])
            sp = float(np.hypot(*rel))
            if sp >= 1e-4:
                beta = math.atan2(rel[1], rel[0]) - psi
                area = A_FRONT * abs(math.cos(beta)) + A_SIDE * abs(math.sin(beta))
                F = 0.5 * AIR * CD * area * sp * rel
        tw.step_pwm(list(pulses[k]))
        st = tw.state
        if ws > 0:                                   # direct force on M_RB, applied with the plant's (symplectic)
            st[3] += DT * F[0] / m
            st[4] += DT * F[1] / m
            cpsi, spsi = math.cos(psi), math.sin(psi)
            fy_body = -spsi * F[0] + cpsi * F[1]
            st[5] += DT * LEVER_BOW * fy_body / iz
            st[0] += DT * DT * F[0] / m
            st[1] += DT * DT * F[1] / m
        x, y, psi, vx, vy, r = st
        c, s = math.cos(psi), math.sin(psi)
        out.append(np.array([x, y, U.wrap(psi), c * vx + s * vy, -s * vx + c * vy, r]))
        T.append(np.array([th.T for th in tw.thrusters]))
    return np.array(out), np.array(T)


def residual_force(profile, ref, pulses, cur):
    """One-step residual of Unity against the exact twin of its planar model: from Unity's reported state at each
    tick, step the twin with that tick's thrust (thruster states carried open loop on the same pulses) and compare
    the next body velocity with Unity's. F_res = M_eff dv / dt [N, N, N m] is everything outside the planar model
    (waves through buoyancy, buoyancy leakage through the added-mass coupling, roll/pitch coupling, thrust
    submersion beta < 1, contacts; and unmodelled wind if any)."""
    tw = ProfilePlant(profile, DT, current=tuple(cur))
    taus = [tw.thrust(list(p)) for p in pulses]
    am = profile['addedMass']
    meff = np.array([profile['mass'] + am['a11'], profile['mass'] + am['a22'], profile['izz'] + am['a66']])
    F = []
    for k in range(len(pulses)):
        x, y, psi, u, v, r = ref[k]
        c, s_ = math.cos(psi), math.sin(psi)
        tw.state = [x, y, psi, c * u - s_ * v, s_ * u + c * v, r]
        tw.step_wrench(taus[k])
        _, _, psi1, vx, vy, r1 = tw.state
        c1, s1 = math.cos(ref[k + 1][2]), math.sin(ref[k + 1][2])
        pred = np.array([c1 * vx + s1 * vy, -s1 * vx + c1 * vy, r1])
        F.append(meff * (ref[k + 1][3:6] - pred) / DT)
    F = np.array(F)
    k = max(1, int(round(0.5 / DT)))                 # 0.5 s moving average (tick-level quantisation is float32)
    Fs = np.array([np.convolve(F[:, j], np.ones(k) / k, mode='valid') for j in range(3)]).T
    return {'mean': F.mean(0).tolist(), 'rms_0.5s': np.sqrt((Fs ** 2).mean(0)).tolist(),
            'max_abs_0.5s': np.abs(Fs).max(0).tolist()}


def metrics(ref, got):
    dp = np.hypot(ref[:, 0] - got[:, 0], ref[:, 1] - got[:, 1])
    dpsi = np.abs(U.wrap_arr(ref[:, 2] - got[:, 2])) if hasattr(U, 'wrap_arr') else np.abs((ref[:, 2] - got[:, 2] + np.pi) % (2 * np.pi) - np.pi)
    dvel = np.abs(ref[:, 3:6] - got[:, 3:6]).max(1)
    first = lambda m, th: (float(np.argmax(m > th) * DT) if (m > th).any() else None)
    return {'max_dpos_m': float(dp.max()), 'final_dpos_m': float(dp[-1]), 'max_dpsi_deg': float(np.degrees(dpsi.max())),
            'max_dvel': float(dvel.max()), 't_first_dpos_gt_0.03': first(dp, 0.03), 't_first_dpos_gt_0.10': first(dp, 0.10)}


def summary_of(ref):
    return {'disp_m': float(np.hypot(ref[-1, 0] - ref[0, 0], ref[-1, 1] - ref[0, 1])),
            'dpsi_deg': float(np.degrees(ref[-1, 2] - ref[0, 2])), 'final_speed': float(np.hypot(ref[-1, 3], ref[-1, 4])),
            'final_r': float(ref[-1, 5]), 'max_speed': float(np.hypot(ref[:, 3], ref[:, 4]).max())}


def compare(label):
    d, meta, seqs = load(label)
    spec, cur, wind = numpy_spec(meta)
    prof = apply_faults(json.loads((d / 'profile.json').read_text()), json.loads((d / 'faults.json').read_text())) \
        if (d / 'profile.json').exists() else None
    res = {'label': label, 'boat': meta['boat'], 'env': meta['env'], 'current': cur.tolist(), 'wind': wind.tolist(),
           'sequences': {}}
    for name, s in seqs.items():
        ref = s['course']
        n = len(s['q'])
        ref = ref[:n + 1]
        q = (s['pulses'] - 1500.0) / 400.0
        x0 = ref[0].copy()
        row = {'unity': summary_of(ref), 'n_ticks': int(n),
               'reset_state': x0.tolist(), 'tick_steps_ok': bool(np.all(np.diff(s['tick'][:n + 1]) == 1)),
               't_vs_tick_max_err': float(np.abs(s['t'][:n + 1] - s['t'][0] - (np.arange(n + 1) * np.float32(DT))).max())}
        layers = {'np_snap': replay_numpy(spec, x0, q, True), 'np_nosnap': replay_numpy(spec, x0, q, False)}
        if meta['env'] == 'none':
            layers['np_nosnap_noair'] = replay_numpy(spec, x0, q, False, air=False)
        if prof is not None:
            layers['twin'] = replay_twin(prof, x0, s['pulses'], cur, wind)
        for k, (st, T) in layers.items():
            row[k] = {**metrics(ref, st), **{'final_' + kk: vv for kk, vv in summary_of(st).items()},
                      'max_thrust_N': float(np.abs(T).max())}
        if prof is not None and meta['mode'] == 'live':
            row['unity_residual_force'] = residual_force(prof, ref, s['pulses'], cur)
        res['sequences'][name] = row
    out = d / 'compare.json'
    out.write_text(json.dumps(res, indent=1))
    return res


def table(res):
    print(f"== {res['label']} boat {res['boat']} env {res['env']} current {np.round(res['current'], 3).tolist()} wind {np.round(res['wind'], 3).tolist()}")
    print(f"{'sequence':18s} {'unity disp/dpsi/v':>22s} | " + ' | '.join(f'{k:>16s}' for k in ('np_snap', 'np_nosnap', 'np_nosnap_noair', 'twin')))
    for n, r in res['sequences'].items():
        u = r['unity']
        cells = []
        for k in ('np_snap', 'np_nosnap', 'np_nosnap_noair', 'twin'):
            cells.append(f"{r[k]['max_dpos_m']:6.3f}m {r[k]['max_dpsi_deg']:5.1f}d" if k in r else ' ' * 16)
        rf = r.get('unity_residual_force')
        extra = '' if rf is None else ' | Fres mean ' + ' '.join(f'{v:+.2f}' for v in rf['mean']) + ' rms ' + ' '.join(f'{v:.2f}' for v in rf['rms_0.5s']) + ' max ' + ' '.join(f'{v:.1f}' for v in rf['max_abs_0.5s'])
        print(f"{n:18s} {u['disp_m']:7.3f} {u['dpsi_deg']:7.1f} {u['final_speed']:6.3f} | " + ' | '.join(cells) + extra)


if __name__ == '__main__':
    for lab in sys.argv[1:]:
        table(compare(lab))
