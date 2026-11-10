#!/usr/bin/env python3
"""EXPLORATORY. Step 1 of the fidelity-gap study: the paired table (boat, controller, seed) -> outcome in the
integrated layer (Unity, stored X1/X2 summaries, read only) and in the fast layer (NumPy), with boat,
controller and fast-trajectory interaction features.

Unity side: ~/.cache/decisioncal/unity-check/x1-*, x2-* summaries, unchanged. No Unity is run.
NumPy side: the stored `numpy_representable` / `numpy_hidden` metrics in the same summaries. The stored data
has no fast-layer trajectories, so the same 128 missions are re-flown once per fast variant with
`mission.run(..., log_every=1)` (unedited harness, 16 batched calls of 16 envs, about 13 s each) and every
re-flown J is asserted equal to the stored one. That is re-computation for exact pairing, not a new result.

The interaction features below were written down from parity/PARITY.md's mechanism list BEFORE any of them
was correlated with a Unity outcome (see README "Feature provenance"). They use the fast layer's trajectory
and the boat's declared parameters only; nothing from the Unity run enters a feature.

    ulimit -v 1500000; nice -n 19 python3 -I Tools/DecisionCal/studies/fidelity_gap/build_table.py
"""

import csv
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DC = HERE.parents[1]
sys.dont_write_bytecode = True
sys.path.insert(0, str(DC))

import _paths  # noqa: E402,F401
import mission  # noqa: E402
import plant  # noqa: E402
import unity_check as U  # noqa: E402
from library import DOCK, LIBRARY  # noqa: E402
from twin import CONFIG, NOMINAL  # noqa: E402

UC = Path.home() / '.cache' / 'decisioncal' / 'unity-check'
OUT = HERE / 'out'
HIDDEN_SEED = 4242
BOATS = ('steady', 'timing', 'transient', 'mixed')
BLOCKS = [('steady', 1, 'x1-steady-c{:02d}'), ('timing', 1, 'x1-timing-c{:02d}'),
          ('transient', 1, 'x1-transient-c{:02d}'), ('transient', 2, 'x2-transient-s2-c{:02d}'),
          ('transient', 3, 'x2-transient-s3-c{:02d}'), ('mixed', 1, 'x1-mixed-c{:02d}'),
          ('mixed', 2, 'x2-mixed-s2-c{:02d}'), ('mixed', 3, 'x2-mixed-s3-c{:02d}')]
C = len(LIBRARY)
V_HOLD = 0.08                 # mission.run dock-hold speed threshold, m/s
F_SWAY_UNITY = 5.0            # N, scale of Unity's unmodelled sideways forcing (PARITY.md: 1-8 N RMS)
RHO_AIR, A_LAT = 1.225, 0.40  # Unity lateral projected area (PARITY.md table)
OUTCOMES = ('docked', 't_dock', 'contacts', 'J')


def boat_descriptors(hid):
    """Mechanics descriptors of the hidden boat (one row spec). Ratios are to the twin's nominal card."""
    r = {k: np.asarray(v) for k, v in hid.row(0).items()}
    lag = 0.5 * (r['tau_up'] + r['tau_down'])
    return {
        'b_dead_time': float(r['dead_time']), 'b_odom_latency': float(r['odom_latency']),
        'b_latency_total': float(r['dead_time'] + r['odom_latency']), 'b_lag': float(lag),
        'b_tau_up': float(r['tau_up']), 'b_tau_down': float(r['tau_down']),
        'b_deadband': float(r['deadband'].mean()), 'b_deadband_max': float(r['deadband'].max()),
        'b_rev_hold': float(r['rev_hold']), 'b_inertia_ratio': float(r['mass'] / CONFIG['m_rb']),
        'b_m11_ratio': float((r['mass'] + r['a11']) / NOMINAL['m11']),
        'b_m22_ratio': float((r['mass'] + r['a22']) / NOMINAL['m22']),
        'b_d_surge_ratio': float(r['d1'][0] / NOMINAL['d1_u']), 'b_d_sway_ratio': float(r['d1'][1] / NOMINAL['d1_v']),
        'b_d_yaw_ratio': float(r['d1'][2] / NOMINAL['d1_r']),
        'b_thrust_scale': float(r['k_fwd'].mean() / CONFIG['k_ref']), 'b_rev_ratio': float((r['k_rev'] / r['k_fwd']).mean()),
        'b_gamma': float(r['gamma']), 'b_wind': float(np.hypot(*r['wind'])), 'b_gust_sigma': float(r['gust_sigma']),
        'b_current': float(np.hypot(*r['current'])), 'b_ou_sigma': float(r['ou_sigma']),
    }


def controller_descriptors(c):
    d = {'c_turn_alat': c.turn_alat, 'c_brake': c.brake, 'c_cruise': c.cruise, 'c_ff': float(c.feedforward),
         'c_latcomp': c.latcomp}
    for name in DOCK:
        d[f'c_dock_{name}'] = float(c.dock == name)
    return d


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def interaction_features(t, truth, q_req, m, spec_row, hid_row):
    """Interaction features of ONE fast-layer run.

    t [T] control times (25 Hz), truth [T, 6], q_req [T, 4]; m the run's metrics; spec_row the fast plant's
    parameters (representable or hidden); hid_row the hidden boat's (for the Unity deadband = hidden deadband,
    which the Unity player applies as a hard zero)."""
    dtc = plant.DT * mission.CONTROL_EVERY
    docked = bool(m['docked'])
    t_end = (m['t_dock'] + 2.0) if docked else mission.T_MAX
    live = t <= t_end + 1e-9
    x, y, psi = truth[:, 0], truth[:, 1], truth[:, 2]
    speed = np.hypot(truth[:, 3], truth[:, 4])
    tx, ty, tpsi = mission.TARGET
    pos_ok = (np.abs(y - ty) <= 0.25) & (x >= 2.4) & (x <= 3.0) & (np.abs(_wrap(psi - tpsi)) <= np.radians(15)) & live
    slow = speed <= V_HOLD
    # dock phase: on the return leg within 4 m of the target (where the library's dock schedule applies)
    leg = (y < -4.0) & (x <= tx + 4.0) & live
    first = int(np.argmax(leg)) if leg.any() else None
    phase = np.zeros_like(live)
    if first is not None:
        phase[first:] = live[first:]
    f = {}
    f['x_no_dock_phase'] = float(first is None)
    n_win = int(pos_ok.sum())
    f['x_t_in_window'] = n_win * dtc                                   # s inside the dock window before the hold completed
    f['x_t_slow_in_window'] = float((pos_ok & slow).sum() * dtc)       # s below the hold speed threshold in the window
    f['x_frac_slow_in_window'] = float((pos_ok & slow).sum() / n_win) if n_win else 0.0
    # hold attempts that broke: slow -> not slow while still in the window
    w = (pos_ok & slow).astype(int)
    breaks = int(((w[:-1] == 1) & (w[1:] == 0) & pos_ok[1:]).sum())
    f['x_hold_breaks'] = float(breaks)
    # knife-edge margin of the hold, in units of the threshold: + = comfortable, - = fast layer itself fails
    if docked:
        hold = (t >= m['t_dock'] - 1e-9) & (t <= m['t_dock'] + 2.0 + 1e-9)
        margin = V_HOLD - float(speed[hold].max())
    elif n_win:
        margin = V_HOLD - float(np.median(speed[pos_ok]))
    else:
        margin = V_HOLD - float(np.median(speed[live][-250:]))        # last 10 s
    f['x_hold_margin'] = margin / V_HOLD
    f['x_speed_p90_window'] = float(np.quantile(speed[pos_ok], 0.9) / V_HOLD) if n_win else float(np.quantile(speed[live][-250:], 0.9) / V_HOLD)
    f['x_dwell_before_hold'] = float((m['t_dock'] - t[np.argmax(pos_ok)]) if (docked and n_win) else f['x_t_in_window'])
    # commands in the dock phase
    q = q_req[phase]
    n_ph = len(q)
    dur = max(n_ph * dtc, dtc)
    aq = np.abs(q)
    sg = np.sign(np.where(aq < 1e-3, 0.0, q))
    rev = 0
    for j in range(4):
        s = sg[:, j][sg[:, j] != 0]
        rev += int((s[1:] != s[:-1]).sum())
    lag = 0.5 * (hid_row['tau_up'] + hid_row['tau_down'])
    f['x_rev_per_s'] = rev / 4.0 / dur                                 # command sign reversals per thruster per second
    f['x_rev_per_lag'] = f['x_rev_per_s'] * lag                        # reversals per thruster-lag time constant (hunting index)
    f['x_rev_hold_duty'] = f['x_rev_per_s'] * hid_row['rev_hold']      # share of time the hidden boat would sit in reversal hold
    db = np.asarray(hid_row['deadband'])                               # Unity hard deadband, per thruster, in q
    in_db = (aq > 1e-3) & (aq <= db[None, :])
    f['x_frac_cmd_in_deadband'] = float(in_db.mean()) if n_ph else 0.0
    # NumPy snap: from rest the lagged thrust never starts when a_up * k q^2 <= SNAP_FRAC * max k (PARITY.md F1)
    kf, kr = np.asarray(spec_row['k_fwd']), np.asarray(spec_row['k_rev'])
    a_up = 1.0 - np.exp(-plant.DT / max(spec_row['tau_up'], 1e-9)) if spec_row['tau_up'] > 0 else 1.0
    eps = plant.SNAP_FRAC * max(kf.max(), kr.max())
    e = np.maximum(aq - np.asarray(spec_row['deadband'])[None, :], 0.0) / (1.0 - np.asarray(spec_row['deadband'])[None, :])
    fstar = np.where(q >= 0, kf[None, :], kr[None, :]) * e ** spec_row['gamma']
    below_snap = (aq > 1e-3) & (a_up * fstar <= eps)
    f['x_frac_cmd_below_snap'] = float(below_snap.mean()) if n_ph else 0.0
    f['x_frac_cmd_dead_any'] = float((in_db | below_snap).mean()) if n_ph else 0.0
    f['x_cmd_min_over_deadband'] = float(np.median(aq.min(1)) / max(db.mean(), 1e-3)) if n_ph else 0.0
    f['x_cmd_med'] = float(np.median(aq)) if n_ph else 0.0
    # lateral force margin against the disturbance scale Unity adds (sway forcing + its wind model)
    B = plant.B
    Y_cmd = (np.where(q >= 0, kf[None, :], kr[None, :]) * np.sign(q) * e ** spec_row['gamma']) @ B[1] if n_ph else np.zeros(1)
    pos = sum(abs(B[1, j]) * (kf[j] if B[1, j] > 0 else kr[j]) for j in range(4))
    neg = sum(abs(B[1, j]) * (kr[j] if B[1, j] > 0 else kf[j]) for j in range(4))
    wind = float(np.hypot(*hid_row['wind']))
    F_dist = F_SWAY_UNITY + 0.5 * RHO_AIR * A_LAT * (1.25 * wind) ** 2
    f['x_lat_force_used'] = float(np.quantile(np.abs(Y_cmd), 0.9) / F_dist)
    f['x_lat_force_margin'] = float((min(pos, neg) - np.quantile(np.abs(Y_cmd), 0.9)) / F_dist)
    # speed the disturbance alone would induce in sway, against the hold threshold (boat-level)
    d1v, d2v = hid_row['d1'][1], hid_row['d2'][1]
    v_dist = (-d1v + np.sqrt(d1v ** 2 + 4 * d2v * F_SWAY_UNITY)) / (2 * d2v)
    f['x_dist_speed_over_hold'] = float(v_dist / V_HOLD)
    f['x_knife_edge'] = float(margin / v_dist)                         # hold margin in units of disturbance-induced speed
    f['x_current_over_hold'] = float(np.hypot(*hid_row['current']) / V_HOLD)
    return f


def unity_trace_stats(d):
    """Descriptive only (never a feature): Unity time in the dock window and share of it below the hold speed
    (0.5 s trace samples)."""
    a = np.loadtxt(d / 'trace.csv', delimiter=',', skiprows=1)
    if a.ndim != 2 or len(a) < 2:
        return {'u_t_in_window': float('nan'), 'u_frac_slow_in_window': float('nan')}
    tx, ty, tpsi = mission.TARGET
    ok = (np.abs(a[:, 2] - ty) <= 0.25) & (a[:, 1] >= 2.4) & (a[:, 1] <= 3.0) & (np.abs(_wrap(a[:, 3] - tpsi)) <= np.radians(15))
    sp = np.hypot(a[:, 4], a[:, 5])
    return {'u_t_in_window': float(ok.sum() * 0.5),
            'u_frac_slow_in_window': float((ok & (sp <= V_HOLD)).sum() / ok.sum()) if ok.any() else float('nan')}


def main():
    OUT.mkdir(exist_ok=True)
    rows = []
    specs = {b: U.get_boat(HIDDEN_SEED, b, 0) for b in BOATS}
    for boat, seed, pat in BLOCKS:
        hid = specs[boat]
        rep = U.representable_numpy(hid)
        bd = boat_descriptors(hid)
        stored = [json.loads((UC / pat.format(c) / 'summary.json').read_text()) for c in range(C)]
        for s in stored:
            assert s['complete'] and not s['water_invalid_searches'] and s['seed'] == seed
        feats = {}
        mets = {}
        for tag, spec, key in (('rep', rep, 'numpy_representable'), ('hid', hid, 'numpy_hidden')):
            m, st = mission.run(spec.repeat(C), np.arange(C), np.full(C, seed, np.uint64), log_every=1)
            Js = np.array([s[key]['J'] for s in stored])
            assert np.allclose(m['J'], Js, atol=1e-9), (boat, seed, tag)       # exact pairing with the stored value
            row = spec.row(0)
            hrow = hid.row(0)
            for c in range(C):
                mc = {k: float(m[k][c]) for k in ('docked', 't_dock', 'contacts', 'J')}
                mets[(tag, c)] = mc
                feats[(tag, c)] = interaction_features(st['t'], st['truth'][:, c], st['q_req'][:, c], mc, row, hrow)
            print(f'{boat} s{seed} {tag}: re-flown J equals stored (max |d| {np.abs(m["J"] - Js).max():.1e})', flush=True)
        for c in range(C):
            s = stored[c]
            r = {'boat': boat, 'seed': seed, 'controller': c, 'label': pat.format(c), 'ctrl_label': LIBRARY[c].label()}
            for k in OUTCOMES:
                r[f'u_{k}'] = float(s['metrics'][k])
            r.update(unity_trace_stats(UC / pat.format(c)))
            for tag in ('rep', 'hid'):
                for k in OUTCOMES:
                    r[f'{tag}_{k}'] = mets[(tag, c)][k]
                for k, v in feats[(tag, c)].items():
                    r[f'{tag}_{k}'] = v
            r.update(bd)
            r.update(controller_descriptors(LIBRARY[c]))
            rows.append(r)
    cols = list(rows[0])
    with open(OUT / 'paired_table.csv', 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow({k: ('' if isinstance(v, float) and np.isnan(v) else v) for k, v in r.items()})
    (OUT / 'paired_table.json').write_text(json.dumps(rows, indent=0, allow_nan=True))
    print(f'wrote {len(rows)} rows x {len(cols)} columns to {OUT / "paired_table.csv"}')


if __name__ == '__main__':
    main()
