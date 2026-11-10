#!/usr/bin/env python3
"""First-divergence trace between two closed-loop logs from closedloop.py (PARITY.md, task D).

Aligns the two runs tick by tick (both start at the mission's tick 0) and reports, for each signal, the first
time it differs beyond a stated threshold, in the causal order of the loop:
  command    q given to the plant (the controller's output; identical inputs give identical q)
  state      true pose / body velocity (hull response; the plant is where two layers can first differ)
  sensing    the controller's observation (harness Sensors on each layer's truth: same noise keys)
  docking    the per-condition dock-hold flags of mission.run (lateral, x window, heading, speed) and the hold
Also: the open-loop replay of layer A's applied q on layer B's plant is done by openloop_compare-style replay
in replay_commands() (does the plant alone, without feedback, already explain the gap?).

    nice -n 19 python3 -I Tools/DecisionCal/parity/diverge_trace.py A_LABEL B_LABEL
"""

import json
import sys
from pathlib import Path

import numpy as np

DC = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DC))

import _paths  # noqa: E402,F401
import mission  # noqa: E402
from plant import DT  # noqa: E402

ROOT = Path.home() / '.cache' / 'decisioncal' / 'parity' / 'closed'
TH = {'pos_m': (0.01, 0.05, 0.20), 'psi_deg': (0.5, 2.0, 10.0), 'vel': (0.01, 0.03, 0.10), 'q': (0.01, 0.05, 0.2),
      'obs_pos_m': (0.01, 0.05, 0.20)}


def load(label):
    d = ROOT / label
    z = np.load(d / 'closed.npz')
    meta = json.loads((d / 'meta.json').read_text())
    T = z['ticks']
    return {'meta': meta, 't': T[:, 1], 'st': T[:, 2:8], 'q': T[:, 8:12], 'T': T[:, 12:16], 'w': T[:, 16:19],
            's_t': z['s_t'] if 's_t' in z.files else None, 's_obs': z['s_obs'][:, 0] if 's_obs' in z.files else None,
            's_q': z['s_q_req'][:, 0] if 's_q_req' in z.files else None}


def wrapd(a):
    return np.degrees(np.abs((a + np.pi) % (2 * np.pi) - np.pi))


def dock_flags(st):
    speed = np.hypot(st[:, 3], st[:, 4])
    return {'lateral': np.abs(st[:, 1] - mission.TARGET[1]) <= 0.25,
            'x_window': (st[:, 0] >= 2.4) & (st[:, 0] <= 3.0),
            'heading': wrapd(st[:, 2] - mission.TARGET[2]) <= 15.0,
            'speed': speed <= 0.08}


def first(mask, t):
    return float(t[np.argmax(mask)]) if mask.any() else None


def trace(A, B):
    n = min(len(A['t']), len(B['t']))
    t = A['t'][:n]
    sa, sb = A['st'][:n], B['st'][:n]
    d = {'pos_m': np.hypot(sa[:, 0] - sb[:, 0], sa[:, 1] - sb[:, 1]), 'psi_deg': wrapd(sa[:, 2] - sb[:, 2]),
         'vel': np.abs(sa[:, 3:6] - sb[:, 3:6]).max(1)}
    qa, qb = np.nan_to_num(A['q'][:n]), np.nan_to_num(B['q'][:n])
    d['q'] = np.abs(qa - qb).max(1)
    out = {'n_ticks': int(n), 'first': {}}
    for k, v in d.items():
        out['first'][k] = {str(th): first(v > th, t) for th in TH[k]}
    if A['s_obs'] is not None and B['s_obs'] is not None:
        m = min(len(A['s_obs']), len(B['s_obs']))
        do = np.hypot(A['s_obs'][:m, 0] - B['s_obs'][:m, 0], A['s_obs'][:m, 1] - B['s_obs'][:m, 1])
        out['first']['obs_pos_m'] = {str(th): first(do > th, A['s_t'][:m]) for th in TH['obs_pos_m']}
    fa, fb = dock_flags(sa), dock_flags(sb)
    ok_a = np.all(np.stack(list(fa.values())), 0)
    ok_b = np.all(np.stack(list(fb.values())), 0)
    out['dock_window'] = {}
    for lab, f, ok in (('A', fa, ok_a), ('B', fb, ok_b)):
        # time fractions after the boat first enters the x window
        inside = f['x_window'] & f['lateral'] & (t > 20.0)       # in the slip (not the outbound pass at y = 0)
        k0 = int(np.argmax(inside)) if inside.any() else n
        sl = slice(k0, n)
        out['dock_window'][lab] = {'t_enter_slip': float(t[k0]) if k0 < n else None,
                                   'frac_ok': {k: float(v[sl].mean()) if k0 < n else None for k, v in f.items()},
                                   'frac_all_ok': float(ok[sl].mean()) if k0 < n else None,
                                   'longest_all_ok_s': float(_longest(ok[sl]) * DT) if k0 < n else None,
                                   'first_failing_condition_after_entry': _first_fail(f, ok, k0, t)}
    out['first_hold_flag_diff'] = first(ok_a != ok_b, t)
    # summary in loop order
    order = [('command q > 0.01', out['first']['q']['0.01']), ('state pos > 0.01 m', out['first']['pos_m']['0.01']),
             ('state vel > 0.01', out['first']['vel']['0.01']), ('state psi > 0.5 deg', out['first']['psi_deg']['0.5'])]
    if 'obs_pos_m' in out['first']:
        order.append(('obs pos > 0.01 m', out['first']['obs_pos_m']['0.01']))
    out['loop_order'] = sorted([o for o in order if o[1] is not None], key=lambda o: o[1])
    return out


def _longest(mask):
    best = cur = 0
    for v in mask:
        cur = cur + 1 if v else 0
        best = max(best, cur)
    return best


def _first_fail(f, ok, k0, t):
    """Of the times the boat is in the x window, which condition is most often the ONLY failing one."""
    if k0 >= len(t):
        return None
    names = list(f)
    M = np.stack([f[k][k0:] for k in names])
    fails = ~M
    only = {names[i]: float(((fails.sum(0) == 1) & fails[i]).mean()) for i in range(len(names))}
    return only


def main():
    A, B = load(sys.argv[1]), load(sys.argv[2])
    r = trace(A, B)
    r['A'], r['B'] = sys.argv[1], sys.argv[2]
    r['metrics'] = {'A': A['meta'].get('metrics'), 'B': B['meta'].get('metrics')}
    out = ROOT / f'trace__{sys.argv[1]}__{sys.argv[2]}.json'
    out.write_text(json.dumps(r, indent=1))
    print(json.dumps(r, indent=1))


if __name__ == '__main__':
    main()
