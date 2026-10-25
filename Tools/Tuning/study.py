"""Plant-variant x disturbance study of the velocity loop and goal approach under Nav2 limits.

Run: python3 -I study.py <matrix.yaml> <out.jsonl>. Every line carries the variant, the nav limits,
the seed and the git heads, so any row re-runs alone (``rerun(row)``).
"""
from __future__ import annotations

import hashlib
import json
import math
import subprocess
import sys
from dataclasses import asdict, replace

import numpy as np

sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))  # python3 -I drops it
import yaml

from loop import HW, Nav, characterize, run, wrap
from plant import Variant

ROUTE = [(6.0, 0.0, 0.0), (6.0, 4.0, math.pi / 2), (0.0, 4.0, math.pi), (0.0, 0.0, -math.pi / 2),
         (4.0, 0.0, 0.0), (1.0, 0.0, math.pi)]   # legs, a corner, a turn-around


def heads():
    out = {}
    for name, path in (('mhseals_hardware', HW), ('crane_tuning', '.')):
        try:
            out[name] = subprocess.run(['git', '-C', path, 'rev-parse', '--short', 'HEAD'],
                                       capture_output=True, text=True).stdout.strip()
        except OSError:
            out[name] = None
    return out


def reversals(x, h):
    """Sign reversals of x with hysteresis h."""
    state, n = 0, 0
    for value in x:
        s = 1 if value > h else -1 if value < -h else 0
        if s and state and s != state:
            n += 1
        if s:
            state = s
    return n


def step_metrics(variant, nav, hw, seed):
    """Per-axis step to the nav limit through the smoother: rise, overshoot, steady error,
    tracking lag behind the smoothed command, and the residual coast after the stop."""
    out = {}
    for axis, amp in ((0, nav.vx_max), (1, nav.vy_max), (2, nav.wz_max)):
        trace, _ = run(variant, nav, hw=hw, seed=seed, duration=24.0,
                       script=lambda t: [amp if (i == axis and 1 < t < 13) else 0 for i in range(3)])
        t, y, c = trace['t'], trace['vel'][:, axis], trace['cmd'][:, axis]
        hold = (t > 9) & (t < 13)
        ss = float(y[hold].mean())
        on = (t > 1) & (t < 13)
        # Lag: the shift (s) that best aligns the truth with the smoothed command.
        lags = np.arange(0, 41)
        err = [np.mean((y[on][k:] - c[on][:len(c[on]) - k]) ** 2) for k in lags]
        lag = float(lags[int(np.argmin(err))] * 0.05)
        after = t > 13
        pos = np.cumsum(y) * 0.05
        stop_i = np.argmax(after)
        cross = np.argmax(after & (np.abs(y) < 0.02 * max(amp, 1e-3)))
        coast = float(pos[cross] - pos[stop_i])
        commanded = float(np.sum(c[after]) * 0.05)  # distance the smoother itself plans to coast
        eff = trace['thruster']
        out[('surge', 'sway', 'yaw')[axis]] = {
            'steady': round(ss, 4), 'steady_err': round(amp - ss, 4),
            'overshoot': round(float((y[on].max() - ss) / max(abs(ss), 1e-6)), 3),
            'lag_s': lag, 'coast_excess': round(coast - commanded, 3),
            'peak_thruster': round(float(np.abs(eff).max()), 3),
            'cross_axis_peak': round(float(np.abs(trace['vel'][on][:, [i for i in range(3) if i != axis]]).max()), 3)}
    return out


def turn_metrics(variant, nav, hw, seed):
    """Combined surge + yaw at the limits (a carved turn): skid (sway) and yaw tracking."""
    trace, _ = run(variant, nav, hw=hw, seed=seed, duration=16.0,
                   script=lambda t: [nav.vx_max, 0, nav.wz_max] if 1 < t < 13 else [0, 0, 0])
    t, vel = trace['t'], trace['vel']
    hold = (t > 6) & (t < 13)
    return {'skid_rms': round(float(np.sqrt(np.mean(vel[hold, 1] ** 2))), 4),
            'yaw_err': round(float(nav.wz_max - vel[hold, 2].mean()), 4),
            'surge_err': round(float(nav.vx_max - vel[hold, 0].mean()), 4),
            'peak_thruster': round(float(np.abs(trace['thruster']).max()), 3)}


def route_metrics(variant, nav, hw, seed, route=ROUTE, limit=200.0):
    trace, info = run(variant, nav, hw=hw, seed=seed, goals=route, duration=limit)
    t, pose, vel, raw = trace['t'], trace['pose'], trace['vel'], trace['raw']
    done = info['goals_done']
    end = info['reached_at'][-1] if done == len(route) else limit
    active = t <= end
    speed = np.hypot(vel[:, 0], vel[:, 1])
    path = float(np.sum(np.hypot(*np.diff(pose[active, :2], axis=0).T)))
    straight = 0.0
    prev = np.zeros(2)
    for g in route[:done]:
        straight += float(np.hypot(g[0] - prev[0], g[1] - prev[1]))
        prev = np.array(g[:2])
    overshoot = []
    for i, g in enumerate(route[:done]):
        leg = trace['goal'] == i
        if not leg.any():
            continue
        start = pose[leg][0, :2]
        d_vec = np.array(g[:2]) - start
        n = d_vec / max(np.hypot(*d_vec), 1e-9)
        # After the goal is reached (next leg), how far past it along the approach does the boat go
        nxt = (trace['goal'] == i + 1) & (t < (info['reached_at'][i] + 6.0 if i < len(info['reached_at']) else 0))
        near = pose[leg | nxt, :2]
        past = (near - np.array(g[:2])) @ n
        overshoot.append(float(max(0.0, past.max())))
    minutes = max(end, 1e-6) / 60
    spin = (speed < 0.05) & (np.abs(vel[:, 2]) > 0.15) & active
    energy = float(np.sum(np.abs(trace['thruster'][active]) ** 2) * 0.05)
    return {'done': done, 'of': len(route), 'time_s': round(end, 1),
            'path_ratio': round(path / max(straight, 1e-6), 3),
            'overshoot_max': round(max(overshoot) if overshoot else float('nan'), 3),
            'yaw_rev_per_min': round(reversals(vel[active, 2], 0.05) / minutes, 2),
            'cmd_wz_rev_per_min': round(reversals(raw[active, 2], 0.02) / minutes, 2),
            'thr_rev_per_min': round(float(np.mean([reversals(trace['thruster'][active, j], 0.0625)
                                                     for j in range(4)])) / minutes, 2),
            'spin_s': round(float(spin.sum() * 0.05), 1),
            'sat_frac': round(float(np.mean(np.abs(trace['thruster'][active]) >= 0.999)), 4),
            'energy_e2s': round(energy, 1),
            'drift_estimate': [round(x, 3) for x in info['drift_estimate']]}


def hold_metrics(variant, nav, hw, seed, duration=60.0):
    """Station keeping at one goal (the pursuit holds xy; yaw free within tolerance)."""
    trace, info = run(variant, nav, hw=hw, seed=seed, goals=[(0.0, 0.0, 0.0)] * 1000, duration=duration)
    t, pose = trace['t'], trace['pose']
    late = t > duration / 3
    r = np.hypot(pose[late, 0], pose[late, 1])
    return {'rms_m': round(float(np.sqrt(np.mean(r ** 2))), 3), 'max_m': round(float(r.max()), 3),
            'drift_estimate': [round(x, 3) for x in info['drift_estimate']]}


TESTS = {'step': step_metrics, 'turn': turn_metrics, 'route': route_metrics, 'hold': hold_metrics}


def row(case, seed):
    variant = Variant(**case.get('variant', {}))
    nav = Nav(**case.get('nav', {}))
    hw = case.get('hw')
    if hw == 'characterized':  # the controller holds an ideal characterization of this plant
        ratio = float(np.mean(variant.rev) / np.mean(variant.fwd))
        hw = characterize(variant, round(min(1.0, max(0.5, ratio)), 3))
    elif isinstance(hw, dict) and hw.get('characterized'):  # plus overrides (e.g. a wrong ratio)
        hw = {**characterize(variant), **{k: v for k, v in hw.items() if k != 'characterized'}}
    result = {'case': case['name'], 'seed': seed, 'variant': asdict(variant), 'nav': asdict(nav), 'hw': hw}
    result['case_sha'] = hashlib.sha256(json.dumps(case, sort_keys=True).encode()).hexdigest()[:12]
    for test in case.get('tests', list(TESTS)):
        result[test] = TESTS[test](variant, nav, hw, seed)
    return result


def main(matrix_path, out_path):
    matrix = yaml.safe_load(open(matrix_path))
    base = matrix.get('base', {})
    git = heads()
    with open(out_path, 'a') as out:
        for case in matrix['cases']:
            merged = {'name': case['name'], 'tests': case.get('tests', matrix.get('tests')),
                      'variant': {**base.get('variant', {}), **case.get('variant', {})},
                      'nav': {**base.get('nav', {}), **case.get('nav', {})},
                      'hw': case.get('hw', base.get('hw'))}
            for seed in case.get('seeds', matrix.get('seeds', [0])):
                result = row(merged, seed)
                result['git'] = git
                out.write(json.dumps(result) + '\n')
                out.flush()
                print(case['name'], seed, {k: result[k] for k in merged['tests'] if k in result}, flush=True)


if __name__ == '__main__':
    main(*sys.argv[1:3])
