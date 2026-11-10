#!/usr/bin/env python3
"""The same case on the NumPy plant, written as a planar ``.ctraj`` in the Unity run's WORLD frame, so Theater
can draw it next to the Unity run.

Which boat: by default ``unity_check.representable_numpy(get_boat(...))`` -- the hidden boat reduced to what the
Unity plant was given (mean current/wind only, gamma = 2 with Unity's matched k, one symmetric lag, no
deadband, reversal hold, slew, OU/spatial/gust). That is the like-for-like comparison (X1 compares Unity J with
it). ``--spec hidden`` uses the full hidden boat instead (the study's numpy plant as scored).

Trajectory: ``mission.run(spec, [c], [seed], log_every=1)`` streams the TRUE state (not the measurement) at
every control step (25 Hz, t = 0, 0.04, ...) until the episode ends (docked + 2 s hold, or 120 s). Course ->
world uses the start pose of the Unity run (its ``hello``, from RUN/summary.json, or ``--start``):
w = start + R(yaw0) c, yaw_w = yaw_c + yaw0. Columns robot.x, robot.y, robot.yaw (rad), robot.speed
(|(u, v)|, m/s); pose_dims = planar: the NumPy plant has no heave, pitch or roll, so Theater seats this boat
flat on the mean water level.

    python3 numpy_to_ctraj.py --unity-run ~/.cache/decisioncal/demo/runs/<run id> [--spec representable|hidden]
"""

import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import unity_check as U  # noqa: E402
import mission  # noqa: E402
from plant import DT  # noqa: E402
from theater import traj as T  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--unity-run', help='a capture.py run dir: reads hidden seed, stratum, boat, controller, seed, hello')
    ap.add_argument('--hidden-seed', type=int, default=4242)
    ap.add_argument('--stratum', default='steady')
    ap.add_argument('--boat-index', type=int, default=0)
    ap.add_argument('--controller', type=int, default=8)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--start', default=None, help='world start x,y,yaw (default: the Unity run hello)')
    ap.add_argument('--spec', choices=('representable', 'hidden'), default='representable')
    ap.add_argument('--out', default=None, help='default: <unity run>/numpy-<spec>.ctraj')
    ap.add_argument('--run-id', default=None)
    a = ap.parse_args(argv)
    s = {}
    if a.unity_run:
        s = json.loads((Path(a.unity_run) / 'summary.json').read_text())
        a.hidden_seed, a.stratum, a.boat_index = s['hidden_seed'], s['stratum'], s['boat_index']
        a.controller, a.seed = s['controller'], s['seed']
    start = tuple(float(v) for v in a.start.split(',')) if a.start else tuple(s['hello']) if s else (0.0, 0.0, 0.0)
    boat = U.get_boat(a.hidden_seed, a.stratum, a.boat_index)
    spec = U.representable_numpy(boat) if a.spec == 'representable' else boat
    m, streams = mission.run(spec, [a.controller], [a.seed], log_every=1)
    met = U.fmt(m)
    st = streams['truth'][:, 0, :]
    t = streams['t']
    hz = 1.0 / (DT * mission.CONTROL_EVERY)
    assert np.allclose(np.diff(t), 1.0 / hz, atol=1e-6), 'streams are not at a fixed rate'
    x0, y0, yaw0 = start
    c, sn = math.cos(yaw0), math.sin(yaw0)
    wx = x0 + c * st[:, 0] - sn * st[:, 1]
    wy = y0 + sn * st[:, 0] + c * st[:, 1]
    wyaw = (st[:, 2] + yaw0 + np.pi) % (2 * np.pi) - np.pi
    speed = np.hypot(st[:, 3], st[:, 4])
    run_id = a.run_id or f"numpy-{a.spec[:4]}-{a.stratum}{a.hidden_seed}b{a.boat_index}-c{a.controller:02d}-s{a.seed}"
    events = []
    gi = np.where((st[:-1, 0] < mission.GATE_X) & (st[1:, 0] >= mission.GATE_X))[0]
    if len(gi):
        i = int(gi[0]) + 1
        events.append({'t': float(t[i]), 'kind': 'gate', 'x': float(wx[i]), 'y': float(wy[i]), 'course_y': float(st[i, 1]),
                       'id': 'route.gate'})
    if met['docked']:
        i = min(len(t) - 1, int(round(met['t_dock'] * hz)))
        events.append({'t': met['t_dock'], 'kind': 'finish', 'x': float(wx[i]), 'y': float(wy[i]),
                       'note': 'docked: hold began (mission.py rule)'})
    rev = subprocess.run(['git', '-C', str(HERE), 'rev-parse', 'HEAD'], capture_output=True, text=True).stdout.strip()
    header = {
        'run_id': run_id, 'study': 'decisioncal-demo', 'iteration': 0, 'pose_dims': 'planar',
        'frame': {'type': 'enu', 'origin': 'world', 'course_to_world': list(start),
                  'note': 'world = the Unity run frame: w = start + R(yaw0) c'},
        'robot': {'model': 'decisioncal-numpy-plant', 'agents': ['robot']},
        'build': {'recorder': 'numpy-plant', 'plant': 'Tools/DecisionCal/plant.py (RK4, 50 Hz)', 'code_commit': rev,
                  'spec': a.spec, 'created': time.strftime('%Y-%m-%dT%H:%M:%S%z')},
        'water': {'mode': 'none', 'note': 'planar plant: no waves, heave, pitch or roll'},
        'outcome': 'finished' if met['docked'] else 'failed', 'J': met['J'], 'score': -met['J'],
        'score_metric': 'minus_J (Theater ranks higher scores first; J = mission.py score, lower is better)',
        'progress': met['progress'], 'valid': True, 'metrics': met,
        'params': {'controller': a.controller, 'noise_seed': a.seed, 'hidden_seed': a.hidden_seed, 'stratum': a.stratum,
                   'boat_index': a.boat_index, 'spec': a.spec},
        'samples': 'true state (not the measurement) at every control step, t = 0 .. end of episode'}
    out = Path(a.out) if a.out else (Path(a.unity_run) / f'{run_id}.ctraj' if a.unity_run else Path(f'{run_id}.ctraj'))
    sha = T.write(out, header, {'robot.x': wx.tolist(), 'robot.y': wy.tolist(), 'robot.yaw': wyaw.tolist(),
                                'robot.speed': speed.tolist()}, events, hz=hz, t0_s=0.0)
    print(json.dumps({'run_id': run_id, 'spec': a.spec, 'metrics': met, 'samples': len(t), 'hz': hz, 'start': start,
                      'out': str(out), 'sha256': sha}, indent=1))
    return out


if __name__ == '__main__':
    main()
