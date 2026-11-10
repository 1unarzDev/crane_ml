#!/usr/bin/env python3
"""Capture ONE lockstep Unity docking run for the demo renders, with a full-rate trajectory and a per-tick
controller-side log.

It reuses ``unity_check``'s machinery (Link, the boat -> Unity config map, ``player_argv``, ``UnityPlant``) and
``mission.run`` unedited (the same rebinding of ``mission.Plant`` that ``unity_check.run_mission`` does), with
three additions:

  * the player also records ``--crane-traj RUN/run.ctraj --crane-traj-hz 50`` (pose after physics, every tick);
  * the course is ``--crane-course-replay RUN/course_truth.in.json`` (route_truth.py: the route's gate buoys
    and slip walls as real scene objects; the course family / course seed flags are dropped so the replay
    is the only course source), and the player writes back what it built to RUN/course_truth.json;
  * ``LoggingUnityPlant`` logs, per physics tick: the command q the controller passed to ``step`` (q_req) and
    the clipped q, the four pulses written to the link, the player's tick and episode time on the state
    line that came back, the harness's own clock, the reported truth (world-ROS fields and the course-frame
    state) and the measurement the controller last read (the harness Sensors on the Unity truth: latency +
    noise) with the tick it was read at.

What "applied" means here: the lockstep protocol (CraneLockstepControl) has the controller send ``p FL FR RR
RL`` after reading state ``s`` at tick k; the player applies those pulses to its ESC/thruster model for the
physics step k -> k+1 and replies with ``s`` at tick k+1 (tick continuity is checked every step). The
protocol does NOT echo the thrust that resulted (Unity's thruster faults -- dead time, spin-up lag, deadband
-- sit between the pulse and the force), so "applied" in the sidecar means "delivered for the step that
produced the next state", not "force acting". q_applied in mission.run's streams is the clipped q.

    nice -n 10 python3 capture.py [--run-id ID] [--hidden-seed 4242 --stratum steady --boat-index 0
                                   --controller 8 --seed 1]

Outputs (RUN = ~/.cache/decisioncal/demo/runs/<run id>): run.ctraj (player, raw, from the first physics
step), run.mission.ctraj (the mission window, t = 0 at the first lockstep state, stamped header + events),
steps.csv (per tick), control.npz (mission.run streams at 25 Hz + the per-tick arrays), summary.json,
run_manifest.json (backend, build, scenario, seeds, controller, code revision, files + sha256),
course_truth.in.json / course_truth.json, profile.json / faults.json / scenario.json, launch.txt,
player.log, heavy.out, result.json.
"""

import argparse
import csv
import hashlib
import json
import math
import os
import shlex
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import unity_check as U  # noqa: E402  (also imports _paths: crane root on sys.path)
import mission  # noqa: E402
from library import LIBRARY  # noqa: E402
from plant import DT  # noqa: E402
from theater import traj as T  # noqa: E402

import route_truth  # noqa: E402

OUT_ROOT = Path.home() / '.cache' / 'decisioncal' / 'demo' / 'runs'
PLAYER_COMMIT = 'e27629bdc255b114a9ff7bb7ec8d164628f49c4d'      # branch nlc-lockstep; the prebuilt player
REFERENCE = Path.home() / '.cache' / 'decisioncal' / 'unity-check' / 'x1-steady-c08' / 'summary.json'
HULL_L, HULL_B = 1.063, 0.895      # the scene boat's bounds (course truth `vessel`, blastoise-bounds)


def sha256(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def git_rev(path):
    def g(*a):
        return subprocess.run(['git', '-C', str(path), *a], capture_output=True, text=True).stdout.strip()
    return {'commit': g('rev-parse', 'HEAD'), 'branch': g('rev-parse', '--abbrev-ref', 'HEAD'),
            'dirty_files': len([x for x in g('status', '--porcelain').splitlines() if x.strip()])}


class LoggingUnityPlant(U.UnityPlant):
    """UnityPlant that keeps one row per physics tick (see the module docstring)."""

    def __init__(self, *a, **kw):
        self.rows = []
        self.last_meas, self.last_meas_tick = None, -1
        self.last_raw = None
        super().__init__(*a, **kw)

    def _receive(self, first=False):
        tick, t_player, fields = self.link.read_state()
        if first:
            self.tick0 = tick
        elif tick != self.tick0 + self.tick:
            raise U.LinkError(f'tick mismatch: player {tick}, expected {self.tick0 + self.tick}')
        self.last_raw = (tick, t_player, fields)
        self.st = self.frame.world_to_state(*fields)[None, :]
        if self.tick % self.trace_every == 0:
            self.trace.append([self.t, *self.st[0].tolist()])

    def measure(self, channel=0):
        m = super().measure(channel)
        pose, vel, _ = m
        self.last_meas = np.concatenate([np.asarray(pose)[0], np.asarray(vel)[0]])
        self.last_meas_tick = self.tick
        return m

    def step(self, q):
        q_req = np.asarray(q, float).reshape(-1).copy()
        sent_tick = self.tick0 + self.tick
        sent_t = self.t
        st_before = self.st[0].copy()
        super().step(q)
        tick, t_player, fields = self.last_raw
        q_clip = self.q_applied.reshape(-1)
        meas = self.last_meas if self.last_meas is not None else np.full(6, np.nan)
        self.rows.append([self.tick, sent_tick, sent_t, *q_req, *q_clip, *(1500.0 + 400.0 * q_clip),
                          tick, t_player, self.t, *fields, *self.st[0], *st_before,
                          self.last_meas_tick, *meas, time.time() - self.wall0])


COLS = (['k', 'tick_sent', 't_sent', 'q_req_fl', 'q_req_fr', 'q_req_rr', 'q_req_rl', 'q_fl', 'q_fr', 'q_rr', 'q_rl',
         'pulse_fl', 'pulse_fr', 'pulse_rr', 'pulse_rl', 'tick_reply', 't_player', 't_harness',
         'world_x', 'world_y', 'world_yaw', 'world_vx', 'world_vy', 'world_r',
         'x', 'y', 'psi', 'u', 'v', 'r', 'x_prev', 'y_prev', 'psi_prev', 'u_prev', 'v_prev', 'r_prev',
         'meas_k', 'meas_x', 'meas_y', 'meas_psi', 'meas_u', 'meas_v', 'meas_r', 'wall_s'])


def run_mission(link, spec, ctrl, seed, deadline):
    holder = {}

    def factory(sp, seeds, t_max=mission.T_MAX, **_):
        holder['plant'] = LoggingUnityPlant(link, sp, seeds, t_max=t_max, deadline=deadline)
        return holder['plant']

    original = mission.Plant
    mission.Plant = factory
    try:
        m, streams = mission.run(spec, [ctrl], [seed], t_max=mission.T_MAX, lib=LIBRARY, log_every=1)
    finally:
        mission.Plant = original
        if 'plant' in holder:
            holder['plant'].close()
    return m, streams, holder['plant']


def capture_argv(out, port, a, truth_in):
    """unity_check.player_argv plus the course replay and the trajectory recorder; the generated-course flags
    are removed (the replay truth is the only course source)."""
    base = U.player_argv(out, port, a.seed, a)
    argv, i = [], 0
    while i < len(base):
        if base[i] in ('--crane-course-family', '--crane-course-seed'):
            i += 2
            continue
        argv.append(base[i])
        i += 1
    return argv + ['--crane-course-replay', str(truth_in), '--crane-traj', str(out / 'run.ctraj'),
                   '--crane-traj-hz', str(a.traj_hz), '--crane-source-commit', PLAYER_COMMIT]


# ---------------------------------------------------------------------------------------------- post

def align(ctraj_path, rows):
    """Offset (samples) of the raw ctraj against the per-tick reported world poses (the ctraj starts at the
    player's first physics step; the lockstep link connects ~6 s later). Row i is the state after i + 1 steps,
    so the returned offset o maps row i to sample o + i and the first lockstep state (mission t = 0) to sample
    o - 1. Returns ((o, rms m, max m), traj)."""
    t = T.read(ctraj_path)
    ag = t.agents()[0]
    cx, cy = np.array(t.columns[f'{ag}.x']), np.array(t.columns[f'{ag}.y'])
    wx = np.array([r[COLS.index('world_x')] for r in rows])
    wy = np.array([r[COLS.index('world_y')] for r in rows])
    n = len(wx)
    best = (None, math.inf, math.inf)
    for o in range(0, len(cx) - n + 1):
        d = np.hypot(cx[o:o + n] - wx, cy[o:o + n] - wy)
        rms = float(np.sqrt(np.mean(d ** 2)))
        if rms < best[1]:
            best = (o, rms, float(d.max()))
    return best, t


def hull_clearance(x, y, psi):
    """Least distance (m) from the scene boat's hull rectangle (1.063 x 0.895) to the slip walls and the gate
    poles, course frame, over the samples. Negative = overlap. Points sampled on the rectangle's area."""
    S = mission.SLIP
    a = np.linspace(-HULL_L / 2, HULL_L / 2, 23)
    b = np.linspace(-HULL_B / 2, HULL_B / 2, 19)
    A, B = np.meshgrid(a, b)
    A, B = A.ravel(), B.ravel()
    side, end, pole = math.inf, math.inf, math.inf
    for xi, yi, pi_ in zip(x, y, psi):
        c, s = math.cos(pi_), math.sin(pi_)
        px, py = xi + c * A - s * B, yi + s * A + c * B
        inx = (px >= S['x0']) & (px <= S['x1']) & (np.abs(py - S['y']) < 2.0)   # the slip, not the outbound leg
        if inx.any():
            side = min(side, float(np.min(S['half'] - np.abs(py[inx] - S['y']))))
        iny = np.abs(py - S['y']) <= S['half'] + S['wall']
        if iny.any() and yi < -3:
            end = min(end, float(np.min(px[iny] - S['x0'])))
        for gy in (mission.GATE_Y, -mission.GATE_Y):
            pole = min(pole, float(np.min(np.hypot(px - mission.GATE_X, py - gy))) - 0.229)
    return {'slip_side_wall_m': side, 'slip_end_wall_m': end, 'gate_pole_m': pole}


def write_mission_ctraj(path, t_raw, offset, n, header_extra, events):
    ag = t_raw.agents()[0]
    cols = {f'robot.{k}': list(t_raw.columns[f'{ag}.{k}'][offset:offset + n + 1])
            for k in ('x', 'y', 'z', 'yaw', 'pitch', 'roll', 'speed') if f'{ag}.{k}' in t_raw.columns}
    # Raw sample `offset` was recorded after the last warm-up physics step, BEFORE the link's reset put the boat
    # back on the start pose (the boat drifts ~0.5 m in the 6 s warm-up); the first reported lockstep state is the
    # start pose. Sample 0 therefore repeats sample 1 (the state after the first command, 0.02 s later, within
    # 1 mm of the start pose at rest).
    for v in cols.values():
        v[0] = v[1]
    h = {k: v for k, v in t_raw.header.items() if k not in ('ticks', 'duration_s', 'columns', 'box', 'schema', 'hz', 't0_s')}
    h.update(header_extra)
    h['robot'] = {'model': (t_raw.header.get('robot') or {}).get('model'), 'agents': ['robot']}
    specs = [T.Column(s.name.replace(f'{ag}.', 'robot.'), s.scale, s.offset) for s in t_raw.specs]
    return T.write(path, h, cols, events, hz=t_raw.hz, t0_s=0.0, specs=specs)


def post_traj(out, rows, res, start, run_id, a):
    """Align the raw ctraj to the per-tick log, write run.mission.ctraj, hull clearances and events into res."""
    # trajectory: align, trim to the mission, check geometry
    ok_traj = (out / 'run.ctraj').is_file()
    res['ctraj_raw'] = ok_traj
    if ok_traj:
        (off, rms, mx), t_raw = align(out / 'run.ctraj', rows)
        res['ctraj_align'] = {'offset_samples': off, 'rms_m': rms, 'max_m': mx, 'hz': t_raw.hz,
                              'raw_samples': t_raw.ticks,
                              'note': 'raw sample offset + i equals the reported state after i + 1 commands (rms above); '
                                      'mission t 0 = run.mission.ctraj sample 0 = raw sample offset - 1 slot, filled with '
                                      'sample 1 because that raw sample predates the link reset to the start pose'}
        # sample offset+k  <->  row k-1 (state after k steps) ; the first lockstep state is sample offset-1
        off0 = off - 1
        ag = t_raw.agents()[0]
        n = len(rows)
        wx = np.array(t_raw.columns[f'{ag}.x'][off0:off0 + n + 1])
        wx[0] = wx[1]
        wy = np.array(t_raw.columns[f'{ag}.y'][off0:off0 + n + 1])
        wy[0] = wy[1]
        wyaw = np.array(t_raw.columns[f'{ag}.yaw'][off0:off0 + n + 1])
        wyaw[0] = wyaw[1]
        dx, dy = wx - start[0], wy - start[1]
        c, s = math.cos(start[2]), math.sin(start[2])
        cx, cy, cpsi = c * dx + s * dy, -s * dx + c * dy, wyaw - start[2]
        res['hull_clearance_50hz'] = hull_clearance(cx, cy, cpsi)
        events = []
        gi = np.where((cx[:-1] < mission.GATE_X) & (cx[1:] >= mission.GATE_X))[0]
        if len(gi):
            i = int(gi[0]) + 1
            events.append({'t': i / t_raw.hz, 'kind': 'gate', 'x': float(wx[i]), 'y': float(wy[i]),
                           'course_y': float(cy[i]), 'id': 'route.gate'})
        met = res.get('metrics') or {}
        if met.get('docked'):
            i = int(round(met['t_dock'] * t_raw.hz))
            if 0 <= i < len(wx):
                events.append({'t': met['t_dock'], 'kind': 'finish', 'x': float(wx[i]), 'y': float(wy[i]),
                               'note': 'docked: hold began (mission.py rule)'})
        score = met.get('J')
        hdr = {'run_id': run_id, 'study': 'decisioncal-demo', 'iteration': 0,
               'outcome': 'finished' if met.get('docked') else 'failed', 'score': -score if score is not None else None,
               'score_metric': 'minus_J (higher is better for Theater; make_pack.py shows J itself in the HUD)',
               'J': score, 'progress': 1.0 if met.get('docked') else met.get('progress'), 'valid': True,
               'params': {'controller': a.controller, 'noise_seed': a.seed, 'hidden_seed': a.hidden_seed,
                          'stratum': a.stratum, 'boat_index': a.boat_index},
               'build': dict(t_raw.header.get('build') or {}, recorder='unity', player_commit=PLAYER_COMMIT),
               'course': {'replay': 'course_truth.in.json', 'name': 'decisioncal-route-v1'},
               'source': {'raw_ctraj': 'run.ctraj', 'raw_offset_samples': off0}}
        hdr['source']['sample0'] = 'repeats sample 1 (raw sample at offset predates the link reset to the start pose)'
        sha = write_mission_ctraj(out / 'run.mission.ctraj', t_raw, off0, n, hdr, events)
        res['mission_ctraj_sha256'] = sha
        res['events'] = events



def write_run_manifest(out, run_id, a, start, scenario, argv):
    files = {p.name: {'sha256': sha256(p), 'bytes': p.stat().st_size} for p in sorted(out.iterdir())
             if p.is_file() and p.name != 'run_manifest.json'}
    old = json.loads((out / 'run_manifest.json').read_text()) if (out / 'run_manifest.json').is_file() else {}
    old['files'] = files
    (out / 'run_manifest.json').write_text(json.dumps(old, indent=1))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--run-id', default=None)
    ap.add_argument('--hidden-seed', type=int, default=4242)
    ap.add_argument('--stratum', default='steady')
    ap.add_argument('--boat-index', type=int, default=0)
    ap.add_argument('--controller', type=int, default=8)
    ap.add_argument('--seed', type=int, default=1, help='noise seed (CRN key) and player --crane-seed')
    ap.add_argument('--traj-hz', type=float, default=50.0)
    ap.add_argument('--sim-profile', default='minimal')
    ap.add_argument('--time-scale', type=float, default=4.0)
    ap.add_argument('--player-wall', type=float, default=600.0, help='player --crane-duration (wall s from launch)')
    ap.add_argument('--mission-wall', type=float, default=150.0)
    ap.add_argument('--accept-timeout', type=float, default=1800.0, help='s to wait for heavy admission + load')
    ap.add_argument('--no-match', action='store_true')
    ap.add_argument('--no-env', action='store_true')
    ap.add_argument('--start', default=','.join(str(v) for v in route_truth.SCENE_START))
    ap.add_argument('--out-root', default=str(OUT_ROOT))
    ap.add_argument('--post', metavar='RUN_DIR', help='redo only the trajectory post-processing of an existing capture (no Unity)')
    a = ap.parse_args(argv)
    if a.post:
        out = Path(a.post)
        res = json.loads((out / 'summary.json').read_text())
        for k in ('hidden_seed', 'stratum', 'boat_index', 'controller', 'seed'):
            setattr(a, k, res[k])
        with open(out / 'steps.csv') as f:
            rows = [[float(x) for x in r] for r in list(csv.reader(f))[1:]]
        post_traj(out, rows, res, tuple(res['hello']), res['run_id'], a)
        res['post_redone'] = time.strftime('%Y-%m-%dT%H:%M:%S%z')
        (out / 'summary.json').write_text(json.dumps(res, indent=1))
        write_run_manifest(out, res['run_id'], a, None, None, argv)
        print(json.dumps({k: res.get(k) for k in ('ctraj_align', 'hull_clearance_50hz', 'events')}, indent=1))
        return res

    spec = U.get_boat(a.hidden_seed, a.stratum, a.boat_index)
    run_id = a.run_id or (f"unity-ls-{time.strftime('%Y%m%d')}-{a.stratum}{a.hidden_seed}b{a.boat_index}"
                          f"-c{a.controller:02d}-s{a.seed}")
    out = Path(a.out_root) / run_id
    if (out / 'summary.json').exists():
        sys.exit(f'{out} already has a summary; pick another --run-id')
    out.mkdir(parents=True, exist_ok=True)
    start = tuple(float(v) for v in a.start.split(','))
    truth_in = out / 'course_truth.in.json'
    truth_in.write_text(json.dumps(route_truth.build(start), indent=1) + '\n')
    scenario, lost = U.write_config(out, spec, 0, match=not a.no_match, env=not a.no_env)

    link = U.Link()
    argv_ = capture_argv(out, link.port, a, truth_in)
    (out / 'launch.txt').write_text(' '.join(shlex.quote(x) for x in argv_) + '\n')
    print('launch:', (out / 'launch.txt').read_text().strip(), flush=True)
    env = {k: v for k, v in os.environ.items() if k not in ('DISPLAY', 'WAYLAND_DISPLAY')}
    env.update({'HEAVY_WHO': os.environ.get('HEAVY_WHO', 'decisioncal-demo'), 'HEAVY_MAX_S': str(int(a.player_wall) + 120),
                'SDL_VIDEODRIVER': 'dummy'})
    t_launch = time.time()
    proc = subprocess.Popen(argv_, env=env, stdout=open(out / 'heavy.out', 'w'), stderr=subprocess.STDOUT,
                            start_new_session=True)
    res = {'run_id': run_id, 'backend': 'unity-lockstep', 'controller': a.controller, 'seed': a.seed,
           'hidden_seed': a.hidden_seed, 'stratum': a.stratum, 'boat_index': a.boat_index,
           'scenario': scenario, 'not_representable': lost}
    plant = None
    try:
        link.accept(a.accept_timeout, alive=lambda: proc.poll() is None)
        t_conn = time.time()
        res['admission_and_load_wall_s'] = t_conn - t_launch
        fr = link.hello
        res['hello'] = [fr.x0, fr.y0, fr.yaw0]
        res['hello_matches_truth_start'] = bool(np.allclose(res['hello'], start, atol=1e-3))
        link.scenario(scenario['cur_speed'], scenario['cur_toward_deg'], scenario['wind_speed'], scenario['wind_toward_deg'])
        try:
            m, streams, plant = run_mission(link, spec, a.controller, a.seed, time.time() + a.mission_wall)
            res['metrics'] = U.fmt(m)
            res['complete'] = True
        except U.WallLimit as e:
            res['complete'], res['wall_limit'] = False, str(e)
            streams = None
        res['mission_wall_s'] = time.time() - t_conn
    finally:
        link.close()
        try:
            proc.wait(90)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, 15)
        res['player_rc'] = proc.poll()
        res['launch_to_end_wall_s'] = time.time() - t_launch
    if (out / 'result.json').is_file():
        r = json.loads((out / 'result.json').read_text())
        res['water_invalid_searches'] = int(r.get('invalidWaterSearches') or 0)
    if plant is None or not plant.rows:
        (out / 'summary.json').write_text(json.dumps(res, indent=1))
        sys.exit('no mission data')

    rows = plant.rows
    with open(out / 'steps.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(COLS)
        w.writerows(rows)
    arr = np.array(rows, float)
    np.savez_compressed(out / 'control.npz', cols=np.array(COLS), steps=arr,
                        **({f'stream_{k}': v for k, v in streams.items()} if streams else {}))
    res['ticks'] = len(rows)
    res['t_final'] = float(plant.t)
    res['final_state_course'] = plant.st[0].tolist()

    post_traj(out, rows, res, start, run_id, a)

    if REFERENCE.is_file():
        ref = json.loads(REFERENCE.read_text())
        res['reference_x1_steady_c08'] = {'metrics': ref.get('metrics'), 'hello': ref.get('hello'),
                                          'note': 'same boat/controller/seed with the generated seed-1 course, no traj recorder'}
    ref_np, _ = mission.run(spec, [a.controller], [a.seed])
    res['numpy_hidden'] = U.fmt(ref_np)
    ref2, _ = mission.run(U.representable_numpy(spec), [a.controller], [a.seed])
    res['numpy_representable'] = U.fmt(ref2)
    (out / 'summary.json').write_text(json.dumps(res, indent=1))

    files = {}
    for p in sorted(out.iterdir()):
        if p.is_file() and p.name != 'run_manifest.json':
            files[p.name] = {'sha256': sha256(p), 'bytes': p.stat().st_size}
    manifest = {
        'schema': 'decisioncal-demo-run/1', 'run_id': run_id, 'created': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'backend': {'plant': 'unity-lockstep (CRANE C# boat plant, MarinePlanarModel + ArticulationBody, HDRP water)',
                    'player': str(U.PLAYER), 'player_build_commit': PLAYER_COMMIT, 'player_branch': 'nlc-lockstep',
                    'scene': U.SCENE, 'time_scale': a.time_scale, 'sim_profile': a.sim_profile, 'physics_dt': DT,
                    'controller_side': 'Tools/DecisionCal mission.run (unedited) via unity_check.UnityPlant'},
        'scenario': {'hidden_seed': a.hidden_seed, 'stratum': a.stratum, 'boat_index': a.boat_index,
                     'course': 'course_truth.in.json (route_truth.py: gate buoys + slip walls at the route)',
                     'start_world': list(start), 'env': scenario},
        'seeds': {'noise_seed': a.seed, 'player_crane_seed': a.seed},
        'controller': {'id': a.controller, 'library': 'library.LIBRARY (v1)', 'spec': repr(LIBRARY[a.controller])},
        'code': {'worktree': git_rev(HERE), 'crane_ml': git_rev(Path.home() / 'crane_ml')},
        'command': 'nice -n 10 python3 ' + ' '.join(shlex.quote(x) for x in [str(Path(__file__).resolve())] + list(sys.argv[1:] if argv is None else argv)),
        'files': files}
    (out / 'run_manifest.json').write_text(json.dumps(manifest, indent=1))
    print(json.dumps({k: res.get(k) for k in ('run_id', 'metrics', 'complete', 'hello', 'hello_matches_truth_start',
                                              'ctraj_align', 'hull_clearance_50hz', 'events', 'launch_to_end_wall_s',
                                              'water_invalid_searches', 'reference_x1_steady_c08', 'numpy_representable')},
                     indent=1))
    print('->', out)
    return res


if __name__ == '__main__':
    main()
