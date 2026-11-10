"""Shared pieces of the three writers: the standard channel table of SCHEMA_V2.md, the intermediate
record structure, header-block builders, and the dock-mission event reconstruction (the rules of
``mission.run``, re-applied to a record's truth so outcomes can be scored from ``truth.*`` + events).

Intermediate record (plain data that mirrors judge/SCHEMA_V2.md field for field)::

    {'schema': 'crane.judge.run/2',
     'header': {run_id, tier, source, domain, robot{id, model, morphology, plant_card}, task_id,
                scenario{id, pair_key, truth_sha256, seed, family, condition}, provenance, clock{...},
                window[t0, t1, reason], source_version,
                backend, controller, calibration, clock_mode, code, evidence, reset},
     'channels': {name: {fields, units, frame, observability, rate_hz, angles, t, v, tick}},
     'absent':   {name: reason},          # standard channels this backend cannot provide
     'events':   [{t, kind, actor, payload}],
     'caps': {}, 'truth': {}}

``provenance['infra']`` carries source-reported facts (player return code, link errors, water
searches) for ``judge.validity``; a source never writes Validity.
"""

from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
import math
import subprocess
from pathlib import Path

import numpy as np

import mission
from plant import DT

SCHEMA_V2 = 'crane.judge.run/2'
DC = Path(mission.__file__).resolve().parent
REPO = DC.parents[1]
Q4 = ('q0', 'q1', 'q2', 'q3')
THR_FRAME = 'thrusters.FL_FR_RR_RL'

# name -> (fields, units, frame, angles)
STANDARD = {
    'truth.pose': (('x', 'y', 'yaw'), ('m', 'm', 'rad'), 'scenario.enu', ('yaw',)),
    'truth.twist': (('u', 'v', 'r'), ('m/s', 'm/s', 'rad/s'), 'body.flu', ()),
    'obs.pose': (('x', 'y', 'yaw'), ('m', 'm', 'rad'), 'scenario.enu', ('yaw',)),
    'obs.twist': (('u', 'v', 'r'), ('m/s', 'm/s', 'rad/s'), 'body.flu', ()),
    'obs.stamp': (('t_acq', 't_recv'), ('s', 's'), 'record.clock', ()),
    'cmd.requested': (Q4, ('1',) * 4, THR_FRAME, ()),
    'cmd.sent': (('p0', 'p1', 'p2', 'p3', 't_send'), ('us',) * 4 + ('s',), THR_FRAME, ()),
    'cmd.applied': (Q4 + ('t_apply',), ('1',) * 4 + ('s',), THR_FRAME, ()),
    'act.thrust': (('f0', 'f1', 'f2', 'f3'), ('N',) * 4, THR_FRAME, ()),
    'env.current': (('x', 'y'), ('m/s', 'm/s'), 'scenario.enu', ()),
    'env.wind': (('x', 'y'), ('m/s', 'm/s'), 'scenario.enu', ()),
}

# files whose sha256 goes into header.code.files_sha (the harness a numpy/lockstep run depends on)
HARNESS_FILES = ('mission.py', 'plant.py', 'law.py', 'library.py', 'library_v2.py', 'library_v3.py', 'hidden.py',
                 'twin.py', 'unity_check.py', '_paths.py', 'preplay/raw.py')


def channel(name, t, v, tick=None, observability='exact', rate_hz=None, fields=None, units=None, frame=None,
            angles=None, note=''):
    """One channel entry. Standard names take their fields/units/frame from ``STANDARD``."""
    if name in STANDARD:
        f0, u0, fr0, a0 = STANDARD[name]
    else:
        f0, u0, fr0, a0 = fields, units, frame, angles or ()
    fields, units = tuple(fields or f0), tuple(units or u0)
    t = np.asarray(t, float).reshape(-1)
    v = np.asarray(v, float).reshape(len(t), len(fields))
    if tick is not None:
        tick = np.asarray(tick, np.int64).reshape(-1)
        assert len(tick) == len(t), name
    return {'fields': fields, 'units': units, 'frame': frame or fr0, 'observability': observability,
            'rate_hz': rate_hz, 'angles': tuple(angles if angles is not None else a0), 'note': note, 't': t, 'v': v,
            'tick': tick}


def sha256_file(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def code_block(extra=()):
    """{repo, commit, dirty, files_sha}: harness + records files, relative to the crane root."""
    files = [DC / f for f in HARNESS_FILES] + sorted((DC / 'vendor').glob('*.py')) + sorted(
        (DC / 'records').glob('*.py')) + [Path(e) for e in extra]
    sha = {}
    for f in files:
        if f.is_file():
            try:
                key = str(f.resolve().relative_to(REPO))
            except ValueError:
                key = str(f)
            sha[key] = sha256_file(f)
    try:
        head = subprocess.run(['git', '-C', str(REPO), 'rev-parse', 'HEAD'], capture_output=True, text=True,
                              timeout=10).stdout.strip() or 'unknown'
        rel = [k for k in sha if not k.startswith('/')]
        dirty = bool(subprocess.run(['git', '-C', str(REPO), 'status', '--porcelain', '--'] + rel,
                                    capture_output=True, text=True, timeout=10).stdout.strip())
    except (OSError, subprocess.SubprocessError):
        head, dirty = 'unknown', True
    return {'repo': str(REPO), 'commit': head, 'dirty': dirty, 'files_sha': sha}


def library_of(name):
    """(library tuple, library id) for 'v1' | 'v2' | 'v3'."""
    if name in ('v1', 'library_v1', None):
        from library import LIBRARY
        return LIBRARY, 'decisioncal.library_v1'
    if name in ('v2', 'library_v2'):
        from library_v2 import LIBRARY_V2
        return LIBRARY_V2, 'decisioncal.library_v2'
    if name in ('v3', 'library_v3'):
        from library_v3 import LIBRARY_V3
        return LIBRARY_V3, 'decisioncal.library_v3'
    raise ValueError(f'unknown library {name!r}')


def controller_block(lib, lib_id, idx):
    c = lib[idx]
    params = asdict(c)
    kp_s = getattr(c, 'kp_scale', 1.0 if c.feedforward else 1.5)
    ki_s = getattr(c, 'ki_scale', 1.0)
    params['derived'] = {'kp': [kp_s * 0.8, kp_s * 0.8, kp_s * 0.6], 'ki': [ki_s * 0.6, ki_s * 0.6, ki_s * 0.4],
                         'control_hz': 1.0 / (DT * mission.CONTROL_EVERY), 'label': c.label() if hasattr(c, 'label') else None}
    full = hashlib.sha256(json.dumps(asdict(c), sort_keys=True).encode()).hexdigest()
    return {'id': f'c{idx:02d}', 'library': lib_id, 'spec_sha': full, 'params': params}


def calibration_block(cal=None):
    """``cal`` is passed by the caller (the twin that picked this controller); None -> all null."""
    cal = dict(cal or {})
    return {'twin': cal.get('twin'), 'arm': cal.get('arm'), 'evidence_sha': cal.get('evidence_sha'),
            'lineage': list(cal.get('lineage') or [])}


def mission_scenario(stratum, boat_index, hidden_seed, seed, t_max, lib_id, projection='hidden'):
    sid = f'dock-{stratum}{boat_index}-h{hidden_seed}'
    return {'id': sid, 'pair_key': f'{sid}:s{seed}', 'truth_sha256': None, 'seed': int(seed),
            'family': 'decisioncal.gate_uturn_dock',
            'condition': {'scenario_id': sid, 'start_pose': [0.0, 0.0, 0.0], 'stratum': stratum, 'boat_index': int(boat_index), 'hidden_seed': int(hidden_seed),
                          't_max_s': float(t_max), 'library': lib_id, 'plant_projection': projection}}


def short_sha(obj, n=8):
    return hashlib.sha256(json.dumps(obj, sort_keys=True, default=str).encode()).hexdigest()[:n]


def run_id(layer, scenario_id, controller_id, seed, identity):
    """``<layer>-<scenario>-<controller>-s<seed>-<shortsha>``; shortsha over the run's identity
    (backend, plant, controller, calibration, code), known before the run starts."""
    return f'{layer}-{scenario_id}-{controller_id}-s{seed}-{short_sha(identity)}'


def fast_mechanisms(row):
    """Mechanism names active in one PlantSpec row (``PlantSpec.row(i)``)."""
    m = ['rigid_body.fossen3dof.rk4', 'thrust.static_map', 'wind.drag']
    tu, td = row['tau_up'], row['tau_down']
    if tu > 0 or td > 0:
        m.append('thrust.lag.up_down' if abs(tu - td) > 1e-12 else 'thrust.lag.first_order')
    if row['rev_hold'] > 0:
        m.append('thrust.reversal_hold')
    if math.isfinite(row['slew']):
        m.append('thrust.slew')
    if any(d > 0 for d in row['deadband']):
        m.append('thrust.deadband.rescaled')
    if abs(row['gamma'] - 2.0) > 1e-12:
        m.append('thrust.exponent')
    if any(abs(a - b) > 1e-12 for a, b in zip(row['k_fwd'], row['k_rev'])):
        m.append('thrust.reverse_ratio')
    if row['dead_time'] >= DT / 2:
        m.append('cmd.dead_time')
    if row['odom_latency'] >= DT / 2:
        m.append('obs.latency')
    if any(row[k] > 0 for k in ('pos_sd', 'vel_sd', 'yaw_sd', 'gyro_sd')):
        m.append('obs.noise')
    if any(abs(c) > 0 for c in row['current']):
        m.append('current.mean')
    if row['ou_sigma'] > 0:
        m.append('current.ou')
    if row['spatial_amp'] > 0:
        m.append('current.spatial')
    if any(abs(w) > 0 for w in row['wind']):
        m.append('wind.mean')
    if row['gust_sigma'] > 0:
        m.append('wind.gust')
    return sorted(m)


def new_rec(header, channels, absent, events, provenance_extra=None):
    h = dict(header)
    h.setdefault('provenance', {})
    if provenance_extra:
        h['provenance'] = {**h['provenance'], **provenance_extra}
    for name in absent:
        assert name not in channels, name
    missing = [n for n in STANDARD if n not in channels and n not in absent]
    assert not missing, f'standard channels neither provided nor declared absent: {missing}'
    return {'schema': SCHEMA_V2, 'header': h, 'channels': dict(channels), 'absent': dict(absent),
            'events': list(events), 'caps': {}, 'truth': {}}


# ---------------------------------------------------------------------------------------------
# dock-mission events from truth (the rules of mission.run, applied after the fact)
# ---------------------------------------------------------------------------------------------

def _objects_hit(pose):
    """[N, 5] bool: footprint inside (wall_north, wall_south, end_wall, buoy_north, buoy_south);
    the any() of this equals ``mission._contacts`` (asserted by the caller)."""
    S, FOOT = mission.SLIP, mission.FOOT
    c, s = np.cos(pose[:, 2]), np.sin(pose[:, 2])
    px = pose[:, None, 0] + c[:, None] * FOOT[None, :, 0] - s[:, None] * FOOT[None, :, 1]
    py = pose[:, None, 1] + s[:, None] * FOOT[None, :, 0] + c[:, None] * FOOT[None, :, 1]
    w, y0, h = S['wall'], S['y'], S['half']
    in_x = (px >= S['x0']) & (px <= S['x1'])
    north = in_x & (py >= y0 + h) & (py <= y0 + h + w)
    south = in_x & (py <= y0 - h) & (py >= y0 - h - w)
    end = (px <= S['x0']) & (px >= S['x0'] - w) & (py >= y0 - h - w) & (py <= y0 + h + w)
    bn = np.hypot(px - mission.GATE_X, py - mission.GATE_Y) <= mission.BUOY_R
    bs = np.hypot(px - mission.GATE_X, py + mission.GATE_Y) <= mission.BUOY_R
    return np.stack([a.any(1) for a in (north, south, end, bn, bs)], -1)


OBJECTS = ('wall_north', 'wall_south', 'end_wall', 'buoy_north', 'buoy_south')


def mission_events(t, st, t_max=mission.T_MAX, rec_t=None):
    """Events and outcome of one dock mission from its truth.

    ``t`` [K+1]: the harness clock (``plant.t``) at ticks 0..K; ``st`` [K+1, 6]: (x, y, psi, u, v, r)
    in the course frame with body velocities, sample 0 = the start state. ``rec_t`` [K+1]: the
    record clock to stamp events with (default ``t``). Returns (events, outcome) where outcome has
    docked, t_dock, contacts, gate_miss, pose_err, J (``mission.score``), terminal and the ticks.
    The rules are mission.run's line for line (n = 1: the loop ends at the step that docks)."""
    t = np.asarray(t, float)
    st = np.asarray(st, float)
    rec_t = t if rec_t is None else np.asarray(rec_t, float)
    K = len(t) - 1
    T = mission.TARGET
    hit_any = mission._contacts(st)
    objs = _objects_hit(st)
    assert np.array_equal(objs.any(1), hit_any), 'contact geometry drifted from mission._contacts'
    ev = [{'t': float(rec_t[0]), 'kind': 'episode_start', 'actor': 'harness', 'payload': {'tick': 0}}]
    gate_y, contacts, in_contact, hold_since, docked, t_dock = math.nan, 0, False, math.nan, False, math.nan
    in_slip_prev = False
    for k in range(1, K + 1):
        x_prev, s = st[k - 1, 0], st[k]
        if math.isnan(gate_y) and x_prev < mission.GATE_X and s[0] >= mission.GATE_X:
            gate_y = float(s[1])
            ev.append({'t': float(rec_t[k]), 'kind': 'gate_cross', 'actor': 'boat',
                       'payload': {'y': gate_y, 'miss': bool(abs(gate_y) > mission.GATE_Y), 'tick': k}})
        hit = bool(hit_any[k])
        if hit and not in_contact:
            contacts += 1
            ev.append({'t': float(rec_t[k]), 'kind': 'contact', 'actor': 'boat',
                       'payload': {'with': [o for o, h in zip(OBJECTS, objs[k]) if h], 'n': contacts, 'tick': k}})
        in_contact = hit
        in_slip = (mission.SLIP['x0'] <= s[0] <= mission.SLIP['x1']) and abs(s[1] - mission.SLIP['y']) <= mission.SLIP['half']
        if in_slip and not in_slip_prev:
            ev.append({'t': float(rec_t[k]), 'kind': 'dock_enter', 'actor': 'boat', 'payload': {'tick': k}})
        in_slip_prev = in_slip
        speed = float(np.hypot(s[3], s[4]))          # np.hypot, as mission.run (ulp-exact)
        ok = (abs(s[1] - T[1]) <= 0.25 and 2.4 <= s[0] <= 3.0
              and abs(float(mission._wrap(s[2] - T[2]))) <= np.radians(15) and speed <= 0.08)
        if ok:
            if math.isnan(hold_since):
                hold_since = float(t[k])
                ev.append({'t': float(rec_t[k]), 'kind': 'dock_hold_start', 'actor': 'boat', 'payload': {'tick': k}})
        else:
            hold_since = math.nan
        if ok and t[k] - hold_since >= 2.0 - 1e-9:
            docked, t_dock = True, hold_since
            k_hold = next(e['payload']['tick'] for e in reversed(ev) if e['kind'] == 'dock_hold_start')
            ev.append({'t': float(rec_t[k]), 'kind': 'docked', 'actor': 'boat',
                       'payload': {'t_dock': t_dock, 't_dock_record': float(rec_t[k_hold]), 'tick': k,
                                   'rule': 'time to dock = start of the 2 s hold (mission.run)'}})
            if k != K:
                raise AssertionError(f'docked at tick {k} but the truth continues to {K}')
            break
    steps = int(round(t_max / DT))
    if docked:
        terminal = 'docked'
    elif K >= steps:
        terminal = 'timeout'
        ev.append({'t': float(rec_t[K]), 'kind': 'timeout', 'actor': 'harness', 'payload': {'t_max': t_max, 'tick': K}})
    else:
        terminal = 'truncated'
    pose_err = 0.0 if docked else float(np.hypot(st[K, 0] - T[0], st[K, 1] - T[1]))
    gate_miss = float(math.isnan(gate_y) or abs(gate_y) > mission.GATE_Y)
    m = {'docked': np.array([float(docked)]), 't_dock': np.array([t_dock if docked else np.nan]),
         'contacts': np.array([float(contacts)]), 'gate_miss': np.array([gate_miss]), 'pose_err': np.array([pose_err])}
    J = float(mission.score(m)[0])
    retries = sum(e['kind'] == 'dock_hold_start' for e in ev) - (1 if docked else 0)
    return ev, {'docked': float(docked), 't_dock': t_dock if docked else math.nan, 'contacts': float(contacts),
                'gate_miss': gate_miss, 'gate_y': gate_y, 'pose_err': pose_err, 'J': J, 'terminal': terminal,
                'ticks': K, 'retries': int(retries)}


def assert_outcome_matches(outcome, metrics, tol_t=0.0):
    """Event-derived outcome == mission.run metrics (docked, t_dock, contacts, gate_miss, J)."""
    get = lambda k: float(np.asarray(metrics[k]).ravel()[0])
    for k in ('docked', 'contacts', 'gate_miss'):
        assert outcome[k] == get(k), (k, outcome[k], get(k))
    a, b = outcome['t_dock'], get('t_dock')
    assert (math.isnan(a) and math.isnan(b)) or abs(a - b) <= tol_t, ('t_dock', a, b)
    assert abs(outcome['J'] - get('J')) <= tol_t + 1e-12 * max(1.0, abs(get('J'))), ('J', outcome['J'], get('J'))
