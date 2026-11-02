"""Run directories (the layout fastsim and the mhseals_testing recorder write) -> crane-traj/1 for the viewer.

A run dir has ``track.jsonl`` ({t, x, y, yaw}), ``task_events.jsonl``, ``task_status.jsonl``,
``course_truth.json`` and, once scored, ``analysis/score*.json``. One ``run.ctraj`` per run keeps the pose
(resampled to ``hz``) and every event the viewer draws: task progress, decisions, goals (task frame turned
into the world frame through the course start pose), behaviour changes, contacts, near misses and the
referee's rows. Times stay on the run's own clock (``t0_s`` is its first pose); nothing is scored here.

A paired archive (``index.jsonl`` + ``pairs/<id>/{fast,unity}``, the fastsim-pairs layout) becomes one
study: the fast run of each pair (``<pair>/fast``, the judge's id) names its Unity twin
(``unity:<session>``), and a Unity run shared by several pairs is written once.
"""

from __future__ import annotations

import glob
import hashlib
import json
import math
from pathlib import Path

from theater import traj as ctraj

DECISION_KINDS = ('execute', 'search', 'retry', 'give_up')
TASK_EVENTS = ('task_started', 'task_finished', 'gate_cleared', 'stage_started', 'stage_completed', 'stage_skipped',
               'docked', 'undocked', 'returned_home', 'debris', 'species_count')


def _jl(p):
    out = []
    try:
        with open(p) as f:
            for line in f:
                line = line.strip()
                if line:
                    try:
                        out.append(json.loads(line))
                    except ValueError:
                        pass  # a torn last line
    except OSError:
        pass
    return out


def _jload(p):
    try:
        with open(p) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def resample(track, hz):
    """Poses at a fixed rate from irregular samples (linear, yaw the short way round)."""
    track = [p for p in track if all(isinstance(p.get(k), (int, float)) for k in ('t', 'x', 'y', 'yaw'))]
    if len(track) < 2:
        return (track[0]['t'] if track else 0.0), [[p['x'] for p in track], [p['y'] for p in track], [p['yaw'] for p in track]]
    t0, t1 = track[0]['t'], track[-1]['t']
    n = int(math.floor((t1 - t0) * hz)) + 1
    xs, ys, yaws = [], [], []
    j = 0
    for i in range(n):
        t = t0 + i / hz
        while j < len(track) - 2 and track[j + 1]['t'] < t:
            j += 1
        a, b = track[j], track[j + 1]
        f = 0.0 if b['t'] == a['t'] else min(1.0, max(0.0, (t - a['t']) / (b['t'] - a['t'])))
        d = math.atan2(math.sin(b['yaw'] - a['yaw']), math.cos(b['yaw'] - a['yaw']))
        xs.append(a['x'] + (b['x'] - a['x']) * f)
        ys.append(a['y'] + (b['y'] - a['y']) * f)
        yaws.append(math.atan2(math.sin(a['yaw'] + d * f), math.cos(a['yaw'] + d * f)))
    return t0, [xs, ys, yaws]


def _world(start, x, y):
    sx, sy, syaw = start
    c, s = math.cos(syaw), math.sin(syaw)
    return round(sx + c * x - s * y, 3), round(sy + s * x + c * y, 3)


def truth_sha(ct: dict, path: Path) -> str:
    sha = (ct.get('generator') or {}).get('truth_sha256')
    return sha or hashlib.sha256(path.read_bytes()).hexdigest()


def events_of(d: Path, t0: float, start, positions: dict) -> list:
    """The run's events on its own clock, seconds from t0 (the first pose)."""
    out = []
    rel = lambda t: None if t is None else round(float(t) - t0, 3)
    for e in _jl(d / 'task_events.jsonl'):
        k, t = e.get('event'), rel(e.get('time'))
        if t is None:
            continue
        if k == 'decision':
            out.append({'t': t, 'kind': 'decision', 'lane': 'decision', 'decision': e.get('kind'), 'text': e.get('reason')})
        elif k == 'goal' and isinstance(e.get('x'), (int, float)):
            wx, wy = _world(start, e['x'], e['y'])
            out.append({'t': t, 'kind': 'goal', 'lane': 'goal', 'x': wx, 'y': wy, 'text': e.get('why')})
        elif k in TASK_EVENTS:
            txt = k.replace('_', ' ') + (f" {e['stage']}" if e.get('stage') else '') + (f" {e['gate']}" if e.get('gate') is not None else '')
            if e.get('outcome'):
                txt += f": {e['outcome']}"
            ev = {'t': t, 'kind': k, 'lane': 'task', 'text': txt, 'task': e.get('task')}
            for key in ('gate', 'stage', 'outcome'):
                if e.get(key) is not None:
                    ev[key] = e[key]
            out.append(ev)
    # behaviour changes; task_status counts `elapsed` from task_started
    started = next((e['t'] for e in out if e['kind'] == 'task_started'), 0.0)
    prev = None
    for s in _jl(d / 'task_status.jsonl'):
        b = (s.get('phase'), s.get('behaviour'))
        if b != prev and isinstance(s.get('elapsed'), (int, float)):
            out.append({'t': round(started + s['elapsed'], 3), 'kind': 'behaviour', 'lane': 'behaviour',
                        'text': s.get('behaviour') or s.get('phase'), 'phase': s.get('phase'), 'stage': s.get('stage')})
            prev = b
    # the referee: rows with a time, contacts, near misses, strikes
    scf = sorted(glob.glob(str(d / 'analysis' / 'score*.json')))
    sc = _jload(scf[-1]) if scf else None
    if sc:
        for tid, task in (sc.get('tasks') or {}).items():
            for row in task.get('rows', []):
                if row.get('t') is not None and row.get('possible'):
                    out.append({'t': rel(row['t']), 'kind': 'row', 'lane': 'judge', 'id': row.get('id'), 'task': tid,
                                'text': row.get('desc'), 'status': row.get('status'), 'points': row.get('points'),
                                'possible': row.get('possible')})
        con = (sc.get('diagnostics') or {}).get('contacts') or {}
        for kind, key in (('near_miss', 'near_misses'), ('contact', 'contacts')):
            for c in con.get(key) or []:
                if c.get('t') is None:
                    continue
                obj = c.get('object') or c.get('id')
                x, y = positions.get(obj, (None, None))
                out.append({'t': rel(c['t']), 'kind': kind, 'lane': 'judge', 'id': obj, 'x': x, 'y': y,
                            'clearance_m': c.get('min_clearance_m', c.get('clearance_m')), 'speed_mps': c.get('speed_mps')})
    else:
        fs = _jload(d / 'fastsim.json') or {}
        for c in fs.get('contacts') or []:
            x, y = positions.get(c.get('id'), (None, None))
            out.append({'t': rel(c['t']), 'kind': 'contact', 'lane': 'judge', 'id': c.get('id'),
                        'x': x, 'y': y, 'clearance_m': c.get('clearance_m'), 'geometric': True})
    out.sort(key=lambda e: e['t'])
    return [e for e in out if e['t'] is not None and math.isfinite(e['t'])]


def header_of(d: Path, tier: str) -> dict:
    scf = sorted(glob.glob(str(d / 'analysis' / 'score*.json')))
    sc = _jload(scf[-1]) if scf else None
    fs = _jload(d / 'fastsim.json') or {}
    ses = _jload(d / 'session.json') or {}
    te = _jl(d / 'task_events.jsonl')
    fin = next((e for e in te if e.get('event') == 'task_finished'), None)
    h = {'tier': tier, 'session': ses.get('id'), 'fidelity': fs.get('fidelity') or ses.get('fidelity') or
         ('Unity HDRP full stack via ROS: physics contacts, simulated sensors, Nav2' if tier == 'unity' else None),
         'task': (te[0].get('task') if te else None), 'outcome': 'unknown'}
    if fin:
        o = str(fin.get('outcome') or '').upper()
        h['outcome'] = 'finished' if o in ('SUCCESS', 'SUCCEEDED') else 'timeout' if 'TIME' in o else 'failed'
        h['task_outcome'] = fin.get('outcome')
    if sc:
        tot = sc.get('total') or {}
        rel = sc.get('reliability') or {}
        h.update(score=tot.get('points'), referee={'summary': sc.get('summary'), 'total': tot,
                 'failed': rel.get('failed'), 'clean': rel.get('clean'),
                 'causes': [c for f in (rel.get('flags') or {}).values() for c in f.get('causes', [])]})
    h['flags'] = list(fs.get('flags') or []) + list((_jload(d / 'summary.json') or {}).get('flags') or [])
    # the channel status grammar: what this run can and cannot say (present | empty | absent | invalid)
    ch = {'pose': 'present', 'decisions': 'present' if any(e.get('event') == 'decision' for e in te) else 'empty',
          'goals': 'present' if any(e.get('event') == 'goal' for e in te) else 'empty',
          'referee': 'present' if sc else 'absent', 'plan': 'absent', 'nlc_tube': 'absent'}
    if tier == 'unity':
        ch.update(perception='empty', validity='absent')
    elif tier == 'fast':
        ch.update(perception='present', validity='present')
    h['channels'] = ch
    return h


def write_run(d, out_dir: Path, tier: str, run_id: str, extra=None, hz: float = 10.0, courses: Path | None = None) -> dict | None:
    """One run dir -> ``out_dir/run.ctraj`` (+ the course truth under ``courses/<sha>.json``). Returns the header."""
    d = Path(d)
    track = _jl(d / 'track.jsonl')
    if len(track) < 2:
        return None
    ct_path = d / 'course_truth.json'
    ct = _jload(ct_path) or {}
    start = (ct.get('startX', 0.0), ct.get('startY', 0.0), ct.get('startYaw', 0.0))
    positions = {e['id']: (e['x'], e['y']) for e in ct.get('elements', []) if 'id' in e}
    t0, (xs, ys, yaws) = resample(track, hz)
    h = header_of(d, tier)
    h.update(run_id=run_id, pose_dims='planar', robot={'model': 'boat', 'agents': ['boat']},
             frame={'type': 'enu', 'origin': 'course'}, **(extra or {}))
    if ct:
        sha = truth_sha(ct, ct_path)
        h['course'] = {'truth_sha256': sha, 'name': ct.get('name'), 'family': ct.get('family')}
        h['seed'] = h.get('seed', (ct.get('generator') or {}).get('seed'))
        if courses is not None:
            courses.mkdir(parents=True, exist_ok=True)
            dst = courses / f'{sha}.json'
            if not dst.exists():
                dst.write_text(json.dumps(ct, separators=(',', ':')))
    out_dir.mkdir(parents=True, exist_ok=True)
    ctraj.write(out_dir / 'run.ctraj', {k: v for k, v in h.items() if v is not None},
                {'boat.x': xs, 'boat.y': ys, 'boat.yaw': yaws}, events_of(d, t0, start, positions), hz=hz, t0_s=t0)
    ctraj.write_lod(out_dir / 'run.ctraj')
    return h


def _safe(run_id: str) -> str:
    return ''.join(c if c.isalnum() or c in '._-' else '_' for c in run_id)


def ingest_pairs(root, out, study: str | None = None, log=print) -> int:
    """A paired archive -> ``out/runs/<id>/run.ctraj`` and ``out/courses``. Returns the runs written."""
    root, out = Path(root).expanduser(), Path(out).expanduser()
    study = study or root.name
    seen, n = set(), 0
    for row in _jl(root / 'index.jsonl'):
        pid = row.get('pair_id')
        p = root / 'pairs' / str(pid)
        if not p.is_dir():
            continue
        ses = (_jload(p / 'unity' / 'session.json') or {}).get('id') or Path(str(row.get('unity_from') or pid)).name
        high = f'unity:{ses}'
        nav = ((row.get('fast') or {}).get('git', {}).get('mhseals_nav') or {}).get('commit', '')[:7]
        cond = row.get('condition') or {}
        common = {'study': study, 'seed': row.get('seed'), 'task': cond.get('task')}
        h = write_run(p / 'fast', out / 'runs' / _safe(f'{pid}/fast'), 'fast', f'{pid}/fast',
                      {**common, 'arm': f'fast nav {nav}' if nav else 'fast', 'pair': {'id': pid, 'high': high}},
                      courses=out / 'courses')
        n += bool(h)
        if high not in seen:
            seen.add(high)
            h = write_run(p / 'unity', out / 'runs' / _safe(high), 'unity', high, {**common, 'arm': 'unity'},
                          courses=out / 'courses')
            n += bool(h)
        log(f'pair {pid}: seed {row.get("seed")} {cond.get("task")}')
    return n


def ingest_dirs(dirs, out, tier: str, study: str, arm: str | None = None, log=print) -> int:
    """Plain run dirs (one tier) -> ``out/runs``."""
    out = Path(out).expanduser()
    n = 0
    for d in dirs:
        d = Path(d).expanduser()
        rid = f'{tier}:{d.name}'
        h = write_run(d, out / 'runs' / _safe(rid), tier, rid, {'study': study, 'arm': arm or tier}, courses=out / 'courses')
        if h:
            n += 1
            log(f'{rid}: {h.get("outcome")} {h.get("score")}')
    return n
