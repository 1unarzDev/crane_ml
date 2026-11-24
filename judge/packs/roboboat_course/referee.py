"""Referee: what the boat really did on each course task, from the course truth and the true track (ported from mhseals_testing analysis/{referee,mission,geometry}.py, GPL-3.0, at the owner's request).

The task tree reports what it believes it did (task events); the referee checks the course the
simulator placed (``course_truth.json``) against the boat's true track, both in the world frame.
``Tools/RoboBoat/course_truth.explain`` supplies the facts (gate crossings with direction, circle
sweeps, slip dwell, horn zone); this module turns them into a verdict per task:

    PASS   every check holds          FAIL   a check failed
    N/A    the course has no such task, or the task is not judged from the track

A track is a list of (t, x, y, yaw). The hull is the truth's ``vessel`` rectangle (length x beam)
about the track point, so contact is judged on the rectangle, not a circle.
"""

import math
import os
import sys
from collections import Counter
from pathlib import Path

ROBOBOAT = Path(__file__).resolve().parents[3] / 'Tools' / 'RoboBoat'
if str(ROBOBOAT) not in sys.path:
    sys.path.insert(0, str(ROBOBOAT))
import course_truth  # noqa: E402

CIRCLED_DEG = 345.0      # a full circle round a beacon, allowing for the entry and exit chords
HULL = (1.063, 0.895)    # m, length and beam when the truth has no vessel


def hull_of(truth):
    v = truth.get('vessel') or {}
    return (v.get('length', HULL[0]), v.get('beam', HULL[1]))


def hull_clearance(pose, centre, radius, hull):
    """Distance from a round object's edge to the hull rectangle at pose (x, y, yaw); negative is contact."""
    x, y, yaw = pose
    dx, dy = centre[0] - x, centre[1] - y
    c, s = math.cos(yaw), math.sin(yaw)
    along, across = c * dx + s * dy, -s * dx + c * dy
    return math.hypot(max(abs(along) - hull[0] / 2, 0), max(abs(across) - hull[1] / 2, 0)) - radius


def contacts(track, items, hull):
    """Ids of the round course objects the hull touched, each once."""
    reach = math.hypot(*hull) / 2
    touched = []
    for e in items:
        if 'length' in e:                     # box elements (piers, hulls by footprint) are not judged here
            continue
        cx, cy, r = e['x'], e['y'], e.get('radius', 0.2)
        near = (reach + r) ** 2               # cheap reject before the rectangle test
        if any((x - cx) ** 2 + (y - cy) ** 2 <= near and hull_clearance((x, y, yaw), (cx, cy), r, hull) < 0
               for _, x, y, yaw in track):
            touched.append(e['id'])
    return touched


def _gate_checks(info):
    gates = info.get('gates', [])
    out = sum(any(d == 'out' for _, d in g['crossings']) for g in gates)
    back = sum(any(d == 'in' for _, d in g['crossings']) for g in gates)
    return {'out through every gate': bool(gates) and out == len(gates)}, \
        {'gates_out': f'{out}/{len(gates)}', 'gates_back': f'{back}/{len(gates)}'}


def _checks(kind, info):
    """(checks, details) for one task type from its explain facts; None when not judged from the track."""
    if kind == 'entry_gates':
        return _gate_checks(info)
    if kind == 'debris_clearance':
        checks, details = _gate_checks(info)
        rescue, avoid = info.get('rescue', {}), info.get('avoid', {})
        checks['circled every rescue beacon'] = all(abs(v) >= CIRCLED_DEG for v in rescue.values())
        checks['circled no avoid beacon'] = all(abs(v['sweep_deg']) < CIRCLED_DEG for v in avoid.values())
        details.update(rescue_deg=rescue, avoid_deg={k: v['sweep_deg'] for k, v in avoid.items()})
        return checks, details
    if kind == 'speed_challenge':
        cross = info.get('gate_crossings', [])
        out = next((t for t, d in cross if d == 'out'), None)
        back = next((t for t, d in cross if d == 'in' and out is not None and t > out), None)
        details = {'direction': info.get('direction'), 'expected': info.get('expected'),
                   'sweep_deg': info.get('sweep_deg')}
        if back is not None:
            details['gate_to_gate_s'] = round(back - out, 2)
        return {'out and back through the gate': back is not None,
                'circled the mark the right way': bool(info.get('correct'))}, details
    if kind == 'marina':
        return {'docked in a free slip': info.get('tier') not in (None, 'occupied')}, \
            {'entered': info.get('entered'), 'tier': info.get('tier')}
    if kind == 'harbor_alert':
        return {'reached the called zone': bool(info.get('correct'))}, \
            {'target_zone': info.get('target_zone'), 'reached': info.get('reached')}
    return None


def judge(task, truth, track, events=(), session=None, facts=None):
    """Verdict for one task (its id or type) over ``track``; ``session`` holds an optional states.jsonl."""
    spec = course_truth.task(truth, task)
    if spec is None:
        return {'task': task, 'verdict': 'N/A', 'reason': f'the course has no task {task!r}'}
    track = [tuple(p[:4]) if len(p) >= 4 else (*p[:3], 0.0) for p in track]
    if facts is None:
        facts = course_truth.explain(session or os.devnull, truth, [p[:3] for p in track], list(events))
    judged = _checks(spec['type'], facts['tasks'].get(spec['id'], {}))
    if judged is None:
        return {'task': task, 'verdict': 'N/A', 'reason': f"{spec['type']} is not judged from the track"}
    checks, details = judged
    touched = contacts(track, course_truth.objects(truth, task=spec['id']), hull_of(truth))
    checks['no contact'] = not touched
    return {'task': task, 'verdict': 'PASS' if all(checks.values()) else 'FAIL', 'checks': checks,
            'contacts': touched, **details}


def judge_mission(truth, track, events, tasks, required=None, session=None):
    """Each stage judged over the part of the track between its stage_started and stage_completed or
    stage_skipped events. A stage not in ``required`` (default: every task) may be skipped, but a
    contact during it still fails it; a required stage never started fails."""
    required = set(tasks if required is None else required)
    results = {}
    for name in tasks:
        started = next((e['time'] for e in events
                        if e.get('event') == 'stage_started' and e.get('stage') == name), None)
        terminal = next((e for e in events if e.get('event') in ('stage_completed', 'stage_skipped')
                         and e.get('stage') == name), None)
        end = terminal['time'] if terminal else math.inf
        part = [p for p in track if started is not None and started <= p[0] <= end]
        if name not in required and (started is None or terminal and terminal['event'] == 'stage_skipped'):
            spec = course_truth.task(truth, name)
            items = course_truth.objects(truth, task=spec['id']) if spec else []
            touched = contacts([tuple(p[:4]) for p in part], items, hull_of(truth))
            results[name] = {'task': name, 'verdict': 'FAIL' if touched else 'SKIPPED',
                             'checks': {'no contact': not touched}, 'contacts': touched}
        elif started is None:
            results[name] = {'task': name, 'verdict': 'FAIL', 'checks': {'attempted': False}}
        else:
            stage_events = [e for e in events if started <= e.get('time', 0) <= end]
            results[name] = judge(name, truth, part, stage_events, session)
    passed = all(r['verdict'] in ('PASS', 'SKIPPED', 'N/A') for r in results.values())
    return {'task': 'mission', 'verdict': 'PASS' if passed else 'FAIL', 'stages': results}


def score(events, rubric, outcome=None):
    """Score task events against a rubric: {required: {event: n}, points: {event: p},
    max_counts: {event: n}, time_limit: s}. An event counts once unless max_counts caps it higher;
    the run passes when it finished SUCCESS within the limit with every required event present."""
    counts = Counter(e['event'] for e in events)
    finished = next((e for e in reversed(events) if e['event'] == 'task_finished'), None)
    outcome = (finished or {}).get('outcome') or outcome or 'INCOMPLETE'   # the stream's own wins
    elapsed = (finished or (events[-1] if events else {})).get('elapsed', 0.0)
    missing = {n: c - counts[n] for n, c in rubric.get('required', {}).items() if counts[n] < c}
    points = rubric.get('points', {})
    caps = {n: rubric.get('max_counts', {}).get(n, 1) for n in points}
    earned = {n: p * min(counts[n], caps[n]) for n, p in points.items() if counts[n]}
    limit = rubric.get('time_limit')
    late = limit is not None and elapsed > limit
    return {'outcome': outcome, 'elapsed': elapsed, 'events': dict(counts), 'missing': missing,
            'points': sum(earned.values()), 'points_by_event': earned,
            'possible': sum(p * caps[n] for n, p in points.items()), 'late': late,
            'verdict': 'PASS' if outcome == 'SUCCESS' and not missing and not late else 'FAIL'}


def judge_session(session, tasks=None, rubric=None):
    """A session directory (course_truth.json, track.jsonl, events.jsonl): one verdict per task
    (default every task on the course), or the mission when ``tasks`` is ['mission'] and a rubric
    names its stages."""
    s = Path(session)
    truth = course_truth.load(s)
    track = [(r['t'], r['x'], r['y'], r.get('yaw', 0.0)) for r in course_truth.read_jsonl(s / 'track.jsonl')]
    events = course_truth.read_jsonl(s / 'events.jsonl')
    if tasks == ['mission']:
        rubric = rubric or {}
        return judge_mission(truth, track, events, rubric.get('tasks') or [t['id'] for t in truth.get('tasks', [])],
                             rubric.get('required_stages'), s)
    facts = course_truth.explain(s, truth, [p[:3] for p in track], events)
    return {name: judge(name, truth, track, events, s, facts)
            for name in (tasks or [t['id'] for t in truth.get('tasks', [])])}
