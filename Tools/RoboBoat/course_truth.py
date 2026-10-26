#!/usr/bin/env python3
"""Agent API over crane course truth (design §4, §6), stdlib only.

    load(path)                    truth dict from a file or a session directory
    objects(t, task=, role=, label=, kind=)   elements matching every given filter
    gate(t, id)                   gate record with its port/starboard elements
    state_at(t, state_id, t_sim)  a state's value at sim time (states.jsonl if given, else its program)
    to_geodetic(t, x, y)          (lat, lon) from geodetic_origin, or None
    explain(session)              per-task timeline: gates crossed, clearances, state at decision,
                                  circle sweeps, slip entered vs target, horn zone; JSON for agents
    identity(t)                   generator.identity: seed, options, generator version, truth_sha256,
                                  pair_key (cli/README.md)
    check_identity(t)             problems with it: pair key and (course-frame truth) content hash
                                  recomputed from the file

CLI: course_truth.py summary|query|render|diff|validate|explain|identity ...  (``-h`` on each)
"""

import argparse
import decimal
import hashlib
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import render  # noqa: E402
import validate  # noqa: E402

DWELL_S = 2.0        # D: time inside a slip that counts as docked (handbook sets none)
CIRCLE_RADIUS = 8.0  # sweeps count only within this distance of the centre (as the referee)


# ---- loading and queries ---------------------------------------------------------------------

def load(path):
    p = Path(path)
    return json.loads((p / 'course_truth.json' if p.is_dir() else p).read_text())


def objects(t, task=None, role=None, label=None, kind=None):
    want = dict(task=task, role=role, label=label, kind=kind)
    return [e for e in t.get('elements', []) if all(v is None or e.get(k) == v for k, v in want.items())]


def element(t, ident):
    return next(e for e in t['elements'] if e['id'] == ident)


def gate(t, ident):
    g = dict(next(g for g in t['gates'] if g['id'] == ident))
    g['port_element'], g['starboard_element'] = element(t, g['port']), element(t, g['starboard'])
    return g


def task(t, ident):
    return next((x for x in t.get('tasks', []) if x['id'] == ident or x['type'] == ident), None)


def program_value(p, t_sim):
    """Programs.ValueAt mirrored."""
    kind = p['type']
    if kind == 'fixed':
        return p['value']
    if kind == 'toggle':
        k = math.floor((t_sim + p['phase_s']) / p['period_s'])
        return p['values'][k % 2]
    if kind == 'scripted':
        value = p['initial']
        for s in p['steps']:
            if s['t'] <= t_sim:
                value = s['value']
        return value
    if kind == 'horn':
        for on, off in horn_blasts(p):
            if on <= t_sim < off:
                return f"on:{p['freq_hz']:g}"
        return 'off'
    raise ValueError(f'unknown program {kind}')


def horn_blasts(p):
    length = 1.5 if p['pattern'] == 'two' else 1.0
    out = []
    for k in range(int(p['repeats'])):
        start = p['trigger_s'] + k * (length + p['gap_s'])
        out += [(start, start + 0.5), (start + 1.0, start + 1.5)] if p['pattern'] == 'two' else [(start, start + 1.0)]
    return out


def state_at(t, state_id, t_sim, states_jsonl=None):
    if states_jsonl and Path(states_jsonl).exists():
        value = None
        for line in Path(states_jsonl).read_text().splitlines():
            r = json.loads(line)
            if r['id'] == state_id and r['t_sim'] <= t_sim:
                value = r['value']
        if value is not None:
            return value
    s = next(s for s in t.get('states', []) if s['id'] == state_id)
    return program_value(s['program'], t_sim)


def to_geodetic(t, x, y):
    """World (x east, y north, metres; REP-105) to (lat, lon) about geodetic_origin."""
    o = t.get('geodetic_origin')
    if not o:
        return None
    lat = o['lat'] + math.degrees(y / 6378137.0)
    lon = o['lon'] + math.degrees(x / (6378137.0 * math.cos(math.radians(o['lat']))))
    return lat, lon


# ---- explain ---------------------------------------------------------------------------------

def read_jsonl(path):
    p = Path(path)
    return [json.loads(line) for line in p.read_text().splitlines() if line.strip()] if p.exists() else []


def crossings(track, a, b, normal_yaw):
    """[(t, 'out'|'in')] where the track crosses segment a-b; 'out' is along the gate normal."""
    out = []
    n = (math.cos(normal_yaw), math.sin(normal_yaw))
    for (t0, x0, y0), (t1, x1, y1) in zip(track, track[1:]):
        d1, d2 = validate.cross(a, b, (x0, y0)), validate.cross(a, b, (x1, y1))
        d3, d4 = validate.cross((x0, y0), (x1, y1), a), validate.cross((x0, y0), (x1, y1), b)
        # Half-open on the track side, so a sample exactly on the gate line counts once.
        if ((d1 < 0 <= d2) or (d2 < 0 <= d1)) and d3 * d4 < 0:
            out.append((round(t1, 2), 'out' if (x1 - x0) * n[0] + (y1 - y0) * n[1] > 0 else 'in'))
    return out


def sweep(track, centre, t_from=-math.inf, t_to=math.inf):
    """Signed degrees swept round ``centre`` (ccw positive) while within CIRCLE_RADIUS."""
    total, last = 0.0, None
    for t, x, y in track:
        if not t_from <= t <= t_to or math.dist((x, y), centre) > CIRCLE_RADIUS:
            last = None
            continue
        a = math.atan2(y - centre[1], x - centre[0])
        if last is not None:
            total += validate.wrap(a - last)
        last = a
    return math.degrees(total)


def explain(session, truth=None, track=None, events=None):
    """Per-task timeline joining truth, track.jsonl and events (bag /detections: later)."""
    s = Path(session)
    t = truth or load(s)
    rows = track if track is not None else [(r['t'], r['x'], r['y']) for r in read_jsonl(s / 'track.jsonl')]
    beam = (t.get('vessel') or {}).get('beam', 0.895)
    states = s / 'states.jsonl'
    out = {'course': t.get('name'), 'sha256': (t.get('generator') or {}).get('truth_sha256'), 'tasks': {}}

    clear = {e['id']: min((math.dist((x, y), (e['x'], e['y'])) - e['radius'] - beam / 2 for _, x, y in rows), default=None)
             for e in t['elements'] if 'length' not in e}
    out['contacts'] = sorted(i for i, c in clear.items() if c is not None and c < 0)
    out['min_clearance'] = {i: round(c, 2) for i, c in clear.items() if c is not None and c < 3}

    def gates_of(task_id):
        res = []
        for g in sorted((g for g in t['gates'] if g['task'] == task_id), key=lambda g: g['order']):
            p, q = element(t, g['port']), element(t, g['starboard'])
            res.append({'id': g['id'], 'crossings': crossings(rows, (p['x'], p['y']), (q['x'], q['y']), g['normal_yaw'])})
        return res

    for x in t.get('tasks', []):
        kind, sem, info = x['type'], x.get('semantics') or {}, {}
        if kind in ('entry_gates', 'debris_clearance'):
            info['gates'] = gates_of(x['id'])
            info['all_out'] = all(any(d == 'out' for _, d in g['crossings']) for g in info['gates'])
            info['all_in'] = all(any(d == 'in' for _, d in g['crossings']) for g in info['gates'])
        if kind == 'debris_clearance':
            info['rescue'] = {i: round(sweep(rows, xy(element(t, i))), 1) for i in sem.get('rescue', [])}
            info['avoid'] = {i: {'sweep_deg': round(sweep(rows, xy(element(t, i))), 1), 'min_clearance': round(clear[i], 2)}
                             for i in sem.get('avoid', []) if clear.get(i) is not None}
        if kind == 'speed_challenge':
            g = gate(t, sem['gate'])
            p, q = g['port_element'], g['starboard_element']
            cross = crossings(rows, (p['x'], p['y']), (q['x'], q['y']), g['normal_yaw'])
            entry = next((tt for tt, d in cross if d == 'out'), None)
            source = sem['marker']['id'] if sem['direction_source'] == 'marker' else sem['sign']['id']
            color = state_at(t, source, entry if entry is not None else 0.0, states)
            expected = 'ccw' if color == 'red' else 'cw'
            swept = sweep(rows, xy(element(t, sem['turn_mark'])), entry if entry is not None else -math.inf)
            info.update(gate_crossings=cross, source=source, source_color_at_entry=color, expected=expected,
                        sweep_deg=round(swept, 1), direction=('ccw' if swept > 0 else 'cw') if abs(swept) >= 300 else None)
            info['correct'] = info['direction'] == expected
        if kind == 'marina':
            info['target'] = sem.get('target')
            entered = None
            for b in (b for b in t['bays'] if b['task'] == x['id']):
                if dwell(rows, b) >= DWELL_S:
                    entered = b
                    break
            tiers = sem.get('tiers') or {}
            info['entered'] = entered and entered['id']
            info['tier'] = (None if entered is None else 'occupied' if entered['occupied'] else
                            'lowest_green' if entered['id'] == tiers.get('lowest_green') else
                            'green' if entered['id'] in tiers.get('green', []) else 'unoccupied')
        if kind == 'harbor_alert':
            horn = sem['horn']
            trigger = horn['program']['trigger_s']
            zones = x['anchors']
            reached = None
            for tt, px, py in rows:
                if tt < trigger:
                    continue
                for name in ('zone1', 'zone2'):
                    z = zones[name]
                    if math.dist((px, py), (z['x'], z['y'])) <= z['r']:
                        reached = reached or (name, round(tt, 2))
            info.update(trigger_s=trigger, pattern=horn['pattern'], target_zone=horn['target_zone'],
                        reached=reached and reached[0], reached_t=reached and reached[1],
                        correct=bool(reached) and reached[0] == horn['target_zone'])
        out['tasks'][x['id']] = info
    return out


def xy(e):
    return (e['x'], e['y'])


def dwell(track, bay):
    """Longest continuous time the boat centre is inside the slip rectangle."""
    f = (math.cos(bay['yaw']), math.sin(bay['yaw']))
    best = start = 0.0
    inside = False
    for t, x, y in track:
        d = (x - bay['x'], y - bay['y'])
        ok = abs(d[0] * f[0] + d[1] * f[1]) <= bay['depth'] / 2 and abs(-d[0] * f[1] + d[1] * f[0]) <= bay['width'] / 2
        if ok and not inside:
            start = t
        inside = ok
        if ok:
            best = max(best, t - start)
    return best


# ---- CLI -------------------------------------------------------------------------------------

def summary(t):
    lines = [f"{t.get('name')}  sha256 {(t.get('generator') or {}).get('truth_sha256', '')[:12]}  side {t.get('side')}  "
             f"labels {t.get('label_mode')}  vessel beam {t['vessel']['beam']}"]
    for x in t.get('tasks', []):
        n = sum(1 for e in t['elements'] if e['task'] == x['id'])
        sem = {k: v for k, v in (x.get('semantics') or {}).items() if not isinstance(v, (dict, list))}
        lines.append(f"  {x['id']:<17} {n:>3} objects  attempts {x.get('attempts', 1):g}  {json.dumps(sem)}")
    failed = [c['id'] for c in t.get('constraints', []) if not c['pass']]
    lines.append(f"  constraints: {len(t.get('constraints', [])) - len(failed)}/{len(t.get('constraints', []))} pass {failed or ''}")
    return '\n'.join(lines)


def diff(a, b):
    ea, eb = {e['id']: e for e in a['elements']}, {e['id']: e for e in b['elements']}
    moved = {i: round(math.dist(xy(ea[i]), xy(eb[i])), 3) for i in ea.keys() & eb.keys() if math.dist(xy(ea[i]), xy(eb[i])) > 0.001}
    return {'added': sorted(eb.keys() - ea.keys()), 'removed': sorted(ea.keys() - eb.keys()), 'moved': dict(sorted(moved.items()))}


# ---- course identity (cli/README.md) -----------------------------------------------------------

# Root keys Assemble writes before the hashed content (not hashed; Unity sets start/current/labels).
UNHASHED_ROOT = ('schema', 'schema_version', 'name', 'frame', 'startX', 'startY', 'startYaw', 'currentSpeed',
                 'currentToward', 'course_frame', 'geodetic_origin', 'label_mode')
# generator keys added after the hash.
UNHASHED_GENERATOR = ('crane_commit', 'truth_sha256', 'identity')


def _number(v):
    if isinstance(v, int):
        return str(v)
    d = decimal.Decimal(repr(v)).quantize(decimal.Decimal('1e-6'), rounding=decimal.ROUND_HALF_UP)
    text = format(d, 'f')
    text = text.rstrip('0').rstrip('.') if '.' in text else text
    return '0' if text in ('-0', '') else text


def _quote(s):
    out = ['"']
    for c in s:
        out.append({'"': '\\"', '\\': '\\\\', '\n': '\\n', '\t': '\\t', '\r': '\\r'}.get(c)
                   or (f'\\u{ord(c):04x}' if ord(c) < 0x20 else c))
    out.append('"')
    return ''.join(out)


def canonical(v):
    """Json.Write (compact) of the C# core: insertion order, numbers rounded to 6 decimals."""
    if v is None:
        return 'null'
    if isinstance(v, bool):
        return 'true' if v else 'false'
    if isinstance(v, (int, float)):
        return _number(v)
    if isinstance(v, str):
        return _quote(v)
    if isinstance(v, dict):
        return '{' + ','.join(_quote(k) + ':' + canonical(x) for k, x in v.items()) + '}'
    return '[' + ','.join(canonical(x) for x in v) + ']'


def sha256(text):
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def truth_sha256(t):
    """The content hash recomputed from the file, or None for a truth moved off the course frame
    (a Unity session's world-frame truth: compare its stored hash with the CLI's for the identity)."""
    if any(abs(float(v)) > 0 for v in (t.get('course_frame') or {}).values()):
        return None
    hashed = {k: v for k, v in t.items() if k not in UNHASHED_ROOT}
    hashed['generator'] = {k: v for k, v in t['generator'].items() if k not in UNHASHED_GENERATOR}
    return sha256(canonical(hashed))


def identity(t):
    return (t.get('generator') or {}).get('identity')


def pair_key(seed, options, source_sha256):
    return sha256(canonical({'seed': seed, 'options': options, 'source_sha256': source_sha256}))


def check_identity(t):
    """Problems with a truth's identity (empty list = consistent)."""
    g, i = t.get('generator') or {}, identity(t)
    if not i:
        return ['no generator.identity (truth from a generator before identity/1)']
    bad = []
    if i.get('version') != 1:
        bad.append(f"identity version {i.get('version')}")
    if i.get('seed') != g.get('seed') or i['options'].get('mode') != g.get('mode') or i['options'].get('set') != g.get('overrides') \
            or i['options'].get('family') != t.get('family'):
        bad.append('identity seed/options differ from generator.seed/mode/overrides/family')
    if 'vessel' in i['options'] and i['options']['vessel'] != t.get('vessel'):
        bad.append('identity options.vessel differs from the truth vessel')
    if i.get('truth_sha256') != g.get('truth_sha256'):
        bad.append('identity truth_sha256 differs from generator.truth_sha256')
    if 'unknown' in (i['generator'].get('source_sha256'), i['generator'].get('crane_commit')):
        bad.append('generator version unknown')
    if i.get('pair_key') != pair_key(i.get('seed'), i.get('options'), i['generator'].get('source_sha256')):
        bad.append('pair_key does not match seed + options + source_sha256')
    content = truth_sha256(t)
    if content is not None and content != g.get('truth_sha256'):
        bad.append(f'content hash {content} != truth_sha256 {g.get("truth_sha256")} (file edited)')
    return bad


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('summary').add_argument('truth')
    q = sub.add_parser('query')
    q.add_argument('truth')
    for key in ('task', 'role', 'label', 'kind'):
        q.add_argument(f'--{key}')
    r = sub.add_parser('render')
    r.add_argument('truth')
    r.add_argument('--track')
    d = sub.add_parser('diff')
    d.add_argument('a')
    d.add_argument('b')
    sub.add_parser('validate').add_argument('truth')
    sub.add_parser('explain').add_argument('session')
    sub.add_parser('identity', help='print generator.identity; exit 1 if inconsistent').add_argument('truth')
    args = ap.parse_args(argv)
    if args.cmd == 'summary':
        print(summary(load(args.truth)))
    elif args.cmd == 'query':
        print(json.dumps(objects(load(args.truth), args.task, args.role, args.label, args.kind), indent=1))
    elif args.cmd == 'render':
        return render.main([str(Path(args.truth) / 'course_truth.json' if Path(args.truth).is_dir() else args.truth)]
                           + (['--track', args.track] if args.track else []))
    elif args.cmd == 'diff':
        print(json.dumps(diff(load(args.a), load(args.b)), indent=1))
    elif args.cmd == 'validate':
        return validate.main([str(Path(args.truth) / 'course_truth.json' if Path(args.truth).is_dir() else args.truth)])
    elif args.cmd == 'explain':
        print(json.dumps(explain(args.session), indent=1))
    elif args.cmd == 'identity':
        t = load(args.truth)
        bad = check_identity(t)
        print(json.dumps({'identity': identity(t), 'content_checked': truth_sha256(t) is not None, 'problems': bad}, indent=1))
        return 1 if bad else 0
    return 0


if __name__ == '__main__':
    sys.exit(main())
