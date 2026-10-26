#!/usr/bin/env python3
"""Independent evaluator of the course constraint spec (design §3), stdlib only.

Reads a course truth (crane.course_truth/2) and the spec rows in
``constraints/<family>.json`` and re-judges every row; it never trusts the truth's own
``constraints[]``. Mirrors Assets/Scripts/Course/Core/{ConstraintEvaluator,PathCheck,Occupancy}.cs.

    validate.py TRUTH.json [--spec SPEC.json] [--json]     exit 0 when every row passes
"""

import argparse
import heapq
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
CELL, STEP, WINDOW = 0.25, 0.05, 15.0


def default_spec(truth):
    return HERE / 'constraints' / f"{truth.get('family') or 'roboboat_2026'}.json"


# ---- geometry (Occupancy.cs Geometry) ------------------------------------------------------

def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def direction(yaw):
    return (math.cos(yaw), math.sin(yaw))


def left(v):
    return (-v[1], v[0])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def xy(o):
    return (o['x'], o['y'])


def box_distance(p, centre, yaw, length, width):
    d = sub(p, centre)
    c, s = math.cos(yaw), math.sin(yaw)
    lx = abs(c * d[0] + s * d[1]) - length / 2
    ly = abs(-s * d[0] + c * d[1]) - width / 2
    if lx <= 0 and ly <= 0:
        return max(lx, ly)
    return math.hypot(max(lx, 0.0), max(ly, 0.0))


def is_box(e):
    return 'length' in e


def surface(e, p):
    if is_box(e):
        return box_distance(p, xy(e), e['yaw'], e['length'], e['width'])
    return math.dist(p, xy(e)) - e['radius']


def cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def convex_overlap(a, b):
    for poly in (a, b):
        for i in range(len(poly)):
            e = sub(poly[(i + 1) % len(poly)], poly[i])
            n = (-e[1], e[0])
            pa = [dot(p, n) for p in a]
            pb = [dot(p, n) for p in b]
            if max(pa) <= min(pb) + 1e-9 or max(pb) <= min(pa) + 1e-9:
                return False
    return True


def inside_margin(poly, p):
    best = math.inf
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        best = min(best, cross(a, b, p) / math.dist(a, b))
    return best


def segment_distance(p, a, b):
    ab = sub(b, a)
    u = max(0.0, min(1.0, dot(sub(p, a), ab) / dot(ab, ab)))
    return math.dist(p, (a[0] + ab[0] * u, a[1] + ab[1] * u))


def points(lst):
    return [(float(p[0]), float(p[1])) for p in lst]


# ---- selection and references --------------------------------------------------------------

def path_get(node, path):
    if not path:
        return node
    for part in path.split('.'):
        if not isinstance(node, dict):
            return None
        node = node.get(part)
    return node


def same(a, b):
    if isinstance(a, bool) or isinstance(b, bool):
        return a is b if isinstance(a, bool) and isinstance(b, bool) else False
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(a - b) < 1e-9
    return a == b


def where(item, clause):
    for key, want in (clause or {}).items():
        value = path_get(item, key)
        if isinstance(want, list):
            if not any(same(w, value) for w in want):
                return False
        elif not same(want, value):
            return False
    return True


def task_ids(t, kind):
    return {kind} | {x['id'] for x in t.get('tasks', []) if x.get('type') == kind}


def strings(v):
    return set(v) if isinstance(v, list) else {v} if isinstance(v, str) else set()


def select(t, args):
    s = args.get('select') or {}
    collection = s.get('collection', 'elements')
    items = list(t.get(collection, []))
    if 'task' in s:
        ids = task_ids(t, s['task'])
        key = 'id' if collection == 'tasks' else 'task'
        items = [x for x in items if x.get(key) in ids]
    if 'role' in s:
        roles = strings(s['role'])
        items = [x for x in items if x.get('role') in roles]
    if 'kind_prefix' in s:
        prefixes = strings(s['kind_prefix'])
        items = [x for x in items if any((x.get('kind') or '').startswith(p) for p in prefixes)]
    if 'where' in s:
        items = [x for x in items if where(x, s['where'])]
    return items


class Missing(KeyError):
    pass


def ref(t, r):
    """(point, yaw) of "elem:ID", "gate:ID", "gate:TASK@first|@last", "bay:ID", "anchor:TASK.NAME"."""
    kind, _, ident = r.partition(':')
    if kind == 'gate' and '@' in ident:
        ids = task_ids(t, ident.split('@')[0])
        gates = sorted((g for g in t.get('gates', []) if g.get('task') in ids), key=lambda g: g['order'])
        if not gates:
            raise Missing(r)
        g = gates[-1] if ident.endswith('@last') else gates[0]
        return xy(g), g['normal_yaw']
    if kind == 'anchor':
        task, _, name = ident.partition('.')
        ids = task_ids(t, task)
        for x in t.get('tasks', []):
            if x.get('id') in ids and name in (x.get('anchors') or {}):
                return xy(x['anchors'][name]), math.nan
        raise Missing(r)
    collection = {'elem': 'elements', 'gate': 'gates'}.get(kind, 'bays')
    for o in t.get(collection, []):
        if o.get('id') == ident:
            return xy(o), o.get('normal_yaw' if kind == 'gate' else 'yaw', math.nan)
    raise Missing(r)


def element(t, ident):
    for e in t.get('elements', []):
        if e.get('id') == ident:
            return e
    raise Missing(ident)


def along(t, gate, point):
    g, yaw = ref(t, gate)
    return dot(sub(ref(t, point)[0], g), direction(yaw))


def shape_inside(poly, e):
    if not is_box(e):
        return inside_margin(poly, xy(e)) - e['radius']
    c, f = xy(e), direction(e['yaw'])
    l = left(f)
    best = math.inf
    for u, v in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
        p = (c[0] + f[0] * u * e['length'] / 2 + l[0] * v * e['width'] / 2,
             c[1] + f[1] * u * e['length'] / 2 + l[1] * v * e['width'] / 2)
        best = min(best, inside_margin(poly, p))
    return best


def min_clearance(items, exclude):
    kept = [e for e in items if e.get('role') not in exclude]
    best = 1e9
    for i in range(len(kept)):
        for j in range(i + 1, len(kept)):
            a, b = kept[i], kept[j]
            if is_box(a) and is_box(b):
                continue
            if is_box(a):
                d = surface(a, xy(b)) - b['radius']
            elif is_box(b):
                d = surface(b, xy(a)) - a['radius']
            else:
                d = math.dist(xy(a), xy(b)) - a['radius'] - b['radius']
            best = min(best, d)
    return best


def gate_clearance(t, items, gates):
    best = 1e9
    for g in gates:
        a, b = xy(element(t, g['port'])), xy(element(t, g['starboard']))
        for e in items:
            if e.get('id') in (g['port'], g['starboard']):
                continue
            best = min(best, segment_distance(xy(e), a, b) - (0 if is_box(e) else e['radius']))
    return best


# ---- path checks (PathCheck.cs) ------------------------------------------------------------

class World:
    def __init__(self, t, args):
        v = t.get('vessel') or {}
        ignore = set(args.get('ignore_roles') or [])
        self.obstacles = [e for e in t.get('elements', []) if e.get('role') not in ignore]
        self.boundary = points(t.get('boundary', []))
        self.beam, self.turn = v.get('beam', 0.895), v.get('turn_radius', 1.0)
        self.inflate = self.beam / 2 + args.get('margin', 0.3) / 2

    def free(self, p):
        if inside_margin(self.boundary, p) < self.beam / 2:
            return False
        return all(surface(e, p) > self.inflate for e in self.obstacles)

    def segment(self, a, b):
        n = max(1, math.ceil(math.dist(a, b) / STEP))
        return all(self.free((a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n)) for i in range(n + 1))

    def reachable(self, a, b):
        if not self.free(a) or not self.free(b):
            return False
        if self.segment(a, b):
            return True
        x0, y0 = min(a[0], b[0]) - WINDOW, min(a[1], b[1]) - WINDOW
        nx = math.ceil((abs(a[0] - b[0]) + 2 * WINDOW) / CELL)
        ny = math.ceil((abs(a[1] - b[1]) + 2 * WINDOW) / CELL)
        si, sj = int((a[0] - x0) / CELL), int((a[1] - y0) / CELL)
        gi, gj = int((b[0] - x0) / CELL), int((b[1] - y0) / CELL)
        state = {(si, sj): False, (gi, gj): False}

        def blocked(i, j):
            if (i, j) not in state:
                state[(i, j)] = not self.free((x0 + (i + 0.5) * CELL, y0 + (j + 0.5) * CELL))
            return state[(i, j)]

        def h(i, j):
            dx, dy = abs(i - gi), abs(j - gj)
            return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

        g = {(si, sj): 0.0}
        heap = [(h(si, sj), si, sj)]
        while heap:
            f, ci, cj = heapq.heappop(heap)
            if (ci, cj) == (gi, gj):
                return True
            if f > g[(ci, cj)] + h(ci, cj) + 1e-9:
                continue
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    ni, nj = ci + di, cj + dj
                    if (di == 0 and dj == 0) or ni < 0 or nj < 0 or ni >= nx or nj >= ny:
                        continue
                    if blocked(ni, nj) or (di and dj and (blocked(ci, nj) or blocked(ni, cj))):
                        continue
                    ng = g[(ci, cj)] + (math.sqrt(2) if di and dj else 1.0)
                    if ng >= g.get((ni, nj), math.inf):
                        continue
                    g[(ni, nj)] = ng
                    heapq.heappush(heap, (ng + h(ni, nj), ni, nj))
        return False

    def chain(self, pts):
        return all(self.reachable(pts[i], pts[i + 1]) for i in range(len(pts) - 1))

    def circle(self, c, r):
        pts = [(c[0] + r * math.cos(2 * math.pi * k / 72), c[1] + r * math.sin(2 * math.pi * k / 72)) for k in range(73)]
        return all(self.segment(pts[i], pts[i + 1]) for i in range(72))


def offset(p, yaw, d):
    v = direction(yaw)
    return (p[0] + v[0] * d, p[1] + v[1] * d)


def run_path(t, a):
    w = World(t, a)
    kind = a.get('kind')
    if kind == 'channel':
        gates = sorted(select(t, a), key=lambda g: g['order'])
        if not gates:
            return True
        pts = [offset(xy(gates[0]), gates[0]['normal_yaw'], -3)] + [xy(g) for g in gates]
        pts.append(offset(xy(gates[-1]), gates[-1]['normal_yaw'], 3))
        return w.chain(pts)
    if kind == 'loop':
        gate = next(iter(select(t, a)), None)
        mark = next(iter(select(t, {'select': a.get('mark')})), None)
        if gate is None or mark is None:
            return False
        mid, c = xy(gate), xy(mark)
        r = max(w.turn, mark['radius'] + w.inflate + 0.5)
        start = math.atan2(mid[1] - c[1], mid[0] - c[0])
        pts = [offset(mid, gate['normal_yaw'], -3), mid]
        pts += [offset(c, start + 2 * math.pi * k / 8, r) for k in range(9)]
        pts.append(mid)
        return w.chain(pts)
    if kind == 'circle':
        return all(w.circle(xy(e), w.turn + w.beam) for e in select(t, a))
    if kind == 'slips':
        approach = ref(t, a['from'])[0]
        for b in select(t, a):
            centre = xy(b)
            aisle = offset(centre, b['yaw'], b['depth'] / 2 + 2)
            if not w.segment(aisle, centre) or not w.reachable(approach, aisle):
                return False
        return True
    if kind == 'anchors':
        start = ref(t, a['from'])[0]
        return all(w.reachable(start, xy(x['anchors'][a['anchor']]))
                   for x in t.get('tasks', []) if a['anchor'] in (x.get('anchors') or {}))
    raise ValueError(f'unknown path kind {kind}')


# ---- rows ----------------------------------------------------------------------------------

def values(row, t):
    """(values, limit) of a row; ConstraintEvaluator.Values."""
    a, limit = row.get('args') or {}, dict(row.get('limit') or {})
    beam = (t.get('vessel') or {}).get('beam', 0.895)
    if a.get('plus_beam') is True:
        limit = {'min': limit['min'] + beam}
    p = row['predicate']
    if p == 'distance':
        if 'select' in a:
            src = ref(t, a['a'])[0]
            return [surface(e, src) for e in select(t, a)], limit
        return [math.dist(ref(t, a['a'])[0], ref(t, a['b'])[0])], limit
    if p in ('gate_cc', 'gate_clear'):  # recomputed from the posts, never read from the gate record
        out = []
        for g in select(t, a):
            port, starboard = element(t, g['port']), element(t, g['starboard'])
            cc = math.dist(xy(port), xy(starboard))
            out.append(cc if p == 'gate_cc' else cc - port['radius'] - starboard['radius'])
        return out, limit
    if p == 'gate_side':
        out = []
        for g in select(t, a):
            port = element(t, g['port'])
            lateral = dot(sub(xy(port), xy(g)), left(direction(g['normal_yaw'])))
            out.append(lateral if a['port_color'] in port['kind'] else -abs(lateral))
        return out, limit
    if p == 'along':
        if 'select' in a:
            return [along(t, a['gate'], 'elem:' + e['id']) for e in select(t, a)], limit
        s = along(t, a['gate'], a['point'])
        if 'relative_to' in a:
            s /= along(t, a['gate'], a['relative_to'])
        return [s], limit
    if p == 'lateral':
        g, yaw = ref(t, a['gate'])
        return [abs(dot(sub(ref(t, a['point'])[0], g), left(direction(yaw))))], limit
    if p == 'count':
        return [len(select(t, a))], limit
    if p == 'angle_between':
        d = ref(t, a['a'])[1] - ref(t, a['b'])[1] - (math.pi if a.get('antiparallel') is True else 0)
        return [abs(wrap(d)) * 180 / math.pi], limit
    if p == 'inside':
        poly = points(t.get('boundary', []))
        return [shape_inside(poly, e) for e in select(t, a)], limit
    if p == 'clearance':
        if 'gates' in a:
            return [gate_clearance(t, select(t, a), select(t, {'select': a['gates']}))], limit
        return [min_clearance(select(t, a), strings(a.get('exclude_roles')))], limit
    if p == 'no_overlap':
        polys = [points(x.get('footprint', [])) for x in t.get('tasks', [])]
        polys = [q for q in polys if len(q) > 2]
        n = sum(convex_overlap(polys[i], polys[j]) for i in range(len(polys)) for j in range(i + 1, len(polys)))
        return [n], limit
    if p == 'unique':
        allowed = a.get('allowed')
        bad, groups = 0, {}
        for x in select(t, a):
            groups.setdefault(json.dumps(path_get(x, a.get('group', ''))), []).append(x)
        for members in groups.values():
            by = {}
            for x in members:
                by.setdefault(json.dumps(path_get(x, a['key'])), []).append(x)
            for same_key in by.values():
                bad += len(same_key) - 1
                if allowed is not None and not any(same(v, path_get(same_key[0], a['key'])) for v in allowed):
                    bad += len(same_key)
        return [bad], limit
    if p == 'implies':
        return [sum(1 for x in select(t, a) if where(x, a.get('if')) and not where(x, a.get('then')))], limit
    if p == 'argmin_unique':
        keys = [k for k in (path_get(x, a['key']) for x in select(t, a)) if isinstance(k, (int, float)) and not isinstance(k, bool)]
        return [0 if not keys else sum(1 for k in keys if k == min(keys))], limit
    if p == 'value':
        return [path_get(x, a['path']) for x in select(t, a)], limit
    if p == 'path':
        return [1 if run_path(t, a) else 0], limit
    raise ValueError(f"unknown predicate {p} in row {row['id']}")


def margin(v, limit, tol):
    if 'in' in limit:
        return 1 if any(same(x, v) for x in limit['in']) else -1
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return -1
    m = math.inf
    if 'min' in limit:
        m = min(m, v - limit['min'] + tol)
    if 'max' in limit:
        m = min(m, limit['max'] + tol - v)
    return m


def matches(t, when):
    return any(where(x, {when['path']: when.get('equals')}) for x in select(t, when))


def evaluate_row(row, t):
    """{id, tag, pass, value, limit} for one row."""
    out = {'id': row['id'], 'tag': row.get('tag'), 'pass': False, 'value': None, 'limit': row.get('limit') or {}}
    if row.get('when') and not matches(t, row['when']):
        out['pass'] = True
        return out
    try:
        vals, limit = values(row, t)
    except Missing as missing:
        out['value'] = f'missing: {missing.args[0]}'
        return out
    out['limit'] = limit
    if not vals:
        out['pass'] = True
        return out
    worst = min(vals, key=lambda v: margin(v, limit, row.get('tol', 0)))
    out['value'] = worst
    out['pass'] = margin(worst, limit, row.get('tol', 0)) >= 0
    return out


def evaluate(spec, truth):
    return [evaluate_row(row, truth) for row in spec['rows']]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('truth')
    ap.add_argument('--spec')
    ap.add_argument('--json', action='store_true')
    args = ap.parse_args(argv)
    truth = json.loads(Path(args.truth).read_text())
    spec = json.loads(Path(args.spec or default_spec(truth)).read_text())
    results = evaluate(spec, truth)
    failed = [r for r in results if not r['pass']]
    if args.json:
        print(json.dumps({'pass': not failed, 'rows': results}, indent=2))
    else:
        for r in failed:
            print(f"FAIL {r['id']} [{r['tag']}] value={r['value']} limit={r['limit']}")
        print(f"{len(results) - len(failed)}/{len(results)} rows pass")
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
