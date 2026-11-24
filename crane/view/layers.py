"""World layers a view draws under the runs: one generic list, from a run record, a sidecar or a course truth.

A layer is a mapping with a ``type``:

    {type: disks,      name, items: [{x, y, r, label, id}]}                  buoys, markers, obstacles
    {type: polygons,   name, polys: [[[x, y], ...], ...]}                    footprints, boundary
    {type: segments,   name, items: [[[x0, y0], [x1, y1]], ...]}            gates
    {type: constraint, mode: keep_out|keep_in, shape: polygon|corridor|circle,
     points: [[x, y], ...], width: m (corridor), radius: m (circle), z: [lo, hi] (optional), id, label}

Constraint volumes (keep-out / keep-in polygons, corridors, circles) are what a run may not enter or must stay
in. A run record carries them as ``layers`` in its meta, or as ``layers.json`` beside it
(``{schema: crane.layers/1, layers: [...]}``); both are in the run's frame. Stdlib only: the board uses this too.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

SCHEMA = 'crane.layers/1'
SIDECAR = 'layers.json'
MODES = ('keep_out', 'keep_in')
SHAPES = ('polygon', 'corridor', 'circle')


class LayerError(ValueError):
    pass


def check_constraint(c: dict) -> dict:
    """``c`` if it is a well-formed constraint volume; LayerError otherwise."""
    if c.get('mode') not in MODES:
        raise LayerError(f"constraint {c.get('id', '?')}: mode {c.get('mode')!r} is not one of {MODES}")
    if c.get('shape') not in SHAPES:
        raise LayerError(f"constraint {c.get('id', '?')}: shape {c.get('shape')!r} is not one of {SHAPES}")
    pts = c.get('points')
    need = {'polygon': 3, 'corridor': 2, 'circle': 1}[c['shape']]
    if not isinstance(pts, list) or len(pts) < need or not all(
            isinstance(p, (list, tuple)) and len(p) >= 2 and all(isinstance(v, (int, float)) for v in p[:2]) for p in pts):
        raise LayerError(f"constraint {c.get('id', '?')}: a {c['shape']} needs {need}+ points [x, y]")
    if c['shape'] == 'corridor' and not (isinstance(c.get('width'), (int, float)) and c['width'] > 0):
        raise LayerError(f"constraint {c.get('id', '?')}: a corridor needs a positive width (m)")
    if c['shape'] == 'circle' and not (isinstance(c.get('radius'), (int, float)) and c['radius'] > 0):
        raise LayerError(f"constraint {c.get('id', '?')}: a circle needs a positive radius (m)")
    return c


def check(layers) -> list:
    out = []
    for layer in layers or []:
        if not isinstance(layer, dict) or 'type' not in layer:
            raise LayerError(f'a layer is a mapping with a type, not {layer!r}')
        out.append(check_constraint(layer) if layer['type'] == 'constraint' else layer)
    return out


def of_run(run_dir, meta: dict) -> list:
    """The run's own layers: ``meta['layers']`` then the ``layers.json`` sidecar."""
    out = list(meta.get('layers') or [])
    p = Path(run_dir) / SIDECAR
    if p.is_file():
        side = json.loads(p.read_text())
        if side.get('schema') != SCHEMA:
            raise LayerError(f"{p}: schema {side.get('schema')!r}, not {SCHEMA!r}")
        out += side.get('layers') or []
    return check(out)


def course_layers(truth: dict) -> list:
    """A course truth (crane_truth: boundary, task footprints, elements, gates) as generic layers."""
    if not isinstance(truth, dict) or not isinstance(truth.get('elements'), list):
        return []
    els = [e for e in truth['elements'] if isinstance(e, dict) and 'x' in e and 'y' in e]
    by_id = {e.get('id'): e for e in els}
    out = []
    if truth.get('boundary'):
        out.append({'type': 'polygons', 'name': 'boundary', 'polys': [truth['boundary']]})
    feet = [t['footprint'] for t in truth.get('tasks') or [] if isinstance(t, dict) and t.get('footprint')]
    if feet:
        out.append({'type': 'polygons', 'name': 'footprints', 'polys': feet})
    gates = []
    for g in truth.get('gates') or []:
        p, s = by_id.get(g.get('port')), by_id.get(g.get('starboard'))
        if p and s:
            gates.append([[p['x'], p['y']], [s['x'], s['y']]])
    if gates:
        out.append({'type': 'segments', 'name': 'gates', 'items': gates})
    out.append({'type': 'disks', 'name': 'elements', 'items': [
        {'x': float(e['x']), 'y': float(e['y']), 'r': float(e.get('radius', e.get('r', 0.25)) or 0.25),
         'label': str(e.get('label') or ''), 'id': e.get('id')} for e in els]})
    return out


def world_of(run_dir, meta: dict) -> list:
    """World layers from the scenario file a run keeps (``meta.scenario.file``, else ``course_truth.json``)."""
    d = Path(run_dir)
    names = [((meta.get('scenario') or {}).get('file')), 'course_truth.json']
    for name in filter(None, names):
        p = d / name
        if p.is_file():
            try:
                return course_layers(json.loads(p.read_text()))
            except ValueError:
                return []
    return []


# ---- geometry ----------------------------------------------------------------------------------------------

def corridor_polygon(points, width) -> list:
    """The outline of a corridor (a polyline with a width) as one polygon: mitred offsets both sides."""
    pts = [(float(p[0]), float(p[1])) for p in points]
    h = width / 2.0
    left, right = [], []
    for i, (x, y) in enumerate(pts):
        dirs = []
        if i > 0:
            dirs.append(_unit(x - pts[i - 1][0], y - pts[i - 1][1]))
        if i < len(pts) - 1:
            dirs.append(_unit(pts[i + 1][0] - x, pts[i + 1][1] - y))
        nx, ny = 0.0, 0.0
        for dx, dy in dirs:
            nx, ny = nx - dy, ny + dx
        n = math.hypot(nx, ny) or 1.0
        nx, ny = nx / n, ny / n
        cos = max(0.25, abs(nx * -dirs[0][1] + ny * dirs[0][0]))   # mitre, clipped at 4x
        left.append((x + nx * h / cos, y + ny * h / cos))
        right.append((x - nx * h / cos, y - ny * h / cos))
    return left + right[::-1]


def _unit(dx, dy):
    n = math.hypot(dx, dy) or 1.0
    return dx / n, dy / n


def circle_polygon(cx, cy, r, n=48) -> list:
    return [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def outline(c: dict) -> list:
    """A constraint volume's outline as a polygon [(x, y), ...]."""
    if c['shape'] == 'polygon':
        return [(float(p[0]), float(p[1])) for p in c['points']]
    if c['shape'] == 'corridor':
        return corridor_polygon(c['points'], float(c['width']))
    x, y = c['points'][0][:2]
    return circle_polygon(float(x), float(y), float(c['radius']))


def _inside(poly, x, y) -> bool:
    hit = False
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            hit = not hit
    return hit


def _seg_dist(px, py, a, b) -> float:
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    dx, dy = bx - ax, by - ay
    t = 0.0 if dx == dy == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def inside(c: dict, x: float, y: float) -> bool:
    """Is the point inside the volume (planar; a corridor by distance to its centreline)?"""
    if c['shape'] == 'circle':
        cx, cy = c['points'][0][:2]
        return math.hypot(x - cx, y - cy) <= c['radius']
    if c['shape'] == 'corridor':
        p = c['points']
        return any(_seg_dist(x, y, p[i], p[i + 1]) <= c['width'] / 2 for i in range(len(p) - 1))
    return _inside(outline(c), x, y)


def violations(layers, xs, ys) -> list:
    """Per sample: True where the point is inside a keep-out volume or outside every keep-in one."""
    cons = [c for c in layers if c.get('type') == 'constraint']
    outs = [c for c in cons if c['mode'] == 'keep_out']
    ins = [c for c in cons if c['mode'] == 'keep_in']
    return [any(inside(c, x, y) for c in outs) or (bool(ins) and not any(inside(c, x, y) for c in ins))
            for x, y in zip(xs, ys)]
