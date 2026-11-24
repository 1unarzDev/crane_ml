"""The public reference mission over a course truth: plan, geometry and scoring.

    plan = Plan.from_truth(truth)        # world frame, from the truth's gates, tasks and bays
    plan.course                          # entry gates -> nav channel -> U-turn round the mark
    plan.dock                            # the chosen slip: (x, y, yaw into the slip), approach points
    score(plan, track, final_pose)       # gates passed, mark sweep, dock error

Routes avoid the course's elements with a simple detour (go round a disk that a straight leg
would touch); there is no planner beyond that.
"""

from dataclasses import dataclass, field
import math

APPROACH_M = 3.0      # a gate is entered this far before its line and left this far after
UTURN_RADIUS_M = 3.0  # the circle round the mark
UTURN_POINTS = 12
CLEARANCE_M = 0.9     # a leg keeps this much (half the beam and a margin) off an element's edge
DETOUR_WIDE_M = 0.15  # a detour passes this much wider than CLEARANCE_M ...
DETOUR_ALONG_M = 0.3  # ... on a leg this long either side of abeam
SLIP_STANDOFF_M = 2.0  # the slip is entered from this far outside its mouth
PIER_END_M = 3.0      # the fairway is entered this far past the pier's end


def unit(yaw):
    return math.cos(yaw), math.sin(yaw)


def heading(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def seg_dist(p, a, b):
    """(distance from p to segment a-b, the closest point's fraction along it)."""
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy), t


def detour(a, b, obstacles, depth=4):
    """Points to insert between a and b so that no leg passes within CLEARANCE_M of an
    obstacle (x, y, r); obstacles too close to an end of the leg are left alone."""
    if depth == 0:
        return []
    hits = []
    for x, y, r in obstacles:
        need = r + CLEARANCE_M
        d, t = seg_dist((x, y), a, b)
        if d < need and math.dist((x, y), a) > need and math.dist((x, y), b) > need:
            hits.append((t, x, y, r))
    if not hits:
        return []
    _, x, y, r = min(hits)
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = math.hypot(dx, dy)
    nx, ny = -dy / L, dx / L  # the leg's left
    side = (a[0] - x) * nx + (a[1] - y) * ny  # which side of the obstacle the leg runs
    off = (1.0 if side >= 0 else -1.0) * (r + CLEARANCE_M + DETOUR_WIDE_M)
    ux, uy = dx / L, dy / L
    p = (x + off * nx - DETOUR_ALONG_M * ux, y + off * ny - DETOUR_ALONG_M * uy)  # abeam, a little before
    q = (x + off * nx + DETOUR_ALONG_M * ux, y + off * ny + DETOUR_ALONG_M * uy)  # and after
    return detour(a, p, obstacles, depth - 1) + [p, q] + detour(q, b, obstacles, depth - 1)


def route(points, obstacles=(), fixed=()):
    """(x, y, yaw) poses through ``points`` with detours, each yaw along its incoming leg.
    A point (after the first, and not ``fixed[k]``: a gate centre) that lies too close to an
    obstacle is dropped; the detours of the legs either side of it go round the obstacle."""
    def keep(k, p):
        return (k < len(fixed) and fixed[k]) or all(math.dist(p, (x, y)) > r + CLEARANCE_M for x, y, r in obstacles)
    path = [points[0]]
    for b in [p for k, p in enumerate(points) if k and keep(k, p)]:
        path += detour(path[-1], b, obstacles) + [b]
    out = []
    for k, p in enumerate(path):
        yaw = heading(path[k - 1], p) if k else heading(p, path[1]) if len(path) > 1 else 0.0
        out.append((p[0], p[1], yaw))
    return out


@dataclass
class Gate:
    id: str
    port: tuple
    starboard: tuple
    centre: tuple
    normal: float          # yaw of the pass direction
    direction: str = 'out'  # 'out' along the normal, 'in' against it


@dataclass
class Plan:
    gates: list                       # the gate passes the course needs, in order
    course: list                      # (x, y) of the course leg, gates and U-turn
    centres: list                     # per course point: True for a gate centre (never dropped)
    mark: tuple                       # the U-turn mark (x, y)
    dock: dict                        # {id, x, y, yaw (into the slip), mouth, approach}
    obstacles: list = field(default_factory=list)  # (x, y, r) of every element

    @classmethod
    def from_truth(cls, t):
        els = {e['id']: e for e in t['elements']}
        obstacles = [(float(e['x']), float(e['y']), float(e.get('radius') or 0.0)) for e in t['elements']]
        gates = []
        for g in t['gates']:
            p, s = els[g['port']], els[g['starboard']]
            gates.append(Gate(g['id'], (p['x'], p['y']), (s['x'], s['y']),
                              ((p['x'] + s['x']) / 2, (p['y'] + s['y']) / 2), float(g['normal_yaw'])))
        by_task = lambda task: sorted((g for g in gates if g.id.startswith(task + '.')), key=lambda g: g.id)  # noqa: E731
        entry, channel, speed = by_task('entry_gates'), by_task('debris_clearance'), by_task('speed_challenge')
        mark = next(e for e in t['elements'] if e.get('task') == 'speed_challenge' and e.get('role') == 'mark')
        mark = (float(mark['x']), float(mark['y']))
        g = speed[0]
        back = Gate(g.id, g.port, g.starboard, g.centre, g.normal + math.pi, 'in')
        passes = entry + channel + [g]
        course = [p for g in passes for p in through(g)]
        course += uturn(mark, course[-1]) + through(back)
        passes.append(back)
        centres = [p in {g.centre for g in passes} for p in course]
        return cls(passes, course, centres, mark, slip(t, els), obstacles)


def through(g):
    nx, ny = unit(g.normal)
    cx, cy = g.centre
    return [(cx - APPROACH_M * nx, cy - APPROACH_M * ny), (cx, cy), (cx + APPROACH_M * nx, cy + APPROACH_M * ny)]


def uturn(mark, start):
    """Points on a circle round ``mark``, a full turn from the side ``start`` is on, turning
    the way that leaves the mark to port (counter-clockwise)."""
    a0 = heading(mark, start)
    return [(mark[0] + UTURN_RADIUS_M * math.cos(a0 + 2 * math.pi * k / UTURN_POINTS),
             mark[1] + UTURN_RADIUS_M * math.sin(a0 + 2 * math.pi * k / UTURN_POINTS))
            for k in range(1, UTURN_POINTS + 1)]


def slip(t, els):
    """The slip to dock in: the first free one with a green indicator (else the first free one).
    Its approach runs down the fairway: past the pier's end, then square into the mouth."""
    bays = [b for b in t.get('bays', []) if not b.get('occupied')]
    bay = next((b for b in bays if (b.get('indicator') or {}).get('color') == 'green'), bays[0])
    ox, oy = unit(bay['yaw'])  # out of the slip
    x, y = bay['x'], bay['y']
    out = bay['depth'] / 2 + SLIP_STANDOFF_M
    approach = (x + out * ox, y + out * oy)
    spine = els.get(f"{bay['task']}.{bay.get('pier')}.spine")
    mouths = [approach]
    if spine is not None:
        dx, dy = unit(spine['yaw'])
        sx, sy = spine['x'], spine['y']
        along = (approach[0] - sx) * dx + (approach[1] - sy) * dy
        reach = spine['length'] / 2 + PIER_END_M
        mouths = [(approach[0] + (k * reach - along) * dx, approach[1] + (k * reach - along) * dy)
                  for k in (1, -1)]
    return {'id': bay['id'], 'x': x, 'y': y, 'yaw': bay['yaw'] + math.pi, 'approach': approach,
            'mouths': mouths, 'pier': bay.get('pier')}


def crossed(track, a, b, normal):
    """Crossings of segment a-b by a track [(t, x, y)]: [(t, 'out'|'in')]."""
    def side(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    n = unit(normal)
    out = []
    for (t0, x0, y0), (t1, x1, y1) in zip(track, track[1:]):
        d1, d2 = side(a, b, (x0, y0)), side(a, b, (x1, y1))
        d3, d4 = side((x0, y0), (x1, y1), a), side((x0, y0), (x1, y1), b)
        if ((d1 < 0 <= d2) or (d2 < 0 <= d1)) and d3 * d4 < 0:
            out.append((t1, 'out' if (x1 - x0) * n[0] + (y1 - y0) * n[1] > 0 else 'in'))
    return out


def score(plan, track, pose):
    """{gates_passed, gates_total, mark_sweep_deg, dock_error_m, dock_yaw_error_rad} of a world
    track [(t, x, y)] ending at ``pose`` (x, y, yaw). A gate pass counts once, in order."""
    passed, after = 0, -math.inf
    for g in plan.gates:
        normal = g.normal if g.direction == 'out' else g.normal - math.pi
        times = [t for t, way in crossed(track, g.port, g.starboard, normal) if way == g.direction and t > after]
        if times:
            passed, after = passed + 1, times[0]
    sweep, last = 0.0, None
    for _, x, y in track:
        if math.dist((x, y), plan.mark) > 2 * UTURN_RADIUS_M + 2:
            last = None
            continue
        a = heading(plan.mark, (x, y))
        if last is not None:
            sweep += math.atan2(math.sin(a - last), math.cos(a - last))
        last = a
    d = plan.dock
    yaw_err = math.atan2(math.sin(pose[2] - d['yaw']), math.cos(pose[2] - d['yaw']))
    return {'gates_passed': passed, 'gates_total': len(plan.gates), 'mark_sweep_deg': round(math.degrees(sweep), 1),
            'dock_error_m': round(math.dist(pose[:2], (d['x'], d['y'])), 3),
            'dock_yaw_error_rad': round(abs(yaw_err), 3)}
