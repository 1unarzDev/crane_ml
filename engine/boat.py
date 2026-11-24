"""The boat's reference mission (domains/boat: Mission, mission.py) as a batched controller.

    plans = BoatPlans.pack(adapter, scenarios, worlds)   # host: every env's plan as arrays
    M = plans.init(E)                                   # phase, retried, seen buoys, outcome, events
    M, G = tick(ops, plans.A, M, G, pose, vis, step)    # the FSM at tick_hz, all envs at once
    A = score_row(ops, plans.A, A, row, step)           # gate crossings and the mark sweep, per track row
    numbers = plans.finish(e, ...)                      # boat.json's numbers of env e (host)

The course route is planned on the host (it starts at the start pose, so it is fixed per course). The
fairway route depends on where the course ended, so it is planned in the batch: ``mission.route``'s
detours as a complete binary tree of depth 4 (each node a leg that may need a detour round its first
obstacle), read in order. Its numbers are mission.py's, operation for operation (engine/tests).
"""

from dataclasses import dataclass
import math
from types import SimpleNamespace

import numpy as np

from domains.boat import mission as BM
from domains.boat.adapter import DOCK_TOL_M, Mission
from engine.jit import nbexact, numba
from engine.tracker import SUCCEEDED, STATUS

PHASES = Mission.PHASES
PHASE, GOAL, RETRY, FAILED, DOCKED = 1, 2, 3, 4, 5
EVENT_NAMES = {PHASE: 'phase', GOAL: 'goal', RETRY: 'retry', FAILED: 'failed', DOCKED: 'docked'}
OUT_NONE, OUT_SUCCEEDED, OUT_FAILED = 0, 1, 2
OUTCOMES = {OUT_SUCCEEDED: 'SUCCEEDED', OUT_FAILED: 'FAILED'}
DEPTH = 4
NODES = 2 ** DEPTH - 1                  # 15 legs
FAIRWAY_R = 2 * NODES + 2               # detour points, the mouth and the approach
EVENTS_K, CROSS_C = 24, 8


def _inorder(n=0):
    return [] if n >= NODES else _inorder(2 * n + 1) + [n] + _inorder(2 * n + 2)


ORDER = _inorder()


@dataclass
class BoatPlans:
    A: dict             # arrays for the batch (see pack)
    docks: list         # per env: the plan's dock {id, x, y, yaw} (world frame)
    gates_total: list   # per env: the gate passes the course needs
    goal_last: list     # per env, per phase: the goal event's fields
    slips: list

    PADS = {}           # every array pads with zeros (padding is masked: obst_valid, gate_valid, tmatch)

    @classmethod
    def stack(cls, parts):
        """One BoatPlans from per-env (or per-chunk) ones, arrays padded to the widest."""
        from engine.world import stack
        return cls(stack([p.A for p in parts], cls.PADS), [d for p in parts for d in p.docks],
                   [g for p in parts for g in p.gates_total], [x for p in parts for x in p.goal_last],
                   [x for p in parts for x in p.slips])

    @classmethod
    def pack(cls, adapter, scenarios, worlds):
        E = len(scenarios)
        plans, missions, courses, last, slips = [], [], [], [], []
        for s, w in zip(scenarios, worlds):
            plan = BM.Plan.from_truth(s.raw)
            m = Mission(SimpleNamespace(world=w), plan)
            route, _ = m.route((0.0, 0.0), 'course')
            plans.append(plan)
            missions.append(m)
            courses.append(route)
            ends = [route[-1], (*m.approach, m.berth[2]), m.berth]
            last.append([{'x': round(x, 2), 'y': round(y, 2), 'yaw': round(yaw, 2), 'why': PHASES[k]}
                         for k, (x, y, yaw) in enumerate(ends)])
            slips.append(plan.dock['id'])
        R = max(max(len(r) for r in courses), FAIRWAY_R)
        N = max(len(w.ids) for w in worlds)
        K = max(len(m.buoys) for m in missions)
        G = max(len(p.gates) for p in plans)
        A = {'course': np.zeros((E, R, 3)), 'course_len': np.zeros(E, np.int64),
             'mouths': np.zeros((E, 2, 2)), 'mouth_keep': np.zeros((E, 2), bool), 'approach': np.zeros((E, 3)),
             'berth': np.zeros((E, 3)), 'obst': np.zeros((E, N, 3)), 'obst_valid': np.zeros((E, N), bool),
             'match': np.zeros((E, N, K), bool), 'port': np.zeros((E, G, 2)), 'stbd': np.zeros((E, G, 2)),
             'normal': np.zeros((E, G, 2)), 'out': np.zeros((E, G), bool), 'gate_valid': np.zeros((E, G), bool),
             'mark': np.zeros((E, 2)), 'buoys': np.zeros(E, np.int64)}
        for e, (m, plan, w, route) in enumerate(zip(missions, plans, worlds, courses)):
            A['course'][e, :len(route)] = route
            A['course_len'][e] = len(route)
            mouths = list(m.mouths) + list(m.mouths[-1:]) * (2 - len(m.mouths))
            A['mouths'][e] = mouths
            A['mouth_keep'][e] = [all(math.dist(p, (x, y)) > r + BM.CLEARANCE_M for x, y, r in m.obstacles)
                                  for p in mouths]
            A['approach'][e] = (*m.approach, m.berth[2])
            A['berth'][e] = m.berth
            A['obst'][e, :len(m.obstacles)] = m.obstacles
            A['obst'][e, len(m.obstacles):, :2] = 1e6 * (1 + np.arange(N - len(m.obstacles)))[:, None]
            A['obst_valid'][e, :len(m.obstacles)] = True
            for i in range(len(w.ids)):
                if not w.visible[i]:
                    continue
                tx, ty = float(w.xy[i, 0]), float(w.xy[i, 1])
                for k, b in enumerate(m.buoys):
                    A['match'][e, i, k] = abs(tx - b[0]) < 0.5 and abs(ty - b[1]) < 0.5
            A['buoys'][e] = len(m.buoys)
            for k, g in enumerate(plan.gates):
                normal = g.normal if g.direction == 'out' else g.normal - math.pi
                A['port'][e, k], A['stbd'][e, k] = (float(g.port[0]), float(g.port[1])), (float(g.starboard[0]), float(g.starboard[1]))
                A['normal'][e, k] = BM.unit(normal)
                A['out'][e, k], A['gate_valid'][e, k] = g.direction == 'out', True
            for k in range(len(plan.gates), G):  # padding gates never cross
                A['port'][e, k], A['stbd'][e, k] = (1e6, 1e6 + k), (1e6 + 1, 1e6 + k)
            A['mark'][e] = plan.mark
        targets = A['match'].any(axis=2)
        T = max(1, int(targets.sum(axis=1).max()))
        A['targets'] = np.zeros((E, T), np.int64)
        A['tmatch'] = np.zeros((E, T, K), bool)
        for e in range(E):
            idx = np.flatnonzero(targets[e])
            A['targets'][e, :len(idx)] = idx
            A['tmatch'][e, :len(idx)] = A['match'][e, idx]
        return cls(A, [{k: p.dock[k] for k in ('id', 'x', 'y', 'yaw')} for p in plans],
                   [len(p.gates) for p in plans], last, slips)

    def init(self, E):
        K = self.A['match'].shape[2]
        G = self.A['port'].shape[1]
        return {'phase': np.zeros(E, np.int64), 'retried': np.zeros(E, bool), 'seen': np.zeros((E, K), bool),
                'outcome': np.zeros(E, np.int64), 'events': np.zeros((E, EVENTS_K, 4), np.int64),
                'ecount': np.zeros(E, np.int64),
                # scoring accumulators (per track row)
                'cross': np.zeros((E, G, CROSS_C), np.int64), 'ccount': np.zeros((E, G), np.int64),
                'sweep': np.zeros(E), 'last_a': np.zeros(E), 'has_last': np.zeros(E, bool),
                'prev': np.zeros((E, 2)), 'has_prev': np.zeros(E, bool)}

    # ---- host side ------------------------------------------------------------------------------

    def events(self, e, rows, now):
        out = []
        for step, code, ph, status in rows:
            t = now(step)
            if code == PHASE:
                out.append({'t': t, 'event': 'phase', 'name': PHASES[ph]})
            elif code == GOAL:
                out.append({'t': t, 'event': 'goal', **self.goal_last[e][ph]})
            elif code in (RETRY, FAILED):
                out.append({'t': t, 'event': EVENT_NAMES[code], 'phase': PHASES[ph], 'status': STATUS[status]})
            else:
                out.append({'t': t, 'event': 'docked', 'slip': self.slips[e]})
        return out

    def finish(self, e, cross, ccount, sweep, last_row, outcome, now):
        """boat.json's numbers but contacts, time, seen and tier (mission.score from the accumulators)."""
        passed, after = 0, -math.inf
        for k in range(self.gates_total[e]):
            times = [now(s) for s in cross[k, :min(ccount[k], CROSS_C)]]
            times = [t for t in times if t > after]
            if times:
                passed, after = passed + 1, times[0]
        d = self.docks[e]
        x, y, yaw = last_row
        yaw_err = math.atan2(math.sin(yaw - d['yaw']), math.cos(yaw - d['yaw']))
        return {'gates_passed': passed, 'gates_total': self.gates_total[e],
                'mark_sweep_deg': round(math.degrees(sweep), 1),
                'dock_error_m': round(math.dist((x, y), (d['x'], d['y'])), 3),
                'dock_yaw_error_rad': round(abs(yaw_err), 3), 'slip': d['id'],
                'overflow': bool((np.asarray(ccount) > CROSS_C).any())}

    @staticmethod
    def success(outcome, numbers):
        return int(outcome == 'SUCCEEDED' and numbers['gates_passed'] == numbers['gates_total']
                   and numbers['dock_error_m'] <= DOCK_TOL_M)


# ---- the batched FSM (numpy or jax) --------------------------------------------------------------

def goals(plans):
    """(goals [E, R, 3], count [E]): each env's goal poses in the map frame, in order: the course route, then the
    approach and the berth (what crane.env rewards progress toward)."""
    A = plans.A
    E = A['course'].shape[0]
    n = A['course_len'] + 2
    G = np.zeros((E, int(n.max()), 3))
    for e in range(E):
        k = int(A['course_len'][e])
        G[e, :k] = A['course'][e, :k]
        G[e, k], G[e, k + 1] = A['approach'][e], A['berth'][e]
    return G, n.astype(np.int64)


def metrics(numbers):
    """BoatAdapter.metrics(run_dir) of one env, from its boat.json numbers (``BoatPlans.finish``)."""
    return {'boat': {k: v for k, v in numbers.items() if isinstance(v, (int, float)) and not isinstance(v, bool)}}


def append(xp, buf, count, mask, values):
    """Append ``values`` [E, ...] to ``buf`` [E, K, ...] where ``mask``; a full buffer drops (count goes on)."""
    K = buf.shape[1]
    slot = (xp.arange(K)[None] == count[:, None]) & mask[:, None]
    slot = slot.reshape(slot.shape + (1,) * (buf.ndim - 2))
    return xp.where(slot, values[:, None], buf), count + mask


def _event(xp, M, mask, step, code, phase, status):
    v = xp.stack([xp.broadcast_to(step, phase.shape), xp.broadcast_to(code, phase.shape), phase,
                  xp.broadcast_to(status, phase.shape)], axis=-1).astype(M['events'].dtype)
    ev, n = append(xp, M['events'], M['ecount'], mask, v)
    return {**M, 'events': ev, 'ecount': n}


def tick(ops, A, M, G, pose, vis, step):
    """Mission.tick for every active env. ``vis`` [E, T]: the targets perceived this tick."""
    xp = ops.xp
    active = M['outcome'] == OUT_NONE
    hit = xp.any(vis[:, :, None] & A['tmatch'], axis=1)
    M = {**M, 'seen': M['seen'] | (active[:, None] & hit)}
    exists, live = G['exists'], G['live']
    none = active & ~exists
    done = active & exists & ~live
    succ = done & (G['status'] == SUCCEEDED)
    last = succ & (M['phase'] == len(PHASES) - 1)
    adv = succ & ~last
    fail = done & (G['status'] != SUCCEEDED)
    failed = fail & M['retried']
    retry = fail & ~M['retried']
    phase = M['phase'] + adv
    new = none | adv
    z = 0 * phase
    first = new | retry | failed | last
    code = xp.where(new, PHASE, xp.where(retry, RETRY, xp.where(failed, FAILED, DOCKED)))
    M = _event(xp, M, first, step, code, phase, xp.where(retry | failed, G['status'], z))
    M = _event(xp, M, new | retry, step, GOAL + z, phase, z)
    M = {**M, 'phase': phase, 'retried': xp.where(adv | last, False, xp.where(retry, True, M['retried'])),
         'outcome': xp.where(last, OUT_SUCCEEDED, xp.where(failed, OUT_FAILED, M['outcome']))}
    # the new goals: the course, the fairway (planned from here), the slip
    R = G['route'].shape[1]
    x, y, _ = pose
    planner = {k: A[k] for k in ('mouths', 'mouth_keep', 'approach', 'obst', 'obst_valid', 'course_len')}
    fair, flen = ops.masked(new & (phase == 1), lambda P, x, y: fairway(ops, P, x, y, R), (planner, x, y),
                            (xp.zeros_like(G['route']), xp.zeros_like(G['rlen'])))
    slip = xp.zeros_like(G['route']).at[:, 0].set(A['berth']) if xp is not np else _slip(A, R)
    route = xp.where((phase == 0)[:, None, None], _fit(xp, A['course'], R),
                     xp.where((phase == 1)[:, None, None], fair, slip))
    rlen = xp.where(phase == 0, A['course_len'], xp.where(phase == 1, flen, 1))
    G = {**G, 'route': xp.where(new[:, None, None], route, G['route']),
         'rlen': xp.where(new, rlen, G['rlen']), 'head': xp.where(new, 0, G['head']),
         'precise': xp.where(new, phase == 2, G['precise']), 'exists': G['exists'] | new,
         'live': G['live'] | new | retry, 'status': xp.where(new | retry, 0, G['status']),
         'anchored': G['anchored'] & ~(new | retry)}
    return M, G


def _slip(A, R):
    out = np.zeros((A['berth'].shape[0], R, 3))
    out[:, 0] = A['berth']
    return out


def _fit(xp, a, R):
    if a.shape[1] == R:
        return a
    pad = xp.zeros((a.shape[0], R - a.shape[1], 3), a.dtype)
    return xp.concatenate([a, pad], axis=1)


def _seg_hits(ops, A, ax, ay, bx, by):
    """detour's test of legs [E, M] per env: (hit, (x, y, r)) of each leg's first obstacle, the
    lexicographic least (t, x, y, r) as Python's min of tuples."""
    xp = ops.xp
    ox, oy, r = (A['obst'][:, None, :, k] for k in range(3))                 # [E, 1, N]
    ax, ay, bx, by = (v[..., None] for v in (ax, ay, bx, by))                 # [E, M, 1]
    need = r + BM.CLEARANCE_M
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    raw = ((ox - ax) * dx + (oy - ay) * dy) / xp.where(L2 == 0, 1.0, L2)
    m = xp.where(raw < 1.0, raw, 1.0)                                         # min(1.0, raw)
    t = xp.where(L2 == 0, 0.0, xp.where(0.0 < m, m, 0.0))                     # max(0.0, ...)
    d = ops.chypot(ox - ax - t * dx, oy - ay - t * dy)
    hit = (A['obst_valid'][:, None, :] & (d < need) & (ops.chypot(ox - ax, oy - ay) > need)
           & (ops.chypot(ox - bx, oy - by) > need))
    shape = hit.shape
    keys = [xp.broadcast_to(v, shape) for v in (t, ox, oy, r)]
    keep = hit
    for key in keys:
        best = xp.min(xp.where(keep, key, xp.inf), axis=-1, keepdims=True)
        keep = keep & (key == best)
    k = xp.argmax(keep, axis=-1)[..., None]
    pick = [xp.take_along_axis(v, k, axis=-1)[..., 0] for v in keys[1:]]
    return xp.any(hit, axis=-1), pick


def _interleave(xp, u, v):
    return xp.stack([u, v], axis=-1).reshape(u.shape[0], -1)


# in-order position of each (level, node, p|q) candidate, the heap read level by level
_HEAP = [(L, j) for L in range(DEPTH) for j in range(2 ** L)]
INORDER_CAND = np.array([2 * _HEAP.index((int(np.log2(n + 1)), n + 1 - 2 ** int(np.log2(n + 1)))) + w
                         for n in ORDER for w in (0, 1)])


def fairway(ops, A, x, y, R):
    """Mission.route(here, 'fairway') for every env: (route [E, R, 3], length [E]). The detour tree is
    evaluated a level at a time (every leg of a level at once); a node's children are its legs to p and
    from q, valid where it hit."""
    xp = ops.xp
    m0x, m0y, m1x, m1y = A['mouths'][:, 0, 0], A['mouths'][:, 0, 1], A['mouths'][:, 1, 0], A['mouths'][:, 1, 1]
    second = ops.chypot(m1x - x, m1y - y) < ops.chypot(m0x - x, m0y - y)
    mx, my = xp.where(second, m1x, m0x), xp.where(second, m1y, m0y)
    keep = xp.where(second, A['mouth_keep'][:, 1], A['mouth_keep'][:, 0])
    ax, ay, bx, by = x[:, None], y[:, None], mx[:, None], my[:, None]
    valid = keep[:, None]
    cands = []                                                                 # per level: [E, 2^L, 2] of (ok, x, y)
    for L in range(DEPTH):
        hit, (ox, oy, r) = _seg_hits(ops, A, ax, ay, bx, by)
        hit = hit & valid
        dx, dy = bx - ax, by - ay
        Ln = ops.chypot(dx, dy)
        Ln = xp.where(Ln == 0, 1.0, Ln)
        nx, ny = -dy / Ln, dx / Ln
        side = (ax - ox) * nx + (ay - oy) * ny
        off = xp.where(side >= 0, 1.0, -1.0) * (r + BM.CLEARANCE_M + BM.DETOUR_WIDE_M)
        ux, uy = dx / Ln, dy / Ln
        px, py = ox + off * nx - BM.DETOUR_ALONG_M * ux, oy + off * ny - BM.DETOUR_ALONG_M * uy
        qx, qy = ox + off * nx + BM.DETOUR_ALONG_M * ux, oy + off * ny + BM.DETOUR_ALONG_M * uy
        cands.append((_interleave(xp, hit, hit), _interleave(xp, px, qx), _interleave(xp, py, qy)))
        if L + 1 < DEPTH:
            ax, ay, bx, by = (_interleave(xp, ax, qx), _interleave(xp, ay, qy), _interleave(xp, px, bx),
                              _interleave(xp, py, by))
            valid = _interleave(xp, hit, hit)
    order = INORDER_CAND
    ok = xp.concatenate([c[0] for c in cands], axis=1)[:, order]
    cx = xp.concatenate([c[1] for c in cands], axis=1)[:, order]
    cy = xp.concatenate([c[2] for c in cands], axis=1)[:, order]
    ok = xp.concatenate([ok, keep[:, None]], axis=1)                          # then the mouth
    cx = xp.concatenate([cx, mx[:, None]], axis=1)
    cy = xp.concatenate([cy, my[:, None]], axis=1)
    count = xp.sum(ok, axis=1).astype(A['course_len'].dtype)
    rank = xp.cumsum(ok, axis=1)                                              # slot s takes the (s+1)-th kept candidate
    slot = xp.arange(R)[None, :]
    src = xp.argmax(rank[:, None, :] >= (slot + 1)[..., None], axis=-1)       # [E, R]
    rx = xp.take_along_axis(cx, src, axis=1)
    ry = xp.take_along_axis(cy, src, axis=1)
    prevx = xp.concatenate([x[:, None], rx[:, :-1]], axis=1)
    prevy = xp.concatenate([y[:, None], ry[:, :-1]], axis=1)
    yaw = xp.arctan2(ry - prevy, rx - prevx)
    at = slot == count[:, None]
    rx = xp.where(at, A['approach'][:, 0:1], rx)
    ry = xp.where(at, A['approach'][:, 1:2], ry)
    yaw = xp.where(at, A['approach'][:, 2:3], yaw)
    return xp.stack([rx, ry, yaw], axis=-1), count + 1


def score_row(ops, A, M, row, step, active):
    """One track row (world frame, rounded) into the gate crossings and the mark sweep."""
    xp = ops.xp
    X, Y = row[0], row[1]
    x0, y0 = M['prev'][:, 0:1], M['prev'][:, 1:2]
    x1, y1 = X[:, None], Y[:, None]
    ax, ay, bx, by = A['port'][..., 0], A['port'][..., 1], A['stbd'][..., 0], A['stbd'][..., 1]

    def side(px, py, qx, qy, rx, ry):
        return (qx - px) * (ry - py) - (qy - py) * (rx - px)
    d1, d2 = side(ax, ay, bx, by, x0, y0), side(ax, ay, bx, by, x1, y1)
    d3, d4 = side(x0, y0, x1, y1, ax, ay), side(x0, y0, x1, y1, bx, by)
    cross = (((d1 < 0) & (0 <= d2)) | ((d2 < 0) & (0 <= d1))) & (d3 * d4 < 0)
    out = (x1 - x0) * A['normal'][..., 0] + (y1 - y0) * A['normal'][..., 1] > 0
    match = cross & (out == A['out']) & A['gate_valid'] & (M['has_prev'] & active)[:, None]
    C = M['cross'].shape[2]
    slot = (xp.arange(C)[None, None] == M['ccount'][..., None]) & match[..., None]
    crosses = xp.where(slot, step[:, None, None], M['cross'])
    mx, my = A['mark'][:, 0], A['mark'][:, 1]
    inside = ~(ops.chypot(X - mx, Y - my) > 2 * BM.UTURN_RADIUS_M + 2)
    a = xp.arctan2(Y - my, X - mx)
    add = xp.arctan2(xp.sin(a - M['last_a']), xp.cos(a - M['last_a']))
    on = active & inside
    return {**M, 'cross': crosses, 'ccount': M['ccount'] + match,
            'sweep': xp.where(on & M['has_last'], M['sweep'] + add, M['sweep']),
            'last_a': xp.where(on, a, M['last_a']), 'has_last': xp.where(active, inside, M['has_last']),
            'prev': xp.where(active[:, None], xp.stack([X, Y], axis=-1), M['prev']),
            'has_prev': M['has_prev'] | active}


# ---- numba twins over one env -------------------------------------------------------------------

@numba.njit(cache=True, inline='always')
def nb_seg_hit(obst, nobst, ax, ay, bx, by):
    """(hit, x, y, r) of detour's first obstacle on leg a-b (lexicographic in (t, x, y, r))."""
    found = False
    bt, bxo, byo, br = 0.0, 0.0, 0.0, 0.0
    for i in range(nobst):
        ox, oy, r = obst[i, 0], obst[i, 1], obst[i, 2]
        need = r + BM_CLEARANCE
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 == 0:
            t = 0.0
        else:
            t = max(0.0, min(1.0, ((ox - ax) * dx + (oy - ay) * dy) / L2))
        d = nbexact.chypot(ox - ax - t * dx, oy - ay - t * dy)
        if d < need and nbexact.chypot(ox - ax, oy - ay) > need and nbexact.chypot(ox - bx, oy - by) > need:
            if (not found or t < bt or (t == bt and (ox < bxo or (ox == bxo and (oy < byo or (oy == byo and r < br)))))):
                found, bt, bxo, byo, br = True, t, ox, oy, r
    return found, bxo, byo, br


@numba.njit(cache=True, inline='always')
def nb_fairway(obst, nobst, mouths, keep2, approach, x, y, route):
    """Mission.route(here, 'fairway') into ``route``; returns its length."""
    second = nbexact.chypot(mouths[1, 0] - x, mouths[1, 1] - y) < nbexact.chypot(mouths[0, 0] - x, mouths[0, 1] - y)
    k = 1 if second else 0
    mx, my, keep = mouths[k, 0], mouths[k, 1], keep2[k]
    n = 0
    if keep:
        ends = np.zeros((NODES, 4))
        valid = np.zeros(NODES, np.bool_)
        pts = np.zeros((NODES, 4))
        hits = np.zeros(NODES, np.bool_)
        ends[0, 0], ends[0, 1], ends[0, 2], ends[0, 3] = x, y, mx, my
        valid[0] = True
        for node in range(NODES):
            if not valid[node]:
                continue
            ax, ay, bx, by = ends[node, 0], ends[node, 1], ends[node, 2], ends[node, 3]
            hit, ox, oy, r = nb_seg_hit(obst, nobst, ax, ay, bx, by)
            if not hit:
                continue
            hits[node] = True
            dx, dy = bx - ax, by - ay
            L = nbexact.chypot(dx, dy)
            nx, ny = -dy / L, dx / L
            side = (ax - ox) * nx + (ay - oy) * ny
            off = (1.0 if side >= 0 else -1.0) * (r + BM_CLEARANCE + BM_WIDE)
            ux, uy = dx / L, dy / L
            px, py = ox + off * nx - BM_ALONG * ux, oy + off * ny - BM_ALONG * uy
            qx, qy = ox + off * nx + BM_ALONG * ux, oy + off * ny + BM_ALONG * uy
            pts[node, 0], pts[node, 1], pts[node, 2], pts[node, 3] = px, py, qx, qy
            if 2 * node + 2 < NODES:
                c = 2 * node + 1
                ends[c, 0], ends[c, 1], ends[c, 2], ends[c, 3] = ax, ay, px, py
                valid[c] = True
                c += 1
                ends[c, 0], ends[c, 1], ends[c, 2], ends[c, 3] = qx, qy, bx, by
                valid[c] = True
        for j in range(NODES):
            node = NB_ORDER[j]
            if hits[node]:
                route[n, 0], route[n, 1] = pts[node, 0], pts[node, 1]
                route[n + 1, 0], route[n + 1, 1] = pts[node, 2], pts[node, 3]
                n += 2
        route[n, 0], route[n, 1] = mx, my
        n += 1
    px, py = x, y
    for j in range(n):
        route[j, 2] = math.atan2(route[j, 1] - py, route[j, 0] - px)
        px, py = route[j, 0], route[j, 1]
    route[n, 0], route[n, 1], route[n, 2] = approach[0], approach[1], approach[2]
    return n + 1


BM_CLEARANCE, BM_WIDE, BM_ALONG = BM.CLEARANCE_M, BM.DETOUR_WIDE_M, BM.DETOUR_ALONG_M
NB_ORDER = np.array(ORDER, dtype=np.int64)


@numba.njit(cache=True, inline='always')
def nb_score_row(port, stbd, normal, out, ngates, cross, ccount, X, Y, x0, y0, has_prev, step):
    if not has_prev:
        return
    for g in range(ngates):
        ax, ay, bx, by = port[g, 0], port[g, 1], stbd[g, 0], stbd[g, 1]
        d1 = (bx - ax) * (y0 - ay) - (by - ay) * (x0 - ax)
        d2 = (bx - ax) * (Y - ay) - (by - ay) * (X - ax)
        d3 = (X - x0) * (ay - y0) - (Y - y0) * (ax - x0)
        d4 = (X - x0) * (by - y0) - (Y - y0) * (bx - x0)
        if ((d1 < 0 and 0 <= d2) or (d2 < 0 and 0 <= d1)) and d3 * d4 < 0:
            way_out = (X - x0) * normal[g, 0] + (Y - y0) * normal[g, 1] > 0
            if way_out == out[g]:
                if ccount[g] < cross.shape[1]:
                    cross[g, ccount[g]] = step
                ccount[g] += 1

