"""The modular reference task: drive a waypoint route inside a corridor, with the assembly's footprint.

    route = load('square')                      # domains/modular/routes/square.json, or random (from each seed)
    res = drive(envs, routes, dt=0.005, nsub=4, backend='numba')    # per env: outcome, time, corridor excess

A route (``crane.modular.route/1``): ``waypoints`` [[x, y], ...] from the start (0, 0, yaw 0), ``corridor_m`` the
half-width the whole footprint must stay inside (around the polyline start -> waypoints), ``tol_m`` the reach
radius, ``vmax`` [vx, vy, wz] and ``time_limit_s``. ``random`` draws 4 waypoints in a 4 m box from each seed.

The controller is holonomic go-to-waypoint at the tick rate: world velocity k (wp - pos) clipped to the speed
limits, heading held at zero, turned into the body twist the firmware takes. Metrics per env: outcome (SUCCEEDED,
TIMED_OUT), time_s, path_m, track_rms_m (centre to the polyline), corridor_excess_m (the footprint's worst
distance beyond the corridor), corridor_out_s (time with any footprint corner outside).
"""

import json
from pathlib import Path

import numpy as np

from domains.modular import assembly as AS
from engine.kernels import mecanum_run as R

ROUTES = Path(__file__).resolve().parent / 'routes'
SCHEMA = 'crane.modular.route/1'
GAIN, YAW_GAIN = 1.5, 2.0


def load(name, seed=None):
    if name == 'random':
        from fastsim.identity import rng
        r = rng(int(seed or 0), 'modular.route')
        wps = np.cumsum(r.uniform(-1.0, 1.0, (4, 2)) + [[0.6, 0.0]], axis=0)
        return {'schema': SCHEMA, 'name': f'random-{seed}', 'waypoints': wps.round(3).tolist(), 'corridor_m': 0.6,
                'tol_m': 0.08, 'vmax': [0.5, 0.4, 1.0], 'time_limit_s': 60.0}
    path = ROUTES / f'{name}.json' if not str(name).endswith('.json') else Path(name).expanduser()
    if not path.is_file():
        known = ', '.join(['random'] + sorted(p.stem for p in ROUTES.glob('*.json')))
        raise FileNotFoundError(f'no route {name!r}; routes: {known}')
    doc = json.loads(path.read_text())
    if doc.get('schema') != SCHEMA:
        raise ValueError(f'{path}: schema {doc.get("schema")!r}, want {SCHEMA}')
    return doc


def _pad(arrays):
    """[E, n_max, 2] from arrays [n_e, 2], each padded by repeating its last point."""
    n = max(len(a) for a in arrays)
    return np.stack([np.vstack([a, np.repeat(a[-1:], n - len(a), 0)]) for a in arrays])


def seg_dist(pts, poly):
    """Distance [E, K] of points [E, K, 2] to each env's polyline [E, R, 2]."""
    best = np.full(pts.shape[:-1], np.inf)
    for i in range(poly.shape[1] - 1):
        a, ab = poly[:, None, i], poly[:, None, i + 1] - poly[:, None, i]
        t = np.clip(((pts - a) * ab).sum(-1) / np.maximum((ab * ab).sum(-1), 1e-12), 0.0, 1.0)
        best = np.minimum(best, np.linalg.norm(pts - (a + t[..., None] * ab), axis=-1))
    return best


def drive(envs, routes, dt=0.005, nsub=4, backend='numba'):
    """Run every env on its route (routes [E] dicts) to the end or its time limit: {metric: [E]}, ticks."""
    E = envs.p.shape[0]
    tick = dt * nsub
    T = int(max(np.ceil(r['time_limit_s'] / tick) for r in routes))
    R_ = max(len(r['waypoints']) for r in routes)
    wps = np.zeros((E, R_, 2))
    nwp = np.array([len(r['waypoints']) for r in routes])
    for e, r in enumerate(routes):
        w = np.asarray(r['waypoints'], float)
        wps[e, :len(w)], wps[e, len(w):] = w, w[-1]
    vmax = np.array([r['vmax'] for r in routes], float)
    tol = np.array([r['tol_m'] for r in routes])
    corridor = np.array([r['corridor_m'] for r in routes])
    limit = np.array([int(np.ceil(r['time_limit_s'] / tick)) for r in routes])
    polys = _pad([np.vstack([[0.0, 0.0], np.asarray(r['waypoints'], float)]) for r in routes])
    feet = _pad(envs.footprint)
    plant = R.Ticks(envs.p, dt, nsub, envs.delay, T, backend)
    k = np.zeros(E, int)
    done = np.zeros(E, bool)
    t_end = np.full(E, np.nan)
    path = np.zeros(E)
    track2 = np.zeros(E)
    excess = np.full(E, -np.inf)
    out_ticks = np.zeros(E)
    x = plant.x
    prev = x[:, 0:2].copy()
    for j in range(T):
        pos, yaw = x[:, 0:2], x[:, 2]
        tgt = wps[np.arange(E), np.minimum(k, R_ - 1)]
        err = tgt - pos
        vw = GAIN * err
        c, s = np.cos(yaw), np.sin(yaw)
        vb = np.stack([c * vw[:, 0] + s * vw[:, 1], -s * vw[:, 0] + c * vw[:, 1]], -1)
        scale = np.maximum(1.0, np.max(np.abs(vb) / vmax[:, :2], axis=1))
        wz = np.clip(YAW_GAIN * -np.arctan2(s, c), -vmax[:, 2], vmax[:, 2])
        u = np.concatenate([vb / scale[:, None], wz[:, None]], -1)
        live = ~done & (j < limit)
        u = np.where(live[:, None], u, 0.0)
        x = plant.tick(u)
        pos = x[:, 0:2]
        path += np.where(live, np.linalg.norm(pos - prev, axis=1), 0.0)
        prev = pos.copy()
        reached = live & (np.linalg.norm(wps[np.arange(E), np.minimum(k, R_ - 1)] - pos, axis=1) < tol)
        k = k + reached
        fin = reached & (k >= nwp)
        t_end = np.where(fin, (j + 1) * tick, t_end)
        worst = (seg_dist(AS.footprint_world(feet, x[:, 0:3]), polys) - corridor[:, None]).max(-1)
        excess = np.where(live, np.maximum(excess, worst), excess)
        out_ticks += live & (worst > 0)
        track2 += np.where(live, seg_dist(pos[:, None], polys)[:, 0] ** 2, 0.0)
        done |= fin
        if (done | (j + 1 >= limit)).all():
            break
    steps = np.minimum(np.where(done, np.round(t_end / tick), limit), j + 1)
    return {'outcome': np.where(done, 'SUCCEEDED', 'TIMED_OUT'), 'time_s': np.where(done, t_end, steps * tick),
            'path_m': path, 'track_rms_m': np.sqrt(track2 / np.maximum(steps, 1)), 'corridor_excess_m': excess,
            'corridor_out_s': out_ticks * tick, 'pose': x[:, 0:3].copy()}, j + 1
