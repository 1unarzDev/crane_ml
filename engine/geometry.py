"""Batched perception and contact geometry: fastsim's ConeCamera and footprint clearance over [E, N].

    vis = visibility(xp, W, pose, cam, targets)        # [E, T] what ConeCamera.see reports, of the targets
    least, k = clearance(xp, W, pose, foot)            # footprint_clearance: [E] metres and element index
    cause = contact_cause(xp, W, k, route, head, rlen, live)   # 0 no_goal, 1 goal_at_obstacle, 2 route_through

``W`` is a dict of the Worlds arrays on the backend, ``pose`` a tuple of [E] arrays (map frame).
Occlusion is vectorised over envs x targets x blockers; the dot product is numpy's own (a plain sum
for one blocker, OpenBLAS's fma otherwise; ``blas=False`` sums plainly). ``nb_*`` are numba twins
over one env.
"""

import math

import numpy as np

from engine import exact
from engine.jit import nbexact, numba

CAUSES = ('no_goal', 'goal_at_obstacle', 'route_through_obstacle')


def camera(spec):
    """(hfov rad, half, range, occlusion, mx, my, myaw) of a fastsim ``cone`` perception spec."""
    spec = dict(spec or {'name': 'cone'})
    if spec.get('name', 'cone') != 'cone':
        raise ValueError(f"engine: perception {spec.get('name')!r}; the engine has the cone camera")
    hfov = math.radians(spec.get('hfov_deg', 90.0))
    mount = tuple(spec.get('mount') or (0.0, 0.0, 0.0))
    return (hfov, math.radians(spec.get('hfov_deg', 90.0)) / 2, float(spec.get('range_m', 20.0)),
            1.0 if spec.get('occlusion', True) else 0.0, *(float(v) for v in mount))


def _take(xp, a, idx):
    """a [E, N, ...] at idx [E, T] -> [E, T, ...]."""
    return xp.take_along_axis(a, idx.reshape(idx.shape + (1,) * (a.ndim - 2)), axis=1)


def visibility(xp, W, pose, cam, targets, blas=True):
    """[E, T] bool: target element ``targets[e, t]`` is perceived from ``pose`` (ConeCamera.see)."""
    bx, by, byaw = pose
    hfov, half, rng, occ, mx, my, myaw = (cam[k] for k in range(7))
    c, s = xp.cos(byaw), xp.sin(byaw)
    x, y, yaw = bx + c * mx - s * my, by + s * mx + c * my, byaw + myaw
    d0 = W['xy'][..., 0] - x[:, None]
    d1 = W['xy'][..., 1] - y[:, None]
    dist = xp.hypot(d0, d1)
    bearing = xp.arctan2(d1, d0) - yaw[:, None]
    bearing = xp.arctan2(xp.sin(bearing), xp.cos(bearing))
    inrange = dist <= rng
    cone = W['visible'] & inrange & (xp.abs(bearing) <= half)
    seen = _take(xp, cone, targets)
    if not occ:
        return seen
    many = xp.sum(cone, axis=1) > 1                                  # len(idx) > 1
    blockers = W['visible'] & ~W['box'] & inrange                    # [E, N]
    nb = xp.sum(blockers, axis=1)[:, None, None]
    t0, t1 = _take(xp, d0, targets), _take(xp, d1, targets)          # [E, T]
    td = _take(xp, dist, targets)
    m = xp.where(td < 1e-9, 1e-9, td)
    u0, u1 = (t0 / m)[..., None], (t1 / m)[..., None]
    b0, b1 = d0[:, None, :], d1[:, None, :]
    reach = (td - _take(xp, W['radius'], targets))[..., None]
    p0, p1 = b0 * u0, b1 * u1
    along = p0 + p1
    pos, short = along > 0, along < reach
    if blas:  # numpy's d @ u (OpenBLAS fma for more than one blocker) where it can decide a comparison
        tol = 8 * np.finfo(np.float64).eps * (xp.abs(p0) + xp.abs(p1) + xp.abs(reach))
        unsure = (nb > 1) & ((xp.abs(along) <= tol) | (xp.abs(along - reach) <= tol))
        if unsure.any():
            i = np.nonzero(unsure)
            exact_along = exact.fma(xp.broadcast_to(b0, unsure.shape)[i], xp.broadcast_to(u0, unsure.shape)[i],
                                    xp.broadcast_to(b1 * u1, unsure.shape)[i])
            pos, short = pos.copy(), short.copy()
            pos[i], short[i] = exact_along > 0, exact_along < xp.broadcast_to(reach, unsure.shape)[i]
    across = xp.abs(b0 * u1 - b1 * u0)
    other = targets[..., None] != xp.arange(d0.shape[1])[None, None, :]
    hidden = (blockers[:, None, :] & other & pos & short & (across < W['radius'][:, None, :]))
    return seen & ~(many[:, None] & xp.any(hidden, axis=-1))


def _to_polygon(xp, px, py, ax, ay):
    """Distance from points (px, py) [..., M] to the convex polygons (ax, ay) [..., 4] (negative inside):
    fastsim.world.to_polygon with the points on the second-last axis."""
    bx, by = xp.roll(ax, -1, axis=-1), xp.roll(ay, -1, axis=-1)
    abx, aby = (bx - ax)[..., None, :], (by - ay)[..., None, :]
    apx, apy = px[..., :, None] - ax[..., None, :], py[..., :, None] - ay[..., None, :]
    t = xp.clip((apx * abx + apy * aby) / (abx * abx + aby * aby), 0, 1)
    edge = xp.min(xp.hypot(apx - t * abx, apy - t * aby), axis=-1)
    cross = abx * apy - aby * apx
    inside = xp.all(cross >= 0, axis=-1) | xp.all(cross <= 0, axis=-1)
    return xp.where(inside, -edge, edge)


def footprint_poly(xp, pose, foot):
    x, y, yaw = pose
    c, s = xp.cos(yaw), xp.sin(yaw)
    px = xp.stack([x + c * float(a) - s * float(b) for a, b in foot], axis=-1)
    py = xp.stack([y + s * float(a) + c * float(b) for a, b in foot], axis=-1)
    return px, py


def clearance(xp, W, pose, foot, reach):
    """(least clearance [E], element index [E]) of footprint_clearance; (inf, -1) with nothing in reach."""
    x, y, _ = pose
    near = xp.hypot(W['xy'][..., 0] - x[:, None], W['xy'][..., 1] - y[:, None]) <= W['radius'] + reach
    px, py = footprint_poly(xp, pose, foot)                          # [E, 4]
    disk = _to_polygon(xp, W['xy'][..., 0], W['xy'][..., 1], px, py) - W['radius']
    bi = W['boxes']                                                  # [E, B] box element indices
    bxy = _take(xp, W['xy'], bi)
    dims = _take(xp, W['box_dims'], bi)
    L, Wd, byaw = dims[..., 0], dims[..., 1], dims[..., 2]
    cb, sb = xp.cos(byaw), xp.sin(byaw)
    corners = ((L / 2, Wd / 2), (L / 2, -Wd / 2), (-L / 2, -Wd / 2), (-L / 2, Wd / 2))
    qx = xp.stack([bxy[..., 0] + (cb * u - sb * v) for u, v in corners], axis=-1)   # [E, B, 4]
    qy = xp.stack([bxy[..., 1] + (sb * u + cb * v) for u, v in corners], axis=-1)
    Px, Py = px[:, None, :], py[:, None, :]
    shape = qx.shape
    a = xp.min(_to_polygon(xp, xp.broadcast_to(Px, shape), xp.broadcast_to(Py, shape), qx, qy), axis=-1)
    b = xp.min(_to_polygon(xp, qx, qy, xp.broadcast_to(Px, shape), xp.broadcast_to(Py, shape)), axis=-1)
    boxed = xp.where(b < a, b, a)                                    # [E, B]
    slot = bi[:, :, None] == xp.arange(disk.shape[1])[None, None, :]
    slot = slot & W['box_valid'][:, :, None]
    c = xp.where(xp.any(slot, axis=1), xp.sum(xp.where(slot, boxed[:, :, None], 0.0), axis=1), disk)
    c = xp.where(near, c, xp.inf)
    k = xp.argmin(c, axis=1)
    least = xp.take_along_axis(c, k[:, None], axis=1)[:, 0]
    return least, xp.where(xp.isinf(least), -1, k)


def contact_cause(xp, W, k, route, head, rlen, live):
    """contact_cause of element ``k`` [E] given each env's live goal (route [E, R, 3] from ``head``)."""
    kk = xp.maximum(k, 0)[:, None]
    ex = xp.take_along_axis(W['xy'][..., 0], kk, axis=1)
    ey = xp.take_along_axis(W['xy'][..., 1], kk, axis=1)
    near = xp.take_along_axis(W['near'], kk, axis=1)
    slot = xp.arange(route.shape[1])[None]
    left = (slot >= head[:, None]) & (slot < rlen[:, None])
    hit = xp.any(left & (xp.hypot(route[..., 0] - ex, route[..., 1] - ey) <= near), axis=1)
    return xp.where(live, xp.where(hit, 1, 2), 0)


# ---- numba twins over one env ------------------------------------------------------------------------

@numba.njit(cache=True, inline='always')
def nb_visible(xy, radius, visible, box, n, bx, by, byaw, cam, target, d0, d1, dist, cone, blockers, blas):
    """Whether element ``target`` is perceived; d0/d1/dist/cone/blockers are this tick's scratch,
    filled by nb_cone."""
    if not cone[target]:
        return False
    if cam[3] == 0.0:
        return True
    count = 0
    nbl = 0
    for i in range(n):
        if cone[i]:
            count += 1
        if blockers[i]:
            nbl += 1
    if count <= 1:
        return True
    m = dist[target]
    if m < 1e-9:
        m = 1e-9
    u0, u1 = d0[target] / m, d1[target] / m
    reach = dist[target] - radius[target]
    for b in range(n):
        if not blockers[b] or b == target:
            continue
        if blas:
            along = nbexact.dot2_blas(d0[b], d1[b], u0, u1, nbl)
        else:
            along = d0[b] * u0 + d1[b] * u1
        across = abs(d0[b] * u1 - d1[b] * u0)
        if along > 0 and along < reach and across < radius[b]:
            return False
    return True


@numba.njit(cache=True, inline='always')
def nb_cone(xy, radius, visible, box, n, bx, by, byaw, cam, d0, d1, dist, cone, blockers):
    c, s = math.cos(byaw), math.sin(byaw)
    x, y, yaw = bx + c * cam[4] - s * cam[5], by + s * cam[4] + c * cam[5], byaw + cam[6]
    for i in range(n):
        d0[i] = xy[i, 0] - x
        d1[i] = xy[i, 1] - y
        dist[i] = math.hypot(d0[i], d1[i])
        inr = dist[i] <= cam[2]
        cone[i] = False
        if visible[i] and inr:  # only a visible element in range can be in the cone
            b = math.atan2(d1[i], d0[i]) - yaw
            b = math.atan2(math.sin(b), math.cos(b))
            cone[i] = abs(b) <= cam[1]
        blockers[i] = visible[i] and not box[i] and inr


@numba.njit(cache=True, inline='always')
def _nb_to_poly(px, py, ax, ay):
    """Distance from one point to a 4-gon (fastsim.world.to_polygon)."""
    edge = np.inf
    pos = True
    neg = True
    for j in range(4):
        jn = (j + 1) % 4
        abx, aby = ax[jn] - ax[j], ay[jn] - ay[j]
        apx, apy = px - ax[j], py - ay[j]
        t = (apx * abx + apy * aby) / (abx * abx + aby * aby)
        t = min(max(t, 0.0), 1.0)
        e = math.hypot(apx - t * abx, apy - t * aby)
        if e < edge:
            edge = e
        cr = abx * apy - aby * apx
        pos = pos and cr >= 0
        neg = neg and cr <= 0
    return -edge if (pos or neg) else edge


@numba.njit(cache=True, inline='always')
def nb_clearance(xy, radius, box, dims, n, x, y, yaw, foot, reach):
    c, s = math.cos(yaw), math.sin(yaw)
    px = np.empty(4)
    py = np.empty(4)
    for j in range(4):
        px[j] = x + c * foot[j, 0] - s * foot[j, 1]
        py[j] = y + s * foot[j, 0] + c * foot[j, 1]
    best = np.inf
    k = -1
    qx = np.empty(4)
    qy = np.empty(4)
    for i in range(n):
        dx, dy = xy[i, 0] - x, xy[i, 1] - y
        lim = radius[i] + reach
        if dx * dx + dy * dy > lim * lim * 1.000001 + 1e-12:  # surely beyond reach; hypot decides the rest
            continue
        if not math.hypot(dx, dy) <= lim:
            continue
        if box[i]:
            L, Wd, byaw = dims[i, 0], dims[i, 1], dims[i, 2]
            cb, sb = math.cos(byaw), math.sin(byaw)
            us = (L / 2, L / 2, -L / 2, -L / 2)
            vs = (Wd / 2, -Wd / 2, -Wd / 2, Wd / 2)
            for j in range(4):
                qx[j] = xy[i, 0] + (cb * us[j] - sb * vs[j])
                qy[j] = xy[i, 1] + (sb * us[j] + cb * vs[j])
            a = np.inf
            for j in range(4):
                v = _nb_to_poly(px[j], py[j], qx, qy)
                if v < a:
                    a = v
            b = np.inf
            for j in range(4):
                v = _nb_to_poly(qx[j], qy[j], px, py)
                if v < b:
                    b = v
            cl = b if b < a else a
        else:
            cl = _nb_to_poly(xy[i, 0], xy[i, 1], px, py) - radius[i]
        if cl < best:
            best = cl
            k = i
    return best, k


@numba.njit(cache=True, inline='always')
def nb_cause(xy, near, k, route, head, rlen, live):
    if not live:
        return 0
    for j in range(head, rlen):
        if math.hypot(route[j, 0] - xy[k, 0], route[j, 1] - xy[k, 1]) <= near[k]:
            return 1
    return 2
