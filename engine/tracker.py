"""fastsim's GoalTracker and the Navigator's progress checker on a batch of goals.

A goal is a route buffer ``route [E, R, 3]`` read from ``head`` to ``rlen`` (passed poses are popped
by moving ``head``), with ``live`` (a goal that is not done), ``precise`` and the progress anchor.

    G, target = command(ops, G, pose, lim)        # GoalTracker.command: the body-velocity target [E, 3]
    G = advance(ops, G, pose, now, lim)           # Navigator.advance: abort a goal that stopped moving

``lim`` is a dict of the Limits numbers; ``ops`` the backend (numpy or jax). ``nb_command`` and
``nb_advance`` are the numba twins over one env. Status codes: 1 SUCCEEDED, 2 ABORTED, 3 CANCELED.
"""

import math

import numba
import numpy as np

from engine import exact, nbexact

SUCCEEDED, ABORTED, CANCELED = 1, 2, 3
STATUS = {0: None, SUCCEEDED: 'SUCCEEDED', ABORTED: 'ABORTED', CANCELED: 'CANCELED'}
LIMIT_KEYS = ('vmax0', 'vmax1', 'vmax2', 'vmin0', 'vmin1', 'vmin2', 'decel0', 'goal_xy', 'goal_yaw',
              'precise_xy', 'precise_yaw', 'rotate_above', 'rotate_wz', 'progress_radius', 'progress_s',
              'pass_radius', 'gain', 'brake')


def limits(lim, gain):
    """The tracker's numbers from a fastsim Limits and the vehicle's gain, as one row."""
    return np.array([*lim.vmax, *lim.vmin, lim.decel[0], *lim.goal_tol, *lim.precise_tol, lim.rotate_above,
                     lim.rotate_wz, lim.progress_radius, lim.progress_s, lim.pass_radius, float(gain),
                     2 * -lim.decel[0]], dtype=np.float64)


L = {k: i for i, k in enumerate(LIMIT_KEYS)}


def at(xp, route, head):
    """route[e, head[e]] -> three [E] arrays."""
    r = xp.take_along_axis(route, head[:, None, None], axis=1)[:, 0]
    return r[:, 0], r[:, 1], r[:, 2]


def pop(ops, G, x, y, lim):
    """Pass every via-pose closer than pass_radius (never the last). With ``ops.window`` the passes are
    found in the next ``window`` poses at once (a pass beyond it sets ``G['popmax']``, never silently)."""
    xp = ops.xp
    if ops.window is None:
        def more(h):
            tx, ty, _ = at(xp, G['route'], h)
            return G['live'] & (G['rlen'] - h > 1) & (ops.chypot(tx - x, ty - y) < lim[L['pass_radius']])

        return {**G, 'head': ops.while_loop(lambda h: xp.any(more(h)), lambda h: xp.where(more(h), h + 1, h), G['head'])}
    W = ops.window
    R = G['route'].shape[1]
    j = G['head'][:, None] + xp.arange(W)[None]
    jj = xp.clip(j, 0, R - 1)
    tx = xp.take_along_axis(G['route'][..., 0], jj, axis=1)
    ty = xp.take_along_axis(G['route'][..., 1], jj, axis=1)
    near = (ops.chypot(tx - x[:, None], ty - y[:, None]) < lim[L['pass_radius']]) & (j < G['rlen'][:, None] - 1)
    run = xp.cumprod(near, axis=1)                    # passes while every pose so far was passed
    passed = (xp.sum(run, axis=1) * G['live']).astype(G['head'].dtype)
    return {**G, 'head': G['head'] + passed, 'popmax': G['popmax'] | (passed == W)}


def command(ops, G, pose, lim):
    xp = ops.xp
    x, y, yaw = pose
    G = pop(ops, G, x, y, lim)
    tx, ty, tyaw = at(xp, G['route'], G['head'])
    dx, dy = tx - x, ty - y
    d = ops.chypot(dx, dy)
    final = G['rlen'] - G['head'] == 1
    xy_tol = xp.where(G['precise'], lim[L['precise_xy']], lim[L['goal_xy']])
    yaw_tol = xp.where(G['precise'], lim[L['precise_yaw']], lim[L['goal_yaw']])
    arrived = final & (d <= xy_tol)
    err_a = exact.wrap(tyaw - yaw, xp)
    done_now = G['live'] & arrived & (xp.abs(err_a) <= yaw_tol)
    err = exact.wrap(xp.arctan2(dy, dx) - yaw, xp)
    turn = xp.abs(err) > lim[L['rotate_above']]
    top = lim[L['vmax0']]
    if 'cap' in G:  # Navigator.speed_limit: min(vmax, cap)
        top = xp.where(G['cap'] < top, G['cap'], top)
    dd = xp.where(d > 0, d, 1.0)
    brake = xp.sqrt(lim[L['brake']] * d)
    speed = xp.where(final, xp.where(brake < top, brake, top), top)
    c, s = xp.cos(yaw), xp.sin(yaw)
    bx = (c * dx + s * dy) / dd * speed
    by = (-s * dx + c * dy) / dd * speed
    k = xp.ones_like(bx)
    for v, hi, lo in ((bx, lim[L['vmax0']], lim[L['vmin0']]), (by, lim[L['vmax1']], lim[L['vmin1']])):
        k = xp.where(v > hi, xp.minimum(k, hi / v), xp.where(v < lo, xp.minimum(k, lo / v), k))
    wz = xp.maximum(lim[L['vmin2']], xp.minimum(lim[L['vmax2']], lim[L['gain']] * err))
    rot = lim[L['rotate_wz']]
    zero = 0.0 * x
    vx = xp.where(arrived | turn, zero, bx * k)
    vy = xp.where(arrived | turn, zero, by * k)
    w = xp.where(arrived, xp.where(done_now, zero, xp.copysign(rot, err_a)),
                 xp.where(turn, xp.copysign(rot, err), wz))
    live = G['live']
    target = xp.stack([xp.where(live & ~done_now, vx, zero), xp.where(live & ~done_now, vy, zero),
                       xp.where(live & ~done_now, w, zero)], axis=-1)
    G = {**G, 'live': live & ~done_now, 'status': xp.where(done_now, SUCCEEDED, G['status'])}
    return G, target


def advance(ops, G, pose, now, lim):
    xp = ops.xp
    x, y, _ = pose
    ax, ay = G['anchor'][:, 0], G['anchor'][:, 1]
    far = exact.pow_half(exact.pow2(ax - x, xp) + exact.pow2(ay - y, xp), xp) > lim[L['progress_radius']]
    reset = G['live'] & (~G['anchored'] | far)
    stall = G['live'] & ~reset & (now - G['anchor_t'] > lim[L['progress_s']])
    return {**G, 'anchor': xp.where(reset[:, None], xp.stack([x, y], axis=-1), G['anchor']),
            'anchor_t': xp.where(reset, now, G['anchor_t']), 'anchored': G['anchored'] | reset,
            'live': G['live'] & ~stall, 'status': xp.where(stall, ABORTED, G['status'])}


# ---- numba twins ---------------------------------------------------------------------------------

@numba.njit(cache=True, inline='always')
def nb_command(route, head, rlen, precise, x, y, yaw, lim, out):
    """GoalTracker.command for one live goal: writes the target into ``out`` [3]; returns
    (head, succeeded)."""
    while rlen - head > 1 and nbexact.chypot(route[head, 0] - x, route[head, 1] - y) < lim[15]:
        head += 1
    tx, ty, tyaw = route[head, 0], route[head, 1], route[head, 2]
    dx, dy = tx - x, ty - y
    d = nbexact.chypot(dx, dy)
    final = rlen - head == 1
    xy_tol = lim[9] if precise else lim[7]
    yaw_tol = lim[10] if precise else lim[8]
    if final and d <= xy_tol:
        err = nbexact.wrap(tyaw - yaw)
        out[0] = 0.0
        out[1] = 0.0
        if abs(err) <= yaw_tol:
            out[2] = 0.0
            return head, True
        out[2] = math.copysign(lim[12], err)
        return head, False
    err = nbexact.wrap(math.atan2(dy, dx) - yaw)
    if abs(err) > lim[11]:
        out[0] = 0.0
        out[1] = 0.0
        out[2] = math.copysign(lim[12], err)
        return head, False
    top = lim[0]
    speed = top
    if final:
        b = math.sqrt(lim[17] * d)
        speed = b if b < top else top
    c, s = math.cos(yaw), math.sin(yaw)
    bx = (c * dx + s * dy) / d * speed
    by = (-s * dx + c * dy) / d * speed
    k = 1.0
    if bx > lim[0]:
        k = min(k, lim[0] / bx)
    elif bx < lim[3]:
        k = min(k, lim[3] / bx)
    if by > lim[1]:
        k = min(k, lim[1] / by)
    elif by < lim[4]:
        k = min(k, lim[4] / by)
    out[0] = bx * k
    out[1] = by * k
    out[2] = max(lim[5], min(lim[2], lim[16] * err))
    return head, False


@numba.njit(cache=True, inline='always')
def nb_far(ax, ay, x, y, r):
    """Python's ``((ax - x) ** 2 + (ay - y) ** 2) ** 0.5 > r``: libm pow only when the plain sqrt is
    within 1e-12 of r (pow and sqrt differ by an ulp at most, so elsewhere they decide alike)."""
    a, b = ax - x, ay - y
    d = math.sqrt(a * a + b * b)
    if d > r * (1.0 + 1e-12):
        return True
    if d < r * (1.0 - 1e-12):
        return False
    return nbexact.pow_half(nbexact.pow2(a) + nbexact.pow2(b)) > r
