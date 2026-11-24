"""The numba backend: each env's whole episode in one compiled call, envs in parallel (prange).

    S = run(setup, record=0, fastmath=False, threads=None)   # the same state dict as engine/episode.py

The loop is fastsim/run.py's, per env: track row, scoring and clearance at track_hz; perception and
the boat mission's FSM at tick_hz; idle accounting, tracker, vehicle and progress checker every step.
Every helper is inlined (``inline='always'``), so ``fastmath=True`` compiles the whole episode with
LLVM's fast-math flags; without it the episode is the numpy reference's arithmetic bit for bit.
Env i's episode reads only env i's rows: its output never depends on the batch.
"""

import math
import types

import numba
import numpy as np

from engine import boat as BB, nbexact
from engine.episode import CONTACTS_K, OUT_TIMEOUT, init_state
from engine.geometry import nb_cause, nb_clearance, nb_cone, nb_visible
from engine.tracker import ABORTED, SUCCEEDED, nb_command, nb_far
from engine.vehicles import nb_drive, nb_pose

NONE, OK, FAIL = BB.OUT_NONE, BB.OUT_SUCCEEDED, BB.OUT_FAILED
EK, CC = BB.EVENTS_K, BB.CROSS_C


def _episode(e, kind, X, Cl, P, Q, dt, tick_s, substeps, blas,
             xy, radius, visible, box, dims, near, nel, start, c0, s0,
             course, course_len, mouths, mkeep, approach, berth, obst, targets, ntargets, tmatch,
             port, stbd, normal, gout, ngates, mark,
             tlim, cam, foot, reach, track_every, spt, tl, nowt, mux,
             outcome, end, events, ecount, contacts, cval, kcount, least, idle_first, idle_n, last_row,
             cross, ccount, sweep_out, seen, rec, record):
    R = course.shape[1]
    route = np.zeros((R, 3))
    target = np.zeros(3)
    pwm = np.zeros(4)
    n = nel[e]
    d0 = np.zeros(n)
    d1 = np.zeros(n)
    dist = np.zeros(n)
    cone = np.zeros(n, np.bool_)
    blockers = np.zeros(n, np.bool_)
    x = X[e]
    c = Cl[e]
    phase, retried = 0, False
    exists, live, status, precise = False, False, 0, False
    head, rlen = 0, 1
    anchored, anx, any_, ant = False, 0.0, 0.0, 0.0
    driven = 0.0
    touching = False
    lst = np.inf
    has_prev, px0, py0 = False, 0.0, 0.0
    has_last, last_a, sweep = False, 0.0, 0.0
    out = NONE
    ne, nk, ni, first = 0, 0, 0, -1
    step = 0
    while True:
        now = nowt[step]
        bx, by, byaw = nb_pose(kind, x)
        if step % track_every == 0:
            X_ = start[e, 0] + c0[e] * bx - s0[e] * by
            Y_ = start[e, 1] + s0[e] * bx + c0[e] * by
            r0 = nbexact.pyround(X_, 1e4)
            r1 = nbexact.pyround(Y_, 1e4)
            r2 = nbexact.pyround(nbexact.wrap(byaw + start[e, 2]), 1e5)
            j = step // track_every
            if record > 0 and j % record == 0:
                rec[e, j // record, 0], rec[e, j // record, 1], rec[e, j // record, 2] = r0, r1, r2
            BB.nb_score_row(port[e], stbd[e], normal[e], gout[e], ngates[e], cross[e], ccount[e],
                            r0, r1, px0, py0, has_prev, step)
            has_prev, px0, py0 = True, r0, r1
            mx, my = mark[e, 0], mark[e, 1]
            inside = not (nbexact.chypot(r0 - mx, r1 - my) > 8.0)
            if inside:
                a = math.atan2(r1 - my, r0 - mx)
                if has_last:
                    sweep += math.atan2(math.sin(a - last_a), math.cos(a - last_a))
                last_a = a
            has_last = inside
            last_row[e, 0], last_row[e, 1], last_row[e, 2] = r0, r1, r2
            cl, k = nb_clearance(xy[e], radius[e], box[e], dims[e], n, bx, by, byaw, foot, reach)
            if cl < lst:
                lst = cl
            if cl < 0 and not touching:
                if nk < CONTACTS_K:
                    contacts[e, nk, 0], contacts[e, nk, 1] = step, k
                    contacts[e, nk, 2] = nb_cause(xy[e], near[e], k, route, head, rlen, exists and live)
                    cval[e, nk] = cl
                nk += 1
            touching = cl < 0
        if step % spt == 0:
            useful = False  # perception only adds seen buoys: skip it once no target can add one
            for t in range(ntargets[e]):
                for kk in range(tmatch.shape[2]):
                    if tmatch[e, t, kk] and not seen[e, kk]:
                        useful = True
            if useful:
                nb_cone(xy[e], radius[e], visible[e], box[e], n, bx, by, byaw, cam, d0, d1, dist, cone, blockers)
            for t in range(ntargets[e] if useful else 0):
                if nb_visible(xy[e], radius[e], visible[e], box[e], n, bx, by, byaw, cam, targets[e, t],
                              d0, d1, dist, cone, blockers, blas):
                    for kk in range(tmatch.shape[2]):
                        if tmatch[e, t, kk]:
                            seen[e, kk] = True
            send = False
            if not exists:
                send = True
            elif not live:
                if status == SUCCEEDED:
                    retried = False
                    if phase == 2:
                        if ne < EK:
                            events[e, ne, 0], events[e, ne, 1], events[e, ne, 2], events[e, ne, 3] = step, BB.DOCKED, phase, 0
                        ne += 1
                        out = OK
                    else:
                        phase += 1
                        send = True
                else:
                    if retried:
                        if ne < EK:
                            events[e, ne, 0], events[e, ne, 1], events[e, ne, 2], events[e, ne, 3] = step, BB.FAILED, phase, status
                        ne += 1
                        out = FAIL
                    else:
                        retried = True
                        if ne < EK:
                            events[e, ne, 0], events[e, ne, 1], events[e, ne, 2], events[e, ne, 3] = step, BB.RETRY, phase, status
                        ne += 1
                        if ne < EK:
                            events[e, ne, 0], events[e, ne, 1], events[e, ne, 2], events[e, ne, 3] = step, BB.GOAL, phase, 0
                        ne += 1
                        live, status, anchored = True, 0, False
            if send:
                if ne < EK:
                    events[e, ne, 0], events[e, ne, 1], events[e, ne, 2], events[e, ne, 3] = step, BB.PHASE, phase, 0
                ne += 1
                if ne < EK:
                    events[e, ne, 0], events[e, ne, 1], events[e, ne, 2], events[e, ne, 3] = step, BB.GOAL, phase, 0
                ne += 1
                if phase == 0:
                    for j in range(course_len[e]):
                        route[j, 0], route[j, 1], route[j, 2] = course[e, j, 0], course[e, j, 1], course[e, j, 2]
                    rlen = course_len[e]
                    precise = False
                elif phase == 1:
                    rlen = BB.nb_fairway(obst[e], n, mouths[e], mkeep[e], approach[e], bx, by, route)
                    precise = False
                else:
                    route[0, 0], route[0, 1], route[0, 2] = berth[e, 0], berth[e, 1], berth[e, 2]
                    rlen = 1
                    precise = True
                head, exists, live, status, anchored = 0, True, True, 0, False
        is_live = exists and live
        if is_live:
            driven = now
        elif out == NONE and now - driven > mux:
            if first < 0:
                first = step
            ni += 1
        if is_live:
            head, done = nb_command(route, head, rlen, precise, bx, by, byaw, tlim, target)
            if done:
                live, status = False, SUCCEEDED
                target[0], target[1], target[2] = 0.0, 0.0, 0.0
        else:
            target[0], target[1], target[2] = 0.0, 0.0, 0.0
        nb_drive(kind, x, c, target, P[e], Q[e], dt, tick_s, substeps, pwm, blas)
        if exists and live:
            nx, ny, _ = nb_pose(kind, x)
            if not anchored or nb_far(anx, any_, nx, ny, tlim[13]):
                anchored, anx, any_, ant = True, nx, ny, now
            elif now - ant > tlim[14]:
                live, status = False, ABORTED
        step += 1
        if out != NONE:
            break
        if step > tl[e]:
            out = OUT_TIMEOUT
            break
    outcome[e], end[e], ecount[e], kcount[e], least[e] = out, step, ne, nk, lst
    idle_first[e], idle_n[e], sweep_out[e] = first, ni, sweep


def _copy(fn, name):
    out = types.FunctionType(fn.__code__, fn.__globals__, name, fn.__defaults__, fn.__closure__)
    out.__qualname__ = name
    return out


episode = numba.njit(cache=True)(_episode)
episode_fastmath = numba.njit(cache=True, fastmath=True)(_copy(_episode, '_episode_fastmath'))


@numba.njit(cache=True, parallel=True)
def drive(E, *args):
    for e in numba.prange(E):
        episode(e, *args)


@numba.njit(cache=True, parallel=True)
def drive_fastmath(E, *args):
    for e in numba.prange(E):
        episode_fastmath(e, *args)


def compiled(fastmath):
    return drive_fastmath if fastmath else drive


def run(setup, record=0, fastmath=False, threads=None):
    if threads:
        numba.set_num_threads(int(threads))
    S = init_state(setup, record)
    A, W, M = setup.plans.A, setup.W, S['M']
    rec = S['rec'] if record else np.zeros((setup.E, 1, 3))
    cam = np.array(setup.cam)
    E = setup.E
    seen = np.zeros_like(M['seen'])
    compiled(fastmath)(
        E, setup.vehicle.kind, S['V']['x'], S['V']['c'], setup.P, setup.Q, setup.dt, setup.vehicle.tick_s,
        setup.vehicle.substeps, not fastmath,
        W.xy, W.radius, W.visible, W.box, W.box_dims, W.near, W.count, W.start,
        np.cos(W.start[:, 2]), np.sin(W.start[:, 2]),
        A['course'], A['course_len'], A['mouths'], A['mouth_keep'], A['approach'], A['berth'], A['obst'],
        A['targets'], A['tmatch'].any(axis=2).sum(axis=1), A['tmatch'],
        A['port'], A['stbd'], A['normal'], A['out'], A['gate_valid'].sum(axis=1), A['mark'],
        setup.tlim, cam, setup.foot, setup.reach, setup.track_every, setup.steps_per_tick, setup.tl, setup.now,
        float(setup.cfg['mux_idle_s']),
        M['outcome'], S['end'], M['events'], M['ecount'], S['contacts'], S['cval'], S['kcount'], S['least'],
        S['idle_first'], S['idle_n'], S['last_row'], M['cross'], M['ccount'], M['sweep'], seen, rec, int(record))
    M['seen'] = seen
    return S
