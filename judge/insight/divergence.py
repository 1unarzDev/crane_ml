"""Where and why two runs of one scenario part: residuals, first divergence, counterfactual split.

For a pair (low tier, high tier):

* time residual  e_t(t) = |p_low(t) - p_high(t)| on relative run time
* path residual  e_p(t) = distance from p_low(t) to the high tier's whole path. A large time
  residual with a small path residual means the same route at a different pace (dynamics or
  timing); both large means the runs went different ways (decisions or compounding).
* first divergence: one-sided CUSUM on (e_t - tol) crosses h.

Counterfactual replay decomposition (when the pack gives a ``Replayer``), per segment of the high
run, all against the high tier's track from the segment's start state:

  open loop      the high tier's *commanded actuation* through the low tier's plant  -> plant error
  reset closed   the low tier's controller+plant from the high tier's start state   -> + interaction
  free           the two runs as run                                                 -> + compounding

  plant = e_open, interaction = max(0, e_reset - e_open), compounding = max(0, e_free - e_reset)

A missing replay mode degrades the split (and the insight says which term is unavailable).
"""

from __future__ import annotations

from collections import defaultdict
import math
from typing import Optional

import numpy as np

from judge.insight import Insight
from judge.metric import Span
from judge.record import RunRecord

DT = 0.1


class Replayer:
    """Domain hook. ``horizon_s``: local error horizon (None = whole segment). Each method returns (t, xy[N,2]) predicted over [t0, t1] on the high run's
    clock, or None when the mode is unavailable."""
    horizon_s = None

    def open_loop(self, high: RunRecord, t0: float, t1: float):
        return None

    def reset_closed_loop(self, high: RunRecord, t0: float, t1: float):
        return None


def _xy(rec, pose):
    ch = rec.channel(pose)
    return ch, ch.v[:, :2]


def start(rec: RunRecord, anchor=''):
    """The record's task start: the first ``anchor`` event, else the window start."""
    if anchor:
        ev = rec.events_of(anchor)
        if ev:
            return float(ev[0].t)
    return float(rec.header.window[0])


def aligned(low: RunRecord, high: RunRecord, pose='pose.se2', dt=DT, anchor=''):
    """(tau, xy_low, xy_high) on relative time from each task start, over the common span."""
    cl, cl_xy = _xy(low, pose)
    ch, ch_xy = _xy(high, pose)
    l0, h0 = start(low, anchor), start(high, anchor)
    span = min(cl.t[-1] - l0, ch.t[-1] - h0)
    if span <= dt:
        return None
    tau = np.arange(0.0, span, dt)
    a = np.column_stack([np.interp(tau + l0, cl.t, cl_xy[:, k]) for k in range(2)])
    b = np.column_stack([np.interp(tau + h0, ch.t, ch_xy[:, k]) for k in range(2)])
    return tau, a, b


def path_distance(points, path):
    """Distance from each point to the polyline ``path`` (N x 2)."""
    if len(path) < 2:
        return np.linalg.norm(points - path[0], axis=1)
    p0, p1 = path[:-1], path[1:]
    d = p1 - p0
    L2 = np.maximum((d ** 2).sum(1), 1e-12)
    out = np.empty(len(points))
    for i, q in enumerate(points):
        s = np.clip(((q - p0) * d).sum(1) / L2, 0, 1)
        proj = p0 + s[:, None] * d
        out[i] = np.sqrt(((proj - q) ** 2).sum(1)).min()
    return out


def cusum_first(tau, e, tol, h):
    """First tau where sum of (e - tol) dt, floored at 0, exceeds h; None if never."""
    s = 0.0
    for k in range(1, len(tau)):
        s = max(0.0, s + (e[k] - tol) * (tau[k] - tau[k - 1]))
        if s > h:
            # back off to where this excursion started
            j = k
            while j > 0 and e[j - 1] > tol:
                j -= 1
            return float(tau[j])
    return None


def motion_class(high: RunRecord, t0, t1, pose='pose.se2', turn_wz=0.15, hold_v=0.05):
    ch = high.channel(pose).window(t0, t1)
    if len(ch) < 3 or ch.t[-1] - ch.t[0] < 0.5:
        return 'short'
    t = np.arange(ch.t[0], ch.t[-1], 0.2)          # uniform: jittery logs inflate rates
    p = np.column_stack([np.interp(t, ch.t, ch.v[:, k]) for k in range(2)])
    v = np.hypot(*(np.diff(p, axis=0) / 0.2).T)
    wz = (np.abs(np.diff(np.interp(t, ch.t, np.unwrap(ch.v[:, 2])))) / 0.2 if ch.v.shape[1] > 2
          else np.zeros(len(v)))
    if np.mean(wz) > turn_wz:
        return 'turn'
    if np.mean(v) < hold_v:
        return 'hold'
    return 'straight'


def _rms_against(high, pred, pose):
    if pred is None:
        return None
    t, xy = pred
    hc = high.channel(pose)
    hx = np.column_stack([np.interp(t, hc.t, hc.v[:, k]) for k in range(2)])
    return float(np.sqrt(np.mean(((np.asarray(xy) - hx) ** 2).sum(1))))


def _local(replayer, mode, high, t0, t1, pose, H):
    """(mean RMS over sub-windows of length H re-anchored on the high run, sum of end errors)."""
    if replayer is None:
        return None, None
    fn = getattr(replayer, mode)
    edges = [t0] if not H else list(np.arange(t0, t1 - 0.5 * H, H)) or [t0]
    rms, ends = [], 0.0
    hc = high.channel(pose)
    for a in edges:
        b = t1 if not H else min(a + H, t1)
        pred = fn(high, a, b)
        if pred is None:
            return None, None
        t, xy = pred
        hx = np.column_stack([np.interp(t, hc.t, hc.v[:, k]) for k in range(2)])
        err = np.linalg.norm(np.asarray(xy) - hx, axis=1)
        rms.append(float(np.sqrt(np.mean(err ** 2))))
        ends += float(err[-1])
    return float(np.mean(rms)), ends


def decompose(low: RunRecord, high: RunRecord, replayer: Optional[Replayer], segments,
              pose='pose.se2', anchor=''):
    """Per segment {t0, t1, label, motion, open, reset, free, local_sum, ...}.

    ``open`` and ``reset`` are RMS errors over sub-windows of ``replayer.horizon_s`` (None: the
    whole segment) re-anchored on the high run's state, so an open-loop unstable plant (a
    quadrotor) is judged on its n-step prediction, not on a divergent long replay. ``local_sum``
    adds the reset errors at the sub-window ends: the drift that dynamics alone would accumulate."""
    hc = high.channel(pose)
    lc = low.channel(pose)
    h0, l0 = start(high, anchor), start(low, anchor)
    H = getattr(replayer, 'horizon_s', None) if replayer else None
    rows = []
    for t0, t1, label in segments:
        e_open, _ = _local(replayer, 'open_loop', high, t0, t1, pose, H)
        e_reset, local_sum = _local(replayer, 'reset_closed_loop', high, t0, t1, pose, H)
        tt = hc.t[(hc.t >= t0) & (hc.t <= t1)]
        e_free = e_end = None
        if len(tt) >= 2 and (tt[-1] - h0) <= (lc.t[-1] - l0):
            lx = np.column_stack([np.interp(tt - h0 + l0, lc.t, lc.v[:, k]) for k in range(2)])
            hx = np.column_stack([np.interp(tt, hc.t, hc.v[:, k]) for k in range(2)])
            d = np.sqrt(((lx - hx) ** 2).sum(1))
            e_free, e_end = float(np.sqrt(np.mean(d ** 2))), float(d[-1])
        rows.append({'t0': float(t0), 't1': float(t1), 'label': label,
                     'motion': motion_class(high, t0, t1, pose), 'open': e_open, 'reset': e_reset,
                     'free': e_free, 'free_end': e_end, 'local_sum': local_sum, 'horizon_s': H})
    return rows


def uniform_segments(rec, seg_s=10.0):
    t0, t1 = rec.header.window[:2]
    edges = np.arange(t0, t1, seg_s)
    return [(float(a), float(min(a + seg_s, t1)), f'w{k}') for k, a in enumerate(edges) if min(a + seg_s, t1) - a > 1.0]


def pair_insights(low: RunRecord, high: RunRecord, replayer=None, segments=None, pose='pose.se2',
                  tol=0.5, h=2.0, anchor=''):
    """Insights for one pair: first divergence, time vs path residual, and the split."""
    out = []
    scope = {'pair': [low.run_id, high.run_id], 'tiers': [low.tier, high.tier]}
    al = aligned(low, high, pose, anchor=anchor)
    if al is None:
        return out, {}
    tau, a, b = al
    e_t = np.linalg.norm(a - b, axis=1)
    e_p = path_distance(a, high.channel(pose).v[:, :2])
    first = cusum_first(tau, e_t, tol, h)
    summary = {'time_rms': float(np.sqrt(np.mean(e_t ** 2))), 'path_rms': float(np.sqrt(np.mean(e_p ** 2))),
               'time_max': float(e_t.max()), 'path_max': float(e_p.max()), 'first_divergence_s': first,
               'duration_low': low.duration, 'duration_high': high.duration}
    if first is not None:
        k = int(np.searchsorted(tau, first))
        where = b[min(k, len(b) - 1)]
        out.append(Insight('divergence.first', 'warn',
                           f'{low.tier} leaves {high.tier} at t+{first:.1f} s near ({where[0]:.1f}, {where[1]:.1f}); '
                           f'time RMS {summary["time_rms"]:.2f} m, path RMS {summary["path_rms"]:.2f} m',
                           evidence=(Span(high.run_id, start(high, anchor) + first,
                                          start(high, anchor) + min(first + 10, tau[-1]), (pose,), 'diverge'),),
                           scope=scope, data=summary))
    ratio = summary['time_rms'] / max(summary['path_rms'], 1e-6)
    if summary['time_rms'] > tol and ratio > 2.5:
        out.append(Insight('divergence.pace', 'info',
                           f'same route, different pace: time residual {summary["time_rms"]:.2f} m vs path '
                           f'{summary["path_rms"]:.2f} m; durations {low.duration:.1f} s vs {high.duration:.1f} s',
                           scope=scope, data=summary, basis='descriptive'))
    elif summary['path_rms'] > tol:
        out.append(Insight('divergence.route', 'warn',
                           f'different routes: path residual {summary["path_rms"]:.2f} m (max {summary["path_max"]:.1f} m)',
                           scope=scope, data=summary))
    segs = segments if segments is not None else uniform_segments(high)
    rows = decompose(low, high, replayer, segs, pose, anchor) if segs else []
    summary['segments'] = rows
    split = split_summary(rows, tol)
    summary['split'] = split
    if split.get('dominant') and split['dominant'] != 'none':
        H = rows[0]['horizon_s'] if rows else None
        out.append(Insight('divergence.split', 'warn',
                           f'{split["dominant"]} origin of the {low.tier}/{high.tier} gap: '
                           + ', '.join(f'{k} {v:.2f} m' for k, v in split['mean'].items())
                           + (f' (local horizon {H:g} s)' if H else ' (per segment)')
                           + f', dynamics explain {split["explained"]:.0%}; worst in {split["worst_motion"]} segments'
                           + ('' if split['complete'] else f'; unavailable: {", ".join(split["missing"])}'),
                           scope=scope, data=split, basis='model-relative:replay'))
    return out, summary


def split_summary(rows, tol=0.5):
    """Origin of a pair's gap. ``explained`` = how much of the free residual the local (dynamics)
    drift accounts for; below 0.3 the gap is not dynamics: a decision or an upstream divergence.
    Among local terms, ``plant`` is the open-loop share of the reset error, ``interaction`` the rest.
    """
    by_motion = defaultdict(list)
    for r in rows:
        if r['free'] is not None:
            by_motion[r['motion']].append(r['free'])
    free = [r['free'] for r in rows if r['free'] is not None]
    if not free:
        return {}
    out = {'n_segments': len(rows), 'free_mean': float(np.mean(free)),
           'free_by_motion': {m: float(np.mean(v)) for m, v in by_motion.items()},
           'worst_motion': max(by_motion, key=lambda m: np.mean(by_motion[m])) if by_motion else None,
           'missing': sorted({k for r in rows for k in ('open', 'reset') if r[k] is None})}
    out['complete'] = not out['missing']
    resets = [r for r in rows if r['reset'] is not None and r['free'] is not None]
    if resets:
        # drift dynamics alone would accumulate by each segment end vs the residual there
        cum, ratios = 0.0, []
        for r in resets:
            cum += r['local_sum']
            if r['free_end'] is not None and r['free_end'] > tol:
                ratios.append(min(1.0, cum / r['free_end']))
        out['explained'] = float(np.mean(ratios)) if ratios else 1.0
        out['reset_mean'] = float(np.mean([r['reset'] for r in resets]))
        opens = [r['open'] for r in resets if r['open'] is not None]
        out['open_mean'] = float(np.mean(opens)) if opens else None
        if out['free_mean'] < tol and out['reset_mean'] < tol:
            out['dominant'] = 'none'
        elif out['explained'] < 0.3:
            out['dominant'] = 'decision/upstream'
        elif out['open_mean'] is None:
            out['dominant'] = 'execution'      # plant + controller + local planning, not separable
        else:
            share = out['open_mean'] / max(out['reset_mean'], 1e-9)
            out['plant_share'] = float(min(1.0, share))
            out['dominant'] = 'plant' if share >= 0.5 else 'interaction'
        out['mean'] = {k: v for k, v in (('open', out['open_mean']), ('reset', out['reset_mean']),
                                         ('free', out['free_mean'])) if v is not None}
    return out
