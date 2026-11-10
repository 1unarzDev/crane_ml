"""Divergence between two matched records of one run on two backends (layers).

Same scenario, seed and controller; different backend. The steps, each reported:

1. **Matching guard.** Pairing keys (scenario pair_key / family / seed / condition), controller id and
   params, clock dt. A different scenario, seed or controller id refuses the pair (``allow_mismatch``
   overrides and records the warning); different controller params or dt warn.
2. **Common time base.** When both records carry ticks with one dt, samples are placed on relative
   ticks (``tick - anchor``; anchor = the ``episode_start`` event's ``tick`` payload, else 0), so a
   constant tick-counter offset is harmless. Without ticks, the record clocks (seconds since episode
   start) are used. When dt differs both sides are resampled on the coarser grid (``resampled`` says
   so). Commands are sampled zero-order hold, states linearly (angles unwrapped).
3. **Residuals and the first meaningful disagreement.** Per signal (truth / obs position, heading,
   speed, yaw rate; cmd.requested; cmd.applied or cmd.sent; act.thrust) the residual between the two
   sides is smoothed by a trailing max over ``smooth_s``. A signal *exceeds* at the first sample where
   its smoothed residual is above the declared threshold and stays above for ``hold_s``; it *departs*
   at the first excursion above ``onset_frac`` x threshold lasting ``min_excursion_s`` (an upstream
   difference the closed loop cancels quickly still departs first) or where the exceeding excursion
   began, whichever is earlier. Only departures up to the first exceedance are ordered. The first
   meaningful disagreement is the earliest exceedance. The event timelines are compared with
   ``insight.events.align`` and the pose residuals summarised with ``insight.divergence``.
4. **Attribution**, tests in this order (the first that passes decides; see ``ATTRIBUTION_DOC``).
5. **Batch summary** over many pairs (``summarize_pairs``).

The core names no robot: channel names are the ``/2`` standard channels.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, is_dataclass, replace
import json
import math
from typing import Any, Callable, Mapping, Optional, Sequence

import numpy as np

from judge.insight import Insight
from judge.insight import divergence as _div
from judge.insight import events as _events
from judge.metric import Span
from judge.record import RunRecord

CAUSES = ('clock_or_timestamp', 'command_delivery', 'plant', 'sensor', 'controller', 'unknown')

ATTRIBUTION_DOC = """\
1 clock_or_timestamp: with ticks, the records' time bases (t - (tick - anchor) dt) differ by > 0.5 tick,
  or the obs.stamp conventions differ; without ticks, one record equals the other shifted in time
  (least-squares lag over truth, obs and cmd.requested; the shift removes >= 90% of the residual and
  leaves every signal under its threshold).
2-5 by the signal that departed first, in the causal loop obs -> cmd.requested -> cmd.applied/sent ->
  act.thrust -> truth -> obs (ties within tie_ticks go to the upstream member of a contiguous run):
  cmd.applied/sent first, requested still agreeing      -> command_delivery
  act.thrust or truth first, applied still agreeing      -> plant (thrust vs hull said when act.thrust exists)
  obs first, truth still agreeing                        -> sensor
  cmd.requested first, obs still agreeing                -> controller (same controller+seed: nondeterminism
                                                             or a version mismatch)
6 unknown: a channel upstream of the first departure is absent on either side and its cause differs
  (named: the channel that would resolve it), truth is absent, or the tied departures wrap the loop."""


# ------------------------------------------------------------------------------------ header

def header_block(rec: RunRecord, name: str) -> dict:
    """A ``/2`` header block as a dict: the header attribute when the schema has it, else
    ``provenance[name]`` (or ``provenance['v2'][name]``), else {}."""
    h = rec.header
    v = getattr(h, name, None)
    if v is None:
        prov = h.provenance or {}
        v = prov.get(name)
        if v is None and isinstance(prov.get('v2'), Mapping):
            v = prov['v2'].get(name)
    if v is None:
        return {}
    if is_dataclass(v):
        v = asdict(v)
    return dict(v) if isinstance(v, Mapping) else {'value': v}


# ------------------------------------------------------------------------------------ thresholds

@dataclass(frozen=True)
class Thresholds:
    """Declared thresholds (justifications in ``JUSTIFY``)."""
    pos_m: float = 0.10
    heading_rad: float = math.radians(5.0)
    speed_mps: float = 0.05
    yaw_rate_rps: float = 0.05
    cmd: float = 0.05              # normalised command, max over thrusters
    thrust_frac: float = 0.10      # of the larger side's peak |thrust|
    hold_s: float = 0.5
    onset_frac: float = 0.25
    min_excursion_s: float = 0.1
    smooth_s: float = 0.2
    tie_ticks: float = 1.0
    shift_explains: float = 0.9    # fraction of the residual a pure time shift must remove
    max_lag_s: float = 1.0
    window_before_s: float = 1.0
    window_after_s: float = 2.0

    def as_dict(self):
        return asdict(self)


JUSTIFY = {
    'pos_m': '0.10 m = 40% of the 0.25 m lateral docking tolerance; a difference that can flip a dock',
    'heading_rad': '5 deg = 1/3 of the 15 deg docking heading tolerance',
    'speed_mps': '0.05 m/s = 5/8 of the 0.08 m/s docking speed limit',
    'yaw_rate_rps': '0.05 rad/s accumulates 5 deg of heading in under 2 s',
    'cmd': '0.05 of full scale = the size of a thruster deadband (0..0.06 in the study ranges)',
    'thrust_frac': '10% of peak thrust = the 0.05 command threshold carried through the gamma~2 static map '
                   '(dF/F = gamma dq/q near full command); a smaller value lets thrust depart before the command '
                   'that caused it',
    'hold_s': '0.5 s (25 ticks at 50 Hz, 12 control steps): longer than a transient blip, shorter than a manoeuvre',
    'onset_frac': 'departure = start of the excursion above 25% of the threshold that leads into the exceedance',
    'min_excursion_s': '0.1 s (5 ticks): an onset-level excursion this long is a departure even if the closed loop '
                       'cancels it before hold_s; a one- or two-sample glitch is not',
    'smooth_s': '0.2 s trailing max: bridges sensor-noise dips without moving an onset earlier',
    'tie_ticks': 'departures within 1 tick are simultaneous; resolved by causal order within the tick',
    'shift_explains': 'a pure time shift must remove 90% of the RMS residual and leave every signal under threshold',
    'max_lag_s': 'lags searched up to 1 s (beyond that it is not a timestamp convention)',
}

# group -> cause, and the causal loop (each group's predecessor)
GROUP_CAUSE = {'obs': 'sensor', 'requested': 'controller', 'delivery': 'command_delivery', 'thrust': 'plant',
               'truth': 'plant'}
LOOP = ('obs', 'requested', 'delivery', 'thrust', 'truth')
GROUP_CHANNELS = {'obs': ('obs.pose', 'obs.twist'), 'requested': ('cmd.requested',),
                  'delivery': ('cmd.applied', 'cmd.sent'), 'thrust': ('act.thrust',),
                  'truth': ('truth.pose', 'truth.twist')}
Q = ('q0', 'q1', 'q2', 'q3')
P = ('p0', 'p1', 'p2', 'p3')


# ------------------------------------------------------------------------------------ guard

def guard(a: RunRecord, b: RunRecord, allow_mismatch=False):
    """{'status': ok|warn|refused, 'issues': [{code, severity, a, b}]}."""
    issues = []

    def chk(code, va, vb, severity):
        if va != vb:
            issues.append({'code': code, 'severity': severity, 'a': va, 'b': vb})
    sa, sb = a.header.scenario, b.header.scenario
    chk('scenario.pair_key', sa.pair_key, sb.pair_key, 'refuse')
    chk('scenario.seed', sa.seed, sb.seed, 'refuse')
    chk('scenario.family', sa.family, sb.family, 'refuse')
    chk('scenario.condition', json.dumps(dict(sa.condition or {}), sort_keys=True, default=str),
        json.dumps(dict(sb.condition or {}), sort_keys=True, default=str), 'warn')
    ca, cb = header_block(a, 'controller'), header_block(b, 'controller')
    chk('controller.id', ca.get('id'), cb.get('id'), 'refuse')
    chk('controller.params', json.dumps(ca.get('params'), sort_keys=True, default=str),
        json.dumps(cb.get('params'), sort_keys=True, default=str), 'warn')
    chk('controller.spec_sha', ca.get('spec_sha'), cb.get('spec_sha'), 'warn')
    da, db = record_dt(a), record_dt(b)
    if da and db and abs(da - db) > 1e-9:
        issues.append({'code': 'clock.dt_mismatch', 'severity': 'warn', 'a': da, 'b': db})
    ba, bb = header_block(a, 'backend'), header_block(b, 'backend')
    if ba and bb and (ba.get('layer'), ba.get('name')) == (bb.get('layer'), bb.get('name')):
        issues.append({'code': 'backend.same', 'severity': 'info', 'a': ba.get('name'), 'b': bb.get('name')})
    refuse = [i for i in issues if i['severity'] == 'refuse']
    if refuse and allow_mismatch:
        for i in refuse:
            i['severity'] = 'warn'
            i['overridden'] = True
        refuse = []
    status = 'refused' if refuse else 'warn' if any(i['severity'] == 'warn' for i in issues) else 'ok'
    return {'status': status, 'issues': issues}


# ------------------------------------------------------------------------------------ clock

def _ticks(ch):
    tk = getattr(ch, 'tick', None)
    if tk is None and 'tick' in ch.spec.fields:
        tk = ch.col('tick')
    return None if tk is None else np.asarray(tk, float)


def record_dt(rec):
    """Plant tick dt: header clock_mode.dt_s, else estimated from truth.pose ticks, else None."""
    dt = header_block(rec, 'clock_mode').get('dt_s')
    if dt:
        return float(dt)
    for name in ('truth.pose', 'cmd.applied', 'obs.pose'):
        if rec.has(name):
            ch = rec.channel(name)
            tk = _ticks(ch)
            if tk is not None and len(tk) > 2:
                d = np.diff(tk)
                ok = d > 0
                if ok.any():
                    return float(np.median(np.diff(ch.t)[ok] / d[ok]))
    return None


def anchor_tick(rec):
    ev = rec.events_of('episode_start')
    if ev and isinstance(ev[0].payload, Mapping) and ev[0].payload.get('tick') is not None:
        return float(ev[0].payload['tick'])
    return 0.0


def time_base_offset(rec, dt):
    """median(t - (tick - anchor) dt) over the channels that carry ticks, or None."""
    a0 = anchor_tick(rec)
    vals = []
    for name, ch in rec.channels.items():
        if not rec.has(name):
            continue
        tk = _ticks(ch)
        if tk is not None:
            vals.append(np.median(ch.t - (tk - a0) * dt))
    return float(np.median(vals)) if vals else None


@dataclass
class Base:
    """How one record's samples map to the common grid time tau."""
    use_ticks: bool
    dt: Optional[float]
    anchor: float = 0.0
    shift: float = 0.0              # tau = t - shift (time base) or (tick - anchor) dt - shift

    def tau(self, ch):
        tk = _ticks(ch) if self.use_ticks else None
        if tk is not None:
            return (tk - self.anchor) * self.dt - self.shift
        return np.asarray(ch.t, float) - self.shift

    def to_record_t(self, rec, tau):
        """Record clock at grid times (via truth.pose)."""
        if not rec.has('truth.pose'):
            return tau + self.shift
        ch = rec.channel('truth.pose')
        return np.interp(tau, self.tau(ch), ch.t)


# ------------------------------------------------------------------------------------ sampling

def _sample(rec, base: Base, name, fields, grid, hold=False, angles=()):
    """[len(grid), len(fields)] or None when the channel is absent."""
    if not rec.has(name):
        return None
    ch = rec.channel(name)
    if any(f not in ch.spec.fields for f in fields):
        return None
    tau = base.tau(ch)
    out = np.empty((len(grid), len(fields)))
    if hold:
        idx = np.clip(np.searchsorted(tau, grid + 1e-9, 'right') - 1, 0, len(tau) - 1)
    for k, f in enumerate(fields):
        col = ch.col(f)
        if hold:
            out[:, k] = col[idx]
        elif f in angles or f in ch.spec.angles:
            out[:, k] = np.interp(grid, tau, np.unwrap(col))
        else:
            out[:, k] = np.interp(grid, tau, col)
    return out


def _wrapd(d):
    return (d + math.pi) % (2 * math.pi) - math.pi


def _sent_as_q(rec, base, grid):
    s = _sample(rec, base, 'cmd.sent', P, grid, hold=True)
    return None if s is None else (s - 1500.0) / 400.0


def sample_side(rec, base, grid):
    """Every standard signal of one side on the grid: {key: array or None}."""
    out = {}
    for src in ('truth', 'obs'):
        out[f'{src}.pose'] = _sample(rec, base, f'{src}.pose', ('x', 'y', 'yaw'), grid, angles=('yaw',))
        out[f'{src}.twist'] = _sample(rec, base, f'{src}.twist', ('u', 'v', 'r'), grid)
    out['cmd.requested'] = _sample(rec, base, 'cmd.requested', Q, grid, hold=True)
    out['cmd.applied'] = _sample(rec, base, 'cmd.applied', Q, grid, hold=True)
    out['cmd.sent'] = _sent_as_q(rec, base, grid)
    out['act.thrust'] = _sample(rec, base, 'act.thrust', ('f0', 'f1', 'f2', 'f3'), grid, hold=True)
    return out


def residuals(sa, sb, th: Thresholds):
    """{signal: (group, residual array, threshold, channels)} for every signal present on both sides."""
    out = {}
    for src in ('truth', 'obs'):
        pa, pb = sa[f'{src}.pose'], sb[f'{src}.pose']
        if pa is not None and pb is not None:
            out[f'{src}.pos'] = (src, np.hypot(pa[:, 0] - pb[:, 0], pa[:, 1] - pb[:, 1]), th.pos_m, (f'{src}.pose',))
            out[f'{src}.heading'] = (src, np.abs(_wrapd(pa[:, 2] - pb[:, 2])), th.heading_rad, (f'{src}.pose',))
        ta, tb = sa[f'{src}.twist'], sb[f'{src}.twist']
        if ta is not None and tb is not None:
            out[f'{src}.speed'] = (src, np.abs(np.hypot(ta[:, 0], ta[:, 1]) - np.hypot(tb[:, 0], tb[:, 1])),
                                   th.speed_mps, (f'{src}.twist',))
            out[f'{src}.yaw_rate'] = (src, np.abs(ta[:, 2] - tb[:, 2]), th.yaw_rate_rps, (f'{src}.twist',))
    if sa['cmd.requested'] is not None and sb['cmd.requested'] is not None:
        out['cmd.requested'] = ('requested', np.abs(sa['cmd.requested'] - sb['cmd.requested']).max(1), th.cmd,
                                ('cmd.requested',))
    for name in ('cmd.applied', 'cmd.sent'):
        if sa[name] is not None and sb[name] is not None:
            out[name] = ('delivery', np.abs(sa[name] - sb[name]).max(1), th.cmd, (name,))
            break
    fa, fb = sa['act.thrust'], sb['act.thrust']
    if fa is not None and fb is not None:
        peak = max(float(np.abs(fa).max()), float(np.abs(fb).max()), 1e-9)
        out['act.thrust'] = ('thrust', np.abs(fa - fb).max(1) / peak, th.thrust_frac, ('act.thrust',))
    return out


def present_groups(a, b):
    """Groups whose channels exist on BOTH sides, and {group: [absent channel on side]}."""
    present, missing = set(), {}
    for g, names in GROUP_CHANNELS.items():
        ok = [n for n in names if a.has(n) and b.has(n)]
        if ok:
            present.add(g)
        else:
            missing[g] = [f'{n} on {side}' for n in names[:1] for side, r in (('A', a), ('B', b)) if not r.has(n)]
    return present, missing


# ------------------------------------------------------------------------------------ onsets

def _trailing_max(r, w):
    out = r.copy()
    for s in range(1, max(1, w)):
        out[s:] = np.maximum(out[s:], r[:-s])
    return out


def _persistent(mask, H):
    """First k with mask[k : k+H] all True (or all True to the end when fewer remain), else None."""
    n = len(mask)
    if n == 0:
        return None
    bad = np.concatenate([[0], np.cumsum(~mask)])
    for k in np.flatnonzero(mask):
        j = min(n, k + H)
        if bad[j] - bad[k] == 0 and (j - k >= H or j - k >= max(1, H // 5)):
            return int(k)
    return None


def _sustained(mask, M):
    """First k starting a run of at least M True samples (a shorter run at the very end counts)."""
    n = len(mask)
    k = 0
    while k < n:
        if mask[k]:
            j = k
            while j < n and mask[j]:
                j += 1
            if j - k >= M or j == n and j - k >= max(1, M // 2):
                return int(k)
            k = j
        else:
            k += 1
    return None


def onsets(res, dt, th: Thresholds):
    """{signal: {exceed_i, depart_i, peak, group, threshold, sub_threshold}} on the grid."""
    H = max(1, int(round(th.hold_s / dt)))
    W = max(1, int(round(th.smooth_s / dt)))
    M = max(1, int(round(th.min_excursion_s / dt)))
    out = {}
    for name, (group, r, thr, chans) in res.items():
        r = np.nan_to_num(np.asarray(r, float))
        rs = _trailing_max(r, W)
        k = _persistent(rs > thr, H)
        lev = th.onset_frac * thr
        # departure: the first excursion above the onset level that lasts min_excursion_s (an
        # upstream difference the closed loop cancels quickly still departs first), or the start of
        # the excursion that leads into the exceedance, whichever is earlier
        j = _sustained(rs > lev, M)
        if k is not None:
            w = k
            while w > 0 and rs[w - 1] > lev:
                w -= 1
            j = w if j is None else min(j, w)
        sub = k is None and j is not None
        out[name] = {'group': group, 'threshold': thr, 'channels': chans, 'peak': float(r.max()) if len(r) else 0.0,
                     'exceed_i': k, 'depart_i': j, 'sub_threshold': sub}
    return out


# ------------------------------------------------------------------------------------ result

@dataclass
class LayerDivergence:
    pair: tuple
    backends: tuple
    verdict: str                    # agree | diverged | refused
    attribution: Optional[str]
    first_s: Optional[float]        # first meaningful disagreement, grid time (s since episode start)
    first_signal: Optional[str]
    guard: Mapping[str, Any]
    alignment: Mapping[str, Any]
    tests: list
    departures: list
    resolve_with: list
    events: Mapping[str, Any]
    residual: Mapping[str, Any]
    quantify: Mapping[str, Any]
    evidence: Mapping[str, Any]
    explanation: str
    outcome: Mapping[str, Any] = field(default_factory=dict)
    secondary: Optional[Mapping[str, Any]] = None
    insight: Optional[Insight] = None

    def as_dict(self):
        d = {k: getattr(self, k) for k in ('pair', 'backends', 'verdict', 'attribution', 'first_s', 'first_signal',
                                            'guard', 'alignment', 'tests', 'departures', 'resolve_with', 'events',
                                            'residual', 'quantify', 'outcome', 'secondary', 'explanation')}
        d['evidence'] = {k: v for k, v in self.evidence.items() if k != 'rows'}
        d['insight'] = self.insight.as_dict() if self.insight else None
        return _jsonable(d)


def _jsonable(x):
    if isinstance(x, Mapping):
        return {str(k): _jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_jsonable(v) for v in x]
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, (np.floating, float)):
        return None if not math.isfinite(float(x)) else float(x)
    if isinstance(x, np.ndarray):
        return _jsonable(x.tolist())
    if isinstance(x, (np.bool_,)):
        return bool(x)
    return x


# ------------------------------------------------------------------------------------ main

def _grid_bases(a, b, th):
    """(grid, base_a, base_b, info)."""
    da, db = record_dt(a), record_dt(b)
    ta = a.has('truth.pose') and _ticks(a.channel('truth.pose')) is not None
    tb = b.has('truth.pose') and _ticks(b.channel('truth.pose')) is not None
    same_dt = bool(da and db and abs(da - db) < 1e-9)
    use_ticks = bool(ta and tb and same_dt)
    dt = max(da or 0.0, db or 0.0) or 0.02
    ba = Base(use_ticks, da, anchor_tick(a))
    bb = Base(use_ticks, db, anchor_tick(b))
    info = {'time_base': 'ticks' if use_ticks else 'record clock', 'dt_a': da, 'dt_b': db, 'grid_dt': dt,
            'resampled': not same_dt, 'tick_anchor': [ba.anchor, bb.anchor],
            'tick_offset': (bb.anchor - ba.anchor) if use_ticks else None}
    if not same_dt:
        info['note'] = (f'dt differs ({da} vs {db} s): both sides resampled on the coarser {dt:g} s grid '
                        '(states linear, commands zero-order hold)')
    return dt, ba, bb, info


def _span(a, b, ba, bb, dt):
    ends = []
    for rec, base in ((a, ba), (b, bb)):
        taus = [base.tau(rec.channel(n)) for n in rec.channels if rec.has(n)]
        ends.append((min(float(t[0]) for t in taus), max(float(t[-1]) for t in taus)) if taus else (0.0, 0.0))
    t0, t1 = max(0.0, max(e[0] for e in ends)), min(e[1] for e in ends)
    if t1 <= t0 + dt:
        return np.zeros(0)
    return np.arange(0.0, t1 - t0 + 1e-9, dt) + t0


def _lag_scan(sa, sb, grid, dt, th):
    """Least-squares lag (B relative to A, seconds) over truth/obs/requested normalised by thresholds."""
    keys = [('truth.pose', (0, 1), th.pos_m), ('truth.twist', (0, 1, 2), th.speed_mps),
            ('obs.pose', (0, 1), th.pos_m), ('cmd.requested', (0, 1, 2, 3), th.cmd)]
    A, B = [], []
    for k, cols, s in keys:
        if sa[k] is not None and sb[k] is not None:
            A.append(sa[k][:, cols] / s)
            B.append(sb[k][:, cols] / s)
    if not A:
        return None
    A, B = np.hstack(A), np.hstack(B)
    L = int(round(th.max_lag_s / dt))
    n = len(grid)
    if n < 4 * L:
        L = max(1, n // 4)
    lags = np.arange(-L, L + 1)
    rms = np.empty(len(lags))
    for i, l in enumerate(lags):
        if l >= 0:
            d = A[:n - l] - B[l:]
        else:
            d = A[-l:] - B[:n + l]
        rms[i] = float(np.sqrt(np.mean(d ** 2))) if len(d) else np.inf
    i = int(np.argmin(rms))
    frac = 0.0
    if 0 < i < len(lags) - 1:
        y0, y1, y2 = rms[i - 1], rms[i], rms[i + 1]
        den = y0 - 2 * y1 + y2
        frac = 0.5 * (y0 - y2) / den if den > 0 else 0.0
    return {'lag_s': float((lags[i] + frac) * dt), 'lag_ticks': int(lags[i]), 'rms_at_lag': float(rms[i]),
            'rms_at_zero': float(rms[L])}


def _evidence(a, b, ba, bb, grid, sa, sb, res, t_ref, th):
    lo = max(grid[0], t_ref - th.window_before_s)
    hi = min(grid[-1], t_ref + th.window_after_s)
    i0, i1 = int(np.searchsorted(grid, lo - 1e-9)), int(np.searchsorted(grid, hi + 1e-9))
    cols, rows = ['tau'], [grid[i0:i1]]
    for side, rec, base, s in (('a', a, ba, sa), ('b', b, bb, sb)):
        cols.append(f'{side}.t')
        rows.append(base.to_record_t(rec, grid[i0:i1]))
        for key, fields in (('cmd.requested', Q), ('cmd.applied', Q), ('cmd.sent', Q),
                            ('act.thrust', ('f0', 'f1', 'f2', 'f3')), ('obs.pose', ('x', 'y', 'yaw')),
                            ('obs.twist', ('u', 'v', 'r')), ('truth.pose', ('x', 'y', 'yaw')),
                            ('truth.twist', ('u', 'v', 'r'))):
            if s.get(key) is None:
                continue
            for k, f in enumerate(fields):
                cols.append(f'{side}.{key}.{f}')
                v = s[key][i0:i1, k]
                rows.append(_wrapd(v) if f == 'yaw' else v)
    for name, (_, r, _, _) in res.items():
        cols.append(f'res.{name}')
        rows.append(r[i0:i1])
    idx = {}
    for side, rec, base in (('a', a, ba), ('b', b, bb)):
        idx[side] = {}
        for n in rec.channels:
            if rec.has(n):
                tau = base.tau(rec.channel(n))
                idx[side][n] = [int(np.searchsorted(tau, lo - 1e-9)), int(np.searchsorted(tau, hi + 1e-9))]
    return {'t0': float(lo), 't1': float(hi), 'grid_index': [i0, i1], 'sample_index': idx, 'columns': cols,
            'rows': np.column_stack(rows) if rows and len(rows[0]) else np.zeros((0, len(cols)))}


def evidence_csv(result: LayerDivergence) -> str:
    ev = result.evidence
    lines = [','.join(ev['columns'])]
    for row in ev['rows']:
        lines.append(','.join('' if not np.isfinite(x) else f'{x:.6g}' for x in row))
    return '\n'.join(lines) + '\n'


def _order(departs, present, missing, tie):
    """(attribution or None, ordered departures, test notes, resolve_with)."""
    by_group = {}
    for d in departs:
        g = d['group']
        if g not in by_group or d['depart_s'] < by_group[g]['depart_s']:
            by_group[g] = d
    if not by_group:
        return None, [], ['no signal departs'], []
    first_t = min(d['depart_s'] for d in by_group.values())
    tied = {g for g, d in by_group.items() if d['depart_s'] <= first_t + tie}
    notes = [f'earliest departure t={first_t:.3f} s in {sorted(tied)}']
    # contiguous run on the loop -> its upstream member
    lead = None
    if len(tied) == len(LOOP):
        return 'unknown', by_group, notes + ['every group departs together: the loop gives no order'], []
    for g in tied:
        prev = LOOP[(LOOP.index(g) - 1) % len(LOOP)]
        if prev not in tied:
            if lead is not None:
                return 'unknown', by_group, notes + [f'tied departures {sorted(tied)} are not one causal run'], []
            lead = g
    notes.append(f'causal lead {lead}')
    if 'truth' not in present:
        return 'unknown', by_group, notes + ['truth absent on a side: states cannot be compared'], \
            missing.get('truth', ['truth.pose'])
    # walk upstream through absent groups: a hidden departure there would change the cause
    cause = GROUP_CAUSE[lead]
    hidden, resolve = [], []
    g = lead
    while True:
        g = LOOP[(LOOP.index(g) - 1) % len(LOOP)]
        if g in present or g == lead:
            break
        if GROUP_CAUSE[g] != cause:
            hidden.append(GROUP_CAUSE[g])
            resolve += missing.get(g, list(GROUP_CHANNELS[g]))
    if hidden:
        return 'unknown', by_group, notes + [
            f'{lead} departs first, but upstream {", ".join(resolve)} is absent: {cause} and '
            f'{", ".join(sorted(set(hidden)))} both fit'], resolve
    return cause, by_group, notes, []


def _quantify(cause, sa, sb, grid, dt, by_group, res, th, present, all_departs=()):
    q = {}
    if not by_group:
        return q
    t_lead = min(d['depart_s'] for d in by_group.values())
    w = (grid >= t_lead - 0.25) & (grid <= t_lead + 1.0)
    if not w.any():
        return q
    if cause == 'plant':
        # the window in which the applied commands still agree: from the lead departure to the
        # delivery departure (at least 4 ticks)
        dels = [d['depart_s'] for d in all_departs if d['group'] == 'delivery']
        t_del = min(dels) if dels else t_lead + 1.0
        w = (grid >= t_lead) & (grid < max(t_del, t_lead + 4 * dt) - 1e-9)
        q['same_command_window_s'] = [float(t_lead), float(max(t_del, t_lead + 4 * dt))]
        if 'cmd.applied' in res or 'cmd.sent' in res:
            key = 'cmd.applied' if 'cmd.applied' in res else 'cmd.sent'
            q['applied_max_diff_in_window'] = float(res[key][1][w].max())
        for side, s in (('a', sa), ('b', sb)):
            tw = s['truth.twist']
            if tw is not None:
                acc = np.gradient(tw[:, 0], grid)[w]
                ra = np.gradient(tw[:, 2], grid)[w]
                q[f'{side}.mean_surge_accel'] = float(np.mean(acc))
                q[f'{side}.mean_yaw_accel'] = float(np.mean(ra))
        if 'a.mean_surge_accel' in q and abs(q['a.mean_surge_accel']) > 1e-9:
            q['surge_accel_ratio_b_over_a'] = q['b.mean_surge_accel'] / q['a.mean_surge_accel']
        if 'thrust' in present:
            th_d, tr_d = by_group.get('thrust'), by_group.get('truth')
            q['response'] = ('thrust' if th_d is not None and (tr_d is None or th_d['depart_s'] <= tr_d['depart_s'] + dt)
                             else 'hull')
        else:
            q['response'] = 'thrust or hull (act.thrust absent: cannot split)'
    if cause == 'command_delivery':
        key = 'cmd.applied' if sa['cmd.applied'] is not None and sb['cmd.applied'] is not None else 'cmd.sent'
        A, B = sa[key], sb[key]
        m = (grid >= t_lead - 0.5) & (grid <= t_lead + 2.0)
        A, B = A[m], B[m]
        L = int(round(th.max_lag_s / dt))
        best = (np.inf, 0)
        for l in range(-L, L + 1):
            d = A[:len(A) - l] - B[l:] if l >= 0 else A[-l:] - B[:len(B) + l]
            if len(d) > 5:
                e = float(np.sqrt(np.mean(d ** 2)))
                if e < best[0] - 1e-12:
                    best = (e, l)
        q['delivery_lag_s'] = best[1] * dt          # >0: B applies later than A
        q['delivery_rms_after_lag'] = best[0]
    if cause == 'sensor':
        for side, s in (('a', sa), ('b', sb)):
            if s['obs.pose'] is not None and s['truth.pose'] is not None:
                e = np.hypot(*(s['obs.pose'][:, :2] - s['truth.pose'][:, :2]).T)
                q[f'{side}.obs_pos_error_rms'] = float(np.sqrt(np.mean(e ** 2)))
                tw, ow = s['truth.twist'], s['obs.twist']
                if tw is not None and ow is not None:
                    best = (np.inf, 0)
                    for l in range(0, int(round(th.max_lag_s / dt)) + 1):
                        d = ow[l:, 0] - tw[:len(tw) - l, 0]
                        if len(d) > 10:
                            e2 = float(np.sqrt(np.mean(d ** 2)))
                            if e2 < best[0] - 1e-12:
                                best = (e2, l)
                    q[f'{side}.obs_latency_s'] = best[1] * dt
    return q


def _event_compare(a, b, kinds, tol_s):
    ta = [(t, l, e) for t, l, e in _events.timeline(a, kinds)]
    tb = [(t, l, e) for t, l, e in _events.timeline(b, kinds)]
    if not ta and not tb:
        return {'n_a': 0, 'n_b': 0, 'first_difference': None}
    al = _events.align(ta, tb, tol_s)
    first = next((x for x in al if x[2] != 'match'), None)
    out = {'n_a': len(ta), 'n_b': len(tb), 'matched': sum(1 for x in al if x[2] == 'match')}
    if first is None:
        out['first_difference'] = None
    else:
        i, j, kind = first
        out['first_difference'] = {'kind': kind, 'a': None if i is None else [ta[i][0], ta[i][1]],
                                   'b': None if j is None else [tb[j][0], tb[j][1]]}
    return out


DEFAULT_EVENT_KINDS = ('gate_cross', 'contact', 'dock_enter', 'dock_hold_start', 'docked', 'timeout', 'abort',
                       'link_lost')


def diverge(a: RunRecord, b: RunRecord, th: Thresholds = Thresholds(), allow_mismatch=False,
            outcome: Optional[Callable[[RunRecord], tuple]] = None, event_kinds=DEFAULT_EVENT_KINDS,
            event_tol_s=0.25) -> LayerDivergence:
    """Divergence of B from A (A is the reference side; lags are B relative to A).

    ``outcome(rec) -> (category, t_decided)`` (a pack's outcome) lets the result say whether the
    divergence precedes the outcome difference."""
    g = guard(a, b, allow_mismatch)
    pair = (a.run_id, b.run_id)
    backends = (header_block(a, 'backend').get('name') or a.header.source,
                header_block(b, 'backend').get('name') or b.header.source)
    if g['status'] == 'refused':
        why = '; '.join(f"{i['code']} {i['a']!r} vs {i['b']!r}" for i in g['issues'] if i['severity'] == 'refuse')
        return LayerDivergence(pair, backends, 'refused', None, None, None, g, {}, [], [], [], {}, {}, {}, {},
                               f'refused: not a matched pair ({why}); pass allow_mismatch to compare anyway')
    dt, ba, bb, align_info = _grid_bases(a, b, th)
    tests = []
    resolve = []
    # ---- (1) clock / timestamp
    clock = None
    if ba.use_ticks:
        oa, ob = time_base_offset(a, ba.dt), time_base_offset(b, bb.dt)
        align_info['time_base_offset'] = [oa, ob]
        tb = _tick_base_onset(a, b, ba, bb, dt)
        if tb is not None:
            clock = {'how': 'tick base', **tb}
        tests.append({'test': 'clock.tick_base', 'result': 'pass' if clock else 'fail',
                      'detail': f'time-base offsets A {oa} s, B {ob} s (median of t - (tick - anchor) dt)'
                                + (f"; B - A exceeds 0.5 tick from t={tb['onset_s']:.2f} s" if tb else '')})
    stamp = _stamp_convention(a, b)
    if stamp is not None:
        sa_off, sb_off = stamp
        bad = abs(sb_off - sa_off) > 0.5 * dt
        tests.append({'test': 'clock.obs_stamp', 'result': 'pass' if bad else 'fail',
                      'detail': f'median(row t - t_recv) A {sa_off:.4f} s, B {sb_off:.4f} s'})
        if bad and clock is None:
            clock = {'how': 'obs.stamp convention', 'offset_s': sb_off - sa_off, 'offset_ticks': (sb_off - sa_off) / dt}
    grid = _span(a, b, ba, bb, dt)
    if len(grid) < 3:
        return LayerDivergence(pair, backends, 'agree', None, None, None, g, align_info, tests, [], [], {}, {}, {},
                               {}, 'no common time span to compare')
    sa, sb = sample_side(a, ba, grid), sample_side(b, bb, grid)
    if not ba.use_ticks:
        lag = _lag_scan(sa, sb, grid, dt, th)
        if lag is not None:
            align_info['lag_scan'] = lag
            shifted = None
            ok = (lag['lag_ticks'] != 0 and lag['rms_at_zero'] > 0
                  and lag['rms_at_lag'] <= (1 - th.shift_explains) * lag['rms_at_zero'])
            if ok:
                bb2 = replace(bb, shift=lag['lag_s'])
                grid2 = _span(a, b, ba, bb2, dt)
                sb2 = sample_side(b, bb2, grid2)
                sa2 = sample_side(a, ba, grid2)
                on2 = onsets(residuals(sa2, sb2, th), dt, th)
                shifted = [n for n, o in on2.items() if o['exceed_i'] is not None]
                ok = not shifted
            tests.append({'test': 'clock.pure_shift', 'result': 'pass' if ok else 'fail',
                          'detail': f"best lag {lag['lag_s']:+.3f} s: RMS {lag['rms_at_zero']:.3g} -> "
                                    f"{lag['rms_at_lag']:.3g} (normalised)"
                                    + (f'; still exceeding after the shift: {shifted}' if shifted else '')})
            if ok and clock is None:
                clock = {'how': 'pure time shift (least-squares lag)', 'offset_s': lag['lag_s'],
                         'offset_ticks': lag['lag_s'] / dt}
    # ---- residuals on the (corrected) common base
    if clock is not None:
        if ba.use_ticks:
            pass                                    # ticks already align the samples; offset is in the time base
        elif clock['how'].startswith('pure'):
            bb = replace(bb, shift=clock['offset_s'])
            grid = _span(a, b, ba, bb, dt)
            sa, sb = sample_side(a, ba, grid), sample_side(b, bb, grid)
    res = residuals(sa, sb, th)
    on = onsets(res, dt, th)
    present, missing = present_groups(a, b)
    departs = []
    for name, o in on.items():
        if o['depart_i'] is None:
            continue
        departs.append({'signal': name, 'group': o['group'], 'depart_s': float(grid[o['depart_i']]),
                        'exceed_s': None if o['exceed_i'] is None else float(grid[o['exceed_i']]),
                        'peak': o['peak'], 'threshold': o['threshold'], 'sub_threshold': o['sub_threshold']})
    departs.sort(key=lambda d: (d['depart_s'], LOOP.index(d['group'])))
    exceed = [d for d in departs if d['exceed_s'] is not None]
    first = min(exceed, key=lambda d: d['exceed_s']) if exceed else None
    cause, by_group, notes, resolve = (None, {}, ['no signal exceeds its threshold'], [])
    if first is not None:
        relevant = [d for d in departs if d['depart_s'] <= first['exceed_s'] + 1e-9]
        cause, by_group, notes, resolve = _order(relevant, present, missing, th.tie_ticks * dt + 1e-9)
    tests.append({'test': 'signal_order', 'result': cause or 'none', 'detail': '; '.join(notes)})
    # ---- verdict
    if clock is not None:
        secondary = {'attribution': cause, 'first_s': None if first is None else first['exceed_s'],
                     'first_signal': None if first is None else first['signal'], 'notes': notes,
                     'resolve_with': resolve}
        # a time-base or stamp offset is present from the first sample; a shift found by the lag scan
        # shows where the uncorrected residual first exceeded (0 when it never did)
        attribution, first_sig = 'clock_or_timestamp', 'time_base'
        first_s = float(clock.get('onset_s', 0.0))
        if clock['how'].startswith('pure'):
            first_s = _uncorrected_first(a, b, ba, replace(bb, shift=0.0), dt, th)
        verdict = 'diverged'
    else:
        secondary = None
        attribution = cause
        first_s = None if first is None else first['exceed_s']
        first_sig = None if first is None else first['signal']
        verdict = 'diverged' if first is not None else 'agree'
    quant = _quantify(cause, sa, sb, grid, dt, by_group if isinstance(by_group, dict) else {}, res, th, present,
                      departs)
    if clock is not None:
        quant = {**quant, 'clock': clock}
    # ---- pose residual summary (insight.divergence) and event timelines (insight.events)
    resid = {}
    if sa['truth.pose'] is not None and sb['truth.pose'] is not None:
        e_t = np.hypot(*(sa['truth.pose'][:, :2] - sb['truth.pose'][:, :2]).T)
        e_p = _div.path_distance(sa['truth.pose'][::5, :2], sb['truth.pose'][:, :2])
        resid = {'time_rms_m': float(np.sqrt(np.mean(e_t ** 2))), 'time_max_m': float(e_t.max()),
                 'path_rms_m': float(np.sqrt(np.mean(e_p ** 2))), 'path_max_m': float(e_p.max()),
                 'cusum_first_s': _div.cusum_first(grid, e_t, th.pos_m, th.pos_m * 1.0),
                 'span_s': [float(grid[0]), float(grid[-1])]}
    evc = _event_compare(a, b, event_kinds, event_tol_s)
    # ---- outcome
    out = {}
    if outcome is not None:
        try:
            ca, cb = outcome(a), outcome(b)
            out = {'a': ca[0], 'b': cb[0], 'differ': ca[0] != cb[0], 't_decided': [ca[1], cb[1]]}
            if out['differ']:
                td = min(x for x in (ca[1], cb[1]) if x is not None) if any(x is not None for x in (ca[1], cb[1])) else None
                out['divergence_precedes_outcome'] = (first_s is not None and td is not None and first_s < td)
        except Exception as e:                       # an outcome bug never sinks a divergence report
            out = {'error': f'{type(e).__name__}: {e}'}
    t_ref = first_s if first_s is not None else (departs[0]['depart_s'] if departs else float(grid[0]))
    if clock is not None and first is not None:
        t_ref = first['exceed_s']
    elif by_group:
        t_ref = min(d['depart_s'] for d in by_group.values())
    ev = _evidence(a, b, ba, bb, grid, sa, sb, res, t_ref, th)
    text = _explain(a, b, backends, verdict, attribution, first_s, first_sig, by_group, departs, quant, clock,
                    secondary, resolve, g, align_info, evc, out, th)
    span_ch = tuple(sorted({c for d in departs for c in on[d['signal']]['channels']})) or ('truth.pose',)
    ins = Insight('layers.divergence' if verdict == 'diverged' else 'layers.agree',
                  'warn' if verdict == 'diverged' else 'info', text.splitlines()[0],
                  evidence=(Span(a.run_id, float(ev['t0']), float(ev['t1']), span_ch, 'evidence'),
                            Span(b.run_id, float(ev['t0']), float(ev['t1']), span_ch, 'evidence')),
                  scope={'pair': list(pair), 'backends': list(backends)},
                  basis='descriptive:signal-order', data={'attribution': attribution, 'first_s': first_s})
    return LayerDivergence(pair, backends, verdict, attribution, first_s, first_sig, g, align_info, tests,
                           departs, resolve, evc, resid, quant, ev, text, out, secondary, ins)


def _tick_base_onset(a, b, ba, bb, dt):
    """Per tick, the difference of the two records' time bases (t - (tick - anchor) dt) on
    truth.pose; the first tick from which it stays above half a tick, the offset after it."""
    if not (a.has('truth.pose') and b.has('truth.pose')):
        return None
    ca, cb = a.channel('truth.pose'), b.channel('truth.pose')
    ka, kb = _ticks(ca) - ba.anchor, _ticks(cb) - bb.anchor
    common, ia, ib = np.intersect1d(ka, kb, return_indices=True)
    if not len(common):
        return None
    d = (cb.t[ib] - kb[ib] * bb.dt) - (ca.t[ia] - ka[ia] * ba.dt)
    bad = np.abs(d) > 0.5 * dt
    k = _sustained(bad, 1) if bad.any() else None
    if k is None:
        return None
    off = float(np.median(d[k:][bad[k:]]))
    return {'offset_s': off, 'offset_ticks': off / dt, 'onset_s': float(common[k] * dt),
            'affected_fraction': float(bad.mean())}


def _uncorrected_first(a, b, ba, bb, dt, th):
    grid = _span(a, b, ba, bb, dt)
    on = onsets(residuals(sample_side(a, ba, grid), sample_side(b, bb, grid), th), dt, th)
    ks = [o['exceed_i'] for o in on.values() if o['exceed_i'] is not None]
    return float(grid[min(ks)]) if ks else 0.0


def _stamp_convention(a, b):
    vals = []
    for r in (a, b):
        if not r.has('obs.stamp'):
            return None
        ch = r.channel('obs.stamp')
        vals.append(float(np.median(ch.t - ch.col('t_recv'))))
    return tuple(vals)


def _explain(a, b, backends, verdict, attribution, first_s, first_sig, by_group, departs, quant, clock, secondary,
             resolve, g, align, evc, out, th):
    A, B = backends
    L = []
    if verdict == 'agree':
        L.append(f'{A} and {B} agree: no signal exceeds its threshold for {th.hold_s:g} s over '
                 f'{align.get("time_base")} alignment')
    elif attribution == 'clock_or_timestamp':
        L.append(f'{B} differs from {A} in its time base by {clock["offset_s"]:+.3f} s '
                 f'({clock["offset_ticks"]:+.1f} ticks) from t={first_s:.2f} s [{clock["how"]}] -> clock_or_timestamp')
        if secondary and secondary.get('attribution'):
            L.append(f'after the correction: first disagreement {secondary["first_signal"]} at '
                     f't={secondary["first_s"]:.2f} s, {secondary["attribution"]}')
        else:
            L.append('after the correction the runs agree')
    else:
        L.append(f'{B} departs from {A} at t={first_s:.2f} s on {first_sig} -> {attribution}')
    if by_group and verdict == 'diverged':
        order = sorted(by_group.values(), key=lambda d: d['depart_s'])
        L.append('departure order: ' + ', '.join(f"{d['signal']} {d['depart_s']:.2f} s" for d in order))
    if attribution == 'command_delivery':
        L.append(f"cmd.requested agree; applied commands differ first (B applies {quant.get('delivery_lag_s', 0):+.2f} s "
                 'relative to A)')
    elif attribution == 'plant':
        L.append(f"applied commands agree (max diff {quant.get('applied_max_diff_in_window', float('nan')):.3f}) but the "
                 f"response differs: {quant.get('response')}; surge accel ratio B/A "
                 f"{quant.get('surge_accel_ratio_b_over_a', float('nan')):.2f}")
    elif attribution == 'sensor':
        L.append('truth agrees while obs differ; obs latency A {:.2f} s, B {:.2f} s'.format(
            quant.get('a.obs_latency_s', float('nan')), quant.get('b.obs_latency_s', float('nan'))))
    elif attribution == 'controller':
        L.append('obs agree but cmd.requested differ: same controller and seed should not do this '
                 '(nondeterminism or a controller version/parameter mismatch)')
    elif attribution == 'unknown':
        L.append('cannot separate the candidates' + (f'; would resolve it: {", ".join(resolve)}' if resolve else ''))
    warn = [i for i in g['issues'] if i['severity'] in ('warn', 'refuse')]
    if warn:
        L.append('guard: ' + '; '.join(f"{i['code']} differs" for i in warn))
    if align.get('note'):
        L.append(align['note'])
    if evc.get('first_difference'):
        fd = evc['first_difference']
        L.append(f"events: first difference {fd['kind']} A {fd['a']} vs B {fd['b']}")
    if out.get('differ'):
        L.append(f"outcome {out['a']} vs {out['b']}; divergence precedes it: {out.get('divergence_precedes_outcome')}")
    return '\n'.join(L)


# ------------------------------------------------------------------------------------ batch

def summarize_pairs(results: Sequence[LayerDivergence]):
    """Counts by attribution, first-divergence distribution, and whether divergence precedes the
    outcome difference."""
    counts = {}
    for r in results:
        k = r.attribution if r.verdict == 'diverged' else r.verdict
        counts[k] = counts.get(k, 0) + 1
    firsts = np.array([r.first_s for r in results if r.first_s is not None], float)
    dist = ({'n': int(len(firsts)), 'min': float(firsts.min()), 'median': float(np.median(firsts)),
             'p90': float(np.quantile(firsts, 0.9)), 'max': float(firsts.max())} if len(firsts) else {'n': 0})
    by_attr = {}
    for r in results:
        if r.first_s is not None:
            by_attr.setdefault(r.attribution, []).append(r.first_s)
    differ = [r for r in results if r.outcome.get('differ')]
    precedes = sum(1 for r in differ if r.outcome.get('divergence_precedes_outcome'))
    silent = [r.pair for r in differ if r.verdict != 'diverged']
    return {'n_pairs': len(results), 'by_attribution': counts, 'first_divergence_s': dist,
            'first_divergence_by_attribution': {k: float(np.median(v)) for k, v in by_attr.items()},
            'outcome_differs': len(differ), 'divergence_precedes_outcome': precedes,
            'outcome_differs_without_divergence': [list(p) for p in silent]}
