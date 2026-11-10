"""Validity: is this record a fair measurement of the run it claims to be? (judge/SCHEMA_V2.md, question 1)

``check(record, expect=None) -> ValidityReport`` looks only at the measurement apparatus -- clocks,
frames, telemetry coverage, reset, command delivery, infrastructure -- and never at the mission: a
run that times out against the dock with clean telemetry is VALID. Each reason has a code from
``CODES``, a severity (``invalid`` makes the run invalid, ``warning`` does not) and machine-readable
evidence that always says what was measured (``measured``) and where (channel, indices, times).
``apply(record, report)`` writes the verdict into the record's ``Validity``; sources never do.

Checks that cannot run on a record (a channel missing, nothing to correlate, too little motion to
identify a frame) are listed in ``report.skipped`` with the reason, so "no finding" and "not
checked" stay apart. Per-task requirements live in ``Expect``; ``DEFAULT_EXPECT`` is the docking
study's pre-registered timing (control 25 Hz, plant tick 0.02 s) and nothing else task-specific.
``judge.insight.integrity`` narrates validity in reports; its gap and clock-residual constants are
reused here.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math
from typing import Any, Mapping, Optional

import numpy as np

from judge.insight.integrity import CLOCK_RESIDUAL_WARN, GAP_FACTOR
from judge.record import (EVENTS, STANDARD, STANDARD_NAMES, TERMINAL, V2_BLOCKS, Channel, END, EV,
                          RunRecord, _clean, obs_truth_aliases, wrap)

INVALID, WARNING = 'invalid', 'warning'
CODES = ('clock.nonmonotonic', 'clock.dt_mismatch', 'clock.offset', 'clock.rtf_starved', 'frame.mismatch',
         'telemetry.missing_channel', 'telemetry.gap', 'telemetry.rate', 'reset.failed', 'reset.unverified',
         'cmd.delivery_gap', 'cmd.rejected', 'infra.crash', 'infra.link_lost', 'infra.backend_flag',
         'record.schema')

# ----------------------------------------------------------------------------- thresholds
# clock
DT_REL_TOL = 0.02          # tick->time slope within 2% of dt_s: float32 engine time drifts ~1e-4, a wrong dt is >=10%
DT_MULT_TOL = 0.1          # median spacing within 0.1 tick of an integer number of ticks (fixed-step logs sit on the grid)
CAUSAL_TOL_S = 0.005       # 5 ms: >> 1e-9 s rounding on disk and float32 time drift, << one 20 ms plant tick
OFFSET_TOL_S = 0.05        # obs/truth time bases may disagree by < 50 ms (just over one 40 ms control period)
OFFSET_SEARCH_S = 1.0      # search obs/truth shifts within +-1 s (offsets beyond that show up as frame or gap faults)
OFFSET_STEP_S = 0.005      # shift grid step; refined by a parabola through the minimum
RTF_STARVED_FRAC = 0.8     # realtime runs below 80% of the requested time scale skip or stretch control periods
# frames (model selection between identity, time shift, rigid/reflected transform)
MODEL_IMPROVE_RATIO = 2.0  # an alternative model must halve the identity residual to be believed
POS_ABS_MIN_M = 0.1        # identity residuals under 10 cm are sensor noise at docking scale, never a frame fault
YAW_ABS_MIN_RAD = 0.1      # 5.7 deg: below this a heading residual is estimator noise
FRAME_MIN_EXTENT_M = 2.0   # a frame is identifiable only if the track spans >= 2 m
FRAME_MIN_SAMPLES = 20     # fewer matched samples cannot separate noise from a transform
TWIST_ABS_MIN = 0.1        # m/s: body/world velocity differences below this are not identifiable
RATE_ABS_MIN = 0.02        # rad/s: yaw-rate magnitude needed to test the yaw-rate sign
COURSE_MIN_SPEED = 0.3     # m/s: course over ground is only meaningful when moving
COURSE_SPREAD_MAX = 0.5    # rad circular std of heading-minus-course; sideslip in a docking manoeuvre stays well below
# telemetry
GAP_INVALID_S = 0.5        # an undeclared hole >= 0.5 s (>= 12 control periods, ~0.5 m at 1 m/s) invalidates the run
EDGE_GRACE_S = 1.0         # channels may start/stop within 1 s of the window edges (sensor bring-up, last flush)
# reset
RESET_POS_TOL_M = 0.5      # start within 0.5 m of the declared start (a quarter hull length, << the 3 m faults we must catch)
RESET_YAW_TOL_RAD = math.radians(10)   # start heading within 10 deg
RESET_SPEED_TOL = 0.1      # m/s: "at rest" means slower than 0.1 m/s on the first sample
RESET_DERIV_S = 0.2        # without truth.twist, initial speed from the first 0.2 s of truth.pose
# commands
Q_MATCH_TOL = 0.02         # requested vs applied normalised command agree within 2% of full scale
Q_RANGE_EPS = 1e-6         # normalised commands outside [-1, 1] by more than float noise are out of range
DELIVERY_TOL_S = 0.5       # applied may lag requested by <= 0.5 s (above the 0.25-0.35 s physical lumped delay)
DELIVERY_GAP_S = 0.2       # a run of unmatched requests >= 0.2 s (5 control periods) is a delivery gap
DELIVERY_MISS_FRAC = 0.05  # scattered unmatched requests above 5% are reported as a warning
PULSE_MIN_US, PULSE_MAX_US = 1000.0, 2000.0   # standard RC ESC pulse range
FLAG_CODES = {'cmd_rejected': 'cmd.rejected', 'crash': 'infra.crash', 'link_lost': 'infra.link_lost'}
REASON_SEVERITY = {c: INVALID for c in CODES} | {'reset.unverified': WARNING}


@dataclass(frozen=True)
class Expect:
    """Per-task requirements. Defaults = the docking study; override per task."""
    required: tuple = ('truth.pose', 'truth.twist', 'obs.pose', 'cmd.requested')
    accounted: tuple = STANDARD_NAMES        # each must be logged OR declared absent (else a warning)
    rates_hz: Mapping[str, tuple] = field(default_factory=lambda: {
        'cmd.requested': (22.5, 27.5),       # 25 Hz control +-10%
        'truth.pose': (20.0, math.inf),      # >= 20 Hz: <= 5 cm between samples at 1 m/s
    })
    control_period_s: float = 0.04
    plant_dt_s: Optional[float] = 0.02
    start_pose: Optional[tuple] = None       # (x, y, yaw); else scenario.condition['start_pose']
    start_twist: Optional[tuple] = None      # (u, v, r);   else scenario.condition['start_twist']
    reset_pos_tol_m: float = RESET_POS_TOL_M
    reset_yaw_tol_rad: float = RESET_YAW_TOL_RAD
    reset_speed_tol: float = RESET_SPEED_TOL
    delivery_tol_s: float = DELIVERY_TOL_S
    max_obs_latency_s: float = 0.5           # without obs.stamp, obs may lag truth by up to this much
    benign_flags: tuple = ()                 # backend flags downgraded to warnings for this task
    events: tuple = EVENTS
    terminal: tuple = TERMINAL
    excluded_from: tuple = ('G3', 'G4')


DEFAULT_EXPECT = Expect()


@dataclass(frozen=True)
class Reason:
    code: str
    severity: str
    message: str
    evidence: Mapping[str, Any] = field(default_factory=dict)

    def as_dict(self):
        return _clean({'code': self.code, 'severity': self.severity, 'message': self.message,
                       'evidence': dict(self.evidence)}, 6)


@dataclass(frozen=True)
class ValidityReport:
    run_id: str
    valid: bool
    reasons: tuple = ()
    checked: tuple = ()
    skipped: Mapping[str, str] = field(default_factory=dict)

    @property
    def codes(self):
        return sorted({r.code for r in self.reasons})

    def of(self, code):
        return [r for r in self.reasons if r.code == code]

    def as_dict(self):
        return {'run_id': self.run_id, 'valid': self.valid, 'reasons': [r.as_dict() for r in self.reasons],
                'checked': list(self.checked), 'skipped': dict(self.skipped)}


# -------------------------------------------------------------------------------- helpers

def _r(x, nd=6):
    return None if x is None or not math.isfinite(float(x)) else round(float(x), nd)


def _idx(a, k=10):
    return [int(i) for i in np.asarray(a).reshape(-1)[:k]]


def _rms(a):
    a = np.asarray(a, float)
    return float(np.sqrt(np.mean(a ** 2))) if a.size else 0.0


def _ordered(ch: Channel):
    """t, v, tick sorted by time (stable): later checks see a usable channel even after a reversal."""
    if len(ch) < 2 or np.all(np.diff(ch.t) >= 0):
        return ch.t, ch.v, ch.tick
    o = np.argsort(ch.t, kind='stable')
    return ch.t[o], ch.v[o], None if ch.tick is None else ch.tick[o]


def _interp_pose(t, v, times):
    """truth pose (x, y, yaw) at ``times``; yaw unwrapped for interpolation, rewrapped."""
    return np.column_stack([np.interp(times, t, v[:, 0]), np.interp(times, t, v[:, 1]),
                            wrap(np.interp(times, t, np.unwrap(v[:, 2])))])


def _circ(d):
    """(circular mean, circular std) of angles."""
    z = np.mean(np.exp(1j * np.asarray(d)))
    R = min(max(abs(z), 1e-12), 1.0)
    return float(np.angle(z)), float(math.sqrt(-2 * math.log(R)))


def _median_dt(t):
    dt = np.diff(t)
    dt = dt[dt > 0]
    return float(np.median(dt)) if dt.size else None


def _start_spec(rec, ex, key, attr):
    v = getattr(ex, attr)
    if v is None:
        v = (rec.header.scenario.condition or {}).get(key)
    return None if v is None else tuple(float(x) for x in v)


class _Run:
    def __init__(self, rec: RunRecord, ex: Expect):
        self.rec, self.ex = rec, ex
        self.reasons, self.checked, self.skipped = [], [], {}
        self.dt_flagged = set()
        self.ch = {n: c for n, c in rec.channels.items() if c.spec.observability != 'absent' and len(c)}
        self.v2 = rec.header.schema is not None and not rec.header.missing_blocks

    def add(self, code, message, severity=None, **evidence):
        assert code in CODES, code
        self.reasons.append(Reason(code, severity or REASON_SEVERITY[code], message, evidence))

    def skip(self, check, why):
        self.skipped[check] = why

    def ok(self, check):
        self.checked.append(check)

    def acq_times(self, name):
        """Acquisition time per row of obs channel ``name`` from obs.stamp (by tick, then by time,
        then by index); None when there is no usable stamp."""
        st, ob = self.ch.get('obs.stamp'), self.ch.get(name)
        if st is None or ob is None:
            return None, 'no obs.stamp'
        t_acq = st.col('t_acq')
        if st.tick is not None and ob.tick is not None:
            m = {int(k): i for i, k in enumerate(st.tick)}
            idx = np.array([m.get(int(k), -1) for k in ob.tick])
            how = 'tick'
        else:
            j = np.searchsorted(st.t, ob.t)
            j = np.clip(j, 0, len(st.t) - 1)
            idx = np.where(np.abs(st.t[j] - ob.t) <= 1e-6, j, -1)
            how = 'time'
            if (idx < 0).mean() > 0.5 and len(st) == len(ob):
                idx, how = np.arange(len(ob)), 'index'
        out = np.where(idx >= 0, t_acq[np.maximum(idx, 0)], np.nan)
        return out, how


# ------------------------------------------------------------------------------- checks

def _schema(R: _Run):
    rec, h = R.rec, R.rec.header
    missing = h.missing_blocks
    if len(missing) == len(V2_BLOCKS):
        R.add('record.schema', f'{rec.run_id} is a /1 record: no /2 header blocks', WARNING,
              measured='header blocks present', missing_blocks=list(missing), schema='crane.judge.run/1')
    elif missing:
        R.add('record.schema', f'{rec.run_id}: /2 header incomplete ({", ".join(missing)})',
              measured='header blocks present', missing_blocks=list(missing))
    end = h.window[2] if len(h.window) > 2 else ''
    if end not in R.ex.terminal:
        R.add('record.schema', f'{rec.run_id}: terminal reason {end!r} not in {list(R.ex.terminal)}',
              None if R.v2 else WARNING, measured='header.window[2]', terminal=end)
    for name, ch in rec.channels.items():
        if name in STANDARD and ch.spec.fields != STANDARD[name].fields:
            R.add('record.schema', f'{name} has fields {ch.spec.fields}, standard {STANDARD[name].fields}',
                  measured='channel fields', channel=name, fields=list(ch.spec.fields),
                  expected=list(STANDARD[name].fields))
        if ch.v.shape != (len(ch.t), len(ch.spec.fields)) or (ch.tick is not None and len(ch.tick) != len(ch.t)):
            R.add('record.schema', f'{name}: array shapes disagree', measured='array shapes', channel=name,
                  t=len(ch.t), v=list(ch.v.shape), tick=None if ch.tick is None else len(ch.tick))
        if ch.spec.observability == 'absent' and not ch.spec.note:
            R.add('record.schema', f'{name} is absent without a reason', WARNING,
                  measured='declared-absent note', channel=name)
    for o, t, how in obs_truth_aliases(rec):
        R.add('record.schema', f'{o} is {t} ({how}) without an obs_is_truth declaration',
              measured='obs vs truth arrays', channel=o, truth=t, how=how)
    if R.v2:
        unknown = sorted({e.kind for e in rec.events} - set(R.ex.events))
        if unknown:
            R.add('record.schema', f'event kinds outside the task vocabulary: {", ".join(unknown)}', WARNING,
                  measured='event kinds', unknown=unknown, vocabulary=list(R.ex.events))
    R.ok('record.schema')


def _monotonic(R: _Run):
    for name, ch in sorted(R.ch.items()):
        if len(ch) < 2:
            continue
        dt = np.diff(ch.t)
        back = np.nonzero(dt < 0)[0]
        if back.size:
            R.add('clock.nonmonotonic', f'{name} time runs backwards {back.size} times (worst {-dt.min():.3f} s)',
                  measured='diff(t) < 0', channel=name, field='t', n=int(back.size), indices=_idx(back + 1),
                  times=[_r(x) for x in ch.t[back[:10] + 1]], max_step_back_s=_r(-dt.min()))
        dup = np.nonzero(dt == 0)[0]
        if dup.size:
            R.add('clock.nonmonotonic', f'{name} repeats {dup.size} timestamps', WARNING,
                  measured='diff(t) == 0', channel=name, field='t', n=int(dup.size), indices=_idx(dup + 1),
                  times=[_r(x) for x in ch.t[dup[:10] + 1]])
        if ch.tick is not None:
            dk = np.diff(ch.tick)
            for mask, what in ((dk < 0, 'runs backwards'), (dk == 0, 'repeats')):
                bad = np.nonzero(mask)[0]
                if bad.size:
                    R.add('clock.nonmonotonic', f'{name} tick {what} {bad.size} times',
                          measured='diff(tick) < 0' if what == 'runs backwards' else 'diff(tick) == 0',
                          channel=name, field='tick', n=int(bad.size), indices=_idx(bad + 1),
                          ticks=[int(x) for x in ch.tick[bad[:10] + 1]],
                          times=[_r(x) for x in ch.t[bad[:10] + 1]])
    R.ok('clock.nonmonotonic')


def _dt(R: _Run):
    cm = R.rec.header.clock_mode
    if cm is None or cm.dt_s is None:
        R.skip('clock.dt_mismatch', 'no clock_mode.dt_s')
        return
    dt = cm.dt_s
    if R.ex.plant_dt_s and abs(dt / R.ex.plant_dt_s - 1) > DT_REL_TOL:
        R.add('clock.dt_mismatch', f'header dt_s {dt:g} s, task expects {R.ex.plant_dt_s:g} s', WARNING,
              measured='clock_mode.dt_s vs Expect.plant_dt_s', declared_dt_s=dt, expected_dt_s=R.ex.plant_dt_s)
    for name, ch in sorted(R.ch.items()):
        t, _, tick = _ordered(ch)
        if tick is not None:
            k, i = np.unique(tick, return_index=True)
            if len(k) < 3 or k[-1] == k[0]:
                continue
            slope, icpt = np.polyfit(k.astype(float), t[i], 1)
            rel = slope / dt - 1
            if abs(rel) > DT_REL_TOL:
                R.dt_flagged.add(name)
                R.add('clock.dt_mismatch', f'{name}: {slope * 1000:.2f} ms per tick, header dt_s {dt * 1000:.2f} ms',
                      measured='least-squares slope of t on tick', channel=name, slope_s_per_tick=_r(slope),
                      intercept_s=_r(icpt), declared_dt_s=dt, rel_error=_r(rel), n=int(len(k)))
        elif cm.mode in ('fixed_step', 'lockstep'):
            med = _median_dt(t)
            if med is None:
                continue
            m = med / dt
            if round(m) < 1 or abs(m - round(m)) > DT_MULT_TOL:
                R.dt_flagged.add(name)
                R.add('clock.dt_mismatch', f'{name}: median spacing {med * 1000:.2f} ms is {m:.2f} plant ticks',
                      measured='median sample spacing / dt_s (must be an integer in fixed-step modes)',
                      channel=name, median_spacing_s=_r(med), declared_dt_s=dt, ticks_per_sample=_r(m))
    R.ok('clock.dt_mismatch')


def _rtf(R: _Run):
    cm = R.rec.header.clock_mode
    if cm is None or cm.rtf is None or cm.mode in ('fixed_step', 'recorded'):
        R.skip('clock.rtf_starved', 'no rtf, or a mode whose time does not depend on wall speed')
        return
    ts = cm.time_scale or 1.0
    if cm.rtf < RTF_STARVED_FRAC * ts:
        R.add('clock.rtf_starved', f'rtf {cm.rtf:.2f} vs time_scale {ts:g} in {cm.mode} mode',
              INVALID if cm.mode == 'realtime' else WARNING, measured='clock_mode.rtf / time_scale',
              rtf=cm.rtf, time_scale=ts, ratio=_r(cm.rtf / ts), mode=cm.mode)
    R.ok('clock.rtf_starved')


def _tick_time(R: _Run):
    """One backend tick is one instant: every ticked channel must put a tick at the same time."""
    cm = R.rec.header.clock_mode
    if cm is not None and cm.mode == 'recorded':
        R.skip('clock.offset.ticks', 'recorded logs: ticks are per-source sequence numbers, not one clock')
        return
    ticked = {n: c for n, c in R.ch.items() if c.tick is not None and len(c) >= FRAME_MIN_SAMPLES}
    if len(ticked) < 2:
        R.skip('clock.offset.ticks', 'fewer than two ticked channels')
        return
    ref = 'truth.pose' if 'truth.pose' in ticked else max(sorted(ticked), key=lambda n: len(ticked[n]))
    rk, ri = np.unique(ticked[ref].tick, return_index=True)
    rt = ticked[ref].t[ri]
    for name, ch in sorted(ticked.items()):
        if name == ref:
            continue
        k, i = np.unique(ch.tick, return_index=True)
        common, a, b = np.intersect1d(k, rk, return_indices=True)
        if len(common) < FRAME_MIN_SAMPLES:
            continue
        d = ch.t[i][a] - rt[b]
        med = float(np.median(d))
        if abs(med) > CAUSAL_TOL_S:
            R.add('clock.offset', f'{name} puts the same ticks {med * 1000:+.0f} ms from {ref}',
                  measured='median of t(channel) - t(reference) over common ticks', channel=name, against=ref,
                  pairing='tick', n=int(len(common)), median_s=_r(med), max_abs_s=_r(float(np.abs(d).max())))
    R.ok('clock.offset.ticks')


def _stamps(R: _Run):
    """obs.stamp ordering (acquired <= received <= used) and command-after-observation causality."""
    c = R.rec.header.clock
    if c.residual_s is not None and c.residual_s > CLOCK_RESIDUAL_WARN:
        R.add('clock.offset', f'source clock alignment residual {c.residual_s:.2f} s', WARNING,
              measured='header.clock.residual_s', residual_s=c.residual_s, limit_s=CLOCK_RESIDUAL_WARN)
    _tick_time(R)
    st = R.ch.get('obs.stamp')
    if st is None:
        R.skip('clock.offset.stamps', 'no obs.stamp')
        return
    t, v, tick = _ordered(st)
    ta, tr = v[:, 0], v[:, 1]
    for lag, what in ((tr - ta, 't_recv < t_acq'), (t - tr, 'used before received (t < t_recv)')):
        bad = np.nonzero(lag < -CAUSAL_TOL_S)[0]
        if bad.size:
            R.add('clock.offset', f'obs.stamp: {what} in {bad.size} rows (worst {-lag.min() * 1000:.0f} ms)',
                  measured=what, channel='obs.stamp', n=int(bad.size), indices=_idx(bad),
                  times=[_r(x) for x in t[bad[:10]]], worst_s=_r(-lag.min()))
    cmd = R.ch.get('cmd.requested')
    if cmd is None:
        R.skip('clock.offset.cmd_vs_stamp', 'no cmd.requested')
    else:
        ct, _, ctick = _ordered(cmd)
        if ctick is not None and tick is not None:
            common, ic, is_ = np.intersect1d(ctick, tick, return_indices=True)
            how = 'tick'
        elif len(ct) == len(t):
            ic = is_ = np.arange(len(ct))
            how = 'index'
        else:
            ic = is_ = np.zeros(0, int)
            how = None
        if len(ic) < FRAME_MIN_SAMPLES:
            R.skip('clock.offset.cmd_vs_stamp', 'cannot pair commands with observations (no common ticks)')
        else:
            lag = ct[ic] - tr[is_]
            med = float(np.median(lag))
            if med < -CAUSAL_TOL_S:
                R.add('clock.offset', f'commands precede the observations they used by {-med * 1000:.0f} ms (median)',
                      measured='cmd.requested.t - obs.stamp.t_recv, paired by ' + how, channel='cmd.requested',
                      against='obs.stamp', pairing=how, n=int(len(lag)), median_s=_r(med), min_s=_r(lag.min()),
                      frac_negative=_r((lag < -CAUSAL_TOL_S).mean()))
            elif med > R.ex.max_obs_latency_s:
                R.add('clock.offset', f'commands follow their observations by {med * 1000:.0f} ms (median)', WARNING,
                      measured='cmd.requested.t - obs.stamp.t_recv, paired by ' + how, channel='cmd.requested',
                      against='obs.stamp', pairing=how, n=int(len(lag)), median_s=_r(med),
                      limit_s=R.ex.max_obs_latency_s)
            R.ok('clock.offset.cmd_vs_stamp')
    R.ok('clock.offset.stamps')


def _obs_vs_truth(R: _Run):
    """obs.pose against truth.pose: identity, a pure time shift, or a rigid/reflected transform?"""
    tr, ob = R.ch.get('truth.pose'), R.ch.get('obs.pose')
    if tr is None or ob is None:
        R.skip('frame.obs_pose', 'needs truth.pose and obs.pose')
        return
    tt, tv, _ = _ordered(tr)
    ot, ov, _ = _ordered(ob)
    acq, how = R.acq_times('obs.pose')
    if acq is not None:
        acq = acq[np.argsort(ob.t, kind='stable')] if not np.all(np.diff(ob.t) >= 0) else acq
    stamped = acq is not None and np.isfinite(acq).mean() > 0.5
    ta = np.where(np.isfinite(acq), acq, ot) if stamped else ot.copy()
    keep = (ta >= tt[0]) & (ta <= tt[-1]) & np.all(np.isfinite(ov), axis=1)
    if keep.sum() < FRAME_MIN_SAMPLES:
        R.skip('frame.obs_pose', f'{int(keep.sum())} obs samples inside the truth span')
        return
    ta, ov = ta[keep], ov[keep]
    T = _interp_pose(tt, tv, ta)
    extent = float(np.hypot(*(T[:, :2].max(0) - T[:, :2].min(0))))
    if extent < FRAME_MIN_EXTENT_M:
        R.skip('frame.obs_pose', f'track spans {extent:.2f} m < {FRAME_MIN_EXTENT_M} m: frame not identifiable')
        return
    B = ov[:, :2]
    r_id = _rms(np.linalg.norm(B - T[:, :2], axis=1))
    # time-shift model
    grid = np.arange(-OFFSET_SEARCH_S, OFFSET_SEARCH_S + OFFSET_STEP_S / 2, OFFSET_STEP_S)
    rs = np.array([_rms(np.linalg.norm(B - np.column_stack([np.interp(ta + s, tt, tv[:, 0]),
                                                             np.interp(ta + s, tt, tv[:, 1])]), axis=1))
                   for s in grid])
    k = int(np.argmin(rs))
    s_hat = float(grid[k])
    if 0 < k < len(grid) - 1:
        den = rs[k - 1] - 2 * rs[k] + rs[k + 1]
        if den > 0:
            s_hat += 0.5 * OFFSET_STEP_S * (rs[k - 1] - rs[k + 1]) / den
    r_shift = float(rs[k])
    # rigid / reflected transform model: obs ~ M truth + c (orthogonal Procrustes, reflection allowed)
    A = T[:, :2]
    am, bm = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - am).T @ (B - bm))
    M = (U @ Vt).T
    c = bm - am @ M.T
    r_frame = _rms(np.linalg.norm(B - (A @ M.T + c), axis=1))
    det = float(np.linalg.det(M))
    rot = float(math.degrees(math.atan2(M[1, 0], M[0, 0])))
    ev = dict(channel='obs.pose', against='truth.pose', acq_times='obs.stamp by ' + how if stamped else 'obs.pose.t',
              n=int(len(ta)), residual_identity_m=_r(r_id), residual_time_shift_m=_r(r_shift), shift_s=_r(s_hat),
              residual_rigid_m=_r(r_frame), rigid_det=_r(det), rigid_rotation_deg=_r(rot),
              rigid_translation_m=[_r(x) for x in c])
    best = min(r_shift, r_frame)
    shift_used = 0.0
    pos_flagged = False
    if r_id > POS_ABS_MIN_M and r_id >= MODEL_IMPROVE_RATIO * best:
        if r_shift <= r_frame:
            lo, hi = (-OFFSET_TOL_S, OFFSET_TOL_S) if stamped else (-R.ex.max_obs_latency_s - OFFSET_TOL_S, CAUSAL_TOL_S)
            if not lo <= s_hat <= hi:
                R.add('clock.offset', f'obs.pose matches truth.pose {s_hat:+.3f} s away from its stamped time '
                      f'(residual {r_id:.2f} m -> {r_shift:.2f} m)',
                      measured='shift s minimising |obs(t) - truth(t_acq + s)|', allowed_shift_s=[lo, hi], **ev)
                shift_used = s_hat
        else:
            kind = 'reflection' if det < 0 else 'rotation' if abs(rot) > 2.0 else 'translation'
            R.add('frame.mismatch', f'obs.pose is truth.pose under a {kind} (det {det:+.0f}, {rot:+.1f} deg, '
                  f'offset {np.hypot(*c):.2f} m; residual {r_id:.2f} m -> {r_frame:.2f} m)',
                  measured='orthogonal Procrustes fit obs ~ M truth + c', kind=kind, **ev)
            pos_flagged = True
    # heading
    ty = wrap(np.interp(ta + shift_used, tt, np.unwrap(tv[:, 2])))
    oy = ov[:, 2]
    dp, dm = wrap(oy - ty), wrap(oy + ty)
    cp, sp = _circ(dp)
    cmn, sm = _circ(dm)
    res_id, res_p, res_m = _rms(dp), _rms(wrap(dp - cp)), _rms(wrap(dm - cmn))
    yev = dict(channel='obs.pose', field='yaw', against='truth.pose', residual_identity_rad=_r(res_id),
               residual_offset_rad=_r(res_p), offset_rad=_r(cp), residual_sign_flip_rad=_r(res_m),
               sign_flip_offset_rad=_r(cmn), yaw_spread_rad=_r(float(np.std(np.unwrap(ty)))))
    if not pos_flagged and res_id > YAW_ABS_MIN_RAD and res_id >= MODEL_IMPROVE_RATIO * min(res_p, res_m):
        kind = 'yaw_sign' if res_m < res_p else 'yaw_offset'
        R.add('frame.mismatch', f'obs.pose yaw is {"-yaw" if kind == "yaw_sign" else "yaw"} + '
              f'{cmn if kind == "yaw_sign" else cp:+.2f} rad of truth (residual {res_id:.2f} -> '
              f'{min(res_p, res_m):.2f} rad)', measured='circular fit of obs yaw ~ +-truth yaw + c', kind=kind, **yev)
    R.ok('frame.obs_pose')


def _pose_derivative(tr: Channel):
    t, v, _ = _ordered(tr)
    t, i = np.unique(t, return_index=True)
    v = v[i]
    yaw = np.unwrap(v[:, 2])
    dx, dy, dyaw = (np.gradient(a, t) for a in (v[:, 0], v[:, 1], yaw))
    c, s = np.cos(yaw), np.sin(yaw)
    return t, np.column_stack([dx, dy]), np.column_stack([c * dx + s * dy, -s * dx + c * dy]), dyaw, yaw


def _twist_frames(R: _Run):
    tr = R.ch.get('truth.pose')
    if tr is None or len(tr) < FRAME_MIN_SAMPLES:
        R.skip('frame.twist', 'needs truth.pose')
        return
    t, world, body, dyaw, yaw = _pose_derivative(tr)
    for name in ('truth.twist', 'obs.twist'):
        ch = R.ch.get(name)
        if ch is None:
            R.skip(f'frame.{name}', f'no {name}')
            continue
        ct, cv, _ = _ordered(ch)
        at = ct
        if name == 'obs.twist':
            acq, _ = R.acq_times(name)
            if acq is not None and np.isfinite(acq).mean() > 0.5:
                if not np.all(np.diff(ch.t) >= 0):
                    acq = acq[np.argsort(ch.t, kind='stable')]
                at = np.where(np.isfinite(acq), acq, ct)
        keep = (at >= t[0]) & (at <= t[-1]) & np.all(np.isfinite(cv), axis=1)
        if keep.sum() < FRAME_MIN_SAMPLES:
            R.skip(f'frame.{name}', 'too few samples inside the truth.pose span')
            continue
        at, cv = at[keep], cv[keep]
        W = np.column_stack([np.interp(at, t, world[:, k]) for k in range(2)])
        Bd = np.column_stack([np.interp(at, t, body[:, k]) for k in range(2)])
        dr = np.interp(at, t, dyaw)
        sep = _rms(np.linalg.norm(W - Bd, axis=1))
        rb, rw = _rms(np.linalg.norm(cv[:, :2] - Bd, axis=1)), _rms(np.linalg.norm(cv[:, :2] - W, axis=1))
        ev = dict(channel=name, against='d/dt truth.pose', n=int(len(at)), residual_body=_r(rb),
                  residual_world=_r(rw), body_world_separation=_r(sep))
        if sep < TWIST_ABS_MIN:
            R.skip(f'frame.{name}.uv', f'body and world velocity differ by {sep:.3f} m/s: frame not identifiable')
        elif rb > TWIST_ABS_MIN and rb >= MODEL_IMPROVE_RATIO * rw:
            R.add('frame.mismatch', f'{name} (u, v) is world-frame velocity, not body (residual body {rb:.2f}, '
                  f'world {rw:.2f} m/s)', measured='rms |twist(u,v) - pose derivative| in body vs world frame',
                  kind='twist_world_not_body', **ev)
        rp, rm = _rms(cv[:, 2] - dr), _rms(cv[:, 2] + dr)
        if _rms(dr) < RATE_ABS_MIN:
            R.skip(f'frame.{name}.r', 'too little yaw rate to test its sign')
        elif rp > RATE_ABS_MIN and rp >= MODEL_IMPROVE_RATIO * rm:
            R.add('frame.mismatch', f'{name} r has the opposite sign of d(yaw)/dt (residual {rp:.3f} vs {rm:.3f} rad/s)',
                  measured='rms |r -+ d(yaw)/dt|', kind='yaw_rate_sign', channel=name, field='r',
                  residual_same_sign=_r(rp), residual_flipped=_r(rm), n=int(len(at)))
        R.ok(f'frame.{name}')
    # truth heading convention against course over ground
    sp = np.linalg.norm(world, axis=1)
    m = sp >= COURSE_MIN_SPEED
    if m.sum() < FRAME_MIN_SAMPLES:
        R.skip('frame.truth_heading', 'too little motion for course over ground')
        return
    course = np.arctan2(world[m, 1], world[m, 0])
    if _circ(course)[1] < 0.3:
        R.skip('frame.truth_heading', 'course barely changes: heading sign not identifiable')
        return
    hd = wrap(yaw[m])
    (cp, sp_), (cm_, sm) = _circ(wrap(hd - course)), _circ(wrap(hd + course))
    if sp_ > COURSE_SPREAD_MAX and sm * MODEL_IMPROVE_RATIO < sp_:
        R.add('frame.mismatch', f'truth.pose yaw runs against its course over ground (spread {sp_:.2f} rad vs '
              f'{sm:.2f} flipped)', measured='circular std of yaw -+ course where speed >= '
              f'{COURSE_MIN_SPEED} m/s', kind='truth_yaw_sign', channel='truth.pose', field='yaw',
              spread_same_sign_rad=_r(sp_), spread_flipped_rad=_r(sm), n=int(m.sum()))
    R.ok('frame.truth_heading')


def _telemetry(R: _Run):
    rec, ex = R.rec, R.ex
    for name in ex.required:
        ch = rec.channels.get(name)
        if ch is None:
            R.add('telemetry.missing_channel', f'required {name} is not in the record',
                  measured='channel present', channel=name, required=True)
        elif ch.spec.observability == 'absent':
            R.add('telemetry.missing_channel', f'required {name} declared absent: {ch.spec.note}',
                  measured='channel present', channel=name, required=True, declared_absent=ch.spec.note)
        elif len(ch) == 0:
            R.add('telemetry.missing_channel', f'required {name} has no samples', measured='sample count',
                  channel=name, required=True, n=0)
    for name in ex.accounted:
        if name not in ex.required and name not in rec.channels:
            R.add('telemetry.missing_channel', f'{name} neither logged nor declared absent', WARNING,
                  measured='channel present or declared absent', channel=name, required=False)
    t0, t1 = rec.header.window[:2]
    for name, ch in sorted(R.ch.items()):
        t, _, _ = _ordered(ch)
        lo, hi = ex.rates_hz.get(name, (None, None))
        period = 1.0 / hi if hi and math.isfinite(hi) else _median_dt(t)
        if period is None:
            continue
        period = max(period, _median_dt(t) or 0.0)
        limit = GAP_FACTOR * period
        dt = np.diff(t)
        holes = [(float(t[i]), float(t[i + 1])) for i in np.nonzero(dt > limit)[0]]
        if t1 > t0:
            if t[0] - t0 > max(limit, EDGE_GRACE_S):
                holes.insert(0, (float(t0), float(t[0])))
            if t1 - t[-1] > max(limit, EDGE_GRACE_S):
                holes.append((float(t[-1]), float(t1)))
        if not holes:
            continue
        declared = [g for g in holes if any(a <= g[0] + 1e-9 and b >= g[1] - 1e-9 for a, b in ch.gaps)]
        undeclared = [g for g in holes if g not in declared]
        longest = max(b - a for a, b in holes)
        worst_und = max((b - a for a, b in undeclared), default=0.0)
        sev = INVALID if worst_und >= GAP_INVALID_S else WARNING
        R.add('telemetry.gap', f'{name}: {len(holes)} holes > {limit:.2f} s, longest {longest:.2f} s',
              sev, measured=f'sample spacing > {GAP_FACTOR:g} x period, and window edges > {EDGE_GRACE_S} s',
              channel=name, period_s=_r(period), limit_s=_r(limit), n=len(holes), longest_s=_r(longest),
              holes=[[_r(a), _r(b)] for a, b in holes[:10]], declared=len(declared))
    for name, (lo, hi) in sorted(ex.rates_hz.items()):
        ch = R.ch.get(name)
        if ch is None:
            continue
        if name in R.dt_flagged:
            R.skip(f'telemetry.rate.{name}', 'clock.dt_mismatch on this channel explains its rate')
            continue
        med = _median_dt(_ordered(ch)[0])
        if med is None:
            continue
        rate = 1.0 / med
        if not lo <= rate <= hi:
            R.add('telemetry.rate', f'{name} at {rate:.1f} Hz, task requires [{lo:g}, {hi:g}] Hz',
                  measured='1 / median sample spacing', channel=name, rate_hz=_r(rate), band_hz=[lo, hi])
    R.ok('telemetry')


def _reset(R: _Run):
    rec, ex, rs = R.rec, R.ex, R.rec.header.reset
    start = _start_spec(rec, ex, 'start_pose', 'start_pose')
    start_tw = _start_spec(rec, ex, 'start_twist', 'start_twist')
    tr = R.ch.get('truth.pose')
    judged = False
    if start is not None and tr is not None:
        t, v, _ = _ordered(tr)
        dpos = float(math.hypot(v[0, 0] - start[0], v[0, 1] - start[1]))
        dyaw = float(abs(wrap(v[0, 2] - start[2]))) if len(start) > 2 else 0.0
        judged = True
        if dpos > ex.reset_pos_tol_m or dyaw > ex.reset_yaw_tol_rad:
            R.add('reset.failed', f'run starts {dpos:.2f} m / {math.degrees(dyaw):.1f} deg from the declared start',
                  measured='first truth.pose sample vs declared start pose', channel='truth.pose', t=_r(t[0]),
                  start=list(start), first=[_r(x) for x in v[0]], pos_error_m=_r(dpos), yaw_error_rad=_r(dyaw),
                  tol=[ex.reset_pos_tol_m, _r(ex.reset_yaw_tol_rad)])
    want = start_tw if start_tw is not None else (0.0, 0.0, 0.0) if rs is not None and rs.claims_rest else None
    if want is not None:
        tw = R.ch.get('truth.twist')
        if tw is not None:
            t, v, _ = _ordered(tw)
            first, src = v[0], 'first truth.twist sample'
            tfirst = t[0]
        elif tr is not None:
            t, v, _ = _ordered(tr)
            m = t <= t[0] + RESET_DERIV_S
            if m.sum() < 2:
                m[:2] = True
            dx, dy = (np.polyfit(t[m], v[m, k], 1)[0] for k in (0, 1))
            first, src, tfirst = np.array([math.hypot(dx, dy), 0.0, 0.0]), f'truth.pose slope over {RESET_DERIV_S} s', t[0]
        else:
            first = None
        if first is None:
            R.skip('reset.velocity', 'no truth.twist or truth.pose')
        else:
            dv = float(math.hypot(first[0] - want[0], first[1] - want[1]))
            judged = True
            if dv > ex.reset_speed_tol:
                R.add('reset.failed', f'initial speed error {dv:.2f} m/s ({"reset claims rest" if start_tw is None else "vs declared start twist"})',
                      measured=src, t=_r(tfirst), expected=list(want), first=[_r(x) for x in first],
                      speed_error=_r(dv), tol=ex.reset_speed_tol, method=rs.method if rs else None)
    if rs is not None and rs.pose_error is not None:
        pe = rs.pose_error
        if math.hypot(*pe[:2]) > ex.reset_pos_tol_m or (len(pe) > 2 and abs(pe[2]) > ex.reset_yaw_tol_rad):
            R.add('reset.failed', f'source-reported reset pose error {list(pe)}', measured='header.reset.pose_error',
                  pose_error=list(pe), tol=[ex.reset_pos_tol_m, _r(ex.reset_yaw_tol_rad)])
    if rs is not None and rs.verified is not True and not judged:
        R.add('reset.unverified', f'start state not checked by the source (verified={rs.verified}) and no declared '
              'start to check it against', measured='header.reset.verified and declared start',
              verified=rs.verified, method=rs.method)
    R.ok('reset')


def _commands(R: _Run):
    rq, ex = R.ch.get('cmd.requested'), R.ex
    flags = R.rec.header.backend.flags if R.rec.header.backend else {}
    for k, d in flags.items():
        if FLAG_CODES.get(k) == 'cmd.rejected':
            R.add('cmd.rejected', f'backend reports rejected commands: {d}', measured='header.backend.flags',
                  flag=k, detail=d)
    if rq is None:
        R.skip('cmd', 'no cmd.requested')
        return
    t, q, _ = _ordered(rq)
    nf = np.nonzero(~np.all(np.isfinite(q), axis=1))[0]
    if nf.size:
        R.add('cmd.rejected', f'{nf.size} non-finite requested commands', measured='finite cmd.requested',
              channel='cmd.requested', n=int(nf.size), indices=_idx(nf), times=[_r(x) for x in t[nf[:10]]])
    out = np.nonzero(np.nanmax(np.abs(q), axis=1) > 1 + Q_RANGE_EPS)[0] if len(q) else np.zeros(0, int)
    if out.size:
        R.add('cmd.rejected', f'{out.size} requested commands outside [-1, 1]', WARNING,
              measured='|cmd.requested| <= 1', channel='cmd.requested', n=int(out.size), indices=_idx(out),
              max_abs=_r(float(np.nanmax(np.abs(q)))))
    sent = R.ch.get('cmd.sent')
    if sent is not None:
        p = sent.v[:, :4]
        bad = np.nonzero(np.any((p < PULSE_MIN_US) | (p > PULSE_MAX_US), axis=1))[0]
        if bad.size:
            R.add('cmd.rejected', f'{bad.size} sent pulses outside [{PULSE_MIN_US:g}, {PULSE_MAX_US:g}] us', WARNING,
                  measured='cmd.sent p0..p3 range', channel='cmd.sent', n=int(bad.size), indices=_idx(bad),
                  min_us=_r(float(p.min())), max_us=_r(float(p.max())))
    ap = R.ch.get('cmd.applied')
    if ap is None:
        why = R.rec.channels.get('cmd.applied')
        R.skip('cmd.delivery_gap', 'cmd.applied declared absent: ' + why.spec.note if why is not None else 'no cmd.applied')
        R.ok('cmd.rejected')
        return
    ta = ap.col('t_apply')
    o = np.argsort(ta, kind='stable')
    ta, qa = ta[o], ap.v[o, :4]
    t1 = R.rec.header.window[1] if R.rec.header.window[1] > R.rec.header.window[0] else t[-1]
    cand = (t + ex.delivery_tol_s <= t1) & np.all(np.isfinite(q), axis=1)
    lo = np.searchsorted(ta, t - CAUSAL_TOL_S, 'left')
    hi = np.searchsorted(ta, t + ex.delivery_tol_s, 'right')
    matched = np.ones(len(t), bool)
    for i in np.nonzero(cand)[0]:
        seg = qa[lo[i]:hi[i]]
        matched[i] = bool(seg.size) and bool(np.any(np.max(np.abs(seg - q[i]), axis=1) <= Q_MATCH_TOL))
    miss = ~matched
    runs, i = [], 0
    while i < len(miss):
        if miss[i]:
            j = i
            while j + 1 < len(miss) and miss[j + 1]:
                j += 1
            runs.append((i, j))
            i = j + 1
        else:
            i += 1
    period = _median_dt(t) or ex.control_period_s
    durs = [(t[b] - t[a] + period, a, b) for a, b in runs]
    n_c = int(cand.sum())
    frac = miss.sum() / n_c if n_c else 0.0
    ev = dict(measured=f'requested commands with an applied match (|dq| <= {Q_MATCH_TOL}) within '
                       f'{ex.delivery_tol_s} s', channel='cmd.requested', against='cmd.applied', n=n_c,
              unmatched=int(miss.sum()), frac_unmatched=_r(frac))
    if durs:
        d, a, b = max(durs)
        ev |= dict(longest_s=_r(d), longest_span=[_r(t[a]), _r(t[b])], longest_indices=[int(a), int(b)],
                   spans=[[_r(t[x]), _r(t[y])] for _, x, y in sorted(durs, key=lambda r: r[1])[:10]])
        if d >= DELIVERY_GAP_S:
            R.add('cmd.delivery_gap', f'{d:.2f} s of requested commands never applied ({t[a]:.2f}-{t[b]:.2f} s)', **ev)
        elif frac >= DELIVERY_MISS_FRAC:
            R.add('cmd.delivery_gap', f'{100 * frac:.0f}% of requested commands never applied', WARNING, **ev)
    R.ok('cmd.delivery_gap')
    R.ok('cmd.rejected')


def _infra(R: _Run):
    rec, ex = R.rec, R.ex
    end = rec.header.window[2] if len(rec.header.window) > 2 else ''
    if end == END.INFRA_ERROR:
        R.add('infra.crash', 'run ended in infra_error', measured='terminal reason', terminal=end)
    if 'crash' in (rec.validity.flags or {}):
        R.add('infra.crash', f'source reported a crash: {rec.validity.flags["crash"]}',
              measured='source validity flag (/1 sources)', detail=rec.validity.flags['crash'])
    lost = rec.events_of(EV.LINK_LOST)
    if end == END.LINK_LOST or lost:
        R.add('infra.link_lost', f'link lost ({"terminal" if end == END.LINK_LOST else "event"}'
              f'{f" at t={lost[0].t:.2f} s" if lost else ""})', measured='terminal reason and link_lost events',
              terminal=end, times=[_r(e.t) for e in lost[:10]])
    for name in ('truth.pose', 'truth.twist'):
        ch = R.ch.get(name)
        if ch is not None:
            bad = np.nonzero(~np.all(np.isfinite(ch.v), axis=1))[0]
            if bad.size:
                R.add('infra.crash', f'{name} has {bad.size} non-finite samples (plant blew up)',
                      measured='finite truth values', channel=name, n=int(bad.size), indices=_idx(bad),
                      times=[_r(x) for x in ch.t[bad[:10]]])
    flags = rec.header.backend.flags if rec.header.backend else {}
    for k, d in flags.items():
        code = FLAG_CODES.get(k, 'infra.backend_flag')
        if code == 'cmd.rejected':
            continue
        R.add(code, f'backend flag {k}: {d}', WARNING if k in ex.benign_flags else None,
              measured='header.backend.flags', flag=k, detail=d)
    R.ok('infra')


CHECKS = (_schema, _monotonic, _dt, _rtf, _stamps, _obs_vs_truth, _twist_frames, _telemetry, _reset,
          _commands, _infra)


def check(record: RunRecord, expect: Optional[Expect] = None) -> ValidityReport:
    """Every validity reason of ``record`` (mission outcome is never one)."""
    R = _Run(record, expect or DEFAULT_EXPECT)
    for fn in CHECKS:
        fn(R)
    reasons = tuple(R.reasons)
    return ValidityReport(record.run_id, not any(r.severity == INVALID for r in reasons), reasons,
                          tuple(R.checked), dict(sorted(R.skipped.items())))


def apply(record: RunRecord, report: ValidityReport, expect: Optional[Expect] = None) -> RunRecord:
    """The record with ``Validity`` set from the report: flags = {code: worst severity}; an invalid
    run is excluded from the expectation's gates. The full report rides in ``attachments``."""
    if report.run_id != record.run_id:
        raise ValueError(f'report for {report.run_id} applied to {record.run_id}')
    flags = {}
    for r in report.reasons:
        if flags.get(r.code) != INVALID:
            flags[r.code] = r.severity
    ex = expect or DEFAULT_EXPECT
    out = record.with_validity(valid=report.valid, flags=dict(sorted(flags.items())),
                               excluded_from=frozenset() if report.valid else frozenset(ex.excluded_from))
    return replace(out, attachments={**dict(record.attachments), 'validity_report': report})
