"""Metrics return a value with a status and evidence, never a bare float.

Status merges the referee's vocabulary (earned/provisional/unscorable) with the NLC evaluator's
three-valued verdicts:

  ok             the value stands
  provisional    computed, but an input is surveyed/estimated and the decision threshold lies
                 within 2 sigma, or a rule input is planned rather than logged
  unscorable     the record cannot answer (a channel or capability is missing); never a zero
  indeterminate  a Property whose margin straddles zero once bounds e (localisation) and L
                 (inter-sample motion) are applied

``Property`` carries the NLC monitor semantics (nl-constraints-bench ``nlc/monitor/evaluator.py``,
MATH.md 1.4), so offline violations, the NLC study and a runtime shield use one predicate:

  VIOLATED       g + e < -eps          (some sample)
  SATISFIED      g - e - L > eps       (every sample)
  INDETERMINATE  otherwise
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Mapping, Optional

import numpy as np

from judge.record import RunRecord, Unobservable

VIOLATED, SATISFIED, INDETERMINATE = 'violated', 'satisfied', 'indeterminate'


class Status(str, Enum):
    OK = 'ok'
    PROVISIONAL = 'provisional'
    UNSCORABLE = 'unscorable'
    INDETERMINATE = 'indeterminate'


@dataclass(frozen=True)
class Span:
    run_id: str
    t0: float
    t1: float
    channels: tuple = ()
    label: str = ''


@dataclass(frozen=True)
class MetricValue:
    value: Optional[float]
    status: Status = Status.OK
    sigma: Optional[float] = None
    spans: tuple = ()
    why: str = ''
    extra: Mapping[str, Any] = field(default_factory=dict)

    @property
    def usable(self):
        return self.value is not None and self.status in (Status.OK, Status.PROVISIONAL)

    def as_dict(self):
        return {'value': self.value, 'status': self.status.value, 'sigma': self.sigma,
                'why': self.why, 'extra': dict(self.extra),
                'spans': [[s.t0, s.t1, list(s.channels), s.label] for s in self.spans]}


def unscorable(why):
    return MetricValue(None, Status.UNSCORABLE, why=why)


class Metric:
    """Base metric. Subclasses set the class attributes and implement ``compute``.

    ``morphology_invariant`` False means the value depends on the robot's body (a modular robot's
    configuration), so the comparator refuses to compare it across different morphologies."""
    name = ''
    unit = ''
    direction = 'none'          # 'higher' | 'lower' | 'none'
    requires: frozenset = frozenset()
    morphology_invariant = True

    def compute(self, rec: RunRecord, task) -> MetricValue:
        raise NotImplementedError

    def __call__(self, rec: RunRecord, task=None) -> MetricValue:
        for name in self.requires:
            if name.startswith('cap:'):
                if not rec.caps.get(name[4:]):
                    return unscorable(f'capability {name[4:]} not provided by {rec.header.source}')
            elif not rec.has(name):
                return unscorable(f'no channel {name} in {rec.header.source}')
        try:
            out = self.compute(rec, task)
        except Unobservable as e:
            return unscorable(str(e))
        if not isinstance(out, MetricValue):
            out = MetricValue(None if out is None else float(out))
        if out.value is None and out.status in (Status.OK, Status.PROVISIONAL):
            out = MetricValue(None, Status.UNSCORABLE, out.sigma, out.spans, out.why or 'no value', out.extra)
        return out


class FnMetric(Metric):
    def __init__(self, fn, name, unit, direction, requires, morphology_invariant=True):
        self.fn, self.name, self.unit, self.direction = fn, name, unit, direction
        self.requires, self.morphology_invariant = frozenset(requires), morphology_invariant
        self.__doc__ = fn.__doc__

    def compute(self, rec, task):
        return self.fn(rec, task)


def metric(name, unit='', direction='none', requires=(), morphology_invariant=True):
    """Decorator: a function ``(rec, task) -> MetricValue | float | None`` becomes a Metric."""
    def wrap(fn):
        return FnMetric(fn, name, unit, direction, requires, morphology_invariant)
    return wrap


@dataclass(frozen=True)
class Margin:
    """A margin trace: g(t) >= 0 means the property holds at t; e(t) its uncertainty bound."""
    t: np.ndarray
    g: np.ndarray
    e: Optional[np.ndarray] = None     # localisation bound per sample (None = exact)
    L: Optional[np.ndarray] = None     # inter-sample bound per sample (None = 0)
    active: Optional[np.ndarray] = None  # bool mask; None = all samples active


def three_valued(m: Margin, eps=1e-3):
    """(verdict, per-sample labels array) under the NLC semantics."""
    e = np.zeros_like(m.g) if m.e is None else np.asarray(m.e, float)
    L = np.zeros_like(m.g) if m.L is None else np.asarray(m.L, float)
    act = np.ones(len(m.g), bool) if m.active is None else np.asarray(m.active, bool)
    lab = np.full(len(m.g), '', dtype=object)
    viol = (m.g + e < -eps) & act
    sat = (m.g - e - L > eps) & act
    lab[viol], lab[sat] = VIOLATED, SATISFIED
    lab[act & ~viol & ~sat] = INDETERMINATE
    if not act.any():
        return SATISFIED, lab
    if viol.any():
        return VIOLATED, lab
    if sat[act].all():
        return SATISFIED, lab
    return INDETERMINATE, lab


def _runs(mask):
    """[(i0, i1)] index runs where mask is True (i1 exclusive)."""
    out, i, n = [], 0, len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j < n and mask[j]:
                j += 1
            out.append((i, j))
            i = j
        else:
            i += 1
    return out


class Property(Metric):
    """A predicate over a run with a signed margin (robustness). Implement ``margin``.

    The value is the minimum margin over active samples (direction 'higher'); ``extra`` holds the
    verdict, first violation time, violation duration and integral excursion, and the spans are
    the violating (or indeterminate) intervals for the viewer."""
    direction = 'higher'
    unit = 'm'
    eps = 1e-3

    def margin(self, rec: RunRecord, task) -> Margin:
        raise NotImplementedError

    def compute(self, rec, task):
        m = self.margin(rec, task)
        if m is None or len(m.g) == 0:
            return unscorable('no active samples')
        verdict, lab = three_valued(m, self.eps)
        act = np.ones(len(m.g), bool) if m.active is None else np.asarray(m.active, bool)
        dt = np.diff(m.t, append=m.t[-1]) if len(m.t) > 1 else np.zeros(1)
        viol = lab == VIOLATED
        spans = tuple(Span(rec.run_id, float(m.t[a]), float(m.t[b - 1]), self.requires_tuple(),
                           VIOLATED) for a, b in _runs(viol))
        if verdict == INDETERMINATE:
            spans += tuple(Span(rec.run_id, float(m.t[a]), float(m.t[b - 1]), self.requires_tuple(),
                                INDETERMINATE) for a, b in _runs(lab == INDETERMINATE))
        g = m.g[act]
        first = float(m.t[np.argmax(viol)]) if viol.any() else None
        e_max = float(np.max(m.e[act])) if m.e is not None and act.any() else 0.0
        extra = {'verdict': verdict, 'first_violation_t': first,
                 'violation_s': float(np.sum(dt[viol])),
                 'integral': float(np.sum((-m.g[viol]) * dt[viol])),
                 'indeterminate_fraction': float(np.mean(lab[act] == INDETERMINATE)) if act.any() else 0.0}
        status = Status.INDETERMINATE if verdict == INDETERMINATE else Status.OK
        return MetricValue(float(np.min(g)) if len(g) else None, status, e_max or None, spans,
                           extra=extra)

    def requires_tuple(self):
        return tuple(sorted(r for r in self.requires if not r.startswith('cap:')))


def verdict_of(v: MetricValue):
    return (v.extra or {}).get('verdict')
