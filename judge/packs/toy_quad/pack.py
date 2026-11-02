"""The planar quadrotor pack: Properties, gate metrics, outcome, segmenter, dependency graph,
and a Replayer for the counterfactual decomposition."""

from __future__ import annotations

import numpy as np

from judge.insight.divergence import Replayer
from judge.metric import Margin, Metric, MetricValue, Property, Span
from judge.pack import DepGraph, HintRule, Node, Pack
from judge.packs.toy_quad import sim
from judge.task import Objective, TaskSpec

FLOOR = 0.3
TILT = 0.6


class Floor(Property):
    name = 'quad.floor'
    requires = frozenset({'pose.xz'})

    def margin(self, rec, task):
        ch = rec.channel('pose.xz')
        return Margin(ch.t, ch.col('z') - FLOOR, ch.sigma_of('z') * 2 if ch.sigma is not None else None)


class Tilt(Property):
    name, unit = 'quad.tilt', 'rad'
    requires = frozenset({'pose.xz'})

    def margin(self, rec, task):
        ch = rec.channel('pose.xz')
        return Margin(ch.t, TILT - np.abs(ch.col('pitch')))


def crossings(rec):
    """Per gate: (t, z error) at the first crossing of the gate's x, or None."""
    ch = rec.channel('pose.xz')
    x, z = ch.col('x'), ch.col('z')
    out = []
    for gx, gz in rec.truth.get('gates', sim.GATES):
        k = np.nonzero(x >= gx)[0]
        if not len(k) or k[0] == 0:
            out.append(None)
            continue
        i = k[0]
        a = (gx - x[i - 1]) / max(x[i] - x[i - 1], 1e-9)
        zc = z[i - 1] + a * (z[i] - z[i - 1])
        out.append((float(ch.t[i - 1] + a * (ch.t[i] - ch.t[i - 1])), float(zc - gz)))
    return out


class GatesPassed(Metric):
    name, unit, direction = 'quad.gates_passed', 'gates', 'higher'
    requires = frozenset({'pose.xz'})

    def compute(self, rec, task):
        half = rec.truth.get('half', sim.GATE_HALF)
        c = crossings(rec)
        ok = [x is not None and abs(x[1]) <= half for x in c]
        spans = tuple(Span(rec.run_id, x[0] - 0.5, x[0] + 0.5, ('pose.xz',), f'g{k} {"pass" if o else "miss"}')
                      for k, (x, o) in enumerate(zip(c, ok)) if x is not None)
        return MetricValue(float(sum(ok)), spans=spans, extra={'per_gate': ok})


class Completion(Metric):
    name, unit, direction = 'quad.completion', '', 'higher'
    requires = frozenset({'pose.xz'})

    def compute(self, rec, task):
        half = rec.truth.get('half', sim.GATE_HALF)
        c = crossings(rec)
        return MetricValue(float(all(x is not None and abs(x[1]) <= half for x in c)))


class GateError(Metric):
    name, unit, direction = 'quad.gate_error_rms', 'm', 'lower'
    requires = frozenset({'pose.xz'})

    def compute(self, rec, task):
        e = [x[1] for x in crossings(rec) if x is not None]
        return MetricValue(float(np.sqrt(np.mean(np.square(e)))) if e else None)


class TimeToFinish(Metric):
    name, unit, direction = 'quad.time_to_last_gate', 's', 'lower'
    requires = frozenset({'pose.xz'})

    def compute(self, rec, task):
        c = crossings(rec)
        return MetricValue(c[-1][0] - rec.header.window[0] if c and c[-1] is not None else None)


def outcome(rec, values):
    if rec.events_of('crash') or (values.get('quad.floor') and values['quad.floor'].extra.get('verdict') == 'violated'):
        return 'crash'
    comp = values.get('quad.completion')
    if comp is not None and comp.value == 1.0:
        return 'success'
    return 'missed_gate'


def segments(rec):
    """Goal segments: between successive gate.target decisions."""
    ts = [rec.header.window[0]] + [e.t for e in rec.events_of('gate.target')] + [rec.header.window[1]]
    return [(a, b, f'seg{k}') for k, (a, b) in enumerate(zip(ts, ts[1:])) if b - a > 1.0]


class QuadReplayer(Replayer):
    horizon_s = 1.0      # the quadrotor is open-loop unstable: judge n-step prediction

    def __init__(self, low_params=sim.PARAMS_LOW, dt=0.01):
        self.p, self.dt = low_params, dt

    def _state(self, high, t0):
        pose, tw = high.channel('pose.xz'), high.channel('twist.xz')
        return tuple(pose.at([t0])[0]) + tuple(tw.at([t0])[0])

    def open_loop(self, high, t0, t1):
        if not high.has('actuator.cmd'):
            return None
        c = high.channel('actuator.cmd')
        T, S, _, _ = sim.rollout(self.p, self._state(high, t0), t0, t1, self.dt, cmds=(c.t, c.v))
        return T, S[:, :2]

    def reset_closed_loop(self, high, t0, t1):
        tg = [e for e in high.events_of('gate.target') if e.t <= t0 + 1e-9]
        gate = 0
        if tg:
            why = tg[-1].payload.get('why', 'g0')
            gate = len(sim.GATES) if why == 'exit' else int(why[1:])
        T, S, _, _ = sim.rollout(self.p, self._state(high, t0), t0, t1, self.dt, gate=gate)
        return T, S[:, :2]


def _skipped(rec):
    out, prev = [], 0
    for e in rec.events_of('gate.target'):
        why = e.payload.get('why', '')
        k = len(sim.GATES) if why == 'exit' else int(why[1:])
        if k > prev + 1:
            out.append((e.t, f'target jumped g{prev}->g{k}'))
        prev = k
    return out


def _saturated(rec):
    c = rec.channel('actuator.cmd')
    hit = np.nonzero((c.v >= 0.99 * rec.attachments.get('params', sim.PARAMS_LOW).t_max).any(axis=1))[0] \
        if 'params' in rec.attachments else np.nonzero((c.v >= 0.99 * sim.PARAMS_LOW.t_max).any(axis=1))[0]
    return [(float(c.t[hit[0]]), f'motor saturation ({len(hit)} samples)')] if len(hit) else []


GRAPH = DepGraph('quad-v1', (
    Node('planner', (), _skipped, ('events',)),
    Node('controller', ('planner',), None),
    Node('motors', ('controller',), _saturated, ('actuator.cmd',)),
    Node('plant', ('motors',), None, ('pose.xz',)),
))


def failure_node(rec, values):
    oc = outcome(rec, values)
    return {'crash': 'plant', 'missed_gate': 'plant'}.get(oc)


def _tilt_hint(by, ctx):
    ins = [i for i in by.get('divergence.split', []) if i.data.get('dominant') == 'plant']
    return 'raise the low tier motor lag/drag (plant card) before retuning attitude gains' if ins else None


PACK = Pack('toy_quad', 'planar_quad',
            {m.name: m for m in (Floor(), Tilt(), GatesPassed(), Completion(), GateError(), TimeToFinish())},
            event_vocab={'gate.target': 'planner picks the next gate', 'gate.passed': 'gate cleared',
                         'crash': 'ground contact'},
            decision_events=('gate.target',), outcome=outcome, segmenter=segments,
            dependency=GRAPH, failure_node=failure_node,
            taxonomy={'crash': 'hit the ground', 'missed_gate': 'a gate was not passed'},
            hints=(HintRule('quad-plant', _tilt_hint),))

TASK = TaskSpec('gates4', '1', 'planar_quad', (
    Objective('quad.floor', 'higher', 'safety', band=0.3),
    Objective('quad.completion', 'higher', 'task', ni_margin=0.1, margin_cite='toy'),
    Objective('quad.gate_error_rms', 'lower', 'efficiency', band=0.15),
    Objective('quad.time_to_last_gate', 'lower', 'efficiency', band=0.5),
), params={'cmd_limit': sim.QuadParams().t_max})
