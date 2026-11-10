"""Dock-mission metrics from evaluator truth and events only (never ``obs.*``), identical on every
backend.

One pass over ``truth.pose`` / ``truth.twist`` (``facts``) applies mission.run's rules per sample
(sample 0 = the start state; rules are evaluated on samples 1..K, as the harness evaluates them after
each plant step): the first gate crossing (x passes 10 from below; miss when |y| > 1), contact
rising edges of the footprint against the slip walls and gate buoys while not docked, slip entries
and exits of the centre, the docking predicate held 2 s (time to dock = start of the hold). Every
metric is unscorable when truth.pose is absent (a physical open-loop log has no truth: nothing is
scored, nothing is zero). Without truth.twist the speed is differentiated from the pose and the
speed-dependent metrics are provisional.

The writer's stored events are compared with the events recomputed here; a disagreement is a
writer bug or a backend semantic difference and is reported (``event_check``), never silently used.
"""

from __future__ import annotations

import math

import numpy as np

from judge.insight import Insight
from judge.insight.events import align
from judge.metric import Metric, MetricValue, Span, Status, unscorable
from judge.pack import Pack
from judge.record import Event, RunRecord, twist_from_pose
from judge.task import Objective, TaskSpec

from judge.packs.dock_mission import geometry as G
from judge.packs.dock_mission import _hdr

TRUTH = ('truth.pose', 'truth.twist')
DOCK_KINDS = ('gate_cross', 'contact', 'dock_enter', 'dock_hold_start', 'docked', 'timeout')
EVENT_TOL_S = 0.05             # 2.5 ticks at 50 Hz: a writer stamping on the neighbouring tick still matches
GATE_Y_TOL = 0.05
OUTCOMES = ('docked', 'timeout_no_approach', 'timeout_in_slip', 'contact_fail', 'gate_miss', 'unscorable')

_CACHE = {}


def facts(rec: RunRecord):
    """Per-run mission facts from truth (cached per record object)."""
    hit = _CACHE.get(id(rec))
    if hit is not None and hit[0] is rec:
        return hit[1]
    out = _facts(rec)
    if len(_CACHE) > 256:
        _CACHE.clear()
    _CACHE[id(rec)] = (rec, out)
    return out


def _facts(rec):
    if not rec.has('truth.pose'):
        return None
    pose = rec.channel('truth.pose')
    t = np.asarray(pose.t, float)
    x, y, yaw = pose.col('x'), pose.col('y'), pose.col('yaw')
    provisional = False
    if rec.has('truth.twist'):
        tw = rec.channel('truth.twist')
        if len(tw.t) == len(t) and np.array_equal(tw.t, t):
            u, v = tw.col('u'), tw.col('v')
        else:
            u, v = np.interp(t, tw.t, tw.col('u')), np.interp(t, tw.t, tw.col('v'))
    else:
        d = twist_from_pose(pose)
        u, v = d.col('vx'), d.col('vy')
        provisional = True
    speed = np.hypot(u, v)
    P = np.column_stack([x, y, yaw])
    objs = G.objects_hit(P)
    hit_any = objs.any(1)
    ok = G.dock_ok(x, y, yaw, speed)
    slip = G.in_slip(x, y)
    t0 = float(rec.header.window[0]) if rec.header.window else 0.0
    K = len(t) - 1
    gate = None
    contacts = []
    in_contact = False
    hold_since, hold_k = math.nan, None
    # The hold is counted in samples: a backend clock that steps a hair short of the nominal dt (the
    # integrated player steps 0.0199999921 s) must not turn a 100-tick hold into 1.99998 s.
    hold_tol = 0.5 * float(np.median(np.diff(t))) if K >= 1 else 1e-9
    holds, entries, exits = [], [], []
    docked_k = None
    in_slip_prev = bool(slip[0])
    for k in range(1, K + 1):
        if gate is None and x[k - 1] < G.GATE_X <= x[k]:
            gate = k
        h = bool(hit_any[k])
        if h and not in_contact:
            contacts.append(k)
        in_contact = h
        if slip[k] and not in_slip_prev:
            entries.append(k)
        elif in_slip_prev and not slip[k]:
            exits.append(k)
        in_slip_prev = bool(slip[k])
        if ok[k]:
            if math.isnan(hold_since):
                hold_since, hold_k = float(t[k]), k
                holds.append(k)
        else:
            hold_since, hold_k = math.nan, None
        if ok[k] and t[k] - hold_since >= G.DOCK['hold_s'] - hold_tol:
            docked_k = k
            break
    end_k = docked_k if docked_k is not None else K
    seg = P[:end_k + 1]
    path = float(np.hypot(*np.diff(seg[:, :2], axis=0).T).sum()) if end_k > 0 else 0.0
    clear = G.wall_clearance(seg)
    ck = int(np.argmin(clear))
    docked = docked_k is not None
    gate_y = float(y[gate]) if gate is not None else None
    gate_miss = gate_y is None or abs(gate_y) > G.GATE_Y
    pos_err = float(np.hypot(x[end_k] - G.TARGET[0], y[end_k] - G.TARGET[1]))
    head_err = float(abs(G.wrap(yaw[end_k] - G.TARGET[2])))
    t_dock = (hold_since - t0) if docked else None
    J, comp = G.score(t_dock if docked else 0.0, docked, len(contacts), gate_miss, 0.0 if docked else pos_err)
    # retries: slip entries that were followed by an exit before docking
    retries = sum(1 for e in entries if any(e < q <= end_k for q in exits))
    return {'t': t, 'x': x, 'y': y, 'yaw': yaw, 'speed': speed, 'objs': objs, 'provisional': provisional,
            'end_k': end_k, 'docked': docked, 'docked_k': docked_k, 'hold_k': hold_k if docked else None,
            't_dock': t_dock, 'gate_k': gate, 'gate_y': gate_y, 'gate_miss': gate_miss, 'contacts': contacts,
            'entries': entries, 'exits': exits, 'holds': holds, 'retries': retries, 'pos_err': pos_err,
            'head_err': head_err, 'path': path, 'clearance': float(clear[ck]), 'clearance_k': ck,
            'J': J, 'J_components': comp, 'duration': float(t[end_k] - t[0]), 't0': t0,
            'end_reason': rec.header.window[2] if len(rec.header.window) > 2 else ''}


def _sp(rec, f, k0, k1, label, chans=TRUTH):
    t = f['t']
    return Span(rec.run_id, float(t[k0]), float(t[min(k1, len(t) - 1)]), chans, label)


class _M(Metric):
    requires = frozenset({'truth.pose'})

    def compute(self, rec, task):
        f = facts(rec)
        if f is None:
            return unscorable(f'no truth.pose in {rec.header.source}')
        out = self.value(rec, f)
        if f['provisional'] and getattr(self, 'speed_dependent', False) and out.status == Status.OK:
            out = MetricValue(out.value, Status.PROVISIONAL, out.sigma, out.spans,
                              (out.why + '; ' if out.why else '') + 'speed differentiated from truth.pose (no truth.twist)',
                              out.extra)
        return out


class Docked(_M):
    name, unit, direction, speed_dependent = 'dock.docked', 'bool', 'higher', True

    def value(self, rec, f):
        if f['docked']:
            return MetricValue(1.0, spans=(_sp(rec, f, f['hold_k'], f['docked_k'], 'dock hold'),),
                               extra={'t_docked_s': float(f['t'][f['docked_k']])})
        return MetricValue(0.0, spans=(_sp(rec, f, f['end_k'], f['end_k'], 'end, not docked'),))


class TimeToDock(_M):
    name, unit, direction, speed_dependent = 'dock.time_to_dock', 's', 'lower', True

    def value(self, rec, f):
        if not f['docked']:
            return MetricValue(None, Status.UNSCORABLE, why='did not dock')
        return MetricValue(f['t_dock'], spans=(_sp(rec, f, f['hold_k'], f['docked_k'], 'dock hold'),))


class Retries(_M):
    name, unit, direction, speed_dependent = 'dock.retries', 'count', 'lower', True

    def value(self, rec, f):
        spans = tuple(_sp(rec, f, e, next((q for q in f['exits'] if q > e), f['end_k']), 'slip attempt')
                      for e in f['entries'])
        return MetricValue(float(f['retries']), spans=spans,
                           extra={'entries': len(f['entries']), 'hold_starts': len(f['holds'])})


class PoseError(_M):
    name, unit, direction = 'dock.pose_error', 'm', 'lower'

    def value(self, rec, f):
        return MetricValue(f['pos_err'], spans=(_sp(rec, f, f['end_k'], f['end_k'], 'terminal pose', ('truth.pose',)),),
                           extra={'heading_error_rad': f['head_err'], 'heading_error_deg': math.degrees(f['head_err']),
                                  'j_term_m': 0.0 if f['docked'] else f['pos_err']})


class HeadingError(_M):
    name, unit, direction = 'dock.heading_error', 'rad', 'lower'

    def value(self, rec, f):
        return MetricValue(f['head_err'], spans=(_sp(rec, f, f['end_k'], f['end_k'], 'terminal pose', ('truth.pose',)),))


class Contacts(_M):
    name, unit, direction = 'dock.contacts', 'count', 'lower'

    def value(self, rec, f):
        t = f['t']
        spans = tuple(_sp(rec, f, k, k, 'contact ' + '+'.join(o for o, h in zip(G.OBJECTS, f['objs'][k]) if h),
                          ('truth.pose',)) for k in f['contacts'])
        first = float(t[f['contacts'][0]]) if f['contacts'] else None
        return MetricValue(float(len(f['contacts'])), spans=spans, extra={'first_t': first})


class GateCrossed(_M):
    name, unit, direction = 'dock.gate_crossed', 'bool', 'higher'

    def value(self, rec, f):
        k = f['gate_k']
        return MetricValue(float(k is not None), spans=(_sp(rec, f, k, k, 'gate', ('truth.pose',)),) if k else ())


class GateOffset(_M):
    name, unit, direction = 'dock.gate_offset', 'm', 'none'

    def value(self, rec, f):
        k = f['gate_k']
        if k is None:
            return MetricValue(None, Status.UNSCORABLE, why='gate not crossed')
        return MetricValue(f['gate_y'], spans=(_sp(rec, f, k, k, 'gate', ('truth.pose',)),))


class GateMiss(_M):
    name, unit, direction = 'dock.gate_miss', 'bool', 'lower'

    def value(self, rec, f):
        k = f['gate_k']
        return MetricValue(float(f['gate_miss']), spans=(_sp(rec, f, k, k, 'gate', ('truth.pose',)),) if k else (),
                           why='' if k is not None else 'gate not crossed (counts as a miss)')


class PathLength(_M):
    name, unit, direction = 'dock.path_length', 'm', 'none'

    def value(self, rec, f):
        return MetricValue(f['path'], spans=(_sp(rec, f, 0, f['end_k'], 'path', ('truth.pose',)),))


class Duration(_M):
    name, unit, direction = 'dock.duration', 's', 'none'

    def value(self, rec, f):
        return MetricValue(f['duration'], spans=(_sp(rec, f, 0, f['end_k'], 'scored span', ('truth.pose',)),))


class Score(_M):
    name, unit, direction, speed_dependent = 'dock.J', 's', 'lower', True

    def value(self, rec, f):
        sp = [_sp(rec, f, 0, f['end_k'], 'scored span')]
        sp += [_sp(rec, f, k, k, 'contact', ('truth.pose',)) for k in f['contacts']]
        if f['gate_k']:
            sp.append(_sp(rec, f, f['gate_k'], f['gate_k'], 'gate', ('truth.pose',)))
        return MetricValue(f['J'], spans=tuple(sp), extra={'components': dict(f['J_components']),
                                                           'formula': 't_dock|120 + 60(1-docked) + 20 min(c,3) '
                                                                      '+ 30 gate_miss + 10 min(pose_err,3)'})


class WallClearance(_M):
    name, unit, direction = 'dock.min_wall_clearance', 'm', 'higher'

    def value(self, rec, f):
        k = f['clearance_k']
        return MetricValue(f['clearance'], spans=(_sp(rec, f, k, k, 'closest to a wall', ('truth.pose',)),),
                           extra={'definition': 'min over samples of the footprint-point distance to a slip wall '
                                                'rectangle (0 = touching or inside)'})


class EventsAgree(_M):
    name, unit, direction = 'dock.events_agree', 'bool', 'higher'

    def value(self, rec, f):
        chk = event_check(rec)
        if chk['stored'] == 0:
            return MetricValue(None, Status.UNSCORABLE, why='the writer stored no dock events')
        return MetricValue(float(not chk['disagreements']), extra=chk,
                           spans=tuple(Span(rec.run_id, d['t'], d['t'], (), d['what']) for d in chk['disagreements']))


# ------------------------------------------------------------------------------------ events

def recompute_events(rec):
    """Dock events from truth, the writer's vocabulary (SCHEMA_V2.md)."""
    f = facts(rec)
    if f is None:
        return ()
    t = f['t']
    ev = [Event(float(t[0]), 'episode_start', 'judge', {'tick': 0})]
    if f['gate_k'] is not None:
        ev.append(Event(float(t[f['gate_k']]), 'gate_cross', 'judge', {'y': f['gate_y'], 'miss': f['gate_miss']}))
    for k in f['contacts']:
        ev.append(Event(float(t[k]), 'contact', 'judge', {'with': [o for o, h in zip(G.OBJECTS, f['objs'][k]) if h]}))
    for k in f['entries']:
        ev.append(Event(float(t[k]), 'dock_enter', 'judge', {}))
    for k in f['holds']:
        ev.append(Event(float(t[k]), 'dock_hold_start', 'judge', {}))
    if f['docked']:
        ev.append(Event(float(t[f['docked_k']]), 'docked', 'judge', {'t_dock': f['t_dock']}))
    elif f['end_reason'] == 'timeout' or f['duration'] >= G.T_MAX - 1e-6:
        ev.append(Event(float(t[f['end_k']]), 'timeout', 'judge', {}))
    return tuple(sorted(ev, key=lambda e: e.t))


def event_check(rec):
    """Stored vs recomputed dock events: {stored, recomputed, kinds_checked, not_emitted, disagreements}."""
    stored = [e for e in rec.events if e.kind in DOCK_KINDS]
    mine = [e for e in recompute_events(rec) if e.kind in DOCK_KINDS]
    declared = (rec.caps or {}).get('event_kinds')
    # every dock kind is checked once the writer stores any; a writer that does not emit some kind
    # says so in caps['event_kinds'] (the kinds it emits), otherwise a missing kind is a disagreement
    kinds = sorted(set(DOCK_KINDS) & set(declared)) if declared is not None else list(DOCK_KINDS)
    out = {'stored': len(stored), 'recomputed': len(mine), 'kinds_checked': kinds,
           'not_emitted': sorted(set(DOCK_KINDS) - set(kinds)), 'disagreements': []}
    if not stored:
        return out
    for kind in kinds:
        a = [(e.t, kind, e) for e in stored if e.kind == kind]
        b = [(e.t, kind, e) for e in mine if e.kind == kind]
        for i, j, s in align(a, b, EVENT_TOL_S):
            if s == 'match':
                ea, eb = a[i][2], b[j][2]
                if kind == 'gate_cross' and 'y' in (ea.payload or {}):
                    if abs(float(ea.payload['y']) - float(eb.payload['y'])) > GATE_Y_TOL:
                        out['disagreements'].append({'kind': kind, 't': ea.t, 'what': (
                            f"gate_cross y stored {ea.payload['y']:.3f} vs truth {eb.payload['y']:.3f}")})
                continue
            if i is not None and j is not None:
                out['disagreements'].append({'kind': kind, 't': a[i][0],
                                             'what': f'{kind} stored t={a[i][0]:.3f} vs truth t={b[j][0]:.3f}'})
            elif i is not None:
                out['disagreements'].append({'kind': kind, 't': a[i][0],
                                             'what': f'{kind} stored at t={a[i][0]:.3f}, not in truth'})
            else:
                out['disagreements'].append({'kind': kind, 't': b[j][0],
                                             'what': f'{kind} in truth at t={b[j][0]:.3f}, not stored'})
    return out


def event_insight(rec):
    """Insight when the writer's events disagree with truth (writer bug or backend semantic
    difference), None when they agree or none were stored."""
    chk = event_check(rec)
    if not chk['disagreements']:
        return None
    be = _hdr.backend(rec).get('name') or rec.header.source
    d = chk['disagreements']
    return Insight('dock.events_disagree', 'warn',
                   f'{rec.run_id} ({be}): {len(d)} stored dock event(s) disagree with events recomputed from truth; '
                   f'first: {d[0]["what"]} -> a writer bug or a backend semantic difference',
                   evidence=tuple(Span(rec.run_id, x['t'], x['t'], ('truth.pose',), x['kind']) for x in d[:10]),
                   scope={'run': rec.run_id, 'backend': be}, basis='descriptive', data=chk)


# ------------------------------------------------------------------------------------ outcome

def outcome(rec, values=None):
    f = facts(rec)
    if f is None:
        return 'unscorable'
    if f['docked']:
        return 'docked'
    if f['contacts']:
        return 'contact_fail'
    if f['gate_miss']:
        return 'gate_miss'
    if f['entries'] or G.in_slip(f['x'][f['end_k']], f['y'][f['end_k']]):
        return 'timeout_in_slip'
    return 'timeout_no_approach'


def outcome_time(rec):
    """(category, t at which the outcome was decided) for the divergence module."""
    cat = outcome(rec)
    f = facts(rec)
    if f is None:
        return cat, None
    t = f['t']
    if cat == 'docked':
        return cat, float(t[f['docked_k']])
    if cat == 'contact_fail':
        return cat, float(t[f['contacts'][0]])
    if cat == 'gate_miss' and f['gate_k'] is not None:
        return cat, float(t[f['gate_k']])
    return cat, float(t[f['end_k']])


METRICS = (Docked(), TimeToDock(), Retries(), PoseError(), HeadingError(), Contacts(), GateCrossed(), GateOffset(),
           GateMiss(), PathLength(), Duration(), Score(), WallClearance(), EventsAgree())

PACK = Pack('dock_mission', 'marine.asv', {m.name: m for m in METRICS},
            event_vocab={'episode_start': 'episode begins', 'gate_cross': 'first crossing of x = 10 (payload y)',
                         'contact': 'footprint enters a wall or buoy (payload with)',
                         'dock_enter': 'centre enters the slip', 'dock_hold_start': 'docking predicate becomes true',
                         'docked': 'predicate held 2 s', 'timeout': 'time limit reached'},
            decision_events=('dock_enter', 'dock_hold_start'), anchor_event='episode_start', outcome=outcome,
            taxonomy={'docked': 'held the docking predicate for 2 s',
                      'timeout_no_approach': 'never entered the slip',
                      'timeout_in_slip': 'entered the slip but never held the docking predicate',
                      'contact_fail': 'touched a wall or buoy and did not dock',
                      'gate_miss': 'missed the gate and did not dock',
                      'unscorable': 'no truth.pose: the record cannot be scored'})

TASK = TaskSpec('decisioncal.dock_mission', '1', 'marine.asv', (
    Objective('dock.contacts', 'lower', 'safety'),
    Objective('dock.docked', 'higher', 'task'),
    Objective('dock.time_to_dock', 'lower', 'efficiency'),
    Objective('dock.J', 'lower', 'diagnostic'),
), params={'t_max_s': G.T_MAX})
