"""The RoboBoat course Pack: referee verdicts as judge metrics over a RunRecord.

The record carries the course truth in ``rec.truth`` and the true track as ``pose.se2``; task
events (``rec.events`` whose payload has an ``event`` field) feed the referee as they do in a session.
"""

from __future__ import annotations

from judge.metric import Metric, MetricValue, Span
from judge.pack import Pack
from judge.packs.roboboat_course import referee

POSE = 'pose.se2'


def verdicts(rec):
    """{task id: referee verdict} for every task on the record's course (cached per record)."""
    cache = rec.attachments.get('referee') if isinstance(rec.attachments, dict) else None
    if cache is not None:
        return cache
    ch = rec.channel(POSE)
    track = [(float(t), float(x), float(y), float(yaw)) for t, (x, y, yaw) in zip(ch.t, ch.v[:, :3])]
    events = [dict(e.payload, time=e.t) for e in rec.events if 'event' in e.payload]
    out = {t['id']: referee.judge(t['id'], rec.truth, track, events) for t in rec.truth.get('tasks', [])}
    if isinstance(rec.attachments, dict):
        rec.attachments['referee'] = out
    return out


class TasksPassed(Metric):
    """Fraction of the judged tasks (not N/A) that passed."""
    name, unit, direction = 'course.tasks_passed', '', 'higher'
    requires = frozenset({POSE})

    def compute(self, rec, task):
        judged = {k: v for k, v in verdicts(rec).items() if v['verdict'] != 'N/A'}
        if not judged:
            return MetricValue(None, why='no task on this course is judged from the track')
        passed = sorted(k for k, v in judged.items() if v['verdict'] == 'PASS')
        return MetricValue(len(passed) / len(judged), extra={'passed': passed, 'judged': sorted(judged)})


class Contacts(Metric):
    """Course objects the hull rectangle touched, over every task."""
    name, unit, direction = 'course.contacts', 'objects', 'lower'
    requires = frozenset({POSE})

    def compute(self, rec, task):
        touched = sorted({c for v in verdicts(rec).values() for c in v.get('contacts', [])})
        t0, t1 = rec.header.window[:2]
        spans = (Span(rec.run_id, t0, t1, (POSE,), 'contact'),) if touched else ()
        return MetricValue(float(len(touched)), spans=spans, extra={'contacts': touched})


def outcome(rec, values):
    if (values.get('course.contacts') and values['course.contacts'].value):
        return 'contact'
    done = values.get('course.tasks_passed')
    if done is not None and done.value == 1.0:
        return 'success'
    return 'task_failed' if done is not None and done.value is not None else 'unknown'


PACK = Pack('roboboat_course', 'roboboat', {m.name: m for m in (TasksPassed(), Contacts())},
            event_vocab={'task': 'a task-tree event (payload.event: stage_started, stage_completed, ...)'},
            outcome=outcome,
            taxonomy={'contact': 'the hull touched a course object', 'task_failed': 'a task check failed'})
