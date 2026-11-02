"""Robot-agnostic motion metrics over any planar pose channel (first two fields are position).

Usable by any robot that logs a pose: boats (pose.se2), planar aerial (pose.xz), ground robots."""

from __future__ import annotations

import numpy as np

from judge.metric import Metric, MetricValue, Status, unscorable
from judge.pack import Pack


class _PoseMetric(Metric):
    def __init__(self, pose):
        self.pose = pose
        self.requires = frozenset({pose})

    def xy(self, rec):
        ch = rec.channel(self.pose)
        t0, t1 = rec.header.window[:2]
        ch = ch.window(t0, t1) if t1 > t0 else ch
        return ch, ch.v[:, :2]


class PathLength(_PoseMetric):
    name, unit, direction = 'motion.path_length', 'm', 'lower'

    def compute(self, rec, task):
        ch, xy = self.xy(rec)
        if len(xy) < 2:
            return unscorable('fewer than 2 pose samples')
        d = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
        status = Status.PROVISIONAL if ch.spec.observability in ('estimated', 'surveyed') else Status.OK
        return MetricValue(d, status)


class Duration(Metric):
    name, unit, direction = 'motion.duration', 's', 'lower'

    def compute(self, rec, task):
        return MetricValue(rec.duration)


class MeanSpeed(_PoseMetric):
    name, unit, direction = 'motion.mean_speed', 'm/s', 'none'

    def compute(self, rec, task):
        ch, xy = self.xy(rec)
        if len(xy) < 2 or ch.t[-1] <= ch.t[0]:
            return unscorable('too short')
        return MetricValue(float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)) / (ch.t[-1] - ch.t[0])))


class HeadingRateRMS(_PoseMetric):
    name, unit, direction = 'motion.heading_rate_rms', 'rad/s', 'lower'

    def compute(self, rec, task):
        ch, _ = self.xy(rec)
        if ch.v.shape[1] < 3 or len(ch) < 3 or ch.t[-1] - ch.t[0] < 1.0:
            return unscorable('no heading field')
        t = np.arange(ch.t[0], ch.t[-1], 0.1)      # uniform: jittery high-rate logs inflate rates
        yaw = np.interp(t, ch.t, np.unwrap(ch.v[:, 2]))
        w = np.diff(yaw) / 0.1
        return MetricValue(float(np.sqrt(np.mean(w ** 2))))


class JerkRMS(_PoseMetric):
    """RMS of the third derivative of position on a uniform resample (smoothness)."""
    name, unit, direction = 'motion.jerk_rms', 'm/s^3', 'lower'

    def compute(self, rec, task):
        ch, xy = self.xy(rec)
        if len(ch) < 8:
            return unscorable('too short')
        dt = 0.1
        t = np.arange(ch.t[0], ch.t[-1], dt)
        p = np.column_stack([np.interp(t, ch.t, xy[:, k]) for k in range(2)])
        j = np.diff(p, n=3, axis=0) / dt ** 3
        return MetricValue(float(np.sqrt(np.mean((j ** 2).sum(1)))) if len(j) else None)


class Effort(Metric):
    name, unit, direction = 'actuation.effort_mean', 'cmd', 'lower'
    requires = frozenset({'actuator.cmd'})

    def compute(self, rec, task):
        return MetricValue(float(np.mean(np.abs(rec.channel('actuator.cmd').v))))


class Saturation(Metric):
    """Fraction of command samples with any axis at >= 98% of the declared limit (``task.params``
    ``cmd_limit``, default 1.0)."""
    name, unit, direction = 'actuation.saturation_fraction', '', 'lower'
    requires = frozenset({'actuator.cmd'})

    def compute(self, rec, task):
        lim = float(((task.params if task else {}) or {}).get('cmd_limit', 1.0))
        v = np.abs(rec.channel('actuator.cmd').v)
        return MetricValue(float(np.mean((v >= 0.98 * lim).any(axis=1))))


def make(pose='pose.se2'):
    ms = [PathLength(pose), Duration(), MeanSpeed(pose), HeadingRateRMS(pose), JerkRMS(pose), Effort(), Saturation()]
    return Pack('generic_motion', '*', {m.name: m for m in ms})


PACK = make()
