"""Perception: what the controller is told it sees. Default: truth labels inside a cone.

  cone        a cone (hfov, range) from the sensor mount (x, y, yaw in the body frame); a
              labelled disk nearer on the same ray hides what is behind it. Labels are truth.
  (a plugin)  any Perception subclass by reference

``HoldoverTracker`` turns sightings into tracks the way a tracker with hold-over does: every
track it still holds, out of view ones included, until its motion class's ``timeout`` after the
last sighting; a ``static`` track in view and range but unseen for ``miss_in_fov_s`` is dropped
sooner. confidence = exp(-age / tau). A class of no group turns static once seen within
``promote.within_m`` for ``promote.after_s``. Ids are opaque (creation order); a dropped object
returns under a new id.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from fnmatch import fnmatch
import math

import numpy as np

from fastsim import plugin


@dataclass(frozen=True)
class Track:
    id: str
    label: str
    x: float          # map frame
    y: float
    confidence: float
    static: bool
    index: int        # the scenario element (for diagnostics; a controller must not use it)


class Perception(ABC):
    def __init__(self, world, rng=None):
        self.world, self.rng = world, rng

    @abstractmethod
    def cone(self, pose):
        """(offsets, distances, in-view mask) of every element, occlusion aside."""

    @abstractmethod
    def see(self, pose):
        """Indices of the elements perceived from ``pose`` (map frame)."""

    def fov(self):
        """What the controller is told of the sensor's reach, or None."""
        return None


class ConeCamera(Perception):
    def __init__(self, world, rng=None, hfov_deg=90.0, range_m=20.0, occlusion=True, mount=None):
        super().__init__(world, rng)
        self.hfov, self.half, self.range = math.radians(hfov_deg), math.radians(hfov_deg) / 2, float(range_m)
        self.occlusion, self.mount = bool(occlusion), tuple(mount or (0.0, 0.0, 0.0))

    def fov(self):
        return (self.hfov, self.range)

    def cone(self, pose):
        bx, by, byaw = pose
        mx, my, myaw = self.mount
        c, s = math.cos(byaw), math.sin(byaw)
        x, y, yaw = bx + c * mx - s * my, by + s * mx + c * my, byaw + myaw
        w = self.world
        d = w.xy - (x, y)
        dist = np.hypot(d[:, 0], d[:, 1])
        bearing = np.arctan2(d[:, 1], d[:, 0]) - yaw
        bearing = np.arctan2(np.sin(bearing), np.cos(bearing))
        return d, dist, w.visible & (dist <= self.range) & (np.abs(bearing) <= self.half)

    def see(self, pose):
        w = self.world
        d, dist, seen = self.cone(pose)
        idx = np.flatnonzero(seen)
        if self.occlusion and len(idx) > 1:
            blockers = np.flatnonzero(w.visible & ~w.box & (dist <= self.range))
            keep = []
            for i in idx:
                u = d[i] / max(dist[i], 1e-9)
                along = d[blockers] @ u
                across = np.abs(d[blockers, 0] * u[1] - d[blockers, 1] * u[0])
                hidden = ((blockers != i) & (along > 0) & (along < dist[i] - w.radius[i])
                          & (across < w.radius[blockers]))
                if not hidden.any():
                    keep.append(i)
            idx = np.array(keep, dtype=int)
        return idx


PERCEPTION = {'cone': ConeCamera}


def perception(spec, world, rng=None, base=None):
    return plugin.build(spec or {'name': 'cone'}, PERCEPTION, base, world=world, rng=rng)


class HoldoverTracker:
    def __init__(self, world, sensor, class_groups, timeout, tau, miss_in_fov_s, promote=None):
        self.world, self.sensor = world, sensor
        self.timeout, self.tau, self.miss_in_fov_s = timeout, tau, float(miss_in_fov_s)
        self.promote = promote or {'within_m': 0.0, 'after_s': math.inf}
        self.prior = [class_groups[next((g for g, v in class_groups.items() if g != 'unknown' and any(
            fnmatch(label, p) for p in v['match'])), 'unknown')]['prior'] for label in world.labels]
        self.held, self.count = {}, 0

    def update(self, pose, now):
        w = self.world
        _, dist, cone = self.sensor.cone(pose)
        seen = set(self.sensor.see(pose).tolist())
        for i in sorted(seen):
            h = self.held.get(i)
            if h is None:
                self.count += 1
                h = self.held[i] = {'id': str(self.count), 'motion': self.prior[i], 'near': None}
            h['last'], h['miss'] = now, None
            if h['motion'] == 'unknown':
                h['near'] = (h['near'] if h['near'] is not None else now) if (
                    dist[i] <= self.promote['within_m']) else None
                if h['near'] is not None and now - h['near'] >= self.promote['after_s']:
                    h['motion'] = 'static'
        tracks = []
        for i, h in list(self.held.items()):
            age = now - h['last']
            if h['motion'] == 'static' and cone[i] and i not in seen:
                h['miss'] = now if h['miss'] is None else h['miss']
            elif i not in seen:
                h['miss'] = None
            if age > self.timeout[h['motion']] or (
                    h['miss'] is not None and now - h['miss'] > self.miss_in_fov_s):
                del self.held[i]
                continue
            tracks.append(Track(h['id'], w.labels[i], float(w.xy[i, 0]), float(w.xy[i, 1]),
                                math.exp(-age / self.tau[h['motion']]), h['motion'] == 'static', int(i)))
        return sorted(tracks, key=lambda t: int(t.id))
