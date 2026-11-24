"""A batch of worlds as padded arrays: one fastsim ``World`` (a scenario in its start frame) per env.

    worlds = Worlds.pack([World(s) for s in scenarios], limits)
    worlds.xy        [E, N, 2]  element centres, map frame (padding: far away, invisible, radius 0)
    worlds.radius    [E, N]     disk radius (m)
    worlds.visible   [E, N]     labelled (the sensor can report it)
    worlds.box       [E, N]     a box element; box_dims [E, N, 3] = (length, width, yaw), map frame
    worlds.valid     [E, N]     a real element, not padding
    worlds.start     [E, 3]     the start pose in the world frame (to write tracks back)
    worlds.near      [E, N]     contact_cause's reach of each element (its radius, half a box diagonal, the footprint's reach)

Every number is the one the scalar path computes (World and the contact geometry's own expressions).
"""

from dataclasses import dataclass
import math

import numpy as np

FAR = 1e6  # padding elements sit here, out of every range


def footprint_reach(footprint):
    return max(math.hypot(*p) for p in footprint)


def stack(parts, pads):
    """Concatenate dicts of [e, ...] arrays along the env axis, each axis padded to the widest part;
    ``pads[key]`` fills a key's padding (default 0) or is ``fn(array, width)`` for computed padding."""
    out = {}
    for key in parts[0]:
        arrays = [p[key] for p in parts]
        shape = [max(a.shape[d] for a in arrays) for d in range(arrays[0].ndim)]
        rows = []
        for a in arrays:
            if list(a.shape[1:]) != shape[1:]:
                pad = pads.get(key, 0)
                b = np.zeros([a.shape[0]] + shape[1:], a.dtype)
                if callable(pad):
                    b = pad(b)
                elif pad:
                    b[...] = pad
                b[tuple(slice(0, n) for n in a.shape)] = a
                a = b
            rows.append(a)
        out[key] = np.concatenate(rows, axis=0)
    return out


def _far(b):
    b[..., 0] = FAR
    b[..., 1] = FAR * (1 + np.arange(b.shape[1]))[None]
    return b


@dataclass
class Worlds:
    xy: np.ndarray
    radius: np.ndarray
    visible: np.ndarray
    box: np.ndarray
    box_dims: np.ndarray
    valid: np.ndarray
    start: np.ndarray
    near: np.ndarray
    ids: list
    count: np.ndarray
    boxes: np.ndarray       # [E, B] the box elements' indices (box_valid marks real ones)
    box_valid: np.ndarray

    @classmethod
    def pack(cls, worlds, footprint, n=None):
        E = len(worlds)
        N = n or max(len(w.ids) for w in worlds)
        reach = footprint_reach(footprint)
        xy = np.full((E, N, 2), FAR)
        xy[:, :, 1] = FAR * (1 + np.arange(N))[None]  # distinct, so padding never ties
        radius = np.zeros((E, N))
        visible = np.zeros((E, N), bool)
        box = np.zeros((E, N), bool)
        dims = np.zeros((E, N, 3))
        valid = np.zeros((E, N), bool)
        near = np.zeros((E, N))
        start = np.zeros((E, 3))
        for e, w in enumerate(worlds):
            k = len(w.ids)
            xy[e, :k], radius[e, :k], visible[e, :k] = w.xy, w.radius, w.visible
            box[e, :k], dims[e, :k], valid[e, :k] = w.box, w.box_dims, True
            start[e] = w.start
            for i in range(k):
                length, width, _ = w.box_dims[i]
                near[e, i] = w.radius[i] + (math.hypot(length, width) / 2 if w.box[i] else 0) + reach
        B = max(1, int(box.sum(axis=1).max()))
        boxes = np.zeros((E, B), np.int64)
        box_valid = np.zeros((E, B), bool)
        for e in range(E):
            idx = np.flatnonzero(box[e])
            boxes[e, :len(idx)], box_valid[e, :len(idx)] = idx, True
        return cls(xy, radius, visible, box, dims, valid, start, near, [list(w.ids) for w in worlds],
                   np.array([len(w.ids) for w in worlds]), boxes, box_valid)

    PADS = {'xy': _far}

    @classmethod
    def stack(cls, parts):
        A = stack([{**p.arrays(), 'count': p.count} for p in parts], cls.PADS)
        count = A.pop('count')
        return cls(**{k: A[k] for k in ('xy', 'radius', 'visible', 'box', 'box_dims', 'valid', 'start', 'near')},
                   ids=[i for p in parts for i in p.ids], count=count, boxes=A['boxes'], box_valid=A['box_valid'])

    def arrays(self):
        """The numeric fields, for a backend to move to its device."""
        return {k: getattr(self, k) for k in ('xy', 'radius', 'visible', 'box', 'box_dims', 'valid', 'start', 'near',
                                              'boxes', 'box_valid')}
