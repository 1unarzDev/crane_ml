"""Course geometry and the pre-registered score of the gate -> U-turn -> dock mission.

Values mirror ``Tools/DecisionCal/mission.py`` (2026-11-09); the pack does not import the harness
(``judge/tests/test_dock_pack.py`` pins every value and the J to mission.run's own output).
"""

import math

import numpy as np

T_MAX = 120.0
TARGET = (2.6, -5.0, math.pi)
GATE_X, GATE_Y, BUOY_R = 10.0, 1.0, 0.15
SLIP = {'x0': 2.0, 'x1': 5.0, 'y': -5.0, 'half': 0.6, 'wall': 0.1}
FOOT = np.array([[0.5, 0.35], [0.5, -0.35], [-0.5, -0.35], [-0.5, 0.35],
                 [0.5, 0.0], [0.0, -0.35], [-0.5, 0.0], [0.0, 0.35]])
J_WEIGHTS = {'fail': 60.0, 'contact': 20.0, 'gate_miss': 30.0, 'pose': 10.0}
CONTACT_CAP, POSE_CAP = 3.0, 3.0
DOCK = {'lateral': 0.25, 'x_lo': 2.4, 'x_hi': 3.0, 'heading_deg': 15.0, 'speed': 0.08, 'hold_s': 2.0}
OBJECTS = ('wall_north', 'wall_south', 'end_wall', 'buoy_north', 'buoy_south')


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def footprint(pose):
    """World footprint points [N, 8, 2] of poses [N, 3]."""
    c, s = np.cos(pose[:, 2]), np.sin(pose[:, 2])
    px = pose[:, None, 0] + c[:, None] * FOOT[None, :, 0] - s[:, None] * FOOT[None, :, 1]
    py = pose[:, None, 1] + s[:, None] * FOOT[None, :, 0] + c[:, None] * FOOT[None, :, 1]
    return px, py


def walls():
    """(name, xmin, xmax, ymin, ymax) of the three slip walls."""
    w, y0, h = SLIP['wall'], SLIP['y'], SLIP['half']
    return (('wall_north', SLIP['x0'], SLIP['x1'], y0 + h, y0 + h + w),
            ('wall_south', SLIP['x0'], SLIP['x1'], y0 - h - w, y0 - h),
            ('end_wall', SLIP['x0'] - w, SLIP['x0'], y0 - h - w, y0 + h + w))


def objects_hit(pose):
    """[N, 5] bool per object (OBJECTS order); any() equals mission._contacts."""
    px, py = footprint(pose)
    out = []
    for _, x0, x1, y0, y1 in walls():
        out.append(((px >= x0) & (px <= x1) & (py >= y0) & (py <= y1)).any(1))
    for by in (GATE_Y, -GATE_Y):
        out.append((np.hypot(px - GATE_X, py - by) <= BUOY_R).any(1))
    return np.stack(out, -1)


def wall_clearance(pose):
    """[N] min distance from any footprint point to any slip wall rectangle (0 inside)."""
    px, py = footprint(pose)
    best = np.full(len(pose), np.inf)
    for _, x0, x1, y0, y1 in walls():
        dx = np.maximum(np.maximum(x0 - px, 0.0), px - x1)
        dy = np.maximum(np.maximum(y0 - py, 0.0), py - y1)
        best = np.minimum(best, np.hypot(dx, dy).min(1))
    return best


def in_slip(x, y):
    return (x >= SLIP['x0']) & (x <= SLIP['x1']) & (np.abs(y - SLIP['y']) <= SLIP['half'])


def dock_ok(x, y, yaw, speed):
    return ((np.abs(y - TARGET[1]) <= DOCK['lateral']) & (x >= DOCK['x_lo']) & (x <= DOCK['x_hi'])
            & (np.abs(wrap(yaw - TARGET[2])) <= np.radians(DOCK['heading_deg'])) & (speed <= DOCK['speed']))


def score(t_dock, docked, contacts, gate_miss, pose_err, t_max=T_MAX):
    """(J, components) with mission.score's formula."""
    W = J_WEIGHTS
    comp = {'time': float(t_dock if docked else t_max), 'fail': W['fail'] * (1.0 - float(docked)),
            'contact': W['contact'] * min(float(contacts), CONTACT_CAP), 'gate_miss': W['gate_miss'] * float(gate_miss),
            'pose': W['pose'] * min(float(pose_err), POSE_CAP)}
    J = comp['time'] + comp['fail'] + comp['contact'] + comp['gate_miss'] + comp['pose']
    return J, comp
