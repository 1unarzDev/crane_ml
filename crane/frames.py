"""Frames and clocks of crane.run/1 records: put two runs into one frame and onto one clock, or refuse.

A frame is ``{world: enu|ned, origin: <name>, body: flu|frd}``. Angles are intrinsic Z-Y-X (yaw, pitch,
roll) of the body axes in the world axes, so in ENU/FLU yaw is counter-clockwise from east and in NED/FRD
yaw is clockwise from north. ENU <-> NED swaps x and y and negates z; FLU <-> FRD negates the body's y and
z; together they map yaw to ``pi/2 - yaw``, pitch to ``-pitch`` and keep roll.

Poses are columns ``<agent>.<axis>`` with axes x y z yaw pitch roll (planar: x y yaw). Times are seconds
from the record clock's ``t0`` (``crane.record`` documents the clock).
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math

import numpy as np

WORLDS = ('enu', 'ned')
BODIES = ('flu', 'frd')
POSE6 = ('x', 'y', 'z', 'yaw', 'pitch', 'roll')
PLANAR = ('x', 'y', 'yaw')
ANGLES = ('yaw', 'pitch', 'roll')
CLOCKS = ('sim', 'wall')

_SWAP = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])   # ENU <-> NED, its own inverse
_FLIP = np.diag([1.0, -1.0, -1.0])                                      # FLU <-> FRD, its own inverse


class FrameError(ValueError):
    pass


def frame_of(f: dict) -> dict:
    """A checked frame; a missing body is the world's usual one (enu: flu, ned: frd); ``type`` reads as world
    (the ``.ctraj`` header spelling)."""
    if not isinstance(f, dict):
        raise FrameError(f'a frame is a mapping {{world, origin, body}}, not {f!r}')
    world = f.get('world', f.get('type'))
    if world not in WORLDS:
        raise FrameError(f'frame world {world!r} is not one of {WORLDS}')
    body = f.get('body') or {'enu': 'flu', 'ned': 'frd'}[world]
    if body not in BODIES:
        raise FrameError(f'frame body {body!r} is not one of {BODIES}')
    origin = f.get('origin')
    if not isinstance(origin, str) or not origin:
        raise FrameError('a frame names its origin')
    return {'world': world, 'origin': origin, 'body': body}


def wrap(a):
    return (np.asarray(a, float) + math.pi) % (2 * math.pi) - math.pi


def _rot(yaw, pitch, roll):
    """(N, 3, 3) body-to-world rotations from Z-Y-X angles."""
    cy, sy, cp, sp, cr, sr = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch), np.cos(roll), np.sin(roll)
    return np.stack([np.stack([cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], -1),
                     np.stack([sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], -1),
                     np.stack([-sp, cp * sr, cp * cr], -1)], -2)


def _angles(r):
    return (np.arctan2(r[..., 1, 0], r[..., 0, 0]), -np.arcsin(np.clip(r[..., 2, 0], -1.0, 1.0)),
            np.arctan2(r[..., 2, 1], r[..., 2, 2]))


def convert(pose: dict, src: dict, dst: dict, transform: dict | None = None) -> dict:
    """One agent's pose ``{x, y, [z], yaw, [pitch, roll]}`` (floats or arrays) from frame ``src`` to ``dst``.

    Different origins need ``transform`` ``{from, to, xyz: [dx, dy, dz], yaw}``: the ``src`` origin's pose in
    the ``dst`` origin, in ``src``'s world axes (a position p becomes ``Rz(yaw) p + xyz``). A planar pose stays
    planar, so a change that would tilt it (world changed but body not, or the reverse) is refused."""
    src, dst = frame_of(src), frame_of(dst)
    if src['origin'] != dst['origin']:
        if not transform or transform.get('from') != src['origin'] or transform.get('to') != dst['origin']:
            raise FrameError(f"origins differ ({src['origin']} vs {dst['origin']}): give a transform "
                             f"{{from: {src['origin']}, to: {dst['origin']}, xyz, yaw}}")
    planar = 'pitch' not in pose and 'roll' not in pose and 'z' not in pose
    n = np.shape(pose['x'])
    get = lambda k: np.broadcast_to(np.asarray(pose.get(k, 0.0), float), n)  # noqa: E731
    p = np.stack([get('x'), get('y'), get('z')], -1)
    r = _rot(get('yaw'), get('pitch'), get('roll'))
    if src['origin'] != dst['origin']:
        tz = _rot(np.float64(transform.get('yaw', 0.0)), np.float64(0.0), np.float64(0.0))
        p = p @ tz.T + np.asarray(transform.get('xyz', (0.0, 0.0, 0.0)), float)
        r = tz @ r
    w = _SWAP if src['world'] != dst['world'] else np.eye(3)
    b = _FLIP if src['body'] != dst['body'] else np.eye(3)
    p, r = p @ w.T, w @ r @ b
    yaw, pitch, roll = _angles(r)
    if planar:
        if np.any(np.abs(pitch) > 1e-9) or np.any(np.abs(wrap(roll)) > 1e-9):
            raise FrameError(f"a planar pose cannot go from {src['world']}/{src['body']} to "
                             f"{dst['world']}/{dst['body']}: it would tilt")
        return {'x': p[..., 0], 'y': p[..., 1], 'yaw': yaw}
    return {'x': p[..., 0], 'y': p[..., 1], 'z': p[..., 2], 'yaw': yaw, 'pitch': pitch, 'roll': roll}


@dataclass
class Poses:
    """Pose columns on a time grid: ``t`` (seconds from ``clock['t0']``), ``columns`` ``<agent>.<axis>``."""
    t: np.ndarray
    columns: dict
    frame: dict
    clock: dict
    pose_dims: str = '6dof'
    notes: list = field(default_factory=list)

    def agents(self) -> list:
        return sorted({n.rsplit('.', 1)[0] for n in self.columns if n.endswith('.x')})

    def pose(self, agent: str) -> dict:
        return {a: self.columns[f'{agent}.{a}'] for a in POSE6 if f'{agent}.{a}' in self.columns}


def poses(record) -> Poses:
    """The pose columns of a ``crane.record.RunRecord`` (or a ``theater.traj.Traj`` plus its frame and clock)."""
    traj = record.traj
    if traj is None:
        raise FrameError(f'{getattr(record, "run_id", "run")}: no trajectory (an event-kind run has no poses)')
    cols = {n: np.asarray(v, float) for n, v in traj.columns.items()
            if n.rsplit('.', 1)[-1] in POSE6}
    t = np.asarray(traj.times(), float)
    dims = traj.header.get('pose_dims') or ('6dof' if any(n.endswith('.z') for n in cols) else 'planar')
    return Poses(t, cols, frame_of(record.frame), dict(record.clock), dims)


def _as_poses(x) -> Poses:
    return x if isinstance(x, Poses) else poses(x)


def to_common(record, target: dict, transform: dict | None = None) -> Poses:
    """Every agent's poses in ``target``; refuses different origins unless ``transform`` maps them."""
    ps = _as_poses(record)
    target = frame_of(target)
    cols = {}
    for agent in ps.agents():
        for a, v in convert(ps.pose(agent), ps.frame, target, transform).items():
            cols[f'{agent}.{a}'] = v
    return replace(ps, columns=cols, frame=target)


def project_planar(record) -> Poses:
    """A 6-DOF run as x, y, yaw: z, pitch and roll are dropped (Z-Y-X yaw is the heading of the body x axis
    projected on the ground plane, so it is kept as is). Planar input is returned unchanged."""
    ps = _as_poses(record)
    if ps.pose_dims == 'planar':
        return ps
    cols = {n: v for n, v in ps.columns.items() if n.rsplit('.', 1)[-1] in PLANAR}
    return replace(ps, columns=cols, pose_dims='planar', notes=[*ps.notes, 'projected_planar_from_6dof'])


def _interp(t_new, t, v, angle):
    if angle:
        return wrap(np.interp(t_new, t, np.unwrap(v)))
    return np.interp(t_new, t, v)


def resample(ps: Poses, t_new) -> Poses:
    """``ps`` at times ``t_new`` (same clock): linear, angles the short way round."""
    t_new = np.asarray(t_new, float)
    cols = {n: _interp(t_new, ps.t, v, n.rsplit('.', 1)[-1] in ANGLES) for n, v in ps.columns.items()}
    return replace(ps, t=t_new, columns=cols)


def align(a, b, hz: float | None = None, *, offset_s: float | None = None) -> tuple:
    """Two runs on one clock: ``(a', b')`` sampled at the same times over their overlap, on ``a``'s clock.

    Without ``offset_s`` both clocks must have the same source (sim or wall) and their ``t0`` places them
    (b's t = 0 is a's t = b.t0 - a.t0). A sim run and a wall run need an explicit ``offset_s`` (a's time of
    b's t = 0). ``hz`` defaults to the lower of the two rates. Frames are not touched: call ``to_common``."""
    a, b = _as_poses(a), _as_poses(b)
    for ps in (a, b):
        if ps.clock.get('source') not in CLOCKS:
            raise FrameError(f"clock source {ps.clock.get('source')!r} is not one of {CLOCKS}")
        if len(ps.t) < 2:
            raise FrameError('a run needs two samples to be resampled')
    if offset_s is None:
        if a.clock['source'] != b.clock['source']:
            raise FrameError(f"clocks differ ({a.clock['source']} vs {b.clock['source']}): give offset_s")
        offset_s = float(b.clock.get('t0') or 0.0) - float(a.clock.get('t0') or 0.0)
    tb = b.t + offset_s
    lo, hi = max(a.t[0], tb[0]), min(a.t[-1], tb[-1])
    if not hi > lo:
        raise FrameError(f'the runs do not overlap in time (a {a.t[0]:.3f}..{a.t[-1]:.3f} s, '
                         f'b {tb[0]:.3f}..{tb[-1]:.3f} s on a\'s clock)')
    rate = hz or min((len(a.t) - 1) / (a.t[-1] - a.t[0]), (len(b.t) - 1) / (b.t[-1] - b.t[0]))
    grid = lo + np.arange(int(math.floor((hi - lo) * rate + 1e-9)) + 1) / rate
    a2 = resample(a, grid)
    b2 = resample(replace(b, t=tb), grid)
    return a2, replace(b2, clock=dict(a.clock), notes=[*b.notes, f'aligned_offset_s={offset_s:g}'])
