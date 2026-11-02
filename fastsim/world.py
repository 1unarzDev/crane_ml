"""The world in the controller's map frame (the start pose): element arrays and contact geometry.

The controller sees the start frame, as odometry starting at the start pose does in the Unity
sim; tracks are written back in the scenario's world frame.
"""

import math

import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class World:
    def __init__(self, scenario):
        self.scenario = scenario
        self.start = scenario.start
        elements = scenario.elements
        self.ids = [e.id for e in elements]
        self.labels = [e.label for e in elements]
        self.xy = self.to_map(np.array([(e.x, e.y) for e in elements], dtype=float).reshape(-1, 2))
        self.radius = np.array([e.radius for e in elements], dtype=float)
        self.visible = np.array([bool(e.label) for e in elements], dtype=bool)
        self.box = np.array([e.box is not None for e in elements], dtype=bool)
        self.box_dims = np.array([(e.box[0], e.box[1], e.box[2] - self.start[2]) if e.box else
                                  (0.0, 0.0, 0.0) for e in elements], dtype=float).reshape(-1, 3)
        self.boundary = self.to_map(np.array(scenario.boundary, dtype=float).reshape(-1, 2))

    def to_map(self, world):
        x0, y0, yaw0 = self.start
        c, s = math.cos(yaw0), math.sin(yaw0)
        d = np.asarray(world, dtype=float) - (x0, y0)
        return np.stack([c * d[..., 0] + s * d[..., 1], -s * d[..., 0] + c * d[..., 1]], axis=-1)

    def to_world(self, pose):
        x0, y0, yaw0 = self.start
        x, y, yaw = pose
        c, s = math.cos(yaw0), math.sin(yaw0)
        return (x0 + c * x - s * y, y0 + s * x + c * y, wrap(yaw + yaw0))


def to_polygon(points, poly):
    """Distance (m, negative inside) from each point to a convex polygon."""
    a, b = poly, np.roll(poly, -1, axis=0)
    ab = b - a
    ap = points[:, None, :] - a[None]
    t = np.clip((ap * ab).sum(-1) / (ab * ab).sum(-1), 0, 1)
    edge = np.hypot(*(ap - t[..., None] * ab).transpose(2, 0, 1)).min(1)
    cross = ab[None, :, 0] * ap[..., 1] - ab[None, :, 1] * ap[..., 0]
    inside = (cross >= 0).all(1) | (cross <= 0).all(1)
    return np.where(inside, -edge, edge)


def footprint_clearance(world, pose, footprint, reach):
    """(least clearance m, element index) between the footprint and the elements within ``reach``.

    Disks by centre distance to the polygon less their radius; boxes by the nearer of the
    footprint's corners in the box and the box's corners in the footprint. (inf, -1) when
    nothing is within reach.
    """
    x, y, yaw = pose
    near = np.flatnonzero(np.hypot(*(world.xy - (x, y)).T) <= world.radius + reach)
    if not len(near):
        return math.inf, -1
    c, s = math.cos(yaw), math.sin(yaw)
    poly = np.array([(x + c * px - s * py, y + s * px + c * py) for px, py in footprint])
    clearance = to_polygon(world.xy[near], poly) - world.radius[near]
    for k, i in enumerate(near):
        if world.box[i]:
            length, width, byaw = world.box_dims[i]
            cb, sb = math.cos(byaw), math.sin(byaw)
            box = world.xy[i] + np.array([(cb * u - sb * v, sb * u + cb * v) for u, v in (
                (length / 2, width / 2), (length / 2, -width / 2), (-length / 2, -width / 2),
                (-length / 2, width / 2))])
            clearance[k] = min(to_polygon(poly, box).min(), to_polygon(box, poly).min())
    k = int(clearance.argmin())
    return float(clearance[k]), int(near[k])


def contact_cause(world, i, goal, reach):
    """Why the vehicle touched element ``i`` (the no-planner blind spot; no decision).

    ``no_goal``: no live goal (braking or holding). ``goal_at_obstacle``: a route point the
    controller sent lies within the element's reach, which no planner avoids. Otherwise
    ``route_through_obstacle``: the straight leg ran into it, where a planner would go round.
    """
    if goal is None or goal.done:
        return 'no_goal'
    length, width, _ = world.box_dims[i]
    near = world.radius[i] + (math.hypot(length, width) / 2 if world.box[i] else 0) + reach
    points = np.array([p[:2] for p in goal.route], dtype=float).reshape(-1, 2)
    hit = (np.hypot(*(points - world.xy[i]).T) <= near).any()
    return 'goal_at_obstacle' if hit else 'route_through_obstacle'
