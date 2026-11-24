# SPDX-License-Identifier: GPL-3.0-or-later
"""The drones' T1 plant for fastsim: the rigid-body multirotor (engine/kernels/aerial.py) under the cascaded
controller (aerial_control.py), driven by the tracker's planar body-velocity targets at a held altitude.

    vehicle: {ref: domains.drones.plant:AerialPlant, gain: 1.5, profile: x500, dt_s: 0.004, altitude_m: 10,
              control: {kp_xy: ..., ...}}

Each core step the target (vx, vy along the body, wz) becomes a world velocity feed-forward and a yaw setpoint
(the yaw rate integrated, kept within 0.5 rad of the yaw), held for dt / dt_s plant steps (numba when it is
installed, else the numpy map; the same numbers). The pose is the rigid body's (x, y, yaw) in the map frame
(the run's start pose); vel is its body-frame planar velocity. Parameters come only from the card (``profile``,
with ``variant`` values replaced); the controller's gains from ``control``
(default: config/domains/drones.yaml ``control``, the same gains braking uses).
"""

import math

import numpy as np

from domains.drones import card as cards
from engine.kernels import aerial as A, aerial_control as C
from fastsim.plant import PlantModel
from fastsim.world import wrap


class AerialPlant(PlantModel):
    def __init__(self, limits, pose=(0.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0), profile=None, variant=None,
                 control=None, dt_s=0.004, altitude_m=10.0, base=None):
        super().__init__(limits, pose, vel)
        if not control:
            from domains.drones.braking import config
            control = config()['control']   # config/domains/drones.yaml control
        doc = cards.override(profile, variant) if variant else cards.read(profile)
        self.card_id, self.card_sha = doc.get('id'), cards.sha256(doc)
        self.p = A.pack(cards.vehicle(doc))
        self.c = C.pack(self.p, control)
        self.dt_s, self.alt, self.gains = float(dt_s), float(altitude_m), dict(control)
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        world_v = (c * vel[0] - s * vel[1], s * vel[0] + c * vel[1], 0.0)
        self.x = A.init(self.p[None], pos=(x, y, self.alt), vel=world_v, yaw=yaw,
                        rotor_speed=A.hover_speed(self.p))[0]
        self.yaw_sp, self.cut_steps = yaw, 0.0
        self.alt_dev = self.tilt_max = 0.0
        self.wind = np.zeros(3)
        try:
            import numba  # noqa: F401
            self.fast = True
        except ImportError:
            self.fast = False

    def ticks(self, dt):
        n = round(dt / self.dt_s)
        if n < 1 or abs(n * self.dt_s - dt) > 1e-9:
            raise ValueError(f'dt {dt} is not a multiple of the plant step dt_s {self.dt_s}')
        return n

    def _yaw(self):
        w, x, y, z = self.x[A.QUAT:A.QUAT + 4]
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def velocity(self, target, dt):
        self.step(target, dt)
        return self.vel

    def step(self, target, dt):
        n = self.ticks(dt)
        yaw = self._yaw()
        self.yaw_sp = yaw + max(-0.5, min(0.5, wrap(self.yaw_sp + target[2] * dt - yaw)))
        c, s = math.cos(yaw), math.sin(yaw)
        sp = np.array([self.x[0], self.x[1], self.alt, c * target[0] - s * target[1], s * target[0] + c * target[1],
                       self.yaw_sp, 0.0])
        x0, y0 = self.x[0], self.x[1]
        if self.fast:
            self.cut_steps += C.nb_fly(self.x, sp, self.p, self.c, self.dt_s, n, self.wind)
        else:
            x, cut = C.fly(self.x[None], sp[None], self.p[None], self.c[None], self.dt_s, n, self.wind[None])
            self.x, self.cut_steps = x[0], self.cut_steps + float(cut[0])
        yaw = self._yaw()
        c, s = math.cos(yaw), math.sin(yaw)
        vx, vy = self.x[A.VEL], self.x[A.VEL + 1]
        self.vel = (c * vx + s * vy, -s * vx + c * vy, float(self.x[A.RATE + 2]))
        self.pose = (float(self.x[0]), float(self.x[1]), yaw)
        alt, tilt, _ = self.state()
        self.alt_dev, self.tilt_max = max(self.alt_dev, abs(alt - self.alt)), max(self.tilt_max, tilt)
        return float(self.x[0] - x0), float(self.x[1] - y0)

    def params(self):
        return {'card': self.card_id, 'card_sha256': self.card_sha, 'dt_s': self.dt_s, 'altitude_m': self.alt,
                'control': self.gains, 'alt_dev_m': round(self.alt_dev, 4), 'max_tilt_deg': round(self.tilt_max, 3),
                'saturated_s': round(self.cut_steps * self.dt_s, 4)}

    def state(self):
        """altitude (m), tilt (deg), seconds the collective was cut."""
        q1, q2 = self.x[A.QUAT + 1], self.x[A.QUAT + 2]
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (q1 * q1 + q2 * q2)))))
        return float(self.x[2]), tilt, self.cut_steps * self.dt_s
