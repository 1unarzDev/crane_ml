"""Fast-tier plant backed by a plant card: ``PlantModel`` plugin for crane ``fastsim``.

The fast tier's tracker emits body-velocity targets; this plant turns them into thruster commands
with a per-axis feedforward + proportional velocity loop on the card's nominal model (the card's
``control`` view, a stand-in for the boat's velocity controller), allocates them through the
pseudo-inverse of B with the box limits, and steps the card family with ONE particle per plant
(posterior-informed domain randomisation: pass ``particle`` or ``seed``).

Vehicle spec: ``{ref: plantcard.marine.fastsim_plant:CardPlant, gain: .., card: <ref|path>,
particle: k | null, seed: 0, kp: [2, 2, 2]}``. ``params()`` records the card sha and the particle.
"""

import math

import numpy as np

try:
    from fastsim.plant import PlantModel
except ImportError:          # plantcard does not need fastsim; the plugin does
    class PlantModel:        # type: ignore[no-redef]
        def __init__(self, limits, pose=(0.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0)):
            self.lim, self.pose, self.vel = limits, tuple(pose), tuple(vel)

import plantcard
from plantcard.family import ParamBatch


class CardPlant(PlantModel):
    def __init__(self, limits, pose=(0.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0), card='roboboat/blastoise@HEAD',
                 particle=None, seed=None, kp=(2.0, 2.0, 2.0), substeps=2):
        super().__init__(limits, pose, vel)
        self.card = card if isinstance(card, plantcard.Card) else plantcard.load(card)
        fam = self.card.family
        if particle is None and seed is not None:
            particle = int(np.random.default_rng(seed).integers(self.card.particles.n))
        self.particle = particle
        self.p = self.card.nominal() if particle is None else self.card.particles.take([int(particle)])
        self.nom = self.card.nominal()
        self.fam, self.kp, self.substeps = fam, np.asarray(kp, float), int(substeps)
        self.Bpinv = np.linalg.pinv(fam.B)
        x, y, yaw = self.pose
        self.x = np.array([[x, y, yaw, *self.vel]], float)
        self.queue = []          # (time due, q) for the actuation delay
        self.t = 0.0

    def _command(self, target):
        nom = {k: float(v[0]) for k, v in self.nom.items()}
        nu = self.x[0, 3:6]
        tau = np.empty(3)
        for j, (k, m) in enumerate(zip('uvr', ('m11', 'm22', 'm66'))):
            d = (nom['d1_' + k] + nom['d2_' + k] * abs(target[j])) * target[j]
            tau[j] = d + self.kp[j] * nom[m] * (target[j] - nu[j])
        f = self.Bpinv @ tau
        rho = nom['rev_ratio']
        kf = 2 * self.fam.k_ref / (1 + rho)
        cap = np.where(f >= 0, kf, rho * kf)
        scale = max(1.0, float(np.max(np.abs(f) / cap)))
        f = f / scale
        return np.sign(f) * np.sqrt(np.abs(f) / cap)          # q with f = k q|q|

    def velocity(self, target, dt):
        q_now = self._command(np.asarray(target, float))
        delay = float(self.p['delay_s'][0])
        self.queue.append((self.t + delay, q_now))
        while len(self.queue) > 1 and self.queue[1][0] <= self.t + 1e-9:
            self.queue.pop(0)
        q = self.queue[0][1] if self.queue[0][0] <= self.t + 1e-9 else np.zeros_like(q_now)
        h = dt / self.substeps
        for _ in range(self.substeps):
            self.x = self.fam.step(self.x, q[None], self.p, h, None)
        self.t += dt
        return tuple(float(v) for v in self.x[0, 3:6])

    def step(self, target, dt):
        vel = self.velocity(target, dt)
        x0, y0 = self.pose[:2]
        self.vel = vel
        xn, yn, yaw = (float(v) for v in self.x[0, :3])
        self.pose = (xn, yn, math.atan2(math.sin(yaw), math.cos(yaw)))
        return (xn - x0, yn - y0)

    def params(self):
        return {'card': self.card.sha, 'card_status': self.card.status, 'particle': self.particle,
                **{k: float(v[0]) for k, v in ParamBatch(self.p).items()}}
