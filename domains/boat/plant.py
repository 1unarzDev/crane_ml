"""The boat's T1 plant: a fastsim ``PlantModel`` that steps ``ProfilePlant``, the exact twin of the
C# ``MarinePlanarModel`` (symplectic Euler with implicit damping), with its PWM thrusters.

    vehicle: {ref: domains.boat.plant:ProfileBoat, gain: 1.0,
              profile: Tools/Characterization/profiles/characterized.json, kp: [2, 2, 2]}

Each core step takes the tracker's body-velocity target through a velocity smoother (the
``Limits`` accel/decel, as a navigation stack's smoother does), a feedforward + proportional
velocity loop on the profile's own damping and effective mass, a pseudo-inverse allocation over
the thruster columns with uniform desaturation, and the inverse thrust curve to PWM. The PWM is
held over ``dt / tick_s`` plant ticks (``tick_s`` defaults to the profile's ``fixedDeltaTime``),
each one ``ProfilePlant.step_pwm``, as Unity holds a command over its FixedUpdate ticks.

The plant's map frame is the run's start pose; ``ProfilePlant``'s state (world velocity,
unwrapped heading) is kept as is, so its numbers are the twin's numbers.
"""

import hashlib
import json
import math
from pathlib import Path

import numpy as np

from fastsim import plugin
from fastsim.plant import PlantModel
from fastsim.world import wrap

ROOT = Path(__file__).resolve().parents[2]  # the crane checkout
CHARACTERIZATION = ROOT / 'Tools' / 'Characterization' / 'plant.py'
DEFAULT_PROFILE = 'Tools/Characterization/profiles/characterized.json'


def characterization(name):
    """An attribute of Tools/Characterization/plant.py (a flat module, loaded by path)."""
    return plugin.load(f'{CHARACTERIZATION}:{name}')


def profile_path(profile):
    path = Path(profile).expanduser()
    return path if path.is_absolute() else ROOT / path


class ProfileBoat(PlantModel):
    def __init__(self, limits, pose=(0.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0), profile=DEFAULT_PROFILE,
                 kp=(2.0, 2.0, 2.0), tick_s=None, current=(0.0, 0.0), smooth=True):
        super().__init__(limits, pose, vel)
        self.path = profile_path(profile)
        raw = self.path.read_bytes()
        self.profile, self.sha256 = json.loads(raw), hashlib.sha256(raw).hexdigest()
        self.tick_s = float(tick_s or self.profile['fixedDeltaTime'])
        self.twin = characterization('ProfilePlant')(self.profile, self.tick_s, current)
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        self.twin.reset((x, y, yaw, c * vel[0] - s * vel[1], s * vel[0] + c * vel[1], vel[2]))
        self.kp, self.smooth, self.current = tuple(float(k) for k in kp), bool(smooth), tuple(current)
        self.cmd = tuple(float(v) for v in vel)  # the smoothed velocity command
        p, am = self.profile, self.profile['addedMass']
        self.meff = (p['mass'] + am['a11'], p['mass'] + am['a22'], p['izz'] + am['a66'])
        self.damp = tuple(p[a] for a in ('surge', 'sway', 'yaw'))
        cols = []
        for t in p['thrusters']:
            a = math.radians(t['alphaDeg'])
            cols.append((math.cos(a), math.sin(a), t['x'] * math.sin(a) - t['y'] * math.cos(a)))
        self.Bpinv = np.linalg.pinv(np.array(cols).T)  # thrust per thruster from a body wrench
        self.thrusters = p['thrusters']
        self.pwm = [1500.0] * len(self.thrusters)

    # ---- the stand-in velocity controller -------------------------------------------------

    def smoothed(self, target, dt):
        if not self.smooth:
            return tuple(target)
        out = []
        for v, want, up, down in zip(self.cmd, target, self.lim.accel, self.lim.decel):
            rate = up if abs(want) > abs(v) and want * v >= 0 else -down
            out.append(v + max(-rate * dt, min(rate * dt, want - v)))
        return tuple(out)

    def pwm_for(self, target, dt):
        """The four pulses (us) that drive the body velocity toward ``target`` this step."""
        self.cmd = want = self.smoothed(target, dt)
        nu = self.twin.body_velocity()
        tau = []
        for j in range(3):
            c, w = self.damp[j], want[j]
            d1, d2 = (c['d1Pos'], c['d2Pos']) if w >= 0 else (c['d1Neg'], c['d2Neg'])
            tau.append((d1 + d2 * abs(w)) * w + self.kp[j] * self.meff[j] * (w - nu[j]))
        f = self.Bpinv @ np.array(tau)
        caps = [t['kForward'] if fi >= 0 else t['kReverse'] for t, fi in zip(self.thrusters, f)]
        scale = max(1.0, max(abs(fi) / cap for fi, cap in zip(f, caps)))
        pwm = []
        for t, fi, cap in zip(self.thrusters, f, caps):
            q = math.copysign(math.sqrt(abs(fi) / scale / cap), fi)  # thrust = k q|q|
            pwm.append(1500.0 + t['offsetUs'] + 400.0 * q)
        return pwm

    # ---- the plant ---------------------------------------------------------------------------

    def ticks(self, dt):
        n = round(dt / self.tick_s)
        if n < 1 or abs(n * self.tick_s - dt) > 1e-9:
            raise ValueError(f'dt {dt} is not a multiple of the plant tick {self.tick_s}')
        return n

    def apply_pwm(self, pwm, dt):
        """Hold ``pwm`` for one core step of ``dt``: ProfilePlant ticks, nothing else."""
        self.pwm = list(pwm)
        for _ in range(self.ticks(dt)):
            self.twin.step_pwm(self.pwm)
        x, y, psi = self.twin.state[:3]
        self.pose = (x, y, wrap(psi))
        self.vel = self.twin.body_velocity()
        return self.vel

    def velocity(self, target, dt):
        return self.apply_pwm(self.pwm_for(target, dt), dt)

    def step(self, target, dt):
        x0, y0 = self.twin.state[:2]
        self.velocity(target, dt)
        return (self.twin.state[0] - x0, self.twin.state[1] - y0)

    def params(self):
        try:
            name = str(self.path.relative_to(ROOT))
        except ValueError:
            name = str(self.path)
        return {'profile': name, 'profile_sha256': self.sha256, 'tick_s': self.tick_s,
                'kp': list(self.kp), 'current': list(self.current), 'smooth': self.smooth}
