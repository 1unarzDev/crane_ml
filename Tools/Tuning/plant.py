"""3-DOF RoboBoat plant with every knob the Nav2 tuning study varies. ROS-free, numpy only.

Evidence label: FIXTURE. Nominal values copy crane_ml's RoboBoatPhysics.cs @ bf82744 (30 kg,
Izz 4.1, T200 51.5 / 40.2 N, thrust ~ e|e|, spin-up 0.1 s, hull drag Surge (10, 19.4), Sway
(15, 41.7), Yaw (10, 19.3), no added mass) and the thruster geometry of mhseals_nav's
blastoise.xacro (|x| 0.266, |y| 0.343, 45 deg mounts; yaw lever 0.43 m). Nothing here is a
measurement of the real boat.

Model (sim2real.md §3.1): body FLU, origin at the CoM, water-relative velocity
nu_r = nu - [R^T c; 0] for a constant world current c:

    M nu_r' + C(nu_r) nu_r + D(nu_r) nu_r = tau_thr + tau_env
    M = diag(m + a_u, m + a_v, Iz + a_r)
    C(nu_r) = [[0, 0, -M22 v_r], [0, 0, M11 u_r], [M22 v_r, -M11 u_r, 0]]

The rigid body may integrate in nu_r because C_RB does not depend on linear velocity
(sim2real §3.1, the identity used by the sim mapping).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

import numpy as np

# Geometric thruster positions in base_link (FL, FR, RR, RL) and +effort thrust axes, from
# blastoise.xacro (thrust along the thruster link's -z; matches the mixer's rows).
POS = np.array([[0.266, 0.343], [0.266, -0.343], [-0.266, -0.343], [-0.266, 0.343]])
AXIS_DEG = np.array([135.0, -135.0, -45.0, 45.0])


@dataclass(frozen=True)
class Variant:
    """One plant. Defaults are the crane_ml plant (the sim the stack was characterized on)."""
    name: str = 'nominal'
    mass: float = 30.0
    izz: float = 4.1
    added: tuple = (0.0, 0.0, 0.0)          # a_u, a_v (kg), a_r (kg m^2)
    com: tuple = (0.0, 0.0)                 # CoM offset from base_link (bow, port), m
    drag_u: tuple = (10.0, 19.4)            # (linear, quadratic)
    drag_v: tuple = (15.0, 41.7)
    drag_r: tuple = (10.0, 19.3)
    fwd: tuple = (51.5,) * 4                # N at full effort, FL FR RR RL
    rev: tuple = (40.2,) * 4                # N at full reverse
    mount_err_deg: tuple = (0.0,) * 4       # mount yaw error, CCW
    deadband: float = 0.0                   # ESC deadband as a fraction of full effort (real 0.0625)
    spinup: float = 0.1                     # s, first-order thrust lag
    dead_time: float = 0.0                  # s, command to thrust
    sag: float = 1.0                        # thrust multiplier from pack voltage
    # Environment
    current: tuple = (0.0, 0.0)             # world m/s
    wind: tuple = (0.0, 0.0)                # world m/s (air), drag on 0.30 / 0.40 m^2 front / side
    gust: float = 0.25                      # gust SD / mean speed
    wave_sd: float = 0.0                    # m/s SD of wave-induced body velocity (surge, sway)
    wave_period: float = 2.0                # s
    # Sensing (what the velocity controller and the nav layer see)
    vel_sd: float = 0.0                     # m/s white noise per odometry sample (30 Hz)
    yaw_rate_sd: float = 0.0                # rad/s
    odom_latency: float = 0.0               # s
    heading_bias: float = 0.0               # rad, EKF heading minus truth
    pos_sd: float = 0.0                     # m, GPS position noise (AR(1), 2 s correlation)
    pos_tau: float = 2.0


def variant(**changes):
    return replace(Variant(), **changes)


def thrust_columns(v: Variant):
    """Unit-force wrench columns (Fx, Fy, N) about the CoM, one per thruster."""
    a = np.radians(AXIS_DEG + np.array(v.mount_err_deg))
    rel = POS - np.array(v.com)
    return np.stack([np.cos(a), np.sin(a), rel[:, 0] * np.sin(a) - rel[:, 1] * np.cos(a)])


def static_thrust(v: Variant, effort):
    """Thrust (N) per thruster for efforts in [-1, 1]: e|e| outside the ESC deadband."""
    e = np.clip(np.asarray(effort, float), -1, 1)
    mag = np.clip((np.abs(e) - v.deadband) / (1 - v.deadband), 0, 1)
    gain = np.where(e >= 0, np.array(v.fwd), np.array(v.rev))
    return np.sign(e) * mag * mag * gain * v.sag


def quad(x, c):
    return (c[0] + c[1] * abs(x)) * x


class Boat:
    """Integrates the plant at dt; ``step(efforts)`` takes the four thruster efforts."""

    def __init__(self, v: Variant, seed=0, dt=0.005):
        self.v, self.dt = v, dt
        self.rng = np.random.default_rng(seed)
        self.M = np.array([v.mass + v.added[0], v.mass + v.added[1], v.izz + v.added[2]])
        self.B = thrust_columns(v)
        self.eta = np.zeros(3)          # x, y, psi (world, CoM)
        self.nu_r = np.zeros(3)         # water-relative body velocity
        self.f = np.zeros(4)            # realized thrust
        self.queue = []                 # (time, efforts) for the dead time
        self.t = 0.0
        self.gust = 0.0
        self.wave_phase = self.rng.uniform(0, 2 * math.pi, 3)

    def R(self, psi=None):
        psi = self.eta[2] if psi is None else psi
        c, s = math.cos(psi), math.sin(psi)
        return np.array([[c, -s], [s, c]])

    @property
    def nu(self):
        """Ground velocity in the body frame at the CoM."""
        cur = self.R().T @ np.array(self.v.current)
        return self.nu_r + np.array([cur[0], cur[1], 0.0])

    def base_velocity(self):
        """Body velocity of base_link (offset -com from the CoM)."""
        u, w, r = self.nu
        dx, dy = -self.v.com[0], -self.v.com[1]
        return np.array([u - r * dy, w + r * dx, r])

    def base_pose(self):
        dx, dy = self.R() @ (-np.array(self.v.com))
        return np.array([self.eta[0] + dx, self.eta[1] + dy, self.eta[2]])

    def env_force(self):
        v = self.v
        tau = np.zeros(3)
        if any(v.wind):
            dt = self.dt
            self.gust += -self.gust * dt / 8.0 + v.gust * math.sqrt(2 * dt / 8.0) * self.rng.standard_normal()
            air = np.array(v.wind) * max(0.0, 1 + self.gust)
            ground = self.R() @ self.nu[:2]
            rel = self.R().T @ (air - ground)
            speed = np.hypot(*rel)
            if speed > 1e-4:
                beta = math.atan2(rel[1], rel[0])
                area = 0.30 * abs(math.cos(beta)) + 0.40 * abs(math.sin(beta))
                fw = 0.5 * 1.225 * area * speed * rel
                tau += [fw[0], fw[1], 0.05 * fw[1]]  # pressure centre 5 cm ahead (RoboBoatWind)
        if v.wave_sd > 0:
            w = 2 * math.pi / v.wave_period
            amp = self.M * w * v.wave_sd * math.sqrt(2) * np.array([1.0, 1.0, 0.3])
            tau += amp * np.sin(w * self.t + self.wave_phase)
        return tau

    def step(self, efforts, steps=1):
        v = self.v
        for _ in range(steps):
            self.queue.append((self.t + v.dead_time, np.array(efforts, float)))
            while len(self.queue) > 1 and self.queue[1][0] <= self.t + 1e-9:
                self.queue.pop(0)
            e = self.queue[0][1] if self.queue[0][0] <= self.t + 1e-9 else np.zeros(4)
            target = static_thrust(v, e)
            self.f += (target - self.f) * min(1.0, self.dt / max(v.spinup, self.dt))
            tau = self.B @ self.f + self.env_force()
            u, w, r = self.nu_r
            M11, M22, _ = self.M
            coriolis = np.array([-M22 * w * r, M11 * u * r, (M22 - M11) * u * w])
            # Munk sign: N = -(M22 - M11) u v (sim2real §3.1); C nu gives +(M22-M11)u v on the left.
            damping = np.array([quad(u, v.drag_u), quad(w, v.drag_v), quad(r, v.drag_r)])
            acc = (tau - coriolis - damping) / self.M
            self.nu_r = self.nu_r + acc * self.dt
            g = self.R() @ self.nu[:2]
            self.eta += np.array([g[0], g[1], self.nu[2]]) * self.dt
            self.t += self.dt
