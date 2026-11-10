"""The boat's velocity law and mixer, batched over envs.

``BatchedLaw`` is ``vendor/velocity_law.VelocityLaw`` (feed-forward through the velocity gain, a
world-frame drift observer that learns while holding still, conditional-integration PI) with every
quantity carrying a leading env axis; ``mix`` is ``vendor/thruster_mixer.mix_thrusters``
(identity decoupling, desaturation that keeps the direction). ``tests/test_law_parity.py`` pins both
to the vendored scalar code. The velocity gain comes from the PRIOR card only: controllers never
see a twin or a hidden boat.
"""

import numpy as np

import _paths  # noqa: F401
from twin import NOMINAL, K_REF
from plant import B as B_GEOM
from vendor.thruster_mixer import THRUSTER_MIXER

MIX = np.asarray(THRUSTER_MIXER, float)                     # [4, 3]
LIMITS = np.array([1.5, 0.6, 0.8])                          # m/s, m/s, rad/s


def steady_velocity(effort_axis, e):
    """Steady body velocity on the prior card's nominal boat for effort e on one axis."""
    cmd = np.zeros(3)
    cmd[effort_axis] = e
    q = MIX @ cmd
    q = q / max(1.0, np.abs(q).max())
    f = K_REF * np.sign(q) * q * q
    F = (f @ B_GEOM.T)[effort_axis]
    d1 = NOMINAL[['d1_u', 'd1_v', 'd1_r'][effort_axis]]
    d2 = NOMINAL[['d2_u', 'd2_v', 'd2_r'][effort_axis]]
    s = np.sign(F)
    F = abs(F)
    return s * (-d1 + np.sqrt(d1 * d1 + 4 * d2 * F)) / (2 * d2)


def prior_gain():
    """Diagonal velocity gain (velocity per unit effort), least squares through the origin over
    efforts 0.1..1 on the prior card: the characterization procedure's straight-line gain."""
    es = np.linspace(0.1, 1.0, 10)
    g = [float(np.dot(es, [steady_velocity(j, e) for e in es]) / np.dot(es, es)) for j in range(3)]
    return np.diag(g)


GAIN = prior_gain()
GAIN_INV = np.linalg.inv(GAIN)


def mix(effort):
    """effort [N, 3] -> q [N, 4] (FL, FR, RR, RL), scaled together when any exceeds 1."""
    c = effort @ MIX.T
    peak = np.maximum(1.0, np.abs(c).max(-1, keepdims=True))
    return c / peak


class BatchedLaw:
    def __init__(self, n, kp=(0.8, 0.8, 0.6), ki=(0.6, 0.6, 0.4), integral_limit=(0.3, 0.3, 0.3),
                 drift_time_constant=5.0, odom_timeout=0.3, drift_command_limit=0.1, gain=GAIN, limits=LIMITS):
        self.n = n
        bc = lambda v: np.broadcast_to(np.asarray(v, float), (n, 3)).copy()
        self.kp, self.ki, self.ilim = bc(kp), bc(ki), bc(integral_limit)
        self.gain, self.gain_inv = np.asarray(gain, float), np.linalg.inv(gain)
        self.bias = np.zeros(3)
        self.limits = np.asarray(limits, float)
        self.dtc, self.timeout, self.dcl = drift_time_constant, odom_timeout, drift_command_limit
        self.drift = np.zeros((n, 2))
        self.velocity = np.zeros((n, 3))
        self.yaw = np.zeros(n)
        self.odom_time = np.full(n, np.nan)
        self.integral = np.zeros((n, 3))
        self.effort = np.zeros((n, 3))
        self.target = np.zeros((n, 3))
        self.last_step = np.full(n, np.nan)

    def reset(self, mask):
        for a in (self.integral, self.effort, self.target):
            a[mask] = 0.0
        self.last_step[mask] = np.nan

    def observe(self, velocity, yaw, now, mask=None):
        mask = np.ones(self.n, bool) if mask is None else mask
        prev = self.odom_time
        dt = np.where(np.isnan(prev), 0.0, np.clip(now - np.nan_to_num(prev), 0.0, 0.2))
        self.velocity = np.where(mask[:, None], velocity, self.velocity)
        self.yaw = np.where(mask, yaw, self.yaw)
        self.odom_time = np.where(mask, now, self.odom_time)
        under_way = np.any(np.abs(self.target) > self.dcl, axis=1)
        saturated = np.any(np.abs(self.effort) >= 0.999, axis=1)
        learn = mask & (dt > 0) & ~under_way & ~saturated & (self.dtc > 0)
        expected = self.bias + self.effort @ self.gain.T
        res = (self.velocity - expected)[:, :2]
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        resw = np.stack([c * res[:, 0] - s * res[:, 1], s * res[:, 0] + c * res[:, 1]], -1)
        alpha = (dt / (self.dtc + dt + 1e-300))[:, None]
        self.drift = np.where(learn[:, None], self.drift + alpha * (resw - self.drift), self.drift)

    def stale(self, now):
        return np.isnan(self.odom_time) | (now - np.nan_to_num(self.odom_time) > self.timeout)

    def step(self, desired, now, feedforward=None, feedback=True):
        """desired body velocity [N, 3] -> effort [N, 3]. ``feedforward`` [N] bool (default all)."""
        ff = np.ones(self.n, bool) if feedforward is None else np.asarray(feedforward, bool)
        stale = self.stale(now)
        dt = np.where(np.isnan(self.last_step), 0.0, np.clip(now - np.nan_to_num(self.last_step), 0.0, 0.2))
        self.last_step = np.full(self.n, now)
        target = np.clip(desired, -self.limits, self.limits)
        self.target = target
        c, s = np.cos(-self.yaw), np.sin(-self.yaw)
        drift_body = np.stack([c * self.drift[:, 0] - s * self.drift[:, 1], s * self.drift[:, 0] + c * self.drift[:, 1],
                               np.zeros(self.n)], -1)
        forward = (target - self.bias - drift_body) @ self.gain_inv.T
        forward = np.where(ff[:, None], forward, 0.0)
        error = target - self.velocity
        if feedback:
            unclipped = forward + self.kp * error + self.ki * self.integral
            pinned = ((unclipped >= 1.0) & (error > 0)) | ((unclipped <= -1.0) & (error < 0))
            self.integral = np.where(pinned, self.integral,
                                     np.clip(self.integral + error * dt[:, None], -self.ilim, self.ilim))
            effort = forward + self.kp * error + self.ki * self.integral
        else:
            effort = forward
        effort = np.where(np.isfinite(effort), np.clip(effort, -1.0, 1.0), 0.0)
        self.effort = effort
        if np.any(stale):
            self.reset(stale)
            effort = np.where(stale[:, None], 0.0, effort)
        return effort
