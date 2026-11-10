"""One batched boat plant for both the hidden boats and the twins.

State per env: (x, y, psi, u, v, r), world ENU / body FLU at the CG (the frames of crane's
``Tools/Characterization/plant.py``). The rigid body is the NLC study's ``nlc.sim.vessel.rhs``
(Fossen 3-DOF with added mass, Coriolis, linear+quadratic damping and current), integrated with
RK4 at ``dt``; the wind load is ``nlc.sim.vessel.wind_wrench``.

Input: one normalised ESC command q_i in [-1, 1] per thruster (PWM = 1500 + 400 q), canonical order
FL, FR, RR, RL. The thruster bank is

    q -> dead time (whole ticks) -> static map f* = k_sign * e^gamma, e = (|q| - db)/(1 - db)
      -> reversal (a sign change passes through zero and waits ``rev_hold`` s there)
      -> first-order lag (tau_up when |thrust| grows, tau_down when it shrinks) -> slew limit

so the twin family (gamma = 2, identical thrusters, tau_up = tau_down, no slew limit, uniform
current, no gusts) is a strict sub-family of the hidden boats. Hidden and twin share this code, so
"hidden = twin parameters" is the same plant exactly (the metamorphic test).

Random processes (current/gust OU paths, sensor noise) use counter-based draws keyed by
(env seed, tick, channel): a batch of N envs equals N single runs, and every controller sees the
same noise on the same (plant, repeat) (common random numbers).
"""

from dataclasses import dataclass, fields, replace

import numpy as np

import _paths  # noqa: F401  (sys.path + nlc pin)
from nlc.sim.vessel import CurrentField, CurrentSpec, VesselParams, rhs as nlc_rhs, wind_wrench, wrap

DT = 0.02                  # s, plant tick (the Unity fixed step)
N_THR = 4
SNAP_FRAC = 1e-3           # |T| <= SNAP_FRAC * max k is zeroed each tick (read when a ThrusterState is built;
                           # 0.0 disables the snap; see parity/PARITY.md)
NAMES = ('front_left', 'front_right', 'rear_right', 'rear_left')
_M64 = np.uint64(0xFFFFFFFFFFFFFFFF)


# ---------------------------------------------------------------------------------------------
# counter-based randomness
# ---------------------------------------------------------------------------------------------

def _splitmix(x):
    x = (x + np.uint64(0x9E3779B97F4A7C15)) & _M64
    z = x
    z = ((z ^ (z >> np.uint64(30))) * np.uint64(0xBF58476D1CE4E5B9)) & _M64
    z = ((z ^ (z >> np.uint64(27))) * np.uint64(0x94D049BB133111EB)) & _M64
    return z ^ (z >> np.uint64(31))


def crn_normal(seeds, tick, channel, k=1):
    """Standard normals [N, k] that depend only on (seeds[i], tick, channel, j)."""
    with np.errstate(over='ignore'):
        s = np.asarray(seeds, np.uint64)[:, None]
        j = np.arange(2 * k, dtype=np.uint64)[None, :]
        key = _splitmix(s * np.uint64(0x100000001B3) ^ _splitmix(np.uint64(tick) * np.uint64(1315423911)
                                                                  + np.uint64(channel) * np.uint64(2654435761) + j))
    u = (key >> np.uint64(11)).astype(np.float64) * (1.0 / 9007199254740992.0)
    u1, u2 = np.maximum(u[:, :k], 1e-300), u[:, k:]
    return np.sqrt(-2.0 * np.log(u1)) * np.cos(2.0 * np.pi * u2)


# ---------------------------------------------------------------------------------------------
# parameters
# ---------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class PlantSpec:
    """Structure of arrays, one row per env. Shapes: [N] unless noted."""

    mass: np.ndarray
    izz: np.ndarray
    a11: np.ndarray
    a22: np.ndarray
    a26: np.ndarray
    a66: np.ndarray
    d1: np.ndarray            # [N, 3]
    d2: np.ndarray            # [N, 3]
    k_fwd: np.ndarray         # [N, 4] N at full command
    k_rev: np.ndarray         # [N, 4]
    gamma: np.ndarray
    deadband: np.ndarray      # [N, 4] in q
    dead_time: np.ndarray     # s
    tau_up: np.ndarray        # s
    tau_down: np.ndarray      # s
    rev_hold: np.ndarray      # s
    slew: np.ndarray          # N/s (inf = none)
    odom_latency: np.ndarray  # s, arrival - acquisition of every measurement
    pos_sd: np.ndarray        # m
    vel_sd: np.ndarray        # m/s
    yaw_sd: np.ndarray        # rad
    gyro_sd: np.ndarray       # rad/s
    current: np.ndarray       # [N, 2] m/s ENU mean current
    ou_sigma: np.ndarray      # m/s stationary std of the current's OU part
    spatial_amp: np.ndarray   # m/s of the divergence-free spatial field
    wind: np.ndarray          # [N, 2] m/s ENU
    gust_sigma: np.ndarray    # m/s

    @property
    def n(self):
        return len(self.mass)

    def take(self, idx):
        idx = np.asarray(idx)
        return PlantSpec(**{f.name: np.asarray(getattr(self, f.name))[idx] for f in fields(self)})

    def repeat(self, k):
        """Each env repeated k times consecutively (env i -> rows i*k .. i*k+k-1)."""
        return self.take(np.repeat(np.arange(self.n), k))

    def with_(self, **kw):
        return replace(self, **kw)

    def row(self, i):
        return {f.name: np.asarray(getattr(self, f.name))[i].tolist() for f in fields(self)}

    @classmethod
    def concat(cls, specs):
        return cls(**{f.name: np.concatenate([np.asarray(getattr(s, f.name)) for s in specs])
                      for f in fields(cls)})

    @classmethod
    def build(cls, n, **kw):
        """Broadcast scalars/rows to n envs; missing sensor/env fields default to zero."""
        shapes = {'d1': (3,), 'd2': (3,), 'k_fwd': (4,), 'k_rev': (4,), 'deadband': (4,), 'current': (2,),
                  'wind': (2,)}
        zero = {'pos_sd', 'vel_sd', 'yaw_sd', 'gyro_sd', 'ou_sigma', 'spatial_amp', 'gust_sigma', 'odom_latency',
                'current', 'wind', 'rev_hold', 'dead_time', 'tau_up', 'tau_down', 'deadband', 'a26'}
        out = {}
        for f in fields(cls):
            v = kw.get(f.name, 0.0 if f.name in zero else None)
            if v is None and f.name == 'slew':
                v = np.inf
            if v is None and f.name == 'gamma':
                v = 2.0
            if v is None:
                raise ValueError(f'PlantSpec.build: {f.name} is required')
            out[f.name] = np.broadcast_to(np.asarray(v, float), (n,) + shapes.get(f.name, ())).copy()
        return cls(**out)

    def vessel_params(self):
        return VesselParams(mass=self.mass, izz=self.izz, a11=self.a11, a22=self.a22, a26=self.a26, a66=self.a66,
                            d1=tuple(self.d1[:, j] for j in range(3)), d2=tuple(self.d2[:, j] for j in range(3)),
                            source='decisioncal.PlantSpec')


# ---------------------------------------------------------------------------------------------
# thrusters
# ---------------------------------------------------------------------------------------------

def static_thrust(q, k_fwd, k_rev, gamma, deadband):
    """f* [..., 4] from q [..., 4]: sign(q) k_sign e^gamma, e = max(|q| - db, 0) / (1 - db)."""
    q = np.clip(q, -1.0, 1.0)
    e = np.maximum(np.abs(q) - deadband, 0.0) / (1.0 - deadband)
    mag = e ** np.asarray(gamma)[..., None]
    return np.where(q >= 0, k_fwd * mag, -k_rev * mag) * (e > 0)


class ThrusterState:
    """Thrust T, the sign of the last thrust produced and the time spent at zero waiting to reverse,
    for any leading batch shape (envs, or segments x grid points in the fitter)."""

    def __init__(self, shape, k_scale, dt, tau_up, tau_down, rev_hold, slew):
        self.T = np.zeros(shape + (N_THR,))
        self.last = np.zeros(shape + (N_THR,))
        self.wait = np.zeros(shape + (N_THR,))
        self.eps = SNAP_FRAC * np.asarray(k_scale)[..., None]
        ex = lambda tau: np.where(np.asarray(tau) > 0, 1.0 - np.exp(-dt / np.maximum(tau, 1e-9)), 1.0)[..., None]
        self.a_up, self.a_down = ex(tau_up), ex(tau_down)
        self.hold = np.asarray(rev_hold, float)[..., None]
        self.dmax = (np.asarray(slew, float) * dt)[..., None]
        self.dt = dt

    def step(self, f_star):
        """One tick. The lag pulls T toward f*; a sign change stops at exactly zero, and thrust of
        the other sign than the last one produced may only build after T has sat at zero for
        ``rev_hold`` s (an ESC/propeller reversal); a stopped motor that has waited may start
        either way."""
        T0 = self.T
        want = np.sign(f_star)
        zero = T0 == 0
        self.wait = np.where(zero, self.wait + self.dt, 0.0)
        blocked = (want != 0) & (self.last != 0) & (want != self.last)
        held = blocked & zero & (self.wait < self.hold + 0.5 * self.dt)
        grow = (np.abs(f_star) > np.abs(T0)) & ((f_star * T0) >= 0)
        a = np.where(grow, self.a_up, self.a_down)
        T = T0 + np.clip(a * (f_star - T0), -self.dmax, self.dmax)
        crossed = (T0 != 0) & (np.sign(T) != np.sign(T0))
        T = np.where(crossed | held | (np.abs(T) <= self.eps), 0.0, T)
        self.T = T
        self.last = np.where(T != 0, np.sign(T), self.last)
        return T


class DelayLine:
    """Per-env dead time in whole ticks over a value of shape [N, k]."""

    def __init__(self, n, k, ticks):
        self.ticks = np.asarray(ticks, int)
        self.buf = np.zeros((int(self.ticks.max()) + 1, n, k))
        self.i = 0
        self.ar = np.arange(n)

    def push(self, v):
        self.i = (self.i + 1) % len(self.buf)
        self.buf[self.i] = v
        return self.buf[(self.i - self.ticks) % len(self.buf), self.ar]


def thruster_geometry():
    """B (3 x 4): body (X, Y, N) per unit thrust of each thruster, canonical FL, FR, RR, RL, from the
    characterization profile's mounts (identical to plantcard Fossen3.B and the nlc VesselParams)."""
    P = VesselParams()
    return P.B()


B = thruster_geometry()


# ---------------------------------------------------------------------------------------------
# environment
# ---------------------------------------------------------------------------------------------

class Environment:
    """Current c(p, t) = mean + OU(t) + spatial(p) and wind w(t) = mean + gust(t), per env.
    The OU parts are piecewise-linear between 0.5 s knots drawn with ``crn_normal``."""

    KNOT = 0.5

    def __init__(self, spec: PlantSpec, seeds, t_max):
        n = spec.n
        self.spec, self.seeds = spec, np.asarray(seeds, np.uint64)
        nk = int(np.ceil(t_max / self.KNOT)) + 2
        self.ou_c = self._ou(spec.ou_sigma, 20.0, nk, 101)
        self.ou_w = self._ou(spec.gust_sigma, 5.0, nk, 202)
        self.has_spatial = bool(np.any(spec.spatial_amp > 0))
        if self.has_spatial:
            rngs = [np.random.default_rng([int(s), 303]) for s in self.seeds]
            self.field = CurrentField(CurrentSpec(spatial_amp=1.0, spatial_scale=25.0, spatial_modes=6), rngs)
            self.field.A = self.field.A * spec.spatial_amp[:, None]
        self.n = n

    def _ou(self, sigma, tau, nk, ch):
        a = np.exp(-self.KNOT / tau)
        out = np.zeros((nk, len(self.seeds), 2))
        out[0] = np.asarray(sigma)[:, None] * crn_normal(self.seeds, 0, ch, 2)
        for k in range(1, nk):
            out[k] = a * out[k - 1] + np.asarray(sigma)[:, None] * np.sqrt(1 - a * a) * crn_normal(self.seeds, k, ch, 2)
        return out

    def _interp(self, path, t):
        f = t / self.KNOT
        k = min(int(f), len(path) - 2)
        w = f - k
        return (1 - w) * path[k] + w * path[k + 1]

    def current(self, p, t):
        c = self.spec.current + self._interp(self.ou_c, t)
        if self.has_spatial:
            c = c + self.field.spatial(p[:, None, :])[:, 0]
        return c

    def wind(self, t):
        return self.spec.wind + self._interp(self.ou_w, t)


# ---------------------------------------------------------------------------------------------
# sensors
# ---------------------------------------------------------------------------------------------

class Sensors:
    """Measurements of the true state, delayed by ``odom_latency`` (stamped on arrival, as a logged
    bag is) with white noise. ``read`` returns (pose [N,3], body velocity [N,3], gyro r [N])."""

    def __init__(self, spec: PlantSpec, seeds, dt=DT):
        self.spec, self.seeds = spec, np.asarray(seeds, np.uint64)
        self.lat = DelayLine(spec.n, 6, np.round(spec.odom_latency / dt))

    def push(self, st):
        return self.lat.push(st)

    def read(self, delayed, tick, channel=0):
        s = self.spec
        z = crn_normal(self.seeds, tick, 500 + channel, 7)
        pose = delayed[:, :3].copy()
        pose[:, :2] += s.pos_sd[:, None] * z[:, :2]
        pose[:, 2] = wrap(pose[:, 2] + s.yaw_sd * z[:, 2])
        vel = delayed[:, 3:6] + np.stack([s.vel_sd, s.vel_sd, s.gyro_sd], -1) * z[:, 3:6]
        gyro = delayed[:, 5] + s.gyro_sd * z[:, 6]
        return pose, vel, gyro


# ---------------------------------------------------------------------------------------------
# plant
# ---------------------------------------------------------------------------------------------

class Plant:
    """N boats stepped together. ``step(q)`` applies the command for one tick and advances."""

    def __init__(self, spec: PlantSpec, seeds, state0=None, t_max=180.0, dt=DT):
        self.spec, self.dt, self.n = spec, dt, spec.n
        self.seeds = np.asarray(seeds, np.uint64)
        if len(self.seeds) != self.n:
            raise ValueError('one seed per env')
        self.P = spec.vessel_params()
        self.st = np.zeros((self.n, 6)) if state0 is None else np.array(state0, float)
        self.cmd_delay = DelayLine(self.n, N_THR, np.round(spec.dead_time / dt))
        self.thr = ThrusterState((self.n,), np.maximum(spec.k_fwd, spec.k_rev).max(-1), dt, spec.tau_up,
                                 spec.tau_down, spec.rev_hold, spec.slew)
        self.env = Environment(spec, self.seeds, t_max + 1.0)
        self.sensors = Sensors(spec, self.seeds, dt)
        self.t, self.tick = 0.0, 0
        self.delayed = self.sensors.push(self.st)
        self.q_applied = np.zeros((self.n, N_THR))
        self.wrench = np.zeros((self.n, 3))

    def thrust_of(self, q):
        s = self.spec
        return static_thrust(q, s.k_fwd, s.k_rev, s.gamma, s.deadband)

    def step(self, q):
        q = np.clip(np.nan_to_num(np.asarray(q, float)), -1.0, 1.0)
        self.q_applied = qd = self.cmd_delay.push(q)
        T = self.thr.step(self.thrust_of(qd))
        tau = T @ B.T
        n, dt, st = self.n, self.dt, self.st
        cur = self.env.current(st[:, :2], self.t)
        tau = tau + wind_wrench(self.P, st, self.env.wind(self.t), n)     # air drag always (also in still air)
        self.wrench = tau
        k1 = nlc_rhs(self.P, st, tau, cur, n)
        k2 = nlc_rhs(self.P, st + 0.5 * dt * k1, tau, cur, n)
        k3 = nlc_rhs(self.P, st + 0.5 * dt * k2, tau, cur, n)
        k4 = nlc_rhs(self.P, st + dt * k3, tau, cur, n)
        st = st + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        st[:, 2] = wrap(st[:, 2])
        self.st = st
        self.t += dt
        self.tick += 1
        self.delayed = self.sensors.push(st)

    def measure(self, channel=0):
        return self.sensors.read(self.delayed, self.tick, channel)
