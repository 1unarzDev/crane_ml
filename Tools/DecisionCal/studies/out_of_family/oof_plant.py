"""Out-of-family mechanisms on top of ``plant.Plant`` (no edits to plant.py).

``MechPlant(spec, mech, seeds)`` is a ``Plant`` whose rows additionally carry a ``MechSpec`` row. Seven
switchable mechanisms, none of which the twin family (rigid body + lumped delay + one lag + reversal hold +
rev ratio + deadband) can represent exactly:

  link       command link at 25 Hz with exponential per-frame jitter, i.i.d. frame drops and Gilbert-type
             outages (bursts of dropped frames); the actuator applies the newest frame that has arrived
             ("latest wins", no reordering) and holds it otherwise.
  prophull   propeller-hull interaction: thrust loss with advance speed through the water (Fossen bilinear
             thrust law, loss fraction ~ J ~ V_a / n, so larger at low RPM) and a sway-induced hull lift
             force and yaw moment (Y = -y_uv |u| v, N = n_uv u v).
  battery    battery voltage sag under load (ohmic + one polarization state), thrust ~ V^1.25.
  waves      first-order wave sway force and yaw moment (three narrow-band sinusoids, random phases) and a
             roll-coupled heading-sensor wobble.
  thrlag     thrust-level-dependent thruster time constant (slower at low thrust, tau(0)/tau(full) = R) and
             a reverse-direction multiplier.
  estimator  GPS-rate state estimate: position and world-frame (Doppler-like) velocity sampled at 5 Hz
             with a random receiver phase and held, velocity first-order low-passed (frequency-dependent
             group delay), rotated into the body with the current measured heading.
  multipath  GPS multipath steps: position offsets of a fixed size, random direction, exponential duration,
             Poisson onsets.

Guarantees (tested in test_oof_plant.py):
* a row whose mechanisms are all off evolves and measures bit-for-bit like ``plant.Plant``;
* a batch of N rows equals N single-row runs (per-row state; randomness from counter-based draws keyed by
  (env seed, tick/frame/knot, channel), channels 701-760, unused by plant.py);
* same env seed => same mechanism randomness for every controller (common random numbers).
"""

import sys
from dataclasses import dataclass, fields, replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import _paths  # noqa: E402,F401
from plant import B, DT, N_THR, Plant, ThrusterState, _splitmix  # noqa: E402
from nlc.sim.vessel import rhs as nlc_rhs, wind_wrench, wrap  # noqa: E402

MECHS = ('link', 'prophull', 'battery', 'waves', 'thrlag', 'estimator', 'multipath')
LINK_PERIOD = 2               # ticks: 25 Hz command frames (the controller's rate)
LINK_MAX_EXTRA = 25           # ticks: jitter clipped at 0.5 s
MP_KNOT = 0.5                 # s, multipath onset grid

# Upper ends of the plausible ranges (level lambda = 1). README.md justifies every number.
HI = {
    'link': {'jitter_mean': 0.030, 'drop_p': 0.05, 'outage_rate': 0.10, 'outage_mean': 0.30},
    'prophull': {'adv_loss': 0.25, 'y_uv': 6.0, 'n_uv': 3.0},
    'battery': {'sag_full': 0.12, 'sag_tau': 20.0},
    'waves': {'wave_Y': 12.0, 'wave_N': 1.5, 'head_wobble': np.radians(2.0)},
    'thrlag': {'lag_low_ratio': 3.0, 'lag_rev_ratio': 1.6},
    'estimator': {'gps_period': 0.2, 'vel_tau': 0.30},
    'multipath': {'mp_rate': 1.0 / 60.0, 'mp_size': 0.6, 'mp_dur': 3.0},
}


# ---------------------------------------------------------------------------------------------
# parameters
# ---------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class MechSpec:
    """Structure of arrays, one row per env, shape [N]. All zeros (ratios 1) = every mechanism off."""

    jitter_mean: np.ndarray     # s, mean of the exponential extra transport delay per frame
    drop_p: np.ndarray          # i.i.d. frame drop probability
    outage_rate: np.ndarray     # 1/s, onset rate of link outages
    outage_mean: np.ndarray     # s, mean outage duration
    adv_loss: np.ndarray        # fractional thrust loss per 1 m/s advance speed at full thrust
    y_uv: np.ndarray            # N/(m/s)^2, hull lift sway force  Y = -y_uv |u| v
    n_uv: np.ndarray            # N m/(m/s)^2, hull lift yaw moment N = n_uv u v (signed)
    sag_full: np.ndarray        # voltage sag fraction with all four thrusters at full thrust
    sag_tau: np.ndarray         # s, polarization time constant (30% of the sag)
    wave_Y: np.ndarray          # N, sway force amplitude
    wave_N: np.ndarray          # N m, yaw moment amplitude
    wave_period: np.ndarray     # s, peak period
    head_wobble: np.ndarray     # rad, heading-sensor wobble amplitude
    lag_low_ratio: np.ndarray   # tau(zero thrust) / tau(full thrust), >= 1 (1 = off)
    lag_rev_ratio: np.ndarray   # tau multiplier while thrust is reverse (1 = off)
    gps_period: np.ndarray      # s, estimator sample period (0 = estimator off)
    vel_tau: np.ndarray         # s, velocity low-pass time constant
    mp_rate: np.ndarray         # 1/s multipath onsets
    mp_size: np.ndarray         # m
    mp_dur: np.ndarray          # s, mean duration

    @property
    def n(self):
        return len(self.jitter_mean)

    @classmethod
    def off(cls, n):
        ones = {'lag_low_ratio', 'lag_rev_ratio'}
        return cls(**{f.name: np.full(n, 1.0 if f.name in ones else 0.0) for f in fields(cls)})

    @classmethod
    def build(cls, n, **kw):
        m = cls.off(n)
        return replace(m, **{k: np.broadcast_to(np.asarray(v, float), (n,)).copy() for k, v in kw.items()})

    def take(self, idx):
        idx = np.asarray(idx)
        return MechSpec(**{f.name: np.asarray(getattr(self, f.name))[idx] for f in fields(self)})

    def repeat(self, k):
        return self.take(np.repeat(np.arange(self.n), k))

    @classmethod
    def concat(cls, specs):
        return cls(**{f.name: np.concatenate([np.asarray(getattr(s, f.name)) for s in specs]) for f in fields(cls)})

    def with_(self, **kw):
        return replace(self, **kw)

    def row(self, i):
        return {f.name: float(np.asarray(getattr(self, f.name))[i]) for f in fields(self)}

    # on-masks ---------------------------------------------------------------------------------
    def on(self, mech):
        if mech == 'link':
            return (self.jitter_mean > 0) | (self.drop_p > 0) | (self.outage_rate > 0)
        if mech == 'prophull':
            return (self.adv_loss != 0) | (self.y_uv != 0) | (self.n_uv != 0)
        if mech == 'battery':
            return self.sag_full > 0
        if mech == 'waves':
            return (self.wave_Y != 0) | (self.wave_N != 0) | (self.head_wobble != 0)
        if mech == 'thrlag':
            return (self.lag_low_ratio != 1) | (self.lag_rev_ratio != 1)
        if mech == 'estimator':
            return self.gps_period > 0
        if mech == 'multipath':
            return (self.mp_rate > 0) & (self.mp_size > 0)
        raise ValueError(mech)


def at_level(n, lam, mechs, seed):
    """MechSpec of n boats with every mechanism in ``mechs`` at level ``lam`` in [0, 1] of its plausible
    upper end (magnitudes = lam * HI; discrete parts on whenever lam > 0). Per-boat structural draws (wave
    period, sign of the lift moment) come from ``seed``. ``mechs`` may be 'all'."""
    mechs = MECHS if mechs in ('all', ('all',)) else tuple(mechs)
    for m in mechs:
        if m not in MECHS:
            raise ValueError(f'unknown mechanism {m!r}')
    rng = np.random.default_rng([int(seed), 7171])
    period = rng.uniform(1.5, 3.0, n)
    sign = rng.choice([-1.0, 1.0], n)
    kw = {}
    if lam <= 0:
        return MechSpec.off(n)
    if 'link' in mechs:
        h = HI['link']
        kw.update(jitter_mean=lam * h['jitter_mean'], drop_p=lam * h['drop_p'], outage_rate=lam * h['outage_rate'],
                  outage_mean=h['outage_mean'])
    if 'prophull' in mechs:
        h = HI['prophull']
        kw.update(adv_loss=lam * h['adv_loss'], y_uv=lam * h['y_uv'], n_uv=lam * h['n_uv'] * sign)
    if 'battery' in mechs:
        kw.update(sag_full=lam * HI['battery']['sag_full'], sag_tau=HI['battery']['sag_tau'])
    if 'waves' in mechs:
        h = HI['waves']
        kw.update(wave_Y=lam * h['wave_Y'], wave_N=lam * h['wave_N'], head_wobble=lam * h['head_wobble'],
                  wave_period=period)
    if 'thrlag' in mechs:
        h = HI['thrlag']
        kw.update(lag_low_ratio=1 + lam * (h['lag_low_ratio'] - 1), lag_rev_ratio=1 + lam * (h['lag_rev_ratio'] - 1))
    if 'estimator' in mechs:
        kw.update(gps_period=HI['estimator']['gps_period'], vel_tau=lam * HI['estimator']['vel_tau'])
    if 'multipath' in mechs:
        h = HI['multipath']
        kw.update(mp_rate=h['mp_rate'], mp_size=lam * h['mp_size'], mp_dur=h['mp_dur'])
    return MechSpec.build(n, **kw)


# ---------------------------------------------------------------------------------------------
# counter-based uniforms (same keying as plant.crn_normal)
# ---------------------------------------------------------------------------------------------

def crn_uniform(seeds, tick, channel, k=1):
    """Uniforms [N, k] in [0, 1) that depend only on (seeds[i], tick, channel, j)."""
    with np.errstate(over='ignore'):
        s = np.asarray(seeds, np.uint64)[:, None]
        j = np.arange(k, dtype=np.uint64)[None, :]
        key = _splitmix(s * np.uint64(0x100000001B3) ^ _splitmix(np.uint64(tick) * np.uint64(1315423911)
                                                                  + np.uint64(channel) * np.uint64(2654435761) + j))
    return (key >> np.uint64(11)).astype(np.float64) * (1.0 / 9007199254740992.0)


# ---------------------------------------------------------------------------------------------
# mechanism blocks
# ---------------------------------------------------------------------------------------------

class CommandLink:
    """25 Hz command frames with jitter, drops and outages; newest arrived frame wins.

    A frame is sent on every tick with tick % LINK_PERIOD == 0 (it carries the q passed to that step) and
    arrives after D + J ticks, D = the plant's dead time in ticks, J = round(Exp(jitter_mean) / dt) clipped at
    LINK_MAX_EXTRA. With zero jitter and no drops a command held for LINK_PERIOD ticks (as mission.run holds
    it) is applied exactly as by ``plant.DelayLine``. ``applied_idx`` is the send tick of the frame in force."""

    def __init__(self, n, dead_ticks, mech: MechSpec, seeds, dt=DT, record=False):
        self.D = np.asarray(dead_ticks, int)
        self.W = int(self.D.max()) + LINK_MAX_EXTRA + 2
        self.q = np.zeros((self.W, n, N_THR))
        self.sent = np.full((self.W, n), -1, np.int64)
        self.arrive = np.full((self.W, n), np.iinfo(np.int64).max, np.int64)
        self.applied = np.zeros((n, N_THR))
        self.applied_idx = np.full(n, -1, np.int64)
        self.bad = np.zeros(n, bool)
        self.seeds = np.asarray(seeds, np.uint64)
        per = LINK_PERIOD * dt
        self.p_gb = np.clip(mech.outage_rate * per, 0, 1)
        self.p_bg = np.clip(per / np.maximum(mech.outage_mean, per), 0, 1)
        self.drop_p, self.jit = mech.drop_p, mech.jitter_mean / dt
        self.ar = np.arange(n)
        self.record = record
        self.log_idx, self.log_dropped = [], []

    def push(self, q, tick):
        if tick % LINK_PERIOD == 0:
            u = crn_uniform(self.seeds, tick // LINK_PERIOD, 701, 3)
            self.bad = np.where(self.bad, u[:, 0] >= self.p_bg, u[:, 0] < self.p_gb)
            drop = self.bad | (u[:, 1] < self.drop_p)
            extra = np.minimum(np.round(-self.jit * np.log1p(-u[:, 2])), LINK_MAX_EXTRA).astype(np.int64)
            slot = (tick // LINK_PERIOD) % self.W
            self.q[slot] = q
            self.sent[slot] = tick
            self.arrive[slot] = np.where(drop, np.iinfo(np.int64).max, tick + self.D + extra)
            if self.record:
                self.log_dropped.append(drop.copy())
        ok = (self.arrive <= tick) & (self.sent > self.applied_idx[None, :])
        has = ok.any(0)
        if has.any():
            best = np.argmax(np.where(ok, self.sent, -1), axis=0)
            self.applied = np.where(has[:, None], self.q[best, self.ar], self.applied)
            self.applied_idx = np.where(has, self.sent[best, self.ar], self.applied_idx)
        if self.record:
            self.log_idx.append(self.applied_idx.copy())
        return self.applied.copy()


class MechThrusterState(ThrusterState):
    """ThrusterState whose lag depends on the thrust level and direction on rows with thrlag on:
    tau = tau_base * (1 + (R - 1) (1 - sqrt(|T| / k))) * (rho if reverse), tau_base = tau_up when growing,
    tau_down when shrinking (as the base). Off rows use the base coefficients unchanged."""

    def __init__(self, shape, k_scale, dt, tau_up, tau_down, rev_hold, slew, mech: MechSpec, k_fwd, k_rev):
        super().__init__(shape, k_scale, dt, tau_up, tau_down, rev_hold, slew)
        self.on = np.asarray(mech.on('thrlag'))
        self.any = bool(self.on.any())
        self.R = mech.lag_low_ratio[:, None]
        self.rho = mech.lag_rev_ratio[:, None]
        self.tu = np.asarray(tau_up, float)[:, None]
        self.td = np.asarray(tau_down, float)[:, None]
        self.kf, self.kr = np.asarray(k_fwd, float), np.asarray(k_rev, float)
        self.base_up, self.base_down = self.a_up, self.a_down

    def step(self, f_star):
        if not self.any:
            return super().step(f_star)
        T0 = self.T
        k = np.where(T0 >= 0, self.kf, self.kr)
        op = np.clip(np.abs(T0) / k, 0.0, 1.0)
        fac = (1.0 + (self.R - 1.0) * (1.0 - np.sqrt(op))) * np.where((T0 < 0) | (f_star < 0), self.rho, 1.0)
        ex = lambda tau: np.where(tau > 0, 1.0 - np.exp(-self.dt / np.maximum(tau, 1e-9)), 1.0)
        on = self.on[:, None]
        self.a_up = np.where(on, ex(self.tu * fac), self.base_up)
        self.a_down = np.where(on, ex(self.td * fac), self.base_down)
        try:
            return super().step(f_star)
        finally:
            self.a_up, self.a_down = self.base_up, self.base_down


def advance_thrust(T, k, v_adv, adv_loss):
    """Thrust after the advance-speed loss: |T| (1 - frac), frac = adv_loss * v_adv / sqrt(|T|/k) (the advance
    ratio J ~ V_a / n with n ~ sqrt(T)), clipped to [-0.5, 0.9]; v_adv is the speed of the thruster through
    the water along its own thrust direction (negative = moving against it, which increases thrust)."""
    rel = np.sqrt(np.clip(np.abs(T) / k, 0.0, 1.0))
    frac = np.clip(adv_loss * v_adv / np.maximum(rel, 0.1), -0.5, 0.9)
    return T * (1.0 - frac)


# ---------------------------------------------------------------------------------------------
# plant
# ---------------------------------------------------------------------------------------------

class MechPlant(Plant):
    """``Plant`` plus per-row mechanisms. ``mech`` is a MechSpec with one row per env (or None = all off)."""

    def __init__(self, spec, mech=None, seeds=None, state0=None, t_max=180.0, dt=DT, record_link=False):
        super().__init__(spec, seeds, state0=state0, t_max=t_max, dt=dt)
        n = self.n
        self.mech = MechSpec.off(n) if mech is None else mech
        if self.mech.n != n:
            raise ValueError('one MechSpec row per env')
        M = self.mech
        self.on = {m: np.asarray(M.on(m)) for m in MECHS}
        self.any = {m: bool(v.any()) for m, v in self.on.items()}
        if self.any['thrlag']:
            self.thr = MechThrusterState((n,), np.maximum(spec.k_fwd, spec.k_rev).max(-1), dt, spec.tau_up,
                                         spec.tau_down, spec.rev_hold, spec.slew, M, spec.k_fwd, spec.k_rev)
        if self.any['link'] or record_link:
            self.link = CommandLink(n, np.round(spec.dead_time / dt), M, self.seeds, dt, record=record_link)
        else:
            self.link = None
        self.sag_slow = np.zeros(n)
        self.sag = np.zeros(n)
        if self.any['waves']:
            u = crn_uniform(self.seeds, 0, 720, 6)
            self.w_om = 2 * np.pi / np.maximum(M.wave_period, 0.1)[:, None] * np.array([0.85, 1.0, 1.2])[None, :]
            self.w_ph, self.w_ph2 = 2 * np.pi * u[:, :3], 2 * np.pi * u[:, 3:]
        if self.any['estimator']:
            self.g_every = np.maximum(np.round(M.gps_period / dt), 1).astype(int)
            self.g_phase = np.floor(crn_uniform(self.seeds, 0, 740, 1)[:, 0] * self.g_every).astype(int)
            st0 = self.delayed
            c, s = np.cos(st0[:, 2]), np.sin(st0[:, 2])
            self.g_pos = st0[:, :2].copy()
            self.g_vel = np.stack([c * st0[:, 3] - s * st0[:, 4], s * st0[:, 3] + c * st0[:, 4]], -1)
            self.v_f = self.g_vel.copy()
            self.a_v = np.where(M.vel_tau > 0, 1.0 - np.exp(-dt / np.maximum(M.vel_tau, 1e-9)), 1.0)
            self.last_gps_tick = np.zeros(n, int)
        if self.any['multipath']:
            self._multipath(t_max + 1.0)

    # ------------------------------------------------------------------ multipath schedule
    def _multipath(self, t_max):
        M, n = self.mech, self.n
        nk = int(np.ceil(t_max / MP_KNOT)) + 2
        delta = np.zeros((nk + 1, n, 2))
        for k in range(nk):
            u = crn_uniform(self.seeds, k, 730, 3)
            start = (u[:, 0] < M.mp_rate * MP_KNOT) & self.on['multipath']
            if not start.any():
                continue
            dur = np.maximum(1, np.round(-M.mp_dur * np.log1p(-u[:, 1]) / MP_KNOT)).astype(int)
            ang = 2 * np.pi * u[:, 2]
            v = (M.mp_size * start)[:, None] * np.stack([np.cos(ang), np.sin(ang)], -1)
            delta[k] += v
            end = np.minimum(k + dur, nk)
            for i in np.flatnonzero(start):
                delta[end[i], i] -= v[i]
        self.mp_off = np.cumsum(delta, 0)[:nk]

    def mp_offset(self, t):
        k = min(int(t / MP_KNOT + 1e-9), len(self.mp_off) - 1)
        return self.mp_off[k]

    # ------------------------------------------------------------------ dynamics
    def _waves(self, t):
        z = np.sin(self.w_om * t + self.w_ph).sum(1) / np.sqrt(3.0)
        return z

    def step(self, q):
        q = np.clip(np.nan_to_num(np.asarray(q, float)), -1.0, 1.0)
        qd = self.cmd_delay.push(q)
        if self.link is not None:
            ql = self.link.push(q, self.tick)
            if self.any['link']:
                qd = np.where(self.on['link'][:, None], ql, qd)
        self.q_applied = qd
        T = self.thr.step(self.thrust_of(qd))
        s, M, n, dt, st = self.spec, self.mech, self.n, self.dt, self.st
        cur = self.env.current(st[:, :2], self.t)
        if self.any['battery']:
            load = (np.abs(T) / np.where(T >= 0, s.k_fwd, s.k_rev)) ** 1.5
            load = np.clip(load, 0, 1).mean(1)
            a = np.where(M.sag_tau > 0, 1.0 - np.exp(-dt / np.maximum(M.sag_tau, 1e-9)), 1.0)
            self.sag_slow = self.sag_slow + a * (load - self.sag_slow)
            self.sag = M.sag_full * (0.7 * load + 0.3 * self.sag_slow)
            T = np.where(self.on['battery'][:, None], T * (1.0 - self.sag[:, None]) ** 1.25, T)
        if self.any['prophull']:
            c, sn = np.cos(st[:, 2]), np.sin(st[:, 2])
            ur = st[:, 3] - (c * cur[:, 0] + sn * cur[:, 1])
            vr = st[:, 4] - (-sn * cur[:, 0] + c * cur[:, 1])
            va = np.stack([ur, vr, st[:, 5]], -1) @ B                      # [N, 4] along each thrust axis
            k = np.where(T >= 0, s.k_fwd, s.k_rev)
            Tm = advance_thrust(T, k, np.sign(T) * va, M.adv_loss[:, None])
            T_used = np.where(self.on['prophull'][:, None], Tm, T)
        else:
            T_used = T
        tau = T_used @ B.T
        tau = tau + wind_wrench(self.P, st, self.env.wind(self.t), n)
        if self.any['prophull']:
            extra = np.stack([np.zeros(n), -M.y_uv * np.abs(ur) * vr, M.n_uv * ur * vr], -1)
            tau = np.where(self.on['prophull'][:, None], tau + extra, tau)
        if self.any['waves']:
            z = self._waves(self.t)
            extra = np.stack([np.zeros(n), M.wave_Y * z, M.wave_N * z], -1)
            tau = np.where(self.on['waves'][:, None], tau + extra, tau)
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
        if self.any['estimator']:
            self._estimator_update()

    # ------------------------------------------------------------------ sensors
    def _estimator_update(self):
        d, s = self.delayed, self.spec
        due = ((self.tick + self.g_phase) % self.g_every == 0) & self.on['estimator']
        if due.any():
            z = crn_normal_u(self.seeds, self.tick, 750, 4)
            c, sn = np.cos(d[:, 2]), np.sin(d[:, 2])
            vw = np.stack([c * d[:, 3] - sn * d[:, 4], sn * d[:, 3] + c * d[:, 4]], -1)
            self.g_pos = np.where(due[:, None], d[:, :2] + s.pos_sd[:, None] * z[:, :2], self.g_pos)
            self.g_vel = np.where(due[:, None], vw + s.vel_sd[:, None] * z[:, 2:4], self.g_vel)
            self.last_gps_tick = np.where(due, self.tick, self.last_gps_tick)
        self.v_f = self.v_f + self.a_v[:, None] * (self.g_vel - self.v_f)

    def measure(self, channel=0):
        pose, vel, gyro = super().measure(channel)
        if self.any['waves']:
            wob = self.mech.head_wobble * np.sin(self.w_om * self.t + self.w_ph2).sum(1) / np.sqrt(3.0)
            pose = pose.copy()
            pose[:, 2] = np.where(self.on['waves'], wrap(pose[:, 2] + wob), pose[:, 2])
        if self.any['estimator']:
            on = self.on['estimator']
            c, sn = np.cos(pose[:, 2]), np.sin(pose[:, 2])
            vb = np.stack([c * self.v_f[:, 0] + sn * self.v_f[:, 1], -sn * self.v_f[:, 0] + c * self.v_f[:, 1]], -1)
            pose = pose.copy()
            vel = vel.copy()
            pose[:, :2] = np.where(on[:, None], self.g_pos, pose[:, :2])
            vel[:, :2] = np.where(on[:, None], vb, vel[:, :2])
        if self.any['multipath']:
            pose = pose.copy()
            pose[:, :2] = np.where(self.on['multipath'][:, None], pose[:, :2] + self.mp_offset(self.t), pose[:, :2])
        return pose, vel, gyro


def crn_normal_u(seeds, tick, channel, k):
    """Standard normals from crn_uniform pairs (Box-Muller), channel-disjoint from plant.crn_normal."""
    u = crn_uniform(seeds, tick, channel, 2 * k)
    u1, u2 = np.maximum(u[:, :k], 1e-300), u[:, k:]
    return np.sqrt(-2.0 * np.log(u1)) * np.cos(2.0 * np.pi * u2)


# ---------------------------------------------------------------------------------------------
# realised latency
# ---------------------------------------------------------------------------------------------

def latency_stats(spec_row, mech_row, seeds, t_max=120.0, dt=DT, read_every=2):
    """Realised command-to-measurement latency of one boat over its evaluation seeds (one per mission repeat).

    Command path, per frame: the time from sending frame k until a frame sent at or after k is in force
    (= dead time for an ideal link; includes jitter, the hold after a drop and outages; base dead time
    rounded to ticks as the plant applies it, plus the unrounded remainder so that an ideal link gives
    exactly spec.dead_time). Measurement path, per controller read (every ``read_every`` ticks): odometry
    latency plus the age of the held GPS sample when the estimator is on. Returns means, 90th percentiles
    (of each path and of their sum, paths independent), the drop fraction and the velocity filter's DC
    group delay (reported, not counted in L)."""
    seeds = np.asarray(seeds, np.uint64)
    n = len(seeds)
    D = int(np.round(float(spec_row.dead_time[0]) / dt))
    rem = float(spec_row.dead_time[0]) - D * dt
    mech = mech_row.repeat(n)
    steps = int(round(t_max / dt))
    if bool(mech_row.on('link')[0]):
        link = CommandLink(n, np.full(n, D), mech, seeds, dt, record=True)
        q = np.zeros((n, N_THR))
        for k in range(steps):
            link.push(q, k)
        idx = np.asarray(link.log_idx)                     # [steps, n] send tick in force after step k
        cmd = []
        for i in range(n):
            ks = np.arange(0, steps - LINK_MAX_EXTRA - D - 60, LINK_PERIOD)
            first = np.searchsorted(idx[:, i], ks, side='left')
            ok = first < steps
            cmd.append((first[ok] - ks[ok]) * dt + rem)
        cmd = np.concatenate(cmd)
        dropped = float(np.mean(np.asarray(link.log_dropped)))
    else:
        cmd = np.array([D * dt + rem])
        dropped = 0.0
    lat = float(spec_row.odom_latency[0])
    if bool(mech_row.on('estimator')[0]):
        g = int(max(round(float(mech_row.gps_period[0]) / dt), 1))
        ages = []
        for s in seeds:
            ph = int(np.floor(crn_uniform(np.array([s], np.uint64), 0, 740, 1)[0, 0] * g))
            t = np.arange(0, g * read_every * 5, read_every)
            last = t - ((t + ph) % g)
            ages.append((t - last) * dt)
        meas = lat + np.concatenate(ages)
    else:
        meas = np.array([lat])
    rng = np.random.default_rng(0)
    tot = rng.choice(cmd, 20000) + rng.choice(meas, 20000)
    vel_gd = float(mech_row.vel_tau[0]) + float(meas.mean() - lat) if bool(mech_row.on('estimator')[0]) else 0.0
    return {'cmd_mean': float(cmd.mean()), 'cmd_p90': float(np.quantile(cmd, 0.9)), 'cmd_max': float(cmd.max()),
            'meas_mean': float(meas.mean()), 'meas_p90': float(np.quantile(meas, 0.9)),
            'L_mean': float(cmd.mean() + meas.mean()), 'L_p90': float(np.quantile(tot, 0.9)),
            'L_spec': float(spec_row.dead_time[0] + spec_row.odom_latency[0]),
            'frames_dropped': dropped, 'vel_group_delay_dc': vel_gd}


# ---------------------------------------------------------------------------------------------
# swapping the plant into probes / mission (process-local, restored on exit)
# ---------------------------------------------------------------------------------------------

class use_mech:
    """``with use_mech(mech_row):`` makes probes.generate and mission.run build MechPlant with ``mech_row``
    repeated to the batch size (a 1-row MechSpec) or used as is (n rows)."""

    def __init__(self, mech):
        self.mech = mech

    def __enter__(self):
        import mission
        import probes
        self._mods = (mission, probes)
        self._old = (mission.Plant, probes.Plant)
        mech = self.mech

        def factory(spec, seeds, state0=None, t_max=180.0, dt=DT):
            m = mech if mech.n == spec.n else mech.repeat(spec.n // mech.n)
            if m.n != spec.n:
                raise ValueError('MechSpec rows do not divide the batch')
            return MechPlant(spec, m, seeds, state0=state0, t_max=t_max, dt=dt)

        mission.Plant = probes.Plant = factory
        return self

    def __exit__(self, *exc):
        (mission, probes), (a, b) = self._mods, self._old
        mission.Plant, probes.Plant = a, b
        return False
