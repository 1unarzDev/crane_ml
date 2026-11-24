"""The sub's reference mission on the batched 6-DOF kernel: one compiled episode per env, envs in parallel.

    setup = Setup(adapter, cfg, base, seeds)        # host: courses, plans [E, K, 4], params [E, P], keys
    out = run(setup, record=0)                       # numba: _episode per env (prange)
    out.summary(e), out.events(e), out.numbers(e), out.track(e); out.write(e, run_dir)

Per control period ``dt``: the autopilot turns the active waypoint into a body wrench (a PI position loop
in NED to a body velocity, a velocity loop with drag feedforward and net-weight compensation; roll and pitch
held level; yaw toward the waypoint while far, then to its heading), ``nb_allocate`` makes thruster forces,
the profile's deadband and an optional thrust noise (the env's own counter-based stream) act on them, and
``nb_advance`` integrates the plant. Then the
mission checks the armed gate's plane crossing, the waypoint, the hold and the pool clearance.
Env i reads only its own rows, so its episode is the same at any batch size.
"""

from dataclasses import dataclass, field
import json
import math
from pathlib import Path

import numpy as np

from engine.jit import need_numba, numba
from engine.kernels import marine6 as K

NONE, SUCCEEDED, FAILED, TIMEOUT = 0, 1, 2, 3
OUTCOMES = {SUCCEEDED: 'SUCCEEDED', FAILED: 'FAILED', TIMEOUT: 'TIMEOUT', NONE: 'TIMEOUT'}
APPROACH, THROUGH, HOLD = 0, 1, 2
EVENTS = ('waypoint', 'gate_passed', 'gate_missed', 'contact', 'hold_done')
EV_WP, EV_PASS, EV_MISS, EV_CONTACT, EV_HOLD = range(5)
SIDES = ('wall_x0', 'wall_xmax', 'wall_y0', 'wall_ymax', 'floor', 'surface')
EK = 64
NUMS = ('path_m', 'effort_Ns', 'max_roll_deg', 'max_pitch_deg', 'hold_rms_m', 'gate_depth_rms_m', 'min_clearance_m',
        'gates_passed', 'gates_missed')


# ---- host side -------------------------------------------------------------------------------------------

def half_extents(data):
    """(half length, half height, half width) of the profile's ``publishedPerformance.dimensionsLxHxWM``."""
    node = (data.get('publishedPerformance') or {}).get('dimensionsLxHxWM')
    if node is None:
        raise ValueError('sub: the profile needs publishedPerformance.dimensionsLxHxWM (the gate and wall margins)')
    L, H, W = (float(v) for v in node['value'])
    return L / 2, H / 2, W / 2


def plan(course, gains):
    """(waypoints [K, 4] x y z yaw, kinds [K], gate of each [K], gates [G, 6] x y z yaw width height)."""
    wps, kinds, which, gates = [], [], [], []
    for g, gt in enumerate(course['gates']):
        c, s = math.cos(gt['yaw']), math.sin(gt['yaw'])
        for d, kind in ((-gains['approach_m'], APPROACH), (gains['exit_m'], THROUGH)):
            wps.append((gt['x'] + d * c, gt['y'] + d * s, gt['z'], gt['yaw']))
            kinds.append(kind)
            which.append(g)
        gates.append((gt['x'], gt['y'], gt['z'], gt['yaw'], gt['width_m'], gt['height_m']))
    f = course['finish']
    wps.append((f['x'], f['y'], f['z'], f['yaw']))
    kinds.append(HOLD)
    which.append(-1)
    return (np.array(wps, float), np.array(kinds, np.int64), np.array(which, np.int64),
            np.array(gates, float).reshape(-1, 6))


@dataclass
class Setup:
    adapter: object
    cfg: dict
    base: object
    seeds: list
    scenarios: list = None
    ident: dict = field(init=False)

    def __post_init__(self):
        from fastsim.identity import derive_seed
        from fastsim.scenario import ScenarioSource
        cfg, veh = self.cfg, self.cfg['vehicle']
        self.dt = float(cfg['dt'])
        if self.scenarios is None:
            source = ScenarioSource(**cfg['scenarios'], base=self.base)
            source.generate(self.seeds)
            self.scenarios = [source.load(s) for s in self.seeds]
        courses = [s.raw for s in self.scenarios]
        gains = self.adapter.mission
        from domains.sub.adapter import GAINS
        self.C = np.array([float(gains[k]) for k in GAINS])
        profile = self.adapter.profile             # adapter.options.profile: the one place the profile is named
        data = json.loads(profile.read_text())
        self.half = np.array(half_extents(data))
        root, where = Path(__file__).resolve().parents[2], profile.resolve()
        self.card_ref = str(where.relative_to(root)) if where.is_relative_to(root) else str(where)
        p = K.pack(profile, self.dt, tuple(veh.get('current_ned', (0.0, 0.0, 0.0))))
        E = len(self.seeds)
        self.P = np.repeat(p[None], E, axis=0)
        rand = dict(veh.get('randomize') or {})
        sd = float(rand.get('current_sd', 0.0))
        if sd > 0:
            for e, s in enumerate(self.seeds):
                g = np.random.default_rng(derive_seed(s, 'engine', 'current'))
                self.P[e, K.CUR:K.CUR + 2] += sd * g.standard_normal(2)
        self.noise_sd = np.full(E, float(veh.get('thrust_noise_sd', 0.0)))
        self.keys = np.array([K.key(s, 'thrust_noise') for s in self.seeds], dtype=np.uint64)
        plans = [plan(c, gains) for c in courses]
        Kmax = max(len(pl[1]) for pl in plans)
        Gmax = max(len(pl[3]) for pl in plans)
        self.WP = np.zeros((E, Kmax, 4))
        self.WK = np.full((E, Kmax), HOLD, np.int64)
        self.WG = np.full((E, Kmax), -1, np.int64)
        self.G = np.zeros((E, max(1, Gmax), 6))
        self.NWP = np.zeros(E, np.int64)
        self.NG = np.zeros(E, np.int64)
        for e, (wp, wk, wg, gt) in enumerate(plans):
            self.WP[e, :len(wk)], self.WK[e, :len(wk)], self.WG[e, :len(wk)] = wp, wk, wg
            self.G[e, :len(gt)] = gt
            self.NWP[e], self.NG[e] = len(wk), len(gt)
        self.POOL = np.array([[c['pool']['length_m'], c['pool']['width_m'], c['pool']['depth_m']] for c in courses])
        self.time_limit = [float(cfg['time_limit_s'])] * E
        self.TL = np.array([int(round(t / self.dt)) for t in self.time_limit], np.int64)
        self.X = np.concatenate([K.init(self.P[e:e + 1], (c['start'][0], c['start'][1], c['start_depth_m']),
                                        (0.0, 0.0, c['start'][2])) for e, c in enumerate(courses)])
        self.track_every = max(1, round(1.0 / (float(cfg['track_hz']) * self.dt)))
        nsub = int(p[K.NSUB])
        self.ident = {'kernel': 'marine6', 'substeps': {'policy': 'lag_bound', 'n': nsub, 'h_s': self.dt / nsub},
                      'randomize': rand, 'thrust_noise_sd': float(veh.get('thrust_noise_sd', 0.0))}

    @property
    def E(self):
        return len(self.seeds)

    def identity(self, e, backend='numba', dtype='float64'):
        from fastsim.identity import core_version, run_identity
        from fastsim.run import UNHASHED
        hashed = {k: v for k, v in self.cfg.items() if k not in UNHASHED}
        hashed['engine'] = {'engine': 'batched', 'backend': backend, 'dtype': dtype, **self.ident}
        return run_identity(self.scenarios[e].content_sha256, hashed, self.adapter.identity(), self.seeds[e],
                            core_version())


# ---- the episode (numba; one env) ---------------------------------------------------------------------------

@numba.njit(cache=True)
def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@numba.njit(cache=True)
def _clip(v, m):
    return min(max(v, -m), m)


@numba.njit(cache=True)
def _event(ev, evv, ne, kind, step, arg, a, b):
    if ne < ev.shape[0]:
        ev[ne, 0], ev[ne, 1], ev[ne, 2] = kind, step, arg
        evv[ne, 0], evv[ne, 1] = a, b
    return ne + 1


@numba.njit(cache=True)
def _pose(x, out):
    w, qx, qy, qz = x[3], x[4], x[5], x[6]
    r00, r10 = 1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy + qz * w)
    r20, r21, r22 = 2 * (qx * qz - qy * w), 2 * (qy * qz + qx * w), 1 - 2 * (qx * qx + qy * qy)
    out[0], out[1], out[2] = x[0], x[1], x[2]
    out[3] = math.atan2(r10, r00)
    out[4] = -math.asin(min(1.0, max(-1.0, r20)))
    out[5] = math.atan2(r21, r22)


@numba.njit(cache=True)
def _episode(x, p, key, sd, wp, wk, wg, nwp, gates, ngates, pool, half, C, dt, tl, rec_every, ev, evv, num, rec):
    kp_pos, vmax_u, vmax_v, vmax_w, kv = C[0], C[1], C[2], C[3], C[4]
    kp_att, kd_att, kp_yaw, kd_yaw, rmax = C[5], C[6], C[7], C[8], C[9]
    turn_rad, face_m, wp_tol, hold_tol, hold_s = C[10], C[11], C[12], C[13], C[14]
    ki_pos, imax = C[17], C[18]
    integ = np.zeros(3)
    w = np.zeros((6, K.S))
    f = np.zeros(K.NT)
    u = np.zeros(K.U)
    tau = np.zeros(6)
    vmax = np.array([vmax_u, vmax_v, vmax_w])
    meff = np.empty(6)
    for i in range(6):
        meff[i] = p[K.MRB + 7 * i] + p[K.MA + 7 * i]
    net = p[K.WW] - p[K.BB]
    nthr = int(p[K.NTHR])
    radius = max(half[0], max(half[1], half[2]))
    pose = np.zeros(6)
    k, armed, s_prev, held = 0, False, 0.0, 0.0
    touching = False
    ne, nrec, out, step = 0, 0, NONE, 0
    path, effort, max_roll, max_pitch = 0.0, 0.0, 0.0, 0.0
    hold_sq, hold_n, depth_sq, depth_n, least = 0.0, 0, 0.0, 0, np.inf
    passed, missed = 0, 0
    while True:
        if rec_every > 0 and step % rec_every == 0 and nrec < rec.shape[0]:
            _pose(x, rec[nrec])
            nrec += 1
        if step >= tl:
            out = TIMEOUT
            break
        # -- autopilot: the active waypoint to a body wrench --
        qw, qx, qy, qz = x[3], x[4], x[5], x[6]
        r00, r01, r02 = 1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)
        r10, r11, r12 = 2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)
        r20, r21, r22 = 2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)
        phi = math.atan2(r21, r22)
        theta = -math.asin(min(1.0, max(-1.0, r20)))
        psi = math.atan2(r10, r00)
        e0, e1, e2 = wp[k, 0] - x[0], wp[k, 1] - x[1], wp[k, 2] - x[2]
        dh = math.sqrt(e0 * e0 + e1 * e1)
        psi_d = math.atan2(e1, e0) if dh > face_m else wp[k, 3]
        yerr = _wrap(psi_d - psi)
        if e0 * e0 + e1 * e1 + e2 * e2 < 1.0:          # integrate near the waypoint only (no wind-up in transit)
            integ[0] = _clip(integ[0] + e0 * dt, imax)
            integ[1] = _clip(integ[1] + e1 * dt, imax)
            integ[2] = _clip(integ[2] + e2 * dt, imax)
        eb0 = r00 * e0 + r10 * e1 + r20 * e2
        eb1 = r01 * e0 + r11 * e1 + r21 * e2
        eb2 = r02 * e0 + r12 * e1 + r22 * e2
        ib0 = r00 * integ[0] + r10 * integ[1] + r20 * integ[2]
        ib1 = r01 * integ[0] + r11 * integ[1] + r21 * integ[2]
        ib2 = r02 * integ[0] + r12 * integ[1] + r22 * integ[2]
        for i, (eb, ib) in enumerate(((eb0, ib0), (eb1, ib1), (eb2, ib2))):
            vd = _clip(kp_pos * eb, vmax[i]) + ki_pos * ib
            if i < 2 and abs(yerr) > turn_rad and dh > 2.0 * face_m:   # far and facing away: turn first
                vd = 0.0
            tau[i] = (p[K.DL + 7 * i] + p[K.DQ + i] * abs(vd)) * vd + kv * meff[i] * (vd - x[K.NU + i])
        tau[0] -= net * r20
        tau[1] -= net * r21
        tau[2] -= net * r22
        tau[3] = meff[3] * (-kp_att * phi - kd_att * x[K.NU + 3])
        tau[4] = meff[4] * (-kp_att * theta - kd_att * x[K.NU + 4])
        rd = _clip(kp_yaw * yerr, rmax)
        tau[5] = (p[K.DL + 35] + p[K.DQ + 5] * abs(rd)) * rd + kd_yaw * meff[5] * (rd - x[K.NU + 5])
        K.nb_allocate(tau, p, f)
        # -- actuators: the profile's deadband, then the env's thrust noise --
        for j in range(K.NT):
            fj = f[j]
            if j >= nthr or abs(fj) < p[K.DB + j]:
                fj = 0.0
            elif sd > 0.0:
                fj *= 1.0 + sd * K.normal(key, np.uint64(step) * np.uint64(K.NT) + np.uint64(j))
            u[6 + j] = fj
            effort += abs(min(max(fj, p[K.FMIN + j]), p[K.FMAX + j])) * dt
        px, py, pz = x[0], x[1], x[2]
        K.nb_advance(x, u, p, dt, w)
        step += 1
        # -- the mission --
        dx, dy, dz = x[0] - px, x[1] - py, x[2] - pz
        path += math.sqrt(dx * dx + dy * dy + dz * dz)
        _pose(x, pose)
        max_roll = max(max_roll, abs(pose[5]))
        max_pitch = max(max_pitch, abs(pose[4]))
        kind = wk[k]
        if armed:
            g = wg[k]
            c, s = math.cos(gates[g, 3]), math.sin(gates[g, 3])
            ox, oy = x[0] - gates[g, 0], x[1] - gates[g, 1]
            sn = ox * c + oy * s
            if s_prev < 0.0 <= sn:
                lat, dzg = -ox * s + oy * c, x[2] - gates[g, 2]
                inside = abs(lat) <= gates[g, 4] / 2 - half[2] and abs(dzg) <= gates[g, 5] / 2 - half[1]
                if inside:
                    passed += 1
                else:
                    missed += 1
                ne = _event(ev, evv, ne, EV_PASS if inside else EV_MISS, step, g, lat, dzg)
                armed = False
            s_prev = sn
        if kind == THROUGH:
            ez = x[2] - wp[k, 2]
            depth_sq += ez * ez
            depth_n += 1
        e0, e1, e2 = wp[k, 0] - x[0], wp[k, 1] - x[1], wp[k, 2] - x[2]
        d3 = math.sqrt(e0 * e0 + e1 * e1 + e2 * e2)
        if kind == HOLD:
            held = held + dt if d3 < hold_tol else 0.0
            if held > 0.0:
                hold_sq += d3 * d3
                hold_n += 1
            if held >= hold_s - 1e-9:
                ne = _event(ev, evv, ne, EV_HOLD, step, k, d3, held)
                out = SUCCEEDED if passed == ngates else FAILED
        elif d3 < wp_tol:
            ne = _event(ev, evv, ne, EV_WP, step, k, d3, 0.0)
            if armed:                       # left a gate's exit point without crossing its plane
                missed += 1
                ne = _event(ev, evv, ne, EV_MISS, step, wg[k], np.nan, np.nan)
                armed = False
            k += 1
            if wk[k] == THROUGH:
                g = wg[k]
                armed = True
                s_prev = (x[0] - gates[g, 0]) * math.cos(gates[g, 3]) + (x[1] - gates[g, 1]) * math.sin(gates[g, 3])
        clear = min(min(x[0], pool[0] - x[0]), min(x[1], pool[1] - x[1]))
        clear = min(clear, min(pool[2] - x[2], x[2])) - radius
        least = min(least, clear)
        if clear < 0.0 and not touching:
            cl = np.array([x[0], pool[0] - x[0], x[1], pool[1] - x[1], pool[2] - x[2], x[2]])
            ne = _event(ev, evv, ne, EV_CONTACT, step, int(np.argmin(cl)), clear, 0.0)
        touching = clear < 0.0
        if out != NONE:
            break
    if rec_every > 0 and nrec < rec.shape[0]:
        _pose(x, rec[nrec])
        nrec += 1
    num[0], num[1] = path, effort
    num[2], num[3] = math.degrees(max_roll), math.degrees(max_pitch)
    num[4] = math.sqrt(hold_sq / hold_n) if hold_n else np.nan
    num[5] = math.sqrt(depth_sq / depth_n) if depth_n else np.nan
    num[6], num[7], num[8] = least, passed, missed
    return out, step, ne, nrec


@numba.njit(cache=True, parallel=True)
def _run(X, P, keys, SD, WP, WK, WG, NWP, G, NG, POOL, HALF, C, dt, TL, rec_every, EV, EVV, NUM, REC,
         OUT, END, NE, NREC):
    for e in numba.prange(X.shape[0]):
        o, s, ne, nr = _episode(X[e], P[e], keys[e], SD[e], WP[e], WK[e], WG[e], NWP[e], G[e], NG[e], POOL[e],
                                HALF, C, dt, TL[e], rec_every, EV[e], EVV[e], NUM[e], REC[e])
        OUT[e], END[e], NE[e], NREC[e] = o, s, ne, nr


# ---- results ---------------------------------------------------------------------------------------------

@dataclass
class Output:
    setup: Setup
    S: dict
    record: int
    wall_s: float = 0.0
    backend: str = 'numba'
    dtype: str = 'float64'

    def now(self, step):
        return round(int(step) * self.setup.dt, 9)

    def outcome(self, e):
        return OUTCOMES[int(self.S['out'][e])]

    def events(self, e):
        n = min(int(self.S['ne'][e]), EK)
        rows = []
        for (kind, step, arg), (a, b) in zip(self.S['ev'][e, :n].tolist(), self.S['evv'][e, :n].tolist()):
            r = {'t': self.now(step), 'kind': EVENTS[kind]}
            if kind == EV_WP:
                r.update(waypoint=int(arg), error_m=round(a, 4))
            elif kind in (EV_PASS, EV_MISS):
                r.update(gate=self.setup.scenarios[e].raw['gates'][arg]['id'],
                         **({'lateral_m': round(a, 4), 'depth_off_m': round(b, 4)} if math.isfinite(a) else {}))
            elif kind == EV_CONTACT:
                r.update(side=SIDES[arg], clearance_m=round(a, 4))
            else:
                r.update(error_m=round(a, 4), held_s=round(b, 3))
            rows.append(r)
        return rows

    def numbers(self, e):
        n = {k: (round(float(v), 4) if math.isfinite(v) else None) for k, v in zip(NUMS, self.S['num'][e].tolist())}
        n['gates_passed'], n['gates_missed'] = int(n['gates_passed']), int(n['gates_missed'])
        n.update(gates=int(self.setup.NG[e]), time_s=self.now(self.S['end'][e]), success=self.outcome(e) == 'SUCCEEDED',
                 tier='T1')
        return n

    def summary(self, e):
        st = self.setup
        contacts = [ev for ev in self.events(e) if ev['kind'] == 'contact']
        return {'outcome': self.outcome(e), 'seed': int(st.seeds[e]), 'sim_s': self.now(self.S['end'][e]),
                'time_limit_s': st.time_limit[e], 'flags': ['open_water_pool', 'no_contact_forces']
                + (['contact'] if contacts else []), 'contacts': len(contacts), 'error': None,
                'overflow': int(self.S['ne'][e]) > EK}

    def track(self, e):
        """[(t, x, y, z, yaw, pitch, roll)] every record-th track row (needs record)."""
        if not self.record:
            return None
        every = self.setup.track_every * self.record
        n = int(self.S['nrec'][e])
        rows = [(self.now(j * every), *map(float, r)) for j, r in enumerate(self.S['rec'][e, :n - 1])]
        return rows + [(self.now(self.S['end'][e]), *map(float, self.S['rec'][e, n - 1]))] if n else rows

    def metrics(self, e):
        s = self.summary(e)
        return {'outcome': s['outcome'], 'run_sha256': self.setup.identity(e)['run_sha256'], 'sim_s': s['sim_s'],
                **{f'sub.{k}': v for k, v in self.numbers(e).items() if isinstance(v, (int, float))
                   and not isinstance(v, bool)}}

    def write(self, e, run_dir, provenance=None):
        """Env e as a crane.run/1 directory: run.json (NED/FRD), events.jsonl, metrics.json, and with
        ``record`` traj.ctraj (6-DOF, track_hz / record)."""
        import tempfile
        from crane import record as crec
        from theater import traj as ctraj
        st = self.setup
        s, ident = self.summary(e), st.identity(e, self.backend, self.dtype)
        sc = st.scenarios[e]
        pair_key, truth_sha = sc.pair_key
        frame = {'world': 'ned', 'origin': 'scenario', 'body': 'frd'}
        engine = {'engine': 'batched', 'backend': self.backend, 'dtype': self.dtype, **st.ident}
        meta = crec.meta(ident['run_sha256'], domain='sub', tier='T1', kind='dynamic',
                         card={'ref': st.card_ref, 'view': 'marine6'},
                         params={'vehicle': st.cfg['vehicle'], 'engine': engine},
                         seed=int(st.seeds[e]), backend=f'engine/{self.backend}', dtype=self.dtype, identity=ident,
                         provenance=provenance or {}, frame=frame,
                         clock={'source': 'sim', 't0': 0.0, 'time_multiplier': None}, timing={'t_src': 'sim'},
                         outcome=s['outcome'], scenario={'name': sc.name, 'file': sc.filename,
                                                         'pair_key': pair_key, 'truth_sha256': truth_sha})
        events = self.events(e)
        traj = None
        if self.record:
            rows = self.track(e)
            agent = 'auv'
            axes = ('x', 'y', 'z', 'yaw', 'pitch', 'roll')
            cols = {f'{agent}.{a}': [r[i + 1] for r in rows] for i, a in enumerate(axes)}
            outcome = {'SUCCEEDED': 'finished', 'TIMEOUT': 'timeout'}.get(s['outcome'], 'failed')
            header = {'run_id': ident['run_sha256'], 'seed': int(st.seeds[e]), 'pose_dims': '6dof',
                      'robot': {'model': 'sub', 'agents': [agent]},
                      'frame': {'type': 'ned', 'origin': 'scenario', 'body': 'frd'},
                      'course': {'name': sc.name, 'pair_key': pair_key, 'truth_sha256': truth_sha},
                      'build': {'recorder': 'engine/marine6'}, 'water': {'mode': 'current'}, 'outcome': outcome,
                      'valid': True}
            traj = Path(tempfile.mkdtemp(prefix='sub-traj-')) / 'traj.ctraj'
            ctraj.write(traj, header, cols, events, hz=float(st.cfg['track_hz']) / self.record)
        try:
            return crec.write_run(run_dir, meta, traj=traj, events=events,
                                  metrics={'summary': s, 'sub': self.numbers(e)})
        finally:
            if traj is not None:
                traj.unlink()
                traj.parent.rmdir()


def run(setup, record=0, threads=None):
    """Every env's episode (numba, envs in parallel)."""
    import time
    E = setup.E
    rec_every = setup.track_every * record if record else 0
    R = (int(setup.TL.max()) // rec_every + 2) if record else 1
    S = {'x': setup.X.copy(), 'out': np.zeros(E, np.int64), 'end': np.zeros(E, np.int64),
         'ne': np.zeros(E, np.int64), 'nrec': np.zeros(E, np.int64), 'ev': np.zeros((E, EK, 3), np.int64),
         'evv': np.zeros((E, EK, 2)), 'num': np.zeros((E, len(NUMS))), 'rec': np.zeros((E, R, 6))}
    old = numba.get_num_threads()
    if threads:
        numba.set_num_threads(int(threads))
    t0 = time.perf_counter()
    try:
        _run(S['x'], setup.P, setup.keys, setup.noise_sd, setup.WP, setup.WK, setup.WG, setup.NWP, setup.G, setup.NG,
             setup.POOL, setup.half, setup.C, setup.dt, setup.TL, rec_every, S['ev'], S['evv'], S['num'], S['rec'],
             S['out'], S['end'], S['ne'], S['nrec'])
    finally:
        numba.set_num_threads(old)
    return Output(setup, S, record, time.perf_counter() - t0)


def run_seeds(adapter, cfg, base, seeds, out, variant='base', backend='numba', dtype=None, record=0, provenance=None):
    """[(seed, outcome, sim_s, run_dir)] and the episodes' wall seconds. A seed whose run dir already holds
    this run (the same run identity) is reused; a dir holding another run is refused."""
    if backend not in adapter.backends or (dtype or 'float64') != 'float64':
        raise NotImplementedError(f'sub: the mission runs on numba float64 only, not {backend} {dtype or "float64"}; '
                                  'fix: --backend numba --dtype float64')
    need_numba()
    from crane import record as crec
    setup = Setup(adapter, cfg, base, list(seeds))
    dirs = [Path(out) / variant / f's{s}' for s in setup.seeds]
    keep = []
    for e, d in enumerate(dirs):
        meta = d / crec.META
        if meta.is_file():
            have = json.loads(meta.read_text()).get('run_id')
            if have != setup.identity(e)['run_sha256']:
                raise FileExistsError(f'{d} holds another run ({have[:12]}); remove it or use another --out')
            continue
        if d.exists() and any(d.iterdir()):
            raise FileExistsError(f'{d} is incomplete (no {crec.META}); remove it and rerun')
        keep.append(e)
    rows = {}
    wall = 0.0
    if keep:
        todo = Setup(adapter, cfg, base, [setup.seeds[e] for e in keep], [setup.scenarios[e] for e in keep])
        result = run(todo, record)
        wall = result.wall_s
        for i, e in enumerate(keep):
            result.write(i, dirs[e], {'variant': variant, **(provenance or {})})
    for e, d in enumerate(dirs):
        m = crec.read_run(d).meta
        rows[e] = (setup.seeds[e], m['outcome'], json.loads((d / 'metrics.json').read_text())['summary']['sim_s'], d)
    return [rows[e] for e in range(len(dirs))], wall
