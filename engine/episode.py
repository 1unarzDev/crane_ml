"""The batched episode: fastsim/run.py's loop for E envs at once.

    setup = Setup(cfg, base, adapter, seeds)          # host: worlds, plans, vehicle, limits, clock
    out = run(setup, backend='numpy'|'numba'|'jax', dtype='float64', record=0)
    out.summary(e), out.events(e), out.contacts(e), out.numbers(e), out.track(e)

Per step, as fastsim/run.py: at track_hz the track row, the scoring accumulators and the footprint
clearance; at tick_hz perception and the decision; then the idle accounting, the tracker, the plant
and the progress checker. numpy and jax run all envs in lockstep (a finished env stops counting);
numba runs each env's whole episode in one compiled call, envs in parallel (engine/nbepisode.py); jax
runs a pool of slots refilled from the seed queue (engine/jaxrun.py).
``record`` keeps every record-th track row (0: none); events and metrics are always kept.
"""

from dataclasses import dataclass, field
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from engine import boat as BB, exact, geometry as GEO, tracker as TR
from engine.ops import numpy_ops
from engine.vehicles import vehicle
from engine.world import Worlds, footprint_reach
from fastsim.config import merge
from fastsim.plant import Limits
from fastsim.world import World

OUT_TIMEOUT = 3
CONTACTS_K = 32
CONTROLLERS = {'BoatAdapter': BB}


class Unsupported(NotImplementedError):
    pass


@dataclass
class Setup:
    cfg: dict
    base: object
    adapter: object
    seeds: list
    scenarios: list = None
    randomize: dict = None
    mode: str = 'batched'
    workers: int = None          # host processes that load scenarios (default: one per core)
    lim: object = field(init=False)

    def __post_init__(self):
        self.cfg = merge(self.cfg, self.adapter.overrides() or {})
        cfg = self.cfg
        if cfg.get('tracker'):
            raise Unsupported('engine: the hold-over tracker (config `tracker`) is not batched; use fastsim')
        name = type(self.adapter).__name__
        if self.mode == 'batched' and name not in CONTROLLERS:
            raise Unsupported(f'engine: no batched controller for {name}; decoupled mode runs it (engine/decoupled.py)')
        self.lim = self.adapter.limits() or Limits.from_dict(cfg['limits'])
        self.dt = float(cfg['dt'])
        self.steps_per_tick = max(1, round(1.0 / (float(cfg['tick_hz']) * self.dt)))
        self.track_every = max(1, round(1.0 / (float(cfg['track_hz']) * self.dt)))
        self.vehicle = vehicle(cfg['vehicle'], self.lim, self.base, self.dt, self.randomize)
        if self.scenarios is None and self.mode == 'batched':
            from engine.prepare import prepare
            self.scenarios, self.W, self.plans, self.time_limit = prepare(cfg, self.base, list(self.seeds), self.workers)
            self.worlds = None
        else:
            if self.scenarios is None:
                from fastsim.scenario import ScenarioSource
                source = ScenarioSource(**cfg['scenarios'], base=self.base)
                self.scenarios = [source.load(s) for s in self.seeds]
            self.worlds = [World(s) for s in self.scenarios]
            self.W = Worlds.pack(self.worlds, self.lim.footprint)
            self.plans = BB.BoatPlans.pack(self.adapter, self.scenarios, self.worlds) if self.mode == 'batched' else None
            self.time_limit = [float(self.adapter.time_limit(SimpleNamespace(seed=seed, scenario=s, world=w))
                                     or cfg['time_limit_s']) for seed, s, w in zip(self.seeds, self.scenarios, self.worlds)]
        self.reach = footprint_reach(self.lim.footprint)
        self.foot = np.array(self.lim.footprint, dtype=float)
        self.cam = GEO.camera(cfg['perception'])
        self.tl = np.zeros(len(self.seeds), np.int64)   # the last step each env may run
        for e, tlim in enumerate(self.time_limit):
            limit = tlim + float(cfg['grace_s'])
            n = int(limit / self.dt) + 2
            while n * self.dt > limit:
                n -= 1
            self.tl[e] = n
        self.max_steps = int(self.tl.max()) + 1
        self.now = np.array([round(k * self.dt, 9) for k in range(self.max_steps + 1)])
        self.tlim = TR.limits(self.lim, self.vehicle.gain)
        self.P = self.vehicle.rows(self.seeds)
        self.Q = np.repeat(self.vehicle.q[None], len(self.seeds), axis=0)

    @property
    def E(self):
        return len(self.seeds)

    def domain_metrics(self, numbers):
        """What the domain adapter's ``metrics(run_dir)`` gives, from its finalize numbers (no run directory):
        the adapter's ``metrics_from(numbers)`` when it has one, else its batched controller module's
        ``metrics(numbers)`` (engine/boat.py for the boat). The metric names are the domain's."""
        fn = getattr(self.adapter, 'metrics_from', None)
        if fn is None:
            fn = CONTROLLERS[type(self.adapter).__name__].metrics
        return fn(numbers)

    def identity(self, backend, dtype, fastmath=False):
        """What the engine adds to every run identity: backend, dtype, the substep policy."""
        return {'engine': self.mode, 'backend': backend + ('-fastmath' if fastmath else ''), 'dtype': dtype,
                **self.vehicle.ident}


def init_state(setup, record):
    E = setup.E
    R = setup.plans.A['course'].shape[1] if setup.plans else 8
    G = {'route': np.zeros((E, R, 3)), 'head': np.zeros(E, np.int64), 'rlen': np.ones(E, np.int64),
         'live': np.zeros(E, bool), 'exists': np.zeros(E, bool), 'status': np.zeros(E, np.int64),
         'precise': np.zeros(E, bool), 'anchor': np.zeros((E, 2)), 'anchor_t': np.zeros(E),
         'anchored': np.zeros(E, bool), 'popmax': np.zeros(E, bool)}
    M = setup.plans.init(E) if setup.plans else {'outcome': np.zeros(E, np.int64)}
    S = {'t': np.zeros(E, np.int64), 'V': setup.vehicle.init(E), 'G': G, 'M': M,
         'end': np.zeros(E, np.int64), 'driven': np.zeros(E), 'driven_set': np.zeros(E, bool), 'held': np.zeros(E), 'idle_first': np.full(E, -1, np.int64),
         'idle_n': np.zeros(E, np.int64), 'least': np.full(E, np.inf), 'touching': np.zeros(E, bool),
         'contacts': np.zeros((E, CONTACTS_K, 3), np.int64), 'cval': np.zeros((E, CONTACTS_K)),
         'kcount': np.zeros(E, np.int64), 'last_row': np.zeros((E, 3))}
    if record:
        S['rec'] = np.zeros((E, (setup.max_steps // setup.track_every) // record + 2, 3))
    return S


def consts(setup):
    W = setup.W.arrays()
    return {'W': W, 'A': setup.plans.A if setup.plans else {}, 'P': setup.P, 'Q': setup.Q, 'tlim': setup.tlim,
            'foot': setup.foot, 'now': setup.now, 'tl': setup.tl,
            'c0': np.cos(setup.W.start[:, 2]), 's0': np.sin(setup.W.start[:, 2])}


def step_body(ops, setup, C, S, record, do_track, do_tick, decide=None):
    """One step of fastsim/run.py's loop for every env still running. ``do_track``/``do_tick`` say
    whether this step is a track row and a decision tick (static: every env's step index is the
    same modulo the tick period). ``decide(S, pose, act, t)`` replaces the batched controller
    (decoupled mode: scalar controllers per env)."""
    xp = ops.xp
    t = S['t']
    now = C['now'][t]
    act = (S['M']['outcome'] == BB.OUT_NONE)
    veh = setup.vehicle
    pose = veh.pose(xp, S['V'])
    W, A = C['W'], C['A']

    def track(S):
        x, y, yaw = pose
        st = W['start']
        X = st[:, 0] + C['c0'] * x - C['s0'] * y
        Y = st[:, 1] + C['s0'] * x + C['c0'] * y
        row = (ops.pyround(X, 4), ops.pyround(Y, 4), ops.pyround(exact.wrap(yaw + st[:, 2], xp), 5))
        rowa = xp.stack(row, axis=-1)
        out = dict(S)
        if record:
            j = t // setup.track_every
            out['rec'] = ops.put_row(S['rec'], j // record, rowa, act & (j % record == 0))
        if setup.plans:
            out['M'] = BB.score_row(ops, A, S['M'], row, t, act)
        out['last_row'] = xp.where(act[:, None], rowa, S['last_row'])
        c, k = GEO.clearance(xp, W, pose, setup.foot, setup.reach)
        out['least'] = xp.where(act & (c < S['least']), c, S['least'])
        new = act & (c < 0) & ~S['touching']
        G = S['G']
        cause = GEO.contact_cause(xp, W, k, G['route'], G['head'], G['rlen'], G['exists'] & G['live'])
        rec = xp.stack([t, k, cause], axis=-1).astype(S['contacts'].dtype)
        out['contacts'], _ = BB.append(xp, S['contacts'], S['kcount'], new, rec)
        out['cval'], out['kcount'] = BB.append(xp, S['cval'], S['kcount'], new, c)
        out['touching'] = xp.where(act, c < 0, S['touching'])
        return out

    def tick(S):
        if decide is not None:
            return decide(S, pose, act, t)
        vis = GEO.visibility(xp, W, pose, setup.cam, A['targets'], ops.blas)
        M, G = BB.tick(ops, A, S['M'], S['G'], pose, vis & act[:, None], t)
        M = {**M, 'outcome': xp.where(act, M['outcome'], S['M']['outcome'])}
        return {**S, 'M': M, 'G': G}

    if do_track:
        S = track(S)
    if do_tick:
        S = tick(S)
    G = S['G']
    running = act & (S['M']['outcome'] == BB.OUT_NONE)
    live = G['exists'] & G['live']
    driven = xp.where(act & live, now, S['driven'])
    since = xp.where(S['held'] > driven, S['held'], driven)          # max(driven or 0, held or 0)
    idle = running & ~live & (now - since > setup.cfg['mux_idle_s'])
    G, target = TR.command(ops, G, pose, C['tlim'])
    V = veh.drive(ops, S['V'], target, C['P'], C['Q'], setup.dt)
    G = TR.advance(ops, G, veh.pose(xp, V), now, C['tlim'])
    finished = act & ~running
    timeout = running & (t + 1 > C['tl'])
    M = {**S['M'], 'outcome': xp.where(timeout, OUT_TIMEOUT, S['M']['outcome'])}
    return {**S, 't': xp.where(act, t + 1, t), 'V': V, 'G': G, 'M': M, 'driven': driven,
            'driven_set': S['driven_set'] | (act & live),
            'idle_first': xp.where(idle & (S['idle_first'] < 0), t, S['idle_first']),
            'idle_n': S['idle_n'] + idle, 'end': xp.where(finished | timeout, t + 1, S['end'])}


def run_numpy(setup, record=0, decide=None, after=None):
    ops = numpy_ops()
    C = consts(setup)
    S = init_state(setup, record)
    step = 0
    with np.errstate(divide='ignore', invalid='ignore'):
        while (S['M']['outcome'] == BB.OUT_NONE).any():
            S = step_body(ops, setup, C, S, record, step % setup.track_every == 0, step % setup.steps_per_tick == 0,
                          decide)
            step += 1
            if after:
                S = after(S)
    return S


def period(setup):
    return math.lcm(setup.track_every, setup.steps_per_tick)


@dataclass
class Output:
    setup: Setup
    S: dict
    backend: str
    dtype: str
    record: int
    wall_s: float = 0.0
    fastmath: bool = False

    def identity(self, e):
        """fastsim's run identity block, with the engine's backend, dtype and substep policy hashed in."""
        from fastsim.identity import core_version, run_identity
        from fastsim.run import UNHASHED
        st = self.setup
        hashed = {k: v for k, v in st.cfg.items() if k not in UNHASHED}
        hashed['engine'] = st.identity(self.backend, self.dtype, self.fastmath)
        return run_identity(st.scenarios[e].content_sha256, hashed, st.adapter.identity(), st.seeds[e], core_version())

    def metrics(self, e):
        """crane_backends.fastsim.FastsimBackend.metrics of env e, without a run directory: run.* plus the domain's
        own metric names (``Setup.domain_metrics``)."""
        from fastsim.pairs import flat_numbers
        s = self.summary(e)
        numbers = flat_numbers({'run': {'sim_s': s['sim_s'], 'contacts': len(s['contacts']),
                                        'min_clearance_m': s['min_clearance_m']}})
        numbers.update(flat_numbers(self.setup.domain_metrics(self.numbers(e))))
        return {'outcome': s['outcome'], 'run_sha256': self.identity(e)['run_sha256'], 'flags': s['flags'], **numbers}

    def write(self, e, run_dir, provenance=None):
        """Env e as a crane.run/1 directory (crane/record.py): run.json, events.jsonl, metrics.json and,
        with ``record``, traj.ctraj at track_hz / record."""
        import tempfile
        from crane import record as crec
        from fastsim.run import write_traj
        st = self.setup
        s, ident = self.summary(e), self.identity(e)
        events = self.events(e)
        sc = st.scenarios[e]
        pair_key, truth_sha = sc.pair_key
        tier = getattr(st.adapter, 'tier', 'T1')
        meta = crec.meta(ident['run_sha256'], domain=str(st.adapter.identity().get('domain', '')), tier=tier,
                         kind='dynamic' if tier == 'T1' else 'kinematic', card=None,
                         params={'vehicle': st.cfg['vehicle'], 'engine': st.identity(self.backend, self.dtype, self.fastmath)},
                         seed=int(st.seeds[e]), backend=f'engine/{self.backend}', dtype=self.dtype, identity=ident,
                         provenance=provenance or {}, frame={'world': 'enu', 'origin': 'scenario', 'body': 'flu'},
                         clock={'source': 'sim', 't0': 0.0, 'time_multiplier': None}, timing={'t_src': 'sim'},
                         outcome=s['outcome'], scenario={'name': sc.name, 'file': sc.filename, 'pair_key': pair_key,
                                                         'truth_sha256': truth_sha})
        traj = None
        if self.record:
            traj = Path(tempfile.mkdtemp(prefix='engine-traj-')) / 'traj.ctraj'
            head = {**s, 'identity': ident, 'scenario': meta['scenario']}
            write_traj(traj, head, self.track(e), events, float(st.cfg['track_hz']) / self.record, st.adapter)
        rec = crec.write_run(run_dir, meta, traj=traj, events=events,
                             metrics={'summary': s, 'boat': self.numbers(e)})
        if traj is not None:
            traj.unlink()
            traj.parent.rmdir()
        return rec

    def now(self, step):
        return float(self.setup.now[int(step)])

    def outcome(self, e):
        o = int(self.S['M']['outcome'][e])
        return {BB.OUT_SUCCEEDED: 'SUCCEEDED', BB.OUT_FAILED: 'FAILED', OUT_TIMEOUT: 'TIMEOUT'}.get(o, 'TIMEOUT')

    def events(self, e):
        n = min(int(self.S['M']['ecount'][e]), BB.EVENTS_K)
        rows = [tuple(int(v) for v in r) for r in self.S['M']['events'][e, :n]]
        out = self.setup.plans.events(e, rows, self.now)
        return out

    def contacts(self, e):
        n = min(int(self.S['kcount'][e]), CONTACTS_K)
        ids = self.setup.W.ids[e]
        return [{'t': self.now(s), 'id': ids[k], 'clearance_m': round(float(v), 3), 'cause': GEO.CAUSES[c]}
                for (s, k, c), v in zip(self.S['contacts'][e, :n].tolist(), self.S['cval'][e, :n].tolist())]

    def summary(self, e):
        st, dt = self.setup, self.setup.dt
        sim_s = round(int(self.S['end'][e]) * dt, 3)
        contacts = self.contacts(e)
        first = int(self.S['idle_first'][e])
        s = 0.0
        for _ in range(int(self.S['idle_n'][e])):
            s += dt
        least = float(self.S['least'][e])
        mount = (st.cfg['perception'] or {}).get('mount')
        flags = (['oracle_labels'] + (['state_program'] if st.scenarios[e].changing_states() else [])
                 + (['contact'] if contacts else []) + sorted({c['cause'] for c in contacts} & {'route_through_obstacle'})
                 + (['sensor_at_origin'] if not mount else []) + (['idle_drift'] if first >= 0 else []))
        return {'outcome': self.outcome(e), 'seed': int(st.seeds[e]), 'sim_s': sim_s,
                'time_limit_s': st.time_limit[e], 'flags': flags, 'contacts': contacts, 'error': None,
                'idle': {'first_t': self.now(first) if first >= 0 else None, 's': round(s, 3)},
                'min_clearance_m': round(least, 3) if least < math.inf else None,
                'overflow': self.overflow(e)}

    def overflow(self, e):
        return (int(self.S['M']['ecount'][e]) > BB.EVENTS_K or int(self.S['kcount'][e]) > CONTACTS_K
                or bool((self.S['M']['ccount'][e] > BB.CROSS_C).any()))

    def numbers(self, e):
        """boat.json of env e (domains/boat/adapter.py's finalize)."""
        st = self.setup
        M = self.S['M']
        n = st.plans.finish(e, M['cross'][e], M['ccount'][e], float(M['sweep'][e]),
                            [float(v) for v in self.S['last_row'][e]], self.outcome(e), self.now)
        n.pop('overflow')
        k = int(st.plans.A['buoys'][e])
        n.update(contacts=len(self.contacts(e)), time_s=round(int(self.S['end'][e]) * st.dt, 3),
                 gate_buoys_seen=int(M['seen'][e, :k].sum()), tier=st.adapter.tier)
        n['success'] = st.plans.success(self.outcome(e), n)
        return n

    def track(self, e):
        """[(t, x, y, yaw)] at every record-th track row (needs record)."""
        if not self.record:
            return None
        st = self.setup
        rows = int(self.S['end'][e] - 1) // st.track_every // self.record + 1
        every = st.track_every * self.record
        return [(self.now(j * every), *(float(v) for v in r)) for j, r in enumerate(self.S['rec'][e, :rows])]


def run(setup, backend='numpy', dtype='float64', record=0, fastmath=False, threads=None, slots=None):
    import time
    t0 = time.perf_counter()
    if backend == 'numpy':
        if dtype != 'float64':
            raise ValueError('engine: the numpy backend is the float64 reference')
        S = run_numpy(setup, record)
    elif backend == 'jax':
        from engine import jaxrun
        S = jaxrun.run(setup, dtype, record, slots=slots)
    elif backend == 'numba':
        from engine.jit import need_numba
        need_numba()
        from engine import nbepisode
        S = nbepisode.run(setup, record, fastmath=fastmath, threads=threads)
    else:
        raise ValueError(f'engine: backend {backend!r}; known: numpy, numba, jax')
    return Output(setup, S, backend, dtype, record, time.perf_counter() - t0, fastmath)
