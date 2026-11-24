"""crane.env: a Gym-style batched environment over the batched engine (Docs/Env.md).

    import crane.env
    env = crane.env.make('boat', tier='T1', n=64, backend='numpy')
    obs = env.reset(seed=0)                                        # [n, obs_dim] float64
    obs, reward, terminated, truncated, info = env.step(action)   # action [n, act_dim]; each [n]

Everything specific comes from the domain: ``config/domains/<D>.yaml`` ``gym:`` declares the action spaces, the
observation channels and the reward weights; the tier config (plant card, limits, perception, scenarios) is the one
``crane run`` and ``crane study`` use; the goals come from the domain's batched controller module (engine/boat.py's
``goals``). ``crane env`` stays the course/world generator: this env has no CLI noun, only this Python API.

Env i of a batch runs scenario seed ``seed + i``, reads only its own rows and draws no randomness, so env i is the
same at n=1 and n=4096 and a reset with the same seed repeats an episode bit for bit. An env that ends
(terminated or truncated) holds still and returns reward 0 until the next reset.
"""

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

ACTIONS = ('velocity', 'pwm')
CHANNELS = {'pose': ('x', 'y', 'yaw'), 'vel': ('u', 'v', 'r'), 'goal': ('goal.dx', 'goal.dy', 'goal.dist',
                                                                        'goal.done'),
            'perception': ('seen.n', 'seen.dx', 'seen.dy')}
REWARD = {'progress': 1.0, 'goal': 0.0, 'contact': -1.0, 'step': 0.0, 'success': 0.0}


class EnvError(ValueError):
    """A make() or step() that cannot go on; one line naming the fix."""


@dataclass
class Space:
    """A box: ``low``/``high`` [dim] and the channel names (and units)."""
    names: list
    low: np.ndarray
    high: np.ndarray
    units: list = field(default_factory=list)

    @property
    def shape(self):
        return (len(self.names),)

    def clip(self, a):
        return np.minimum(np.maximum(a, self.low), self.high)


def _gym(domain):
    from crane import select
    cfg = select.domain_config(domain)
    gym = cfg.get('gym')
    if not gym:
        raise EnvError(f'domain {domain} declares no gym: section; fix: add gym: {{actions, obs, reward}} to '
                       f'config/domains/{domain}.yaml')
    return cfg, gym


def _config(domain, tier, card, sets):
    """(fastsim cfg, base) of the domain's tier with ``sets`` and the card bound to its card keys."""
    from crane import paths, select
    from crane.run import tier_config
    from fastsim import config as fsconfig
    dcfg = select.domain_config(domain)
    _, path, over = tier_config(domain, tier)
    flat = dict(sets or {})
    if card is not None:
        keys = (dcfg.get('card') or {}).get('keys') or (dcfg.get('view') or {}).get('card') or []
        folder = (dcfg.get('card') or {}).get('dir')
        ref = Path(str(card)).expanduser()
        found = next((c for c in [ref, paths.ROOT / ref, *([paths.ROOT / folder / ref, paths.ROOT / folder /
                                                            f'{card}.json'] if folder else [])] if c.is_file()), None)
        if found is None:
            raise EnvError(f'no card {card!r}; fix: a profile JSON path, or a name in {folder or "the card dir"}')
        for k in keys:
            flat[k] = str(found.resolve())
    from crane.study.plan import nest
    return fsconfig.load(path, fsconfig.merge(over, nest(flat)))


class Env:
    """n envs of one domain tier on the batched engine. See the module doc; ``make`` builds one."""

    def __init__(self, domain, tier='T1', n=1, backend='numpy', card=None, seed=0, action=None, obs=None,
                 reward=None, dtype=None, overrides=None, record=None, hz=None):
        from crane import tiers
        self.domain, self.tier, self.n = domain, tiers.normalise(tier), int(n)
        if self.n < 1:
            raise EnvError(f'n {n}: one env at least; fix: make(..., n=64)')
        if backend not in ('numpy', 'numba', 'jax'):
            raise EnvError(f'backend {backend!r}; fix: numpy, numba or jax')
        self.backend = backend
        self.dtype = dtype or ('float32' if backend == 'jax' else 'float64')
        if backend == 'numpy' and self.dtype != 'float64':
            raise EnvError('the numpy backend is the float64 reference; fix: dtype float64, or backend jax')
        _, gym = _gym(domain)
        self.gym = gym
        self.cfg, self.base = _config(domain, self.tier, card, overrides)
        from fastsim.batch import adapter_of
        self.adapter = adapter_of(self.cfg, self.base)
        from engine.episode import CONTROLLERS
        self.controller = CONTROLLERS.get(type(self.adapter).__name__)
        if self.controller is None or not hasattr(self.controller, 'goals'):
            raise EnvError(f'{type(self.adapter).__name__} has no batched controller with goals; fix: a domain '
                           'with an engine controller module (engine/episode.py CONTROLLERS)')
        self.action_kind = action or gym.get('action', 'velocity')
        decl = (gym.get('actions') or {}).get(self.action_kind)
        if self.action_kind not in ACTIONS or decl is None:
            raise EnvError(f'action {self.action_kind!r}; fix: one of {", ".join(gym.get("actions") or {})} '
                           f'(config/domains/{domain}.yaml gym.actions)')
        self.decl = dict(decl)
        self.channels = list(obs or gym.get('obs') or ['pose', 'vel', 'goal'])
        bad = [c for c in self.channels if c not in CHANNELS]
        if bad:
            raise EnvError(f'obs channel {bad[0]!r}; fix: some of {", ".join(CHANNELS)}')
        self.weights = {**REWARD, **(gym.get('reward') or {}), **(reward or {})}
        unknown = sorted(set(self.weights) - set(REWARD))
        if unknown:
            raise EnvError(f'reward term {unknown[0]!r}; fix: some of {", ".join(REWARD)}')
        self.hz = float(hz or self.cfg['tick_hz'])
        self._seed = int(seed)
        self.setup = None
        self.recorder = Recorder(record, self) if record else None
        self.observation_space = Space([n for c in self.channels for n in CHANNELS[c]],
                                       np.full(sum(len(CHANNELS[c]) for c in self.channels), -np.inf),
                                       np.full(sum(len(CHANNELS[c]) for c in self.channels), np.inf))
        self.action_space = None
        self._jax = None

    # ---- episodes ------------------------------------------------------------------------------------------------

    def reset(self, seed=None):
        """Start n episodes on scenario seeds seed .. seed+n-1 (None: the next n after the last reset's)."""
        from engine.episode import Setup
        from fastsim.scenario import ScenarioSource
        if self.recorder is not None and self.setup is not None:
            self.recorder.flush()
        seed = self._seed if seed is None else int(seed)
        self._seed = seed + self.n
        seeds = list(range(seed, seed + self.n))
        ScenarioSource(**self.cfg['scenarios'], base=self.base).generate(seeds)
        st = Setup(self.cfg, self.base, self.adapter, seeds, workers=1)
        self.setup, self.seeds = st, seeds
        veh = st.vehicle
        if self.action_kind not in veh.actions:
            raise EnvError(f'action {self.action_kind!r}: the {self.tier} plant ({veh.name}) takes '
                           f'{", ".join(veh.actions)}; fix: make(..., action={veh.actions[0]!r})')
        self.action_space = self._space(st)
        self.frame = max(1, round(1.0 / (self.hz * st.dt)))
        self.V = veh.init(self.n)
        self.goals, self.glen = self.controller.goals(st.plans)
        self.gi = np.zeros(self.n, np.int64)
        self.steps = np.zeros(self.n, np.int64)
        self.done = np.zeros(self.n, bool)
        self.success = np.zeros(self.n, bool)
        self.touching = np.zeros(self.n, bool)
        self.contacts = np.zeros(self.n, np.int64)
        self.cap = np.array([int(round(t / st.dt)) for t in st.time_limit], np.int64)
        lim = st.lim
        self.tol = np.where(np.arange(self.goals.shape[1])[None] >= (self.glen - 1)[:, None], lim.precise_tol[0],
                            lim.goal_tol[0])
        self.dist = self._dist()
        obs = self._obs()
        if self.recorder is not None:
            self.recorder.start(obs)
        return obs

    def _space(self, st):
        d = self.decl
        names = list(d.get('channels') or [])
        if self.action_kind == 'velocity':
            low = np.array(d.get('low', st.lim.vmin), float)
            high = np.array(d.get('high', st.lim.vmax), float)
        else:
            low = np.broadcast_to(np.array(d.get('low', 1100.0), float), (len(names) or 4,)).copy()
            high = np.broadcast_to(np.array(d.get('high', 1900.0), float), (len(names) or 4,)).copy()
        names = names or [f'a{i}' for i in range(len(low))]
        unit = d.get('unit') or []
        return Space(names, low, high, list(unit) if isinstance(unit, list) else [unit] * len(names))

    def step(self, action):
        """Hold ``action`` [n, dim] (clipped to the action space) for one decision period (1/hz s)."""
        if self.setup is None:
            raise EnvError('step before reset; fix: env.reset(seed=0)')
        a = np.asarray(action, dtype=float).reshape(self.n, -1)
        if a.shape[1] != self.action_space.shape[0]:
            raise EnvError(f'action has {a.shape[1]} columns, the space {self.action_space.shape[0]} '
                           f'({", ".join(self.action_space.names)})')
        a = self.action_space.clip(a)
        active = ~self.done
        self.V = self._plant(a, active)
        self.steps = np.where(active, self.steps + self.frame, self.steps)
        before = self.dist
        now = self._dist()
        reached = active & (now <= self.tol[np.arange(self.n), np.minimum(self.gi, self.goals.shape[1] - 1)])
        progress = np.where(active, before - now, 0.0)
        self.gi = np.where(reached, self.gi + 1, self.gi)
        finished = active & (self.gi >= self.glen)
        self.success |= finished
        c, _ = self._clearance()
        new = active & (c < 0) & ~self.touching
        self.touching = np.where(active, c < 0, self.touching)
        self.contacts += new
        truncated = active & ~finished & (self.steps >= self.cap)
        w = self.weights
        reward = np.where(active, w['progress'] * progress + w['goal'] * reached + w['contact'] * new + w['step']
                          + w['success'] * finished, 0.0)
        self.done = self.done | finished | truncated
        self.dist = self._dist()
        obs = self._obs()
        info = {'t': self.steps * self.setup.dt, 'goal_index': self.gi.copy(), 'contact': new,
                'contacts': self.contacts.copy(), 'clearance': c, 'success': self.success.copy(),
                'seed': np.array(self.seeds)}
        if self.recorder is not None:
            self.recorder.step(obs, a, reward, active)
        return obs, reward, finished, truncated, info

    def close(self):
        """Write the recorder's dataset (when ``record`` was given); returns its manifest or None."""
        return self.recorder.close() if self.recorder is not None else None

    # ---- pieces ------------------------------------------------------------------------------------------------

    def pose(self):
        from engine.ops import numpy_ops
        return self.setup.vehicle.pose(numpy_ops().xp, self.V)

    def _goal(self):
        k = np.minimum(self.gi, self.goals.shape[1] - 1)
        return self.goals[np.arange(self.n), k]

    def _dist(self):
        x, y, _ = self.pose()
        g = self._goal()
        return np.hypot(g[:, 0] - x, g[:, 1] - y)

    def _clearance(self):
        from engine import geometry as GEO
        st = self.setup
        c, k = GEO.clearance(np, st.W.arrays(), self.pose(), st.foot, st.reach)
        return np.where(np.isfinite(c), c, np.inf), k

    def _obs(self):
        x, y, yaw = self.pose()
        cols = []
        for ch in self.channels:
            if ch == 'pose':
                cols += [x, y, yaw]
            elif ch == 'vel':
                cols += list(self.setup.vehicle.velocity(np, self.V))
            elif ch == 'goal':
                g = self._goal()
                dx, dy = g[:, 0] - x, g[:, 1] - y
                c, s = np.cos(yaw), np.sin(yaw)
                cols += [c * dx + s * dy, -s * dx + c * dy, np.hypot(dx, dy), self.gi / np.maximum(self.glen, 1)]
            else:
                cols += list(self._seen(x, y, yaw))
        return np.stack(cols, axis=-1).astype(np.float64)

    def _seen(self, x, y, yaw):
        """(how many targets the camera sees, the nearest one's body-frame dx, dy; 0 when none)."""
        from engine import geometry as GEO
        st = self.setup
        W, T = st.W.arrays(), st.plans.A['targets']
        vis = GEO.visibility(np, W, (x, y, yaw), st.cam, T, True) & st.plans.A['tmatch'].any(axis=2)
        xy = np.take_along_axis(W['xy'], T[..., None], axis=1)
        dx, dy = xy[..., 0] - x[:, None], xy[..., 1] - y[:, None]
        d = np.where(vis, np.hypot(dx, dy), np.inf)
        k = np.argmin(d, axis=1)
        any_ = vis.any(axis=1)
        ndx, ndy = dx[np.arange(self.n), k], dy[np.arange(self.n), k]
        c, s = np.cos(yaw), np.sin(yaw)
        return (vis.sum(axis=1).astype(float), np.where(any_, c * ndx + s * ndy, 0.0),
                np.where(any_, -s * ndx + c * ndy, 0.0))

    def _plant(self, a, active):
        st = self.setup
        if self.backend == 'numba':
            return self._plant_numba(a, active)
        if self.backend == 'jax':
            return self._plant_jax(a, active)
        from engine.ops import numpy_ops
        ops = numpy_ops()
        V = self.V
        with np.errstate(divide='ignore', invalid='ignore'):
            for _ in range(self.frame):
                V = self._one(ops, V, a, st.P, st.Q)
        return {k: np.where(active.reshape((-1,) + (1,) * (v.ndim - 1)), v, self.V[k]) for k, v in V.items()}

    def _one(self, ops, V, a, P, Q):
        veh = self.setup.vehicle
        if self.action_kind == 'pwm':
            return veh.actuate(ops, V, a, P)
        return veh.drive(ops, V, a, P, Q, self.setup.dt)

    def _plant_numba(self, a, active):
        st, veh = self.setup, self.setup.vehicle
        fn = _nb_plant()
        x, c = self.V['x'].copy(), self.V['c'].copy()
        fn(veh.kind, x, c, np.ascontiguousarray(a), st.P, st.Q, st.dt, veh.tick_s, veh.substeps, active,
           self.frame, self.action_kind == 'pwm')
        return {'x': x, 'c': c}

    def _plant_jax(self, a, active):
        from engine.ops import jax_ops
        if self._jax is None or self._jax[0] is not self.setup:
            ops = jax_ops(self.dtype)
            jax = ops.jax
            P, Q = ops.to(self.setup.P), ops.to(self.setup.Q)

            @jax.jit
            def plant(V, a):
                for _ in range(self.frame):
                    V = self._one(ops, V, a, P, Q)
                return V
            self._jax = (self.setup, ops, plant)
        _, ops, plant = self._jax
        V = plant({k: ops.to(v) for k, v in self.V.items()}, ops.to(a))
        V = {k: np.asarray(v, dtype=np.float64) for k, v in V.items()}
        return {k: np.where(active.reshape((-1,) + (1,) * (v.ndim - 1)), v, self.V[k]) for k, v in V.items()}


_NB = {}


def _nb_plant():
    """The numba plant step: every active env's row, in parallel, through the vehicle's numba twin."""
    if 'fn' not in _NB:
        from engine.jit import need_numba, numba
        need_numba()
        from engine.kernels import profile as PK
        from engine.vehicles import PROFILE, nb_drive

        @numba.njit(parallel=True, cache=True)
        def plant(kind, X, C, U, P, Q, dt, tick_s, substeps, active, frame, raw):
            for e in numba.prange(X.shape[0]):
                if not active[e]:
                    continue
                pwm = np.zeros(4)
                for _ in range(frame):
                    if raw and kind == PROFILE:
                        for _ in range(substeps):
                            PK.nb_step(X[e], U[e], P[e], tick_s)
                    else:
                        nb_drive(kind, X[e], C[e], U[e], P[e], Q[e], dt, tick_s, substeps, pwm, True)
        _NB['fn'] = plant
    return _NB['fn']


class Recorder:
    """Obs/action datasets of an Env's episodes (crane.data.rows; the layout crane.data.recorder.Store writes): one
    row per (env, episode) in DIR/obs with ``obs`` T x C, ``action`` T x A and ``reward`` T, plus seed, env,
    episode, steps, return and success; the manifest names the channels and the rate."""

    def __init__(self, out, env):
        self.out, self.env = Path(out).expanduser(), env
        self.episode, self.writer, self.buf = -1, None, None

    def start(self, obs):
        self.episode += 1
        self.buf = [{'obs': [o], 'action': [], 'reward': []} for o in obs]

    def step(self, obs, action, reward, active):
        for e in np.flatnonzero(active):
            b = self.buf[e]
            b['obs'].append(obs[e])
            b['action'].append(action[e])
            b['reward'].append(reward[e])

    def flush(self):
        from crane.data import rows as R
        if self.buf is None:
            return
        env = self.env
        if self.writer is None:
            self.writer = R.RowWriter(self.out / 'obs', {'seed': 'int', 'env': 'int', 'episode': 'int',
                                                         'steps': 'int', 'return': 'float', 'success': 'bool',
                                                         'obs': 'array:float32', 'action': 'array:float32',
                                                         'reward': 'array:float32'}, fmt='npz')
        for e, b in enumerate(self.buf):
            self.writer.write({'seed': env.seeds[e], 'env': e, 'episode': self.episode, 'steps': len(b['reward']),
                               'return': float(sum(b['reward'])), 'success': bool(env.success[e]),
                               'obs': np.array(b['obs'][:-1] or b['obs'][:1]), 'action':
                               np.array(b['action']).reshape(len(b['action']), env.action_space.shape[0]),
                               'reward': np.array(b['reward'], dtype=float)})
        self.buf = None

    def close(self):
        from crane.data import rows as R
        self.flush()
        if self.writer is None:
            return None
        env = self.env
        files = self.writer.close()
        return R.write_manifest(self.out / 'obs', self.writer.schema, files, {
            'kind': 'obs', 'domain': env.domain, 'tier': env.tier, 'backend': env.backend, 'dtype': env.dtype,
            'channels': env.observation_space.names, 'actions': env.action_space.names, 'action': env.action_kind,
            'hz': env.hz, 'reward': env.weights})


def make(domain, tier='T1', n=1, backend='numpy', card=None, seed=0, **options):
    """A batched Gym-style Env of ``n`` envs (see the module doc). ``options``: action (a gym.actions name), obs
    (channel groups), reward (weights over the domain's), dtype, set ({dotted key: value} config overrides),
    record (a dir for the obs/action dataset), hz (decisions per second; default the tier's tick_hz)."""
    if 'set' in options:
        options['overrides'] = options.pop('set')
    return Env(domain, tier, n, backend, card, seed, **options)


__all__ = ['make', 'Env', 'EnvError', 'Space', 'CHANNELS', 'REWARD']

