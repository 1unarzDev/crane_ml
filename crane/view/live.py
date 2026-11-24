"""Live mode: a sim stepped in-process behind a small interface, so the window never knows which engine runs.

    sim = FastsimLive.for_domain('boat', 'T1', seed=3)
    sim.step(500)                 # 500 sim steps (or fewer: it stops at the outcome)
    snap = sim.snapshot()         # the full in-memory state
    sim.set_param('current_speed', 0.3)
    sim.restore(snap)             # back to that state; the sliders' current values apply from there
    sim.trail, sim.events, sim.outcome

``LiveSim`` is the interface (reset, step(n), snapshot/restore, set_param). ``FastsimLive`` implements it on
fastsim's scalar path: the same loop as ``fastsim.run.simulate`` (which writes a run directory and so cannot
pause), plus the view's disturbances. A batched engine plugs in as another ``LiveSim``.

A parameter that changes the plant or the limits rebuilds the tracker and the plant at the current pose and
body velocity. Everything is deterministic: a snapshot restored and stepped again gives the same states.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
import copy
import hashlib
import json
import math
from pathlib import Path

from crane import paths
from crane.view import layers as vlayers, spec as vspec


class LiveSim(ABC):
    """One episode a window can step, rewind and retune."""

    dt: float
    seed: int
    tier: str
    domain: str
    frame: dict                    # the crane.run/1 frame its trail is in

    @abstractmethod
    def reset(self, seed: int | None = None) -> None:
        """Back to t = 0 (with ``seed``, a new episode), keeping the current parameter values."""

    @abstractmethod
    def step(self, n: int = 1) -> int:
        """Advance up to ``n`` steps; returns the steps taken (fewer once the episode has an outcome)."""

    @abstractmethod
    def snapshot(self):
        """The full state, opaque; ``restore`` takes it back (any number of times)."""

    @abstractmethod
    def restore(self, snap) -> None:
        """Return to ``snap``; the parameters' current values apply from there."""

    @abstractmethod
    def set_param(self, name: str, value: float) -> None:
        """Change a declared parameter (``params()`` names them) from now on."""

    @abstractmethod
    def params(self) -> dict:
        """{name: current value} of every declared parameter."""

    @property
    @abstractmethod
    def t(self) -> float:
        """Sim seconds."""

    @property
    @abstractmethod
    def done(self) -> bool: ...

    @property
    @abstractmethod
    def outcome(self) -> str | None: ...

    @property
    @abstractmethod
    def trail(self) -> list:
        """[(t, x, y, yaw)] in ``frame``, at the track rate."""

    @property
    @abstractmethod
    def events(self) -> list:
        """[{t, kind, ...}] so far."""

    def world_layers(self) -> list:
        return []


class _State:
    """Everything that changes while an episode runs (deep-copied whole for a snapshot)."""


def _set_dotted(tree: dict, key: str, value):
    """``tree`` with ``key`` (dots nest, an int part indexes a list) set to ``value``, as a nested override."""
    parts = key.split('.')
    node = tree
    for part in parts[:-1]:
        node = node.setdefault(part, {})
    node[parts[-1]] = value
    return tree


def _get_dotted(tree, key: str):
    for part in key.split('.'):
        tree = tree[int(part)] if isinstance(tree, list) else tree[part]
    return tree


def _with_index(cfg: dict, key: str, value) -> tuple:
    """(key, value) as a config override: ``limits.vmax.0 = 1`` becomes ``limits.vmax = [1, ...rest]``."""
    head, _, last = key.rpartition('.')
    if head and last.isdigit():
        seq = list(_get_dotted(cfg, head))
        seq[int(last)] = value
        return head, seq
    return key, value


class FastsimLive(LiveSim):
    """fastsim's scalar loop, steppable. ``config`` is a fastsim domain YAML; ``spec`` the domain's view spec."""

    def __init__(self, config, seed: int, *, overrides=None, spec=None, tier='T1', domain='', card_dir=None):
        from fastsim import batch, config as fsconfig
        from fastsim.scenario import ScenarioSource
        from fastsim.world import World
        self.config_path, self.seed, self.tier, self.domain = Path(config).resolve(), int(seed), tier, domain
        self.spec = spec or vspec.view_spec(domain or None)
        self.overrides = overrides or {}
        cfg, self.base = fsconfig.load(self.config_path, self.overrides or None)
        self.cfg0 = cfg
        self.adapter = batch.adapter_of(cfg, self.base)
        self.source = ScenarioSource(**cfg['scenarios'], base=self.base)
        self.card_dir = Path(card_dir or paths.HOME / '.cache' / 'crane' / 'view' / 'cards')
        self.decl = {p['name']: p for p in vspec.params_for(self.spec, tier)}
        self.cfg_over, self.card_over, self.card_file = {}, {}, None
        self.frame = dict(self.spec['frame'])
        self.generated = False
        self._load(self.seed)
        self.values = {name: self._initial(p) for name, p in self.decl.items()}
        self.reset()

    @classmethod
    def for_domain(cls, domain, tier, seed, **kw):
        config, overrides = vspec.tier_config(domain, tier)
        return cls(config, seed, overrides=overrides, spec=vspec.view_spec(domain), tier=tier, domain=domain, **kw)

    # ---- set-up ---------------------------------------------------------------------------------------------

    def _load(self, seed):
        from fastsim.world import World
        if not self.source.path(seed).exists():
            self.generated = bool(self.source.generate([seed]))
        self.scenario = self.source.load(seed)
        self.world = World(self.scenario)

    def _initial(self, p):
        kind, _, key = p['bind'].partition(':')
        try:
            if kind == 'config':
                return float(_get_dotted(self.cfg0, key))
            if kind == 'card':
                return float(_get_dotted(self._card_tree(), key))
        except (KeyError, IndexError, TypeError, ValueError, OSError):
            pass
        return float(p.get('default', 0.0)) if kind != 'disturbance' else 0.0

    def _card_path(self) -> Path | None:
        for key in self.spec.get('card') or []:
            try:
                v = _get_dotted(self.cfg0, key)
            except (KeyError, IndexError, TypeError):
                continue
            p = Path(str(v)).expanduser()
            for cand in (p, self.base / p, paths.ROOT / p):
                if cand.is_file():
                    return cand.resolve()
        return None

    def _card_tree(self):
        p = self._card_path()
        if p is None:
            raise OSError('this config names no plant card file')
        return json.loads(p.read_text())

    def _cfg(self) -> dict:
        from fastsim.config import merge
        return merge(merge(self.cfg0, self.adapter.overrides() or {}), self.cfg_over)

    def _current_key(self):
        cur = self.spec.get('current')
        return cur.get(self.tier) if isinstance(cur, dict) else cur

    def world_layers(self) -> list:
        return vlayers.course_layers(self.scenario.raw) if isinstance(self.scenario.raw, dict) else []

    # ---- the episode ----------------------------------------------------------------------------------------

    def reset(self, seed=None):
        from fastsim import controller as ctl, identity, perception as perc
        from fastsim.plant import DelayLine, Limits, vehicle
        if seed is not None and int(seed) != self.seed:
            self.seed = int(seed)
            self._load(self.seed)
        cfg = self._cfg()
        S = _State()
        S.cfg = cfg
        S.lim = self.adapter.limits() or Limits.from_dict(cfg['limits'])
        S.dt = float(cfg['dt'])
        S.per_tick = max(1, round(1.0 / (float(cfg['tick_hz']) * S.dt)))
        S.track_every = max(1, round(1.0 / (float(cfg['track_hz']) * S.dt)))
        S.tracker, S.plant = vehicle(cfg['vehicle'], S.lim, base=self.base)
        S.clock = {'now': 0.0}
        S.nav = ctl.Navigator(S.lim, S.clock)
        S.sensor = perc.perception(cfg['perception'], self.world, identity.rng(self.seed, 'perception'), self.base)
        S.holdover = perc.HoldoverTracker(self.world, S.sensor, **cfg['tracker']) if cfg.get('tracker') else None
        S.ctx = ctl.RunContext(self.seed, self.scenario, self.world, S.nav, S.lim,
                               identity.rng(self.seed, 'controller'), S.clock)
        S.nav.report = S.ctx.emit
        S.brain = self.adapter.controller(S.ctx)
        S.limit = float(self.adapter.time_limit(S.ctx) or cfg['time_limit_s']) + float(cfg['grace_s'])
        S.mux_idle_s = float(cfg['mux_idle_s'])
        S.step, S.outcome, S.crash = 0, None, None
        S.track, S.contacts, S.touching, S.least = [], [], False, math.inf
        S.reach = max(math.hypot(*p) for p in S.lim.footprint)
        S.noise_rng = identity.rng(self.seed, 'view-noise')
        S.offset = (0.0, 0.0, 0.0)            # the pose error the controller sees, held between ticks
        S.delay = DelayLine((0.0,) * 3)
        S.drift = (0.0, 0.0)                  # map-frame current on a plant that does not take one
        S.applied = {}
        self.S = S
        self.dt = S.dt
        self._apply_all()

    def _memo(self):
        return {id(self.world): self.world, id(self.scenario): self.scenario, id(self.adapter): self.adapter}

    def snapshot(self):
        return copy.deepcopy(self.S, self._memo())

    def restore(self, snap):
        self.S = copy.deepcopy(snap, self._memo())
        self._apply_all()

    @property
    def t(self):
        return round(self.S.step * self.S.dt, 9)

    @property
    def done(self):
        S = self.S
        return S.outcome is not None or S.step * S.dt > S.limit

    @property
    def outcome(self):
        S = self.S
        return S.outcome or ('TIMEOUT' if S.step * S.dt > S.limit else None)

    @property
    def trail(self):
        return self.S.track

    @property
    def events(self):
        S = self.S
        evs = [{'t': c['t'], 'kind': 'contact', 'id': c['id'], 'clearance_m': c['clearance_m'], 'cause': c['cause']}
               for c in S.contacts]
        evs += [{**{k: v for k, v in e.items() if k != 'event'}, 'kind': str(e.get('event'))} for e in S.ctx.events]
        if S.crash:
            evs.append({'t': S.crash['t'], 'kind': 'failure', 'text': S.crash['error']})
        return sorted(evs, key=lambda e: e['t'])

    def pose(self):
        """The world pose now (x, y, yaw)."""
        return self.world.to_world(self.S.plant.pose)

    def step(self, n=1):
        from fastsim import controller as ctl
        from fastsim.run import sightings
        from fastsim.world import contact_cause, footprint_clearance, wrap
        S, world = self.S, self.world
        taken = 0
        while taken < n and S.outcome is None and S.step * S.dt <= S.limit:
            now = S.clock['now'] = round(S.step * S.dt, 9)
            if S.step % S.track_every == 0:
                x, y, yaw = world.to_world(S.plant.pose)
                S.track.append((now, round(x, 4), round(y, 4), round(yaw, 5)))
                clearance, i = footprint_clearance(world, S.plant.pose, S.lim.footprint, S.reach)
                S.least = min(S.least, clearance)
                if clearance < 0 and not S.touching:
                    S.contacts.append({'t': now, 'id': world.ids[i], 'clearance_m': round(clearance, 3),
                                       'cause': contact_cause(world, i, S.nav.goal, S.reach)})
                S.touching = clearance < 0
            if S.step % S.per_tick == 0:
                self._draw_noise()
                seen = self._seen()
                tracks = S.holdover.update(seen, now) if S.holdover else sightings(world, S.sensor, seen)
                try:
                    S.outcome = S.brain.tick(ctl.Observation(now, seen, tracks, S.sensor.fov()))
                except Exception as error:  # noqa: BLE001  the domain's logic raised: this episode ends
                    S.crash = {'t': now, 'error': f'{type(error).__name__}: {error}'}
                    S.outcome = 'ERROR'
            goal = S.nav.goal
            if goal is not None and not goal.done:
                S.nav.driven = now
            target = S.tracker.command(goal, self._seen(), now, S.nav.cap)
            if any(S.delay.delay_s):
                target = S.delay(target, S.dt, S.plant.vel)
            moved = S.plant.step(target, S.dt)
            if S.drift != (0.0, 0.0):
                x, y, yaw = S.plant.pose
                dx, dy = S.drift[0] * S.dt, S.drift[1] * S.dt
                S.plant.pose = (x + dx, y + dy, wrap(yaw))
                moved = (moved[0] + dx, moved[1] + dy)
            S.nav.advance(S.plant.pose, moved, now)
            S.step += 1
            taken += 1
        return taken

    def run(self, until_s=None):
        """Step to ``until_s`` (sim seconds) or the outcome."""
        while not self.done and (until_s is None or self.t < until_s):
            if not self.step(max(1, min(1000, round(((until_s or 1e9) - self.t) / self.dt)))):
                break
        return self

    # ---- disturbances and parameters --------------------------------------------------------------------------

    def _seen(self):
        S = self.S
        if S.offset == (0.0, 0.0, 0.0):
            return S.plant.pose
        from fastsim.world import wrap
        x, y, yaw = S.plant.pose
        return (x + S.offset[0], y + S.offset[1], wrap(yaw + S.offset[2]))

    def _draw_noise(self):
        S = self.S
        sx = self._dist('pose_noise_m')
        sy = math.radians(self._dist('yaw_noise_deg'))
        if sx > 0 or sy > 0:
            n = S.noise_rng.normal(size=3)
            S.offset = (float(n[0]) * sx, float(n[1]) * sx, float(n[2]) * sy)
        else:
            S.offset = (0.0, 0.0, 0.0)

    def _dist(self, name) -> float:
        for p in self.decl.values():
            if p['bind'] == f'disturbance:{name}':
                return float(self.values[p['name']])
        return 0.0

    def params(self) -> dict:
        return dict(self.values)

    def declared(self) -> list:
        return list(self.decl.values())

    def set_param(self, name, value):
        if name not in self.decl:
            raise KeyError(f'no parameter {name!r}; declared: {sorted(self.decl)}')
        self.values[name] = float(value)
        self._apply_all()

    def _apply_all(self):
        """Bring the episode state in line with ``values``: overrides, card file, disturbances, rebuild."""
        from fastsim.plant import DelayLine
        S = self.S
        cfg_over, card_over = {}, {}
        for name, p in self.decl.items():
            kind, _, key = p['bind'].partition(':')
            v = self.values[name]
            if kind == 'config' and v != self._initial(p):
                k, val = _with_index(self.cfg0, key, v)
                _set_dotted(cfg_over, k, val)
            elif kind == 'card' and v != self._initial(p):
                card_over[key] = v
        speed, toward = self._dist('current_speed'), math.radians(self._dist('current_toward'))
        yaw0 = self.world.start[2]
        cur = (speed * math.cos(toward - yaw0), speed * math.sin(toward - yaw0)) if speed else (0.0, 0.0)
        ckey = self._current_key()
        if ckey and speed:
            _set_dotted(cfg_over, ckey, [round(cur[0], 9), round(cur[1], 9)])
        S.drift = (0.0, 0.0) if ckey else cur
        if card_over:
            path = self._write_card(card_over)
            for key in self.spec.get('card') or []:
                _set_dotted(cfg_over, key, str(path))
        self.cfg_over, self.card_over = cfg_over, card_over
        delay = self._dist('delay_s')
        if tuple(S.delay.delay_s) != (delay,) * 3:
            S.delay = DelayLine((delay,) * 3)
        want = json.dumps(cfg_over, sort_keys=True)
        if S.applied.get('cfg') != want:
            if S.applied:          # built with other overrides: rebuild where the plant is now
                self._rebuild()
            S.applied['cfg'] = want

    def _write_card(self, over: dict) -> Path:
        tree = self._card_tree()
        for key, v in over.items():
            *parents, leaf = key.split('.')
            node = tree
            for part in parents:
                node = node[int(part)] if isinstance(node, list) else node[part]
            node[leaf] = v
        data = (json.dumps(tree, indent=2, sort_keys=True) + '\n').encode()
        self.card_dir.mkdir(parents=True, exist_ok=True)
        p = self.card_dir / f'{self._card_path().stem}-{hashlib.sha256(data).hexdigest()[:12]}.json'
        if not p.is_file():
            p.write_bytes(data)
        self.card_file = p
        return p

    def _rebuild(self):
        from fastsim.plant import Limits, vehicle
        S = self.S
        cfg = self._cfg()
        lim = self.adapter.limits() or Limits.from_dict(cfg['limits'])
        S.tracker, S.plant = vehicle(cfg['vehicle'], lim, pose=S.plant.pose, vel=S.plant.vel, base=self.base)
        S.lim, S.nav.lim, S.ctx.limits, S.cfg = lim, lim, lim, cfg
        S.reach = max(math.hypot(*p) for p in lim.footprint)


class Snapshots:
    """Snapshots of a live sim every ``every_s`` sim seconds, for rerun from t."""

    def __init__(self, every_s=1.0):
        self.every_s, self.snaps = float(every_s), {}

    def keep(self, sim: LiveSim):
        k = int(math.floor(sim.t / self.every_s + 1e-9))
        if k not in self.snaps:
            self.snaps[k] = (sim.t, sim.snapshot())

    def at_or_before(self, t: float):
        ks = [k for k, (ts, _) in self.snaps.items() if ts <= t + 1e-9]
        return self.snaps[max(ks)] if ks else None

    def drop_after(self, t: float):
        self.snaps = {k: v for k, v in self.snaps.items() if v[0] <= t + 1e-9}


# ---- Promote: the current settings as a crane run batch spec ---------------------------------------------------

def _flat(tree, prefix=''):
    for k, v in tree.items():
        key = f'{prefix}{k}'
        if isinstance(v, dict):
            yield from _flat(v, key + '.')
        else:
            yield key, v


def promote(sim: FastsimLive, out_root='runs/promoted', name=None) -> tuple:
    """Write ``<out_root>/<name>/spec.yaml`` (and the derived card beside it) and return (spec path, command).
    The command is a ``crane run`` over the same fastsim config with the sliders as ``--set`` overrides;
    disturbances crane run cannot apply are listed under ``view_only``."""
    import shlex
    import shutil
    import yaml
    name = name or f"{sim.domain or 'run'}-{sim.tier}-s{sim.seed}-{hashlib.sha256(json.dumps([sim.params(), sim.seed], sort_keys=True).encode()).hexdigest()[:8]}"
    out = Path(out_root) / name
    out.mkdir(parents=True, exist_ok=True)
    over = copy.deepcopy(sim.cfg_over)
    if sim.card_over and sim.card_file:
        card = out / 'card.json'
        shutil.copyfile(sim.card_file, card)
        for key in sim.spec.get('card') or []:
            _set_dotted(over, key, str(card.resolve()))
    sets = dict(_flat(over))
    view_only = {}
    for p in sim.decl.values():
        kind, _, key = p['bind'].partition(':')
        v = sim.values[p['name']]
        if kind == 'disturbance' and v and not (key.startswith('current') and sim._current_key()):
            view_only[key] = v
    runs = Path(out_root) / 'runs'
    cmd = ['crane', 'run', '--config', str(sim.config_path), '--seeds', str(sim.seed), '--out', str(runs),
           '--variant', name] + [a for k, v in sets.items() for a in ('--set', f'{k}={json.dumps(v)}')]
    command = shlex.join(cmd)
    spec = {'schema': 'crane.run.spec/1', 'domain': sim.domain, 'tier': sim.tier, 'config': str(sim.config_path),
            'seeds': str(sim.seed), 'variant': name, 'out': str(runs), 'set': sets, 'params': sim.params(),
            'view_only': view_only, 'command': command}
    path = out / 'spec.yaml'
    path.write_text('# crane run batch spec, promoted from crane view; `command` runs it.\n'
                    + yaml.safe_dump(spec, sort_keys=False))
    return path, command
