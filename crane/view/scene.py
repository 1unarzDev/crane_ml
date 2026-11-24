"""What a view shows: runs in one frame and on one clock, their events on lanes, and the world layers under them.

    scene = load(['runs/boat/base/s3', 'unity.ctraj'])     # crane.run/1 dirs or bare .ctraj files
    scene.runs[0].t, .x, .y, .yaw (numpy), .events, .layers

The first run sets the frame and the clock. Every other run is converted with ``crane.frames`` into that frame
(different origins need a transform, or the overlay is refused) and placed on that clock by the rule
``crane.frames.align`` uses (same clock source: their ``t0`` difference; a sim and a wall clock need an offset).
No Qt here: the window and the tests share it.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
from pathlib import Path
import tempfile

import numpy as np

from crane import frames, record
from crane.view import layers as vlayers, spec as vspec

MAX_RUNS = 6


class SceneError(ValueError):
    pass


@dataclass
class Run:
    name: str
    record: object                 # crane.record.RunRecord, or None for a live run
    t: np.ndarray                  # seconds on the scene clock
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    z: np.ndarray | None = None
    pitch: np.ndarray | None = None
    roll: np.ndarray | None = None
    events: list = field(default_factory=list)      # {t (scene clock), kind, lane, ...}
    layers: list = field(default_factory=list)      # constraint volumes the run carries
    world: list = field(default_factory=list)       # world layers from its scenario
    tier: str | None = None
    kind: str | None = None
    domain: str = ''
    offset_s: float = 0.0          # its own t = 0 on the scene clock
    notes: list = field(default_factory=list)

    @property
    def six_dof(self) -> bool:
        return self.z is not None

    @property
    def rerun(self) -> str:
        """What rerunning this run can do: a recorded run has poses only."""
        return 'from 0 only (a recorded run holds poses, not the sim state)'

    def index_at(self, t: float) -> int:
        """The last sample at or before ``t`` (-1 before the first)."""
        return int(np.searchsorted(self.t, t + 1e-9, side='right')) - 1

    def violations(self) -> np.ndarray:
        return np.array(vlayers.violations(self.layers, self.x, self.y), bool) if self.layers else \
            np.zeros(len(self.t), bool)


@dataclass
class Scene:
    runs: list
    frame: dict
    clock: dict
    outline: list
    lanes: list                    # lane names, top to bottom

    @property
    def t_range(self) -> tuple:
        ts = [r.t for r in self.runs if len(r.t)]
        evs = [e['t'] for r in self.runs for e in r.events]
        lo = min([float(t[0]) for t in ts] + evs + [0.0])
        hi = max([float(t[-1]) for t in ts] + evs + [lo + 1.0])
        return lo, hi

    def world(self) -> list:
        """The first run's world layers plus every run's constraint volumes."""
        base = next((r.world for r in self.runs if r.world), [])
        return base + [c for r in self.runs for c in r.layers]


def open_record(path, tmp_root=None):
    """A ``crane.record.RunRecord`` for a crane.run/1 dir, a fastsim run dir, or a bare ``.ctraj`` (wrapped by
    ``crane.record.from_ctraj`` into a temporary dir: T2 unity unless its header says fastsim)."""
    p = Path(path).expanduser()
    if p.is_dir():
        try:
            return record.read_run(p)
        except record.RecordError:
            if (p / 'run.json').is_file():
                return record.from_fastsim(p)
            raise
    if p.suffix == '.ctraj':
        from theater import traj as ctraj
        h = ctraj.read_header(p)
        side = p.parent / record.SIDECAR
        if side.is_file() or (p.parent / record.META).is_file():
            try:
                r = record.read_run(p.parent)
                if r.file('traj') == p:
                    return r
            except record.RecordError:
                pass
        out = Path(tempfile.mkdtemp(prefix='crane-view-', dir=tmp_root)) / p.stem
        fast = (h.get('build') or {}).get('recorder') == 'fastsim'
        kw = {'tier': 'T0', 'kind': 'kinematic', 'backend': 'fastsim'} if fast else {'tier': 'T2', 'kind': 'unity',
                                                                                      'backend': 'unity'}
        origin = None if (h.get('frame') or {}).get('origin') else 'world'
        return record.from_ctraj(p, out, origin=origin, domain=str((h.get('robot') or {}).get('model') or ''), **kw)
    raise SceneError(f'{p}: not a run directory or a .ctraj')


def _offset(base_clock: dict, clock: dict, given: float | None) -> float:
    if given is not None:
        return float(given)
    if base_clock.get('source') != clock.get('source'):
        raise SceneError(f"clocks differ ({base_clock.get('source')} vs {clock.get('source')}): give --offset "
                         f"(the first run's time of this run's t = 0)")
    return float(clock.get('t0') or 0.0) - float(base_clock.get('t0') or 0.0)


def from_record(rec, name, frame, clock, *, spec=None, transform=None, offset_s=None, planar=False) -> Run:
    """One record as a scene run in ``frame`` and on ``clock``."""
    spec = spec or vspec.view_spec(rec.meta.get('domain') or None)
    off = _offset(clock, rec.clock, offset_s)
    events = _events(rec, spec['lanes'], off)
    lay = vlayers.of_run(rec.path, rec.meta)
    world = vlayers.world_of(rec.path, rec.meta)
    kw = dict(record=rec, events=events, layers=lay, world=world, tier=rec.meta.get('tier'),
              kind=rec.meta.get('kind'), domain=rec.meta.get('domain') or '', offset_s=off)
    if rec.traj is None:
        e = np.zeros(0)
        return Run(name, t=e, x=e, y=e, yaw=e, notes=['event-kind: no poses'], **kw)
    ps = frames.to_common(frames.poses(rec), frame, transform)
    kw['layers'] = convert_layers(lay, rec.frame, frame, transform)
    kw['world'] = convert_layers(world, rec.frame, frame, transform)
    if planar:
        ps = frames.project_planar(ps)
    agent = ps.agents()[0]
    pose = ps.pose(agent)
    yaw = pose.get('yaw', np.zeros(len(ps.t)))
    return Run(name, t=ps.t + off, x=pose['x'], y=pose['y'], yaw=yaw, z=pose.get('z'), pitch=pose.get('pitch'),
               roll=pose.get('roll'), notes=list(ps.notes), **kw)


def convert_layers(layers, src, dst, transform=None) -> list:
    """Layers from frame ``src`` into ``dst`` (every [x, y] they hold; radii and widths are unchanged)."""
    src, dst = frames.frame_of(src), frames.frame_of(dst)
    if src == dst or not layers:
        return layers

    def xy(p):
        q = frames.convert({'x': float(p[0]), 'y': float(p[1]), 'z': 0.0, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0},
                           src, dst, transform)
        return [round(float(q['x']), 4), round(float(q['y']), 4)]

    def walk(v):
        if isinstance(v, (list, tuple)) and len(v) >= 2 and all(isinstance(a, (int, float)) for a in v[:2]) \
                and not any(isinstance(a, (list, dict)) for a in v):
            return xy(v)
        if isinstance(v, (list, tuple)):
            return [walk(a) for a in v]
        if isinstance(v, dict) and 'x' in v and 'y' in v:
            x, y = xy((v['x'], v['y']))
            return {**v, 'x': x, 'y': y}
        return v
    return [{k: (walk(v) if k in ('points', 'polys', 'items') else v) for k, v in layer.items()} for layer in layers]


def _events(rec, lanes: dict, off: float) -> list:
    """The record's events (events.jsonl and those in its trajectory, once each) on the scene clock, laned."""
    seen, out = set(), []
    t_evs = list(rec.traj.events) if rec.traj is not None else []
    for e in t_evs + rec.events:
        if not isinstance(e, dict) or e.get('t') is None or not e.get('kind'):
            continue
        key = (round(float(e['t']), 3), str(e['kind']), str(e.get('text', e.get('id', ''))))
        if key in seen:
            continue
        seen.add(key)
        ev = {**e, 't': float(e['t']) + off}
        if 't_end' in e:
            ev['t_end'] = float(e['t_end']) + off
        ev['lane'] = vspec.lane_of(e, lanes)
        out.append(ev)
    return sorted(out, key=lambda e: e['t'])


LANE_ORDER = ('mission', 'events', 'goal', 'judge', 'log')


def lanes_of(runs) -> list:
    names = {e['lane'] for r in runs for e in r.events}
    return sorted(names, key=lambda n: (LANE_ORDER.index(n) if n in LANE_ORDER else len(LANE_ORDER), n))


def load(paths_, *, transforms=None, offsets=None, planar=True, tmp_root=None) -> Scene:
    """A scene over run paths (``MAX_RUNS`` at most). ``transforms[i]``/``offsets[i]`` map run i (i >= 1) onto
    the first run's frame and clock."""
    paths_ = list(paths_)
    if not paths_:
        raise SceneError('no runs to show')
    if len(paths_) > MAX_RUNS:
        raise SceneError(f'{len(paths_)} runs: a view overlays {MAX_RUNS} at most')
    transforms, offsets = list(transforms or []), list(offsets or [])
    recs = [open_record(p, tmp_root) for p in paths_]
    base = recs[0]
    frame, clock = frames.frame_of(base.frame), dict(base.clock)
    spec = vspec.view_spec(base.meta.get('domain') or None)
    runs = []
    for i, (p, rec) in enumerate(zip(paths_, recs)):
        name = _name(p, rec, [r.name for r in runs])
        tr = transforms[i] if i < len(transforms) else None
        off = offsets[i] if i < len(offsets) else None
        runs.append(from_record(rec, name, frame, clock, transform=tr, offset_s=off if i else 0.0, planar=planar,
                                spec=vspec.view_spec(rec.meta.get('domain') or None)))
    return Scene(runs, frame, clock, spec['outline'], lanes_of(runs))


def _name(p, rec, taken) -> str:
    m = rec.meta
    bits = [m.get('tier'), m.get('domain'), f"s{m['seed']}" if m.get('seed') is not None else None]
    name = ' '.join(b for b in bits if b) or Path(p).name
    tail = '/'.join(Path(p).resolve().parts[-3:])
    return name if name not in taken else f'{name} ({tail})'


def live_run(sim, name='live') -> Run:
    """The scene run of a ``LiveSim`` as it stands (its trail and events so far)."""
    tr = np.asarray(sim.trail, float).reshape(-1, 4)
    spec = getattr(sim, 'spec', None) or vspec.view_spec(sim.domain or None)
    evs = [{**e, 'lane': vspec.lane_of(e, spec['lanes'])} for e in sim.events]
    return Run(name, None, t=tr[:, 0], x=tr[:, 1], y=tr[:, 2], yaw=tr[:, 3], events=evs,
               world=sim.world_layers(), tier=sim.tier, kind='dynamic' if sim.tier == 'T1' else 'kinematic',
               domain=sim.domain)


def meta_summary(run: Run) -> str:
    m = run.record.meta if run.record is not None else {}
    bits = [run.tier or '', run.kind or '', m.get('outcome') or '', m.get('backend') or '']
    return ' · '.join(b for b in bits if b)


def dumps_layers(layers) -> str:
    return json.dumps({'schema': vlayers.SCHEMA, 'layers': layers}, indent=1)
