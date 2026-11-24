"""crane.run/1: one run of any tier, domain or simulator, as a directory (Docs/RunRecord.md).

    run.json       the meta: schema, run_id, domain, tier, kind, card, params, seed, backend, dtype, identity,
                   provenance, frame (required), clock (required), timing (optional)
    traj.ctraj     poses and events, crane-traj/1 (theater/traj.py); absent for an event-kind run
    events.jsonl   {t, kind, ...} per line, t in seconds from clock.t0
    metrics.json   the run's numbers

A directory another writer owns keeps its own ``run.json`` and gets ``crane.run.json`` beside it; the meta's
``files`` names where its traj, events and metrics are. ``read_run`` reads either layout.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
import shutil

from crane.frames import BODIES, CLOCKS, WORLDS
from theater import traj as ctraj

SCHEMA = 'crane.run/1'
META = 'run.json'
SIDECAR = 'crane.run.json'           # beside a run.json another schema owns
FILES = {'traj': 'traj.ctraj', 'events': 'events.jsonl', 'metrics': 'metrics.json'}
TIERS = ('T0', 'T1', 'T2')
KIND_TIER = {'event': 'T0', 'kinematic': 'T0', 'dynamic': 'T1', 'unity': 'T2', 'field': None}
TIMING_KEYS = ('injected_delay_s', 'derate', 't_src')


class RecordError(ValueError):
    pass


def _number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def validate(meta: dict) -> dict:
    """``meta`` if it is a crane.run/1 meta; otherwise RecordError naming every problem."""
    if not isinstance(meta, dict):
        raise RecordError(f'a run meta is a mapping, not {type(meta).__name__}')
    bad = []
    if meta.get('schema') != SCHEMA:
        bad.append(f"schema is {meta.get('schema')!r}, not {SCHEMA!r}")
    if not isinstance(meta.get('run_id'), str) or not meta.get('run_id'):
        bad.append('run_id: a non-empty string')
    kind, tier = meta.get('kind'), meta.get('tier')
    if kind not in KIND_TIER:
        bad.append(f'kind {kind!r} is not one of {tuple(KIND_TIER)}')
    elif KIND_TIER[kind] and tier != KIND_TIER[kind]:
        bad.append(f'tier {tier!r}: a {kind} run is {KIND_TIER[kind]}')
    elif tier is not None and tier not in TIERS:
        bad.append(f'tier {tier!r} is not one of {TIERS}')
    frame = meta.get('frame')
    if not isinstance(frame, dict):
        bad.append('frame is required: {world: enu|ned, origin: <name>, body: flu|frd}')
    else:
        if frame.get('world') not in WORLDS:
            bad.append(f"frame.world {frame.get('world')!r} is not one of {WORLDS}")
        if not isinstance(frame.get('origin'), str) or not frame.get('origin'):
            bad.append('frame.origin: the name of the origin the poses are relative to')
        if frame.get('body') not in BODIES:
            bad.append(f"frame.body {frame.get('body')!r} is not one of {BODIES}")
    clock = meta.get('clock')
    if not isinstance(clock, dict):
        bad.append('clock is required: {source: sim|wall, t0, time_multiplier}')
    else:
        if clock.get('source') not in CLOCKS:
            bad.append(f"clock.source {clock.get('source')!r} is not one of {CLOCKS}")
        if not _number(clock.get('t0')):
            bad.append('clock.t0: a finite number (s; unix seconds on a wall clock)')
        tm = clock.get('time_multiplier', 'missing')
        if tm == 'missing' or not (tm is None or (_number(tm) and tm > 0)):
            bad.append('clock.time_multiplier: sim seconds per wall second, or null when unpaced')
    timing = meta.get('timing')
    if timing is not None:
        if not isinstance(timing, dict):
            bad.append('timing: a mapping {injected_delay_s, derate, t_src}')
        elif 't_src' in timing and not isinstance(timing['t_src'], str):
            bad.append('timing.t_src: the name of the clock the stamps come from')
    if bad:
        raise RecordError(f"crane.run/1 meta {meta.get('run_id', '?')!r}: " + '; '.join(bad))
    return meta


def meta(run_id, *, domain, tier, kind, frame, clock, card=None, params=None, seed=None, backend=None,
         dtype=None, identity=None, provenance=None, timing=None, **extra) -> dict:
    """A checked meta with every key present."""
    return validate({'schema': SCHEMA, 'run_id': str(run_id), 'domain': domain, 'tier': tier, 'kind': kind,
                     'card': card, 'params': params or {}, 'seed': seed, 'backend': backend, 'dtype': dtype,
                     'identity': identity or {}, 'provenance': provenance or {}, 'frame': frame,
                     'clock': clock, **({'timing': timing} if timing is not None else {}), **extra})


def _dump(obj) -> str:
    return json.dumps(obj, indent=2, sort_keys=True, allow_nan=False) + '\n'


def _atomic(path: Path, data: bytes) -> None:
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_bytes(data)
    tmp.replace(path)


def _jsonl(path: Path) -> list:
    if not path.is_file():
        return []
    return [json.loads(s) for s in path.read_text().splitlines() if s.strip()]


@dataclass
class RunRecord:
    path: Path
    meta: dict
    _traj: object = field(default=None, repr=False)
    _loaded: bool = field(default=False, repr=False)

    @property
    def run_id(self) -> str:
        return self.meta['run_id']

    @property
    def frame(self) -> dict:
        return self.meta['frame']

    @property
    def clock(self) -> dict:
        return self.meta['clock']

    def file(self, role: str) -> Path | None:
        name = (self.meta.get('files') or {}).get(role, FILES.get(role))
        return self.path / name if name else None

    @property
    def has_traj(self) -> bool:
        p = self.file('traj')
        return bool(p and p.is_file())

    @property
    def traj(self):
        """The ``theater.traj.Traj``, read on first use (None without one)."""
        if not self._loaded:
            self._traj = ctraj.read(self.file('traj')) if self.has_traj else None
            self._loaded = True
        return self._traj

    @property
    def events(self) -> list:
        """``{t, kind, ...}`` per event (a writer's ``event`` key reads as ``kind``)."""
        p = self.file('events')
        rows = _jsonl(p) if p else []
        return [{**e, 'kind': e.get('kind', e.get('event'))} for e in rows]

    @property
    def metrics(self) -> dict | None:
        p = self.file('metrics')
        return json.loads(p.read_text()) if p and p.is_file() else None


def write_run(dir, meta: dict, traj=None, events=(), metrics=None) -> RunRecord:
    """Write a crane.run/1 directory. ``traj`` is a ``theater.traj.Traj``, ``.ctraj`` bytes or a path; its
    header gets the meta's frame (a different frame already in it is refused). An event-kind run has none."""
    validate(meta)
    d = Path(dir)
    if d.exists() and any(d.iterdir()):
        raise FileExistsError(f'{d}: a run directory is written once')
    if traj is not None and meta['kind'] == 'event':
        raise RecordError('an event-kind run has no trajectory')
    d.mkdir(parents=True, exist_ok=True)
    if traj is not None:
        path = d / FILES['traj']
        if isinstance(traj, ctraj.Traj):
            h = {k: v for k, v in traj.header.items() if k not in ('ticks', 'duration_s', 'columns', 'box')}
            ctraj.write(path, h, traj.columns, traj.events, hz=traj.hz, specs=traj.specs)
        elif isinstance(traj, (bytes, bytearray)):
            ctraj.loads(bytes(traj))
            _atomic(path, bytes(traj))
        else:
            shutil.copyfile(traj, path)
        _stamp_frame(path, meta['frame'])
    _atomic(d / FILES['events'], ''.join(ctraj.dumps_json(e) + '\n' for e in events).encode())
    if metrics is not None:
        _atomic(d / FILES['metrics'], _dump(metrics).encode())
    _atomic(d / META, _dump(meta).encode())
    return RunRecord(d, meta)


def _stamp_frame(path: Path, frame: dict) -> None:
    have = ctraj.read_header(path).get('frame')
    if have and (have.get('type', have.get('world')), have.get('origin')) != (frame['world'], frame['origin']):
        raise RecordError(f'{path}: the trajectory is in {have}, the meta says {frame}')
    ctraj.amend(path, frame={**(have or {}), 'type': frame['world'], 'origin': frame['origin'],
                             'body': frame['body']})


def read_run(dir) -> RunRecord:
    """The record of a crane.run/1 directory (``crane.run.json`` first, else ``run.json``)."""
    d = Path(dir)
    for name in (SIDECAR, META):
        p = d / name
        if p.is_file():
            m = json.loads(p.read_text())
            if m.get('schema') == SCHEMA:
                return RunRecord(d, validate(m))
    raise RecordError(f'{d}: no crane.run/1 meta (a fastsim run dir: crane.record.from_fastsim)')


def from_fastsim(run_dir) -> RunRecord:
    """Wrap a fastsim run directory in place: ``crane.run.json`` beside its ``run.json``, which is untouched,
    so the fastsim identity and every reader of the fastsim layout stay as they are."""
    from fastsim.run import CRANE_RUN, SCHEMA as FASTSIM, crane_run_meta
    d = Path(run_dir)
    native = json.loads((d / 'run.json').read_text())
    if native.get('schema') != FASTSIM:
        raise RecordError(f"{d}: run.json is {native.get('schema')!r}, not {FASTSIM!r}")
    m = validate(crane_run_meta(native))
    text = json.dumps(m, indent=2) + '\n'
    if not (d / CRANE_RUN).is_file() or (d / CRANE_RUN).read_text() != text:
        _atomic(d / CRANE_RUN, text.encode())
    return RunRecord(d, m)


def from_ctraj(ctraj_path, out_dir, *, tier, kind, domain='', run_id=None, origin=None, body=None,
               clock=None, card=None, params=None, seed=None, backend=None, dtype='float64', provenance=None,
               timing=None, metrics=None) -> RunRecord:
    """A crane.run/1 directory around a ``.ctraj`` another writer made (copied to ``traj.ctraj``; its events
    also go to ``events.jsonl``). The frame comes from its header (``type``, ``origin``) unless ``origin`` or
    ``body`` say otherwise; a header without a frame needs ``origin``. ``clock`` defaults to the sim clock
    from the header's ``t0_s``, unpaced."""
    import hashlib
    src = Path(ctraj_path)
    data = src.read_bytes()
    t = ctraj.loads(data)
    h = t.header
    have = h.get('frame') or {}
    world = have.get('world', have.get('type'))
    if world is None:
        raise RecordError(f'{src}: the header has no frame; pass origin= and a frame world')
    frame = {'world': world, 'origin': origin or have.get('origin'),
             'body': body or have.get('body') or {'enu': 'flu', 'ned': 'frd'}.get(world)}
    clk = clock or {'source': 'sim', 't0': float(h.get('t0_s', 0.0)), 'time_multiplier': None}
    sha = hashlib.sha256(data).hexdigest()
    m = meta(run_id or h.get('run_id') or sha[:16], domain=domain, tier=tier, kind=kind, frame=frame, clock=clk,
             card=card, params=params if params is not None else h.get('params') or {},
             seed=seed if seed is not None else h.get('seed'), backend=backend, dtype=dtype,
             identity={'traj_sha256': sha}, timing=timing,
             provenance={**(provenance or {}), 'source': str(src), 'build': h.get('build')})
    return write_run(out_dir, m, traj=data, events=t.events, metrics=metrics)


def from_unity_ctraj(ctraj_path, out_dir, *, domain='', **kw) -> RunRecord:
    """A T2 Unity run (``--crane-traj``: 6-DOF, ENU/FLU, origin ``world``, sim clock) as crane.run/1."""
    kw.setdefault('backend', 'unity')
    return from_ctraj(ctraj_path, out_dir, tier='T2', kind='unity', domain=domain, **kw)
