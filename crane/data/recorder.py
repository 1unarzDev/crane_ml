"""What a run records beyond its metrics row: events, a decimated trajectory, full-rate observations and actions.

A domain YAML (or ``crane run --set``) names it under ``record:``; without the key nothing is recorded and a run's
directory is byte for byte what it was. Levels:

    record: metrics                               # the summary only (the default)
    record: events                                # the run's events as rows too
    record: {traj: {hz: 20, window: 12}}          # world-frame pose and body velocity, 20 Hz, the first 12 s
    record: {obs: {channels: [cmd, vel], hz: 50, dtype: float16}, labels: {fault: none}}
    record: [events, {traj: {hz: 20}}, obs]       # several

``window`` is ``[t0, t1]`` sim seconds or one number (the first N s). ``hz`` defaults to every plant step; it is
quantised to whole plant steps and the rate kept is recorded. Obs channels are the core's robot-visible ones

    pose  pose.x pose.y pose.yaw    the map-frame pose the controller sees
    vel   vel.vx vel.vy vel.wz      the plant's body velocity
    cmd   cmd.vx cmd.vy cmd.wz      the body velocity commanded this step (the action)
    tracks.n                        how many tracks the last observation held

plus any a domain adapter offers through an optional ``observe(sample) -> {name: number | numbers}`` (sample:
t, pose, vel, cmd, obs, ctx). Labels are the config's ``labels`` merged with the adapter's optional
``labels(ctx)``. In a run directory the recorder writes ``record/obs.npz`` (``obs`` T x C, ``channels``, ``t0``,
``hz``, ``labels`` JSON) and ``record/traj.npz`` (``traj`` T x 6: x y yaw vx vy wz); ``Store`` appends the same
data as rows keyed by cell and seed (crane.data.rows), one dataset per kind.
"""

import json
from pathlib import Path

import numpy as np

from crane.data import rows as R

CORE = {'pose': ('pose.x', 'pose.y', 'pose.yaw'), 'vel': ('vel.vx', 'vel.vy', 'vel.wz'),
        'cmd': ('cmd.vx', 'cmd.vy', 'cmd.wz'), 'tracks': ('tracks.n',)}
TRAJ = ('x', 'y', 'yaw', 'vx', 'vy', 'wz')
LEVELS = ('metrics', 'events', 'traj', 'obs')
DEFAULTS = {'traj': {'hz': 20.0, 'window': None}, 'obs': {'channels': ['pose', 'vel', 'cmd'], 'hz': None,
                                                         'dtype': 'float32', 'window': None}}
DIR = 'record'


class RecordError(ValueError):
    """A ``record:`` spec that cannot be read, or a channel nobody provides; one line naming the fix."""


def parse(spec):
    """{events: bool, traj: {hz, window} | None, obs: {channels, hz, dtype, window} | None, labels: {}}."""
    out = {'events': False, 'traj': None, 'obs': None, 'labels': {}}
    items = spec if isinstance(spec, (list, tuple)) else [spec]
    for item in items:
        if item in (None, False, 'metrics'):
            continue
        if isinstance(item, str):
            item = {item: {}}
        if not isinstance(item, dict):
            raise RecordError(f'record: {item!r} is not a level; levels: {", ".join(LEVELS)}')
        for level, opts in item.items():
            if level == 'labels':
                if not isinstance(opts, dict):
                    raise RecordError('record.labels: give a map {name: value}')
                out['labels'].update(opts)
            elif level == 'events':
                out['events'] = bool(opts if not isinstance(opts, dict) else True)
            elif level in ('traj', 'obs'):
                opts = {} if opts in (None, True) else opts
                if not isinstance(opts, dict):
                    raise RecordError(f'record.{level}: give a map ({", ".join(DEFAULTS[level])})')
                unknown = sorted(set(opts) - set(DEFAULTS[level]))
                if unknown:
                    raise RecordError(f'record.{level}: unknown key {unknown[0]!r}; keys: {", ".join(DEFAULTS[level])}')
                out[level] = {**DEFAULTS[level], **opts}
                out[level]['window'] = _window(level, out[level]['window'])
            elif level != 'metrics':
                raise RecordError(f'record: unknown level {level!r}; levels: {", ".join(LEVELS)} (and labels)')
    if out['obs']:
        try:
            np.dtype(out['obs']['dtype'])
        except TypeError:
            raise RecordError(f"record.obs.dtype {out['obs']['dtype']!r} is no numpy dtype (float16, float32)") from None
        chans = out['obs']['channels']
        out['obs']['channels'] = [chans] if isinstance(chans, str) else list(chans)
    return out


def _window(level, w):
    if w is None:
        return None
    if isinstance(w, (int, float)):
        w = [0.0, float(w)]
    if not (isinstance(w, (list, tuple)) and len(w) == 2 and float(w[1]) > float(w[0])):
        raise RecordError(f'record.{level}.window: give [t0, t1] (s, t1 > t0) or one number (the first N s)')
    return [float(w[0]), float(w[1])]


def _every(hz, dt):
    return 1 if not hz else max(1, round(1.0 / (float(hz) * dt)))


def _inside(window, now):
    return window is None or window[0] - 1e-9 <= now <= window[1] + 1e-9


class Recorder:
    """One run's recording. The run loop calls ``start``, ``sample`` once per plant step, ``finish``, ``write``."""

    def __init__(self, spec):
        self.spec = parse(spec)
        self.adapter = self.ctx = None
        self.obs, self.traj, self.events, self.labels, self.channels = [], [], [], {}, None
        self.dt = None

    @classmethod
    def of(cls, spec):
        """A Recorder, or None when the spec records nothing beyond the metrics."""
        rec = cls(spec)
        s = rec.spec
        return rec if (s['events'] or s['traj'] or s['obs']) else None

    @property
    def wants(self):
        return [k for k in ('events', 'traj', 'obs') if self.spec[k]]

    def start(self, dt, adapter=None, ctx=None):
        self.dt, self.adapter, self.ctx = float(dt), adapter, ctx
        self.obs, self.traj, self.events, self.channels = [], [], [], None     # a recorder starts empty per run
        self._traj_every = _every((self.spec['traj'] or {}).get('hz'), self.dt)
        self._obs_every = _every((self.spec['obs'] or {}).get('hz'), self.dt)
        own = getattr(adapter, 'labels', None)
        self.labels = {**self.spec['labels'], **((own(ctx) or {}) if callable(own) else {})}

    def sample(self, step, now, pose_map, pose_world, vel, cmd, obs):
        traj, o = self.spec['traj'], self.spec['obs']
        if traj and step % self._traj_every == 0 and _inside(traj['window'], now):
            if not self.traj:
                self._traj_t0 = now
            self.traj.append((*pose_world, *vel))
        if o and step % self._obs_every == 0 and _inside(o['window'], now):
            values = self._observe(now, pose_map, vel, cmd, obs)
            if not self.obs:
                self._obs_t0 = now
            self.obs.append(values)

    def _observe(self, now, pose, vel, cmd, obs):
        core = {'pose': tuple(pose), 'vel': tuple(vel), 'cmd': tuple(cmd),
                'tracks': (float(len(obs.tracks)) if obs is not None else 0.0,)}
        extra = {}
        offer = getattr(self.adapter, 'observe', None)
        if callable(offer):
            extra = offer({'t': now, 'pose': pose, 'vel': vel, 'cmd': cmd, 'obs': obs, 'ctx': self.ctx}) or {}
        values, names = [], []
        for chan in self.spec['obs']['channels']:
            if chan in core:
                got, label = core[chan], list(CORE[chan])
            elif chan in sum(CORE.values(), ()):
                group = chan.split('.')[0]
                got, label = (core[group][CORE[group].index(chan)],), [chan]
            elif chan in extra:
                got = extra[chan]
                got = tuple(got) if isinstance(got, (list, tuple, np.ndarray)) else (got,)
                label = [chan] if len(got) == 1 else [f'{chan}[{i}]' for i in range(len(got))]
            else:
                known = ', '.join([*CORE, *sum(CORE.values(), ()), *sorted(extra)])
                raise RecordError(f'record.obs: no channel {chan!r}; channels: {known} '
                                  "(a domain adds its own with its adapter's observe())")
            values += [float(v) for v in got]
            names += label
        if self.channels is None:
            self.channels = names
        elif len(names) != len(self.channels):
            raise RecordError(f'record.obs: the channels changed width mid-run ({len(self.channels)} -> {len(names)})')
        return values

    def finish(self, events=()):
        if self.spec['events']:
            self.events = [dict(e) for e in events]

    # ---- the outputs ----------------------------------------------------------------------------------------

    def _hz(self, every):
        return round(1.0 / (every * self.dt), 6)

    def arrays(self):
        """{kind: {array name: numpy array}} of what was recorded (obs, traj)."""
        out = {}
        if self.spec['obs']:
            o = self.spec['obs']
            out['obs'] = {'obs': np.asarray(self.obs, dtype=o['dtype']).reshape(len(self.obs), len(self.channels or ())),
                          'channels': np.array(self.channels or [], dtype=np.str_),
                          't0': np.float64(getattr(self, '_obs_t0', 0.0)), 'hz': np.float64(self._hz(self._obs_every)),
                          'labels': np.array(json.dumps(self.labels, sort_keys=True, default=R._plain))}
        if self.spec['traj']:
            out['traj'] = {'traj': np.asarray(self.traj, dtype=np.float32).reshape(len(self.traj), len(TRAJ)),
                           'channels': np.array(TRAJ, dtype=np.str_),
                           't0': np.float64(getattr(self, '_traj_t0', 0.0)),
                           'hz': np.float64(self._hz(self._traj_every))}
        return out

    def write(self, out):
        """Write record/obs.npz and record/traj.npz into a run directory; returns {kind: relative path}."""
        arrays = self.arrays()
        if not arrays:
            return {}
        where = Path(out) / DIR
        where.mkdir(parents=True, exist_ok=True)
        files = {}
        for kind, a in arrays.items():
            (where / f'{kind}.npz').write_bytes(R.npz_bytes(a))
            files[kind] = f'{DIR}/{kind}.npz'
        return files

    def rows(self, key):
        """{kind: [row]}: obs and traj one row per run (key columns, t0, hz, labels as label.NAME, the array);
        events one row per event (key columns, t, event, fields JSON)."""
        out, arrays = {}, self.arrays()
        labels = {f'label.{k}': v for k, v in self.labels.items()}
        for kind, a in arrays.items():
            out[kind] = [{**key, 't0': float(a['t0']), 'hz': float(a['hz']), **(labels if kind == 'obs' else {}),
                          kind: a[kind]}]
        if self.spec['events']:
            out['events'] = [{**key, 't': e.get('t'), 'event': str(e.get('event', e.get('kind', ''))),
                              'fields': {k: v for k, v in e.items() if k not in ('t', 'event')}} for e in self.events]
        return out

    def meta(self):
        """What a dataset manifest records about these arrays (channels, rate, dtype)."""
        doc = {}
        if self.spec['obs']:
            doc['obs'] = {'channels': self.channels or [], 'hz': self._hz(self._obs_every),
                          'dtype': self.spec['obs']['dtype'], 'window': self.spec['obs']['window']}
        if self.spec['traj']:
            doc['traj'] = {'channels': list(TRAJ), 'hz': self._hz(self._traj_every), 'dtype': 'float32',
                           'window': self.spec['traj']['window']}
        return doc


def load_run(run_dir):
    """{kind: {array name: value}} of a run directory's record/*.npz (labels parsed), {} when none."""
    out = {}
    for f in sorted((Path(run_dir) / DIR).glob('*.npz')):
        with np.load(f, allow_pickle=False) as z:
            a = {k: z[k] for k in z.files}
        if 'labels' in a:
            a['labels'] = json.loads(str(a['labels']))
        a['channels'] = [str(c) for c in a.get('channels', [])]
        for k in ('t0', 'hz'):
            if k in a:
                a[k] = float(a[k])
        out[f.stem] = a
    return out


class Store:
    """A study-level store: each kind (obs, traj, events) a dataset DIR/KIND (rows + manifest.json), one row per run
    keyed by the key columns (e.g. cell, arm, seed). A later Store into the same DIR adds its files to the manifest
    (same columns), so jobs append; give each job its own ``part``.

        store = Store(out / 'arrays', part=job_id)
        store.add({'cell': cell_id, 'seed': seed}, recorder)
        store.close()
    """

    def __init__(self, dir, part=None, fmt='npz', extra=None):
        self.dir, self.part, self.fmt, self.extra = Path(dir), part, fmt, dict(extra or {})
        self.writers, self.meta = {}, {}

    def add(self, key, recorder):
        for kind, rows in recorder.rows(key).items():
            if not rows:
                continue
            if kind not in self.writers:
                self.writers[kind] = R.RowWriter(self.dir / kind, R.infer_schema(rows), self.fmt, self.part)
                self.meta[kind] = recorder.meta().get(kind, {})
            for row in rows:
                self.writers[kind].write(row)

    def close(self):
        """Write each kind's files and manifest; returns {kind: manifest}."""
        out = {}
        for kind, w in self.writers.items():
            files = w.close()
            old = R.read_manifest(w.dir)
            if old is not None:
                if old['columns'] != w.schema:
                    raise R.SchemaError(f'{w.dir}: its manifest has other columns; use another store dir')
                new = {f['path'] for f in files}
                files = [f for f in old['files'] if f['path'] not in new] + files
            out[kind] = R.write_manifest(w.dir, w.schema, files, {'kind': kind, **self.meta[kind], **self.extra})
        return out
