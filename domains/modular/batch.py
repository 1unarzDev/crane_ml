"""Batches of the modular domain: envs from (assembly, card, seed, timeline), the fault prior, the recorded channels.

    tl = timelines('logs.npz', tick_hz=50)           # {cmd [M, T, 3], length [M], id [M], faults?, assembly?}
    envs = Envs.build(asms, prof, seeds, M, prior, dr=False, dt=0.005)   # E = M x len(seeds), timeline-major
    rec = Recording(spec, tick_hz, W); rec.rows_of(out, envs) -> rows for crane.data.rows / recorder.Store

A timelines file (npz): ``cmd`` [M, T, 3] body-twist commands (vx m/s, vy m/s, wz rad/s) at the tick rate;
optional ``length`` [M] ticks, ``id`` [M] names, ``hz`` (checked against the domain's tick rate), ``assembly``
[M] names, and logged faults ``mode`` [M, W] (codes of engine.kernels.mecanum.MODES), ``gain``, ``onset`` (s).

The fault prior (``--set faults.KEY=V``): p_healthy (1.0), p_double (0.0), modes {weak, coast, brake, locked:
weight}, gain [lo, hi] (weak), onset_s [lo, hi]. Each (seed, timeline) draws from its own stream, so the same
seed gives the same faults and DR draw in every arm (common random numbers).
"""

from dataclasses import dataclass, field

import numpy as np

from domains.modular import assembly as AS, profile as PR
from engine.kernels import mecanum as K

PRIOR = {'p_healthy': 1.0, 'p_double': 0.0, 'modes': {'weak': 1.0, 'coast': 1.0, 'brake': 1.0, 'locked': 1.0},
         'gain': [0.3, 0.9], 'onset_s': [0.0, 0.0]}
GROUPS = {'cmd': ('cmd', ('vx', 'vy', 'wz')), 'imu': (None, ('gz', 'ax', 'ay')), 'pose': ('pose', ('x', 'y', 'yaw')),
          'vel': ('twist', ('vx', 'vy', 'wz')), 'duty': ('duty', None), 'counts': ('counts', None),
          'speed': ('speed', None)}


class BatchError(ValueError):
    """Inputs a batch cannot use; one line naming the fix."""


def timelines(path, tick_hz):
    """The timelines file as arrays (see the module doc); BatchError names what is wrong."""
    try:
        with np.load(path, allow_pickle=False) as z:
            d = {k: z[k] for k in z.files}
    except (OSError, ValueError) as error:
        raise BatchError(f'{path}: not a readable npz ({error})') from None
    if 'cmd' not in d:
        raise BatchError(f'{path}: no `cmd` array; save cmd [M, T, 3] (vx, vy, wz per tick)')
    cmd = np.asarray(d['cmd'], np.float64)
    if cmd.ndim == 2:
        cmd = cmd[None]
    if cmd.ndim != 3 or cmd.shape[-1] != 3:
        raise BatchError(f'{path}: cmd has shape {cmd.shape}; want [M, T, 3] (vx, vy, wz)')
    if not np.isfinite(cmd).all():
        raise BatchError(f'{path}: cmd has NaN or inf; zero the gaps (and give `length` per timeline)')
    if 'hz' in d and abs(float(d['hz']) - tick_hz) > 1e-6:
        raise BatchError(f'{path}: hz {float(d["hz"])} but the domain ticks at {tick_hz} Hz; resample the timelines')
    M, T = cmd.shape[:2]
    out = {'cmd': cmd, 'length': np.asarray(d.get('length', np.full(M, T)), np.int64),
           'id': [str(s) for s in d['id']] if 'id' in d else [f't{m}' for m in range(M)]}
    if out['length'].shape != (M,) or out['length'].max() > T or out['length'].min() < 1:
        raise BatchError(f'{path}: length must be [M] ticks in 1..{T}')
    if 'assembly' in d:
        out['assembly'] = [str(s) for s in d['assembly']]
    for k in ('mode', 'gain', 'onset'):
        if k in d:
            out[k] = np.asarray(d[k], np.float64)
    return out


def prior(spec):
    """The fault prior with its defaults; BatchError on unknown keys or modes."""
    spec = dict(spec or {})
    unknown = sorted(set(spec) - set(PRIOR))
    if unknown:
        raise BatchError(f'faults.{unknown[0]}: unknown; keys: {", ".join(PRIOR)}')
    out = {**PRIOR, **spec}
    bad = sorted(set(out['modes']) - set(K.MODES[1:]))
    if bad:
        raise BatchError(f'faults.modes.{bad[0]}: unknown mode; modes: {", ".join(K.MODES[1:])}')
    return out


def draw_faults(pr, n, seed, m):
    """(mode [n], gain [n], onset [n]) for timeline m under seed: healthy with p_healthy, else one wheel (two with
    p_double) of a mode drawn by weight."""
    from fastsim.identity import rng
    r = rng(int(seed), 'modular.faults', int(m))
    mode, gain, onset = np.zeros(n), np.ones(n), np.zeros(n)
    if pr['p_healthy'] >= 1.0 or r.random() < pr['p_healthy']:
        return mode, gain, onset
    k = 2 if (n > 1 and r.random() < pr['p_double']) else 1
    names = sorted(pr['modes'])
    w = np.array([float(pr['modes'][q]) for q in names])
    for j in r.choice(n, size=k, replace=False):
        mode[j] = K.MODES.index(names[int(r.choice(len(names), p=w / w.sum()))])
        gain[j] = r.uniform(*pr['gain']) if mode[j] == K.WEAK else 1.0
        onset[j] = r.uniform(*pr['onset_s'])
    return mode, gain, onset


@dataclass
class Envs:
    """E envs: kernel rows, which timeline each replays, its latency, its labels and footprint."""
    p: np.ndarray
    which: np.ndarray
    delay: np.ndarray
    labels: list
    footprint: list = field(default_factory=list)

    @classmethod
    def build(cls, assemblies, prof, seeds, M, pr=None, dr=False, dt=0.005, logged=None, payload=None):
        """assemblies: one name for all, or [M]; envs timeline-major (e = m * len(seeds) + s)."""
        names = [assemblies] * M if isinstance(assemblies, str) else list(assemblies)
        draws = PR.randomize(prof, seeds) if dr and prof['dr'] else [prof] * len(seeds)
        built, rows, which, delay, labels, fps = {}, [], [], [], [], []
        pr = prior(pr)
        for m in range(M):
            for si, s in enumerate(seeds):
                key = (names[m], si if draws[si] is not prof else -1)
                if key not in built:
                    built[key] = AS.build(names[m], draws[si], payload)
                b = built[key]
                n = len(b['wheels'])
                if logged is not None and 'mode' in logged:
                    mode = logged['mode'][m][:n]
                    gain = logged.get('gain', np.ones_like(logged['mode']))[m][:n]
                    onset = logged.get('onset', np.zeros_like(logged['mode']))[m][:n]
                else:
                    mode, gain, onset = draw_faults(pr, n, s, m)
                rows.append(K.pack(b['wheels'], b['body'], {'mode': mode, 'gain': gain, 'onset': onset}))
                which.append(m)
                delay.append(PR.latency_steps(draws[si], dt))
                fps.append(b['footprint']['hull'])
                labels.append({'assembly': names[m], 'wheels': n, 'card': (prof.get('source') or {}).get('file'),
                               'variant': prof.get('variant'), 'latency_steps': delay[-1],
                               'fault_mode': [K.MODES[int(q)] for q in mode], 'fault_gain': [float(g) for g in gain],
                               'fault_onset_s': [float(o) for o in onset], 'dr': draws[si].get('drawn', {})})
        return cls(K.stack(rows), np.array(which, np.int64), np.array(delay, np.int64), labels, fps)


class Recording:
    """The robot-visible channels of a batched run, in crane.data.recorder's obs layout (obs T x C, channel names,
    t0, hz, labels), one row per env for crane.data.recorder.Store or crane.data.rows. ``spec`` is a ``record:``
    spec (crane.data.recorder.parse); its obs channels are groups of GROUPS (cmd duty counts speed imu pose vel)."""

    def __init__(self, spec, tick_hz, W):
        from crane.data import recorder
        self.spec = recorder.parse(spec)
        o = self.spec['obs']
        if not o:
            raise BatchError('record: no obs level; give record.obs.channels (cmd, duty, counts, speed, imu, pose, vel)')
        bad = [c for c in o['channels'] if c not in GROUPS]
        if bad:
            raise BatchError(f'record.obs: no channel {bad[0]!r}; channels: {", ".join(GROUPS)}')
        self.every = max(1, round(tick_hz / float(o['hz']))) if o['hz'] else 1
        self.hz = tick_hz / self.every
        self.dtype, self.window, self.W = o['dtype'], o['window'], W
        self.names = []
        for c in o['channels']:
            src, sub = GROUPS[c]
            self.names += [f'{c}.{q}' for q in sub] if sub else [f'{c}[{j}]' for j in range(W)]

    def arrays(self, out):
        """[E, T', C] of the channels, decimated and windowed; and the first sample's time."""
        parts = []
        for c in self.spec['obs']['channels']:
            src, sub = GROUPS[c]
            a = np.stack([out[q] for q in sub], -1) if src is None else out[src]
            parts.append(a)
        a = np.concatenate(parts, -1)[:, self.every - 1::self.every]
        t = (np.arange(a.shape[1]) + 1) * self.every / (self.every * self.hz)
        if self.window:
            keep = (t >= self.window[0] - 1e-9) & (t <= self.window[1] + 1e-9)
            a, t = a[:, keep], t[keep]
        return a.astype(self.dtype), float(t[0]) if len(t) else 0.0

    def meta(self):
        return {'obs': {'channels': self.names, 'hz': self.hz, 'dtype': self.dtype, 'window': self.window}}

    def env(self, a, t0, labels, length=None):
        """A recorder-like object for one env (Store.add(key, it); it.write(run_dir) writes record/obs.npz)."""
        return _Env(self, a, t0, labels, length)


class _Env:
    def __init__(self, rec, a, t0, labels, length):
        self.rec, self.a, self.t0, self.labels = rec, a if length is None else a[:length], t0, labels

    def arrays(self):
        import json
        from crane.data import rows as R
        return {'obs': {'obs': self.a, 'channels': np.array(self.rec.names, dtype=np.str_), 't0': np.float64(self.t0),
                        'hz': np.float64(self.rec.hz),
                        'labels': np.array(json.dumps(self.labels, sort_keys=True, default=R._plain))}}

    def rows(self, key):
        labels = {f'label.{k}': v for k, v in self.labels.items()}
        return {'obs': [{**key, 't0': self.t0, 'hz': self.rec.hz, **labels, 'obs': self.a}]}

    def meta(self):
        return self.rec.meta()

    def write(self, out):
        from pathlib import Path
        from crane.data import recorder, rows as R
        where = Path(out) / recorder.DIR
        where.mkdir(parents=True, exist_ok=True)
        (where / 'obs.npz').write_bytes(R.npz_bytes(self.arrays()['obs']))
        return {'obs': f'{recorder.DIR}/obs.npz'}
