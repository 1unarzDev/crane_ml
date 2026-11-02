"""RunRecord (``crane.judge.run/1``): one run from any tier, robot or simulator.

A record is a header (identity, tier, scenario key, clock, window), typed channels (columnar
arrays with a declared observability and optional per-sample sigma), typed events, pack-level
capabilities and a validity verdict. Metrics read channels by canonical name; a missing channel
raises ``Unobservable``, which the metric layer turns into status ``unscorable`` (never a zero).

Canonical channel names (domains may add their own, namespaced):
  pose.se2 (x, y, yaw) · pose.se3 (x, y, z, qx, qy, qz, qw) · pose.xz (x, z, pitch: planar aerial)
  twist.body (vx, vy, wz) · actuator.cmd · actuator.fb · plan.goal · authority
  sensor.<id>.<quantity> · module/<id>/<channel> (modular robots)

On disk a record is ``record.json`` (sorted keys, rounded floats) plus ``channels.npz`` written with
fixed zip timestamps, so equal runs produce byte-identical files (G1 determinism checks hash them).
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, replace
import hashlib
import io
import json
import math
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence
import zipfile

import numpy as np

SCHEMA = 'crane.judge.run/1'
OBSERVABILITY = ('exact', 'estimated', 'surveyed', 'commanded', 'absent')


class Unobservable(LookupError):
    """The record has no such channel (or it is declared absent)."""


@dataclass(frozen=True)
class ChannelSpec:
    name: str
    fields: tuple
    units: tuple
    frame: str = 'world'
    observability: str = 'exact'
    rate_hz: Optional[float] = None
    angles: tuple = ()          # names of fields that wrap at +-pi

    def __post_init__(self):
        if len(self.fields) != len(self.units):
            raise ValueError(f'{self.name}: {len(self.fields)} fields but {len(self.units)} units')
        if self.observability not in OBSERVABILITY:
            raise ValueError(f'{self.name}: observability {self.observability!r}')


@dataclass(frozen=True)
class Channel:
    spec: ChannelSpec
    t: np.ndarray               # (N,) seconds on the record clock, non-decreasing
    v: np.ndarray               # (N, F)
    sigma: Optional[np.ndarray] = None   # (N, F) or (F,); None = exact
    gaps: tuple = ()            # ((t0, t1), ...) declared holes

    @staticmethod
    def of(spec, t, v, sigma=None, gaps=()):
        t = np.asarray(t, float).reshape(-1)
        v = np.asarray(v, float).reshape(len(t), -1)
        if v.shape[1] != len(spec.fields):
            raise ValueError(f'{spec.name}: values have {v.shape[1]} columns, spec {len(spec.fields)}')
        if sigma is not None:
            sigma = np.asarray(sigma, float)
        return Channel(spec, t, v, sigma, tuple(tuple(g) for g in gaps))

    def __len__(self):
        return len(self.t)

    def col(self, name):
        return self.v[:, self.spec.fields.index(name)]

    def sigma_of(self, name):
        """Per-sample 1-sigma of one field (zeros when exact)."""
        k = self.spec.fields.index(name)
        if self.sigma is None:
            return np.zeros(len(self.t))
        s = self.sigma
        return np.full(len(self.t), float(s[k])) if s.ndim == 1 else s[:, k]

    def at(self, times):
        """Values interpolated at ``times`` (angles unwrapped first, rewrapped after)."""
        times = np.asarray(times, float)
        out = np.empty((len(times), self.v.shape[1]))
        for k, name in enumerate(self.spec.fields):
            col = self.v[:, k]
            if name in self.spec.angles:
                out[:, k] = wrap(np.interp(times, self.t, np.unwrap(col)))
            else:
                out[:, k] = np.interp(times, self.t, col)
        return out

    def window(self, t0, t1):
        m = (self.t >= t0) & (self.t <= t1)
        sig = self.sigma if self.sigma is None or self.sigma.ndim == 1 else self.sigma[m]
        return Channel(self.spec, self.t[m], self.v[m], sig, self.gaps)


@dataclass(frozen=True)
class Event:
    t: float
    kind: str                   # from the pack's vocabulary: goal.sent, decision, contact, ...
    actor: str = ''
    payload: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class Robot:
    id: str
    model: str = ''
    morphology: Optional[Mapping[str, Any]] = None   # modular robots: {modules: [...], links: [...]}
    plant_card: Optional[str] = None                 # 'card-id@version'


@dataclass(frozen=True)
class Scenario:
    pair_key: Optional[str] = None
    truth_sha256: Optional[str] = None
    seed: Optional[int] = None
    family: Optional[str] = None
    condition: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class Clock:
    base: str = 'sim'           # 'sim' | 'wall'
    anchor: Optional[str] = None   # event or field the record clock is aligned on
    offset_s: float = 0.0       # record t = source t - offset_s
    residual_s: Optional[float] = None   # alignment residual (real sources), None = not measured


@dataclass(frozen=True)
class Header:
    run_id: str
    tier: str
    source: str
    domain: str
    robot: Robot
    task_id: str = ''
    scenario: Scenario = field(default_factory=Scenario)
    provenance: Mapping[str, Any] = field(default_factory=dict)
    clock: Clock = field(default_factory=Clock)
    window: tuple = (0.0, 0.0, '')   # (t0, t1, end reason)
    source_version: str = '1'


@dataclass(frozen=True)
class Validity:
    valid: bool = True
    flags: Mapping[str, Any] = field(default_factory=dict)
    excluded_from: frozenset = frozenset()   # e.g. {'G3', 'G4'}; reports always count exclusions


@dataclass(frozen=True)
class RunRecord:
    header: Header
    channels: Mapping[str, Channel]
    events: tuple = ()
    caps: Mapping[str, Any] = field(default_factory=dict)
    validity: Validity = field(default_factory=Validity)
    truth: Mapping[str, Any] = field(default_factory=dict)
    attachments: Mapping[str, Any] = field(default_factory=dict)  # in-memory pack payloads, not saved

    @property
    def run_id(self):
        return self.header.run_id

    @property
    def tier(self):
        return self.header.tier

    def has(self, name):
        ch = self.channels.get(name)
        return ch is not None and ch.spec.observability != 'absent' and len(ch) > 0

    def channel(self, name) -> Channel:
        if not self.has(name):
            raise Unobservable(f'{self.run_id}: no channel {name!r}')
        return self.channels[name]

    def events_of(self, *kinds):
        return [e for e in self.events if e.kind in kinds or any(
            k.endswith('*') and e.kind.startswith(k[:-1]) for k in kinds)]

    @property
    def duration(self):
        t0, t1 = self.header.window[:2]
        return float(t1 - t0)

    def with_validity(self, **kw):
        return replace(self, validity=replace(self.validity, **kw))


def wrap(a):
    return (np.asarray(a) + math.pi) % (2 * math.pi) - math.pi


# ------------------------------------------------------------------------------------ disk

def _clean(x, nd=9):
    """JSON-ready, deterministic: floats rounded, tuples to lists, sets sorted."""
    if isinstance(x, float):
        return None if not math.isfinite(x) else round(x, nd)
    if isinstance(x, (np.floating,)):
        return _clean(float(x), nd)
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, Mapping):
        return {str(k): _clean(v, nd) for k, v in x.items()}
    if isinstance(x, (set, frozenset)):
        return sorted(_clean(v, nd) for v in x)
    if isinstance(x, (list, tuple)):
        return [_clean(v, nd) for v in x]
    if isinstance(x, np.ndarray):
        return _clean(x.tolist(), nd)
    return x


def header_dict(rec: RunRecord):
    return _clean(asdict(rec.header))


def to_json(rec: RunRecord):
    specs = {n: asdict(c.spec) | {'gaps': c.gaps, 'n': len(c), 'has_sigma': c.sigma is not None}
             for n, c in sorted(rec.channels.items())}
    return _clean({'schema': SCHEMA, 'header': asdict(rec.header), 'channels': specs,
                   'events': [asdict(e) for e in rec.events], 'caps': dict(rec.caps),
                   'validity': asdict(rec.validity), 'truth': dict(rec.truth)})


def _npz_bytes(arrays: Mapping[str, np.ndarray]) -> bytes:
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, 'w', zipfile.ZIP_DEFLATED) as z:
        for name in sorted(arrays):
            info = zipfile.ZipInfo(name + '.npy', date_time=(1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            arr = io.BytesIO()
            np.lib.format.write_array(arr, np.ascontiguousarray(arrays[name]), allow_pickle=False)
            z.writestr(info, arr.getvalue())
    return buf.getvalue()


def save(rec: RunRecord, out) -> Path:
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    (out / 'record.json').write_text(json.dumps(to_json(rec), sort_keys=True, indent=1) + '\n')
    arrays = {}
    for name, ch in rec.channels.items():
        arrays[f'{name}/t'] = ch.t
        arrays[f'{name}/v'] = ch.v
        if ch.sigma is not None:
            arrays[f'{name}/sigma'] = ch.sigma
    (out / 'channels.npz').write_bytes(_npz_bytes(arrays))
    return out


def load(path) -> RunRecord:
    path = Path(path)
    doc = json.loads((path / 'record.json').read_text())
    if doc.get('schema') != SCHEMA:
        raise ValueError(f'{path}: schema {doc.get("schema")!r}, expected {SCHEMA}')
    z = np.load(path / 'channels.npz')
    channels = {}
    for name, spec in doc['channels'].items():
        gaps, has_sigma = spec.pop('gaps'), spec.pop('has_sigma')
        spec.pop('n')
        cs = ChannelSpec(**{k: tuple(v) if isinstance(v, list) else v for k, v in spec.items()})
        channels[name] = Channel(cs, z[f'{name}/t'], z[f'{name}/v'],
                                 z[f'{name}/sigma'] if has_sigma else None, tuple(map(tuple, gaps)))
    h = doc['header']
    header = Header(**{**h, 'robot': Robot(**h['robot']), 'scenario': Scenario(**h['scenario']),
                       'clock': Clock(**h['clock']), 'window': tuple(h['window'])})
    v = doc['validity']
    return RunRecord(header, channels, tuple(Event(**e) for e in doc['events']), doc['caps'],
                     Validity(v['valid'], v['flags'], frozenset(v['excluded_from'])), doc['truth'])


def digest(path) -> str:
    """sha256 over a saved record's two files (G1: equal runs, equal digest)."""
    h = hashlib.sha256()
    for name in ('record.json', 'channels.npz'):
        h.update((Path(path) / name).read_bytes())
    return h.hexdigest()


def se2_channel(t, x, y, yaw, observability='exact', sigma=None, frame='world', name='pose.se2'):
    """The common planar pose channel."""
    spec = ChannelSpec(name, ('x', 'y', 'yaw'), ('m', 'm', 'rad'), frame, observability,
                       angles=('yaw',))
    return Channel.of(spec, t, np.column_stack([x, y, yaw]), sigma)


def twist_from_pose(ch: Channel, name='twist.body'):
    """Body-frame (vx, vy, wz) by central differences of a pose.se2 channel (estimated)."""
    t, x, y = ch.t, ch.col('x'), ch.col('y')
    yaw = np.unwrap(ch.col('yaw'))
    if len(t) < 3:
        v = np.zeros((len(t), 3))
    else:
        dx, dy, dyaw = (np.gradient(a, t) for a in (x, y, yaw))
        c, s = np.cos(yaw), np.sin(yaw)
        v = np.column_stack([c * dx + s * dy, -s * dx + c * dy, dyaw])
    spec = ChannelSpec(name, ('vx', 'vy', 'wz'), ('m/s', 'm/s', 'rad/s'), 'body.flu', 'estimated')
    return Channel.of(spec, t, v)


def dedupe_time(t, *cols):
    """Drop samples whose time does not increase (keeps the first)."""
    t = np.asarray(t, float)
    keep = np.concatenate([[True], np.diff(t) > 0]) if len(t) else np.zeros(0, bool)
    return (t[keep],) + tuple(np.asarray(c)[keep] for c in cols)


def records_by(records: Sequence[RunRecord], key):
    out = {}
    for r in records:
        out.setdefault(key(r), []).append(r)
    return out
