"""RunRecord (``crane.judge.run/2``, ``/1`` still loads): one run from any tier, robot or simulator.

A record is a header (identity, tier, scenario key, clock, window), typed channels (columnar
arrays with a declared observability and optional per-sample sigma), typed events, pack-level
capabilities and a validity verdict. Metrics read channels by canonical name; a missing channel
raises ``Unobservable``, which the metric layer turns into status ``unscorable`` (never a zero).

Canonical channel names (domains may add their own, namespaced):
  pose.se2 (x, y, yaw) · pose.se3 (x, y, z, qx, qy, qz, qw) · pose.xz (x, z, pitch: planar aerial)
  twist.body (vx, vy, wz) · actuator.cmd · actuator.fb · plan.goal · authority
  sensor.<id>.<quantity> · module/<id>/<channel> (modular robots)

Standard channels of ``/2`` (judge/SCHEMA_V2.md; constructors below): truth.pose, truth.twist,
obs.pose, obs.twist, obs.stamp, cmd.requested, cmd.sent, cmd.applied, act.thrust, env.current,
env.wind. Every channel may carry an integer ``tick`` per sample. A source states what its backend
cannot expose with ``declare_absent(name, why)`` (an empty channel, observability ``absent``, with the
reason), which is distinct from a channel that was never logged.

``/2`` adds seven header blocks (backend, controller, calibration, clock_mode, code, evidence,
reset). A record with all seven is saved as ``/2``; one with none is saved as ``/1`` byte-for-byte as
before; one with some is refused. Loading ``/1`` gives explicit ``None`` for every block. ``save``
refuses a record whose ``obs.*`` arrays are the ``truth.*`` arrays (same object, shared memory or
equal bytes) unless the obs channel declares ``obs_is_truth`` with a reason.

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

SCHEMA_V1 = 'crane.judge.run/1'
SCHEMA_V2 = 'crane.judge.run/2'
SCHEMA = SCHEMA_V2              # the current writer version
SCHEMAS = (SCHEMA_V1, SCHEMA_V2)
OBSERVABILITY = ('exact', 'estimated', 'surveyed', 'commanded', 'absent')
V2_BLOCKS = ('backend', 'controller', 'calibration', 'clock_mode', 'code', 'evidence', 'reset')

LAYERS = ('fast', 'integrated', 'physical')
CLOCK_MODES = ('fixed_step', 'lockstep', 'realtime', 'recorded')
EVIDENCE_KINDS = ('confirmatory', 'exploratory', 'demo', 'replay')


class EV:
    """Event vocabulary of the docking task (payload keys in comments)."""
    EPISODE_START = 'episode_start'
    GATE_CROSS = 'gate_cross'          # {y}
    CONTACT = 'contact'                # {with}
    DOCK_ENTER = 'dock_enter'
    DOCK_HOLD_START = 'dock_hold_start'
    DOCKED = 'docked'
    TIMEOUT = 'timeout'
    LINK_LOST = 'link_lost'
    RESET = 'reset'
    ABORT = 'abort'                    # {why}


class END:
    """Terminal reasons (``Header.window[2]``)."""
    DOCKED = 'docked'
    TIMEOUT = 'timeout'
    ABORT = 'abort'
    LINK_LOST = 'link_lost'
    INFRA_ERROR = 'infra_error'
    TRACK_END = 'track_end'


EVENTS = tuple(v for k, v in vars(EV).items() if k.isupper())
TERMINAL = tuple(v for k, v in vars(END).items() if k.isupper())


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
    note: str = ''              # why the channel is absent (``declare_absent``), or other source note
    obs_is_truth: str = ''      # obs.* only: non-empty = the source declares this obs IS truth, and why

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
    tick: Optional[np.ndarray] = None    # (N,) int64 backend tick per sample; None = no ticks

    @staticmethod
    def of(spec, t, v, sigma=None, gaps=(), tick=None):
        t = np.asarray(t, float).reshape(-1)
        v = np.asarray(v, float)
        if v.size != len(t) * len(spec.fields):
            cols = v.size / len(t) if len(t) else v.size
            raise ValueError(f'{spec.name}: values have {cols:g} columns, spec {len(spec.fields)}')
        v = v.reshape(len(t), len(spec.fields))
        if sigma is not None:
            sigma = np.asarray(sigma, float)
        return Channel(spec, t, v, sigma, tuple(tuple(g) for g in gaps), _ticks(tick, len(t), spec.name))

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
        return Channel(self.spec, self.t[m], self.v[m], sig, self.gaps,
                       None if self.tick is None else self.tick[m])


def _ticks(tick, n, name):
    if tick is None:
        return None
    a = np.asarray(tick)
    if a.ndim != 1 or len(a) != n:
        raise ValueError(f'{name}: {a.size} ticks for {n} samples')
    if a.dtype.kind == 'f':
        if not np.all(np.isfinite(a)) or np.any(a != np.round(a)):
            raise ValueError(f'{name}: ticks must be integers')
    elif a.dtype.kind not in 'iu':
        raise ValueError(f'{name}: ticks must be integers, got {a.dtype}')
    return a.astype(np.int64)


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


# --------------------------------------------------------------------- /2 header blocks
# Each block validates its own types on construction (ValueError). Lists become tuples so equal
# blocks compare equal after a JSON round trip.

def _need(cond, what):
    if not cond:
        raise ValueError(what)


def _opt_str(x):
    return x is None or isinstance(x, str)


def _opt_num(x, lo=None, strict=False):
    if x is None:
        return True
    if isinstance(x, bool) or not isinstance(x, (int, float, np.integer, np.floating)) or not math.isfinite(x):
        return False
    return lo is None or (x > lo if strict else x >= lo)


def _strs(block, name, xs):
    xs = tuple(xs)
    _need(all(isinstance(s, str) for s in xs), f'{block}.{name}: list of strings')
    return xs


@dataclass(frozen=True)
class Backend:
    layer: str                       # fast | integrated | physical
    name: str
    build: Optional[str] = None
    mechanisms: tuple = ()           # free vocabulary, e.g. 'thrust.lag.first_order'
    flags: Mapping[str, Any] = field(default_factory=dict)   # backend-reported faults (judge decides)

    def __post_init__(self):
        _need(self.layer in LAYERS, f'backend.layer {self.layer!r} not in {LAYERS}')
        _need(isinstance(self.name, str) and self.name, 'backend.name: non-empty string')
        _need(_opt_str(self.build), 'backend.build: string or null')
        object.__setattr__(self, 'mechanisms', _strs('backend', 'mechanisms', self.mechanisms))
        _need(isinstance(self.flags, Mapping) and all(isinstance(k, str) for k in self.flags),
              'backend.flags: mapping of flag name -> detail')
        object.__setattr__(self, 'flags', dict(sorted(self.flags.items())))


@dataclass(frozen=True)
class Controller:
    id: str
    library: Optional[str] = None
    spec_sha: Optional[str] = None
    params: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        _need(isinstance(self.id, str) and self.id, 'controller.id: non-empty string')
        _need(_opt_str(self.library) and _opt_str(self.spec_sha), 'controller.library/spec_sha: string or null')
        _need(isinstance(self.params, Mapping), 'controller.params: mapping')
        object.__setattr__(self, 'params', dict(self.params))


@dataclass(frozen=True)
class Calibration:
    twin: Optional[str] = None
    arm: Optional[str] = None
    evidence_sha: Optional[str] = None
    lineage: tuple = ()

    def __post_init__(self):
        _need(all(_opt_str(x) for x in (self.twin, self.arm, self.evidence_sha)),
              'calibration.twin/arm/evidence_sha: string or null')
        object.__setattr__(self, 'lineage', _strs('calibration', 'lineage', self.lineage))


@dataclass(frozen=True)
class ClockMode:
    mode: str                        # fixed_step | lockstep | realtime | recorded
    dt_s: Optional[float] = None     # plant tick
    time_scale: Optional[float] = None
    rtf: Optional[float] = None      # measured real-time factor
    episode: Optional[int] = None

    def __post_init__(self):
        _need(self.mode in CLOCK_MODES, f'clock_mode.mode {self.mode!r} not in {CLOCK_MODES}')
        _need(_opt_num(self.dt_s, 0, strict=True), 'clock_mode.dt_s: positive number or null')
        _need(_opt_num(self.time_scale, 0, strict=True), 'clock_mode.time_scale: positive number or null')
        _need(_opt_num(self.rtf, 0), 'clock_mode.rtf: non-negative number or null')
        _need(self.episode is None or (isinstance(self.episode, (int, np.integer))
                                       and not isinstance(self.episode, bool)), 'clock_mode.episode: int or null')


@dataclass(frozen=True)
class Code:
    repo: str
    commit: str
    dirty: bool
    files_sha: Mapping[str, str] = field(default_factory=dict)

    def __post_init__(self):
        _need(isinstance(self.repo, str) and isinstance(self.commit, str), 'code.repo/commit: strings')
        _need(isinstance(self.dirty, bool), 'code.dirty: bool')
        _need(isinstance(self.files_sha, Mapping) and all(
            isinstance(k, str) and isinstance(v, str) for k, v in self.files_sha.items()),
            'code.files_sha: mapping path -> sha')
        object.__setattr__(self, 'files_sha', dict(sorted(self.files_sha.items())))


@dataclass(frozen=True)
class Evidence:
    kind: str                        # confirmatory | exploratory | demo | replay
    prereg: Optional[str] = None
    seed_class: Optional[str] = None

    def __post_init__(self):
        _need(self.kind in EVIDENCE_KINDS, f'evidence.kind {self.kind!r} not in {EVIDENCE_KINDS}')
        _need(_opt_str(self.prereg) and _opt_str(self.seed_class), 'evidence.prereg/seed_class: string or null')


@dataclass(frozen=True)
class Reset:
    method: Optional[str] = None     # e.g. 'teleport.rest' (a method ending in '.rest' claims rest)
    verified: Optional[bool] = None  # True = the source checked the start state
    pose_error: Optional[tuple] = None   # (dx, dy, dyaw) measured by the source, or null

    def __post_init__(self):
        _need(_opt_str(self.method), 'reset.method: string or null')
        _need(self.verified is None or isinstance(self.verified, bool), 'reset.verified: bool or null')
        if self.pose_error is not None:
            pe = tuple(float(x) for x in self.pose_error)
            _need(all(math.isfinite(x) for x in pe), 'reset.pose_error: finite numbers')
            object.__setattr__(self, 'pose_error', pe)

    @property
    def claims_rest(self):
        return bool(self.method) and (self.method == 'rest' or self.method.endswith('.rest'))


BLOCK_TYPES = {'backend': Backend, 'controller': Controller, 'calibration': Calibration,
               'clock_mode': ClockMode, 'code': Code, 'evidence': Evidence, 'reset': Reset}


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
    # /2 blocks; None = not given (every /1 record). /2 save requires all seven.
    backend: Optional[Backend] = None
    controller: Optional[Controller] = None
    calibration: Optional[Calibration] = None
    clock_mode: Optional[ClockMode] = None
    code: Optional[Code] = None
    evidence: Optional[Evidence] = None
    reset: Optional[Reset] = None

    def __post_init__(self):
        for name, typ in BLOCK_TYPES.items():
            b = getattr(self, name)
            if b is not None and not isinstance(b, typ):
                raise ValueError(f'header.{name}: expected {typ.__name__} or None, got {type(b).__name__}')

    @property
    def missing_blocks(self):
        return tuple(b for b in V2_BLOCKS if getattr(self, b) is None)

    @property
    def schema(self):
        """SCHEMA_V2 with every block, SCHEMA_V1 with none, None when partial (unsavable)."""
        m = self.missing_blocks
        return SCHEMA_V2 if not m else SCHEMA_V1 if len(m) == len(V2_BLOCKS) else None


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


_SPEC_OPTIONAL = ('note', 'obs_is_truth')     # written only when non-empty (keeps /1 bytes unchanged)


def _spec_json(c: Channel):
    d = asdict(c.spec)
    for k in _SPEC_OPTIONAL:
        if not d[k]:
            del d[k]
    d |= {'gaps': c.gaps, 'n': len(c), 'has_sigma': c.sigma is not None}
    if c.tick is not None:
        d['has_tick'] = True
    return d


def to_json(rec: RunRecord, schema=None):
    schema = schema or _schema_for_save(rec)
    header = asdict(rec.header)
    if schema == SCHEMA_V1:
        for b in V2_BLOCKS:
            header.pop(b)
    specs = {n: _spec_json(c) for n, c in sorted(rec.channels.items())}
    return _clean({'schema': schema, 'header': header, 'channels': specs,
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


class ObsIsTruth(ValueError):
    """An obs.* channel carries the evaluator's truth arrays without declaring it."""


def obs_truth_aliases(rec: RunRecord):
    """(obs, truth, how) for every obs.* channel whose times or values ARE its truth.* counterpart's
    (same object, shared memory, or byte-equal) and that does not declare ``obs_is_truth``."""
    out = []
    for name, ch in sorted(rec.channels.items()):
        if not name.startswith('obs.') or ch.spec.obs_is_truth:
            continue
        tr = rec.channels.get('truth.' + name[4:])
        if tr is None or len(tr) == 0 or len(ch) == 0:
            continue
        a, b = ch.v, tr.v
        if a is b:
            how = 'same array object'
        elif np.shares_memory(a, b):
            how = 'shared memory'
        elif a.shape == b.shape and a.dtype == b.dtype and a.tobytes() == b.tobytes():
            how = 'byte-equal values'
        else:
            continue
        out.append((name, tr.spec.name, how))
    return out


def _schema_for_save(rec: RunRecord):
    s = rec.header.schema
    if s is None:
        raise ValueError(f'{rec.run_id}: /2 header incomplete, missing {list(rec.header.missing_blocks)} '
                         f'(give every block; unknown fields inside a block are null)')
    return s


def save(rec: RunRecord, out, schema=None) -> Path:
    """Write ``record.json`` + ``channels.npz``. ``schema=None`` writes /2 when every header block is
    present and /1 when none is; ``schema=SCHEMA_V2`` insists on /2. Refuses undeclared obs-is-truth."""
    auto = _schema_for_save(rec)
    if schema is not None and schema != auto:
        if schema == SCHEMA_V2:
            raise ValueError(f'{rec.run_id}: /2 requires header blocks {list(rec.header.missing_blocks)}')
        if schema not in SCHEMAS:
            raise ValueError(f'unknown schema {schema!r}')
        raise ValueError(f'{rec.run_id}: a record with /2 header blocks cannot be saved as {schema}')
    bad = obs_truth_aliases(rec)
    if bad:
        raise ObsIsTruth(f'{rec.run_id}: ' + '; '.join(f'{o} is {t} ({how})' for o, t, how in bad)
                         + ' -- evaluator truth must not silently become an observation; build the obs '
                           'channel with obs_is_truth="<reason>" if the backend really feeds truth')
    for name, ch in rec.channels.items():
        if ch.spec.observability == 'absent' and len(ch):
            raise ValueError(f'{rec.run_id}: {name} is declared absent but has {len(ch)} samples')
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    (out / 'record.json').write_text(json.dumps(to_json(rec, auto), sort_keys=True, indent=1) + '\n')
    arrays = {}
    for name, ch in rec.channels.items():
        arrays[f'{name}/t'] = ch.t
        arrays[f'{name}/v'] = ch.v
        if ch.sigma is not None:
            arrays[f'{name}/sigma'] = ch.sigma
        if ch.tick is not None:
            arrays[f'{name}/tick'] = ch.tick
    (out / 'channels.npz').write_bytes(_npz_bytes(arrays))
    return out


def _block(name, d):
    typ = BLOCK_TYPES[name]
    if d is None:
        return None
    if not isinstance(d, Mapping):
        raise ValueError(f'header.{name}: expected an object, got {type(d).__name__}')
    return typ(**d)


def load(path) -> RunRecord:
    path = Path(path)
    doc = json.loads((path / 'record.json').read_text())
    schema = doc.get('schema')
    if schema not in SCHEMAS:
        raise ValueError(f'{path}: schema {schema!r}, expected one of {SCHEMAS}')
    z = np.load(path / 'channels.npz')
    channels = {}
    for name, spec in doc['channels'].items():
        gaps, has_sigma = spec.pop('gaps'), spec.pop('has_sigma')
        has_tick = spec.pop('has_tick', False)
        spec.pop('n')
        cs = ChannelSpec(**{k: tuple(v) if isinstance(v, list) else v for k, v in spec.items()})
        channels[name] = Channel(cs, z[f'{name}/t'], z[f'{name}/v'],
                                 z[f'{name}/sigma'] if has_sigma else None, tuple(map(tuple, gaps)),
                                 z[f'{name}/tick'] if has_tick else None)
    h = doc['header']
    if schema == SCHEMA_V2:
        missing = [b for b in V2_BLOCKS if h.get(b) is None]
        if missing:
            raise ValueError(f'{path}: {SCHEMA_V2} record without header blocks {missing}')
    blocks = {b: _block(b, h.get(b)) for b in V2_BLOCKS}
    header = Header(**{**h, 'robot': Robot(**h['robot']), 'scenario': Scenario(**h['scenario']),
                       'clock': Clock(**h['clock']), 'window': tuple(h['window']), **blocks})
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


# ------------------------------------------------------------------ /2 standard channels

@dataclass(frozen=True)
class StandardChannel:
    fields: tuple
    units: tuple
    frame: str
    observability: tuple        # allowed values; the first is the default
    angles: tuple = ()


_Q = tuple(f'q{i}' for i in range(4))
STANDARD = {
    'truth.pose': StandardChannel(('x', 'y', 'yaw'), ('m', 'm', 'rad'), 'scenario', ('exact', 'surveyed'), ('yaw',)),
    'truth.twist': StandardChannel(('u', 'v', 'r'), ('m/s', 'm/s', 'rad/s'), 'body', ('exact', 'estimated')),
    'obs.pose': StandardChannel(('x', 'y', 'yaw'), ('m', 'm', 'rad'), 'scenario', ('estimated', 'exact'), ('yaw',)),
    'obs.twist': StandardChannel(('u', 'v', 'r'), ('m/s', 'm/s', 'rad/s'), 'body', ('estimated', 'exact')),
    'obs.stamp': StandardChannel(('t_acq', 't_recv'), ('s', 's'), 'time', ('exact',)),
    'cmd.requested': StandardChannel(_Q, ('1',) * 4, 'actuator', ('commanded',)),
    'cmd.sent': StandardChannel(tuple(f'p{i}' for i in range(4)) + ('t_send',), ('us',) * 4 + ('s',),
                                'actuator', ('commanded',)),
    'cmd.applied': StandardChannel(_Q + ('t_apply',), ('1',) * 4 + ('s',), 'actuator', ('exact',)),
    'act.thrust': StandardChannel(tuple(f'f{i}' for i in range(4)), ('N',) * 4, 'actuator', ('exact',)),
    'env.current': StandardChannel(('x', 'y'), ('m/s', 'm/s'), 'scenario', ('exact',)),
    'env.wind': StandardChannel(('x', 'y'), ('m/s', 'm/s'), 'scenario', ('exact',)),
}
STANDARD_NAMES = tuple(STANDARD)


def standard_spec(name, observability=None, note='', obs_is_truth='', rate_hz=None):
    sc = STANDARD[name]
    obs = observability or sc.observability[0]
    if obs != 'absent' and obs not in sc.observability:
        raise ValueError(f'{name}: observability {obs!r} not in {sc.observability}')
    if obs_is_truth and not name.startswith('obs.'):
        raise ValueError(f'{name}: obs_is_truth applies to obs.* channels only')
    return ChannelSpec(name, sc.fields, sc.units, sc.frame, obs, rate_hz, sc.angles, note, obs_is_truth)


def standard(name, t, values, *, tick=None, observability=None, sigma=None, gaps=(), note='',
             obs_is_truth='', rate_hz=None) -> Channel:
    """A standard /2 channel: ``values`` is (N, F) in the field order of ``STANDARD[name]``."""
    if name not in STANDARD:
        raise ValueError(f'{name!r} is not a standard channel ({", ".join(STANDARD)})')
    return Channel.of(standard_spec(name, observability, note, obs_is_truth, rate_hz), t, values, sigma, gaps, tick)


def _cols(*cols):
    return np.column_stack([np.asarray(c, float).reshape(-1) for c in cols])


def truth_pose(t, x, y, yaw, *, tick=None, observability='exact', sigma=None):
    return standard('truth.pose', t, _cols(x, y, yaw), tick=tick, observability=observability, sigma=sigma)


def truth_twist(t, u, v, r, *, tick=None, observability='exact', sigma=None):
    """Body-frame surge u, sway v, yaw rate r."""
    return standard('truth.twist', t, _cols(u, v, r), tick=tick, observability=observability, sigma=sigma)


def obs_pose(t, x, y, yaw, *, tick=None, sigma=None, observability='estimated', obs_is_truth=''):
    """What the controller received; ``t`` = when it used it (acquisition time goes in obs.stamp)."""
    return standard('obs.pose', t, _cols(x, y, yaw), tick=tick, sigma=sigma, observability=observability,
                    obs_is_truth=obs_is_truth)


def obs_twist(t, u, v, r, *, tick=None, sigma=None, observability='estimated', obs_is_truth=''):
    return standard('obs.twist', t, _cols(u, v, r), tick=tick, sigma=sigma, observability=observability,
                    obs_is_truth=obs_is_truth)


def obs_stamp(t, t_acq, t_recv, *, tick=None):
    """Per observation the controller used: row time ``t`` (use), ``t_acq`` (acquired), ``t_recv``."""
    return standard('obs.stamp', t, _cols(t_acq, t_recv), tick=tick)


def cmd_requested(t, q, *, tick=None):
    """``q``: (N, 4) normalised commands, at the time the controller asked."""
    return standard('cmd.requested', t, np.asarray(q, float), tick=tick)


def cmd_sent(t, p_us, t_send, *, tick=None):
    return standard('cmd.sent', t, np.column_stack([np.asarray(p_us, float), np.asarray(t_send, float)]), tick=tick)


def cmd_applied(t, q, t_apply, *, tick=None):
    return standard('cmd.applied', t, np.column_stack([np.asarray(q, float), np.asarray(t_apply, float)]), tick=tick)


def act_thrust(t, f_n, *, tick=None):
    return standard('act.thrust', t, np.asarray(f_n, float), tick=tick)


def env_current(t, x, y, *, tick=None):
    return standard('env.current', t, _cols(x, y), tick=tick)


def env_wind(t, x, y, *, tick=None):
    return standard('env.wind', t, _cols(x, y), tick=tick)


def declare_absent(name, why, fields=None, units=None) -> Channel:
    """The backend cannot expose ``name`` (stated, with ``why``), as opposed to forgetting to log it.

    Put the result in ``channels[name]``: ``rec.has(name)`` is False (metrics see Unobservable) and
    ``judge.validity`` counts it as accounted for."""
    if not isinstance(why, str) or not why.strip():
        raise ValueError(f'{name}: declare_absent needs a reason')
    if name in STANDARD:
        spec = standard_spec(name, 'absent', note=why)
    else:
        fields = tuple(fields or ())
        spec = ChannelSpec(name, fields, tuple(units or ('',) * len(fields)), observability='absent', note=why)
    return Channel(spec, np.zeros(0), np.zeros((0, len(spec.fields))))


def declared_absent(rec: RunRecord):
    """{name: why} for every channel the source declared absent."""
    return {n: c.spec.note for n, c in sorted(rec.channels.items()) if c.spec.observability == 'absent'}
