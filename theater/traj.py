"""crane-traj/1 (``.ctraj``): one run's poses and events, small enough to keep forever.

A domain-agnostic trajectory log: fixed-rate samples of named numeric columns (per agent,
``<agent>.x`` ...) plus a list of timed events and a free-form JSON header. Stdlib only.

Layout (little-endian)::

    "CTRJ" u16 version=1 u16 flags=0
    u32 header_len   header (UTF-8 JSON, sorted keys)
    u16 n_columns    per column: u8 name_len, name (UTF-8), f64 scale, f64 offset
    u32 ticks  u32 block_ticks  u32 n_blocks
    per block: u32 first_tick, u32 n, u32 byte_len           (the index: seek without decoding)
    blocks           each raw-deflate(column after column: zigzag varint of q[i] - q[i-1], q[-1] = 0)
    u32 events_len   raw-deflate(JSON lines, sorted keys), each {"t": s from t0, "kind": ..., ...}
    u32 crc32        of every byte above

A value is ``q * scale + offset`` with ``q = floor((v - offset) / scale + 0.5)``; positions are
in metres, angles in radians, sample i is at ``t0_s + i / hz``. Raw deflate (no zlib header)
matches .NET's DeflateStream, so the Unity writer and this reader share one format.

Header keys this module fills: ``schema``, ``hz``, ``t0_s``, ``ticks``, ``duration_s``,
``columns``, ``box`` (the pose extent). Writers add ``robot {model, agents}``, ``pose_dims``
(``6dof`` or ``planar``), ``frame``, ``course``, ``build``, ``water``, ``seed`` and, once the
run is scored, ``outcome``, ``score``, ``progress``, ``valid``, ``study``, ``iteration``,
``params`` (see ``theater.meta``).
"""

from __future__ import annotations

import io
import json
import math
import struct
import zlib
from dataclasses import dataclass, field
from pathlib import Path

SCHEMA = 'crane-traj/1'
MAGIC = b'CTRJ'
VERSION = 1
BLOCK_S = 30.0
OUTCOMES = ('finished', 'failed', 'timeout', 'invalid', 'unknown')

M = 0.01                         # position quantum: 1 cm
RAD = math.radians(0.01)         # angle quantum: 0.01 deg
SCALES = {'x': M, 'y': M, 'z': M, 'yaw': RAD, 'pitch': RAD, 'roll': RAD, 'speed': 0.001}
POSE6 = ('x', 'y', 'z', 'yaw', 'pitch', 'roll')
PLANAR = ('x', 'y', 'yaw')


class TrajError(ValueError):
    pass


def scale_of(name: str) -> float:
    """The default quantum of a column by its last dotted part (1 mm-ish for anything unknown)."""
    return SCALES.get(name.rsplit('.', 1)[-1], 0.001)


def _zigzag(n: int) -> int:
    return (n << 1) ^ (n >> 63) if -(1 << 63) <= n < (1 << 63) else _overflow(n)


def _overflow(n):
    raise TrajError(f'quantized delta {n} does not fit 64 bits')


def _unzigzag(z: int) -> int:
    return (z >> 1) ^ -(z & 1)


def _varint(out: bytearray, z: int) -> None:
    while z >= 0x80:
        out.append((z & 0x7F) | 0x80)
        z >>= 7
    out.append(z)


def _deflate(data: bytes) -> bytes:
    c = zlib.compressobj(9, zlib.DEFLATED, -15)
    return c.compress(data) + c.flush()


def _inflate(data: bytes) -> bytes:
    return zlib.decompress(data, -15)


def quantize(v: float, scale: float, offset: float = 0.0) -> int:
    if not math.isfinite(v):
        raise TrajError(f'non-finite value {v!r}')
    return math.floor((v - offset) / scale + 0.5)


def dumps_json(obj) -> str:
    return json.dumps(obj, sort_keys=True, separators=(',', ':'), allow_nan=False)


@dataclass
class Column:
    name: str
    scale: float
    offset: float = 0.0


@dataclass
class Traj:
    header: dict
    columns: dict                 # name -> list[float] (dequantized)
    events: list = field(default_factory=list)
    specs: list = field(default_factory=list)   # [Column] in file order

    @property
    def hz(self) -> float:
        return float(self.header['hz'])

    @property
    def ticks(self) -> int:
        return int(self.header['ticks'])

    def times(self) -> list:
        t0, hz = float(self.header.get('t0_s', 0.0)), self.hz
        return [t0 + i / hz for i in range(self.ticks)]

    def agents(self) -> list:
        return list((self.header.get('robot') or {}).get('agents') or sorted(
            {n.rsplit('.', 1)[0] for n in self.columns if n.endswith('.x')}))

    def pose(self, agent: str, i: int) -> dict:
        return {k: self.columns[f'{agent}.{k}'][i] for k in POSE6 if f'{agent}.{k}' in self.columns}


def _box(columns: dict) -> dict | None:
    lo, hi = [math.inf] * 3, [-math.inf] * 3
    for name, values in columns.items():
        axis = name.rsplit('.', 1)[-1]
        if axis in ('x', 'y', 'z') and values:
            k = 'xyz'.index(axis)
            lo[k], hi[k] = min(lo[k], min(values)), max(hi[k], max(values))
    if lo[0] == math.inf:
        return None
    return {'min': [round(v, 3) if v != math.inf else None for v in lo],
            'max': [round(v, 3) if v != -math.inf else None for v in hi]}


def encode(header: dict, columns: dict, events=(), *, hz: float | None = None, t0_s: float | None = None,
           specs=None, block_s: float = BLOCK_S) -> bytes:
    """The file's bytes. ``columns`` maps name -> equal-length sequences of floats."""
    header = dict(header)
    hz = float(hz if hz is not None else header.get('hz') or 0)
    if not hz > 0:
        raise TrajError('hz must be positive')
    lengths = {len(v) for v in columns.values()}
    if len(lengths) > 1:
        raise TrajError(f'columns differ in length: {sorted(lengths)}')
    ticks = lengths.pop() if lengths else 0
    by_name = {c.name: c for c in (specs or [])}
    cols = [by_name.get(n) or Column(n, scale_of(n)) for n in columns]
    for c in cols:
        if not (c.scale > 0 and math.isfinite(c.offset)) or not 0 < len(c.name.encode()) < 256:
            raise TrajError(f'bad column {c}')
    header.update(schema=SCHEMA, hz=hz, t0_s=float(t0_s if t0_s is not None else header.get('t0_s', 0.0)),
                  ticks=ticks, duration_s=round(ticks / hz, 6), columns=[c.name for c in cols])
    box = _box(columns)
    if box:
        header['box'] = box
    out = io.BytesIO()
    hj = dumps_json(header).encode()
    out.write(MAGIC + struct.pack('<HHI', VERSION, 0, len(hj)) + hj)
    out.write(struct.pack('<H', len(cols)))
    for c in cols:
        nb = c.name.encode()
        out.write(struct.pack('<B', len(nb)) + nb + struct.pack('<dd', c.scale, c.offset))
    block_ticks = max(1, int(round(block_s * hz)))
    blocks = []
    q = {c.name: [quantize(v, c.scale, c.offset) for v in columns[c.name]] for c in cols}
    for first in range(0, ticks, block_ticks):
        n = min(block_ticks, ticks - first)
        raw = bytearray()
        for c in cols:
            prev = 0
            for v in q[c.name][first:first + n]:
                _varint(raw, _zigzag(v - prev))
                prev = v
        blocks.append((first, n, _deflate(bytes(raw))))
    out.write(struct.pack('<III', ticks, block_ticks, len(blocks)))
    for first, n, data in blocks:
        out.write(struct.pack('<III', first, n, len(data)))
    for _, _, data in blocks:
        out.write(data)
    ev = _deflate(''.join(dumps_json(_event(e)) + '\n' for e in events).encode())
    out.write(struct.pack('<I', len(ev)) + ev)
    body = out.getvalue()
    return body + struct.pack('<I', zlib.crc32(body) & 0xFFFFFFFF)


def _event(e: dict) -> dict:
    if not isinstance(e, dict) or 'kind' not in e or not math.isfinite(float(e.get('t', math.nan))):
        raise TrajError(f'an event needs a finite t and a kind: {e!r}')
    return {**{k: _finite(v) for k, v in e.items()}, 't': round(float(e['t']), 4)}


def _finite(v):
    """Non-finite floats become None, so any event is valid JSON."""
    if isinstance(v, float):
        return v if math.isfinite(v) else None
    if isinstance(v, dict):
        return {k: _finite(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_finite(x) for x in v]
    return v


def write(path, header: dict, columns: dict, events=(), **kw) -> str:
    """Write atomically; return the file's sha256."""
    import hashlib
    data = encode(header, columns, events, **kw)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_bytes(data)
    tmp.replace(path)
    return hashlib.sha256(data).hexdigest()


class _Reader:
    def __init__(self, data: bytes):
        self.data, self.pos = data, 0

    def take(self, n: int) -> bytes:
        if self.pos + n > len(self.data):
            raise TrajError('truncated file')
        b = self.data[self.pos:self.pos + n]
        self.pos += n
        return b

    def unpack(self, fmt: str):
        return struct.unpack(fmt, self.take(struct.calcsize(fmt)))


def _prefix(data: bytes, check_crc: bool = True):
    if len(data) < 16 or data[:4] != MAGIC:
        raise TrajError('not a crane-traj file')
    if check_crc:
        (crc,) = struct.unpack('<I', data[-4:])
        if zlib.crc32(data[:-4]) & 0xFFFFFFFF != crc:
            raise TrajError('crc mismatch (corrupt or truncated file)')
    r = _Reader(data)
    r.take(4)
    version, _flags, hlen = r.unpack('<HHI')
    if version != VERSION:
        raise TrajError(f'crane-traj version {version} is not supported (this reader: {VERSION})')
    header = json.loads(r.take(hlen).decode())
    (ncols,) = r.unpack('<H')
    specs = []
    for _ in range(ncols):
        (nlen,) = r.unpack('<B')
        name = r.take(nlen).decode()
        scale, offset = r.unpack('<dd')
        specs.append(Column(name, scale, offset))
    ticks, block_ticks, nblocks = r.unpack('<III')
    index = [r.unpack('<III') for _ in range(nblocks)]
    return header, specs, ticks, block_ticks, index, r


def _decode_block(blob: bytes, specs, n: int) -> dict:
    raw, pos, out = _inflate(blob), 0, {}
    for c in specs:
        vals, prev = [], 0
        for _ in range(n):
            z, shift = 0, 0
            while True:
                if pos >= len(raw):
                    raise TrajError('block shorter than its index says')
                b = raw[pos]
                pos += 1
                z |= (b & 0x7F) << shift
                shift += 7
                if b < 0x80:
                    break
            prev += _unzigzag(z)
            vals.append(prev * c.scale + c.offset)
        out[c.name] = vals
    return out


def loads(data: bytes, *, t_from: float | None = None, t_to: float | None = None) -> Traj:
    """Decode; with ``t_from``/``t_to`` (seconds, header time) only the blocks that overlap are
    decoded and the header's ``window_first_tick`` says where the columns start."""
    header, specs, ticks, block_ticks, index, r = _prefix(data)
    hz, t0 = float(header['hz']), float(header.get('t0_s', 0.0))
    columns = {c.name: [] for c in specs}
    first_tick = None
    for first, n, blen in index:
        blob = r.take(blen)
        if t_from is not None and t0 + (first + n - 1) / hz < t_from:
            continue
        if t_to is not None and t0 + first / hz > t_to:
            continue
        first_tick = first if first_tick is None else first_tick
        for k, v in _decode_block(blob, specs, n).items():
            columns[k].extend(v)
    (elen,) = r.unpack('<I')
    events = [json.loads(line) for line in _inflate(r.take(elen)).decode().splitlines() if line.strip()]
    if t_from is not None or t_to is not None:
        header = {**header, 'window_first_tick': first_tick or 0}
    elif any(len(v) != ticks for v in columns.values()):
        raise TrajError('decoded column length differs from ticks')
    return Traj(header, columns, events, specs)


def read(path, **kw) -> Traj:
    return loads(Path(path).read_bytes(), **kw)


def read_header(path) -> dict:
    """The header only (no block is decoded, no crc check)."""
    p = Path(path)
    with p.open('rb') as f:
        head = f.read(12)
        if head[:4] != MAGIC:
            raise TrajError(f'{p}: not a crane-traj file')
        _v, _f, hlen = struct.unpack('<HHI', head[4:12])
        return json.loads(f.read(hlen).decode())


def amend(path, **updates) -> str:
    """Merge ``updates`` into the header (a None value removes the key); blocks are kept as they are.
    Returns the new sha256."""
    import hashlib
    p = Path(path)
    data = p.read_bytes()
    header, _specs, _t, _bt, _idx, r = _prefix(data)
    for k, v in updates.items():
        if v is None:
            header.pop(k, None)
        else:
            header[k] = v
    hj = dumps_json(header).encode()
    head_end = 12 + struct.unpack('<I', data[8:12])[0]
    body = data[:4] + struct.pack('<HHI', VERSION, 0, len(hj)) + hj + data[head_end:-4]
    out = body + struct.pack('<I', zlib.crc32(body) & 0xFFFFFFFF)
    tmp = p.with_name(p.name + '.tmp')
    tmp.write_bytes(out)
    tmp.replace(p)
    return hashlib.sha256(out).hexdigest()


def problems(traj: Traj, *, max_speed: float = 15.0, bounds: dict | None = None) -> list:
    """Data errors a viewer would see: a jump faster than ``max_speed`` m/s (teleport), a pose
    outside ``bounds`` ({min: [x,y,z], max: [...]}), events outside the run, a bad outcome."""
    out = []
    dt = 1.0 / traj.hz
    for agent in traj.agents():
        xs, ys = traj.columns.get(f'{agent}.x'), traj.columns.get(f'{agent}.y')
        if xs is None or ys is None:
            out.append(f'{agent}: no x/y columns')
            continue
        for i in range(1, len(xs)):
            v = math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]) / dt
            if v > max_speed:
                out.append(f'{agent}: jump of {v:.1f} m/s at tick {i}')
                break
        if bounds:
            lo, hi = bounds['min'], bounds['max']
            if any(not (lo[0] <= x <= hi[0] and lo[1] <= y <= hi[1]) for x, y in zip(xs, ys)):
                out.append(f'{agent}: leaves the bounds')
    end = float(traj.header.get('t0_s', 0.0)) + traj.ticks / traj.hz
    late = [e for e in traj.events if not (float(traj.header.get('t0_s', 0.0)) - 1e-6 <= e['t'] <= end + 1.0)]
    if late:
        out.append(f'{len(late)} events outside the run ({late[0]})')
    if traj.header.get('outcome') not in (None, *OUTCOMES):
        out.append(f'unknown outcome {traj.header["outcome"]!r}')
    return out
