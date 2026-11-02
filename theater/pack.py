"""Track pack ``crane-theater-pack/1`` and manifest ``crane-theater/1``: what the Unity theater reads.

A pack holds every run's pose samples resampled to one rate, quantized to int16 (run-major), so 5000
ten-minute runs at 10 Hz fit in about 600 MB and 500 in 60 MB::

    "CTPK" u32 version=1 u32 header_len header(JSON) u32 n_runs
    n_runs x (u32 first_sample, u32 n_samples)
    samples: n_channels x int16 per sample, run after run
    u32 crc32 of everything above

Header: ``{"schema", "hz", "channels": [{"name", "scale", "offset"}]}``; value = int16 * scale + offset.
Channels: x, y, z (m, the world frame of the course: ENU, x east, y north, z up), yaw, pitch, roll
(deg; yaw CCW from +x), speed (m/s), progress (0..1 along the world's centre line).

The manifest (JSON) names the pack, the world plug-in and its inputs, the robot plug-in, and each run's
metadata (id, iteration, score, outcome, progress, valid, params, events), in pack order.
"""

import json
import math
import struct
import zlib
from pathlib import Path

MAGIC = b'CTPK'
VERSION = 1
SCHEMA = 'crane-theater/1'
PACK_SCHEMA = 'crane-theater-pack/1'
CHANNELS = (('x', 0.01), ('y', 0.01), ('z', 0.001), ('yaw', 0.01), ('pitch', 0.01), ('roll', 0.01),
            ('speed', 0.001), ('progress', 1.0 / 30000))
I16 = 32767


def _quant(v, scale, offset):
    q = round((v - offset) / scale)
    if q > I16 or q < -I16:
        raise ValueError(f'value {v} outside the int16 range of scale {scale} offset {offset}')
    return q


def wrap_deg(a):
    a = math.fmod(a + 180.0, 360.0)
    return a + 180.0 if a < 0 else a - 180.0


def resample(samples, hz):
    """Samples ``[(t, {channel: value})]`` (any rate, sorted by t) at ``hz`` from t0, linear (yaw unwrapped)."""
    if not samples:
        return []
    t0, t1 = samples[0][0], samples[-1][0]
    n = int(math.floor((t1 - t0) * hz + 1e-9)) + 1
    out, j = [], 0
    for i in range(n):
        t = t0 + i / hz
        while j + 1 < len(samples) and samples[j + 1][0] <= t:
            j += 1
        a = samples[j][1]
        if j + 1 >= len(samples) or samples[j + 1][0] <= samples[j][0]:
            out.append(dict(a))
            continue
        b = samples[j + 1][1]
        u = (t - samples[j][0]) / (samples[j + 1][0] - samples[j][0])
        v = {}
        for k in a:
            if k in ('yaw', 'pitch', 'roll'):
                v[k] = wrap_deg(a[k] + u * wrap_deg(b.get(k, a[k]) - a[k]))
            else:
                v[k] = a[k] + u * (b.get(k, a[k]) - a[k])
        out.append(v)
    return out


def write_pack(path, runs, hz, origin=(0.0, 0.0, 0.0)):
    """Write ``runs`` (each a list of {channel: value} at ``hz``) to ``path``; returns its header.

    ``origin`` (x, y, z) is subtracted before quantizing (the int16 box is +-327 m around it)."""
    chans = [{'name': n, 'scale': s, 'offset': (origin[0] if n == 'x' else origin[1] if n == 'y'
                                                 else origin[2] if n == 'z' else 0.0)} for n, s in CHANNELS]
    header = {'schema': PACK_SCHEMA, 'hz': hz, 'channels': chans}
    hb = json.dumps(header, sort_keys=True).encode()
    body = bytearray()
    body += MAGIC + struct.pack('<II', VERSION, len(hb)) + hb + struct.pack('<I', len(runs))
    first = 0
    for r in runs:
        body += struct.pack('<II', first, len(r))
        first += len(r)
    row = struct.Struct('<' + 'h' * len(chans))
    for r in runs:
        for s in r:
            body += row.pack(*(_quant(s.get(c['name'], 0.0), c['scale'], c['offset']) for c in chans))
    body += struct.pack('<I', zlib.crc32(body) & 0xffffffff)
    Path(path).write_bytes(bytes(body))
    return header


def read_pack(path):
    """(header, runs) with each run a list of {channel: value}; checks magic, version and crc."""
    data = Path(path).read_bytes()
    if data[:4] != MAGIC:
        raise ValueError(f'{path}: not a crane theater pack')
    if struct.unpack_from('<I', data, len(data) - 4)[0] != zlib.crc32(data[:-4]) & 0xffffffff:
        raise ValueError(f'{path}: crc mismatch')
    version, hlen = struct.unpack_from('<II', data, 4)
    if version != VERSION:
        raise ValueError(f'{path}: pack version {version}')
    header = json.loads(data[12:12 + hlen])
    o = 12 + hlen
    (n,) = struct.unpack_from('<I', data, o)
    o += 4
    table = [struct.unpack_from('<II', data, o + 8 * i) for i in range(n)]
    o += 8 * n
    chans = header['channels']
    row = struct.Struct('<' + 'h' * len(chans))
    runs = []
    for first, count in table:
        r = []
        for k in range(count):
            q = row.unpack_from(data, o + (first + k) * row.size)
            r.append({c['name']: q[i] * c['scale'] + c['offset'] for i, c in enumerate(chans)})
        runs.append(r)
    return header, runs


def write_manifest(path, *, world, robot, runs, pack='tracks.pack', study=None, iterations=None, script=None,
                   water=None, extra=None):
    """The ``crane-theater/1`` manifest. ``runs``: dicts with id, iteration, score, outcome, progress, valid,
    duration_s, params, events [{t, kind, x, y, ...}], in pack order."""
    doc = {'schema': SCHEMA, 'pack': pack, 'world': world, 'robot': robot, 'study': study,
           'water': water or {'mode': 'still'}, 'iterations': iterations or [], 'runs': runs}
    if script:
        doc['script'] = script
    doc.update(extra or {})
    Path(path).write_text(json.dumps(doc, indent=1, sort_keys=True) + '\n')
    return doc


def centreline_progress(line):
    """A function (x, y) -> progress in 0..1: arc length of the nearest point on the polyline ``line``."""
    seg, total = [], 0.0
    for (ax, ay), (bx, by) in zip(line, line[1:]):
        L = math.hypot(bx - ax, by - ay)
        seg.append((ax, ay, bx, by, L, total))
        total += L
    total = max(total, 1e-9)

    def progress(x, y):
        best, bd = 0.0, float('inf')
        for ax, ay, bx, by, L, s0 in seg:
            if L <= 0:
                continue
            u = max(0.0, min(1.0, ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / (L * L)))
            px, py = ax + u * (bx - ax), ay + u * (by - ay)
            d = (x - px) ** 2 + (y - py) ** 2
            if d < bd:
                bd, best = d, (s0 + u * L) / total
        return best
    return progress
