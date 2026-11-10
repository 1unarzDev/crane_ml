"""Thin shim between the writers' intermediate records (``_common``) and ``judge.record``.

Paths (decided at import, reported by ``SCHEMA_PATH``):

* ``'v2'``  -- ``judge.record`` declares ``SCHEMA == 'crane.judge.run/2'`` with ``standard``,
  ``declare_absent`` and the typed header blocks (landed 2026-11-09): ``save`` builds a real RunRecord
  (standard channels via ``judge.record.standard``, absent ones via ``declare_absent(name, why)``, the
  seven blocks as ``BLOCK_TYPES``) and ``judge.record.save`` writes record.json + channels.npz.
* ``'intermediate'`` -- otherwise: ``save`` writes ``capture.json`` + ``capture.npz`` (sorted keys,
  rounded floats, fixed zip timestamps: equal runs hash equally) in exactly the field layout of
  SCHEMA_V2.md. ``convert(dir)`` turns such a directory into a real record once /2 lands, or, today,
  into a loadable ``crane.judge.run/1`` record (v2 header blocks under ``provenance['v2']``, absent
  channels as observability 'absent' with no samples) so downstream code can start.

Nothing here edits judge/.
"""

from __future__ import annotations

from dataclasses import fields as dc_fields
import inspect
import io
import json
import math
from pathlib import Path
import zipfile

import numpy as np

import judge.record as JR

V2_HEADER_BLOCKS = ('backend', 'controller', 'calibration', 'clock_mode', 'code', 'evidence', 'reset')
V2 = (getattr(JR, 'SCHEMA', '') == 'crane.judge.run/2' and hasattr(JR, 'declare_absent')
      and hasattr(JR, 'standard') and hasattr(JR, 'BLOCK_TYPES'))
SCHEMA_PATH = 'v2' if V2 else 'intermediate'
CAPTURE_JSON, CAPTURE_NPZ = 'capture.json', 'capture.npz'
INTERMEDIATE_TAG = 'decisioncal.records.intermediate/1'


def clean(x, nd=9):
    """JSON-ready and deterministic (judge.record._clean's rules: floats rounded, NaN -> null)."""
    if isinstance(x, (bool, np.bool_)):
        return bool(x)
    if isinstance(x, float):
        return None if not math.isfinite(x) else round(x, nd)
    if isinstance(x, np.floating):
        return clean(float(x), nd)
    if isinstance(x, np.integer):
        return int(x)
    if isinstance(x, dict):
        return {str(k): clean(v, nd) for k, v in x.items()}
    if isinstance(x, (set, frozenset)):
        return sorted(clean(v, nd) for v in x)
    if isinstance(x, (list, tuple)):
        return [clean(v, nd) for v in x]
    if isinstance(x, np.ndarray):
        return clean(x.tolist(), nd)
    return x


def npz_bytes(arrays):
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, 'w', zipfile.ZIP_DEFLATED) as z:
        for name in sorted(arrays):
            info = zipfile.ZipInfo(name + '.npy', date_time=(1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            a = io.BytesIO()
            np.lib.format.write_array(a, np.ascontiguousarray(arrays[name]), allow_pickle=False)
            z.writestr(info, a.getvalue())
    return buf.getvalue()


# ---------------------------------------------------------------------------------------------
# intermediate
# ---------------------------------------------------------------------------------------------

def save_intermediate(rec, out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    meta = {n: {k: v for k, v in c.items() if k not in ('t', 'v', 'tick')} | {'n': len(c['t']), 'has_tick': c['tick'] is not None}
            for n, c in sorted(rec['channels'].items())}
    doc = {'schema': rec['schema'], 'intermediate': INTERMEDIATE_TAG, 'header': rec['header'], 'channels': meta,
           'absent': rec['absent'], 'events': rec['events'], 'caps': rec['caps'], 'truth': rec['truth']}
    (out / CAPTURE_JSON).write_text(json.dumps(clean(doc), sort_keys=True, indent=1) + '\n')
    arrays = {}
    for n, c in rec['channels'].items():
        arrays[f'{n}/t'] = c['t']
        arrays[f'{n}/v'] = c['v']
        if c['tick'] is not None:
            arrays[f'{n}/tick'] = c['tick']
    (out / CAPTURE_NPZ).write_bytes(npz_bytes(arrays))
    return [out / CAPTURE_JSON, out / CAPTURE_NPZ]


def load_intermediate(path):
    path = Path(path)
    doc = json.loads((path / CAPTURE_JSON).read_text())
    if doc.get('intermediate') != INTERMEDIATE_TAG:
        raise ValueError(f'{path}: not a decisioncal intermediate capture')
    z = np.load(path / CAPTURE_NPZ)
    ch = {}
    for n, m in doc['channels'].items():
        has_tick = m.pop('has_tick')
        m.pop('n')
        ch[n] = {**m, 'fields': tuple(m['fields']), 'units': tuple(m['units']), 'angles': tuple(m['angles']),
                 't': z[f'{n}/t'], 'v': z[f'{n}/v'], 'tick': z[f'{n}/tick'] if has_tick else None}
    return {'schema': doc['schema'], 'header': doc['header'], 'channels': ch, 'absent': doc['absent'],
            'events': doc['events'], 'caps': doc['caps'], 'truth': doc['truth']}


# ---------------------------------------------------------------------------------------------
# judge.record builders
# ---------------------------------------------------------------------------------------------

def _accepts(fn, name):
    try:
        return name in inspect.signature(fn).parameters
    except (TypeError, ValueError):
        return False


def _dc_names(cls):
    return {f.name for f in dc_fields(cls)}


def _channel(name, c, v2):
    """Standard names go through ``judge.record.standard`` (fixed fields/units/frame, allowed observability)
    when /2 is available; other names (namespaced extensions) are plain ChannelSpecs."""
    note = c.get('note', '') or ''
    if v2 and name in JR.STANDARD:
        sc = JR.STANDARD[name]
        assert tuple(c['fields']) == sc.fields, (name, c['fields'], sc.fields)
        return JR.standard(name, c['t'], c['v'], tick=c['tick'], observability=c['observability'],
                           note=note, rate_hz=c['rate_hz'])
    kw = dict(name=name, fields=tuple(c['fields']), units=tuple(c['units']), frame=c['frame'],
              observability=c['observability'], rate_hz=c['rate_hz'], angles=tuple(c['angles']))
    if v2 and note:
        kw['note'] = note
    spec = JR.ChannelSpec(**{k: v for k, v in kw.items() if k in _dc_names(JR.ChannelSpec)})
    if len(c['t']) == 0:
        return JR.Channel(spec, np.zeros(0), np.zeros((0, len(spec.fields))))
    if c['tick'] is not None and _accepts(JR.Channel.of, 'tick'):
        return JR.Channel.of(spec, c['t'], c['v'], tick=c['tick'])
    return JR.Channel.of(spec, c['t'], c['v'])


def _header(h, v2):
    sc = h['scenario']
    sc_kw = {k: sc.get(k) for k in _dc_names(JR.Scenario) if k in sc}
    if 'id' not in _dc_names(JR.Scenario):           # Scenario has no id field: keep it in condition
        sc_kw['condition'] = {**sc.get('condition', {}), 'scenario_id': sc.get('id')}
    prov = dict(h.get('provenance', {}))
    kw = dict(run_id=h['run_id'], tier=h['tier'], source=h['source'], domain=h['domain'],
              robot=JR.Robot(**h['robot']), task_id=h.get('task_id', ''), scenario=JR.Scenario(**sc_kw),
              clock=JR.Clock(**h['clock']), window=tuple(h['window']), source_version=h.get('source_version', '2'))
    if v2:
        for b in V2_HEADER_BLOCKS:
            kw[b] = JR.BLOCK_TYPES[b](**h[b])
    else:
        prov['v2'] = {b: h[b] for b in V2_HEADER_BLOCKS}
    kw['provenance'] = prov
    return JR.Header(**kw)


def build_judge_record(rec, v2=None):
    """A ``judge.record.RunRecord`` from an intermediate record (``v2`` default: the import-time path)."""
    v2 = V2 if v2 is None else v2
    header = _header(rec['header'], v2)
    chans = {n: _channel(n, c, v2) for n, c in rec['channels'].items()}
    for n, why in rec['absent'].items():
        if v2:
            chans[n] = JR.declare_absent(n, why)
        else:
            from ._common import STANDARD
            f, u, fr, a = STANDARD[n]
            spec = JR.ChannelSpec(n, f, u, fr, 'absent', angles=a)
            chans[n] = JR.Channel(spec, np.zeros(0), np.zeros((0, len(f))))
    events = tuple(JR.Event(t=e['t'], kind=e['kind'], actor=e.get('actor', ''), payload=e.get('payload', {}))
                   for e in rec['events'])
    caps = dict(rec['caps'])
    if not v2:
        caps['absent'] = dict(rec['absent'])
    return JR.RunRecord(header, chans, events, caps, JR.Validity(), dict(rec['truth']))


def save(rec, out):
    """Write ``rec`` into ``out``; returns (schema path, [files])."""
    out = Path(out)
    if V2:
        r = build_judge_record(rec, v2=True)
        JR.save(r, out)
        return 'v2', sorted(p for p in out.iterdir() if p.name in ('record.json', 'channels.npz'))
    return 'intermediate', save_intermediate(rec, out)


def convert(src, out=None, target=None):
    """Intermediate capture dir -> judge record files (record.json + channels.npz) in ``out``
    (default ``src``). ``target`` 'v2' | 'v1' (default: v2 when judge.record has it, else v1)."""
    target = target or ('v2' if V2 else 'v1')
    if target == 'v2' and not V2:
        raise RuntimeError('judge.record does not provide crane.judge.run/2 yet')
    rec = load_intermediate(src)
    r = build_judge_record(rec, v2=(target == 'v2'))
    out = Path(out or src)
    JR.save(r, out)
    return out
