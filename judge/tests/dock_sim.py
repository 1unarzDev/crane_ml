"""Test helper: real fast-layer dock missions as judge RunRecords.

The run itself is ``Tools/DecisionCal/records/numpy_capture.run_recorded`` (``mission.run`` unedited,
plant instrumented by a subclass); this helper only turns the captured arrays into a
``judge.record.RunRecord`` (the writer's in-memory builder is not used, so these tests do not depend
on its header conversion) and offers the single-cause injections the layer tests need. Imports the
harness from the tests only (sys.path set here)."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DC = ROOT / 'Tools' / 'DecisionCal'
if str(DC) not in sys.path:
    sys.path.insert(0, str(DC))

from records import _common as C  # noqa: E402
from records import numpy_capture as NC  # noqa: E402
import mission  # noqa: E402
import twin  # noqa: E402
from library import LIBRARY  # noqa: E402

import judge.record as JR  # noqa: E402

DT = 0.02


def nominal_spec(**over):
    """One nominal twin boat (no dead time, latency or lag), fields overridden by scalars/rows."""
    spec = twin.to_spec(twin.nominal_particles(1))
    kw = {}
    for k, v in over.items():
        cur = np.asarray(getattr(spec, k))
        kw[k] = np.broadcast_to(np.asarray(v, float), cur.shape).copy()
    return spec.with_(**kw) if kw else spec


def blocks(layer, name, ctrl_id, params, dt=DT, lib='decisioncal.library_v1'):
    return {'backend': JR.Backend(layer, name), 'controller': JR.Controller(ctrl_id, lib, None, params),
            'calibration': JR.Calibration(), 'clock_mode': JR.ClockMode('fixed_step', dt, episode=0),
            'code': JR.Code('crane', 'test', False), 'evidence': JR.Evidence('exploratory'),
            'reset': JR.Reset('construct', True, (0.0, 0.0, 0.0))}


def _channel(name, c):
    spec = JR.ChannelSpec(name, tuple(c['fields']), tuple(c['units']), c['frame'], c['observability'], c['rate_hz'],
                          tuple(c['angles']))
    return JR.Channel.of(spec, c['t'], c['v'], tick=c['tick'])


def run(spec=None, ctrl=0, seed=7, t_max=20.0, lib=LIBRARY, run_id=None, layer='fast', backend='numpy.test',
        ctrl_params=None, scenario='dock-test', absent=(), header_blocks=True, events=True, ctrl_id=None):
    """(RunRecord, mission metrics) of one recorded mission."""
    spec = nominal_spec() if spec is None else spec
    m, p = NC.run_recorded(spec, ctrl, seed, t_max=t_max, lib=lib)
    ch, t, st = NC._channels(p)
    ev, outc = C.mission_events(t, st, t_max)
    chans = {n: _channel(n, c) for n, c in ch.items()}
    chans['cmd.sent'] = JR.declare_absent('cmd.sent', 'fast layer: no transport')
    for n in absent:
        chans[n] = JR.declare_absent(n, 'test: this backend does not expose it')
    c = lib[ctrl]
    params = ctrl_params if ctrl_params is not None else {'idx': int(c.idx)}
    bl = blocks(layer, backend, ctrl_id or f'c{ctrl:02d}', params)
    end = 'docked' if outc['terminal'] == 'docked' else outc['terminal']
    hkw = dict(run_id=run_id or f'{layer}-{scenario}-{ctrl_id or f"c{ctrl:02d}"}-s{seed}', tier=layer, source='test.numpy',
               domain='marine.asv', robot=JR.Robot('test-boat'), task_id='decisioncal.dock_mission',
               scenario=JR.Scenario(pair_key=f'{scenario}:s{seed}', seed=seed, family='decisioncal.gate_uturn_dock',
                                    condition={'t_max_s': t_max}),
               window=(float(t[0]), float(t[-1]), end), source_version='2')
    if header_blocks:
        hkw.update(bl)
    else:
        hkw['provenance'] = {k: _asdict(v) for k, v in bl.items()}
    evs = tuple(JR.Event(e['t'], e['kind'], e.get('actor', ''), e.get('payload', {})) for e in ev) if events else ()
    return JR.RunRecord(JR.Header(**hkw), chans, evs), m


def _asdict(b):
    from dataclasses import asdict
    return asdict(b)


# ------------------------------------------------------------------------------------ injections

def shift_time(rec, dt_s, run_id=None):
    """Every channel's clock (and the time-valued fields of obs.stamp / cmd.applied / cmd.sent) moved
    by ``dt_s``; ticks and events untouched: a timestamp offset on this record's time base."""
    chans = {}
    for n, ch in rec.channels.items():
        if not rec.has(n):
            chans[n] = ch
            continue
        v = ch.v.copy()
        for k, f in enumerate(ch.spec.fields):
            if f in ('t_acq', 't_recv', 't_apply', 't_send'):
                v[:, k] += dt_s
        chans[n] = replace(ch, t=ch.t + dt_s, v=v)
    return replace(rec, header=replace(rec.header, run_id=run_id or rec.run_id + '-shift'), channels=chans)


def strip_ticks(rec):
    return replace(rec, channels={n: replace(ch, tick=None) for n, ch in rec.channels.items()})


def decimate(rec, k, dt):
    """Every k-th sample of every channel, clock_mode.dt_s = dt (a coarser-ticked backend)."""
    chans = {}
    for n, ch in rec.channels.items():
        if not rec.has(n):
            chans[n] = ch
            continue
        tk = ch.tick
        keep = (tk % k == 0) if tk is not None else np.arange(len(ch.t)) % k == 0
        chans[n] = replace(ch, t=ch.t[keep], v=ch.v[keep], tick=None if tk is None else tk[keep] // k)
    h = replace(rec.header, run_id=rec.run_id + f'-dec{k}', clock_mode=JR.ClockMode('fixed_step', dt, episode=0))
    return replace(rec, header=h, channels=chans)


def relabel(rec, run_id, layer='integrated', backend='engine.test'):
    h = replace(rec.header, run_id=run_id, tier=layer, backend=JR.Backend(layer, backend))
    return replace(rec, header=h)
