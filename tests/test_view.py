"""crane view and crane board: records into one frame and clock, the live sim's rerun determinism, an offscreen
screenshot, and the board's index over a mixed root.

Run from the crane root: python3 -m pytest tests/test_view.py   (the screenshot needs the [view] extra; it renders
offscreen and never opens a window)
"""

import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT)]

from crane import record  # noqa: E402
from crane.view import layers as vlayers, scene  # noqa: E402
from theater import traj as ctraj  # noqa: E402

SIM = {'source': 'sim', 't0': 0.0, 'time_multiplier': None}
KEEP_OUT = {'type': 'constraint', 'mode': 'keep_out', 'shape': 'polygon', 'id': 'no-go',
            'points': [[4.0, -1.0], [6.0, -1.0], [6.0, 3.0], [4.0, 3.0]]}
CORRIDOR = {'type': 'constraint', 'mode': 'keep_in', 'shape': 'corridor', 'id': 'lane',
            'points': [[-1.0, 0.0], [12.0, 0.0]], 'width': 4.0}


def path_enu(n=101, hz=10.0):
    """A boat heading east then curving north: (t, x, y, yaw) in ENU."""
    t = np.arange(n) / hz
    x, y = t * 1.0, 0.05 * t ** 2
    yaw = np.arctan2(0.1 * t, 1.0)
    return t, x, y, yaw


def a_record(d, run_id, frame, clock, *, ned=False, t0_shift=0.0, events=(), layers=None, sidecar=None):
    t, x, y, yaw = path_enu()
    if ned:   # the same motion seen in NED/FRD: x north, y east, yaw clockwise from north
        x, y, yaw = y, x, math.pi / 2 - yaw
    keep = t >= t0_shift          # b's t = 0 is a's t = t0_shift: it starts later on a's clock
    cols = {'boat.x': list(x[keep]), 'boat.y': list(y[keep]), 'boat.yaw': list(np.arctan2(np.sin(yaw), np.cos(yaw))[keep])}
    data = ctraj.encode({'pose_dims': 'planar', 'robot': {'model': 'boat', 'agents': ['boat']}}, cols, hz=10.0)
    extra = {'layers': layers} if layers else {}
    m = record.meta(run_id, domain='boat', tier='T1', kind='dynamic', frame=frame, clock=clock, seed=3, **extra)
    rec = record.write_run(d, m, traj=data, events=list(events))
    if sidecar:
        (Path(d) / vlayers.SIDECAR).write_text(json.dumps({'schema': vlayers.SCHEMA, 'layers': sidecar}))
    return rec


def test_records_overlay_in_one_frame_and_on_one_clock(tmp_path):
    enu = {'world': 'enu', 'origin': 'pool', 'body': 'flu'}
    ned = {'world': 'ned', 'origin': 'pool', 'body': 'frd'}
    log = [{'t': 1.5, 'kind': 'log', 'text': 'arming', 'level': 'info'}, {'t': 2.0, 'kind': 'phase', 'name': 'go'},
           {'t': 3.0, 't_end': 4.0, 'kind': 'row', 'text': 'judge span'}]
    a = a_record(tmp_path / 'a', 'a', enu, SIM, events=log, layers=[KEEP_OUT], sidecar=[CORRIDOR])
    b = a_record(tmp_path / 'b', 'b', ned, {**SIM, 't0': 2.0}, ned=True, t0_shift=2.0)
    sc = scene.load([a.path, b.path])
    ra, rb = sc.runs
    assert sc.frame == enu and rb.offset_s == 2.0
    # b's samples land where a's are at the same scene time: NED converted to ENU, its clock shifted by 2 s
    k = rb.index_at(5.0)
    j = ra.index_at(5.0)
    assert rb.t[k] == pytest.approx(5.0) and ra.t[j] == pytest.approx(5.0)
    assert (rb.x[k], rb.y[k]) == pytest.approx((ra.x[j], ra.y[j]), abs=0.011)
    assert rb.x[k] == pytest.approx(5.0, abs=0.011)                      # 5 s at 1 m/s east, not 3
    assert float(np.angle(np.exp(1j * (rb.yaw[k] - ra.yaw[j])))) == pytest.approx(0.0, abs=1e-3)
    # events on lanes (the boat's spec, log lines on their own lane, spans kept), layers from meta and sidecar
    lanes = {e['kind']: e['lane'] for e in ra.events}
    assert lanes == {'log': 'log', 'phase': 'mission', 'row': 'judge'} and 'log' in sc.lanes
    assert [c['id'] for c in ra.layers] == ['no-go', 'lane']
    bad = ra.violations()
    inside = (ra.x > 4.05) & (ra.x < 5.95)
    assert bad[inside].all() and not bad[(ra.x < 3.5)].any()             # in the keep-out box, inside the lane
    # different origins need a transform; a wall clock beside a sim clock needs an offset
    c = a_record(tmp_path / 'c', 'c', {**enu, 'origin': 'dock'}, SIM)
    with pytest.raises(Exception, match='origins differ'):
        scene.load([a.path, c.path])
    sc2 = scene.load([a.path, c.path], transforms=[None, {'from': 'dock', 'to': 'pool', 'xyz': [10.0, 0.0, 0.0], 'yaw': 0.0}])
    assert sc2.runs[1].x[0] == pytest.approx(10.0, abs=0.011)
    w = a_record(tmp_path / 'w', 'w', enu, {'source': 'wall', 't0': 1.7e9, 'time_multiplier': 1.0})
    with pytest.raises(scene.SceneError, match='clocks differ'):
        scene.load([a.path, w.path])
    assert scene.load([a.path, w.path], offsets=[None, 1.0]).runs[1].t[0] == pytest.approx(1.0)
    # a bare .ctraj opens through crane.record.from_ctraj (a T2 unity run unless its header says fastsim)
    bare = tmp_path / 'unity.ctraj'
    bare.write_bytes(ctraj.encode({'pose_dims': 'planar', 'frame': {'type': 'enu', 'origin': 'pool'}},
                                  {'boat.x': [0.0, 1.0], 'boat.y': [0.0, 0.0], 'boat.yaw': [0.0, 0.0]}, hz=10.0))
    sc3 = scene.load([a.path, bare], tmp_root=tmp_path)
    assert sc3.runs[1].tier == 'T2' and len(sc3.runs[1].t) == 2


def test_live_rerun_from_t_equals_the_original(tmp_path):
    from fastsim import config as fsconfig
    from fastsim.batch import adapter_of
    from fastsim.run import simulate
    from crane.view.live import FastsimLive, Snapshots, promote
    sim = FastsimLive.for_domain('boat', 'T1', 3, card_dir=tmp_path / 'cards')
    # the live loop is fastsim's loop: an untouched episode gives simulate()'s track exactly
    cfg, base = fsconfig.load(sim.config_path)
    simulate(sim.source.load(3), adapter_of(cfg, base), cfg, 3, tmp_path / 'batch', base)
    ref = [json.loads(s) for s in (tmp_path / 'batch' / 'track.jsonl').read_text().splitlines()]
    snaps = Snapshots(1.0)
    while sim.t < 40.0:
        sim.step(25)
        snaps.keep(sim)
    t_snap, snap = snaps.at_or_before(20.0)
    assert t_snap == pytest.approx(20.0, abs=0.5)
    sim.run()
    original = (list(sim.trail), sim.events, sim.outcome)
    assert [tuple(r.values()) for r in ref] == [tuple(p) for p in sim.trail[:len(ref)]] and len(ref) == len(sim.trail)
    # rerun from t with the same sliders: the same episode, sample for sample
    sim.restore(snap)
    assert sim.t == pytest.approx(t_snap) and len(sim.trail) < len(original[0])
    sim.run()
    assert (list(sim.trail), sim.events, sim.outcome) == original
    # with a current, a plant delay and a heavier card from t: it departs from there, and again identically
    sim.restore(snap)
    for k, v in (('current_speed', 0.3), ('current_toward', 90.0), ('delay', 0.1), ('mass', 40.0)):
        sim.set_param(k, v)
    sim.run(until_s=80.0)
    changed = list(sim.trail)
    cut = sum(1 for p in original[0] if p[0] <= t_snap)
    assert changed[:cut] == original[0][:cut] and changed[cut + 20] != original[0][cut + 20]
    sim.restore(snap)
    sim.run(until_s=80.0)
    assert sim.trail == changed
    # Promote: a crane run spec whose overrides carry the current and the card
    path, command = promote(sim, tmp_path / 'promoted')
    import yaml
    spec = yaml.safe_load(path.read_text())
    assert spec['set']['vehicle.current'][1] == pytest.approx(0.3) and spec['view_only'] == {'delay_s': 0.1}
    assert json.loads(Path(spec['set']['vehicle.profile']).read_text())['mass'] == 40.0
    assert command.startswith('crane run --domain boat --tier T1 ') and '--set' in command and spec['command'] == command


@pytest.mark.skipif(importlib.util.find_spec('PySide6') is None or importlib.util.find_spec('pyqtgraph') is None,
                    reason='the [view] extra is not installed')
def test_screenshot_renders_offscreen(tmp_path):
    rec = a_record(tmp_path / 'a', 'a', {'world': 'enu', 'origin': 'pool', 'body': 'flu'}, SIM,
                   events=[{'t': 1.0, 'kind': 'log', 'text': 'hello'}], layers=[KEEP_OUT, CORRIDOR])
    env = {**os.environ, 'QT_QPA_PLATFORM': 'offscreen', 'PYTHONPATH': str(ROOT)}
    env.pop('DISPLAY', None)
    env.pop('WAYLAND_DISPLAY', None)
    for args, out in (([str(rec.path), '--at', '6'], tmp_path / 'replay.png'),
                      (['--live', '--domain', 'boat', '--tier', 'T0', '--seed', '3', '--at', '30'], tmp_path / 'live.png')):
        r = subprocess.run([sys.executable, '-m', 'crane', 'view', *args, '--screenshot', str(out),
                            '--promote-dir', str(tmp_path / 'p')], env=env, capture_output=True, text=True,
                           timeout=120, cwd=tmp_path)
        assert r.returncode == 0, r.stderr
        data = out.read_bytes()
        assert data[:8] == b'\x89PNG\r\n\x1a\n' and len(data) > 20_000


def test_board_index_over_a_mixed_root(tmp_path):
    from viewer.store import Store
    root = tmp_path / 'root'
    log = [{'t': 0.5, 'kind': 'log', 'text': '/events: arming'}, {'t': 2.0, 'kind': 'phase', 'name': 'go'}]
    rec = a_record(root / 'study' / 'rec', 'rec-1', {'world': 'enu', 'origin': 'pool', 'body': 'flu'}, SIM,
                   events=log, sidecar=[KEEP_OUT, CORRIDOR])
    ev = record.write_run(root / 'study' / 'ev', record.meta('ev-1', domain='toy', tier='T0', kind='event',
                                                             frame={'world': 'enu', 'origin': 'x', 'body': 'flu'},
                                                             clock=SIM), events=[{'t': 1.0, 'kind': 'done'}])
    # a fastsim run dir: its own run.json beside crane.run.json; its run.ctraj is served once, through the record
    from fastsim import config as fsconfig
    from fastsim.batch import adapter_of
    from fastsim.run import simulate
    cfg, base = fsconfig.load(ROOT / 'domains' / 'boat' / 'fastsim_t0.yaml', {'time_limit_s': 20})
    from fastsim.scenario import ScenarioSource
    summary = simulate(ScenarioSource(**cfg['scenarios'], base=base).load(3), adapter_of(cfg, base), cfg, 3,
                       root / 'fast' / 'base' / 's3', base, {'variant': 'base'})
    bare = root / 'loose' / 'x.ctraj'
    bare.parent.mkdir(parents=True)
    bare.write_bytes(ctraj.encode({'run_id': 'bare-1', 'tier': 'T2', 'study': 'loose'},
                                  {'r.x': [0.0, 1.0], 'r.y': [0.0, 0.0], 'r.yaw': [0.0, 0.0]}, hz=10.0))
    (root / 'other').mkdir()
    (root / 'other' / 'run.json').write_text(json.dumps({'schema': 'something.else/1'}))   # not a run
    store = Store([root])
    ids = {r['run_id'] for r in store.index()['runs']}
    assert ids == {'rec-1', 'ev-1', summary['identity']['run_sha256'], 'bare-1'}
    t = ctraj.loads(store.runs['rec-1'].data())
    assert [c['id'] for c in t.header['layers']] == ['no-go', 'lane'] and t.header['tier'] == 'T1'
    assert t.header['crane_run']['frame']['origin'] == 'pool' and t.header['outline']
    assert {(e['kind'], e['lane']) for e in t.events} == {('log', 'log'), ('phase', 'mission')}
    assert t.ticks == 101 and ctraj.loads(store.runs['rec-1'].lod_data()).hz == 2.0
    e = ctraj.loads(store.runs['ev-1'].data())
    assert e.ticks == 0 and e.events[0]['kind'] == 'done'
    fast = store.runs[summary['identity']['run_sha256']]
    assert fast.header['arm'] == 'base' and fast.header['study'] == 'fast'
    assert store.course(fast.header['course']['truth_sha256'])['name'].startswith('roboboat_2026')
    assert store.runs['bare-1'].data() == bare.read_bytes()
