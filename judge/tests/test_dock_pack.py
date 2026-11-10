"""The dock-mission pack: J and every outcome quantity equal mission.run's on real fast-layer runs;
truth-only scoring; unscorable without truth; stored vs recomputed events; outcome categories."""

from dataclasses import replace
import math

import numpy as np
import pytest

import dock_sim as S                       # judge/tests/dock_sim.py (sets sys.path for the harness)
import hidden
import mission

import judge.record as JR
from judge.pack import evaluate
from judge.packs import dock_mission as D
from judge.packs.dock_mission import geometry as G
from judge.metric import Status

PACKS = [D.PACK]


def test_geometry_mirrors_the_harness():
    assert G.T_MAX == mission.T_MAX and tuple(G.TARGET) == tuple(mission.TARGET)
    assert (G.GATE_X, G.GATE_Y, G.BUOY_R) == (mission.GATE_X, mission.GATE_Y, mission.BUOY_R)
    assert G.SLIP == mission.SLIP and np.array_equal(G.FOOT, mission.FOOT) and G.J_WEIGHTS == mission.J_WEIGHTS
    rng = np.random.default_rng(3)
    pose = np.column_stack([rng.uniform(1, 11, 4000), rng.uniform(-6.5, 1.5, 4000), rng.uniform(-4, 4, 4000)])
    assert np.array_equal(G.objects_hit(pose).any(1), mission._contacts(pose))


def _cases():
    sp = {st: hidden.sample_hidden(8, 4242, st) for st in ('timing', 'transient')}
    return [('nominal-c0', S.nominal_spec(), 0, 60.0),        # docks cleanly
            ('timing5-c0', sp['timing'].take([5]), 0, 60.0),     # grinds a wall: 10 contacts, no dock
            ('transient0-c0', sp['transient'].take([0]), 0, 60.0),  # docks after one contact
            ('transient4-c0', sp['transient'].take([4]), 0, 60.0),  # times out near the slip
            ('nominal-c9', S.nominal_spec(), 9, 25.0)]          # times out far from the slip


@pytest.fixture(scope='module')
def runs():
    out = []
    for name, spec, ctrl, t_max in _cases():
        rec, m = S.run(spec, ctrl=ctrl, t_max=t_max, run_id=name)
        out.append((name, rec, m))
    return out


def test_pack_J_equals_mission_run_J(runs):
    """On trajectories from mission.run itself (recording plant, bit-identical), the pack's J and its
    inputs equal the harness's metrics."""
    seen = set()
    for name, rec, m in runs:
        v = evaluate(rec, None, PACKS)
        g = lambda k: float(np.asarray(m[k]).ravel()[0])
        assert v['dock.J'].status == Status.OK, name
        assert v['dock.J'].value == pytest.approx(g('J'), abs=1e-9), name
        assert v['dock.docked'].value == g('docked'), name
        assert v['dock.contacts'].value == g('contacts'), name
        assert v['dock.gate_miss'].value == g('gate_miss'), name
        if g('docked'):
            assert v['dock.time_to_dock'].value == pytest.approx(g('t_dock'), abs=1e-12), name
        else:
            assert v['dock.time_to_dock'].status == Status.UNSCORABLE
            assert v['dock.pose_error'].value == pytest.approx(g('pose_err'), abs=1e-12), name
        comp = v['dock.J'].extra['components']
        assert sum(comp.values()) == pytest.approx(v['dock.J'].value)
        seen.add(D.outcome(rec, v))
    assert {'docked', 'contact_fail'} <= seen and seen & {'timeout_in_slip', 'timeout_no_approach'}


def test_metrics_never_read_obs(runs):
    """Corrupting every obs.* and cmd.* channel changes nothing: scoring is truth + events only."""
    _, rec, _ = runs[2]
    junk = {n: replace(ch, v=ch.v + 7.0) if n.startswith(('obs.', 'cmd.')) and rec.has(n) else ch
            for n, ch in rec.channels.items()}
    a = evaluate(rec, None, PACKS)
    b = evaluate(replace(rec, channels=junk), None, PACKS)
    assert {k: v.value for k, v in a.items()} == {k: v.value for k, v in b.items()}


def test_spans_point_at_the_evidence(runs):
    _, rec, m = runs[1]
    v = evaluate(rec, None, PACKS)
    c = v['dock.contacts']
    assert len(c.spans) == c.value == 10 and c.extra['first_t'] == c.spans[0].t0
    assert all(s.run_id == rec.run_id and 'truth.pose' in s.channels for s in c.spans)
    _, rec0, _ = runs[0]
    d = evaluate(rec0, None, PACKS)['dock.docked']
    hold = d.spans[0]
    assert hold.t1 - hold.t0 == pytest.approx(2.0, abs=0.021) and hold.label == 'dock hold'


def test_physical_open_loop_record_is_unscorable_not_zero(runs):
    _, rec, _ = runs[0]
    keep = {n: ch for n, ch in rec.channels.items() if not n.startswith('truth.')}
    keep['truth.pose'] = JR.declare_absent('truth.pose', 'open-loop pulse log: no surveyed pose')
    keep['truth.twist'] = JR.declare_absent('truth.twist', 'open-loop pulse log')
    phys = replace(rec, channels=keep, events=())
    vals = evaluate(phys, None, PACKS)
    assert vals and all(v.status == Status.UNSCORABLE and v.value is None for v in vals.values())
    assert D.outcome(phys, vals) == 'unscorable'


def test_twist_absent_is_provisional(runs):
    _, rec, m = runs[0]
    ch = dict(rec.channels)
    ch['truth.twist'] = JR.declare_absent('truth.twist', 'pose-only survey')
    vals = evaluate(replace(rec, channels=ch), None, PACKS)
    assert vals['dock.docked'].status == Status.PROVISIONAL
    assert vals['dock.contacts'].status == Status.OK


def test_stored_events_agree_and_a_writer_bug_is_reported(runs):
    for _, rec, _ in runs:
        chk = D.event_check(rec)
        assert chk['stored'] > 0 and not chk['disagreements'], (rec.run_id, chk)
        assert D.event_insight(rec) is None
    _, rec, _ = runs[2]                               # docks after one contact
    bad = tuple(replace(e, t=e.t + 0.5) if e.kind == 'docked' else e for e in rec.events
                if e.kind != 'contact')
    ins = D.event_insight(replace(rec, events=bad))
    assert ins is not None and ins.kind == 'dock.events_disagree'
    whats = [d['what'] for d in ins.data['disagreements']]
    assert any(w.startswith('contact in truth') for w in whats)
    assert any(w.startswith('docked stored t=') for w in whats)
    assert evaluate(replace(rec, events=bad), None, PACKS)['dock.events_agree'].value == 0.0
    assert evaluate(replace(rec, events=()), None, PACKS)['dock.events_agree'].status == Status.UNSCORABLE


def test_recomputed_events_match_the_writer_rules(runs):
    """Independent implementations (the writer's _common.mission_events and this pack) agree."""
    for _, rec, _ in runs:
        mine = [(e.kind, round(e.t, 9)) for e in D.recompute_events(rec) if e.kind in D.DOCK_KINDS]
        theirs = [(e.kind, round(e.t, 9)) for e in rec.events if e.kind in D.DOCK_KINDS]
        assert sorted(mine) == sorted(theirs), rec.run_id


# ------------------------------------------------------------------ synthetic trajectories

def _synthetic(xs, ys, yaws, dt=0.02, end='timeout', speed=None, run_id='syn'):
    t = np.arange(len(xs)) * dt
    vx = np.gradient(xs, dt) if speed is None else speed
    ch = {'truth.pose': JR.truth_pose(t, xs, ys, yaws),
          'truth.twist': JR.truth_twist(t, vx, np.zeros_like(t), np.zeros_like(t))}
    h = JR.Header(run_id, 'fast', 'synthetic', 'marine.asv', JR.Robot('b'), window=(0.0, float(t[-1]), end))
    return JR.RunRecord(h, ch)


def test_gate_miss_and_retries_on_a_synthetic_track():
    # east through the gate at y = 1.6 (miss, clear of the buoy), then two passes into the slip mouth and out again
    n1 = 600
    x1, y1 = np.linspace(0, 12, n1), np.full(n1, 1.6)
    leg = np.concatenate([np.linspace(6.0, 4.0, 200), np.linspace(4.0, 6.0, 200)])
    x = np.concatenate([x1, leg, leg])
    y = np.concatenate([y1, np.full(800, -5.0)])
    yaw = np.concatenate([np.zeros(n1), np.full(800, math.pi)])
    rec = _synthetic(x, y, yaw, speed=np.full(len(x), 0.5))
    v = evaluate(rec, None, PACKS)
    assert v['dock.gate_miss'].value == 1.0 and v['dock.gate_offset'].value == pytest.approx(1.6)
    assert v['dock.retries'].value == 2.0 and v['dock.retries'].extra['entries'] == 2
    assert D.outcome(rec, v) == 'gate_miss'
    J, _ = G.score(0.0, False, 0, 1.0, min(v['dock.pose_error'].value, 3.0))
    assert v['dock.J'].value == pytest.approx(J)
    m = {'docked': np.array([0.0]), 't_dock': np.array([np.nan]), 'contacts': np.array([0.0]),
         'gate_miss': np.array([1.0]), 'pose_err': np.array([v['dock.pose_error'].value])}
    assert v['dock.J'].value == pytest.approx(float(mission.score(m)[0]))
    assert v['dock.min_wall_clearance'].value > 0.2


def test_clean_dock_track():
    n = 400
    x = np.concatenate([np.linspace(6, 2.7, n), np.full(200, 2.7)])
    y = np.full(len(x), -5.0)
    sp = np.concatenate([np.full(n, 0.4), np.zeros(200)])
    rec = _synthetic(x, y, np.full(len(x), math.pi), speed=sp, end='docked')
    v = evaluate(rec, None, PACKS)
    assert v['dock.docked'].value == 1.0 and v['dock.retries'].value == 0.0
    assert D.outcome(rec, v) == 'docked' and v['dock.gate_miss'].value == 1.0   # docked, gate never crossed
    assert v['dock.time_to_dock'].value == pytest.approx(n * 0.02, abs=0.03)
