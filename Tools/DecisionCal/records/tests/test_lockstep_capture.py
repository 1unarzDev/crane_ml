"""Integrated-layer writer against unity_check's built-in fake player (no Unity)."""

import numpy as np

from records import _common as C
from records import _compat
from records import lockstep_capture as L
from records import numpy_capture as N

import mission


def test_fake_player_identity_start_equals_mission_run():
    ref, _ = mission.run(N.boat('steady', 0, 4242), [8], [1])
    rec, m = L.capture_fake('steady', 0, 4242, 8, 1)
    for k in ('J', 'docked', 't_dock', 'contacts', 'gate_miss'):
        assert np.array_equal(np.asarray(ref[k]), np.asarray(m[k]), equal_nan=True), k
    C.assert_outcome_matches(rec['header']['provenance']['outcome_from_events'], ref)
    assert set(rec['absent']) == {'cmd.applied', 'act.thrust', 'env.current', 'env.wind'}
    assert set(rec['channels']) | set(rec['absent']) >= set(C.STANDARD)
    h = rec['header']
    assert h['backend']['layer'] == 'integrated' and h['clock_mode']['mode'] == 'lockstep'
    assert abs(h['clock_mode']['dt_s'] - 0.02) < 1e-12
    assert h['reset'] == {'method': 'scene_load', 'verified': True, 'pose_error': [0.0, 0.0, 0.0]}
    assert h['provenance']['infra']['link_errors'] == []
    ch = rec['channels']
    # pulses are 1500 + 400 q of the held request, one per tick, sent at the tick they are applied
    sent, req = ch['cmd.sent'], ch['cmd.requested']
    held = np.zeros((len(sent['t']), 4))
    for j, k in enumerate(req['tick']):
        held[k:] = np.clip(req['v'][j], -1, 1)
    assert np.allclose(sent['v'][:, :4], 1500 + 400 * held, atol=0, rtol=0)
    assert np.array_equal(sent['v'][:, 4], sent['t'])
    # the numpy capture of the same case has the same truth (the fake player is the numpy plant)
    nrec, _ = N.capture('steady', 0, 4242, 8, 1)
    # (ulp-level: the state goes body -> world -> body through float repr and the yaw wrap)
    assert np.allclose(nrec['channels']['truth.pose']['v'], ch['truth.pose']['v'], atol=1e-9, rtol=0)
    assert np.allclose(nrec['channels']['obs.pose']['v'], ch['obs.pose']['v'], atol=1e-9, rtol=0)
    # record clocks differ by design (numpy: sum of dt; lockstep: the player's t = tick * dt in the fake)
    assert np.allclose(nrec['channels']['obs.stamp']['v'], ch['obs.stamp']['v'], atol=1e-9, rtol=0)
    assert np.array_equal(nrec['channels']['obs.stamp']['tick'], ch['obs.stamp']['tick'])


def test_fake_player_rotated_start_frames():
    rec, m = L.capture_fake('steady', 0, 4242, 0, 1, start=(12.5, -7.25, 2.2))
    nrec, _ = N.capture('steady', 0, 4242, 0, 1)
    a, b = rec['channels']['truth.pose']['v'], nrec['channels']['truth.pose']['v']
    n = min(len(a), len(b))
    d = a[:n] - b[:n]
    d[:, 2] = (d[:, 2] + np.pi) % (2 * np.pi) - np.pi
    assert np.abs(d[:500]).max() < 1e-4                      # float round trips through the world frame
    w = rec['channels']['link.state_world']['v']
    assert abs(w[0, 0] - 12.5) < 1e-9 and abs(w[0, 2] - 2.2) < 1e-9
    assert rec['header']['reset']['verified'] is True


def test_wall_limit_is_recorded_as_abort(tmp_path):
    rec, m = L.capture_fake('steady', 0, 4242, 8, 1, mission_wall=0.0)
    assert m is None
    assert rec['header']['window'][2] == 'abort'
    assert any(e['kind'] == 'abort' and e['payload']['why'] == 'wall_limit' for e in rec['events'])
    assert rec['header']['provenance']['infra']['link_errors'][0]['why'] == 'wall_limit'
    _compat.save_intermediate(rec, tmp_path / 'x')           # a truncated run still writes


def test_mechanisms_from_unity_config():
    import unity_check as U
    spec = U.get_boat(4242, 'transient', 0)
    profile, faults, scenario, lost = U.unity_config(spec, 0)
    mech, not_rep = L.integrated_mechanisms(profile, U.check_faults(faults), scenario, lost)
    assert 'thrust.lag.first_order' in mech and 'thrust.static_map.kq2' in mech
    for k in ('thrust.reversal_hold', 'thrust.slew', 'thrust.lag.up_down', 'thrust.exponent'):
        assert k in not_rep and k not in mech


def test_fake_v2_record_valid(tmp_path):
    import judge.record as JR
    import judge.validity as V
    import judge.packs.dock_mission as D
    rec, _ = L.capture_fake('steady', 0, 4242, 8, 1, start=(5.0, -3.0, 0.7))
    _compat.save(rec, tmp_path / 'r')
    jr = JR.load(tmp_path / 'r')
    assert jr.header.clock_mode.mode == 'lockstep' and jr.header.reset.method == 'scene_load'
    assert set(JR.declared_absent(jr)) == {'cmd.applied', 'act.thrust', 'env.current', 'env.wind'}
    assert jr.has('env.current.commanded') and jr.has('link.clock')
    rep = V.check(jr)
    assert rep.valid, rep.as_dict()
    assert D.facts(jr)['docked'] is True and D.event_check(jr)['disagreements'] == []
