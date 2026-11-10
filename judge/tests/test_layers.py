"""Layer divergence on matched pairs from the REAL fast plant, each differing by exactly one
injected cause (judge/insight/layers.py). The reference side A is the nominal twin boat (no dead
time, latency or lag); B differs as stated. Every injection is active from t = 0 except the
time-base jump, so the stated tolerance on the first divergence is against onset 0 (or 5 s)."""

from dataclasses import replace

import numpy as np
import pytest

import dock_sim as S
import hidden

from judge.insight import layers as L
from judge.packs import dock_mission as D

T = 15.0
ONSET_TOL_S = 0.3          # first divergence within 0.3 s (15 ticks) of the injected onset


@pytest.fixture(scope='module')
def A():
    return S.run(t_max=T, run_id='A-fast')[0]


def _b(spec=None, **kw):
    kw.setdefault('t_max', T)
    kw.setdefault('run_id', 'B-engine')
    kw.setdefault('layer', 'integrated')
    kw.setdefault('backend', 'engine.test')
    return S.run(spec, **kw)[0]


def _check(res, cause, onset=0.0, tol=ONSET_TOL_S):
    assert res.verdict == 'diverged', res.explanation
    assert res.attribution == cause, res.explanation
    assert res.first_s is not None and abs(res.first_s - onset) <= tol, (res.first_s, res.explanation)
    assert res.insight is not None and res.insight.kind == 'layers.divergence'
    return res


# (a) clock / timestamp ----------------------------------------------------------------------

def test_a_three_tick_time_base_offset_with_ticks(A):
    B = S.shift_time(_b(), 3 * S.DT)
    res = _check(L.diverge(A, B), 'clock_or_timestamp')
    assert res.quantify['clock']['offset_ticks'] == pytest.approx(3.0, abs=1e-6)
    assert res.secondary['attribution'] is None            # after the correction the runs agree


def test_a_three_tick_offset_without_ticks_is_found_by_the_lag(A):
    B = S.strip_ticks(S.shift_time(_b(), 3 * S.DT))
    res = L.diverge(S.strip_ticks(A), B)
    assert res.attribution == 'clock_or_timestamp', res.explanation
    assert res.quantify['clock']['offset_s'] == pytest.approx(0.06, abs=0.005)
    assert res.alignment['time_base'] == 'record clock'


def test_a_time_base_jump_mid_run_is_localised(A):
    B = _b()
    ch = {}
    for n, c in B.channels.items():
        if B.has(n):
            ch[n] = replace(c, t=np.where(c.t >= 5.0 - 1e-9, c.t + 3 * S.DT, c.t))
        else:
            ch[n] = c
    res = _check(L.diverge(A, replace(B, channels=ch)), 'clock_or_timestamp', onset=5.0, tol=S.DT + 1e-9)
    assert res.quantify['clock']['offset_ticks'] == pytest.approx(3.0)


def test_a_constant_tick_counter_offset_is_harmless(A):
    B = _b()
    ch = {n: replace(c, tick=c.tick + 3) if B.has(n) and c.tick is not None else c for n, c in B.channels.items()}
    ev = tuple(replace(e, payload={**e.payload, 'tick': 3}) if e.kind == 'episode_start' else e for e in B.events)
    res = L.diverge(A, replace(B, channels=ch, events=ev))
    assert res.verdict == 'agree', res.explanation
    assert res.alignment['tick_offset'] == 3


# (b) command delivery -----------------------------------------------------------------------

def test_b_extra_command_dead_time_is_command_delivery(A):
    res = _check(L.diverge(A, _b(S.nominal_spec(dead_time=0.1))), 'command_delivery')
    assert res.quantify['delivery_lag_s'] == pytest.approx(0.1, abs=S.DT + 1e-9)
    order = [d['group'] for d in res.departures]
    assert order[0] == 'delivery'
    assert next(d for d in res.departures if d['signal'] == 'cmd.requested')['depart_s'] > \
        next(d for d in res.departures if d['signal'] == 'cmd.applied')['depart_s']


# (c) plant ----------------------------------------------------------------------------------

def test_c_thruster_lag_is_plant_thrust_response():
    A2 = S.run(S.nominal_spec(tau_up=0.05, tau_down=0.05), t_max=T, run_id='A-lag')[0]
    res = _check(L.diverge(A2, _b(S.nominal_spec(tau_up=0.3, tau_down=0.3))), 'plant')
    assert res.quantify['response'] == 'thrust'
    # delivery agrees within the onset level (0.25 x the 0.05 command threshold) up to its own departure
    th = L.Thresholds()
    assert res.quantify['applied_max_diff_in_window'] <= th.onset_frac * th.cmd
    assert next(d for d in res.departures if d['group'] == 'thrust')['depart_s'] == 0.0


def test_c_twenty_percent_less_thrust_is_plant(A):
    s = S.nominal_spec()
    res = _check(L.diverge(A, _b(S.nominal_spec(k_fwd=s.k_fwd * 0.8, k_rev=s.k_rev * 0.8))), 'plant')
    assert res.quantify['response'] == 'thrust'


def test_c_hull_damping_is_plant_hull_response(A):
    s = S.nominal_spec()
    # damping acts only once the boat moves: the response separates ~0.3 s after the start
    res = _check(L.diverge(A, _b(S.nominal_spec(d1=s.d1 * 1.6, d2=s.d2 * 1.6))), 'plant', tol=0.5)
    assert res.quantify['response'] == 'hull'


# (d) sensor ---------------------------------------------------------------------------------

def test_d_extra_odometry_latency_is_sensor(A):
    res = _check(L.diverge(A, _b(S.nominal_spec(odom_latency=0.1))), 'sensor')
    assert res.quantify['b.obs_latency_s'] == pytest.approx(0.1, abs=S.DT + 1e-9)
    assert res.quantify['a.obs_latency_s'] == pytest.approx(0.0, abs=S.DT + 1e-9)


def test_d_extra_position_noise_is_sensor(A):
    _check(L.diverge(A, _b(S.nominal_spec(pos_sd=0.25))), 'sensor')


# (e) controller -----------------------------------------------------------------------------

def test_e_undeclared_controller_parameter_is_controller(A):
    lib = (S.LIBRARY[0], replace(S.LIBRARY[0], cruise=1.0))
    B = _b(lib=lib, ctrl=1, ctrl_id='c00', ctrl_params={'idx': 0})      # same declared controller
    res = _check(L.diverge(A, B), 'controller')
    assert 'nondeterminism' in res.explanation


def test_e_declared_controller_mismatch_is_refused_unless_allowed(A):
    B = _b(ctrl=9)
    assert L.diverge(A, B).verdict == 'refused'
    res = L.diverge(A, B, allow_mismatch=True)
    assert res.guard['status'] == 'warn' and res.attribution == 'controller'


def test_guard_refuses_another_seed(A):
    res = L.diverge(A, _b(seed=8))
    assert res.verdict == 'refused' and 'scenario.seed' in res.explanation


# (f) ambiguous ------------------------------------------------------------------------------

@pytest.mark.parametrize('spec_kw', [{'dead_time': 0.1}, {'tau_up': 0.3, 'tau_down': 0.3}])
def test_f_delay_or_lag_without_applied_commands_is_unknown(A, spec_kw):
    B = _b(S.nominal_spec(**spec_kw), absent=('cmd.applied', 'act.thrust'))
    res = _check(L.diverge(A, B), 'unknown')
    assert res.resolve_with == ['cmd.applied on B']
    assert 'cmd.applied on B' in res.explanation


# (g) identical, (h) a legitimate failed dock on both sides --------------------------------

def test_g_identical_runs_do_not_diverge(A):
    res = L.diverge(A, _b())
    assert res.verdict == 'agree' and res.attribution is None and res.first_s is None
    assert res.events['first_difference'] is None and res.residual['time_max_m'] == 0.0


def test_h_failed_dock_on_both_sides_is_no_divergence_but_a_failure():
    spec = hidden.sample_hidden(8, 4242, 'timing').take([5])
    a = S.run(spec, t_max=60.0, run_id='A-fail')[0]
    b = S.run(spec, t_max=60.0, run_id='B-fail', layer='integrated', backend='engine.test')[0]
    res = L.diverge(a, b, outcome=D.outcome_time)
    assert res.verdict == 'agree'
    assert res.outcome['a'] == res.outcome['b'] == 'contact_fail' and not res.outcome['differ']


def test_resampled_when_dt_differs(A):
    res = L.diverge(A, S.decimate(_b(), 2, 2 * S.DT))
    assert res.alignment['resampled'] and 'resampled' in res.alignment['note']
    assert res.verdict == 'agree'
    assert any(i['code'] == 'clock.dt_mismatch' for i in res.guard['issues'])


# evidence, output, batch --------------------------------------------------------------------

def test_evidence_window_and_csv(A, tmp_path):
    res = L.diverge(A, _b(S.nominal_spec(dead_time=0.1)))
    ev = res.evidence
    assert ev['t0'] <= res.first_s <= ev['t1']
    cols = ev['columns']
    for c in ('tau', 'a.t', 'b.t', 'a.cmd.requested.q0', 'b.cmd.applied.q0', 'a.obs.pose.x', 'b.truth.pose.x',
              'res.cmd.applied'):
        assert c in cols
    i0, i1 = ev['grid_index']
    assert ev['rows'].shape == (i1 - i0, len(cols))
    assert ev['sample_index']['b']['cmd.applied'][1] > ev['sample_index']['b']['cmd.applied'][0]
    csv = L.evidence_csv(res).splitlines()
    assert csv[0].split(',') == cols and len(csv) == i1 - i0 + 1
    import json
    json.dumps(res.as_dict())                        # serialisable
    assert res.insight.evidence[0].run_id == A.run_id


def test_batch_summary(A):
    rs = [L.diverge(A, _b()), L.diverge(A, _b(S.nominal_spec(dead_time=0.1))),
          L.diverge(A, _b(S.nominal_spec(odom_latency=0.1))), L.diverge(A, _b(S.nominal_spec(dead_time=0.1)))]
    s = L.summarize_pairs(rs)
    assert s['by_attribution'] == {'agree': 1, 'command_delivery': 2, 'sensor': 1}
    assert s['first_divergence_s']['n'] == 3 and s['first_divergence_s']['max'] <= ONSET_TOL_S


def test_divergence_precedes_the_outcome_difference():
    """Full missions: 0.3 s extra dead time on B costs the dock; the divergence comes first."""
    a = S.run(t_max=60.0, run_id='A-full')[0]
    b = S.run(S.nominal_spec(dead_time=0.3), t_max=60.0, run_id='B-full', layer='integrated',
              backend='engine.test')[0]
    res = L.diverge(a, b, outcome=D.outcome_time)
    s = L.summarize_pairs([res])
    if res.outcome['differ']:
        assert res.outcome['divergence_precedes_outcome'] and s['divergence_precedes_outcome'] == 1
    else:                                            # same category: the dock time differs instead
        assert res.outcome['t_decided'][0] != res.outcome['t_decided'][1]
    assert res.attribution == 'command_delivery'
