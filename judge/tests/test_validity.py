"""judge.validity: one injected fault at a time fires exactly its code; mission failures stay valid.

The synthetic docking run: plant tick 0.02 s (truth at every tick), control 25 Hz (every 2nd tick),
observations acquired 30 ms and received 10 ms before the controller uses them, commands applied on
the next plant tick. The track curves (heading 0.5 -> ~2.5 rad) so frames and time shifts are
identifiable.
"""

from dataclasses import replace
import math

import numpy as np
import pytest

from judge import record as R
from judge import validity as V
from judge.record import EV, END, Event

DT, CTRL = 0.02, 2           # plant tick, control every 2 ticks (25 Hz)
DUR = 40.0
START = (-20.0, -5.0, 0.5)


def make(*, u0=0.0, end=END.DOCKED, events=None, n_ticks=None, flags=None, verified=True, method='teleport.rest',
         stamp=True, seed=0):
    rng = np.random.default_rng(seed)
    n = n_ticks or int(round(DUR / DT)) + 1
    tick = np.arange(n)
    t = tick * DT
    u = u0 + (1.0 - u0) * (1 - np.exp(-t / 2.0))
    v = 0.05 * np.sin(2 * np.pi * t / 15)
    r = 0.12 * np.sin(2 * np.pi * t / 25) + 0.04
    yaw = START[2] + np.concatenate([[0], np.cumsum(0.5 * (r[1:] + r[:-1]) * DT)])
    c, s = np.cos(yaw), np.sin(yaw)
    dx, dy = c * u - s * v, s * u + c * v
    x = START[0] + np.concatenate([[0], np.cumsum(0.5 * (dx[1:] + dx[:-1]) * DT)])
    y = START[1] + np.concatenate([[0], np.cumsum(0.5 * (dy[1:] + dy[:-1]) * DT)])
    ch = {'truth.pose': R.truth_pose(t, x, y, yaw, tick=tick),
          'truth.twist': R.truth_twist(t, u, v, r, tick=tick)}
    k = tick[CTRL::CTRL]                      # control ticks
    tc = k * DT
    t_acq, t_recv = tc - 0.03, tc - 0.01
    def at(a, tt):
        return np.interp(tt, t, a)
    ox = at(x, t_acq) + rng.normal(0, 0.03, len(k))
    oy = at(y, t_acq) + rng.normal(0, 0.03, len(k))
    oyaw = R.wrap(at(yaw, t_acq) + rng.normal(0, 0.01, len(k)))
    ch['obs.pose'] = R.obs_pose(tc, ox, oy, oyaw, tick=k)
    ch['obs.twist'] = R.obs_twist(tc, *(at(a, t_acq) + rng.normal(0, 0.02, len(k)) for a in (u, v, r)), tick=k)
    ch['obs.stamp'] = R.obs_stamp(tc, t_acq, t_recv, tick=k) if stamp else R.declare_absent('obs.stamp', 'no stamps')
    q = np.column_stack([0.5 + 0.4 * np.sin(2 * np.pi * tc / 4 + ph) for ph in (0, 0.5, 1.0, 1.5)])
    ch['cmd.requested'] = R.cmd_requested(tc, q, tick=k)
    ch['cmd.sent'] = R.cmd_sent(tc, 1500 + 400 * q, tc + 0.005, tick=k)
    ch['cmd.applied'] = R.cmd_applied(tc + DT, q, tc + DT, tick=k + 1)
    ch['act.thrust'] = R.act_thrust(tc + DT, 30 * q, tick=k + 1)
    ch['env.current'] = R.env_current(t, np.full(n, 0.1), np.zeros(n), tick=tick)
    ch['env.wind'] = R.declare_absent('env.wind', 'the fast layer models no wind')
    if events is None:
        events = (Event(0.0, EV.EPISODE_START), Event(12.0, EV.GATE_CROSS, payload={'y': 3.0}),
                  Event(35.0, EV.DOCK_ENTER), Event(36.0, EV.DOCK_HOLD_START), Event(t[-1], EV.DOCKED))
    h = R.Header('dock-s1', 'fast', 'synthetic', 'planar', R.Robot('vessel'), task_id='dock',
                 scenario=R.Scenario('dock-s1', None, 1, 'dock', {'start_pose': list(START)}),
                 window=(0.0, float(t[-1]), end),
                 backend=R.Backend('fast', 'numpy-plant', 'abc123', ('thrust.lag.first_order',), flags or {}),
                 controller=R.Controller('pid-a', 'lib-v3', 'sha-c', {'kp': 1.5}),
                 calibration=R.Calibration('twin-b', 'arm-1', 'sha-e', ('m4', 'm5')),
                 clock_mode=R.ClockMode('fixed_step', DT, 1.0, None, 0),
                 code=R.Code('crane', 'deadbeef', False, {'judge/record.py': 'aa'}),
                 evidence=R.Evidence('exploratory', None, 'dev'),
                 reset=R.Reset(method, verified, (0.0, 0.0, 0.0)))
    return R.RunRecord(h, ch, tuple(events))


def failed_dock():
    ev = (Event(0.0, EV.EPISODE_START), Event(12.0, EV.GATE_CROSS, payload={'y': 3.0}),
          Event(33.0, EV.CONTACT, payload={'with': 'piling'}), Event(34.5, EV.CONTACT, payload={'with': 'piling'}),
          Event(DUR, EV.TIMEOUT))
    return make(end=END.TIMEOUT, events=ev)


def with_ch(rec, **chs):
    out = dict(rec.channels)
    for k, v in chs.items():
        name = k.replace('__', '.')
        if v is None:
            out.pop(name)
        else:
            out[name] = v
    return replace(rec, channels=out)


def edit(ch, t=None, v=None, tick=None):
    return R.Channel(ch.spec, ch.t if t is None else t, ch.v if v is None else v, ch.sigma, ch.gaps,
                     ch.tick if tick is None else tick)


def codes(rec, **kw):
    rep = V.check(rec, **kw)
    return rep, rep.codes


# --------------------------------------------------------------------------- baselines

def test_clean_run_is_valid_with_no_reasons():
    rep = V.check(make())
    assert rep.valid and rep.reasons == (), [r.message for r in rep.reasons]
    assert 'frame.obs_pose' in rep.checked and 'cmd.delivery_gap' in rep.checked


def test_legitimately_failed_dock_is_valid():
    rep = V.check(failed_dock())
    assert rep.valid and rep.reasons == (), [r.message for r in rep.reasons]


def test_clean_run_without_stamps_is_valid():
    rep = V.check(make(stamp=False))
    assert rep.valid and rep.reasons == (), [r.message for r in rep.reasons]
    assert 'clock.offset.stamps' in rep.skipped


# ------------------------------------------------------------------------------- clock

def test_obs_time_base_offset():
    rec = make()
    sh = {}
    for n in ('obs.pose', 'obs.twist'):
        sh[n.replace('.', '__')] = edit(rec.channels[n], t=rec.channels[n].t + 0.2)
    st = rec.channels['obs.stamp']
    sh['obs__stamp'] = edit(st, t=st.t + 0.2, v=st.v + 0.2)
    rep, c = codes(with_ch(rec, **sh))
    assert c == ['clock.offset'] and not rep.valid
    shift = [r for r in rep.reasons if 'shift_s' in r.evidence][0]
    assert abs(shift.evidence['shift_s'] + 0.2) < 0.02               # obs matches truth 0.2 s earlier
    assert any(r.evidence.get('against') == 'obs.stamp' for r in rep.reasons)   # command before its obs
    ticks = [r for r in rep.reasons if r.evidence.get('pairing') == 'tick' and r.evidence['against'] == 'truth.pose']
    assert ticks and all(abs(r.evidence['median_s'] - 0.2) < 1e-6 for r in ticks)


def test_obs_time_base_offset_without_stamps_is_seen_through_ticks():
    rec = make(stamp=False)
    rec = with_ch(rec, **{n.replace('.', '__'): edit(rec.channels[n], t=rec.channels[n].t + 0.2)
                          for n in ('obs.pose', 'obs.twist')})
    rep, c = codes(rec)
    assert c == ['clock.offset'] and {r.evidence['channel'] for r in rep.reasons} == {'obs.pose', 'obs.twist'}


def test_time_reversal():
    rec = make()
    tp = rec.channels['truth.pose']
    t = tp.t.copy()
    t[500:510] = t[500:510][::-1]
    rep, c = codes(with_ch(rec, truth__pose=edit(tp, t=t)))
    assert c == ['clock.nonmonotonic'] and not rep.valid
    assert rep.reasons[0].evidence['channel'] == 'truth.pose' and rep.reasons[0].evidence['field'] == 't'


def test_duplicated_tick():
    rec = make()
    tp = rec.channels['truth.pose']
    tk = tp.tick.copy()
    tk[300] = tk[299]
    rep, c = codes(with_ch(rec, truth__pose=edit(tp, tick=tk)))
    assert c == ['clock.nonmonotonic'] and rep.reasons[0].evidence['field'] == 'tick'


def test_dt_mismatch_time_scaled():
    rec = make()
    k = 1.25                                   # timestamps advance 25 ms per 20 ms tick
    chs = {}
    for n, ch in rec.channels.items():
        if not len(ch):
            continue
        v = ch.v.copy()
        for f in ('t_acq', 't_recv', 't_send', 't_apply'):
            if f in ch.spec.fields:
                v[:, ch.spec.fields.index(f)] *= k
        chs[n.replace('.', '__')] = edit(ch, t=ch.t * k, v=v)
    rec = with_ch(rec, **chs)
    h = rec.header
    rec = replace(rec, header=replace(h, window=(0.0, h.window[1] * k, h.window[2])),
                  events=tuple(replace(e, t=e.t * k) for e in rec.events))
    rep, c = codes(rec)
    assert c == ['clock.dt_mismatch'] and not rep.valid
    ev = [r.evidence for r in rep.reasons if r.evidence.get('channel') == 'truth.pose'][0]
    assert abs(ev['slope_s_per_tick'] - 0.025) < 1e-6


def test_dt_mismatch_header_lies():
    rec = make()
    rec = replace(rec, header=replace(rec.header, clock_mode=R.ClockMode('fixed_step', 0.025, 1.0, None, 0)))
    rep, c = codes(rec)
    assert c == ['clock.dt_mismatch']


def test_rtf_starved_realtime():
    rec = make()
    rec = replace(rec, header=replace(rec.header, clock_mode=R.ClockMode('realtime', DT, 1.0, 0.4, 0)))
    rep, c = codes(rec)
    assert c == ['clock.rtf_starved'] and not rep.valid


# ------------------------------------------------------------------------------ telemetry

def test_two_second_hole():
    rec = make()
    chs = {}
    for n in ('obs.pose', 'obs.twist', 'obs.stamp'):
        ch = rec.channels[n]
        m = ~((ch.t >= 15) & (ch.t < 17))
        chs[n.replace('.', '__')] = R.Channel(ch.spec, ch.t[m], ch.v[m], None, (), ch.tick[m])
    rep, c = codes(with_ch(rec, **chs))
    assert c == ['telemetry.gap'] and not rep.valid
    assert {r.evidence['channel'] for r in rep.reasons} == {'obs.pose', 'obs.twist', 'obs.stamp'}
    assert all(abs(r.evidence['longest_s'] - 2.04) < 0.05 for r in rep.reasons)


def test_missing_required_channel():
    rep, c = codes(with_ch(make(), obs__pose=None))
    assert c == ['telemetry.missing_channel'] and not rep.valid
    assert rep.reasons[0].evidence == {'measured': 'channel present', 'channel': 'obs.pose', 'required': True}


def test_declared_absent_channel_is_no_finding():
    rec = with_ch(make(), cmd__applied=R.declare_absent('cmd.applied', 'backend does not expose applied thrust'))
    rep = V.check(rec)
    assert rep.valid and rep.reasons == ()
    assert 'declared absent' in rep.skipped['cmd.delivery_gap']


def test_unlogged_optional_channel_is_a_warning():
    rep, c = codes(with_ch(make(), cmd__applied=None))
    assert c == ['telemetry.missing_channel'] and rep.valid
    assert rep.reasons[0].severity == V.WARNING


def test_low_control_rate():
    rec = make()
    ch = rec.channels['cmd.requested']
    rep, c = codes(with_ch(rec, cmd__requested=R.Channel(ch.spec, ch.t[::2], ch.v[::2], None, (), ch.tick[::2]),
                           cmd__applied=None, cmd__sent=None))
    assert 'telemetry.rate' in c and set(c) <= {'telemetry.rate', 'telemetry.missing_channel'}


# --------------------------------------------------------------------------------- frames

def _obs(rec, f):
    ch = rec.channels['obs.pose']
    return with_ch(rec, obs__pose=edit(ch, v=f(ch.v.copy())))


def test_yaw_sign_flip():
    def f(v):
        v[:, 2] = -v[:, 2]
        return v
    rep, c = codes(_obs(make(), f))
    assert c == ['frame.mismatch'] and rep.reasons[0].evidence['kind'] == 'yaw_sign'


def test_xy_swapped():
    rep, c = codes(_obs(make(), lambda v: v[:, [1, 0, 2]]))
    assert c == ['frame.mismatch'] and rep.reasons[0].evidence['kind'] == 'reflection'


def test_rotated_90():
    def f(v):
        return np.column_stack([-v[:, 1], v[:, 0], R.wrap(v[:, 2] + np.pi / 2)])
    rep, c = codes(_obs(make(), f))
    assert c == ['frame.mismatch'] and rep.reasons[0].evidence['kind'] == 'rotation'
    assert abs(rep.reasons[0].evidence['rigid_rotation_deg'] - 90) < 2


def test_world_twist_logged_as_body():
    rec = make()
    tp, tw = rec.channels['truth.pose'], rec.channels['truth.twist']
    yaw = tp.col('yaw')
    u, v = tw.col('u'), tw.col('v')
    world = np.column_stack([np.cos(yaw) * u - np.sin(yaw) * v, np.sin(yaw) * u + np.cos(yaw) * v, tw.col('r')])
    rep, c = codes(with_ch(rec, truth__twist=edit(tw, v=world)))
    assert c == ['frame.mismatch'] and rep.reasons[0].evidence['kind'] == 'twist_world_not_body'
    assert rep.reasons[0].evidence['channel'] == 'truth.twist'


def test_truth_heading_sign_convention():
    rec = make()
    tp = rec.channels['truth.pose']
    v = tp.v.copy()
    v[:, 2] = -v[:, 2]
    rec = with_ch(rec, truth__pose=edit(tp, v=v))
    # the scenario declares its start in the same (wrong) convention, so only the frame check can see it
    sc = replace(rec.header.scenario, condition={'start_pose': [START[0], START[1], -START[2]]})
    rep, c = codes(replace(rec, header=replace(rec.header, scenario=sc)))
    assert c == ['frame.mismatch']
    assert {r.evidence['kind'] for r in rep.reasons} >= {'truth_yaw_sign'}


# --------------------------------------------------------------------------------- reset

def test_start_three_metres_off():
    rec = make()
    chs = {}
    for n in ('truth.pose', 'obs.pose'):
        ch = rec.channels[n]
        v = ch.v.copy()
        v[:, 0] += 3.0
        chs[n.replace('.', '__')] = edit(ch, v=v)
    rep, c = codes(with_ch(rec, **chs))
    assert c == ['reset.failed'] and not rep.valid
    assert abs(rep.reasons[0].evidence['pos_error_m'] - 3.0) < 1e-6


def test_initial_speed_when_reset_claims_rest():
    rep, c = codes(make(u0=0.5))
    assert c == ['reset.failed']
    assert abs(rep.reasons[0].evidence['speed_error'] - 0.5) < 1e-6


def test_unverified_reset_without_declared_start():
    rec = make(verified=None, method=None)
    rec = replace(rec, header=replace(rec.header, scenario=R.Scenario('dock-s1', None, 1, 'dock', {})))
    rep, c = codes(rec)
    assert c == ['reset.unverified'] and rep.valid


def test_unverified_flag_is_quiet_when_the_judge_checked_the_start():
    rep = V.check(make(verified=False))
    assert rep.valid and rep.reasons == ()


# ------------------------------------------------------------------------------ commands

def test_command_delivery_gap():
    rec = make()
    ap = rec.channels['cmd.applied']
    v = ap.v.copy()
    m = (ap.t >= 10) & (ap.t < 12)
    v[m, :4] = v[np.argmax(m) - 1, :4]          # applied holds a stale command for 2 s
    rep, c = codes(with_ch(rec, cmd__applied=edit(ap, v=v)))
    assert c == ['cmd.delivery_gap'] and not rep.valid
    span = rep.reasons[0].evidence['longest_span']
    assert 9.9 <= span[0] <= 10.6 and 11.3 <= span[1] <= 12.1


def test_nonfinite_command_is_rejected():
    rec = make()
    ch = rec.channels['cmd.requested']
    v = ch.v.copy()
    v[100, 0] = np.nan
    ap = rec.channels['cmd.applied']
    rep, c = codes(with_ch(rec, cmd__requested=edit(ch, v=v)))
    assert c == ['cmd.rejected']


# --------------------------------------------------------------------------------- infra

def test_link_lost():
    n = int(round(25.0 / DT)) + 1
    ev = (Event(0.0, EV.EPISODE_START), Event(12.0, EV.GATE_CROSS, payload={'y': 3.0}), Event(25.0, EV.LINK_LOST))
    rep, c = codes(make(n_ticks=n, end=END.LINK_LOST, events=ev))
    assert c == ['infra.link_lost'] and not rep.valid


def test_backend_flag():
    rep, c = codes(make(flags={'physics_substep_overrun': '12 steps'}))
    assert c == ['infra.backend_flag'] and not rep.valid
    assert rep.reasons[0].evidence['flag'] == 'physics_substep_overrun'
    rep = V.check(make(flags={'physics_substep_overrun': '12 steps'}),
                  expect=V.Expect(benign_flags=('physics_substep_overrun',)))
    assert rep.valid and rep.codes == ['infra.backend_flag']


def test_infra_error_terminal_is_crash():
    rep, c = codes(make(end=END.INFRA_ERROR))
    assert c == ['infra.crash']


# --------------------------------------------------------------------------------- schema

def test_obs_is_truth_undeclared_is_a_schema_reason():
    rec = make()
    tp = rec.channels['truth.pose']
    obs = R.Channel(R.standard_spec('obs.pose'), tp.t, tp.v, None, (), tp.tick)
    rep, c = codes(with_ch(rec, obs__pose=obs))
    assert 'record.schema' in c and not rep.valid
    assert any(r.evidence.get('how') == 'same array object' for r in rep.of('record.schema'))


def test_v1_record_is_a_schema_warning():
    from judge.packs import toy_quad as tq
    rep = V.check(tq.simulate(tq.PARAMS_LOW, 1, 'low'), expect=V.Expect(required=(), accounted=(), rates_hz={}))
    assert [r.severity for r in rep.of('record.schema')][0] == V.WARNING


# ---------------------------------------------------------------------------------- apply

def test_apply_writes_validity():
    rec = make(flags={'physics_substep_overrun': '12 steps'})
    out = V.apply(rec, V.check(rec))
    assert not out.validity.valid
    assert out.validity.flags == {'infra.backend_flag': 'invalid'}
    assert out.validity.excluded_from == frozenset({'G3', 'G4'})
    clean = V.apply(make(), V.check(make()))
    assert clean.validity.valid and clean.validity.flags == {} and not clean.validity.excluded_from
    with pytest.raises(ValueError):
        V.apply(replace(rec, header=replace(rec.header, run_id='other')), V.check(rec))


def test_report_is_deterministic_and_json_ready():
    import json
    a, b = V.check(make()), V.check(make())
    assert json.dumps(a.as_dict(), sort_keys=True) == json.dumps(b.as_dict(), sort_keys=True)
    bad = V.check(make(u0=0.5))
    json.dumps(bad.as_dict())
