"""Fast (numpy) layer writer: one (boat, controller, noise seed) dock mission at full rate.

``mission.run`` is called unedited; for the duration of the call ``mission.Plant`` is rebound to
``RecordingPlant``, a subclass of ``plant.Plant`` that only observes (it calls the parent for every
step and measurement and does no arithmetic of its own on the state), so the run is bit-identical to
``mission.run`` with the same inputs (tests: J and every metric equal).

What is captured, per plant tick k (t_k = ``plant.t`` after k steps, the harness clock):
  truth.pose / truth.twist  ``plant.st`` (x, y, psi | u, v, r body) at ticks 0..K
  obs.pose / obs.twist      ``plant.measure()`` as the controller received it (control ticks, 25 Hz)
  obs.stamp                 t_recv = t_k, t_acq = t at tick max(k - L, 0), L = round(odom_latency / dt)
                            (before L ticks have passed the delay buffer still holds its zero fill,
                            which equals the start state (0, 0, 0, 0, 0, 0): acquisition tick 0)
  cmd.requested             q passed to ``plant.step`` at control ticks (after the docked mask)
  cmd.applied               ``plant.q_applied`` after the dead-time line, t_apply = t_k: held over [t_k, t_k+1)
  act.thrust                ``plant.thr.T`` over [t_k, t_k+1) (the thrust the RK4 step integrated)
  env.current / env.wind    ``plant.env.current(st_k, t_k)`` / ``env.wind(t_k)`` (pure functions; the
                            values the step used)
cmd.sent is declared absent: the fast layer has no transport (q goes straight into the dead-time line).
"""

from __future__ import annotations

import math

import numpy as np

from . import _common as C
from . import _compat  # noqa: F401  (import-time schema path)

import _paths
import hidden
import mission
import plant as P
import unity_check as U

LAYER = 'fast'


class RecordingPlant(P.Plant):
    """``plant.Plant`` (n = 1) that records what mission.run gives and gets, without changing it."""

    def __init__(self, spec, seeds, state0=None, t_max=180.0, dt=P.DT):
        super().__init__(spec, seeds, state0=state0, t_max=t_max, dt=dt)
        if self.n != 1:
            raise ValueError('RecordingPlant captures one env')
        self.lat_ticks = int(np.round(spec.odom_latency / dt)[0])
        self.dead_ticks = int(np.round(spec.dead_time / dt)[0])
        self.r_t, self.r_tick, self.r_st = [self.t], [0], [self.st[0].copy()]
        self.r_obs, self.r_req, self.r_app, self.r_thr, self.r_cur, self.r_wind = [], [], [], [], [], []
        self.st0 = self.st[0].copy()

    def measure(self, channel=0):
        out = super().measure(channel)
        self.r_obs.append((self.tick, out[0][0].copy(), out[1][0].copy()))
        return out

    def step(self, q):
        k = self.tick
        if self.r_obs and self.r_obs[-1][0] == k:             # a control step happened at this tick
            self.r_req.append((k, np.asarray(q, float)[0].copy()))
        cur = self.env.current(self.st[:, :2], self.t)[0].copy()
        wind = self.env.wind(self.t)[0].copy()
        super().step(q)
        self.r_app.append((k, self.q_applied[0].copy()))
        self.r_thr.append((k, self.thr.T[0].copy()))
        self.r_cur.append((k, cur))
        self.r_wind.append((k, wind))
        self.r_t.append(self.t)
        self.r_tick.append(self.tick)
        self.r_st.append(self.st[0].copy())


def run_recorded(spec, ctrl_idx, seed, t_max=mission.T_MAX, lib=None):
    """``mission.run`` on one env with the recording plant. Returns (metrics, plant)."""
    lib = lib if lib is not None else C.library_of('v1')[0]
    holder = {}

    def factory(sp, seeds, t_max=t_max, **kw):
        holder['p'] = RecordingPlant(sp, seeds, t_max=t_max, **kw)
        return holder['p']

    original = mission.Plant
    mission.Plant = factory
    try:
        m, _ = mission.run(spec, [ctrl_idx], [seed], t_max=t_max, lib=lib)
    finally:
        mission.Plant = original
    return m, holder['p']


def boat(stratum, boat_index, hidden_seed, projection='hidden'):
    spec = U.get_boat(hidden_seed, stratum, boat_index)
    if projection == 'representable':
        spec = U.representable_numpy(spec)
    elif projection != 'hidden':
        raise ValueError(projection)
    return spec


def _channels(p):
    t = np.asarray(p.r_t)
    tick = np.asarray(p.r_tick)
    st = np.asarray(p.r_st)
    ch = {'truth.pose': C.channel('truth.pose', t, st[:, :3], tick, 'exact', 1 / P.DT),
          'truth.twist': C.channel('truth.twist', t, st[:, 3:6], tick, 'exact', 1 / P.DT)}
    ok = np.array([o[0] for o in p.r_obs])
    t_recv = t[ok]
    t_acq = t[np.maximum(ok - p.lat_ticks, 0)]
    obs_hz = 1 / (P.DT * mission.CONTROL_EVERY)
    ch['obs.pose'] = C.channel('obs.pose', t_recv, np.array([o[1] for o in p.r_obs]), ok, 'estimated', obs_hz)
    ch['obs.twist'] = C.channel('obs.twist', t_recv, np.array([o[2] for o in p.r_obs]), ok, 'estimated', obs_hz)
    ch['obs.stamp'] = C.channel('obs.stamp', t_recv, np.stack([t_acq, t_recv], -1), ok, 'exact', obs_hz)
    rk = np.array([r[0] for r in p.r_req])
    ch['cmd.requested'] = C.channel('cmd.requested', t[rk], np.array([r[1] for r in p.r_req]), rk, 'commanded', obs_hz)
    ak = np.array([a[0] for a in p.r_app])
    ch['cmd.applied'] = C.channel('cmd.applied', t[ak], np.column_stack([np.array([a[1] for a in p.r_app]), t[ak]]),
                                  ak, 'exact', 1 / P.DT)
    ch['act.thrust'] = C.channel('act.thrust', t[ak], np.array([a[1] for a in p.r_thr]), ak, 'exact', 1 / P.DT)
    ch['env.current'] = C.channel('env.current', t[ak], np.array([a[1] for a in p.r_cur]), ak, 'exact', 1 / P.DT)
    ch['env.wind'] = C.channel('env.wind', t[ak], np.array([a[1] for a in p.r_wind]), ak, 'exact', 1 / P.DT)
    return ch, t, st


ABSENT = {'cmd.sent': 'fast layer: no transport between controller and plant; cmd.requested goes straight into '
                      'the plant dead-time line (no pulse message exists)'}


def capture(stratum, boat_index, hidden_seed, ctrl_idx, seed, library='v1', projection='hidden',
            t_max=mission.T_MAX, calibration=None, evidence=None, spec=None, verify=False):
    """Run and record. Returns (intermediate record, mission metrics). ``verify`` also runs
    ``mission.run`` directly and asserts every metric equal (bit-identity)."""
    lib, lib_id = C.library_of(library)
    spec = spec if spec is not None else boat(stratum, boat_index, hidden_seed, projection)
    m, p = run_recorded(spec, ctrl_idx, seed, t_max=t_max, lib=lib)
    if verify:
        ref, _ = mission.run(spec, [ctrl_idx], [seed], t_max=t_max, lib=lib)
        for k in ref:
            a, b = np.asarray(ref[k]), np.asarray(m[k])
            assert np.array_equal(a, b, equal_nan=True), (k, a, b)
    ch, t, st = _channels(p)
    events, outcome = C.mission_events(t, st, t_max)
    C.assert_outcome_matches(outcome, m)
    row = spec.row(0)
    scen = C.mission_scenario(stratum, boat_index, hidden_seed, seed, t_max, lib_id, projection)
    scen['condition']['start_twist'] = [0.0, 0.0, 0.0]          # Plant(state0=None): at rest
    ctrl = C.controller_block(lib, lib_id, ctrl_idx)
    cal = C.calibration_block(calibration)
    code = C.code_block()
    backend = {'layer': LAYER, 'name': f'decisioncal.numpy.{projection}', 'build': None,
               'mechanisms': C.fast_mechanisms(row), 'flags': {}}
    ident = {'backend': backend, 'plant': row, 'controller': ctrl['spec_sha'], 'cal': cal,
             'code': code['files_sha'], 'scenario': scen, 't_max': t_max}
    rid = C.run_id(LAYER, scen['id'], ctrl['id'], seed, ident)
    seed_class = 'development' if hidden_seed == 4242 else None
    ev = {'kind': 'exploratory', 'prereg': None, 'seed_class': seed_class, **(evidence or {})}
    pose_err = (st[0] - p.st0)[:3].tolist()
    end = 'docked' if outcome['terminal'] == 'docked' else outcome['terminal']
    header = {
        'run_id': rid, 'tier': LAYER, 'source': 'decisioncal.records.numpy_capture', 'domain': 'marine.asv',
        'robot': {'id': 'decisioncal-boat', 'model': f'hidden:{stratum}:{boat_index}@{hidden_seed}', 'morphology': None,
                  'plant_card': None},
        'task_id': 'decisioncal.dock_mission', 'scenario': scen,
        'clock': {'base': 'sim', 'anchor': 'episode_start', 'offset_s': 0.0, 'residual_s': None},
        'window': [float(t[0]), float(t[-1]), end], 'source_version': '2',
        'backend': backend, 'controller': ctrl, 'calibration': cal,
        'clock_mode': {'mode': 'fixed_step', 'dt_s': P.DT, 'time_scale': None, 'rtf': None, 'episode': 0},
        'code': code, 'evidence': ev,
        'reset': {'method': 'construct.rest', 'verified': bool(np.abs(pose_err).max() == 0.0), 'pose_error': pose_err},
        'provenance': {
            'harness': _paths.provenance(), 'plant_spec': row,
            'latency_ticks': p.lat_ticks, 'dead_time_ticks': p.dead_ticks, 'control_every_ticks': mission.CONTROL_EVERY,
            'mission_metrics': {k: float(np.asarray(v).ravel()[0]) for k, v in m.items()},
            'outcome_from_events': outcome, 'absent_reasons': ABSENT,
            'infra': {'errors': []},
            'notes': ['record clock = plant.t (sum of dt per tick); tick = plant tick (50 Hz)',
                      'docked event payload t_dock = hold start (mission.run time to dock)',
                      'gyro (third output of measure) is not used by the controller and not recorded']}}
    rec = C.new_rec(header, ch, ABSENT, events)
    return rec, m
