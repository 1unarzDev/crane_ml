"""Integrated layer writer: the dock mission on the CRANE Unity player over the lockstep link.

The mission is ``mission.run`` unedited, the plant ``unity_check.UnityPlant`` (subclassed here only to
record), the link ``unity_check.Link`` (subclassed only to record what crosses it). Per tick the link
gives ``s tick t x y yaw vx vy r``: the boat ArticulationBody root pose (world-ROS of /crane/odom, yaw
CCW from +x) and its WORLD-frame linear velocity and yaw rate, read in the player's FixedUpdate before
that tick's pulses are applied and before the physics step, printed as float32 ("R"), and the player's
episode time ``t`` (``Time.fixedTimeAsDouble - episodeStart`` cast to float32).

Channels:
  truth.pose (exact)        reported pose -> course frame (``Frame.world_to_state``), ticks 0..K
  truth.twist (exact)       reported world velocity rotated into the body with the world yaw, r as reported
  obs.pose / obs.twist      the HARNESS sensor model (``plant.Sensors``: odometry latency + noise from the
                            hidden spec, counter-based draws) applied to the reported state, as the
                            controller received it; obs.stamp t_acq = t(k - L), t_recv = t(k)
  cmd.requested             q at control ticks
  cmd.sent                  the pulses written to the link (1500 + 400 q per thruster) + t_send = the
                            record time of the tick they were sent at (applied in that FixedUpdate)
  env.current.commanded     the uniform current set by the ``scenario`` line (course frame, constant;
                            env.current itself is declared absent: /2 allows only 'exact' there)
  link.state_world          the reported fields unconverted (world-ROS, world-frame velocity)
  link.clock                harness clock (``UnityPlant.t``) and player tick per state line
Declared absent: env.current (see above), cmd.applied, act.thrust (the link does not report what the plant applied; the
lockstep pulses call ``Thruster.SetPulse`` directly, bypassing ``CraneActionGate``, so the
``--crane-record`` stream's accepted-action log (application ticks) has no entry for them -- the
player's own result.json shows acceptedActions 0 for lockstep runs), env.wind (Unity adds its own
25 % gusts and 10 deg veer with its own seed; the actual wind is not reported).

Record clock = the player's episode time ``t`` (backend clock); events are reconstructed on the harness
clock by mission.run's rules (asserted equal to mission.run's metrics) and stamped with the record clock.
"""

from __future__ import annotations

import json
import math
import os
import shlex
import subprocess
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from . import _common as C

import _paths
import mission
import plant as P
import unity_check as U

LAYER = 'integrated'
HEAVY = U.HEAVY
PLAYER_DIR = U.PLAYER.parent
RESET_TOL = (0.05, 0.05, 0.02)        # m, m, rad: first reported pose vs the hello start pose


class RecordingLink(U.Link):
    """``unity_check.Link`` that keeps the hello line, every state line and every pulse it sends."""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.hello_raw, self.states, self.sent, self.scenario_sent = None, [], [], None
        self.n_pulses = 0

    def _read(self):
        parts = super()._read()
        if parts and parts[0] == 'hello' and self.hello_raw is None:
            self.hello_raw = parts
        return parts

    def scenario(self, *v):
        self.scenario_sent = [float(x) for x in v]
        super().scenario(*v)

    def read_state(self):
        tick, t, fields = super().read_state()
        self.states.append((tick, t, fields))
        return tick, t, fields

    def pulses(self, p):
        self.sent.append(np.asarray(p, float).copy())
        super().pulses(p)


class RecordingUnityPlant(U.UnityPlant):
    def __init__(self, link, spec, seeds, t_max=120.0, deadline=None):
        self.r_obs, self.r_req, self.r_st, self.r_t = [], [], [], []
        super().__init__(link, spec, seeds, t_max=t_max, deadline=deadline, trace_every=10 ** 9)
        self.lat_ticks = int(np.round(spec.odom_latency / P.DT)[0])

    def _receive(self, first=False):
        super()._receive(first)
        self.r_st.append(self.st[0].copy())
        self.r_t.append(self.t)

    def measure(self, channel=0):
        out = super().measure(channel)
        self.r_obs.append((self.tick, out[0][0].copy(), out[1][0].copy()))
        return out

    def step(self, q):
        if self.r_obs and self.r_obs[-1][0] == self.tick:
            self.r_req.append((self.tick, np.asarray(q, float)[0].copy()))
        super().step(q)


def run_over_link(link, spec, ctrl_idx, seed, t_max=mission.T_MAX, deadline=None, lib=None):
    """``mission.run`` with the recording Unity plant on an accepted ``link``. Returns
    (metrics or None, plant or None, error dict or None); a wall limit or link error is captured."""
    lib = lib if lib is not None else C.library_of('v1')[0]
    holder, err = {}, None

    def factory(sp, seeds, t_max=t_max, **_):
        holder['p'] = RecordingUnityPlant(link, sp, seeds, t_max=t_max, deadline=deadline)
        return holder['p']

    original = mission.Plant
    mission.Plant = factory
    m = None
    try:
        m, _ = mission.run(spec, [ctrl_idx], [seed], t_max=t_max, lib=lib)
    except U.WallLimit as e:
        err = {'kind': 'abort', 'why': 'wall_limit', 'detail': str(e)}
    except (U.LinkError, OSError) as e:
        err = {'kind': 'link_lost', 'why': type(e).__name__, 'detail': str(e)}
    finally:
        mission.Plant = original
        if 'p' in holder:
            holder['p'].close()
    return m, holder.get('p'), err


# ---------------------------------------------------------------------------------------------
# mechanisms of the integrated layer (from unity_check.unity_config)
# ---------------------------------------------------------------------------------------------

def integrated_mechanisms(profile, faults, scenario, lost):
    m = ['rigid_body.unity_articulation.implicit_50hz', 'water.hdrp.heave_roll_pitch', 'thrust.static_map.kq2',
         'obs.latency.harness_model', 'obs.noise.harness_model']
    if any(abs(r - 1.0) > 1e-12 for r in faults['reverse']):
        m.append('thrust.reverse_ratio')
    if any(d > 0 for d in faults['deadband_us']):
        m.append('thrust.deadband.hard')
    if max(faults['deadtime_s']) >= P.DT / 2:
        m.append('cmd.dead_time')
    if faults['spinup'] > 0.001:
        m.append('thrust.lag.first_order')
    if scenario['cur_speed'] > 0:
        m.append('current.mean')
    if scenario['wind_speed'] > 0:
        m += ['wind.mean', 'wind.gust.unity_fixed']
    not_rep = []
    if lost['rev_hold_s'] > 0:
        not_rep.append('thrust.reversal_hold')
    if math.isfinite(lost['slew_N_per_s']):
        not_rep.append('thrust.slew')
    if abs(lost['tau_up_s'] - lost['tau_down_s']) > 1e-12:
        not_rep.append('thrust.lag.up_down')
    if abs(lost['gamma'] - 2.0) > 1e-12:
        not_rep.append('thrust.exponent')
    if any(d > 0 for d in faults['deadband_us']):
        not_rep.append('thrust.deadband.rescaled')
    if lost['ou_sigma'] > 0:
        not_rep.append('current.ou')
    if lost['spatial_amp'] > 0:
        not_rep.append('current.spatial')
    if lost['gust_sigma'] > 0:
        not_rep.append('wind.gust (harness sigma)')
    if scenario['wind_speed'] == 0:
        not_rep.append('wind.drag in still air')
    return sorted(m), sorted(not_rep)


def player_build():
    """(build string, manifest facts) of the cached player."""
    commit = (PLAYER_DIR / 'commit').read_text().strip() if (PLAYER_DIR / 'commit').is_file() else None
    man = {}
    mf = PLAYER_DIR / 'crane-build-manifest.json'
    if mf.is_file():
        d = json.loads(mf.read_text())
        man = {k: d.get(k) for k in ('schema', 'unityVersion', 'buildGuid', 'sourceCommit', 'sourceDirty')}
        man['manifest_sha256'] = C.sha256_file(mf)
    return commit, man


# ---------------------------------------------------------------------------------------------
# record
# ---------------------------------------------------------------------------------------------

def backend_flags(infra, err):
    """Faults the player/link reported, as header.backend.flags (judge.validity decides what they mean).
    Facts that are not faults stay in provenance.infra only."""
    f = {}
    if err is not None and err['kind'] == 'link_lost':
        f['link_lost'] = f"{err['why']}: {err['detail']}"
    rc = infra.get('player_rc')
    if rc not in (0, None) and not infra.get('player_killed'):
        f['crash'] = f'player exit code {rc}'
    w = infra.get('water_invalid_searches')
    if w:
        f['water_invalid_searches'] = f'{w} invalid HDRP water searches (result.json invalidWaterSearches)'
    pr = infra.get('player_result') or {}
    if pr.get('loggedExceptions'):
        f['player_exceptions'] = f"{pr['loggedExceptions']} exceptions in player.log"
    return f


ABSENT = {
    'cmd.applied': 'integrated layer: the lockstep link reports state only; the player applies dead time and spin-up '
                   'inside Thruster, and lockstep pulses bypass CraneActionGate, so --crane-record has no accepted-'
                   'action (application tick) entries for them (result.json acceptedActions = 0)',
    'act.thrust': 'integrated layer: Unity thruster force is not reported over the link (UnityPlant.thr.T is the '
                  'harness static map of the commanded q, a model, not the plant)',
    'env.current': 'integrated layer: the current Unity applied is not reported; the commanded uniform value '
                   '(scenario line) is in env.current.commanded',
    'env.wind': 'integrated layer: Unity adds its own gusts/veer with its own seed and does not report the wind; the '
                'configured mean is in scenario.condition',
}


def build_record(link, plant, m, err, spec, stratum, boat_index, hidden_seed, ctrl_idx, seed, lib_id, lib,
                 t_max, scenario, profile, faults, lost, backend_name, build, manifest, clock_extra, infra,
                 calibration=None, evidence=None, mechanisms=None):
    st = np.asarray(plant.r_st)
    K = len(st) - 1
    states = link.states[-(K + 1):]                         # the states this plant received (tick0..)
    ticks_p = np.array([s[0] for s in states], np.int64)
    rec_t = np.array([s[1] for s in states], float)       # player episode time (record clock)
    t_h = np.asarray(plant.r_t)
    world = np.array([s[2] for s in states], float)
    tick = ticks_p - ticks_p[0]
    hz = 1 / P.DT
    ch = {'truth.pose': C.channel('truth.pose', rec_t, st[:, :3], tick, 'exact', hz),
          'truth.twist': C.channel('truth.twist', rec_t, st[:, 3:6], tick, 'exact', hz),
          'link.state_world': C.channel('link.state_world', rec_t, world, tick, 'exact', hz,
                                        fields=('x', 'y', 'yaw', 'vx', 'vy', 'r'),
                                        units=('m', 'm', 'rad', 'm/s', 'm/s', 'rad/s'), frame='world.ros',
                                        angles=('yaw',)),
          'link.clock': C.channel('link.clock', rec_t, np.stack([t_h, ticks_p.astype(float)], -1), tick, 'exact', hz,
                                  fields=('t_harness', 'player_tick'), units=('s', '1'), frame='record.clock')}
    ok = np.array([o[0] for o in plant.r_obs], np.int64)
    obs_hz = 1 / (P.DT * mission.CONTROL_EVERY)
    if len(ok):
        t_recv, t_acq = rec_t[ok], rec_t[np.maximum(ok - plant.lat_ticks, 0)]
        ch['obs.pose'] = C.channel('obs.pose', t_recv, np.array([o[1] for o in plant.r_obs]), ok, 'estimated', obs_hz)
        ch['obs.twist'] = C.channel('obs.twist', t_recv, np.array([o[2] for o in plant.r_obs]), ok, 'estimated', obs_hz)
        ch['obs.stamp'] = C.channel('obs.stamp', t_recv, np.stack([t_acq, t_recv], -1), ok, 'exact', obs_hz)
    rk = np.array([r[0] for r in plant.r_req], np.int64)
    if len(rk):
        ch['cmd.requested'] = C.channel('cmd.requested', rec_t[rk], np.array([r[1] for r in plant.r_req]), rk,
                                        'commanded', obs_hz)
    n_sent = len(link.sent)
    sk = np.arange(n_sent, dtype=np.int64)                 # pulse i is sent at harness tick i (after state i)
    sk = sk[sk <= K]
    if len(sk):
        pul = np.array(link.sent[:len(sk)])
        ch['cmd.sent'] = C.channel('cmd.sent', rec_t[sk], np.column_stack([pul, rec_t[sk]]), sk, 'commanded', hz)
    cur = np.array([scenario['cur_speed'] * math.cos(math.radians(scenario['cur_toward_deg'])),
                    scenario['cur_speed'] * math.sin(math.radians(scenario['cur_toward_deg']))])
    ch['env.current.commanded'] = C.channel('env.current.commanded', rec_t, np.tile(cur, (K + 1, 1)), tick, 'commanded',
                                            hz, fields=('x', 'y'), units=('m/s', 'm/s'), frame='scenario.enu',
                                            note='uniform current set by the lockstep scenario line (course frame)')
    absent = dict(ABSENT)
    for n in ('obs.pose', 'obs.twist', 'obs.stamp', 'cmd.requested', 'cmd.sent'):
        if n not in ch:
            absent[n] = 'run ended before any sample'
    events, outcome = C.mission_events(t_h, st, t_max, rec_t=rec_t)
    if m is not None:
        C.assert_outcome_matches(outcome, m)
        end = outcome['terminal']
    else:
        end = err['kind']
        events.append({'t': float(rec_t[-1]), 'kind': err['kind'] if err['kind'] == 'link_lost' else 'abort',
                       'actor': 'harness', 'payload': {'why': err['why'], 'detail': err['detail'], 'tick': int(K)}})
    fr = link.hello
    e0 = np.array(world[0, :3]) - np.array([fr.x0, fr.y0, fr.yaw0])
    e0[2] = U.wrap(e0[2])
    verified = bool(all(abs(a) <= b for a, b in zip(e0, RESET_TOL)))
    events.insert(1, {'t': float(rec_t[0]), 'kind': 'reset', 'actor': 'player',
                      'payload': {'method': 'scene_load', 'pose_error': e0.tolist(), 'verified': verified}})
    lib_ctrl = C.controller_block(lib, lib_id, ctrl_idx)
    scen = C.mission_scenario(stratum, boat_index, hidden_seed, seed, t_max, lib_id, 'hidden')
    scen['condition']['lockstep_scenario_line'] = scenario
    cal = C.calibration_block(calibration)
    code = C.code_block()
    mech, not_rep = integrated_mechanisms(profile, faults, scenario, lost)
    if mechanisms is not None:                            # fake player: the numpy plant behind TCP
        mech, not_rep = mechanisms, []
    backend = {'layer': LAYER, 'name': backend_name, 'build': build, 'mechanisms': mech, 'flags': backend_flags(infra, err)}
    ident = {'backend': backend, 'plant': spec.row(0), 'controller': lib_ctrl['spec_sha'], 'cal': cal,
             'code': code['files_sha'], 'scenario': scen, 't_max': t_max, 'profile': profile, 'faults': faults}
    rid = C.run_id(LAYER, scen['id'], lib_ctrl['id'], seed, ident)
    dt_hello = float(link.hello_raw[4]) if link.hello_raw else None
    ev = {'kind': 'exploratory', 'prereg': None, 'seed_class': 'development' if hidden_seed == 4242 else None,
          **(evidence or {})}
    header = {
        'run_id': rid, 'tier': LAYER, 'source': 'decisioncal.records.lockstep_capture', 'domain': 'marine.asv',
        'robot': {'id': 'decisioncal-boat', 'model': f'hidden:{stratum}:{boat_index}@{hidden_seed}', 'morphology': None,
                  'plant_card': None},
        'task_id': 'decisioncal.dock_mission', 'scenario': scen,
        'clock': {'base': 'sim', 'anchor': 'episode_start', 'offset_s': 0.0, 'residual_s': None},
        'window': [float(rec_t[0]), float(rec_t[-1]), end], 'source_version': '2',
        'backend': backend, 'controller': lib_ctrl, 'calibration': cal,
        'clock_mode': {'mode': 'lockstep', 'dt_s': dt_hello, 'episode': 0, **clock_extra},
        'code': code, 'evidence': ev,
        'reset': {'method': 'scene_load', 'verified': verified, 'pose_error': e0.tolist()},
        'provenance': {
            'harness': _paths.provenance(), 'plant_spec': spec.row(0), 'unity_profile': profile, 'unity_faults': faults,
            'not_representable': lost, 'not_representable_mechanisms': not_rep, 'player_manifest': manifest,
            'hello': link.hello_raw, 'reset_tolerance': list(RESET_TOL), 'latency_ticks': plant.lat_ticks,
            'player_tick0': int(ticks_p[0]),
            'mission_metrics': None if m is None else {k: float(np.asarray(v).ravel()[0]) for k, v in m.items()},
            'outcome_from_events': outcome, 'absent_reasons': absent, 'infra': infra,
            'notes': ['record clock = player episode time t (float32 seconds since the lockstep reset)',
                      'link.clock.t_harness = UnityPlant.t (sum of 0.02 per tick), the clock mission.run scored on',
                      'events: mission.run rules on the harness clock, stamped with the record clock; docked payload '
                      't_dock is on the harness clock',
                      'obs.* come from the harness Sensors model on the reported state, not from Unity sensors']}}
    return C.new_rec(header, ch, absent, events)


# ---------------------------------------------------------------------------------------------
# runs: fake player (tests, no Unity) and live (through heavy admission)
# ---------------------------------------------------------------------------------------------

def capture_fake(stratum, boat_index, hidden_seed, ctrl_idx, seed, start=(0.0, 0.0, 0.0), library='v1',
                 t_max=mission.T_MAX, calibration=None, evidence=None, mission_wall=None):
    """The mission against ``unity_check.fake_player`` (numpy Plant behind the TCP protocol)."""
    lib, lib_id = C.library_of(library)
    spec = U.get_boat(hidden_seed, stratum, boat_index)
    profile, faults, scenario, lost = U.unity_config(spec, 0)
    faults = U.check_faults(faults)
    link = RecordingLink()
    log = {}
    th = threading.Thread(target=U.fake_player, args=(link.host, link.port, spec, seed, start, t_max, log), daemon=True)
    th.start()
    try:
        link.accept(30)
        link.scenario(scenario['cur_speed'], scenario['cur_toward_deg'], scenario['wind_speed'], scenario['wind_toward_deg'])
        deadline = None if mission_wall is None else time.time() + mission_wall
        m, plant, err = run_over_link(link, spec, ctrl_idx, seed, t_max=t_max, deadline=deadline, lib=lib)
    finally:
        th.join(30)
        link.close()
    infra = {'player_rc': 0, 'link_errors': [] if err is None else [err], 'water_invalid_searches': None,
             'fake_player_ticks': log.get('ticks')}
    rec = build_record(link, plant, m, err, spec, stratum, boat_index, hidden_seed, ctrl_idx, seed, lib_id, lib, t_max,
                       scenario, profile, faults, lost, 'unity_check.fake_player(numpy Plant over TCP)',
                       'fake-player', {}, {'time_scale': None, 'rtf': None}, infra, calibration, evidence,
                       mechanisms=C.fast_mechanisms(spec.row(0)))
    return rec, m


def heavy_probe():
    r = subprocess.run(['python3', str(HEAVY), 'probe', 'unity'], capture_output=True, text=True, timeout=120)
    return r.returncode, (r.stdout + r.stderr).strip()


def capture_live(stratum, boat_index, hidden_seed, ctrl_idx, seed, work_dir, library='v1', t_max=mission.T_MAX,
                 time_scale=4.0, player_wall=400.0, mission_wall=150.0, accept_timeout=400.0, calibration=None,
                 evidence=None, who='decisioncal-records', sim_profile='minimal'):
    """One live mission. ``work_dir`` receives the player's files (profile/faults/scenario json, player.log,
    result.json, launch.txt, heavy.out). Admission: ``heavy probe unity`` must return 0, then the player
    is started as ``heavy unity <player> ...`` with HEAVY_WHO=``who``."""
    lib, lib_id = C.library_of(library)
    if not U.PLAYER.is_file():
        raise FileNotFoundError(U.PLAYER)
    rc, msg = heavy_probe()
    if rc != 0:
        raise RuntimeError(f'heavy probe unity: exit {rc}: {msg}')
    spec = U.get_boat(hidden_seed, stratum, boat_index)
    work = Path(work_dir)
    work.mkdir(parents=True, exist_ok=True)
    profile, faults, scenario, lost = U.unity_config(spec, 0)
    faults = U.check_faults(faults)
    U.write_config(work, spec, 0, match=True, env=True)
    link = RecordingLink()
    a = SimpleNamespace(sim_profile=sim_profile, time_scale=time_scale, player_wall=player_wall)
    argv = U.player_argv(work, link.port, seed, a)
    (work / 'launch.txt').write_text(' '.join(shlex.quote(x) for x in argv) + '\n')
    env = {k: v for k, v in os.environ.items() if k not in ('DISPLAY', 'WAYLAND_DISPLAY')}
    env.update({'HEAVY_WHO': who, 'HEAVY_MAX_S': str(int(player_wall) + 120), 'SDL_VIDEODRIVER': 'dummy'})
    t_launch = time.time()
    proc = subprocess.Popen(argv, env=env, stdout=open(work / 'heavy.out', 'w'), stderr=subprocess.STDOUT,
                            start_new_session=True)
    infra = {'link_errors': [], 'heavy_probe': msg}
    m = plant = err = None
    try:
        try:
            link.accept(accept_timeout, alive=lambda: proc.poll() is None)
        except U.LinkError as e:
            infra['link_errors'].append({'kind': 'infra_error', 'why': 'accept', 'detail': str(e)})
            raise
        link.scenario(scenario['cur_speed'], scenario['cur_toward_deg'], scenario['wind_speed'], scenario['wind_toward_deg'])
        t_run = time.time()
        m, plant, err = run_over_link(link, spec, ctrl_idx, seed, t_max=t_max, deadline=time.time() + mission_wall, lib=lib)
        infra['mission_wall_s'] = time.time() - t_run
        if err:
            infra['link_errors'].append(err)
    finally:
        link.close()
        try:
            proc.wait(90)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, 15)
            infra['player_killed'] = True
        infra['player_rc'] = proc.poll()
        infra['launch_to_end_wall_s'] = time.time() - t_launch
    res = {}
    if (work / 'result.json').is_file():
        res = json.loads((work / 'result.json').read_text())
    infra['water_invalid_searches'] = int(res.get('invalidWaterSearches') or 0) if res else None
    infra['player_result'] = {k: res.get(k) for k in ('valid', 'realTimeFactor', 'timeScale', 'wallSeconds',
                                                      'simulatedSeconds', 'staleObservations', 'acceptedActions',
                                                      'loggedErrors', 'loggedExceptions', 'failedObservations')} if res else None
    log = (work / 'player.log').read_text(errors='replace') if (work / 'player.log').is_file() else ''
    infra['player_log_markers'] = [ln.strip() for ln in log.splitlines()
                                   if ln.startswith(('CRANE_LOCKSTEP_', 'CRANE_LOCKSTEP'))][:20]
    commit, manifest = player_build()
    clock_extra = {'time_scale': time_scale, 'rtf': res.get('realTimeFactor') if res else None}
    rec = build_record(link, plant, m, err, spec, stratum, boat_index, hidden_seed, ctrl_idx, seed, lib_id, lib, t_max,
                       scenario, profile, faults, lost, 'crane.unity.player(CRANE.x86_64, Roboboat Course)',
                       commit, manifest, clock_extra, infra, calibration, evidence)
    return rec, m
