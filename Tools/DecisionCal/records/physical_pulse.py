"""Physical replay writer: one record per 2026-10-18 open-loop pulse trial (read-only toward the bags).

Trial list and onset rule are ``preplay/raw.py``'s (``load_all`` keys, ``read_raw`` onset/offset: first
non-neutral PWM message, first neutral message after the last non-neutral one). The bag is read with
``plantcard.marine.mcap_lite`` (the reader raw.py uses) so every message keeps BOTH its recorder log
time and, where the message has a header, its header stamp.

Record clock: recorder log time (epoch s) minus the first log time of the four topics read
(t = 0 = first message). Channels:
  cmd.sent        every /thruster_pwm_node/test_pulses message, reordered from the topic's
                  (fl, rr, fr, rl) to canonical (FL, FR, RR, RL); t_send = recorder log time (the
                  message has no header; this is when the recorder got it, not when it left the node)
  cmd.requested   the same messages as q = (pwm - 1500) / 400 (open loop: the script's request IS the
                  published PWM; commanded)
  obs.pose        at each valid GPS fix: x, y local equirectangular metres around the first fix (raw.py's
                  projection); yaw = the last /odom/local yaw received at or before the fix (sample and hold)
  obs.stamp       for obs.pose rows: t_acq = GPS header stamp, t_recv = recorder log time (header stamps
                  look like arrival stamps, ~2 ms before log time: PREPLAY_1018 section 1)
  obs.heading (+ obs.heading.stamp)  /odom/local yaw at its own rate, header stamp + log time
  obs.gyro (+ obs.gyro.stamp)        /imu/raw angular_velocity.z (rad/s), header stamp + log time
Declared absent: truth.* (no independent truth on the water: GPS/IMU/odom are the boat's own
sensors), obs.twist (no body-velocity sensor; GPS-differenced velocity is an estimator), cmd.applied and
act.thrust (no ESC/thruster feedback), env.current / env.wind (not measured; the baseline GPS drift
is an estimate, kept in provenance). Events: episode_start, pulse_start, pulse_end; terminal track_end.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np

from . import _common as C

from plantcard.marine import fieldtrials as F
from plantcard.marine import mcap_lite as M
from plantcard.marine.fossen3 import pwm_to_q

LAYER = 'physical'
RAW_PY = C.DC / 'preplay' / 'raw.py'
TO_CANON = [F.CMD_ORDER.index(n) for n in ('front_left', 'front_right', 'rear_right', 'rear_left')]
JSON_CLOCK_OFFSET_S = 1792356317.351      # trial-JSON clock - bag clock (PREPLAY_1018, sd 1-3 ms)


def _raw():
    spec = importlib.util.spec_from_file_location('preplay_raw', RAW_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def trials():
    """{stem: {session, labels, doc_settings, t_on, t_off}} of the usable trials (raw.load_all)."""
    return _raw().load_all()


def read_trial(stem, folder=F.DEFAULT_DIR):
    raw = _raw()
    jf = Path(folder) / f'{stem}.json'
    bag = jf.with_suffix('.files') / 'bag' / 'bag_0.mcap'
    d = M.read(bag, raw.TOP)
    t0 = min(v[0][0] for v in d.values() if v)
    pw_t = np.array([t for t, _ in d['/thruster_pwm_node/test_pulses']])
    pw = np.array([M.uint16_array(x) for _, x in d['/thruster_pwm_node/test_pulses']], float)
    on = np.flatnonzero((pw != 1500).any(1))
    t_on = pw_t[on[0]]
    after = [t for t, v in zip(pw_t[on[-1]:], pw[on[-1]:]) if (v == 1500).all()]
    t_off = after[0] if after else pw_t[on[-1]] + 0.05
    gps = [(t, M.navsatfix(x)) for t, x in d['/gps/fix']]
    gps = [(t, g) for t, g in gps if g['status'] >= 0]
    lat0, lon0 = gps[0][1]['lat'], gps[0][1]['lon']
    gx = np.array([(g['lon'] - lon0) * np.pi / 180 * F.EARTH_R * np.cos(np.radians(lat0)) for _, g in gps])
    gy = np.array([(g['lat'] - lat0) * np.pi / 180 * F.EARTH_R for _, g in gps])
    g_log = np.array([t for t, _ in gps])
    g_stamp = np.array([g['stamp'] for _, g in gps])
    odo = [(t, M.odometry(x)) for t, x in d['/odom/local']]
    o_log = np.array([t for t, _ in odo])
    o_stamp = np.array([o['stamp'] for _, o in odo])
    o_yaw = np.array([o['yaw'] for _, o in odo])
    imu = [(t, M.imu(x)) for t, x in d['/imu/raw']]
    i_log = np.array([t for t, _ in imu])
    i_stamp = np.array([i['stamp'] for _, i in imu])
    i_wz = np.array([i['w'][2] for _, i in imu])
    return dict(stem=stem, json=jf, bag=bag, doc=json.loads(jf.read_bytes()), t0=t0, t_on=t_on, t_off=t_off,
                pw_t=pw_t, pw=pw[:, TO_CANON], g_log=g_log, g_stamp=g_stamp, gx=gx, gy=gy,
                g_sd=float(np.sqrt(gps[0][1]['cov'][0])), o_log=o_log, o_stamp=o_stamp, o_yaw=o_yaw,
                i_log=i_log, i_stamp=i_stamp, i_wz=i_wz, latlon0=(lat0, lon0))


ABSENT = {
    'truth.pose': 'physical replay: no independent truth (no survey/mocap); GPS/odom are the boat\'s own sensors (obs.*)',
    'truth.twist': 'physical replay: no independent truth',
    'obs.twist': 'no body-velocity sensor; GPS-differenced velocity is an estimator with smoothing bias '
                 '(PREPLAY_1018 section 2); yaw rate is in obs.gyro',
    'cmd.applied': 'no ESC / thruster feedback in the bags (PWM node -> hardware PWM -> ESC latency unknown)',
    'act.thrust': 'no thrust measurement',
    'env.current': 'not measured (baseline GPS drift is an estimate that mixes wind drift; see provenance)',
    'env.wind': 'not measured',
}


def _stamp_stats(log, stamp):
    d = log - stamp
    return {'median_s': float(np.median(d)), 'sd_s': float(np.std(d)), 'max_s': float(np.max(d)),
            'min_s': float(np.min(d)), 'n': int(len(d))}


def build(stem, info=None, folder=F.DEFAULT_DIR, evidence=None):
    r = read_trial(stem, folder)
    info = info or {}
    t0 = r['t0']
    rel = lambda a: np.asarray(a) - t0
    ch = {}
    pw_t = rel(r['pw_t'])
    ch['cmd.sent'] = C.channel('cmd.sent', pw_t, np.column_stack([r['pw'], pw_t]), None, 'commanded', None)
    ch['cmd.requested'] = C.channel('cmd.requested', pw_t, pwm_to_q(r['pw']), None, 'commanded', None)
    k = np.searchsorted(r['o_log'], r['g_log'], side='right') - 1      # last odom at or before each fix
    keep = k >= 0
    g_t = rel(r['g_log'][keep])
    yaw = r['o_yaw'][k[keep]]
    ch['obs.pose'] = C.channel('obs.pose', g_t, np.column_stack([r['gx'][keep], r['gy'][keep], yaw]), None,
                               'estimated', None)
    ch['obs.stamp'] = C.channel('obs.stamp', g_t, np.column_stack([rel(r['g_stamp'][keep]), g_t]), None,
                                'exact', None, note='t_acq = GPS header stamp as recorded (it looks like an arrival '
                                'stamp, ~1-2 ms before log time; true acquisition time unknown); t_recv = recorder log time')
    o_t = rel(r['o_log'])
    ch['obs.heading'] = C.channel('obs.heading', o_t, r['o_yaw'][:, None], None, 'estimated', None, fields=('yaw',),
                                  units=('rad',), frame='odom.local', angles=('yaw',))
    ch['obs.heading.stamp'] = C.channel('obs.heading.stamp', o_t, np.column_stack([rel(r['o_stamp']), o_t]), None,
                                        'estimated', None, fields=('t_acq', 't_recv'), units=('s', 's'),
                                        frame='record.clock')
    i_t = rel(r['i_log'])
    ch['obs.gyro'] = C.channel('obs.gyro', i_t, r['i_wz'][:, None], None, 'estimated', None, fields=('r',),
                               units=('rad/s',), frame='imu.body')
    ch['obs.gyro.stamp'] = C.channel('obs.gyro.stamp', i_t, np.column_stack([rel(r['i_stamp']), i_t]), None,
                                     'estimated', None, fields=('t_acq', 't_recv'), units=('s', 's'),
                                     frame='record.clock')
    t_end = float(max(pw_t[-1], g_t[-1], o_t[-1], i_t[-1]))
    q_on = pwm_to_q(r['pw'][np.flatnonzero((r['pw'] != 1500).any(1))[0]])
    events = [{'t': 0.0, 'kind': 'episode_start', 'actor': 'recorder', 'payload': {}},
              {'t': float(r['t_on'] - t0), 'kind': 'pulse_start', 'actor': 'pulse_script',
               'payload': {'q': q_on.tolist(), 'rule': 'first non-neutral PWM message (preplay/raw.py)'}},
              {'t': float(r['t_off'] - t0), 'kind': 'pulse_end', 'actor': 'pulse_script',
               'payload': {'rule': 'first neutral PWM message after the last non-neutral one'}}]
    doc = r['doc']
    settings = doc.get('settings', {})
    sess = info.get('session') or F.session_of(stem)
    scen = {'id': stem.removeprefix('agent_'), 'pair_key': stem, 'truth_sha256': None, 'seed': None,
            'family': 'roboboat.open_loop_pulse.20261018',
            'condition': {'settings': settings, 'session': sess, 'labels': info.get('labels'),
                          'status': doc.get('status'), 'pwm_canonical_on': r['pw'][np.flatnonzero(
                              (r['pw'] != 1500).any(1))[0]].tolist()}}
    ctrl = {'id': 'open_loop_pulse', 'library': None,
            'spec_sha': hashlib.sha256(json.dumps(settings, sort_keys=True).encode()).hexdigest(),
            'params': {'settings': settings, 'pulse_s': float(r['t_off'] - r['t_on'])}}
    code = C.code_block(extra=[F.__file__, M.__file__])
    bag_sha = C.sha256_file(r['bag'])
    backend = {'layer': LAYER, 'name': 'mhseals.roboboat.field_trials_20261018', 'build': None, 'mechanisms': ['unknown'],
               'flags': {}}
    rid = C.run_id(LAYER, scen['id'], ctrl['id'], 'na', {'bag': bag_sha, 'code': code['files_sha']})
    header = {
        'run_id': rid, 'tier': LAYER, 'source': 'decisioncal.records.physical_pulse', 'domain': 'marine.asv',
        'robot': {'id': 'roboboat-2026', 'model': 'mhseals roboboat (one boat, 2026-10-18)', 'morphology': None,
                  'plant_card': None},
        'task_id': 'roboboat.open_loop_pulse', 'scenario': scen,
        'clock': {'base': 'wall', 'anchor': 'first_message', 'offset_s': float(t0), 'residual_s': None},
        'window': [0.0, t_end, 'track_end'], 'source_version': '2',
        'backend': backend, 'controller': ctrl, 'calibration': C.calibration_block(None),
        'clock_mode': {'mode': 'recorded', 'dt_s': None, 'time_scale': None, 'rtf': None, 'episode': None},
        'code': code,
        'evidence': {'kind': 'replay', 'prereg': None, 'seed_class': None, **(evidence or {})},
        'reset': {'method': None, 'verified': None, 'pose_error': None},
        'provenance': {
            'bag': str(r['bag']), 'bag_sha256': bag_sha, 'trial_json': str(r['json']),
            'trial_json_sha256': C.sha256_file(r['json']), 'record_t0_epoch_s': float(t0),
            'gps_origin_latlon': list(r['latlon0']), 'gps_sd_m_first_fix': r['g_sd'],
            'log_minus_stamp': {'gps': _stamp_stats(r['g_log'], r['g_stamp']),
                                'odom': _stamp_stats(r['o_log'], r['o_stamp']),
                                'imu': _stamp_stats(r['i_log'], r['i_stamp'])},
            'json_clock_offset_s': JSON_CLOCK_OFFSET_S,
            'absent_reasons': ABSENT, 'infra': {'errors': []},
            'unknown': ['GPS receiver / mavros latency', 'FCU IMU filter latency', 'whether header stamps are '
                        'measurement or arrival time', 'PWM node -> ESC -> motor latency', 'FCU-vs-companion clock offset'],
            'notes': ['obs.pose yaw is held from the last /odom/local message received at or before each GPS fix',
                      'odom translation is dead reckoning and is not recorded (plantcard fieldtrials)',
                      'cmd.requested == the published PWM as q (open loop)']}}
    return C.new_rec(header, ch, ABSENT, events), r
