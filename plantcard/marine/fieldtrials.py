"""RoboBoat field-trial evidence: the 10-18 open-loop pulse trials -> plant-card Segments.

Each ``agent_*.json`` in a field-trials folder (the team's characterization procedure) has
its own bag (``<trial>.files/bag/bag_0.mcap``) with the executed PWM pulse
(``/thruster_pwm_node/test_pulses``, order [fl, rr, fr, rl]), ``/gps/fix`` (5 Hz), ``/imu/raw``
(20 Hz gyro) and ``/odom/local``. The pulse is open loop, so the executed commands are exact.

**Translation truth is GPS, not odometry.** ``/odom/local`` translation on 10-18 is dead
reckoning: over the 12 s baselines it moves up to 11 m while GPS moves 0.1-1.7 m, and after a
1.5 s single-thruster pulse it reports 2 m/s for 8 s while GPS moves under 1 m. So:

* position: GPS (local equirectangular metres around the first fix);
* body velocity u, v: a local quadratic (Savitzky-Golay) fit to GPS positions over +-1 s, rotated
  into the body by the heading (the antenna lever arm is ignored: flag ``gps_lever_arm``);
* heading: ``/odom/local`` yaw (gyro-integrated, matches the IMU attitude); r: the IMU gyro.

Grid: 10 Hz. The session is the trial block (``matrix_04``, ``pairs_01``, ...), the replication
unit for validation. The world current is the GPS drift over the baseline window (flag
``current_from_drift``: wind drift is not separated from water motion).
"""

import hashlib
import json
from pathlib import Path
import re

import numpy as np

from plantcard.family import Evidence, Segment
from plantcard.marine import mcap_lite as M
from plantcard.marine.fossen3 import pwm_to_q

DEFAULT_DIR = Path.home() / 'astro_dock' / 'bags' / 'field_trials_20261018'
CMD_ORDER = ('front_left', 'rear_right', 'front_right', 'rear_left')
DT = 0.1
EARTH_R = 6378137.0
KEEP_STATUS = ('MEASURED', 'INCONCLUSIVE')
TOPICS = ['/gps/fix', '/imu/raw', '/odom/local', '/thruster_pwm_node/test_pulses']


def session_of(stem):
    s = stem.removeprefix('agent_')
    m = re.match(r'(battery|isolated|long|matrix|pairs)_(\d+)', s)
    return f'{m.group(1)}_{m.group(2)}' if m else re.sub(r'_\d+$', '', s)


def sg_velocity(t, p, tq, half=1.0, causal=False):
    """d/dt of a local quadratic fit of p(t) [n, 2] around each tq (window +-half, or the past
    ``2 half`` seconds when causal)."""
    out = np.full((len(tq), p.shape[1]), np.nan)
    for i, t0 in enumerate(tq):
        m = (t >= t0 - 2 * half) & (t <= t0) if causal else (t >= t0 - half) & (t <= t0 + half)
        if m.sum() < 4:
            continue
        A = np.stack([np.ones(m.sum()), t[m] - t0, (t[m] - t0) ** 2], -1)
        coef = np.linalg.lstsq(A, p[m], rcond=None)[0]
        out[i] = coef[1]
    return out


def read_trial(path, thruster_names, baseline_s=12.0, post_s=10.0):
    """A Segment (or (None, reason)) from one trial JSON and its bag."""
    path = Path(path)
    doc = json.loads(path.read_bytes())
    if doc.get('status') not in KEEP_STATUS:
        return None, f'status {doc.get("status")}'
    bag = path.with_suffix('.files') / 'bag' / 'bag_0.mcap'
    if not bag.exists():
        return None, 'no bag'
    raw = bag.read_bytes()
    d = M.read(bag, TOPICS)
    pulses = [(t, M.uint16_array(x)) for t, x in d['/thruster_pwm_node/test_pulses']]
    on = [i for i, (_, v) in enumerate(pulses) if any(a != 1500 for a in v)]
    if not on:
        return None, 'no pulse in the bag'
    t_on = pulses[on[0]][0]
    after = [t for t, v in pulses[on[-1]:] if all(a == 1500 for a in v)]
    t_off = after[0] if after else pulses[on[-1]][0] + 0.05
    pwm = pulses[on[0]][1]
    if any(pulses[i][1] != pwm for i in on):
        return None, 'pulse command not constant'
    gps = [M.navsatfix(x) for _, x in d['/gps/fix']]
    gps = [g for g in gps if g['status'] >= 0]
    if len(gps) < 50:
        return None, f'{len(gps)} GPS fixes'
    tg = np.array([t for (t, _), g in zip(d['/gps/fix'], gps)])
    lat0, lon0 = gps[0]['lat'], gps[0]['lon']
    P = np.array([[(g['lon'] - lon0) * np.pi / 180 * EARTH_R * np.cos(np.radians(lat0)),
                   (g['lat'] - lat0) * np.pi / 180 * EARTH_R] for g in gps])
    odo = [(t, M.odometry(x)) for t, x in d['/odom/local']]
    to = np.array([t for t, _ in odo])
    yaw = np.unwrap([o['yaw'] for _, o in odo])
    ti = np.array([t for t, _ in d['/imu/raw']])
    wz = np.array([M.imu(x)['w'][2] for _, x in d['/imu/raw']])
    lo = max(t_on - baseline_s, tg[0] + 1.0, to[0], ti[0])
    hi = min(t_off + post_s, tg[-1] - 1.0, to[-1], ti[-1])
    if t_on - lo < 4.0 or hi - t_off < 3.0:
        return None, 'too little data around the pulse'
    grid = np.arange(lo, hi, DT)
    pos = np.stack([np.interp(grid, tg, P[:, 0]), np.interp(grid, tg, P[:, 1])], -1)
    psi = np.interp(grid, to, yaw)
    ker = np.ones(5) / 5
    r = np.interp(grid, ti, np.convolve(wz, ker, 'same'))
    vw = sg_velocity(tg, P, grid)
    c, s = np.cos(psi), np.sin(psi)
    u, v = c * vw[:, 0] + s * vw[:, 1], -s * vw[:, 0] + c * vw[:, 1]
    X = np.stack([pos[:, 0], pos[:, 1], psi, u, v, r], -1)
    if np.isnan(X).any():
        return None, 'GPS gaps'
    q_cmd = dict(zip(CMD_ORDER, pwm_to_q(pwm)))
    q = np.array([q_cmd[n] for n in thruster_names])
    U = np.zeros((len(grid), len(thruster_names)))
    U[(grid >= t_on) & (grid < t_off)] = q
    bw = (grid >= t_on - baseline_s) & (grid < t_on - 0.2)
    k0, k1 = np.flatnonzero(bw)[[0, -1]]
    cur = tuple(float(x) for x in (pos[k1] - pos[k0]) / (grid[k1] - grid[k0]))
    vb = vw[bw]
    cur_sd = float(np.sqrt(vb.var(0).mean()))
    odo_xy = np.stack([np.interp(grid, to, [o['x'] for _, o in odo]), np.interp(grid, to, [o['y'] for _, o in odo])], -1)
    settings = doc.get('settings', {})
    t0 = grid[0]
    seg = Segment(
        session=session_of(path.stem), tier='real', t=grid - t0, x=X, u=U, u_kind='pwm_q',
        quality={'status': doc['status'], 'gps_fixes': len(gps), 'gps_sd_m': float(np.sqrt(gps[0]['cov'][0])),
                 'odom_baseline_disp_m': float(np.hypot(*(odo_xy[k1] - odo_xy[k0]))),
                 'gps_baseline_disp_m': float(np.hypot(*(pos[k1] - pos[k0]))),
                 'wave_translation_acc': doc.get('wave_envelope', {}).get('translation_acceleration')},
        labels=[settings.get('action', ''), settings.get('axis') or settings.get('position', ''),
                f'effort={settings.get("effort")}'] + list(settings.get('positions') or []),
        meta={'current': cur, 'current_sd': cur_sd, 'fit_window': (t_on - 1.0 - t0, min(hi, t_off + 8.0) - t0),
              'pwm': list(pwm), 'pulse_s': float(t_off - t_on), 'gps': (tg - t0, P),
              'flags': ['current_from_drift', 'gps_lever_arm', 'translation_from_gps']},
        name=path.stem, source_sha=hashlib.sha256(raw).hexdigest(), t_event=float(t_on - t0))
    return seg, 'ok'


def load(folder=DEFAULT_DIR, thruster_names=('front_left', 'front_right', 'rear_right', 'rear_left'), log=None):
    """Evidence from every usable trial in ``folder`` (and the trials skipped, with reasons)."""
    folder = Path(folder)
    segs, skipped = [], []
    for f in sorted(folder.glob('agent_*.json')):
        if '.' in f.stem:
            continue                      # .late / .analysis / .excluded side files
        seg, why = read_trial(f, list(thruster_names))
        if seg is None:
            skipped.append((f.stem, why))
        else:
            segs.append(seg)
    prov = {'kind': 'field_trials', 'folder': str(folder), 'date': '2026-10-18',
            'truth': 'GPS position + IMU gyro + odom heading (odom translation is dead reckoning)',
            'n_segments': len(segs), 'sessions': sorted({s.session for s in segs}),
            'bags_sha256': sorted(s.source_sha[:12] for s in segs),
            'skipped': [f'{a}: {b}' for a, b in skipped]}
    if log:
        log(f'{len(segs)} segments in {len(prov["sessions"])} sessions; skipped {len(skipped)}')
    return Evidence(segs, prov)
