"""Score the motion quality of a recorded Nav2 run (crane_ml sim) from a rosbag2 directory.

    python3 motion_quality.py <bag_dir> [thresholds.yaml]

prints one JSON document: {"metrics", "pass", "segments", "inputs", "bag"}. Metric
definitions, parameters and pass thresholds live in motion_quality.yaml beside this file.

Two layers:
  * score(series, thresholds) is the ROS-free core. It takes numpy time series (seconds from
    the bag start) and needs only numpy, so it runs on the host under `python3 -I`.
  * read_bag(path) is the thin reader. It decodes with rosbag2_py when that imports (inside
    the ROS container); otherwise it copies the bag into the `roboboat_dev` container, extracts
    plain rows there, and assembles the series on the host.

Series layout (all arrays 1-D float unless noted):
  odom         t, x, y, yaw, vx, vy, r [, stamp]  truth pose in the goal (map) frame, body
                                                   twist; `stamp` is the physics clock used
                                                   for derivatives (falls back to t)
  cmd_vel_nav  t, vx, vy, wz
  pwm          t, us (N x 4 pulse widths, µs)
  goals        t, x, y   one row per /plan update: the plan's last pose (goal proxy)
  inputs       {topic: message count}
"""
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml

DEFAULTS = Path(__file__).with_name('motion_quality.yaml')
CONTAINER = 'roboboat_dev'
TRUTH, LOCAL = '/crane/odom', '/odom/local'
TWISTS = ('/cmd_vel_nav', '/cmd_vel')
PWM, PLAN = '/thrusters/pwm', '/plan'


# --------------------------------------------------------------------------- core helpers

def _hold_dt(t, cap):
    """Per-sample duration under a zero-order hold, capped so dropouts are not integrated."""
    return np.clip(np.append(np.diff(t), 0.0), 0.0, cap) if len(t) else np.zeros(0)


def _flips(x, band, valid=None):
    """Sign changes of x with hysteresis: the sign is set only when |x| > band.

    Returns (flip, lobe): index of each change and index where the lobe it ends began.
    Where `valid` is False the sign state is cleared, so a change across an invalid
    stretch (the boat stopped) does not count.
    """
    flip, lobe = [], []
    state, start = 0, 0
    for i, value in enumerate(x):
        if valid is not None and not valid[i]:
            state = 0
            continue
        sign = 1 if value > band else -1 if value < -band else 0
        if sign == 0 or sign == state:
            continue
        if state != 0:
            flip.append(i)
            lobe.append(start)
        state, start = sign, i
    return np.array(flip, dtype=int), np.array(lobe, dtype=int)


def _span(t):
    return float(t[-1] - t[0]) if len(t) > 1 else 0.0


def _movavg(x, n):
    """Centred moving average over n samples, renormalised at the edges."""
    kernel = np.ones(n)
    return np.convolve(x, kernel, 'same') / np.convolve(np.ones_like(x), kernel, 'same')


def _runs(mask):
    """(start, stop) index pairs of the True runs in a boolean array, stop exclusive."""
    edges = np.diff(np.concatenate(([0], mask.astype(np.int8), [0])))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def _clock(odom):
    """Physics clock for derivatives: header stamps when strictly increasing, else t."""
    stamp = odom.get('stamp')
    if stamp is not None and len(stamp) > 1 and np.all(np.diff(stamp) > 0):
        return np.asarray(stamp, float)
    return np.asarray(odom['t'], float)


# --------------------------------------------------------------------------- metrics

def yaw_reversals(odom, p):
    """Metrics 1 and 2: hysteresis reversals of truth yaw rate, and the shaking subset."""
    t, r = odom['t'], odom['r']
    h = p['yaw_band_rad_s']
    speed = np.hypot(odom['vx'], odom['vy'])
    underway = (speed > p['underway_speed_m_s']) | (np.abs(r) > h)
    flip, lobe = _flips(r, h, underway)
    minutes = np.sum(_hold_dt(t, p['hold_cap_s'])[underway]) / 60.0
    shakes = []
    for i, j in zip(flip, lobe):
        half = t[i] - t[j]
        peak = float(np.max(np.abs(r[j:i])))
        if peak > p['shake_peak_rad_s'] and half < p['shake_half_period_s']:
            shakes.append({'t': float(t[i]), 'half_period_s': float(half), 'peak_rad_s': peak})
    per_min = (lambda n: n / minutes if minutes > 0 else 0.0)
    return {'yaw_reversals_per_min': per_min(len(flip)),
            'shake_index': per_min(len(shakes)),
            'underway_min': float(minutes)}, shakes


def cmd_chatter(cmd, p):
    """Metric 3: hysteresis sign changes per second on each /cmd_vel_nav axis."""
    span = _span(cmd['t'])
    axes = {axis: (len(_flips(cmd[axis], p['cmd_band'])[0]) / span if span else 0.0)
            for axis in ('vx', 'vy', 'wz')}
    return {'cmd_chatter_hz': max(axes.values()), 'cmd_chatter_hz_axes': axes}


def thruster_chatter(pwm, p):
    """Metric 4: sign changes of (pwm - neutral) outside the ESC deadband, per second."""
    span = _span(pwm['t'])
    error = np.asarray(pwm['us'], float) - p['pwm_neutral_us']
    each = [len(_flips(error[:, k], p['esc_deadband_us'])[0]) / span if span else 0.0
            for k in range(error.shape[1])]
    return {'thruster_chatter_hz': max(each), 'thruster_chatter_hz_mean': float(np.mean(each)),
            'thruster_chatter_hz_each': each}


def jerk(odom, p):
    """Metric 5: RMS planar jerk and yaw angular jerk of the truth.

    The body twist is rotated into the world, resampled on a uniform grid of the physics
    clock, smoothed with a centred moving average over smooth_s, then differentiated twice.
    """
    clock = _clock(odom)
    if len(clock) < 5:
        return {'jerk_rms': None, 'yaw_jerk_rms': None}
    c, s = np.cos(odom['yaw']), np.sin(odom['yaw'])
    world = (odom['vx'] * c - odom['vy'] * s, odom['vx'] * s + odom['vy'] * c, odom['r'])
    dt = float(np.median(np.diff(clock)))
    grid = np.arange(clock[0], clock[-1], dt)
    n = max(1, int(round(p['smooth_s'] / dt))) | 1  # odd, so the average stays centred
    vx, vy, r = (_movavg(np.interp(grid, clock, v), n) for v in world)
    if len(grid) <= 2 * n:
        return {'jerk_rms': None, 'yaw_jerk_rms': None}
    # Drop one window at each end, where the truncated average and one-sided differences ring.
    second = (lambda v: np.gradient(np.gradient(v, dt), dt)[n:-n])
    rms = (lambda v: float(np.sqrt(np.mean(v ** 2))))
    return {'jerk_rms': rms(np.hypot(second(vx), second(vy))), 'yaw_jerk_rms': rms(second(r))}


def spins(odom, p):
    """Metric 6: time turning on the spot and the segments longer than spin_min_s.

    Spinning means |r| > spin_rate and either |v| < spin_speed or a turn radius |v|/|r| under
    spin_radius. The radius term matters: base_link is not the pivot, so a boat spinning at
    0.3 rad/s still shows |v| ~ 0.07 m/s at base_link (a ~0.2 m circle).
    """
    t, r = odom['t'], np.abs(odom['r'])
    speed = np.hypot(odom['vx'], odom['vy'])
    mask = (r > p['spin_rate_rad_s']) & ((speed < p['spin_speed_m_s']) | (speed < p['spin_radius_m'] * r))
    held = _hold_dt(t, p['hold_cap_s'])
    yaw = np.unwrap(odom['yaw'])
    segments = []
    for a, b in _runs(mask):
        duration = float(t[b - 1] - t[a] + held[b - 1])
        if duration > p['spin_min_s']:
            segments.append({'start': float(t[a]), 'duration_s': duration,
                             'dyaw_rad': float(yaw[b - 1] - yaw[a])})
    total = float(np.sum(held[mask]))
    span = _span(t)
    return {'spin_in_place_s': total, 'spin_fraction': total / span if span else 0.0}, segments


def goal_legs(goals, p):
    """Group /plan updates into legs: a new leg starts when the endpoint jumps goal_jump_m.

    A leg ends at the next leg's start, or one plan period after its last update.
    """
    t, x, y = (np.asarray(goals[k], float) for k in ('t', 'x', 'y'))
    if not len(t):
        return []
    period = float(np.median(np.diff(t))) if len(t) > 1 else 1.0
    jump = np.hypot(np.diff(x), np.diff(y)) > p['goal_jump_m']
    starts = np.concatenate(([0], np.flatnonzero(jump) + 1))
    stops = np.append(starts[1:], len(t))
    legs = []
    for a, b in zip(starts, stops):
        end = t[b] if b < len(t) else t[b - 1] + period
        # Plan endpoints jitter by a few cm; the median is the goal.
        legs.append({'start': float(t[a]), 'end': float(end),
                     'x': float(np.median(x[a:b])), 'y': float(np.median(y[a:b]))})
    return legs


def goal_metrics(odom, legs, p):
    """Metrics 7, 8 and 10 per leg: overshoot past the goal, settle time, path ratio, time."""
    t, x, y = odom['t'], odom['x'], odom['y']
    radius, hold = p['goal_radius_m'], p['settle_hold_s']
    out = []
    for k, leg in enumerate(legs):
        # Overshoot and settling are watched until the next goal arrives (or the data ends).
        window_end = legs[k + 1]['start'] if k + 1 < len(legs) else t[-1]
        inside = (t >= leg['start']) & (t <= leg['end'])
        start = int(np.argmax(t >= leg['start']))
        gx, gy = leg['x'], leg['y']
        straight = math.hypot(gx - x[start], gy - y[start])
        path = float(np.sum(np.hypot(np.diff(x[inside]), np.diff(y[inside]))))
        item = {**leg, 'time_s': leg['end'] - leg['start'], 'path_m': path,
                'path_ratio': path / straight if straight > radius else None,
                'reached_s': None, 'overshoot_m': None, 'settle_s': None}
        dist = np.hypot(x - gx, y - gy)
        watch = (t >= leg['start']) & (t <= window_end)
        near = np.flatnonzero(watch & (dist < radius))
        if len(near):
            first = near[0]
            item['reached_s'] = float(t[first] - leg['start'])
            if straight > radius:
                # Approach direction: from the leg start towards the goal.
                ux, uy = (gx - x[start]) / straight, (gy - y[start]) / straight
                later = watch & (t >= t[first])
                item['overshoot_m'] = float(np.max((x[later] - gx) * ux + (y[later] - gy) * uy))
            item['settle_s'] = _settle(t, dist, first, window_end, radius, hold, leg['start'])
        out.append(item)
    return out


def _settle(t, dist, first, window_end, radius, hold, t_start):
    """First time (from the leg start) after which the boat stays inside radius for hold
    seconds, or until the window ends if that comes sooner. None if it never does."""
    for i in range(first, len(t)):
        if t[i] > window_end:
            break
        if dist[i] >= radius:
            continue
        stay_until = min(t[i] + hold, window_end)
        span = (t >= t[i]) & (t <= stay_until)
        if np.all(dist[span] < radius) and t[span][-1] >= stay_until - 0.1:
            return float(t[i] - t_start)
    return None


def stalls(cmd, legs, p):
    """Metric 11: gaps in /cmd_vel_nav longer than stall_gap_s that begin while a goal is active."""
    t = cmd['t']
    found = []
    for i in np.flatnonzero(np.diff(t) > p['stall_gap_s']):
        if any(leg['start'] <= t[i] < leg['end'] for leg in legs):
            found.append({'start': float(t[i]), 'duration_s': float(t[i + 1] - t[i])})
    return found


def energy(pwm, odom, p):
    """Metric 9: ∫ Σ|F_i| dt with F = k·e|e| (k forward or reverse), e = (pwm - neutral)/span."""
    t = pwm['t']
    e = (np.asarray(pwm['us'], float) - p['pwm_neutral_us']) / p['pwm_span_us']
    force = np.where(e > 0, p['thrust_fwd_n'], p['thrust_rev_n']) * e * np.abs(e)
    total = float(np.sum(np.abs(force).sum(axis=1) * _hold_dt(t, p['hold_cap_s'])))
    travelled = float(np.sum(np.hypot(np.diff(odom['x']), np.diff(odom['y']))))
    return {'energy_proxy_Ns': total, 'distance_m': travelled,
            'energy_per_m_Ns': total / travelled if travelled > 0 else None}


# --------------------------------------------------------------------------- core API

def score(series, thresholds):
    """Score one run. `series` follows the module docstring; `thresholds` is the parsed YAML.

    Returns {"metrics", "pass", "segments", "inputs"}; a metric that lacks its inputs is
    None, and so is its pass flag. Pass means value <= the YAML max; a null max is not judged.
    """
    p = thresholds['params']
    odom = {k: np.asarray(v, float) for k, v in series['odom'].items()}
    cmd = {k: np.asarray(v, float) for k, v in series.get('cmd_vel_nav', {'t': []}).items()}
    pwm = series.get('pwm')
    legs = goal_legs(series['goals'], p) if series.get('goals') else []

    metrics, segments = {}, {}
    rev, segments['shakes'] = yaw_reversals(odom, p)
    metrics.update(rev)
    metrics.update(jerk(odom, p))
    spin, segments['spins'] = spins(odom, p)
    metrics.update(spin)
    if len(cmd['t']) > 1:
        metrics.update(cmd_chatter(cmd, p))
    if pwm is not None and len(pwm['t']) > 1:
        metrics.update(thruster_chatter(pwm, p))
        metrics.update(energy(pwm, odom, p))
    segments['goals'] = goal_metrics(odom, legs, p)
    overshoots = [g['overshoot_m'] for g in segments['goals'] if g['overshoot_m'] is not None]
    ratios = [g['path_ratio'] for g in segments['goals'] if g['path_ratio'] is not None]
    metrics['goal_overshoot_m'] = max(overshoots) if overshoots else None
    metrics['path_ratio'] = max(ratios) if ratios else None
    metrics['time_s'] = _span(odom['t'])
    metrics['goal_time_s'] = sum(g['time_s'] for g in segments['goals'])
    segments['stalls'] = stalls(cmd, legs, p) if len(cmd['t']) > 1 else []
    metrics['stall_count'] = len(segments['stalls'])
    metrics['stall_s'] = sum(s['duration_s'] for s in segments['stalls'])

    verdict = {}
    for name, limit in thresholds['metrics'].items():
        if limit is not None:  # a null max is report-only
            value = metrics.get(name)
            verdict[name] = None if value is None else bool(value <= limit)
    return {'metrics': metrics, 'pass': verdict, 'segments': segments,
            'inputs': dict(series.get('inputs', {}))}


# --------------------------------------------------------------------------- bag reader

def _segments(path):
    """Bag directories in recording order: `path` itself, or its bag_NN / NN children
    (the layout mhseals_testing's recorder writes; same rule as its harness/bagread.py)."""
    path = Path(path)
    is_bag = (lambda d: (d / 'metadata.yaml').exists() or any(d.glob('*.mcap')) or any(d.glob('*.db3')))
    if is_bag(path):
        return [path]
    if is_bag(path / 'bag'):
        return [path / 'bag']
    return sorted(d for d in path.iterdir() if d.is_dir() and is_bag(d)) if path.is_dir() else []


def _extract(path):
    """Decode the topics the score needs into plain JSON-able rows (needs rosbag2_py).

    Times are receive stamps in seconds since the epoch; odometry also keeps its header stamp.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    def yaw(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    rows = {'counts': {}, 'odom': {TRUTH: [], LOCAL: []}, 'twist': {k: [] for k in TWISTS},
            'pwm': [], 'plan': [], 't0': None}
    wanted = {TRUTH, LOCAL, PWM, PLAN, *TWISTS}
    for segment in _segments(path):
        mcap = any(segment.glob('*.mcap')) or 'storage_identifier: mcap' in (
            (segment / 'metadata.yaml').read_text() if (segment / 'metadata.yaml').exists() else '')
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(segment), storage_id='mcap' if mcap else 'sqlite3'),
                    rosbag2_py.ConverterOptions('cdr', 'cdr'))
        types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
        while reader.has_next():
            topic, data, t_ns = reader.read_next()
            t = t_ns * 1e-9
            rows['t0'] = t if rows['t0'] is None else min(rows['t0'], t)
            rows['counts'][topic] = rows['counts'].get(topic, 0) + 1
            if topic not in wanted:
                continue
            msg = deserialize_message(data, get_message(types[topic]))
            if topic in rows['odom']:
                pose, twist = msg.pose.pose, msg.twist.twist
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                rows['odom'][topic].append([t, stamp, pose.position.x, pose.position.y, yaw(pose.orientation),
                                            twist.linear.x, twist.linear.y, twist.angular.z])
            elif topic in rows['twist']:
                rows['twist'][topic].append([t, msg.linear.x, msg.linear.y, msg.angular.z])
            elif topic == PWM:
                rows['pwm'].append([t, *list(msg.data)[:4]])
            elif topic == PLAN and msg.poses:
                end = msg.poses[-1].pose.position
                rows['plan'].append([t, end.x, end.y])
    return rows


def _extract_in_container(path):
    """Run _extract inside the ROS container on a temporary copy of the bag.

    The container does not mount every host path, so the bag and this file are copied in
    with `docker cp` and the copy is removed afterwards.
    """
    tmp = f'/tmp/motion_quality.{os.getpid()}'
    run = (lambda *cmd: subprocess.run(cmd, check=True, capture_output=True, text=True).stdout)
    run('docker', 'exec', CONTAINER, 'mkdir', '-p', tmp)
    try:
        run('docker', 'cp', str(Path(path).resolve()), f'{CONTAINER}:{tmp}/bag')
        run('docker', 'cp', str(Path(__file__).resolve()), f'{CONTAINER}:{tmp}/motion_quality.py')
        code = (f"import sys, json; sys.path.insert(0, '{tmp}'); import motion_quality as m; "
                f"print(json.dumps(m._extract('{tmp}/bag')))")
        out = run('docker', 'exec', CONTAINER, 'bash', '-c',
                  f'source /opt/ros/jazzy/setup.bash && python3 -c "{code}"')
    finally:
        subprocess.run(['docker', 'exec', CONTAINER, 'rm', '-rf', tmp], capture_output=True)
    return json.loads(out.strip().splitlines()[-1])


def _align(truth, local):
    """Rigid 2-D transform taking truth poses into the local odometry (= map, goal) frame.

    The sim's truth odometry has its own origin; /odom/local shares the frame of /plan
    (map->odom is near identity in the sim). Fit yaw by circular mean, then translation by
    median, over local samples matched to truth by header stamp.
    """
    stamp = truth[:, 1]
    keep = (local[:, 1] >= stamp[0]) & (local[:, 1] <= stamp[-1])
    if keep.sum() < 10:
        return truth, None
    local = local[keep]
    tx, ty = np.interp(local[:, 1], stamp, truth[:, 2]), np.interp(local[:, 1], stamp, truth[:, 3])
    tyaw = np.interp(local[:, 1], stamp, np.unwrap(truth[:, 4]))
    dyaw = math.atan2(np.mean(np.sin(local[:, 4] - tyaw)), np.mean(np.cos(local[:, 4] - tyaw)))
    c, s = math.cos(dyaw), math.sin(dyaw)
    dx = float(np.median(local[:, 2] - (c * tx - s * ty)))
    dy = float(np.median(local[:, 3] - (s * tx + c * ty)))
    out = truth.copy()
    out[:, 2] = c * truth[:, 2] - s * truth[:, 3] + dx
    out[:, 3] = s * truth[:, 2] + c * truth[:, 3] + dy
    out[:, 4] = np.angle(np.exp(1j * (truth[:, 4] + dyaw)))
    return out, {'dx': dx, 'dy': dy, 'dyaw': dyaw}


def _bag_time(received, stamp, window=25):
    """Receive times de-jittered: header stamp plus a rolling median of (receive - stamp).

    Odometry arrives in bursts (±30 ms) while its stamps are even; the rolling offset also
    follows the sim when it runs slower than real time.
    """
    offset = received - stamp
    if len(offset) >= window:
        padded = np.pad(offset, window // 2, mode='edge')
        offset = np.median(np.lib.stride_tricks.sliding_window_view(padded, window), axis=1)
    return np.maximum.accumulate(stamp + offset)


def read_bag(path):
    """Read a rosbag2 directory (sqlite3 or mcap, one bag or a segmented session) into the
    series score() takes. Truth is /crane/odom aligned to /odom/local's frame, else /odom/local."""
    try:
        import rosbag2_py  # noqa: F401  (present inside the ROS container)
        rows = _extract(path)
    except ImportError:
        rows = _extract_in_container(path)
    t0 = rows['t0']
    truth = np.array(rows['odom'][TRUTH], float).reshape(-1, 8)
    local = np.array(rows['odom'][LOCAL], float).reshape(-1, 8)
    source, alignment = (TRUTH, None) if len(truth) else (LOCAL, None)
    odom = truth if len(truth) else local
    odom = odom[np.argsort(odom[:, 1], kind='stable')]
    if source == TRUTH and len(local):
        odom, alignment = _align(odom, local[np.argsort(local[:, 1], kind='stable')])
    series = {'odom': dict(zip(('t', 'stamp', 'x', 'y', 'yaw', 'vx', 'vy', 'r'), odom.T)),
              'odom_source': source, 'alignment': alignment, 'inputs': rows['counts']}
    series['odom']['t'] = _bag_time(odom[:, 0], odom[:, 1]) - t0
    cmd = np.array(rows['twist']['/cmd_vel_nav'], float).reshape(-1, 4)
    series['cmd_vel_nav'] = dict(zip(('t', 'vx', 'vy', 'wz'), cmd.T))
    series['cmd_vel_nav']['t'] = series['cmd_vel_nav']['t'] - t0
    pwm = np.array(rows['pwm'], float).reshape(-1, 5)
    series['pwm'] = {'t': pwm[:, 0] - t0, 'us': pwm[:, 1:]}
    plan = np.array(rows['plan'], float).reshape(-1, 3)
    series['goals'] = {'t': plan[:, 0] - t0, 'x': plan[:, 1], 'y': plan[:, 2]}
    return series


def _jsonable(value):
    if isinstance(value, dict):
        return {k: _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [_jsonable(v) for v in value]
    if isinstance(value, (float, np.floating)):
        return round(float(value), 4)
    if isinstance(value, np.integer):
        return int(value)
    return value


def main(argv):
    if len(argv) not in (2, 3):
        sys.exit('usage: motion_quality.py <bag_dir> [thresholds.yaml]')
    thresholds = yaml.safe_load(Path(argv[2] if len(argv) == 3 else DEFAULTS).read_text())
    series = read_bag(argv[1])
    result = score(series, thresholds)
    result['segments']['frame'] = {'odom': series['odom_source'], 'truth_to_map': series['alignment']}
    result['bag'] = str(Path(argv[1]).resolve())
    print(json.dumps(_jsonable(result), indent=1))


if __name__ == '__main__':
    main(sys.argv)
