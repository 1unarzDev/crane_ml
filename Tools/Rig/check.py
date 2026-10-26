"""Smoke gates for a sim session: odometry, thrust and course truth all flowed.

Usage: python3 check.py SESSION_DIR RIG_YAML [DURATION_S]
Reads SESSION_DIR/bag (rosbag2) and SESSION_DIR/course_truth.json, writes
SESSION_DIR/smoke.json and exits 1 when any gate fails.
With DURATION_S the bag must also span smoke.min_span_frac of it on smoke.span_topics,
so a recorder cut short by an outside signal cannot pass on its own short span. The bag is read with
rosbag2_py, so run it inside the ROS container (or with ROS sourced).
"""

import json
import math
import sys
from pathlib import Path

import yaml


def rates(stamps):
    """Messages per second over each topic's own span (receive time, ns)."""
    out = {}
    for topic, times in stamps.items():
        span = (max(times) - min(times)) / 1e9 if len(times) > 1 else 0.0
        out[topic] = round(len(times) / span, 2) if span > 0 else 0.0
    return out


def spans(stamps):
    """Seconds between each topic's first and last message (receive time, ns)."""
    return {topic: round((max(times) - min(times)) / 1e9, 2) for topic, times in stamps.items() if times}


def travel(points):
    """Largest distance from the first point, metres."""
    if not points:
        return 0.0
    x0, y0 = points[0]
    return max(math.hypot(x - x0, y - y0) for x, y in points)


def active_pwm(frames, neutral, band):
    """Number of PWM frames with any output outside neutral +- band."""
    return sum(any(abs(int(value) - neutral) > band for value in frame) for frame in frames)


def truth_ok(truth):
    return (truth.get('schema') == 'crane-roboboat-course-truth-v1'
            and len(truth.get('elements', [])) > 0)


def gates(stats, truth, smoke, duration=None):
    """Each gate as (name, passed, measured, required)."""
    out = []
    if duration:
        need = round(smoke['min_span_frac'] * duration, 2)
        out += [(f'span {topic}', stats['span'].get(topic, 0.0) >= need, stats['span'].get(topic, 0.0), need)
                for topic in smoke['span_topics']]
    out += [(f'hz {topic}', stats['hz'].get(topic, 0.0) >= hz, stats['hz'].get(topic, 0.0), hz)
           for topic, hz in smoke['min_hz'].items()]
    out.append(('travel /crane/odom', stats['travel'] >= smoke['min_travel_m'],
                round(stats['travel'], 3), smoke['min_travel_m']))
    out.append(('active /thrusters/pwm', stats['active_pwm'] >= smoke['min_active_pwm'],
                stats['active_pwm'], smoke['min_active_pwm']))
    out.append(('course_truth', truth_ok(truth), len(truth.get('elements', [])), '>0'))
    return out


def read_bag(path, topics):
    """Receive stamps per topic, /crane/odom xy and /thrusters/pwm frames."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path)), rosbag2_py.ConverterOptions('', ''))
    types = {info.name: info.type for info in reader.get_all_topics_and_types()}
    stamps = {topic: [] for topic in topics}
    odom, pwm = [], []
    while reader.has_next():
        topic, data, received = reader.read_next()
        stamps.setdefault(topic, []).append(received)
        if topic == '/crane/odom':
            position = deserialize_message(data, get_message(types[topic])).pose.pose.position
            odom.append((position.x, position.y))
        elif topic == '/thrusters/pwm':
            pwm.append(list(deserialize_message(data, get_message(types[topic])).data))
    return stamps, odom, pwm


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    session, rig = Path(argv[0]), yaml.safe_load(Path(argv[1]).read_text())
    smoke = rig['smoke']
    stamps, odom, pwm = read_bag(session / 'bag', rig['record'])
    truth_path = session / 'course_truth.json'
    truth = json.loads(truth_path.read_text()) if truth_path.exists() else {}
    stats = {'span': spans(stamps),
             'hz': rates({t: s for t, s in stamps.items() if s}),
             'counts': {t: len(s) for t, s in stamps.items()},
             'travel': travel(odom),
             'active_pwm': active_pwm(pwm, smoke['neutral_us'], smoke['pwm_band_us'])}
    duration = float(argv[2]) if len(argv) > 2 else None
    results = gates(stats, truth, smoke, duration)
    passed = all(ok for _, ok, _, _ in results)
    report = {'passed': passed, 'stats': stats,
              'gates': [{'name': n, 'passed': ok, 'measured': m, 'required': r}
                        for n, ok, m, r in results]}
    (session / 'smoke.json').write_text(json.dumps(report, indent=2))
    for name, ok, measured, required in results:
        print(f"{'ok  ' if ok else 'FAIL'} {name}: {measured} (need {required})")
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
