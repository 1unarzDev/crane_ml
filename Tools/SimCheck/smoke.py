"""Smoke gates over a recorded sim session: odometry, sensors and thrust flowed, and the course truth exists.

Input is a session directory (``bag/`` plus ``course_truth.json``) or a rosbag2 directory. The gates
are pure; ``read_bag`` decodes with rosbag2_py, so run it where ROS is sourced (the sim_up_ros
container: ``docker exec sim_up_ros python3 -m Tools.SimCheck smoke ...``).
"""

import json
import math
from pathlib import Path

from . import report

ODOM, PWM = '/crane/odom', '/thrusters/pwm'
TRUTH_SCHEMA = 'crane-roboboat-course-truth-v1'


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
    return max(math.dist(points[0], p) for p in points)


def active_pwm(frames, neutral, band):
    """Number of PWM frames with any output outside neutral +- band."""
    return sum(any(abs(int(value) - neutral) > band for value in frame) for frame in frames)


def stats_of(stamps, odom, pwm, limits):
    return {'span': spans(stamps), 'hz': rates({t: s for t, s in stamps.items() if s}),
            'counts': {t: len(s) for t, s in stamps.items()}, 'travel': travel(odom),
            'active_pwm': active_pwm(pwm, limits['neutral_us'], limits['pwm_band_us'])}


def gates(stats, truth, limits, duration=None):
    """Each gate as (name, passed, measured, required)."""
    out = []
    if duration:
        need = round(limits['min_span_frac'] * duration, 2)
        out += [(f'span {topic}', stats['span'].get(topic, 0.0) >= need, stats['span'].get(topic, 0.0), need)
                for topic in limits['span_topics']]
    out += [(f'hz {topic}', stats['hz'].get(topic, 0.0) >= hz, stats['hz'].get(topic, 0.0), hz)
            for topic, hz in limits['min_hz'].items()]
    out.append((f'travel {ODOM}', stats['travel'] >= limits['min_travel_m'], round(stats['travel'], 3),
                limits['min_travel_m']))
    out.append((f'active {PWM}', stats['active_pwm'] >= limits['min_active_pwm'], stats['active_pwm'],
                limits['min_active_pwm']))
    elements = len(truth.get('elements', []))
    out.append(('course_truth', truth.get('schema') == TRUTH_SCHEMA and elements > 0, elements, '>0'))
    return out


def read_bag(path):
    """Receive stamps per topic, /crane/odom xy and /thrusters/pwm frames (needs rosbag2_py)."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as e:
        raise SystemExit(f'smoke reads the bag with rosbag2_py ({e}); run it where ROS is sourced, '
                         'e.g. docker exec sim_up_ros python3 -m Tools.SimCheck smoke ...')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path)), rosbag2_py.ConverterOptions('', ''))
    types = {info.name: info.type for info in reader.get_all_topics_and_types()}
    stamps, odom, pwm = {}, [], []
    while reader.has_next():
        topic, data, received = reader.read_next()
        stamps.setdefault(topic, []).append(received)
        if topic == ODOM:
            p = deserialize_message(data, get_message(types[topic])).pose.pose.position
            odom.append((p.x, p.y))
        elif topic == PWM:
            pwm.append(list(deserialize_message(data, get_message(types[topic])).data))
    return stamps, odom, pwm


def check(path, limits, duration=None, truth_path=None):
    path = Path(path)
    bag = path / 'bag' if (path / 'bag').is_dir() else path
    truth_path = Path(truth_path) if truth_path else path / 'course_truth.json'
    truth = json.loads(truth_path.read_text(encoding='utf-8')) if truth_path.is_file() else {}
    stats = stats_of(*read_bag(bag), limits)
    rows = gates(stats, truth, limits, duration)
    return report('smoke', [f'{n}: {m} (need {r})' for n, ok, m, r in rows if not ok], stats=stats,
                  gates=[{'name': n, 'passed': ok, 'measured': m, 'required': r} for n, ok, m, r in rows])
