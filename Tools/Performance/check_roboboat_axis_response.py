#!/usr/bin/env python3
"""Assert the RoboBoat command axes and yaw feedback agree with the ROS FLU contract."""

import argparse
import json
import math


def phase_by_name(result, name):
    return next(phase for phase in result['phases'] if phase['name'] == name)


def tail(phase, field):
    value = phase[field]['meanTail']
    return float(value) if value is not None else math.nan


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('result')
    parser.add_argument('--dominance-ratio', type=float, default=3.0)
    args = parser.parse_args()

    with open(args.result, encoding='utf-8') as stream:
        result = json.load(stream)

    failures = []
    if result.get('status') != 'completed':
        failures.append(f"fixture did not complete: status={result.get('status')!r}")

    surge = phase_by_name(result, 'surge_positive')
    surge_response = tail(surge, 'body_surge')
    surge_cross = tail(surge, 'body_sway')
    if surge['sampleCount'] == 0 or not math.isfinite(surge_response) or not math.isfinite(surge_cross):
        failures.append('surge request phase has no usable odometry samples')
    elif surge_response <= 0.0 or abs(surge_response) < args.dominance_ratio * abs(surge_cross):
        failures.append(
            f'surge request mapped incorrectly: surge={surge_response:.6f}, '
            f'sway={surge_cross:.6f}')

    sway = phase_by_name(result, 'sway_positive')
    sway_response = tail(sway, 'body_sway')
    sway_cross = tail(sway, 'body_surge')
    if sway['sampleCount'] == 0 or not math.isfinite(sway_response) or not math.isfinite(sway_cross):
        failures.append('sway request phase has no usable odometry samples')
    elif sway_response <= 0.0 or abs(sway_response) < args.dominance_ratio * abs(sway_cross):
        failures.append(
            f'sway request mapped incorrectly: sway={sway_response:.6f}, '
            f'surge={sway_cross:.6f}')

    yaw = phase_by_name(result, 'yaw_positive')
    yaw_rate = tail(yaw, 'body_yaw_rate')
    pose_delta = float(yaw.get('deltaPose', {}).get('yaw', math.nan))
    if (yaw['sampleCount'] == 0 or not math.isfinite(yaw_rate)
            or not math.isfinite(pose_delta)):
        failures.append('yaw request phase has no usable odometry samples')
    elif yaw_rate <= 0.0 or pose_delta <= 0.0:
        failures.append(
            f'positive yaw request mapped incorrectly: pose_delta={pose_delta:.6f}, '
            f'yaw_rate={yaw_rate:.6f}')
    elif math.copysign(1.0, yaw_rate) != math.copysign(1.0, pose_delta):
        failures.append(
            f'pose/twist yaw signs disagree: pose_delta={pose_delta:.6f}, '
            f'yaw_rate={yaw_rate:.6f}')

    report = {
        'schema': 'crane-roboboat-axis-response-check-v1',
        'passed': not failures,
        'failures': failures,
    }
    print(json.dumps(report, sort_keys=True))
    raise SystemExit(1 if failures else 0)


if __name__ == '__main__':
    main()
