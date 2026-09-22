#!/usr/bin/env python3
"""Detect pathological rotate-only behavior before a supplied RoboBoat path."""

import argparse
import json
import math


def wrapped(value):
    return math.remainder(value, 2.0 * math.pi)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('result')
    parser.add_argument('--linear-command-threshold', type=float, default=0.01)
    parser.add_argument('--maximum-yaw-travel', type=float, default=2.0 * math.pi)
    parser.add_argument('--maximum-drift', type=float, default=1.0)
    args = parser.parse_args()

    with open(args.result, encoding='utf-8') as stream:
        result = json.load(stream)
    trajectory = result.get('trajectory', [])
    failures = []
    if len(trajectory) < 2:
        failures.append('trajectory has fewer than two samples')

    first = trajectory[0] if trajectory else None
    before_translation = []
    translation_sample = None
    for sample in trajectory:
        if abs(sample.get('commandSurge', 0.0)) > args.linear_command_threshold:
            translation_sample = sample
            break
        before_translation.append(sample)

    yaw_travel = sum(
        abs(wrapped(end['yaw'] - start['yaw']))
        for start, end in zip(before_translation, before_translation[1:]))
    if first and before_translation:
        final = before_translation[-1]
        drift = math.hypot(final['x'] - first['x'], final['y'] - first['y'])
        maximum_yaw_rate = max(abs(row['bodyYawRate']) for row in before_translation)
    else:
        drift = math.nan
        maximum_yaw_rate = math.nan

    if translation_sample is None:
        failures.append('controller never issued a forward command')
    if yaw_travel > args.maximum_yaw_travel:
        failures.append(
            f'rotate-only yaw travel {yaw_travel:.3f} rad exceeds '
            f'{args.maximum_yaw_travel:.3f} rad')
    if math.isfinite(drift) and drift > args.maximum_drift:
        failures.append(
            f'rotate-only XY drift {drift:.3f} m exceeds {args.maximum_drift:.3f} m')

    report = {
        'schema': 'crane-roboboat-rotate-to-path-check-v1',
        'passed': not failures,
        'failures': failures,
        'translationCommandObserved': translation_sample is not None,
        'rotateOnlyYawTravelRadians': yaw_travel,
        'rotateOnlyDriftMeters': drift,
        'maximumAbsYawRateRadiansPerSecond': maximum_yaw_rate,
        'rotateOnlySampleCount': len(before_translation),
    }
    print(json.dumps(report, sort_keys=True))
    raise SystemExit(1 if failures else 0)


if __name__ == '__main__':
    main()
