#!/usr/bin/env python3
"""Summarize opt-in RoboBoat thruster samples and detect magnitude collapse."""

import argparse
import json
import re
import statistics


SAMPLE = re.compile(
    r'fl_command=(?P<flc>[-+0-9.eE]+) fl_speed=(?P<fls>[-+0-9.eE]+) '
    r'fr_command=(?P<frc>[-+0-9.eE]+) fr_speed=(?P<frs>[-+0-9.eE]+) '
    r'rl_command=(?P<rlc>[-+0-9.eE]+) rl_speed=(?P<rls>[-+0-9.eE]+) '
    r'rr_command=(?P<rrc>[-+0-9.eE]+) rr_speed=(?P<rrs>[-+0-9.eE]+)')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('player_log')
    parser.add_argument('--output')
    parser.add_argument('--require-proportional', action='store_true')
    parser.add_argument('--minimum-command-ratio', type=float, default=4.0)
    parser.add_argument('--maximum-collapsed-speed-ratio', type=float, default=1.2)
    args = parser.parse_args()

    groups = {}
    with open(args.player_log, encoding='utf-8', errors='replace') as stream:
        for line in stream:
            match = SAMPLE.search(line)
            if not match:
                continue
            values = {name: float(value) for name, value in match.groupdict().items()}
            magnitude = round(max(abs(values[name]) for name in ('flc', 'frc', 'rlc', 'rrc')), 6)
            if magnitude == 0.0:
                continue
            groups.setdefault(magnitude, []).extend(
                abs(values[name]) for name in ('fls', 'frs', 'rls', 'rrs'))

    summaries = []
    for command, speeds in sorted(groups.items()):
        summaries.append({
            'commandMagnitude': command,
            'shaftSampleCount': len(speeds),
            'medianAbsShaftSpeedRadiansPerSecond': statistics.median(speeds),
            'meanAbsShaftSpeedRadiansPerSecond': statistics.fmean(speeds),
            'minimumAbsShaftSpeedRadiansPerSecond': min(speeds),
            'maximumAbsShaftSpeedRadiansPerSecond': max(speeds),
        })

    command_ratio = None
    speed_ratio = None
    collapsed = False
    if len(summaries) >= 2:
        command_ratio = summaries[-1]['commandMagnitude'] / summaries[0]['commandMagnitude']
        medians = [row['medianAbsShaftSpeedRadiansPerSecond'] for row in summaries]
        minimum_speed = min(medians)
        speed_ratio = max(medians) / minimum_speed if minimum_speed > 0.0 else None
        collapsed = (command_ratio >= args.minimum_command_ratio and
                     speed_ratio is not None and
                     speed_ratio <= args.maximum_collapsed_speed_ratio)

    report = {
        'schema': 'crane-roboboat-thruster-response-check-v1',
        'passed': bool(summaries) and not collapsed,
        'magnitudeCollapsed': collapsed,
        'commandMagnitudeRatio': command_ratio,
        'medianShaftSpeedRatio': speed_ratio,
        'responses': summaries,
    }
    rendered = json.dumps(report, indent=2, sort_keys=True) + '\n'
    if args.output:
        with open(args.output, 'w', encoding='utf-8') as stream:
            stream.write(rendered)
    print(rendered, end='')
    if args.require_proportional and not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
