#!/usr/bin/env python3
"""Compare matched fixed-step command programs without spatial registration."""
import argparse
import bisect
import json
import math
from pathlib import Path
import statistics

from calibration_evidence import command_origin


def load_run(path):
    rows = [json.loads(line) for line in (path / 'telemetry.jsonl').read_text().splitlines()]
    events = [json.loads(line) for line in (path / 'events.jsonl').read_text().splitlines()]
    program = next((json.loads(e['detail']) for e in events
                    if e['type'] == 'calibration-program'), None)
    commands = [(e['simulationTime'], json.loads(e['detail'])) for e in events
                if e['type'] == 'calibration-command']
    origin, precision = command_origin(path, rows)
    scenario = json.loads((path / 'scenario.json').read_text())
    manifest = json.loads((path / 'manifest.json').read_text())
    robot_model = next((e['detail'] for e in events if e['type'] == 'robot-model'), None)
    return dict(rows=rows, program=program, commands=commands, origin=origin,
                precision=precision, scenario=scenario,robotModel=robot_model,
                physicsManifest={key:manifest[key] for key in ['boxes','surfaceProfiles']})


def resample(run, times):
    rows = run['rows']
    stamps = [row['simulationTime'] - run['origin'] for row in rows]
    yaw = [rows[0]['yaw']]
    for before, after in zip(rows, rows[1:]):
        yaw.append(yaw[-1] + (after['yaw'] - before['yaw'] + 180) % 360 - 180)
    result = []
    for stamp in times:
        i = max(0, min(len(rows) - 2, bisect.bisect_right(stamps, stamp) - 1))
        fraction = (stamp - stamps[i]) / (stamps[i + 1] - stamps[i])
        def blend(a, b):
            return a + fraction * (b - a)
        result.append(dict(
            position=[blend(rows[i]['position'][axis], rows[i+1]['position'][axis])
                      for axis in 'xyz'],
            yaw=blend(yaw[i], yaw[i+1]),
            speed=blend(math.hypot(rows[i]['velocity']['x'], rows[i]['velocity']['z']),
                        math.hypot(rows[i+1]['velocity']['x'], rows[i+1]['velocity']['z'])),
            angular=blend(rows[i]['angularVelocity']['y'], rows[i+1]['angularVelocity']['y']),
            slip=blend(rows[i]['slipProxy'], rows[i+1]['slipProxy'])))
    return result


def compare(left, right, mode):
    step = max(left['rows'][0]['fixedDeltaTime'], right['rows'][0]['fixedDeltaTime'])
    limits = (dict(positionRmse=.005, maximumPositionError=.015, finalYawError=1., yawRmse=1., maximumYawError=2.,
                   speedRmse=.01, angularRmse=.02, meanSlipDifference=.01)
              if mode == 'parity' else
              dict(positionRmse=.02, maximumPositionError=.04, finalYawError=3., yawRmse=3., maximumYawError=5.,
                   speedRmse=.03, angularRmse=.06, meanSlipDifference=.02))
    start = max(left['rows'][0]['simulationTime'] - left['origin'],
                right['rows'][0]['simulationTime'] - right['origin'])
    end = min(left['rows'][-1]['simulationTime'] - left['origin'],
              right['rows'][-1]['simulationTime'] - right['origin'])
    times = [start + i * .02 for i in range(max(0, int((end-start)/.02) + 1))]
    if not times:
        raise ValueError('Matched trials have no overlapping acquisition interval')
    l, r = resample(left, times), resample(right, times)
    distances = [math.dist(a['position'], b['position']) for a, b in zip(l, r)]
    yaw_errors = [(a['yaw']-b['yaw']+180) % 360-180 for a,b in zip(l,r)]
    rmse = lambda values: math.sqrt(statistics.mean(v*v for v in values))
    metrics = dict(positionRmse=rmse(distances), maximumPositionError=max(distances),
                   finalPositionError=distances[-1],
                   finalYawError=abs((l[-1]['yaw']-r[-1]['yaw']+180) % 360-180),
                   yawRmse=rmse(yaw_errors), maximumYawError=max(abs(v) for v in yaw_errors),
                   speedRmse=rmse([a['speed']-b['speed'] for a,b in zip(l,r)]),
                   angularRmse=rmse([a['angular']-b['angular'] for a,b in zip(l,r)]),
                   meanSlipDifference=abs(statistics.mean(a['slip'] for a in l)
                                          -statistics.mean(a['slip'] for a in r)),
                   comparisonSamples=len(times), commonDurationSeconds=end-start,
                   collisionCounts=[left['rows'][-1]['collisionCount'],
                                    right['rows'][-1]['collisionCount']])
    lc = [(t-left['origin'], c) for t,c in left['commands']]
    rc = [(t-right['origin'], c) for t,c in right['commands']]
    commands_match = (len(lc) == len(rc) and bool(lc)
                      and all(a == b and abs(ta-tb) <= step + 1e-5
                              for (ta,a),(tb,b) in zip(lc,rc)))
    checks = dict(exactCommandTimesAvailable=left['precision'] == right['precision'] == 'fixed-step-command-event',
                  sameCommandProgram=left['program'] is not None and left['program'] == right['program'],
                  sameScenario=left['scenario'] == right['scenario'],
                  sameRobotModel=left['robotModel'] is not None and left['robotModel'] == right['robotModel'],
                  sameSurfaceAndGeometry=left['physicsManifest'] == right['physicsManifest'],
                  commandEdgesMatchWithinFixedStep=commands_match,
                  fullCommandProgramObserved=bool(lc) and bool(rc)
                    and lc[-1][1] == rc[-1][1] == dict(linear=0,angular=0)
                    and end > max(lc[-1][0],rc[-1][0]) + .5,
                  contactPresenceMatches=bool(metrics['collisionCounts'][0]) == bool(metrics['collisionCounts'][1]))
    checks.update({name: metrics[name] <= bound for name,bound in limits.items()})
    return dict(schema='crane-campus-physics-comparison-v1',
                state='PHYSICS_PASS' if all(checks.values()) else 'PARTIAL',
                mode=mode, checks=checks, metrics=metrics, limits=limits,
                boundaries=['Acquisition times align to exact first command edges; poses are not registered or rescaled.',
                            'This compares simulated command response, not a physical TurtleBot fit.',
                            'Contact callback counts are reported; equal contact presence does not prove identical impulses.'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('left', type=Path)
    parser.add_argument('right', type=Path)
    parser.add_argument('--mode', choices=['parity','timestep'], default='parity')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = compare(load_run(args.left), load_run(args.right), args.mode)
    report['runs'] = [str(args.left), str(args.right)]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report['state'] == 'PHYSICS_PASS' else 1)


if __name__ == '__main__':
    main()
