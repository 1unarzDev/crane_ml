"""Read exact fixed-step command edges when available; retain old evidence scope."""
import json
import math


def command_origin(run, rows):
    path = run / 'events.jsonl'
    if path.exists():
        for line in path.read_text().splitlines():
            event = json.loads(line)
            if event['type'] == 'calibration-command':
                command = json.loads(event['detail'])
                if command['linear'] or command['angular']:
                    return event['simulationTime'], 'fixed-step-command-event'
    return next(row['simulationTime'] for row in rows
                if row['commandLinear'] or row['commandAngular']), 'sampled-command-observation'


def initial_response(rows, origin):
    """First observed 90% command crossing during the initial command segment."""
    first=next(row for row in rows if row['commandLinear'] or row['commandAngular'])
    linear=first['commandLinear'];angular=first['commandAngular']
    channel='linear' if linear else 'angular'
    target=abs(linear if linear else angular)*.9
    rise=None
    for row in rows:
        if row['simulationTime']<first['simulationTime']:continue
        if (row['commandLinear'],row['commandAngular'])!=(linear,angular):break
        measured=(math.hypot(row['velocity']['x'],row['velocity']['z']) if linear
                  else row['angularVelocity']['y']*(1 if angular>0 else -1))
        if measured>=target:
            rise=row['simulationTime']-origin
            break
    return dict(riseTime90PercentSeconds=rise,responseChannel=channel,
                riseTimeReference='First observed crossing of 90% of initial command, bounded to that command segment',
                riseTimeThreshold=target)
