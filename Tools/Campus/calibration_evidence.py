"""Read exact fixed-step command edges when available; retain old evidence scope."""
import json


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
