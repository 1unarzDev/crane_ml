"""Distinguish traversable terrain contacts from obstacle contact callbacks."""
from collections import Counter


def classified_contacts(manifest,scenario,events):
    roles={b['id']:b['kind'] for b in manifest['boxes']+scenario['obstacles']}
    terrain=Counter();obstacles=Counter();unknown=Counter()
    for event in events:
        if event['type']!='contact-enter':continue
        identifier=event['id'];role=roles.get(identifier)
        target=terrain if role in ['floor','ramp','threshold'] else unknown if role is None else obstacles
        target[identifier]+=1
    return dict(terrainContactCallbacks=sum(terrain.values()),obstacleContactCallbacks=sum(obstacles.values()),
                unknownContactCallbacks=sum(unknown.values()),terrainContactsById=dict(terrain),
                obstacleContactsById=dict(obstacles),unknownContactsById=dict(unknown),
                noProhibitedObstacleContacts=not obstacles and not unknown,
                boundary='Counts are observed enter callbacks, not unique impacts or penetrations. Ordinary floor callbacks are suppressed by existing telemetry; ramp/threshold callbacks are retained.')
