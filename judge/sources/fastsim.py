"""fastsim run directories (``run.json``, ``track.jsonl``, ``sim_events.jsonl``) -> RunRecords.

Domain-agnostic: the pose is the core's planar (x, y, yaw); events are whatever the domain's
controller emitted through ``RunContext``. Twist is derived (estimated) from the pose because
fast runs do not log it yet (requested from the fastsim owner, design-judge 7.4)."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from judge.record import (Clock, Event, Header, Robot, RunRecord, Scenario, Validity, dedupe_time,
                          se2_channel, twist_from_pose)
from judge.source import FidelityProfile, Refused, Source


def _jsonl(path):
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


class FastsimSource(Source):
    name, tier = 'fastsim', 'fast'
    profile = FidelityProfile('kinematic', 'truth_cone', 'fixed_step', 'none', 0.05,
                              'kinematic fast tier: no hydrodynamics, truth-label perception, no latency')

    def __init__(self, domain='', robot='vehicle'):
        self.domain, self.robot = domain, robot

    def discover(self, root):
        yield from sorted(p.parent for p in Path(root).rglob('run.json'))

    def load(self, ref):
        ref = Path(ref)
        try:
            run = json.loads((ref / 'run.json').read_text())
        except (OSError, ValueError) as e:
            raise Refused(f'{ref}: no readable run.json ({e})')
        rows = _jsonl(ref / 'track.jsonl')
        if len(rows) < 2:
            raise Refused(f'{ref}: track has {len(rows)} rows')
        t, x, y, yaw = (np.array([r[k] for r in rows], float) for k in ('t', 'x', 'y', 'yaw'))
        t, x, y, yaw = dedupe_time(t, x, y, yaw)
        pose = se2_channel(t, x, y, yaw)
        events = tuple(Event(float(e.get('t', 0.0)), str(e.get('kind') or e.get('event') or 'event'),
                             str(e.get('actor', '')), {k: v for k, v in e.items() if k not in ('t',)})
                       for e in _jsonl(ref / 'sim_events.jsonl'))
        events += tuple(Event(float(c['t']), 'contact', 'world', c) for c in run.get('contacts', []))
        sc = run.get('scenario', {})
        crash = run.get('crash') or run.get('logic_exception')
        validity = Validity(not crash, {'crash': str(crash)[:80]} if crash else {},
                            frozenset({'G3', 'G4'}) if crash else frozenset())
        header = Header(run_id=ref.name, tier=self.tier, source=self.name, domain=self.domain,
                        robot=Robot(self.robot), task_id=str((run.get('condition') or {}).get('task', '')),
                        scenario=Scenario(sc.get('pair_key'), sc.get('truth_sha256'), run.get('seed'),
                                          sc.get('family'), run.get('condition') or {}),
                        provenance={'identity': run.get('identity'), 'git': (run.get('provenance') or {}).get('git')},
                        clock=Clock('sim'), window=(float(t[0]), float(t[-1]), 'track_end'))
        return RunRecord(header, {'pose.se2': pose, 'twist.body': twist_from_pose(pose)}, events,
                         {'contacts': 'geometric', 'truth': 'exact'}, validity)
