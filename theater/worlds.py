"""World plug-ins on the Python side: what a world needs before Unity starts.

A world turns its course reference into the player arguments that build it, a centre line (progress
is the arc length along it) and bounds. The Unity theater has the matching ``ITheaterWorld``
(``Assets/Scripts/Theater/RoboBoat``). Only plug-ins here name a domain.
"""

import json
import math
from pathlib import Path


class RoboBoatWorld:
    """RoboBoat courses: a crane course truth (``crane-roboboat-course-truth-v1``, world frame = /crane/odom,
    ENU) rebuilt by the player from ``--crane-course-replay``."""
    name = 'roboboat'
    scene = 'Roboboat Course'

    def __init__(self, truth):
        self.truth = truth if isinstance(truth, dict) else json.loads(Path(truth).read_text())

    @staticmethod
    def only_tasks(truth, tasks):
        """A copy of ``truth`` keeping only ``tasks`` (ids), e.g. entry_gates + debris_clearance."""
        keep = set(tasks)
        t = json.loads(json.dumps(truth))
        for key in ('elements', 'gates', 'bays'):
            t[key] = [e for e in t.get(key) or [] if e.get('task') in keep]
        t['tasks'] = [x for x in t.get('tasks') or [] if x.get('id') in keep]
        used = {e.get('state') for e in t['elements']} | {b.get('sign', {}).get('id') for b in t['bays']}
        t['states'] = [s for s in t.get('states') or [] if s.get('id') in used]
        t['constraints'] = [c for c in t.get('constraints') or [] if c.get('task') in keep]
        t['name'] = f"{truth.get('name', 'course')}-{'+'.join(tasks)}"
        return t

    def start(self):
        t = self.truth
        return float(t.get('startX', 0.0)), float(t.get('startY', 0.0)), float(t.get('startYaw', 0.0))

    def gates(self):
        order = [x['id'] for x in self.truth.get('tasks') or []]
        gs = sorted(self.truth.get('gates') or [],
                    key=lambda g: (order.index(g['task']) if g['task'] in order else 99, g.get('order', 0)))
        return gs

    def centreline(self):
        """Start, the gates in task order (a field task's gates are entered and left again: entry gates out
        to the last channel gate, a loop around the task's field, then back out through the channel)."""
        sx, sy, syaw = self.start()
        line = [(sx, sy)]
        gates = self.gates()
        by_task = {}
        for g in gates:
            by_task.setdefault(g['task'], []).append(g)
        for task in [x['id'] for x in self.truth.get('tasks') or []]:
            gs = by_task.get(task, [])
            line += [(g['x'], g['y']) for g in gs]
            if task == 'debris_clearance' and gs:
                field = [(e['x'], e['y']) for e in self.truth['elements']
                         if e.get('task') == task and not any(e['id'] in (g['port'], g['starboard']) for g in gs)]
                if field:
                    cx = sum(p[0] for p in field) / len(field)
                    cy = sum(p[1] for p in field) / len(field)
                    lx, ly = gs[-1]['x'], gs[-1]['y']
                    a0 = math.atan2(ly - cy, lx - cx)
                    r = max(4.0, 0.6 * max(math.hypot(p[0] - cx, p[1] - cy) for p in field))
                    line += [(cx + r * math.cos(a0 + math.pi * 2 * k / 12),
                              cy + r * math.sin(a0 + math.pi * 2 * k / 12)) for k in range(13)]
                    line += [(g['x'], g['y']) for g in reversed(gs)]
        return line

    def bounds(self, margin=10.0):
        pts = [(e['x'], e['y']) for e in self.truth.get('elements') or []] + [self.start()[:2]]
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        return min(xs) - margin, min(ys) - margin, max(xs) + margin, max(ys) + margin

    def manifest_world(self, truth_file='course_truth.json'):
        return {'plugin': self.name, 'scene': self.scene, 'truth': truth_file,
                'name': self.truth.get('name'), 'truth_sha256': (self.truth.get('generator') or {}).get('truth_sha256'),
                'centreline': [[round(x, 3), round(y, 3)] for x, y in self.centreline()],
                'bounds': [round(v, 3) for v in self.bounds()],
                'args': ['--crane-scene', self.scene, '--crane-course-replay', '{dir}/' + truth_file]}


WORLDS = {'roboboat': RoboBoatWorld}
