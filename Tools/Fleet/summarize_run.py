#!/usr/bin/env python3
"""Summarize a fleet run directory: end reason, per-rover distance, odometry drift, the strafe/rotate
qualification displacements (commanded vs truth vs estimate), contacts and traction-limit events.

    python3 Tools/Fleet/summarize_run.py RUN_DIR
"""
import json
import math
import sys
from collections import Counter
from pathlib import Path


def main(d: Path):
    m = json.loads((d / 'manifest.json').read_text())
    rows = [json.loads(x) for x in (d / 'states.jsonl').read_text().splitlines()]
    ev = [json.loads(x) for x in (d / 'events.jsonl').read_text().splitlines()]
    print(f"run {m['run_id']} seed {m['seed']} end {m['end_reason']} t_end {m['t_end']:.2f}s steps {m['steps']} "
          f"wall {m['wall_seconds']:.1f}s rows {len(rows)} events {len(ev)}")
    ids = list(rows[0]['truth'].keys())
    for i in ids:
        dist = sum(math.hypot(b['truth'][i]['x'] - a['truth'][i]['x'], b['truth'][i]['y'] - a['truth'][i]['y'])
                   for a, b in zip(rows, rows[1:]))
        z = [r['truth'][i]['z'] for r in rows]
        tilt = max(max(abs(r['truth'][i]['pitch']), abs(r['truth'][i]['roll'])) for r in rows)
        f = m['results']['final'][i]
        print(f"  {i:9s} dist {dist:6.2f} m  z [{min(z):+.4f},{max(z):+.4f}]  max tilt {tilt:.4f} rad  "
              f"final truth ({f['truth']['x']:.3f},{f['truth']['y']:.3f},{f['truth']['yaw']:+.3f}) "
              f"odom err {f['odometry_error_xy']:.3f} m / {f['odometry_error_yaw']:+.4f} rad  step {f['script_step']}")
    for k, v in sorted(m['results'].items()):
        if k == 'final':
            continue
        fmt = lambda a: '(' + ', '.join(f'{x:+.3f}' for x in a) + ')'
        print(f"  {k:24s} cmd {fmt(v['commanded'])} truth {fmt(v['truth'])} est {fmt(v['estimate'])}")
    kinds = Counter(e['kind'] for e in ev)
    print('  events:', dict(kinds))
    timeouts = [e for e in ev if e['kind'] == 'step_end' and e.get('timed_out')]
    for e in timeouts:
        print(f"  TIMEOUT {e['rover']} {e['step']} at t={e['t']:.2f}")
    for e in [e for e in ev if e['kind'] == 'contact'][:20]:
        print(f"  contact t={e['t']:.2f} {e['a']} <-> {e['b']} impulse {e['impulse']:.3f}")


if __name__ == '__main__':
    main(Path(sys.argv[1]))
