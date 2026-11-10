#!/usr/bin/env python3
"""Summarize a demo-mission run directory: milestones, which controller drove each rover, the closed-loop docking estimator
against truth (rover-side estimate of the target in its odometry frame vs the evaluator's truth, and the follower's odometry vs truth),
payload support and delivery, injected failures and contacts with impulse.

    python3 Tools/Fleet/summarize_demo.py RUN_DIR
"""
import json
import math
import sys
from pathlib import Path

MILESTONES = ('mission_start', 'payload_found', 'payload_not_found', 'payload_gripped', 'payload_lifted', 'connector_paired', 'hooks_closing',
              'coupled', 'payload_released', 'payload_placed', 'delivered', 'delivery_missed', 'failure_injected', 'step_timeout',
              'separated', 'controller')


def main(d: Path):
    m = json.loads((d / 'manifest.json').read_text())
    ev = [json.loads(x) for x in (d / 'events.jsonl').read_text().splitlines()]
    rows = [json.loads(x) for x in (d / 'states.jsonl').read_text().splitlines()]
    print(f"run {m['run_id']} end {m['end_reason']} t_end {m['t_end']:.2f}s wall {m['wall_seconds']:.1f}s physics={m['parameters'].get('physics_scene')}")
    print(f"controller_input: {m['parameters']['controller_input']}")
    if m['parameters'].get('failures'):
        print(f"failures: {m['parameters']['failures']}")
    for e in ev:
        if e['kind'] in MILESTONES or (e['kind'] == 'connector' and e.get('event') == 'Transition'):
            f = {k: v for k, v in e.items() if k not in ('t', 'step', 'kind', 'rover')}
            s = json.dumps(f)
            print(f"  {e['t']:7.2f} {e['kind']:16s} {e['rover']:9s} {s[:200]}")
    contacts = [e for e in ev if e['kind'] == 'contact']
    print(f"  contacts with impulse: {len(contacts)}")
    for e in contacts[:10]:
        print(f"    t={e['t']:.2f} {e['a']} <-> {e['b']} impulse {e['impulse']:.4f}")
    # docking estimator vs truth
    td = [r['mission']['tag_docking'] for r in rows if 'mission' in r and 'tag_docking' in r['mission']]
    if td:
        errs = []
        for x in td:
            est, tru = x['estimate'], x['truth']
            if est['target_x'] is None:
                continue
            # compare the target as seen relative to self (estimate frame vs truth frame): removes the odometry frame offset
            def rel(p, prefix_t, prefix_s):
                dx, dy = p[prefix_t + 'x'] - p[prefix_s + 'x'], p[prefix_t + 'y'] - p[prefix_s + 'y']
                c, s = math.cos(p[prefix_s + 'yaw']), math.sin(p[prefix_s + 'yaw'])
                return c * dx + s * dy, -s * dx + c * dy
            ex, ey = rel(est, 'target_', 'self_')
            tx, ty = rel(tru, 'target_', 'self_')
            odo = math.hypot(est['self_x'] - tru['self_x'], est['self_y'] - tru['self_y'])
            errs.append((x['phase'], math.hypot(ex - tx, ey - ty), odo, x['observations'], x['rejected']))
        last = {}
        for ph, e, o, n, rj in errs:
            last[ph] = (e, o, n, rj)
        for ph, (e, o, n, rj) in last.items():
            print(f"  tag_docking phase {ph:9s}: relative-target error {e * 1000:6.1f} mm, follower odometry error {o * 1000:6.1f} mm, obs {n}, rejected {rj}")
    demo = m['results'].get('demo', {})
    for k in ('find', 'lift_static_check', 'coupling', 'placed', 'delivered', 'tag_docking', 'connector_final_state', 'max_slip_on_decks', 'flags'):
        if k in demo:
            print(f"  {k}: {json.dumps(demo[k])[:300]}")


if __name__ == '__main__':
    main(Path(sys.argv[1]))
