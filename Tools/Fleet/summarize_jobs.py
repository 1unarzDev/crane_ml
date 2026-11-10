#!/usr/bin/env python3
"""Success-rate table for a harness output directory (summary.jsonl + runs/*/manifest.json): per seed the evaluator-truth outcome,
what the autonomy library believed, the host verdict, disputed claims, jams, failure class; overall success rate with a Wilson 95%
interval. Truth is the only success criterion.

    python3 Tools/Fleet/summarize_jobs.py OUT_DIR [--json OUT.json]
"""
import json
import math
import sys
from pathlib import Path


def wilson(k, n, z=1.96):
    if n == 0:
        return (float('nan'), float('nan'))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def main(out: Path, js=None):
    rows = []
    for line in (out / 'summary.jsonl').read_text().splitlines():
        s = json.loads(line)
        row = {'id': s['id'], 'success': s['success'], 'outcome': s['outcome']}
        man = Path(s['dir']) / 'manifest.json' if s.get('dir') else None
        if man and man.exists():
            m = json.loads(man.read_text())
            a = m['results'].get('autonomy', {})
            row.update(seed=m['seed'], t_end=round(m['t_end'], 1), believed_complete=a.get('library_believed_complete'),
                       host=(a.get('host_verdict') or {}).get('ok'), disputed=a.get('disputed'), first_failure_class=a.get('first_failure_class'),
                       first_failure=a.get('first_failure'), jams=len(a.get('jams') or []),
                       payload=(a.get('payload_truth') or {}).get('state'),
                       tasks_failed=[t['id'] for t in a.get('tasks', []) if t['state'] not in ('Succeeded',)])
            row['disputed'] = a.get('disagreements')
        replay = out / 'replays' / s['id']
        row['replay'] = str(replay) if replay.exists() else None
        rows.append(row)
    k = sum(r['success'] for r in rows)
    lo, hi = wilson(k, len(rows))
    print(f"truth success {k}/{len(rows)}  (Wilson 95% {lo:.2f}-{hi:.2f})")
    false_success = sum(1 for r in rows if not r['success'] and r.get('believed_complete'))
    print(f"library believed complete but truth failed: {false_success}")
    for r in rows:
        print(f"  {r['id']:28s} {('OK ' if r['success'] else 'FAIL')} {r['outcome']:40s} believed={r.get('believed_complete')} host={r.get('host')} "
              f"disputed={r.get('disputed')} jams={r.get('jams')} class={r.get('first_failure_class')} failed={r.get('tasks_failed')} payload={r.get('payload')}")
    if js:
        Path(js).write_text(json.dumps({'success': k, 'n': len(rows), 'wilson95': [lo, hi], 'false_success': false_success, 'rows': rows}, indent=2) + '\n')


if __name__ == '__main__':
    a = sys.argv[1:]
    main(Path(a[0]), a[a.index('--json') + 1] if '--json' in a else None)
