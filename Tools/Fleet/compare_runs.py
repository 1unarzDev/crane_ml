#!/usr/bin/env python3
"""Compare two fleet run directories: byte equality of states.jsonl and events.jsonl, and the largest
numeric difference per top-level section (truth / estimate / actuation) when they differ. Also reads the
trajectory with theater.traj when the repo is importable.

    python3 Tools/Fleet/compare_runs.py RUN_A RUN_B [--json OUT]
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path


def sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def walk(a, b, path, out):
    if isinstance(a, dict) and isinstance(b, dict):
        for k in a.keys() | b.keys():
            if k in a and k in b:
                walk(a[k], b[k], path + (k,), out)
            else:
                out.append((path + (k,), float('inf')))
    elif isinstance(a, list) and isinstance(b, list) and len(a) == len(b):
        for i, (x, y) in enumerate(zip(a, b)):
            walk(x, y, path + (i,), out)
    elif isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool):
        if a != b:
            out.append((path, abs(a - b)))
    elif a != b:
        out.append((path, float('inf')))


def compare(a: Path, b: Path) -> dict:
    r = {'a': str(a), 'b': str(b)}
    for name in ('states.jsonl', 'events.jsonl', 'trajectory.ctraj'):
        fa, fb = a / name, b / name
        r[name] = {'sha256_a': sha(fa), 'sha256_b': sha(fb), 'identical': fa.read_bytes() == fb.read_bytes()}
    la = (a / 'states.jsonl').read_text().splitlines()
    lb = (b / 'states.jsonl').read_text().splitlines()
    r['rows'] = [len(la), len(lb)]
    first = next((i for i, (x, y) in enumerate(zip(la, lb)) if x != y), None)
    r['first_differing_row'] = first
    worst = {}
    for x, y in zip(la, lb):
        if x == y:
            continue
        diffs = []
        walk(json.loads(x), json.loads(y), (), diffs)
        for path, d in diffs:
            key = path[0] if path else ''
            if d > worst.get(key, (0, None))[0]:
                worst[key] = (d, '.'.join(map(str, path)))
    r['max_abs_diff'] = {k: {'value': v[0], 'at': v[1]} for k, v in worst.items()}
    if first is not None:
        r['first_differing_t'] = json.loads(la[first])['t']
    try:
        sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
        from theater import traj
        t = traj.read(a / 'trajectory.ctraj')
        r['trajectory'] = {'agents': t.agents(), 'ticks': t.ticks, 'hz': t.hz, 'events': len(t.events),
                           'problems': traj.problems(t, max_speed=2.0)}
    except Exception as e:  # noqa: BLE001
        r['trajectory'] = {'error': repr(e)}
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('a'); ap.add_argument('b'); ap.add_argument('--json')
    args = ap.parse_args()
    r = compare(Path(args.a), Path(args.b))
    s = json.dumps(r, indent=2, default=str)
    print(s)
    if args.json:
        Path(args.json).write_text(s + '\n')


if __name__ == '__main__':
    main()
