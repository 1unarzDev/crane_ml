#!/usr/bin/env python3
"""Bins resolved task parameters and corner flags across course truths (design §6).

    coverage.py TRUTH.json|SESSION_DIR... [--bins 5] [--json]

Numeric params get ``--bins`` equal-width bins over the values seen; choices are counted.
"""

import argparse
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path


def truths(paths):
    for p in map(Path, paths):
        for f in ([p] if p.is_file() else sorted(p.rglob('course_truth.json')) or sorted(p.glob('*.json'))):
            t = json.loads(f.read_text())
            if 'tasks' in t:
                yield t


def coverage(items, bins=5):
    values, modes = defaultdict(list), Counter()
    for t in items:
        modes[(t.get('generator') or {}).get('mode', '?')] += 1
        for task in t['tasks']:
            for key, v in task.get('params', {}).items():
                values[f"{task['id']}.{key}"].append(v)
    out = {'courses': sum(modes.values()), 'modes': dict(modes), 'params': {}}
    for key, vs in sorted(values.items()):
        if all(isinstance(v, (int, float)) for v in vs):
            lo, hi = min(vs), max(vs)
            width = (hi - lo) / bins or 1
            counts = Counter(min(int((v - lo) / width), bins - 1) for v in vs)
            out['params'][key] = {'min': lo, 'max': hi, 'bins': [counts.get(i, 0) for i in range(bins)]}
        else:
            out['params'][key] = dict(Counter(map(str, vs)))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('paths', nargs='+')
    ap.add_argument('--bins', type=int, default=5)
    ap.add_argument('--json', action='store_true')
    args = ap.parse_args(argv)
    cov = coverage(truths(args.paths), args.bins)
    if args.json:
        print(json.dumps(cov, indent=1))
        return 0
    print(f"{cov['courses']} courses, modes {cov['modes']}")
    for key, v in cov['params'].items():
        print(f"  {key:<45} {v}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
