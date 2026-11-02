"""Twin-pair archive: a fast run and a twin run (another simulator, or the field) of one scenario.

    python -m fastsim.pairs wait FAST_RUN...  [--root DIR]           # fast runs await a twin
    python -m fastsim.pairs pair TWIN_RUN... [--root DIR] [--describe REF] [--metrics REF]

Append-only, outside every work tree (``CRANE_FASTSIM_PAIRS``, default
~/.cache/crane-fastsim-pairs): ``waiting/<sha>/`` holds whole fast runs (``waiting.jsonl``
lists them), ``pairs/<pair_id>/`` holds ``fast/``, ``twin/`` and ``diff.json``, and
``index.jsonl`` one line per pair. Runs pair only on an equal scenario identity (pair_key, then
truth_sha256; equal keys with different truths are an error) and an equal condition. Nothing is
rewritten: an existing id is skipped and a tool change bumps ``ARCHIVE_VERSION``.

A run is described by ``describe(run_dir) -> {pair_key, truth_sha256, seed, condition, source,
provenance}`` and scored by ``metrics(run_dir) -> dict``; the defaults read a fast run's
run.json, and a domain passes its own for twin runs laid out its way.
"""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import sys

from fastsim import plugin
from fastsim.identity import digest, tree_sha

ARCHIVE_VERSION = 1


def default_root():
    return Path(os.environ.get('CRANE_FASTSIM_PAIRS', '~/.cache/crane-fastsim-pairs')).expanduser()


def describe_run(run_dir):
    run = json.loads((Path(run_dir) / 'run.json').read_text())
    return {'pair_key': run['scenario']['pair_key'], 'truth_sha256': run['scenario']['truth_sha256'],
            'seed': run['seed'], 'condition': run['condition'], 'source': 'fastsim',
            'provenance': {'identity': run['identity'], 'git': run['provenance'].get('git'),
                           'calibration': run['config'].get('calibration')}}


def flat_numbers(tree, prefix=''):
    """Numeric leaves of a nested dict, keyed by dotted path (booleans left out)."""
    out = {}
    for key, value in (tree or {}).items():
        name = f'{prefix}.{key}' if prefix else str(key)
        if isinstance(value, dict):
            out.update(flat_numbers(value, name))
        elif isinstance(value, (int, float)) and not isinstance(value, bool):
            out[name] = value
    return out


def run_metrics(run_dir):
    run = json.loads((Path(run_dir) / 'run.json').read_text())
    return {'run': {'duration_s': run['sim_s'], 'contacts': len(run['contacts'])}}


def pairable(a, b):
    """True when two descriptions ran the same scenario (pair_key, then truth_sha256)."""
    if a['pair_key'] != b['pair_key']:
        return False
    if a['truth_sha256'] != b['truth_sha256']:
        raise ValueError(f"pair_key {a['pair_key']}: truth_sha256 differs, not a pair")
    return True


def _append(path, row):
    with open(path, 'a') as out:
        out.write(json.dumps(row, sort_keys=True) + '\n')


def _rows(path):
    return [json.loads(s) for s in path.read_text().splitlines() if s.strip()] if path.exists() else []


def wait(runs, root=None, describe=describe_run):
    """Copy fast runs into the archive to wait for their twins; returns the new waiting ids."""
    root = Path(root or default_root())
    (root / 'waiting').mkdir(parents=True, exist_ok=True)
    added = []
    for run in runs:
        sha = tree_sha(run)[:16]
        if (root / 'waiting' / sha).exists():
            continue
        shutil.copytree(run, root / 'waiting' / sha)
        _append(root / 'waiting.jsonl', {'waiting_id': sha, 'from': str(Path(run).resolve()),
                                        'archive_version': ARCHIVE_VERSION, **describe(run)})
        added.append(sha)
    return added


def pair(twins, root=None, describe=describe_run, metrics=run_metrics, fast_metrics=run_metrics):
    """Archive every waiting fast run of the same scenario and condition as a twin run."""
    root = Path(root or default_root())
    waiting = _rows(root / 'waiting.jsonl')
    done = {r['pair_id'] for r in _rows(root / 'index.jsonl')}
    made = []
    for twin in twins:
        t = describe(twin)
        for row in waiting:
            if row['condition'] != t['condition'] or not pairable(row, t):
                continue
            pair_id = digest([t['pair_key'], t['truth_sha256'], t['condition'], row['waiting_id'],
                              tree_sha(twin)])[:16]
            if pair_id in done:
                continue
            target = root / 'pairs' / pair_id
            shutil.copytree(root / 'waiting' / row['waiting_id'], target / 'fast')
            shutil.copytree(twin, target / 'twin')
            numbers = {'fast': flat_numbers(fast_metrics(target / 'fast')),
                       'twin': flat_numbers(metrics(target / 'twin'))}
            diff = {k: {'fast': numbers['fast'].get(k), 'twin': numbers['twin'].get(k),
                        'fast_minus_twin': numbers['fast'][k] - numbers['twin'][k]
                        if k in numbers['fast'] and k in numbers['twin'] else None}
                    for k in sorted(set(numbers['fast']) | set(numbers['twin']))}
            (target / 'diff.json').write_text(json.dumps(diff, indent=1, sort_keys=True) + '\n')
            _append(root / 'index.jsonl', {
                'pair_id': pair_id, 'archive_version': ARCHIVE_VERSION,
                'archived_at': datetime.now(timezone.utc).isoformat(timespec='seconds'),
                'pair_key': t['pair_key'], 'truth_sha256': t['truth_sha256'], 'seed': t.get('seed'),
                'condition': t['condition'], 'waiting_id': row['waiting_id'],
                'twin_from': str(Path(twin).resolve()), 'twin_source': t.get('source'),
                'fast': row.get('provenance'), 'twin': t.get('provenance')})
            done.add(pair_id)
            made.append(pair_id)
    return made


def main(argv=None):
    parser = argparse.ArgumentParser(prog='fastsim.pairs', description=__doc__.split('\n')[0])
    parser.add_argument('action', choices=('wait', 'pair'))
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--root', type=Path, default=None)
    parser.add_argument('--describe', help="twin runs' describer MODULE:FUNC (default: run.json)")
    parser.add_argument('--metrics', help="twin runs' metrics MODULE:FUNC (default: run.json)")
    args = parser.parse_args(argv)
    if args.action == 'wait':
        made = wait(args.runs, args.root)
    else:
        made = pair(args.runs, args.root, plugin.load(args.describe) if args.describe else describe_run,
                    plugin.load(args.metrics) if args.metrics else run_metrics)
    print(json.dumps({'action': args.action, 'root': str(args.root or default_root()), 'new': made}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
