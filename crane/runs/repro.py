"""`crane repro RUN [--check]`: rerun one recorded run at its recorded commit and compare.

    crane repro 3fa2c1d0 --check      # exit 1 when it does not reproduce, naming the first differing file

The run's identity (fastsim/identity.py: run_sha256 over scenario, config, adapter, seed and core commit) says what
to rerun. The recorded commit is checked out into a temporary worktree (``git worktree add --detach``, so the
checkout you work in is never touched), and that commit's own fastsim reruns the one seed on the scenario file
the run dir holds, with the recorded config and provenance. Then run_sha256, the directory's tree_sha (file by
file) and the metrics are compared. The worktree is always removed; a rerun that differs is kept for
``crane runs diff``.

Refused with one line (exit 2): a run that is not a fastsim run, a run made from a dirty tree (its diff sha is
shown when recorded), or a commit no repository here has.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

from crane import output, paths
from crane.runs.index import RunNotFound, find, read_dir

# Runs inside the temporary worktree with that commit's code on sys.path: only fastsim APIs a run dir implies.
RERUN = r"""
import json, sys
from pathlib import Path
args = json.loads(sys.argv[1])
src = Path(args['run'])
native = json.loads((src / 'run.json').read_text())
from fastsim import batch, run, scenario as sc
cfg, base = native['config'], Path(args['base'])
adapter = batch.adapter_of(cfg, base)
fmt = sc.scenario_format((cfg.get('scenarios') or {}).get('format'), base)
scenario = fmt.load(src / native['scenario']['file'])
summary = run.simulate(scenario, adapter, cfg, native['seed'], Path(args['out']), base, native.get('provenance') or {})
print(json.dumps({'run_sha256': summary['identity']['run_sha256'], 'outcome': summary['outcome']}))
"""


class Refused(Exception):
    """A run that cannot be reproduced from a commit; the message is the one line."""


def _git(repo, *args, check=False):
    try:
        res = subprocess.run(['git', '-C', str(repo), *args], capture_output=True, text=True, check=False)
    except OSError:
        if check:
            raise Refused('git is not installed') from None
        return None
    if check and res.returncode:
        raise Refused(f'git {" ".join(args[:2])} failed: {(res.stderr or res.stdout).strip()[-200:]}')
    return res.stdout.strip() if res.returncode == 0 else None


def recipe(row) -> dict:
    """What to rerun: {native, seed, commit, repo, base, run_sha256}; Refused names why it cannot be."""
    if row.get('sealed'):
        raise Refused(f"{row['run_id'][:12]} is in a held-out block that is not done; it is sealed (progress only) "
                      'until the block finishes')
    if row.get('source') != 'fastsim':
        raise Refused(f"{row['run_id'][:12]} is a {row.get('source')} record ({row.get('backend') or 'no backend'}); "
                      'repro reruns fastsim runs (T0/T1) only')
    native = json.loads((Path(row['path']) / 'run.json').read_text())
    ident = native.get('identity') or {}
    git = (native.get('provenance') or {}).get('git') or {}
    core = git.get('core') or {}
    dirty = [name for name, part in (('core', ident.get('core') or {}), ('code', core), ('adapter', git.get('adapter')))
             if isinstance(part, dict) and part.get('dirty')]
    if dirty:
        diff_sha = next((p.get('diff_sha') for p in (core, git.get('adapter') or {}) if p.get('diff_sha')), None)
        raise Refused(f"{row['run_id'][:12]} was made from a dirty tree ({', '.join(dirty)}; diff sha "
                      f"{diff_sha[:16] if diff_sha else 'not recorded'}): no commit holds that code")
    commit = (ident.get('core') or {}).get('commit') or core.get('commit')
    if not commit:
        raise Refused(f"{row['run_id'][:12]} records no code commit")
    candidates = [core.get('source'), paths.ROOT]
    repo = None
    for c in candidates:
        if c and Path(c).is_dir() and _git(c, 'rev-parse', '--show-toplevel'):
            if _git(c, 'cat-file', '-e', f'{commit}^{{commit}}') is not None:
                repo = Path(_git(c, 'rev-parse', '--show-toplevel'))
                break
    if repo is None:
        where = ', '.join(str(c) for c in dict.fromkeys(str(c) for c in candidates if c))
        raise Refused(f'commit {commit[:12]} is in no repository here ({where}); git fetch it first')
    config_file = (native.get('provenance') or {}).get('config_file')
    if not config_file:
        raise Refused(f"{row['run_id'][:12]} records no config_file, so its adapter's folder is unknown")
    return {'native': native, 'seed': native['seed'], 'commit': commit, 'repo': repo,
            'config_file': Path(config_file), 'run_sha256': ident.get('run_sha256'),
            'source_root': Path(core['source']) if core.get('source') else repo}


def _base_in(tree, rec):
    """The config's folder inside the worktree when the config lives in the repo, else where it is."""
    cfg = rec['config_file']
    for root in (rec['source_root'], rec['repo']):
        try:
            return tree / cfg.resolve().relative_to(Path(root).resolve()).parent
        except ValueError:
            continue
    return cfg.parent


def first_difference(a, b, skip=('analysis',)):
    """The first file (sorted relative path) that differs between two dirs, or is in only one; None if none."""
    def files(root):
        root = Path(root)
        return {str(p.relative_to(root)): p for p in root.rglob('*')
                if p.is_file() and p.relative_to(root).parts[0] not in skip}
    fa, fb = files(a), files(b)
    for rel in sorted(set(fa) | set(fb)):
        if rel not in fa:
            return rel, 'only in the rerun'
        if rel not in fb:
            return rel, 'missing from the rerun'
        if fa[rel].read_bytes() != fb[rel].read_bytes():
            return rel, 'bytes differ'
    return None


def reproduce(row, timeout=1800) -> dict:
    """Rerun ``row`` (an index row) at its commit and compare; the crane.repro/1 document."""
    from fastsim.identity import tree_sha
    rec = recipe(row)
    tmp = Path(tempfile.mkdtemp(prefix='crane-repro-'))
    tree, out = tmp / 'tree', tmp / 'rerun' / Path(row['path']).name
    keep = False
    try:
        _git(rec['repo'], 'worktree', 'add', '--detach', '--quiet', str(tree), rec['commit'], check=True)
        if not (tree / 'fastsim' / 'run.py').is_file():
            raise Refused(f"commit {rec['commit'][:12]} has no fastsim/run.py")
        env = {**os.environ, 'PYTHONPATH': os.pathsep.join([str(tree), os.environ.get('PYTHONPATH', '')]).rstrip(
            os.pathsep), 'PYTHONDONTWRITEBYTECODE': '1'}
        env.pop('CRANE_RUN_DIR', None)
        args = json.dumps({'run': str(row['path']), 'out': str(out), 'base': str(_base_in(tree, rec))})
        try:
            res = subprocess.run([sys.executable, '-c', RERUN, args], cwd=tree, env=env, capture_output=True,
                                 text=True, timeout=timeout)
        except subprocess.TimeoutExpired:
            raise Refused(f'the rerun took over {timeout} s') from None
        if res.returncode:
            keep = out.exists()
            tail = (res.stderr or res.stdout).strip().splitlines()[-1:] or ['no output']
            return {'schema': 'crane.repro/1', 'run_id': row['run_id'], 'path': row['path'], 'commit': rec['commit'],
                    'reproduced': False, 'error': tail[0], 'kept': str(out) if keep else None}
        got = json.loads(res.stdout.strip().splitlines()[-1])
        first = first_difference(row['path'], out)
        ma, mb = (read_dir(row['path']) or {}).get('metrics') or {}, (read_dir(out) or {}).get('metrics') or {}
        metrics = [{'key': k, 'recorded': ma.get(k), 'rerun': mb.get(k)} for k in sorted(set(ma) | set(mb))
                   if ma.get(k) != mb.get(k)]
        doc = {'schema': 'crane.repro/1', 'run_id': row['run_id'], 'path': row['path'], 'commit': rec['commit'],
               'repo': str(rec['repo']),
               'run_sha256': {'recorded': rec['run_sha256'], 'rerun': got['run_sha256']},
               'tree_sha': {'recorded': tree_sha(row['path']), 'rerun': tree_sha(out)},
               'first_difference': {'file': first[0], 'why': first[1]} if first else None,
               'metrics': metrics}
        doc['reproduced'] = (doc['run_sha256']['recorded'] == doc['run_sha256']['rerun'] and first is None
                             and not metrics)
        keep = not doc['reproduced']
        doc['kept'] = str(out) if keep else None
        return doc
    finally:
        _git(rec['repo'], 'worktree', 'remove', '--force', str(tree))
        _git(rec['repo'], 'worktree', 'prune')
        shutil.rmtree(tree, ignore_errors=True)
        if not keep:
            shutil.rmtree(tmp, ignore_errors=True)


def main(args):
    out = args.output
    try:
        row = find(args.run)
    except RunNotFound as error:
        return out.usage('repro', str(error), 'crane runs ls (or crane repro RUN_DIR)')
    try:
        doc = reproduce(row, timeout=args.timeout)
    except Refused as error:
        return out.usage('repro', str(error))
    bad = output.FAILED if (args.check and not doc['reproduced']) else output.OK
    if out.doc(doc):
        return bad
    out.line(f"repro {doc['run_id'][:12]} at {doc['commit'][:12]} (a temporary worktree, removed)")
    if doc.get('error'):
        out.say(f"repro: the rerun failed: {doc['error']}")
        return output.FAILED if args.check else output.OK
    rs, ts = doc['run_sha256'], doc['tree_sha']
    first = doc['first_difference']
    rows = [('run_sha256', 'same' if rs['recorded'] == rs['rerun'] else 'differs', rs['rerun'][:16]),
            ('tree_sha', 'same' if ts['recorded'] == ts['rerun'] else 'differs',
             f"first differing file: {first['file']} ({first['why']})" if first else ts['rerun'][:16]),
            ('metrics', f"{len(doc['metrics'])} differ" if doc['metrics'] else 'same',
             ', '.join(f"{m['key']} {m['recorded']} -> {m['rerun']}" for m in doc['metrics'][:4]))]
    out.table(rows)
    if doc['reproduced']:
        out.say('repro: reproduced')
        return output.OK
    out.say(f"repro: does not reproduce; the rerun is kept at {doc['kept']}")
    out.next(f"crane runs diff {row['path']} {doc['kept']}")
    return bad
