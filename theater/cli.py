"""``crane-theater`` (``python3 -m theater``): ``select`` picks recorded runs from the coordinator's
index into a render manifest.

    crane-theater select --study S | --hypothesis H [--iterations 1..50] [--top-k 20] [--best-per-iteration]
                         [--failures | --failures-only] [--outcome finished,failed] [--valid-only]
                         [--sample 500 --seed 1] [--recorder unity] [--course SHA] --out manifest.json
    crane-theater fetch manifest.json --dir trajs/      # download each .ctraj, check its sha256
    crane-theater info run.ctraj                        # header, size and data problems

The manifest (``crane-theater-select/1``) lists each chosen run with its iteration, score, outcome,
params, insight, roles (``top``: in its iteration's top k; ``failure``; ``sample``; ``all``
when nothing narrows the choice) and its trajectory
(the R2 key, sha256 and size), plus per-iteration summaries and the course and water every run should
share; runs that differ are flagged (``course_mismatch``, ``water_mismatch``), as are runs without an
uploaded trajectory (``traj_missing``) and invalid runs. The render step reads it (``theater/pack``).
Selection is deterministic: equal index contents and arguments give an equal manifest.

Environment: CRANE_COORDINATOR (URL) and CRANE_TOKEN (a viewer token is enough).
"""

from __future__ import annotations

import collections
import hashlib
import json
import os
import random
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

SCHEMA = 'crane-theater-select/1'
PAGE = 1000
FAILED = ('failed', 'timeout', 'invalid')
RUN_KEYS = ('run_id', 'job_id', 'seed', 'study', 'hypothesis_id', 'arm', 'iteration', 'generation', 'score', 'outcome',
            'failure_reason', 'progress', 'valid', 'duration_s', 'params', 'params_sha256', 'insight', 'recorder',
            'water_mode', 'course_sha256', 'course_family', 'build_hash', 'rank_in_iteration')


class Coordinator:
    """The few coordinator reads the theater needs (stdlib only)."""

    def __init__(self, url: str, token: str | None, timeout: float = 120.0):
        self.url, self.token, self.timeout = url.rstrip('/'), token, timeout

    def _open(self, path: str):
        req = urllib.request.Request(self.url + path, headers={'authorization': f'Bearer {self.token}'} if self.token else {})
        try:
            return urllib.request.urlopen(req, timeout=self.timeout)
        except urllib.error.HTTPError as e:
            raise SystemExit(f'coordinator {path.split("?")[0]}: HTTP {e.code} {e.read()[:300].decode(errors="replace")}')

    def get_json(self, path: str, **q):
        qs = urllib.parse.urlencode({k: v for k, v in q.items() if v is not None})
        with self._open(path + (f'?{qs}' if qs else '')) as r:
            return json.loads(r.read())

    def get_bytes(self, path: str) -> bytes:
        with self._open(path) as r:
            return r.read()

    def runs(self, **q) -> list:
        """Every run the query matches (all pages)."""
        out, offset = [], 0
        while offset is not None:
            page = self.get_json('/v1/runs/query', **q, limit=PAGE, offset=offset)
            out += page['runs']
            offset = page.get('next_offset')
        return out


def parse_iterations(text: str | None):
    """'3', '3..9', '3:9', '..9', '3..' -> (from, to)."""
    if not text:
        return None, None
    for sep in ('..', ':'):
        if sep in text:
            a, b = text.split(sep, 1)
            return (int(a) if a.strip() else None), (int(b) if b.strip() else None)
    return int(text), int(text)


def query_args(a) -> dict:
    lo, hi = parse_iterations(a.iterations)
    return {'study': a.study, 'hypothesis': a.hypothesis, 'iter_from': lo, 'iter_to': hi,
            'outcome': a.outcome, 'valid': 1 if a.valid_only else None, 'recorder': a.recorder,
            'course_sha256': a.course, 'order': 'asc' if a.lowest_best else 'desc'}


def _majority(values):
    counts = collections.Counter(v for v in values if v is not None)
    return min(counts, key=lambda v: (-counts[v], str(v))) if counts else None


def _score_key(run, lowest_best):
    s = run.get('score')
    return (s is None, (s if lowest_best else -s) if s is not None else 0, run['run_id'])


def choose(runs: list, *, top_k=None, failures=False, failures_only=False, sample=None, seed=0, lowest_best=False):
    """{run_id: [roles]} from the queried runs (already filtered by the index)."""
    roles = collections.defaultdict(list)
    by_iter = collections.defaultdict(list)
    for r in runs:
        by_iter[r['iteration']].append(r)
    if not failures_only:
        for it in sorted(by_iter):
            ranked = sorted(by_iter[it], key=lambda r: _score_key(r, lowest_best))
            if top_k:
                for r in ranked[:top_k]:
                    roles[r['run_id']].append('top')
            elif not sample:
                for r in ranked:
                    roles[r['run_id']].append('all')
    if failures or failures_only:
        for r in runs:
            if r.get('outcome') in FAILED:
                roles[r['run_id']].append('failure')
    if sample and not failures_only:
        rest = sorted(r['run_id'] for r in runs if r['run_id'] not in roles)
        for rid in random.Random(seed).sample(rest, min(sample, len(rest))):
            roles[rid].append('sample')
    return roles


def manifest(runs: list, roles: dict, query: dict, *, lowest_best=False) -> dict:
    chosen = [r for r in runs if r['run_id'] in roles]
    chosen.sort(key=lambda r: (r['iteration'],) + _score_key(r, lowest_best))
    course = _majority(r.get('course_sha256') for r in chosen)
    water = _majority(r.get('water_mode') for r in chosen)
    out_runs, warnings = [], []
    for r in chosen:
        flags = []
        if not r.get('traj_key') or r.get('traj_size') is None:
            flags.append('traj_missing')
        if r.get('traj_deleted_at'):
            flags.append('traj_deleted')
        if course and r.get('course_sha256') not in (None, course):
            flags.append('course_mismatch')
        if water and r.get('water_mode') not in (None, water):
            flags.append('water_mismatch')
        if r.get('valid') is False or r.get('outcome') == 'invalid':
            flags.append('invalid')
        out_runs.append({**{k: r.get(k) for k in RUN_KEYS}, 'id': r['run_id'], 'roles': sorted(set(roles[r['run_id']])),
                         'flags': flags, 'traj': {'key': r.get('traj_key'), 'sha256': r.get('traj_sha256'),
                                                  'size': r.get('traj_size')}})
    iterations = []
    for it, group in sorted(_group(runs).items()):
        ranked = sorted(group, key=lambda r: _score_key(r, lowest_best))
        best = ranked[0]
        insights = sorted({r['insight'] for r in group if r.get('insight')})
        iterations.append({'iteration': it, 'runs': len(group), 'selected': sum(r['run_id'] in roles for r in group),
                           'finished': sum(r.get('outcome') == 'finished' for r in group),
                           'failed': sum(r.get('outcome') in FAILED for r in group),
                           'best_run': best['run_id'], 'best_score': best.get('score'), 'insights': insights})
    for flag in ('traj_missing', 'course_mismatch', 'water_mismatch', 'invalid'):
        n = sum(flag in r['flags'] for r in out_runs)
        if n:
            warnings.append(f'{n} selected runs are {flag}')
    return {'schema': SCHEMA, 'query': {k: v for k, v in query.items() if v is not None},
            'study': query.get('study'), 'hypothesis': query.get('hypothesis'),
            'score_order': 'asc' if lowest_best else 'desc',
            'course': {'truth_sha256': course, 'family': _majority(r.get('course_family') for r in chosen)},
            'water': {'mode': water}, 'counts': {'matched': len(runs), 'selected': len(out_runs)},
            'iterations': iterations, 'runs': out_runs, 'warnings': warnings}


def _group(runs):
    g = collections.defaultdict(list)
    for r in runs:
        g[r['iteration']].append(r)
    return g


def cmd_select(a, coord=None) -> dict:
    if not (a.study or a.hypothesis or a.from_file):
        raise SystemExit('give --study or --hypothesis')
    q = query_args(a)
    if a.from_file:
        runs = json.loads(Path(a.from_file).read_text())
        runs = runs['runs'] if isinstance(runs, dict) else runs
    else:
        coord = coord or client(a)
        runs = coord.runs(**q)
    top_k = 1 if a.best_per_iteration else a.top_k
    roles = choose(runs, top_k=top_k, failures=a.failures, failures_only=a.failures_only, sample=a.sample,
                   seed=a.seed, lowest_best=a.lowest_best)
    doc = manifest(runs, roles, {**q, 'top_k': top_k, 'failures': a.failures or None,
                                 'failures_only': a.failures_only or None, 'sample': a.sample, 'seed': a.seed if a.sample else None},
                   lowest_best=a.lowest_best)
    text = json.dumps(doc, indent=1, sort_keys=True) + '\n'
    if a.out:
        Path(a.out).write_text(text)
    print(f'{doc["counts"]["selected"]} of {doc["counts"]["matched"]} runs over {len(doc["iterations"])} iterations'
          + (f' -> {a.out}' if a.out else ''))
    for w in doc['warnings']:
        print(f'warning: {w}')
    return doc


def cmd_fetch(a, coord=None) -> list:
    doc = json.loads(Path(a.manifest).read_text())
    coord = coord or client(a)
    out = Path(a.dir)
    out.mkdir(parents=True, exist_ok=True)
    got, bad = [], []
    for r in doc['runs']:
        key, want = r['traj']['key'], r['traj']['sha256']
        if not key or 'traj_missing' in r['flags']:
            continue
        dest = out / f'{r["run_id"]}.ctraj'
        if dest.is_file() and hashlib.sha256(dest.read_bytes()).hexdigest() == want:
            got.append(dest)
            continue
        data = coord.get_bytes('/v1/artifact/' + urllib.parse.quote(key))
        if want and hashlib.sha256(data).hexdigest() != want:
            bad.append(r['run_id'])
            continue
        dest.write_bytes(data)
        got.append(dest)
    print(f'{len(got)} trajectories in {out}' + (f'; sha256 mismatch: {", ".join(bad)}' if bad else ''))
    if bad:
        raise SystemExit(1)
    return got


def cmd_info(a):
    from theater import traj
    t = traj.read(a.file)
    h = dict(t.header)
    print(json.dumps(h, indent=1, sort_keys=True))
    print(f'{Path(a.file).stat().st_size} bytes, {t.ticks} ticks at {t.hz} Hz, {len(t.events)} events')
    for p in traj.problems(t):
        print(f'problem: {p}')


def client(a) -> Coordinator:
    url = getattr(a, 'coordinator', None) or os.environ.get('CRANE_COORDINATOR')
    if not url:
        raise SystemExit('set CRANE_COORDINATOR or pass --coordinator')
    return Coordinator(url, getattr(a, 'token', None) or os.environ.get('CRANE_TOKEN'))


def main(argv=None):
    import argparse
    p = argparse.ArgumentParser(prog='crane-theater', description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--coordinator', help='coordinator URL (default: CRANE_COORDINATOR)')
    p.add_argument('--token', help='viewer or admin token (default: CRANE_TOKEN)')
    sub = p.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('select', help='choose runs from the index into a render manifest')
    s.add_argument('--study')
    s.add_argument('--hypothesis')
    s.add_argument('--iterations', help='N, A..B, A.. or ..B')
    s.add_argument('--top-k', type=int, help='the best k runs of each iteration')
    s.add_argument('--best-per-iteration', action='store_true', help='same as --top-k 1')
    s.add_argument('--failures', action='store_true', help='also every failed, timed-out or invalid run')
    s.add_argument('--failures-only', action='store_true')
    s.add_argument('--outcome', help='comma list: finished,failed,timeout,invalid,unknown')
    s.add_argument('--valid-only', action='store_true')
    s.add_argument('--sample', type=int, help='also N more runs drawn with --seed')
    s.add_argument('--seed', type=int, default=0)
    s.add_argument('--recorder', help='unity, fastsim, ...')
    s.add_argument('--course', help='only runs on this course truth sha256')
    s.add_argument('--lowest-best', action='store_true', help='a lower score is better')
    s.add_argument('--from-file', help='a saved /v1/runs/query response instead of the coordinator')
    s.add_argument('--out', help='manifest path (default: print the summary only)')
    f = sub.add_parser('fetch', help="download a manifest's trajectories")
    f.add_argument('manifest')
    f.add_argument('--dir', required=True)
    i = sub.add_parser('info', help="a .ctraj's header and problems")
    i.add_argument('file')
    a = p.parse_args(argv)
    return {'select': cmd_select, 'fetch': cmd_fetch, 'info': cmd_info}[a.cmd](a)


if __name__ == '__main__':
    main()
