"""The local run store behind ``python -m viewer serve``: crane-traj/1 runs, course truths and judge reports
found under a set of roots, served in the coordinator's feed shapes (``cluster/coordinator/src/replay.ts``) so
one page reads both. Run ids are the header's ``run_id`` (else the file's path under its root)."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

from theater import traj as ctraj

JUDGE_SCHEMA = 'crane.judge.report/'


def _f(v):
    return round(v, 3) if isinstance(v, (int, float)) and math.isfinite(v) else 0.0


def reduce_course(t: dict) -> dict:
    """Course truth -> what a 2-D view draws: the feed's reduced form plus ids, gates and task footprints."""
    els = []
    for e in t.get('elements') or []:
        if not isinstance(e, dict):
            continue
        els.append({'id': e.get('id'), 'x': _f(e.get('x')), 'y': _f(e.get('y')),
                    'r': _f(e.get('radius', e.get('r', 0.25))) or 0.25, 'label': str(e.get('label') or ''),
                    'role': str(e.get('role') or ''), 'task': str(e.get('task') or '')})
    gates = [{'id': g.get('id'), 'task': g.get('task'), 'port': g.get('port'), 'starboard': g.get('starboard'),
              'x': _f(g.get('x')), 'y': _f(g.get('y'))} for g in t.get('gates') or [] if isinstance(g, dict)]
    tasks = [{'id': k.get('id'), 'footprint': [[_f(p[0]), _f(p[1])] for p in k.get('footprint') or []]}
             for k in t.get('tasks') or [] if isinstance(k, dict)]
    v = t.get('vessel') or {}
    return {'name': t.get('name'), 'family': t.get('family'),
            'start': [_f(t.get('startX')), _f(t.get('startY')), _f(t.get('startYaw'))],
            'boundary': [[_f(p[0]), _f(p[1])] for p in t.get('boundary') or [] if isinstance(p, (list, tuple))],
            'elements': els, 'gates': gates, 'tasks': tasks,
            'current': [_f(t.get('currentSpeed')), _f(t.get('currentToward'))],
            'vessel': {'length': _f(v.get('length')) or None, 'beam': _f(v.get('beam')) or None}}


def _ranks(v):
    o = sorted(range(len(v)), key=lambda i: v[i])
    rk = [0.0] * len(v)
    i = 0
    while i < len(o):
        j = i
        while j + 1 < len(o) and v[o[j + 1]] == v[o[i]]:
            j += 1
        for k in range(i, j + 1):
            rk[o[k]] = (i + j) / 2 + 1
        i = j + 1
    return rk


def rank_agreement(pairs) -> dict | None:
    """Spearman rho of low- vs high-tier scores over the scored pairs, and MMRV (the largest high-tier score
    gap among pairs the low tier ranks the other way). Computed here so the page draws, never derives."""
    ps = [p for p in pairs if isinstance(p.get('low_score'), (int, float)) and isinstance(p.get('high_score'), (int, float))]
    if len(ps) < 3:
        return None
    xs, ys = [p['low_score'] for p in ps], [p['high_score'] for p in ps]
    rx, ry = _ranks(xs), _ranks(ys)
    n = len(ps)
    mx, my = sum(rx) / n, sum(ry) / n
    cov = sum((a - mx) * (b - my) for a, b in zip(rx, ry))
    sx = math.sqrt(sum((a - mx) ** 2 for a in rx))
    sy = math.sqrt(sum((b - my) ** 2 for b in ry))
    mmrv, rev = 0.0, 0
    for i in range(n):
        for j in range(n):
            if xs[i] > xs[j] and ys[i] < ys[j]:
                rev += 1
                mmrv = max(mmrv, ys[j] - ys[i])
    return {'n': n, 'spearman': round(cov / (sx * sy), 3) if sx and sy else None, 'mmrv': mmrv, 'reversed_pairs': rev,
            'agree': sum(1 for x, y in zip(xs, ys) if x == y)}


class Run:
    __slots__ = ('id', 'path', 'lod', 'header', '_sha', '_lsha')

    def __init__(self, rid, path, lod, header):
        self.id, self.path, self.lod, self.header = rid, path, lod, header
        self._sha = self._lsha = None

    @property
    def sha(self):
        if self._sha is None:
            self._sha = hashlib.sha256(self.path.read_bytes()).hexdigest()
        return self._sha

    @property
    def lod_sha(self):
        if self.lod and self._lsha is None:
            self._lsha = hashlib.sha256(self.lod.read_bytes()).hexdigest()
        return self._lsha

    def brief(self) -> dict:
        h = self.header
        c = h.get('course') or {}
        ref = h.get('referee') or {}
        return {'run_id': self.id, 'job_id': None, 'study': h.get('study'), 'arm': h.get('arm'), 'tier': h.get('tier'),
                'seed': h.get('seed'), 'iteration': h.get('iteration', 0), 'task': h.get('task'), 'score': h.get('score'),
                'outcome': h.get('outcome'), 'course': c.get('truth_sha256'), 'replay': 'ready',
                'traj_sha256': self.sha, 'lod_sha256': self.lod_sha, 'duration_s': h.get('duration_s'),
                'pair': h.get('pair'), 'failed': ref.get('failed'), 'flags': h.get('flags')}


class Store:
    def __init__(self, roots):
        self.roots = [Path(r).expanduser().resolve() for r in roots]
        self.runs: dict[str, Run] = {}
        self.courses: dict[str, Path] = {}
        self.reports: dict[str, Path] = {}
        self._parsed: dict = {}
        self.scan()

    def scan(self):
        runs, courses, reports = {}, {}, {}
        for root in self.roots:
            files = [root] if root.is_file() else sorted(root.rglob('*'))
            for p in files:
                if not p.is_file():
                    continue
                n = p.name
                if n.endswith('.ctraj') and not n.endswith('.lod.ctraj'):
                    try:
                        h = ctraj.read_header(p)
                    except Exception:  # noqa: BLE001 - an unreadable file is skipped, not fatal
                        continue
                    rid = str(h.get('run_id') or p.relative_to(root if root.is_dir() else root.parent).with_suffix(''))
                    lod = p.with_name(p.stem + '.lod.ctraj')
                    runs[rid] = Run(rid, p, lod if lod.exists() else None, h)
                    sha = (h.get('course') or {}).get('truth_sha256')
                    ct = p.with_name('course_truth.json')
                    if sha and ct.exists():
                        courses.setdefault(sha, ct)
                elif p.parent.name == 'courses' and n.endswith('.json'):
                    courses.setdefault(p.stem, p)
                elif n == 'report.json':
                    try:
                        head = json.loads(p.read_text())
                    except (OSError, ValueError):
                        continue
                    if str(head.get('schema', '')).startswith(JUDGE_SCHEMA):
                        rid = p.parent.name
                        reports[rid if rid not in reports else str(p.parent)] = p
        self.runs, self.courses, self.reports = runs, courses, reports

    # ---- the feed shapes ----
    def index(self) -> dict:
        runs = [r.brief() for r in self.runs.values()]
        runs.sort(key=lambda b: (str(b['study']), str(b['arm']), b['seed'] if b['seed'] is not None else -1, b['run_id']))
        studies = {}
        for b in runs:
            s = studies.setdefault(b['study'] or '', {'study': b['study'], 'runs': 0, 'arms': set(), 'seeds': set()})
            s['runs'] += 1
            s['arms'].add(b['arm'])
            s['seeds'].add(b['seed'])
        pairs = [{'id': b['pair']['id'], 'low': b['run_id'], 'high': b['pair'].get('high'), 'seed': b['seed'],
                  'task': b['task'], 'low_score': b['score'],
                  'high_score': (self.runs[b['pair']['high']].header.get('score') if b['pair'].get('high') in self.runs else None)}
                 for b in runs if isinstance(b.get('pair'), dict) and b['pair'].get('id')]
        return {'source': 'local', 'runs': runs, 'pairs': pairs, 'rank': rank_agreement(pairs),
                'studies': [{'study': s['study'], 'runs': s['runs'], 'arms': sorted(a for a in s['arms'] if a),
                             'seeds': sorted(x for x in s['seeds'] if x is not None)} for s in studies.values()],
                'judge': [self.report_brief(k) for k in sorted(self.reports)]}

    def report_brief(self, key) -> dict:
        p = self.reports[key]
        r = json.loads(p.read_text())
        v = p.with_name('verdict.txt')
        return {'id': key, 'kind': r.get('kind'), 'created': r.get('created'), 'n_pairs': r.get('n_pairs'),
                'verdict': v.read_text().strip() if v.exists() else None, 'insights': len(r.get('insights') or [])}

    def report(self, key) -> dict | None:
        p = self.reports.get(key)
        if not p:
            return None
        f = p.with_name('feed.json')
        out = {'id': key, 'report': json.loads(p.read_text()), 'feed': json.loads(f.read_text()) if f.exists() else {}}
        v = p.with_name('verdict.txt')
        out['verdict'] = v.read_text().strip() if v.exists() else None
        return out

    def _load(self, key):
        p = self.reports[key]
        m = p.stat().st_mtime
        hit = self._parsed.get(key)
        if not hit or hit[0] != m:
            f = p.with_name('feed.json')
            v = p.with_name('verdict.txt')
            hit = (m, json.loads(p.read_text()), json.loads(f.read_text()) if f.exists() else {},
                   v.read_text().strip() if v.exists() else None)
            self._parsed[key] = hit
        return hit[1:]

    def judge_for(self, run_ids) -> list:
        """The judge's evidence about these runs, from every report: anchors, the insights they cite, pair
        records, and (for a report that pairs them) its G4 table and gap split."""
        ids = set(run_ids)
        out = []
        for key in sorted(self.reports):
            rep, feed, verdict = self._load(key)
            fsub = {iid: [a for a in anchors if a and a[0] in ids] for iid, anchors in feed.items()}
            fsub = {k: v for k, v in fsub.items() if v}
            pairs = [p for p in rep.get('pairs') or [] if p.get('low') in ids or p.get('high') in ids]
            if not fsub and not pairs:
                continue
            cited = set(fsub)
            ins = [i for i in rep.get('insights') or [] if i.get('id') in cited or (pairs and str(i.get('kind', '')).startswith(('tiers.', 'hint.')))]
            body = {'kind': rep.get('kind'), 'created': rep.get('created'), 'n_pairs': rep.get('n_pairs'), 'insights': ins, 'pairs': pairs}
            if pairs:
                body.update(g4=rep.get('g4'), split=rep.get('split'), decisions=rep.get('decisions'))
            out.append({'id': key, 'verdict': verdict, 'report': body, 'feed': fsub})
        return out

    def course(self, sha) -> dict | None:
        p = self.courses.get(sha)
        if not p:
            hits = [k for k in self.courses if k.startswith(sha)]
            p = self.courses[hits[0]] if len(hits) == 1 else None
        if not p:
            return None
        return reduce_course(json.loads(p.read_text()))

    def ghosts(self, run_id=None, study=None, seed=None, limit=8) -> dict | None:
        if run_id:
            r = self.runs.get(run_id)
            if not r:
                return None
            study, seed = r.header.get('study'), r.header.get('seed')
        rows = [x for x in self.runs.values() if x.header.get('study') == study and x.header.get('seed') == seed and x.id != run_id]
        rows.sort(key=lambda x: (-(x.header.get('iteration') or 0), x.header.get('score') is None, -(x.header.get('score') or 0), x.id))
        return {'study': study, 'seed': seed, 'runs': [x.brief() for x in rows[:limit]]}

    def fleet_rows(self, study, k=5, limit=120) -> list:
        rows = [x for x in self.runs.values() if x.header.get('study') == study]
        rows.sort(key=lambda x: (x.header.get('iteration') or 0, x.header.get('score') is None, -(x.header.get('score') or 0), x.id))
        return rows[:max(1, min(int(limit), 2000))]

    def fleet(self, study, **kw) -> dict:
        rows = self.fleet_rows(study, **kw)
        its = sorted({x.header.get('iteration') or 0 for x in rows})
        sha = hashlib.sha256(','.join((x.lod_sha or x.sha) for x in rows).encode()).hexdigest()
        return {'study': study, 'iterations': its, 'runs': [x.brief() for x in rows], 'pack_sha256': sha}

    def fleet_pack(self, study, **kw) -> bytes:
        return ctraj.fleet_pack([(x.lod or x.path).read_bytes() for x in self.fleet_rows(study, **kw)])
