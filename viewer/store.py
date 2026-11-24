"""The local run store behind ``python -m viewer serve`` (``crane board``): runs found under a set of roots, served
in the coordinator's feed shapes (research-cluster ``coordinator/src/replay.ts``) so one page reads both.

A run is a crane.run/1 record (a directory with ``crane.run.json`` or ``run.json`` of that schema) or a bare
``.ctraj``. A record is served as one crane-traj/1 file: its trajectory (an event-kind run gets an empty one) with
the meta's identity, tier, kind, domain and frame in the header, its constraint layers (``layers``; from the meta
or ``layers.json``), the domain's vehicle outline, and its ``events.jsonl`` merged into the events, each with a
``lane`` (``log`` lines included). Course truths, judge reports and the bare files' headers are used as they are.
Run ids are the meta's ``run_id``, else the header's, else the file's path under its root."""

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
    """One served run: a bare ``.ctraj`` (``path``, its ``.lod.ctraj`` beside it) or a crane.run/1 record whose
    served bytes are composed on first use (``compose``)."""
    __slots__ = ('id', 'path', 'lod', 'header', 'compose', '_data', '_lod', '_sha', '_lsha')

    def __init__(self, rid, path, lod, header, compose=None):
        self.id, self.path, self.lod, self.header, self.compose = rid, path, lod, header, compose
        self._data = self._lod = self._sha = self._lsha = None

    def data(self) -> bytes:
        if self.compose is None:
            return self.path.read_bytes()
        if self._data is None:
            self._data = self.compose()
        return self._data

    def lod_data(self) -> bytes | None:
        if self.compose is None:
            return self.lod.read_bytes() if self.lod else None
        if self._lod is None:
            self._lod = ctraj.lod(self.data())
        return self._lod

    @property
    def has_lod(self) -> bool:
        return self.compose is not None or self.lod is not None

    @property
    def sha(self):
        if self._sha is None:
            self._sha = hashlib.sha256(self.data()).hexdigest()
        return self._sha

    @property
    def lod_sha(self):
        if self.has_lod and self._lsha is None:
            self._lsha = hashlib.sha256(self.lod_data()).hexdigest()
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
            claimed = set()
            for p in files:          # crane.run/1 records first: their trajectory is served through them
                if p.name in (SIDECAR, META) and p.is_file() and p.parent not in claimed:
                    rec = _record(p.parent)
                    if rec is None:
                        continue
                    claimed.add(p.parent)
                    traj = rec.file('traj')
                    if traj is not None and traj.is_file():
                        claimed.add(traj)
                    h, compose = record_header(rec, root), _composer(rec, root)
                    runs[h['run_id']] = Run(h['run_id'], traj if traj and traj.is_file() else None, None, h, compose)
                    sha = (h.get('course') or {}).get('truth_sha256')
                    sc = _scenario_file(rec)
                    if sha and sc:
                        courses.setdefault(sha, sc)
            for p in files:
                if not p.is_file() or p in claimed:
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
        return ctraj.fleet_pack([x.lod_data() or x.data() for x in self.fleet_rows(study, **kw)])


# ---- crane.run/1 records ---------------------------------------------------------------------------------------

SIDECAR, META, RUN_SCHEMA = 'crane.run.json', 'run.json', 'crane.run/1'
OUTCOMES = {'SUCCEEDED': 'finished', 'TIMEOUT': 'timeout', 'FAILED': 'failed', 'ERROR': 'failed'}


def _record(d: Path):
    """The crane.run/1 record of a directory, else None (another schema's run.json is not one)."""
    from crane import record
    try:
        return record.read_run(d)
    except (record.RecordError, ValueError, OSError):
        return None


def _scenario_file(rec):
    for name in ((rec.meta.get('scenario') or {}).get('file'), 'course_truth.json'):
        if name and (rec.path / name).is_file():
            return rec.path / name
    return None


def record_header(rec, root: Path) -> dict:
    """The header a record is served with (no trajectory is decoded; ``_composer`` adds the box and ticks)."""
    from crane.view import spec as vspec
    m = rec.meta
    traj = rec.file('traj')
    h = dict(ctraj.read_header(traj)) if traj is not None and traj.is_file() else {}
    prov = m.get('provenance') or {}
    try:
        rel = rec.path.relative_to(root).parts
    except ValueError:
        rel = ()
    h.update(run_id=m['run_id'], tier=m.get('tier'), kind=m.get('kind'), domain=m.get('domain') or None,
             crane_run={k: m.get(k) for k in ('schema', 'frame', 'clock', 'timing', 'card', 'backend') if m.get(k)})
    h.setdefault('seed', m.get('seed'))
    # OUT/<variant>/s<seed> (a batch) is study OUT; ROOT/<study>/<run> is that study
    batch = len(rel) > 1 and rel[0] == prov.get('variant')
    h.setdefault('study', prov.get('study') or (rel[0] if len(rel) > 1 and not batch else root.name))
    h.setdefault('arm', prov.get('variant') or m.get('tier'))
    if 'outcome' not in h and m.get('outcome'):
        h['outcome'] = OUTCOMES.get(str(m['outcome']).upper(), 'unknown')
    if rec.metrics and isinstance(rec.metrics.get('score'), (int, float)):
        h.setdefault('score', rec.metrics['score'])
    try:
        from crane.view import layers as vlayers
        lay = vlayers.of_run(rec.path, m)
    except ValueError:
        lay = []
    if lay:
        h['layers'] = lay
    spec = vspec.view_spec(m.get('domain') or None)
    h['outline'] = spec['outline']
    h['lanes'] = spec['lanes']
    return {k: v for k, v in h.items() if v is not None}


def _composer(rec, root):
    def compose() -> bytes:
        from crane.view import spec as vspec
        h = record_header(rec, root)
        t = rec.traj
        lanes = h.get('lanes') or {}
        seen, events = set(), []
        for e in list(t.events if t is not None else []) + rec.events:
            if not isinstance(e, dict) or e.get('t') is None or not e.get('kind'):
                continue
            key = (round(float(e['t']), 3), str(e['kind']), str(e.get('text', e.get('id', ''))))
            if key not in seen:
                seen.add(key)
                events.append({**e, 'lane': vspec.lane_of(e, lanes)})
        events.sort(key=lambda e: e['t'])
        keep = {k: v for k, v in h.items() if k not in ('ticks', 'duration_s', 'columns', 'box', 'schema')}
        if t is None:
            return ctraj.encode(keep, {}, events, hz=1.0, t0_s=0.0)
        return ctraj.encode(keep, t.columns, events, hz=t.hz, t0_s=float(t.header.get('t0_s', 0.0)), specs=t.specs)
    return compose
