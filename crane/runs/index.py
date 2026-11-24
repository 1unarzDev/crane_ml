"""The local run index: one SQLite file (``~/.cache/crane/index.sqlite``; ``CRANE_INDEX`` moves it).

    idx = Index()                                 # opens (and creates) the file
    idx.update(['~/.cache/crane/runs', '/tmp/study-out'])   # -> {seen, added, changed, removed, roots}
    idx.rows('r.tier = ?', ['T1'])                # row dicts, newest first

A row per record found under a root:
- a crane.run/1 directory (``crane.run.json``, or ``run.json`` whose schema is crane.run/1);
- a fastsim run directory (``run.json`` schema crane.fastsim.run/1), read through ``fastsim.run.crane_run_meta``
  without writing anything into it;
- a study or dataset output (``manifest.json`` schema crane.data.manifest/1), with ``metrics.json`` beside it.

Columns: path, root, source, run_id, domain, tier, kind, seed, card, backend, dtype, code_commit, dirty, outcome,
created, study, stage, arm, block, run_sha256, plus the meta as JSON. Each record's numbers go to ``metrics``
(path, key, value), flattened to dotted keys (at most 1000 per record, the compute contract's bound).

Updates are incremental: a record whose meta files keep their mtime is skipped, one whose bytes keep their sha256
only gets the new mtime; records gone from a root are dropped. Run dirs are not descended into; hidden dirs
(``.s3.partial-*``) and ``*.stale-N`` dirs are skipped.
"""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sqlite3

SCHEMA = 'crane.runs.index/2'
FASTSIM = 'crane.fastsim.run/1'
RUN = 'crane.run/1'
MANIFEST = 'crane.data.manifest/1'
META_FILES = ('crane.run.json', 'run.json', 'metrics.json', 'manifest.json')
MAX_METRICS = 1000
STALE = re.compile(r'\.stale-\d+$')

COLUMNS = ('path', 'root', 'source', 'run_id', 'domain', 'tier', 'kind', 'seed', 'card', 'backend', 'dtype',
           'code_commit', 'dirty', 'outcome', 'created', 'study', 'stage', 'arm', 'block', 'run_sha256')

DDL = f"""
CREATE TABLE IF NOT EXISTS info (key TEXT PRIMARY KEY, value TEXT);
CREATE TABLE IF NOT EXISTS roots (root TEXT PRIMARY KEY, indexed TEXT);
CREATE TABLE IF NOT EXISTS runs (
  path TEXT PRIMARY KEY, root TEXT, source TEXT, run_id TEXT, domain TEXT, tier TEXT, kind TEXT, seed INTEGER,
  card TEXT, backend TEXT, dtype TEXT, code_commit TEXT, dirty INTEGER, outcome TEXT, created TEXT, study TEXT,
  stage TEXT, arm TEXT, block TEXT, run_sha256 TEXT, mtime INTEGER, sha TEXT, meta TEXT);
CREATE INDEX IF NOT EXISTS runs_run_id ON runs (run_id);
CREATE INDEX IF NOT EXISTS runs_created ON runs (created);
CREATE TABLE IF NOT EXISTS metrics (path TEXT, key TEXT, value REAL, PRIMARY KEY (path, key)) WITHOUT ROWID;
CREATE INDEX IF NOT EXISTS metrics_key ON metrics (key, value);
"""


class RunNotFound(LookupError):
    """A run reference that finds no run, or more than one."""


def default_path() -> Path:
    return Path(os.environ.get('CRANE_INDEX') or Path.home() / '.cache' / 'crane' / 'index.sqlite').expanduser()


# ---- reading one directory -----------------------------------------------------------------------------------

def _json(path: Path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def flat_numbers(tree, prefix='', out=None) -> dict:
    """Finite numeric leaves of nested dicts as {dotted.key: float} (booleans left out)."""
    out = {} if out is None else out
    if isinstance(tree, dict):
        for key, value in tree.items():
            flat_numbers(value, f'{prefix}.{key}' if prefix else str(key), out)
    elif isinstance(tree, (int, float)) and not isinstance(tree, bool) and math.isfinite(tree) and prefix:
        out[prefix] = float(tree)
    return out


def _cap(metrics: dict) -> dict:
    return dict(sorted(metrics.items())[:MAX_METRICS])


def _first(*values):
    return next((v for v in values if v not in (None, '')), None)


def _study_fields(*sources) -> dict:
    """study/stage/arm/block from the first source naming each (a source may hold a ``study`` mapping)."""
    out = {}
    for key in ('study', 'stage', 'arm', 'block'):
        for s in sources:
            if not isinstance(s, dict):
                continue
            v = s.get(key)
            if isinstance(v, dict):            # {'study': {'name', 'stage', 'arm', 'block'}}
                v = v.get('name') if key == 'study' else None
            if v in (None, '') and isinstance(s.get('study'), dict):
                v = s['study'].get(key)
            if v not in (None, ''):
                out[key] = str(v)
                break
    return out


def _git(meta) -> tuple:
    """(commit, dirty) of the code that made a run: provenance.git.core, else identity.core, else top level."""
    prov = meta.get('provenance') or {}
    git = (prov.get('git') or {}) if isinstance(prov.get('git'), dict) else {}
    core = git.get('core') or (meta.get('identity') or {}).get('core') or {}
    commit = _first(core.get('commit') if isinstance(core, dict) else None, prov.get('commit'),
                    meta.get('code_commit'), meta.get('commit'))
    dirty = any(bool((part or {}).get('dirty')) for part in (core, git.get('adapter')) if isinstance(part, dict))
    dirty = dirty or bool(prov.get('dirty') or meta.get('dirty'))
    return commit, dirty


def _created(meta, mtime_ns) -> str:
    when = _first(meta.get('created'), (meta.get('provenance') or {}).get('created'))
    if when:
        return str(when)
    return datetime.fromtimestamp(mtime_ns / 1e9, timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')


def _card(card):
    if isinstance(card, dict):
        return _first(card.get('ref'), card.get('sha'), card.get('name'))
    return card


def fastsim_metrics(native: dict) -> dict:
    """The numbers of a fastsim run.json: ``time`` (= sim_s), sim_s, time_limit_s, min_clearance_m, contacts, idle_s."""
    m = {'time': native.get('sim_s'), 'sim_s': native.get('sim_s'), 'time_limit_s': native.get('time_limit_s'),
         'min_clearance_m': native.get('min_clearance_m'), 'contacts': len(native.get('contacts') or ()),
         'idle_s': (native.get('idle') or {}).get('s')}
    return flat_numbers({k: v for k, v in m.items() if v is not None})


def read_dir(path) -> dict | None:
    """The index row of one directory (with ``meta`` and ``metrics``), or None when it holds no record."""
    d = Path(path).expanduser().resolve()
    present = [d / n for n in META_FILES if (d / n).is_file()]
    if not present:
        return None
    mtime = max(p.stat().st_mtime_ns for p in present)
    native, sidecar = _json(d / 'run.json'), _json(d / 'crane.run.json')
    metrics_file = _json(d / 'metrics.json')
    row = None
    if isinstance(native, dict) and native.get('schema') == FASTSIM:
        from fastsim.run import crane_run_meta
        from crane import record
        try:
            meta = record.validate(crane_run_meta(native))
        except (KeyError, TypeError, record.RecordError):
            return None
        metrics = {**fastsim_metrics(native), **flat_numbers(metrics_file)}
        row = _run_row(meta, 'fastsim', metrics, native)
    else:
        meta = next((m for m in (sidecar, native) if isinstance(m, dict) and m.get('schema') == RUN), None)
        if meta is not None:
            from crane import record
            try:
                rec = record.RunRecord(d, record.validate(meta))
                metrics = flat_numbers(rec.metrics if isinstance(rec.metrics, dict) else {})
            except (record.RecordError, ValueError, OSError):
                return None
            row = _run_row(meta, RUN, metrics, None)
        else:
            manifest = _json(d / 'manifest.json')
            if isinstance(manifest, dict) and manifest.get('schema') == MANIFEST:
                row = _manifest_row(d, manifest, metrics_file)
    if row is None:
        return None
    row['metrics'] = _cap(row['metrics'])
    row.update(path=str(d), mtime=mtime, created=row.get('created') or _created(row['meta'], mtime))
    return row


def _run_row(meta, source, metrics, native) -> dict:
    commit, dirty = _git(meta)
    prov = meta.get('provenance') or {}
    params = meta.get('params') or {}
    study = _study_fields(meta, prov, params)
    if 'arm' not in study and prov.get('variant'):
        study['arm'] = str(prov['variant'])    # a fastsim batch's variant is its arm
    return {'source': source, 'run_id': meta['run_id'], 'domain': meta.get('domain') or None,
            'tier': meta.get('tier'), 'kind': meta.get('kind'), 'seed': meta.get('seed'),
            'card': _card(meta.get('card')), 'backend': meta.get('backend'), 'dtype': meta.get('dtype'),
            'code_commit': commit, 'dirty': int(dirty), 'outcome': _first(meta.get('outcome'),
                                                                        (native or {}).get('outcome')),
            'run_sha256': (meta.get('identity') or {}).get('run_sha256'), **study,
            'meta': meta, 'metrics': metrics}


def _manifest_row(d: Path, manifest, metrics_file) -> dict:
    commit, dirty = _git(manifest)
    metrics = {**flat_numbers(manifest.get('metrics')), **flat_numbers(metrics_file)}
    seeds = manifest.get('seeds')
    seed = manifest.get('seed', seeds[0] if isinstance(seeds, list) and len(seeds) == 1 else None)
    run_id = _first(manifest.get('run_id'), manifest.get('job_id'), manifest.get('id'),
                    hashlib.sha256(str(d).encode()).hexdigest()[:16])
    return {'source': 'manifest', 'run_id': str(run_id), 'domain': manifest.get('domain'),
            'tier': manifest.get('tier'), 'kind': manifest.get('kind') or 'dataset', 'seed': seed,
            'card': _card(manifest.get('card')), 'backend': manifest.get('backend'), 'dtype': manifest.get('dtype'),
            'code_commit': commit, 'dirty': int(dirty), 'outcome': manifest.get('outcome'),
            'run_sha256': None, **_study_fields(manifest), 'meta': manifest, 'metrics': metrics}


def _sha(path: Path) -> str:
    h = hashlib.sha256()
    for name in META_FILES:
        p = path / name
        if p.is_file():
            h.update(name.encode() + b'\0' + p.read_bytes() + b'\0')
    return h.hexdigest()


def walk(root):
    """Directories under ``root`` that may hold a record (a run dir is not descended into)."""
    root = Path(root).expanduser().resolve()
    for here, dirs, files in os.walk(root):
        dirs[:] = sorted(x for x in dirs if not x.startswith('.') and not STALE.search(x))
        names = set(files)
        if names & set(META_FILES):
            yield Path(here)
            if names & {'run.json', 'crane.run.json'}:
                dirs[:] = []                    # a run dir's subdirs are its own files


# ---- the index -----------------------------------------------------------------------------------------------

class Index:
    def __init__(self, path=None):
        self.path = Path(path).expanduser() if path else default_path()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(self.path)
        self.db.row_factory = sqlite3.Row
        have = self._version()
        if have not in (None, SCHEMA):        # an older layout: start over (the index is derived data)
            self.db.executescript('DROP TABLE IF EXISTS runs; DROP TABLE IF EXISTS metrics; '
                                  'DROP TABLE IF EXISTS roots; DROP TABLE IF EXISTS info;')
        self.db.executescript(DDL)
        self.db.execute('INSERT OR REPLACE INTO info VALUES (?, ?)', ('schema', SCHEMA))
        self.db.commit()

    def _version(self):
        try:
            got = self.db.execute("SELECT value FROM info WHERE key = 'schema'").fetchone()
        except sqlite3.OperationalError:
            return None
        return got[0] if got else None

    def close(self):
        self.db.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def roots(self) -> list:
        return [r[0] for r in self.db.execute('SELECT root FROM roots ORDER BY root')]

    def count(self) -> int:
        return self.db.execute('SELECT count(*) FROM runs').fetchone()[0]

    def rebuild(self):
        self.db.executescript('DELETE FROM runs; DELETE FROM metrics;')
        self.db.commit()

    def update(self, roots, rebuild=False) -> dict:
        """Index every record under ``roots`` (incremental unless ``rebuild``); returns the counts."""
        if rebuild:
            self.rebuild()
        stats = {'seen': 0, 'added': 0, 'changed': 0, 'unchanged': 0, 'removed': 0, 'roots': []}
        for root in roots:
            root = Path(root).expanduser().resolve()
            if not root.is_dir():
                raise FileNotFoundError(str(root))
            stats['roots'].append(str(root))
            known = {r['path']: (r['mtime'], r['sha']) for r in self.db.execute(
                'SELECT path, mtime, sha FROM runs WHERE path = ? OR substr(path, 1, ?) = ?',
                (str(root), len(str(root)) + 1, str(root) + os.sep))}
            seen = set()
            for d in walk(root):
                key = str(d)
                present = [d / n for n in META_FILES if (d / n).is_file()]
                mtime = max(p.stat().st_mtime_ns for p in present)
                if key in known and known[key][0] == mtime:
                    seen.add(key)
                    stats['unchanged'] += 1
                    continue
                sha = _sha(d)
                if key in known and known[key][1] == sha:
                    self.db.execute('UPDATE runs SET mtime = ? WHERE path = ?', (mtime, key))
                    seen.add(key)
                    stats['unchanged'] += 1
                    continue
                row = read_dir(d)
                if row is None:
                    continue
                seen.add(key)
                stats['changed' if key in known else 'added'] += 1
                self._put(row, str(root), sha)
            gone = set(known) - seen
            for key in gone:
                self.db.execute('DELETE FROM runs WHERE path = ?', (key,))
                self.db.execute('DELETE FROM metrics WHERE path = ?', (key,))
            stats['removed'] += len(gone)
            stats['seen'] += len(seen)
            self.db.execute('INSERT OR REPLACE INTO roots VALUES (?, ?)',
                            (str(root), datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')))
            self.db.commit()
        return stats

    def _put(self, row, root, sha):
        values = {**{c: row.get(c) for c in COLUMNS}, 'root': root}
        if values['seed'] is not None:
            try:
                values['seed'] = int(values['seed'])
            except (TypeError, ValueError):
                values['seed'] = None
        self.db.execute(f'INSERT OR REPLACE INTO runs ({", ".join(COLUMNS)}, mtime, sha, meta) VALUES '
                        f'({", ".join("?" * len(COLUMNS))}, ?, ?, ?)',
                        [values[c] for c in COLUMNS] + [row['mtime'], sha, json.dumps(row['meta'], default=str)])
        self.db.execute('DELETE FROM metrics WHERE path = ?', (row['path'],))
        self.db.executemany('INSERT INTO metrics VALUES (?, ?, ?)',
                            [(row['path'], k, v) for k, v in row['metrics'].items()])

    def rows(self, where='1', params=(), limit=None, metrics=False) -> list:
        """Row dicts matching a compiled condition over ``runs r`` (crane.runs.query.compile), newest first."""
        sql = f'SELECT * FROM runs r WHERE {where} ORDER BY r.created DESC, r.path'
        if limit:
            sql += f' LIMIT {int(limit)}'
        out = [self._row(r) for r in self.db.execute(sql, list(params))]
        if metrics:
            for r in out:
                r['metrics'] = self.metrics(r['path'])
        return out

    @staticmethod
    def _row(r) -> dict:
        d = {k: r[k] for k in COLUMNS}
        d['dirty'] = bool(d['dirty'])
        d['meta'] = json.loads(r['meta']) if r['meta'] else {}
        return d

    def metrics(self, path) -> dict:
        return {k: v for k, v in self.db.execute('SELECT key, value FROM metrics WHERE path = ? ORDER BY key',
                                                 (path,))}


def find(ref, idx=None) -> dict:
    """One run: a directory (read directly, index or not), else an indexed run_id or path prefix.
    RunNotFound names the problem when none or several match."""
    p = Path(str(ref)).expanduser()
    if p.is_dir():
        row = read_dir(p)
        if row is None:
            raise RunNotFound(f'{p} holds no run record (crane.run.json, run.json or manifest.json)')
        return row
    own = idx is None
    idx = idx or Index()
    try:
        hits = idx.rows('r.run_id = ? OR r.path = ?', [str(ref), str(ref)])
        if not hits:
            like = str(ref).replace('\\', '\\\\').replace('%', '\\%').replace('_', '\\_') + '%'
            hits = idx.rows("r.run_id LIKE ? ESCAPE '\\'", [like])
        if not hits:
            raise RunNotFound(f'no run {ref!r} in the index ({idx.path})')
        if len({h['path'] for h in hits}) > 1:
            if len({h['run_id'] for h in hits}) == 1:     # one run in several places: the newest copy
                pass
            else:
                raise RunNotFound(f'{ref!r} matches {len(hits)} runs; give more of the run_id or the path')
        row = hits[0]
        row['metrics'] = idx.metrics(row['path'])
        return row
    finally:
        if own:
            idx.close()
