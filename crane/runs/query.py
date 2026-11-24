"""`crane runs ls|query|show|diff|index`, and the --where language.

    outcome=TIMEOUT and tier=T1 and metric.time>120
    (arm=a or arm~'base*') and not dirty=true and seed in (1, 2, 3)
    commit=7597894c and metric.min_clearance_m<0.5 and created>2026-11-20

A condition is FIELD OP VALUE, FIELD in (V, ...), joined by and / or / not and parentheses. OP is one of
= != < <= > >= and ~ (a glob: * and ?; a value without either matches anywhere). FIELD is a column (run_id, path,
source, domain, tier, kind, seed, card, backend, dtype, commit, dirty, outcome, created, study, stage, arm, block,
run_sha256, sealed) or metric.NAME. A sealed (held-out, not done) run has no outcome and no metrics to match. VALUE is a bare word, a number or a quoted string; ``null`` (bare) with = / != tests
for a missing value. tier takes T0|T1|T2 or abstract|dynamic|unity; commit= and run_id= with a short hex value
match its prefix. The expression is parsed, never evaluated: fields come from a fixed list and every value is a
bound SQL parameter.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import re

from crane import output, select
from crane.runs.index import (COLUMNS, DONE, MAX_METRICS, Index, RunNotFound, default_path, find, flat_numbers,
                              held_out, seal)

FIELDS = {**{c: c for c in COLUMNS if c not in ('root', 'code_commit')}, 'commit': 'code_commit', 'id': 'run_id'}
TEXT_OPS = {'=': '=', '==': '=', '!=': '!=', '<': '<', '<=': '<=', '>': '>', '>=': '>=', '~': 'GLOB'}
EXAMPLE = 'outcome=TIMEOUT and tier=T1 and metric.time>120'
MAX_LEN, MAX_DEPTH = 4000, 40
HEX = re.compile(r'^[0-9a-f]{4,63}$')
TOKEN = re.compile(r"""\s*(?:(?P<op><=|>=|!=|==|=|<|>|~)|(?P<lp>\()|(?P<rp>\))|(?P<comma>,)
                       |'(?P<sq>[^']*)'|"(?P<dq>[^"]*)"|(?P<word>[A-Za-z0-9_.:/*?+@-]+))""", re.X)


def _got(value):
    return repr(value) if value else 'the end'


class QueryError(ValueError):
    """A --where expression that does not parse; the message names where and gives an example."""


def tokens(text):
    if len(text) > MAX_LEN:
        raise QueryError(f'--where is {len(text)} characters (at most {MAX_LEN})')
    out, at = [], 0
    while at < len(text):
        if text[at:].strip() == '':
            break
        m = TOKEN.match(text, at)
        if not m or m.end() == at:
            raise QueryError(f'unexpected {text[at:].strip()[:12]!r} at column {at + 1}')
        group = m.lastgroup
        out.append(('str' if group in ('sq', 'dq') else group, m.group(group), m.start(group) + 1))
        at = m.end()
    out.append(('end', '', len(text) + 1))
    return out


class _Parser:
    def __init__(self, text):
        self.toks, self.i, self.params, self.depth = tokens(text), 0, [], 0

    def peek(self):
        return self.toks[self.i]

    def take(self):
        tok = self.toks[self.i]
        self.i += 1
        return tok

    def keyword(self, word):
        kind, value, _ = self.peek()
        return kind == 'word' and value.lower() == word

    def expect(self, kind, what):
        tok = self.take()
        if tok[0] != kind:
            raise QueryError(f'expected {what} at column {tok[2]}, got {_got(tok[1])}')
        return tok

    def parse(self):
        sql = self.or_()
        tok = self.peek()
        if tok[0] != 'end':
            raise QueryError(f'unexpected {tok[1]!r} at column {tok[2]} (join conditions with and / or)')
        return sql, self.params

    def or_(self):
        parts = [self.and_()]
        while self.keyword('or'):
            self.take()
            parts.append(self.and_())
        return parts[0] if len(parts) == 1 else '(' + ' OR '.join(parts) + ')'

    def and_(self):
        parts = [self.not_()]
        while self.keyword('and'):
            self.take()
            parts.append(self.not_())
        return parts[0] if len(parts) == 1 else '(' + ' AND '.join(parts) + ')'

    def not_(self):
        if self.keyword('not'):
            self.take()
            return f'NOT {self.nest(self.not_)}'
        if self.peek()[0] == 'lp':
            self.take()
            sql = self.nest(self.or_)
            self.expect('rp', "')'")
            return sql
        return self.comparison()

    def nest(self, fn):
        self.depth += 1
        if self.depth > MAX_DEPTH:
            raise QueryError(f'--where nests deeper than {MAX_DEPTH}')
        try:
            return fn()
        finally:
            self.depth -= 1

    def value(self):
        kind, value, col = self.take()
        if kind not in ('word', 'str'):
            raise QueryError(f'expected a value at column {col}, got {_got(value)}')
        return value, kind == 'str'

    def comparison(self):
        kind, name, col = self.take()
        if kind != 'word':
            raise QueryError(f'expected a field at column {col}, got {_got(name)}')
        metric = name.startswith('metric.') and len(name) > 7
        if not metric and name not in FIELDS:
            raise QueryError(f'no field {name!r} at column {col}; fields: {", ".join(sorted(FIELDS))}, '
                             f'metric.NAME')
        if self.keyword('in'):
            self.take()
            self.expect('lp', "'(' after in")
            values = [self.value()]
            while self.peek()[0] == 'comma':
                self.take()
                values.append(self.value())
            self.expect('rp', "')'")
            converted = [self.convert(name, v, quoted, col) for v, quoted in values]
            marks = ', '.join('?' * len(converted))
            if metric:
                self.params += [name[7:], *converted]
                return f'EXISTS (SELECT 1 FROM metrics m WHERE m.path = r.path AND m.key = ? AND m.value IN ({marks}))'
            self.params += converted
            return f'r.{FIELDS[name]} IN ({marks})'
        kind, op, opcol = self.take()
        if kind != 'op':
            raise QueryError(f'expected an operator (= != < <= > >= ~ in) after {name!r} at column {opcol}')
        value, quoted = self.value()
        if value.lower() == 'null' and not quoted:
            if op not in ('=', '==', '!='):
                raise QueryError(f'null takes = or != (column {opcol})')
            if metric:
                self.params.append(name[7:])
                return ('NOT ' if op != '!=' else '') + 'EXISTS (SELECT 1 FROM metrics m WHERE m.path = r.path ' \
                                                        'AND m.key = ?)'
            return f'r.{FIELDS[name]} IS {"NOT " if op == "!=" else ""}NULL'
        if metric:
            if op == '~':
                raise QueryError(f'~ matches text; {name} is a number (column {opcol})')
            self.params += [name[7:], self.convert(name, value, quoted, col)]
            return f'EXISTS (SELECT 1 FROM metrics m WHERE m.path = r.path AND m.key = ? AND m.value {TEXT_OPS[op]} ?)'
        column, v = FIELDS[name], self.convert(name, value, quoted, col)
        if op == '~':
            text = str(v)
            self.params.append(text if any(c in text for c in '*?[') else f'*{text}*')
            return f'r.{column} GLOB ?'
        if op in ('=', '==') and column in ('code_commit', 'run_id', 'run_sha256') and HEX.match(str(v)):
            self.params.append(f'{v}*')
            return f'r.{column} GLOB ?'
        self.params.append(v)
        return f'r.{column} {TEXT_OPS[op]} ?'

    @staticmethod
    def convert(name, value, quoted, col):
        try:
            if name.startswith('metric.'):
                return float(value)
            if name == 'seed':
                return int(value)
            if name in ('dirty', 'sealed'):
                lowered = value.lower()
                if lowered not in ('true', 'false', '1', '0', 'yes', 'no'):
                    raise ValueError
                return int(lowered in ('true', '1', 'yes'))
            if name == 'tier' and not quoted:
                return select.parse_tier(value)
        except (ValueError, select.SelectError):
            want = {'seed': 'an integer', 'dirty': 'true or false', 'sealed': 'true or false',
                    'tier': 'T0, T1 or T2'}.get(name, 'a number')
            raise QueryError(f'{name} wants {want}, got {value!r} (column {col})') from None
        return value


def compile(text):
    """(SQL condition over ``runs r``, params) of a --where expression; QueryError when it does not parse."""
    if not str(text or '').strip():
        return '1', []
    return _Parser(str(text)).parse()


def query(where='', idx=None, limit=None, metrics=True) -> list:
    sql, params = compile(where)
    own = idx is None
    idx = idx or Index()
    try:
        return idx.rows(sql, params, limit, metrics=metrics)
    finally:
        if own:
            idx.close()


# ---- cluster runs (read-only; skipped cleanly offline) ---------------------------------------------------------

def _token():
    for env in ('CRANE_VIEWER_TOKEN', 'CRANE_TOKEN'):
        if os.environ.get(env):
            return os.environ[env].strip()
    path = Path.home() / '.config' / 'research-cluster' / 'crane-viewer.token'
    try:
        return path.read_text().strip()
    except OSError:
        return None


def cluster_rows(study, limit=1000, timeout=5.0):
    """(rows, note): the coordinator's runs of a study (GET /v1/runs/query) as index rows, or ([], why skipped)."""
    url = os.environ.get('CRANE_COORDINATOR')
    if not url:
        return [], 'cluster skipped: CRANE_COORDINATOR is not set'
    if not study:
        return [], 'cluster skipped: the coordinator selects by study; give --study NAME'
    import urllib.error
    import urllib.parse
    import urllib.request
    q = urllib.parse.urlencode({'study': study, 'limit': max(1, min(int(limit or 1000), 5000))})
    req = urllib.request.Request(f'{url.rstrip("/")}/v1/runs/query?{q}', method='GET')
    token = _token()
    if token:
        req.add_header('Authorization', f'Bearer {token}')
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            doc = json.loads(resp.read().decode())
    except (urllib.error.URLError, OSError, ValueError) as error:
        return [], f'cluster skipped: {url} unreachable ({getattr(error, "reason", error)})'
    rows = []
    for m in doc.get('runs') or ():
        run_id = str(m.get('run_id'))
        metrics = flat_numbers({'score': m.get('score'), 'iteration': m.get('iteration'),
                                'progress': m.get('progress')})
        study = m.get('study')
        block = m.get('block') or (study.get('block') if isinstance(study, dict) else None)
        row = {'path': f'cluster:{run_id}', 'source': 'cluster', 'run_id': run_id,
                     'domain': m.get('domain'), 'tier': m.get('tier'), 'kind': None, 'seed': m.get('seed'),
                     'card': None, 'backend': m.get('backend'), 'dtype': None, 'code_commit': m.get('commit'),
                     'dirty': 0, 'outcome': m.get('outcome'), 'created': m.get('created_at') or m.get('reported_at'),
                     'study': _name(m.get('study')), 'stage': m.get('stage') or (None if m.get('iteration') is None
                                                                          else str(m['iteration'])),
                     'arm': m.get('variant') or m.get('arm'), 'block': block, 'run_sha256': None,
                     'meta': m, 'metrics': metrics, 'mtime': 0, 'sealed': 0, 'truncated': 0}
        done = m.get('complete') is True or str(m.get('status') or m.get('state') or '').lower() in DONE
        if held_out(block) and not done:
            row = seal(row, {'done': m.get('progress'), 'total': None, 'phase': m.get('status') or m.get('state')})
        rows.append(row)
    return rows, f'cluster: {len(rows)} runs of {study} from {url}'


def _name(study):
    return study.get('name') if isinstance(study, dict) else study


def _with_cluster(rows, sql, params, limit, study, out):
    found, note = cluster_rows(study, limit)
    print(f'crane runs: {note}', file=out.stderr)
    if not found:
        return rows
    with Index(':memory:') as mem:
        for r in found:
            mem._put(r, 'cluster', '')
        rows = rows + mem.rows(sql, params, limit, metrics=True)
    rows.sort(key=lambda r: str(r.get('created') or ''), reverse=True)
    return rows[:limit] if limit else rows


# ---- the verbs ---------------------------------------------------------------------------------------------------

def _home(path):
    text, home = str(path), str(Path.home())
    return '~' + text[len(home):] if text.startswith(home + os.sep) else text


def _short_time(created):
    text = str(created or '')
    return text[:16].replace('T', ' ') if len(text) >= 16 else text


def _fmt(v):
    if v is None:
        return '-'
    if isinstance(v, float):
        return f'{v:.6g}'
    return str(v)


def _public(row):
    return {**{k: row.get(k) for k in COLUMNS if k != 'root'}, 'commit': row.get('code_commit'),
            'sealed': bool(row.get('sealed')), 'progress': row.get('progress') if row.get('sealed') else None,
            'metrics': {} if row.get('sealed') else row.get('metrics') or {}}


def _progress(p):
    p = p or {}
    if p.get('total') is not None:
        return f"{p.get('done')}/{p.get('total')} {p.get('phase') or ''}".strip()
    return 'no progress.json' if not p else f"{p.get('done') if p.get('done') is not None else '?'} " \
        f"{p.get('phase') or ''}".strip()


SEALED_NOTE = 'held-out: {n} run(s) sealed until done (progress only; no outcome or metrics)'


def _sealed_note(out, idx, sql, params, rows, where_text):
    """One stderr line when sealed held-out runs are shown, or could have matched a metric/outcome term."""
    shown = sum(1 for r in rows if r.get('sealed'))
    if shown:
        print('crane runs: ' + SEALED_NOTE.format(n=shown), file=out.stderr)
    elif re.search(r'\b(metric\.|outcome\b)', where_text or ''):
        n = idx.sealed()
        if n:
            print(f'crane runs: held-out: {n} sealed run(s) in the index never match metric or outcome terms '
                  'until done', file=out.stderr)


def _open(out, what):
    """The index, indexing the default runs root first when the index is new and empty."""
    idx = Index()
    if idx.count() == 0 and not idx.roots() and select.RUNS_ROOT.is_dir():
        print(f'crane runs: indexing {_home(select.RUNS_ROOT)} (first use)', file=out.stderr)
        idx.update([select.RUNS_ROOT])
    return idx


def _listing(args, where_parts, params, what):
    out = args.output
    limit = args.limit if args.limit and args.limit > 0 else None
    sql = ' AND '.join(where_parts) or '1'
    with _open(out, what) as idx:
        rows = idx.rows(sql, params, limit, metrics=True)
        total = idx.db.execute(f'SELECT count(*) FROM runs r WHERE {sql}', params).fetchone()[0]
        indexed, where = idx.count(), idx.path
        if getattr(args, 'cluster', False):
            rows = _with_cluster(rows, sql, params, limit, getattr(args, 'study', None), out)
        _sealed_note(out, idx, sql, params, rows, getattr(args, 'where', ''))
    shown_metrics = list(args.metric or [])
    if out.doc({'schema': 'crane.runs.list/1', 'index': str(where), 'n': len(rows), 'matched': total,
                'indexed': indexed, 'runs': [_public(r) for r in rows]}):
        return output.OK
    if not rows:
        out.say(f'no runs match (of {indexed} indexed)' if indexed else 'no runs indexed yet')
        out.next('crane runs index ROOT' if not indexed else 'crane runs ls')
        return output.OK
    head = ['run', 'domain', 'tier', 'seed', 'outcome', 'arm', 'created', *shown_metrics, 'path']
    body = [[(r['run_id'] or '')[:12], _fmt(r['domain']), _fmt(r['tier']), _fmt(r['seed']),
             'sealed' if r.get('sealed') else _fmt(r['outcome']), _fmt(r['arm']), _short_time(r['created']),
             *['sealed' if r.get('sealed') else _fmt((r['metrics'] or {}).get(m)) for m in shown_metrics],
             _home(r['path'])] for r in rows]
    out.table([head, *body])
    out.line(f'{len(rows)} shown, {total} matched, {indexed} indexed ({_home(where)})')
    out.next(f'crane runs show {(rows[0]["run_id"] or "")[:12]}')
    return output.OK


def ls_main(args):
    out = args.output
    parts, params = [], []
    for name, value in (('domain', args.domain), ('tier', args.tier), ('study', args.study)):
        if value:
            parts.append(f'r.{name} = ?')
            params.append(select.parse_tier(value) if name == 'tier' else value)
    try:
        sql, extra = compile(args.where)
    except QueryError as error:
        return out.usage('runs ls', str(error), f'crane runs ls --where "{EXAMPLE}"')
    if sql != '1':
        parts.append(sql)
        params += extra
    return _listing(args, parts, params, 'runs ls')


def query_main(args):
    out = args.output
    try:
        sql, params = compile(args.where)
    except QueryError as error:
        return out.usage('runs query', str(error), f'crane runs query --where "{EXAMPLE}"')
    return _listing(args, [sql], params, 'runs query')


def _files(path):
    p = Path(path)
    if not p.is_dir():
        return []
    out = []
    for child in sorted(p.iterdir()):
        if child.is_dir():
            out.append({'name': child.name + '/', 'size': sum(f.stat().st_size for f in child.rglob('*')
                                                             if f.is_file())})
        else:
            out.append({'name': child.name, 'size': child.stat().st_size})
    return out


def _size(n):
    for unit in ('B', 'KB', 'MB', 'GB'):
        if n < 1024 or unit == 'GB':
            return f'{n:.0f} {unit}' if unit == 'B' else f'{n:.1f} {unit}'
        n /= 1024


def _find(out, what, ref):
    try:
        return find(ref), None
    except RunNotFound as error:
        return None, out.usage(what, str(error), 'crane runs ls (or crane runs index ROOT)')


def _next_for(row):
    return f'crane data schema {_home(row["path"])}' if row.get('source') == 'manifest' else \
        f'crane view {_home(row["path"])}'


def show_main(args):
    out = args.output
    row, code = _find(out, 'runs show', args.run)
    if row is None:
        return code
    files = _files(row['path'])
    meta = row.get('meta') or {}
    ident = meta.get('identity') or {}
    if row.get('sealed'):
        print('crane runs: ' + SEALED_NOTE.format(n=1), file=out.stderr)
    if out.doc({'schema': 'crane.runs.show/1', 'run': _public(row), 'meta': meta, 'files': files,
                'next': _next_for(row)}):
        return output.OK
    fields = [('run', row['run_id']), ('path', _home(row['path'])), ('source', row['source']),
              ('domain', row['domain']), ('tier', row['tier']), ('kind', row['kind']), ('seed', row['seed']),
              ('card', row['card']), ('backend', row['backend']), ('dtype', row['dtype']),
              ('commit', (row['code_commit'] or '-')[:12] + (' (dirty)' if row['dirty'] else '')),
              ('outcome', row['outcome']), ('created', row['created'])]
    fields += [(k, row[k]) for k in ('study', 'stage', 'arm', 'block') if row.get(k)]
    fields += [(k, str(v)[:16]) for k, v in ident.items() if k.endswith('_sha256') and k != 'run_sha256']
    out.line('meta')
    out.table([(k, _fmt(v)) for k, v in fields])
    if row.get('sealed'):
        out.line(f"metrics: sealed (held-out, not done; progress {_progress(row.get('progress'))})")
    else:
        metrics = row.get('metrics') or {}
        out.line(f'metrics ({len(metrics)})')
        out.table([(k, _fmt(v)) for k, v in metrics.items()] or [('(none)',)])
    out.line(f'files ({len(files)})')
    out.table([(f['name'], _size(f['size'])) for f in files] or [('(none)',)])
    out.next(_next_for(row))
    return output.OK


# ---- diff ----------------------------------------------------------------------------------------------------------

IDENTITY_PARTS = (('scenario_sha256', 'scenario'), ('config_sha256', 'config'), ('adapter_sha256', 'adapter'),
                  ('seed', 'seed'), ('core', 'core'))
META_FIELDS = ('source', 'domain', 'tier', 'kind', 'seed', 'card', 'backend', 'dtype', 'code_commit', 'dirty',
               'outcome', 'study', 'stage', 'arm', 'block')


def _leaves(tree, prefix=''):
    if isinstance(tree, dict):
        out = {}
        for k, v in tree.items():
            out.update(_leaves(v, f'{prefix}.{k}' if prefix else str(k)))
        return out
    return {prefix: tree}


def _native(row):
    if row.get('source') != 'fastsim':
        return {}
    try:
        return json.loads((Path(row['path']) / 'run.json').read_text())
    except (OSError, ValueError):
        return {}


def _changed(a, b, limit=6):
    la, lb = _leaves(a or {}), _leaves(b or {})
    keys = [k for k in sorted(set(la) | set(lb)) if la.get(k) != lb.get(k)]
    shown = [f'{k} {json.dumps(la.get(k), default=str)} -> {json.dumps(lb.get(k), default=str)}' for k in keys[:limit]]
    return shown + ([f'... {len(keys) - limit} more'] if len(keys) > limit else [])


def explain_identity(a, b):
    """[{part, a, b, why}] for each identity part that differs between two rows (empty: the same run)."""
    ia, ib = (a.get('meta') or {}).get('identity') or {}, (b.get('meta') or {}).get('identity') or {}
    na, nb = _native(a), _native(b)
    out = []
    for key, part in IDENTITY_PARTS:
        va, vb = ia.get(key), ib.get(key)
        if va == vb:
            continue
        if part == 'config':
            why = _changed(na.get('config'), nb.get('config')) or ['config hashes differ (no config recorded)']
        elif part == 'adapter':
            why = _changed(na.get('adapter'), nb.get('adapter')) or ['adapter identities differ']
        elif part == 'scenario':
            sa, sb = na.get('scenario') or {}, nb.get('scenario') or {}
            why = [f"{sa.get('name') or sa.get('file') or '?'} -> {sb.get('name') or sb.get('file') or '?'}"]
        elif part == 'seed':
            why = [f'{va} -> {vb}']
        else:
            ca, cb = va or {}, vb or {}
            why = [f"{str(ca.get('commit'))[:12]}{' dirty' if ca.get('dirty') else ''} -> "
                   f"{str(cb.get('commit'))[:12]}{' dirty' if cb.get('dirty') else ''}"]
        out.append({'part': part, 'a': va, 'b': vb, 'why': why})
    return out


def diff(a, b):
    meta = [{'field': 'commit' if f == 'code_commit' else f, 'a': a.get(f), 'b': b.get(f)}
            for f in META_FIELDS if a.get(f) != b.get(f)]
    ma, mb = a.get('metrics') or {}, b.get('metrics') or {}
    metrics = [{'key': k, 'a': ma.get(k), 'b': mb.get(k),
                'delta': (mb[k] - ma[k]) if k in ma and k in mb else None}
               for k in sorted(set(ma) | set(mb)) if ma.get(k) != mb.get(k)]
    ra, rb = a.get('run_sha256'), b.get('run_sha256')
    return {'schema': 'crane.runs.diff/1', 'a': _public(a), 'b': _public(b),
            'same_run': bool(ra) and ra == rb, 'identity': explain_identity(a, b), 'meta': meta,
            'metrics': metrics, 'metrics_equal': sum(1 for k in set(ma) & set(mb) if ma[k] == mb[k])}


def diff_main(args):
    out = args.output
    a, code = _find(out, 'runs diff', args.a)
    if a is None:
        return code
    b, code = _find(out, 'runs diff', args.b)
    if b is None:
        return code
    d = diff(a, b)
    sealed = sum(1 for r in (a, b) if r.get('sealed'))
    if sealed:
        print('crane runs: ' + SEALED_NOTE.format(n=sealed) + '; their metrics are not compared', file=out.stderr)
    if out.doc(d):
        return output.OK
    out.line(f'A {a["run_id"][:12]}  {_home(a["path"])}')
    out.line(f'B {b["run_id"][:12]}  {_home(b["path"])}')
    if d['same_run']:
        out.say('identity: the same run (run_sha256 equal)')
    elif d['identity']:
        out.say('identity differs: ' + ', '.join(p['part'] for p in d['identity']))
        out.table([(p['part'], line) for p in d['identity'] for line in p['why']])
    else:
        out.say('identity: not recorded on both runs')
    if d['meta']:
        out.line('meta')
        out.table([(m['field'], _fmt(m['a']), '->', _fmt(m['b'])) for m in d['meta']])
    out.line(f'metrics: {len(d["metrics"])} differ, {d["metrics_equal"]} equal')
    if d['metrics']:
        out.table([('key', 'A', 'B', 'B-A'), *[(m['key'], _fmt(m['a']), _fmt(m['b']), _fmt(m['delta']))
                                               for m in d['metrics']]])
    out.next(f'crane view {_home(a["path"])} {_home(b["path"])}')
    return output.OK


def index_main(args):
    out = args.output
    with Index() as idx:
        given = [Path(r).expanduser() for r in (args.roots or [])]
        roots = given or [Path(r) for r in idx.roots()] or ([select.RUNS_ROOT] if select.RUNS_ROOT.is_dir() else [])
        if not roots:
            return out.usage('runs index', f'no roots given and {_home(select.RUNS_ROOT)} does not exist',
                             'crane runs index ROOT')
        missing = [r for r in roots if not r.is_dir()]
        if missing and given:
            return out.usage('runs index', f'{missing[0]} is not a directory', 'crane runs index ROOT')
        dropped = [idx.forget(r) for r in missing]       # remembered roots that are gone: their rows go too
        for r, n in zip(missing, dropped):
            print(f'crane runs: {_home(r)} is gone; dropped it and its {n} record(s)', file=out.stderr)
        stats = idx.update([r for r in roots if r.is_dir()], rebuild=args.rebuild)
        stats['removed'] += sum(dropped)
        total = idx.count()
    for path, n in stats['truncated']:
        print(f'crane runs: {_home(path)} has {n} metrics past the {MAX_METRICS} kept (by key)', file=out.stderr)
    if out.doc({'schema': 'crane.runs.index/1', 'index': str(default_path()), 'indexed': total, **stats}):
        return output.OK
    for root in stats['roots']:
        out.line(f'  {_home(root)}')
    out.say(f"{stats['seen']} records under {len(stats['roots'])} root(s): {stats['added']} added, "
            f"{stats['changed']} changed, {stats['unchanged']} unchanged, {stats['removed']} removed"
            f"{'; ' + str(stats['sealed']) + ' held-out sealed' if stats['sealed'] else ''}; "
            f"{total} indexed ({_home(default_path())})")
    out.next('crane runs ls')
    return output.OK
