"""crane cluster fetch STUDY: every finished job's rows into OUT/<study>/<stage>/<arm>/<cell>/, as a local run leaves
them, so `crane study report` reads both the same way.

    crane cluster fetch boat01                  # from the plan OUT/boat01/cluster/jobs.json
    crane cluster fetch boat01 --json           # crane.cluster.fetch/1

A job's outputs come from a rented box's pull (~/.cache/vast-out/<box>/<study>/<job_id>/, CRANE_VAST_OUT moves it),
else from the coordinator's artifacts (artifacts/rows/*, plus cell.json, progress.json, metrics.json, failure.json).
They land in <cell>/parts/<job_id>/, each file checked against the job's manifest.json sha256, and the cell's
manifest.json lists every part's files (paths relative to the cell dir). Fetching again skips what is in place.
"""

import hashlib
import json
import os
import re
import shutil
import urllib.parse
import urllib.request
from pathlib import Path

from crane import output
from crane.cluster import PLAN, studies_root
from crane.data.rows import MANIFEST, write_manifest

VAST_OUT = Path.home() / '.cache' / 'vast-out'
SIDE = ('cell.json', 'progress.json', 'metrics.json', 'failure.json')


def _sha(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def _json(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


class Client:
    """The coordinator's artifact routes (stdlib). Artifact reads take a viewer token: RC_VIEWER_TOKEN, else
    ~/.config/research-cluster/platform-viewer.token, else the project token."""

    def __init__(self, project, url=None):
        cfg = Path.home() / '.config' / 'research-cluster'
        self.url = (url or os.environ.get('RC_URL') or 'https://research.lunarz.dev').rstrip('/')
        self.project = project
        tok = os.environ.get('RC_VIEWER_TOKEN') or os.environ.get('RC_TOKEN')
        for name in ('platform-viewer', project, f'{project}-submitter'):
            if tok:
                break
            p = cfg / f'{name}.token'
            tok = p.read_text().strip() if p.is_file() else None
        if not tok:
            raise RuntimeError(f'no research token (RC_VIEWER_TOKEN or ~/.config/research-cluster/{project}.token)')
        self.token = tok

    def _get(self, path):
        req = urllib.request.Request(self.url + path, headers={'Authorization': f'Bearer {self.token}',
                                                               'User-Agent': 'crane-fetch/1'})
        with urllib.request.urlopen(req, timeout=120) as r:
            return r.read()

    def artifacts(self, job_id):
        q = urllib.parse.urlencode({'job': f'{self.project}/{job_id}'})
        return json.loads(self._get(f'/v1/artifacts?{q}')).get('artifacts', [])

    def get(self, key):
        return self._get('/v1/artifact/' + urllib.parse.quote(key, safe='/'))


def box_dirs(study, job_id, root=None):
    """Pulled box outputs of this job: <vast-out>/<box>/<study>/<job_id>/ (or <box>/<job_id>/)."""
    base = Path(root or os.environ.get('CRANE_VAST_OUT') or VAST_OUT).expanduser()
    if not base.is_dir():
        return []
    found = [d for d in sorted(base.glob(f'*/{study}/{job_id}')) if (d / MANIFEST).is_file()]
    return found or [d for d in sorted(base.glob(f'*/{job_id}')) if (d / MANIFEST).is_file()]


def _attempt(run_id):
    m = re.search(r'\.a(\d+)\.s\d+$', run_id or '')
    return int(m.group(1)) if m else 0


def _place(dest, man, opener):
    """Copies every manifest file into dest by opener(name) -> bytes/Path, checked by sha256. Returns rows placed."""
    dest.mkdir(parents=True, exist_ok=True)
    for f in man.get('files', []):
        target = dest / f['path']
        if target.is_file() and f.get('sha256') and _sha(target) == f['sha256']:
            continue
        src = opener(f['path'])
        tmp = target.with_name(f'.{target.name}.part')
        if isinstance(src, Path):
            shutil.copyfile(src, tmp)
        else:
            tmp.write_bytes(src)
        if f.get('sha256') and _sha(tmp) != f['sha256']:
            tmp.unlink()
            raise ValueError(f'{f["path"]}: sha256 differs from its manifest')
        os.replace(tmp, target)
    (dest / MANIFEST).write_text(json.dumps(man, indent=1, sort_keys=True) + '\n')
    return int(man.get('rows') or sum(int(f.get('rows') or 0) for f in man.get('files', [])))


def fetch_job(job, cell_root, client=None, vast_root=None):
    """One job into cell_root/parts/<job_id>/: (status, source, rows). status: fetched, present, missing, failed."""
    spec, jid = job.get('spec') or {}, job['job_id']
    dest = cell_root / 'parts' / jid
    have = _json(dest / MANIFEST)
    if have and have.get('files') and all((dest / f['path']).is_file() and f.get('sha256') and
                                          _sha(dest / f['path']) == f['sha256'] for f in have['files']):
        return 'present', 'here', int(have.get('rows') or sum(int(f.get('rows') or 0) for f in have['files']))
    for d in box_dirs(spec.get('study'), jid, vast_root):
        man = _json(d / MANIFEST)
        rows = _place(dest, man, lambda name, d=d: d / name)
        for side in SIDE:
            if (d / side).is_file():
                shutil.copyfile(d / side, dest / side)
        _side_from_cluster(dest, jid, client)
        return 'fetched', f'box {d.parent.parent.name if d.parent.name == spec.get("study") else d.parent.name}', rows
    if client is None:
        return 'missing', '', 0
    arts = client.artifacts(jid)
    if not arts:
        return 'missing', 'cluster', 0
    best = max({a.get('run_id') for a in arts}, key=_attempt)
    arts = {a['name']: a for a in arts if a.get('run_id') == best}
    if 'failure.json' in arts and not any(n.startswith('artifacts/rows/') and n != 'artifacts/rows/manifest.json'
                                          for n in arts):
        dest.mkdir(parents=True, exist_ok=True)
        for side in SIDE:
            if side in arts:
                (dest / side).write_bytes(client.get(arts[side]['key']))
        return 'failed', 'cluster', 0
    man_art = arts.get('artifacts/rows/manifest.json')
    if not man_art:
        return 'missing', 'cluster', 0
    man = json.loads(client.get(man_art['key']))
    names = {f['path'] for f in man.get('files', [])}
    if any(f'artifacts/rows/{n}' not in arts for n in names):
        return 'missing', 'cluster (rows on a box not pulled yet)', 0
    rows = _place(dest, man, lambda name: client.get(arts[f'artifacts/rows/{name}']['key']))
    for side in SIDE:
        if side in arts:
            (dest / side).write_bytes(client.get(arts[side]['key']))
    return 'fetched', 'cluster', rows


def _side_from_cluster(dest, jid, client):
    if client is None or all((dest / s).is_file() for s in SIDE[:3]):
        return
    try:
        arts = client.artifacts(jid)
    except Exception:   # noqa: BLE001 - the rows are what matter; the side files are a convenience
        return
    for a in arts:
        if a['name'] in SIDE and not (dest / a['name']).is_file():
            (dest / a['name']).write_bytes(client.get(a['key']))


def merge_cell(cell_root):
    """cell_root/manifest.json over the cell's own files and every part's (paths relative to the cell dir)."""
    old = _json(cell_root / MANIFEST) or {}
    files = {f['path']: f for f in old.get('files', []) if (cell_root / f['path']).is_file()}
    columns = dict(old.get('columns') or {})
    jobs = set(old.get('jobs') or [])
    parts = cell_root / 'parts'
    for d in sorted(parts.iterdir()) if parts.is_dir() else []:
        man = _json(d / MANIFEST)
        if not man:
            continue
        jobs.add(d.name)
        columns.update(man.get('columns') or {})
        for f in man.get('files', []):
            rel = f'parts/{d.name}/{f["path"]}'
            files[rel] = {**f, 'path': rel}
    keep = {k: v for k, v in old.items() if k not in ('schema', 'columns', 'files', 'rows', 'jobs')}
    return write_manifest(cell_root, columns, sorted(files.values(), key=lambda f: f['path']),
                          {**keep, 'jobs': sorted(jobs)})


def fetch(plan, root=None, client=None, vast_root=None):
    """Every plan job: [{job, cell, status, source, rows}]; each touched cell's manifest merged."""
    name = plan['study']
    base = studies_root(root) / name
    out, cells = [], set()
    for job in plan['jobs']:
        spec = job.get('spec') or {}
        cell_root = base / spec.get('cell_dir', f"{spec.get('stage') or 'main'}/{spec.get('arm')}/{spec.get('cell')}")
        try:
            status, source, rows = fetch_job(job, cell_root, client, vast_root)
        except (ValueError, OSError) as error:
            status, source, rows = 'failed', f'{type(error).__name__}: {error}'[:160], 0
        except Exception as error:   # noqa: BLE001 - an unreachable coordinator: the job stays missing
            status, source, rows = 'missing', f'cluster: {type(error).__name__}: {error}'[:160], 0
        if status == 'fetched':
            cells.add(cell_root)
        out.append({'job': job['job_id'], 'cell': spec.get('cell'), 'stage': spec.get('stage'), 'arm': spec.get('arm'),
                    'block': spec.get('block'), 'status': status, 'source': source, 'rows': rows,
                    'dir': str(cell_root)})
    for c in sorted(cells):
        merge_cell(c)
    return out


def main(args):
    from crane.cluster.submit import load_plan
    out = args.output
    plan, path = load_plan(args.study, args.out)
    if plan is None:
        return out.usage('cluster fetch', f'no plan at {path}', f'crane cluster submit SPEC (writes {PLAN})')
    client, why = None, None
    if not args.boxes_only:
        try:
            client = Client(plan['project'])
        except RuntimeError as error:
            why = str(error)
    rows = fetch(plan, args.out, client)
    counts = {}
    for r in rows:
        counts[r['status']] = counts.get(r['status'], 0) + 1
    doc = {'schema': 'crane.cluster.fetch/1', 'study': plan['study'],
           'out': str(studies_root(args.out) / plan['study']), 'counts': counts, 'jobs': rows, 'note': why}
    if out.doc(doc):
        return output.FAILED if counts.get('failed') else output.OK
    out.line(f"cluster fetch  {plan['study']}  -> {doc['out']}")
    shown = [r for r in rows if r['status'] != 'present']
    out.table([('status', 'job', 'stage/arm/block', 'rows', 'from')] +
              [(r['status'], r['job'], f"{r['stage'] or '-'}/{r['arm']}/{r['block'] or '-'}", str(r['rows']),
                r['source']) for r in shown[:40]])
    if len(shown) > 40:
        out.line(f'  ... {len(shown) - 40} more (--json has every one)')
    if why:
        out.line(f'note: cluster artifacts not read ({why}); box pulls only')
    out.say('cluster fetch: ' + ', '.join(f'{k} {n}' for k, n in sorted(counts.items())))
    if counts.get('missing'):
        out.next(f"crane cluster status {plan['study']}", f"crane cluster fetch {plan['study']} (again once they end)")
    else:
        out.next(f"crane study report {plan.get('source') or plan['study']}")
    return output.FAILED if counts.get('failed') else output.OK
