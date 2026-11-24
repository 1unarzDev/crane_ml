"""crane monitor: local progress.json files and the cluster's view of a study's jobs, merged into one table.

    crane monitor boat01                 # OUT/boat01 under ~/.cache/crane/studies (CRANE_STUDIES), or a dir
    crane monitor ~/runs/x --follow      # refresh every 30 s (5..300), at most --timeout s
    crane monitor boat01 --json          # one crane.monitor/1 document (one per refresh with --follow)

Local: every progress.json under OUT, with the cell.json (study, stage, arm, block, role, host, job), failure.json
(reason, traceback) and metrics.json beside it. Cluster: when OUT/<study>/cluster/jobs.json exists, the project's job
list (research token) and the public feed (/v1/feed: progress, worker, eta, error). A job seen both ways counts once
(the local copy wins once it is terminal). Groups are (study, stage, arm, block). Interim metric means are shown for
dev blocks only (crane.study.guard.shows_means); every other block (held-out above all) shows progress only, and
a held-out block that has started is flagged touched_held_out. A failure's traceback line shows for dev and check
blocks only (guard.shows_outcomes).

A running job's progress.json is at most 60 s old (crane-batch heartbeats it), so one older than STALE_S (or the
job's stall_s when cell.json gives one) means the runner is gone: the job shows as stalled.
"""

import json
import os
import time
import urllib.request
from pathlib import Path

from crane import output, progress
from crane.cluster import PLAN, studies_root
from crane.study import guard

STALE_S = 180.0                # progress.json is rewritten at most 60 s apart; three missed writes is a stall
FOLLOW_MIN_S, FOLLOW_MAX_S = 5.0, 300.0
METRICS_SHOWN = 4
FEED_TIMEOUT_S = 10.0
TERMINAL = ('done', 'failed', 'cancelled', 'dead', 'closed')


def _json(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


def resolve(target=None, root=None):
    """(the dir to scan, the study name or None): an existing dir as is, else OUT/<target>, else OUT."""
    if target and Path(target).expanduser().is_dir():
        d = Path(target).expanduser().resolve()
        return d, d.name if (d / PLAN).is_file() or (d / 'cluster').is_dir() else None
    base = studies_root(root)
    return (base / target, target) if target else (base, None)


# ---- local ----------------------------------------------------------------------------------------------------------

def local_entries(where):
    """One entry per progress.json under `where`."""
    out = []
    where = Path(where)
    if not where.is_dir():
        return out
    for path in sorted(where.rglob(progress.FILE)):
        d = path.parent
        state = progress.read(d)
        if state is None:
            continue
        meta = _json(d / 'cell.json') or _json(d / 'manifest.json') or {}
        rel = d.relative_to(where).parts
        guess = dict(zip(('stage', 'arm', 'cell'), rel[-3:])) if len(rel) >= 3 else {}
        fail = _json(d / 'failure.json')
        phase = state.get('phase')
        stale = max(STALE_S, float(meta.get('stall_s') or 0))
        status = 'done' if phase == 'done' else 'failed' if fail or phase == 'failed' else (
            'cancelled' if phase == 'cancelled' else 'yielded' if phase == 'yielded' else
            'stalled' if (state.get('age_s') or 0) > stale else 'running')
        if not (d / 'cell.json').is_file() and isinstance(meta.get('units'), dict):
            out += _study_entries(d, meta, state, status, fail)          # a `crane study run` output
            continue
        out.append({
            'source': 'local', 'key': meta.get('job_id') or str(d), 'dir': str(d),
            'study': meta.get('study') or (where.name if len(rel) <= 3 else rel[0]),
            'stage': meta.get('stage', guess.get('stage')),
            'arm': meta.get('arm', guess.get('arm', state.get('label'))),
            'block': meta.get('block'), 'role': meta.get('role'), 'cell': meta.get('cell', guess.get('cell')),
            'host': meta.get('host'), 'status': status, 'done': int(state.get('done') or 0),
            'total': int(state.get('total') or 0), 'failed': int(state.get('failed') or 0),
            'rate_per_s': float(state.get('rate_per_s') or 0.0) if status == 'running' else 0.0,
            'eta_s': state.get('eta_s') if status == 'running' else None, 'age_s': state.get('age_s'),
            'failure': _failure(fail, status, state, meta.get('role')), 'metrics': _metrics(d, state),
            'touched_held_out': bool(meta.get('touched_held_out')) or _held(meta.get('role'))})
    return out


def _study_entries(d, manifest, state, status, fail):
    """A local study output (one progress.json for the run): one entry per (stage, block) from the manifest's units,
    with metrics.json's per-arm means (mean.STAGE.ARM.METRIC) as ARM.METRIC on blocks whose role shows means."""
    means = {k: v for k, v in (_json(Path(d) / 'metrics.json') or {}).items() if k.startswith('mean.')}
    out = []
    for stage, blocks in manifest['units'].items():
        mine = {k[len(f'mean.{stage}.'):] + '.mean': v for k, v in means.items() if k.startswith(f'mean.{stage}.')}
        for block, u in blocks.items():
            done, total = int(u.get('done') or 0), int(u.get('total') or 0)
            st = 'done' if total and done >= total else status if status != 'done' else 'queued'
            out.append({'source': 'local', 'key': f'{d}:{stage}:{block}', 'dir': str(d), 'study': manifest.get('study'),
                        'stage': stage, 'arm': '*', 'block': block, 'role': u.get('role'), 'cell': None,
                        'host': None, 'status': st, 'done': done, 'total': total,
                        'failed': int(state.get('failed') or 0) if st == status else 0,
                        'rate_per_s': float(state.get('rate_per_s') or 0.0) if st == 'running' else 0.0,
                        'eta_s': state.get('eta_s') if st == 'running' else None, 'age_s': state.get('age_s'),
                        'failure': _failure(fail, st, state, u.get('role')) if st == status else None,
                        'metrics': mine if guard.shows_means(u.get('role')) else {},
                        'touched_held_out': _held(u.get('role')) and done > 0})
    return out


def _held(role):
    from crane.study import guard
    return guard.is_held_out(role)


def _failure(fail, status, state, role=None):
    if fail:
        tb = (fail.get('traceback') or '').strip().splitlines()
        if not guard.shows_outcomes(role):      # a traceback may quote outcome numbers: the reason only
            tb = [f'({role or "no role"}: traceback in failure.json)'] if tb else []
        return {'reason': fail.get('reason') or 'crashed', 'traceback': tb[-1] if tb else '', 'host': fail.get('host'),
                'traceback_head': tb[:1]}
    if status == 'stalled':
        return {'reason': 'timed out without progress', 'traceback': f'progress.json not updated for '
                f'{progress.duration(state.get("age_s"))}', 'host': None}
    if status == 'cancelled':
        return {'reason': 'cancelled', 'traceback': '', 'host': None}
    return None


def _metrics(d, state):
    m = _json(Path(d) / 'metrics.json') or state.get('interim') or {}
    return {k: v for k, v in m.items() if isinstance(v, (int, float)) and not isinstance(v, bool)}


# ---- cluster --------------------------------------------------------------------------------------------------------

def _rc():
    from crane_backends import find_platform
    if not find_platform():
        return None
    try:
        from crane_cluster import rc
        return rc
    except ImportError:
        return None


def cluster_entries(plan, feed=None, views=None):
    """Entries for the plan's jobs from the project job list (`views`: {job_id: view}) and the public feed (`feed`:
    the /v1/feed document); either may be None (unreachable). Jobs neither knows are 'unknown'."""
    jobs = plan.get('jobs') or []
    by_feed = {}
    for j in (feed or {}).get('jobs') or []:
        jid = str(j.get('id') or '')
        by_feed[jid.split('/', 1)[-1]] = j
    out = []
    for job in jobs:
        jid, spec = job['job_id'], job.get('spec') or {}
        v, f = (views or {}).get(jid) or {}, by_feed.get(jid) or {}
        state = v.get('status') or {'running': 'leased'}.get(f.get('state'), f.get('state')) or 'unknown'
        n = len(spec.get('seeds') or [])
        prog = f.get('progress') or {}
        frac = prog.get('fraction')
        done = n if state == 'done' else int(round(n * frac)) if frac is not None and state != 'queued' else 0
        hb = (v.get('progress') or {}) if isinstance(v.get('progress'), dict) else {}
        if hb.get('total'):
            done = int(n * min(1.0, float(hb.get('done') or 0) / float(hb['total']))) if state != 'done' else n
        status = {'leased': 'running', 'dead': 'failed', 'closed': 'cancelled'}.get(state, state)
        err = v.get('last_error') or f.get('error')
        eta = (f.get('eta') or {}).get('remaining_s')
        out.append({
            'source': 'cluster', 'key': jid, 'dir': None, 'study': spec.get('study'), 'stage': spec.get('stage'),
            'arm': spec.get('arm'), 'block': spec.get('block'), 'role': spec.get('role'), 'cell': spec.get('cell'),
            'host': f.get('worker') or v.get('worker'), 'status': status, 'done': done, 'total': n,
            'failed': n if status == 'failed' else 0, 'rate_per_s': 0.0, 'eta_s': eta if status in ('running', 'queued')
            else None, 'age_s': None,
            'failure': _cluster_failure(status, err), 'metrics': {},
            'touched_held_out': (bool(spec.get('touched_held_out')) or _held(spec.get('role')))
            and status not in ('queued', 'unknown')})
    return out


def _cluster_failure(status, err):
    if status not in ('failed', 'cancelled'):
        return None
    text = str(err or '').strip()
    low = text.lower()
    reason = ('cancelled' if status == 'cancelled' or 'cancel' in low else
              'timed out without progress' if 'without progress' in low or 'stalled' in low or 'timed out' in low
              else 'crashed')
    return {'reason': reason, 'traceback': text.splitlines()[-1][:300] if text else '', 'host': None}


def fetch_feed(url=None, timeout=FEED_TIMEOUT_S):
    """The public /v1/feed document, or (None, why)."""
    url = (url or os.environ.get('RC_URL') or 'https://research.lunarz.dev').rstrip('/')
    try:
        req = urllib.request.Request(url + '/v1/feed', headers={'User-Agent': 'crane-monitor/1'})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read()), None
    except Exception as error:   # noqa: BLE001 - any failure means: no cluster view this refresh
        return None, f'{type(error).__name__}: {error}'[:160]


def project_views(plan):
    """{job_id: view} from the project's job list (needs the research token), or (None, why)."""
    rc = _rc()
    if rc is None:
        return None, 'no research-cluster checkout (RESEARCH_CLUSTER or ~/research-cluster)'
    project, campaign = plan.get('project') or 'crane', plan.get('campaign')
    try:
        api = rc.Api(os.environ.get('RC_URL', rc.DEFAULT_URL), rc.read_token(project))
        q = f'?campaign={campaign}&limit=1000' if campaign else '?limit=1000'
        rows = api.call('GET', f'/v1/projects/{project}/jobs{q}').get('jobs', [])
    except Exception as error:   # noqa: BLE001
        return None, f'{type(error).__name__}: {error}'[:160]
    return {str(v.get('job_id', '')).split('/', 1)[-1]: v for v in rows}, None


# ---- merge ----------------------------------------------------------------------------------------------------------

def merge(entries):
    """Groups by (study, stage, arm, block): progress, rate, ETA, hosts, failures, and dev-only interim metrics."""
    seen = {}
    for e in entries:
        k = e['key']
        old = seen.get(k)
        if old is None or (e['source'] == 'local' and (e['status'] in TERMINAL or old['status'] not in TERMINAL)):
            seen[k] = e
    groups = {}
    for e in seen.values():
        g = groups.setdefault((e['study'], e['stage'], e['arm'], e['block']), {
            'study': e['study'], 'stage': e['stage'], 'arm': e['arm'], 'block': e['block'], 'role': e['role'],
            'done': 0, 'total': 0, 'failed': 0, 'rate_per_s': 0.0, 'eta_s': None, 'hosts': set(),
            'jobs': {'done': 0, 'running': 0, 'queued': 0, 'failed': 0, 'other': 0}, 'failures': [], '_m': {},
            'touched_held_out': False})
        g['role'] = g['role'] or e['role']
        g['touched_held_out'] = g['touched_held_out'] or bool(e.get('touched_held_out'))
        g['done'] += e['done']
        g['total'] += e['total']
        g['failed'] += e['failed']
        g['rate_per_s'] += e['rate_per_s']
        if e['eta_s'] is not None:
            g['eta_s'] = max(g['eta_s'] or 0.0, float(e['eta_s']))
        if e['host']:
            g['hosts'].add(e['host'])
        bucket = e['status'] if e['status'] in g['jobs'] else 'failed' if e['status'] == 'stalled' else 'other'
        g['jobs'][bucket] += 1
        if e['failure']:
            g['failures'].append({'where': e['cell'] or e['key'], 'job': e['key'], 'host': e['failure'].get('host')
                                  or e['host'], 'reason': e['failure']['reason'],
                                  'traceback': e['failure'].get('traceback', '')})
        rows = float(e['metrics'].get('rows') or 1.0)
        for name, v in e['metrics'].items():
            if name.endswith('.mean'):
                s = g['_m'].setdefault(name[:-5], [0.0, 0.0])
                s[0] += float(v) * rows
                s[1] += rows
    out = []
    for g in groups.values():
        m = g.pop('_m')
        g['hosts'] = sorted(g['hosts'])
        left = max(g['total'] - g['done'], 0)
        if g['eta_s'] is None and g['rate_per_s'] > 0:
            g['eta_s'] = round(left / g['rate_per_s'], 1)
        if guard.shows_means(g['role']):
            g['metrics'] = {k: round(s / n, 6) for k, (s, n) in sorted(m.items()) if n}
        else:
            g['metrics'] = None          # progress only: held-out, train, calib and a role-less block
        out.append(g)
    return sorted(out, key=lambda g: tuple(str(g[k] or '') for k in ('study', 'stage', 'arm', 'block')))


def snapshot(target=None, root=None, cluster=True, feed_url=None):
    """The crane.monitor/1 document for one refresh."""
    where, name = resolve(target, root)
    entries = local_entries(where)
    plans = [where / PLAN] if name else sorted(where.glob('*/' + PLAN))
    notes, sources = [], {'local': len(entries), 'cluster': 0}
    for path in plans:
        plan = _json(path)
        if not plan:
            continue
        views = feed = None
        if cluster:
            views, why = project_views(plan)
            if why:
                notes.append(f'cluster jobs: {why}')
            feed, why = fetch_feed(feed_url)
            if why:
                notes.append(f'cluster feed: {why}')
        if cluster and (views is not None or feed is not None):
            got = cluster_entries(plan, feed, views)
            sources['cluster'] += len(got)
            entries += got
    groups = merge(entries)
    done = bool(groups) and all(g['jobs']['running'] == 0 and g['jobs']['queued'] == 0 and g['jobs']['other'] == 0
                                for g in groups)
    return {'schema': 'crane.monitor/1', 'at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()), 'where': str(where),
            'study': name, 'sources': sources, 'notes': sorted(set(notes)), 'finished': done, 'groups': groups}


# ---- text -----------------------------------------------------------------------------------------------------------

def _fmt_metrics(g):
    if g['metrics'] is None:
        touched = '; touched' if g.get('touched_held_out') else ''
        return f'({g["role"] or "no role"}: progress only{touched})' if not guard.shows_means(g['role']) else ''
    shown = list(g['metrics'].items())[:METRICS_SHOWN]
    return '  '.join(f'{k} {v:.4g}' for k, v in shown)


def render(doc, out):
    s = doc['sources']
    out.line(f"monitor  {doc['study'] or doc['where']}  (local {s['local']}, cluster {s['cluster']} jobs)  {doc['at']}")
    if not doc['groups']:
        out.line('  nothing yet: no progress.json under ' + doc['where'])
    rows = [('stage', 'arm', 'block', 'done/total', 'rate', 'eta', 'failed', 'jobs r/q/d/f', 'hosts', 'interim')]
    for g in doc['groups']:
        j = g['jobs']
        rows.append((g['stage'] or '-', g['arm'] or '-', g['block'] or '-', f"{g['done']}/{g['total']}",
                     f"{g['rate_per_s']:.2f}/s", progress.duration(g['eta_s']) if g['eta_s'] is not None else '-',
                     str(g['failed']), f"{j['running']}/{j['queued']}/{j['done']}/{j['failed']}",
                     ','.join(g['hosts'][:3]) + ('...' if len(g['hosts']) > 3 else '') or '-', _fmt_metrics(g)))
    if doc['groups']:
        out.table(rows)
    fails = [(g, f) for g in doc['groups'] for f in g['failures']]
    if fails:
        out.line('failures:')
        out.table([(f"{g['stage'] or '-'}/{g['arm'] or '-'}/{g['block'] or '-'}", f['where'], f['reason'],
                    f"on {f['host']}" if f['host'] else '', f['traceback'][:120]) for g, f in fails[:10]])
        if len(fails) > 10:
            out.line(f'  ... {len(fails) - 10} more (--json has every one)')
    for note in doc['notes']:
        out.line(f'note: {note}')


def main(args):
    out = args.output
    interval = min(max(float(args.interval or 30.0), FOLLOW_MIN_S), FOLLOW_MAX_S)
    timeout = float(args.timeout or 3600.0)
    target = args.target
    if target and not Path(target).expanduser().is_dir() and not (studies_root(args.out) / target).is_dir():
        return out.usage('monitor', f'no study or dir {target!r} under {studies_root(args.out)}',
                         'crane monitor (every study) or crane monitor DIR')
    end = time.monotonic() + timeout
    while True:
        doc = snapshot(target, args.out, cluster=not args.local)
        if out.json:
            out.record(doc) if args.follow else out.doc(doc)
        else:
            render(doc, out)
        if not args.follow:
            break
        if doc['finished']:
            out.line('monitor: every job has ended')
            break
        left = end - time.monotonic()
        if left <= 0:
            out.line(f'monitor: --follow stopped after {progress.duration(timeout)} (--timeout)')
            return output.TIMEOUT
        out.line(f'(next refresh in {interval:.0f} s; Ctrl-C stops)')
        try:
            time.sleep(min(interval, left))
        except KeyboardInterrupt:
            break
    name = doc['study']
    failed = any(g['failures'] for g in doc['groups'])
    if failed:
        out.next(f'crane cluster status {name}' if name else 'crane monitor STUDY --json')
    elif doc['finished'] and name:
        out.next(f'crane cluster fetch {name}', f'crane study report {name}')
    elif not args.follow:
        out.next(f'crane monitor {target or ""} --follow'.replace('  ', ' '))
    return output.OK
