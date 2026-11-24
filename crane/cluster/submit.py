"""crane cluster submit | wait | status: a study's jobs through the research CLI, digest style.

    crane cluster submit study.yaml --dry-run          # the plan table; nothing is sent
    crane cluster submit study.yaml --est-unit-s 12    # writes OUT/<study>/cluster/jobs.json, then research submit
    crane cluster submit jobs.json                      # an rc-job/1 list (or {jobs: [...]}) as is
    crane cluster wait boat01 --timeout 3600            # one bounded research wait, then the status table
    crane cluster status boat01                         # per stage/arm/block: done, running, queued, failed

submit, wait and fetch shell out to `research` (RESEARCH_CLUSTER/bin/research or ~/research-cluster/bin/research);
status reads the project's job list through research-cluster's own client. Without either: one line naming the fix.
"""

import json
import subprocess
import sys
from pathlib import Path

from crane import output, paths, progress
from crane.cluster import PLAN, plan_path
from crane.cluster import compile as cc

NO_CLI = ('no research-cluster CLI', 'clone research-cluster to ~/research-cluster or set RESEARCH_CLUSTER')
RESEARCH_EXIT = {0: output.OK, 1: output.FAILED, 2: output.FAILED, 3: output.TIMEOUT, 4: output.USAGE,
                 75: output.TIMEOUT}


def load_study(path):
    """crane.study.load (w3-study owns it), with a one-line reason when this checkout lacks it."""
    try:
        from crane import study as study_mod
    except ImportError:
        raise cc.CompileError('crane study is not in this checkout (crane/study/); give a jobs.json instead') from None
    return study_mod.load(path)


def load_plan(target, root=None):
    """(plan doc, path) for a study name, an OUT/<study> dir or a jobs file; (None, path) when there is none."""
    p = Path(str(target)).expanduser()
    if p.is_file() and p.suffix == '.json':
        return _plan_of(json.loads(p.read_text()), p), p
    if p.is_dir() and (p / PLAN).is_file():
        p = p / PLAN
    else:
        p = plan_path(root, target)
    return (_plan_of(json.loads(p.read_text()), p) if p.is_file() else None), p


def _plan_of(doc, path):
    if isinstance(doc, list):
        doc = {'jobs': doc}
    jobs = doc.get('jobs') or []
    first = (jobs[0].get('spec') or {}) if jobs else {}
    return {'schema': 'crane.cluster.plan/1', 'study': doc.get('study') or first.get('study'),
            'project': doc.get('project') or 'crane',
            'campaign': doc.get('campaign') or (jobs[0].get('campaign') if jobs else None), **doc, 'jobs': jobs,
            'path': str(path)}


def write_plan(root, name, jobs, project, extra=None):
    path = plan_path(root, name)
    path.parent.mkdir(parents=True, exist_ok=True)
    doc = {'schema': 'crane.cluster.plan/1', 'study': name, 'project': project,
           'campaign': jobs[0]['campaign'] if jobs else None, **(extra or {}), 'jobs': list(jobs)}
    from crane.progress import write_atomic
    write_atomic(path, json.dumps(doc, indent=1) + '\n')
    return path


def _research(args, capture=True, timeout=None):
    cli = paths.research_cli()
    if cli is None:
        return None
    return subprocess.run([sys.executable, str(cli), *args], capture_output=capture, text=True, timeout=timeout)


def _plan_rows(jobs):
    rows = [('stage', 'arm', 'block', 'role', 'kind', 'jobs', 'units', 'est/job', 'compute')]
    for r in cc.summarize(jobs):
        est = (progress.duration(r['est_min_s']) if r['est_min_s'] == r['est_max_s'] else
               f"{progress.duration(r['est_min_s'])}-{progress.duration(r['est_max_s'])}")
        rows.append((r['stage'] or '-', r['arm'], r['block'] or '-', r['role'] or '-', r['kind'], str(r['jobs']),
                     str(r['units']), est, f"{r['compute_s'] / 3600:.2f} h"))
    return rows


# ---- submit ---------------------------------------------------------------------------------------------------------

def compile_from(args):
    """(jobs, study name, warnings) from a study spec or a jobs file."""
    src = Path(args.source).expanduser()
    if not src.is_file():
        raise cc.CompileError(f'no file {src}')
    if src.suffix == '.json':
        plan = _plan_of(json.loads(src.read_text()), src)
        if not plan['jobs']:
            raise cc.CompileError(f'{src} holds no jobs (a list of rc-job/1 bodies or {{"jobs": [...]}})')
        return plan['jobs'], plan['study'] or src.stem, []
    study = load_study(src)
    est = args.est_unit_s
    if not est and args.est_from:
        est = cc.bench_estimate(args.est_from)
    if not est and args.measure:
        first = next(c for c in study.cells() if (not args.stage or c.stage == args.stage)
                     and (not args.block or c.block == args.block))
        est = cc.measure(src, first, first.seeds[0])
    done = None
    cache_dir = getattr(study, 'cache_dir', None)
    if cache_dir and not args.no_skip_done:
        from crane.cache import UnitCache
        done = UnitCache(cache_dir)
    jobs = cc.compile(study, stage=args.stage, block=args.block, spec_path=src, est_unit_s=est, done=done,
                      tags=list(args.tag or []) + (['cloud-ok', 'cpu'] if args.cloud else []), project=args.project,
                      campaign=args.campaign, purpose=args.purpose, confirm=args.confirm,
                      allow_dirty=args.allow_dirty, out_root=args.out, kind=args.kind)
    if args.cloud:
        for j in jobs:
            if j['needs'].get('gpu'):
                j['needs']['tags'] = sorted((set(j['needs']['tags']) - {'cpu'}) | {'torch'})
    return jobs, cc.study_name(study), jobs.warnings


def submit_main(args):
    out = args.output
    try:
        jobs, name, warnings = compile_from(args)
    except (cc.CompileError, ValueError) as error:
        return out.usage('cluster submit', str(error))
    project = args.project or 'crane'
    if not jobs:
        out.say(f'cluster submit: nothing to run for {name} (every unit is done)')
        out.doc({'schema': 'crane.cluster.submit/1', 'study': name, 'jobs': [], 'submitted': False})
        return output.OK
    if out.json and args.dry_run:
        out.doc({'schema': 'crane.cluster.submit/1', 'study': name, 'project': project, 'dry_run': True,
                 'warnings': warnings, 'plan': cc.summarize(jobs), 'jobs': jobs})
        return output.OK
    out.line(f'cluster submit  {name}  {len(jobs)} jobs  project {project}  campaign {jobs[0]["campaign"]}')
    out.table(_plan_rows(jobs))
    for w in warnings:
        out.line(f'warning: {w}')
    if args.dry_run:
        out.say('cluster submit: dry run; nothing sent')
        out.next(f'crane cluster submit {args.source}' + (f' --est-unit-s {args.est_unit_s:g}' if args.est_unit_s
                                                           else ''))
        return output.OK
    if paths.research_cli() is None:
        return out.usage('cluster submit', NO_CLI[0], NO_CLI[1])
    path = write_plan(args.out, name, jobs, project, {'source': str(Path(args.source).expanduser().resolve())})
    r = _research(['submit', '-p', project, str(path)])
    states = [ln.split()[0] for ln in (r.stdout or '').splitlines() if ln.strip()]
    doc = {'schema': 'crane.cluster.submit/1', 'study': name, 'project': project, 'plan': str(path),
           'exit': r.returncode, 'created': states.count('created'), 'exists': states.count('exists'),
           'duplicate': states.count('duplicate'), 'warnings': warnings}
    if out.doc(doc) and not r.returncode:
        return output.OK
    if r.returncode:
        tail = ((r.stderr or r.stdout or '').strip().splitlines() or [f'exit {r.returncode}'])[-1]
        return out.fail('cluster submit', f'research submit: {tail}',
                        f'crane cluster submit {path} (the plan is saved)',
                        RESEARCH_EXIT.get(r.returncode, output.FAILED))
    out.say(f'cluster submit: {doc["created"]} created, {doc["exists"]} already there, {doc["duplicate"]} equal to '
            f'running/done jobs; plan {path}')
    out.next(f'crane cluster wait {name} --timeout 3600', f'crane monitor {name}')
    return output.OK


# ---- status ---------------------------------------------------------------------------------------------------------

def status_doc(plan, views):
    """Per (stage, arm, block): job counts by state and the failures, from the plan and {job_id: view}."""
    groups, fails = {}, []
    for j in plan['jobs']:
        s = j.get('spec') or {}
        v = views.get(j['job_id']) or {}
        state = v.get('status') or 'unknown'
        g = groups.setdefault((s.get('stage'), s.get('arm'), s.get('block')), {
            'stage': s.get('stage'), 'arm': s.get('arm'), 'block': s.get('block'), 'role': s.get('role'),
            'jobs': 0, 'units': 0, 'states': {}})
        g['jobs'] += 1
        g['units'] += len(s.get('seeds') or [])
        g['states'][state] = g['states'].get(state, 0) + 1
        if state in ('failed', 'dead', 'cancelled') or (state == 'queued' and v.get('why')):
            err = str(v.get('last_error') or v.get('why') or '').strip().splitlines()
            fails.append({'job': j['job_id'], 'state': state, 'error': err[-1][:240] if err else ''})
    rows = sorted(groups.values(), key=lambda g: tuple(str(g[k] or '') for k in ('stage', 'arm', 'block')))
    total = {}
    for g in rows:
        for k, n in g['states'].items():
            total[k] = total.get(k, 0) + n
    return {'schema': 'crane.cluster.status/1', 'study': plan.get('study'), 'project': plan.get('project'),
            'campaign': plan.get('campaign'), 'states': total, 'groups': rows, 'failures': fails,
            'finished': all(k in ('done', 'failed', 'dead', 'cancelled', 'closed') for k in total)}


def fetch_views(plan):
    """{job_id: view} for the plan's jobs (the campaign's list, then any missing one by id)."""
    from crane.monitor import _rc
    rc = _rc()
    if rc is None:
        raise RuntimeError('no research-cluster checkout (RESEARCH_CLUSTER or ~/research-cluster)')
    import os
    import urllib.parse
    api = rc.Api(os.environ.get('RC_URL', rc.DEFAULT_URL), rc.read_token(plan['project']))
    views = {}
    if plan.get('campaign'):
        q = urllib.parse.urlencode({'campaign': plan['campaign'], 'limit': 1000})
        for v in api.call('GET', f"/v1/projects/{plan['project']}/jobs?{q}").get('jobs', []):
            views[str(v.get('job_id', '')).split('/', 1)[-1]] = v
    for j in plan['jobs']:
        if j['job_id'] not in views and len(views) < 5000:
            try:
                v = api.call('GET', f"/v1/projects/{plan['project']}/jobs/{urllib.parse.quote(j['job_id'], safe='')}")
                views[j['job_id']] = v
            except Exception:   # noqa: BLE001 - not on the server (never submitted): stays unknown
                pass
    return views


def render_status(doc, out):
    states = '  '.join(f'{k} {n}' for k, n in sorted(doc['states'].items()))
    out.line(f"cluster status  {doc['study']}  project {doc['project']}  campaign {doc['campaign']}  {states}")
    keys = sorted({k for g in doc['groups'] for k in g['states']})
    out.table([('stage', 'arm', 'block', 'jobs', 'units', *keys)] +
              [(g['stage'] or '-', g['arm'] or '-', g['block'] or '-', str(g['jobs']), str(g['units']),
                *(str(g['states'].get(k, 0)) for k in keys)) for g in doc['groups']])
    if doc['failures']:
        out.line('failures:')
        out.table([(f['state'], f['job'], f['error']) for f in doc['failures'][:10]])
        if len(doc['failures']) > 10:
            out.line(f"  ... {len(doc['failures']) - 10} more (--json has every one)")


def _status(args, plan):
    out = args.output
    try:
        views = fetch_views(plan)
    except Exception as error:   # noqa: BLE001 - unreachable, no token, no checkout: one line
        return None, out.fail('cluster status', f'cannot read the cluster ({str(error)[:160]})',
                              'crane cluster status STUDY once the coordinator answers (research digest)')
    doc = status_doc(plan, views)
    if not out.doc(doc):
        render_status(doc, out)
    return doc, None


def status_main(args):
    out = args.output
    plan, path = load_plan(args.study, args.out)
    if plan is None:
        return out.usage('cluster status', f'no plan at {path}', f'crane cluster submit SPEC (writes {PLAN})')
    doc, code = _status(args, plan)
    if code is not None:
        return code
    name = plan['study']
    if doc['failures']:
        out.next(f'crane monitor {name}', 'research digest')
    elif doc['finished']:
        out.next(f'crane cluster fetch {name}')
    else:
        out.next(f'crane cluster wait {name} --timeout 3600')
    return output.OK


# ---- wait -----------------------------------------------------------------------------------------------------------

def wait_main(args):
    out = args.output
    plan, path = load_plan(args.study, args.out)
    if plan is None:
        return out.usage('cluster wait', f'no plan at {path}', f'crane cluster submit SPEC (writes {PLAN})')
    if paths.research_cli() is None:
        return out.usage('cluster wait', NO_CLI[0], NO_CLI[1])
    ids = [j['job_id'] for j in plan['jobs']]
    timeout = float(args.timeout)
    print(f'cluster wait: {len(ids)} jobs of {plan["study"]}, up to {progress.duration(timeout)}', file=out.stderr,
          flush=True)
    try:
        r = _research(['wait', '-p', plan['project'], *ids, '--timeout', str(int(timeout))], timeout=timeout + 120)
    except subprocess.TimeoutExpired:
        r = None
    code = output.TIMEOUT if r is None else RESEARCH_EXIT.get(r.returncode, output.FAILED)
    doc, failed = _status(args, plan)
    if failed is not None and code == output.OK:
        code = failed
    if code == output.TIMEOUT:
        out.fail('cluster wait', f'not every job ended in {progress.duration(timeout)} (jobs untouched; the digest '
                 'carries anything stuck)', f'crane cluster wait {plan["study"]} --timeout 3600', output.TIMEOUT)
    elif code == output.FAILED and r is not None and r.returncode in (1, 2):
        out.fail('cluster wait', 'some jobs failed or were cancelled', f'crane monitor {plan["study"]}')
    if code == output.OK:
        out.next(f'crane cluster fetch {plan["study"]}')
    return code
