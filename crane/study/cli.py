"""crane study init|plan|run|status|report|promote|run-cell: the handlers the registry (crane/commands.py) names."""

import json
from pathlib import Path
import sys

from crane import output
from crane.study import exec as ex, plan, report as rp
from crane.study.spec import StudyError, load

INIT = '''\
# A crane.study/1 spec. Docs/Study.md has every key; `crane study plan THIS.yaml` counts it, nothing runs.
schema: crane.study/1
name: {name}
domain: {domain}  # config/domains/{domain}.yaml; sim arms run its tiers
tier: T0  # or tiers: [T0, T1]
# Arms: what is compared. set: config overrides (dotted keys); card: a plant card; task: module:function or
# argv: [...] with JSON args for work that is not a sim. sample: {{n, seed, set: {{key: {{uniform: [lo, hi]}}}}}}
# draws n arms.
arms:
  base: {{}}
  wide: {{set: {{adapter.options.reach_m: 2.5}}}}
# Axes: named params crossed with every arm. Levels are a list, or named sources {{name: value}} (a value set
# when there is no param; card: true makes the values card refs).
axes:
  fov: {{param: perception.hfov_deg, levels: [60, 90]}}
# conditions: {{csv: conditions.csv, id: id, set: {{column: config.key}}}}   rows of a CSV, each a condition
# splits: {{column: split, roles: {{calib: calib, test: held-out}}}}       conditions split by a column
# Seed blocks: the seed is the pairing unit (the same seeds run in every arm). Roles: dev check train calib held-out.
seeds:
  dev: {{role: dev, seeds: '0:8'}}          # N, A:B, a,b,c, @FILE or block:NAME
  test: {{role: held-out, seeds: '1000:1008'}}
targets: {name}.targets.yaml  # committed before a held-out block may run (--confirm)
# scorer: scorers.py:score       # result, cell -> {{part: number}}; rows get score.PART
# metrics: [succeeded, run.sim_s] # the fixed row columns (default: every number the runs give)
# stages: {{R: {{blocks: [dev]}}, V: {{after: R, select: {{top_k: 2, metric: score.total}}, tier: T1}}}}
pack: {{unit_s: 0.05, target_s: 900}}   # seconds per unit, and the job size the cluster plan packs to
# campaign: ..., purpose: ..., expect: ..., pin: {{hosts: [...], tags: [...]}}   passed to the cluster compiler
'''


def _load(out, verb, spec):
    try:
        return load(spec), None
    except StudyError as error:
        return None, out.usage(f'study {verb}', str(error))


def _out(args, study):
    return Path(args.out).expanduser() if getattr(args, 'out', None) else plan.default_out(study)


def _dur(seconds):
    from crane.progress import duration
    return duration(seconds) if seconds is not None else '?'


def init_main(args):
    out = args.output
    name = Path(args.out).stem if args.out else 'my_study'
    text = INIT.format(name=name.replace('.', '_'), domain=args.domain or 'toy')
    if args.out:
        path = Path(args.out).expanduser()
        if path.exists() and not args.force:
            return out.usage('study init', f'{path} exists', f'crane study init --out {path} --force')
        path.write_text(text)
        if out.doc({'schema': 'crane.study.init/1', 'path': str(path)}):
            return output.OK
        out.say(f'wrote {path}')
        out.next(f'crane study plan {path}')
        return output.OK
    if out.doc({'schema': 'crane.study.init/1', 'path': None, 'spec': text}):
        return output.OK
    sys.stdout.write(text)
    return output.OK


def plan_main(args):
    out = args.output
    study, code = _load(out, 'plan', args.spec)
    if not study:
        return code
    where = _out(args, study)
    have = where if (where / 'cache').is_dir() else None
    try:
        cells = plan.cells(study, args.stage, args.block, have)
        jobs = plan.jobs(study, args.stage, args.block, have)
    except StudyError as error:
        return out.usage('study plan', str(error))
    done = plan.cached_rows(cells, have) if have else {}
    stages = []
    for name in ([args.stage] if args.stage else study.order()):
        cs = [c for c in cells if c.stage == name]
        units = sum(len(c.seeds) for c in cs)
        unit_s = plan.unit_s(study, name)
        js = [j for j in jobs if j['stage'] == name]
        tiers = sorted({c.tier for c in cs}) or (study.stages[name].tiers or study.tiers)
        stages.append({'stage': name, 'tiers': tiers,
                       'cells': len(cs), 'units': units, 'done': sum(len(done.get(c.id, {})) for c in cs),
                       'held_out_units': sum(len(c.seeds) for c in cs if c.held_out), 'jobs': len(js),
                       'unit_s': unit_s, 'est_s': round(units * unit_s, 1) if unit_s else None,
                       'job_s': [min(j['est_seconds'] for j in js), max(j['est_seconds'] for j in js)]
                       if js and unit_s else None,
                       'after': study.stages[name].after, 'select': study.stages[name].select})
    pending = plan.pending(study, have)
    doc = {'schema': 'crane.study.plan/1', 'study': study.name, 'spec': str(study.path), 'out': str(where),
           'domain': study.domain, 'commit': plan.code_commit(),
           'blocks': {b.name: {'role': b.role, 'seeds': len(b.seeds)} for b in study.blocks.values()},
           'splits': study.splits, 'stages': stages, 'pending': pending, 'cluster': study.cluster, 'jobs': jobs}
    if out.doc(doc):
        return output.OK
    out.say(f'study {study.name}  {study.path}  domain {study.domain or "-"}  (nothing runs)')
    rows = [('stage', 'tiers', 'cells', 'units', 'done', 'held-out', 'jobs', 'est', 'per job')]
    for s in stages:
        per = f'{_dur(s["job_s"][0])}-{_dur(s["job_s"][1])}' if s['job_s'] else '?'
        rows.append((s['stage'], ','.join(s['tiers']), s['cells'], s['units'], s['done'], s['held_out_units'],
                     s['jobs'], _dur(s['est_s']) if s['unit_s'] else '? (pack.unit_s)', per))
    out.table(rows)
    out.line('  blocks: ' + '; '.join(f'{n} {b["role"]} {b["seeds"]} seeds' for n, b in doc['blocks'].items()))
    if study.splits:
        out.line('  splits: ' + '; '.join(f'{n} {r}' for n, r in study.splits.items()))
    for name, why in pending.items():
        out.line(f'  pending: stage {name} ({why})')
    short = [s['stage'] for s in stages if s['job_s'] and s['job_s'][0] < 60]
    if short:
        out.line(f'  note: stage {", ".join(short)} has jobs under 1 min; raise pack.target_s or seeds per block')
    out.next(f'crane study run {args.spec}' + (f' --stage {args.stage}' if args.stage else ''))
    return output.OK


def run_main(args):
    out = args.output
    study, code = _load(out, 'run', args.spec)
    if not study:
        return code
    if args.where != 'local':
        return out.usage('study run', f'--where {args.where} submits through the research cluster',
                         f'crane study compile {args.spec} --write jobs.json && research submit -p crane jobs.json')
    where = _out(args, study)
    if out.dry_run:
        args.out = str(where)
        return plan_main(args)
    stream = sys.stderr if out.json else None
    out.line(f'study {study.name}  out {where}  workers {args.max_workers}')
    try:
        summary = ex.run_study(study, where, args.stage, args.block, args.confirm, args.max_workers, stream=stream)
    except StudyError as error:
        return out.usage('study run', str(error))
    except KeyboardInterrupt:
        return out.fail('study run', 'interrupted; finished units are cached', f'crane study run {args.spec} (resumes)')
    manifest = _manifest(where)
    doc = {'schema': 'crane.study.run/1', 'study': study.name, **summary,
           'rows': manifest.get('rows'), 'manifest': str(where / 'manifest.json')}
    if summary['refused']:
        out.doc(doc)
        return out.usage('study run', summary['refused'])
    if out.doc(doc):
        return output.FAILED if summary['failed'] else output.OK
    rows = [('stage', 'cells', 'units', 'ran', 'reused', 'failed')]
    rows += [(n, s['cells'], s['units'], s['ran'], s['reused'], s['failed']) for n, s in summary['stages'].items()]
    out.table(rows)
    for name in summary['skipped_held_out']:
        out.line(f'  held-out {name} skipped: it runs with --block {name.split(":")[1]} --confirm')
    if summary['touched_held_out']:
        out.line('  held-out touched: recorded on its rows and in the manifest')
    out.say(f'rows: {manifest.get("rows", 0)} in {where / "rows"}, manifest {where / "manifest.json"}')
    if summary['failed']:
        out.fail('study run', f'{summary["failed"]} units failed (not cached; a rerun retries them)',
                 f'crane study status {where}')
        return output.FAILED
    out.next(f'crane study report {where}', f'crane study status {where}')
    return output.OK


def _manifest(where):
    try:
        return json.loads((Path(where) / 'manifest.json').read_text())
    except (OSError, ValueError):
        return {}


def _spec_of(target, out_arg=None):
    """(spec path, out dir) of `status TARGET`: a spec (its default out, or --out) or an out dir (its study.json)."""
    path = Path(target).expanduser()
    if not path.exists():
        raise StudyError(f'no spec or study output at {path}; fix: crane study status SPEC.yaml, or an output dir '
                         'holding study.json (crane study run writes it)')
    if path.is_dir():
        state = path / 'study.json'
        if not state.is_file():
            raise StudyError(f'{path} holds no study.json (not a study output); fix: crane study status SPEC')
        spec = Path(json.loads(state.read_text())['spec']).expanduser()
        if not spec.is_absolute():                 # recorded relative to the crane checkout
            from crane import paths
            spec = paths.ROOT / spec
        return str(spec), path
    return str(path), Path(out_arg).expanduser() if out_arg else None


def status_main(args):
    out = args.output
    try:
        spec, where = _spec_of(args.target, args.out)
    except StudyError as error:
        return out.usage('study status', str(error))
    study, code = _load(out, 'status', spec)
    if not study:
        return code
    where = where or plan.default_out(study)
    doc = rp.status(study, where)
    if out.doc(doc):
        return output.FAILED if doc['failed'] else output.OK
    out.say(f'study {study.name}  out {where}  commit {doc["commit"][:12]}')
    rows = [('stage', 'block', 'role', 'units', 'done', '')]
    for name, st in doc['stages'].items():
        for b, x in st['blocks'].items():
            pct = f'{100 * x["done"] // max(x["units"], 1)}%'
            rows.append((name, b, x['role'], x['units'], x['done'], pct + ('  progress only' if x['role'] == 'held-out'
                                                                          else '')))
    out.table(rows)
    for name, why in doc['pending'].items():
        out.line(f'  pending: stage {name} ({why})')
    if doc['progress']:
        from crane.progress import line
        p = doc['progress']
        out.line(f'  {line(p)}  updated {_dur(p.get("age_s"))} ago')
    if doc['cached_other_commits']:
        out.line(f'  {doc["cached_other_commits"]} units cached at other commits (not counted; they rerun here)')
    if doc['failure']:
        f = doc['failure']
        out.say(f'failed {doc["failed"]} units; latest: {f["cell"]} seed {f["seed"]}: {f["error"]}')
        for ln in f['traceback'].splitlines():
            out.line(f'    {ln}')
    if doc['interim']:
        names = doc['interim_metrics']
        out.line('interim means (dev blocks only):')
        rows = [('stage', 'arm', 'n', *names)]
        for stage, arms in doc['interim'].items():
            for arm, ms in arms.items():
                n = max((v['n'] for v in ms.values()), default=0)
                rows.append((stage, arm, n, *(rp.fmt(ms[m]['mean']) if m in ms else '-' for m in names)))
        out.table(rows)
    if doc['failed']:
        out.next(f'crane study run {spec} (retries the failed units)')
        return output.FAILED
    out.next(f'crane study report {where}')
    return output.OK


def report_main(args):
    out = args.output
    try:
        doc = rp.report(args.out, args.baseline, args.metric or None, B=args.bootstrap)
    except FileNotFoundError as error:
        return out.usage('study report', str(error), 'crane study run SPEC (writes OUT/manifest.json)')
    if out.doc(doc):
        return output.OK
    out.say(f'study {doc["study"]}: {doc["rows"]} rows, baseline {doc["baseline"]}, each arm minus the baseline '
            'paired on the seed')
    for k, v in doc['held_out_waiting'].items():
        out.line(f'  held-out {k}: {v["done"]}/{v["total"]} units, progress only until done')
    if doc['pooled']:
        out.table(rp.pooled_rows(doc))
    else:
        out.line('  no paired rows yet (each cell needs the baseline and another arm on the same seeds)')
    if doc['targets']:
        out.line('held-out targets:')
        out.table([('contrast', 'metric', 'direction', 'verdict'),
                   *((t['contrast'], t['metric'], t['direction'], t['verdict'] + (f'  ({t["why"]})' if t['why']
                                                                                   else '')) for t in doc['targets'])])
    out.say(f'wrote {Path(args.out) / "report.md"} and report.json ({len(doc["cells"])} cell contrasts)')
    out.next(f'cat {Path(args.out) / "report.md"}')
    return output.OK


def promote_main(args):
    out = args.output
    study, code = _load(out, 'promote', args.spec)
    if not study:
        return code
    if args.stage not in study.stages or not study.stages[args.stage].select:
        adaptive = [n for n, s in study.stages.items() if s.select]
        return out.usage('study promote', f'stage {args.stage!r} selects from no other stage',
                         f'crane study promote {args.spec} --stage {adaptive[0]}' if adaptive else
                         'add select: {top_k, metric} to a stage with after:')
    where = Path(args.out).expanduser() if args.out else plan.default_out(study)
    try:
        sel = plan.selection(study, args.stage, where)
    except plan.StageNotReady as error:
        return out.usage('study promote', str(error).split(': ', 1)[-1] if ': stages.' in str(error) else str(error))
    raw = json.loads(json.dumps(study.raw, default=str))
    stage = dict(raw['stages'][args.stage] or {})
    stage.pop('select', None)
    stage.pop('after', None)
    chosen = [t for t, _, _ in sel['table'] if t in sel['chosen']]
    if sel['by'] == ['arm']:
        stage['arms'] = [t[0] for t in chosen]
    else:
        stage['only'] = [dict(zip(sel['by'], t)) for t in chosen]
    raw['stages'][args.stage] = stage
    target = Path(args.write).expanduser() if args.write else study.path.with_name(
        f'{study.path.stem}.{args.stage}.yaml')
    if target.resolve().parent != study.base:
        _absolute(raw, study.base)
    import yaml
    header = (f'# Promoted from {study.path.name} stage {args.stage}: top {sel["top_k"]} by {sel["metric"]} '
              f'({sel["order"]}) of stage {sel["from"]} at commit {plan.code_commit()[:12]}.\n')
    doc = {'schema': 'crane.study.promote/1', 'study': study.name, 'stage': args.stage, 'from': sel['from'],
           'metric': sel['metric'], 'order': sel['order'], 'by': sel['by'], 'top_k': sel['top_k'],
           'table': [{'group': list(t), 'mean': m, 'n': n, 'chosen': t in sel['chosen']} for t, m, n in sel['table']],
           'written': str(target)}
    target.write_text(header + yaml.safe_dump(raw, sort_keys=False, width=120))
    if out.doc(doc):
        return output.OK
    out.say(f'stage {args.stage}: top {sel["top_k"]} of stage {sel["from"]} by {sel["metric"]} ({sel["order"]})')
    rows = [('rank', *sel['by'], 'mean', 'n', '')]
    rows += [(i + 1, *t, rp.fmt(m), n, 'chosen' if t in sel['chosen'] else '') for i, (t, m, n)
             in enumerate(sel['table'])]
    out.table(rows)
    out.say(f'wrote {target}')
    out.next(f'crane study run {target} --stage {args.stage}', f'crane study plan {target} --stage {args.stage} --json')
    return output.OK


def _absolute(raw, base):
    """Make the spec's relative paths absolute (a promoted spec written outside the spec's dir)."""
    def fix(path):
        return str((base / path).resolve()) if path and not Path(path).is_absolute() else path

    def ref(r):
        target, sep, attr = str(r).rpartition(':')
        return f'{fix(target)}:{attr}' if sep and target.endswith('.py') else r
    if raw.get('targets'):
        raw['targets'] = fix(raw['targets'])
    if isinstance(raw.get('conditions'), dict) and raw['conditions'].get('csv'):
        raw['conditions']['csv'] = fix(raw['conditions']['csv'])
    elif isinstance(raw.get('conditions'), str):
        raw['conditions'] = fix(raw['conditions'])
    if isinstance(raw.get('scorer'), dict):
        raw['scorer']['ref'] = ref(raw['scorer']['ref'])
    elif raw.get('scorer'):
        raw['scorer'] = ref(raw['scorer'])
    for arm in (raw.get('arms') or {}).values() if isinstance(raw.get('arms'), dict) else ():
        if isinstance(arm, dict) and arm.get('task'):
            arm['task'] = ref(arm['task'])
    for block in (raw.get('seeds') or {}).values():
        if isinstance(block, dict) and isinstance(block.get('seeds'), str):
            block['seeds'] = ','.join('@' + fix(p[1:]) if p.strip().startswith('@') else p
                                      for p in block['seeds'].split(','))


def _seed_list(text):
    try:
        return [int(x) for x in str(text).split(',') if x.strip()]
    except ValueError:
        raise StudyError(f'--seeds {text!r}: comma-separated ints, e.g. 3,4,5') from None


def run_cell_main(args):
    out = args.output
    study, code = _load(out, 'run-cell', args.spec)
    if not study:
        return code
    try:
        summary, refused = ex.run_job(study, args.cell, args.out, args.confirm,
                                      stream=sys.stderr if out.json else None,
                                      targets_sha256=args.targets_sha256,
                                      only=_seed_list(args.seeds) if getattr(args, 'seeds', None) else None)
    except StudyError as error:
        return out.usage('study run-cell', str(error))
    if refused:
        return out.usage('study run-cell', refused)
    if out.doc({'schema': 'crane.study.run_cell/1', **summary}):
        return output.FAILED if summary['failed'] else output.OK
    out.say(f'{summary["job_id"]}: {summary["ran"]} ran, {summary["reused"]} reused, {summary["failed"]} failed; '
            f'{summary["rows"]} rows in {summary["out"]}')
    if summary['oversize']:
        out.line(f'  over 95 MB: {", ".join(summary["oversize"])}; fix: a smaller pack.target_s, or record: summary')
    return output.FAILED if summary['failed'] else output.OK


def compile_main(args):
    from crane.cluster import compile as cc
    out = args.output
    try:
        study, jobs = cc.compile_spec(args.spec, args.stage, args.block, confirm=args.confirm,
                                      allow_dirty=getattr(args, 'allow_dirty', False))
    except cc.CompileError as error:
        return out.usage('study compile', str(error))
    if not jobs and jobs.skip_reason:
        return out.usage('study compile', jobs.skip_reason, f'crane study compile {args.spec} --block BLOCK --confirm')
    body = {'jobs': list(jobs)}
    if args.write:
        from crane import progress
        progress.write_atomic(Path(args.write).expanduser(), json.dumps(body, indent=1) + '\n')
    doc = {'schema': 'crane.study.compile/1', 'study': study.name, 'written': args.write, 'skipped_held_out':
           jobs.skipped, 'warnings': jobs.warnings, 'est_seconds': sum(j['est_seconds'] for j in jobs),
           'plan': cc.summarize(jobs), **body}
    if out.doc(doc):
        return output.OK
    if not args.write:
        sys.stdout.write(json.dumps(body, indent=1) + '\n')
        return output.OK
    kinds = sorted({j['kind'] for j in jobs})
    out.say(f'{len(jobs)} rc-job/1 jobs ({", ".join(kinds) or "-"}, {_dur(doc["est_seconds"])} in all) in '
            f'{args.write}')
    for w in jobs.warnings:
        out.line(f'  {w}')
    out.next(f'research submit -p crane {args.write}')
    return output.OK
