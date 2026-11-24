"""crane run: episodes at a tier. T0 runs the fastsim batch runner (fastsim.batch.run_batch)."""

import shlex
import subprocess
import sys
import time
from collections import Counter
from pathlib import Path

from crane import output, paths, select
from crane.output import Output


def tier_config(domain, tier):
    """(tier, config path, overrides) of DOMAIN's TIER (None: its first tier): bench.fast's reading of
    config/domains/DOMAIN.yaml, so `crane run` and `bench.fast` run the same thing."""
    if str(paths.ROOT) not in sys.path:
        sys.path.insert(0, str(paths.ROOT))
    from bench.fast import domain_tier
    return domain_tier(domain, tier)


def overrides(pairs):
    """['a.b=1', 'c=x'] -> {'a': {'b': 1}, 'c': 'x'}; each value is parsed as YAML (crane.select.parse_sets)."""
    return select.parse_sets(pairs)


def main(args):
    out = getattr(args, 'output', None) or Output.of(args)
    try:
        tier = select.parse_tier(args.tier) if args.tier else None
    except select.SelectError as error:
        return out.usage('run', str(error), 'crane run --tier T0|T1')
    if getattr(args, 'astro_test', None):
        from crane import astrotest
        args.out = args.out or str(select.default_out(args.domain or 'boat', tier or 'T1'))
        return astrotest.main(args)
    if not args.seeds:
        source = f'--domain {args.domain}' if args.domain else f'--config {args.config}'
        return out.usage('run', '--seeds is required (except with --astro-test)', f'crane run {source} --seeds 0:3')
    if tier == 'T2':
        return out.usage('run', '--tier T2 (unity) is not yet available', 'crane sim up --headless (the T2 world)')
    from fastsim import batch, config as fsconfig, identity
    from fastsim.scenario import ScenarioSource
    try:
        if args.config:
            tier, config, merged = tier or 'T0', Path(args.config).resolve(), {}
        else:
            tier, config, merged = tier_config(args.domain, tier)
    except LookupError as error:
        return out.usage('run', str(error).strip('"\''), 'crane run --domain DOMAIN (see config/domains)')
    if not args.out:
        args.out = str(select.default_out(args.domain or Path(args.config).stem, tier))
    try:
        seeds = select.parse_seeds(args.seeds, args.domain)
        sets = overrides(args.set)
    except select.SelectError as error:
        return out.usage('run', str(error), 'crane run --help')
    cfg, base = fsconfig.load(config, fsconfig.merge(merged, sets) if sets else (merged or None))
    source = ScenarioSource(**cfg['scenarios'], base=base)
    if out.dry_run:
        plan = {'schema': 'crane.run.plan/1', 'domain': args.domain, 'tier': tier, 'config': str(config),
                'seeds': seeds, 'out': str(Path(args.out).expanduser()), 'variant': args.variant,
                'workers': args.workers or cfg.get('workers'), 'overrides': list(args.set),
                'scenarios': str(source.dir), 'generator': 'yes' if source.generate_cmd else 'no',
                'regenerate': bool(args.generate), 'allow_dirty': bool(args.allow_dirty)}
        if not out.doc(plan):
            out.say('crane run (dry run): nothing runs')
            out.say('\n'.join(output.table([(k, v if not isinstance(v, list) or len(v) <= 12 else
                                             f'{v[:6]} ... {len(v)} seeds') for k, v in plan.items()
                                            if k != 'schema'])))
            out.next(f'crane run {" ".join(_argv(args))}')
        return output.OK
    taken = [d for d in (Path(args.out).expanduser() / args.variant / f's{s}' for s in seeds)
             if d.is_dir() and any(d.iterdir())]
    if taken:
        where = Path(args.out).expanduser() / args.variant
        return out.usage('run', f'{where} already holds {len(taken)} of these runs ({taken[0].name}...); a run dir '
                         'is written once', 'pass a new --out, or omit --out for a stamped dir')
    missing = [s for s in seeds if not source.path(s).exists()]
    if (missing or args.generate) and not source.generate_cmd:
        what = f'{len(missing)} scenario file(s) missing in {source.dir}' if missing else '--generate: no generator'
        return out.usage('run', f'{what}, and the config has no `scenarios.generate` command',
                         f'write the scenarios into {source.dir} (pattern {source.pattern})')
    if args.generate:       # force: drop the requested seeds' files so the generator rewrites them
        for s in seeds:
            source.path(s).unlink(missing_ok=True)
    try:
        source.generate(seeds)
    except subprocess.CalledProcessError as error:
        detail = (error.stderr or b'').decode(errors='replace').strip()[-400:]
        return out.usage('run', f'the scenario generator failed ({error.returncode}): {detail}',
                         'run the scenarios.generate command by hand to see it fail')
    where = batch.adapter_source(cfg, base)
    adapter_git = identity.git_state(where) if where else None
    if adapter_git and adapter_git['dirty'] and not args.allow_dirty:
        return out.usage('run', f"{adapter_git['source']} has uncommitted changes", 'commit, or pass --allow-dirty')
    provenance = {'git': {'core': identity.git_state(Path(batch.__file__).parent), 'adapter': adapter_git},
                  'config_file': str(config), 'tier': tier, 'overrides': list(args.set)}
    started = time.perf_counter()
    try:
        results = batch.run_batch(cfg, base, seeds, args.out, args.variant, args.workers, provenance)
    except FileExistsError as error:      # a run dir written between the check above and its worker
        return out.usage('run', f'{str(error).partition(":")[0]} already holds a run',
                         'pass a new --out, or omit --out for a stamped dir')
    except FileNotFoundError as error:
        return out.usage('run', f'{error.filename or error} is missing', 'crane doctor')
    wall = time.perf_counter() - started
    rate = len(results) / wall if wall > 0 else 0.0
    if out.doc({'schema': 'crane.run.summary/1', 'tier': tier, 'domain': args.domain, 'config': str(config),
                'out': str(args.out), 'variant': args.variant, 'n': len(results), 'wall_s': round(wall, 3),
                'runs_per_s': round(rate, 3), 'fidelity': cfg['fidelity'],
                'outcomes': dict(Counter(r.outcome for r in results)),
                'runs': [{'out': str(r.out), 'outcome': r.outcome, 'sim_s': r.sim_s, 'wall_s': r.wall_s}
                         for r in results]}):
        return output.OK
    for r in results:
        out.line(f'{r.out}: {r.outcome} in {r.sim_s:.0f} s sim ({r.wall_s:.2f} s wall)')
    out.say(f'{len(results)} runs at {tier}, {wall:.1f} s wall: {rate:.1f} runs/s')
    out.say(f'fidelity: {cfg["fidelity"]}')
    out.next(f'crane board {args.out}')
    return output.OK


def _argv(args):
    """The command line of a dry run without --dry-run (the one to run next): every argument given."""
    words = [f'--domain {args.domain}' if args.domain else f'--config {shlex.quote(str(args.config))}']
    words += [f'--tier {args.tier}'] if args.tier else []
    words += [f'--seeds {shlex.quote(str(args.seeds))}', f'--out {shlex.quote(str(args.out))}']
    words += [f'--variant {shlex.quote(args.variant)}'] if args.variant != 'base' else []
    words += [f'--workers {args.workers}'] if args.workers else []
    words += [f'--set {shlex.quote(s)}' for s in args.set]
    words += ['--generate'] if args.generate else []
    words += ['--allow-dirty'] if args.allow_dirty else []
    return words
