"""crane run: episodes at a tier. T0 runs the fastsim batch runner (fastsim.batch.run_batch)."""

import json
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
    try:
        seeds = select.parse_seeds(args.seeds, args.domain)
        sets = overrides(args.set)
    except select.SelectError as error:
        return out.usage('run', str(error), 'crane run --help')
    try:
        card = card_sets(args.domain, getattr(args, 'card', None))
    except CardError as error:
        return out.usage('run', *error.args)
    if card:
        sets = fsconfig.merge(sets, card[1]) if sets else card[1]
    if not args.out:   # one place per (config, overrides): the same command resumes, a change goes elsewhere
        from crane.ids import digest
        key = f"{getattr(args, 'variant', None) or 'base'}-{digest([str(config), merged, sets])[:8]}"
        args.out = str(select.default_out(args.domain or Path(args.config).stem, tier, key=key))
    cfg, base = fsconfig.load(config, fsconfig.merge(merged, sets) if sets else (merged or None))
    source = ScenarioSource(**cfg['scenarios'], base=base)
    if out.dry_run:
        plan = {'schema': 'crane.run.plan/1', 'domain': args.domain, 'tier': tier, 'config': str(config),
                'seeds': seeds, 'out': str(Path(args.out).expanduser()), 'variant': args.variant,
                'workers': args.workers or cfg.get('workers'), 'overrides': list(args.set),
                'scenarios': str(source.dir), 'generator': 'yes' if source.generate_cmd else 'no',
                'regenerate': bool(args.generate), 'allow_dirty': bool(args.allow_dirty),
                'force': bool(getattr(args, 'force', False)), 'card': str(card[0]) if card else None}
        if not out.doc(plan):
            out.say('crane run (dry run): nothing runs')
            out.say('\n'.join(output.table([(k, v if not isinstance(v, list) or len(v) <= 12 else
                                             f'{v[:6]} ... {len(v)} seeds') for k, v in plan.items()
                                            if k != 'schema'])))
            out.next(f'crane run {" ".join(_argv(args))}')
        return output.OK
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
                  'config_file': str(config), 'tier': tier, 'overrides': list(args.set),
                  **({'card': str(card[0])} if card else {})}
    engine = getattr(args, 'engine', 'fastsim')
    adapter = batch.adapter_of(cfg, base)
    engines, backends = getattr(adapter, 'engines', None), getattr(adapter, 'backends', None)
    rest = [w for w in _argv(args) if not w.startswith(('--engine ', '--backend '))]
    if engines and engine not in engines:
        return out.usage('run', f'--engine {engine}: {args.domain or config} runs on the {"/".join(engines)} '
                         'engine only', f'crane run {" ".join(rest)} --engine {engines[0]}')
    if engine == 'batched' and backends and args.backend not in backends:
        return out.usage('run', f'--backend {args.backend}: {args.domain or config} runs on {"/".join(backends)} '
                         'only', f'crane run {" ".join(rest)} --engine batched --backend {backends[0]}')
    if engine == 'batched':
        from engine.runs import run_seeds
        started = time.perf_counter()
        try:
            rows, engine_s = run_seeds(cfg, base, seeds, args.out, args.variant, args.backend, args.dtype,
                                       args.record, {**provenance, 'engine': 'batched', 'backend': args.backend})
        except (NotImplementedError, FileExistsError) as error:
            return out.usage('run', str(error))
        wall = time.perf_counter() - started
        if out.doc({'schema': 'crane.run.summary/1', 'tier': tier, 'domain': args.domain, 'config': str(config),
                    'out': str(args.out), 'variant': args.variant, 'n': len(rows), 'wall_s': round(wall, 3),
                    'runs_per_s': round(len(rows) / wall, 3) if wall > 0 else 0.0, 'reused': 0,
                    'engine': {'name': 'batched', 'backend': args.backend, 'dtype': args.dtype or 'float64',
                               'episodes_s': round(engine_s, 3)}, 'fidelity': cfg['fidelity'],
                    'outcomes': dict(Counter(r[1] for r in rows)),
                    'runs': [{'out': str(d), 'outcome': o, 'sim_s': t} for _, o, t, d in rows]}):
            return output.OK
        for seed, outcome, sim_s, d in rows:
            out.line(f'{d}: {outcome} in {sim_s:.0f} s sim')
        out.say(f'{len(rows)} runs at {tier} on the batched engine ({args.backend}), {wall:.1f} s wall '
                f'({engine_s:.1f} s episodes): {len(rows) / wall:.1f} runs/s')
        out.say(f'fidelity: {cfg["fidelity"]}')
        out.next(f'crane board {args.out}')
        return output.OK
    started = time.perf_counter()
    tracker = batch._progress(True, args.out, args.variant, len(seeds))
    if tracker and out.json:
        tracker.stream = sys.stderr          # the minute line stays off a --json stdout
    try:
        results = batch.run_batch(cfg, base, seeds, args.out, args.variant, args.workers, provenance,
                                  force=args.force, progress=tracker or False)
    except batch.RunDirConflict as error:      # another identity or an incomplete dir: one line, the fix in it
        return out.usage('run', str(error))
    except FileNotFoundError as error:
        return out.usage('run', f'{error.filename or error} is missing', 'crane doctor')
    wall = time.perf_counter() - started
    reused = tracker.skipped if tracker else sum(r.wall_s == 0.0 for r in results)
    rate = (len(results) - reused) / wall if wall > 0 else 0.0
    if out.doc({'schema': 'crane.run.summary/1', 'tier': tier, 'domain': args.domain, 'config': str(config),
                'out': str(args.out), 'variant': args.variant, 'n': len(results), 'wall_s': round(wall, 3),
                'runs_per_s': round(rate, 3), 'reused': reused, 'fidelity': cfg['fidelity'],
                'outcomes': dict(Counter(r.outcome for r in results)),
                'runs': [{'out': str(r.out), 'outcome': r.outcome, 'sim_s': r.sim_s, 'wall_s': r.wall_s}
                         for r in results]}):
        return output.OK
    for r in results:
        out.line(f'{r.out}: {r.outcome} in {r.sim_s:.0f} s sim ({r.wall_s:.2f} s wall)')
    ran = len(results) - reused
    out.say(f'{ran} runs at {tier}, {wall:.1f} s wall: {rate:.1f} runs/s'
            + (f', {reused} reused (same run identity)' if reused else ''))
    out.say(f'fidelity: {cfg["fidelity"]}')
    out.next(f'crane board {args.out}')
    return output.OK


class CardError(ValueError):
    """(message, fix) of a --card crane run cannot bind."""


def card_sets(domain, refs):
    """(card file, {key: path} overrides) for --card REF outside --astro-test: the domain's card.keys
    (config/domains/D.yaml) all bound to the card's file; None without --card. CardError (message, fix) when
    the domain binds no card or REF names no file."""
    refs = [r for r in (refs or ()) if r]
    if not refs:
        return None
    if len(refs) > 1:
        raise CardError('--card: one card per run (several only with --astro-test)',
                        'a study with one arm per card (crane study)')
    if not domain:
        raise CardError('--card needs --domain (the domain names the keys a card binds)', '--set KEY=PATH')
    cfg = select.domain_config(domain)
    keys, folder = (cfg.get('card') or {}).get('keys'), (cfg.get('card') or {}).get('dir')
    if not keys:
        raise CardError(f'--card: domain {domain} binds no card in crane run',
                        f'add card: {{keys: [config.key]}} to config/domains/{domain}.yaml, or --set KEY=PATH')
    ref = str(refs[0])
    candidates = [Path(ref).expanduser(), paths.ROOT / ref]
    if folder:
        candidates += [paths.ROOT / folder / ref, paths.ROOT / folder / f'{ref}.json']
    found = next((c.resolve() for c in candidates if c.is_file()), None)
    if found is None:
        raise CardError(f'--card {ref}: no such file', f'a profile JSON path, or a name in {folder or "the card dir"}')
    return found, overrides([f'{k}={json.dumps(str(found))}' for k in keys])


def _argv(args):
    """The command line of a dry run without --dry-run (the one to run next): every argument given."""
    words = [f'--domain {args.domain}' if args.domain else f'--config {shlex.quote(str(args.config))}']
    words += [f'--tier {args.tier}'] if args.tier else []
    words += [f'--seeds {shlex.quote(str(args.seeds))}', f'--out {shlex.quote(str(args.out))}']
    words += [f'--variant {shlex.quote(args.variant)}'] if args.variant != 'base' else []
    words += [f'--workers {args.workers}'] if args.workers else []
    words += [f'--set {shlex.quote(s)}' for s in args.set]
    words += [f'--card {shlex.quote(str(c))}' for c in getattr(args, 'card', None) or ()]
    words += ['--generate'] if args.generate else []
    words += ['--allow-dirty'] if args.allow_dirty else []
    words += ['--force'] if getattr(args, 'force', False) else []
    if getattr(args, 'engine', 'fastsim') != 'fastsim':
        words += [f'--engine {args.engine}', f'--backend {args.backend}']
        words += [f'--dtype {args.dtype}'] if args.dtype else []
        words += [f'--record {args.record}'] if args.record else []
    return words
