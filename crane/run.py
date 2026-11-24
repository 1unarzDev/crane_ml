"""crane run: episodes at a tier. T0 runs the fastsim batch runner (fastsim.batch.run_batch)."""

import subprocess
import sys
import time
from pathlib import Path

from crane import paths, tiers



def tier_config(domain, tier):
    """(tier, config path, overrides) of DOMAIN's TIER (None: its first tier): bench.fast's reading of
    config/domains/DOMAIN.yaml, so `crane run` and `bench.fast` run the same thing."""
    if str(paths.ROOT) not in sys.path:
        sys.path.insert(0, str(paths.ROOT))
    from bench.fast import domain_tier
    return domain_tier(domain, tier)


def overrides(pairs):
    """['a.b=1', 'c=x'] -> {'a': {'b': 1}, 'c': 'x'}; each value is parsed as YAML."""
    import yaml
    out = {}
    for pair in pairs:
        key, eq, value = pair.partition('=')
        if not eq or not key:
            raise SystemExit(f'crane run: --set wants KEY=VALUE, got {pair!r}')
        *parents, leaf = key.split('.')
        node = out
        for part in parents:
            node = node.setdefault(part, {})
        node[leaf] = yaml.safe_load(value)
    return out


def main(args):
    if getattr(args, 'astro_test', None):
        from crane import astrotest
        return astrotest.main(args)
    if not args.seeds or not args.out:
        print('crane run: --seeds and --out are required (except with --astro-test)', file=sys.stderr)
        return 2
    try:
        tier = tiers.normalise(args.tier) if args.tier else None
    except ValueError as error:
        print(f'crane run: {error}', file=sys.stderr)
        return 2
    if tier == 'T2':
        print('crane run: --tier T2 (unity) is not yet available; use `crane sim up --headless` for the T2 world.',
              file=sys.stderr)
        return 2
    from fastsim import batch, config as fsconfig, identity
    from fastsim.scenario import ScenarioSource
    try:
        if args.config:
            tier, config, merged = tier or 'T0', Path(args.config).resolve(), {}
        else:
            tier, config, merged = tier_config(args.domain, tier)
    except LookupError as error:
        print(f'crane run: {error}', file=sys.stderr)
        return 2
    seeds = batch.parse_seeds(args.seeds)
    cfg, base = fsconfig.load(config, fsconfig.merge(merged, overrides(args.set)) if args.set else (merged or None))
    source = ScenarioSource(**cfg['scenarios'], base=base)
    missing = [s for s in seeds if not source.path(s).exists()]
    if (missing or args.generate) and not source.generate_cmd:
        what = f'{len(missing)} scenario file(s) missing in {source.dir}' if missing else f'--generate: no generator'
        print(f'crane run: {what}, and the config has no `scenarios.generate` command; write the scenarios into '
              f'{source.dir} (pattern {source.pattern})', file=sys.stderr)
        return 2
    if args.generate:       # force: drop the requested seeds' files so the generator rewrites them
        for s in seeds:
            source.path(s).unlink(missing_ok=True)
    try:
        source.generate(seeds)
    except subprocess.CalledProcessError as error:
        print(f'crane run: the scenario generator failed ({error.returncode}): '
              f'{(error.stderr or b"").decode(errors="replace").strip()[-400:]}', file=sys.stderr)
        return 2
    where = batch.adapter_source(cfg, base)
    adapter_git = identity.git_state(where) if where else None
    if adapter_git and adapter_git['dirty'] and not args.allow_dirty:
        print(f"crane run: {adapter_git['source']} has uncommitted changes; commit, or pass --allow-dirty",
              file=sys.stderr)
        return 2
    provenance = {'git': {'core': identity.git_state(Path(batch.__file__).parent), 'adapter': adapter_git},
                  'config_file': str(config), 'tier': tier, 'overrides': list(args.set)}
    started = time.perf_counter()
    try:
        results = batch.run_batch(cfg, base, seeds, args.out, args.variant, args.workers, provenance)
    except FileNotFoundError as error:
        print(f'crane run: {error.filename or error} is missing', file=sys.stderr)
        return 2
    wall = time.perf_counter() - started
    for r in results:
        print(f'{r.out}: {r.outcome} in {r.sim_s:.0f} s sim ({r.wall_s:.2f} s wall)')
    print(f'{len(results)} runs at {tier}, {wall:.1f} s wall: {len(results) / wall:.1f} runs/s')
    print(f'fidelity: {cfg["fidelity"]}')
    return 0
