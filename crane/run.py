"""crane run: episodes at a tier. T0 runs the fastsim batch runner (fastsim.batch.run_batch)."""

import sys
import time
from pathlib import Path

from crane import paths, tiers

ARRIVES = {'T1': 'P2 (the boat reference mission on the batched plant)', 'T2': 'P3 (run_batch on the Unity player)'}


def domain_config(domain, tier):
    """The fastsim config of DOMAIN at TIER: config/domains/DOMAIN.yaml's tiers.<TIER>.config, relative to it."""
    import yaml
    path = paths.DOMAINS / f'{domain}.yaml'
    if not path.is_file():
        known = sorted(p.stem for p in paths.DOMAINS.glob('*.yaml'))
        listed = ', '.join(known) if known else f'none yet in {paths.DOMAINS}'
        raise LookupError(f'no domain {domain!r} ({path} is missing); domain configs: {listed}')
    spec = (yaml.safe_load(path.read_text()) or {}).get('tiers', {}).get(tier, {})
    if not spec.get('config'):
        raise LookupError(f'{path} names no {tier} config (tiers.{tier}.config)')
    return (path.parent / spec['config']).resolve()


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
    try:
        tier = tiers.normalise(args.tier)
    except ValueError as error:
        print(f'crane run: {error}', file=sys.stderr)
        return 2
    if tier != 'T0':
        print(f'crane run: --tier {tier} ({tiers.NAMES[tier]}) arrives in {ARRIVES[tier]}.', file=sys.stderr)
        return 2
    from fastsim import batch, config as fsconfig, identity
    from fastsim.scenario import ScenarioSource
    try:
        config = Path(args.config).resolve() if args.config else domain_config(args.domain, tier)
    except LookupError as error:
        print(f'crane run: {error}', file=sys.stderr)
        return 2
    seeds = batch.parse_seeds(args.seeds)
    cfg, base = fsconfig.load(config, overrides(args.set) or None)
    if args.generate:
        ScenarioSource(**cfg['scenarios'], base=base).generate(seeds)
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
        print(f'crane run: {error.filename or error} is missing (write the scenarios, or --generate when the '
              "config's scenario source has a generator)", file=sys.stderr)
        return 2
    wall = time.perf_counter() - started
    for r in results:
        print(f'{r.out}: {r.outcome} in {r.sim_s:.0f} s sim ({r.wall_s:.2f} s wall)')
    print(f'{len(results)} runs at {tier}, {wall:.1f} s wall: {len(results) / wall:.1f} runs/s. {cfg["fidelity"]}')
    return 0
