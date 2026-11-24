"""The shared selectors every crane command reads the same way: domain, tier, env, robot, card, seeds, sets, out.

    parse_seeds('0:4')               -> [0, 1, 2, 3]
    parse_seeds('block:dev', 'boat') -> the seeds config/domains/boat.yaml names under seeds.dev
    parse_sets(['a.b=1'])            -> {'a': {'b': 1}}
    default_out('boat', 'T1', key)   -> ~/.cache/crane/runs/boat/T1/<key>

Stdlib only at import (YAML is read lazily), so `crane sim` and `crane help` need nothing installed.
"""

import os
import time
from pathlib import Path

from crane import paths, tiers

# One spec per selector; crane/commands.py turns these into params (argparse, help.yaml, Docs/CLI.md).
SPECS = {
    'domain': dict(name='--domain',
                   description='A domain: config/domains/DOMAIN.yaml.'),
    'tier': dict(name='--tier', type='tier', choices=('T0', 'T1', 'T2', 'abstract', 'dynamic', 'unity'),
                 description='The tier: T0 abstract, T1 dynamic, T2 unity.'),
    'env': dict(name='--env', metavar='KIND:NAME', type='env',
                description='The world: course:NAME, scene:NAME or world:NAME.'),
    'robot': dict(name='--robot', description="The domain's robot."),
    'card': dict(name='--card', metavar='REF', description='A plant card ref or a profile JSON.'),
    'seed': dict(name='--seed', type='int', description='One seed.'),
    'seeds': dict(name='--seeds', metavar='SPEC',
                  description='Seeds: N, A:B (B exclusive), a,b,c, @FILE or block:NAME.'),
    'set': dict(name='--set', metavar='KEY=VALUE', repeat=True,
                description='Override a config key; dotted keys nest, the value is YAML (repeatable).'),
    'out': dict(name='--out', type='path', description='Output dir (default ~/.cache/crane/runs/DOMAIN/TIER/STAMP).'),
}

RUNS_ROOT = Path.home() / '.cache' / 'crane' / 'runs'


class SelectError(ValueError):
    """A selector that cannot be read; its message names the fix."""


def parse_seeds(text, domain=None, blocks=None, base=None):
    """A seed spec to a list of ints: 'N', 'A:B' (B exclusive), 'a,b,c', '@FILE' (whitespace- or comma-separated
    specs, '#' comments; FILE relative to ``base``, a file named inside a file relative to that file) or
    'block:NAME' (blocks, else seeds.NAME in config/domains/DOMAIN.yaml). Order is kept, duplicates are refused (a
    repeated seed would pair a run with itself). SelectError (a ValueError) names the fix."""
    seeds = _seeds(str(text).strip(), domain, blocks, Path(base or '.'), depth=0)
    seen = set()
    dup = [s for s in seeds if s in seen or seen.add(s)]
    if dup:
        raise SelectError(f'seed {dup[0]} appears twice in {text!r}')
    return seeds


def _seeds(text, domain, blocks, base, depth):
    if depth > 8:
        raise SelectError(f'seed spec {text!r} nests too deep (@file or block: inside each other)')
    if not text:
        raise SelectError('empty seed spec; give N, A:B, a,b,c, @FILE or block:NAME')
    if ',' in text:
        return [s for part in text.split(',') if part.strip() for s in _seeds(part.strip(), domain, blocks, base,
                                                                              depth + 1)]
    if text.startswith('@'):
        path = base / Path(text[1:]).expanduser()
        try:
            body = path.read_text()
        except OSError as error:
            raise SelectError(f'seed file {path}: {error.strerror or error}') from None
        words = [w for line in body.splitlines() for w in line.split('#', 1)[0].replace(',', ' ').split()]
        if not words:
            raise SelectError(f'seed file {path} names no seeds')
        return [s for w in words for s in _seeds(w, domain, blocks, path.parent, depth + 1)]
    if text.startswith('block:'):
        name = text[6:]
        table = blocks if blocks is not None else seed_blocks(domain)
        if name not in table:
            where = f'config/domains/{domain}.yaml seeds:' if domain else 'a --domain whose YAML has seeds:'
            known = ', '.join(sorted(table)) or 'none'
            raise SelectError(f'no seed block {name!r} in {where} (blocks: {known})')
        spec = table[name]
        spec = spec.get('seeds') if isinstance(spec, dict) else spec
        if isinstance(spec, (list, tuple, range)):
            return [s for x in spec for s in _seeds(str(x), domain, blocks, base, depth + 1)]
        return _seeds(str(spec), domain, blocks, base, depth + 1)
    try:
        lo, colon, hi = text.partition(':')
        if colon:
            lo, hi = int(lo), int(hi)
            if hi <= lo:
                raise SelectError(f'seed range {text!r} is empty (A:B takes B exclusive, B > A)')
            return list(range(lo, hi))
        return [int(text)]
    except ValueError as error:
        if isinstance(error, SelectError):
            raise
        raise SelectError(f'bad seed spec {text!r}; give N, A:B, a,b,c, @FILE or block:NAME') from None


def seed_blocks(domain):
    """{name: spec} from config/domains/DOMAIN.yaml `seeds:` ({} without a domain or a seeds section)."""
    if not domain:
        return {}
    return dict(domain_config(domain).get('seeds') or {})


def block_role(domain, name):
    """The tag (dev, check, train, calib, held-out) of a seed block, or None."""
    spec = seed_blocks(domain).get(name)
    return spec.get('role') if isinstance(spec, dict) else None


def domains():
    return sorted(p.stem for p in paths.DOMAINS.glob('*.yaml'))


def domain_config(domain):
    path = paths.DOMAINS / f'{domain}.yaml'
    if not path.is_file():
        raise SelectError(f"no domain {domain!r}; domain configs: {', '.join(domains()) or 'none'}")
    from crane import miniyaml
    return miniyaml.safe_load(path.read_text()) or {}


def parse_tier(text):
    """'T1' | 'dynamic' | '1' -> 'T1' (SelectError names the choices)."""
    try:
        return tiers.normalise(text)
    except ValueError as error:
        raise SelectError(str(error)) from None


def parse_env(text):
    """'course:roboboat_2026' -> ('course', 'roboboat_2026')."""
    kind, colon, name = str(text).partition(':')
    if not colon or not kind or not name:
        raise SelectError(f'--env wants KIND:NAME (course:NAME, scene:NAME, world:NAME), got {text!r}')
    return kind, name


def parse_sets(pairs, what='--set'):
    """['a.b=1', 'c=x'] -> {'a': {'b': 1}, 'c': 'x'}; each value is parsed as YAML."""
    out = {}
    for pair in pairs or ():
        key, eq, value = str(pair).partition('=')
        if not eq or not key:
            raise SelectError(f'{what} wants KEY=VALUE, got {pair!r}')
        if value.strip() == '':
            leaf_value = ''
        else:
            import yaml
            leaf_value = yaml.safe_load(value)
        *parents, leaf = key.split('.')
        node = out
        for part in parents:
            node = node.setdefault(part, {})
            if not isinstance(node, dict):
                raise SelectError(f'{what} {pair!r}: {part} is already set to a value')
        node[leaf] = leaf_value
    return out


def stamp():
    return time.strftime('%Y%m%d-%H%M%S')


def default_out(domain=None, tier=None, root=None, key=None):
    """~/.cache/crane/runs/DOMAIN/TIER/KEY (CRANE_RUNS overrides the root). Never inside the repo. KEY names what
    was run (e.g. a digest of the config and overrides), so the same command reruns into the same place and
    resumes; without a key it is a time stamp."""
    base = Path(root or os.environ.get('CRANE_RUNS') or RUNS_ROOT).expanduser()
    return base / (domain or 'custom') / (tier or 'any') / (key or stamp())

