"""Which sim to start: a named sim (config/sims/NAME.yaml, or the user's ~/.config/crane/sims/NAME.yaml), this
machine's settings (~/.config/crane/crane.yaml) and the flags, resolved into one recipe with the source of each value.

    r = recipe.resolve('roboboat-high', {'seed': 3})
    r.values['tier'], r.values['profile'], r.sources['profile']   # 'T2', 'interactive-high', 'fidelity high (sim roboboat-high)'

The layers, lowest first: the domain's defaults, the sim file, this machine's facts (display, player, stack), then
the flags. Which sim: the SIM word, else CRANE_SIM, else this machine's ``sim.default``, else none (the domain's
defaults alone). ``--domain`` without a SIM word drops the machine's default sim (it may be another domain's).

A sim file names a domain, a course and a fidelity, and may pin any other up setting:

    about: RoboBoat 2026 course, full-quality Unity
    domain: boat
    course: roboboat_2026
    fidelity: high            # a rung of the domain's fidelity ladder (config/domains/D.yaml fidelity:)

Stdlib only (crane sim's paths need no PyYAML).
"""

import hashlib
import json
import os
from pathlib import Path

from crane import miniyaml, paths

# The settings a sim file, this machine's config or a flag may give (the flag's dest names).
KEYS = ('domain', 'env', 'seed', 'fidelity', 'tier', 'profile', 'stack', 'display', 'card', 'current', 'robot',
        'sets', 'nav', 'odom', 'rate', 'world', 'player')
MACHINE_KEYS = ('display', 'player', 'stack')     # facts about this machine, not about the world
IDENTITY_SKIP = ('display', 'player', 'fidelity')  # a sim is the same world whatever window it is drawn in
FIDELITY = ('twin', 'low', 'high', 'eval')         # the shared rung names; each domain maps them


class RecipeError(ValueError):
    """A sim or setting that cannot be resolved; the message names the fix."""


def user_dir():
    return Path(os.environ.get('HOME') or Path.home()) / '.config' / 'crane'


def machine_path():
    return Path(os.environ.get('CRANE_CONFIG') or user_dir() / 'crane.yaml')


def _load(path):
    try:
        doc = miniyaml.safe_load(Path(path).read_text()) or {}
    except OSError:
        return {}
    except ValueError as error:
        raise RecipeError(f'{path}: {error}') from None
    if not isinstance(doc, dict):
        raise RecipeError(f'{path}: want a mapping of settings')
    return doc


def machine():
    """This machine's settings: the ``sim:`` block of ~/.config/crane/crane.yaml ({} without one)."""
    return dict((_load(machine_path()).get('sim') or {}))


def sims():
    """{name: (doc, path)}: the repo's config/sims, then the user's ~/.config/crane/sims (a user file wins)."""
    out = {}
    for root in (paths.CONFIG / 'sims', user_dir() / 'sims'):
        for path in sorted(root.glob('*.yaml')) if root.is_dir() else ():
            out[path.stem] = (_load(path), path)
    return out


def load_sim(name):
    found = sims()
    if name not in found:
        known = ', '.join(sorted(found)) or 'none'
        raise RecipeError(f'no sim {name!r}; sims: {known} (crane sim list)')
    return found[name]


def default_name(flags=None):
    """The sim a bare up starts: CRANE_SIM, else this machine's sim.default, else None."""
    flags = flags or {}
    if flags.get('domain'):
        return None
    return os.environ.get('CRANE_SIM') or machine().get('default') or None


def ladder(domain_cfg):
    """{rung: {tier, profile, about}} of the domain's fidelity ladder, and its default rung."""
    spec = dict(domain_cfg.get('fidelity') or {})
    default = spec.pop('default', None)
    return {str(k): dict(v or {}) for k, v in spec.items()}, default


class Recipe:
    """The resolved settings (values) and where each came from (sources)."""

    def __init__(self, name, values, sources, about=''):
        self.name, self.values, self.sources, self.about = name, values, sources, about

    def identity(self):
        """A short hash of the world this recipe starts: the same hash is the same running sim."""
        doc = {k: v for k, v in sorted(self.values.items()) if k not in IDENTITY_SKIP and v not in (None, [], {})}
        return hashlib.sha256(json.dumps(doc, sort_keys=True, default=str).encode()).hexdigest()[:12]

    def as_dict(self):
        return {'schema': 'crane.sim.recipe/1', 'sim': self.name, 'about': self.about, 'identity': self.identity(),
                'values': self.values, 'sources': self.sources}

    def lines(self):
        width = max(len(k) for k in self.values) if self.values else 0
        head = [f'sim {self.name}' + (f': {self.about}' if self.about else '') if self.name else
                'sim (none named: the domain defaults)']
        return head + [f'  {k.ljust(width)}  {_text(v)}' + (f'   ({self.sources[k]})' if k in self.sources else '')
                       for k, v in self.values.items()]


def _text(v):
    if isinstance(v, (list, tuple)):
        return ' '.join(str(x) for x in v) or '-'
    if isinstance(v, dict):
        return ' '.join(f'{k}:={x}' for k, x in v.items()) or '-'
    return '-' if v is None else str(v)


def _course_env(doc):
    """A sim file's or flag set's world: course NAME is env course:NAME."""
    if doc.get('course') and doc.get('env'):
        raise RecipeError('give course or env, not both')
    return f"course:{doc['course']}" if doc.get('course') else doc.get('env')


def resolve(name=None, flags=None, load_domain=None):
    """The recipe for sim NAME (None: the default sim, see default_name) with FLAGS ({dest: value}, None = not given)."""
    from crane import player
    flags = {k: v for k, v in (flags or {}).items() if v not in (None, [], ())}
    load_domain = load_domain or player.load_domain
    name = name or default_name(flags)
    values, sources = {}, {}

    def put(key, value, where):
        if value not in (None, [], (), {}):
            values[key], sources[key] = value, where

    doc, about = {}, ''
    if name:
        doc, path = load_sim(name)
        about = str(doc.get('about') or '')
        unknown = sorted(set(doc) - set(KEYS) - {'about', 'course'})
        if unknown:
            raise RecipeError(f'{path}: unknown setting {unknown[0]!r}; settings: course, {", ".join(KEYS)}')
    flags_env = _course_env(flags)
    doc_env = _course_env(doc)
    domain = flags.get('domain') or doc.get('domain')
    if not domain:
        from crane.sim import config
        domain = config.default_domain()
        put('domain', domain, 'config/sim.yaml')
    else:
        put('domain', domain, 'flag' if flags.get('domain') else f'sim {name}')
    try:
        cfg = load_domain(domain)
    except player.ConfigError as error:
        raise RecipeError(str(error)) from None

    # 1. the domain's defaults
    put('env', (cfg.get('env') or {}).get('default'), f'domain {domain}')
    rungs, rung_default = ladder(cfg)
    # 2. the sim file; 3. this machine; 4. the flags
    mine = machine()
    for key in KEYS:
        if key == 'domain':
            continue
        if key == 'env':
            put('env', doc_env, f'sim {name}')
            put('env', flags_env, 'flag')
            continue
        if key == 'nav':                                 # nav arguments merge: the flags' over the sim's
            merged = {**dict(doc.get('nav') or {}), **dict(flags.get('nav') or {})}
            put('nav', {str(k): str(v) for k, v in merged.items()}, 'flag' if flags.get('nav') else f'sim {name}')
            continue
        put(key, doc.get(key), f'sim {name}')
        if key in MACHINE_KEYS:
            put(key, mine.get(key), str(machine_path()).replace(str(Path.home()), '~'))
        put(key, flags.get(key), 'flag')
    if 'fidelity' not in values and rung_default and not flags.get('tier'):
        put('fidelity', rung_default, f'domain {domain} default')

    # the fidelity rung gives the tier and profile; an explicit tier or profile (sim file or flag) wins over it
    rung = values.get('fidelity')
    if rung:
        if rung not in rungs:
            known = ', '.join(rungs) or 'none (the domain has no fidelity ladder)'
            raise RecipeError(f'domain {domain} has no fidelity {rung!r}; fidelity: {known}')
        where = f"fidelity {rung} ({sources['fidelity']})"
        for key in ('tier', 'profile'):
            if key not in values and rungs[rung].get(key):
                put(key, rungs[rung][key], where)
    values.setdefault('tier', 'T2')
    sources.setdefault('tier', 'default')
    from crane import tiers
    try:
        values['tier'] = tiers.normalise(values['tier'])
    except ValueError as error:
        raise RecipeError(str(error)) from None
    if values['tier'] != 'T2' and sources.get('profile', 'flag') != 'flag':
        values.pop('profile', None)                      # a T1 world runs no player: a rung's profile does not apply
        sources.pop('profile', None)
    if values['tier'] == 'T2':
        for key in ('odom', 'rate', 'world'):
            if key in values and sources[key] != 'flag':
                values.pop(key)                          # T1-only pins of a sim file do not reach a T2 override
                sources.pop(key)
    order = {k: i for i, k in enumerate(('domain', 'env', 'fidelity', 'tier', 'profile') + KEYS)}
    values = dict(sorted(values.items(), key=lambda kv: order.get(kv[0], 99)))
    return Recipe(name, values, sources, about)


def save_machine(updates):
    """Merge UPDATES into the ``sim:`` block of this machine's config (other blocks and keys kept); the path."""
    path = machine_path()
    doc = _load(path)
    block = dict(doc.get('sim') or {})
    for k, v in updates.items():
        if v is None:
            block.pop(k, None)
        else:
            block[k] = v
    doc['sim'] = block
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix('.tmp')
    tmp.write_text('# crane on this machine (crane sim use SIM writes the sim block). crane sim show explains each value.\n'
                   + _dump(doc))
    tmp.replace(path)
    return path


def _scalar(v):
    if isinstance(v, bool):
        return 'true' if v else 'false'
    if v is None:
        return 'null'
    text = str(v)
    if isinstance(v, (int, float)) or text.replace('-', '').replace('_', '').replace('.', '').isalnum() \
            and text not in ('true', 'false', 'null', 'yes', 'no', 'on', 'off') and not text[:1].isdigit():
        return text
    return json.dumps(text)


def _dump(doc, indent=0):
    """A small YAML writer for nested mappings of scalars and flat lists (what crane.yaml holds)."""
    out = []
    for k, v in doc.items():
        pad = ' ' * indent
        if isinstance(v, dict):
            out.append(f'{pad}{k}:')
            out.append(_dump(v, indent + 2).rstrip('\n'))
        elif isinstance(v, (list, tuple)):
            out.append(f"{pad}{k}: [{', '.join(_scalar(x) for x in v)}]")
        else:
            out.append(f'{pad}{k}: {_scalar(v)}')
    return '\n'.join(line for line in out if line) + '\n'
