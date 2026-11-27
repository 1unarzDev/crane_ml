"""The software beside a sim's world: a stack lays its T2 and T1 sections over the domain's tiers.T2 and tiers.T1.sim.

    cfg = stacks.apply(player.load_domain('boat'), stacks.choose(cfg, 'auto'))

- astro: the robot team's own stack (the domain's ``stacks.astro``: its container, launch, mounts, the host tooling).
- bridge: crane's own (config/stacks/bridge.yaml, under the domain's ``stacks.bridge``): the ROS-TCP endpoint the
  player talks to, or the T1 node alone; bring your own nodes.
- none: the T2 player alone, no ROS side.

``auto`` is the domain's ``stacks.default`` resolved on this machine: astro when its checkout is here, else bridge.
"""

import copy
from pathlib import Path

from crane import miniyaml, paths

BUILTIN = ('astro', 'bridge', 'none')


class StackError(ValueError):
    pass


def _spec(cfg):
    return dict(cfg.get('stacks') or {})


def allowed(cfg):
    spec = _spec(cfg)
    return [str(s) for s in spec.get('allowed') or [k for k in spec if k not in ('default', 'allowed')]]


def _file(name):
    path = paths.CONFIG / 'stacks' / f'{name}.yaml'
    try:
        return miniyaml.safe_load(path.read_text()) or {}
    except OSError:
        return {}


def sections(cfg, name):
    """The stack's {T2: {...}, T1: {...}}: config/stacks/NAME.yaml with the domain's stacks.NAME over it."""
    base = _file(name)
    mine = _spec(cfg).get(name) or {}
    return {t: {**(base.get(t) or {}), **(mine.get(t) or {})} for t in ('T2', 'T1')}


def astro_here(cfg):
    """The astro stack's host checkout exists on this machine."""
    from crane.sim import config
    t2 = sections(cfg, 'astro')['T2']
    if not t2.get('host'):
        return False
    d = config.dock(t2)
    return bool(d and Path(d).is_dir())


def choose(cfg, wanted=None):
    """(stack, why) for WANTED (None or auto: the domain's default, resolved on this machine)."""
    names = allowed(cfg)
    if not names:
        return None, 'the domain names no stacks'
    wanted = wanted or str(_spec(cfg).get('default') or 'auto')
    if wanted == 'auto':
        if 'astro' in names and astro_here(cfg):
            return 'astro', 'auto: the astro checkout is here'
        if 'bridge' in names:
            return 'bridge', 'auto: no astro checkout here'
        return names[0], 'auto: the domain\'s first'
    if wanted not in names:
        raise StackError(f"domain {cfg.get('name', '?')} has no stack {wanted!r}; stacks: auto, {', '.join(names)}")
    return wanted, None


def apply(cfg, name):
    """CFG with stack NAME laid over tiers.T2 and tiers.T1.sim (a copy; a domain without stacks is returned as is)."""
    if not name or not _spec(cfg):
        return cfg
    out = copy.deepcopy(cfg)
    tiers = out.setdefault('tiers', {})
    sec = sections(cfg, name)
    if name == 'none':
        sec = {'T2': {'ros': {}, 'host': {}, 'next_up': [], 'next_down': []}, 'T1': {}}
    if tiers.get('T2'):
        tiers['T2'] = {**tiers['T2'], **sec['T2'], 'stack': name}
    t1 = (tiers.get('T1') or {}).get('sim')
    if t1 is not None:
        tiers['T1']['sim'] = {**t1, **sec['T1'], 'stack': name}
    return out
