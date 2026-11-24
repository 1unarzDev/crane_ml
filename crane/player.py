"""The one place a Unity player's argv is built: from a domain's config (config/domains/<domain>.yaml).

    cfg = player.load_domain('boat')
    args = player.resolve(cfg, seed=3, out=run_dir, ros={'port': 10101}, current='0.2,90')
    subprocess.Popen([player_path, *args])

The crane player's generic flags (-screen-*, -logFile, --crane-profile, --crane-scene, --crane-ros-ip/port) are
named here; every domain specific (scene, runtime profile, flag bundle, course truth, card and current flags)
comes from the domain's ``tiers.T2`` section and its ``env``, ``robot`` and ``card`` sections.
"""

from pathlib import Path

from crane import paths, tiers


class ConfigError(ValueError):
    """A domain config or a selector that cannot give a player argv; the message names the fix."""


def load_domain(name):
    """config/domains/NAME.yaml as a dict (with ``name`` set). Raises ConfigError naming the known domains."""
    path = paths.DOMAINS / f'{name}.yaml'
    if not path.is_file():
        known = ', '.join(sorted(p.stem for p in paths.DOMAINS.glob('*.yaml'))) or 'none'
        raise ConfigError(f'no domain {name!r} ({path} is missing); domains: {known}')
    from crane import miniyaml      # PyYAML when installed, else the stdlib subset reader
    try:
        cfg = miniyaml.safe_load(path.read_text()) or {}
    except ValueError as error:
        raise ConfigError(f'{path}: {error}') from None
    cfg.setdefault('name', name)
    return cfg


def tier_cfg(domain_cfg, tier='T2'):
    """The domain's section for TIER (T2 or 'unity'); ConfigError when it has none."""
    try:
        tier = tiers.normalise(tier)
    except ValueError as error:
        raise ConfigError(str(error)) from None
    section = (domain_cfg.get('tiers') or {}).get(tier)
    if tier != 'T2':
        raise ConfigError(f'{tier} runs no Unity player; the player tier is T2 (unity)')
    if not section or not section.get('scene'):
        raise ConfigError(f"domain {domain_cfg.get('name', '?')} has no tiers.T2 with a scene "
                          f"(config/domains/{domain_cfg.get('name', 'DOMAIN')}.yaml)")
    return section


def parse_current(text):
    """'SPEED,TOWARD_DEG' (m/s, deg), '0' (still water, '0,0') or 'default' (the scene's: None); ConfigError
    otherwise."""
    import re
    text = str(text).strip()
    if text == 'default':
        return None
    if text == '0':
        return '0,0'
    if not re.fullmatch(r'[0-9]+(\.[0-9]*)?,-?[0-9]+(\.[0-9]*)?', text):
        raise ConfigError(f'--current wants default, 0 or SPEED,TOWARD_DEG (m/s, deg); got {text}')
    return text


def env_args(domain_cfg, env, seed, tier='T2'):
    """--env KIND:NAME (default: the domain's env.default) as the tier's player flags."""
    section = domain_cfg.get('env') or {}
    env = env or section.get('default')
    if not env:
        return []
    kind, sep, name = str(env).partition(':')
    kinds = section.get('kinds') or {}
    if not sep or not name or kind not in kinds:
        raise ConfigError(f"--env wants KIND:NAME with KIND one of {', '.join(kinds) or 'none'}; got {env}")
    template = (kinds[kind] or {}).get(tier)
    if template is None:
        raise ConfigError(f'--env {kind}:... has no {tier} form in the domain config')
    return [str(t).replace('{name}', name).replace('{seed}', str(seed)) for t in template]


def card_path(domain_cfg, card):
    """A card REF as an absolute path: a JSON file, or a name in the domain's card.dir (REF or REF.json)."""
    tries = [Path(card).expanduser()]
    folder = (domain_cfg.get('card') or {}).get('dir')
    if folder:
        tries += [paths.ROOT / folder / str(card), paths.ROOT / folder / f'{card}.json']
    for p in tries:
        if p.is_file():
            return str(p.resolve(strict=True))
    raise ConfigError(f'no card {card} (a plant profile JSON, or a name in {folder or "the card dir"}); '
                      f'list them: ls {paths.ROOT / folder if folder else "."}')


def robot_cfg(domain_cfg, robot):
    """The robot's entry (its optional card and flags); the domain's default robot when robot is None."""
    section = domain_cfg.get('robot') or {}
    robots = section.get('robots') or {}
    robot = robot or section.get('default')
    if robot is None:
        return {}
    if robot not in robots:
        raise ConfigError(f"no robot {robot!r} in domain {domain_cfg.get('name', '?')}; robots: "
                          f"{', '.join(robots) or 'none'}")
    return robots[robot] or {}


def resolve(domain_cfg, tier='T2', env=None, robot=None, card=None, seed=None, profile=None, current=None,
            sets=(), ros=None, extra=(), out=None):
    """The Unity player's arguments (without the executable) for one run.

    env 'KIND:NAME' (default the domain's), robot (default the domain's), card a plant JSON or card name,
    seed (default the tier's), profile the runtime quality (default the tier's), current 'SPEED,TOWARD' or '0'
    (default: the scene's own), sets ['k=v'], ros {'ip', 'port'} for the ROS-TCP endpoint (None: no endpoint
    flags), extra appended last, out the run dir for the player log and the course truth (None: neither)."""
    t2 = tier_cfg(domain_cfg, tier)
    seed = t2.get('seed', 0) if seed is None else seed
    width, height = (t2.get('screen') or [1280, 720])[:2]
    a = ['-screen-fullscreen', '0', '-screen-width', str(width), '-screen-height', str(height)]
    if out is not None:
        a += ['-logFile', str(Path(out) / 'player.log')]
    profile = str(profile or t2.get('profile') or 'interactive-low')
    known = [str(k) for k in t2.get('profiles') or ()]
    if known and profile not in known:
        raise ConfigError(f"no runtime profile {profile!r}; profiles: {', '.join(known)}")
    a += ['--crane-profile', profile, '--crane-scene', str(t2['scene'])]
    if ros is not None:
        ip = ros.get('ip') or (t2.get('ros') or {}).get('ip') or '127.0.0.1'
        a += ['--crane-ros-ip', str(ip), '--crane-ros-port', str(ros['port'])]
    a += [str(f) for f in t2.get('flags') or []]
    a += env_args(domain_cfg, env, seed, 'T2')
    truth = t2.get('truth') or {}
    if out is not None and truth.get('flag'):
        a += [truth['flag'], str(Path(out) / truth.get('file', 'course_truth.json'))]
    bot = robot_cfg(domain_cfg, robot)
    card = card or bot.get('card')
    if card:
        flag = ((domain_cfg.get('card') or {}).get('T2') or {}).get('flag')
        if not flag:
            raise ConfigError(f"domain {domain_cfg.get('name', '?')} has no card.T2.flag: its player takes no card")
        a += [flag, card_path(domain_cfg, card)]
    if current is not None:
        flag = (t2.get('current') or {}).get('flag')
        if not flag:
            raise ConfigError(f"domain {domain_cfg.get('name', '?')} has no tiers.T2.current.flag")
        a += [flag, parse_current(current)]
    for kv in sets or ():
        if '=' not in str(kv) or not t2.get('set_flag'):
            raise ConfigError(f'--set wants KEY=VALUE and a domain with tiers.T2.set_flag; got {kv}')
        a += [t2['set_flag'], str(kv)]
    a += [str(f) for f in bot.get('flags') or []]
    return a + [str(x) for x in extra or ()]
