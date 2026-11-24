"""Where crane sim's settings come from: the environment, the host tooling's profiles, the domain config.

The domain's ``tiers.T2.host`` block names the host tooling the world stands in for (its checkout, its vessel
profiles, the env that marks a ROS_DOMAIN_ID as its own, its heavy guard and its up command); ``tiers.T2.ros.image``
names the image env. crane's own env: SIM_UP_VESSEL, SIM_UP_RUN (~/.cache/sim_up), SIM_UP_READY_S, CRANE_PLAYER,
HEAVY (default crane's bin/heavy; ``none`` runs bare), CRANE_SIM_AS (the name messages carry; the sim_up shim sets it).
"""

import os
import re
from pathlib import Path

from crane import paths
from crane.player import ConfigError, load_domain, tier_cfg

PROG = 'sim_up'          # the status header and the state dir keep sim_up's names (the host tooling reads them)
_AS = {'name': 'crane sim'}


def invoked_as():
    """The name messages and fixes carry: 'crane sim', or 'sim_up' when the sim_up shim ran it."""
    return _AS['name']


def set_invoked_as(name):
    _AS['name'] = name or 'crane sim'


class SimError(Exception):
    """A one-line failure: the message names the cause and the fix. code 2 config/usage, 1 the world failed."""

    def __init__(self, message, code=2):
        super().__init__(message)
        self.code = code


def home():
    return Path(os.environ.get('HOME') or Path.home())


def run_dir():
    """The state dir: pids, logs, the course truth (SIM_UP_RUN, default ~/.cache/sim_up)."""
    return Path(os.environ.get('SIM_UP_RUN') or home() / '.cache' / PROG)


def _host(t2):
    return (t2 or {}).get('host') or {}


def _expand(text, **values):
    text = subst(text or '', **values)
    return str(home()) + text[1:] if text.startswith('~') else text


def _setting(spec, **values):
    """A host setting {env, default}: $env when set, else default ({name} and a leading ~ expanded)."""
    spec = spec if isinstance(spec, dict) else {'default': spec}
    value = os.environ.get(spec['env']) if spec.get('env') else None
    return _expand(value or spec.get('default') or '', **values)


def dock(t2):
    """The host tooling's checkout (tiers.T2.host.dock), or None when the domain names none."""
    d = _setting(_host(t2).get('dock'))
    return Path(d) if d else None


def default_domain():
    """config/sim.yaml's ``domain:`` (flat key), else boat."""
    try:
        for line in (paths.CONFIG / 'sim.yaml').read_text().splitlines():
            if line.startswith('domain:'):
                return line.split(':', 1)[1].split('#')[0].strip() or 'boat'
    except OSError:
        pass
    return 'boat'


def domain(name=None, tier=None, odom=None):
    """(domain config, its sim section); SimError(2) naming the fix when either is missing.

    The sim section is tiers.T2, or for T1 tiers.T2 overlaid with tiers.T1.sim (its ros.overrides and
    ros.ready_nodes picked for the odom mode), with ``tier`` (and ``odom``) set. tier None: the running world's
    (the state dir's ``tier``), else T2."""
    tier = tier or state_tier()
    try:
        cfg = load_domain(name or default_domain())
        t2 = tier_cfg(cfg, 'T2')
    except ConfigError as error:
        worlds = _t2_domains()
        fix = f'; next: {invoked_as()} up --domain {worlds[0]}' if worlds and worlds[0] != name else ''
        raise SimError(f'{error}{fix}') from None
    if tier == 'T2':
        return cfg, t2
    return cfg, t1_section(cfg, t2, odom or _state('odom'))


def _state(name):
    try:
        return (run_dir() / name).read_text().strip()
    except OSError:
        return ''


def state_tier():
    """The running (or last) world's tier from the state dir: T1 or T2 (the default)."""
    return 'T1' if _state('tier') == 'T1' else 'T2'


ODOM = ('sensors', 'truth')


def t1_section(cfg, t2, odom=None):
    """tiers.T2 overlaid with tiers.T1.sim for ODOM (default the section's odom): the T1 world's sim section."""
    t1 = ((cfg.get('tiers') or {}).get('T1') or {}).get('sim')
    if not t1:
        raise SimError(f"domain {cfg.get('name', '?')} has no tiers.T1.sim (config/domains/{cfg.get('name')}.yaml): "
                       f'its T1 runs no ROS world; {invoked_as()} up runs its T2 world')
    odom = odom or t1.get('odom') or 'sensors'
    if odom not in ODOM:
        raise SimError(f"--odom wants {' or '.join(ODOM)}, got {odom}")
    ros = {**(t2.get('ros') or {}), **(t1.get('ros') or {})}
    for key in ('overrides', 'ready_nodes'):
        if isinstance(ros.get(key), dict):
            ros[key] = ros[key].get(odom)
    return {**t2, **t1, 'ros': ros, 'tier': 'T1', 'odom': odom}


def _t2_domains():
    """The domains whose config has a T2 world (a tiers.T2 with a scene)."""
    found = []
    for path in sorted(paths.DOMAINS.glob('*.yaml')):
        try:
            tier_cfg(load_domain(path.stem), 'T2')
            found.append(path.stem)
        except Exception:          # an unreadable domain file is someone else's error
            continue
    return found


def vessel(t2):
    return os.environ.get('SIM_UP_VESSEL') or t2.get('vessel') or 'sim'


def profiles_root(t2):
    """host.profiles: its env, else the last KEY= line of its file, else its default; None when unnamed."""
    spec = _host(t2).get('profiles') or {}
    p = os.environ.get(spec['env']) or '' if spec.get('env') else ''
    f = spec.get('file') or {}
    if not p and f.get('key'):
        try:
            key = f'{f["key"]}='
            found = [line[len(key):] for line in Path(_setting(f)).read_text().splitlines() if line.startswith(key)]
            p = found[-1] if found else ''
        except OSError:
            p = ''
    p = _expand(p or spec.get('default') or '', dock=dock(t2) or '')
    return Path(p) if p else None


def _hint(t2, how):
    up = _host(t2).get('up')
    return f' ({how} {up})' if up else ''


def domain_id(t2):
    """The ROS_DOMAIN_ID: ROS_DOMAIN_ID when one of host.trust_domain_id is set (the host tooling exported it),
    else the vessel profile's network.env. Never guessed: a bare shell's ROS_DOMAIN_ID may be a robot's."""
    d = os.environ.get('ROS_DOMAIN_ID') or ''
    if d and any(os.environ.get(k) for k in _host(t2).get('trust_domain_id') or ()):
        if not d.isdigit():
            raise SimError(f'ROS_DOMAIN_ID={d} is not a number{_hint(t2, "it is exported by")}')
        return int(d)
    name, root = vessel(t2), profiles_root(t2)
    if root is None:
        raise SimError('no vessel profiles to read ROS_DOMAIN_ID from: name them in tiers.T2.host.profiles of '
                       'config/domains/DOMAIN.yaml')
    f = root / name / 'network.env'
    try:
        text = f.read_text()
    except OSError:
        raise SimError(f"no {f}: the {name} vessel profile names the sim's ROS_DOMAIN_ID{_hint(t2, 'or run it as')}")
    ids = [m.group(1) for m in (re.match(r'ROS_DOMAIN_ID=([0-9]*)', line) for line in text.splitlines()) if m]
    if not ids or not ids[-1]:
        raise SimError(f'{f} has no ROS_DOMAIN_ID: add ROS_DOMAIN_ID=N to it')
    return int(ids[-1])


def heavy(t2):
    """The guard argv prefix: HEAVY, else crane's bin/heavy, else host.heavy; HEAVY=none: []."""
    h = os.environ.get('HEAVY') or ''
    if not h:
        h = str(paths.HEAVY)
        fallback = _host(t2).get('heavy')
        if not os.access(h, os.X_OK) and fallback:
            h = _expand(fallback, dock=dock(t2) or '')
    if h == 'none':
        return []
    if not os.access(h, os.X_OK):
        raise SimError(f'no heavy guard at {h}: set HEAVY=/path/to/heavy (HEAVY=none runs bare)')
    return [h, 'unity']


def image(t2):
    """The ROS image: $ros.image.env, else the host checkout's compose file's ${ENV:-IMAGE} default."""
    spec = (t2.get('ros') or {}).get('image') or {}
    var = spec.get('env')
    if var and os.environ.get(var):
        return os.environ[var]
    root = dock(t2)
    compose = root / (spec.get('compose') or '.devcontainer/docker-compose.yml') if root else None
    try:
        for line in compose.read_text().splitlines() if compose and var else ():
            m = re.search(r'\$\{' + re.escape(var) + r':-([^}]*)\}', line)
            if m:
                return m.group(1)
    except OSError:
        pass
    if not var:
        raise SimError('no ROS image: name its env in tiers.T2.ros.image.env of config/domains/DOMAIN.yaml')
    raise SimError(f'no dev image ({var}): set {var}=IMAGE or check {compose}')


def has_flag(player, flag):
    """A managed assembly beside the player holds the flag (UTF-16LE, as .NET stores string literals)."""
    needle = flag.encode('utf-16-le')
    for dll in Path(player).parent.glob('*_Data/Managed/*.dll'):
        try:
            if needle in dll.read_bytes():
                return True
        except OSError:
            continue
    return False


def find_player(t2, given=None):
    """--player / CRANE_PLAYER, crane's build, then the phase-1 player link: the first that knows the flag."""
    flag = t2.get('required_flag') or ''
    env = given or os.environ.get('CRANE_PLAYER') or ''
    phase1 = home() / '.cache' / 'crane-phase1' / 'player' / 'CRANE.x86_64'
    for c in [p for p in (env, str(paths.PLAYER_BUILD / 'CRANE.x86_64'), str(phase1)) if p]:
        if os.path.isfile(c) and os.access(c, os.X_OK) and (not flag or has_flag(c, flag)):
            return c
    raise SimError(f'no player with {flag} (tried {env + " " if env else ""}{paths.PLAYER_BUILD}, '
                   f'~/.cache/crane-phase1/player); build one (crane build player) or set CRANE_PLAYER=/path/CRANE.x86_64')


def ready_s(t2):
    v = os.environ.get('SIM_UP_READY_S') or t2.get('ready_s') or 300
    try:
        return int(v)
    except ValueError:
        raise SimError(f'SIM_UP_READY_S wants seconds, got {v!r}') from None


def subst(text, **values):
    """{name} tokens in a config string (plain replace, so JSON braces stay)."""
    text = str(text)
    for k, v in values.items():
        text = text.replace('{' + k + '}', str(v))
    return text
