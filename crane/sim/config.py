"""Where crane sim's settings come from: the environment, the host tooling's profiles, the domain config.

The domain's ``tiers.T2.host`` block names the host tooling the world stands in for (its checkout, its vessel
profiles, the env that marks a ROS_DOMAIN_ID as its own, its heavy guard and its up command); ``tiers.T2.ros.image``
names the image env. crane's own env: SIM_UP_VESSEL, SIM_UP_RUN (~/.cache/sim_up), SIM_UP_READY_S, CRANE_PLAYER,
HEAVY (default crane's bin/heavy; ``none`` runs bare), CRANE_SIM_AS (the name messages carry; the sim_up shim sets it).
"""

import json
import os
import re
import shlex
import zlib
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


def state_base():
    """The state root (SIM_UP_RUN, default ~/.cache/sim_up): the world without an instance lives here."""
    return Path(os.environ.get('SIM_UP_RUN') or home() / '.cache' / PROG)


def run_dir():
    """The state dir: pids, logs, the course truth. An instance N has its own, state_base()/N."""
    n = instance()
    return state_base() / str(n) if n else state_base()


_INSTANCE = {'n': None}


def instance():
    """The parallel-world number (1, 2, ...) this process works on, or None (the one world)."""
    return _INSTANCE['n']


def set_instance(value):
    """Set the instance from --instance or CRANE_SIM_INSTANCE (a whole number; 0 or None is the default world)."""
    if value in (None, ''):
        _INSTANCE['n'] = None
        return
    try:
        n = int(str(value))
    except ValueError:
        n = 0
    if n < 0 or str(value).strip().lstrip('+') != str(n):
        raise SimError(f'--instance wants a whole number, 0 (the default world) or more, got {value}')
    _INSTANCE['n'] = n or None          # 0 is the default world: domain base, sim_up_ros, the legacy state dir


def env_instance():
    """The instance the environment names: CRANE_SIM_INSTANCE, else the cluster worker's CRANE_ISOLATION_ID (a whole
    number, else its text hashed to 1..100), else None."""
    if os.environ.get('CRANE_SIM_INSTANCE'):
        return os.environ['CRANE_SIM_INSTANCE']
    iso = (os.environ.get('CRANE_ISOLATION_ID') or '').strip()
    if not iso:
        return None
    return iso if iso.isdigit() and int(iso) >= 1 else zlib.crc32(iso.encode()) % 100 + 1


def port_base(t2):
    """The ROS-TCP endpoint's port base: CRANE_PORT_BASE when set, else tiers.T2.ros.port_base (10000)."""
    try:
        return int(os.environ.get('CRANE_PORT_BASE') or (t2.get('ros') or {}).get('port_base', 10000))
    except ValueError:
        raise SimError(f"CRANE_PORT_BASE={os.environ['CRANE_PORT_BASE']} is not a number") from None


def instances():
    """The instance numbers that have a state dir (digit-named folders in the state root)."""
    try:
        return sorted(int(p.name) for p in state_base().iterdir() if p.name.isdigit() and p.is_dir())
    except OSError:
        return []


def with_instance(t2):
    """T2 section for the running instance: its container is NAME_N (a copy; the section itself stays)."""
    n = instance()
    if not n or not (t2.get('ros') or {}).get('container'):
        return t2
    return {**t2, 'ros': {**t2['ros'], 'container': f"{t2['ros']['container']}_{n}"}}


NAV_NAME = re.compile(r'^[A-Za-z_][A-Za-z0-9_.]*$')


def parse_nav(words, nav_args=()):
    """NAME:=VALUE words (plus --nav-args strings of several) as {name: value}; a bad word is one line, code 2."""
    flat = list(words or [])
    for text in nav_args or ():
        try:
            flat += shlex.split(text)
        except ValueError as error:
            raise SimError(f'--nav-args {text!r}: {error}; give NAME:=VALUE words, e.g. --nav-args "preset:=pool"') from None
    out = {}
    for word in flat:
        name, sep, value = word.partition(':=')
        if not sep or not NAV_NAME.match(name):
            raise SimError(f'{word!r} is not NAME:=VALUE (e.g. preset:=pool)')
        if "'" in value:
            raise SimError(f'{word!r}: a quote in a value cannot be passed to the launch')
        out[name] = value
    return out


def container_path(t2, run, value):
    """A nav argument value as the ROS container sees it. A host path under the workspace dock, ~/.config/astro,
    ~/.config/mhseals or the state dir is mapped; any other path under $HOME is refused (the container cannot see it);
    anything else (a name, a container path) passes unchanged."""
    if not str(value).startswith(('/', '~')):
        return value
    ros = t2.get('ros') or {}
    path = str(home()) + value[1:] if value.startswith('~') else value
    host = str(home()).rstrip('/')

    def under(root):
        root = str(root).rstrip('/')
        return bool(root) and (path == root or path.startswith(root + '/')), path[len(root):]
    for root, there in ((run, str(run)), (dock(t2), ros.get('workspace') or '/workspace'),
                        (home() / '.config' / 'astro', None), (home() / '.config' / 'mhseals', None)):
        if root is None:
            continue
        hit, rest = under(root)
        if hit:
            if there is None:
                return str(ros.get('home') or home()).rstrip('/') + path[len(host):]
            return there + rest
    if path == host or path.startswith(host + '/'):
        where = f'{dock(t2) or "the workspace"}, ~/.config/astro or ~/.config/mhseals'
        raise SimError(f'{value} is not visible inside the ROS container: put the file under {where}')
    return value


def with_nav(overrides, nav, part, t2, run):
    """The overrides JSON text with nav {name: value} merged under parts.PART.args (values mapped for the container).
    No nav: the text unchanged."""
    if not nav:
        return overrides
    if not part:
        raise SimError("this domain's ros.restart.nav recipe has no args_part: nav arguments cannot be passed")
    doc = json.loads(overrides) if overrides else {}
    args = doc.setdefault('parts', {}).setdefault(part, {}).setdefault('args', {})
    args.update({k: container_path(t2, run, v) for k, v in nav.items()})
    return json.dumps(doc)


def nav_part(t2):
    return (((t2.get('ros') or {}).get('restart') or {}).get('nav') or {}).get('args_part')


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
    cfg, section = _domain(name, tier, odom)
    return cfg, with_instance(section)


def _domain(name, tier, odom):
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
    """The sim's ROS_DOMAIN_ID: the sim's base plus the instance number. Never a robot's: a domain that a robot
    vessel profile names is refused, whatever the environment says."""
    n = instance() or 0
    if n:
        try:
            d = _base_domain(t2, trust=False) + n      # the profile's base, so an instance's export is not counted twice
        except SimError:
            d = _base_domain(t2) + n
    else:
        d = _base_domain(t2)
    robots = robot_domains(t2)
    if d in robots:
        raise SimError(f'ROS domain {d} is the {robots[d]} vessel\'s: the sim never shares a robot\'s domain; '
                       f'run it as {_host(t2).get("up") or "crane sim up"} (or unset ROS_DOMAIN_ID)')
    return d


def _trusted_export(t2):
    """An exported ROS_DOMAIN_ID counts only when the host tooling exported it for the sim vessel itself
    (host.vessel_env, e.g. ASTRO_VESSEL=sim); a shell on a robot's vessel exports the robot's domain."""
    var = _host(t2).get('vessel_env')
    return bool(var) and os.environ.get(var) == vessel(t2)


def _profiles_named(t2):
    """True when the host's profile set is named explicitly (its env, or its key in its file), not just defaulted."""
    spec = _host(t2).get('profiles') or {}
    if spec.get('env') and os.environ.get(spec['env']):
        return True
    f = spec.get('file') or {}
    try:
        return bool(f.get('key')) and any(line.startswith(f'{f["key"]}=') for line in Path(_setting(f)).read_text().splitlines())
    except OSError:
        return False


def _network_domain(path):
    """The last ROS_DOMAIN_ID=N line of a network.env, or None (no file, no line, an empty value)."""
    try:
        ids = [m.group(1) for m in (re.match(r'ROS_DOMAIN_ID=([0-9]*)', line) for line in path.read_text().splitlines()) if m]
    except OSError:
        return None
    return int(ids[-1]) if ids and ids[-1] else None


def robot_domains(t2):
    """{ROS_DOMAIN_ID: vessel} of every other vessel profile in the host's profile set ({} without one)."""
    root, own = profiles_root(t2), vessel(t2)
    out = {}
    try:
        files = sorted(root.glob('*/network.env')) if root and root.is_dir() else []
    except OSError:
        files = []
    for f in files:
        d = _network_domain(f)
        if f.parent.name != own and d is not None:
            out.setdefault(d, f.parent.name)
    return out


def crane_domain_base():
    """config/sim.yaml's ``ros_domain_base:`` (flat key), else 101: the sim's domain with no host profiles."""
    try:
        for line in (paths.CONFIG / 'sim.yaml').read_text().splitlines():
            if line.startswith('ros_domain_base:'):
                return int(line.split(':', 1)[1].split('#')[0].strip())
    except (OSError, ValueError):
        pass
    return 101


def _base_domain(t2, trust=True):
    """The exported ROS_DOMAIN_ID when the host tooling exported it for the sim vessel (host.vessel_env), else the
    sim vessel profile's network.env, else (no profile set on this machine) crane's own base (config/sim.yaml)."""
    d = os.environ.get('ROS_DOMAIN_ID') or ''
    if trust and d and _trusted_export(t2):
        if not d.isdigit():
            raise SimError(f'ROS_DOMAIN_ID={d} is not a number{_hint(t2, "it is exported by")}')
        return int(d)
    name, root = vessel(t2), profiles_root(t2)
    if root is None or (not root.is_dir() and not _profiles_named(t2)):
        return crane_domain_base()                     # crane on its own: no host tooling here
    f = root / name / 'network.env'
    if not f.is_file():
        raise SimError(f"no {f}: the {name} vessel profile names the sim's ROS_DOMAIN_ID{_hint(t2, 'or run it as')}")
    found = _network_domain(f)
    if found is None:
        raise SimError(f'{f} has no ROS_DOMAIN_ID: add ROS_DOMAIN_ID=N to it')
    return found


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
    """--player / CRANE_PLAYER, crane's build, the phase-1 player link, then the installed player (crane update):
    the first that knows the flag."""
    flag = t2.get('required_flag') or ''
    env = given or os.environ.get('CRANE_PLAYER') or ''
    phase1 = home() / '.cache' / 'crane-phase1' / 'player' / 'CRANE.x86_64'
    store = home() / '.local' / 'share' / 'crane' / 'player' / 'current' / 'CRANE.x86_64'
    for c in [p for p in (env, str(paths.PLAYER_BUILD / 'CRANE.x86_64'), str(phase1), str(store)) if p]:
        if os.path.isfile(c) and os.access(c, os.X_OK) and (not flag or has_flag(c, flag)):
            return c
    raise SimError(f'no player with {flag} (tried {env + " " if env else ""}{paths.PLAYER_BUILD}, '
                   f'~/.cache/crane-phase1/player, ~/.local/share/crane/player/current); build one (crane build player), '
                   'install the release one (crane update --player-only) or set CRANE_PLAYER=/path/CRANE.x86_64')


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
