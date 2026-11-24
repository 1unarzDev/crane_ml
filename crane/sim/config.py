"""Where crane sim's settings come from: the environment astro exports, the astro profiles, the domain config.

Environment (unchanged from sim_up): ROS_DOMAIN_ID (trusted only beside ASTRO_STATE or MHSEALS_HARDWARE_CONFIG,
as astro exports it), ASTRO_DOCK (default ~/astro_dock), ASTRO_PROFILES (else ~/.config/astro/profiles.env, else
$ASTRO_DOCK/profiles), ASTRO_PROFILES_CONFIG, SIM_UP_VESSEL, SIM_UP_RUN (~/.cache/sim_up), SIM_UP_READY_S,
CRANE_PLAYER, HEAVY (default crane's bin/heavy; ``none`` runs bare), ASTRO_IMAGE.
"""

import os
import re
from pathlib import Path

from crane import paths
from crane.player import ConfigError, load_domain, tier_cfg

PROG = 'sim_up'          # the status header, the messages and the state dir keep sim_up's names for astro


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


def dock():
    return Path(os.environ.get('ASTRO_DOCK') or home() / 'astro_dock')


def default_domain():
    """config/sim.yaml's ``domain:`` (flat key), else boat."""
    try:
        for line in (paths.CONFIG / 'sim.yaml').read_text().splitlines():
            if line.startswith('domain:'):
                return line.split(':', 1)[1].split('#')[0].strip() or 'boat'
    except OSError:
        pass
    return 'boat'


def domain(name=None):
    """(domain config, its T2 section); SimError(2) naming the fix when either is missing."""
    try:
        cfg = load_domain(name or default_domain())
        return cfg, tier_cfg(cfg, 'T2')
    except ConfigError as error:
        raise SimError(str(error)) from None


def vessel(t2):
    return os.environ.get('SIM_UP_VESSEL') or t2.get('vessel') or 'sim'


def profiles_root():
    """ASTRO_PROFILES, else the last ASTRO_PROFILES= of the profiles.env, else $ASTRO_DOCK/profiles."""
    p = os.environ.get('ASTRO_PROFILES') or ''
    if not p:
        env = Path(os.environ.get('ASTRO_PROFILES_CONFIG') or home() / '.config' / 'astro' / 'profiles.env')
        try:
            found = [line[len('ASTRO_PROFILES='):] for line in env.read_text().splitlines()
                     if line.startswith('ASTRO_PROFILES=')]
            p = found[-1] if found else ''
        except OSError:
            p = ''
    if p.startswith('~'):
        p = str(home()) + p[1:]
    return Path(p) if p else dock() / 'profiles'


def domain_id(t2):
    """The ROS_DOMAIN_ID: astro's (ROS_DOMAIN_ID with ASTRO_STATE or MHSEALS_HARDWARE_CONFIG), else the sim
    vessel profile's network.env. Never guessed: a bare shell's ROS_DOMAIN_ID may be a boat's."""
    d = os.environ.get('ROS_DOMAIN_ID') or ''
    if d and (os.environ.get('ASTRO_STATE') or os.environ.get('MHSEALS_HARDWARE_CONFIG')):
        return int(d) if d.isdigit() else _bad_id(d)
    name = vessel(t2)
    f = profiles_root() / name / 'network.env'
    try:
        text = f.read_text()
    except OSError:
        raise SimError(f'no {f}: the {name} vessel profile names the sim\'s ROS_DOMAIN_ID (or run it as astro sim up)')
    ids = [m.group(1) for m in (re.match(r'ROS_DOMAIN_ID=([0-9]*)', line) for line in text.splitlines()) if m]
    if not ids or not ids[-1]:
        raise SimError(f'{f} has no ROS_DOMAIN_ID')
    return int(ids[-1])


def _bad_id(d):
    raise SimError(f'ROS_DOMAIN_ID={d} is not a number; astro exports the vessel\'s (astro sim up)')


def heavy():
    """The guard argv prefix: HEAVY, else crane's bin/heavy, else the dock's handoff copy; HEAVY=none: []."""
    h = os.environ.get('HEAVY') or ''
    if not h:
        h = str(paths.HEAVY)
        if not os.access(h, os.X_OK):
            h = str(dock() / '.handoff' / 'tools' / 'heavy')
    if h == 'none':
        return []
    if not os.access(h, os.X_OK):
        raise SimError(f'no heavy guard at {h} (HEAVY=none runs bare)')
    return [h, 'unity']


def image(ros):
    """The ROS image: $ASTRO_IMAGE (or the config's env name), else the dock compose file's ${ASTRO_IMAGE:-X}."""
    spec = ros.get('image') or {}
    var = spec.get('env') or 'ASTRO_IMAGE'
    if os.environ.get(var):
        return os.environ[var]
    compose = dock() / (spec.get('compose') or '.devcontainer/docker-compose.yml')
    try:
        for line in compose.read_text().splitlines():
            m = re.search(r'\$\{' + re.escape(var) + r':-([^}]*)\}', line)
            if m:
                return m.group(1)
    except OSError:
        pass
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
