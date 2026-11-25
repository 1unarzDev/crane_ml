"""crane sim up --tier T1, the host side: resolve the T1 node's config (card, course, start, current, lever arms,
topics) into the state dir's t1.json, and the ROS container's command that runs the node beside the launch.

The node (crane/sim/t1node.py) and the plant file's folder (tiers.T1.sim.plant: plant.py reads profiles/ beside
it) are mounted read-only into the container under /opt/crane_t1; the state dir is mounted at its own path, so
t1.json, t1.log and t1.alive are shared.
"""

import hashlib
import importlib
import json
import math
import shlex
import shutil
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from crane import paths
from crane import player as resolver
from crane.sim import config
from crane.sim.config import SimError

IN = '/opt/crane_t1'           # where the node and the plant are mounted in the container
LOG, ALIVE, CONFIG = 't1.log', 't1.alive', 't1.json'
STALE_S = 5.0                  # a heartbeat older than this: the node is not running


def _root_path(rel):
    p = Path(rel).expanduser()
    return p if p.is_absolute() else paths.ROOT / p


def load_plant(sec):
    from crane.sim import t1node
    return t1node.load_plant(str(_root_path(sec['plant'])))


def lever_arms(urdf, links):
    """{link: (x, y, z)} of each link in base_link, through the URDF's fixed-joint chain (origins xyz + rpy)."""
    tree = ET.parse(urdf).getroot()
    parent = {}
    for joint in tree.iter('joint'):
        origin = joint.find('origin')
        xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
        rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
        parent[joint.find('child').get('link')] = (joint.find('parent').get('link'), xyz, rpy)

    def rot(rpy):
        (cr, sr), (cp, sp), (cy, sy) = ((math.cos(a), math.sin(a)) for a in rpy)
        return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr]]

    out = {}
    for name, link in links.items():
        p, cur = [0.0, 0.0, 0.0], link
        while cur != 'base_link':
            if cur not in parent:
                raise SimError(f'{urdf}: no fixed-joint chain from base_link to {link}')
            up, xyz, rpy = parent[cur]
            r = rot(rpy)
            p = [sum(r[i][j] * p[j] for j in range(3)) + xyz[i] for i in range(3)]
            cur = up
        out[name] = tuple(round(v, 6) for v in p)
    return out


def current_xy(sec, value):
    """'default', '0' or 'SPEED,TOWARD_DEG' as the world current (cx, cy) m/s."""
    if value in (None, 'default'):
        value = str((sec.get('current') or {}).get('default') or '0')
    value = resolver.parse_current(value)
    speed, _, toward = value.partition(',')
    s, d = float(speed), math.radians(float(toward or 0))
    return round(s * math.cos(d), 12) + 0.0, round(s * math.sin(d), 12) + 0.0   # never -0.0


def course_truth(cfg, sec, env, seed, sets, out):
    """Write the course truth for (family, seed, sets) to OUT: the generator, else the committed truth (no sets).
    Returns where it came from."""
    env = env or (cfg.get('env') or {}).get('default') or ''
    kind, _, name = str(env).partition(':')
    if kind != 'course' or not name:
        raise SimError(f'T1 builds courses only: --env course:FAMILY, got {env or "none"}')
    spec = sec.get('course') or {}
    problem = 'no course.generate in tiers.T1.sim'
    if spec.get('generate'):
        mod, _, fn = spec['generate'].partition(':')
        try:
            getattr(importlib.import_module(mod), fn)(name, seed, list(sets or ()), out)
            json.loads(Path(out).read_text())
            return 'coursegen'
        except Exception as error:          # no dotnet, a failed build or generation: say why, then the fallback
            problem = f'coursegen failed ({str(error).splitlines()[0][:120] if str(error) else type(error).__name__})'
    committed = _root_path(spec.get('dir') or '.') / config.subst(spec.get('file') or '{name}-s{seed}.json',
                                                                  name=name, seed=seed)
    if sets:
        raise SimError(f'{problem}; --set needs the generator (dotnet): crane doctor')
    if not committed.is_file():
        raise SimError(f'{problem} and no committed {committed}; try a seed in {committed.parent}')
    shutil.copyfile(committed, out)
    return f'committed {committed.relative_to(paths.ROOT) if committed.is_relative_to(paths.ROOT) else committed}'


def resolve(cfg, sec, run, o, current, truth=None):
    """The node's config (for RUN/t1.json); the course truth is written to TRUTH (default RUN/course_truth.json).
    SimError naming the fix."""
    world = sec.get('world') or {}
    bot = resolver.robot_cfg(cfg, getattr(o, 'robot', None))
    card = resolver.card_path(cfg, o.card or bot.get('card') or sec.get('card'))
    raw = Path(card).read_bytes()
    profile = json.loads(raw)
    plant = load_plant(sec)
    try:
        plant.ProfilePlant(profile, float(profile['fixedDeltaTime']))
    except (KeyError, ValueError, TypeError) as error:
        raise SimError(f'card {card} is not a plant profile the twin steps ({error})') from None
    if not bot.get('urdf'):
        raise SimError(f"robot {getattr(o, 'robot', None) or (cfg.get('robot') or {}).get('default')} has no urdf "
                       f"in config/domains/{cfg.get('name')}.yaml: T1 reads its sensor lever arms from it")
    lever = lever_arms(_root_path(bot['urdf']), {'gps': world['gps']['link'], 'imu': world['imu']['link']})
    seed = o.seed if o.seed is not None else sec.get('seed', 0)
    truth = Path(truth or run / ((sec.get('truth') or {}).get('file') or 'course_truth.json'))
    source = course_truth(cfg, sec, o.env, seed, o.sets, truth)
    course = json.loads(truth.read_text())
    noise = dict(world.get('noise') or {})
    doc = {'schema': 'crane.sim.t1/1', 'odom': sec['odom'], 'rate': float(o.rate or sec.get('rate') or 1),
           'dt': float(profile['fixedDeltaTime']), 'profile': profile,
           'profile_sha256': hashlib.sha256(raw).hexdigest(), 'card': card,
           'plant': f'{IN}/plant/{Path(sec["plant"]).name}', 'current': list(current_xy(sec, current)),
           'start': [float(course.get('startX', 0)), float(course.get('startY', 0)), float(course.get('startYaw', 0))],
           'course': course.get('name') or truth.name, 'seed': seed, 'course_source': source,
           'lever': {k: list(v) for k, v in lever.items()}, 'world': {**world, 'noise': noise},
           'alive': str(run / ALIVE)}
    return doc


def write(run, doc):
    (run / CONFIG).write_text(json.dumps(doc, indent=1) + '\n')


def command(sec, run):
    """The container's bash command: the node in the background, then the launch (each logged to the state dir)."""
    node = f'{IN}/{Path(sec["node"]).name}'
    return f'python3 {node} {shlex.quote(str(run / CONFIG))} >{run}/{LOG} 2>&1 & '


def mounts(sec):
    """The node file and the plant's folder, read-only."""
    return [f'{_root_path(sec["node"])}:{IN}/{Path(sec["node"]).name}:ro',
            f'{_root_path(sec["plant"]).parent}:{IN}/plant:ro']


def heartbeat(run):
    """The node's last heartbeat {t_sim, wall, rate, age_s}, or None when it has none or it is stale."""
    try:
        doc = json.loads((run / ALIVE).read_text())
        age = time.time() - float(doc['wall'])
    except (OSError, ValueError, KeyError, TypeError):
        return None
    return {**doc, 'age_s': round(age, 1)} if age <= STALE_S else None


def rate_text(rate):
    r = float(rate)
    return f'{r:g}x'
