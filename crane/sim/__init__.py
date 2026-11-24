"""crane sim: the T2 world of a domain (the Unity player plus its ROS side), used exactly like the robot.

    crane sim up [--seed N] [--env KIND:NAME | --course FAMILY] [--card F | --profile-json F] [--profile P]
                 [--current default|0|SPEED,TOWARD_DEG] [--display auto|window|headless | --window | --headless]
                 [--player PATH] [--robot R] [--set K=V]... [--domain D] [--tier T2] [--json] [--dry-run]
                 [-- PLAYER_ARGS...]
    crane sim up --list seed|env|current|tier   the choices, one per line as VALUE<TAB>meaning (astro's pickers)
    crane sim down [--json]      stop what up started (its player's process group and its ROS container only)
    crane sim restart nav|ros|player [--json]   restart one part, keep the rest (nav: only the Nav2 nodes, to read new
                                 nav2_limits; ros: the ROS container, the player stays; player: the player, ROS stays)
    crane sim status [--json]    the status block: mode, domain, port, pids, current, logs, the next commands
    crane sim check [--wait S] [--json]   live readiness: player, course, plant, ROS side, nodes, player errors
    crane sim logs [player|stdout|ros] [-n LINES]   the last lines of each log

crane sim names its verb (bare sim_up still means up). The domain (default config/sim.yaml's, boat) names everything
specific in config/domains/<domain>.yaml tiers.T2. --display auto (the default): a window when stdin is a terminal
and DISPLAY or WAYLAND_DISPLAY is set, else headless under xvfb-run, so agents and scripts stay headless.
--current default is the scene's current, 0 still water (for characterization). up runs T2 only; --tier T1/T0
refuse and name crane run, the batch form of those tiers. The player
always runs through the host guard (`heavy unity`). up waits up to the ROS endpoint's endpoint_s (60 s) and the
course's ready_s (300 s, SIM_UP_READY_S). --json: progress on stderr, the status as one JSON document on stdout.
up --dry-run prints the resolved player argv and the ROS side's docker run, and starts nothing.
Exit 0 ok, 1 the world failed or is not ready, 2 usage or config, 75 a restart timed out waiting.

Environment: ROS_DOMAIN_ID (counted only beside an env named in tiers.T2.host.trust_domain_id, else read from
the vessel profile's network.env under tiers.T2.host.profiles), the image env (tiers.T2.ros.image.env),
SIM_UP_VESSEL, SIM_UP_RUN (~/.cache/sim_up), SIM_UP_READY_S, CRANE_PLAYER, HEAVY (none: bare).
"""

import argparse
import os
import sys

from crane.sim import config
from crane.sim.config import SimError

VERBS = ('up', 'down', 'restart', 'status', 'check', 'logs')
TIERS = ('T2', 'T1', 'T0', 'unity', 'dynamic', 'abstract')
TIER_ABOUT = {'T2': 'the Unity world: real time, full sensors and ROS (the default)',
              'T1': 'the dynamic plant-card twin: fast, no rendered sensors (crane run --tier T1)',
              'T0': 'abstract or kinematic, no forces (crane run --tier T0)'}


def choices(what, cfg, t2):
    """[(value, meaning)] for --list WHAT, from the domain config (astro's choices_from pickers)."""
    import glob
    from crane import paths
    env = cfg.get('env') or {}
    default = str(env.get('default') or '')
    if what == 'tier':
        return list(TIER_ABOUT.items())
    if what == 'current':
        ds, _, dt = str((t2.get('current') or {}).get('default') or '0,0').partition(',')
        return [('default', f"the scene's current, {ds} m/s toward {dt} deg (realistic)"),
                ('0', 'still water (clean characterization)'),
                (f'{ds},{dt}', 'custom SPEED,TOWARD_DEG (m/s, deg): edit the value')]
    if what == 'env':
        out = [(default, 'the default')] if default else []
        for kind, spec in (env.get('kinds') or {}).items():
            for path in sorted(glob.glob(str(paths.ROOT / str((spec or {}).get('list') or '')))) if \
                    (spec or {}).get('list') else []:
                value = f'{kind}:{os.path.basename(path).split(".")[0]}'
                if value != default:
                    out.append((value, f'the {value.split(":", 1)[1]} {kind}'))
        return out
    seed = int(t2.get('seed') or 1)
    name = default.partition(':')[2] or cfg.get('name') or 'world'
    return [(str(s), f'{name} seed {s}' + (' (the default)' if s == seed else '')) for s in range(seed, seed + 8)]


class _Parser(argparse.ArgumentParser):
    def error(self, message):
        raise SimError(f'{message} (crane sim {self.prog.split()[-1]} --help)')


def _parser(verb):
    p = _Parser(prog=f'crane sim {verb}', add_help=True)
    p.add_argument('--domain', help='config/domains/DOMAIN.yaml (default: config/sim.yaml domain)')
    if verb == 'up':
        p.add_argument('--seed', type=int, help='the world seed (default the tier seed, 1)')
        p.add_argument('--env', help='KIND:NAME, e.g. course:FAMILY (default the domain env.default)')
        p.add_argument('--course', metavar='FAMILY', help='the course family: same as --env course:FAMILY')
        p.add_argument('--card', '--profile-json', dest='card', metavar='F', help='a plant profile JSON or card name')
        p.add_argument('--profile', help='the runtime profile (--crane-profile; default the tier profile)')
        p.add_argument('--current', metavar='default|0|SPEED,TOWARD_DEG',
                       help='the water current: default (the scene), 0 (still water) or m/s,deg')
        p.add_argument('--display', choices=('auto', 'window', 'headless'), default='auto')
        p.add_argument('--tier', choices=TIERS, default='T2', help='T2 only (T1/T0: crane run)')
        p.add_argument('--list', choices=('seed', 'env', 'current', 'tier'), help='print the choices and exit')
        p.add_argument('--window', dest='display', action='store_const', const='window')
        p.add_argument('--headless', dest='display', action='store_const', const='headless')
        p.add_argument('--player', metavar='PATH', help='the player executable (else CRANE_PLAYER, the build)')
        p.add_argument('--robot', help='the domain robot (default its robot.default)')
        p.add_argument('--set', dest='sets', action='append', default=[], metavar='K=V', help='pin a world parameter')
        p.add_argument('--dry-run', action='store_true', help='print the resolved player argv and docker run; start nothing')
    if verb == 'restart':
        p.add_argument('what', choices=('nav', 'ros', 'player'), help='the part to restart')
    if verb in ('up', 'down', 'restart', 'status'):
        p.add_argument('--json', action='store_true', help='one JSON document on stdout')
    if verb == 'logs':
        p.add_argument('which', nargs='*', metavar='LOG', help='player, stdout or ros (default all three)')
        p.add_argument('-n', '--lines', type=int, default=40)
    return p


def _usage(stream=sys.stdout):
    """The usage text under the name this ran as (crane sim, or sim_up through the shim)."""
    print(__doc__.strip().replace('crane sim', config.invoked_as()), file=stream)


def main(argv=None):
    """crane sim VERB ARGS: the exit code (0 ok, 1 the world failed or is not ready, 2 usage or config)."""
    argv = list(sys.argv[1:] if argv is None else argv)
    config.set_invoked_as(os.environ.pop('CRANE_SIM_AS', None))   # the sim_up shim: messages keep its name
    if argv[:1] in (['-h'], ['--help'], ['help']):
        _usage()
        return 0
    if not argv or argv[0].startswith('-'):   # the sim_up shim inserts `up` itself; crane sim names its verb
        _usage()
        return 2
    verb, rest = argv[0], argv[1:]
    if verb == 'check':
        from crane.sim import check
        return check.main(rest)
    if verb not in VERBS:
        _usage()
        print(f"{config.invoked_as()}: no verb {verb!r}; choose {'|'.join(VERBS)} ({config.invoked_as()} --help)",
              file=sys.stderr)
        return 2
    from crane import player as resolver
    from crane.sim import world
    extra = []
    if verb == 'up' and '--' in rest:
        i = rest.index('--')
        rest, extra = rest[:i], rest[i + 1:]
    try:
        o = _parser(verb).parse_args(rest)
        if verb == 'up':
            o.extra = extra
            if o.course:
                if o.env:
                    raise SimError('give --env or --course, not both')
                o.env = f'course:{o.course}'
        cfg, t2 = config.domain(o.domain)
        run = config.run_dir()
        if verb == 'up' and o.list:
            for value, about in choices(o.list, cfg, t2):
                print(f'{value}\t{about}')
            return 0
        if verb == 'up' and o.tier not in ('T2', 'unity'):
            tier = {'dynamic': 'T1', 'abstract': 'T0'}.get(o.tier, o.tier)
            raise SimError(f'up starts the T2 world only; {tier} runs as a batch: crane run --domain '
                           f'{cfg.get("name") or o.domain or "boat"} --tier {tier} --seeds 0:4 (a ROS-facing T1 world '
                           'is being built separately)')
        if verb == 'up':
            return world.up(cfg, t2, run, o)
        if verb == 'down':
            return world.down(cfg, t2, run, o.json)
        if verb == 'restart':
            from crane.sim import restart
            return restart.run_restart(cfg, t2, run, o.what, o.json)
        if verb == 'status':
            return world.status(cfg, t2, run, o.json)
        bad = [w for w in o.which if w not in world.LOGS]
        if bad:
            raise SimError(f"no log {bad[0]!r}; choose {', '.join(world.LOGS)}")
        return world.logs(run, tuple(o.which), o.lines)
    except resolver.ConfigError as error:
        print(f'{config.invoked_as()}: {error}', file=sys.stderr)
        return 2
    except SimError as error:
        print(f'{config.invoked_as()}: {error}', file=sys.stderr)
        return error.code
    except KeyboardInterrupt:
        name = config.invoked_as()
        print(f'{name}: interrupted; what started keeps running: {name} status, or {name} down', file=sys.stderr)
        return 130
