"""crane sim: the T2 world of a domain (the Unity player plus its ROS side), or its T1 world (the plant card's twin
as a ROS node beside the same ROS side), used exactly like the robot.

    crane sim up [--seed N] [--env KIND:NAME | --course FAMILY] [--card F | --profile-json F] [--profile P]
                 [--current SPEED,TOWARD_DEG | --current 0] [--display window|headless | --window | --headless]
                 [--player PATH] [--robot R] [--set K=V]... [--domain D] [--json] [--dry-run] [-- PLAYER_ARGS...]
    crane sim up --tier T1 [--odom sensors|truth] [--rate N] [--seed N] [--env KIND:NAME] [--card F]
                 [--current SPEED,TOWARD_DEG | --current 0] [--set K=V]... [--json] [--dry-run]
                                 the T1 world: no player, no window (Docs/SimT1.md); --odom truth turns
                                 localization off and publishes /odom/local and TF itself; --rate N runs at N x real time
    crane sim down [--json]      stop what up started (its player's process group and its ROS container only)
    crane sim restart nav|ros|player [--json]   restart one part, keep the rest (nav: only the Nav2 nodes, to read new
                                 nav2_limits; ros: the ROS container, the player stays; player: the player, ROS stays)
    crane sim status [--json]    the status block: mode, domain, port, pids, current, logs, the next commands
    crane sim check [--wait S] [--json]   live readiness: player, course, plant, ROS side, nodes, player errors
    crane sim logs [player|stdout|ros|t1] [-n LINES]   the last lines of each log (the running tier's by default)

crane sim names its verb (bare sim_up still means up). The domain (default config/sim.yaml's, boat) names everything
specific in config/domains/<domain>.yaml tiers.T2. Without --display: a window when stdin is a terminal and
DISPLAY or WAYLAND_DISPLAY is set, else headless under xvfb-run, so agents and scripts stay headless. The player
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
        p.add_argument('--current', metavar='SPEED,TOWARD_DEG', help='the water current (m/s, deg); 0 still water')
        p.add_argument('--display', choices=('window', 'headless'))
        p.add_argument('--window', dest='display', action='store_const', const='window')
        p.add_argument('--headless', dest='display', action='store_const', const='headless')
        p.add_argument('--player', metavar='PATH', help='the player executable (else CRANE_PLAYER, the build)')
        p.add_argument('--robot', help='the domain robot (default its robot.default)')
        p.add_argument('--set', dest='sets', action='append', default=[], metavar='K=V', help='pin a world parameter')
        p.add_argument('--dry-run', action='store_true', help='print the resolved player argv and docker run; start nothing')
        p.add_argument('--tier', default='T2', help='T2 (unity, the default) or T1 (dynamic: the twin, no player)')
        p.add_argument('--odom', choices=('sensors', 'truth'), help='T1: raw sensors for the real EKFs (default), or '
                       'truth odometry and TF with localization off')
        p.add_argument('--rate', type=float, metavar='N', help='T1: N x real time (default 1)')
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


def _up_tier(o, extra):
    """up's tier (T1 or T2), refusing the flags the other tier has no use for (one line, exit 2)."""
    from crane import tiers
    try:
        tier = tiers.normalise(o.tier)
    except ValueError as error:
        raise SimError(str(error)) from None
    if tier not in ('T1', 'T2'):
        raise SimError(f'crane sim runs T1 or T2 worlds, not {tier}; T0 runs with crane run --tier T0')
    if tier == 'T2':
        given = [f for f, v in (('--odom', o.odom), ('--rate', o.rate)) if v is not None]
        if given:
            raise SimError(f'{given[0]} is for the T1 world: {config.invoked_as()} up --tier T1 {given[0]} ...')
        return tier
    given = [f for f, v in (('--profile', o.profile), ('--player', o.player), ('-- PLAYER_ARGS', extra or None))
             if v is not None]
    if given:
        raise SimError(f'{given[0]} is for the T2 player; the T1 world runs no player')
    if o.rate is not None and not o.rate > 0:
        raise SimError(f'--rate wants a positive factor of real time, got {o.rate:g}')
    return tier


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
        tier = _up_tier(o, extra) if verb == 'up' else None
        cfg, t2 = config.domain(o.domain, tier, getattr(o, 'odom', None))
        run = config.run_dir()
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
        return world.logs(run, tuple(o.which), o.lines, t2.get('tier') or 'T2')
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
