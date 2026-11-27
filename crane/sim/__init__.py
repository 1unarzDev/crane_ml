"""crane sim: the T2 world of a domain (the Unity player plus its ROS side), or its T1 world (the plant card's twin
as a ROS node beside the same ROS side), used exactly like the robot.

    crane sim up [SIM] [--fidelity twin|low|high|eval] [--course NAME] ...
                                 SIM is a named sim (crane sim list), default this machine's (crane sim use SIM)
    crane sim list [--json]      the sims, this machine's default marked
    crane sim show [SIM] [up's flags] [--json]   the resolved settings, each with where it came from
    crane sim use SIM [--display window|headless|auto] [--stack S]   this machine's default sim (~/.config/crane/crane.yaml)
    crane sim watch              open a headless T2 sim in a window (only the player restarts; ROS and the course stay)
    crane sim env                export lines for the running sim's ROS (eval "$(crane sim env)"), for tools other than astro
    crane sim up [--seed N] [--env KIND:NAME | --course FAMILY] [--card F | --profile-json F] [--profile P]
                 [--current default|0|SPEED,TOWARD_DEG] [--display auto|window|headless | --window | --headless]
                 [--player PATH] [--robot R] [--set K=V]... [--domain D] [--tier T2] [--json] [--dry-run]
                 [--nav NAME:=VALUE]... [--nav-args "A:=x B:=y"] [--instance N] [-- PLAYER_ARGS...]
    crane sim up --list seed|env|current|tier|sim|fidelity|stack   the choices, one per line as VALUE<TAB>meaning (astro's pickers)
    crane sim up --tier T1 [--odom sensors|truth] [--rate N] [--seed N] [--env KIND:NAME] [--world course|pool[:LxW]] [--card F]
                 [--current default|0|SPEED,TOWARD_DEG] [--set K=V]... [--json] [--dry-run]
                                 the T1 world: no player, no window (Docs/SimT1.md); --odom truth turns
                                 localization off and publishes /odom/local and TF itself; --rate N runs at N x real time;
                                 --world pool[:LxW] a rectangular pool (default 10x4.5 m) whose walls the lidar sees
    crane sim down [--json]      stop what up started (its player's process group and its ROS container only)
    crane sim restart nav|ros|player [NAME:=VALUE ...] [--json]   restart one part, keep the rest (nav: only the Nav2 nodes, to read new
                                 nav2_limits; ros: the ROS container, the player stays; player: the player, ROS stays)
    crane sim status [--json]    the status block: mode, domain, port, pids, current, logs, the next commands
    crane sim check [--wait S] [--json]   live readiness: player, course, plant, ROS side, nodes, player errors
    crane sim logs [player|stdout|ros|t1] [-n LINES]   the last lines of each log (the running tier's by default)

Every verb takes --instance N (N >= 0, 0 the default world; else CRANE_SIM_INSTANCE, else the cluster worker's CRANE_ISOLATION_ID): a
parallel world with ROS_DOMAIN_ID = the profile's + N, endpoint port 10000 + that (CRANE_PORT_BASE replaces 10000),
container sim_up_ros_N and state dir ~/.cache/sim_up/N. down stops only that instance; status without one also lists
the running instances. T1 worlds run side by side; a T2 world needs the one Unity player free.
up and restart nav take NAME:=VALUE navigation launch arguments (up: --nav NAME:=VALUE, repeatable; restart nav:
the words themselves; --nav-args "A:=x B:=y" is several at once): they go to the navigation part of the launch and
are kept in the state, so a bare restart nav reuses them and new ones replace them.

crane sim names its verb (bare sim_up still means up). The domain (default config/sim.yaml's, boat) names everything
specific in config/domains/<domain>.yaml tiers.T2. --display auto (the default): a window when stdin is a terminal
and DISPLAY or WAYLAND_DISPLAY is set, else headless under xvfb-run, so agents and scripts stay headless.
--current default is the scene's current, 0 still water (for characterization). --tier T1 runs the plant-card
twin as the ROS world (no player, Docs/SimT1.md); --tier T0 refuses and names crane run. The T2 player
always runs through the host guard (`heavy unity`). up waits up to the ROS endpoint's endpoint_s (60 s) and the
course's ready_s (300 s, SIM_UP_READY_S). --json: progress on stderr, the status as one JSON document on stdout.
up --dry-run prints the resolved player argv and the ROS side's docker run, and starts nothing.
Exit 0 ok, 1 the world failed or is not ready, 2 usage or config, 75 a restart timed out waiting.

Environment: ROS_DOMAIN_ID (counted only when tiers.T2.host.vessel_env names the sim vessel, e.g. ASTRO_VESSEL=sim;
else the sim vessel profile's network.env under tiers.T2.host.profiles; else config/sim.yaml ros_domain_base; a robot
vessel's domain is always refused), the image env (tiers.T2.ros.image.env),
SIM_UP_VESSEL, SIM_UP_RUN (~/.cache/sim_up), SIM_UP_READY_S, CRANE_PLAYER, HEAVY (none: bare).
"""

import argparse
import os
import sys

from crane.sim import config, place, recipe
from crane.sim.config import SimError

VERBS = ('up', 'down', 'restart', 'status', 'check', 'logs', 'list', 'show', 'use', 'env', 'watch')
TIERS = ('T2', 'T1', 'T0', 'unity', 'dynamic', 'abstract', 'auto')
TIER_ABOUT = {'T2': 'the Unity world: real time, full sensors and ROS (the default)',
              'T1': 'the dynamic plant-card twin: no Unity, the same ROS topics, can run faster than real time',
              'T0': 'abstract or kinematic, no forces (crane run --tier T0)'}


def choices(what, cfg, t2):
    """[(value, meaning)] for --list WHAT, from the domain config (astro's choices_from pickers)."""
    import glob
    from crane import paths
    env = cfg.get('env') or {}
    default = str(env.get('default') or '')
    if what == 'tier':
        return list(TIER_ABOUT.items())
    if what == 'sim':
        mine = recipe.default_name()
        rows = [(n, str(doc.get('about') or n) + (' (this machine\'s default)' if n == mine else ''))
                for n, (doc, _) in sorted(recipe.sims().items())]
        return sorted(rows, key=lambda r: r[0] != mine)
    if what == 'fidelity':
        rungs, default = recipe.ladder(cfg)
        rows = [(k, str(v.get('about') or k) + (' (the default)' if k == default else '')) for k, v in rungs.items()]
        return sorted(rows, key=lambda r: r[0] != default)
    if what == 'stack':
        from crane.sim import stacks
        found = stacks.choose(cfg, 'auto')[0]
        about = {'astro': "the robot team's software (astro_dock's container, as on the boat)",
                 'bridge': "crane's own ROS-TCP endpoint; bring your own nodes", 'none': 'the 3-D world alone, no ROS'}
        return sorted([(n, about.get(n, n) + (' (auto finds it here)' if n == found else ''))
                       for n in stacks.allowed(cfg)], key=lambda r: r[0] != found)
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
    if verb in ('up', 'show'):
        p.add_argument('sim', nargs='?', metavar='SIM', help='a named sim (crane sim list; default this machine\'s)')
        p.add_argument('--fidelity', help='a rung of the domain\'s fidelity ladder: twin, low, high, eval')
    if verb == 'use':
        p.add_argument('sim', metavar='SIM', help='the sim a bare crane sim up starts on this machine (none: clear it)')
        p.add_argument('--stack', help="this machine's stack: auto, astro, bridge or none")
        p.add_argument('--display', choices=('auto', 'window', 'headless'), help='this machine\'s display mode')
        p.add_argument('--player', metavar='PATH', help='this machine\'s player')
    if verb in ('up', 'show'):
        p.add_argument('--seed', type=int, help='the world seed (default the tier seed, 1)')
        p.add_argument('--env', help='KIND:NAME, e.g. course:FAMILY (default the domain env.default)')
        p.add_argument('--course', metavar='FAMILY', help='the course family: same as --env course:FAMILY')
        p.add_argument('--card', '--profile-json', dest='card', metavar='F', help='a plant profile JSON or card name')
        p.add_argument('--profile', help='the runtime profile (--crane-profile; default the tier profile)')
        p.add_argument('--current', metavar='default|0|SPEED,TOWARD_DEG',
                       help='the water current: default (the scene), 0 (still water) or m/s,deg')
        p.add_argument('--display', choices=('auto', 'window', 'headless'))
        p.add_argument('--tier', choices=TIERS, help='T2 (unity) or T1 (the twin, no player); default the fidelity\'s')
        p.add_argument('--list', choices=('seed', 'env', 'current', 'tier', 'sim', 'fidelity', 'stack'), help='print the choices and exit')
        p.add_argument('--window', dest='display', action='store_const', const='window')
        p.add_argument('--headless', dest='display', action='store_const', const='headless')
        p.add_argument('--player', metavar='PATH', help='the player executable (else CRANE_PLAYER, the build)')
        p.add_argument('--robot', help='the domain robot (default its robot.default)')
        p.add_argument('--set', dest='sets', action='append', default=[], metavar='K=V', help='pin a world parameter')
        p.add_argument('--dry-run', action='store_true', help='print the resolved player argv and docker run; start nothing')
        p.add_argument('--stack', help='the software beside the world: auto (the default), astro, bridge or none')
        p.add_argument('--fresh', action='store_true', help='restart a sim that is already up instead of reusing it')
        p.add_argument('--odom', choices=('sensors', 'truth'), help='T1: raw sensors for the real EKFs (default), or '
                       'truth odometry and TF with localization off')
        p.add_argument('--rate', type=float, metavar='N', help='T1: N x real time (default 1)')
        p.add_argument('--nav', action='append', default=[], metavar='NAME:=VALUE',
                       help='a navigation launch argument, e.g. preset:=pool (repeat for more)')
        p.add_argument('--nav-args', action='append', default=[], metavar='"A:=x B:=y"',
                       help='several --nav at once, in one quoted string')
        p.add_argument('--world', metavar='course|pool[:LxW]', help='T1: course (the default) or a rectangular pool, '
                       'e.g. pool:10x4.5 (--env pool:LxW is the same)')
    if verb == 'restart':
        p.add_argument('what', choices=('nav', 'ros', 'player'), help='the part to restart')
        p.add_argument('words', nargs='*', metavar='NAME:=VALUE', help='restart nav: navigation launch arguments')
        p.add_argument('--nav-args', action='append', default=[], metavar='"A:=x B:=y"',
                       help='several NAME:=VALUE at once, in one quoted string')
        p.add_argument('--display', choices=('window', 'headless'), help='restart player: in a window or headless')
    if verb in ('up', 'down', 'restart', 'status', 'list', 'show', 'use', 'watch'):
        p.add_argument('--json', action='store_true', help='one JSON document on stdout')
    if verb == 'down':
        p.add_argument('--force', action='store_true', help="stop a person's sim from a script too")
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
        raise SimError(f'up starts T2 or T1 worlds; {tier} runs as a batch: crane run --domain {o.domain or "boat"} '
                       f'--tier {tier} --seeds 0:4')
    if str(o.env or '').split(':')[0] == 'pool':          # --env pool[:LxW] is --world pool[:LxW]
        if o.world not in (None, 'course'):
            raise SimError('give --world or --env pool, not both')
        o.world, o.env = o.env, None
    pool = str(o.world or 'course').split(':')[0] == 'pool'
    if tier == 'T2':
        if pool:
            raise SimError(f'the pool world is T1 only for now; {config.invoked_as()} up --tier T1 --world pool')
        given = [f for f, v in (('--odom', o.odom), ('--rate', o.rate)) if v is not None]
        if given:
            raise SimError(f'{given[0]} is for the T1 world: {config.invoked_as()} up --tier T1 {given[0]} ...')
        return tier
    given = [f for f, v in (('--profile', o.profile), ('--player', o.player), ('-- PLAYER_ARGS', extra or None))
             if v is not None]
    if given:
        raise SimError(f'{given[0]} is for the T2 player; the T1 world runs no player')
    if pool and o.sets:
        raise SimError('--set pins a course parameter; the pool world has none')
    if o.rate is not None and not o.rate > 0:
        raise SimError(f'--rate wants a positive factor of real time, got {o.rate:g}')
    return tier


def _take_instance(rest):
    """Set the instance from --instance N (any verb; removed from REST) or the environment; --instance wins."""
    out, value, i = [], None, 0
    while i < len(rest):
        word = rest[i]
        if word == '--':
            out += rest[i:]
            break
        if word == '--instance':
            if i + 1 >= len(rest):
                raise SimError('--instance wants a number, e.g. --instance 2')
            value, i = rest[i + 1], i + 2
        elif word.startswith('--instance='):
            value, i = word.split('=', 1)[1], i + 1
        else:
            out.append(word)
            i += 1
    value = value if value is not None else config.env_instance()
    _AUTO['on'] = str(value).strip() == 'auto'
    config.set_instance(None if _AUTO['on'] else value)
    return out


_AUTO = {'on': False}


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
    try:
        rest = _take_instance(rest)
    except SimError as error:
        print(f'{config.invoked_as()}: {error}', file=sys.stderr)
        return error.code
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
        if verb in ('list', 'use', 'env'):
            return _small_verb(verb, o)
        if verb in ('up', 'show'):
            o.extra = extra
            if o.course:
                if o.env:
                    raise SimError('give --env or --course, not both')
                o.env = f'course:{o.course}'
            r = apply_recipe(o)
            if verb == 'show':
                return _show(r, o.json)
        tier = _up_tier(o, extra) if verb == 'up' else None
        if verb == 'up' and not o.list and not o.dry_run:
            placed = _place(o, tier, r)
            if placed is not None:
                return placed
        cfg, t2 = config.domain(o.domain, tier, getattr(o, 'odom', None))
        run = config.run_dir()
        if verb == 'down' and not o.force:
            _guard_down(t2, run)
        if verb == 'up' and o.list:
            for value, about in choices(o.list, cfg, t2):
                print(f'{value}\t{about}')
            return 0
        if verb == 'up':
            o.recipe = r
            return world.up(cfg, t2, run, o)
        if verb == 'down':
            return world.down(cfg, t2, run, o.json)
        if verb == 'restart':
            from crane.sim import restart
            return restart.run_restart(cfg, t2, run, o.what, o.json,
                                       config.parse_nav(o.words, o.nav_args) or None, o.display)
        if verb == 'watch':
            return _watch(cfg, t2, run)
        if verb == 'status':
            return world.status(cfg, t2, run, o.json)
        bad = [w for w in o.which if w not in world.LOGS]
        if bad:
            raise SimError(f"no log {bad[0]!r}; choose {', '.join(world.LOGS)}")
        return world.logs(run, tuple(o.which), o.lines, t2.get('tier') or 'T2')
    except (resolver.ConfigError, recipe.RecipeError) as error:
        print(f'{config.invoked_as()}: {error}', file=sys.stderr)
        return 2
    except SimError as error:
        print(f'{config.invoked_as()}: {error}', file=sys.stderr)
        return error.code
    except KeyboardInterrupt:
        name = config.invoked_as()
        print(f'{name}: interrupted; what started keeps running: {name} status, or {name} down', file=sys.stderr)
        return 130


def apply_recipe(o):
    """Resolve up's (or show's) sim, this machine's settings and the flags into one recipe; set O from it."""
    for key in ('sim', 'fidelity', 'tier', 'profile', 'stack', 'display', 'player'):
        if getattr(o, key, None) == 'auto':              # astro's form may pass a default through as is
            setattr(o, key, None)
    flags = {k: getattr(o, k, None) for k in recipe.KEYS}
    flags['nav'] = config.parse_nav(getattr(o, 'nav', None), getattr(o, 'nav_args', None)) or None
    r = recipe.resolve(getattr(o, 'sim', None), flags)
    v = r.values
    o.domain, o.env, o.tier = v['domain'], v.get('env'), v['tier']
    for key in ('seed', 'profile', 'card', 'current', 'robot', 'odom', 'rate', 'world', 'player'):
        setattr(o, key, v.get(key))
    o.display = v.get('display') or 'auto'
    o.sets = list(v.get('sets') or [])
    o.nav, o.nav_args = [f'{k}:={x}' for k, x in (v.get('nav') or {}).items()], []
    o.fidelity = v.get('fidelity')
    from crane import player
    from crane.sim import stacks
    try:
        name, why = stacks.choose(player.load_domain(o.domain), v.get('stack'))
    except stacks.StackError as error:
        raise SimError(str(error)) from None
    if name:
        if why:
            r.sources['stack'] = why
        r.values['stack'] = o.stack = name
        config.set_stack(name)
    return r


def _show(r, as_json):
    if as_json:
        import json
        print(json.dumps(r.as_dict(), indent=2))
    else:
        print('\n'.join(r.lines()))
        print(f'  identity  {r.identity()}')
        print(f'next: {config.invoked_as()} up' + (f' {r.name}' if r.name else '') + '   (--dry-run: the player argv)')
    return 0


def _small_verb(verb, o):
    import json
    if verb == 'list':
        rows = []
        default = recipe.default_name()
        for name, (doc, path) in sorted(recipe.sims().items()):
            try:
                v = recipe.resolve(name).values
                place = v.get('world') if str(v.get('world') or 'course') != 'course' else \
                    str(v.get('env') or '').partition(':')[2]
                what = f"{v['domain']} {place or ''} {v.get('fidelity') or v['tier']}"
            except recipe.RecipeError as error:
                what = f'broken: {error}'
            rows.append({'sim': name, 'default': name == default, 'what': what.strip(),
                         'about': str(doc.get('about') or ''), 'path': str(path)})
        if o.json:
            print(json.dumps({'schema': 'crane.sim.list/1', 'default': default, 'sims': rows}, indent=2))
            return 0
        w = max([len(r['sim']) for r in rows] + [3])
        ww = max([len(r['what']) for r in rows] + [4])
        for r in rows:
            print(f"{'*' if r['default'] else ' '} {r['sim'].ljust(w)}  {r['what'].ljust(ww)}  {r['about']}")
        print(f'next: {config.invoked_as()} up SIM   or   {config.invoked_as()} use SIM (a bare up starts it)'
              + ('' if default else '; no default yet (a bare up starts the domain defaults)'))
        return 0
    if verb == 'use':
        clear = o.sim in ('none', '-')
        if not clear:
            recipe.load_sim(o.sim)
        updates = {'default': None if clear else o.sim}
        for key in ('display', 'player', 'stack'):
            if getattr(o, key, None):
                updates[key] = None if getattr(o, key) == 'auto' and key in ('display', 'stack') else getattr(o, key)
        path = recipe.save_machine(updates)
        if o.json:
            print(json.dumps({'schema': 'crane.sim.use/1', 'path': str(path), 'sim': recipe.machine()}))
        else:
            print(f'{config.invoked_as()}: this machine\'s sim is {"none" if clear else o.sim} ({path})')
            print(f'next: {config.invoked_as()} up' + ('' if clear else f'   (starts {o.sim}; {config.invoked_as()} show explains it)'))
        return 0
    # env: what a tool other than astro exports to reach the running (or the next) sim's ROS
    cfg, t2 = config.domain(None)
    run = config.run_dir()
    try:
        d = int((run / 'domain').read_text().strip())
        port = int((run / 'port').read_text().strip())
    except (OSError, ValueError):
        d = config.domain_id(t2)
        port = config.port_base(t2) + d
    print(f'export ROS_DOMAIN_ID={d}')
    print('export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')
    if t2.get('tier') != 'T1':                           # a T1 world has no ROS-TCP endpoint
        print(f'export CRANE_SIM_ENDPOINT={(t2.get("ros") or {}).get("ip") or "127.0.0.1"}:{port}')
    if config.instance():
        print(f'export CRANE_SIM_INSTANCE={config.instance()}')
    return 0


def _place(o, tier, r):
    """Pick up's instance: reuse a sim that is the same and the caller's, start beside or refuse (crane.sim.place).
    Returns the exit code when nothing needs starting (a reused sim), else None with the instance set."""
    from crane.sim import world
    who, auto = place.caller(), _AUTO['on']
    first = config.instance() or 0
    for n in range(first, place.MAX_INSTANCES):
        config.set_instance(n)
        cfg, t2 = config.domain(o.domain, tier, getattr(o, 'odom', None))
        run = config.run_dir()
        running = place.running_doc(t2, run)
        try:
            verdict = place.decide(running, r.identity(), who, o.fresh, auto)
        except SimError:
            world.status(cfg, t2, run, o.json)          # what runs, then the one line naming the fix
            raise
        if verdict == 'start':
            return None
        if verdict == 'reuse':
            say = (lambda m: print(m, file=sys.stderr)) if o.json else print
            say(f'{config.invoked_as()}: already up and the same sim ({r.name or "the domain defaults"}'
                f'{", instance " + str(n) if n else ""}): reusing it ({config.invoked_as()} up --fresh restarts it)')
            return world.status(cfg, t2, run, o.json)
        if verdict == 'replace':
            world.down(cfg, t2, run, stream=sys.stderr if o.json else None)
            return None
        if tier == 'T2' and running and running.get('owner') == 'person' and who == 'script':
            raise SimError(f"a person's sim is up ({running.get('sim') or 'T2'}): a script never takes it over and "
                           f'the one Unity player runs one T2 world; use --tier T1, or ask them to {config.invoked_as()} down')
    raise SimError(f'no free instance below {place.MAX_INSTANCES}: {config.invoked_as()} status, then down the old ones')


def _guard_down(t2, run):
    """A script never stops a person's sim without --force."""
    running = place.running_doc(t2, run)
    if running and running.get('owner') == 'person' and place.caller() == 'script':
        raise SimError(f"the sim here was started by a person ({running.get('sim') or 'a sim'}); "
                       f'{config.invoked_as()} down --force stops it anyway')


def _watch(cfg, t2, run):
    """See the running sim: a headless T2 player comes back in a window (the ROS side and course stay up)."""
    from crane.sim import restart, world
    me = config.invoked_as()
    if t2.get('tier') == 'T1':
        raise SimError(f'the T1 world has no 3-D view; follow it with ros2 topic echo /crane/odom after eval "$({me} env)", '
                       f'or start the 3-D world: {me} up --fidelity low', 2)
    if not world.alive(run):
        raise SimError(f'no player running here: {me} status', 2)
    if (world._read(run / 'mode') or 'headless') == 'window':
        print(f'{me}: already in a window')
        return 0
    return restart.run_restart(cfg, t2, run, 'player', False, None, 'window')
