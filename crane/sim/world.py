"""crane sim up|down|status|logs: the T2 world (the ROS side in its container, the player on heavy unity), or the
T1 world (the T1 node beside the launch in the same container, crane/sim/t1.py).

The state dir (config.run_dir, ~/.cache/sim_up) holds what down and status read: player.pid (the player's
process group), player.path, mode, tier, domain, port, seed, current, the logs and the course truth; for T1 also
odom, rate, t1.json, t1.log and t1.alive. down stops only that process group and the domain's own container, never
a tmux server or another container.
"""

import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

from crane import paths, player as resolver
from crane.sim import config, t1
from crane.sim.config import PROG, SimError


def _say(msg, stream=None):
    print(f'{config.invoked_as()}: {msg}', file=stream or sys.stdout, flush=True)


def _read(path):
    try:
        return Path(path).read_text().strip()
    except OSError:
        return ''


def alive(run):
    """The player's pid (its own process group) is alive."""
    pid = _read(run / 'player.pid')
    try:
        os.kill(int(pid), 0)
        return True
    except (OSError, ValueError):
        return False


def ros_up(container):
    if not container:                          # the none stack: no ROS side
        return False
    try:
        out = subprocess.run(['docker', 'inspect', '-f', '{{.State.Running}}', container],
                             capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return out.returncode == 0 and out.stdout.strip() == 'true'


def listening(port):
    """A TCP socket listens on PORT (any address), from /proc/net/tcp{,6}; else `ss -ltn`."""
    found = False
    for name in ('/proc/net/tcp', '/proc/net/tcp6'):
        try:
            lines = Path(name).read_text().splitlines()[1:]
        except OSError:
            continue
        found = True
        for line in lines:
            f = line.split()
            if len(f) > 3 and f[3] == '0A' and int(f[1].rsplit(':', 1)[1], 16) == port:
                return True
    if found:
        return False
    try:
        out = subprocess.run(['ss', '-ltn', f'sport = :{port}'], capture_output=True, text=True, timeout=10)
        return 'LISTEN' in out.stdout
    except (OSError, subprocess.TimeoutExpired):
        return False


def _current_text(t2, value):
    default = str((t2.get('current') or {}).get('default') or '')
    ds, _, dt = default.partition(',')
    if value == 'default':
        return f'{ds} m/s toward {dt} deg (the scene default)'
    s, _, d = value.partition(',')
    return f'{s} m/s toward {d} deg (--current; the scene default is {ds} m/s toward {dt} deg)'


def state(cfg, t2, run):
    """What status reports, as a dict (status --json prints it)."""
    ros = t2.get('ros') or {}
    container = ros.get('container')
    is_t1 = t2.get('tier') == 'T1'
    beat = t1.heartbeat(run) if is_t1 else None
    player_up, ros_running = (beat is not None) if is_t1 else alive(run), ros_up(container)
    log = run / (t1.LOG if is_t1 else 'player.log')
    marker = t2.get('ready_marker') or ''
    try:
        ready = bool(marker) and marker in log.read_text(errors='replace')
    except OSError:
        ready = False
    current = _read(run / 'current')
    truth = (t2.get('truth') or {}).get('file') or 'course_truth.json'
    launch = str(ros.get('launch') or '').split(None, 1) if ros.get('launch') else \
        ['', str(ros.get('command') or '').split(' --', 1)[0]]
    extra = {}
    if is_t1:
        extra = {'tier': 'T1', 'odom': _read(run / 'odom') or t2.get('odom'), 'rate': _read(run / 'rate') or None,
                 'world_name': _read(run / 'world') or 'course',
                 'world': {'running': player_up, 't_sim': beat and beat.get('t_sim'), 'rate': beat and beat.get('rate'),
                           'heartbeat_age_s': beat and beat.get('age_s')}}
    n = config.instance()
    nxt = list((t2.get('next_up') if player_up or ros_running else t2.get('next_down')) or [])
    flag = f' --instance {n}' if n else ''
    if n and (player_up or ros_running) and _read(run / 'domain'):   # {instance}: the line names it itself
        nxt = [cmd if '{instance}' in cmd else f"ROS_DOMAIN_ID={_read(run / 'domain')} {cmd}" for cmd in nxt]
    nxt = [config.subst(cmd, instance=flag) for cmd in nxt]
    return {
        **extra,
        'instance': n, 'nav_args': read_nav(run) or None, 'instances': None if n else other_instances(t2),
        'schema': 'crane.sim.status/1', 'domain': cfg.get('name'), 'mode': _read(run / 'mode') or None, 'vessel': config.vessel(t2),
        'ros_domain_id': _read(run / 'domain') or None, 'port': _read(run / 'port') or None,
        'player': {'running': player_up, 'pgid': _read(run / 'player.pid') if player_up else None,
                   'path': _read(run / 'player.path') or None},
        'ros': {'running': ros_running, 'container': container, 'launch': launch[1] if len(launch) > 1 else ''},
        'course': {'ready': ready, 'seed': _read(run / 'seed') or None, 'truth': str(run / truth)},
        'current': current or None, 'current_text': _current_text(t2, current) if current else None,
        'restarted': _read(run / 'restarted') or None,
        'logs': [str(run / n) for n in ((t1.LOG, 'ros.log') if is_t1 else ('player.log', 'player-stdout.log', 'ros.log'))],
        'next': nxt,
    }


def other_instances(t2):
    """The running instances, one dict each (instance, tier, ros_domain_id, container); [] when none."""
    found = []
    name = t2['ros']['container']
    for i in config.instances():
        container, path = f'{name}_{i}', config.state_base() / str(i)
        if ros_up(container):
            found.append({'instance': i, 'tier': 'T1' if _read(path / 'tier') == 'T1' else 'T2',
                          'ros_domain_id': _read(path / 'domain') or None, 'container': container})
    return found


def status_lines(s):
    """The status block, line for line as sim_up printed it."""
    lines = [f'{PROG} status', f"  mode:      {s['mode'] or '-'}"]
    if s.get('tier') == 'T1':
        lines += [f"  tier:      T1 (odom {s['odom']}, rate {t1.rate_text(s['rate'] or 1)})",
                  f"  vessel:    {s['vessel']}   ROS_DOMAIN_ID {s['ros_domain_id'] or '-'}   endpoint none (T1)"]
        w = s['world']
        pace = f"{w['rate']:g}x real time" if w['rate'] else 'starting'     # the first heartbeat has no rate yet
        name = '' if s.get('world_name', 'course') == 'course' else f"{s['world_name']}, "
        lines.append(f"  world:     {name}T1 node running, t {w['t_sim']:.1f} s sim, {pace}"
                     if w['running'] else f'  world:     {name}T1 node stopped')
    else:
        lines.append(f"  vessel:    {s['vessel']}   ROS_DOMAIN_ID {s['ros_domain_id'] or '-'}   endpoint 127.0.0.1:{s['port'] or '-'}")
        p = s['player']
        lines.append(f"  player:    running, pgid {p['pgid']} ({p['path'] or ''})" if p['running'] else '  player:    stopped')
    r = s['ros']
    lines.append(f"  ros:       container {r['container']} running ({r['launch']})" if r['running'] else '  ros:       stopped')
    if s['course']['ready'] and s.get('world_name', 'course') != 'course':
        lines.append('  course:    none (pool)')
    elif s['course']['ready']:
        lines.append(f"  course:    ready (seed {s['course']['seed'] or ''}, truth {s['course']['truth']})")
    if s.get('nav_args'):
        lines.append('  nav args:  ' + ' '.join(f'{k}:={v}' for k, v in s['nav_args'].items()))
    if s['current_text']:
        lines.append(f"  current:   {s['current_text']}")
    if s.get('restarted'):
        lines.append(f"  restarted: {s['restarted']}")
    lines.append(f"  logs:      {'  '.join(s['logs'])}")
    if s.get('instance'):
        lines.insert(1, f"  instance:  {s['instance']}")
    for o in s.get('instances') or []:
        lines.append(f"  instance {o['instance']}: {o['tier']}, ROS_DOMAIN_ID {o['ros_domain_id'] or '-'}, "
                     f"container {o['container']} running")
    for i, cmd in enumerate(s['next']):
        lines.append(f"  {'next:' if i == 0 else '':10s} {cmd}")
    return lines


def status(cfg, t2, run, as_json=False):
    s = state(cfg, t2, run)
    if as_json:
        print(json.dumps(s, indent=2))
    else:
        print('\n'.join(status_lines(s)), flush=True)
    return 0


GRACE_S = 10     # down: how long the player's process group gets after TERM before KILL


def stop_group(pg, grace_s=GRACE_S):
    """TERM the process group, poll every 0.5 s for up to grace_s, then KILL it: 'gone', 'term' or 'kill'."""
    try:
        os.killpg(pg, signal.SIGTERM)
    except OSError:
        return 'gone'
    deadline = time.monotonic() + grace_s
    while time.monotonic() < deadline:
        time.sleep(0.5)
        try:
            os.killpg(pg, 0)
        except OSError:
            return 'term'
    try:
        os.killpg(pg, signal.SIGKILL)
    except OSError:
        return 'term'
    return 'kill'


def down(cfg, t2, run, as_json=False, stream=None):
    """Stop the player's process group (TERM, up to GRACE_S, then KILL) and the domain's container."""
    pg, stopped = _read(run / 'player.pid'), None
    if pg:
        try:
            stopped = stop_group(int(pg))
        except ValueError:
            pass
        (run / 'player.pid').unlink(missing_ok=True)
    container = (t2.get('ros') or {}).get('container')
    try:
        if container:
            subprocess.run(['docker', 'rm', '-f', container], capture_output=True, timeout=60)
    except (OSError, subprocess.TimeoutExpired):
        pass
    for n in ('domain', 'port', 'current', 'player.argv', 'ros.docker', 'restarted', 'tier', 'odom', 'rate', 'world',
              'nav.args', 'ros.overrides', 'stack', t1.ALIVE):
        (run / n).unlink(missing_ok=True)
    if as_json:
        print(json.dumps({'schema': 'crane.sim.down/1', 'down': True, 'player': stopped, 'container': container, 'run': str(run)}))
    else:
        _say('down', stream)
    return 0


def in_container(path, container_home):
    """PATH with a leading $HOME as the container user's home (container_home), else unchanged."""
    host = str(config.home()).rstrip('/')
    if container_home and host and (path == host or path.startswith(host + '/')):
        return str(container_home).rstrip('/') + path[len(host):]
    return path


def launch_overrides(t2, run, port, nav=None):
    """The overrides JSON text the launch gets: the domain's, with the {port} filled and nav {name: value} merged
    under the recipe's args_part. Without nav the domain's text, unchanged."""
    text = config.subst((t2.get('ros') or {}).get('overrides') or '', port=port)
    return config.with_nav(text, nav, config.nav_part(t2), t2, run)


def docker_args(t2, run, d, port, img, nav=None):
    """The `docker run` argv of the ROS side: host network and IPC like the dev container, on domain d; nav
    {name: value} are the nav arguments for the first launch."""
    ros = t2.get('ros') or {}
    ws = ros.get('workspace') or '/workspace'
    values = {'ws': ws, 'run': run, 'dock': config.dock(t2) or '', 'home': config.home(), 'port': port,
              'ip': ros.get('ip') or '127.0.0.1', 'uid': os.getuid(), 'gid': os.getgid()}
    if ros.get('command'):                       # a stack that runs one command (crane's bridge), not a launch
        command = config.subst(ros['command'], **values)
        launch = command if command == 'wait' else 'exec ' + command    # wait: the T1 node in the background is all
    else:
        overrides = launch_overrides(t2, run, port, nav)
        launch = f"exec ros2 launch {ros.get('launch')}" + (f" overrides:='{overrides}'" if overrides else '')
    is_t1 = t2.get('tier') == 'T1'
    if is_t1:                                    # the T1 node in the background, beside the launch
        launch = t1.command(t2, run) + launch
    a = ['docker', 'run', '-d', '--name', ros['container']]
    if ros.get('label'):
        a += ['--label', ros['label']]
    a += ['--init', '--network', 'host', '--ipc', 'host']
    if ros.get('user'):
        a += ['--user', config.subst(ros['user'], **values)]
    a += ['-e', f'ROS_DOMAIN_ID={d}']
    for e in ros.get('env') or []:
        a += ['-e', config.subst(e, **values)]
    for name in ros.get('state_env') or []:      # the exported state path, as seen in the container
        if os.environ.get(name):
            a += ['-e', f'{name}={in_container(os.environ[name], ros.get("home"))}']
    for m in ros.get('mounts') or []:
        a += ['-v', config.subst(m, **values)]
    for m in t1.mounts(t2) if is_t1 else ():
        a += ['-v', m]
    a += ['-w', ws, '--entrypoint', 'bash', img, '-c', f'{launch} >{run}/ros.log 2>&1']
    return a


def launch_cmd(t2, guard, mode, player, args):
    """guard + the display wrapper + nice + the player: what runs in the player's own process group."""
    width, height = (t2.get('screen') or [1280, 720])[:2]
    view = ['env', 'SDL_VIDEODRIVER=x11']   # the owner's display (XWayland under Wayland)
    if mode == 'headless':                 # only xvfb's display: with WAYLAND_DISPLAY set, SDL would pick Wayland
        view = ['env', '-u', 'WAYLAND_DISPLAY', 'SDL_VIDEODRIVER=x11', 'xvfb-run', '-a', '-s',
                f'-screen 0 {width}x{height}x24']
    return [*guard, *view, 'nice', '-n', '10', player, *args]


def _tail(path, n=20):
    try:
        lines = Path(path).read_text(errors='replace').splitlines()[-n:]
    except OSError:
        return
    if lines:
        print('\n'.join(lines), file=sys.stderr, flush=True)


def display_mode(given=None):
    """--display, else a window only for a person at a terminal with a display; agents and scripts: headless."""
    if given and given != 'auto':
        return given
    on_tty = sys.stdin is not None and sys.stdin.isatty()
    return 'window' if on_tty and (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')) else 'headless'


def plan(cfg, t2, run, o, mode, current, as_json=False):
    """up --dry-run: the resolved player argv and the ROS side's docker run, and nothing starts. What cannot be
    resolved on this host (no player, no vessel profile, no image) shows as <...> with its one-line reason."""
    import shlex
    ros = t2.get('ros') or {}
    problems = []

    def get(fn, *a, placeholder):
        try:
            return fn(*a)
        except SimError as error:
            problems.append(str(error))
            return placeholder
    guard = get(config.heavy, t2, placeholder=['<heavy>', 'unity'])
    player = get(config.find_player, t2, o.player, placeholder='<player>')
    d = get(config.domain_id, t2, placeholder=None)
    port = config.port_base(t2) + (d or 0)
    img = get(config.image, t2, placeholder='<image>')
    args = resolver.resolve(cfg, env=o.env, robot=o.robot, card=o.card, seed=o.seed, profile=o.profile,
                            current=current, sets=o.sets, ros={'ip': ros.get('ip'), 'port': port},
                            extra=o.extra, out=run)
    player_argv = launch_cmd(t2, guard, mode, player, args)
    docker = docker_args(t2, run, '<domain>' if d is None else d, port, img, nav_from(o))
    doc = {'schema': 'crane.sim.plan/1', 'domain': cfg.get('name'), 'mode': mode, 'run': str(run),
           'ros_domain_id': d, 'port': port, 'player': player, 'player_argv': player_argv, 'docker_run': docker,
           'unresolved': problems}
    if as_json:
        print(json.dumps(doc, indent=2))
        return 0
    me = config.invoked_as()
    print(f'{me} up (dry run): nothing starts')
    for key in ('domain', 'mode', 'run', 'ros_domain_id', 'port', 'player'):
        print(f'  {key:14s} {doc[key] if doc[key] is not None else "-"}')
    print(f'  {"player argv":14s} {shlex.join(str(a) for a in player_argv)}')
    print(f'  {"docker run":14s} {shlex.join(str(a) for a in docker)}')
    for p in problems:
        print(f'  {"unresolved":14s} {p}')
    print(f'next: {me} up --{mode}' if not problems else f'next: fix the unresolved line(s), then {me} up --{mode}')
    return 0


def up(cfg, t2, run, o):
    """Start the ROS side, wait for its endpoint (stated timeout), start the player, wait for the course."""
    mode = display_mode(o.display)
    as_json = bool(getattr(o, 'json', False))
    say = (lambda m: _say(m, sys.stderr)) if as_json else _say
    me = config.invoked_as()
    ros = t2.get('ros') or {}
    container = ros.get('container')
    current = resolver.parse_current(o.current) if o.current is not None else None
    if t2.get('tier') == 'T1':
        return up_t1(cfg, t2, run, o, current)
    if getattr(o, 'dry_run', False):
        return plan(cfg, t2, run, o, mode, current, as_json)
    if alive(run) or ros_up(container):
        status(cfg, t2, run, as_json)
        raise SimError(f'already up; {me} down first')
    refuse_second_t2(t2, run)
    nav = nav_from(o)
    if nav:
        launch_overrides(t2, run, 0, nav)                 # a nav argument the domain cannot take is refused here
    guard = config.heavy(t2)
    if mode == 'headless' and not shutil.which('xvfb-run'):
        raise SimError('headless needs xvfb-run: install xvfb or use --window')
    if container and not shutil.which('docker'):
        raise SimError('no docker on PATH: install docker (the ROS side runs in a container), or --stack none')
    player = config.find_player(t2, o.player)
    d = config.domain_id(t2)
    port = config.port_base(t2) + d
    img = config.image(t2) if container else None
    if img:
        need_image(t2, img)
    args = resolver.resolve(cfg, env=o.env, robot=o.robot, card=o.card, seed=o.seed, profile=o.profile,
                            current=current, sets=o.sets, ros={'ip': ros.get('ip'), 'port': port},
                            extra=o.extra, out=run)
    seed = o.seed if o.seed is not None else t2.get('seed', 0)
    run.mkdir(parents=True, exist_ok=True)
    for p in [*run.glob('player*.log'), run / 'ros.log', run / ((t2.get('truth') or {}).get('file') or 'course_truth.json')]:
        p.unlink(missing_ok=True)
    for n in ('odom', 'rate', 'world', 'nav.args', 'ros.overrides', t1.ALIVE):
        (run / n).unlink(missing_ok=True)
    for name, value in (('current', current or 'default'), ('domain', d), ('port', port), ('mode', mode),
                        ('seed', seed), ('player.path', player), ('tier', 'T2'), ('stack', t2.get('stack') or '')):
        (run / name).write_text(f'{value}\n')
    remember(run, o)
    if container:
        start_ros(t2, run, d, port, img, nav, container, say, me)
    else:
        say('no ROS side (--stack none): the player runs alone')

    # 2. the player, through heavy unity, in its own process group (pid == pgid)
    argv = launch_cmd(t2, guard, mode, player, args)
    save_argv(run, 'player.argv', argv)
    proc = spawn_player(run, argv, f'{me} {mode}')
    limit = config.ready_s(t2)
    say(f'player queued on heavy unity ({mode}), waiting up to {limit} s for the course')
    outcome = wait_course(t2, run, proc, limit)
    if outcome == 'exited':
        _tail(run / 'player-stdout.log')
        down(cfg, t2, run, stream=sys.stderr if as_json else None)
        raise SimError(f'the player exited, see {run}/player.log', 1)
    if outcome == 'timeout':
        down(cfg, t2, run, stream=sys.stderr if as_json else None)
        raise SimError(f'course not ready in {limit} s, see {run}/player.log and {run}/player-stdout.log', 1)
    return status(cfg, t2, run, as_json)


def start_ros(t2, run, d, port, img, nav, container, say, me):
    """1. the ROS side, logged to the state dir; wait for its endpoint (stated timeout)."""
    ros = t2.get('ros') or {}
    subprocess.run(['docker', 'rm', '-f', container], capture_output=True, timeout=60)
    prepare_dirs(t2, run)
    docker = docker_args(t2, run, d, port, img, nav)
    save_nav(run, t2, d, port, nav)
    save_argv(run, 'ros.docker', docker)
    out = subprocess.run(docker, capture_output=True, text=True, timeout=120)
    if out.returncode != 0:
        last = (out.stderr.strip().splitlines() or [''])[-1]
        raise SimError(f'docker run failed (exit {out.returncode}): {last}; check the image: docker image ls {img}', 1)
    say(f'ROS side starting (domain {d}, endpoint port {port}, log {run}/ros.log)')
    limit, t0 = int(ros.get('endpoint_s', 60)), time.monotonic()
    while not listening(port):
        if not ros_up(container):
            _tail(run / 'ros.log')
            raise SimError(f'the ROS side exited, see {run}/ros.log', 1)
        if time.monotonic() - t0 >= limit:
            raise SimError(f'no endpoint on port {port} in {limit} s, see {run}/ros.log; then {me} down', 1)
        time.sleep(1)


def remember(run, o):
    """recipe.json: which sim this is and who started it, so a later up can reuse it (crane/sim/place.py)."""
    if getattr(o, 'recipe', None) is not None:
        from crane.sim import place
        place.write(run, o.recipe, place.caller())


def need_image(t2, img):
    """The stack's image is on this machine, else one line naming how to get it (build or pull)."""
    try:
        out = subprocess.run(['docker', 'image', 'inspect', img], capture_output=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired):
        return
    if out.returncode == 0:
        return
    build = ((t2.get('ros') or {}).get('image') or {}).get('build')
    fix = f'docker build -t {img} {paths.ROOT / build}' if build else f'docker pull {img}'
    raise SimError(f'no image {img} on this machine: {fix}')


def up_t1(cfg, sec, run, o, current):
    """The T1 world: write the node's config, start the ROS container with the node beside the launch, wait for the
    node's ready line (stated timeout). No player, no endpoint."""
    as_json = bool(getattr(o, 'json', False))
    say = (lambda m: _say(m, sys.stderr)) if as_json else _say
    me = config.invoked_as()
    container = sec['ros']['container']
    dry = getattr(o, 'dry_run', False)
    if not dry and (alive(run) or ros_up(container)):
        status(cfg, sec, run, as_json)
        raise SimError(f'already up; {me} down first')
    if not dry and not shutil.which('docker'):
        raise SimError('no docker on PATH: install docker (the ROS side runs in a container)')
    problems = []

    def get(fn, *a, placeholder):
        if not dry:
            return fn(*a)
        try:
            return fn(*a)
        except SimError as error:
            problems.append(str(error))
            return placeholder
    if current is None and t1.parse_world(sec, getattr(o, 'world', None)):
        current = '0'                           # a pool has still water unless --current says otherwise
    d = get(config.domain_id, sec, placeholder='<domain>')
    img = get(config.image, sec, placeholder='<image>')
    seed = o.seed if o.seed is not None else sec.get('seed', 0)
    run.mkdir(parents=True, exist_ok=True)
    if not dry:
        for p in [*run.glob('player*.log'), run / 'ros.log', run / t1.LOG, run / t1.ALIVE, run / 'port',
                  run / 'player.pid', run / 'player.argv', run / 'player.path',
                  run / ((sec.get('truth') or {}).get('file') or 'course_truth.json')]:
            p.unlink(missing_ok=True)
    if dry:                                     # the course goes to a scratch file: a running world keeps its own
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            doc = t1.resolve(cfg, sec, run, o, current, truth=Path(tmp) / 'course_truth.json')
    else:
        doc = t1.resolve(cfg, sec, run, o, current)
    nav = nav_from(o)
    docker = docker_args(sec, run, d, '', img, nav)
    if dry:
        plan_doc = {'schema': 'crane.sim.plan/1', 'domain': cfg.get('name'), 'tier': 'T1', 'odom': doc['odom'],
                    'rate': doc['rate'], 'run': str(run), 'ros_domain_id': None if problems else d,
                    'docker_run': docker, 'card': doc['card'], 'course': doc['course'],
                    'course_source': doc['course_source'], 'current': doc['current'], 'lever': doc['lever'],
                    'unresolved': problems}
        if as_json:
            print(json.dumps(plan_doc, indent=2))
            return 0
        import shlex
        print(f'{me} up --tier T1 (dry run): nothing starts')
        for key in ('domain', 'odom', 'rate', 'run', 'ros_domain_id', 'card', 'course', 'course_source', 'current'):
            print(f'  {key:14s} {plan_doc[key] if plan_doc[key] is not None else "-"}')
        print(f'  {"docker run":14s} {shlex.join(str(a) for a in docker)}')
        for p in problems:
            print(f'  {"unresolved":14s} {p}')
        print(f'next: {me} up --tier T1' if not problems else f'next: fix the unresolved line(s), then {me} up --tier T1')
        return 0
    t1.write(run, doc)
    for n in ('nav.args', 'ros.overrides'):
        (run / n).unlink(missing_ok=True)
    save_nav(run, sec, d, '', nav)
    world_name = t1.world_text((doc['pool']['length'], doc['pool']['width']) if doc.get('pool') else None)
    for name, value in (('tier', 'T1'), ('odom', doc['odom']), ('rate', f"{doc['rate']:g}"), ('world', world_name),
                        ('domain', d),
                        ('mode', 'headless'), ('seed', seed), ('current', current or 'default'),
                        ('stack', sec.get('stack') or '')):
        (run / name).write_text(f'{value}\n')
    remember(run, o)
    need_image(sec, img)
    subprocess.run(['docker', 'rm', '-f', container], capture_output=True, timeout=60)
    prepare_dirs(sec, run)
    save_argv(run, 'ros.docker', docker)
    out = subprocess.run(docker, capture_output=True, text=True, timeout=120)
    if out.returncode != 0:
        last = (out.stderr.strip().splitlines() or [''])[-1]
        raise SimError(f'docker run failed (exit {out.returncode}): {last}; check the image: docker image ls {img}', 1)
    limit = int(sec.get('ready_s') or 60)
    say(f"ROS side and T1 node starting (domain {d}, odom {doc['odom']}, {t1.rate_text(doc['rate'])}, course "
        f"{doc['course']} from {doc['course_source']}, log {run}/{t1.LOG}), waiting up to {limit} s for the world")
    outcome = wait_world(sec, run, limit)
    if outcome != 'ready':
        _tail(run / t1.LOG)
        down(cfg, sec, run, stream=sys.stderr if as_json else None)
        why = 'the ROS side exited' if outcome == 'exited' else f'the T1 node not ready in {limit} s'
        raise SimError(f'{why}, see {run}/{t1.LOG} and {run}/ros.log', 1)
    return status(cfg, sec, run, as_json)


def wait_world(sec, run, limit):
    """Poll once a second until the T1 node's ready line is in its log: 'ready', 'exited' (the container went) or
    'timeout' after limit s."""
    marker, container, t0 = sec.get('ready_marker') or '', sec['ros']['container'], time.monotonic()
    while True:
        try:
            if marker and marker in (run / t1.LOG).read_text(errors='replace'):
                return 'ready'
        except OSError:
            pass
        if not ros_up(container):
            return 'exited'
        if time.monotonic() - t0 >= limit:
            return 'timeout'
        time.sleep(1)


def prepare_dirs(t2, run):
    """The host dirs the ROS container mounts, made before docker run."""
    ros = t2.get('ros') or {}
    values = {'ws': ros.get('workspace') or '', 'run': run, 'dock': config.dock(t2) or '', 'home': config.home()}
    for p in ros.get('dirs') or []:
        Path(config.subst(p, **values)).mkdir(parents=True, exist_ok=True)


def nav_from(o):
    """The nav arguments up was given (--nav NAME:=VALUE, --nav-args 'A:=x B:=y'), as {name: value}."""
    return config.parse_nav(getattr(o, 'nav', None), getattr(o, 'nav_args', None))


def read_nav(run):
    """The nav arguments the world was started (or last restarted) with, {} when none."""
    try:
        return json.loads((run / 'nav.args').read_text())
    except (OSError, ValueError):
        return {}


def save_nav(run, t2, d, port, nav):
    """Keep what restart nav needs: the overrides the world launched with (before nav args), and the nav args."""
    ros = t2.get('ros') or {}
    (run / 'ros.overrides').write_text(config.subst(ros.get('overrides') or '', port=port) + '\n')
    if nav:
        (run / 'nav.args').write_text(json.dumps(nav) + '\n')


def refuse_second_t2(t2, run):
    """An instance's T2 world needs the one Unity player to be free: refuse while another T2 world runs."""
    n = config.instance()
    if not n:
        return
    name = t2['ros']['container']
    base = name[:-len(f'_{n}')]
    others = [(config.state_base(), base)] + [(config.state_base() / str(i), f'{base}_{i}')
                                               for i in config.instances() if i != n]
    for path, container in others:
        if path != run and _read(path / 'tier') != 'T1' and (alive(path) or ros_up(container)):
            raise SimError(f'a T2 world is already running ({container}): the one Unity player runs one at a time; '
                           f'{config.invoked_as()} down first, or use --tier T1 for side-by-side worlds')


def save_argv(run, name, argv):
    """The resolved argv of what up started, for crane sim restart (identical args)."""
    (run / name).write_text(json.dumps([str(a) for a in argv]) + '\n')


def saved_argv(run, name):
    """The argv up saved, or None."""
    try:
        argv = json.loads((run / name).read_text())
    except (OSError, ValueError):
        return None
    return argv if isinstance(argv, list) and argv else None


def spawn_player(run, argv, who):
    """Start the player argv in its own process group (pid == pgid) with its stdout logged; writes player.pid."""
    env = dict(os.environ, HEAVY_WHO=who)
    with open(run / 'player-stdout.log', 'wb') as log:
        proc = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, env=env,
                                start_new_session=True)
    (run / 'player.pid').write_text(f'{proc.pid}\n')
    return proc


def wait_course(t2, run, proc, limit):
    """Poll once a second until the ready_marker is in the player log: 'ready', 'exited' or 'timeout' after limit s."""
    marker, t0 = t2.get('ready_marker') or '', time.monotonic()
    while True:
        try:
            if marker and marker in (run / 'player.log').read_text(errors='replace'):
                return 'ready'
        except OSError:
            pass
        if proc.poll() is not None:
            return 'exited'
        if time.monotonic() - t0 >= limit:
            return 'timeout'
        time.sleep(1)


LOGS = {'player': 'player.log', 'stdout': 'player-stdout.log', 'ros': 'ros.log', 't1': t1.LOG}


def logs(run, which=(), lines=40, tier='T2'):
    """The last LINES of each log (the tier's by default), each under its path."""
    shown = 0
    for key in which or (('t1', 'ros') if tier == 'T1' else ('player', 'stdout', 'ros')):
        path = run / LOGS[key]
        print(f'==> {path} <==')
        try:
            text = path.read_text(errors='replace').splitlines()
            print('\n'.join(text[-lines:]) if text else '(empty)')
            shown += 1
        except OSError:
            print('(none yet)')
    if not shown:
        print(f'next: {config.invoked_as()} up   (no logs in {run})')
    return 0
