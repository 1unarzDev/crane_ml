"""crane sim up|down|status|logs: the T2 world (the ROS side in its container, the player on heavy unity).

The state dir (config.run_dir, ~/.cache/sim_up) holds what down and status read: player.pid (the player's
process group), player.path, mode, domain, port, seed, current, the logs and the course truth. down stops only
that process group and the domain's own container, never a tmux server or another container.
"""

import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

from crane import player as resolver
from crane.sim import config
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
    player_up, ros_running = alive(run), ros_up(container)
    log = run / 'player.log'
    marker = t2.get('ready_marker') or ''
    try:
        ready = bool(marker) and marker in log.read_text(errors='replace')
    except OSError:
        ready = False
    current = _read(run / 'current')
    truth = (t2.get('truth') or {}).get('file') or 'course_truth.json'
    launch = str(ros.get('launch') or '').split(None, 1)
    return {
        'domain': cfg.get('name'), 'mode': _read(run / 'mode') or None, 'vessel': config.vessel(t2),
        'ros_domain_id': _read(run / 'domain') or None, 'port': _read(run / 'port') or None,
        'player': {'running': player_up, 'pgid': _read(run / 'player.pid') if player_up else None,
                   'path': _read(run / 'player.path') or None},
        'ros': {'running': ros_running, 'container': container, 'launch': launch[1] if len(launch) > 1 else ''},
        'course': {'ready': ready, 'seed': _read(run / 'seed') or None, 'truth': str(run / truth)},
        'current': current or None, 'current_text': _current_text(t2, current) if current else None,
        'logs': [str(run / n) for n in ('player.log', 'player-stdout.log', 'ros.log')],
        'next': list((t2.get('next_up') if player_up or ros_running else t2.get('next_down')) or []),
    }


def status_lines(s):
    """The status block, line for line as sim_up printed it."""
    lines = [f'{PROG} status', f"  mode:      {s['mode'] or '-'}",
             f"  vessel:    {s['vessel']}   ROS_DOMAIN_ID {s['ros_domain_id'] or '-'}   endpoint 127.0.0.1:{s['port'] or '-'}"]
    p = s['player']
    lines.append(f"  player:    running, pgid {p['pgid']} ({p['path'] or ''})" if p['running'] else '  player:    stopped')
    r = s['ros']
    lines.append(f"  ros:       container {r['container']} running ({r['launch']})" if r['running'] else '  ros:       stopped')
    if s['course']['ready']:
        lines.append(f"  course:    ready (seed {s['course']['seed'] or ''}, truth {s['course']['truth']})")
    if s['current_text']:
        lines.append(f"  current:   {s['current_text']}")
    lines.append(f"  logs:      {'  '.join(s['logs'])}")
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
        subprocess.run(['docker', 'rm', '-f', container], capture_output=True, timeout=60)
    except (OSError, subprocess.TimeoutExpired):
        pass
    for n in ('domain', 'port', 'current'):
        (run / n).unlink(missing_ok=True)
    if as_json:
        print(json.dumps({'down': True, 'player': stopped, 'container': container, 'run': str(run)}))
    else:
        _say('down', stream)
    return 0


def docker_args(t2, run, d, port, img):
    """The `docker run` argv of the ROS side: host network and IPC like the dev container, on domain d."""
    ros = t2.get('ros') or {}
    ws = ros.get('workspace') or '/workspace'
    values = {'ws': ws, 'run': run, 'dock': config.dock(t2) or '', 'home': config.home(), 'port': port}
    overrides = config.subst(ros.get('overrides') or '', port=port)
    launch = f"exec ros2 launch {ros.get('launch')}" + (f" overrides:='{overrides}'" if overrides else '')
    a = ['docker', 'run', '-d', '--name', ros['container']]
    if ros.get('label'):
        a += ['--label', ros['label']]
    a += ['--init', '--network', 'host', '--ipc', 'host']
    if ros.get('user'):
        a += ['--user', ros['user']]
    a += ['-e', f'ROS_DOMAIN_ID={d}']
    for e in ros.get('env') or []:
        a += ['-e', config.subst(e, **values)]
    for m in ros.get('mounts') or []:
        a += ['-v', config.subst(m, **values)]
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
    if given:
        return given
    on_tty = sys.stdin is not None and sys.stdin.isatty()
    return 'window' if on_tty and (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')) else 'headless'


def up(cfg, t2, run, o):
    """Start the ROS side, wait for its endpoint (stated timeout), start the player, wait for the course."""
    mode = display_mode(o.display)
    as_json = bool(getattr(o, 'json', False))
    say = (lambda m: _say(m, sys.stderr)) if as_json else _say
    me = config.invoked_as()
    ros = t2.get('ros') or {}
    container = ros['container']
    current = resolver.parse_current(o.current) if o.current is not None else None
    if alive(run) or ros_up(container):
        status(cfg, t2, run, as_json)
        raise SimError(f'already up; {me} down first')
    guard = config.heavy(t2)
    if mode == 'headless' and not shutil.which('xvfb-run'):
        raise SimError('headless needs xvfb-run: install xvfb or use --window')
    if not shutil.which('docker'):
        raise SimError('no docker on PATH: install docker (the ROS side runs in a container)')
    player = config.find_player(t2, o.player)
    d = config.domain_id(t2)
    port = int(ros.get('port_base', 10000)) + d
    img = config.image(t2)
    args = resolver.resolve(cfg, env=o.env, robot=o.robot, card=o.card, seed=o.seed, profile=o.profile,
                            current=current, sets=o.sets, ros={'ip': ros.get('ip'), 'port': port},
                            extra=o.extra, out=run)
    seed = o.seed if o.seed is not None else t2.get('seed', 0)
    run.mkdir(parents=True, exist_ok=True)
    for p in [*run.glob('player*.log'), run / 'ros.log', run / ((t2.get('truth') or {}).get('file') or 'course_truth.json')]:
        p.unlink(missing_ok=True)
    for name, value in (('current', current or 'default'), ('domain', d), ('port', port), ('mode', mode),
                        ('seed', seed), ('player.path', player)):
        (run / name).write_text(f'{value}\n')

    # 1. the ROS side, logged to the state dir
    subprocess.run(['docker', 'rm', '-f', container], capture_output=True, timeout=60)
    values = {'ws': ros.get('workspace') or '', 'run': run, 'dock': config.dock(t2) or '', 'home': config.home()}
    for p in ros.get('dirs') or []:
        Path(config.subst(p, **values)).mkdir(parents=True, exist_ok=True)
    out = subprocess.run(docker_args(t2, run, d, port, img), capture_output=True, text=True, timeout=120)
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

    # 2. the player, through heavy unity, in its own process group (pid == pgid)
    env = dict(os.environ, HEAVY_WHO=f'{me} {mode}')
    with open(run / 'player-stdout.log', 'wb') as log:
        proc = subprocess.Popen(launch_cmd(t2, guard, mode, player, args), stdin=subprocess.DEVNULL, stdout=log,
                                stderr=subprocess.STDOUT, env=env, start_new_session=True)
    (run / 'player.pid').write_text(f'{proc.pid}\n')
    limit = config.ready_s(t2)
    say(f'player queued on heavy unity ({mode}), waiting up to {limit} s for the course')
    marker, t0 = t2.get('ready_marker') or '', time.monotonic()
    while True:
        try:
            if marker and marker in (run / 'player.log').read_text(errors='replace'):
                break
        except OSError:
            pass
        if proc.poll() is not None:
            _tail(run / 'player-stdout.log')
            down(cfg, t2, run, stream=sys.stderr if as_json else None)
            raise SimError(f'the player exited, see {run}/player.log', 1)
        if time.monotonic() - t0 >= limit:
            down(cfg, t2, run, stream=sys.stderr if as_json else None)
            raise SimError(f'course not ready in {limit} s, see {run}/player.log and {run}/player-stdout.log', 1)
        time.sleep(1)
    return status(cfg, t2, run, as_json)


LOGS = {'player': 'player.log', 'stdout': 'player-stdout.log', 'ros': 'ros.log'}


def logs(run, which=(), lines=40):
    """The last LINES of each log (all three by default), each under its path."""
    shown = 0
    for key in which or LOGS:
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
