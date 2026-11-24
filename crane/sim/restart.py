"""crane sim restart nav|ros|player: restart one part of the running world, keeping the others.

    nav     only the navigation nodes, inside the running ROS container (the domain's ros.restart.nav recipe:
            stop them, start them again, wait for the ready nodes), so new nav2_limits in the state take effect
    ros     the ROS container, with the args up used; the player stays and its endpoint reconnects
    player  the player, with the argv up resolved; the ROS side stays

Each step prints one line; waits are bounded and state their timeout. The state dir is updated (status and check
read it). Exit 0 ok, 1 the part failed, 2 usage or config (also: nothing running), 75 timed out waiting.
"""

import json
import os
import shlex
import subprocess
import sys
import time
from datetime import datetime, timezone

from crane.sim import config, world
from crane.sim.config import SimError

WHAT = ('nav', 'ros', 'player')


def _say(msg, as_json):
    world._say(msg, sys.stderr if as_json else None)


def dexec(t2, run, argv, detach=False, timeout=60):
    """docker exec into the ROS container (no shell unless argv says so); the astro state path as exported now."""
    ros = t2.get('ros') or {}
    cmd = ['docker', 'exec']
    if detach:
        cmd.append('-d')
    for name in ros.get('state_env') or []:
        if os.environ.get(name):
            cmd += ['-e', f'{name}={world.in_container(os.environ[name], ros.get("home"))}']
    return subprocess.run([*cmd, ros['container'], *argv], capture_output=True, text=True, timeout=timeout)


def node_list(t2, run):
    """ros2 node list inside the container, or None when it cannot be read."""
    try:
        out = dexec(t2, run, ['bash', '-c', 'source $BASH_ENV 2>/dev/null; ros2 node list'], timeout=30)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return out.stdout.splitlines() if out.returncode == 0 else None


def _node_ready(t2, run, node, ready):
    """The node answers its lifecycle state with the expected word, else it is in ros2 node list."""
    if ready.get('state'):
        cmd = config.subst(ready['state'], node=node)
        try:
            out = dexec(t2, run, ['bash', '-c', f'source $BASH_ENV 2>/dev/null; {cmd}'], timeout=30)
        except (OSError, subprocess.TimeoutExpired):
            return False
        return out.returncode == 0 and out.stdout.split()[:1] == [ready.get('expect', '')]   # 'active [3]', not 'inactive'
    return any(line.strip().endswith('/' + node) for line in node_list(t2, run) or [])


def wait_nodes(t2, run, nodes, limit, ready=None, poll=2.0):
    """Poll until every node is ready (see _node_ready): the missing names, [] once ready, after at most limit s."""
    ready = ready or {}
    deadline = time.monotonic() + limit
    while True:
        if ready.get('state'):
            missing = [n for n in nodes if not _node_ready(t2, run, n, ready)]
        else:
            seen = node_list(t2, run) or []
            missing = [n for n in nodes if not any(line.strip().endswith('/' + n) for line in seen)]
        if not missing or time.monotonic() >= deadline:
            return missing
        time.sleep(poll)


def _record(run, what):
    stamp = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
    (run / 'restarted').write_text(f'{what} {stamp}\n')
    return stamp


def _argv(template, **values):
    """A recipe command as argv: split first, then fill (a {match} regex stays one word)."""
    return [config.subst(word, **values) for word in shlex.split(template)]


def _need_container(t2):
    me = config.invoked_as()
    if not world.ros_up(t2['ros']['container']):
        raise SimError(f'the ROS container {t2["ros"]["container"]} is not running: {me} up', 2)
    return me


def nav(cfg, t2, run, as_json=False):
    """Stop the recipe's nav processes in the container, start them again, wait for the ready nodes."""
    ros = t2.get('ros') or {}
    me = config.invoked_as()
    recipe = (ros.get('restart') or {}).get('nav')
    if not recipe:
        raise SimError(f'no ros.restart.nav recipe in config/domains/{cfg.get("name")}.yaml: {me} restart ros '
                       'restarts the whole ROS side instead', 2)
    _need_container(t2)
    values = {'match': recipe['match']} if recipe.get('match') else {}
    stop_s = float(recipe.get('stop_s', 20))

    def run_all(commands):
        for cmd in commands:                               # pkill exits 1 when nothing matched: fine
            dexec(t2, run, _argv(cmd, **values))

    def gone(limit):
        deadline = time.monotonic() + limit
        while dexec(t2, run, _argv(recipe['alive'], **values)).returncode == 0:
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.5)
        return True

    _say(f'nav stopping (up to {stop_s:g} s, then KILL and up to {stop_s:g} s more)', as_json)
    run_all(recipe.get('stop') or [])
    if not recipe.get('alive') or not (gone(stop_s) or (run_all(recipe.get('kill') or []) or gone(stop_s))):
        raise SimError(f'the nav processes did not exit (see {run}/ros.log); not starting a second set: '
                       f'{me} restart ros', 1)
    overrides = config.subst(ros.get('overrides') or '', port=_read_port(run))
    start_values = {'launch': ros.get('launch') or '', 'overrides': shlex.quote(overrides)}
    with_log = ' >>{} 2>&1'.format(shlex.quote(str(run / 'ros.log')))
    for cmd in recipe.get('start') or []:
        text = config.subst(cmd, **start_values)
        out = dexec(t2, run, ['bash', '-c', f'echo "--- crane sim restart nav" {with_log.strip()}; exec {text}{with_log}'],
                    detach=True)
        if out.returncode != 0:
            raise SimError(f'docker exec failed (exit {out.returncode}): '
                           f'{(out.stderr.strip().splitlines() or [""])[-1]}', 1)
    ready = recipe.get('ready') or {}
    nodes, limit = list(ready.get('nodes') or []), float(recipe.get('ready_s', 90))
    _say(f'nav started, waiting up to {limit:g} s for {", ".join(nodes) or "nothing"}', as_json)
    missing = wait_nodes(t2, run, nodes, limit, ready)
    stamp = _record(run, 'nav')
    if missing:
        raise SimError(f'not ready in {limit:g} s: {", ".join(missing)}; see {run}/ros.log; then {me} check', 75)
    return {'restarted': 'nav', 'ready': nodes, 'at': stamp}


def ros(cfg, t2, run, as_json=False):
    """Remove and re-run the ROS container with the argv up saved; wait for the endpoint and the ready nodes."""
    ros_cfg = t2.get('ros') or {}
    me = config.invoked_as()
    container = ros_cfg['container']
    docker = world.saved_argv(run, 'ros.docker')
    if not docker:
        raise SimError(f'no saved ROS launch in {run}: {me} up', 2)
    port = int(_read_port(run))
    world.prepare_dirs(t2, run)
    subprocess.run(['docker', 'rm', '-f', container], capture_output=True, timeout=60)
    out = subprocess.run(docker, capture_output=True, text=True, timeout=120)
    if out.returncode != 0:
        raise SimError(f'docker run failed (exit {out.returncode}): {(out.stderr.strip().splitlines() or [""])[-1]}', 1)
    limit = int(ros_cfg.get('endpoint_s', 60))
    _say(f'ROS side restarting (endpoint port {port}, the player is kept), waiting up to {limit} s for the endpoint',
         as_json)
    t0 = time.monotonic()
    while not world.listening(port):
        if not world.ros_up(container):
            world._tail(run / 'ros.log')
            raise SimError(f'the ROS side exited, see {run}/ros.log', 1)
        if time.monotonic() - t0 >= limit:
            raise SimError(f'no endpoint on port {port} in {limit} s, see {run}/ros.log; then {me} down', 75)
        time.sleep(1)
    nodes, limit = list(ros_cfg.get('ready_nodes') or []), float(ros_cfg.get('ready_s', 120))
    _say(f'endpoint up, waiting up to {limit:g} s for {", ".join(nodes) or "nothing"}', as_json)
    missing = wait_nodes(t2, run, nodes, limit)
    stamp = _record(run, 'ros')
    if missing:
        raise SimError(f'not ready in {limit:g} s: {", ".join(missing)}; see {run}/ros.log; then {me} check', 75)
    return {'restarted': 'ros', 'ready': nodes, 'at': stamp}


def _read_port(run):
    try:
        return (run / 'port').read_text().strip()
    except OSError:
        raise SimError('no endpoint port in the state dir: crane sim up', 2) from None


def player(cfg, t2, run, as_json=False):
    """Stop the player's process group and start the argv up saved, in a new one; wait for the course."""
    me = config.invoked_as()
    argv = world.saved_argv(run, 'player.argv')
    if not argv:
        raise SimError(f'no saved player launch in {run}: {me} up', 2)
    _need_container(t2)
    pg = world._read(run / 'player.pid')
    if pg:
        try:
            how = world.stop_group(int(pg))
        except ValueError:
            how = 'gone'
        (run / 'player.pid').unlink(missing_ok=True)
        _say(f'player stopped ({how})', as_json)
    truth = (t2.get('truth') or {}).get('file') or 'course_truth.json'
    for p in [*run.glob('player*.log'), run / truth]:
        p.unlink(missing_ok=True)
    mode = world._read(run / 'mode') or 'headless'
    proc = world.spawn_player(run, argv, f'{me} {mode}')
    limit = config.ready_s(t2)
    _say(f'player queued on heavy unity ({mode}), waiting up to {limit} s for the course', as_json)
    outcome = world.wait_course(t2, run, proc, limit)
    stamp = _record(run, 'player')
    if outcome == 'exited':
        world._tail(run / 'player-stdout.log')
        (run / 'player.pid').unlink(missing_ok=True)
        raise SimError(f'the player exited, see {run}/player.log; then {me} restart player', 1)
    if outcome == 'timeout':
        raise SimError(f'course not ready in {limit} s, see {run}/player.log and {run}/player-stdout.log; then '
                       f'{me} check --wait 120', 75)
    return {'restarted': 'player', 'ready': [t2.get('ready_marker')], 'at': stamp}


def run_restart(cfg, t2, run, what, as_json=False):
    """Restart WHAT and print the result; the exit code (SimError carries its own)."""
    me = config.invoked_as()
    result = {'nav': nav, 'ros': ros, 'player': player}[what](cfg, t2, run, as_json)
    if as_json:
        print(json.dumps({'schema': 'crane.sim.restart/1', **result, 'status': world.state(cfg, t2, run)}, indent=2))
    else:
        print(f'{me}: {what} restarted, ready: {", ".join(str(r) for r in result["ready"])}')
        print(f'next: {me} check')
    return 0
