"""`crane sim check`: is the running sim (crane sim up / astro sim up) ready to be used like the robot?

    crane sim check [--wait SECONDS] [--json] [--domain D]   poll every 2 s until ready or the timeout
    crane sim check CHECK INPUTS...                          an offline Tools/SimCheck check (axis, sitl, ...)

Rows: the player runs, the course is ready (the domain's ready_marker), the plant was applied (plant_marker),
the ROS container runs, its ready_nodes are in `ros2 node list`, the player log has no exceptions.
Exit 0 ready, 1 not ready, 2 usage. Stdlib only (plus PyYAML for the domain config).
"""

import json
import os
import re
import subprocess
import sys
import time

from crane.sim import config

RUN = config.run_dir()
ERROR = re.compile(r'\b(NullReferenceException|MissingReferenceException|IndexOutOfRangeException)\b')
WARN = re.compile(r'\bCRANE_[A-Z_]*(FAILED|INVALID)\b')
_T2 = None   # the domain's T2 section; main() sets it, else the default domain's


def _t2():
    global _T2
    if _T2 is None:
        _T2 = config.domain()[1]
    return _T2


def _ros():
    return _t2().get('ros') or {}


def _container():
    return _ros().get('container') or 'crane-sim-ros'


def _alive(pid_file):
    try:
        os.killpg(int(pid_file.read_text().strip()), 0)
        return True
    except (OSError, ValueError):
        return False


def _container_running():
    try:
        out = subprocess.run(['docker', 'inspect', '-f', '{{.State.Running}}', _container()],
                             capture_output=True, text=True, timeout=10)
        return out.returncode == 0 and out.stdout.strip() == 'true'
    except (OSError, subprocess.TimeoutExpired):
        return False


def _nodes():
    """ros2 node list from inside the ROS container, or None when it cannot be read (30 s timeout)."""
    try:
        out = subprocess.run(['docker', 'exec', _container(), 'bash', '-c',
                              'source $BASH_ENV 2>/dev/null; ros2 node list'],
                             capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return out.stdout.splitlines() if out.returncode == 0 else None


def live(run=None):
    """[(name, ok, detail)] for the running sim. The player's log is read only while the player runs."""
    run = run or RUN
    t2, container = _t2(), _container()
    ready, plant = t2.get('ready_marker') or 'CRANE_READY', t2.get('plant_marker') or 'CRANE_PLANT_APPLIED'
    want = tuple(_ros().get('ready_nodes') or ())
    truth = (t2.get('truth') or {}).get('file') or 'course_truth.json'
    log = run / 'player.log'
    running = _alive(run / 'player.pid')
    last = (run / 'player.path').read_text().strip() if (run / 'player.path').is_file() else 'no player started yet'
    text = log.read_text(errors='replace') if running and log.is_file() else ''
    errors = [line for line in text.splitlines() if ERROR.search(line)]
    up = _container_running()
    nodes = _nodes() if up else None
    missing = [n for n in want if not any(n in line for line in (nodes or []))]
    gone = 'player not running'
    return [
        ('player running', running, f'running ({last})' if running else f'not running (last: {last}); crane sim up --headless'),
        ('course ready', ready in text, str(run / truth) if running else gone),
        ('plant applied', plant in text,
         next((l.strip() for l in text.splitlines() if plant in l), f'no {plant} line' if running else gone)),
        ('ROS side running', up, f'container {container}' + ('' if up else ' is not running')),
        ('nav stack up', up and nodes is not None and not missing,
         (f'{", ".join(want)}' if not missing else f'no {", ".join(missing)} in ros2 node list')
         if up and nodes is not None else f'cannot read ros2 node list in {container}'),
        ('no player errors', running and not errors, (errors[0][:160] if errors else str(log)) if running else gone),
    ]


def notices(run=None):
    """The player's CRANE_*FAILED / CRANE_*INVALID lines, while it runs."""
    run = run or RUN
    log = run / 'player.log'
    if not (_alive(run / 'player.pid') and log.is_file()):
        return []
    return sorted({m.group(0) for m in map(WARN.search, log.read_text(errors='replace').splitlines()) if m})


def _report(wait):
    """The rows and the failed names; with wait, poll every 2 s until ready or WAIT s pass."""
    deadline = time.monotonic() + wait
    while True:
        rows = live()
        bad = [name for name, ok, _ in rows if not ok]
        if not bad or time.monotonic() >= deadline:
            return rows, bad
        time.sleep(2)


def _usage(message):
    print(f'crane sim check: {message} (crane sim check --help)', file=sys.stderr)
    return 2


def main(rest):
    global _T2
    rest = list(rest)
    if rest[:1] in (['-h'], ['--help']):
        print(__doc__.strip())
        return 0
    if rest and not rest[0].startswith('-'):        # an offline Tools/SimCheck check
        from crane import paths
        from crane.wrap import execute
        return execute([sys.executable, '-m', 'Tools.SimCheck', *rest], python_path=True) if paths.ROOT else 2
    wait, as_json, name = 0.0, False, None
    while rest:
        arg = rest.pop(0)
        key, eq, value = arg.partition('=')
        if key in ('--wait', '--domain') and not eq:
            if not rest:
                return _usage(f'{key} wants a value')
            value = rest.pop(0)
        if key == '--wait':
            try:
                wait = float(value)
            except ValueError:
                return _usage(f'--wait wants SECONDS, got {value!r}')
        elif key == '--domain':
            name = value
        elif arg == '--json':
            as_json = True
        else:
            return _usage(f'unknown option {arg}')
    if name:
        try:
            _T2 = config.domain(name)[1]
        except config.SimError as error:
            print(f'crane sim check: {error}', file=sys.stderr)
            return error.code
    rows, bad = _report(wait)
    warns = notices()
    if as_json:
        print(json.dumps({'ready': not bad, 'failed': bad, 'notices': warns, 'run': str(RUN),
                          'checks': [{'name': n, 'ok': bool(ok), 'detail': d} for n, ok, d in rows]}, indent=2))
        return 0 if not bad else 1
    for n, ok, detail in rows:
        print(f"  {'ok  ' if ok else 'FAIL'}  {n:18s} {detail}")
    if warns:
        print(f"  warn  player notices     {', '.join(warns)} (see {RUN / 'player.log'})")
    print('sim check: ready' if not bad else f"sim check: not ready, failed: {', '.join(bad)}; see `crane sim status`")
    return 0 if not bad else 1
