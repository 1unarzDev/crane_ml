"""`crane sim check`: is the running sim (sim_up / astro sim up) ready to be used like the robot?

With no arguments it checks the live sim; `crane sim check CHECK ...` runs one of Tools/SimCheck's offline
acceptance checks (axis, sitl, sitl-peer, docking, smoke) on recorded results. Stdlib only.

    crane sim check [--wait SECONDS]    poll until ready (or the timeout), e.g. right after `crane sim up`
"""

import os
import re
import subprocess
import sys
import time
from pathlib import Path

RUN = Path(os.environ.get('SIM_UP_RUN', Path.home() / '.cache' / 'sim_up'))
CONTAINER = 'sim_up_ros'
NAV_NODES = ('ekf_local', 'rtabmap', 'bt_navigator')
ERROR = re.compile(r'\b(NullReferenceException|MissingReferenceException|IndexOutOfRangeException)\b')
WARN = re.compile(r'\bCRANE_[A-Z_]*(FAILED|INVALID)\b')


def _alive(pid_file):
    try:
        pgid = int(pid_file.read_text().strip())
        os.killpg(pgid, 0)
        return True
    except (OSError, ValueError):
        return False


def _container_running():
    try:
        out = subprocess.run(['docker', 'inspect', '-f', '{{.State.Running}}', CONTAINER],
                             capture_output=True, text=True, timeout=10)
        return out.returncode == 0 and out.stdout.strip() == 'true'
    except (OSError, subprocess.TimeoutExpired):
        return False


def _nodes():
    """ros2 node list from inside the ROS container, or None when it cannot be read."""
    try:
        out = subprocess.run(['docker', 'exec', CONTAINER, 'bash', '-c',
                              'source $BASH_ENV 2>/dev/null; ros2 node list'],
                             capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return out.stdout.splitlines() if out.returncode == 0 else None


def live(run=None):
    """[(name, ok, detail)] for the running sim. The player's log is read only while the player runs."""
    run = run or RUN
    log = run / 'player.log'
    running = _alive(run / 'player.pid')
    last = (run / 'player.path').read_text().strip() if (run / 'player.path').is_file() else 'no player started yet'
    text = log.read_text(errors='replace') if running and log.is_file() else ''
    errors = [line for line in text.splitlines() if ERROR.search(line)]
    container = _container_running()
    nodes = _nodes() if container else None
    missing = [n for n in NAV_NODES if not any(n in line for line in (nodes or []))]
    gone = 'player not running'
    return [
        ('player running', running, f'running ({last})' if running else f'not running (last: {last}); crane sim up --headless'),
        ('course ready', 'CRANE_ROBOBOAT_COURSE_READY' in text, str(run / 'course_truth.json') if running else gone),
        ('plant applied', 'CRANE_PLANT_APPLIED' in text,
         next((l.strip() for l in text.splitlines() if 'CRANE_PLANT_APPLIED' in l),
              'no CRANE_PLANT_APPLIED line' if running else gone)),
        ('ROS side running', container, f'container {CONTAINER}' + ('' if container else ' is not running')),
        ('nav stack up', container and nodes is not None and not missing,
         (f'{", ".join(NAV_NODES)}' if not missing else f'no {", ".join(missing)} in ros2 node list')
         if container and nodes is not None else f'cannot read ros2 node list in {CONTAINER}'),
        ('no player errors', running and not errors, (errors[0][:160] if errors else str(log)) if running else gone),
    ]


def _report(wait):
    """Print the rows once and return the failed names; with wait, poll every 2 s until ready or WAIT s pass."""
    deadline = time.monotonic() + wait
    while True:
        rows = live()
        bad = [name for name, ok, _ in rows if not ok]
        if not bad or time.monotonic() >= deadline:
            return rows, bad
        time.sleep(2)


def main(rest):
    if rest and rest[0] in ('-h', '--help'):
        print(__doc__.strip())
        return 0
    wait = 0.0
    if rest[:1] == ['--wait'] or (rest and rest[0].startswith('--wait=')):
        value = rest[0].partition('=')[2] if '=' in rest[0] else (rest[1] if len(rest) > 1 else '')
        try:
            wait = float(value)
        except ValueError:
            print(f'crane sim check: --wait wants SECONDS, got {value!r}', file=sys.stderr)
            return 2
        rest = rest[1 if '=' in rest[0] else 2:]
    if rest:
        from crane import paths
        from crane.wrap import execute
        return execute([sys.executable, '-m', 'Tools.SimCheck', *rest], python_path=True) if paths.ROOT else 2
    rows, bad = _report(wait)
    for name, ok, detail in rows:
        print(f"  {'ok  ' if ok else 'FAIL'}  {name:18s} {detail}")
    log = RUN / 'player.log'
    warns = sorted({WARN.search(l).group(0) for l in (log.read_text(errors='replace').splitlines()
                                                      if _alive(RUN / 'player.pid') and log.is_file() else [])
                    if WARN.search(l)})
    if warns:
        print(f"  warn  player notices     {', '.join(warns)} (see {log})")
    print('sim check: ready' if not bad else f"sim check: not ready, failed: {', '.join(bad)}; see `crane sim status`")
    return 0 if not bad else 1
