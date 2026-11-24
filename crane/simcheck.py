"""`crane sim check`: is the running sim (sim_up / astro sim up) ready to be used like the robot?

With no arguments it checks the live sim; `crane sim check CHECK ...` runs one of Tools/SimCheck's offline
acceptance checks (axis, sitl, sitl-peer, docking, smoke) on recorded results. Stdlib only.
"""

import os
import re
import subprocess
import sys
from pathlib import Path

RUN = Path(os.environ.get('SIM_UP_RUN', Path.home() / '.cache' / 'sim_up'))
CONTAINER = 'sim_up_ros'
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


def live(run=RUN):
    """[(name, ok, detail)] for the running sim."""
    log = run / 'player.log'
    text = log.read_text(errors='replace') if log.is_file() else ''
    errors = [line for line in text.splitlines() if ERROR.search(line)]
    return [
        ('player running', _alive(run / 'player.pid'), (run / 'player.path').read_text().strip()
         if (run / 'player.path').is_file() else 'no player.path (sim_up up not run)'),
        ('course ready', 'CRANE_ROBOBOAT_COURSE_READY' in text, str(run / 'course_truth.json')),
        ('plant applied', 'CRANE_PLANT_APPLIED' in text,
         next((l.strip() for l in text.splitlines() if 'CRANE_PLANT_APPLIED' in l), 'no CRANE_PLANT_APPLIED line')),
        ('ROS side running', _container_running(), f'container {CONTAINER}'),
        ('no player errors', not errors, errors[0][:160] if errors else str(log)),
    ]


def main(rest):
    if rest and rest[0] not in ('-h', '--help'):
        from crane import paths
        from crane.wrap import execute
        return execute([sys.executable, '-m', 'Tools.SimCheck', *rest], python_path=True) if paths.ROOT else 2
    if rest:
        print(__doc__.strip())
        return 0
    rows = live()
    for name, ok, detail in rows:
        print(f"  {'ok  ' if ok else 'FAIL'}  {name:18s} {detail}")
    log = RUN / 'player.log'
    warns = sorted({WARN.search(l).group(0) for l in (log.read_text(errors='replace').splitlines() if log.is_file() else [])
                    if WARN.search(l)})
    if warns:
        print(f"  warn  player notices     {', '.join(warns)} (see {log})")
    bad = [name for name, ok, _ in rows if not ok]
    print('sim check: ready' if not bad else f"sim check: not ready ({', '.join(bad)}); see `crane sim status`")
    return 0 if not bad else 1
