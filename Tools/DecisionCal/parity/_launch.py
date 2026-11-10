"""Shared player launch for the parity drivers: ``heavy unity`` may queue for a coordinator grant before the
player starts, so the 60 s budget is measured from the moment the player process exists (its -logFile appears),
not from the heavy call. A watchdog SIGTERMs the process group ``wall`` s after the player started, SIGKILLs 5 s
later, and gives up waiting for a grant after ``grant_timeout`` s (no Unity ran: not a launch)."""

import os
import signal
import subprocess
import threading
import time
from pathlib import Path

HEAVY = Path.home() / 'astro_dock' / 'src' / 'mhseals_agents' / 'tools' / 'heavy'
RACE_EXIT = 75          # driver exit code: the launch raced another Unity and was stopped; requeue, does not count


def unity_slots_held():
    """Number of unity slots `heavy status` shows as held (by anyone); None if the status cannot be read."""
    try:
        out = subprocess.run(['python3', str(HEAVY), 'status'], capture_output=True, text=True, timeout=30).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    rows = [l for l in out.splitlines() if l.startswith('unity.')]
    if not rows:
        return None
    return sum(1 for l in rows if not l.rstrip().endswith('free'))


class Player:
    def __init__(self, argv, env, out_dir, wall=60.0, grant_timeout=900.0):
        self.out_dir, self.wall, self.grant_timeout = out_dir, wall, grant_timeout
        self.t_call = time.time()
        self.t_start = None
        self.proc = subprocess.Popen(argv, env=env, stdout=open(out_dir / 'heavy.out', 'w'), stderr=subprocess.STDOUT,
                                     start_new_session=True)
        self._stop = False
        self.race_abort = False
        self.race_slots = None
        self.th = threading.Thread(target=self._watch, daemon=True)
        self.th.start()

    def _watch(self):
        log = self.out_dir / 'player.log'
        while not self._stop and self.proc.poll() is None:
            now = time.time()
            if self.t_start is None and log.exists():
                self.t_start = now
                # host rule: one Unity process in total. Right after the grant, re-read the slots; if two are held
                # (ours and someone else's, started in the same instant), stop ours at once.
                self.race_slots = unity_slots_held()
                if self.race_slots is not None and self.race_slots >= 2:
                    self.race_abort = True
                    self._kill(signal.SIGTERM)
                    time.sleep(3)
                    self._kill(signal.SIGKILL)
                    return
            if self.t_start is None and now - self.t_call > self.grant_timeout:
                self._kill(signal.SIGTERM)
                return
            if self.t_start is not None and now - self.t_start > self.wall:
                self._kill(signal.SIGTERM)
                time.sleep(5)
                self._kill(signal.SIGKILL)
                return
            time.sleep(0.2)

    def _kill(self, sig):
        try:
            os.killpg(self.proc.pid, sig)
        except ProcessLookupError:
            pass

    def alive(self):
        return self.proc.poll() is None

    def deadline(self, margin=6.0):
        """Wall time by which the session must end (player start + wall - margin)."""
        t0 = self.t_start if self.t_start is not None else time.time() - 10.0
        return t0 + self.wall - margin

    def finish(self, meta):
        try:
            self.proc.wait(max(1.0, (self.t_start or time.time()) + self.wall + 8 - time.time()))
        except subprocess.TimeoutExpired:
            self._kill(signal.SIGKILL)
        self._stop = True
        meta['player_rc'] = self.proc.poll()
        meta['heavy_wait_s'] = (self.t_start - self.t_call) if self.t_start else None
        meta['player_wall_s'] = (time.time() - self.t_start) if self.t_start else None
        meta['player_started'] = self.t_start is not None
        meta['race_abort'] = self.race_abort
        meta['race_slots_held'] = self.race_slots
