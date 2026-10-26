"""A fake rig run for the queue tests: writes $RIGQ_PROGRESS like probe.py, then behaves per MODE.

  succeed   progress with sim time and travel, exits 0 after ~1.5 s
  stall     progress for ~0.5 s, then sim time and counts freeze (process stays up)
  static    progress advances and a motion command is active, but travel stays 0
  hang      never writes progress, ignores SIGTERM, and leaves a setsid'd grandchild behind
  fail      exits 3 at once
Writes its ROS_DOMAIN_ID and timing to $FAKE_LOG (one JSON line) when set.
"""

import json
import os
import signal
import subprocess
import sys
import time


def write(snap):
    path = os.environ['RIGQ_PROGRESS']
    with open(path + '.tmp', 'w') as out:
        json.dump(snap, out)
    os.replace(path + '.tmp', path)


def main():
    mode = sys.argv[1]
    if os.environ.get('FAKE_LOG'):
        with open(os.environ['FAKE_LOG'], 'a') as out:
            out.write(json.dumps({'job': os.environ['RIGQ_JOB'], 'mode': mode, 'start': time.time(),
                                  'domain': os.environ.get('ROS_DOMAIN_ID')}) + '\n')
    if mode == 'fail':
        sys.exit(3)
    if mode == 'hang':
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        child = subprocess.Popen(['setsid', 'sleep', '300'])
        with open(os.environ['RIGQ_PROGRESS'] + '.child', 'w') as out:
            out.write(str(child.pid))
        while True:
            time.sleep(0.1)
    t0 = time.time()
    n = 0
    while True:
        n += 1
        el = time.time() - t0
        if mode == 'succeed':
            write({'wall': time.time(), 'sim_time': el, 'msgs': n, 'travel': el, 'cmd_active_s': el,
                   'travel_since_cmd': el})
            if el > 1.5:
                return
        elif mode == 'stall':
            if el < 0.5:
                write({'wall': time.time(), 'sim_time': el, 'msgs': n, 'travel': 0, 'cmd_active_s': 0})
        elif mode == 'static':
            write({'wall': time.time(), 'sim_time': el, 'msgs': n, 'travel': 0.01, 'cmd_active_s': el,
                   'travel_since_cmd': 0.01})
        time.sleep(0.1)


if __name__ == '__main__':
    main()
