# SPDX-License-Identifier: GPL-3.0-or-later
"""Sequential job-file worker (holds one fsrun slot; runs each job file in a fresh child so code edits apply).

  fsrun python Tools/NavPilot/experiments/worker.py --queue ~/.cache/navpilot_e3 --name w1 --idle-min 20
Job files: <queue>/pending/*.json, claimed by atomic rename into running/, finished into done/.
"""
import argparse, glob, os, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))

ap = argparse.ArgumentParser()
ap.add_argument("--queue", required=True)
ap.add_argument("--name", default="w")
ap.add_argument("--idle-min", type=float, default=20.0)
a = ap.parse_args()
q = os.path.expanduser(a.queue)
last = time.time()
while time.time() - last < a.idle_min * 60:
    files = sorted(glob.glob(os.path.join(q, "pending", "*.json")))
    if not files:
        time.sleep(5)
        continue
    f = files[0]
    run = os.path.join(q, "running", "%s__%s" % (a.name, os.path.basename(f)))
    try:
        os.rename(f, run)
    except OSError:
        continue
    log = open(os.path.join(q, "logs", os.path.basename(f) + ".log"), "w")
    rc = subprocess.call([sys.executable, os.path.join(HERE, "e3.py"), "run-file", run], stdout=log, stderr=subprocess.STDOUT)
    log.close()
    os.rename(run, os.path.join(q, "done", os.path.basename(f) + ("" if rc == 0 else ".failed")))
    last = time.time()
print("worker %s idle, exiting" % a.name)
