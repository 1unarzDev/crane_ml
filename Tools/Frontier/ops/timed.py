# SPDX-License-Identifier: GPL-3.0-or-later
"""timed.py OUT.json -- CMD...: run CMD, write {wall_s, cpu_s, max_rss_mb, rc, start, end} (children rusage)."""
import json, resource, subprocess, sys, time
i = sys.argv.index("--"); out = sys.argv[1]
t0 = time.time(); p = subprocess.run(sys.argv[i + 1:]); t1 = time.time()
r = resource.getrusage(resource.RUSAGE_CHILDREN)
json.dump({"wall_s": t1 - t0, "cpu_s": r.ru_utime + r.ru_stime, "max_rss_mb": r.ru_maxrss / 1024, "rc": p.returncode,
           "start": time.strftime("%F %T", time.localtime(t0)), "end": time.strftime("%F %T", time.localtime(t1)), "cmd": sys.argv[i + 1:]}, open(out, "w"))
sys.exit(p.returncode)
