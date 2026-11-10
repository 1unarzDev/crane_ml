# SPDX-License-Identifier: GPL-3.0-or-later
"""Append the recfeas / pending-1-5 job records (from *.time.json and the paired tables) to Docs/DroneVisuals/frontier/OPS_LOG.md."""
import glob, json, os
REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
O = os.path.expanduser("~/.cache/frontier/crane/recfeas")
out = ["", "## Auto-appended by Tools/Frontier/ops/oplog_append.py (end of the recfeas driver job)", ""]
for f in sorted(glob.glob(O + "/*.time.json")):
    d = json.load(open(f))
    n = os.path.basename(f)[:-10]
    out.append("- `%s`: start %s, end %s, wall %.0f s, cpu %.0f s, max RSS %.0f MB, rc %s; out `%s/%s/`" % (
        n, d.get("start", "?"), d.get("end", "?"), d["wall_s"], d.get("cpu_s", float("nan")), d.get("max_rss_mb", float("nan")), d.get("rc"), O, n))
for f in sorted(glob.glob(O + "/paired_after_stage*.md")):
    out += ["", "### " + os.path.basename(f), "", open(f).read()]
open(os.path.join(REPO, "Docs/DroneVisuals/frontier/OPS_LOG.md"), "a").write("\n".join(out) + "\n")
