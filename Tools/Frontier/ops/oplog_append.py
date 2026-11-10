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
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import recfeas  # noqa: E402
H = os.path.expanduser("~/.cache/frontier/crane/harness")
try:
    tw = {}
    for fn in ("navpilot_fidelity.jsonl", "navpilot_twin_seed2.jsonl"):
        if os.path.exists(os.path.join(H, fn)):
            for l in open(os.path.join(H, fn)):
                r = json.loads(l); tw[(r["speed"], r["mission"], r["run_seed"])] = r
    net = {}
    for r in recfeas.load(glob.glob(O + "/recfeas_legacy_s*")):
        net[(r["speed"], r["mission"], r["run_seed"])] = r
    out += ["", "### D4 readout (a): level gap, twin-legacy minus .NET-legacy collision rate, matched cross units (mission-cluster bootstrap 95% CI)", "",
            "| speed | matched units | twin coll | .NET coll | gap (pp) | 95% CI (pp) | twin seed1-vs-seed2 collision disagreements (cross_0-3) |", "|---|---|---|---|---|---|---|"]
    for v in (5.0, 6.0, 8.0):
        keys = [k for k in net if k[0] == v and k[1].startswith("cross_") and k in tw]
        if not keys:
            continue
        pairs = {}
        for k in keys:
            pairs.setdefault(k[1], []).append((int(bool(tw[k]["collision"])), int(bool(net[k]["r_Collision"]))))
        a = sum(x for p in pairs.values() for x, y in p); b = sum(y for p in pairs.values() for x, y in p); n = len(keys)
        lo, hi = recfeas.boot(pairs)
        dis = [(m, int(bool(tw[(v, m, 1)]["collision"])) != int(bool(tw[(v, m, 2)]["collision"]))) for m in ["cross_%d" % i for i in range(4)] if (v, m, 1) in tw and (v, m, 2) in tw]
        out.append("| %g | %d | %d | %d | %+.1f | [%+.1f, %+.1f] | %d of %d |" % (v, n, a, b, 100 * (a - b) / n, 100 * lo, 100 * hi, sum(d for _, d in dis), len(dis)))
    out += ["", "Readout (b) = the paired_after_stage tables above (.NET recfeas minus legacy)."]
except Exception as e:  # keep the log append robust
    out.append("readout (a) failed: %r" % e)
open(os.path.join(REPO, "Docs/DroneVisuals/frontier/OPS_LOG.md"), "a").write("\n".join(out) + "\n")
