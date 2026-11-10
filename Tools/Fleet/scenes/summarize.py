"""Summarise fastsim fleet runs (any folder of <family>-<split>-<plan>/s<seed>/fleet.json) as a markdown table.

    python3 -m scenes.summarize DIR [--json OUT.json]
"""
import argparse
import json
import statistics
from collections import Counter, defaultdict
from pathlib import Path


def collect(root):
    rows = defaultdict(list)
    for f in sorted(Path(root).glob("*/s*/fleet.json")):
        fam, split, plan = f.parent.parent.name.split("-", 2)
        d = json.loads(f.read_text())
        rows[(fam, split, plan)].append({"seed": int(f.parent.name[1:]), "outcome": d["outcome"], "t": d["t_end"],
                                         "reason": d["reason"]})
    return rows


def short(reason):
    r = reason or ""
    for k in ("no_path", "goal_blocked", "face_blocked", "collision", "deadline", "stuck", "not_found",
              "could not reach", "unsupported_payload", "overload"):
        if k in r:
            return k
    return r[:30]


def table(rows):
    out = ["| family | split | plan | n | success | median t (s, successes) | failure causes |", "|---|---|---|---|---|---|---|"]
    for (fam, split, plan), rs in sorted(rows.items()):
        ok = [r for r in rs if r["outcome"] == "SUCCEEDED"]
        causes = Counter(short(r["reason"]) for r in rs if r["outcome"] != "SUCCEEDED")
        med = "%.1f" % statistics.median(r["t"] for r in ok) if ok else "-"
        out.append("| %s | %s | %s | %d | %d/%d | %s | %s |" % (fam, split, plan, len(rs), len(ok), len(rs), med,
                   ", ".join("%s x%d" % kv for kv in causes.most_common()) or "-"))
    return "\n".join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(prog="scenes.summarize")
    ap.add_argument("root")
    ap.add_argument("--json")
    a = ap.parse_args(argv)
    rows = collect(a.root)
    print(table(rows))
    if a.json:
        Path(a.json).write_text(json.dumps({"/".join(k): v for k, v in rows.items()}, indent=1) + "\n")


if __name__ == "__main__":
    main()
