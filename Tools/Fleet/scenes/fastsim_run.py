"""Run scene bundles through fastsim in-process (one process, no pool): the same path fastsim.batch takes per seed.

    python3 -m scenes.fastsim_run --family F [--split S] [--plan P|all] --seeds 1:4 --out DIR

Uses the family's YAML in fastsim_domain/configs (made by make_configs.py), generates missing bundles, and prints one
line per run. For the official batch command see README.md (``python3 -m fastsim.batch --config ... --generate``).
"""
import argparse
import json
import sys
from pathlib import Path

from .common import FLEET
from .families import generate

ROOT = FLEET.parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from fastsim import config as fsconfig  # noqa: E402
from fastsim.batch import adapter_of, parse_seeds  # noqa: E402
from fastsim.run import simulate  # noqa: E402
from fastsim.scenario import ScenarioSource  # noqa: E402

CONFIGS = Path(__file__).parent / "fastsim_domain" / "configs"


def run(family, split, plan, seeds, out):
    cfg, base = fsconfig.load(CONFIGS / ("%s-%s-%s.yaml" % (family, split, plan)))
    src = ScenarioSource(**cfg["scenarios"], base=base)
    for s in seeds:   # always regenerate (deterministic): a stale bundle would be refused by the format check
        if src.path(s).exists():
            src.path(s).unlink()
    src.generate(seeds)
    ad = adapter_of(cfg, base)
    rows = []
    for s in seeds:
        d = Path(out) / ("%s-%s-%s" % (family, split, plan)) / ("s%d" % s)
        summ = simulate(src.load(s), ad, cfg, s, d, base, {"runner": "scenes.fastsim_run"})
        f = json.loads((d / "fleet.json").read_text())
        rows.append({"family": family, "split": split, "plan": plan, "seed": s, "outcome": f["outcome"],
                     "reason": f["reason"], "t_end": f["t_end"], "dir": str(d), "core_contacts": len(summ["contacts"])})
        print("%-20s %-7s %-12s s%-2d %-9s %6.1f s  %s" % (family, split, plan, s, f["outcome"], f["t_end"], f["reason"]))
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--family", required=True)
    ap.add_argument("--split", default="train")
    ap.add_argument("--plan", default="all")
    ap.add_argument("--seeds", type=parse_seeds, default=[1])
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    plans = list(generate(a.family, 1, a.split)[1]["plans"]) if a.plan == "all" else [a.plan]
    rows = []
    for p in plans:
        rows += run(a.family, a.split, p, a.seeds, a.out)
    Path(a.out).mkdir(parents=True, exist_ok=True)
    with open(Path(a.out) / "results.jsonl", "a") as fh:
        for r in rows:
            fh.write(json.dumps(r) + "\n")


if __name__ == "__main__":
    main()
