"""Write scene bundles (crane.fleet_scene_bundle/v1) for fastsim: one per seed, from the v1 family generators.

    python3 gen_scenarios.py --family F --split train|heldout --plan P --from A --to B --dir DIR

fastsim's ScenarioSource calls this through the domain YAML's ``generate`` template (``fastsim.batch --generate``).
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # Tools/Fleet

from scenes.common import BUNDLE_SCHEMA, dump  # noqa: E402
from scenes.families import generate  # noqa: E402
from scenes.validate import validate_pair  # noqa: E402


def bundle(family, split, plan, seed):
    scn, task = generate(family, seed, split)
    if plan not in task["plans"]:
        raise SystemExit("%s has no plan %r (plans: %s)" % (family, plan, sorted(task["plans"])))
    problems = validate_pair(scn, task)
    if problems:
        raise SystemExit("%s s%d: invalid: %s" % (family, seed, "; ".join(problems)))
    return {"schema": BUNDLE_SCHEMA, "scenario": scn, "task": task, "plan": plan}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--family", required=True)
    ap.add_argument("--split", default="train")
    ap.add_argument("--plan", required=True)
    ap.add_argument("--from", dest="lo", type=int, required=True)
    ap.add_argument("--to", dest="hi", type=int, required=True, help="exclusive")
    ap.add_argument("--dir", required=True)
    a = ap.parse_args(argv)
    for seed in range(a.lo, a.hi):
        path = Path(a.dir) / ("%s-%s-%s-s%d.json" % (a.family, a.split, a.plan, seed))
        dump(bundle(a.family, a.split, a.plan, seed), path)
        print(path)


if __name__ == "__main__":
    main()
