"""Train / held-out environment families by RULE (the capplan H7 control), frozen with a hash, and the leakage check.

    python3 -m scenes.splits freeze [--train N] [--heldout M]   # writes frozen/splits_v1.json (refuses to overwrite)
    python3 -m scenes.splits check                             # leakage + reproducibility checks; exit 1 on a problem

The split of a scene is decided by ``families.RULES`` over its parameters, never by the seed. The frozen manifest lists,
per family and split, every seed's parameters and canonical scene hash (``common.geometry_signature``: solids, zones,
patches, unit catalogue keys and starts, payloads, faults, cameras; ids, notes and seeds excluded), the rule texts,
the generator version and the hash of the generator code, the catalogue hash, and a manifest hash over all of it.

Leakage checks (each reported separately):
  L1 rule disjointness  every train scene satisfies its family's train rule and every held-out scene does not
  L2 exact leakage      no held-out scene hash equals a train scene hash (across all families)
  L3 parameter leakage  no held-out parameter vector equals a train one
  L4 reproducibility    regenerating each frozen seed gives the frozen hash (the generator did not drift)
  L5 code freeze        the generator code hash equals the frozen one (else: re-freeze as v2, never edit v1)
  L6 H7 control         every unit of every scene resolves to capplan train_v1 or the train type pool (no held-out
                        configuration enters an environment split), and the catalogue hash is the frozen one
"""
import argparse
import json
import sys
from datetime import date

from .common import HERE, geometry_signature, sha
from .families import GENERATOR_VERSION, GENERATORS, RULES, generate, in_split
from . import catalog

FROZEN = HERE / "frozen" / "splits_v1.json"
CODE = ["families.py", "common.py", "catalog.py"]


def code_hash():
    return sha(b"".join((HERE / f).read_bytes() for f in CODE))


def scene_hash(scn):
    return sha(geometry_signature(scn))


def build(n_train=20, n_heldout=10):
    fams = {}
    for f in GENERATORS:
        fams[f] = {}
        for split, n in (("train", n_train), ("heldout", n_heldout)):
            rows = []
            for seed in range(1, n + 1):
                scn, task = generate(f, seed, split)
                rows.append({"seed": seed, "params": scn["family"]["params"], "scene_hash": scene_hash(scn),
                             "plans": sorted(task["plans"])})
            fams[f][split] = rows
    body = {"schema": "crane.fleet_scene_splits/v1", "generator": GENERATOR_VERSION, "generator_code_sha": code_hash(),
            "code_files": CODE, "catalog_hash": catalog.load()["catalog_hash"], "rules": RULES,
            "control": "H7: environment/task families held out by rule; every fleet uses capplan train_v1 / train-pool "
                       "configurations only (CONFIG_SPACE.md section 4)",
            "frozen": str(date.today()), "families": fams}
    body["manifest_hash"] = sha({k: v for k, v in body.items() if k != "frozen"})
    return body


def freeze(n_train=20, n_heldout=10, force=False):
    if FROZEN.exists() and not force:
        raise SystemExit("%s exists: a frozen split is never rewritten (bump to v2)" % FROZEN)
    body = build(n_train, n_heldout)
    FROZEN.parent.mkdir(parents=True, exist_ok=True)
    FROZEN.write_text(json.dumps(body, indent=1) + "\n")
    return body


def check(man=None, regenerate=True):
    man = man or json.loads(FROZEN.read_text())
    res = {k: [] for k in ("L1_rule_disjointness", "L2_exact_leakage", "L3_parameter_leakage", "L4_reproducibility",
                           "L5_code_freeze", "L6_h7_control")}
    body = {k: v for k, v in man.items() if k not in ("frozen", "manifest_hash")}
    if sha(body) != man["manifest_hash"]:
        res["L5_code_freeze"].append("manifest hash mismatch (the frozen file was edited)")
    if man["generator_code_sha"] != code_hash():
        res["L5_code_freeze"].append("generator code changed since the freeze (%s -> %s)" % (man["generator_code_sha"], code_hash()))
    if man["catalog_hash"] != catalog.load()["catalog_hash"]:
        res["L6_h7_control"].append("catalogue changed since the freeze")
    train_h = {r["scene_hash"]: (f, r["seed"]) for f, d in man["families"].items() for r in d["train"]}
    train_p = {(f, json.dumps(r["params"], sort_keys=True)) for f, d in man["families"].items() for r in d["train"]}
    for f, d in man["families"].items():
        for split in ("train", "heldout"):
            for r in d[split]:
                if in_split(f, r["params"]) != split:
                    res["L1_rule_disjointness"].append("%s %s s%d falls in %s by rule" % (f, split, r["seed"], in_split(f, r["params"])))
                if split == "heldout":
                    if r["scene_hash"] in train_h:
                        res["L2_exact_leakage"].append("%s heldout s%d equals train %s" % (f, r["seed"], train_h[r["scene_hash"]]))
                    if (f, json.dumps(r["params"], sort_keys=True)) in train_p:
                        res["L3_parameter_leakage"].append("%s heldout s%d has train parameters" % (f, r["seed"]))
                if regenerate:
                    scn, _ = generate(f, r["seed"], split)
                    if scene_hash(scn) != r["scene_hash"]:
                        res["L4_reproducibility"].append("%s %s s%d regenerates differently" % (f, split, r["seed"]))
                    for u in scn["fleet"]["units"]:
                        ref = catalog.unit(u["catalog"])["ref"]
                        ok = ref.get("manifest") == "train_v1" or "compose" in ref
                        if not ok:
                            res["L6_h7_control"].append("%s %s s%d unit %s is not a train configuration" % (f, split, r["seed"], u["id"]))
    # compositions must use train-pool types only (checked against capplan when it is importable)
    try:
        configs, _ = catalog._capplan()
        lib = configs.library()
        pool = set(configs.train_pool(lib))
        for k, u in catalog.load()["units"].items():
            if "compose" in u["ref"]:
                bad = [t for t in u["ref"]["compose"]["types"] if t not in pool]
                if bad:
                    res["L6_h7_control"].append("catalogue %s composes non-train types %s" % (k, bad))
    except ImportError:
        res["L6_h7_control"].append("(capplan not importable: train-pool membership not verified)")
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(prog="scenes.splits")
    ap.add_argument("cmd", choices=["freeze", "check", "summary"])
    ap.add_argument("--train", type=int, default=20)
    ap.add_argument("--heldout", type=int, default=10)
    a = ap.parse_args(argv)
    if a.cmd == "freeze":
        b = freeze(a.train, a.heldout)
        print("frozen %s: manifest %s, code %s" % (FROZEN.name, b["manifest_hash"], b["generator_code_sha"]))
        return 0
    if a.cmd == "summary":
        m = json.loads(FROZEN.read_text())
        for f, d in m["families"].items():
            print("%-20s train %2d  heldout %2d  | %s || %s" % (f, len(d["train"]), len(d["heldout"]), m["rules"][f]["train"], m["rules"][f]["heldout"]))
        return 0
    res = check()
    bad = 0
    for k, v in res.items():
        real = [x for x in v if not x.startswith("(")]
        bad += bool(real)
        print("%-22s %s" % (k, "pass" if not real else "FAIL: " + "; ".join(real[:5])), *(x for x in v if x.startswith("(")))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
