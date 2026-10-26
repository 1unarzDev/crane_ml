"""Import an mhseals_testing characterization into a crane boat profile JSON through the gate.

    python3 -I Tools/Characterization/import_yaml.py --yaml Y --provenance P --run R \
        --config-snapshot C --out-profile OUT.json --out-report REPORT.json [--base profiles/ideal.json]

Starts from the base profile (Ideal by default), runs gate.run_gate, and overwrites only the
parameters the gate marks identified or provisional:
  * D per signed axis: d1Pos/d2Pos from <axis>+, d1Neg/d2Neg from <axis>-, as D = d * k_j^+-
  * added mass a11/a22/a66 = M_eff - m (or - I_z) where M_eff = mu * k_j^+ from t95
  * the envelope (motion_limits.velocity) of an axis whose D was taken
Everything else (mass, inertia, geometry, thrusters) stays at the base value. Provenance records
per-parameter status and reason, input sha256 values, the summary, kind "characterized" and the
gate report path. --provenance, --run and --config-snapshot may be omitted; the gate then abstains
on what they would have provided (PROVENANCE_MISSING, no holds), so nothing is overwritten.

An mhseals.physics/1 survey record (characterization-v2) is the whole input:

    python3 -I Tools/Characterization/import_yaml.py --yaml <root>/characterization/<fit>-survey.yaml \
        [--history <root>/config-history] --out-profile OUT.json --out-report REPORT.json

mhseals_physics.adapt turns it, its samples file and the config-history snapshot into the gate inputs.
"""
from __future__ import annotations

import argparse
import copy
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # python3 -I drops the script dir
import gate as G  # noqa: E402
import mhseals_physics as MP  # noqa: E402
import plant as P  # noqa: E402

TAKEN = (G.IDENTIFIED, G.PROVISIONAL)
ADDED_MASS_KEY = {"surge": "a11", "sway": "a22", "yaw": "a66"}


def _rank(p):
    """0 for prior (worst) .. 2 for identified."""
    return (G.PRIOR, G.PROVISIONAL, G.IDENTIFIED).index(p["status"])


def build_profile(base, report, inputs, report_path):
    prof = copy.deepcopy(base)
    params = report["parameters"]
    entries = [e for e in base.get("provenance", {}).get("parameters", [])
               if e["name"] in ("mass", "izz", "thrusters")]
    for name in G.DIRECTIONS:
        p = params[name]
        axis, suffix = name[:-1], ("Pos" if name.endswith("+") else "Neg")
        if p["status"] in TAKEN and "D1" in p:
            prof[axis]["d1" + suffix] = p["D1"]
            prof[axis]["d2" + suffix] = p["D2"]
        entries.append({"name": name, "status": p["status"], "reason": p["reason"]})
    for j, axis in enumerate(G.AXES):
        p = params[f"meff.{axis}"]
        key = ADDED_MASS_KEY[axis]
        rigid = prof["mass"] if j < 2 else prof["izz"]
        if p["status"] in TAKEN and "meff" in p and p["meff"] >= rigid:
            prof["addedMass"][key] = p["meff"] - rigid
        entries.append({"name": f"addedMass.{key}", "status": p["status"], "reason": p["reason"]})
        statuses = {params[axis + "+"]["status"], params[axis + "-"]["status"]}
        env = (report.get("envelope") or {}).get(axis)
        if env and statuses <= set(TAKEN):
            prof["envelope"][axis] = float(env)
    for axis in G.AXES:  # aggregate rows under the base names: the worse of the two signs
        pair = [params[axis + "+"], params[axis + "-"]]
        worst = min(pair, key=_rank)
        entries.append({"name": axis, "status": worst["status"],
                        "reason": f"+: {pair[0]['reason']}; -: {pair[1]['reason']}"})
    worst = min((params[f"meff.{a}"] for a in G.AXES), key=_rank)
    entries.append({"name": "addedMass", "status": worst["status"], "reason": worst["reason"]})
    reasons = ", ".join(report["reasons"])
    prof["name"] = "Characterized"
    prof["provenance"] = {
        "kind": "characterized",
        "source": "mhseals_testing characterization through Tools/Characterization/gate.py, base "
                  + base.get("name", "?"),
        "summary": report["summary"] + (f": {reasons}" if reasons else ""),
        "inputs": inputs,
        "gateReport": report_path or "",
        "parameters": entries,
    }
    P.validate_profile(prof)
    return prof


def main(argv=None):
    ap = argparse.ArgumentParser(description="Import a characterization YAML through the gate.")
    ap.add_argument("--yaml", required=True)
    ap.add_argument("--provenance")
    ap.add_argument("--run")
    ap.add_argument("--config-snapshot")
    ap.add_argument("--history", help="config-history dir for an mhseals.physics/1 record "
                                      "[<record>/../../config-history]")
    ap.add_argument("--out-profile", required=True)
    ap.add_argument("--out-report", required=True)
    ap.add_argument("--base", default=P.IDEAL_PATH)
    ap.add_argument("--tau-est", type=float, default=None, help="estimator lag prior [s] (default: measured or 0)")
    a = ap.parse_args(argv)
    base = P.load_profile(a.base)
    paths = [("yaml", a.yaml), ("provenance", a.provenance), ("run", a.run), ("config-snapshot", a.config_snapshot),
             ("base", a.base)]
    inputs = [{"path": p, "sha256": G.sha256_file(p)} for _, p in paths if p]
    gate_in = (a.yaml, a.provenance, a.run, a.config_snapshot)
    if MP.is_record(G.load_doc(a.yaml)):  # characterization-v2 survey: samples file, config-history
        gate_in = MP.adapt(a.yaml, a.history)
        sf = G.load_doc(a.yaml).get("samples_file")
        if sf:
            sp = os.path.join(os.path.dirname(a.yaml), sf["path"])
            inputs.append({"path": sp, "sha256": G.sha256_file(sp)})
    report = G.run_gate(*gate_in, yaml_path=a.yaml, base_profile=base, tau_est=a.tau_est)
    report["inputs"] = inputs
    prof = build_profile(base, report, inputs, a.out_report)
    for path, doc in ((a.out_report, report), (a.out_profile, prof)):
        d = os.path.dirname(os.path.abspath(path))
        os.makedirs(d, exist_ok=True)
        with open(path, "w") as f:
            json.dump(doc, f, indent=4, allow_nan=False)
            f.write("\n")
    print(f"{report['summary']}: profile {a.out_profile}, report {a.out_report}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
