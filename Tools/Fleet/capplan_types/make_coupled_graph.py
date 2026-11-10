#!/usr/bin/env python3
"""Build the pre-latched capplan.module_graph/v0 for the M2 probe (Tools/Fleet/probes/coupled_graph_v0.json) with capplan's own serialiser.
The capplan tree is read only (no bytecode). Three uncoupled-from-each-other assemblies, each pre-latched (LATCHED_VERIFIED):
  inline: carrier c2.front mated to carrier c1.rear (cargo pair in line)
  L:      mecanum_pod pb.rear mated to mecanum_pod pa.left
  mixed:  diff_carrier dm.front mated to mecanum_pod mm.rear (loses lateral motion: one hard row)
Assembly frames: column 1 at y = 0, the L at y = 1.6, the mixed pair at y = 3.2 (job origin places them in the east bay).
"""
import argparse, json, math, sys
from pathlib import Path
ap = argparse.ArgumentParser()
ap.add_argument("--capplan", default="/home/lunarz/crane_ml/Tools/Fleet")
ap.add_argument("--out", default=str(Path(__file__).resolve().parents[1] / "probes" / "coupled_graph_v0.json"))
a = ap.parse_args()
sys.dont_write_bytecode = True
sys.path.insert(0, a.capplan)
from capplan.modgraph import Module, Coupling, ModuleGraph  # noqa: E402
from capplan.library import TYPE_REGISTRY  # noqa: E402


def mated(pa, fa, fb):
    """pose of module b whose face fb is coincident with module a's face fa, facing it (hardware-agnostic)."""
    x, y, yaw = pa
    c, s = math.cos(yaw), math.sin(yaw)
    fx, fy, fyaw = x + c * fa["x"] - s * fa["y"], y + s * fa["x"] + c * fa["y"], yaw + fa["yaw"] + math.pi
    byaw = fyaw - fb["yaw"]
    cb, sb = math.cos(byaw), math.sin(byaw)
    return fx - (cb * fb["x"] - sb * fb["y"]), fy - (sb * fb["x"] + cb * fb["y"]), math.atan2(math.sin(byaw), math.cos(byaw))


F = lambda t, f: TYPE_REGISTRY[t]["faces"][f]  # noqa: E731
mods, cps = [], []
p1 = (0.0, 0.0, 0.0); p2 = mated(p1, F("carrier", "rear"), F("carrier", "front"))
mods += [Module("c1", "carrier", *p1), Module("c2", "carrier", *p2)]; cps.append(Coupling("c1", "rear", "c2", "front"))
pa = (0.0, 1.6, 0.0); pb = mated(pa, F("mecanum_pod", "left"), F("mecanum_pod", "rear"))
mods += [Module("pa", "mecanum_pod", *pa), Module("pb", "mecanum_pod", *pb)]; cps.append(Coupling("pa", "left", "pb", "rear"))
pd = (0.0, 3.6, 0.0); pm = mated(pd, F("diff_carrier", "front"), F("mecanum_pod", "rear"))
mods += [Module("dm", "diff_carrier", *pd), Module("mm", "mecanum_pod", *pm)]; cps.append(Coupling("dm", "front", "mm", "rear"))
g = ModuleGraph(mods, cps, {"note": "CRANE M2 probe: pre-latched assemblies", "generator": "Tools/Fleet/capplan_types/make_coupled_graph.py"})
Path(a.out).write_text(json.dumps(g.to_dict(), indent=1, sort_keys=True) + "\n")
print("wrote", a.out, [(m.id, round(m.x, 3), round(m.y, 3), round(m.yaw, 3)) for m in mods])
