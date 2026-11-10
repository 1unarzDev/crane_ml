#!/usr/bin/env python3
"""Build the uncoupled capplan.module_graph/v0 used by the CRANE open-loop probe (Tools/Fleet/probes/probe_graph_v0.json) with capplan's own
ModuleGraph serialiser, so the file is exactly what the generator emits. The capplan tree is read only (no bytecode is written).

    python3 Tools/Fleet/capplan_types/make_probe_graph.py [--capplan /home/lunarz/crane_ml/Tools/Fleet]

The modules are the WMAC promoted-scene types (diff_carrier, lifter, sensor_mast) plus a mecanum_pod, each healthy and with one wheel
fault. They sit in one column (assembly frame y spacing 1.1 m) and the job's origin places that column in the east bay.
"""
import argparse
import json
import sys
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--capplan", default="/home/lunarz/crane_ml/Tools/Fleet")
ap.add_argument("--out", default=str(Path(__file__).resolve().parents[1] / "probes" / "probe_graph_v0.json"))
a = ap.parse_args()
sys.dont_write_bytecode = True
sys.path.insert(0, a.capplan)
from capplan.modgraph import Module, ModuleGraph  # noqa: E402

mods = [
    Module("dc_ok", "diff_carrier", 0.0, 0.0, 0.0),
    Module("dc_free0", "diff_carrier", 0.0, 1.1, 0.0, act_health=["dead_free", "ok", "ok", "ok"]),
    Module("lift_ok", "lifter", 0.0, 2.2, 0.0),
    Module("lift_lockfl", "lifter", 0.0, 3.3, 0.0, act_health=["dead_locked", "ok", "ok", "ok"]),
    Module("mec_freefl", "mecanum_pod", 0.0, 4.4, 0.0, act_health=["dead_free", "ok", "ok", "ok"]),
    Module("mast", "sensor_mast", 0.0, 5.5, 0.0),
]
g = ModuleGraph(mods, [], {"note": "CRANE open-loop probe, uncoupled", "generator": "Tools/Fleet/capplan_types/make_probe_graph.py"})
Path(a.out).write_text(json.dumps(g.to_dict(), indent=1, sort_keys=True) + "\n")
print("wrote", a.out)
