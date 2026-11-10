#!/usr/bin/env python3
"""Export the capplan module TYPE_REGISTRY (library.py) as pure data for CRANE to spawn module graphs.

The capplan source tree is read only. Run with PYTHONDONTWRITEBYTECODE=1 so that no __pycache__ is written there:

    PYTHONDONTWRITEBYTECODE=1 python3 Tools/Fleet/capplan_types/export_types.py [--capplan /home/lunarz/crane_ml/Tools/Fleet] [--out Tools/Fleet/spec/module_types_v0.json]

The output is the registry's types with their geometry, wheels (kind, pose, hand, radius, phi_max, tau_max), sensors, decks, tools and
faces, plus each type's capplan type_hash and the sha256 of library.py, so that a spawned module can be checked against the generator.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--capplan", default="/home/lunarz/crane_ml/Tools/Fleet")
ap.add_argument("--out", default=str(Path(__file__).resolve().parents[1] / "spec" / "module_types_v0.json"))
a = ap.parse_args()
sys.dont_write_bytecode = True
sys.path.insert(0, a.capplan)
from capplan import library  # noqa: E402

lib_path = Path(library.__file__)
out = {"schema": "crane.fleet.module_types/v0", "source": "capplan/library.py TYPE_REGISTRY",
       "source_sha256": hashlib.sha256(lib_path.read_bytes()).hexdigest(), "types": {}}
for name, t in sorted(library.TYPE_REGISTRY.items()):
    d = json.loads(json.dumps(t))          # plain data (tuples -> lists)
    d["type_hash"] = library.type_hash(t)
    out["types"][name] = d
Path(a.out).write_text(json.dumps(out, indent=1, sort_keys=True) + "\n")
print(f"wrote {a.out}: {len(out['types'])} types; library sha256 {out['source_sha256'][:12]}")
