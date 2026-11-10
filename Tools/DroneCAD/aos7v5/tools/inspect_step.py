"""Read-only inspection of a vendor STEP (never executes it): bbox, volume, solids count, circular holes (cylindrical faces).
usage: python -I inspect_step.py <file.step> [--mesh out.stl --tol 0.2]"""
import sys, json
import numpy as np
from build123d import import_step, GeomType
p = sys.argv[1]
s = import_step(p)
bb = s.bounding_box()
print("file", p)
print("bbox mm min", [round(v, 2) for v in (bb.min.X, bb.min.Y, bb.min.Z)], "max", [round(v, 2) for v in (bb.max.X, bb.max.Y, bb.max.Z)], "size", [round(v, 2) for v in (bb.size.X, bb.size.Y, bb.size.Z)])
sol = s.solids()
print("solids", len(sol), "volume mm3", round(s.volume, 1))
holes = {}
for f in s.faces():
    if f.geom_type == GeomType.CYLINDER:
        try:
            r = f.radius; c = f.center()
            ax = f.axis_at(c) if hasattr(f, "axis_at") else None
        except Exception:
            continue
        if r < 6:
            holes[(round(r, 2), round(c.X, 1), round(c.Y, 1), round(c.Z, 1))] = holes.get((round(r, 2), round(c.X, 1), round(c.Y, 1), round(c.Z, 1)), 0) + 1
print("small cylinders (r, x, y, z):", len(holes))
for k in sorted(holes)[:60]:
    print(" ", k)
