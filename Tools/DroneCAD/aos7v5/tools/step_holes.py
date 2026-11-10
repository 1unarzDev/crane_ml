"""Read-only: list through-hole-like cylinders (r in [rmin,rmax]) and the biggest solids of a STEP.  python -I step_holes.py file rmin rmax"""
import sys
from build123d import import_step, GeomType
s = import_step(sys.argv[1]); rmin, rmax = float(sys.argv[2]), float(sys.argv[3])
rows = {}
for f in s.faces():
    if f.geom_type != GeomType.CYLINDER: continue
    try: r = f.radius
    except Exception: continue
    if rmin <= r <= rmax:
        c = f.center(); a = f.axis_at(c).direction if hasattr(f, "axis_at") else None
        d = (round(abs(a.X), 1), round(abs(a.Y), 1), round(abs(a.Z), 1)) if a is not None else None
        rows[(round(r, 2), round(c.X, 1), round(c.Y, 1), round(c.Z, 1), d)] = 1
for k in sorted(rows): print(k)
big = sorted(s.solids(), key=lambda q: -q.volume)[:6]
for q in big:
    b = q.bounding_box(); print("solid vol", round(q.volume), "min", [round(v,1) for v in (b.min.X,b.min.Y,b.min.Z)], "size", [round(v,1) for v in (b.size.X,b.size.Y,b.size.Z)])
