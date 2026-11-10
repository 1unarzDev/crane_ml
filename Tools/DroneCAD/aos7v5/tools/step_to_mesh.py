"""Read-only conversion: STEP -> decimated STL (never executes the file).  python -I step_to_mesh.py in.step out.stl [tol=0.15] [faces=12000]"""
import sys
import numpy as np, trimesh
from build123d import import_step
s = import_step(sys.argv[1]); tol = float(sys.argv[3]) if len(sys.argv) > 3 else 0.15; nf = int(sys.argv[4]) if len(sys.argv) > 4 else 12000
meshes = []
for sol in s.solids():
    v, t = sol.tessellate(tol, 0.3)
    if len(t): meshes.append(trimesh.Trimesh(np.array([(p.X, p.Y, p.Z) for p in v]), np.array(t), process=True))
m = trimesh.util.concatenate(meshes)
print("raw faces", len(m.faces), "bounds", m.bounds.round(2).tolist())
if len(m.faces) > nf:
    m = m.simplify_quadric_decimation(face_count=nf)
print("out faces", len(m.faces)); m.export(sys.argv[2])
