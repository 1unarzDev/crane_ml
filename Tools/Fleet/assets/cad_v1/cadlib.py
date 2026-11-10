"""Shared helpers for the v1 parametric CAD (build123d / OpenCASCADE).

Units: metres, kilograms. Frames: right-handed, x forward, y left, z up (ASSET_MANIFEST_SPEC.md).
An `Item` is one solid with a render material, an OBJ group name and a mass rule (density or fixed mass).
A `Link` is a rigid body (manifest render part / URDF link) made of items expressed in the link frame.
"""
import math
import os
import sys

import numpy as np
from build123d import (Axis, Box, Cylinder, Location, Plane, Polyline, Pos, Rot, Solid, Sphere,
                       ThreePointArc, Circle, Wire, make_face, revolve, sweep, extrude, loft, Sketch, Face)
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.dirname(HERE)
sys.path.insert(0, ASSETS)
import geom as g  # noqa: E402

# Densities kg/m^3. PETG: Prusament PETG TDS 1.27 g/cm3; walls printed with 4 perimeters -> 0.9 solid fraction.
DENSITY = {
    "petg_wall": 1270 * 0.90,      # 3 mm printed walls, brackets
    "petg_infill": 1270 * 0.35,    # thick printed parts (arm links, blocks) at ~25 % infill + perimeters
    "aluminium": 2700,             # 6061/5052
    "acm": 1270,                   # 3 mm aluminium composite panel (3.8 kg/m^2), provisional
    "steel": 7850,
    "ndfeb": 7500,
    "rubber": 1200,
}


def pv(x):
    return x["value"] if isinstance(x, dict) and "value" in x else x


# ------------------------------------------------------------------ transforms
def T(x=0.0, y=0.0, z=0.0, R=None):
    return g.make_T(R, [x, y, z])


def loc_of(Tm):
    """numpy 4x4 -> build123d Location."""
    from OCP.gp import gp_Trsf
    tr = gp_Trsf()
    R, t = Tm[:3, :3], Tm[:3, 3]
    tr.SetValues(R[0, 0], R[0, 1], R[0, 2], t[0], R[1, 0], R[1, 1], R[1, 2], t[1], R[2, 0], R[2, 1], R[2, 2], t[2])
    return Location(tr)


def place(solid, Tm):
    return solid.moved(loc_of(Tm))


# ------------------------------------------------------------------ primitives
def box(lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c, s = (lo + hi) / 2, hi - lo
    return Pos(*c) * Box(*s)


def cyl(r, h, axis="z", center=(0, 0, 0)):
    c = Cylinder(r, h)
    if axis == "x":
        c = Rot(0, 90, 0) * c
    elif axis == "y":
        c = Rot(90, 0, 0) * c
    return Pos(*center) * c


def cyl_between(p0, p1, r):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    L = float(np.linalg.norm(d))
    R = g.R_align_z(d / L)
    return place(Cylinder(r, L), T(*((p0 + p1) / 2), R=R))


def prism_xy(pts, z0, z1):
    """Extrude a closed planar polygon (xy) from z0 to z1."""
    w = Polyline(*[(float(x), float(y), z0) for x, y in pts], close=True)
    f = make_face(w)
    return extrude(f, z1 - z0, dir=(0, 0, 1))


def polygon_prism_x(pts_yz, x0, x1):
    """Extrude a closed polygon given in (y, z) from x0 to x1 along +x."""
    w = Polyline(*[(x0, float(a), float(b)) for a, b in pts_yz], close=True)
    return extrude(make_face(w), x1 - x0, dir=(1, 0, 0))


def octagon(hx, hy, c):
    return [(hx, hy - c), (hx - c, hy), (-hx + c, hy), (-hx, hy - c), (-hx, -hy + c), (-hx + c, -hy), (hx - c, -hy), (hx, -hy + c)]


def loft_octagons(o0, z0, o1, z1):
    w0 = Polyline(*[(x, y, z0) for x, y in o0], close=True)
    w1 = Polyline(*[(x, y, z1) for x, y in o1], close=True)
    return loft([make_face(w0), make_face(w1)], ruled=True)


def rounded_rect_prism_x(yc, zc, wy, hz, r, x0, x1):
    """Prism along x with a rounded-rectangle section (pocket cutter)."""
    from build123d import RectangleRounded
    face = Plane.YZ.offset(x0) * Pos(yc, zc) * RectangleRounded(wy, hz, r)
    return extrude(face, x1 - x0, dir=(1, 0, 0))


def chamfer_box_yz(lo, hi, c):
    """Box whose 4 edges parallel to x are chamfered by c (section in yz)."""
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    pts = [(y0 + c, z0), (y1 - c, z0), (y1, z0 + c), (y1, z1 - c), (y1 - c, z1), (y0 + c, z1), (y0, z1 - c), (y0, z0 + c)]
    return polygon_prism_x(pts, x0, x1)


def revolve_profile_z(profile):
    """Solid of revolution about +z from [(z, r), ...] (r>0)."""
    pts = [(0.0, 0.0, profile[0][0])] + [(r, 0.0, z) for z, r in profile] + [(0.0, 0.0, profile[-1][0])]
    w = Polyline(*pts, close=True)
    f = make_face(w)
    return revolve(f, Axis.Z, 360)


def torus_arc(center, radius, a0_deg, a1_deg, r_tube, plane="xz"):
    """Tube of radius r_tube swept along a circular arc in the x-z plane (angles from +x towards +z)."""
    cx, cy, cz = center
    pt = lambda a: (cx + radius * math.cos(math.radians(a)), cy, cz + radius * math.sin(math.radians(a)))
    path = Wire([ThreePointArc(pt(a0_deg), pt((a0_deg + a1_deg) / 2), pt(a1_deg))])
    a = math.radians(a0_deg)
    tangent = (math.sin(a), 0.0, -math.cos(a)) if a1_deg < a0_deg else (-math.sin(a), 0.0, math.cos(a))
    sec = Plane(origin=pt(a0_deg), z_dir=tangent) * Circle(r_tube)
    return sweep(sec, path)


# ------------------------------------------------------------------ items / links
class Item:
    def __init__(self, solid, mat, group="main", density=None, mass=None, label=None, render=True):
        self.solid, self.mat, self.group = solid, mat, group
        self.density, self.mass, self.label, self.render = density, mass, label or group, render

    def props(self):
        """(mass, com, inertia about com) in the item's (link) frame."""
        p = GProp_GProps()
        BRepGProp.VolumeProperties_s(self.solid.wrapped, p)
        vol = p.Mass()
        c = p.CentreOfMass()
        M = p.MatrixOfInertia()
        I = np.array([[M.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)])
        if self.mass is not None:
            rho = self.mass / vol
        else:
            rho = DENSITY[self.density] if isinstance(self.density, str) else float(self.density)
        return vol * rho, np.array([c.X(), c.Y(), c.Z()]), I * rho, vol


def combine(components):
    M = sum(m for m, _, _ in components)
    com = sum(m * np.asarray(c, float) for m, c, _ in components) / M
    I = np.zeros((3, 3))
    for m, c, Ic in components:
        d = np.asarray(c, float) - com
        I += Ic + m * (d @ d * np.eye(3) - np.outer(d, d))
    return M, com, I


def transform_inertia(m, com, I, Tm):
    R = Tm[:3, :3]
    return m, R @ com + Tm[:3, 3], R @ I @ R.T


class Link:
    """Rigid body. joint: dict(type, axis, origin (4x4 at q=0 in parent link frame), limits, authored)."""

    def __init__(self, name, parent="base_link", joint=None, role="body", visible=True, kind="solid", hide_group=None,
                 contacts_ground=False, render_part=True):
        self.name, self.parent, self.joint, self.role = name, parent, joint, role
        self.visible, self.kind, self.hide_group, self.contacts_ground = visible, kind, hide_group, contacts_ground
        self.items, self.quads, self.render_part = [], [], render_part
        self.frames = {}          # extra named frames (4x4 in the link frame), e.g. engage points, sensors

    def add(self, *items):
        self.items.extend(items)
        return self

    def mass_props(self, include_hidden=True):
        comps, rows = [], []
        for it in self.items:
            m, c, I, vol = it.props()
            comps.append((m, c, I))
            rows.append({"item": it.label, "material": it.mat, "rule": (f"mass {it.mass} kg" if it.mass is not None else f"density {it.density}"),
                         "volume_m3": vol, "mass": m, "com_link": c.tolist()})
        if not comps:
            return 0.0, np.zeros(3), np.zeros((3, 3)), rows
        M, com, I = combine(comps)
        return M, com, I, rows

    def origin_authored(self):
        """Pose of this link frame in the parent frame at q = authored (manifest 'origin')."""
        j = self.joint
        if not j:
            return np.eye(4)
        return j["origin"] @ joint_motion(j, j.get("authored", 0.0))


def joint_motion(j, q):
    if j["type"] in ("revolute", "continuous"):
        ax = np.asarray(j["axis"], float)
        ax = ax / np.linalg.norm(ax)
        K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        return g.make_T(np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K)
    if j["type"] == "prismatic":
        return g.make_T(None, np.asarray(j["axis"], float) * q)
    return np.eye(4)


# ------------------------------------------------------------------ tessellation
def solid_to_mesh(solid, mat, group, tol=4e-4, ang=0.35, uv_scale=0.1):
    """Absolute chordal deflection tol [m] and angular deflection ang [rad] (build123d's own mesh() is relative)."""
    from OCP.BRepTools import BRepTools
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    BRepTools.Clean_s(solid.wrapped)
    BRepMesh_IncrementalMesh(solid.wrapped, tol, False, ang, True)
    verts, tris = solid.tessellate(1e6, ang)
    V = np.array([[v.X, v.Y, v.Z] for v in verts], float)
    F = np.array(tris, int)
    if len(F) == 0:
        return g.Mesh()
    P = V[F]
    area = np.linalg.norm(np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0]), axis=1)
    P = P[area > 1e-12]                     # drop zero-area pole triangles (edges stay balanced)
    m = g.Mesh.from_tris(P, mat, group)
    if m.signed_volume() < 0:
        m.P = m.P[:, [0, 2, 1]]
    n = np.cross(m.P[:, 1] - m.P[:, 0], m.P[:, 2] - m.P[:, 0])
    ax = np.argmax(np.abs(n), axis=1)
    uv = np.zeros((len(m.P), 3, 2))
    for a in range(3):
        sel = ax == a
        keep = [i for i in range(3) if i != a]
        uv[sel] = m.P[sel][:, :, keep] / uv_scale
    m.UV = uv
    return m


def link_mesh(link, tol=4e-4, ang=0.35):
    mesh = g.Mesh()
    for it in link.items:
        if not it.render:
            continue
        mesh.append(solid_to_mesh(it.solid, it.mat, it.group, tol, ang))
    for q in link.quads:
        mesh.append(q)
    return mesh


def export_step_link(link, path):
    from build123d import Compound, export_step
    sol = [it.solid for it in link.items if it.render]
    if not sol:
        return None
    comp = Compound(children=[s for s in sol])
    comp.label = link.name
    export_step(comp, path)
    return path


def write_stl(path, mesh):
    """Binary STL (metres) of a geom.Mesh (for URDF visuals)."""
    import struct
    P = mesh.P.astype(np.float32)
    n = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0])
    nn = np.linalg.norm(n, axis=1, keepdims=True)
    n = (n / np.maximum(nn, 1e-30)).astype(np.float32)
    with open(path, "wb") as f:
        f.write(b"crane fleet v1 concept CAD, metres".ljust(80, b" "))
        f.write(struct.pack("<I", len(P)))
        rec = np.zeros(len(P), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
        rec["n"], rec["v"] = n, P
        f.write(rec.tobytes())
