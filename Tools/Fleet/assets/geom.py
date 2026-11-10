"""Small geometry toolkit: transforms, triangle meshes, convex-polytope clipping, lathe solids,
OBJ/MTL read+write, watertightness and column-integrated overlap volumes. Pure numpy.

Frames: right-handed, x forward, y left, z up, metres (see ASSET_MANIFEST_SPEC.md).
All structural numbers come from the caller; only numerical epsilons live here.
"""
import math
import os
import numpy as np

EPS = 1e-9


# ----------------------------------------------------------------- transforms
def rotx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)


def roty(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)


def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


def rpy_to_R(rpy):
    """URDF convention: R = Rz(yaw) Ry(pitch) Rx(roll) (extrinsic x, then y, then z)."""
    return rotz(rpy[2]) @ roty(rpy[1]) @ rotx(rpy[0])


def R_to_rpy(R):
    sp = max(-1.0, min(1.0, -R[2, 0]))
    p = math.asin(sp)
    if abs(sp) < 1 - 1e-9:
        return [math.atan2(R[2, 1], R[2, 2]), p, math.atan2(R[1, 0], R[0, 0])]
    return [0.0, p, math.atan2(-R[0, 1], R[1, 1])]


def make_T(R=None, t=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = t
    return T


def T_from_pose(d):
    return make_T(rpy_to_R(d.get("rpy", [0, 0, 0])), d.get("xyz", [0, 0, 0]))


def pose_of(T):
    clean = lambda v: [0.0 if abs(x) < 1e-12 else round(float(x), 9) for x in v]
    return {"xyz": clean(T[:3, 3]), "rpy": clean(R_to_rpy(T[:3, :3]))}


def inv_T(T):
    R = T[:3, :3]
    return make_T(R.T, -R.T @ T[:3, 3])


def R_align_z(axis):
    """Rotation matrix taking +z to the given unit axis."""
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    z = np.array([0, 0, 1.0])
    v = np.cross(z, a)
    s, c = np.linalg.norm(v), float(z @ a)
    if s < 1e-12:
        return np.eye(3) if c > 0 else rotx(math.pi)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K * ((1 - c) / s ** 2)


# ----------------------------------------------------------------------- mesh
class Mesh:
    """Triangle soup with per-corner attributes.
    P (T,3,3) positions, UV (T,3,2), N (T,3,3) normals or None, M material index, G group index,
    S shell id (pieces that must not be smoothed together / overlap-integrated separately)."""

    def __init__(self):
        self.P = np.zeros((0, 3, 3))
        self.UV = np.zeros((0, 3, 2))
        self.N = None
        self.M = np.zeros(0, int)
        self.G = np.zeros(0, int)
        self.S = np.zeros(0, int)
        self.mats = []
        self.groups = []

    def __len__(self):
        return len(self.P)

    @staticmethod
    def _idx(lst, name):
        if name not in lst:
            lst.append(name)
        return lst.index(name)

    @classmethod
    def from_tris(cls, P, mat, grp="main", UV=None):
        m = cls()
        P = np.asarray(P, float).reshape(-1, 3, 3)
        m.P = P
        m.UV = np.zeros((len(P), 3, 2)) if UV is None else np.asarray(UV, float).reshape(-1, 3, 2)
        m.mats = [mat]
        m.groups = [grp]
        m.M = np.zeros(len(P), int)
        m.G = np.zeros(len(P), int)
        m.S = np.zeros(len(P), int)
        return m

    def append(self, o):
        if len(o) == 0:
            return self
        mm = np.array([self._idx(self.mats, n) for n in o.mats])
        gg = np.array([self._idx(self.groups, n) for n in o.groups])
        off = (self.S.max() + 1) if len(self.S) else 0
        self.P = np.concatenate([self.P, o.P])
        self.UV = np.concatenate([self.UV, o.UV])
        self.M = np.concatenate([self.M, mm[o.M]])
        self.G = np.concatenate([self.G, gg[o.G]])
        self.S = np.concatenate([self.S, o.S + off])
        if self.N is not None or o.N is not None:
            self.N = None  # recomputed at write time
        return self

    def copy(self):
        m = Mesh()
        m.P, m.UV, m.M, m.G, m.S = self.P.copy(), self.UV.copy(), self.M.copy(), self.G.copy(), self.S.copy()
        m.N = None if self.N is None else self.N.copy()
        m.mats, m.groups = list(self.mats), list(self.groups)
        return m

    def transformed(self, T):
        m = self.copy()
        R, t = T[:3, :3], T[:3, 3]
        m.P = m.P @ R.T + t
        if m.N is not None:
            m.N = m.N @ R.T
        if np.linalg.det(R) < 0:  # mirror: keep outward winding
            m.P = m.P[:, [0, 2, 1]]
            m.UV = m.UV[:, [0, 2, 1]]
            if m.N is not None:
                m.N = -m.N[:, [0, 2, 1]]
        return m

    def bbox(self):
        if len(self) == 0:
            return np.zeros(3), np.zeros(3)
        v = self.P.reshape(-1, 3)
        return v.min(0), v.max(0)

    def signed_volume(self):
        a, b, c = self.P[:, 0], self.P[:, 1], self.P[:, 2]
        return float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0)

    def face_normals(self):
        n = np.cross(self.P[:, 1] - self.P[:, 0], self.P[:, 2] - self.P[:, 0])
        a = np.linalg.norm(n, axis=1)
        return n, a

    def smooth_normals(self, crease_deg=40.0):
        """Per-corner normals: average area-weighted face normals of coplanar-ish neighbours
        (angle < crease) that share a vertex position inside the same shell."""
        T = len(self)
        n, a = self.face_normals()
        unit = n / np.maximum(a, 1e-30)[:, None]
        unit[a < 1e-14] = [0, 0, 1]
        keys = np.round(self.P.reshape(-1, 3) * 1e6).astype(np.int64)
        keys = np.concatenate([keys, np.repeat(self.S, 3)[:, None]], axis=1)
        _, inv = np.unique(keys, axis=0, return_inverse=True)
        inv = inv.reshape(-1)
        order = np.argsort(inv, kind="stable")
        bounds = np.flatnonzero(np.diff(inv[order])) + 1
        cosc = math.cos(math.radians(crease_deg))
        N = np.zeros((T * 3, 3))
        for grp in np.split(order, bounds):
            f = grp // 3
            u = unit[f]
            w = a[f]
            sim = (u @ u.T) > cosc
            acc = (sim * w[None, :]) @ u
            nr = np.linalg.norm(acc, axis=1, keepdims=True)
            acc = np.where(nr > 1e-14, acc / np.maximum(nr, 1e-30), u)
            N[grp] = acc
        self.N = N.reshape(T, 3, 3)
        return self

    def components(self):
        """Connected shells by welded vertices (union-find). Returns per-triangle component id."""
        keys = np.round(self.P.reshape(-1, 3) * 1e6).astype(np.int64)
        _, inv = np.unique(keys, axis=0, return_inverse=True)
        inv = inv.reshape(-1, 3)
        parent = np.arange(inv.max() + 1 if len(inv) else 0)

        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        for a, b, c in inv:
            ra, rb, rc = find(a), find(b), find(c)
            parent[rb] = ra
            parent[find(c)] = find(a)
        roots = np.array([find(i[0]) for i in inv]) if len(inv) else np.zeros(0, int)
        _, comp = np.unique(roots, return_inverse=True)
        return comp.reshape(-1)

    def closedness(self):
        """Return dict(unbalanced, nonmanifold, degenerate). 'Closed' means every undirected edge
        is traversed equally often in both directions (so coincident touching solids still pass)."""
        keys = np.round(self.P.reshape(-1, 3) * 1e6).astype(np.int64)
        _, inv = np.unique(keys, axis=0, return_inverse=True)
        inv = inv.reshape(-1, 3)
        _, area = self.face_normals()
        degenerate = int((area < 1e-12).sum())
        good = area >= 1e-12
        tri = inv[good]
        e = np.concatenate([tri[:, [0, 1]], tri[:, [1, 2]], tri[:, [2, 0]]])
        e = e[e[:, 0] != e[:, 1]]
        lo, hi = e.min(1), e.max(1)
        sgn = np.where(e[:, 0] < e[:, 1], 1, -1)
        k = lo.astype(np.int64) * (int(inv.max()) + 1 if len(inv) else 1) + hi
        uk, ki = np.unique(k, return_inverse=True)
        bal = np.zeros(len(uk), int)
        cnt = np.zeros(len(uk), int)
        np.add.at(bal, ki.reshape(-1), sgn)
        np.add.at(cnt, ki.reshape(-1), 1)
        return {"unbalanced": int((bal != 0).sum()), "nonmanifold": int((cnt > 2).sum()), "degenerate": degenerate}


# ------------------------------------------------------- convex polytope clip
def _newell(pts):
    n = np.zeros(3)
    for i in range(len(pts)):
        a, b = pts[i], pts[(i + 1) % len(pts)]
        n += np.cross(a, b)
    return n


class Poly:
    """Convex polytope as a list of (polygon[n,3], tag). Clipped by half-spaces n.p <= d."""

    def __init__(self, faces):
        self.faces = faces

    @classmethod
    def box(cls, lo, hi):
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        c = (lo + hi) / 2
        x0, y0, z0 = lo
        x1, y1, z1 = hi
        quads = {
            "+x": [(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)],
            "-x": [(x0, y0, z0), (x0, y0, z1), (x0, y1, z1), (x0, y1, z0)],
            "+y": [(x0, y1, z0), (x0, y1, z1), (x1, y1, z1), (x1, y1, z0)],
            "-y": [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)],
            "+z": [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)],
            "-z": [(x0, y0, z0), (x0, y1, z0), (x1, y1, z0), (x1, y0, z0)],
        }
        out = []
        for tag, q in quads.items():
            q = np.array(q, float)
            out.append([q, tag])
        p = cls(out)
        p._orient()
        return p

    def _orient(self):
        allp = np.concatenate([f[0] for f in self.faces])
        c = allp.mean(0)
        for f in self.faces:
            pts = f[0]
            n = _newell(pts)
            if n @ (pts.mean(0) - c) < 0:
                f[0] = pts[::-1].copy()

    def clip(self, n, d, tag):
        n = np.asarray(n, float)
        n = n / np.linalg.norm(n)
        new, cut = [], []
        coplanar = False
        for pts, t in self.faces:
            s = pts @ n - d
            if np.all(np.abs(s) <= 1e-9):
                coplanar = True
            out = []
            m = len(pts)
            for i in range(m):
                a, b = pts[i], pts[(i + 1) % m]
                sa, sb = s[i], s[(i + 1) % m]
                if sa <= 1e-9:
                    out.append(a)
                    if sa >= -1e-9:
                        cut.append(a)
                if (sa < -1e-9 and sb > 1e-9) or (sa > 1e-9 and sb < -1e-9):
                    p = a + (b - a) * (sa / (sa - sb))
                    out.append(p)
                    cut.append(p)
            if len(out) >= 3:
                out = np.array(out)
                if np.linalg.norm(_newell(out)) > 1e-12:
                    new.append([out, t])
        if cut and not coplanar:
            cp = np.unique(np.round(np.array(cut), 9), axis=0)
            if len(cp) >= 3:
                c = cp.mean(0)
                u = np.cross(n, [1, 0, 0] if abs(n[0]) < 0.9 else [0, 1, 0])
                u /= np.linalg.norm(u)
                v = np.cross(n, u)
                ang = np.arctan2((cp - c) @ v, (cp - c) @ u)
                cp = cp[np.argsort(ang)]
                if np.linalg.norm(_newell(cp)) > 1e-12:
                    new.append([cp, tag])
        self.faces = new
        return self

    def chamfer(self, edge_pt, a, b, c, tag="chamfer"):
        """Cut the edge through edge_pt whose two outward face directions are a and b by width c."""
        a, b = np.asarray(a, float), np.asarray(b, float)
        a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
        n = (a + b) / np.linalg.norm(a + b)
        d = float(n @ np.asarray(edge_pt, float)) - c * float(n @ a)
        return self.clip(n, d, tag)

    def to_mesh(self, tag_to_mat, default_mat, grp="main", uv_scale=1.0):
        tris, uvs, mats = [], [], []
        for pts, tag in self.faces:
            mat = tag_to_mat.get(tag, default_mat)
            n = _newell(pts)
            nn = np.linalg.norm(n)
            if nn < 1e-14:
                continue
            n /= nn
            ax = np.argmax(np.abs(n))
            uvc = [i for i in range(3) if i != ax]
            uv = pts[:, uvc] / uv_scale
            tl = _triangulate(pts)
            for i, j, k in tl:
                if np.linalg.norm(np.cross(pts[j] - pts[i], pts[k] - pts[i])) < 1e-9:
                    continue
                tris.append([pts[i], pts[j], pts[k]])
                uvs.append([uv[i], uv[j], uv[k]])
                mats.append(mat)
        m = Mesh()
        if not tris:
            return m
        names = []
        for mt in mats:
            if mt not in names:
                names.append(mt)
        m.P = np.array(tris, float)
        m.UV = np.array(uvs, float)
        m.mats = names
        m.groups = [grp]
        m.M = np.array([names.index(x) for x in mats])
        m.G = np.zeros(len(tris), int)
        m.S = np.zeros(len(tris), int)
        return m


def _triangulate(pts):
    n = len(pts)
    if n == 3:
        return [(0, 1, 2)]
    fan = [(0, i, i + 1) for i in range(1, n - 1)]
    ok = all(np.linalg.norm(np.cross(pts[j] - pts[i], pts[k] - pts[i])) > 1e-12 for i, j, k in fan)
    if ok:
        return fan
    # centroid fan: add centroid as extra point is not possible with indices; fall back to ear-free
    # triangulation around the vertex with maximum interior angle quality
    best = None
    for s in range(n):
        idx = [(s + i) % n for i in range(n)]
        tl = [(idx[0], idx[i], idx[i + 1]) for i in range(1, n - 1)]
        if all(np.linalg.norm(np.cross(pts[j] - pts[i], pts[k] - pts[i])) > 1e-12 for i, j, k in tl):
            best = tl
            break
    return best or fan


def box_mesh(lo, hi, mat, grp="main", chamfers=(), tag_to_mat=None, uv_scale=1.0):
    """Box with optional chamfers [(edge_pt, a, b, c[, tag]), ...]."""
    p = Poly.box(lo, hi)
    for ch in chamfers:
        p.chamfer(*ch)
    return p.to_mesh(tag_to_mat or {}, mat, grp, uv_scale)


def lathe(profile, nseg, mat, grp="main", cap_start=True, cap_end=True):
    """Solid of revolution about +z. profile = [(z, r), ...] ascending in z."""
    prof = [(float(z), float(r)) for z, r in profile]
    ang = np.linspace(0, 2 * math.pi, nseg, endpoint=False)
    ring = lambda z, r: np.stack([r * np.cos(ang), r * np.sin(ang), np.full(nseg, z)], 1)
    tris, uvs = [], []
    for (z0, r0), (z1, r1) in zip(prof[:-1], prof[1:]):
        A, B = ring(z0, r0), ring(z1, r1)
        for k in range(nseg):
            k2 = (k + 1) % nseg
            u0, u1 = k / nseg, (k + 1) / nseg
            if r0 > 1e-12:
                tris.append([A[k], A[k2], B[k2]] if r1 > 1e-12 else [A[k], A[k2], B[k]])
                uvs.append([[u0, z0], [u1, z0], [u1, z1]])
            if r1 > 1e-12 and r0 > 1e-12:
                tris.append([A[k], B[k2], B[k]])
                uvs.append([[u0, z0], [u1, z1], [u0, z1]])
            elif r0 <= 1e-12 and r1 > 1e-12:
                tris.append([A[k], B[k2], B[k]])
                uvs.append([[u0, z0], [u1, z1], [u0, z1]])
    for (z, r), flag, up in ((prof[0], cap_start, False), (prof[-1], cap_end, True)):
        if flag and r > 1e-12:
            R = ring(z, r)
            c = np.array([0, 0, z])
            for k in range(nseg):
                k2 = (k + 1) % nseg
                tris.append([c, R[k], R[k2]] if up else [c, R[k2], R[k]])
                uvs.append([[0.5, 0.5], [0.5 + 0.5 * math.cos(ang[k]), 0.5 + 0.5 * math.sin(ang[k])],
                            [0.5 + 0.5 * math.cos(ang[k2] if k2 else 2 * math.pi), 0.5 + 0.5 * math.sin(ang[k2] if k2 else 2 * math.pi)]])
    m = Mesh.from_tris(np.array(tris), mat, grp, np.array(uvs))
    if m.signed_volume() < 0:
        m.P = m.P[:, [0, 2, 1]]
        m.UV = m.UV[:, [0, 2, 1]]
    return m


def cylinder(radius, height, nseg, mat, grp="main", center=True):
    z0 = -height / 2 if center else 0.0
    return lathe([(z0, radius), (z0 + height, radius)], nseg, mat, grp)


def quad_mesh(corners, uvs, mat, grp="main"):
    c = np.asarray(corners, float)
    u = np.asarray(uvs, float)
    return Mesh.from_tris(np.array([[c[0], c[1], c[2]], [c[0], c[2], c[3]]]), mat, grp,
                          np.array([[u[0], u[1], u[2]], [u[0], u[2], u[3]]]))


# ------------------------------------------------------------------------ OBJ
def write_obj(path, parts_meshes, mtl_name=None, header="", crease_deg=40.0):
    """parts_meshes: list of (object_name, Mesh). Writes v/vt/vn and f v/vt/vn with usemtl and g."""
    lines = ["# " + header] if header else []
    vmap, tmap, nmap = {}, {}, {}
    vl, tl, nl, fl = [], [], [], []

    def gid(table, lst, key, fmt):
        if key not in table:
            table[key] = len(lst) + 1
            lst.append(fmt)
        return table[key]

    for name, m in parts_meshes:
        m = m.copy()
        m.smooth_normals(crease_deg)
        fl.append(f"o {name}")
        cur_g = cur_m = None
        order = np.lexsort((m.M, m.G))
        for t in order:
            g, mt = m.G[t], m.M[t]
            if g != cur_g:
                fl.append(f"g {m.groups[g]}")
                cur_g, cur_m = g, None
            if mt != cur_m:
                fl.append(f"usemtl {m.mats[mt]}")
                cur_m = mt
            ids = []
            for c in range(3):
                p, uv, n = m.P[t, c], m.UV[t, c], m.N[t, c]
                kp = tuple(np.round(p, 6) + 0.0)
                ku = tuple(np.round(uv, 6) + 0.0)
                kn = tuple(np.round(n, 5) + 0.0)
                ids.append("%d/%d/%d" % (
                    gid(vmap, vl, kp, "v %.6f %.6f %.6f" % kp),
                    gid(tmap, tl, ku, "vt %.6f %.6f" % ku),
                    gid(nmap, nl, kn, "vn %.5f %.5f %.5f" % kn)))
            fl.append("f " + " ".join(ids))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    head = lines[:1] if header else []
    if mtl_name:
        head = head + [f"mtllib {mtl_name}"]
    with open(path, "w") as f:
        f.write("\n".join(head + vl + tl + nl + fl) + "\n")


def read_obj(path):
    V, VT, VN = [], [], []
    m = Mesh()
    tris, uvs, nrm, mats, grps = [], [], [], [], []
    mtl, cur_m, cur_g = None, "default", "main"
    objects = []
    for line in open(path):
        s = line.split()
        if not s or s[0].startswith("#"):
            continue
        if s[0] == "v":
            V.append([float(x) for x in s[1:4]])
        elif s[0] == "vt":
            VT.append([float(x) for x in s[1:3]])
        elif s[0] == "vn":
            VN.append([float(x) for x in s[1:4]])
        elif s[0] == "mtllib":
            mtl = s[1]
        elif s[0] == "usemtl":
            cur_m = s[1]
        elif s[0] == "g":
            cur_g = s[1] if len(s) > 1 else "main"
        elif s[0] == "o":
            objects.append(s[1])
        elif s[0] == "f":
            idx = []
            for tok in s[1:]:
                p = tok.split("/")
                vi = int(p[0])
                vi = vi - 1 if vi > 0 else len(V) + vi
                ti = int(p[1]) - 1 if len(p) > 1 and p[1] else None
                ni = int(p[2]) - 1 if len(p) > 2 and p[2] else None
                idx.append((vi, ti, ni))
            for i in range(1, len(idx) - 1):
                tri = [idx[0], idx[i], idx[i + 1]]
                tris.append([V[a[0]] for a in tri])
                uvs.append([VT[a[1]] if a[1] is not None else [0, 0] for a in tri])
                nrm.append([VN[a[2]] if a[2] is not None else [0, 0, 0] for a in tri])
                mats.append(cur_m)
                grps.append(cur_g)
    m.P = np.array(tris, float).reshape(-1, 3, 3)
    m.UV = np.array(uvs, float).reshape(-1, 3, 2)
    m.N = np.array(nrm, float).reshape(-1, 3, 3)
    m.mats = sorted(set(mats))
    m.groups = sorted(set(grps), key=grps.index) if grps else []
    m.M = np.array([m.mats.index(x) for x in mats], int)
    m.G = np.array([m.groups.index(x) for x in grps], int)
    m.S = np.zeros(len(tris), int)
    return m, {"mtllib": mtl, "objects": objects, "n_v": len(V), "n_vn": len(VN), "n_vt": len(VT)}


def write_mtl(path, materials):
    """materials: {name: {base_color:[r,g,b], alpha, specular, shininess, texture}}"""
    out = []
    for name, d in materials.items():
        out += [f"newmtl {name}", "Kd %.4f %.4f %.4f" % tuple(d["base_color"]),
                "Ka %.4f %.4f %.4f" % tuple(0.2 * np.array(d["base_color"])),
                "Ks %.3f %.3f %.3f" % ((d.get("specular", 0.1),) * 3), "Ns %.1f" % d.get("shininess", 20.0),
                "d %.3f" % d.get("alpha", 1.0)]
        if d.get("texture"):
            out.append("map_Kd " + d["texture"])
        out.append("")
    with open(path, "w") as f:
        f.write("\n".join(out))


def read_mtl_textures(path):
    tex = []
    for line in open(path):
        s = line.split()
        if s and s[0] == "map_Kd":
            tex.append(s[1])
    return tex


# ------------------------------------------------- column-integrated overlaps
def _column_intervals(P, xs, ys):
    """For a closed shell (triangles P) return list of [(z_in,z_out),..] per query column (xs,ys)."""
    a, b, c = P[:, 0, :2], P[:, 1, :2], P[:, 2, :2]
    out = [[] for _ in range(len(xs))]
    if len(P) == 0:
        return out
    n = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0])
    lo, hi = P[:, :, :2].min(1), P[:, :, :2].max(1)
    for q in range(len(xs)):
        x, y = xs[q], ys[q]
        cand = np.flatnonzero((lo[:, 0] <= x) & (hi[:, 0] >= x) & (lo[:, 1] <= y) & (hi[:, 1] >= y) & (np.abs(n[:, 2]) > 1e-14))
        if len(cand) == 0:
            continue
        A, B, C = a[cand], b[cand], c[cand]
        d = (B[:, 1] - C[:, 1]) * (A[:, 0] - C[:, 0]) + (C[:, 0] - B[:, 0]) * (A[:, 1] - C[:, 1])
        l1 = ((B[:, 1] - C[:, 1]) * (x - C[:, 0]) + (C[:, 0] - B[:, 0]) * (y - C[:, 1])) / d
        l2 = ((C[:, 1] - A[:, 1]) * (x - C[:, 0]) + (A[:, 0] - C[:, 0]) * (y - C[:, 1])) / d
        l3 = 1 - l1 - l2
        hit = (l1 >= 0) & (l2 >= 0) & (l3 >= 0)
        if not hit.any():
            continue
        pz = P[cand][hit]
        zs = np.sort(l1[hit] * pz[:, 0, 2] + l2[hit] * pz[:, 1, 2] + l3[hit] * pz[:, 2, 2])
        # drop duplicate hits on shared edges
        keep = [zs[0]]
        for z in zs[1:]:
            if abs(z - keep[-1]) > 1e-9:
                keep.append(z)
        if len(keep) % 2 == 0:
            out[q] = [(keep[i], keep[i + 1]) for i in range(0, len(keep), 2)]
    return out


def _merge(iv):
    iv = sorted(iv)
    res = []
    for lo, hi in iv:
        if res and lo <= res[-1][1]:
            res[-1][1] = max(res[-1][1], hi)
        else:
            res.append([lo, hi])
    return res


def solid_columns(mesh, xs, ys, region=None):
    comp = mesh.components()
    cols = [[] for _ in range(len(xs))]
    for c in np.unique(comp):
        P = mesh.P[comp == c]
        lo, hi = P.reshape(-1, 3).min(0), P.reshape(-1, 3).max(0)
        if region is not None and (np.any(hi < region[0]) or np.any(lo > region[1])):
            continue
        sel = np.flatnonzero((xs >= lo[0]) & (xs <= hi[0]) & (ys >= lo[1]) & (ys <= hi[1]))
        if len(sel) == 0:
            continue
        iv = _column_intervals(P, xs[sel], ys[sel])
        for q, lst in zip(sel, iv):
            cols[q].extend(lst)
    return [_merge(c) for c in cols]


def overlap_volume(meshA, meshB, cell=0.003):
    """Volume (m^3) of the intersection of two mesh unions, integrated on a cell x cell grid."""
    if len(meshA) == 0 or len(meshB) == 0:
        return 0.0
    la, ha = meshA.bbox()
    lb, hb = meshB.bbox()
    lo, hi = np.maximum(la, lb), np.minimum(ha, hb)
    if np.any(hi <= lo):
        return 0.0
    gx = np.arange(lo[0] + 0.37 * cell, hi[0], cell)
    gy = np.arange(lo[1] + 0.41 * cell, hi[1], cell)
    if len(gx) == 0 or len(gy) == 0:
        return 0.0
    X, Y = np.meshgrid(gx, gy)
    xs, ys = X.ravel() + 1.7e-6, Y.ravel() + 2.3e-6
    ca, cb = solid_columns(meshA, xs, ys, (lo, hi)), solid_columns(meshB, xs, ys, (lo, hi))
    vol = 0.0
    for ia, ib in zip(ca, cb):
        for l1, h1 in ia:
            for l2, h2 in ib:
                o = min(h1, h2) - max(l1, l2)
                if o > 0:
                    vol += o
    return vol * cell * cell
