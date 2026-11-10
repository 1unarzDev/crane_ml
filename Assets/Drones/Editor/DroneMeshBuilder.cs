using System;
using System.Collections.Generic;
using UnityEngine;

namespace Crane.Drones.EditorTools
{
    /// <summary>Tiny deterministic procedural mesh kit: prisms with chamfers, lathes, tubes, surface grids.</summary>
    public sealed class DroneMeshBuilder
    {
        public readonly List<Vector3> V = new List<Vector3>();
        public readonly List<Vector3> N = new List<Vector3>();
        public readonly List<Vector2> UV = new List<Vector2>();
        public readonly Dictionary<string, List<int>> T = new Dictionary<string, List<int>>();
        public readonly List<string> Order = new List<string>();
        string _cur = "default";
        Matrix4x4 _xf = Matrix4x4.identity;
        readonly Stack<Matrix4x4> _stack = new Stack<Matrix4x4>();
        public float UvScale = 1f;

        public void Mat(string m) { _cur = m; }
        public void Push(Vector3 pos, Quaternion rot) { Push(Matrix4x4.TRS(pos, rot, Vector3.one)); }
        public void Push(Matrix4x4 m) { _stack.Push(_xf); _xf = _xf * m; }
        public void Pop() { _xf = _stack.Pop(); }
        public void PushMirrorX() { Push(Matrix4x4.Scale(new Vector3(-1f, 1f, 1f))); }

        List<int> Tris()
        {
            List<int> l;
            if (!T.TryGetValue(_cur, out l)) { l = new List<int>(); T[_cur] = l; Order.Add(_cur); }
            return l;
        }

        public static Vector3 V3(Vector2 p, float y) { return new Vector3(p.x, y, p.y); }

        static Vector2 BoxUv(Vector3 p, Vector3 n)
        {
            var a = new Vector3(Mathf.Abs(n.x), Mathf.Abs(n.y), Mathf.Abs(n.z));
            if (a.y >= a.x && a.y >= a.z) return new Vector2(p.x, p.z);
            if (a.x >= a.z) return new Vector2(p.z, p.y);
            return new Vector2(p.x, p.y);
        }

        public int AddVert(Vector3 p, Vector3 n, Vector2 uv)
        {
            V.Add(_xf.MultiplyPoint3x4(p));
            var nn = _xf.MultiplyVector(n);
            N.Add(nn.sqrMagnitude > 1e-12f ? nn.normalized : Vector3.up);
            UV.Add(uv);
            return V.Count - 1;
        }

        public void Face(int a, int b, int c)
        {
            Vector3 cr = Vector3.Cross(V[b] - V[a], V[c] - V[a]);
            Vector3 sn = N[a] + N[b] + N[c];
            var t = Tris();
            if (Vector3.Dot(cr, sn) >= 0f) { t.Add(a); t.Add(b); t.Add(c); }
            else { t.Add(a); t.Add(c); t.Add(b); }
        }

        /// <summary>Flat-shaded quad, box-projected UV (meters), winding fixed from the normal hint.</summary>
        public void Quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Vector3 hint)
        {
            Vector3 h = hint.normalized;
            Vector3 hx = _xf.MultiplyVector(h);
            int i0 = AddVert(a, h, BoxUv(_xf.MultiplyPoint3x4(a), hx) * UvScale);
            int i1 = AddVert(b, h, BoxUv(_xf.MultiplyPoint3x4(b), hx) * UvScale);
            int i2 = AddVert(c, h, BoxUv(_xf.MultiplyPoint3x4(c), hx) * UvScale);
            int i3 = AddVert(d, h, BoxUv(_xf.MultiplyPoint3x4(d), hx) * UvScale);
            Face(i0, i1, i2); Face(i0, i2, i3);
        }

        // ---------------- polygons ----------------
        static float Cross2(Vector2 u, Vector2 v) { return u.x * v.y - u.y * v.x; }
        public static float Area(IList<Vector2> p)
        {
            float s = 0; for (int i = 0; i < p.Count; i++) { var a = p[i]; var b = p[(i + 1) % p.Count]; s += a.x * b.y - b.x * a.y; }
            return 0.5f * s;
        }
        public static List<Vector2> Ccw(List<Vector2> p)
        {
            var r = new List<Vector2>(p); if (Area(r) < 0) r.Reverse(); return r;
        }
        public static List<Vector2> Offset(List<Vector2> p, float d)
        {
            int n = p.Count; var r = new List<Vector2>(n);
            for (int i = 0; i < n; i++)
            {
                Vector2 a = p[(i + n - 1) % n], b = p[i], c = p[(i + 1) % n];
                Vector2 e1 = (b - a).normalized, e2 = (c - b).normalized;
                Vector2 n1 = new Vector2(e1.y, -e1.x), n2 = new Vector2(e2.y, -e2.x);
                Vector2 m = n1 + n2; float k = 1f + Vector2.Dot(n1, n2);
                r.Add(b + d * (m / Mathf.Max(0.35f, k)));
            }
            return r;
        }
        static bool InTri(Vector2 p, Vector2 a, Vector2 b, Vector2 c)
        {
            float d1 = Cross2(b - a, p - a), d2 = Cross2(c - b, p - b), d3 = Cross2(a - c, p - c);
            bool neg = d1 < 0 || d2 < 0 || d3 < 0, pos = d1 > 0 || d2 > 0 || d3 > 0;
            return !(neg && pos);
        }
        public static List<int> Triangulate(List<Vector2> p)
        {
            var idx = new List<int>(); for (int i = 0; i < p.Count; i++) idx.Add(i);
            var res = new List<int>(); int guard = 0;
            while (idx.Count > 3 && guard++ < 4000)
            {
                bool clipped = false;
                for (int i = 0; i < idx.Count; i++)
                {
                    int a = idx[(i + idx.Count - 1) % idx.Count], b = idx[i], c = idx[(i + 1) % idx.Count];
                    if (Cross2(p[b] - p[a], p[c] - p[b]) <= 1e-12f) continue;
                    bool inside = false;
                    foreach (int k in idx) { if (k == a || k == b || k == c) continue; if (InTri(p[k], p[a], p[b], p[c])) { inside = true; break; } }
                    if (inside) continue;
                    res.Add(a); res.Add(b); res.Add(c); idx.RemoveAt(i); clipped = true; break;
                }
                if (!clipped) break;
            }
            if (idx.Count == 3) { res.Add(idx[0]); res.Add(idx[1]); res.Add(idx[2]); }
            return res;
        }

        void Cap(List<Vector2> poly, float y, bool up)
        {
            var tri = Triangulate(poly);
            Vector3 n = up ? Vector3.up : Vector3.down;
            for (int i = 0; i < tri.Count; i += 3)
            {
                Vector3 a = V3(poly[tri[i]], y), b = V3(poly[tri[i + 1]], y), c = V3(poly[tri[i + 2]], y);
                Vector3 nx = _xf.MultiplyVector(n);
                int i0 = AddVert(a, n, BoxUv(_xf.MultiplyPoint3x4(a), nx) * UvScale);
                int i1 = AddVert(b, n, BoxUv(_xf.MultiplyPoint3x4(b), nx) * UvScale);
                int i2 = AddVert(c, n, BoxUv(_xf.MultiplyPoint3x4(c), nx) * UvScale);
                Face(i0, i1, i2);
            }
        }

        /// <summary>Extruded polygon (x,z) from y0 to y1 with a 45-degree chamfer of size bevel.</summary>
        public void Prism(List<Vector2> polyIn, float y0, float y1, float bevel)
        {
            var poly = Ccw(polyIn); int n = poly.Count;
            float b = Mathf.Min(bevel, (y1 - y0) * 0.45f);
            var inner = b > 0 ? Offset(poly, -b) : poly;
            for (int i = 0; i < n; i++)
            {
                int j = (i + 1) % n;
                Vector2 e = (poly[j] - poly[i]).normalized; Vector2 o = new Vector2(e.y, -e.x);
                Quad(V3(poly[i], y0 + b), V3(poly[j], y0 + b), V3(poly[j], y1 - b), V3(poly[i], y1 - b), new Vector3(o.x, 0, o.y));
                if (b > 0)
                {
                    Quad(V3(poly[i], y1 - b), V3(poly[j], y1 - b), V3(inner[j], y1), V3(inner[i], y1), new Vector3(o.x, 1f, o.y));
                    Quad(V3(poly[i], y0 + b), V3(poly[j], y0 + b), V3(inner[j], y0), V3(inner[i], y0), new Vector3(o.x, -1f, o.y));
                }
            }
            Cap(inner, y1, true); Cap(inner, y0, false);
        }

        public static List<Vector2> RectPoly(float cx, float cz, float sx, float sz, float cham)
        {
            float hx = sx * 0.5f, hz = sz * 0.5f, c = Mathf.Min(cham, Mathf.Min(hx, hz) * 0.95f);
            var l = new List<Vector2>();
            if (c <= 0)
            {
                l.Add(new Vector2(cx - hx, cz - hz)); l.Add(new Vector2(cx + hx, cz - hz)); l.Add(new Vector2(cx + hx, cz + hz)); l.Add(new Vector2(cx - hx, cz + hz));
                return l;
            }
            l.Add(new Vector2(cx - hx + c, cz - hz)); l.Add(new Vector2(cx + hx - c, cz - hz)); l.Add(new Vector2(cx + hx, cz - hz + c)); l.Add(new Vector2(cx + hx, cz + hz - c));
            l.Add(new Vector2(cx + hx - c, cz + hz)); l.Add(new Vector2(cx - hx + c, cz + hz)); l.Add(new Vector2(cx - hx, cz + hz - c)); l.Add(new Vector2(cx - hx, cz - hz + c));
            return l;
        }

        public static List<Vector2> RoundRectPoly(float cx, float cz, float sx, float sz, float r, int arcSeg)
        {
            float hx = sx * 0.5f - r, hz = sz * 0.5f - r; var l = new List<Vector2>();
            Vector2[] cs = { new Vector2(cx + hx, cz + hz), new Vector2(cx - hx, cz + hz), new Vector2(cx - hx, cz - hz), new Vector2(cx + hx, cz - hz) };
            for (int c = 0; c < 4; c++)
                for (int k = 0; k <= arcSeg; k++)
                {
                    float a = (c * 90f + 90f * k / arcSeg) * Mathf.Deg2Rad;
                    l.Add(cs[c] + r * new Vector2(Mathf.Cos(a), Mathf.Sin(a)));
                }
            return l;
        }

        public static List<Vector2> CirclePoly(float cx, float cz, float r, int segs)
        {
            var l = new List<Vector2>();
            for (int i = 0; i < segs; i++) { float a = 2 * Mathf.PI * i / segs; l.Add(new Vector2(cx + r * Mathf.Cos(a), cz + r * Mathf.Sin(a))); }
            return l;
        }

        public void Box(Vector3 center, Vector3 size, float bevel)
        {
            Prism(RectPoly(center.x, center.z, size.x, size.z, 0), center.y - size.y * 0.5f, center.y + size.y * 0.5f, bevel);
        }

        // ---------------- lathe ----------------
        /// <summary>Surface of revolution about +Y. Profile of (radius, y), run bottom axis outward, up, inward to top axis.</summary>
        public void Lathe(IList<Vector2> prof, int segs, bool flat)
        {
            for (int e = 0; e < prof.Count - 1; e++)
            {
                Vector2 a = prof[e], b = prof[e + 1], d = b - a;
                if (d.sqrMagnitude < 1e-12f) continue;
                Vector2 nr = new Vector2(d.y, -d.x).normalized;
                float rmax = Mathf.Max(a.x, b.x);
                for (int s = 0; s < segs; s++)
                {
                    float a0 = 2 * Mathf.PI * s / segs, a1 = 2 * Mathf.PI * (s + 1) / segs, am = 0.5f * (a0 + a1);
                    Vector3 n0 = new Vector3(nr.x * Mathf.Cos(a0), nr.y, nr.x * Mathf.Sin(a0));
                    Vector3 n1 = new Vector3(nr.x * Mathf.Cos(a1), nr.y, nr.x * Mathf.Sin(a1));
                    Vector3 nm = new Vector3(nr.x * Mathf.Cos(am), nr.y, nr.x * Mathf.Sin(am));
                    if (flat) { n0 = nm; n1 = nm; }
                    float u0 = a0 / (2 * Mathf.PI) * 2 * Mathf.PI * rmax, u1 = a1 / (2 * Mathf.PI) * 2 * Mathf.PI * rmax;
                    int i0 = AddVert(new Vector3(a.x * Mathf.Cos(a0), a.y, a.x * Mathf.Sin(a0)), n0, new Vector2(u0, a.y));
                    int i1 = AddVert(new Vector3(a.x * Mathf.Cos(a1), a.y, a.x * Mathf.Sin(a1)), n1, new Vector2(u1, a.y));
                    int i2 = AddVert(new Vector3(b.x * Mathf.Cos(a1), b.y, b.x * Mathf.Sin(a1)), n1, new Vector2(u1, b.y));
                    int i3 = AddVert(new Vector3(b.x * Mathf.Cos(a0), b.y, b.x * Mathf.Sin(a0)), n0, new Vector2(u0, b.y));
                    Face(i0, i1, i2); Face(i0, i2, i3);
                }
            }
        }

        public void Cyl(float r, float y0, float y1, int segs, float bevel, bool flat = false)
        {
            var p = new List<Vector2>(); p.Add(new Vector2(0, y0));
            if (bevel > 0) { p.Add(new Vector2(r - bevel, y0)); p.Add(new Vector2(r, y0 + bevel)); p.Add(new Vector2(r, y1 - bevel)); p.Add(new Vector2(r - bevel, y1)); }
            else { p.Add(new Vector2(r, y0)); p.Add(new Vector2(r, y1)); }
            p.Add(new Vector2(0, y1));
            Lathe(p, segs, flat);
        }

        /// <summary>Cylinder lying along an arbitrary direction from p0 to p1.</summary>
        public void CylBetween(Vector3 p0, Vector3 p1, float r, int segs, float bevel)
        {
            Vector3 d = p1 - p0; float len = d.magnitude; if (len < 1e-6f) return;
            Push(p0, Quaternion.FromToRotation(Vector3.up, d / len));
            Cyl(r, 0, len, segs, bevel);
            Pop();
        }

        public void Sphere(Vector3 c, float r, int segs, int rings, float yScale = 1f)
        {
            var p = new List<Vector2>();
            for (int i = 0; i <= rings; i++) { float a = Mathf.PI * i / rings; p.Add(new Vector2(r * Mathf.Sin(a), -r * Mathf.Cos(a) * yScale)); }
            Push(c, Quaternion.identity); Lathe(p, segs, false); Pop();
        }

        // ---------------- tubes and grids ----------------
        public static List<Vector3> CatmullRom(List<Vector3> pts, int sub)
        {
            var r = new List<Vector3>();
            for (int i = 0; i < pts.Count - 1; i++)
            {
                Vector3 p0 = pts[Mathf.Max(0, i - 1)], p1 = pts[i], p2 = pts[i + 1], p3 = pts[Mathf.Min(pts.Count - 1, i + 2)];
                for (int s = 0; s < sub; s++)
                {
                    float t = (float)s / sub, t2 = t * t, t3 = t2 * t;
                    r.Add(0.5f * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2 + (-p0 + 3 * p1 - 3 * p2 + p3) * t3));
                }
            }
            r.Add(pts[pts.Count - 1]); return r;
        }

        public void Tube(List<Vector3> path, float radius, int segs, bool capEnds = true)
        {
            int n = path.Count; if (n < 2) return;
            var rings = new Vector3[n, segs]; var nrm = new Vector3[n, segs];
            Vector3 t0 = (path[1] - path[0]).normalized;
            Vector3 up = Mathf.Abs(Vector3.Dot(t0, Vector3.up)) > 0.9f ? Vector3.right : Vector3.up;
            Vector3 nn = Vector3.Cross(t0, up).normalized;
            for (int i = 0; i < n; i++)
            {
                Vector3 t = (path[Mathf.Min(n - 1, i + 1)] - path[Mathf.Max(0, i - 1)]).normalized;
                nn = (nn - Vector3.Dot(nn, t) * t).normalized; Vector3 bb = Vector3.Cross(t, nn);
                for (int j = 0; j < segs; j++)
                {
                    float a = 2 * Mathf.PI * j / segs; Vector3 dir = Mathf.Cos(a) * nn + Mathf.Sin(a) * bb;
                    rings[i, j] = path[i] + dir * radius; nrm[i, j] = dir;
                }
            }
            var idx = new int[n, segs]; float acc = 0;
            for (int i = 0; i < n; i++)
            {
                if (i > 0) acc += (path[i] - path[i - 1]).magnitude;
                for (int j = 0; j <= segs - 1; j++) idx[i, j] = AddVert(rings[i, j], nrm[i, j], new Vector2(j * 0.25f, acc * 20f));
            }
            for (int i = 0; i < n - 1; i++)
                for (int j = 0; j < segs; j++)
                {
                    int k = (j + 1) % segs; Face(idx[i, j], idx[i, k], idx[i + 1, k]); Face(idx[i, j], idx[i + 1, k], idx[i + 1, j]);
                }
            if (capEnds)
            {
                for (int e = 0; e < 2; e++)
                {
                    int i = e == 0 ? 0 : n - 1; Vector3 dir = e == 0 ? (path[0] - path[1]).normalized : (path[n - 1] - path[n - 2]).normalized;
                    int c = AddVert(path[i], dir, Vector2.zero);
                    for (int j = 0; j < segs; j++)
                    {
                        int k = (j + 1) % segs;
                        int a = AddVert(rings[i, j], dir, Vector2.zero), b = AddVert(rings[i, k], dir, Vector2.zero);
                        Face(c, a, b);
                    }
                }
            }
        }

        /// <summary>Smooth surface from a grid P[i,j]; j closes into a loop. Normals by central differences.</summary>
        public void Grid(Vector3[,] P, bool capEnd)
        {
            int ni = P.GetLength(0), nj = P.GetLength(1);
            var id = new int[ni, nj];
            for (int i = 0; i < ni; i++)
            {
                Vector3 c = Vector3.zero; for (int j = 0; j < nj; j++) c += P[i, j]; c /= nj;
                for (int j = 0; j < nj; j++)
                {
                    Vector3 du = P[Mathf.Min(ni - 1, i + 1), j] - P[Mathf.Max(0, i - 1), j];
                    Vector3 dv = P[i, (j + 1) % nj] - P[i, (j + nj - 1) % nj];
                    Vector3 n = Vector3.Cross(du, dv);
                    if (n.sqrMagnitude < 1e-16f) n = P[i, j] - c;
                    n.Normalize();
                    // outward = away from the section centroid (works for thin lens-shaped sections)
                    if (Vector3.Dot(n, P[i, j] - c) < 0f) n = -n;
                    id[i, j] = AddVert(P[i, j], n, new Vector2(P[i, j].x, P[i, j].z));
                }
            }
            for (int i = 0; i < ni - 1; i++)
                for (int j = 0; j < nj; j++)
                {
                    int k = (j + 1) % nj; Face(id[i, j], id[i, k], id[i + 1, k]); Face(id[i, j], id[i + 1, k], id[i + 1, j]);
                }
            if (capEnd)
            {
                int i = ni - 1; Vector3 c = Vector3.zero; for (int j = 0; j < nj; j++) c += P[i, j]; c /= nj;
                Vector3 dir = (P[i, 0] - P[i - 1, 0]).normalized; dir = (c - (P[i - 1, 0] + P[i - 1, nj / 2]) * 0.5f).normalized;
                int ci = AddVert(c, dir, Vector2.zero);
                for (int j = 0; j < nj; j++)
                {
                    int k = (j + 1) % nj;
                    int a = AddVert(P[i, j], dir, Vector2.zero), b = AddVert(P[i, k], dir, Vector2.zero); Face(ci, a, b);
                }
            }
        }

        public Mesh ToMesh(string name)
        {
            var m = new Mesh { name = name };
            m.indexFormat = V.Count > 65000 ? UnityEngine.Rendering.IndexFormat.UInt32 : UnityEngine.Rendering.IndexFormat.UInt16;
            m.SetVertices(V); m.SetNormals(N); m.SetUVs(0, UV);
            m.subMeshCount = Order.Count;
            for (int i = 0; i < Order.Count; i++) m.SetTriangles(T[Order[i]], i);
            m.RecalculateBounds(); m.RecalculateTangents();
            return m;
        }
    }
}
