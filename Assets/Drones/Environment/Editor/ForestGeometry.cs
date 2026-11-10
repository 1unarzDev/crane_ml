using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace Crane.Drones.Environment.Editor
{
    /// <summary>Indexed mesh accumulator with per-submesh triangle lists.</summary>
    public class MB
    {
        public List<Vector3> v = new List<Vector3>(), n = new List<Vector3>();
        public List<Vector2> uv = new List<Vector2>();
        public List<int>[] tris;
        public MB(int subs) { tris = new List<int>[subs]; for (int i = 0; i < subs; i++) tris[i] = new List<int>(); }
        public int Add(Vector3 p, Vector3 nrm, Vector2 u) { v.Add(p); n.Add(nrm); uv.Add(u); return v.Count - 1; }
        public void Tri(int s, int a, int b, int c) { tris[s].Add(a); tris[s].Add(b); tris[s].Add(c); }
        public int Count { get { return v.Count; } }

        public Mesh ToMesh(string name, bool tangents)
        {
            var m = new Mesh { name = name, indexFormat = IndexFormat.UInt32 };
            m.SetVertices(v); m.SetNormals(n); m.SetUVs(0, uv);
            m.subMeshCount = tris.Length;
            for (int i = 0; i < tris.Length; i++) m.SetTriangles(tris[i], i);
            if (tangents) m.RecalculateTangents();
            m.RecalculateBounds();
            return m;
        }

        /// <summary>Positions + triangles of the given submeshes only (collision proxy).</summary>
        public Mesh ToCollisionMesh(string name, params int[] subs)
        {
            var map = new Dictionary<int, int>(); var nv = new List<Vector3>(); var nt = new List<int>();
            foreach (int s in subs)
                foreach (int idx in tris[s])
                {
                    int j; if (!map.TryGetValue(idx, out j)) { j = nv.Count; map[idx] = j; nv.Add(v[idx]); }
                    nt.Add(j);
                }
            var m = new Mesh { name = name, indexFormat = IndexFormat.UInt32 };
            m.SetVertices(nv); m.SetTriangles(nt, 0); m.RecalculateBounds();
            return m;
        }
    }

    public static class Geo
    {
        /// <summary>Swept tube with parallel-transport frames. ex/ey scale the cross-section (ellipse).</summary>
        public static void Tube(MB mb, int sub, IList<Vector3> pts, IList<float> rad, int sides, float uTiles, float vMeters,
            float ex = 1f, float ey = 1f, Vector3? refA = null, bool capStart = false, bool capEnd = false, float uOff = 0f)
        {
            int N = pts.Count; if (N < 2) return;
            Vector3 t0 = (pts[1] - pts[0]).normalized;
            Vector3 a = refA.HasValue ? refA.Value : (Mathf.Abs(t0.y) < 0.9f ? Vector3.up : Vector3.right);
            a = (a - t0 * Vector3.Dot(a, t0)); if (a.sqrMagnitude < 1e-6f) a = Vector3.Cross(t0, Vector3.right); a.Normalize();
            int first = mb.Count; float cum = 0f; var frames = new Vector3[N]; var bs = new Vector3[N];
            for (int i = 0; i < N; i++)
            {
                Vector3 t = (pts[Mathf.Min(i + 1, N - 1)] - pts[Mathf.Max(i - 1, 0)]).normalized;
                a = a - t * Vector3.Dot(a, t); if (a.sqrMagnitude < 1e-8f) a = Vector3.Cross(t, Vector3.up); a.Normalize();
                Vector3 b = Vector3.Cross(t, a);
                frames[i] = a; bs[i] = b;
                if (i > 0) cum += (pts[i] - pts[i - 1]).magnitude;
                for (int k = 0; k <= sides; k++)
                {
                    float th = (float)k / sides * Mathf.PI * 2f; float c = Mathf.Cos(th), s = Mathf.Sin(th);
                    Vector3 off = a * (c * ex) + b * (s * ey);
                    Vector3 nrm = (a * (c / ex) + b * (s / ey)).normalized;
                    mb.Add(pts[i] + off * rad[i], nrm, new Vector2(uOff + (float)k / sides * uTiles, cum / vMeters));
                }
            }
            int stride = sides + 1;
            for (int i = 0; i < N - 1; i++)
                for (int k = 0; k < sides; k++)
                {
                    int p00 = first + i * stride + k, p01 = p00 + 1, p10 = p00 + stride, p11 = p10 + 1;
                    mb.Tri(sub, p00, p10, p11); mb.Tri(sub, p00, p11, p01);
                }
            if (capEnd) Cap(mb, sub, pts[N - 1], (pts[N - 1] - pts[N - 2]).normalized, frames[N - 1], bs[N - 1], rad[N - 1], sides, ex, ey, true);
            if (capStart) Cap(mb, sub, pts[0], -(pts[1] - pts[0]).normalized, frames[0], bs[0], rad[0], sides, ex, ey, false);
        }

        static void Cap(MB mb, int sub, Vector3 c, Vector3 nrm, Vector3 a, Vector3 b, float r, int sides, float ex, float ey, bool end)
        {
            int ci = mb.Add(c, nrm, new Vector2(0.5f, 0.5f)); int f = mb.Count;
            for (int k = 0; k <= sides; k++)
            {
                float th = (float)k / sides * Mathf.PI * 2f;
                mb.Add(c + (a * (Mathf.Cos(th) * ex) + b * (Mathf.Sin(th) * ey)) * r, nrm, new Vector2(0.5f + 0.5f * Mathf.Cos(th), 0.5f + 0.5f * Mathf.Sin(th)));
            }
            for (int k = 0; k < sides; k++) { if (end) mb.Tri(sub, ci, f + k + 1, f + k); else mb.Tri(sub, ci, f + k, f + k + 1); }
        }

        /// <summary>Double-sided-ready foliage quad. Normal is the shading normal (typically blended outward).</summary>
        public static void Card(MB mb, int sub, Vector3 c, Vector3 right, Vector3 up, float w, float h, Vector3 shadeN, float uMin = 0, float uMax = 1)
        {
            Vector3 r = right.normalized * (w * 0.5f), u = up.normalized * (h * 0.5f);
            int i = mb.Add(c - r - u, shadeN, new Vector2(uMin, 0)); mb.Add(c + r - u, shadeN, new Vector2(uMax, 0));
            mb.Add(c + r + u, shadeN, new Vector2(uMax, 1)); mb.Add(c - r + u, shadeN, new Vector2(uMin, 1));
            mb.Tri(sub, i, i + 2, i + 1); mb.Tri(sub, i, i + 3, i + 2);
        }

        public static Vector3 RandUnit(System.Random r)
        {
            float z = (float)(r.NextDouble() * 2 - 1), t = (float)(r.NextDouble() * Mathf.PI * 2);
            float s = Mathf.Sqrt(1 - z * z); return new Vector3(s * Mathf.Cos(t), z, s * Mathf.Sin(t));
        }
        public static float R(System.Random r, float a, float b) { return a + (b - a) * (float)r.NextDouble(); }
        public static float Gauss(System.Random r)
        {
            double u1 = 1.0 - r.NextDouble(), u2 = r.NextDouble();
            return (float)(Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2));
        }

        /// <summary>Displaced icosphere used for rocks and litter mounds. Submesh 0 = body, 1 = moss shell (top faces, patchy).</summary>
        public static MB Rock(System.Random rng, Vector3 size, float rough, bool moss, int seedOff)
        {
            var ico = Icosphere(3);
            var mb = new MB(2);
            Vector3 off = new Vector3(rng.Next(0, 1000), rng.Next(0, 1000), rng.Next(0, 1000));
            var pos = new Vector3[ico.Item1.Count];
            for (int i = 0; i < pos.Length; i++)
            {
                Vector3 d = ico.Item1[i];
                float nz = Noise3(d * 1.7f + off) * 0.5f + Noise3(d * 4f + off) * 0.18f * rough + Noise3(d * 9f + off) * 0.05f * rough;
                float rr = 0.78f + nz;
                Vector3 p = new Vector3(d.x * size.x, d.y * size.y, d.z * size.z) * rr;
                if (p.y < -size.y * 0.35f) p.y = -size.y * 0.35f; // flat buried base
                pos[i] = p;
            }
            var tmp = new Mesh(); tmp.SetVertices(pos); tmp.SetTriangles(ico.Item2, 0); tmp.RecalculateNormals();
            var nrm = tmp.normals;
            for (int i = 0; i < pos.Length; i++)
            {
                Vector3 an = new Vector3(Mathf.Abs(nrm[i].x), Mathf.Abs(nrm[i].y), Mathf.Abs(nrm[i].z));
                Vector2 uv = an.y >= an.x && an.y >= an.z ? new Vector2(pos[i].x, pos[i].z) : (an.x >= an.z ? new Vector2(pos[i].z, pos[i].y) : new Vector2(pos[i].x, pos[i].y));
                mb.Add(pos[i], nrm[i], uv * 0.6f);
            }
            for (int i = 0; i < ico.Item2.Length; i += 3) mb.Tri(0, ico.Item2[i], ico.Item2[i + 1], ico.Item2[i + 2]);
            if (moss) MossShell(mb, 0, 1, off, 0.015f);
            return mb;
        }

        /// <summary>Copy upward-facing triangles of submesh src into submesh dst, pushed outward, where patch noise is high.</summary>
        public static void MossShell(MB mb, int src, int dst, Vector3 seed, float lift)
        {
            var map = new Dictionary<int, int>();
            var t = mb.tris[src];
            for (int i = 0; i < t.Count; i += 3)
            {
                Vector3 c = (mb.v[t[i]] + mb.v[t[i + 1]] + mb.v[t[i + 2]]) / 3f;
                Vector3 nn = (mb.n[t[i]] + mb.n[t[i + 1]] + mb.n[t[i + 2]]).normalized;
                float patch = Noise3(c * 2.3f + seed * 0.37f) + 0.25f * Noise3(c * 9f);
                if (nn.y < 0.25f || patch < 0.1f) continue;
                int[] idx = new int[3];
                for (int k = 0; k < 3; k++)
                {
                    int o = t[i + k]; int j;
                    if (!map.TryGetValue(o, out j)) { j = mb.Add(mb.v[o] + mb.n[o] * lift, mb.n[o], mb.uv[o] * 3f); map[o] = j; }
                    idx[k] = j;
                }
                mb.Tri(dst, idx[0], idx[1], idx[2]);
            }
        }

        public static float Noise3(Vector3 p)
        {
            float xy = Mathf.PerlinNoise(p.x + 17.3f, p.y + 5.1f), yz = Mathf.PerlinNoise(p.y + 3.7f, p.z + 11.9f), zx = Mathf.PerlinNoise(p.z + 29.1f, p.x + 7.7f);
            return (xy + yz + zx) / 3f - 0.5f;
        }

        public static Tuple<List<Vector3>, int[]> Icosphere(int subdiv)
        {
            float t = (1f + Mathf.Sqrt(5f)) / 2f;
            var v = new List<Vector3> { new Vector3(-1, t, 0), new Vector3(1, t, 0), new Vector3(-1, -t, 0), new Vector3(1, -t, 0), new Vector3(0, -1, t), new Vector3(0, 1, t), new Vector3(0, -1, -t), new Vector3(0, 1, -t), new Vector3(t, 0, -1), new Vector3(t, 0, 1), new Vector3(-t, 0, -1), new Vector3(-t, 0, 1) };
            for (int i = 0; i < v.Count; i++) v[i] = v[i].normalized;
            var f = new List<int> { 0, 11, 5, 0, 5, 1, 0, 1, 7, 0, 7, 10, 0, 10, 11, 1, 5, 9, 5, 11, 4, 11, 10, 2, 10, 7, 6, 7, 1, 8, 3, 9, 4, 3, 4, 2, 3, 2, 6, 3, 6, 8, 3, 8, 9, 4, 9, 5, 2, 4, 11, 6, 2, 10, 8, 6, 7, 9, 8, 1 };
            for (int s = 0; s < subdiv; s++)
            {
                var cache = new Dictionary<long, int>(); var nf = new List<int>();
                Func<int, int, int> mid = (a, b) =>
                {
                    long key = a < b ? ((long)a << 32) + b : ((long)b << 32) + a; int m;
                    if (!cache.TryGetValue(key, out m)) { m = v.Count; v.Add(((v[a] + v[b]) * 0.5f).normalized); cache[key] = m; }
                    return m;
                };
                for (int i = 0; i < f.Count; i += 3)
                {
                    int a = f[i], b = f[i + 1], c = f[i + 2]; int ab = mid(a, b), bc = mid(b, c), ca = mid(c, a);
                    nf.AddRange(new[] { a, ab, ca, b, bc, ab, c, ca, bc, ab, bc, ca });
                }
                f = nf;
            }
            // Unity winding is clockwise for front faces; flip generated (counter-clockwise outward) list.
            for (int i = 0; i < f.Count; i += 3) { int tmp = f[i + 1]; f[i + 1] = f[i + 2]; f[i + 2] = tmp; }
            return Tuple.Create(v, f.ToArray());
        }
    }
}
