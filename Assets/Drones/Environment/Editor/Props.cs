using System;
using System.Collections.Generic;
using UnityEngine;

namespace Crane.Drones.Environment.Editor
{
    public static class Props
    {
        /// <summary>Fallen log along +X: tapered, bowed, flared broken end, branch stubs, optional moss shell. Sub 0 bark, 1 moss, 2 cut wood.</summary>
        public static MB Log(System.Random rng, float len, float r0, bool broadleaf)
        {
            var mb = new MB(3);
            int m = Mathf.Max(6, (int)(len / 0.5f)); var pts = new List<Vector3>(); var rad = new List<float>();
            float bow = Geo.R(rng, -0.12f, 0.12f), ph = Geo.R(rng, 0, 50);
            for (int i = 0; i <= m; i++)
            {
                float t = (float)i / m; pts.Add(new Vector3(t * len, 0.0f + Mathf.Sin(t * 3.14f) * bow * 0.3f, Mathf.Sin(t * 3.14f) * bow + (Mathf.PerlinNoise(ph, t * 4) - 0.5f) * 0.1f));
                rad.Add(r0 * Mathf.Lerp(1f, 0.55f, t) * (1f + 0.06f * Mathf.Sin(t * 19 + ph)) * (1f + 0.25f * Mathf.Exp(-t * 12f)));
            }
            Geo.Tube(mb, 0, pts, rad, 14, Mathf.Max(2, Mathf.Round(Mathf.PI * 2 * r0 / 1.4f)), 1.4f, 1, 1, null, false, false);
            // cut / broken end caps with lighter wood
            Geo.Tube(mb, 2, new List<Vector3> { pts[0] - Vector3.right * 0.001f, pts[0] }, new List<float> { rad[0] * 0.995f, rad[0] * 0.995f }, 14, 1, 1, 1, 1, null, true, false);
            Geo.Tube(mb, 2, new List<Vector3> { pts[m], pts[m] + Vector3.right * 0.001f }, new List<float> { rad[m] * 0.8f, rad[m] * 0.6f }, 14, 1, 1, 1, 1, null, false, true);
            int stubs = rng.Next(3, 8);
            for (int s = 0; s < stubs; s++)
            {
                float t = Geo.R(rng, 0.05f, 0.9f); int i = Mathf.Min(m - 1, (int)(t * m)); Vector3 bp = pts[i];
                Vector3 dir = new Vector3(Geo.R(rng, -0.3f, 0.3f), Geo.R(rng, 0.3f, 1f), Geo.R(rng, -1f, 1f)).normalized;
                float L = Geo.R(rng, 0.25f, 1.1f); float rr = Geo.R(rng, 0.012f, 0.03f);
                var bp2 = new List<Vector3>(); var br = new List<float>(); Vector3 p = bp + dir * rad[i] * 0.7f, d = dir;
                for (int j = 0; j < 4; j++) { bp2.Add(p); br.Add(rr * (1 - 0.7f * j / 3f)); d = (d + Geo.RandUnit(rng) * 0.3f + Vector3.down * .05f).normalized; p += d * L / 3f; }
                Geo.Tube(mb, 0, bp2, br, 5, 1, 0.5f);
            }
            if (broadleaf || rng.NextDouble() < 0.7) Geo.MossShell(mb, 0, 1, new Vector3(rng.Next(100), rng.Next(100), rng.Next(100)), 0.012f);
            return mb;
        }

        /// <summary>Dead stick: thin bent tube.</summary>
        public static MB Stick(System.Random rng, float len, float r)
        {
            var mb = new MB(1); var pts = new List<Vector3>(); var rad = new List<float>(); Vector3 p = Vector3.zero, d = Vector3.right;
            int m = Mathf.Max(3, (int)(len / 0.25f));
            for (int i = 0; i <= m; i++) { pts.Add(p); rad.Add(Mathf.Max(0.003f, r * (1 - 0.7f * i / m))); d = (d + Geo.RandUnit(rng) * 0.22f).normalized; p += d * (len / m); }
            Geo.Tube(mb, 0, pts, rad, 4, 1, 0.5f);
            return mb;
        }

        /// <summary>Fern clump: radiating arching fronds as two-sided cards textured with a frond atlas.</summary>
        public static MB Fern(System.Random rng, float size, int variant = 0)
        {
            var mb = new MB(1); int fronds = variant == 1 ? rng.Next(4, 7) : (variant == 2 ? rng.Next(10, 16) : (variant == 3 ? rng.Next(10, 15) : rng.Next(7, 12)));
            if (variant == 1) size *= 1.25f; else if (variant == 2) size *= 0.85f; else if (variant == 3) size *= 0.55f;
            for (int f = 0; f < fronds; f++)
            {
                float az = (f + Geo.R(rng, -0.3f, 0.3f)) / fronds * 6.283f; Vector3 dir = new Vector3(Mathf.Cos(az), 0, Mathf.Sin(az));
                float len = size * Geo.R(rng, 0.7f, 1.2f); float lift = variant == 1 ? Geo.R(rng, 0.08f, 0.22f) : (variant == 2 ? Geo.R(rng, 0.55f, 0.9f) : Geo.R(rng, 0.25f, 0.55f));
                int seg = 4; int i0 = mb.Count;
                for (int s = 0; s <= seg; s++)
                {
                    float t = (float)s / seg; Vector3 c = dir * (len * t) + Vector3.up * (len * (lift * Mathf.Sin(t * 1.9f) - 0.25f * t * t));
                    Vector3 side = Vector3.Cross(Vector3.up, dir).normalized * (len * (variant == 1 ? 0.3f : (variant == 2 ? 0.14f : 0.2f)) * Mathf.Sin(Mathf.Min(1, t * 2.4f) * 1.57f) * (1f - 0.55f * t) + 0.01f);
                    Vector3 nn = (Vector3.up + dir * 0.3f).normalized;
                    mb.Add(c - side, nn, new Vector2(0, t)); mb.Add(c + side, nn, new Vector2(1, t));
                }
                for (int s = 0; s < seg; s++) { int a = i0 + s * 2; mb.Tri(0, a, a + 2, a + 3); mb.Tri(0, a, a + 3, a + 1); }
            }
            return mb;
        }

        /// <summary>Shrub / bramble: thin arching stems with leaf cards. Sub 0 stems, 1 leaves.</summary>
        public static MB Shrub(System.Random rng, float size, float openness)
        {
            var mb = new MB(2); int stems = rng.Next(7, 14);
            for (int s = 0; s < stems; s++)
            {
                float az = Geo.R(rng, 0, 6.28f); Vector3 dir = new Vector3(Mathf.Cos(az) * 0.5f, 1f, Mathf.Sin(az) * 0.5f).normalized; Vector3 p = new Vector3(Geo.R(rng, -.12f, .12f), 0, Geo.R(rng, -.12f, .12f)), d = dir;
                int m = 6; float len = size * Geo.R(rng, 0.7f, 1.3f); var pts = new List<Vector3>(); var rad = new List<float>();
                for (int i = 0; i < m; i++) { pts.Add(p); rad.Add(Mathf.Max(0.003f, 0.012f * (1 - 0.8f * i / (m - 1)))); d = (d + new Vector3(Mathf.Cos(az), -0.28f, Mathf.Sin(az)) * 0.2f + Geo.RandUnit(rng) * 0.15f).normalized; p += d * len / (m - 1); }
                Geo.Tube(mb, 0, pts, rad, 4, 1, 0.5f);
                for (int i = 2; i < m; i++)
                {
                    if (rng.NextDouble() < openness * 0.3f) continue;
                    float w = Geo.R(rng, 0.28f, 0.45f); Vector3 a = Geo.RandUnit(rng); Vector3 r = Vector3.Cross(a, Vector3.up); if (r.sqrMagnitude < .01f) r = Vector3.right; r.Normalize();
                    Geo.Card(mb, 1, pts[i], r, Vector3.Cross(r, a).normalized, w, w, (Vector3.up + (pts[i] - Vector3.up * size * 0.4f).normalized).normalized);
                    if (rng.NextDouble() < 0.6) Geo.Card(mb, 1, pts[i] + a * 0.03f, Vector3.Cross(Vector3.up, r).normalized, a, w, w, Vector3.up);
                }
            }
            return mb;
        }

        /// <summary>Flat leaf-litter scatter: ground-hugging cards using the leaf atlas.</summary>
        public static MB LitterPatch(System.Random rng, float radius, int n)
        {
            var mb = new MB(1);
            for (int i = 0; i < n; i++)
            {
                float r = radius * Mathf.Sqrt((float)rng.NextDouble()), a = Geo.R(rng, 0, 6.28f); float yaw = Geo.R(rng, 0, 6.28f);
                Vector3 right = new Vector3(Mathf.Cos(yaw), 0, Mathf.Sin(yaw)), up = new Vector3(-Mathf.Sin(yaw), Geo.R(rng, 0, .25f), Mathf.Cos(yaw));
                float s = Geo.R(rng, 0.25f, 0.42f);
                Geo.Card(mb, 0, new Vector3(Mathf.Cos(a) * r, Geo.R(rng, 0.01f, 0.05f), Mathf.Sin(a) * r), right, Vector3.Cross(right, Vector3.up).normalized + up * .1f, s, s, Vector3.up);
            }
            return mb;
        }

        /// <summary>Grass tuft: crossed vertical cards sampling the blade atlas, plus leaning outer cards.</summary>
        public static MB GrassTuft(System.Random rng, float size)
        {
            var mb = new MB(1); int cards = rng.Next(5, 8);
            for (int i = 0; i < cards; i++)
            {
                float yaw = (i + Geo.R(rng, -0.2f, 0.2f)) / cards * 3.1416f; Vector3 right = new Vector3(Mathf.Cos(yaw), 0, Mathf.Sin(yaw));
                Vector3 c = new Vector3(Geo.R(rng, -0.12f, 0.12f), size * 0.4f, Geo.R(rng, -0.12f, 0.12f)); float w = size * Geo.R(rng, 0.8f, 1.2f), h = size * Geo.R(rng, 0.7f, 1f);
                Vector3 up = Vector3.up + new Vector3(Geo.R(rng, -0.25f, 0.25f), 0, Geo.R(rng, -0.25f, 0.25f));
                Geo.Card(mb, 0, c + Vector3.up * (h * 0.1f), right, up.normalized, w, h, Vector3.up);
            }
            return mb;
        }

        /// <summary>Fallen branch along +X: forked thin bough with side twigs, resting on the ground (no collider).</summary>
        public static MB FallenBranch(System.Random rng, float len, float r)
        {
            var mb = new MB(1); AddBough(mb, rng, Vector3.zero, Vector3.right, len, r, 2); return mb;
        }
        static void AddBough(MB mb, System.Random rng, Vector3 o, Vector3 dir, float len, float r0, int depth)
        {
            int m = Mathf.Max(3, (int)(len / 0.25f) + 1); var pts = new List<Vector3>(); var rad = new List<float>(); Vector3 p = o, d = dir; float step = len / (m - 1);
            for (int i = 0; i < m; i++)
            {
                pts.Add(p); rad.Add(Mathf.Max(0.003f, r0 * (1f - 0.8f * i / (m - 1))));
                d = (d + new Vector3(Geo.R(rng, -0.25f, 0.25f), Geo.R(rng, -0.1f, 0.12f), Geo.R(rng, -0.3f, 0.3f))).normalized; p += d * step; if (p.y < 0.015f) { p.y = 0.015f; d.y = Mathf.Max(0f, d.y); }
            }
            Geo.Tube(mb, 0, pts, rad, r0 > 0.02f ? 6 : 4, 1, 0.5f);
            if (depth > 0)
            {
                int kids = Mathf.Max(2, (int)(len * 2.2f));
                for (int k = 0; k < kids; k++)
                {
                    float t = Geo.R(rng, 0.15f, 0.9f); int i = Mathf.Min(m - 2, (int)(t * (m - 1))); Vector3 bp = Vector3.Lerp(pts[i], pts[i + 1], t * (m - 1) - i);
                    Vector3 fw = (pts[i + 1] - pts[i]).normalized; Vector3 sd = new Vector3(-fw.z, 0, fw.x) * (k % 2 == 0 ? 1 : -1);
                    AddBough(mb, rng, bp, (fw * 0.6f + sd * 0.8f + Vector3.up * Geo.R(rng, 0f, 0.35f)).normalized, len * Geo.R(rng, 0.2f, 0.45f), Mathf.Max(0.004f, rad[i] * 0.55f), depth - 1);
                }
            }
        }

        public class MidResult { public Mesh visual, collision; public List<BranchSeg> segs = new List<BranchSeg>(); public float height, crownRadius; public string kind; }

        static void SegsOf(List<BranchSeg> segs, List<Vector3> pts, List<float> rad) { for (int i = 0; i < pts.Count - 1; i++) segs.Add(new BranchSeg { a = pts[i], b = pts[i + 1], r = (rad[i] + rad[i + 1]) * 0.5f }); }

        /// <summary>Mid-story broadleaf shrub / hazel thicket, height 1.5-4 m. Sub 0 stems (collision), sub 1 leaf clusters.</summary>
        public static MB MidShrub(System.Random rng, float height, List<BranchSeg> segs, out float crownR)
        {
            var mb = new MB(2); int stems = rng.Next(5, 9); crownR = 0.3f;
            for (int s = 0; s < stems; s++)
            {
                float az = Geo.R(rng, 0, 6.28f); float lean = Geo.R(rng, 0.12f, 0.55f); Vector3 d = new Vector3(Mathf.Cos(az) * lean, 1f, Mathf.Sin(az) * lean).normalized; Vector3 p = new Vector3(Geo.R(rng, -0.15f, 0.15f), -0.1f, Geo.R(rng, -0.15f, 0.15f));
                int m = 8; float len = height * Geo.R(rng, 0.75f, 1.15f); var pts = new List<Vector3>(); var rad = new List<float>(); float r0 = Geo.R(rng, 0.014f, 0.026f) * Mathf.Sqrt(height / 2.5f);
                for (int i = 0; i < m; i++)
                {
                    pts.Add(p); rad.Add(Mathf.Max(0.004f, r0 * (1f - 0.85f * i / (m - 1)))); d = (d + new Vector3(Mathf.Cos(az), -0.05f, Mathf.Sin(az)) * 0.07f + Geo.RandUnit(rng) * 0.1f).normalized; p += d * len / (m - 1);
                }
                Geo.Tube(mb, 0, pts, rad, 5, 1, 0.5f); SegsOf(segs, pts, rad);
                for (int i = 2; i < m; i++)
                {
                    // side shoots with leaf clusters
                    int shoots = i < 4 ? 1 : 2;
                    for (int k = 0; k < shoots; k++)
                    {
                        Vector3 sd = (Geo.RandUnit(rng) * 0.8f + new Vector3(Mathf.Cos(az), 0.2f, Mathf.Sin(az)) * 0.5f).normalized; float sl = Geo.R(rng, 0.25f, 0.6f) * (1.1f - i / (float)m * 0.4f);
                        var sp = new List<Vector3> { pts[i], pts[i] + sd * sl * 0.5f + Vector3.up * 0.04f, pts[i] + sd * sl }; var sr = new List<float> { rad[i] * 0.5f, rad[i] * 0.35f, 0.003f };
                        Geo.Tube(mb, 0, sp, sr, 3, 1, 0.5f); SegsOf(segs, sp, sr);
                        crownR = Mathf.Max(crownR, new Vector2(sp[2].x, sp[2].z).magnitude + 0.2f);
                        for (int c = 0; c < 4; c++)
                        {
                            Vector3 cp = Vector3.Lerp(sp[1], sp[2], Geo.R(rng, 0f, 1f)) + Geo.RandUnit(rng) * 0.1f; float w = Geo.R(rng, 0.3f, 0.6f); float win = 0.65f, u0 = Geo.R(rng, 0f, 0.35f);
                            Vector3 a = (Geo.RandUnit(rng) * 0.6f + Vector3.up * 0.4f).normalized; Vector3 r = Vector3.Cross(a, Vector3.up); if (r.sqrMagnitude < 0.01f) r = Vector3.right; r.Normalize();
                            Geo.Card(mb, 1, cp, r, Vector3.Cross(r, a).normalized, w * win, w, (Vector3.up + (cp - new Vector3(0, height * 0.4f, 0)).normalized).normalized, u0, u0 + win);
                        }
                    }
                }
                crownR = Mathf.Max(crownR, new Vector2(pts[m - 1].x, pts[m - 1].z).magnitude + 0.35f);
            }
            return mb;
        }

        /// <summary>Young conifer, 1.5-4 m: slim leader with dense whorls of needle boughs from near the ground. Sub 0 wood, 1 needles.</summary>
        public static MB YoungConifer(System.Random rng, float height, List<BranchSeg> segs, out float crownR)
        {
            var mb = new MB(2); crownR = height * 0.17f + 0.25f; float r0 = 0.012f + height * 0.006f;
            int m = Mathf.Max(5, (int)(height / 0.35f)); var pts = new List<Vector3>(); var rad = new List<float>();
            for (int i = 0; i <= m; i++) { float t = (float)i / m; pts.Add(new Vector3(0.03f * Mathf.Sin(t * 5f), -0.1f + (height + 0.1f) * t, 0.03f * Mathf.Cos(t * 4f))); rad.Add(Mathf.Max(0.004f, r0 * (1f - 0.9f * t))); }
            Geo.Tube(mb, 0, pts, rad, 5, 1, 0.5f); SegsOf(segs, pts, rad);
            float h = Mathf.Max(0.35f, height * 0.12f);
            while (h < height * 0.97f)
            {
                float t = h / height; int cnt = rng.Next(5, 8); float az0 = Geo.R(rng, 0, 6.28f); float blen = crownR * Mathf.Pow(1f - t, 0.9f) * 1.5f + 0.18f;
                for (int k = 0; k < cnt; k++)
                {
                    float az = az0 + (k + Geo.R(rng, -0.2f, 0.2f)) / cnt * 6.283f; Vector3 dir = new Vector3(Mathf.Cos(az), Mathf.Lerp(-0.2f, 0.45f, t) + Geo.R(rng, -0.08f, 0.08f), Mathf.Sin(az)).normalized; float L = blen * Geo.R(rng, 0.8f, 1.1f);
                    Vector3 o = new Vector3(pts[Mathf.Min(m, (int)(t * m))].x, h, pts[Mathf.Min(m, (int)(t * m))].z);
                    var bp = new List<Vector3> { o, o + dir * L * 0.5f + Vector3.down * 0.03f, o + dir * L + Vector3.down * 0.07f * (1f - t) }; var br = new List<float> { 0.006f, 0.0045f, 0.003f };
                    Geo.Tube(mb, 0, bp, br, 3, 1, 0.5f); SegsOf(segs, bp, br);
                    Vector3 d = (bp[2] - bp[0]).normalized; Vector3 side = Vector3.Cross(d, Vector3.up); if (side.sqrMagnitude < 1e-4f) side = Vector3.right; side.Normalize(); Vector3 up2 = Vector3.Cross(d, side).normalized; Vector3 c = (bp[0] + bp[2]) * 0.5f;
                    Vector3 outN = (new Vector3(c.x, 0, c.z).normalized * 0.6f + Vector3.up * 0.5f).normalized; float wide = Mathf.Clamp(L * 1.0f, 0.3f, 0.8f);
                    Geo.Card(mb, 1, c, d, side, L * 1.25f + 0.1f, wide, outN);
                    Geo.Card(mb, 1, c + up2 * 0.02f, d, (side + up2 * 1.1f).normalized, L * 1.1f, wide * 0.85f, outN);
                }
                h += Geo.R(rng, 0.2f, 0.32f);
            }
            for (int q = 0; q < 3; q++) { float ang = q * 2.094f; Geo.Card(mb, 1, pts[m] - Vector3.up * 0.2f, Vector3.up, new Vector3(Mathf.Cos(ang), 0, Mathf.Sin(ang)), 0.9f, 0.5f, Vector3.up); }
            return mb;
        }
    }
}
