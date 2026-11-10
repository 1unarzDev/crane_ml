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
        public static MB Fern(System.Random rng, float size)
        {
            var mb = new MB(1); int fronds = rng.Next(7, 12);
            for (int f = 0; f < fronds; f++)
            {
                float az = (f + Geo.R(rng, -0.3f, 0.3f)) / fronds * 6.283f; Vector3 dir = new Vector3(Mathf.Cos(az), 0, Mathf.Sin(az));
                float len = size * Geo.R(rng, 0.7f, 1.2f); float lift = Geo.R(rng, 0.25f, 0.55f);
                int seg = 4; int i0 = mb.Count;
                for (int s = 0; s <= seg; s++)
                {
                    float t = (float)s / seg; Vector3 c = dir * (len * t) + Vector3.up * (len * (lift * Mathf.Sin(t * 1.9f) - 0.25f * t * t));
                    Vector3 side = Vector3.Cross(Vector3.up, dir).normalized * (len * 0.2f * Mathf.Sin(Mathf.Min(1, t * 2.4f) * 1.57f) * (1f - 0.55f * t) + 0.01f);
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
    }
}
