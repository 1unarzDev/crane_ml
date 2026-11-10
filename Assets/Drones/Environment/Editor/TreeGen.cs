using System;
using System.Collections.Generic;
using UnityEngine;

namespace Crane.Drones.Environment.Editor
{
    public struct BranchSeg { public Vector3 a, b; public float r; public int order; }

    public class Species
    {
        public string name; public bool conifer, dead; public int bark;     // bark: index into bark material list
        public float crownBase;   // fraction of height where live limbs start (conifers) / fork height (broadleaf)
        public float limbLen;     // max limb length as fraction of H
        public Color leafTint = Color.white;
    }

    public class TreeResult
    {
        public Mesh visual, collision; public List<BranchSeg> segs = new List<BranchSeg>();
        public float height, trunkR; public Species species; public int sizeClass, variant;
        public float lowestLiveLimb;
    }

    /// <summary>Seeded procedural tree: tapered trunk with root flare + buttress roots, whorled limbs, recursive thin branches,
    /// dead-twig whorls through the 0.8-5 m flight band, alpha-card foliage.</summary>
    public class TreeGen
    {
        readonly System.Random rng; readonly Species sp; readonly ForestSpec spec;
        MB mb; TreeResult res; float H, R0; int branchBudget;
        Vector3 crownCenter;

        public static TreeResult Build(Species sp, ForestSpec spec, int sizeClass, int variant, int seed)
        {
            var g = new TreeGen(sp, spec, seed);
            return g.Make(sizeClass, variant);
        }
        TreeGen(Species s, ForestSpec spec, int seed) { sp = s; this.spec = spec; rng = new System.Random(seed); }

        static readonly float[] ClassH = { 9f, 16f, 24f };
        static readonly float[] ClassR = { 0.055f, 0.14f, 0.27f };
        public static float NominalH(int c) { return ClassH[c]; }
        public static float NominalR(int c) { return ClassR[c]; }

        float F(float a, float b) { return Geo.R(rng, a, b); }

        TreeResult Make(int sizeClass, int variant)
        {
            res = new TreeResult { species = sp, sizeClass = sizeClass, variant = variant };
            H = ClassH[sizeClass] * F(0.93f, 1.07f); R0 = ClassR[sizeClass] * F(0.93f, 1.07f);
            res.height = H; res.trunkR = R0; branchBudget = sp.conifer ? 1900 : 1500;
            mb = new MB(2); // 0 bark, 1 foliage
            crownCenter = new Vector3(0, H * (0.45f + sp.crownBase * 0.5f), 0);

            // trunk spine: dense samples near the base so the smooth buttress flare is resolved; first ring sits below ground
            var hs = new List<float> { -0.25f, 0f, 0.12f, 0.28f, 0.5f, 0.8f, 1.15f };
            for (float hh = 1.15f + 0.45f; hh < H - 0.2f; hh += 0.45f) hs.Add(hh); hs.Add(H);
            int n = hs.Count - 1; var pts = new List<Vector3>(); var rad = new List<float>();
            float lean = F(0.0f, sp.conifer ? 0.012f : 0.03f); Vector3 leanDir = new Vector3(Mathf.Cos(F(0, 6.28f)), 0, Mathf.Sin(F(0, 6.28f)));
            float phase = F(0, 100f);
            float flareK = sp.conifer ? (sp.name == "Pine" ? 0.35f : 0.5f) : 0.8f; float flareH = sp.conifer ? 0.45f : 0.6f;
            for (int i = 0; i <= n; i++)
            {
                float h = Mathf.Max(0f, hs[i]); float t = h / H;
                Vector3 p = new Vector3(0, hs[i], 0) + leanDir * (lean * h * h / 4f) + new Vector3(Mathf.PerlinNoise(phase, h * 0.3f) - 0.5f, 0, Mathf.PerlinNoise(h * 0.3f, phase + 9) - 0.5f) * (sp.conifer ? 0.18f : 0.5f) * Mathf.Clamp01(t * 3f);
                pts.Add(p);
                float taper = Mathf.Pow(1f - t, sp.conifer ? 0.85f : 0.65f);
                float fl = 1f + flareK * Mathf.Exp(-h / flareH) * (0.5f + 0.5f * Mathf.Exp(-h / (flareH * 3f)));
                rad.Add(Mathf.Max(0.012f, R0 * (0.08f + 0.92f * taper)) * fl);
            }
            int sides = sizeClass == 0 ? 8 : 12;
            float tile = 1.4f; float uT = Mathf.Max(1, Mathf.Round(Mathf.PI * 2 * R0 / tile));
            Geo.Tube(mb, 0, pts, rad, sides, uT, tile, 1f, 1f, null, false, true, F(0, 1));
            AddSegs(pts, rad, 0);

            Func<float, Vector3> trunkAt = h => { int i = 0; while (i < n - 1 && hs[i + 1] < h) i++; float tt = Mathf.Clamp01((h - hs[i]) / Mathf.Max(1e-4f, hs[i + 1] - hs[i])); return Vector3.Lerp(pts[i], pts[i + 1], tt); };
            Func<float, float> trunkR = h => { int i = 0; while (i < n - 1 && hs[i + 1] < h) i++; float tt = Mathf.Clamp01((h - hs[i]) / Mathf.Max(1e-4f, hs[i + 1] - hs[i])); return Mathf.Lerp(rad[i], rad[i + 1], tt); };

            DeadTwigBand(trunkAt, trunkR);
            if (!sp.dead) { if (sp.conifer) ConiferLimbs(trunkAt, trunkR); else BroadleafCrown(trunkAt, trunkR, pts, rad); }
            else SnagLimbs(trunkAt, trunkR);

            res.visual = mb.ToMesh(sp.name + "_c" + sizeClass + "_v" + variant, true);
            res.collision = mb.ToCollisionMesh(sp.name + "_c" + sizeClass + "_v" + variant + "_col", 0);
            return res;
        }

        void AddSegs(List<Vector3> pts, List<float> rad, int order)
        {
            for (int i = 0; i < pts.Count - 1; i++) res.segs.Add(new BranchSeg { a = pts[i], b = pts[i + 1], r = (rad[i] + rad[i + 1]) * 0.5f, order = order });
        }

        // ---- thin dead-twig whorls: the flight-band obstacle population ----
        void DeadTwigBand(Func<float, Vector3> trunkAt, Func<float, float> trunkR)
        {
            float top = sp.conifer ? Mathf.Min(H * 0.75f, Mathf.Max(5.5f, H * sp.crownBase + 1.5f)) : Mathf.Min(H * 0.6f, 6.5f);
            float spacing = 0.5f / Mathf.Max(0.15f, spec.lowBranchDensity);
            float h = 0.8f + F(0, spacing); float live = Mathf.Max(spec.branchHeightMin, H * (sp.conifer ? sp.crownBase * 0.55f : 0.3f));
            res.lowestLiveLimb = live;
            float az0 = F(0, 6.28f);
            while (h < top && branchBudget > 0)
            {
                int cnt = sp.conifer ? rng.Next(3, 6) : rng.Next(1, 3);
                if (!sp.conifer && rng.NextDouble() < 0.45) { h += spacing; continue; }
                for (int k = 0; k < cnt; k++)
                {
                    float az = az0 + (k + F(-0.25f, 0.25f)) / cnt * 6.283f; az0 += 0.9f;
                    Vector3 dir = new Vector3(Mathf.Cos(az), F(-0.12f, 0.3f), Mathf.Sin(az)).normalized;
                    float len = F(0.3f, spec.lowTwigMaxLength) * (sp.conifer ? Mathf.Lerp(0.65f, 1f, Mathf.InverseLerp(0.8f, top, h)) : 0.8f);
                    float r0 = F(0.0042f, 0.0105f);
                    Vector3 o = trunkAt(h) + dir * trunkR(h) * 0.8f;
                    Twig(o, dir, len, r0, 2, 3, true);
                }
                // an occasional thicker broken stub
                if (rng.NextDouble() < 0.22 * spec.lowBranchDensity)
                {
                    float az = F(0, 6.28f); Vector3 dir = new Vector3(Mathf.Cos(az), F(0, 0.4f), Mathf.Sin(az)).normalized;
                    Twig(trunkAt(h) + dir * trunkR(h) * 0.8f, dir, F(0.2f, 0.6f), F(0.014f, 0.03f), 1, 2, true);
                }
                h += spacing * F(0.7f, 1.3f);
            }
        }

        /// <summary>Wiry dead/live twig with side twiglets; optional foliage tuft at tip when live.</summary>
        void Twig(Vector3 o, Vector3 dir, float len, float r0, int order, int depth, bool dead)
        {
            if (branchBudget-- <= 0) return;
            int m = Mathf.Max(3, (int)(len / 0.28f) + 1); var pts = new List<Vector3>(); var rad = new List<float>(); Vector3 p = o, d = dir;
            float step = len / (m - 1);
            for (int i = 0; i < m; i++)
            {
                pts.Add(p); rad.Add(Mathf.Max(0.0022f, Mathf.Lerp(r0, r0 * 0.28f, (float)i / (m - 1))));
                d = (d + Geo.RandUnit(rng) * 0.32f + Vector3.down * (dead ? 0.03f : 0.06f)).normalized; p += d * step;
            }
            int sides = r0 > 0.012f ? 6 : (r0 > 0.006f ? 4 : 3);
            Geo.Tube(mb, 0, pts, rad, sides, 1, 0.5f);
            AddSegs(pts, rad, order);
            if (depth > 0)
            {
                int kids = (int)(len * (dead ? 2.4f : 2.0f) * F(0.6f, 1.3f));
                for (int k = 0; k < kids; k++)
                {
                    float t = F(0.25f, 0.9f); int i = Mathf.Min(m - 2, (int)(t * (m - 1))); Vector3 bp = Vector3.Lerp(pts[i], pts[i + 1], t * (m - 1) - i);
                    Vector3 side = Vector3.Cross((pts[i + 1] - pts[i]).normalized, Geo.RandUnit(rng)).normalized;
                    Vector3 cd = ((pts[i + 1] - pts[i]).normalized * F(0.4f, 0.9f) + side * F(0.6f, 1.0f)).normalized;
                    Twig(bp, cd, len * F(0.18f, 0.42f) * (1.15f - t), Mathf.Max(0.0025f, rad[i] * 0.55f), order + 1, depth - 1, dead);
                }
            }
            if (!dead && depth == 0) Foliage(pts[m - 1], (pts[m - 1] - pts[Mathf.Max(0, m - 2)]).normalized, 1f);
        }

        // ---- conifer: whorls of drooping/ascending limbs with needle sprays ----
        void ConiferLimbs(Func<float, Vector3> trunkAt, Func<float, float> trunkR)
        {
            float start = Mathf.Max(spec.branchHeightMin + 0.4f, H * sp.crownBase); float h = start;
            float maxLen = H * sp.limbLen;
            while (h < H * 0.97f && branchBudget > 0)
            {
                float t = (h - start) / (H * 0.97f - start);
                int cnt = sp.name == "Pine" ? rng.Next(4, 7) : rng.Next(6, 10);
                float az0 = F(0, 6.28f);
                for (int k = 0; k < cnt; k++)
                {
                    float az = az0 + (k + F(-0.2f, 0.2f)) / cnt * 6.283f;
                    float len = maxLen * Mathf.Pow(1f - t, 0.85f) * F(0.7f, 1.1f) + 0.3f;
                    float elev = Mathf.Lerp(-0.18f, 0.5f, t) + F(-0.1f, 0.1f);
                    Vector3 dir = new Vector3(Mathf.Cos(az), elev, Mathf.Sin(az)).normalized;
                    Limb(trunkAt(h), dir, len, Mathf.Max(0.008f, trunkR(h) * 0.26f), 2, sp.conifer);
                }
                h += F(0.3f, 0.52f) * (sp.name == "Pine" ? 2.0f : 1f);
            }
            // leader needles
            for (int q = 0; q < 3; q++) { float ang = q * 2.094f; Vector3 side = new Vector3(Mathf.Cos(ang), 0, Mathf.Sin(ang)); Geo.Card(mb, 1, trunkAt(H) - Vector3.up * 0.35f, Vector3.up, side, 1.5f, 0.9f, Vector3.up); }
        }

        void Limb(Vector3 o, Vector3 dir, float len, float r0, int levelsLeft, bool needles)
        {
            if (branchBudget-- <= 0) return;
            int m = Mathf.Max(3, (int)(len / 0.4f) + 1); var pts = new List<Vector3>(); var rad = new List<float>(); Vector3 p = o, d = dir; float step = len / (m - 1);
            for (int i = 0; i < m; i++)
            {
                float t = (float)i / (m - 1);
                pts.Add(p); rad.Add(Mathf.Max(0.004f, r0 * (1f - 0.9f * t)));
                d = (d + Vector3.down * 0.045f * (1 + t) + Vector3.up * (needles ? 0.04f * t * t * 3f : 0f) + Geo.RandUnit(rng) * 0.1f).normalized; p += d * step;
            }
            Geo.Tube(mb, 0, pts, rad, r0 > 0.02f ? 7 : 5, 1, 0.7f);
            AddSegs(pts, rad, 3 - levelsLeft);
            if (levelsLeft > 0)
            {
                int kids = (int)(len * (needles ? 4.2f : 2.2f));
                for (int k = 0; k < kids && branchBudget > 0; k++)
                {
                    float t = F(0.2f, 0.95f); int i = Mathf.Min(m - 2, (int)(t * (m - 1))); Vector3 bp = Vector3.Lerp(pts[i], pts[i + 1], t * (m - 1) - i);
                    Vector3 fwd = (pts[i + 1] - pts[i]).normalized; float sgn = (k % 2 == 0) ? 1 : -1;
                    Vector3 side = Vector3.Cross(fwd, Vector3.up).normalized * sgn;
                    Vector3 cd = (fwd * F(0.45f, 0.8f) + side * F(0.7f, 1.0f) + Vector3.up * F(-0.1f, 0.25f)).normalized;
                    Limb(bp, cd, len * F(0.22f, 0.5f) * (1.2f - t), Mathf.Max(0.004f, rad[i] * 0.5f), levelsLeft - 1, needles);
                }
            }
            else if (needles) { NeedleSpray(pts, len); for (int i = 1; i < m - 1; i += 2) NeedleSpray(new List<Vector3> { pts[i], pts[i + 1] }, len * 0.6f); }
            else Foliage(pts[m - 1], (pts[m - 1] - pts[m - 2]).normalized, 1f);
        }

        void NeedleSpray(List<Vector3> pts, float len)
        {
            float dens = 1f - 0.2f * spec.canopyOpenness;
            if (rng.NextDouble() > dens) return;
            Vector3 a = pts[0], b = pts[pts.Count - 1]; Vector3 d = (b - a); float L = d.magnitude + 0.3f; d.Normalize();
            Vector3 side = Vector3.Cross(d, Vector3.up); if (side.sqrMagnitude < 1e-4f) side = Vector3.right; side.Normalize();
            Vector3 outN = (((a + b) * 0.5f - crownCenter).normalized * 0.6f + Vector3.up * 0.5f).normalized; Vector3 c = (a + b) * 0.5f;
            Vector3 up2 = Vector3.Cross(d, side).normalized; float wide = Mathf.Clamp(L * 0.9f, 0.35f, 0.8f) * F(0.85f, 1.15f);
            // flat spray plus two tilted sprays so the whorl reads as a dense bough from every angle
            Geo.Card(mb, 1, c, d, side, L * 1.15f, wide, outN);
            Geo.Card(mb, 1, c + up2 * 0.03f, d, (side + up2 * 1.1f).normalized, L, wide * 0.85f, outN);
            Geo.Card(mb, 1, c - up2 * 0.03f, d, (side - up2 * 1.1f).normalized, L * 0.95f, wide * 0.85f, outN);
        }

        // ---- broadleaf: forked trunk, spreading limbs, recursive branches, leaf-card clusters ----
        void BroadleafCrown(Func<float, Vector3> trunkAt, Func<float, float> trunkR, List<Vector3> pts, List<float> rad)
        {
            float fork = H * sp.crownBase; int limbs = rng.Next(4, 7) + (H > 15 ? 1 : 0);
            float maxLen = H * sp.limbLen;
            for (int k = 0; k < limbs && branchBudget > 0; k++)
            {
                float h = Mathf.Lerp(Mathf.Max(spec.branchHeightMin + 0.8f, fork), H * 0.82f, Mathf.Pow((k + F(0, 1f)) / limbs, 1.3f));
                float az = (k * 2.399f) + F(-0.3f, 0.3f); float up = Mathf.Lerp(0.35f, 1.2f, (h - fork) / (H - fork + 0.01f));
                Vector3 dir = new Vector3(Mathf.Cos(az), up, Mathf.Sin(az)).normalized;
                Broad(trunkAt(h), dir, maxLen * F(0.7f, 1.1f) * (1.1f - 0.5f * h / H), Mathf.Max(0.012f, trunkR(h) * F(0.34f, 0.5f)), 3);
            }
            // crown-top branches off the leader
            for (int k = 0; k < 3; k++) Broad(trunkAt(H * F(0.82f, 0.97f)), (Vector3.up * 1.2f + Geo.RandUnit(rng)).normalized, maxLen * 0.45f, Mathf.Max(0.01f, trunkR(H * 0.85f) * 0.5f), 2);
        }

        void Broad(Vector3 o, Vector3 dir, float len, float r0, int levelsLeft)
        {
            if (branchBudget-- <= 0) return;
            int m = Mathf.Max(4, (int)(len / 0.5f) + 1); var pts = new List<Vector3>(); var rad = new List<float>(); Vector3 p = o, d = dir; float step = len / (m - 1);
            for (int i = 0; i < m; i++)
            {
                float t = (float)i / (m - 1); pts.Add(p); rad.Add(Mathf.Max(0.0035f, r0 * (1f - 0.88f * Mathf.Pow(t, 0.9f))));
                d = (d + Geo.RandUnit(rng) * 0.2f + Vector3.up * 0.03f - Vector3.up * 0.05f * t).normalized; p += d * step;
            }
            Geo.Tube(mb, 0, pts, rad, r0 > 0.05f ? 9 : (r0 > 0.015f ? 6 : 4), 1, 1.0f);
            AddSegs(pts, rad, 4 - levelsLeft);
            if (levelsLeft > 0)
            {
                int kids = Mathf.Max(2, (int)(len * (levelsLeft == 3 ? 1.5f : 2.6f)));
                for (int k = 0; k < kids && branchBudget > 0; k++)
                {
                    float t = F(0.18f, 0.97f); int i = Mathf.Min(m - 2, (int)(t * (m - 1))); Vector3 bp = Vector3.Lerp(pts[i], pts[i + 1], t * (m - 1) - i);
                    Vector3 fwd = (pts[i + 1] - pts[i]).normalized; Vector3 cd = (fwd * F(0.5f, 0.9f) + Geo.RandUnit(rng) * 0.7f).normalized;
                    if (cd.y < -0.1f) cd.y = -cd.y * 0.3f;
                    Broad(bp, cd.normalized, len * F(0.25f, 0.55f) * (1.1f - 0.6f * t), Mathf.Max(0.004f, rad[i] * 0.55f), levelsLeft - 1);
                }
                if (levelsLeft == 1) for (int i = 2; i < m; i++) if (rng.NextDouble() < 0.6) Foliage(pts[i], Geo.RandUnit(rng), 1.1f);
            }
            else
            {
                Foliage(pts[m - 1], (pts[m - 1] - pts[m - 2]).normalized, 1.25f);
                for (int i = 1; i < m - 1; i++) if (rng.NextDouble() < 0.85) Foliage(pts[i], Geo.RandUnit(rng), 1.0f);
            }
        }

        void SnagLimbs(Func<float, Vector3> trunkAt, Func<float, float> trunkR)
        {
            int n = rng.Next(5, 10);
            for (int k = 0; k < n && branchBudget > 0; k++)
            {
                float h = F(2.5f, H * 0.8f); float az = F(0, 6.28f);
                Vector3 dir = new Vector3(Mathf.Cos(az), F(-0.1f, 0.6f), Mathf.Sin(az)).normalized;
                Twig(trunkAt(h) + dir * trunkR(h), dir, F(0.6f, 2.8f), Mathf.Max(0.01f, trunkR(h) * 0.22f), 1, 3, true);
            }
        }

        /// <summary>Leaf cluster: several cards of different sizes in a small volume, each sampling a random window of the leaf atlas.</summary>
        void Foliage(Vector3 pos, Vector3 dir, float scale)
        {
            if (sp.dead) return;
            if (rng.NextDouble() < spec.canopyOpenness * 0.25f) return; // open canopy thins leaf cover
            Vector3 outN = (pos - crownCenter); if (outN.sqrMagnitude < 0.01f) outN = Vector3.up; outN = (outN.normalized * 0.65f + Vector3.up * 0.45f).normalized;
            if (sp.conifer)
            {
                Vector3 side = Vector3.Cross(dir, Vector3.up); if (side.sqrMagnitude < 1e-3f) side = Vector3.right; side.Normalize();
                Geo.Card(mb, 1, pos + dir * 0.2f * scale, dir, side, 0.7f * scale, 0.6f * scale, outN);
                return;
            }
            int count = 4 + rng.Next(0, 3);
            for (int k = 0; k < count; k++)
            {
                float s = F(0.32f, 0.85f) * scale * (k == 0 ? 1.15f : 1f); float win = 0.65f, u0 = F(0f, 0.35f);
                Vector3 a = (Geo.RandUnit(rng) * 0.6f + Vector3.up * 0.4f).normalized; Vector3 r = Vector3.Cross(a, Vector3.up); if (r.sqrMagnitude < 0.01f) r = Vector3.right; r.Normalize();
                Vector3 u = Vector3.Cross(r, a).normalized;
                Vector3 off = Geo.RandUnit(rng) * (0.1f + 0.14f * scale) * (k == 0 ? 0f : 1f);
                Geo.Card(mb, 1, pos + off, r, u, s * win, s, outN, u0, u0 + win);
            }
        }
    }
}
