using System;
using UnityEngine;

namespace Crane.Drones.Environment.Editor
{
    /// <summary>CPU-painted alpha-cut foliage atlases and a fabric weave. Deterministic from seed.</summary>
    public static class TexGen
    {
        static Color32[] NewBuf(int S, Color32 fill) { var b = new Color32[S * S]; for (int i = 0; i < b.Length; i++) b[i] = fill; return b; }

        static void Leaf(Color32[] buf, int S, float cx, float cy, float ang, float len, float wid, Color baseC, Color tipC, System.Random rng, float curl = 0f)
        {
            float ca = Mathf.Cos(ang), sa = Mathf.Sin(ang);
            int r = (int)(len + wid) + 2; int x0 = Mathf.Max(0, (int)cx - r), x1 = Mathf.Min(S - 1, (int)cx + r), y0 = Mathf.Max(0, (int)cy - r), y1 = Mathf.Min(S - 1, (int)cy + r);
            float jit = (float)rng.NextDouble();
            for (int y = y0; y <= y1; y++)
                for (int x = x0; x <= x1; x++)
                {
                    float dx = x - cx, dy = y - cy; float u = (dx * ca + dy * sa) / len; float v = (-dx * sa + dy * ca) / wid;
                    v -= curl * u * u;
                    if (u < 0 || u > 1) continue;
                    float half = Mathf.Pow(Mathf.Sin(u * Mathf.PI), 0.75f) * (1f - 0.3f * u);
                    if (Mathf.Abs(v) > half) continue;
                    float vein = Mathf.Abs(v) < 0.045f ? 0.72f : (Mathf.Abs(Mathf.Sin((u * 7f + Mathf.Abs(v) * 2.3f) * 3.14f)) < 0.07f ? 0.88f : 1f);
                    float edge = Mathf.Clamp01((half - Mathf.Abs(v)) * wid / 1.5f);
                    Color c = Color.Lerp(baseC, tipC, u) * vein * (0.9f + 0.2f * jit); c.a = 1;
                    buf[y * S + x] = new Color32((byte)(Mathf.Clamp01(c.r) * 255), (byte)(Mathf.Clamp01(c.g) * 255), (byte)(Mathf.Clamp01(c.b) * 255), (byte)(edge * 255));
                }
        }

        static void Line(Color32[] buf, int S, Vector2 a, Vector2 b, float w, Color c)
        {
            int x0 = Mathf.Max(0, (int)(Mathf.Min(a.x, b.x) - w - 1)), x1 = Mathf.Min(S - 1, (int)(Mathf.Max(a.x, b.x) + w + 1)), y0 = Mathf.Max(0, (int)(Mathf.Min(a.y, b.y) - w - 1)), y1 = Mathf.Min(S - 1, (int)(Mathf.Max(a.y, b.y) + w + 1));
            Vector2 ab = b - a; float l2 = Mathf.Max(1e-4f, ab.sqrMagnitude);
            for (int y = y0; y <= y1; y++) for (int x = x0; x <= x1; x++)
                {
                    float t = Mathf.Clamp01(Vector2.Dot(new Vector2(x, y) - a, ab) / l2); float d = (a + ab * t - new Vector2(x, y)).magnitude;
                    if (d > w) continue; float al = Mathf.Clamp01(w - d + 0.3f);
                    if (al * 255 < buf[y * S + x].a) continue;
                    buf[y * S + x] = new Color32((byte)(c.r * 255), (byte)(c.g * 255), (byte)(c.b * 255), (byte)(al * 255));
                }
        }

        static void Bleed(Color32[] buf, Color32 rgb) { for (int i = 0; i < buf.Length; i++) if (buf[i].a == 0) buf[i] = new Color32(rgb.r, rgb.g, rgb.b, 0); }

        public static Texture2D Broadleaf(int seed, int S = 512)
        {
            var rng = new System.Random(seed); var buf = NewBuf(S, new Color32(60, 90, 35, 0));
            Color b = new Color(0.16f, 0.30f, 0.08f), t = new Color(0.30f, 0.46f, 0.12f);
            for (int i = 0; i < 26; i++)
            {
                float ang = (float)(rng.NextDouble() * 6.28); float len = (float)(70 + rng.NextDouble() * 55); float wid = len * (0.32f + (float)rng.NextDouble() * 0.12f);
                float cx = (float)(60 + rng.NextDouble() * (S - 120)), cy = (float)(60 + rng.NextDouble() * (S - 120));
                Color jb = Color.Lerp(b, new Color(0.30f, 0.26f, 0.06f), (float)rng.NextDouble() * 0.35f); // some yellowing
                Leaf(buf, S, cx, cy, ang, len, wid, jb * 0.9f, Color.Lerp(t, new Color(0.45f, 0.40f, 0.1f), (float)rng.NextDouble() * 0.4f), rng, (float)(rng.NextDouble() - 0.5) * 0.3f);
            }
            return Finish(buf, S, true);
        }

        public static Texture2D Needles(int seed, int S = 512)
        {
            var rng = new System.Random(seed); var buf = NewBuf(S, new Color32(30, 60, 30, 0));
            Vector2 a = new Vector2(10, S * 0.5f), b = new Vector2(S - 10, S * 0.5f + 8);
            Line(buf, S, a, b, 2.2f, new Color(0.25f, 0.16f, 0.08f));
            for (int i = 0; i < 100; i++)
            {
                float t = i / 100f; Vector2 p = Vector2.Lerp(a, b, t * 0.97f + 0.01f);
                for (int sgn = -1; sgn <= 1; sgn += 2)
                {
                    float ang = (0.9f + (float)rng.NextDouble() * 0.35f) * sgn; float L = (S * 0.34f) * (1f - 0.5f * t) * (0.8f + (float)rng.NextDouble() * 0.4f) + 20;
                    Vector2 dir = new Vector2(Mathf.Cos(ang) * 0.55f + 0.35f, Mathf.Sin(ang)).normalized;
                    Color c = Color.Lerp(new Color(0.08f, 0.2f, 0.09f), new Color(0.17f, 0.34f, 0.14f), (float)rng.NextDouble());
                    Line(buf, S, p, p + dir * L, 1.5f, c);
                }
            }
            return Finish(buf, S, true);
        }

        public static Texture2D FernFrond(int seed, int S = 512, int variant = 0)
        {
            var rng = new System.Random(seed); var buf = NewBuf(S, new Color32(50, 85, 40, 0));
            // variant: 0 sword fern, 1 bracken (broad, light), 2 lady fern (many thin pairs), 3 dark short sword fern
            int pairs = variant == 1 ? 11 : (variant == 2 ? 30 : (variant == 3 ? 18 : 22));
            float lenK = variant == 1 ? 0.50f : (variant == 2 ? 0.34f : 0.46f); float widK = variant == 1 ? 0.22f : (variant == 2 ? 0.09f : 0.12f);
            Color cDark = variant == 1 ? new Color(0.16f, 0.30f, 0.08f) : (variant == 3 ? new Color(0.05f, 0.15f, 0.06f) : new Color(0.09f, 0.22f, 0.07f));
            Color cLight = variant == 1 ? new Color(0.36f, 0.50f, 0.14f) : (variant == 2 ? new Color(0.28f, 0.48f, 0.16f) : new Color(0.22f, 0.42f, 0.13f));
            // vertical frond: rachis from bottom centre to top; pinnae pairs tapering. (u across, v along)
            float cx = S * 0.5f;
            Line(buf, S, new Vector2(cx, 4), new Vector2(cx + 4, S - 6), 2.4f, new Color(0.18f, 0.28f, 0.1f));
            for (int i = 0; i < pairs; i++)
            {
                float t = (i + 0.5f) / pairs; float y = 12 + t * (S - 40); float L = S * lenK * Mathf.Sin(Mathf.Min(1, t * 1.15f + 0.1f) * 1.5f) * (1f - 0.5f * t);
                for (int sgn = -1; sgn <= 1; sgn += 2)
                {
                    float ang = sgn > 0 ? 0.35f : Mathf.PI - 0.35f; // leaflets angle forward/up
                    Leaf(buf, S, cx + 2, y, ang + (sgn > 0 ? 1 : -1) * 0.0f + (float)(rng.NextDouble() - 0.5) * 0.12f + (sgn > 0 ? 0.6f : -0.6f) * 0f, L, Mathf.Max(4f, L * widK),
                        cDark, cLight, rng);
                }
            }
            return Finish(buf, S, true);
        }


        // ---------- extra foliage / ground textures ----------
        public static Texture2D GrassBlades(int seed, int S = 512)
        {
            var rng = new System.Random(seed); var buf = NewBuf(S, new Color32(70, 100, 40, 0));
            for (int i = 0; i < 90; i++)
            {
                float x = 12 + (float)rng.NextDouble() * (S - 24), lean = ((float)rng.NextDouble() - 0.5f) * 0.55f; float h = S * (0.45f + (float)rng.NextDouble() * 0.5f);
                Color c0 = Color.Lerp(new Color(0.12f, 0.22f, 0.05f), new Color(0.22f, 0.33f, 0.07f), (float)rng.NextDouble()); Color c1 = Color.Lerp(new Color(0.34f, 0.46f, 0.12f), new Color(0.5f, 0.52f, 0.16f), (float)rng.NextDouble());
                int seg = 10; Vector2 prev = new Vector2(x, 2);
                for (int k = 1; k <= seg; k++)
                {
                    float t = (float)k / seg; Vector2 p = new Vector2(x + lean * h * t * t * 2.0f, 2 + h * t);
                    float w = 4.2f * (1f - t) + 0.6f; Line(buf, S, prev, p, w, Color.Lerp(c0, c1, t)); prev = p;
                }
            }
            return Finish(buf, S, true);
        }

        /// <summary>Normal map (tangent space) from the alpha*luma height of a foliage atlas.</summary>
        public static Texture2D NormalFromAlpha(Texture2D src, float strength)
        {
            int S = src.width; var px = src.GetPixels32(); var h = new float[S * S];
            for (int i = 0; i < h.Length; i++) h[i] = px[i].a / 255f * (0.6f + 0.4f * ((px[i].r + px[i].g + px[i].b) / 765f));
            // 2 box blurs to give leaf bodies a soft dome
            for (int pass = 0; pass < 2; pass++)
            {
                var o = new float[h.Length];
                for (int y = 0; y < S; y++) for (int x = 0; x < S; x++)
                    {
                        float a = 0; int n = 0; for (int dy = -2; dy <= 2; dy++) for (int dx = -2; dx <= 2; dx++) { int xx = x + dx, yy = y + dy; if (xx < 0 || yy < 0 || xx >= S || yy >= S) continue; a += h[yy * S + xx]; n++; }
                        o[y * S + x] = a / n;
                    }
                h = o;
            }
            return HeightToNormal(h, S, strength, false);
        }

        /// <summary>Thickness (translucency) map: grey, high inside leaf bodies, lower at edges and veins.</summary>
        public static Texture2D ThicknessFromAlpha(Texture2D src)
        {
            int S = src.width; var px = src.GetPixels32(); var buf = new Color32[S * S];
            for (int y = 0; y < S; y++) for (int x = 0; x < S; x++)
                {
                    int i = y * S + x; float a = px[i].a / 255f; float lum = (px[i].r + px[i].g + px[i].b) / 765f;
                    float v = Mathf.Clamp01(0.25f + 0.75f * a) * Mathf.Clamp01(0.55f + lum * 2.2f); byte g = (byte)(v * 255); buf[i] = new Color32(g, g, g, 255);
                }
            var t = new Texture2D(S, S, TextureFormat.RGBA32, true, true); t.SetPixels32(buf); t.Apply(); return t;
        }

        static Texture2D HeightToNormal(float[] h, int S, float k, bool wrap)
        {
            var buf = new Color32[S * S];
            Func<int, int> w = i => wrap ? ((i % S) + S) % S : Mathf.Clamp(i, 0, S - 1);
            for (int y = 0; y < S; y++) for (int x = 0; x < S; x++)
                {
                    float dx = (h[y * S + w(x + 1)] - h[y * S + w(x - 1)]) * k, dy = (h[w(y + 1) * S + x] - h[w(y - 1) * S + x]) * k;
                    var n = new Vector3(-dx, -dy, 1f).normalized;
                    buf[y * S + x] = new Color32((byte)((n.x * 0.5f + 0.5f) * 255), (byte)((n.y * 0.5f + 0.5f) * 255), (byte)((n.z * 0.5f + 0.5f) * 255), 255);
                }
            var t = new Texture2D(S, S, TextureFormat.RGBA32, true, true); t.SetPixels32(buf); t.Apply(); return t;
        }

        // periodic value noise
        static float Hash(int x, int y, int seed)
        {
            unchecked { uint h = (uint)(x * 374761393 + y * 668265263 + seed * 1442695041); h = (h ^ (h >> 13)) * 1274126177u; h ^= h >> 16; return (h & 0xffff) / 65535f; }
        }
        static float VNoise(float u, float v, int period, int seed)
        {
            float x = u * period, y = v * period; int x0 = Mathf.FloorToInt(x), y0 = Mathf.FloorToInt(y); float fx = x - x0, fy = y - y0;
            fx = fx * fx * (3 - 2 * fx); fy = fy * fy * (3 - 2 * fy);
            int xa = ((x0 % period) + period) % period, xb = (xa + 1) % period, ya = ((y0 % period) + period) % period, yb = (ya + 1) % period;
            return Mathf.Lerp(Mathf.Lerp(Hash(xa, ya, seed), Hash(xb, ya, seed), fx), Mathf.Lerp(Hash(xa, yb, seed), Hash(xb, yb, seed), fx), fy);
        }
        static float Fbm(float u, float v, int baseFreq, int oct, int seed)
        {
            float a = 0, amp = 0.5f, tot = 0; int f = baseFreq;
            for (int o = 0; o < oct; o++) { a += amp * VNoise(u, v, f, seed + o * 17); tot += amp; amp *= 0.5f; f *= 2; }
            return a / tot;
        }
        static void Splat(Color32[] buf, float[] hgt, int S, int x, int y, Color c, float hv)
        {
            x = ((x % S) + S) % S; y = ((y % S) + S) % S; buf[y * S + x] = new Color32((byte)(Mathf.Clamp01(c.r) * 255), (byte)(Mathf.Clamp01(c.g) * 255), (byte)(Mathf.Clamp01(c.b) * 255), 255); hgt[y * S + x] = hv;
        }
        static void WrapLine(Color32[] buf, float[] hgt, int S, Vector2 a, Vector2 b, Color c, float hv, int thick)
        {
            int n = (int)(Vector2.Distance(a, b) * 1.5f) + 1;
            for (int i = 0; i <= n; i++) { Vector2 p = Vector2.Lerp(a, b, (float)i / n); for (int t = 0; t < thick; t++) Splat(buf, hgt, S, (int)p.x + t, (int)p.y, c, hv); }
        }

        /// <summary>Tileable ground textures. kind: "moss" | "needles" | "grass" | "leaves2" (oak-coloured litter). Returns diffuse and normal.</summary>
        public static void GroundTile(string kind, int seed, out Texture2D diffuse, out Texture2D normal, int S = 512)
        {
            var rng = new System.Random(seed); var buf = new Color32[S * S]; var hgt = new float[S * S];
            for (int y = 0; y < S; y++) for (int x = 0; x < S; x++)
                {
                    float u = (float)x / S, v = (float)y / S; float big = Fbm(u, v, 4, 4, seed), fine = Fbm(u, v, 32, 3, seed + 99);
                    Color c; float hv;
                    if (kind == "moss")
                    {
                        float clump = Fbm(u, v, 14, 4, seed + 5); hv = clump * 0.8f + fine * 0.2f;
                        c = Color.Lerp(new Color(0.07f, 0.14f, 0.03f), new Color(0.28f, 0.42f, 0.10f), Mathf.Clamp01(hv * 1.4f - 0.1f)); c *= 0.85f + 0.3f * fine; c = Color.Lerp(c, new Color(0.30f, 0.30f, 0.12f), Mathf.Clamp01((big - 0.6f) * 2f) * 0.4f);
                    }
                    else if (kind == "canopyB" || kind == "canopyC")
                    {
                        float clump = Fbm(u, v, 9, 5, seed + 5); hv = clump; bool b = kind == "canopyB";
                        Color a0 = b ? new Color(0.07f, 0.15f, 0.04f) : new Color(0.035f, 0.085f, 0.055f), a1 = b ? new Color(0.30f, 0.40f, 0.10f) : new Color(0.12f, 0.21f, 0.12f);
                        c = Color.Lerp(a0, a1, Mathf.Pow(clump, 1.3f)) * (0.8f + 0.4f * fine);
                    }
                    else if (kind == "grass") { hv = fine * 0.5f; c = Color.Lerp(new Color(0.10f, 0.16f, 0.05f), new Color(0.20f, 0.28f, 0.08f), big) * (0.8f + 0.4f * fine); }
                    else if (kind == "needles") { hv = fine * 0.3f; c = Color.Lerp(new Color(0.20f, 0.12f, 0.06f), new Color(0.30f, 0.18f, 0.09f), big) * (0.8f + 0.3f * fine); }
                    else { hv = fine * 0.3f; c = Color.Lerp(new Color(0.24f, 0.13f, 0.05f), new Color(0.36f, 0.24f, 0.09f), big) * (0.75f + 0.4f * fine); }
                    buf[y * S + x] = new Color32((byte)(Mathf.Clamp01(c.r) * 255), (byte)(Mathf.Clamp01(c.g) * 255), (byte)(Mathf.Clamp01(c.b) * 255), 255); hgt[y * S + x] = hv;
                }
            if (kind == "needles")
            {
                for (int i = 0; i < 9000; i++)
                {
                    Vector2 p = new Vector2((float)rng.NextDouble() * S, (float)rng.NextDouble() * S); float ang = (float)rng.NextDouble() * 6.283f; float L = 9f + (float)rng.NextDouble() * 14f;
                    Color c = Color.Lerp(new Color(0.32f, 0.17f, 0.07f), new Color(0.62f, 0.40f, 0.17f), (float)rng.NextDouble()); WrapLine(buf, hgt, S, p, p + new Vector2(Mathf.Cos(ang), Mathf.Sin(ang)) * L, c, 0.6f + (float)rng.NextDouble() * 0.4f, 1);
                }
            }
            else if (kind == "grass")
            {
                for (int i = 0; i < 7000; i++)
                {
                    Vector2 p = new Vector2((float)rng.NextDouble() * S, (float)rng.NextDouble() * S); float ang = (float)rng.NextDouble() * 6.283f; float L = 10f + (float)rng.NextDouble() * 16f;
                    Color c = Color.Lerp(new Color(0.14f, 0.24f, 0.06f), new Color(0.46f, 0.52f, 0.16f), (float)rng.NextDouble()); WrapLine(buf, hgt, S, p, p + new Vector2(Mathf.Cos(ang), Mathf.Sin(ang)) * L, c, 0.5f + (float)rng.NextDouble() * 0.5f, 1);
                }
            }
            else if (kind == "leaves2")
            {
                for (int i = 0; i < 1800; i++)
                {
                    Vector2 p = new Vector2((float)rng.NextDouble() * S, (float)rng.NextDouble() * S); float ang = (float)rng.NextDouble() * 6.283f; float L = 14f + (float)rng.NextDouble() * 16f; Vector2 d = new Vector2(Mathf.Cos(ang), Mathf.Sin(ang)), nrm = new Vector2(-d.y, d.x);
                    Color c = Color.Lerp(new Color(0.30f, 0.14f, 0.05f), new Color(0.55f, 0.36f, 0.12f), (float)rng.NextDouble()); float hv = 0.4f + (float)rng.NextDouble() * 0.6f;
                    for (int k = -4; k <= 4; k++) { float wd = 1f - Mathf.Abs(k) / 5f; WrapLine(buf, hgt, S, p + nrm * k * 0.9f, p + d * L * wd + nrm * k * 0.9f, c, hv, 1); }
                }
            }
            diffuse = new Texture2D(S, S, TextureFormat.RGBA32, true, false); diffuse.SetPixels32(buf); diffuse.Apply();
            normal = HeightToNormal(hgt, S, kind == "moss" ? 6f : 4f, true);
        }

        public static Texture2D Weave(int seed, int S = 256)
        {
            var rng = new System.Random(seed); var buf = new Color32[S * S];
            for (int y = 0; y < S; y++) for (int x = 0; x < S; x++)
                {
                    float w = 0.5f + 0.25f * Mathf.Sin(x * Mathf.PI * 2f / 4f) * Mathf.Sin(y * Mathf.PI * 2f / 4f) + 0.12f * (Mathf.PerlinNoise(x * 0.07f, y * 0.07f) - 0.5f) + 0.08f * (float)rng.NextDouble();
                    byte g = (byte)(Mathf.Clamp01(0.55f + w * 0.5f) * 255); buf[y * S + x] = new Color32(g, g, g, 255);
                }
            var t = new Texture2D(S, S, TextureFormat.RGBA32, true, false); t.SetPixels32(buf); t.Apply(); return t;
        }

        static Texture2D Finish(Color32[] buf, int S, bool bleed)
        {
            var t = new Texture2D(S, S, TextureFormat.RGBA32, true, false); t.SetPixels32(buf); t.Apply(); return t;
        }
    }
}
