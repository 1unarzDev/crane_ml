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
            for (int i = 0; i < 70; i++)
            {
                float t = i / 70f; Vector2 p = Vector2.Lerp(a, b, t * 0.97f + 0.01f);
                for (int sgn = -1; sgn <= 1; sgn += 2)
                {
                    float ang = (0.9f + (float)rng.NextDouble() * 0.35f) * sgn; float L = (S * 0.3f) * (1f - 0.55f * t) * (0.8f + (float)rng.NextDouble() * 0.4f) + 20;
                    Vector2 dir = new Vector2(Mathf.Cos(ang) * 0.55f + 0.35f, Mathf.Sin(ang)).normalized;
                    Color c = Color.Lerp(new Color(0.08f, 0.2f, 0.09f), new Color(0.17f, 0.34f, 0.14f), (float)rng.NextDouble());
                    Line(buf, S, p, p + dir * L, 1.5f, c);
                }
            }
            return Finish(buf, S, true);
        }

        public static Texture2D FernFrond(int seed, int S = 512)
        {
            var rng = new System.Random(seed); var buf = NewBuf(S, new Color32(50, 85, 40, 0));
            // vertical frond: rachis from bottom centre to top; pinnae pairs tapering. (u across, v along)
            float cx = S * 0.5f;
            Line(buf, S, new Vector2(cx, 4), new Vector2(cx + 4, S - 6), 2.4f, new Color(0.18f, 0.28f, 0.1f));
            int pairs = 22;
            for (int i = 0; i < pairs; i++)
            {
                float t = (i + 0.5f) / pairs; float y = 12 + t * (S - 40); float L = S * 0.46f * Mathf.Sin(Mathf.Min(1, t * 1.15f + 0.1f) * 1.5f) * (1f - 0.5f * t);
                for (int sgn = -1; sgn <= 1; sgn += 2)
                {
                    float ang = sgn > 0 ? 0.35f : Mathf.PI - 0.35f; // leaflets angle forward/up
                    Leaf(buf, S, cx + 2, y, ang + (sgn > 0 ? 1 : -1) * 0.0f + (float)(rng.NextDouble() - 0.5) * 0.12f + (sgn > 0 ? 0.6f : -0.6f) * 0f, L, Mathf.Max(5f, L * 0.12f),
                        new Color(0.09f, 0.22f, 0.07f), new Color(0.22f, 0.42f, 0.13f), rng);
                }
            }
            return Finish(buf, S, true);
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
