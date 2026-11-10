using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using UnityEngine.Rendering;

namespace Crane.Theater {
    /// <summary>The data HUD, drawn into the captured frame after the camera: two meshes in pixel coordinates
    /// (shapes, and text from Unity's built-in dynamic font) and one command buffer. Layout per act type;
    /// sizes scale with the frame height (designed at 1080 p, legible at the 480 p gif).</summary>
    public sealed class HudRenderer : IDisposable {
        readonly Material mat;
        readonly Font font;
        readonly Mesh shapes = new Mesh { name = "hud shapes" }, text = new Mesh { name = "hud text" };
        readonly List<Vector3> sv = new List<Vector3>(), tv = new List<Vector3>();
        readonly List<Color> sc = new List<Color>(), tc = new List<Color>();
        readonly List<Vector2> tu = new List<Vector2>();
        readonly List<int> si = new List<int>(), ti = new List<int>();
        readonly CommandBuffer cmd = new CommandBuffer { name = "theater hud" };
        readonly int w, h;
        readonly float s;
        static readonly int[] Sizes = { 15, 18, 22, 26, 32, 44, 72 };
        const string Charset = " !\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~→–×·Δ•";
        static readonly Color Ink = new Color(0.94f, 0.96f, 1f, 1f), Dim = new Color(0.66f, 0.72f, 0.8f, 1f), Panel = new Color(0.02f, 0.04f, 0.07f, 0.55f);

        /// <summary>fleet: the study HUD (every panel); minimal: one panel with the title and a caption line
        /// (backend, the act's caption or the hero's run id, run time); none is handled by the caller (no Draw).</summary>
        public string Mode = "fleet";

        public HudRenderer(int width, int height) {
            w = width; h = height; s = height / 1080f;
            var shader = Resources.Load<Shader>("CraneTheaterHud");
            if (shader == null) throw new InvalidOperationException("theater: shader Resources/CraneTheaterHud missing");
            mat = new Material(shader) { hideFlags = HideFlags.DontSave };
            font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            foreach (int size in Sizes) {
                font.RequestCharactersInTexture(Charset, Px(size), FontStyle.Normal);
                font.RequestCharactersInTexture(Charset, Px(size), FontStyle.Bold);
            }
            shapes.MarkDynamic(); text.MarkDynamic();
            Font.textureRebuilt += f => { if (f == font) rebuilt = true; };
        }

        public void Dispose() { cmd.Release(); }

        int Px(int size) => Mathf.Max(8, Mathf.RoundToInt(size * s));

        // ------------------------------------------------------------------ primitives (pixels, origin bottom-left)

        void Quad(Vector2 a, Vector2 b, Vector2 c, Vector2 d, Color ca, Color cb, Color cc2, Color cd) {
            int k = sv.Count;
            sv.Add(a); sv.Add(b); sv.Add(c); sv.Add(d);
            sc.Add(ca.linear.WithA(ca.a)); sc.Add(cb.linear.WithA(cb.a)); sc.Add(cc2.linear.WithA(cc2.a)); sc.Add(cd.linear.WithA(cd.a));
            si.Add(k); si.Add(k + 1); si.Add(k + 2); si.Add(k); si.Add(k + 2); si.Add(k + 3);
        }

        void Rect(float x, float y, float rw, float rh, Color c) =>
            Quad(new Vector2(x, y), new Vector2(x + rw, y), new Vector2(x + rw, y + rh), new Vector2(x, y + rh), c, c, c, c);

        void Line(Vector2 a, Vector2 b, float width, Color c) {
            Vector2 d = b - a;
            if (d.sqrMagnitude < 1e-6f) return;
            Vector2 n = new Vector2(-d.y, d.x).normalized * width * 0.5f;
            Quad(a - n, b - n, b + n, a + n, c, c, c, c);
        }

        void Dot(Vector2 c, float r, Color col) {
            const int seg = 16;
            int k = sv.Count;
            sv.Add(c); sc.Add(col.linear.WithA(col.a));
            for (int i = 0; i <= seg; i++) {
                float a = 2 * Mathf.PI * i / seg;
                sv.Add(c + new Vector2(Mathf.Cos(a), Mathf.Sin(a)) * r); sc.Add(col.linear.WithA(col.a));
                if (i > 0) { si.Add(k); si.Add(k + i); si.Add(k + i + 1); }
            }
        }

        float Measure(string str, int size, FontStyle style) {
            float x = 0;
            foreach (char ch in str) if (font.GetCharacterInfo(ch, out var ci, Px(size), style)) x += ci.advance;
            return x;
        }

        /// <summary>Text with its baseline at y; align 0 left, 0.5 centre, 1 right. Returns the width.</summary>
        float Text(string str, float x, float y, int size, Color c, float align = 0, FontStyle style = FontStyle.Normal, bool shadow = true) {
            if (string.IsNullOrEmpty(str)) return 0;
            font.RequestCharactersInTexture(str, Px(size), style);
            float width = Measure(str, size, style);
            x = Mathf.Round(x - width * align); y = Mathf.Round(y);
            if (shadow) Glyphs(str, x + 1.5f * s, y - 1.5f * s, size, new Color(0, 0, 0, 0.6f * c.a), style);
            Glyphs(str, x, y, size, c, style);
            return width;
        }

        void Glyphs(string str, float x, float y, int size, Color c, FontStyle style) {
            Color lc = c.linear.WithA(c.a);
            foreach (char ch in str) {
                if (!font.GetCharacterInfo(ch, out var ci, Px(size), style)) continue;
                int k = tv.Count;
                tv.Add(new Vector2(x + ci.minX, y + ci.minY)); tu.Add(ci.uvBottomLeft);
                tv.Add(new Vector2(x + ci.maxX, y + ci.minY)); tu.Add(ci.uvBottomRight);
                tv.Add(new Vector2(x + ci.maxX, y + ci.maxY)); tu.Add(ci.uvTopRight);
                tv.Add(new Vector2(x + ci.minX, y + ci.maxY)); tu.Add(ci.uvTopLeft);
                for (int i = 0; i < 4; i++) tc.Add(lc);
                ti.Add(k); ti.Add(k + 1); ti.Add(k + 2); ti.Add(k); ti.Add(k + 2); ti.Add(k + 3);
                x += ci.advance;
            }
        }

        static string F(float v, string fmt) => float.IsNaN(v) ? "-" : v.ToString(fmt, CultureInfo.InvariantCulture);

        // ------------------------------------------------------------------ layout

        bool rebuilt;

        public void Draw(RenderTexture target, HudModel m) {
            // a glyph outside the preloaded set can rebuild the atlas mid-layout; lay out again when it does
            for (int pass = 0; pass < 3; pass++) {
                rebuilt = false;
                Layout(m);
                if (!rebuilt) break;
            }
            shapes.Clear(); text.Clear();
            shapes.SetVertices(sv); shapes.SetColors(sc); shapes.SetTriangles(si, 0, false);
            text.SetVertices(tv); text.SetColors(tc); text.SetUVs(0, tu); text.SetTriangles(ti, 0, false);
            shapes.bounds = text.bounds = new Bounds(Vector3.zero, Vector3.one * 1e6f);
            mat.SetVector("_Screen", new Vector4(w, h, SystemInfo.graphicsUVStartsAtTop ? -1f : 1f, 0));
            mat.SetTexture("_MainTex", font.material.mainTexture);
            cmd.Clear();
            cmd.SetRenderTarget(target);
            cmd.DrawMesh(shapes, Matrix4x4.identity, mat, 0, 0);
            cmd.DrawMesh(text, Matrix4x4.identity, mat, 0, 1);
            Graphics.ExecuteCommandBuffer(cmd);
        }

        /// <summary>The minimal HUD's caption line: backend, then the act's caption (else the hero's run id), then run time.</summary>
        public static string MinimalCaption(HudModel m) {
            var parts = new List<string>();
            if (!string.IsNullOrEmpty(m.backend)) parts.Add(m.backend);
            if (!string.IsNullOrEmpty(m.caption)) parts.Add(m.caption);
            else if (!string.IsNullOrEmpty(m.goldId)) parts.Add(m.goldId);
            parts.Add($"t {F(m.runTime, "0.0")} s");
            return string.Join("   ·   ", parts);
        }

        void LayoutMinimal(HudModel m) {
            float pad = 32 * s, H = h;
            string cap = MinimalCaption(m);
            float tw = Mathf.Max(Measure(m.title ?? "", 30, FontStyle.Bold), Measure(cap, 20, FontStyle.Normal)) + 2 * 18 * s;
            Rect(pad - 18 * s, H - pad - 76 * s, tw, 92 * s, Panel);
            Text(m.title, pad, H - pad - 26 * s, 30, Ink, 0, FontStyle.Bold);
            Text(cap, pad, H - pad - 60 * s, 20, Dim);
        }

        void Layout(HudModel m) {
            sv.Clear(); sc.Clear(); si.Clear(); tv.Clear(); tc.Clear(); tu.Clear(); ti.Clear();
            if (Mode == "minimal") { LayoutMinimal(m); return; }
            float pad = 36 * s, W = w, H = h;

            // top left: title and caption
            float tlw = Mathf.Max(Measure(m.title ?? "", 32, FontStyle.Bold), Measure(m.caption ?? "", 22, FontStyle.Normal)) + 2 * 20 * s;
            Rect(pad - 20 * s, H - pad - 112 * s, Mathf.Max(tlw, 300 * s), 128 * s, Panel);
            Text("CRANE  GHOST  THEATER", pad, H - pad - 4 * s, 15, Dim, 0, FontStyle.Bold);
            Text(m.title, pad, H - pad - 46 * s, 32, Ink, 0, FontStyle.Bold);
            Text(m.caption, pad, H - pad - 84 * s, 22, Dim);

            // top right: iteration counter and run counts
            float rx = W - pad;
            Rect(rx - 330 * s, H - pad - 220 * s, 350 * s, 236 * s, Panel);
            Text(m.iteration >= 0 ? "ITERATION" : "ITERATIONS", rx, H - pad - 4 * s, 15, Dim, 1, FontStyle.Bold);
            string counter = m.iteration >= 0 ? $"{m.iteration}" : $"{m.iterFrom}–{m.iterTo}";
            Text(counter, rx - Measure($" / {m.iterTo}", 26, FontStyle.Normal), H - pad - 74 * s, 72, Ink, 1, FontStyle.Bold);
            Text($" / {m.iterTo}", rx, H - pad - 74 * s, 26, Dim, 1);
            float ry = H - pad - 118 * s;
            Row(rx, ry, "runs", m.runs.ToString(CultureInfo.InvariantCulture), Ink);
            Row(rx, ry - 30 * s, "running", m.running.ToString(CultureInfo.InvariantCulture), Colormaps.Gate);
            Row(rx, ry - 60 * s, "finished", m.finished.ToString(CultureInfo.InvariantCulture), Colormaps.Eval("viridis", 0.85f));
            Row(rx, ry - 90 * s, "failed", m.failed.ToString(CultureInfo.InvariantCulture), Colormaps.Failure);

            // bottom left: legend
            float lx = pad, ly = pad + 8 * s, lw = 360 * s, lh = 14 * s;
            Rect(lx - 20 * s, ly - 20 * s, lw + 40 * s, 92 * s, Panel);
            Text($"colour: {m.colour}", lx, ly + 48 * s, 18, Dim, 0, FontStyle.Bold);
            if (m.colour == "outcome") {
                Rect(lx, ly + 18 * s, lw * 0.48f, lh, Colormaps.Eval(m.ramp, 0.85f));
                Rect(lx + lw * 0.52f, ly + 18 * s, lw * 0.48f, lh, Colormaps.Failure);
            }
            else {
                const int steps = 24;
                for (int i = 0; i < steps; i++) {
                    Color a = Colormaps.Eval(m.colour == "iteration" ? m.ramp : m.ramp, i / (float)steps), b = Colormaps.Eval(m.ramp, (i + 1) / (float)steps);
                    float x0 = lx + lw * i / steps, x1 = lx + lw * (i + 1) / steps;
                    Quad(new Vector2(x0, ly + 18 * s), new Vector2(x1, ly + 18 * s), new Vector2(x1, ly + 18 * s + lh), new Vector2(x0, ly + 18 * s + lh), a, b, b, a);
                }
            }
            Text(m.legendLo, lx, ly, 15, Dim);
            Text(m.legendHi, lx + lw, ly, 15, Dim, 1);

            // bottom right: best score sparkline
            float bw = 440 * s, bh = 150 * s, bx = W - pad - bw, by = pad;
            Rect(bx - 20 * s, by - 20 * s, bw + 40 * s, bh + 40 * s, Panel);
            Text("BEST SCORE", bx, by + bh - 4 * s, 15, Dim, 0, FontStyle.Bold);
            Text(F(m.best, "0.000"), bx + bw, by + bh - 10 * s, 32, Ink, 1, FontStyle.Bold);
            if (m.spark != null && m.spark.Length > 0) {
                float lo = float.PositiveInfinity, hi = float.NegativeInfinity;
                foreach (float v in m.spark) if (!float.IsNaN(v)) { lo = Mathf.Min(lo, v); hi = Mathf.Max(hi, v); }
                if (hi <= lo) { hi = lo + 1; }
                float gx = bx, gy = by + 8 * s, gw = bw, gh = bh - 64 * s;
                Line(new Vector2(gx, gy), new Vector2(gx + gw, gy), 1.2f * s, new Color(1, 1, 1, 0.18f));
                Vector2? prev = null; float run = float.NegativeInfinity;
                for (int i = 0; i < m.spark.Length; i++) {
                    float v = m.spark[i];
                    if (float.IsNaN(v)) continue;
                    float x = gx + (m.spark.Length > 1 ? gw * i / (m.spark.Length - 1) : gw * 0.5f);
                    var pt = new Vector2(x, gy + gh * (v - lo) / (hi - lo));
                    bool past = i <= m.sparkCursor;
                    Color c = past ? new Color(0.55f, 0.85f, 1f, 1f) : new Color(1, 1, 1, 0.22f);
                    if (prev.HasValue) Line(prev.Value, pt, (past ? 3f : 1.5f) * s, c);
                    if (v > run) { run = v; if (past) Dot(pt, 3.2f * s, Colormaps.Gold); }
                    prev = pt;
                    if (i == m.sparkCursor) Dot(pt, 7f * s, Ink);
                }
            }

            // bottom centre: insight and run clock
            string clock = $"t {F(m.runTime, "0.0")} s   {F(m.speed, "0.#")}×";
            Text(clock, W * 0.5f, pad + 2 * s, 18, Dim, 0.5f);
            if (!string.IsNullOrEmpty(m.insight) && (m.actType == "evolution" || m.actType == "compare" || m.actType == "spotlight")) {
                float iw = Measure(m.insight, 22, FontStyle.Normal);
                Rect(W * 0.5f - iw * 0.5f - 18 * s, pad + 30 * s, iw + 36 * s, 40 * s, Panel);
                Text(m.insight, W * 0.5f, pad + 42 * s, 22, Ink, 0.5f);
            }

            // compare: gold vs silver
            if (m.goldId != null && (m.actType == "compare" || m.actType == "spotlight")) {
                float cx = W * 0.5f, top = H - pad;
                float pw = 620 * s;
                bool two = m.silverId != null;
                Rect(cx - pw * 0.5f, top - (two ? 150 : 84) * s, pw, (two ? 166 : 100) * s, Panel);
                HeroRow(cx - pw * 0.5f + 22 * s, top - 30 * s, pw - 44 * s, Colormaps.Gold, "BEST", m.goldId, m.goldScore, m.goldDuration, m.goldProgress, m.goldDone);
                if (two) {
                    HeroRow(cx - pw * 0.5f + 22 * s, top - 96 * s, pw - 44 * s, Colormaps.Silver, "PREVIOUS", m.silverId, m.silverScore, m.silverDuration, m.silverProgress, m.silverDone);
                    float dt = m.goldDuration - m.silverDuration;
                    Text($"Δ {(dt >= 0 ? "+" : "")}{F(dt, "0.0")} s", cx + pw * 0.5f - 22 * s, top - 140 * s, 18, Dim, 1);
                }
                if (m.paramDiff.Count > 0) {
                    float px = pad, py = H - pad - 170 * s;
                    float pwid = 0; foreach (var line in m.paramDiff) pwid = Mathf.Max(pwid, Measure(line, 18, FontStyle.Normal));
                    Rect(px - 20 * s, py - m.paramDiff.Count * 28 * s - 14 * s, Mathf.Max(pwid, 220 * s) + 40 * s, m.paramDiff.Count * 28 * s + 50 * s, Panel);
                    Text("WHAT CHANGED", px, py, 15, Dim, 0, FontStyle.Bold);
                    for (int i = 0; i < m.paramDiff.Count; i++) Text(m.paramDiff[i], px, py - (i + 1) * 28 * s, 18, Ink);
                }
            }

        }

        void Row(float rx, float y, string label, string value, Color c) {
            Dot(new Vector2(rx - 300 * s, y + 7 * s), 5 * s, c);
            Text(label, rx - 286 * s, y, 18, Dim);
            Text(value, rx, y, 22, Ink, 1, FontStyle.Bold);
        }

        void HeroRow(float x, float y, float width, Color c, string label, string id, float score, float duration, float progress, bool done) {
            Dot(new Vector2(x + 8 * s, y + 8 * s), 8 * s, c);
            Text(label, x + 26 * s, y, 18, c, 0, FontStyle.Bold);
            Text(id, x + 140 * s, y, 18, Ink);
            Text($"score {F(score, "0.000")}   {F(duration, "0.0")} s", x + width, y, 18, Ink, 1);
            float bw = width - 26 * s, bx = x + 26 * s, by = y - 26 * s;
            Rect(bx, by, bw, 8 * s, new Color(1, 1, 1, 0.15f));
            Rect(bx, by, bw * Mathf.Clamp01(progress), 8 * s, done ? c : c * new Color(1, 1, 1, 0.8f));
        }
    }

    static class ColorExt {
        public static Color WithA(this Color c, float a) { c.a = a; return c; }
    }
}
