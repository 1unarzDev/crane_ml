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
        readonly Font mono;   // numbers (the style's mono font; the label font without one)
        readonly Mesh shapes = new Mesh { name = "hud shapes" }, text = new Mesh { name = "hud text" }, textMono = new Mesh { name = "hud numbers" };
        readonly List<Vector3> sv = new List<Vector3>(), tv = new List<Vector3>(), mv = new List<Vector3>();
        readonly List<Color> sc = new List<Color>(), tc = new List<Color>(), mc = new List<Color>();
        readonly List<Vector2> tu = new List<Vector2>(), mu = new List<Vector2>();
        readonly List<int> si = new List<int>(), ti = new List<int>(), mi = new List<int>();
        readonly MaterialPropertyBlock textProps = new MaterialPropertyBlock(), monoProps = new MaterialPropertyBlock();
        readonly HudStyle style;
        readonly CommandBuffer cmd = new CommandBuffer { name = "theater hud" };
        readonly int w, h;
        readonly List<Rect> occupied = new List<Rect>();   // styled HUD: the blocks this frame, kept clear of world labels
        readonly float s;
        static readonly int[] Sizes = { 14, 15, 16, 18, 20, 22, 26, 32, 44, 56, 72 };
        const string Charset = " !\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~→–×·Δ•";
        Color Ink = new Color(0.94f, 0.96f, 1f, 1f), Dim = new Color(0.66f, 0.72f, 0.8f, 1f), Panel = new Color(0.02f, 0.04f, 0.07f, 0.55f);

        public HudRenderer(int width, int height, Dictionary<string, object> styleDoc = null) {
            w = width; h = height; s = height / 1080f;
            var shader = Resources.Load<Shader>("CraneTheaterHud");
            if (shader == null) throw new InvalidOperationException("theater: shader Resources/CraneTheaterHud missing");
            mat = new Material(shader) { hideFlags = HideFlags.DontSave };
            style = styleDoc != null ? new HudStyle(styleDoc) : null;
            font = OsFont(style?.font) ?? Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            mono = OsFont(style?.mono) ?? font;
            if (style != null) { Ink = style.ink; Dim = style.dim; Panel = style.panel; }
            foreach (var f in mono == font ? new[] { font } : new[] { font, mono })
                foreach (int size in Sizes) {
                    f.RequestCharactersInTexture(Charset, Px(size), FontStyle.Normal);
                    f.RequestCharactersInTexture(Charset, Px(size), FontStyle.Bold);
                }
            shapes.MarkDynamic(); text.MarkDynamic(); textMono.MarkDynamic();
            Font.textureRebuilt += f => { if (f == font || f == mono) rebuilt = true; };
        }

        /// <summary>A font by family name: one the theater carries (Resources/Fonts: DejaVu Sans, DejaVu Sans Mono),
        /// else an installed OS font; null when absent, or when no name is given.</summary>
        static Font OsFont(string name) {
            if (string.IsNullOrEmpty(name)) return null;
            var carried = Resources.Load<Font>("Fonts/" + name.Replace(" ", ""));
            if (carried != null) return carried;
            try {
                foreach (var n in Font.GetOSInstalledFontNames())
                    if (string.Equals(n, name, StringComparison.OrdinalIgnoreCase)) return Font.CreateDynamicFontFromOSFont(n, 32);
            }
            catch (Exception) { }
            Debug.LogWarning($"theater hud: font '{name}' is not installed; using the built-in font");
            return null;
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

        float Measure(string str, int size, FontStyle style, bool num = false) {
            float x = 0;
            var f = num ? mono : font;
            foreach (char ch in str) if (f.GetCharacterInfo(ch, out var ci, Px(size), style)) x += ci.advance;
            return x;
        }

        /// <summary>Text with its baseline at y; align 0 left, 0.5 centre, 1 right. Returns the width.</summary>
        float Text(string str, float x, float y, int size, Color c, float align = 0, FontStyle style = FontStyle.Normal, bool shadow = true, bool num = false) {
            if (string.IsNullOrEmpty(str)) return 0;
            (num ? mono : font).RequestCharactersInTexture(str, Px(size), style);
            float width = Measure(str, size, style, num);
            x = Mathf.Round(x - width * align); y = Mathf.Round(y);
            if (shadow) Glyphs(str, x + 1.5f * s, y - 1.5f * s, size, new Color(0, 0, 0, 0.6f * c.a), style, num);
            Glyphs(str, x, y, size, c, style, num);
            return width;
        }

        void Glyphs(string str, float x, float y, int size, Color c, FontStyle style, bool num = false) {
            Color lc = c.linear.WithA(c.a);
            var f = num && mono != font ? mono : font;
            var tv = f == font ? this.tv : mv; var tu = f == font ? this.tu : mu; var tc = f == font ? this.tc : mc; var ti = f == font ? this.ti : mi;
            foreach (char ch in str) {
                if (!f.GetCharacterInfo(ch, out var ci, Px(size), style)) continue;
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

        float dipAlpha;   // the act-transition dip, drawn over the HUD's shapes and text
        Mesh dip;

        public void Draw(RenderTexture target, HudModel m) {
            dipAlpha = 0f;
            // a glyph outside the preloaded set can rebuild the atlas mid-layout; lay out again when it does
            for (int pass = 0; pass < 3; pass++) {
                rebuilt = false;
                Layout(m);
                if (!rebuilt) break;
            }
            shapes.Clear(); text.Clear(); textMono.Clear();
            shapes.SetVertices(sv); shapes.SetColors(sc); shapes.SetTriangles(si, 0, false);
            text.SetVertices(tv); text.SetColors(tc); text.SetUVs(0, tu); text.SetTriangles(ti, 0, false);
            textMono.SetVertices(mv); textMono.SetColors(mc); textMono.SetUVs(0, mu); textMono.SetTriangles(mi, 0, false);
            shapes.bounds = text.bounds = textMono.bounds = new Bounds(Vector3.zero, Vector3.one * 1e6f);
            mat.SetVector("_Screen", new Vector4(w, h, SystemInfo.graphicsUVStartsAtTop ? -1f : 1f, 0));
            textProps.SetTexture("_MainTex", font.material.mainTexture);
            monoProps.SetTexture("_MainTex", mono.material.mainTexture);
            cmd.Clear();
            cmd.SetRenderTarget(target);
            cmd.DrawMesh(shapes, Matrix4x4.identity, mat, 0, 0);
            cmd.DrawMesh(text, Matrix4x4.identity, mat, 0, 1, textProps);
            if (mv.Count > 0) cmd.DrawMesh(textMono, Matrix4x4.identity, mat, 0, 1, monoProps);
            if (dipAlpha > 0) {
                dip ??= new Mesh { name = "hud dip" };
                var c = Panel.linear.WithA(dipAlpha);
                dip.Clear();
                dip.SetVertices(new[] { new Vector3(0, 0), new Vector3(w, 0), new Vector3(w, h), new Vector3(0, h) });
                dip.SetColors(new[] { c, c, c, c });
                dip.SetTriangles(new[] { 0, 1, 2, 0, 2, 3 }, 0, false);
                dip.bounds = new Bounds(Vector3.zero, Vector3.one * 1e6f);
                cmd.DrawMesh(dip, Matrix4x4.identity, mat, 0, 0);
            }
            Graphics.ExecuteCommandBuffer(cmd);
        }

        void Layout(HudModel m) {
            sv.Clear(); sc.Clear(); si.Clear(); tv.Clear(); tc.Clear(); tu.Clear(); ti.Clear();
            mv.Clear(); mc.Clear(); mu.Clear(); mi.Clear();
            if (style != null) { LayoutStyled(m); return; }
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

            Heroes(m, pad, 0);
        }

        void Heroes(HudModel m, float pad, float drop) {
            float W = w, H = h;
            // compare: gold vs silver
            if (m.goldId != null && (m.actType == "compare" || m.actType == "spotlight")) {
                float cx = W * 0.5f, top = H - pad - drop;
                float pw = (style != null ? 760 : 620) * s;
                bool two = m.silverId != null;
                if (style != null) Block(cx - pw * 0.5f, top - (two ? 150 : 84) * s, pw, (two ? 166 : 100) * s);
                else Rect(cx - pw * 0.5f, top - (two ? 150 : 84) * s, pw, (two ? 166 : 100) * s, Panel);
                HeroRow(cx - pw * 0.5f + 22 * s, top - 30 * s, pw - 44 * s, Colormaps.Gold, "BEST", m.goldId, m.goldScore, m.goldDuration, m.goldProgress, m.goldDone);
                if (two) {
                    HeroRow(cx - pw * 0.5f + 22 * s, top - 96 * s, pw - 44 * s, Colormaps.Silver, "PREVIOUS", m.silverId, m.silverScore, m.silverDuration, m.silverProgress, m.silverDone);
                    if (m.goldDone && m.goldDuration < m.silverDuration) {   // gold is in: the silver run's lag counts up until it arrives
                        float late = (m.silverDone ? m.silverDuration : m.runTime) - m.goldDuration;
                        Text($"{(m.silverDone ? "arrived " : "")}+{F(late, "0")} s", cx - pw * 0.5f + 48 * s, top - 140 * s, 18, Colormaps.Silver, 0, FontStyle.Bold);
                    }
                    float dt = m.goldDuration - m.silverDuration;
                    Text($"Δ {(dt >= 0 ? "+" : "")}{F(dt, "0.0")} s", cx + pw * 0.5f - 22 * s, top - 140 * s, 18, Dim, 1);
                }
                if (m.paramDiff.Count > 0) {
                    float px = pad, py = H - pad - (style != null ? 196 : 170) * s;
                    float pwid = 0; foreach (var line in m.paramDiff) pwid = Mathf.Max(pwid, Measure(line, 18, FontStyle.Normal));
                    if (style != null) Block(px - 20 * s, py - m.paramDiff.Count * 28 * s - 14 * s, Mathf.Max(pwid, 220 * s) + 40 * s, m.paramDiff.Count * 28 * s + 50 * s);
                    else Rect(px - 20 * s, py - m.paramDiff.Count * 28 * s - 14 * s, Mathf.Max(pwid, 220 * s) + 40 * s, m.paramDiff.Count * 28 * s + 50 * s, Panel);
                    Text("WHAT CHANGED", px, py, 15, Dim, 0, FontStyle.Bold);
                    for (int i = 0; i < m.paramDiff.Count; i++) Text(m.paramDiff[i], px, py - (i + 1) * 28 * s, 18, Ink);
                }
            }

        }

        /// <summary>The figure-style layout (manifest <c>hud</c>): brand, title and caption top left; the iteration
        /// counter with one dot per iteration top right; the iteration's metric rows bottom left (labels in the label
        /// font, values in the mono font); the study's own series as a sparkline with axes bottom right.</summary>
        void LayoutStyled(HudModel m) {
            var st = style;
            float pad = 48 * s, W = w, H = h, top = H - pad;
            int active = m.active;
            bool spot = m.goldId != null && (m.actType == "compare" || m.actType == "spotlight");   // no fleet counters in a spotlight
            occupied.Clear();
            if (Panel.a > 0) {   // scrims: the top band and the bottom band fade into the scene
                Color clear = Panel.WithA(0f);
                float tb = st.scrimTop * s, bb = st.scrimBottom * s;
                Quad(new Vector2(0, H - tb), new Vector2(W, H - tb), new Vector2(W, H), new Vector2(0, H), clear, clear, Panel, Panel);
                Quad(new Vector2(0, 0), new Vector2(W, 0), new Vector2(W, bb), new Vector2(0, bb), Panel, Panel, clear, clear);
            }

            // top left: brand, title and subtitle, caption, on one panel
            float tw = Measure(m.title ?? "", 44, FontStyle.Bold), sw = Measure(st.subtitle, 26, FontStyle.Normal);
            float bw = Mathf.Max(tw + 18 * s + sw, Mathf.Max(Mathf.Max(TitleCaptionMin, Measure(m.caption ?? "", 20, FontStyle.Normal)), Measure(st.brand, 15, FontStyle.Bold)));
            Block(pad - 22 * s, top - 108 * s, bw + 44 * s, 132 * s);
            Text(st.brand, pad, top - 2 * s, 15, Dim, 0, FontStyle.Bold);
            Text(m.title, pad, top - 50 * s, 44, Ink, 0, FontStyle.Bold);
            Text(st.subtitle, pad + tw + 18 * s, top - 50 * s, 26, Ink);
            Text(m.caption, pad, top - 90 * s, 20, Dim);

            // top right: counter and dots (a spotlight: "0 vs 5")
            float rx = W - pad;
            int n = Mathf.Max(1, m.iterCount);
            string counter = spot && m.silverIter >= 0 ? $"{m.silverIter} vs {m.goldIter}"
                : active >= 0 ? active.ToString(CultureInfo.InvariantCulture) : $"{m.iterFrom}–{m.iterTo}";
            int csize = spot ? 56 : 72;
            string runsLine = spot ? null : $"{m.runs} runs   {m.running} running   {m.finished} arrived";
            // one width for the act: sized for the widest the counts can get (every run running and arrived)
            string runsWide = spot ? null : $"{m.runs} runs   {m.runs} running   {m.runs} arrived";
            float cwid = Mathf.Max(Measure(counter, csize, FontStyle.Bold), Mathf.Max(n * 26 * s, runsWide != null ? Measure(runsWide, 16, FontStyle.Normal, true) : 0));
            Block(rx - cwid - 22 * s, top - (runsLine != null ? 160 : 124) * s, cwid + 44 * s, (runsLine != null ? 184 : 148) * s);
            Text(st.iterLabel, rx, top - 2 * s, 18, Dim, 1);
            Text(counter, rx, top - 76 * s, csize, Ink, 1, FontStyle.Bold);
            for (int i = 0; i < n; i++) {
                int it = m.iterMin + i;
                bool on = spot ? it == m.goldIter || it == m.silverIter : active < 0 || it <= active;
                var c = on ? (st.dotsByRamp && m.ramp != null ? Colormaps.Eval(m.ramp, n > 1 ? i / (float)(n - 1) : 1f) : st.accent) : Dim.WithA(0.35f);
                var at = new Vector2(rx - (n - 1 - i) * 26 * s - 8 * s, top - 108 * s);
                Dot(at, (it == active || (spot && on) ? 7f : 5f) * s, c);
            }
            if (runsLine != null) Text(runsLine, rx, top - 146 * s, 16, Dim, 1, FontStyle.Normal, true, true);

            // bottom centre: the headline on its own pill
            if (!string.IsNullOrEmpty(m.headline)) {
                float hw = Measure(m.headline, 26, FontStyle.Bold);
                Block(W * 0.5f - hw * 0.5f - 24 * s, pad - 10 * s, hw + 48 * s, 48 * s);
                Text(m.headline, W * 0.5f, pad + 6 * s, 26, Ink, 0.5f, FontStyle.Bold);
            }

            // bottom left: metric rows, legend above them (not in a spotlight: its panel says what changed)
            if (!spot) {
                var rows = st.Rows(active);
                float ry = pad + 6 * s, vw = 0;
                foreach (var r in active >= 0 ? st.AllIterationRows() : rows) vw = Mathf.Max(vw, Measure(r.value, 22, FontStyle.Normal, true));   // one width for every iteration: the panel does not jump
                float x0 = pad + Measure(st.iterLabel.ToLowerInvariant(), 16, FontStyle.Normal) + 14 * s;
                float cw = 44 * s, ch = 16 * s, gap = 8 * s;
                float blockW = Mathf.Max(300 * s + vw, m.colour == "iteration" ? x0 - pad + n * (cw + gap) : 0);
                float blockH = rows.Count * 36 * s + (m.colour == "iteration" ? 62 * s : 0);
                Block(pad - 22 * s, pad - 16 * s, blockW + 44 * s, blockH + 20 * s);
                for (int i = rows.Count - 1; i >= 0; i--) {
                    Text(rows[i].label, pad, ry, 20, Dim);
                    Text(rows[i].value, pad + 300 * s, ry, 22, rows[i].color ?? Ink, 0, FontStyle.Normal, true, true);
                    ry += 36 * s;
                }
                if (m.colour == "iteration") {   // one chip per iteration in its ramp colour; the current one lit, the rest dimmed
                    float ly = ry + 6 * s;
                    Text(st.iterLabel.ToLowerInvariant(), pad, ly + 2 * s, 16, Dim);
                    int cur = m.current >= 0 ? m.current : active;
                    for (int i = 0; i < n; i++) {
                        int it = m.iterMin + i;
                        var col = Colormaps.Eval(m.ramp, n > 1 ? i / (float)(n - 1) : 1f);
                        bool lit = cur < 0 || it == cur;
                        float x = x0 + i * (cw + gap);
                        if (it == cur) Rect(x - 2 * s, ly - 2 * s, cw + 4 * s, ch + 4 * s, Ink.WithA(0.9f));
                        Rect(x, ly, cw, ch, col.WithA(lit ? 1f : (cur >= 0 && it < cur ? 0.45f : 0.25f)));
                        Text(it.ToString(CultureInfo.InvariantCulture), x + cw * 0.5f, ly + ch + 8 * s, 15, lit ? Ink : Dim, 0.5f, FontStyle.Normal, false, true);   // one weight: the mono font has no bold face
                    }
                }
            }

            // bottom right: the series
            var v = st.spark;
            if (v != null && v.Length > 0) {
                float gw = 400 * s, gh = 150 * s, gx = W - pad - gw, gy = pad + 34 * s;
                Block(gx - 66 * s, gy - 50 * s, gw + 88 * s, gh + 98 * s);
                Text(st.sparkLabel, gx + gw, gy + gh + 30 * s, 20, Ink, 1);
                float lo = st.sparkLo, hi = st.sparkHi;
                if (!(hi > lo)) {
                    lo = float.PositiveInfinity; hi = float.NegativeInfinity;
                    foreach (float x in v) if (!float.IsNaN(x)) { lo = Mathf.Min(lo, x); hi = Mathf.Max(hi, x); }
                    if (!(hi > lo)) hi = lo + 1;
                }
                var grid = Dim.WithA(0.22f);
                for (int k = 0; k <= 2; k++) {
                    float val = lo + (hi - lo) * k / 2f, y = gy + gh * k / 2f;
                    Line(new Vector2(gx, y), new Vector2(gx + gw, y), 1f * s, grid);
                    Text(val.ToString(st.sparkFmt, CultureInfo.InvariantCulture), gx - 24 * s, y - 6 * s, 15, Dim, 1, FontStyle.Normal, false, true);
                }
                int cursor = active >= 0 ? active - m.iterMin : v.Length - 1;
                Vector2? prev = null;
                for (int i = 0; i < v.Length; i++) {
                    float x = gx + (v.Length > 1 ? gw * i / (v.Length - 1) : gw * 0.5f);
                    Text((m.iterMin + i).ToString(CultureInfo.InvariantCulture), x, gy - 28 * s, 15, Dim, 0.5f, FontStyle.Normal, false, true);
                    if (float.IsNaN(v[i])) continue;
                    var pt = new Vector2(x, gy + gh * Mathf.Clamp01((v[i] - lo) / (hi - lo)));
                    bool past = i <= cursor;
                    if (prev.HasValue) Line(prev.Value, pt, (past ? 3f : 1.5f) * s, past ? st.accent : Dim.WithA(0.35f));
                    if (past) {
                        Dot(pt, 4.5f * s, st.accent);
                        string lab = st.sparkPointLabels != null && i < st.sparkPointLabels.Length ? st.sparkPointLabels[i] : v[i].ToString(st.sparkFmt, CultureInfo.InvariantCulture);
                        // below the point where the line comes down into it (a dip or a fall), above at a peak or a climb
                        bool below = i > 0 && !float.IsNaN(v[i - 1]) && v[i - 1] > v[i];
                        HaloText(lab, pt.x, below ? pt.y - 26 * s : pt.y + 12 * s, 14, Dim, 0.5f, true);
                        if (st.sparkNotes.TryGetValue(m.iterMin + i, out var note)) {
                            float nw = Measure(note, 14, FontStyle.Normal);
                            float nx = Mathf.Min(pt.x, gx + gw + 18 * s - nw * 0.5f);
                            Text(note, nx, pt.y + 32 * s, 14, st.noteColor, 0.5f);
                        }
                    }
                    if (i == cursor) Dot(pt, 7f * s, Ink);
                    prev = pt;
                }
            }

            Heroes(m, pad, 150 * s);
            WorldLabels(m, pad);
            // an act transition: the frame dips toward the panel colour, the HUD's text with it (drawn last, over everything)
            dipAlpha = m.fade;
        }

        /// <summary>Widest caption over the whole script (styled HUD): the title panel keeps one width for every act.</summary>
        public float TitleCaptionMin;
        public void LockTitle(IEnumerable<string> captions) {
            foreach (var c in captions) {
                if (string.IsNullOrEmpty(c)) continue;
                font.RequestCharactersInTexture(c, Px(20), FontStyle.Normal);
                TitleCaptionMin = Mathf.Max(TitleCaptionMin, Measure(c, 20, FontStyle.Normal));
            }
        }

        /// <summary>Text with a 2 px halo in the panel colour, so a line under it does not strike it through.</summary>
        void HaloText(string str, float x, float y, int size, Color c, float align, bool num) {
            var hc = Panel.WithA(1f);
            for (int dx = -1; dx <= 1; dx++)
                for (int dy = -1; dy <= 1; dy++)
                    if (dx != 0 || dy != 0) Text(str, x + dx * 2f * s, y + dy * 2f * s, size, hc, align, FontStyle.Normal, false, num);
            Text(str, x, y, size, c, align, FontStyle.Normal, false, num);
        }

        /// <summary>A backing panel for a HUD block (styled HUD): opaque enough that the water, its foam and the buoys
        /// under it do not show through; its rectangle is kept so world labels stay clear of it.</summary>
        void Block(float x, float y, float bw, float bh) {
            Rect(x, y, bw, bh, Panel.WithA(Mathf.Max(Panel.a, style != null ? style.blockAlpha : 0.82f)));
            occupied.Add(new Rect(x, y, bw, bh));
        }

        /// <summary>World labels (projected ring labels and the like): clamped to the safe area and moved off the HUD's
        /// blocks and each other (the nearest free place, a thin leader back to the anchor when it moved far).</summary>
        void WorldLabels(HudModel m, float pad) {
            foreach (var r in m.avoid) occupied.Add(r);
            foreach (var (at, anchor, label, col) in m.labels) {
                float lw = Measure(label, 18, FontStyle.Bold, true) + 16 * s, lh = 28 * s;
                var want = new Rect(at.x - lw * 0.5f, at.y - 7 * s, lw, lh);
                Rect best = Clamp(want, pad);
                float bestD = Free(best) ? 0 : float.MaxValue;
                if (bestD > 0) {
                    for (int ring = 1; ring <= 40 && bestD == float.MaxValue; ring++)
                        for (int k = 0; k < 16; k++) {
                            float a = 2 * Mathf.PI * k / 16;
                            var c = Clamp(new Rect(want.x + Mathf.Cos(a) * ring * 12 * s, want.y + Mathf.Sin(a) * ring * 12 * s, lw, lh), pad);
                            float d = Vector2.Distance(c.center, want.center);
                            if (d < bestD && Free(c)) { best = c; bestD = d; }
                        }
                    if (bestD == float.MaxValue) continue;   // nowhere free: better no label than one over the numbers
                }
                if (Vector2.Distance(best.center, anchor) > 30 * s) {   // a leader from the point the label names to the label's near edge
                    var edge = new Vector2(Mathf.Clamp(anchor.x, best.xMin, best.xMax), Mathf.Clamp(anchor.y, best.yMin, best.yMax));
                    Line(anchor, edge, 1.5f * s, col.WithA(0.7f));
                    Dot(anchor, 3f * s, col);
                }
                Rect(best.x, best.y, best.width, best.height, Panel.WithA(Mathf.Max(0.75f, Panel.a)));
                occupied.Add(best);
                Text(label, best.center.x, best.y + 7 * s, 18, col, 0.5f, FontStyle.Bold, true, true);
            }
        }

        Rect Clamp(Rect r, float pad) {
            r.x = Mathf.Clamp(r.x, pad * 0.5f, w - pad * 0.5f - r.width);
            r.y = Mathf.Clamp(r.y, pad * 0.5f, h - pad * 0.5f - r.height);
            return r;
        }

        bool Free(Rect r) {
            var g = new Rect(r.x - 6 * s, r.y - 6 * s, r.width + 12 * s, r.height + 12 * s);
            foreach (var o in occupied) if (o.Overlaps(g)) return false;
            return true;
        }

        void Row(float rx, float y, string label, string value, Color c) {
            Dot(new Vector2(rx - 300 * s, y + 7 * s), 5 * s, c);
            Text(label, rx - 286 * s, y, 18, Dim);
            Text(value, rx, y, 22, Ink, 1, FontStyle.Bold);
        }

        void HeroRow(float x, float y, float width, Color c, string label, string id, float score, float duration, float progress, bool done) {
            Dot(new Vector2(x + 8 * s, y + 8 * s), 8 * s, c);
            Text(label, x + 26 * s, y, 18, c, 0, FontStyle.Bold);
            Text(id, x + (style != null ? 150 : 140) * s, y, 18, Ink);
            Text($"score {F(score, "0.000")}   {(style != null ? "to goal " : "")}{F(duration, "0.0")} s", x + width, y, 18, Ink, 1);
            float bw = width - 26 * s, bx = x + 26 * s, by = y - 26 * s;
            Rect(bx, by, bw, 8 * s, new Color(1, 1, 1, 0.15f));
            Rect(bx, by, bw * Mathf.Clamp01(progress), 8 * s, done ? c : c * new Color(1, 1, 1, 0.8f));
        }
    }

    /// <summary>Manifest <c>hud</c>: <c>{brand, subtitle, iteration_label, font, mono, palette: {ink, dim, panel,
    /// accent}, metrics: {"&lt;iteration&gt;"|"all": [[label, value, colour?]]}, spark: {label, values, lo, hi, format,
    /// point_labels}}</c>.</summary>
    public sealed class HudStyle {
        public string brand = "", subtitle = "", iterLabel = "ITERATION", font, mono, sparkLabel = "", sparkFmt = "0.00";
        public Color ink = new Color(0.91f, 0.925f, 0.95f), dim = new Color(0.545f, 0.576f, 0.639f), panel = new Color(0.04f, 0.07f, 0.13f, 0f),
            accent = new Color(0.165f, 0.47f, 0.84f);
        public float[] spark;
        public string[] sparkPointLabels;
        public float sparkLo = float.NaN, sparkHi = float.NaN;
        public bool dotsByRamp;
        public float blockAlpha = 0.82f;   // palette.block_alpha: the panels behind the HUD's blocks
        public readonly Dictionary<int, string> sparkNotes = new Dictionary<int, string>();   // spark.notes {"<iteration>": text}
        public Color noteColor = new Color(0.93f, 0.63f, 0f);
        public float scrimTop = 210, scrimBottom = 330, scrimLeft = 900, scrimLeftHeight = 380;   // pixels at 1080 p
        readonly Dictionary<string, List<(string label, string value, Color? color)>> metrics = new Dictionary<string, List<(string, string, Color?)>>();

        public HudStyle(Dictionary<string, object> d) {
            brand = TJson.Str(d, "brand", brand); subtitle = TJson.Str(d, "subtitle", subtitle);
            iterLabel = TJson.Str(d, "iteration_label", iterLabel); font = TJson.Str(d, "font"); mono = TJson.Str(d, "mono");
            var pal = TJson.Obj(d, "palette");
            if (pal != null) {
                ink = TheaterOverlay.Hex(pal, "ink", ink); dim = TheaterOverlay.Hex(pal, "dim", dim); accent = TheaterOverlay.Hex(pal, "accent", accent);
                panel = TheaterOverlay.Hex(pal, "panel", panel); panel.a = (float)TJson.Num(pal, "panel_alpha", 0);
                blockAlpha = (float)TJson.Num(pal, "block_alpha", blockAlpha);
                noteColor = TheaterOverlay.Hex(pal, "note", noteColor);
            }
            dotsByRamp = TJson.Bool(d, "dots_by_ramp", false);
            var sc = TJson.Obj(d, "scrim");   // {top, bottom, left, left_height} in pixels at 1080 p
            if (sc != null) {
                scrimTop = (float)TJson.Num(sc, "top", scrimTop); scrimBottom = (float)TJson.Num(sc, "bottom", scrimBottom);
                scrimLeft = (float)TJson.Num(sc, "left", scrimLeft); scrimLeftHeight = (float)TJson.Num(sc, "left_height", scrimLeftHeight);
            }
            if (TJson.Obj(d, "metrics") is Dictionary<string, object> ms)
                foreach (var kv in ms) {
                    var rows = new List<(string, string, Color?)>();
                    foreach (var r in kv.Value as List<object> ?? new List<object>())
                        if (r is List<object> l && l.Count >= 2) {
                            Color? c = null;
                            if (l.Count > 2 && l[2] is string hex && ColorUtility.TryParseHtmlString(hex, out var col)) c = col;
                            rows.Add((l[0] as string ?? "", l[1] as string ?? "", c));
                        }
                    metrics[kv.Key] = rows;
                }
            var sp = TJson.Obj(d, "spark");
            if (sp != null) {
                sparkLabel = TJson.Str(sp, "label", ""); sparkFmt = TJson.Str(sp, "format", sparkFmt);
                sparkLo = (float)TJson.Num(sp, "lo", double.NaN); sparkHi = (float)TJson.Num(sp, "hi", double.NaN);
                var vals = TJson.Arr(sp, "values");
                if (vals != null) { spark = new float[vals.Count]; for (int i = 0; i < vals.Count; i++) spark[i] = vals[i] is double x ? (float)x : float.NaN; }
                var labs = TJson.Arr(sp, "point_labels");
                if (TJson.Obj(sp, "notes") is Dictionary<string, object> notes)
                    foreach (var kv in notes) if (int.TryParse(kv.Key, out int it) && kv.Value is string t) sparkNotes[it] = t;
                if (labs != null) { sparkPointLabels = new string[labs.Count]; for (int i = 0; i < labs.Count; i++) sparkPointLabels[i] = labs[i] as string ?? ""; }
            }
        }

        public IEnumerable<(string label, string value, Color? color)> AllIterationRows() {
            foreach (var kv in metrics) if (kv.Key != "all") foreach (var r in kv.Value) yield return r;
        }

        public List<(string label, string value, Color? color)> Rows(int iteration) =>
            metrics.TryGetValue(iteration >= 0 ? iteration.ToString(CultureInfo.InvariantCulture) : "all", out var r) ? r : new List<(string, string, Color?)>();
    }

    static class ColorExt {
        public static Color WithA(this Color c, float a) { c.a = a; return c; }
    }
}
