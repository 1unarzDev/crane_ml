using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using UnityEngine.Rendering;

namespace Crane.Theater {
    /// <summary>What the panels show at one frame, read from the frame's state and the manifest (nothing invented:
    /// a value the manifest does not carry is not drawn).</summary>
    sealed class PanelFrame {
        public int frame;
        public string act, title, caption, insight, colour, ramp, legendLo, legendHi;
        public float iteration = float.NaN;          // NaN: a range of iterations
        public int iterFrom, iterTo, planned;
        public int runs, running, finished, failed;
        public float runTime, speed, cursor;         // cursor: the score chart's last shown iteration (index)
        public int candidate = -1, previous = -1;    // run indices (pack order)
        public string candidateRole = "";
        public bool compare;
        public float goldProgress, silverProgress;
        public bool goldDone, silverDone;
    }

    /// <summary>The theater look's data HUD: rounded, frosted-ink panels (one type scale: Fraunces for display,
    /// Recursive for text, Recursive Mono for every number, so digits are tabular), a score history that grows
    /// iteration by iteration with every run's score and the best-so-far step, the candidate's parameters as bars
    /// against their range with the previous candidate's value and the change, the candidate's logged run metrics
    /// as gauges, and an "iteration N / M · candidate" header.
    /// Motion is a pure function of the frame index: every animated value is a fixed filter over the values the
    /// state gives for the previous frames (an eased or a spring kernel), and a panel fades and slides in or out the
    /// same way; nothing integrates over rendered frames, so a chunk renders the same pixels wherever it starts.</summary>
    public sealed class TheaterPanels : IDisposable {
        readonly TheaterState st;
        readonly TheaterManifest M;
        readonly Material mat;
        readonly int w, h;
        readonly float s;
        readonly CommandBuffer cmd = new CommandBuffer { name = "theater panels" };
        readonly Mesh shapes = new Mesh { name = "panel shapes" };
        readonly List<Vector3> sv = new List<Vector3>();
        readonly List<Color> sc = new List<Color>();
        readonly List<Vector4> s0 = new List<Vector4>(), s1 = new List<Vector4>(), s2 = new List<Vector4>(), s3 = new List<Vector4>();
        readonly List<int> si = new List<int>();
        readonly RenderTexture[] blur = new RenderTexture[3];

        sealed class Face {
            public Font font; public Mesh mesh;
            public readonly List<Vector3> v = new List<Vector3>(); public readonly List<Color> c = new List<Color>();
            public readonly List<Vector2> uv = new List<Vector2>(); public readonly List<int> i = new List<int>();
        }
        readonly Face display, sans, sansMed, mono;
        readonly Face[] faces;
        bool rebuilt;

        // palette (the research site's "Petri Ink": calm, AA on the water)
        static Color Hex(string x, float a = 1f) { ColorUtility.TryParseHtmlString(x, out var c); c.a = a; return c; }
        static readonly Color Ink = Hex("#E4E6EB"), Ink2 = Hex("#ADB2BE"), Ink3 = Hex("#8F95A3"), Faint = Hex("#5A606E"),
            Accent = Hex("#88BEC1"), Best = Hex("#DDBE8E"), Done = Hex("#97C3A4"), Fail = Hex("#D9999C"), Lease = Hex("#B4A9CF"),
            Card = Hex("#0D1016", 0.74f), Line = Hex("#E4E6EB", 0.09f), Track = Hex("#E4E6EB", 0.10f);

        // parameter ranges over the manifest, metric ranges, the score chart's domain
        readonly SortedDictionary<string, Vector2> paramRange = new SortedDictionary<string, Vector2>(StringComparer.Ordinal);
        readonly Dictionary<string, Vector2> metricRange = new Dictionary<string, Vector2>();
        readonly List<string> metricKeys = new List<string>();
        float scoreLo, scoreHi;
        int chartIters;

        // frame history (for the filters)
        readonly Dictionary<int, PanelFrame> cache = new Dictionary<int, PanelFrame>();
        readonly float[] ease, spring;
        int now;

        static readonly string[] MetricOrder = { "gates", "time_s", "clearance_m", "contacts", "wrong_crossings", "cross_track_m", "smooth" };

        public TheaterPanels(TheaterState state, int width, int height) {
            st = state; M = state.M; w = width; h = height; s = height / 1080f;
            var shader = Resources.Load<Shader>("CraneTheaterPanels");
            if (shader == null) throw new InvalidOperationException("theater: shader Resources/CraneTheaterPanels missing");
            mat = new Material(shader) { hideFlags = HideFlags.DontSave };
            Face Load(string name) {
                var f = Resources.Load<Font>("LookFonts/" + name);
                if (f == null) throw new InvalidOperationException($"theater: font Resources/LookFonts/{name} missing");
                var face = new Face { font = f, mesh = new Mesh { name = "panel text " + name } };
                face.mesh.MarkDynamic();
                return face;
            }
            display = Load("Fraunces-SoftSemiBold"); sans = Load("RecursiveSans-Regular");
            sansMed = Load("RecursiveSans-Medium"); mono = Load("RecursiveMono-Medium");
            faces = new[] { display, sans, sansMed, mono };
            Font.textureRebuilt += f => { foreach (var x in faces) if (x.font == f) rebuilt = true; };
            shapes.MarkDynamic();
            for (int i = 0; i < blur.Length; i++) {
                int d = 4 << i;   // 1/4, 1/8, 1/16
                blur[i] = new RenderTexture(Mathf.Max(8, w / d), Mathf.Max(8, h / d), 0, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB) { name = "panel blur", filterMode = FilterMode.Bilinear };
                blur[i].Create();
            }
            int fps = Mathf.Max(1, Mathf.RoundToInt(st.S.Fps));
            ease = Kernel(Mathf.RoundToInt(0.42f * fps), j => Mathf.Sin(Mathf.PI * (j + 0.5f) / Mathf.RoundToInt(0.42f * fps)));
            float wd = 2 * Mathf.PI / 0.62f, z = 0.62f, wn = wd / Mathf.Sqrt(1 - z * z);
            spring = Kernel(Mathf.RoundToInt(0.85f * fps), j => { float t = j / (float)fps; return Mathf.Exp(-z * wn * t) * Mathf.Sin(wd * t + 0.35f); });
            Ranges();
        }

        static float[] Kernel(int n, Func<int, float> f) {
            n = Mathf.Max(1, n);
            var k = new float[n]; float sum = 0;
            for (int j = 0; j < n; j++) { k[j] = f(j); sum += k[j]; }
            for (int j = 0; j < n; j++) k[j] /= sum;
            return k;
        }

        public void Dispose() { cmd.Release(); foreach (var r in blur) if (r != null) r.Release(); }

        void Ranges() {
            scoreLo = float.PositiveInfinity; scoreHi = float.NegativeInfinity;
            foreach (var r in M.Runs) {
                if (r.valid) { scoreLo = Mathf.Min(scoreLo, r.score); scoreHi = Mathf.Max(scoreHi, r.score); }
                foreach (var kv in r.parameters) {
                    if (!(kv.Value is double v)) continue;
                    paramRange[kv.Key] = paramRange.TryGetValue(kv.Key, out var g) ? new Vector2(Mathf.Min(g.x, (float)v), Mathf.Max(g.y, (float)v)) : new Vector2((float)v, (float)v);
                }
                if (r.metrics == null) continue;
                foreach (var kv in r.metrics) {
                    if (!(kv.Value is double v)) continue;
                    metricRange[kv.Key] = metricRange.TryGetValue(kv.Key, out var g) ? new Vector2(Mathf.Min(g.x, (float)v), Mathf.Max(g.y, (float)v)) : new Vector2((float)v, (float)v);
                }
            }
            if (float.IsInfinity(scoreLo)) { scoreLo = 0; scoreHi = 1; }
            float pad = Mathf.Max(1e-3f, (scoreHi - scoreLo) * 0.12f);
            scoreLo -= pad; scoreHi += pad;
            foreach (var k in MetricOrder) if (metricRange.ContainsKey(k) && k != "of") metricKeys.Add(k);
            int planned = (int)TJson.Num(M.Raw, "iterations_planned", 0);
            chartIters = Mathf.Max(M.MaxIteration - M.MinIteration + 1, planned);
        }

        // ------------------------------------------------------------------ model

        PanelFrame Frame(int f) {
            f = Mathf.Clamp(f, 0, Mathf.Max(0, st.FrameCount - 1));
            if (cache.TryGetValue(f, out var pf)) return pf;
            return Build(st.Evaluate(f));
        }

        PanelFrame Build(FrameState fs) {
            int f = fs.frame;
            PanelFrame pf;
            var m = fs.hud; var p = fs.plan; var a = p.act;
            pf = new PanelFrame {
                frame = f, act = a.type, title = m.title, caption = m.caption, insight = m.insight, colour = m.colour, ramp = m.ramp,
                legendLo = m.legendLo, legendHi = m.legendHi, iterFrom = m.iterFrom, iterTo = m.iterTo,
                planned = (int)TJson.Num(M.Raw, "iterations_planned", 0), runs = m.runs, running = m.running, finished = m.finished,
                failed = m.failed, runTime = m.runTime, speed = m.speed, cursor = m.sparkCursor,
                compare = m.goldId != null && (a.type == "compare" || a.type == "spotlight"),
                goldProgress = m.goldProgress, silverProgress = m.silverProgress, goldDone = m.goldDone, silverDone = m.silverDone
            };
            if (m.iteration >= 0) pf.iteration = m.iteration;
            if (p.hero >= 0) { pf.candidate = p.hero; pf.previous = p.previous; pf.candidateRole = "best"; }
            else if (m.iteration >= 0) {
                pf.candidate = BestOf(p, m.iteration);
                int k = Array.IndexOf(p.iterations, m.iteration);
                pf.previous = k > 0 ? BestOf(p, p.iterations[k - 1]) : -1;
                pf.candidateRole = "iteration best";
            }
            else {
                var rec = M.Records();
                if (rec.Count > 0) pf.candidate = rec[rec.Count - 1].index;
                if (rec.Count > 1) pf.previous = rec[rec.Count - 2].index;
                pf.candidateRole = "best so far";
            }
            cache[f] = pf;
            if (cache.Count > 400) {
                var old = new List<int>();
                foreach (var k in cache.Keys) if (k < f - 120 || k > f + 120) old.Add(k);
                foreach (var k in old) cache.Remove(k);
            }
            return pf;
        }

        int BestOf(ActPlan p, int iteration) {
            if (!p.byIteration.TryGetValue(iteration, out var l)) return -1;
            int best = -1;
            foreach (int i in l) {
                var r = M.Runs[i];
                if (!r.valid) continue;
                if (best < 0 || r.score > M.Runs[best].score) best = i;
            }
            return best;
        }

        /// <summary>Filtered channel at the current frame: the kernel over the values of the last frames (NaN skipped).</summary>
        float Filter(float[] k, Func<PanelFrame, float> get) {
            float sum = 0, ws = 0;
            for (int j = 0; j < k.Length; j++) {
                float v = get(Frame(now - j));
                if (float.IsNaN(v)) continue;
                sum += k[j] * v; ws += k[j];
            }
            return ws > 1e-6f ? sum / ws : float.NaN;
        }

        /// <summary>A panel's presence (eased 0..1) and the latest frame that showed it (its content while it leaves).</summary>
        (float alpha, PanelFrame content) Presence(Func<PanelFrame, bool> shown) {
            PanelFrame content = null;
            for (int j = 0; j < ease.Length && content == null; j++) { var pf = Frame(now - j); if (shown(pf)) content = pf; }
            float a = Filter(ease, pf => shown(pf) ? 1f : 0f);
            return (float.IsNaN(a) ? 0 : a, content);
        }

        // ------------------------------------------------------------------ primitives (pixels, origin bottom-left)

        void Box(Vector2 c, Vector2 half, float angle, float radius, Color fill, Color border, float borderW, float shadow = 0, float shadowA = 0, float glass = 0, float sheen = 0) {
            float pad = shadow + 2f;
            var ex = new Vector2(Mathf.Cos(angle), Mathf.Sin(angle)); var ey = new Vector2(-ex.y, ex.x);
            int k = sv.Count;
            for (int i = 0; i < 4; i++) {
                float lx = (i == 0 || i == 3) ? -(half.x + pad) : half.x + pad;
                float ly = (i < 2) ? -(half.y + pad) : half.y + pad;
                sv.Add(c + ex * lx + ey * ly);
                sc.Add(Lin(fill));
                s0.Add(new Vector4(lx, ly, half.x, half.y));
                s1.Add(new Vector4(radius, borderW, shadow, shadowA));
                var b = Lin(border); s2.Add(new Vector4(b.r, b.g, b.b, b.a));
                s3.Add(new Vector4(glass, sheen, 0, 0));
            }
            si.Add(k); si.Add(k + 1); si.Add(k + 2); si.Add(k); si.Add(k + 2); si.Add(k + 3);
        }

        static Color Lin(Color c) { var l = c.linear; l.a = c.a; return l; }

        void Rect(float x, float y, float rw, float rh, float radius, Color fill) =>
            Box(new Vector2(x + rw * 0.5f, y + rh * 0.5f), new Vector2(rw * 0.5f, rh * 0.5f), 0, radius, fill, Color.clear, 0);

        void Seg(Vector2 a, Vector2 b, float width, Color c) {
            var d = b - a;
            float len = d.magnitude;
            if (len < 1e-3f) return;
            Box((a + b) * 0.5f, new Vector2(len * 0.5f + width * 0.5f, width * 0.5f), Mathf.Atan2(d.y, d.x), width * 0.5f, c, Color.clear, 0);
        }

        void Dot(Vector2 c, float r, Color col) => Box(c, new Vector2(r, r), 0, r, col, Color.clear, 0);

        void Ring(Vector2 c, float r, float width, float from, float to, Color col) {
            int n = Mathf.Max(2, Mathf.CeilToInt(Mathf.Abs(to - from) / (Mathf.PI / 28)));
            Vector2 prev = c + new Vector2(Mathf.Cos(from), Mathf.Sin(from)) * r;
            for (int i = 1; i <= n; i++) {
                float a = Mathf.Lerp(from, to, i / (float)n);
                var p = c + new Vector2(Mathf.Cos(a), Mathf.Sin(a)) * r;
                Seg(prev, p, width, col);
                prev = p;
            }
        }

        /// <summary>A panel: frosted ink with a hairline border and a soft shadow.</summary>
        void Panel(float x, float y, float pw, float ph, float alpha) {
            var fill = Card; fill.a *= alpha;
            var border = Line; border.a *= alpha;
            Box(new Vector2(x + pw * 0.5f, y + ph * 0.5f), new Vector2(pw * 0.5f, ph * 0.5f), 0, 18 * s, fill, border, 1.2f * s,
                26 * s, 0.32f * alpha, 0.9f * alpha, 0.06f);
        }

        int Px(int size) => Mathf.Max(8, Mathf.RoundToInt(size * s));

        float Measure(Face f, string str, int size, float tracking = 0) {
            if (string.IsNullOrEmpty(str)) return 0;
            f.font.RequestCharactersInTexture(str, Px(size), FontStyle.Normal);
            float x = 0;
            foreach (char ch in str) if (f.font.GetCharacterInfo(ch, out var ci, Px(size), FontStyle.Normal)) x += ci.advance + tracking * s;
            return x;
        }

        /// <summary>Text with its baseline at y; align 0 left, 0.5 centre, 1 right. Returns the width.</summary>
        float Text(Face f, string str, float x, float y, int size, Color c, float align = 0, float tracking = 0) {
            if (string.IsNullOrEmpty(str) || c.a <= 0.002f) return 0;
            float width = Measure(f, str, size, tracking);
            x = Mathf.Round(x - width * align); y = Mathf.Round(y);
            var lc = Lin(c);
            foreach (char ch in str) {
                if (!f.font.GetCharacterInfo(ch, out var ci, Px(size), FontStyle.Normal)) continue;
                int k = f.v.Count;
                f.v.Add(new Vector2(x + ci.minX, y + ci.minY)); f.uv.Add(ci.uvBottomLeft);
                f.v.Add(new Vector2(x + ci.maxX, y + ci.minY)); f.uv.Add(ci.uvBottomRight);
                f.v.Add(new Vector2(x + ci.maxX, y + ci.maxY)); f.uv.Add(ci.uvTopRight);
                f.v.Add(new Vector2(x + ci.minX, y + ci.maxY)); f.uv.Add(ci.uvTopLeft);
                for (int i = 0; i < 4; i++) f.c.Add(lc);
                f.i.Add(k); f.i.Add(k + 1); f.i.Add(k + 2); f.i.Add(k); f.i.Add(k + 2); f.i.Add(k + 3);
                x += ci.advance + tracking * s;
            }
            return width;
        }

        float Label(string str, float x, float y, Color c, float align = 0) => Text(sansMed, str.ToUpperInvariant(), x, y, 15, c, align, 1.6f);

        static Color A(Color c, float a) { c.a *= a; return c; }
        static string F(float v, string fmt) => float.IsNaN(v) ? "–" : v.ToString(fmt, CultureInfo.InvariantCulture);

        // ------------------------------------------------------------------ frame

        public void Draw(RenderTexture target, FrameState fs) {
            now = fs.frame;
            if (!cache.ContainsKey(now)) Build(fs);
            for (int pass = 0; pass < 3; pass++) {
                rebuilt = false;
                sv.Clear(); sc.Clear(); s0.Clear(); s1.Clear(); s2.Clear(); s3.Clear(); si.Clear();
                foreach (var f in faces) { f.v.Clear(); f.c.Clear(); f.uv.Clear(); f.i.Clear(); }
                Layout();
                if (!rebuilt) break;
            }
            shapes.Clear();
            shapes.SetVertices(sv); shapes.SetColors(sc); shapes.SetUVs(0, s0); shapes.SetUVs(1, s1); shapes.SetUVs(2, s2); shapes.SetUVs(3, s3);
            shapes.SetTriangles(si, 0, false);
            shapes.bounds = new Bounds(Vector3.zero, Vector3.one * 1e6f);
            foreach (var f in faces) {
                f.mesh.Clear();
                f.mesh.SetVertices(f.v); f.mesh.SetColors(f.c); f.mesh.SetUVs(0, f.uv); f.mesh.SetTriangles(f.i, 0, false);
                f.mesh.bounds = shapes.bounds;
            }
            cmd.Clear();
            cmd.Blit(target, blur[0]); cmd.Blit(blur[0], blur[1]); cmd.Blit(blur[1], blur[2]);
            cmd.SetRenderTarget(target);
            mat.SetVector("_Screen", new Vector4(w, h, SystemInfo.graphicsUVStartsAtTop ? -1f : 1f, 0));
            mat.SetTexture("_Blur", blur[2]);
            cmd.DrawMesh(shapes, Matrix4x4.identity, mat, 0, 0);
            foreach (var f in faces) {
                if (f.v.Count == 0) continue;
                var block = new MaterialPropertyBlock();
                block.SetTexture("_MainTex", f.font.material.mainTexture);
                cmd.DrawMesh(f.mesh, Matrix4x4.identity, mat, 0, 1, block);
            }
            Graphics.ExecuteCommandBuffer(cmd);
        }

        void Layout() {
            var cur = Frame(now);
            float pad = 40 * s, W = w, H = h;
            float leftW = 430 * s;

            // ---- header (top left)
            {
                float ph = 116 * s, py = H - pad - ph;
                float tw = Mathf.Max(Measure(display, cur.title, 34), Measure(sans, cur.caption, 19));
                float pw = Mathf.Max(leftW, tw + 56 * s);
                Panel(pad, py, pw, ph, 1f);
                Label("crane · ghost theater", pad + 26 * s, py + ph - 32 * s, Ink3);
                Text(display, cur.title, pad + 26 * s, py + ph - 72 * s, 34, Ink);
                Text(sans, cur.caption, pad + 26 * s, py + 20 * s, 19, Ink2);
            }

            // ---- iteration (top right)
            {
                float pw = 330 * s, ph = 176 * s, px = W - pad - pw, py = H - pad - ph;
                Panel(px, py, pw, ph, 1f);
                float itv = Filter(ease, pf => pf.iteration);
                bool single = !float.IsNaN(cur.iteration);
                Label(single ? "iteration" : "iterations", px + 26 * s, py + ph - 32 * s, Ink3);
                string of = cur.planned > 0 ? $"/ {cur.planned}" : $"/ {M.MaxIteration + 1}";
                float right = px + pw - 26 * s;
                float ofw = Text(mono, of, right, py + ph - 100 * s, 24, Ink3, 1);
                float numRight = right - ofw - 10 * s;
                if (single && !float.IsNaN(itv)) {
                    // an odometer: the old number rolls up and fades as the new one rises in
                    int lo = Mathf.FloorToInt(itv + 1e-4f); float fr = Mathf.Clamp01(itv - lo);
                    float fr2 = TheaterCameraRig.Ease(fr);
                    Text(display, (lo + 1).ToString(CultureInfo.InvariantCulture), numRight, py + ph - 100 * s - (1 - fr2) * 34 * s, 72, A(Ink, fr2), 1);
                    Text(display, lo.ToString(CultureInfo.InvariantCulture), numRight, py + ph - 100 * s + fr2 * 34 * s, 72, A(Ink, 1 - fr2), 1);
                }
                else Text(display, $"{cur.iterFrom}–{cur.iterTo}", numRight, py + ph - 100 * s, 60, Ink, 1);
                // candidate line
                if (cur.candidate >= 0) {
                    float x0 = px + 26 * s;
                    float lw = Label(cur.candidateRole, x0, py + 52 * s, Ink3);
                    Text(mono, M.Runs[cur.candidate].id, x0 + lw + 10 * s, py + 52 * s, 16, Best);
                }
                // fleet chips
                float cx = px + 26 * s, cy = py + 22 * s;
                Chip(ref cx, cy, Filter(ease, pf => pf.running), Accent, "run");
                Chip(ref cx, cy, Filter(ease, pf => pf.finished), Done, "done");
                Chip(ref cx, cy, Filter(ease, pf => pf.failed), Fail, "fail");
            }

            // ---- parameters of the candidate (left)
            float colTop = H - pad - 116 * s - 16 * s;
            {
                var (alpha, pf) = Presence(x => x.candidate >= 0 && M.Runs[x.candidate].parameters.Count > 0);
                if (pf != null && alpha > 0.003f) {
                    var keys = new List<string>();
                    foreach (var kv in M.Runs[pf.candidate].parameters) if (kv.Value is double && paramRange.ContainsKey(kv.Key)) keys.Add(kv.Key);
                    keys.Sort(StringComparer.Ordinal);
                    if (keys.Count > 10) keys.RemoveRange(10, keys.Count - 10);
                    float row = 31 * s, ph = (58 + 0) * s + keys.Count * row + 14 * s;
                    float slide = (1 - alpha) * 18 * s;
                    float px = pad - slide, py = colTop - ph;
                    Panel(px, py, leftW, ph, alpha);
                    bool hasPrev = pf.previous >= 0;
                    Label(hasPrev ? "parameters  ·  vs previous" : "parameters", px + 26 * s, py + ph - 34 * s, A(Ink3, alpha));
                    float labelW = 0; foreach (var k in keys) labelW = Mathf.Max(labelW, Measure(sans, Short(k), 16));
                    float bx = px + 26 * s + labelW + 14 * s, bw = leftW - (bx - px) - 120 * s;
                    for (int i = 0; i < keys.Count; i++) {
                        string key = keys[i];
                        float y = py + ph - 58 * s - (i + 1) * row + 10 * s;
                        var range = paramRange[key];
                        float Norm(float v) => range.y > range.x ? (v - range.x) / (range.y - range.x) : 0.5f;
                        float Val(PanelFrame x) => x.candidate >= 0 && M.Runs[x.candidate].parameters.TryGetValue(key, out var o) && o is double d ? Norm((float)d) : float.NaN;
                        float PrevVal(PanelFrame x) => x.previous >= 0 && M.Runs[x.previous].parameters.TryGetValue(key, out var o) && o is double d ? Norm((float)d) : float.NaN;
                        float v = Filter(spring, Val), pv = Filter(ease, PrevVal);
                        Text(sans, Short(key), px + 26 * s, y, 16, A(Ink2, alpha));
                        Rect(bx, y + 2 * s, bw, 7 * s, 3.5f * s, A(Track, alpha));
                        if (!float.IsNaN(v)) Rect(bx, y + 2 * s, Mathf.Max(7 * s, bw * Mathf.Clamp(v, 0, 1.06f)), 7 * s, 3.5f * s, A(Accent, alpha * 0.9f));
                        if (hasPrev && !float.IsNaN(pv)) Rect(bx + bw * Mathf.Clamp01(pv) - 1.5f * s, y - 2 * s, 3 * s, 15 * s, 1.5f * s, A(Ink, alpha * 0.8f));
                        double cv = M.Runs[pf.candidate].parameters[key] is double dd ? dd : double.NaN;
                        Text(mono, Fmt(cv), px + leftW - 70 * s, y, 15, A(Ink, alpha), 1);
                        if (hasPrev && M.Runs[pf.previous].parameters.TryGetValue(key, out var po) && po is double pd) {
                            double dv = cv - pd;
                            string ds = Math.Abs(dv) < 1e-9 ? "·" : (dv > 0 ? "+" : "−") + Fmt(Math.Abs(dv));
                            Text(mono, ds, px + leftW - 22 * s, y, 13, A(Math.Abs(dv) < 1e-9 ? Faint : dv > 0 ? Accent : Best, alpha), 1);
                        }
                    }
                    colTop = py - 16 * s;
                }
            }

            // ---- the candidate's logged metrics (left, under the parameters)
            if (metricKeys.Count > 0) {
                var (alpha, pf) = Presence(x => x.candidate >= 0 && M.Runs[x.candidate].metrics != null && M.Runs[x.candidate].metrics.Count > 0);
                if (pf != null && alpha > 0.003f) {
                    int cols = 3, rows = Mathf.CeilToInt(metricKeys.Count / (float)cols);
                    float cell = (leftW - 52 * s) / cols, rh = 100 * s, ph = 54 * s + rows * rh;
                    float slide = (1 - alpha) * 18 * s;
                    float px = pad - slide, py = Mathf.Max(pad + 70 * s, colTop - ph);
                    Panel(px, py, leftW, ph, alpha);
                    var run = M.Runs[pf.candidate];
                    Label($"run {run.id}  ·  {run.outcome}", px + 26 * s, py + ph - 34 * s, A(Ink3, alpha));
                    for (int i = 0; i < metricKeys.Count; i++) {
                        string key = metricKeys[i];
                        float cxm = px + 26 * s + cell * (i % cols) + cell * 0.5f, cym = py + ph - 54 * s - (i / cols) * rh - rh * 0.5f + 8 * s;
                        var range = metricRange[key];
                        float Val(PanelFrame x) {
                            var md = x.candidate >= 0 ? M.Runs[x.candidate].metrics : null;
                            if (md == null || !md.TryGetValue(key, out var o) || !(o is double d)) return float.NaN;
                            if (key == "gates" && md.TryGetValue("of", out var of) && of is double ofd && ofd > 0) return (float)(d / ofd);
                            return range.y > range.x ? ((float)d - range.x) / (range.y - range.x) : 1f;
                        }
                        float v = Filter(spring, Val);
                        float r = 31 * s;
                        const float a0 = Mathf.PI * 1.25f, sweep = Mathf.PI * 1.5f;
                        Ring(new Vector2(cxm, cym + 8 * s), r, 4.5f * s, a0, a0 - sweep, A(Track, alpha));
                        bool bad = (key == "contacts" || key == "wrong_crossings") && v > 0.001f;
                        if (!float.IsNaN(v) && v > 0.002f) Ring(new Vector2(cxm, cym + 8 * s), r, 4.5f * s, a0, a0 - sweep * Mathf.Clamp(v, 0, 1.04f), A(bad ? Fail : Accent, alpha));
                        Text(mono, MetricText(run, key), cxm, cym + 3 * s, 14, A(Ink, alpha), 0.5f);
                        Text(sans, MetricName(key), cxm, cym - 34 * s, 13, A(Ink3, alpha), 0.5f);
                    }
                }
            }

            // ---- colour legend (bottom left)
            {
                float lw = 300 * s, ph = 86 * s, px = pad, py = pad;
                Panel(px, py, lw + 52 * s, ph, 1f);
                Label("colour · " + cur.colour, px + 26 * s, py + ph - 32 * s, Ink3);
                float bx = px + 26 * s, by = py + 36 * s, bh = 7 * s;
                if (cur.colour == "outcome") {
                    Rect(bx, by, lw * 0.48f, bh, 3.5f * s, Colormaps.Eval(cur.ramp, 0.85f));
                    Rect(bx + lw * 0.52f, by, lw * 0.48f, bh, 3.5f * s, Colormaps.Failure);
                }
                else for (int i = 0; i < 24; i++) Rect(bx + lw * i / 24f, by, lw / 24f + 0.6f * s, bh, i == 0 || i == 23 ? 3.5f * s : 0, Colormaps.Eval(cur.ramp, (i + 0.5f) / 24f));
                Text(mono, cur.legendLo, bx, py + 14 * s, 13, Ink3);
                Text(mono, cur.legendHi, bx + lw, py + 14 * s, 13, Ink3, 1);
            }

            // ---- score history (bottom right)
            Chart(W - pad - 500 * s, pad, 500 * s, 230 * s);

            // ---- compare (top centre)
            {
                var (alpha, pf) = Presence(x => x.compare);
                if (pf != null && alpha > 0.003f) {
                    var p = st.PlanAt(pf.frame);
                    bool two = p.previous >= 0;
                    float pw = 640 * s, ph = (two ? 150 : 88) * s, px = W * 0.5f - pw * 0.5f, py = H - pad - ph + (1 - alpha) * 18 * s;
                    Panel(px, py, pw, ph, alpha);
                    float gp = Filter(ease, x => x.compare ? x.goldProgress : float.NaN), spv = Filter(ease, x => x.compare ? x.silverProgress : float.NaN);
                    HeroRow(px + 26 * s, py + ph - 38 * s, pw - 52 * s, Best, "best", M.Runs[p.hero], gp, pf.goldDone, alpha);
                    if (two) {
                        HeroRow(px + 26 * s, py + ph - 100 * s, pw - 52 * s, Ink2, "previous", M.Runs[p.previous], spv, pf.silverDone, alpha);
                        float dt = M.Runs[p.hero].duration - M.Runs[p.previous].duration;
                        Text(mono, $"Δ {(dt >= 0 ? "+" : "−")}{F(Mathf.Abs(dt), "0.0")} s", px + pw - 26 * s, py + 16 * s, 14, A(Ink3, alpha), 1);
                    }
                }
            }

            // ---- insight and the run clock (bottom centre)
            {
                var (alpha, pf) = Presence(x => !string.IsNullOrEmpty(x.insight) && (x.act == "evolution" || x.act == "compare" || x.act == "spotlight"));
                float cx = W * 0.5f;
                if (pf != null && alpha > 0.003f) {
                    float iw = Measure(sans, pf.insight, 19);
                    float ph = 44 * s, py = pad + 34 * s - (1 - alpha) * 12 * s;
                    Box(new Vector2(cx, py + ph * 0.5f), new Vector2(iw * 0.5f + 24 * s, ph * 0.5f), 0, ph * 0.5f, A(Card, alpha), A(Line, alpha), 1.2f * s, 22 * s, 0.3f * alpha, 0.9f * alpha);
                    Text(sans, pf.insight, cx, py + 14 * s, 19, A(Ink, alpha), 0.5f);
                }
                Text(mono, $"t {F(cur.runTime, "0.0")} s   ×{F(cur.speed, "0.#")}", cx, pad + 6 * s, 14, Ink3, 0.5f);
            }
        }

        void Chip(ref float x, float y, float v, Color c, string label) {
            Dot(new Vector2(x + 5 * s, y + 6 * s), 4.5f * s, c);
            float a = Text(mono, float.IsNaN(v) ? "–" : Mathf.RoundToInt(v).ToString(CultureInfo.InvariantCulture), x + 16 * s, y, 16, Ink);
            float b = Text(sans, label, x + 16 * s + a + 6 * s, y, 14, Ink3);
            x += 16 * s + a + 6 * s + b + 18 * s;
        }

        void HeroRow(float x, float y, float width, Color c, string label, TheaterRun r, float progress, bool done, float alpha) {
            Dot(new Vector2(x + 6 * s, y + 6 * s), 6 * s, A(c, alpha));
            float lw = Label(label, x + 20 * s, y, A(c, alpha));
            Text(mono, r.id, x + 20 * s + Mathf.Max(lw, 92 * s) + 10 * s, y, 15, A(Ink, alpha));
            Text(mono, $"score {F(r.score, "0.00")}   {F(r.duration, "0.0")} s", x + width, y, 15, A(Ink2, alpha), 1);
            float bx = x + 20 * s, bw = width - 20 * s, by = y - 22 * s;
            Rect(bx, by, bw, 6 * s, 3 * s, A(Track, alpha));
            if (!float.IsNaN(progress)) Rect(bx, by, Mathf.Max(6 * s, bw * Mathf.Clamp01(progress)), 6 * s, 3 * s, A(c, alpha * (done ? 1f : 0.85f)));
        }

        void Chart(float px, float py, float pw, float ph) {
            Panel(px, py, pw, ph, 1f);
            var best = st.BestPerIteration;
            float cursor = Filter(ease, pf => pf.cursor);
            float bestSoFar = float.NaN; int bestAt = -1;
            for (int k = 0; k < best.Length && k <= Mathf.FloorToInt(cursor + 0.001f); k++)
                if (!float.IsNaN(best[k]) && (float.IsNaN(bestSoFar) || best[k] > bestSoFar)) { bestSoFar = best[k]; bestAt = k; }
            Label("score by iteration", px + 26 * s, py + ph - 32 * s, Ink3);
            Text(mono, F(bestSoFar, "0.00"), px + pw - 26 * s, py + ph - 38 * s, 26, Best, 1);
            Label("best", px + pw - 26 * s - Measure(mono, F(bestSoFar, "0.00"), 26) - 10 * s, py + ph - 32 * s, A(Best, 0.8f), 1);
            float gx = px + 52 * s, gy = py + 40 * s, gw = pw - 78 * s, gh = ph - 104 * s;
            int n = Mathf.Max(2, chartIters);
            float X(float k) => gx + gw * k / (n - 1);
            float Y(float v) => gy + gh * Mathf.Clamp01((v - scoreLo) / (scoreHi - scoreLo));
            // grid: three score lines and the iteration ticks
            for (int g = 0; g <= 2; g++) {
                float v = Mathf.Lerp(scoreLo, scoreHi, g / 2f), yy = Y(v);
                Seg(new Vector2(gx, yy), new Vector2(gx + gw, yy), 1f * s, A(Line, 0.9f));
                Text(mono, F(v, Mathf.Abs(scoreHi - scoreLo) > 20 ? "0" : "0.0"), gx - 8 * s, yy - 5 * s, 11, Faint, 1);
            }
            for (int k = 0; k < n; k++) {
                bool shown = k <= cursor + 0.001f;
                Dot(new Vector2(X(k), gy - 12 * s), 1.8f * s, shown ? Ink3 : Faint);
            }
            Text(mono, $"{M.MinIteration}", X(0), gy - 30 * s, 11, Faint, 0.5f);
            Text(mono, $"{M.MinIteration + n - 1}", X(n - 1), gy - 30 * s, 11, Faint, 0.5f);
            // every run's score (dots), revealed with its iteration
            foreach (var r in M.Runs) {
                if (!r.valid) continue;
                float k = r.iteration - M.MinIteration;
                float a = Mathf.Clamp01((cursor - k) * 3f + 1f);
                if (a <= 0) continue;
                Dot(new Vector2(X(k), Y(r.score)), 2.6f * s, A(r.Failed ? Fail : Ink2, 0.55f * a));
            }
            // best of each iteration (a line that grows) and the best-so-far step (gold)
            Vector2? prev = null; float run = float.NegativeInfinity; Vector2? prevStep = null;
            for (int k = 0; k < best.Length; k++) {
                if (float.IsNaN(best[k])) continue;
                var pt = new Vector2(X(k), Y(best[k]));
                if (prev.HasValue) {
                    float t = Mathf.Clamp01((cursor - (k - 1)));
                    if (t > 0) {
                        var end = Vector2.Lerp(prev.Value, pt, TheaterCameraRig.Ease(t));
                        Seg(prev.Value, end, 2.4f * s, Accent);
                    }
                }
                if (k <= cursor + 0.001f) {
                    float nv = Mathf.Max(run, best[k]);
                    var sp = new Vector2(X(k), Y(nv));
                    if (prevStep.HasValue) { Seg(prevStep.Value, new Vector2(sp.x, prevStep.Value.y), 1.4f * s, A(Best, 0.75f)); Seg(new Vector2(sp.x, prevStep.Value.y), sp, 1.4f * s, A(Best, 0.75f)); }
                    prevStep = sp; run = nv;
                    Dot(pt, 3.6f * s, Accent);
                }
                prev = pt;
            }
            if (bestAt >= 0) {
                var bp = new Vector2(X(bestAt), Y(best[bestAt]));
                Box(bp, new Vector2(7 * s, 7 * s), 0, 7 * s, Color.clear, Best, 2f * s);
                Dot(bp, 3.6f * s, Best);
            }
            float cx = X(Mathf.Clamp(cursor, 0, n - 1));
            Seg(new Vector2(cx, gy - 4 * s), new Vector2(cx, gy + gh + 4 * s), 1.2f * s, A(Ink, 0.35f));
        }

        static string Short(string k) { int i = k.LastIndexOf('.'); return i >= 0 && i < k.Length - 1 ? k.Substring(i + 1) : k; }
        static string Fmt(double v) => double.IsNaN(v) ? "–" : v.ToString(Math.Abs(v) >= 100 ? "0" : Math.Abs(v) >= 10 ? "0.0" : "0.000", CultureInfo.InvariantCulture);

        static string MetricName(string k) {
            switch (k) {
                case "gates": return "gates";
                case "time_s": return "course time";
                case "clearance_m": return "buoy clearance";
                case "contacts": return "contacts";
                case "wrong_crossings": return "wrong crossings";
                case "cross_track_m": return "cross-track";
                case "smooth": return "smoothness";
                default: return k.Replace('_', ' ');
            }
        }

        static string MetricText(TheaterRun r, string k) {
            var m = r.metrics;
            if (m == null || !m.TryGetValue(k, out var o) || !(o is double v)) return "–";
            switch (k) {
                case "gates": return m.TryGetValue("of", out var of) && of is double ofd ? $"{v:0}/{ofd:0}" : $"{v:0}";
                case "time_s": return v.ToString("0.0", CultureInfo.InvariantCulture) + " s";
                case "clearance_m": case "cross_track_m": return v.ToString("0.00", CultureInfo.InvariantCulture) + " m";
                case "contacts": case "wrong_crossings": return v.ToString("0", CultureInfo.InvariantCulture);
                default: return v.ToString(Math.Abs(v) >= 100 ? "0" : "0.00", CultureInfo.InvariantCulture);
            }
        }
    }
}
