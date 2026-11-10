using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using UnityEngine.Rendering;

namespace Crane.Theater.Aerial {
    public struct DetBox { public Rect px; public string label, sub; public bool occluded; }

    public sealed class OverlayModel {
        public float fade;                                     // black veil 0..1
        public string tagLeft = "", tagRight = "", clock = "";
        public string title, titleSub, lower, lowerSub;
        public float titleAlpha, lowerAlpha;
        public readonly List<DetBox> boxes = new List<DetBox>();
        public bool split;                                     // left = third person, right = onboard
        public string leftLabel = "", rightLabel = "";
        public bool showFrameTags = true;
    }

    /// <summary>The demo overlay: shot tags, title card, lower third, detection boxes, split-screen labels and the fade
    /// veil. Drawn into the frame after the camera with the theater HUD's shader and the same font path
    /// (<see cref="Crane.Theater.HudRenderer"/>'s primitives are private, so the few primitives needed are repeated here).</summary>
    public sealed class DemoOverlay : IDisposable {
        readonly Material mat; readonly Font font;
        readonly Mesh shapes = new Mesh { name = "aerial shapes" }, text = new Mesh { name = "aerial text" };
        readonly List<Vector3> sv = new List<Vector3>(), tv = new List<Vector3>();
        readonly List<Color> sc = new List<Color>(), tc = new List<Color>();
        readonly List<Vector2> tu = new List<Vector2>();
        readonly List<int> si = new List<int>(), ti = new List<int>();
        readonly CommandBuffer cmd = new CommandBuffer { name = "aerial overlay" };
        readonly int w, h; readonly float s;
        bool rebuilt;
        static readonly int[] Sizes = { 15, 18, 22, 26, 32, 44, 72 };
        const string Charset = " !\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~·–°×•";
        static readonly Color Ink = new Color(0.96f, 0.97f, 1f), Dim = new Color(0.72f, 0.78f, 0.86f), Panel = new Color(0.02f, 0.04f, 0.07f, 0.55f),
            Accent = new Color(1f, 0.78f, 0.25f), Det = new Color(0.25f, 1f, 0.55f);

        public DemoOverlay(int width, int height) {
            w = width; h = height; s = height / 1080f;
            var shader = Resources.Load<Shader>("CraneTheaterHud");
            if (shader == null) throw new InvalidOperationException("aerial: shader Resources/CraneTheaterHud missing");
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

        void Quad(Vector2 a, Vector2 b, Vector2 c, Vector2 d, Color ca, Color cb, Color cc, Color cd) {
            int k = sv.Count;
            sv.Add(a); sv.Add(b); sv.Add(c); sv.Add(d);
            sc.Add(Lin(ca)); sc.Add(Lin(cb)); sc.Add(Lin(cc)); sc.Add(Lin(cd));
            si.Add(k); si.Add(k + 1); si.Add(k + 2); si.Add(k); si.Add(k + 2); si.Add(k + 3);
        }
        static Color Lin(Color c) { var l = c.linear; l.a = c.a; return l; }
        void Rect(float x, float y, float rw, float rh, Color c) => Quad(new Vector2(x, y), new Vector2(x + rw, y), new Vector2(x + rw, y + rh), new Vector2(x, y + rh), c, c, c, c);
        void Frame(float x, float y, float rw, float rh, float t, Color c) {
            Rect(x, y, rw, t, c); Rect(x, y + rh - t, rw, t, c); Rect(x, y, t, rh, c); Rect(x + rw - t, y, t, rh, c);
        }

        float Measure(string str, int size, FontStyle style) {
            float x = 0;
            foreach (char ch in str) if (font.GetCharacterInfo(ch, out var ci, Px(size), style)) x += ci.advance;
            return x;
        }

        float Text(string str, float x, float y, int size, Color c, float align = 0, FontStyle style = FontStyle.Normal) {
            if (string.IsNullOrEmpty(str)) return 0;
            font.RequestCharactersInTexture(str, Px(size), style);
            float width = Measure(str, size, style);
            x = Mathf.Round(x - width * align); y = Mathf.Round(y);
            Glyphs(str, x + 1.5f * s, y - 1.5f * s, size, new Color(0, 0, 0, 0.6f * c.a), style);
            Glyphs(str, x, y, size, c, style);
            return width;
        }

        void Glyphs(string str, float x, float y, int size, Color c, FontStyle style) {
            Color lc = Lin(c);
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

        public void Draw(RenderTexture target, OverlayModel m) {
            for (int pass = 0; pass < 3; pass++) { rebuilt = false; Layout(m); if (!rebuilt) break; }
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

        void Layout(OverlayModel m) {
            sv.Clear(); sc.Clear(); si.Clear(); tv.Clear(); tc.Clear(); tu.Clear(); ti.Clear();
            float W = w, H = h, pad = 36 * s;

            if (m.split) {
                Rect(W * 0.5f - 2 * s, 0, 4 * s, H, new Color(1, 1, 1, 0.9f));
                Label(pad, pad + 30 * s, m.leftLabel, Ink);
                Label(W * 0.5f + pad, pad + 30 * s, m.rightLabel, Det);
            }

            // detection boxes: pixel rects with origin top-left; the overlay's origin is bottom-left
            foreach (var b in m.boxes) {
                var c = b.occluded ? new Color(Det.r, Det.g, Det.b, 0.55f) : Det;
                float x = b.px.x, y = H - b.px.yMax, bw = b.px.width, bh = b.px.height, t = Mathf.Max(2f, 2.5f * s), tick = Mathf.Min(bw, bh) * 0.22f;
                Corner(x, y, bw, bh, t, tick, c);
                float ly = y + bh + 8 * s;
                float lw = Mathf.Max(Measure(b.label ?? "", 18, FontStyle.Bold), Measure(b.sub ?? "", 15, FontStyle.Normal)) + 20 * s;
                Rect(x, ly, lw, 46 * s, new Color(0.02f, 0.06f, 0.04f, 0.7f));
                Rect(x, ly, 4 * s, 46 * s, c);
                Text(b.label, x + 12 * s, ly + 24 * s, 18, c, 0, FontStyle.Bold);
                Text(b.sub, x + 12 * s, ly + 6 * s, 15, Dim);
            }

            if (m.showFrameTags) {
                Text("CRANE  AERIAL", pad, H - pad - 4 * s, 15, Dim, 0, FontStyle.Bold);
                Text(m.tagLeft, pad, H - pad - 28 * s, 18, Ink, 0, FontStyle.Bold);
                Text(m.tagRight, W - pad, H - pad - 4 * s, 15, Dim, 1, FontStyle.Bold);
                Text(m.clock, W - pad, pad, 15, Dim, 1);
            }

            if (m.lowerAlpha > 0.01f && !string.IsNullOrEmpty(m.lower)) {
                float a = m.lowerAlpha, slide = (1 - a) * 60 * s;
                float lw = Mathf.Max(Measure(m.lower, 32, FontStyle.Bold), Measure(m.lowerSub ?? "", 22, FontStyle.Normal)) + 56 * s;
                float x = pad - slide, y = 120 * s;
                Rect(x, y, lw, 84 * s, new Color(0.02f, 0.04f, 0.07f, 0.72f * a));
                Rect(x, y, 6 * s, 84 * s, new Color(Accent.r, Accent.g, Accent.b, a));
                Text(m.lower, x + 28 * s, y + 46 * s, 32, new Color(Ink.r, Ink.g, Ink.b, a), 0, FontStyle.Bold);
                Text(m.lowerSub, x + 28 * s, y + 16 * s, 22, new Color(Dim.r, Dim.g, Dim.b, a));
            }

            if (m.titleAlpha > 0.01f && !string.IsNullOrEmpty(m.title)) {
                float a = m.titleAlpha;
                Rect(0, 0, W, H, new Color(0.01f, 0.02f, 0.03f, 0.55f * a));
                Rect(W * 0.5f - 60 * s, H * 0.5f - 14 * s, 120 * s, 3 * s, new Color(Accent.r, Accent.g, Accent.b, a));
                Text(m.title, W * 0.5f, H * 0.5f + 20 * s, 72, new Color(Ink.r, Ink.g, Ink.b, a), 0.5f, FontStyle.Bold);
                Text(m.titleSub, W * 0.5f, H * 0.5f - 62 * s, 26, new Color(Dim.r, Dim.g, Dim.b, a), 0.5f);
            }

            if (m.fade > 0.001f) Rect(0, 0, W, H, new Color(0, 0, 0, Mathf.Clamp01(m.fade)));
        }

        void Label(float x, float y, string t, Color c) {
            if (string.IsNullOrEmpty(t)) return;
            float lw = Measure(t, 18, FontStyle.Bold) + 28 * s;
            Rect(x - 14 * s, y - 10 * s, lw, 40 * s, Panel);
            Text(t, x, y + 2 * s, 18, c, 0, FontStyle.Bold);
        }

        void Corner(float x, float y, float bw, float bh, float t, float len, Color c) {
            Rect(x, y, len, t, c); Rect(x, y, t, len, c);
            Rect(x + bw - len, y, len, t, c); Rect(x + bw - t, y, t, len, c);
            Rect(x, y + bh - t, len, t, c); Rect(x, y + bh - len, t, len, c);
            Rect(x + bw - len, y + bh - t, len, t, c); Rect(x + bw - t, y + bh - len, t, len, c);
            Frame(x, y, bw, bh, Mathf.Max(1f, t * 0.4f), new Color(c.r, c.g, c.b, c.a * 0.35f));
        }
    }
}
