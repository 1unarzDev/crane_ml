using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace Crane.Theater {
    /// <summary>
    /// Static world geometry a study draws over the water (manifest key <c>overlay</c>, <c>crane-theater-overlay/1</c>),
    /// in the trajectory frame, drawn glowing by the theater pass. Shapes:
    ///   <c>disc</c> {x, y, r, fill, rim, curtain}: a filled disc with a bright rim and a curtain rising from it (keep-outs, zones);
    ///   <c>ring</c> {x, y, r, width}; <c>band</c> {points, half_width}: a soft band along a polyline;
    ///   <c>wall</c> {points, height}: a translucent wall along a polyline, fading upward.
    /// Every shape has <c>color</c> ("#rrggbb"), <c>alpha</c>, <c>pulse_hz</c> (0: steady), and is shown only in
    /// <c>acts</c> (indices; absent: all) and, inside an act that shows one iteration at a time, only for its
    /// <c>iterations</c> (absent: all). <c>halo</c> {radius_by_iteration, color, alpha, width, acts, heroes, ghosts}
    /// rings every ghost (and hero) with its iteration's radius. <c>events</c> {kind: {color, radius}} colours event
    /// markers (color "none" hides a kind).
    /// </summary>
    public sealed class TheaterOverlay {
        sealed class Shape {
            public Mesh mesh;
            public HashSet<int> acts, iterations;
            public float pulse;
        }

        readonly List<Shape> shapes = new List<Shape>();
        readonly ITheaterWorld world;
        // halo
        readonly Dictionary<int, float> haloRadius = new Dictionary<int, float>();
        readonly HashSet<int> haloActs;
        readonly Color haloColor;
        readonly float haloWidth;
        readonly bool haloGhosts, haloHeroes;
        readonly Mesh halos;
        readonly Dictionary<string, (Color c, float r, bool hide)> events = new Dictionary<string, (Color, float, bool)>();
        readonly List<Vector3> v = new List<Vector3>();
        readonly List<Color> c = new List<Color>();
        readonly List<Vector2> u = new List<Vector2>();
        readonly List<int> idx = new List<int>();
        bool halosOn;
        public int Shapes => shapes.Count;

        public static Color Hex(object doc, string key, Color fallback) {
            string s = TJson.Str(doc, key);
            return s != null && ColorUtility.TryParseHtmlString(s, out var col) ? col : fallback;
        }

        static HashSet<int> Ints(object doc, string key) {
            var a = TJson.Arr(doc, key);
            if (a == null) return null;
            var set = new HashSet<int>();
            foreach (var x in a) set.Add((int)(double)x);
            return set;
        }

        public TheaterOverlay(Dictionary<string, object> doc, ITheaterWorld world) {
            this.world = world;
            foreach (var o in TJson.Arr(doc, "shapes") ?? new List<object>()) {
                v.Clear(); c.Clear(); u.Clear(); idx.Clear();
                var col = Hex(o, "color", Color.white).linear;
                col.a = (float)TJson.Num(o, "alpha", 0.6);
                switch (TJson.Str(o, "kind", "ring")) {
                    case "disc": Disc(o, col); break;
                    case "ring": Ring(P(o), (float)TJson.Num(o, "r", 1), (float)TJson.Num(o, "width", 0.12), col, 0.06f); break;
                    case "band": Band(Points(o), (float)TJson.Num(o, "half_width", 1), col); break;
                    case "wall": Wall(Points(o), (float)TJson.Num(o, "height", 1.5), col); break;
                    default: continue;
                }
                if (v.Count == 0) continue;
                var m = new Mesh { name = "theater overlay " + TJson.Str(o, "kind", ""), indexFormat = IndexFormat.UInt32 };
                m.SetVertices(v); m.SetColors(c); m.SetUVs(0, u); m.SetTriangles(idx, 0, false);
                m.bounds = new Bounds(Vector3.zero, Vector3.one * 1e5f);
                shapes.Add(new Shape { mesh = m, acts = Ints(o, "acts"), iterations = Ints(o, "iterations"), pulse = (float)TJson.Num(o, "pulse_hz", 0) });
            }
            var h = TJson.Obj(doc, "halo");
            if (h != null) {
                if (TJson.Obj(h, "radius_by_iteration") is Dictionary<string, object> rb)
                    foreach (var kv in rb) if (int.TryParse(kv.Key, out int it) && kv.Value is double d) haloRadius[it] = (float)d;
                haloActs = Ints(h, "acts");
                haloColor = Hex(h, "color", new Color(0.16f, 0.47f, 0.84f)).linear;
                haloColor.a = (float)TJson.Num(h, "alpha", 0.5);
                haloWidth = (float)TJson.Num(h, "width", 0.1);
                haloGhosts = TJson.Bool(h, "ghosts", true); haloHeroes = TJson.Bool(h, "heroes", true);
                halos = new Mesh { name = "theater halos", indexFormat = IndexFormat.UInt32 };
                halos.MarkDynamic();
            }
            if (TJson.Obj(doc, "events") is Dictionary<string, object> ev)
                foreach (var kv in ev) {
                    bool hide = TJson.Str(kv.Value, "color") == "none";
                    events[kv.Key] = (Hex(kv.Value, "color", Colormaps.Gate), (float)TJson.Num(kv.Value, "radius", 0), hide);
                }
        }

        /// <summary>An event kind's marker colour and radius scale from the manifest (false: use the default).</summary>
        public bool EventStyle(string kind, out Color col, out float radius, out bool hide) {
            col = default; radius = 0; hide = false;
            if (!events.TryGetValue(kind ?? "", out var e)) return false;
            col = e.c; radius = e.r; hide = e.hide;
            return true;
        }

        Vector3 W(float x, float y, float lift) {
            var p = world.ToUnity(x, y, 0);
            p.y = world.SurfaceY(p) + 0.15f + lift;   // above the still water's wave crests
            return p;
        }

        static Vector2 P(object o) => new Vector2((float)TJson.Num(o, "x"), (float)TJson.Num(o, "y"));

        static List<Vector2> Points(object o) {
            var pts = new List<Vector2>();
            foreach (var p in TJson.Arr(o, "points") ?? new List<object>())
                if (p is List<object> xy && xy.Count >= 2) pts.Add(new Vector2((float)(double)xy[0], (float)(double)xy[1]));
            return pts;
        }

        void Tri(int a, int b, int d) { idx.Add(a); idx.Add(b); idx.Add(d); }

        void Ring(Vector2 at, float r, float width, Color col, float lift, int seg = 96) {
            int b = v.Count;
            for (int i = 0; i <= seg; i++) {
                float a = 2 * Mathf.PI * i / seg;
                var dir = new Vector2(Mathf.Cos(a), Mathf.Sin(a));
                v.Add(W(at.x + dir.x * (r - width), at.y + dir.y * (r - width), lift)); c.Add(col); u.Add(new Vector2(0, -1));
                v.Add(W(at.x + dir.x * (r + width), at.y + dir.y * (r + width), lift)); c.Add(col); u.Add(new Vector2(0, 1));
                if (i > 0) { int k = b + 2 * i; Tri(k - 2, k - 1, k); Tri(k - 1, k + 1, k); }
            }
        }

        void Disc(object o, Color col) {
            var at = P(o);
            float r = (float)TJson.Num(o, "r", 1), fill = (float)TJson.Num(o, "fill", 0.25), rim = (float)TJson.Num(o, "rim", 0.14);
            float curtain = (float)TJson.Num(o, "curtain", 0);
            const int seg = 96;
            // fill: a fan, alpha = fill (uv.y 0: no edge fade)
            int b = v.Count;
            var fc = col; fc.a = col.a * fill;
            v.Add(W(at.x, at.y, 0.05f)); c.Add(fc); u.Add(Vector2.zero);
            for (int i = 0; i <= seg; i++) {
                float a = 2 * Mathf.PI * i / seg;
                v.Add(W(at.x + r * Mathf.Cos(a), at.y + r * Mathf.Sin(a), 0.05f)); c.Add(fc); u.Add(Vector2.zero);
                if (i > 0) Tri(b, b + i, b + i + 1);
            }
            if (rim > 0) Ring(at, r, rim, col, 0.07f);
            if (curtain > 0) {
                b = v.Count;
                var top = col; top.a = 0f;
                var bottom = col; bottom.a = col.a * 0.8f;
                for (int i = 0; i <= seg; i++) {
                    float a = 2 * Mathf.PI * i / seg;
                    var p0 = W(at.x + r * Mathf.Cos(a), at.y + r * Mathf.Sin(a), 0.05f);
                    v.Add(p0); c.Add(bottom); u.Add(Vector2.zero);
                    v.Add(p0 + Vector3.up * curtain); c.Add(top); u.Add(new Vector2(0, 0.5f));
                    if (i > 0) { int k = b + 2 * i; Tri(k - 2, k - 1, k); Tri(k - 1, k + 1, k); }
                }
            }
        }

        void Band(List<Vector2> pts, float hw, Color col) {
            if (pts.Count < 2) return;
            int b = v.Count;
            for (int i = 0; i < pts.Count; i++) {
                Vector2 a = pts[Mathf.Max(0, i - 1)], z = pts[Mathf.Min(pts.Count - 1, i + 1)];
                Vector2 t = (z - a);
                if (t.sqrMagnitude < 1e-8f) t = Vector2.right;
                t.Normalize();
                var n = new Vector2(-t.y, t.x) * hw;
                v.Add(W(pts[i].x - n.x, pts[i].y - n.y, 0.045f)); c.Add(col); u.Add(new Vector2(0, -1));
                v.Add(W(pts[i].x + n.x, pts[i].y + n.y, 0.045f)); c.Add(col); u.Add(new Vector2(0, 1));
                if (i > 0) { int k = b + 2 * i; Tri(k - 2, k - 1, k); Tri(k - 1, k + 1, k); }
            }
        }

        void Wall(List<Vector2> pts, float height, Color col) {
            if (pts.Count < 2) return;
            int b = v.Count;
            var top = col; top.a = 0;
            for (int i = 0; i < pts.Count; i++) {
                var p0 = W(pts[i].x, pts[i].y, 0.05f);
                v.Add(p0); c.Add(col); u.Add(Vector2.zero);
                v.Add(p0 + Vector3.up * height); c.Add(top); u.Add(new Vector2(0, 0.5f));
                if (i > 0) { int k = b + 2 * i; Tri(k - 2, k - 1, k); Tri(k - 1, k + 1, k); }
            }
        }

        /// <summary>The iteration an act shows at this frame: its evolution step, or its only iteration (else -1).</summary>
        public static int ActiveIteration(FrameState f) =>
            f.iteration >= 0 ? f.iteration : f.plan.iterations.Length == 1 ? f.plan.iterations[0] : -1;

        /// <summary>In the act, and for its iteration: the one it shows, else (a compare, a fleet of several) one of
        /// the act's own iterations.</summary>
        static bool Visible(HashSet<int> acts, HashSet<int> iterations, int act, int it, int[] actIterations) {
            if (acts != null && !acts.Contains(act)) return false;
            if (iterations == null) return true;
            if (it >= 0) return iterations.Contains(it);
            if (actIterations == null) return true;
            foreach (int k in actIterations) if (iterations.Contains(k)) return true;
            return false;
        }

        readonly List<(Mesh m, float gain)> draw = new List<(Mesh, float)>();

        public void Prepare(FrameState f, TheaterState st, float fps) {
            draw.Clear();
            int it = ActiveIteration(f);
            float tv = f.frame / Mathf.Max(1f, fps);
            foreach (var s in shapes)
                if (Visible(s.acts, s.iterations, f.act, it, f.plan?.iterations))
                    draw.Add((s.mesh, s.pulse > 0 ? 0.7f + 0.3f * Mathf.Sin(2 * Mathf.PI * s.pulse * tv) : 1f));
            halosOn = false;
            if (halos == null || !Visible(haloActs, null, f.act, -1, null)) return;
            v.Clear(); c.Clear(); u.Clear(); idx.Clear();
            void Add(GhostDraw g, float alphaScale) {
                int rit = st.M.Runs[g.run].iteration;
                if (!haloRadius.TryGetValue(rit, out float r) || r <= 0) return;
                var col = haloColor; col.a *= alphaScale;
                var en = new Vector2(g.pose.x, g.pose.y);
                Ring(en, r, haloWidth, col, 0.06f, 48);
            }
            if (haloGhosts) foreach (var g in f.ghosts) Add(g, Mathf.Clamp01(g.color.a / 0.9f));
            if (haloHeroes) foreach (var g in f.heroes) Add(g, 1.6f);
            halos.Clear();
            if (v.Count == 0) return;
            halos.SetVertices(v); halos.SetColors(c); halos.SetUVs(0, u); halos.SetTriangles(idx, 0, false);
            halos.bounds = new Bounds(Vector3.zero, Vector3.one * 1e5f);
            halosOn = true;
        }

        public void Draw(CommandBuffer cmd, Material mat, MaterialPropertyBlock props) {
            props.SetFloat("_Now", 1e9f); props.SetFloat("_Tail", 0f); props.SetFloat("_Floor", 1f);
            foreach (var (m, gain) in draw) {
                props.SetFloat("_Gain", gain);
                cmd.DrawMesh(m, Matrix4x4.identity, mat, 0, 1, props);
            }
            if (halosOn) { props.SetFloat("_Gain", 1f); cmd.DrawMesh(halos, Matrix4x4.identity, mat, 0, 1, props); }
        }
    }
}
