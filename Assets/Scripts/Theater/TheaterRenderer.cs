using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

namespace Crane.Theater {
    [StructLayout(LayoutKind.Sequential)]
    struct GhostGpu { public Vector3 pos; public float yaw, pitch, roll, scale, pad; public Vector4 color; }

    [StructLayout(LayoutKind.Sequential)]
    struct MarkerGpu { public Vector3 pos; public float radius; public Vector4 color; public float age; public Vector3 pad; }

    /// <summary>HDRP custom pass (after post-processing, against the camera depth) that draws what
    /// <see cref="TheaterRenderer"/> prepared for the frame: the path-density and failure-heat layer, trails,
    /// comet tails, ghosts, hero shells and event markers.</summary>
    [Serializable]
    public sealed class TheaterPass : CustomPass {
        [NonSerialized] public TheaterRenderer R;
        protected override void Execute(CustomPassContext ctx) {
            if (R == null || !R.Ready) return;
            var cam = ctx.hdCamera.camera;
            if (R.Camera != null && cam != R.Camera) return;
            R.Draw(ctx.cmd, cam, ctx.cameraColorBuffer, ctx.cameraDepthBuffer);
        }
    }

    /// <summary>The theater's GPU side: buffers and meshes for one frame, rebuilt from a <see cref="FrameState"/>.</summary>
    public sealed class TheaterRenderer : IDisposable {
        public bool Ready;
        public Camera Camera;
        readonly TheaterState st;
        readonly ITheaterRobot robot;
        readonly Material mat;
        readonly Mesh ghostMesh, chevronMesh;
        bool chevrons;
        GraphicsBuffer ghostBuf, heroBuf, markerBuf;
        int ghostCount, heroCount, markerCount;
        readonly GhostGpu[] ghostData, heroData = new GhostGpu[4];
        MarkerGpu[] markerData = new MarkerGpu[256];
        readonly Mesh comets;
        readonly List<Vector3> cv = new List<Vector3>();
        readonly List<Color> cc = new List<Color>();
        readonly List<Vector2> cu = new List<Vector2>();
        readonly List<int> ci = new List<int>();
        readonly MaterialPropertyBlock ghostProps = new MaterialPropertyBlock(), heroProps = new MaterialPropertyBlock(),
            cometProps = new MaterialPropertyBlock(), trailProps = new MaterialPropertyBlock(), overlayProps = new MaterialPropertyBlock();
        // per act
        int trailAct = -1, historyKey = int.MinValue;
        Mesh trails, history;
        readonly MaterialPropertyBlock historyProps = new MaterialPropertyBlock();
        float trailNow;
        bool cometsOn;
        // heat
        Texture2D heatTex;
        int hw, hh;
        float cell;
        Vector4 heatRect;
        int[] density, fail;
        float[] heatPixels;
        int heatAct = -1, heatFrame = -1;
        int[] nextSample;
        bool[] failDone;
        float densityLogMax = 1f, failMax = 1f;
        bool heatOn;
        // wash (manifest look.wash {color, alpha, lift, vignette, vignette_start}): a tinted plane over the water
        bool washOn;
        Color washColor;
        float washLift, washVignette, washStart;
        Vector3 washFar;   // look.wash.far [start, full] x the camera's distance to the water ahead; corner: vignette level (0: off)
        public readonly Dictionary<string, int> Stats = new Dictionary<string, int>();

        public TheaterRenderer(TheaterState state, ITheaterRobot robot) {
            st = state; this.robot = robot;
            var shader = Resources.Load<Shader>("CraneTheaterWorld");
            if (shader == null) throw new InvalidOperationException("theater: shader Resources/CraneTheaterWorld missing");
            mat = new Material(shader) { name = "theater world", hideFlags = HideFlags.DontSave };
            ghostMesh = robot.GhostMesh();
            chevronMesh = Chevron();
            int max = Mathf.Max(1, st.M.Runs.Count);
            ghostData = new GhostGpu[max];
            ghostBuf = new GraphicsBuffer(GraphicsBuffer.Target.Structured, max, Marshal.SizeOf<GhostGpu>());
            heroBuf = new GraphicsBuffer(GraphicsBuffer.Target.Structured, heroData.Length, Marshal.SizeOf<GhostGpu>());
            markerBuf = new GraphicsBuffer(GraphicsBuffer.Target.Structured, markerData.Length, Marshal.SizeOf<MarkerGpu>());
            comets = new Mesh { name = "theater comets", indexFormat = IndexFormat.UInt32 };
            comets.MarkDynamic();
            ghostProps.SetBuffer("_Ghosts", ghostBuf); ghostProps.SetFloat("_Core", 0.22f); ghostProps.SetFloat("_Rim", 1.6f);
            heroProps.SetBuffer("_Ghosts", heroBuf); heroProps.SetFloat("_Core", 0.12f); heroProps.SetFloat("_Rim", 2.2f);
            HeatInit();
            var wash = TJson.Obj(TJson.Obj(st.M.Raw, "look"), "wash");
            if (wash != null) {
                washOn = true;
                washColor = TheaterOverlay.Hex(wash, "color", new Color(0.04f, 0.07f, 0.13f)).linear;
                washColor.a = (float)TJson.Num(wash, "alpha", 0.7);
                washLift = (float)TJson.Num(wash, "lift", 0.3);
                washVignette = (float)TJson.Num(wash, "vignette", 0);
                washStart = (float)TJson.Num(wash, "vignette_start", 0.6);
                var far = TJson.Arr(wash, "far");
                if (far != null && far.Count == 2) washFar = new Vector3(Convert.ToSingle(far[0]), Convert.ToSingle(far[1]), 0);
                washFar.z = (float)TJson.Num(wash, "corner", 0);
            }
        }

        public void Dispose() {
            ghostBuf?.Dispose(); heroBuf?.Dispose(); markerBuf?.Dispose();
            ghostBuf = heroBuf = markerBuf = null;
        }

        // ------------------------------------------------------------------ per frame (CPU)

        public void Prepare(FrameState f) {
            var p = f.plan;
            ghostCount = Mathf.Min(f.ghosts.Count, ghostData.Length);
            for (int i = 0; i < ghostCount; i++) ghostData[i] = Gpu(f.ghosts[i]);
            ghostBuf.SetData(ghostData, 0, 0, ghostCount);

            heroCount = Mathf.Min(f.heroes.Count, heroData.Length);
            for (int i = 0; i < heroCount; i++) {
                var g = f.heroes[i];
                g.scale = 1.12f;
                var c = g.color; c.a = 0.55f; g.color = c;
                heroData[i] = Gpu(g);
            }
            if (heroCount > 0) heroBuf.SetData(heroData, 0, 0, heroCount);

            var marks = new List<MarkerDraw>(f.markers);
            foreach (var h in f.heroes) marks.Add(new MarkerDraw { pos = h.pos, age = 0.35f, radius = 1.0f, color = h.color * new Color(1, 1, 1, 0.7f) });
            markerCount = Mathf.Min(marks.Count, markerData.Length);
            for (int i = 0; i < markerCount; i++)
                markerData[i] = new MarkerGpu { pos = marks[i].pos, radius = marks[i].radius, age = marks[i].age, color = marks[i].color };
            if (markerCount > 0) markerBuf.SetData(markerData, 0, 0, markerCount);

            chevrons = p.act.ghostShape == "chevron";
            ghostProps.SetFloat("_Core", p.act.ghostCore >= 0 ? p.act.ghostCore : 0.22f);
            ghostProps.SetFloat("_Rim", p.act.ghostRim >= 0 ? p.act.ghostRim : 1.6f);
            cometsOn = p.act.trails == "comet" || p.act.trails == "top";
            if (cometsOn) BuildComets(f);
            if (trailAct != p.index) BuildTrails(p);
            trailNow = st.ActRunTime(p, f.tau, out _);
            int key = p.act.history ? p.index * 100000 + st.CurrentIteration(p, f.tau) : int.MinValue;
            if (key != historyKey) BuildHistory(p, f.tau, key);
            heatOn = p.act.heat;
            if (heatOn) AdvanceHeat(f);
            st.Overlay?.Prepare(f, st, st.S.Fps);
            Ready = true;
        }

        /// <summary>A flat heading arrow (+z forward, 1.5 m long at scale 1) lifted above the still water's crests, both
        /// faces, normals up: many of them overlapping read as arrows, not as a stack of translucent hulls.</summary>
        static Mesh Chevron() {
            const float y = 0.25f;
            var v = new[] { new Vector3(0, y, 0.9f), new Vector3(0.55f, y, -0.6f), new Vector3(0, y, -0.25f), new Vector3(-0.55f, y, -0.6f) };
            var m = new Mesh { name = "theater chevron" };
            m.SetVertices(v);
            m.SetNormals(new[] { Vector3.up, Vector3.up, Vector3.up, Vector3.up });
            m.SetTriangles(new[] { 0, 1, 2, 0, 2, 3, 0, 2, 1, 0, 3, 2 }, 0);
            m.bounds = new Bounds(Vector3.zero, Vector3.one * 4f);
            return m;
        }

        static GhostGpu Gpu(GhostDraw g) => new GhostGpu {
            pos = g.pos, yaw = g.yaw, pitch = g.pitch, roll = g.roll, scale = g.scale,
            color = new Vector4(g.color.r, g.color.g, g.color.b, g.color.a)
        };

        void Ribbon(List<(Vector3 p, float u)> pts, float width, Color c, bool taper, float uScale) {
            if (pts.Count < 2) return;
            int b = cv.Count;
            for (int i = 0; i < pts.Count; i++) {
                Vector3 a = pts[Mathf.Max(0, i - 1)].p, z = pts[Mathf.Min(pts.Count - 1, i + 1)].p;
                Vector3 tan = z - a; tan.y = 0;
                if (tan.sqrMagnitude < 1e-8f) tan = Vector3.forward;
                Vector3 side = Vector3.Cross(Vector3.up, tan.normalized);
                float w = width * 0.5f * (taper ? Mathf.Lerp(0.15f, 1f, i / (float)(pts.Count - 1)) : 1f);
                Vector3 p = pts[i].p + Vector3.up * 0.2f;   // above the still water's wave crests
                cv.Add(p - side * w); cv.Add(p + side * w);
                cc.Add(c); cc.Add(c);
                cu.Add(new Vector2(pts[i].u * uScale, -1)); cu.Add(new Vector2(pts[i].u * uScale, 1));
                if (i > 0) { int k = b + 2 * i; ci.Add(k - 2); ci.Add(k - 1); ci.Add(k); ci.Add(k - 1); ci.Add(k + 1); ci.Add(k); }
            }
        }

        void BuildComets(FrameState f) {
            cv.Clear(); cc.Clear(); cu.Clear(); ci.Clear();
            var pack = st.M.Pack;
            float tail = f.plan.act.tail;
            var pts = new List<(Vector3, float)>(64);
            foreach (var g in f.ghosts) {
                float head = g.tr;   // already clamped to the run
                float from = Mathf.Max(0f, head - tail);
                if (head - from < 1e-3f) continue;
                pts.Clear();
                int k0 = Mathf.CeilToInt(from * pack.Hz), k1 = Mathf.FloorToInt(head * pack.Hz);
                pts.Add((st.UnityPos(pack.At(g.run, from)), -(head - from)));
                for (int k = k0; k <= k1; k++) {
                    float tk = k / pack.Hz;
                    if (tk <= from || tk >= head) continue;
                    pts.Add((st.UnityPos(pack.Sample(g.run, k)), -(head - tk)));
                }
                pts.Add((g.pos, 0f));
                var c = g.color; c.a = Mathf.Clamp01(g.color.a * 1.1f * f.plan.act.trailAlpha);
                Ribbon(pts, 0.32f * g.scale, c, true, 1f);
            }
            comets.Clear();
            comets.SetVertices(cv); comets.SetColors(cc); comets.SetUVs(0, cu); comets.SetTriangles(ci, 0, false);
            comets.bounds = new Bounds(Vector3.zero, Vector3.one * 1e5f);
            cometProps.SetFloat("_Now", 0f); cometProps.SetFloat("_Tail", tail); cometProps.SetFloat("_Floor", 0f); cometProps.SetFloat("_Gain", 0.9f);
            Stats["comet_vertices"] = cv.Count;
        }

        /// <summary>history acts: every run of an earlier iteration than the current one, as a full dimmed path
        /// (the newest past iteration at past_alpha, each older one x past_decay), rebuilt when the iteration changes.</summary>
        void BuildHistory(ActPlan p, float tau, int key) {
            historyKey = key;
            if (history != null) UnityEngine.Object.Destroy(history);
            history = null;
            if (!p.act.history) return;
            int cur = st.CurrentIteration(p, tau);
            cv.Clear(); cc.Clear(); cu.Clear(); ci.Clear();
            var pack = st.M.Pack;
            var pts = new List<(Vector3, float)>();
            float n0 = Mathf.Sqrt(30f);
            foreach (var r in st.M.Runs) {
                if (r.iteration >= cur || r.iteration < st.M.MinIteration) continue;
                float u = st.M.MaxIteration > st.M.MinIteration ? (r.iteration - st.M.MinIteration) / (float)(st.M.MaxIteration - st.M.MinIteration) : 1f;
                var c = Colormaps.Eval(p.act.ramp, u).linear;
                c.a = p.act.pastAlpha * 0.5f * Mathf.Pow(p.act.pastDecay, cur - 1 - r.iteration) * p.act.trailAlpha;
                if (c.a < 0.01f) continue;
                pts.Clear();
                int n = pack.Samples(r.index), step = Mathf.Max(1, Mathf.RoundToInt(pack.Hz / 5f));
                for (int k = 0; k < n; k += step) pts.Add((st.UnityPos(pack.Sample(r.index, k)), -1e6f));
                pts.Add((st.UnityPos(pack.Sample(r.index, n - 1)), -1e6f));
                Ribbon(pts, 0.12f, c, false, 1f);
            }
            if (cv.Count == 0) return;
            history = new Mesh { name = "theater history", indexFormat = IndexFormat.UInt32 };
            history.SetVertices(cv); history.SetColors(cc); history.SetUVs(0, cu); history.SetTriangles(ci, 0, false);
            history.bounds = new Bounds(Vector3.zero, Vector3.one * 1e5f);
            historyProps.SetFloat("_Now", 0f); historyProps.SetFloat("_Tail", 0f); historyProps.SetFloat("_Floor", 1f); historyProps.SetFloat("_Gain", 1f);
        }

        void BuildTrails(ActPlan p) {
            trailAct = p.index;
            if (trails != null) UnityEngine.Object.Destroy(trails);
            trails = null;
            var runs = new List<(int run, Color c, float w)>();
            if (p.hero >= 0) runs.Add((p.hero, Colormaps.Gold.linear, 0.34f));
            if (p.previous >= 0) runs.Add((p.previous, Colormaps.Silver.linear, 0.28f));
            for (int i = 0; i < p.trailRuns.Count; i++) {
                var r = st.M.Runs[p.trailRuns[i]];
                float u = p.act.colour == "iteration"
                    ? (st.M.MaxIteration > st.M.MinIteration ? (r.iteration - st.M.MinIteration) / (float)(st.M.MaxIteration - st.M.MinIteration) : 1f)
                    : p.scoreHi > p.scoreLo ? (r.score - p.scoreLo) / (p.scoreHi - p.scoreLo) : 1f;
                var c = Colormaps.Eval(p.act.ramp, u).linear;
                c.a = 0.55f * Mathf.Clamp(Mathf.Sqrt(24f / p.trailRuns.Count), 0.12f, 1f) * p.act.trailAlpha;   // many trails add up to white
                runs.Add((r.index, c, 0.16f));
            }
            // under_iteration: that iteration's full paths, dimmed, drawn from the act's first frame (time -1e6)
            var under = new HashSet<int>();
            if (p.act.under != int.MinValue)
                foreach (var r in st.M.Runs)
                    if (r.iteration == p.act.under) {
                        float u = st.M.MaxIteration > st.M.MinIteration ? (r.iteration - st.M.MinIteration) / (float)(st.M.MaxIteration - st.M.MinIteration) : 1f;
                        var c = (p.act.colour == "iteration" ? Colormaps.Eval(p.act.ramp, u) : Colormaps.Silver).linear;
                        c.a = p.act.underAlpha;
                        runs.Add((r.index, c, 0.12f)); under.Add(r.index);
                    }
            if (runs.Count == 0) return;
            cv.Clear(); cc.Clear(); cu.Clear(); ci.Clear();
            var pack = st.M.Pack;
            var pts = new List<(Vector3, float)>();
            foreach (var (run, c, w) in runs) {
                pts.Clear();
                float sg = p.staggerByRun[run];
                int n = pack.Samples(run), step = Mathf.Max(1, Mathf.RoundToInt(pack.Hz / 5f));
                bool all = under.Contains(run);
                for (int k = 0; k < n; k += step) pts.Add((st.UnityPos(pack.Sample(run, k)), all ? -1e6f : k / pack.Hz + sg));
                if ((n - 1) % step != 0) pts.Add((st.UnityPos(pack.Sample(run, n - 1)), all ? -1e6f : (n - 1) / pack.Hz + sg));
                Ribbon(pts, w, c, false, 1f);
            }
            trails = new Mesh { name = "theater trails", indexFormat = IndexFormat.UInt32 };
            trails.SetVertices(cv); trails.SetColors(cc); trails.SetUVs(0, cu); trails.SetTriangles(ci, 0, false);
            trails.bounds = new Bounds(Vector3.zero, Vector3.one * 1e5f);
            trailProps.SetFloat("_Tail", 0f); trailProps.SetFloat("_Floor", 1f); trailProps.SetFloat("_Gain", 1f);
            Stats["trail_vertices"] = cv.Count;
        }

        // ------------------------------------------------------------------ heat (integer accumulation: order-free, exact)

        void HeatInit() {
            var b = st.World.Bounds;
            float ext = Mathf.Max(b.size.x, b.size.z);
            cell = Mathf.Max(0.12f, ext / 1024f);
            hw = Mathf.Clamp(Mathf.CeilToInt(b.size.x / cell), 8, 1024);
            hh = Mathf.Clamp(Mathf.CeilToInt(b.size.z / cell), 8, 1024);
            heatRect = new Vector4(b.min.x, b.min.z, b.min.x + hw * cell, b.min.z + hh * cell);
            density = new int[hw * hh]; fail = new int[hw * hh]; heatPixels = new float[hw * hh * 2];
            nextSample = new int[st.M.Runs.Count]; failDone = new bool[st.M.Runs.Count];
            heatTex = new Texture2D(hw, hh, TextureFormat.RGFloat, false, true) { wrapMode = TextureWrapMode.Clamp, filterMode = FilterMode.Bilinear, name = "theater heat" };
        }

        void Splat(int[] grid, Vector3 p, int weight) {
            float fx = (p.x - heatRect.x) / cell - 0.5f, fz = (p.z - heatRect.y) / cell - 0.5f;
            int x = Mathf.FloorToInt(fx), z = Mathf.FloorToInt(fz);
            int ux = Mathf.Clamp(Mathf.RoundToInt((fx - x) * 16), 0, 16), uz = Mathf.Clamp(Mathf.RoundToInt((fz - z) * 16), 0, 16);
            void Add(int xx, int zz, int w) { if (xx >= 0 && zz >= 0 && xx < hw && zz < hh && w > 0) grid[zz * hw + xx] += w; }
            Add(x, z, (16 - ux) * (16 - uz) * weight); Add(x + 1, z, ux * (16 - uz) * weight);
            Add(x, z + 1, (16 - ux) * uz * weight); Add(x + 1, z + 1, ux * uz * weight);
        }

        void SplatDisc(int[] grid, Vector3 p, float radius, int weight) {
            int r = Mathf.CeilToInt(radius / cell);
            int cx = Mathf.FloorToInt((p.x - heatRect.x) / cell), cz = Mathf.FloorToInt((p.z - heatRect.y) / cell);
            for (int dz = -r; dz <= r; dz++)
                for (int dx = -r; dx <= r; dx++) {
                    int x = cx + dx, z = cz + dz;
                    if (x < 0 || z < 0 || x >= hw || z >= hh) continue;
                    float d2 = (dx * dx + dz * dz) * cell * cell / (radius * radius);
                    if (d2 > 1f) continue;
                    grid[z * hw + x] += Mathf.RoundToInt(weight * Mathf.Exp(-3f * d2));
                }
        }

        IEnumerable<int> HeatRuns(ActPlan p, int iteration) =>
            iteration >= 0 && p.byIteration.TryGetValue(iteration, out var l) ? l : p.runs;

        /// <summary>Accumulate every sample whose run time has been reached (exact integers, so the layer at a frame
        /// does not depend on which frames were rendered before it).</summary>
        void AdvanceHeat(FrameState f) {
            var p = f.plan;
            if (heatAct != p.index || f.frame < heatFrame) {
                Array.Clear(density, 0, density.Length); Array.Clear(fail, 0, fail.Length);
                Array.Clear(nextSample, 0, nextSample.Length); Array.Clear(failDone, 0, failDone.Length);
                heatAct = p.index;
                // normalisation from the act's full accumulation (constant over the act: no pumping)
                var full = new int[density.Length]; var ff = new int[fail.Length];
                foreach (int run in p.runs) {
                    int n = st.M.Pack.Samples(run);
                    for (int k = 0; k < n; k++) Splat(full, st.UnityPos(st.M.Pack.Sample(run, k)), 1);
                    if (st.M.Runs[run].Failed) SplatDisc(ff, st.UnityPos(st.M.Pack.Sample(run, n - 1)), 1.6f, 256);
                }
                int dm = 1, fm = 1;
                foreach (int v in full) dm = Math.Max(dm, v);
                foreach (int v in ff) fm = Math.Max(fm, v);
                densityLogMax = Mathf.Log(1f + dm / 256f * 0.6f);
                failMax = fm;
            }
            heatFrame = f.frame;
            // the run time every run of the act has reached at this frame (an evolution act walks iterations)
            float t = st.ActRunTime(p, f.tau, out int it);
            var pack = st.M.Pack;
            foreach (int run in p.runs) {
                int rit = st.M.Runs[run].iteration;
                float tr;
                if (p.act.type == "evolution") {
                    if (it < 0 || rit > it) continue;
                    tr = rit < it ? float.MaxValue : t;
                }
                else tr = t - p.staggerByRun[run];
                if (tr < 0) continue;
                int n = pack.Samples(run);
                int upto = Mathf.Min(n - 1, Mathf.FloorToInt(Mathf.Min(tr, 1e7f) * pack.Hz));
                for (int k = nextSample[run]; k <= upto; k++) Splat(density, st.UnityPos(pack.Sample(run, k)), 1);
                nextSample[run] = Mathf.Max(nextSample[run], upto + 1);
                if (!failDone[run] && st.M.Runs[run].Failed && tr >= st.M.Runs[run].duration) {
                    SplatDisc(fail, st.UnityPos(pack.Sample(run, n - 1)), 1.6f, 256);
                    failDone[run] = true;
                }
            }
            for (int i = 0; i < density.Length; i++) {
                heatPixels[2 * i] = Mathf.Log(1f + density[i] / 256f * 0.6f) / Mathf.Max(1e-6f, densityLogMax);
                heatPixels[2 * i + 1] = Mathf.Sqrt(fail[i] / failMax);
            }
            heatTex.SetPixelData(heatPixels, 0);
            heatTex.Apply(false);
        }

        // ------------------------------------------------------------------ GPU (inside the custom pass)

        public void Draw(CommandBuffer cmd, Camera cam, RTHandle color, RTHandle depth) {
            var vp = GL.GetGPUProjectionMatrix(cam.projectionMatrix, true) * cam.worldToCameraMatrix;
            mat.SetMatrix("_TheaterVP", vp);
            mat.SetVector("_CamPos", cam.transform.position);
            CoreUtils.SetRenderTarget(cmd, color, depth, ClearFlag.None);
            if (washOn) {
                var b = st.World.Bounds;
                mat.SetVector("_WashRect", new Vector4(b.center.x - 2000f, b.center.z - 2000f, b.center.x + 2000f, b.center.z + 2000f));
                mat.SetFloat("_WashY", st.World.SurfaceY(b.center) + washLift);
                mat.SetVector("_WashColor", new Vector4(washColor.r, washColor.g, washColor.b, washColor.a));
                mat.SetVector("_WashView", new Vector4(cam.pixelWidth, cam.pixelHeight, washVignette, washStart));
                cmd.DrawProcedural(Matrix4x4.identity, mat, 4, MeshTopology.Triangles, 6, 1);
                if (washFar.x > 0 || washFar.z > 0) {
                    // the camera's distance to the water straight ahead (or below it) scales the far band
                    float wy = st.World.SurfaceY(b.center) + washLift;
                    var ct = cam.transform;
                    float d0 = ct.forward.y < -0.05f ? (ct.position.y - wy) / -ct.forward.y : Mathf.Max(1f, ct.position.y - wy) * 3f;
                    var far = washFar.x > 0 ? new Vector4(washFar.x * d0, Mathf.Max(washFar.x + 0.01f, washFar.y) * d0, washFar.z, 0) : new Vector4(1e9f, 2e9f, washFar.z, 0);
                    mat.SetVector("_WashFar", far);
                    cmd.DrawProcedural(Matrix4x4.identity, mat, 5, MeshTopology.Triangles, 6, 1);
                }
            }
            if (heatOn) {
                mat.SetTexture("_Heat", heatTex);
                mat.SetVector("_HeatRect", heatRect);
                mat.SetFloat("_HeatY", st.World.SurfaceY(st.World.Bounds.center) + 0.03f);
                mat.SetFloat("_DensityGain", 1.0f); mat.SetFloat("_FailGain", 1.0f);
                mat.SetVector("_DensityColor", new Vector4(0.35f, 0.75f, 1.0f, 0.55f));
                mat.SetVector("_FailColor", new Vector4(1.0f, 0.18f, 0.08f, 0.85f));
                cmd.DrawProcedural(Matrix4x4.identity, mat, 3, MeshTopology.Triangles, 6, 1);
            }
            st.Overlay?.Draw(cmd, mat, overlayProps);
            if (history != null) cmd.DrawMesh(history, Matrix4x4.identity, mat, 0, 1, historyProps);
            if (trails != null) {
                trailProps.SetFloat("_Now", trailNow);
                cmd.DrawMesh(trails, Matrix4x4.identity, mat, 0, 1, trailProps);
            }
            if (cometsOn && comets.vertexCount > 0) cmd.DrawMesh(comets, Matrix4x4.identity, mat, 0, 1, cometProps);
            if (ghostCount > 0) cmd.DrawMeshInstancedProcedural(chevrons ? chevronMesh : ghostMesh, 0, mat, 0, ghostCount, ghostProps);
            if (heroCount > 0) cmd.DrawMeshInstancedProcedural(ghostMesh, 0, mat, 0, heroCount, heroProps);
            if (markerCount > 0) {
                mat.SetBuffer("_Markers", markerBuf);
                cmd.DrawProcedural(Matrix4x4.identity, mat, 2, MeshTopology.Triangles, 6, markerCount);
            }
        }
    }
}
