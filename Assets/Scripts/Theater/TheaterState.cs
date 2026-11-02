using System;
using System.Collections.Generic;
using UnityEngine;

namespace Crane.Theater {
    public struct GhostDraw {
        public int run;
        public float tr;                 // run time (s) this ghost shows
        public Vector3 pos;
        public float yaw, pitch, roll;   // Unity degrees
        public Color color;              // linear colour; a = opacity
        public float scale;
        public Pose6 pose;
    }

    public struct MarkerDraw {
        public Vector3 pos;
        public float age, radius;
        public Color color;
    }

    public sealed class HudModel {
        public string title, caption, actType, colour, ramp, legendLo, legendHi, insight = "";
        public int iteration = -1, iterFrom, iterTo, iterCount, runs, finished, failed, running;
        public float best = float.NaN, runTime, speed;
        public float[] spark;
        public int sparkCursor = -1;
        public string goldId, silverId;
        public float goldScore, silverScore, goldDuration, silverDuration, goldProgress, silverProgress;
        public bool goldDone, silverDone;
        public List<string> paramDiff = new List<string>();
    }

    /// <summary>One act prepared for playback: its runs, offsets, heroes and a camera path computed once from the
    /// act's start, so every frame's state is a pure function of (manifest, script, frame index).</summary>
    public sealed class ActPlan {
        public TheaterAct act;
        public int index, firstFrame, frames;
        public float start;
        public List<int> runs = new List<int>();
        public float[] stagger;
        public float[] staggerByRun;   // indexed by run (pack order)
        public int[] iterations = Array.Empty<int>();
        public Dictionary<int, List<int>> byIteration = new Dictionary<int, List<int>>();
        public int hero = -1, previous = -1;
        public List<int> trailRuns = new List<int>();
        public float scoreLo, scoreHi;
        public Vector3[] camPos;
        public Quaternion[] camRot;
        public float[] camFov;
    }

    public sealed class FrameState {
        public int frame, act, iteration = -1;
        public float tau;
        public readonly List<GhostDraw> ghosts = new List<GhostDraw>();
        public readonly List<GhostDraw> heroes = new List<GhostDraw>();   // [0] gold, [1] silver (when present)
        public readonly List<MarkerDraw> markers = new List<MarkerDraw>();
        public Vector3 camPos;
        public Quaternion camRot;
        public float fov;
        public HudModel hud = new HudModel();
        public ActPlan plan;

        /// <summary>A stable hash of everything drawn (the determinism test compares it across evaluation orders).</summary>
        public string Digest() {
            var b = new System.Text.StringBuilder();
            b.Append(frame).Append('|').Append(act).Append('|').Append(iteration).Append('|').Append(tau.ToString("R"));
            foreach (var g in ghosts) b.Append(';').Append(g.run).Append(',').Append(g.pos.x.ToString("R")).Append(',').Append(g.pos.z.ToString("R"))
                .Append(',').Append(g.yaw.ToString("R")).Append(',').Append(g.color.a.ToString("R"));
            foreach (var g in heroes) b.Append(";h").Append(g.run).Append(',').Append(g.pos.x.ToString("R")).Append(',').Append(g.pos.z.ToString("R"));
            foreach (var m in markers) b.Append(";m").Append(m.pos.x.ToString("R")).Append(',').Append(m.age.ToString("R"));
            b.Append("|c").Append(camPos.x.ToString("R")).Append(',').Append(camPos.y.ToString("R")).Append(',').Append(camPos.z.ToString("R"))
                .Append(',').Append(camRot.x.ToString("R")).Append(',').Append(camRot.w.ToString("R"));
            b.Append("|hud").Append(hud.iteration).Append(',').Append(hud.finished).Append(',').Append(hud.failed).Append(',').Append(hud.runs);
            using var sha = System.Security.Cryptography.SHA256.Create();
            return BitConverter.ToString(sha.ComputeHash(System.Text.Encoding.UTF8.GetBytes(b.ToString()))).Replace("-", "").ToLowerInvariant();
        }
    }

    /// <summary>The theater's state machine. Nothing here reads a clock: frame f is at video time f / fps.</summary>
    public sealed class TheaterState {
        public readonly TheaterManifest M;
        public readonly TheaterScript S;
        public readonly ITheaterWorld World;
        public readonly List<ActPlan> Plans = new List<ActPlan>();
        public readonly float[] BestPerIteration;
        public readonly bool Planar;
        readonly float aspect;
        public int FrameCount => S.FrameCount;

        public TheaterState(TheaterManifest m, TheaterScript s, ITheaterWorld world) {
            M = m; S = s; World = world;
            aspect = s.Width / (float)s.Height;
            BestPerIteration = m.BestPerIteration();
            Planar = TJson.Str(m.Raw, "pose_dims", "6dof") == "planar";
            float t = 0; int f = 0;
            for (int i = 0; i < s.Acts.Count; i++) {
                var p = Plan(s.Acts[i], i);
                p.start = t; p.firstFrame = f;
                p.frames = Mathf.RoundToInt((t + s.Acts[i].duration) * s.Fps) - f;
                t += s.Acts[i].duration; f += p.frames;
                CameraPath(p);
                Plans.Add(p);
            }
        }

        // ------------------------------------------------------------------ planning

        ActPlan Plan(TheaterAct a, int index) {
            var p = new ActPlan { act = a, index = index };
            var pool = new List<TheaterRun>();
            foreach (var r in M.Runs) {
                if (r.iteration < a.iterFrom || r.iteration > a.iterTo) continue;
                if (a.iterList != null && !a.iterList.Contains(r.iteration)) continue;
                if (a.outcome == "finished" && r.outcome != "finished") continue;
                if (a.outcome == "failed" && !r.Failed) continue;
                pool.Add(r);
            }
            if (a.topK > 0) {
                var keep = new List<TheaterRun>();
                var groups = new SortedDictionary<int, List<TheaterRun>>();
                foreach (var r in pool) { if (!groups.TryGetValue(r.iteration, out var g)) groups[r.iteration] = g = new List<TheaterRun>(); g.Add(r); }
                foreach (var g in groups.Values) {
                    g.Sort((x, y) => y.score != x.score ? y.score.CompareTo(x.score) : x.index.CompareTo(y.index));
                    keep.AddRange(g.GetRange(0, Math.Min(a.topK, g.Count)));
                }
                pool = keep;
            }
            if (a.sample > 0 && pool.Count > a.sample) {
                var rng = new System.Random(a.seed);
                for (int i = pool.Count - 1; i > 0; i--) { int j = rng.Next(i + 1); (pool[i], pool[j]) = (pool[j], pool[i]); }
                pool = pool.GetRange(0, a.sample);
            }
            pool.Sort((x, y) => x.index.CompareTo(y.index));
            foreach (var r in pool) p.runs.Add(r.index);
            p.stagger = new float[p.runs.Count];
            if (a.stagger > 0) {
                var rng = new System.Random(a.seed * 7919 + 17);
                for (int i = 0; i < p.stagger.Length; i++) p.stagger[i] = (float)(rng.NextDouble() * a.stagger);
            }
            p.staggerByRun = new float[M.Runs.Count];
            for (int i = 0; i < p.runs.Count; i++) p.staggerByRun[p.runs[i]] = p.stagger[i];
            var its = new SortedSet<int>();
            foreach (int i in p.runs) {
                int it = M.Runs[i].iteration;
                its.Add(it);
                if (!p.byIteration.TryGetValue(it, out var l)) p.byIteration[it] = l = new List<int>();
                l.Add(i);
            }
            p.iterations = new int[its.Count]; its.CopyTo(p.iterations);
            p.scoreLo = float.PositiveInfinity; p.scoreHi = float.NegativeInfinity;
            foreach (int i in p.runs) { p.scoreLo = Mathf.Min(p.scoreLo, M.Runs[i].score); p.scoreHi = Mathf.Max(p.scoreHi, M.Runs[i].score); }
            if (p.runs.Count == 0) { p.scoreLo = 0; p.scoreHi = 1; }

            var records = M.Records();
            bool heroAct = a.type == "compare" || a.type == "spotlight";
            string heroSel = a.hero == "auto" ? (heroAct ? "best" : "none") : a.hero;
            string prevSel = a.previous == "auto" ? (a.type == "compare" ? "previous" : "none") : a.previous;
            p.hero = heroSel == "none" ? -1 : heroSel == "best" ? (records.Count > 0 ? records[records.Count - 1].index : -1) : FindRun(heroSel);
            p.previous = prevSel == "none" ? -1 : prevSel == "previous" ? (records.Count > 1 ? records[records.Count - 2].index : -1) : FindRun(prevSel);
            if (a.trails == "top") {
                var ranked = new List<int>(p.runs);
                ranked.Sort((x, y) => M.Runs[y].score != M.Runs[x].score ? M.Runs[y].score.CompareTo(M.Runs[x].score) : x.CompareTo(y));
                foreach (int i in ranked) { if (p.trailRuns.Count >= a.trailK) break; if (i != p.hero && i != p.previous) p.trailRuns.Add(i); }
            }
            return p;
        }

        int FindRun(string id) { foreach (var r in M.Runs) if (r.id == id) return r.index; return -1; }

        // ------------------------------------------------------------------ timing

        public ActPlan PlanAt(int frame) {
            foreach (var p in Plans) if (frame < p.firstFrame + p.frames) return p;
            return Plans[Plans.Count - 1];
        }

        /// <summary>The run time (s) the act shows at act time tau for its run i (before the ghost's stagger),
        /// and the current iteration of an evolution act.</summary>
        public float ActRunTime(ActPlan p, float tau, out int iteration) {
            var a = p.act;
            iteration = -1;
            if (a.type == "evolution" && p.iterations.Length > 0) {
                int k = Mathf.Clamp(Mathf.FloorToInt(tau / Mathf.Max(1e-3f, a.perIteration)), 0, p.iterations.Length - 1);
                iteration = p.iterations[k];
                return a.t0 + (tau - k * a.perIteration) * a.speed;
            }
            return a.t0 + tau * a.speed;
        }

        IEnumerable<int> Visible(ActPlan p, int iteration) {
            if (iteration >= 0 && p.byIteration.TryGetValue(iteration, out var l)) return l;
            return p.runs;
        }

        // ------------------------------------------------------------------ poses

        public Vector3 UnityPos(Pose6 q) {
            var v = World.ToUnity(q.x, q.y, q.z);
            if (Planar) v.y = World.SurfaceY(v);
            return v;
        }

        public GhostDraw Ghost(int run, float tr, float scale) {
            var q = M.Pack.At(run, tr);
            return new GhostDraw {
                run = run, tr = tr, pos = UnityPos(q), yaw = World.UnityYaw(q.yaw), pitch = Planar ? 0 : q.pitch,
                roll = Planar ? 0 : -q.roll, scale = scale, pose = q
            };
        }

        Color GhostColour(ActPlan p, TheaterRun r, Pose6 q) {
            var a = p.act;
            switch (a.colour) {
                case "iteration":
                    return Colormaps.Eval(a.ramp, M.MaxIteration > M.MinIteration ? (r.iteration - M.MinIteration) / (float)(M.MaxIteration - M.MinIteration) : 1f);
                case "score":
                    return Colormaps.Eval(a.ramp, p.scoreHi > p.scoreLo ? (r.score - p.scoreLo) / (p.scoreHi - p.scoreLo) : 1f);
                case "outcome":
                    return r.Failed ? Colormaps.Failure : Colormaps.Eval(a.ramp, 0.85f);
                default:
                    return Colormaps.Eval(a.ramp, q.progress);
            }
        }

        // ------------------------------------------------------------------ evaluation

        public FrameState Evaluate(int frame) {
            var p = PlanAt(frame);
            var a = p.act;
            var st = new FrameState { frame = frame, act = p.index, plan = p };
            int local = Mathf.Clamp(frame - p.firstFrame, 0, Mathf.Max(0, p.frames - 1));
            st.tau = local / S.Fps;
            float t = ActRunTime(p, st.tau, out int iteration);
            st.iteration = iteration;
            var shown = new List<int>(Visible(p, iteration));
            float alpha = Mathf.Clamp(a.ghostAlpha * Mathf.Sqrt(30f / Mathf.Max(1, shown.Count)), 0.05f, a.ghostAlpha);
            var hud = st.hud;
            hud.runs = shown.Count;
            for (int k = 0; k < shown.Count; k++) {
                int i = shown[k];
                if (i == p.hero || i == p.previous) continue;
                var r = M.Runs[i];
                float tr = t - p.staggerByRun[i];
                if (tr < 0) continue;
                float dur = r.duration;
                var g = Ghost(i, Mathf.Min(tr, dur), a.ghostScale);
                Color c = GhostColour(p, r, g.pose).linear;
                float op = alpha;
                if (tr > dur) {
                    float after = tr - dur;
                    if (r.Failed) {
                        if (after > 2f) { hud.failed++; continue; }
                        float flash = Mathf.Clamp01(1f - after / 2f);
                        c = Color.Lerp(c, Colormaps.Failure.linear, Mathf.Clamp01(after * 4f));
                        op = Mathf.Max(alpha, 0.6f) * flash;
                        hud.failed++;
                    }
                    else {
                        if (after > 1f) { hud.finished++; continue; }
                        op *= 1f - after;
                        hud.finished++;
                    }
                }
                else hud.running++;
                c.a = op;
                g.color = c;
                st.ghosts.Add(g);
            }
            float heroT = t;
            if (p.hero >= 0) st.heroes.Add(HeroGhost(p.hero, heroT, Colormaps.Gold));
            if (p.previous >= 0) st.heroes.Add(HeroGhost(p.previous, heroT, Colormaps.Silver));
            if (a.markers) Markers(p, st, t);
            st.camPos = p.camPos[local]; st.camRot = p.camRot[local]; st.fov = p.camFov[local];
            Hud(p, st, t, iteration);
            return st;
        }

        GhostDraw HeroGhost(int run, float t, Color c) {
            var g = Ghost(run, Mathf.Clamp(t, 0, M.Runs[run].duration), 1f);
            g.color = c.linear;
            g.tr = t;
            return g;
        }

        void Markers(ActPlan p, FrameState st, float t) {
            void Add(int run, float scale) {
                var r = M.Runs[run];
                foreach (var e in r.events) {
                    float age = t - e.t;
                    if (age < 0 || age > 1.6f * Mathf.Max(1f, p.act.speed / 4f)) continue;
                    Color c = e.kind == "failure" ? Colormaps.Failure : e.kind == "finish" ? Colormaps.Gold : Colormaps.Gate;
                    Vector3 at = float.IsNaN(e.x) ? UnityPos(M.Pack.At(run, e.t)) : World.ToUnity(e.x, e.y, 0);
                    if (float.IsNaN(e.x) || Planar) at.y = World.SurfaceY(at);
                    st.markers.Add(new MarkerDraw { pos = at, age = age / Mathf.Max(1f, p.act.speed / 4f), radius = scale, color = c.linear });
                }
            }
            if (p.hero >= 0) Add(p.hero, 2.2f);
            if (p.previous >= 0) Add(p.previous, 1.8f);
            foreach (int i in p.trailRuns) Add(i, 1.4f);
        }

        void Hud(ActPlan p, FrameState st, float t, int iteration) {
            var a = p.act; var h = st.hud;
            h.title = S.Title; h.caption = a.caption; h.actType = a.type; h.colour = a.colour; h.ramp = a.ramp;
            h.runTime = t; h.speed = a.speed;
            h.iterFrom = p.iterations.Length > 0 ? p.iterations[0] : M.MinIteration;
            h.iterTo = p.iterations.Length > 0 ? p.iterations[p.iterations.Length - 1] : M.MaxIteration;
            h.iterCount = M.MaxIteration - M.MinIteration + 1;
            h.iteration = iteration;
            h.spark = BestPerIteration;
            int cursor = iteration >= 0 ? iteration : h.iterTo;
            h.sparkCursor = cursor - M.MinIteration;
            h.best = float.NaN;
            for (int k = 0; k <= Mathf.Min(h.sparkCursor, BestPerIteration.Length - 1); k++)
                if (!float.IsNaN(BestPerIteration[k]) && (float.IsNaN(h.best) || BestPerIteration[k] > h.best)) h.best = BestPerIteration[k];
            if (M.Insights.TryGetValue(cursor, out var ins)) h.insight = ins ?? "";
            switch (a.colour) {
                case "iteration": h.legendLo = $"iteration {M.MinIteration}"; h.legendHi = $"{M.MaxIteration}"; break;
                case "score": h.legendLo = $"score {p.scoreLo:0.00}"; h.legendHi = $"{p.scoreHi:0.00}"; break;
                case "outcome": h.legendLo = "finished"; h.legendHi = "failed"; break;
                default: h.legendLo = "progress 0%"; h.legendHi = "100%"; break;
            }
            if (p.hero >= 0) {
                var g = M.Runs[p.hero];
                h.goldId = g.id; h.goldScore = g.score; h.goldDuration = g.duration; h.goldDone = t >= g.duration;
                h.goldProgress = M.Pack.At(p.hero, Mathf.Min(t, g.duration)).progress;
            }
            if (p.previous >= 0) {
                var s = M.Runs[p.previous];
                h.silverId = s.id; h.silverScore = s.score; h.silverDuration = s.duration; h.silverDone = t >= s.duration;
                h.silverProgress = M.Pack.At(p.previous, Mathf.Min(t, s.duration)).progress;
                if (p.hero >= 0) {
                    var g = M.Runs[p.hero];
                    var keys = new List<string>(g.parameters.Keys); keys.Sort(StringComparer.Ordinal);
                    foreach (var k in keys) {
                        if (!(g.parameters[k] is double gv)) continue;
                        double sv = s.parameters.TryGetValue(k, out var o) && o is double d ? d : double.NaN;
                        if (double.IsNaN(sv) || Math.Abs(gv - sv) > 1e-9) h.paramDiff.Add($"{Short(k)}  {Fmt(sv)} → {Fmt(gv)}");
                        if (h.paramDiff.Count >= 6) break;
                    }
                }
            }
        }

        static string Short(string k) { int i = k.LastIndexOf('.'); return i >= 0 && i < k.Length - 1 ? k.Substring(i + 1) : k; }
        static string Fmt(double v) => double.IsNaN(v) ? "-" : v.ToString(Math.Abs(v) >= 100 ? "0" : "0.###", System.Globalization.CultureInfo.InvariantCulture);

        // ------------------------------------------------------------------ cameras

        Vector3 North => (World.ToUnity(0, 1, 0) - World.ToUnity(0, 0, 0)).normalized;
        Vector3 East => (World.ToUnity(1, 0, 0) - World.ToUnity(0, 0, 0)).normalized;

        void CameraPath(ActPlan p) {
            int n = Mathf.Max(1, p.frames);
            p.camPos = new Vector3[n]; p.camRot = new Quaternion[n]; p.camFov = new float[n];
            var a = p.act;
            string shot = a.Shot;
            float fov = a.Cam("fov", 50f);
            Bounds B = World.Bounds;
            Vector3 C = B.center;
            Vector3 north = North, east = East;
            float vt = Mathf.Tan(fov * 0.5f * Mathf.Deg2Rad), ht = vt * aspect;
            float spanE = Mathf.Abs(Vector3.Dot(B.size, east)), spanN = Mathf.Abs(Vector3.Dot(B.size, north));
            float ext = Mathf.Max(B.size.x, B.size.z);

            // Targets for tracking shots, smoothed forward from the act's start (critically damped).
            Vector3[] target = null, heading = null;
            if (shot == "follow" || shot == "chase" || shot == "pack") {
                target = new Vector3[n]; heading = new Vector3[n];
                for (int f = 0; f < n; f++) {
                    float tau = f / S.Fps;
                    float t = ActRunTime(p, tau, out int it);
                    if (shot == "pack" || p.hero < 0) {
                        Vector3 sum = Vector3.zero; int cnt = 0;
                        foreach (int i in Visible(p, it)) {
                            float tr = t - p.staggerByRun[i];
                            if (tr < 0 || tr > M.Runs[i].duration) continue;
                            sum += UnityPos(M.Pack.At(i, tr)); cnt++;
                        }
                        target[f] = cnt > 0 ? sum / cnt : (f > 0 ? target[f - 1] : (World.CentreLine.Count > 0 ? World.CentreLine[0] : C));
                        heading[f] = east;
                    }
                    else {
                        var q = M.Pack.At(p.hero, Mathf.Clamp(t, 0, M.Runs[p.hero].duration));
                        target[f] = UnityPos(q);
                        float yaw = World.UnityYaw(q.yaw) * Mathf.Deg2Rad;
                        heading[f] = new Vector3(Mathf.Sin(yaw), 0, Mathf.Cos(yaw));
                    }
                }
                float omega = a.Cam("stiffness", shot == "pack" ? 1.2f : 2.0f);
                Smooth(target, omega, 1f / S.Fps);
                Smooth(heading, omega * 0.6f, 1f / S.Fps);
            }

            for (int f = 0; f < n; f++) {
                float tau = f / S.Fps, u = n > 1 ? f / (float)(n - 1) : 0f;
                Vector3 pos, look; Vector3 up = Vector3.up;
                switch (shot) {
                    case "top": {
                        float pitch = a.Cam("pitch", 62f) * Mathf.Deg2Rad;
                        Vector3 fwd = (north * Mathf.Cos(pitch) - Vector3.up * Mathf.Sin(pitch)).normalized;
                        float d = Mathf.Max(spanE * 0.5f / ht, spanN * 0.5f / vt) * a.Cam("margin", 1.0f);
                        d *= 1f - a.Cam("push", 0.08f) * Smoothstep(u);
                        look = C; pos = C - fwd * d; up = north;
                        break;
                    }
                    case "flyover": {
                        var line = World.CentreLine;
                        look = Along(line, u * 0.92f + 0.04f);
                        Vector3 back = Along(line, Mathf.Max(0, u * 0.92f + 0.04f - 0.06f));
                        Vector3 dir = (look - back); dir.y = 0; dir = dir.sqrMagnitude > 1e-6f ? dir.normalized : east;
                        pos = look - dir * a.Cam("back", 18f) + Vector3.up * a.Cam("height", 14f);
                        break;
                    }
                    case "follow":
                    case "chase": {
                        float back = a.Cam("back", shot == "chase" ? 5f : 9f), height = a.Cam("height", shot == "chase" ? 2.5f : 7f);
                        Vector3 hd = heading[f]; hd.y = 0; hd = hd.sqrMagnitude > 1e-6f ? hd.normalized : east;
                        Vector3 side = Vector3.Cross(Vector3.up, hd) * a.Cam("side", 0f);
                        pos = target[f] - hd * back + Vector3.up * height + side;
                        look = target[f] + hd * a.Cam("lead", 4f);
                        break;
                    }
                    case "pack": {
                        float ang = (a.Cam("bearing", 210f) + a.Cam("deg_per_s", 4f) * tau) * Mathf.Deg2Rad;
                        Vector3 off = (east * Mathf.Cos(ang) + north * Mathf.Sin(ang)) * a.Cam("back", 22f);
                        pos = target[f] + off + Vector3.up * a.Cam("height", 26f);
                        look = target[f];
                        break;
                    }
                    default: { // orbit and establish
                        float r = a.Cam("radius", ext * 0.75f), h = a.Cam("height", ext * 0.45f);
                        if (shot == "establish") { float k = Mathf.Lerp(1.8f, 1f, Smoothstep(u)); r *= k; h *= k; }
                        float ang = (a.Cam("angle", 200f) + a.Cam("deg_per_s", 5f) * tau) * Mathf.Deg2Rad;
                        pos = C + (east * Mathf.Cos(ang) + north * Mathf.Sin(ang)) * r + Vector3.up * h;
                        look = C;
                        break;
                    }
                }
                float minY = World.SurfaceY(pos) + 1.0f;
                if (pos.y < minY) pos.y = minY;
                var fwdDir = look - pos;
                if (fwdDir.sqrMagnitude < 1e-6f) fwdDir = Vector3.forward;
                p.camPos[f] = pos;
                p.camRot[f] = Quaternion.LookRotation(fwdDir.normalized, Mathf.Abs(Vector3.Dot(fwdDir.normalized, up)) > 0.999f ? north : up);
                p.camFov[f] = fov;
            }
        }

        static float Smoothstep(float u) { u = Mathf.Clamp01(u); return u * u * (3 - 2 * u); }

        /// <summary>Critically damped follow of xs (in place), forward from xs[0].</summary>
        static void Smooth(Vector3[] xs, float omega, float dt) {
            if (xs.Length == 0) return;
            Vector3 x = xs[0], v = Vector3.zero;
            for (int i = 0; i < xs.Length; i++) {
                float k = omega * dt, e = 1f / (1f + k + 0.48f * k * k + 0.235f * k * k * k);
                Vector3 change = x - xs[i];
                Vector3 temp = (v + omega * change) * dt;
                v = (v - omega * temp) * e;
                x = xs[i] + (change + temp) * e;
                xs[i] = x;
            }
        }

        static Vector3 Along(IReadOnlyList<Vector3> line, float u) {
            if (line == null || line.Count == 0) return Vector3.zero;
            if (line.Count == 1) return line[0];
            float total = 0;
            for (int i = 1; i < line.Count; i++) total += Vector3.Distance(line[i - 1], line[i]);
            float d = Mathf.Clamp01(u) * total;
            for (int i = 1; i < line.Count; i++) {
                float L = Vector3.Distance(line[i - 1], line[i]);
                if (d <= L) return Vector3.Lerp(line[i - 1], line[i], L > 0 ? d / L : 0);
                d -= L;
            }
            return line[line.Count - 1];
        }
    }
}
