using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;

namespace Crane.Fleet.Sim {
    /// <summary>
    /// Open-loop probe of the modules of a capplan module graph (capplan.module_graph/v0) on the per-wheel <see cref="TractivePlant"/>: each
    /// module runs the same segment list (body twist or driven-wheel speeds, held for a duration) and the run records, per segment and module,
    /// the commanded displacement, the truth displacement in the segment-start body frame, the encoder/gyro estimate's displacement, and the
    /// mean truth body twist over the segment's second half (steady state). This is the CRANE side of the open-loop cross-validation
    /// (Tools/Fleet/xval, OL protocol); it is a measurement, not a pass/fail test: the job "succeeds" when every segment ran and no module
    /// tipped (|pitch| or |roll| above the probe's tip limit).
    /// Rigid couplings (LATCHED_VERIFIED / LOAD_VALIDATED) are pre-latched at spawn and each coupled component is driven by one group twist through
    /// AssemblyDrive (CRANE_MODULAR_PLAN M2, command side); non-rigid couplings spawn as free faces.
    /// Probe file: {"schema": "crane.fleet_probe/v0", "variant": "...", "tip_limit_rad": r, "segments": [{"name": "fwd", "duration": s,
    /// "twist": [vx, vy, wz]} | {"name": "...", "duration": s, "wheels": [rad/s per driven wheel]}, ...], "modules": optional subset of ids}.
    /// Job: "config": {"graph": path-or-object, "origin": [x, y, yaw]}, "mission_file": probe file.
    /// </summary>
    public sealed class ModuleProbeMission : IFleetMission, IFleetComposer {
        readonly JsonNode graph, probe;
        readonly double[] origin;
        readonly string graphHash, graphSource;
        readonly List<(string name, double duration, Twist? twist, double[] wheels)> segments = new List<(string, double, Twist?, double[])>();
        readonly Dictionary<string, Pose2> starts = new Dictionary<string, Pose2>();
        readonly double tipLimit;
        int seg = -1;
        double segStart;
        readonly Dictionary<string, (RoverTruth t, double ox, double oy, double oyaw)> at0 = new Dictionary<string, (RoverTruth, double, double, double)>();
        readonly Dictionary<string, double[]> twistSum = new Dictionary<string, double[]>();
        readonly List<object> rows = new List<object>();
        readonly List<string> tipped = new List<string>();
        bool done;

        public string Variant { get; }
        public bool Done => done;
        public string ControllerInput => "open_loop";
        public IDictionary<string, object> Labels { get; } = new Dictionary<string, object>();
        public string Configuration { get; }
        public string ConfigurationHash => graphHash;

        public ModuleProbeMission(JsonNode config, string probePath) {
            probe = JsonNode.FromText(File.ReadAllText(probePath));
            if (!probe["schema"].String.StartsWith("crane.fleet_probe/")) throw new FormatException(probePath + ": not a crane.fleet_probe file");
            Variant = probe.Has("variant") ? probe["variant"].String : "probe_v0";
            tipLimit = probe["tip_limit_rad"].Double;
            string text;
            if (config["graph"].Unwrapped is string gp) { text = File.ReadAllText(gp); graphSource = Path.GetFullPath(gp); }
            else { text = FleetJson.Write(config["graph"].Raw); graphSource = "inline"; }
            graph = JsonNode.FromText(text);
            graphHash = new SpecFile(text, graphSource).Sha256;
            origin = config.Has("origin") ? config["origin"].Doubles : new double[3];
            Configuration = "capplan_module_graph:" + graphSource;
            foreach (var s in probe["segments"].Items) {
                Twist? tw = s.Has("twist") ? new Twist(s["twist"].Doubles[0], s["twist"].Doubles[1], s["twist"].Doubles[2]) : (Twist?)null;
                segments.Add((s["name"].String, s["duration"].Double, tw, s.Has("wheels") ? s["wheels"].Doubles : null));
            }
            Labels["configuration_kind"] = "capplan_module_graph";
            Labels["probe"] = Path.GetFileName(probePath);
        }

        /// <summary>Why this graph cannot be spawned, or null (rigid couplings, unknown types, intermittent health).</summary>
        public static string Unsupported(JsonNode config) {
            JsonNode g;
            try { g = config["graph"].Unwrapped is string gp ? JsonNode.FromText(File.ReadAllText(gp)) : config["graph"]; }
            catch (Exception e) { return "graph unreadable: " + e.Message; }
            if (g.Has("schema") && g["schema"].String != "capplan.module_graph/v0") return "graph schema " + g["schema"].String;
            var poses = new Dictionary<string, Pose2>();
            foreach (var m in g["modules"].Items)
                poses[m["id"].String] = new Pose2(m.Has("x") ? m["x"].Double : 0, m.Has("y") ? m["y"].Double : 0, m.Has("yaw") ? m["yaw"].Double : 0);
            // rigid couplings are pre-latched at spawn (M2): both modules must exist, both faces must exist in their types, and the graph poses
            // must put the two face frames coincident and facing each other (relative yaw pi). Non-rigid couplings spawn as free faces.
            if (g.Has("couplings"))
                foreach (var cp in g["couplings"].Items) {
                    if (!RigidStates.Contains(cp.Has("state") ? cp["state"].String : "LATCHED_VERIFIED")) continue;
                    string a = cp["a"].String, b = cp["b"].String, fa = cp["face_a"].String, fb = cp["face_b"].String;
                    if (!poses.ContainsKey(a) || !poses.ContainsKey(b)) return $"coupling {a}.{fa}-{b}.{fb}: unknown module";
                    var ta = TypeOf(g, a); var tb = TypeOf(g, b);
                    if (ta == null || tb == null || !ta["faces"].Has(fa) || !tb["faces"].Has(fb)) return $"coupling {a}.{fa}-{b}.{fb}: face not in the module type";
                    var expect = AssemblyDrive.MatedPose(poses[a], Face(ta, fa), Face(tb, fb));
                    double dp = Math.Sqrt(Sq(expect.X - poses[b].X) + Sq(expect.Y - poses[b].Y)), dy = Math.Abs(Angles.Wrap(expect.Yaw - poses[b].Yaw));
                    if (dp > 0.01 || dy > 0.01) return $"coupling {a}.{fa}-{b}.{fb}: graph poses do not mate the faces (off {dp:0.000} m, {dy:0.000} rad)";
                }
            foreach (var m in g["modules"].Items) {
                string t = m["type"].String;
                if (!g.Has("types") || !g["types"].Has(t)) return "module " + m["id"].String + ": type " + t + " not embedded in the graph";
                if (m.Has("act_health"))
                    foreach (var h in m["act_health"].Items)
                        if (Array.IndexOf(ModuleDescription.SupportedHealth, h.String) < 0 && h.String != "dead" && h.String != "locked" && h.String != "free")
                            return "module " + m["id"].String + ": wheel health " + h.String + " is not modelled in CRANE";
            }
            return null;
        }

        static readonly HashSet<string> RigidStates = new HashSet<string> { "LATCHED_VERIFIED", "LOAD_VALIDATED" };
        static double Sq(double v) => v * v;
        static JsonNode TypeOf(JsonNode g, string moduleId) {
            foreach (var m in g["modules"].Items) if (m["id"].String == moduleId) return g["types"].TryGet(m["type"].String);
            return null;
        }
        static (double x, double y, double yaw) Face(JsonNode type, string face) { var f = type["faces"][face]; return (f["x"].Double, f["y"].Double, f["yaw"].Double); }

        // pre-latched assemblies: connected components of the rigid couplings; member poses in the root's frame (the assembly frame)
        readonly List<(string a, string fa, string b, string fb)> rigid = new List<(string, string, string, string)>();
        readonly Dictionary<string, AssemblyDrive> assemblyOf = new Dictionary<string, AssemblyDrive>();
        readonly List<(AssemblyDrive drive, string root)> assemblies = new List<(AssemblyDrive, string)>();

        public IEnumerable<(RoverDescription d, WheelHealth[] health, double[] gain)> Fleet(FleetSpec spec) {
            var count = new Dictionary<string, int>();
            double c = Math.Cos(origin[2]), s = Math.Sin(origin[2]);
            foreach (var m in graph["modules"].Items) {
                string id = m["id"].String, t = m["type"].String;
                count.TryGetValue(t, out int k); count[t] = k + 1;
                var d = ModuleDescription.Resolve(spec, id, t, graph["types." + t], k);
                double x = m.Has("x") ? m["x"].Double : 0, y = m.Has("y") ? m["y"].Double : 0, yaw = m.Has("yaw") ? m["yaw"].Double : 0;
                starts[id] = new Pose2(origin[0] + c * x - s * y, origin[1] + s * x + c * y, origin[2] + yaw);
                var (h, gn) = ModuleDescription.InitialHealth(m, d.WheelUnits.Count);
                yield return (d, h, gn);
            }
        }

        public Pose2 StartPose(string roverId) => starts[roverId];
        /// <summary>Pre-latch the rigid couplings: for each connected component, keep the root module's (jittered) spawn pose, place every
        /// other member exactly at its mated pose (breadth-first through the couplings), and weld each coupled pair with a FixedJoint at that
        /// pose (no face hardware involved). Each component gets an <see cref="AssemblyDrive"/> with member poses in the root's frame.</summary>
        public IEnumerable<GameObject> Build(FleetRun run) {
            if (graph.Has("couplings"))
                foreach (var cp in graph["couplings"].Items)
                    if (RigidStates.Contains(cp.Has("state") ? cp["state"].String : "LATCHED_VERIFIED"))
                        rigid.Add((cp["a"].String, cp["face_a"].String, cp["b"].String, cp["face_b"].String));
                    else run.Event("coupling_not_latched", "", new Dictionary<string, object> { ["coupling"] = $"{cp["a"].String}.{cp["face_a"].String}-{cp["b"].String}.{cp["face_b"].String}",
                        ["state"] = cp["state"].String, ["note"] = "non-rigid state: spawned as free faces" });
            var placed = new Dictionary<string, Pose2>();   // pose in the root (assembly) frame
            foreach (var m in graph["modules"].Items) {
                string rootId = m["id"].String;
                if (placed.ContainsKey(rootId) || !rigid.Exists(c => c.a == rootId || c.b == rootId)) continue;
                var drive = new AssemblyDrive();
                var rootRig = run.Rover(rootId);
                drive.MaxSpeedXy = rootRig.D.MaxSpeedXy; drive.MaxYawRate = rootRig.D.MaxYawRate;
                placed[rootId] = new Pose2(0, 0, 0);
                drive.Add(rootId, placed[rootId], rootRig.D.WheelUnits);
                assemblyOf[rootId] = drive;
                var queue = new Queue<string>(); queue.Enqueue(rootId);
                while (queue.Count > 0) {
                    string cur = queue.Dequeue();
                    foreach (var c in rigid) {
                        string other, fCur, fOther;
                        if (c.a == cur) { other = c.b; fCur = c.fa; fOther = c.fb; } else if (c.b == cur) { other = c.a; fCur = c.fb; fOther = c.fa; } else continue;
                        if (placed.ContainsKey(other)) continue;
                        var pose = AssemblyDrive.MatedPose(placed[cur], Face(TypeOf(graph, cur), fCur), Face(TypeOf(graph, other), fOther));
                        placed[other] = pose;
                        var rig = run.Rover(other);
                        // world pose = root world pose composed with the in-assembly pose (keeps the root's spawn jitter rigidly)
                        var rootT = rootRig.Truth();
                        var world = AssemblyDrive.Compose(new Pose2(rootT.X, rootT.Y, rootT.Yaw), pose);
                        rig.Root.transform.SetPositionAndRotation(FleetFrames.World(world.X, world.Y, rig.Root.transform.position.y), FleetFrames.WorldYaw(world.Yaw));
                        rig.Body.position = rig.Root.transform.position; rig.Body.rotation = rig.Root.transform.rotation;
                        drive.Add(other, pose, rig.D.WheelUnits);
                        assemblyOf[other] = drive;
                        queue.Enqueue(other);
                    }
                }
                assemblies.Add((drive, rootId));
            }
            Physics.SyncTransforms();
            foreach (var c in rigid) {
                var ra = run.Rover(c.a); var rb = run.Rover(c.b);
                var j = rb.Root.AddComponent<FixedJoint>();
                j.connectedBody = ra.Body;
                j.enableCollision = false;
                run.Event("coupled_at_spawn", c.b, new Dictionary<string, object> { ["coupling"] = $"{c.a}.{c.fa}-{c.b}.{c.fb}", ["joint"] = "FixedJoint at the mated face pose (hardware-agnostic)" });
            }
            foreach (var (drive, root) in assemblies)
                run.Event("assembly", root, new Dictionary<string, object> { ["members"] = drive.Members.ConvertAll(x => (object)x.Id), ["mobility"] = drive.Mobility(),
                    ["drive_rank"] = drive.DriveRank(), ["note"] = "group twist at the root's base_link, distributed by AssemblyDrive" });
            yield break;
        }
        public void Start(FleetRun run) { Next(run); }

        void Next(FleetRun run) {
            if (seg >= 0) Close(run);
            seg++;
            if (seg >= segments.Count) { done = true; foreach (var r in run.Rovers) { r.Command = new Twist(); r.Plant.WheelCommandOverride = null; } return; }
            segStart = run.T;
            foreach (var r in run.Rovers) {
                at0[r.D.Id] = (r.Truth(), r.Odometry.X, r.Odometry.Y, r.Odometry.Yaw);
                twistSum[r.D.Id] = new double[4];
            }
            run.Event("probe_segment_start", "", new Dictionary<string, object> { ["segment"] = segments[seg].name, ["index"] = seg });
        }

        void Close(FleetRun run) {
            var sg = segments[seg];
            foreach (var r in run.Rovers) {
                var (t0, ox, oy, oyaw) = at0[r.D.Id];
                var t = r.Truth();
                double c = Math.Cos(t0.Yaw), s = Math.Sin(t0.Yaw), dx = t.X - t0.X, dy = t.Y - t0.Y;
                double ec = Math.Cos(oyaw), es = Math.Sin(oyaw), ex = r.Odometry.X - ox, ey = r.Odometry.Y - oy;
                var sum = twistSum[r.D.Id];
                double n = Math.Max(1, sum[3]);
                var cmd = sg.twist ?? new Twist();
                rows.Add(new Dictionary<string, object> {
                    ["segment"] = sg.name, ["module"] = r.D.Id, ["type"] = r.D.ModuleType, ["duration"] = sg.duration,
                    ["commanded_twist"] = sg.twist.HasValue ? new[] { cmd.Vx, cmd.Vy, cmd.Wz } : null, ["commanded_wheels"] = sg.wheels,
                    ["commanded_displacement"] = sg.twist.HasValue ? new[] { cmd.Vx * sg.duration, cmd.Vy * sg.duration, cmd.Wz * sg.duration } : null,
                    ["truth_displacement_body"] = new[] { c * dx + s * dy, -s * dx + c * dy, Unwrap(t.Yaw - t0.Yaw, cmd.Wz * sg.duration) },
                    ["estimate_displacement_body"] = new[] { ec * ex + es * ey, -es * ex + ec * ey, Unwrap(r.Odometry.Yaw - oyaw, cmd.Wz * sg.duration) },
                    ["steady_twist_truth"] = new[] { sum[0] / n, sum[1] / n, sum[2] / n },
                    ["health"] = HealthOf(r)
                });
            }
        }

        static string[] HealthOf(RoverRig r) {
            var h = new string[r.Plant.WheelCount];
            for (int i = 0; i < h.Length; i++) h[i] = r.D.Wheels[i].Name + ":" + (r.Plant.Health[i] == WheelHealth.Weak ? "weak=" + r.Plant.HealthGain[i].ToString("0.###", System.Globalization.CultureInfo.InvariantCulture) : r.Plant.Health[i].ToString());
            return h;
        }

        static double Unwrap(double d, double expected) {
            d = Angles.Wrap(d);
            while (d < expected - Math.PI) d += 2 * Math.PI;
            while (d > expected + Math.PI) d -= 2 * Math.PI;
            return d;
        }

        public void PreStep(FleetRun run) {
            if (done) return;
            if (run.T - segStart >= segments[seg].duration - 1e-9) { Next(run); if (done) return; }
            var sg = segments[seg];
            foreach (var r in run.Rovers) {
                if (probe.Has("modules") && probe["modules"].Items.TrueForAll(x => x.String != r.D.Id)) { r.Command = new Twist(); continue; }
                if (sg.wheels != null) { r.Plant.WheelCommandOverride = sg.wheels; r.Command = new Twist(); }
                else if (assemblyOf.ContainsKey(r.D.Id)) r.Command = new Twist();   // set below as a group
                else { r.Plant.WheelCommandOverride = null; r.Command = sg.twist ?? new Twist(); }
            }
            // coupled assemblies: one group twist at the root, distributed to every member's driven wheels (common saturation)
            if (sg.wheels == null)
                foreach (var (drive, root) in assemblies) {
                    var t = sg.twist ?? new Twist();
                    drive.Saturate(ref t, out var wheels);
                    foreach (var m in drive.Members) run.Rover(m.Id).Plant.WheelCommandOverride = wheels[m.Id];
                }
        }

        public void PostStep(FleetRun run) {
            if (done) return;
            bool steady = run.T - segStart >= segments[seg].duration / 2;
            foreach (var r in run.Rovers) {
                var t = r.Truth();
                if ((Math.Abs(t.Pitch) > tipLimit || Math.Abs(t.Roll) > tipLimit) && !tipped.Contains(r.D.Id)) {
                    tipped.Add(r.D.Id);
                    run.Event("tip", r.D.Id, new Dictionary<string, object> { ["pitch"] = t.Pitch, ["roll"] = t.Roll, ["evaluator_only"] = true });
                }
                if (steady) { var s = twistSum[r.D.Id]; s[0] += t.Body.Vx; s[1] += t.Body.Vy; s[2] += t.Body.Wz; s[3] += 1; }
            }
        }

        public string StepOf(string roverId) => seg >= 0 && seg < segments.Count ? segments[seg].name : done ? "done" : "start";

        public void LogState(StringBuilder b) {
            b.Append("\"segment\":").Append(FleetJson.Quote(StepOf(""))).Append(",\"segment_index\":").Append(seg);
        }

        public void Finish(FleetRun run, Dictionary<string, object> results) {
            if (!done && seg >= 0 && seg < segments.Count) Close(run);
            results["probe"] = new Dictionary<string, object> {
                ["graph"] = graphSource, ["graph_sha256"] = graphHash, ["origin"] = origin, ["segments_total"] = segments.Count,
                ["segments_completed"] = done ? segments.Count : Math.Max(0, seg), ["complete"] = done, ["tipped"] = tipped, ["rows"] = rows,
                ["plant"] = "tractive (per-wheel, CRANE_MODULAR_PLAN P1)",
                ["assemblies"] = assemblies.ConvertAll(a => (object)new Dictionary<string, object> { ["root"] = a.root, ["members"] = a.drive.Members.ConvertAll(x => (object)x.Id), ["mobility"] = a.drive.Mobility() }),
                ["note"] = "open-loop measurement; rigid couplings pre-latched (FixedJoint) and driven as one group twist (M2 command side)"
            };
        }
    }
}
