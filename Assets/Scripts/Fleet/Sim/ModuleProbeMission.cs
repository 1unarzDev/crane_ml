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
    /// Uncoupled modules only: a graph with rigid couplings needs group (assembly) control (CRANE_MODULAR_PLAN M2) and is refused by the harness.
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
            if (g.Has("couplings") && g["couplings"].Items.Count > 0)
                return "coupled module graphs need assembly (group) control, CRANE_MODULAR_PLAN item M2; only uncoupled modules are spawned";
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
        public IEnumerable<GameObject> Build(FleetRun run) { yield break; }
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
                else { r.Plant.WheelCommandOverride = null; r.Command = sg.twist ?? new Twist(); }
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
                ["plant"] = "tractive (per-wheel, CRANE_MODULAR_PLAN P1)", ["note"] = "open-loop measurement; uncoupled modules only"
            };
        }
    }
}
