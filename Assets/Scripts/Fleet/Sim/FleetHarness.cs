using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

namespace Crane.Fleet.Sim {
    /// <summary>
    /// Paired-run harness: a jobs file (JSON) lists (scenario, configuration, seed, variant, failures, pair_of) entries; each runs headless in
    /// its own physics scene (byte-identical regardless of order) into OUT/runs/&lt;id&gt;/. OUT/summary.jsonl gets one row per job with the outcome
    /// (success = payload delivered inside the delivery zone for the demo mission; script_complete for qualification), the last mission flag,
    /// the connector state, and, for a job with pair_of, the first divergence from its pair (first state row where any rover's truth position
    /// differs by more than divergence_tol, with the rover). A failed job gets OUT/replays/&lt;id&gt;/replay.json (pointers to its trajectory and
    /// events, the divergence, the events within +-3 s of it) and, when the process has a GPU, rendered frames around the divergence
    /// (the job re-run with time-triggered shots aimed at the diverging rover). Python orchestration builds on this entry point.
    /// Jobs file: {"out": DIR, "divergence_tol": 0.001, "jobs": [{"id": "...", "mission": "demo"|"qual", "seed": 1, "variant": "...",
    ///   "failures": "rover:target:mode@t;...", "scenario": "scenarios/warehouse_a_v0.json", "config": {...opaque, recorded...}, "pair_of": "id"}]}
    /// </summary>
    public static class FleetHarness {
        public sealed class JobResult {
            public string Id, Dir, Outcome, Reason, PairOf;
            public bool Success;
            public double DivergenceT = double.NaN;
            public string DivergenceRover;
        }

        public static List<JobResult> RunHeadless(string specDir, string jobsPath, string launcher, string commit, bool dirty) {
            var root = JsonNode.FromText(File.ReadAllText(jobsPath));
            string outDir = Path.GetFullPath(root["out"].String);
            double tol = root.Has("divergence_tol") ? root["divergence_tol"].Double : 1e-3;
            Directory.CreateDirectory(outDir);
            var results = new List<JobResult>();
            var byId = new Dictionary<string, JobResult>();
            using var summary = new StreamWriter(Path.Combine(outDir, "summary.jsonl"), false) { NewLine = "\n" };
            foreach (var j in root["jobs"].Items) {
                string cfg = j.Has("config") && j["config"].Unwrapped is string cs ? cs : "profile";
                if (cfg != "profile") {
                    // module graphs from the shared generator cannot be spawned yet: refuse explicitly rather than run a different fleet
                    summary.WriteLine(FleetJson.Write(new Dictionary<string, object> { ["id"] = j["id"].String, ["success"] = false, ["outcome"] = "unsupported_configuration",
                        ["reason"] = "spawning from a module graph is not implemented yet; only config=profile (fleet_default) runs", ["config"] = cfg }));
                    summary.Flush();
                    Debug.Log($"CRANE_FLEET_JOB id={j["id"].String} success=False outcome=unsupported_configuration");
                    continue;
                }
                var res = RunJob(specDir, j, outDir, launcher, commit, dirty, null);
                if (res.PairOf != null && byId.TryGetValue(res.PairOf, out var pair)) {
                    (res.DivergenceT, res.DivergenceRover) = FirstDivergence(Path.Combine(pair.Dir, "states.jsonl"), Path.Combine(res.Dir, "states.jsonl"), tol);
                }
                if (!res.Success) WriteReplay(outDir, res, root);
                byId[res.Id] = res;
                results.Add(res);
                summary.WriteLine(FleetJson.Write(new Dictionary<string, object> { ["id"] = res.Id, ["success"] = res.Success, ["outcome"] = res.Outcome, ["reason"] = res.Reason,
                    ["dir"] = res.Dir, ["pair_of"] = res.PairOf, ["divergence_t"] = res.DivergenceT, ["divergence_rover"] = res.DivergenceRover,
                    ["config"] = j.Has("config") ? j["config"].Raw : null, ["failures"] = j.Has("failures") ? j["failures"].String : "" }));
                summary.Flush();
                Debug.Log($"CRANE_FLEET_JOB id={res.Id} success={res.Success} outcome={res.Outcome} divergence_t={res.DivergenceT}");
            }
            return results;
        }

        public static FleetRunOptions Options(FleetSpec spec, JsonNode j, string outDir, string launcher, string commit, bool dirty) {
            var o = new FleetRunOptions {
                Seed = (long)j["seed"].Double, Variant = j.Has("variant") ? j["variant"].String : null, OutDir = Path.Combine(outDir, "runs", j["id"].String),
                Failures = j.Has("failures") ? j["failures"].String : "", Launcher = launcher + " (harness)", SourceCommit = commit, SourceDirty = dirty
            };
            string mission = j.Has("mission") ? j["mission"].String : "qual";
            JsonNode mf = j.Has("mission_file") ? JsonNode.FromText(File.ReadAllText(j["mission_file"].String)) : null;
            if (mf != null) mission = mf.Has("schema") && mf["schema"].String.StartsWith("crane.fleet_mission/") ? "autonomy" : mf["mission"].String;
            if (mission == "demo") { o.Mission = new DemoMission(spec); o.MaxDuration = spec.O["demo.max_duration_s"].Double; }
            else if (mission == "autonomy") {
                o.Mission = new AutonomyMission(spec, j["mission_file"].String);
                o.MaxDuration = mf.Has("crane_max_duration_s") ? mf["crane_max_duration_s"].Double : 300.0;
            }
            else if (mission != "qual") throw new ArgumentException("unknown mission " + mission);
            if (j.Has("labels")) foreach (var kv in j["labels"].Members) o.Labels[kv.Key] = kv.Value.Raw;
            o.Labels["harness_job"] = j["id"].String;
            if (j.Has("max_duration_s")) o.MaxDuration = j["max_duration_s"].Double;
            return o;
        }

        public static FleetSpec SpecFor(string specDir, JsonNode j) =>
            FleetSpec.LoadDirectory(specDir, j.Has("scenario") ? j["scenario"].String : FleetSpec.DefaultScenario);

        static JobResult RunJob(string specDir, JsonNode j, string outDir, string launcher, string commit, bool dirty, Action<FleetRun> beforeSetup) {
            var spec = SpecFor(specDir, j);
            var opt = Options(spec, j, outDir, launcher, commit, dirty);
            var run = new FleetRun(spec, opt);
            beforeSetup?.Invoke(run);
            run.RunToEnd();
            var res = Classify(run, j);
            run.Dispose();
            return res;
        }

        public static JobResult Classify(FleetRun run, JsonNode j) {
            var res = new JobResult { Id = j["id"].String, Dir = run.Dir, PairOf = j.Has("pair_of") ? j["pair_of"].String : null };
            var m = JsonNode.FromText(File.ReadAllText(Path.Combine(run.Dir, "manifest.json")));
            var results = m["results"];
            if (run.Opt.Mission is AutonomyMission) {
                var a = results.TryGet("autonomy");
                bool ok = a != null && a["succeeded"].Bool;
                bool inZone = a != null && a.Has("payload_final") && a["payload_final.in_delivery_zone"].Bool;
                res.Success = ok;
                res.Outcome = ok ? (inZone ? "mission_succeeded_payload_delivered" : "mission_succeeded") : "mission_failed";
                res.Reason = a != null ? $"end={run.EndReason} first_failure_class={a["first_failure_class"].String} first_failure={(a["first_failure"].Unwrapped as string ?? "")}" : "end=" + run.EndReason;
            }
            else if (run.Opt.Mission is DemoMission) {
                var demo = results.TryGet("demo");
                bool delivered = demo != null && demo.Has("delivered") && demo["delivered.payload_in_delivery_zone"].Bool;
                res.Success = delivered;
                res.Outcome = delivered ? "delivered" : "not_delivered";
                string lastFlag = "";
                if (demo != null && demo.Has("flags")) { var f = demo["flags"].Items; if (f.Count > 0) lastFlag = f[f.Count - 1].String; }
                res.Reason = $"end={run.EndReason} last_flag={lastFlag} connector={(demo != null && demo.Has("connector_final_state") ? demo["connector_final_state"].String : "")}";
            }
            else {
                res.Success = run.EndReason == "script_complete";
                res.Outcome = run.EndReason;
                res.Reason = run.EndReason;
            }
            return res;
        }

        /// <summary>First state row where any rover's truth (x, y) differs by more than tol between two runs; (t, rover) or (NaN, null).</summary>
        public static (double, string) FirstDivergence(string a, string b, double tol) {
            using var ra = new StreamReader(a); using var rb = new StreamReader(b);
            while (true) {
                string la = ra.ReadLine(), lb = rb.ReadLine();
                if (la == null || lb == null) return la == lb ? (double.NaN, null) : (double.PositiveInfinity, "length");
                if (la == lb) continue;
                var na = JsonNode.FromText(la); var nb = JsonNode.FromText(lb);
                string worst = null; double wd = tol;
                foreach (var kv in na["truth"].Members) {
                    if (!nb["truth"].Has(kv.Key)) return (na["t"].Double, kv.Key);
                    var tb = nb["truth." + kv.Key];
                    double d = Math.Sqrt(Sq(kv.Value["x"].Double - tb["x"].Double) + Sq(kv.Value["y"].Double - tb["y"].Double));
                    if (d > wd) { wd = d; worst = kv.Key; }
                }
                if (worst != null) return (na["t"].Double, worst);
            }
        }

        static double Sq(double x) => x * x;

        static void WriteReplay(string outDir, JobResult res, JsonNode root) {
            string dir = Path.Combine(outDir, "replays", res.Id);
            Directory.CreateDirectory(dir);
            double center = !double.IsNaN(res.DivergenceT) && !double.IsInfinity(res.DivergenceT) ? res.DivergenceT : double.NaN;
            var near = new List<object>();
            foreach (string line in File.ReadAllLines(Path.Combine(res.Dir, "events.jsonl"))) {
                var e = JsonNode.FromText(line);
                double t = e["t"].Double;
                if (double.IsNaN(center) ? false : Math.Abs(t - center) <= 3.0) near.Add(e.Raw);
            }
            File.WriteAllText(Path.Combine(dir, "replay.json"), FleetJson.Write(new Dictionary<string, object> {
                ["schema"] = "crane.fleet_failure_replay/v0", ["job"] = res.Id, ["outcome"] = res.Outcome, ["reason"] = res.Reason,
                ["run_dir"] = res.Dir, ["trajectory"] = Path.Combine(res.Dir, "trajectory.ctraj"), ["events"] = Path.Combine(res.Dir, "events.jsonl"),
                ["pair_of"] = res.PairOf, ["divergence_t"] = res.DivergenceT, ["divergence_rover"] = res.DivergenceRover, ["events_near_divergence"] = near,
                ["frames"] = "frames/ (rendered by the player harness with a GPU; absent in -nographics runs)" }, true) + "\n");
        }

        /// <summary>Player with a GPU: re-run a failed job and render frames around its divergence (or its last seconds), aimed at the rover.</summary>
        public static IEnumerator RenderReplay(string specDir, JsonNode j, JobResult res, string outDir, string launcher, string commit, bool dirty) {
            double end = double.IsNaN(res.DivergenceT) || double.IsInfinity(res.DivergenceT) ? double.NaN : res.DivergenceT;
            var spec = SpecFor(specDir, j);
            var opt = Options(spec, j, Path.Combine(outDir, "replays", res.Id, "rerun"), launcher, commit, dirty);
            opt.OutDir = Path.Combine(outDir, "replays", res.Id, "rerun");
            var run = new FleetRun(spec, opt);
            string target = res.DivergenceRover ?? "cargo_2";
            var times = new List<double>();
            if (!double.IsNaN(end)) foreach (double dt in new[] { -1.0, 0.0, 1.0, 2.0 }) times.Add(Math.Max(0, end + dt));
            var shots = new List<object>();
            int k = 0;
            foreach (double t in times)
                shots.Add(new Dictionary<string, object> { ["name"] = $"replay_{k++:D2}_t{t:0.0}", ["t"] = t, ["fov"] = 45.0,
                    ["camera"] = new Dictionary<string, object> { ["target"] = "rover:" + target, ["position"] = new[] { -1.2, -1.6, 1.4 }, ["look_at"] = new[] { 0.0, 0.0, 0.1 } } });
            string json = FleetJson.Write(new Dictionary<string, object> { ["size"] = new[] { 960, 540 }, ["look"] = true, ["shots"] = shots });
            var fs = new FleetShots(spec, run, Path.Combine(outDir, "replays", res.Id, "frames"), JsonNode.FromText(json));
            run.Setup();
            fs.Begin();
            while (true) {
                while (fs.Due()) yield return fs.RenderNext();
                if (!run.Advance()) break;
                if (double.IsNaN(end) && run.T > 5) break;          // nothing to aim at: render nothing beyond a short window
                if (!double.IsNaN(end) && run.T > end + 2.5) break;
            }
            while (fs.Pending) yield return fs.RenderNext(force: true);
            run.Finish();
            fs.Finish();
            run.Dispose();
        }
    }
}
