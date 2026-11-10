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
                string cfg = !j.Has("config") ? "profile" : j["config"].Unwrapped is string cs ? cs : j["config"].Has("graph") ? "module_graph" : "unknown";
                string refusal = cfg == "profile" ? null
                    : cfg == "module_graph" ? ModuleProbeMission.Unsupported(j["config"]) ?? (IsProbe(j) ? null : "a module graph runs only with a crane.fleet_probe mission_file")
                    : "config " + cfg + ": use \"profile\" or {\"graph\": capplan.module_graph/v0, \"origin\": [x, y, yaw]}";
                if (refusal != null) {
                    // refuse explicitly rather than run a different fleet
                    summary.WriteLine(FleetJson.Write(new Dictionary<string, object> { ["id"] = j["id"].String, ["success"] = false, ["outcome"] = "unsupported_configuration",
                        ["reason"] = refusal, ["config"] = cfg }));
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
            if (mf != null) mission = mf.Has("schema") && mf["schema"].String.StartsWith("crane.fleet_mission/") ? "autonomy"
                : mf.Has("schema") && mf["schema"].String.StartsWith("crane.fleet_probe/") ? "probe" : mf["mission"].String;
            if (mission == "demo") { o.Mission = new DemoMission(spec); o.MaxDuration = spec.O["demo.max_duration_s"].Double; }
            else if (mission == "autonomy") {
                o.Mission = new AutonomyMission(spec, j["mission_file"].String);
                o.MaxDuration = mf.Has("crane_max_duration_s") ? mf["crane_max_duration_s"].Double : 300.0;
            }
            else if (mission == "probe") {
                var pm = new ModuleProbeMission(j["config"], j["mission_file"].String);
                o.Mission = pm;
                o.MaxDuration = mf["max_duration_s"].Double;
            }
            else if (mission != "qual") throw new ArgumentException("unknown mission " + mission);
            if (j.Has("labels")) foreach (var kv in j["labels"].Members) o.Labels[kv.Key] = kv.Value.Raw;
            o.Labels["harness_job"] = j["id"].String;
            if (j.Has("max_duration_s")) o.MaxDuration = j["max_duration_s"].Double;
            return o;
        }

        static bool IsProbe(JsonNode j) {
            if (!j.Has("mission_file")) return false;
            try { var mf = JsonNode.FromText(File.ReadAllText(j["mission_file"].String)); return mf.Has("schema") && mf["schema"].String.StartsWith("crane.fleet_probe/"); }
            catch (IOException) { return false; }
        }

        public static FleetSpec SpecFor(string specDir, JsonNode j) =>
            FleetSpec.LoadDirectory(specDir, j.Has("scenario") ? j["scenario"].String : FleetSpec.DefaultScenario, j.Has("profile") ? j["profile"].String : FleetArgs.ProfileKey());

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
                // outcome from evaluator truth only; the library's own verdict is reported next to it
                var a = results.TryGet("autonomy");
                string truth = a != null && a.Has("truth_outcome") ? a["truth_outcome"].String : "unknown";
                bool libOk = a != null && a["succeeded"].Bool;
                bool flagged = a != null && a.Has("load_statics") && a["load_statics"].Has("flagged") && a["load_statics.flagged"].Bool;
                res.Success = truth == "delivered" && !flagged;   // a delivery with a load-statics margin under the threshold is flagged, not passed
                res.Outcome = "truth:" + truth + (flagged ? " (FLAGGED: load statics margin)" : "");
                res.Reason = (a != null && a.Has("summary") ? a["summary"].String : "") + $" | end={run.EndReason}";
            }
            else if (run.Opt.Mission is ModuleProbeMission) {
                var p = results["probe"];
                bool complete = p["complete"].Bool; int tipped = p["tipped"].Items.Count;
                res.Success = complete && tipped == 0;
                res.Outcome = complete ? (tipped == 0 ? "probe_complete" : "probe_tipped") : "probe_incomplete";
                res.Reason = $"segments {p["segments_completed"].Double}/{p["segments_total"].Double}, tipped {tipped} | end={run.EndReason}";
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

        /// <summary>Earliest failure-related event of an unpaired run: a disputed claim, a jam, a failed autonomy task attempt or a rejected
        /// host verdict; with the rover involved when the event names one.</summary>
        public static (double, string) FirstFailureEvent(string eventsPath) {
            foreach (string line in File.ReadAllLines(eventsPath)) {
                var e = JsonNode.FromText(line);
                string kind = e["kind"].String;
                bool fail = kind == "claim_disputed" || kind == "jam_start" || (kind == "host_verdict" && !e["ok"].Bool)
                    || (kind == "autonomy" && e.Has("event") && e["event"].Unwrapped is string ev && (ev == "task_attempt_failed" || ev == "task_failed"));
                if (!fail) continue;
                string rover = e.Has("rover") && e["rover"].Unwrapped is string r && r.Length > 0 ? r : null;
                return (e["t"].Double, rover);
            }
            return (double.NaN, null);
        }

        /// <summary>Player with a GPU: re-run a failed job and render frames around its divergence (or its last seconds), aimed at the rover.</summary>
        /// <summary>Story frames for a SUCCESSFUL job (jobs key "render": "story"): the lift, the carry, the set-down on the deck and delivered, at fixed
        /// offsets from the run's own events (payload_gripped + 1.5 s; midway between payload_load_on and payload_released; payload_released + 0.5 s;
        /// host_verdict + 0.3 s).
        /// Same rerun, cameras and file naming (replays/ID/frames/replay_NN_tT.png) as the failure replays; the camera follows the rover that
        /// holds the payload at each moment (lifter for lift/carry, place_on rover for set-down and delivered).</summary>
        public static List<(double t, string rover, string label)> StoryTimes(string eventsPath, string receiverFromMission = null) {
            var r = new List<(double, string, string)>();
            double grip = double.NaN, loadOn = double.NaN, rel = double.NaN, verdict = double.NaN; string lifter = null, receiver = receiverFromMission;
            foreach (var line in File.ReadLines(eventsPath)) {
                var e = JsonNode.FromText(line); string k = e["kind"].String; double t = e["t"].Double;
                if (k == "payload_gripped" && double.IsNaN(grip)) { grip = t; lifter = e["rover"].String; }
                else if (k == "payload_load_on" && double.IsNaN(loadOn)) loadOn = t;
                else if (k == "payload_released" && double.IsNaN(rel)) rel = t;
                else if (k == "host_verdict" && double.IsNaN(verdict)) verdict = t;
                else if (k == "top_tag_covered" && receiver == null && e["rover"].String != lifter) receiver = e["rover"].String;
            }
            if (!double.IsNaN(grip)) r.Add((grip + 1.5, lifter, "lift"));
            if (!double.IsNaN(loadOn)) r.Add((!double.IsNaN(rel) && rel > loadOn ? (loadOn + rel) / 2 : loadOn + 3.0, lifter, "carry"));   // mid-carry
            if (!double.IsNaN(rel)) r.Add((rel + 0.5, receiver ?? lifter, "set_down"));
            if (!double.IsNaN(verdict)) r.Add((verdict + 0.3, receiver ?? lifter, "delivered"));
            return r;
        }

        public static IEnumerator RenderStory(string specDir, JsonNode j, JobResult res, string outDir, string launcher, string commit, bool dirty) {
            string receiver = null;   // the handoff task's place_on (else its first supporter)
            if (j.Has("mission_file")) {
                var mf = JsonNode.FromText(File.ReadAllText(j["mission_file"].String));
                if (mf.Has("tasks")) foreach (var t in mf["tasks"].Items)
                    if (t.Has("type") && t["type"].String == "handoff") receiver = t.Has("place_on") ? t["place_on"].String : t["supporters"].Items[0].String;
            }
            var story = StoryTimes(Path.Combine(res.Dir, "events.jsonl"), receiver);
            var spec = SpecFor(specDir, j);
            var opt = Options(spec, j, Path.Combine(outDir, "replays", res.Id, "rerun"), launcher, commit, dirty);
            opt.OutDir = Path.Combine(outDir, "replays", res.Id, "rerun");
            var run = new FleetRun(spec, opt);
            var shots = new List<object>();
            int k = 0;
            foreach (var (t, rover, label) in story)
                shots.Add(new Dictionary<string, object> { ["name"] = $"replay_{k++:D2}_t{t:0.0}", ["label"] = label, ["t"] = t, ["fov"] = 45.0,
                    ["camera"] = new Dictionary<string, object> { ["target"] = "rover:" + (rover ?? "lifter_1"), ["position"] = new[] { -1.2, -1.6, 1.4 }, ["look_at"] = new[] { 0.0, 0.0, 0.1 } } });
            string json = FleetJson.Write(new Dictionary<string, object> { ["size"] = new[] { 960, 540 }, ["look"] = true, ["shots"] = shots });
            var fs = new FleetShots(spec, run, Path.Combine(outDir, "replays", res.Id, "frames"), JsonNode.FromText(json));
            double end = story.Count > 0 ? story[story.Count - 1].t + 0.5 : 5;
            run.Setup();
            fs.Begin();
            while (true) {
                while (fs.Due()) yield return fs.RenderNext();
                if (!run.Advance() || run.T > end) break;
            }
            while (fs.Pending) yield return fs.RenderNext(force: true);
            run.Finish();
            fs.Finish();
            run.Dispose();
            File.WriteAllText(Path.Combine(outDir, "replays", res.Id, "story.json"), FleetJson.Write(new Dictionary<string, object> {
                ["schema"] = "crane.fleet_success_story/v0", ["job"] = res.Id, ["outcome"] = res.Outcome, ["run_dir"] = res.Dir,
                ["frames"] = story.ConvertAll(x => (object)new Dictionary<string, object> { ["t"] = x.t, ["rover"] = x.rover, ["label"] = x.label }) }, true) + "\n");
        }

        public static IEnumerator RenderReplay(string specDir, JsonNode j, JobResult res, string outDir, string launcher, string commit, bool dirty) {
            double end = double.IsNaN(res.DivergenceT) || double.IsInfinity(res.DivergenceT) ? double.NaN : res.DivergenceT;
            string focusRover = res.DivergenceRover;
            if (double.IsNaN(end)) (end, focusRover) = FirstFailureEvent(Path.Combine(res.Dir, "events.jsonl"));
            var spec = SpecFor(specDir, j);
            var opt = Options(spec, j, Path.Combine(outDir, "replays", res.Id, "rerun"), launcher, commit, dirty);
            opt.OutDir = Path.Combine(outDir, "replays", res.Id, "rerun");
            var run = new FleetRun(spec, opt);
            string target = focusRover ?? "cargo_2";
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
