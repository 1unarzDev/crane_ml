// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using System.Collections.Generic;
using System.IO;
using Crane.Drones.Autonomy;

namespace Crane.Drones.Harness {
    /// <summary>
    /// Batch driver: guard -> expand -> run (sharded, resumable) -> label failures with a deterministic replay pass ->
    /// results_shard{k}.jsonl, runs/{run_id}/{run_spec.json, manifest.json, events.jsonl[, state.csv], replay_bundle.json}.
    /// </summary>
    public static class ExperimentBatch {
        public static int Run(HarnessContext ctx, ExperimentSpec spec, string outDir, int shard, int nshards, bool allowHeldoutCli, int limit = -1) {
            var runner = new ExperimentRunner(ctx);
            List<RunSpec> runs;
            string split = spec.Guard(ctx.Guard, allowHeldoutCli, out runs, (seed, manifest) => {
                if (ctx.World(manifest).w.Seed != seed) throw new SplitRefused($"manifest {manifest} has seed {ctx.World(manifest).w.Seed}, spec says {seed}");
                return ctx.MissionNames(manifest, spec.Cross, spec.Corridor);
            });
            Directory.CreateDirectory(outDir);
            File.WriteAllText(Path.Combine(outDir, "experiment.json"), HJson.Write(new Dictionary<string, object> {
                ["schema"] = "crane-experiment-manifest/1", ["name"] = spec.Name, ["split"] = split, ["spec_digest"] = HJson.Fnv64(spec.Raw),
                ["splits_digest"] = ctx.Guard.SplitsSha, ["dr_families_digest"] = ctx.Guard.FamiliesSha, ["runs"] = runs.Count,
                ["run_ids"] = runs.ConvertAll(r => (object)r.RunId), ["spec"] = MiniJson.Parse(spec.Raw) }));
            string resPath = Path.Combine(outDir, $"results_shard{shard}.jsonl");
            var done = new HashSet<string>();
            if (File.Exists(resPath)) foreach (var line in File.ReadAllLines(resPath)) { var o = HJson.Obj(MiniJson.Parse(line)); if (o != null) done.Add(HJson.Text(o, "run_id", "")); }
            int n = 0;
            using (var res = new StreamWriter(resPath, true)) {
                for (int i = 0; i < runs.Count; i++) {
                    if (i % nshards != shard) continue;
                    var rs = runs[i]; if (done.Contains(rs.RunId)) continue;
                    if (limit >= 0 && n >= limit) break;
                    n++;
                    var row = RunAndLabel(runner, rs, outDir, spec.Logs, spec.LabelFailures, spec.BundleFailures);
                    res.WriteLine(HJson.Write(row)); res.Flush();
                }
            }
            return 0;
        }

        public static Dictionary<string, object> RunAndLabel(ExperimentRunner runner, RunSpec rs, string outDir, string logs, bool label, bool bundle) {
            string runDir = Path.Combine(outDir, "runs", rs.RunId); Directory.CreateDirectory(runDir);
            File.WriteAllText(Path.Combine(runDir, "run_spec.json"), HJson.Write(rs.ToDict()));
            var sw = System.Diagnostics.Stopwatch.StartNew();
            var o = runner.Run(rs, runDir, logs);
            var row = o.Row; row["wall_s"] = sw.Elapsed.TotalSeconds;
            if (o.Outcome != "success" && label) {
                var rp = Replay(runner, rs, o.Outcome, o.EndTick, o.Result.CollisionObject, o.Truth.Count > 0 ? o.Truth[o.Truth.Count - 1] : new Vec3());
                row["label"] = rp.Label; row["label_detail"] = rp.LabelDetail; row["label_evidence"] = rp.Evidence;
                row["replay_reproduced"] = rp.Row.TryGetValue("replay_reproduced", out var ok) && ok is bool b && b;
                if (bundle && rp.Bundle != null) File.WriteAllText(Path.Combine(runDir, "replay_bundle.json"), HJson.Write(rp.Bundle));
            } else row["label"] = o.Outcome == "success" ? "" : "unlabelled";
            runner.Ctx.Log($"[harness] {rs.RunId,-48} -> {o.Outcome,-11} {row["label"],-16} t={o.EndTick * 0.005,6:0.0}s wall={sw.Elapsed.TotalSeconds:0.0}s {o.Result.CollisionObject}");
            return row;
        }

        /// <summary>Deterministic re-execution up to the recorded end tick with the probe attached; checks that the same
        /// outcome happens at the same tick against the same object.</summary>
        public static RunOutput Replay(ExperimentRunner runner, RunSpec rs, string outcome, long endTick, string collisionObject, Vec3 contact) {
            var target = new ReplayTarget { EndTick = endTick, Outcome = outcome, CollisionObject = collisionObject ?? "", ContactP = contact, HasContact = outcome == "collision" };
            var rp = runner.Run(rs, null, "none", target);
            bool same = rp.Outcome == outcome && rp.EndTick == endTick && (rp.Result.CollisionObject ?? "") == (collisionObject ?? "");
            rp.Row["replay_reproduced"] = same;
            if (rp.Evidence != null) { rp.Evidence["replay_reproduced"] = same; rp.Evidence["replay_end_tick"] = rp.EndTick; rp.Evidence["recorded_end_tick"] = endTick; }
            if (rp.Bundle != null) rp.Bundle["replay_reproduced"] = same;
            return rp;
        }

        /// <summary>Replay by run id from an experiment output directory (run_spec.json + the results row).</summary>
        public static RunOutput ReplayById(HarnessContext ctx, string outDir, string runId, bool allowHeldoutCli) {
            string runDir = Path.Combine(outDir, "runs", runId);
            var rs = RunSpec.FromDict(HJson.Obj(MiniJson.Parse(File.ReadAllText(Path.Combine(runDir, "run_spec.json")))));
            ctx.Guard.Enforce(rs.Split, new[] { rs.ForestSeed }, rs.DrFamily, new[] { rs.Wind }, new[] { rs.Lighting }, allowHeldoutCli);
            ctx.Guard.EnforceValues(rs.Split, rs.DrNumeric(), "DR sample " + runId);
            Dictionary<string, object> row = null;
            foreach (var f in Directory.GetFiles(outDir, "results*.jsonl"))
                foreach (var line in File.ReadAllLines(f)) { var o = HJson.Obj(MiniJson.Parse(line)); if (o != null && HJson.Text(o, "run_id", "") == runId) row = o; }
            if (row == null) throw new ArgumentException("run id " + runId + " not found in " + outDir + "/results*.jsonl");
            string outcome = HJson.Text(row, "outcome", ""); long endTick = (long)HJson.Num(row, "end_tick", -1);
            string coll = HJson.Text(row, "r_CollisionObject", "");
            var runner = new ExperimentRunner(ctx);
            // contact point: re-derive from a plain pass when the bundle is absent (trajectory is not stored in the row)
            Vec3 contact = new Vec3();
            if (outcome == "collision") { var p1 = runner.Run(rs, null, "none", new ReplayTarget { EndTick = endTick }); contact = p1.Truth[p1.Truth.Count - 1]; }
            var rp = Replay(runner, rs, outcome, endTick, coll, contact);
            if (rp.Bundle != null) File.WriteAllText(Path.Combine(runDir, "replay_bundle.json"), HJson.Write(rp.Bundle));
            return rp;
        }
    }
}
