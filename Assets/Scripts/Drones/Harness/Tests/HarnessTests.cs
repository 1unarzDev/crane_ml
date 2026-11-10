// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using System.Collections.Generic;
using System.IO;
using Crane.Drones.Autonomy;
using NUnit.Framework;

namespace Crane.Drones.Harness.Tests {
    /// <summary>Harness tests: split guard, DR determinism, run determinism, replay reproduces failures (collision and
    /// local minimum), fault injection, comms failsafe. Synthetic capsule fixtures only (fast); the real-manifest
    /// reproduction of the lead's sweep is a CLI check (Harness~/Experiment: `test-lead`).</summary>
    public class HarnessTests {
        static string Repo() {
            foreach (var start in new[] { Directory.GetCurrentDirectory(), AppContext.BaseDirectory }) {
                var p = SplitGuard.FindUp(start, "Tools/DroneData/splits.json");
                if (p != null) return Path.GetFullPath(Path.Combine(Path.GetDirectoryName(p), "..", ".."));
            }
            var env = Environment.GetEnvironmentVariable("CRANE_REPO");
            if (env != null) return env;
            throw new Exception("repo root not found");
        }
        static SplitGuard Guard() { var r = Repo(); return SplitGuard.Load(Path.Combine(r, "Tools/DroneData/splits.json"), Path.Combine(r, "Tools/CraneHarness/dr_families.json")); }

        static void Refused(Action a, string what) {
            try { a(); } catch (SplitRefused) { return; }
            Assert.IsTrue(false, "expected refusal: " + what);
        }

        [Test]
        public void SplitGuardRefusesHeldoutAndCrossSplitUse() {
            var g = Guard();
            Assert.AreEqual("dev", g.Enforce("dev", new[] { 1337 }, "dev_nominal", new[] { "calm" }, new[] { "GoldenHour" }, false));
            Refused(() => g.Enforce(null, new[] { 90001 }, "none", new string[0], new string[0], false), "held-out seed");
            Refused(() => g.Enforce(null, new[] { 150000 }, "none", new string[0], new string[0], false), "unassigned seed");
            Refused(() => g.Enforce("dev", new[] { 1337 }, "none", new[] { "storm_gust" }, new string[0], false), "held-out wind on dev");
            Refused(() => g.Enforce("dev", new[] { 1337 }, "none", new string[0], new[] { "HeavyFog" }, false), "held-out lighting on dev");
            Refused(() => g.Enforce("dev", new[] { 1337 }, "train_wide", new string[0], new string[0], false), "train DR family on a dev seed");
            Refused(() => g.Enforce("dev", new[] { 1337 }, "heldout_tails", new string[0], new string[0], false), "held-out DR family");
            Refused(() => g.Enforce(null, new[] { 1337, 2500 }, "none", new string[0], new string[0], false), "mixed dev/train seeds");
            Refused(() => g.Enforce("dev", new[] { 1337 }, "no_such_family", new string[0], new string[0], false), "unregistered family");
            Refused(() => g.EnforceValues("dev", new Dictionary<string, double> { ["ThinW50Px"] = 5.0 }, "override"), "value in a reserved held-out interval");
            g.EnforceValues("dev", new Dictionary<string, double> { ["ThinW50Px"] = 3.0 }, "override");
            Assert.AreEqual("heldout", g.Enforce(null, new[] { 95000 }, "heldout_tails", new[] { "storm_gust" }, new[] { "HeavyFog" }, true));
            Refused(() => SplitGuard.Load(Path.Combine(Repo(), "no_such_splits.json"), Path.Combine(Repo(), "Tools/CraneHarness/dr_families.json")), "missing splits.json fails closed");
        }

        [Test]
        public void ExperimentSpecGuardNeedsBothOptInsForHeldout() {
            var g = Guard();
            string spec = "{\"schema\":\"crane-experiment/1\",\"name\":\"h\",\"forests\":[{\"seed\":95000,\"manifest\":\"x\"}],\"dr\":{\"family\":\"heldout_tails\"},\"wind\":\"storm_gust\",\"lighting\":\"HeavyFog\"%ALLOW%}";
            var noSpecFlag = ExperimentSpec.Parse(spec.Replace("%ALLOW%", ""), Repo());
            Refused(() => noSpecFlag.Guard(g, true, out _, (s, m) => new List<string> { "m" }), "spec did not opt in");
            var withFlag = ExperimentSpec.Parse(spec.Replace("%ALLOW%", ",\"allow_heldout\":true"), Repo());
            Refused(() => withFlag.Guard(g, false, out _, (s, m) => new List<string> { "m" }), "command line did not opt in");
            withFlag.Guard(g, true, out var runs, (s, m) => new List<string> { "m" });
            Assert.AreEqual(1, runs.Count);
            var devOverride = ExperimentSpec.Parse("{\"schema\":\"crane-experiment/1\",\"forests\":[1337],\"config\":{\"PlanPStall\":0.05}}", Repo());
            Refused(() => devOverride.Guard(g, false, out _, (s, m) => new List<string> { "m" }), "config override into a reserved interval");
        }

        [Test]
        public void DomainRandomizationIsDeterministicAndConditionIndependent() {
            var g = Guard();
            RunSpec Mk(double v, int di) { var r = new RunSpec { Experiment = "e", ForestSeed = 1337, Mission = "cross_3", Speed = v, RunSeed = 2, DrFamily = "train_wide", DrIndex = di }; DomainRandomizer.Sample(g, r, 9); r.Finish(g); return r; }
            var a = Mk(2.5, 0); var b = Mk(6.0, 0); var c = Mk(2.5, 1);
            Assert.AreEqual(HJson.Write(a.Dr), HJson.Write(b.Dr), "speed must not change the DR draw (pairing)");
            Assert.AreNotEqual(HJson.Write(a.Dr), HJson.Write(c.Dr), "sample index changes the draw");
            Assert.AreEqual(a.PairKey, b.PairKey);
            foreach (var kv in a.DrNumeric()) {
                var dim = HJson.Obj(HJson.Get(HJson.Obj(HJson.Get(g.Families["train_wide"], "dims")), kv.Key));
                Assert.GreaterOrEqual(kv.Value, HJson.Num(dim, "lo", 0) - 1e-12, kv.Key); Assert.IsTrue(kv.Value <= HJson.Num(dim, "hi", 0) + 1e-12, kv.Key);
            }
            Assert.IsTrue(g.WindPresets.ContainsKey(a.Wind), "wind preset resolved " + a.Wind);
        }

        // ------------------------------------------------------------------ closed-loop fixtures

        static HarnessContext Fixture(string key, List<(Vec3 a, Vec3 b, double r)> caps, double routeLen) {
            var ctx = new HarnessContext(Repo(), Guard()) { Log = _ => { } };
            var w = ForestManifestWorld.FromCapsules(caps, 0.0, 45);
            var ms = new MissionSpec { Name = "fx", Start = new Vec3(0, 0, 2.2), StartYaw = 0 };
            for (int i = 0; i <= (int)(routeLen / 5); i++) ms.Route.Add(new Vec3(i * 5.0, 0, 2.2));
            ctx.Register(key, w, new List<MissionSpec> { ms });
            return ctx;
        }

        static RunSpec Spec(string key, double v, Dictionary<string, object> cfg = null) {
            var r = new RunSpec { Experiment = "fixture", Split = "dev", ForestSeed = 1, Manifest = key, Profile = "Tools/Aerial/profiles/aos7_v5_f90_6s.json",
                Planner = "map_rhp", Mission = "fx", Speed = v, RunSeed = 3, Config = cfg ?? new Dictionary<string, object>() };
            r.Config["PriorUse"] = "none"; r.Config["LidarPointsPerS"] = 32000.0; r.Finish(null); return r;
        }

        /// <summary>A curtain of 2.4 cm hard branches across the route at flight height, stereo only, thin-dropout midpoint
        /// at 8 px: the stack cannot see it and flies into it.</summary>
        static HarnessContext ThinCurtain() {
            var caps = new List<(Vec3, Vec3, double)>();
            for (double z = 1.4; z <= 3.0; z += 0.2) caps.Add((new Vec3(14, -4, z), new Vec3(14, 4, z), 0.012));
            caps.Add((new Vec3(14, 6, 0), new Vec3(14, 6, 6), 0.2));
            return Fixture("thin_curtain", caps, 30);
        }
        static Dictionary<string, object> BlindCfg() => new Dictionary<string, object> { ["UseLidar"] = false, ["ThinW50Px"] = 8.0 };

        [Test]
        public void RunsAreDeterministic() {
            var ctx = ThinCurtain(); var runner = new ExperimentRunner(ctx);
            var a = runner.Run(Spec("thin_curtain", 4.0, BlindCfg()), null, "none");
            var b = runner.Run(Spec("thin_curtain", 4.0, BlindCfg()), null, "none");
            Assert.AreEqual(a.Outcome, b.Outcome); Assert.AreEqual(a.EndTick, b.EndTick);
            Assert.AreEqual(a.Result.PathLengthM, b.Result.PathLengthM, 0.0, "bit-identical path");
            Assert.AreEqual(a.Rec.Events.Count, b.Rec.Events.Count);
            Assert.AreEqual(Digest(a), Digest(b), "event stream digest");
        }

        static string Digest(RunOutput o) { var sb = new System.Text.StringBuilder(); foreach (var e in o.Rec.Events) { var f = new Dictionary<string, object>(e.f); f.Remove("WallS"); sb.Append(e.t).Append(e.kind).Append(HJson.Write(f)); } return HJson.Fnv64(sb.ToString()); }

        [Test]
        public void ReplayReproducesCollisionAndLabelsThinBranchMiss() {
            var ctx = ThinCurtain(); var runner = new ExperimentRunner(ctx);
            var rs = Spec("thin_curtain", 4.0, BlindCfg());
            var o = runner.Run(rs, null, "none");
            Assert.AreEqual("collision", o.Outcome, "fixture must fail");
            var rp = ExperimentBatch.Replay(runner, rs, o.Outcome, o.EndTick, o.Result.CollisionObject, o.Truth[o.Truth.Count - 1]);
            Assert.IsTrue((bool)rp.Row["replay_reproduced"], $"replay {rp.Outcome}@{rp.EndTick} vs {o.Outcome}@{o.EndTick}");
            Assert.AreEqual("thin_branch_miss", rp.Label, rp.LabelDetail + " " + HJson.Write(rp.Evidence));
            Assert.NotNull(rp.Bundle); Assert.IsTrue(((List<object>)rp.Bundle["truth_capsules"]).Count > 0);
        }

        [Test]
        public void ReplayReproducesLocalMinimum() {
            // the goal is enclosed by a ring of 6 m trunks (radius 4 m, 0.4 m spacing): unreachable, the stack must stall out
            var caps = new List<(Vec3, Vec3, double)>();
            for (int k = 0; k < 64; k++) { double a = 2 * Math.PI * k / 64; caps.Add((new Vec3(20 + 4 * Math.Cos(a), 4 * Math.Sin(a), 0), new Vec3(20 + 4 * Math.Cos(a), 4 * Math.Sin(a), 6), 0.15)); }
            var ctx = Fixture("pocket", caps, 20); var runner = new ExperimentRunner(ctx);
            var rs = Spec("pocket", 2.0, new Dictionary<string, object> { ["StuckS"] = 6.0 });
            var o = runner.Run(rs, null, "none");
            Assert.IsTrue(o.Outcome == "stuck" || o.Outcome == "timeout", "pocket outcome " + o.Outcome);
            var rp = ExperimentBatch.Replay(runner, rs, o.Outcome, o.EndTick, o.Result.CollisionObject, new Vec3());
            Assert.IsTrue((bool)rp.Row["replay_reproduced"]);
            Assert.AreEqual("local_minimum", rp.Label, rp.LabelDetail);
        }

        [Test]
        public void FaultsAndCommsFailsafe() {
            var ctx = ThinCurtain(); var runner = new ExperimentRunner(ctx);
            var rs = Spec("thin_curtain", 2.0);
            rs.Faults.Add(new Dictionary<string, object> { ["kind"] = "sensor_blackout", ["sensor"] = "depth", ["t0"] = 1.0, ["t1"] = 2.0 });
            rs.Comms = new Dictionary<string, object> { ["drop_p"] = 0.0, ["delay_s"] = 0.02, ["heartbeat_hz"] = 5.0, ["failsafe_timeout_s"] = 1.0, ["outages"] = new List<object> { new[] { 3.0, 30.0 } } };
            var o = runner.Run(rs, null, "none");
            Assert.AreEqual("comms_abort", o.Outcome);
            Assert.Greater(Convert.ToDouble(o.Row["faults_blackout_frames"]), 10, "15 Hz depth for 1 s blacked out");
            Assert.Greater(Convert.ToDouble(o.Row["comms_dropped"]), 0);
            var rp = ExperimentBatch.Replay(runner, rs, o.Outcome, o.EndTick, "", new Vec3());
            Assert.AreEqual("comms", rp.Label); Assert.IsTrue((bool)rp.Row["replay_reproduced"]);
            Assert.IsTrue(o.Rec.Events.Exists(e => e.kind == "comms_drop"));
        }
    }
}
