using System;
using System.Collections.Generic;
using System.IO;
using Crane.Fleet.Sim;
using NUnit.Framework;
using UnityEditor.SceneManagement;

namespace Crane.Fleet.Tests {
    /// <summary>Per-wheel tractive plant and module-graph spawning (CRANE_MODULAR_PLAN P1/M1 slice). T0 checks of the plant alone are in
    /// Tools/Fleet/offline/tractive_t0; these run it through the overlay parameters and PhysX.</summary>
    public class FleetModuleTests {
        const string SpecDir = "Assets/StreamingAssets/Fleet";
        const string Graph = "Tools/Fleet/probes/probe_graph_v0.json";

        [Test]
        public void TractiveMecanumKinematicsMatchTheProfileGeometry() {
            var spec = FleetSpec.LoadDirectory(SpecDir);
            var d = spec.Describe("lifter_1", "lifter");
            var tp = new TractivePlant(d.WheelUnits, TractionParams.From(spec, d, spec.Floor()));
            var a = new double[4]; var b = new double[4];
            foreach (var t in new[] { new Twist(0.2, 0, 0), new Twist(0, 0.2, 0), new Twist(0, 0, 1), new Twist(0.1, -0.2, 0.7) }) {
                d.Geometry.Inverse(t, a); tp.Inverse(t, b);
                for (int i = 0; i < 4; i++) Assert.AreEqual(a[i], b[i], 1e-9, $"{t} wheel {i}");
            }
        }

        [Test]
        public void EveryProbeGraphTypeResolves() {
            var spec = FleetSpec.LoadDirectory(SpecDir);
            var g = JsonNode.FromText(File.ReadAllText(Graph));
            foreach (var kv in g["types"].Members) {
                var d = ModuleDescription.Resolve(spec, "m_" + kv.Key, kv.Key, kv.Value, 0);
                Assert.AreEqual("tractive", d.PlantModel);
                Assert.AreEqual(kv.Value["wheels"].Items.Count, d.WheelUnits.Count, kv.Key);
                Assert.AreEqual(d.WheelUnits.Count, d.Wheels.Count, kv.Key);
                Assert.Greater(d.Mass, 0);
                if (kv.Key == "lifter" || kv.Key == "mecanum_pod") Assert.IsNotNull(d.Geometry, kv.Key + " is a 4-wheel mecanum set");
                if (kv.Key == "sensor_mast") { Assert.AreEqual(0, d.WheelUnits.Count); Assert.AreEqual(0.0, d.OriginHeight); }
            }
        }

        const string Coupled = "Tools/Fleet/probes/coupled_graph_v0.json";

        [Test]
        public void RigidCouplingsAreAcceptedOnlyWhenTheGraphPosesMateTheFaces() {
            Assert.IsNull(ModuleProbeMission.Unsupported(JsonNode.FromText("{\"graph\": \"" + Coupled + "\"}")));
            Assert.IsNull(ModuleProbeMission.Unsupported(JsonNode.FromText("{\"graph\": \"" + Graph + "\"}")));
            // move c2 by 5 cm: the faces no longer coincide
            var g = (System.Collections.Generic.Dictionary<string, object>)FleetJson.Parse(File.ReadAllText(Coupled));
            foreach (var m in (System.Collections.Generic.List<object>)g["modules"]) {
                var d = (System.Collections.Generic.Dictionary<string, object>)m;
                if ((string)d["id"] == "c2") d["x"] = (double)d["x"] - 0.05;
            }
            var bad = JsonNode.FromText("{\"graph\": " + FleetJson.Write(g) + "}");
            StringAssert.Contains("do not mate", ModuleProbeMission.Unsupported(bad));
        }

        /// <summary>M2 command side in PhysX: pre-latched assemblies driven by one group twist. The inline pair and the L translate and
        /// spin as one body; the mixed diff + mecanum pair does not strafe (one hard lateral row). UNITY-PENDING when written (2026-11-10).</summary>
        [Test]
        public void PreLatchedAssembliesDriveAsOneGroup() {
            string root = Path.Combine(Path.GetTempPath(), "crane-fleet-m2-" + Guid.NewGuid().ToString("N"));
            try {
                Directory.CreateDirectory(root);
                string probe = Path.Combine(root, "probe.json");
                File.WriteAllText(probe, "{\"schema\": \"crane.fleet_probe/v0\", \"tip_limit_rad\": 0.5, \"max_duration_s\": 12, \"segments\": ["
                    + "{\"name\": \"fwd\", \"duration\": 2.0, \"twist\": [0.2, 0, 0]}, {\"name\": \"strafe\", \"duration\": 2.0, \"twist\": [0, 0.15, 0]},"
                    + "{\"name\": \"spin\", \"duration\": 2.0, \"twist\": [0, 0, 0.6]}]}");
                var cfg = JsonNode.FromText("{\"graph\": \"" + Coupled + "\", \"origin\": [7.6, 1.0, 0.0]}");
                EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
                var run = new FleetRun(FleetSpec.LoadDirectory(SpecDir), new FleetRunOptions { Seed = 1, OutDir = Path.Combine(root, "run"), MaxDuration = 12,
                    Mission = new ModuleProbeMission(cfg, probe) });
                run.RunToEnd(); run.Dispose();
                var p = JsonNode.FromText(File.ReadAllText(Path.Combine(root, "run", "manifest.json")))["results.probe"];
                Assert.IsTrue(p["complete"].Bool);
                Assert.AreEqual(3, p["assemblies"].Items.Count);
                var rows = new Dictionary<string, double[]>();
                foreach (var r in p["rows"].Items) rows[r["module"].String + "/" + r["segment"].String] = r["truth_displacement_body"].Doubles;
                // inline pair: both members move the commanded 0.4 m forward and 0.3 m sideways, together
                foreach (var id in new[] { "c1", "c2" }) { Assert.AreEqual(0.4, rows[id + "/fwd"][0], 0.07, id); Assert.AreEqual(0.3, rows[id + "/strafe"][1], 0.07, id); }
                Assert.AreEqual(rows["c1/spin"][2], rows["c2/spin"][2], 0.02, "rigid: same yaw change");
                Assert.AreEqual(1.2, rows["c1/spin"][2], 0.2);
                // mixed diff + mecanum: the strafe command produces (almost) no lateral motion of the diff module
                Assert.Less(Math.Abs(rows["dm/strafe"][1]), 0.05);
            }
            finally { if (Directory.Exists(root)) Directory.Delete(root, true); }
        }

        [Test]
        public void ProbeShowsEachWheelArraysConstraints() {
            string root = Path.Combine(Path.GetTempPath(), "crane-fleet-probe-" + Guid.NewGuid().ToString("N"));
            try {
                Directory.CreateDirectory(root);
                string probe = Path.Combine(root, "probe.json");
                File.WriteAllText(probe, "{\"schema\": \"crane.fleet_probe/v0\", \"tip_limit_rad\": 0.5, \"max_duration_s\": 10, \"segments\": ["
                    + "{\"name\": \"fwd\", \"duration\": 2.0, \"twist\": [0.2, 0, 0]}, {\"name\": \"strafe\", \"duration\": 2.0, \"twist\": [0, 0.15, 0]}]}");
                var cfg = JsonNode.FromText("{\"graph\": \"" + Graph + "\", \"origin\": [7.0, 0.7, 0.0]}");
                EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
                var run = new FleetRun(FleetSpec.LoadDirectory(SpecDir), new FleetRunOptions { Seed = 1, OutDir = Path.Combine(root, "run"), MaxDuration = 10,
                    Mission = new ModuleProbeMission(cfg, probe) });
                run.RunToEnd(); run.Dispose();
                var p = JsonNode.FromText(File.ReadAllText(Path.Combine(root, "run", "manifest.json")))["results.probe"];
                Assert.IsTrue(p["complete"].Bool);
                Assert.AreEqual(0, p["tipped"].Items.Count);
                var rows = new Dictionary<string, double[]>();
                foreach (var r in p["rows"].Items) rows[r["module"].String + "/" + r["segment"].String] = r["truth_displacement_body"].Doubles;
                // healthy diff carrier: drives forward (motor lag only), cannot strafe
                Assert.AreEqual(0.4, rows["dc_ok/fwd"][0], 0.06);
                Assert.Less(Math.Abs(rows["dc_ok/strafe"][1]), 0.02);
                // healthy lifter (mecanum): strafes
                Assert.AreEqual(0.3, rows["lift_ok/strafe"][1], 0.08);
                // wheel-less mast: stays
                Assert.Less(Math.Abs(rows["mast/fwd"][0]) + Math.Abs(rows["mast/strafe"][1]), 0.01);
                // a dead_free drive wheel on the diff carrier: the remaining wheel turns it instead of driving straight
                Assert.Greater(Math.Abs(rows["dc_free0/fwd"][2]), 0.1);
            }
            finally { if (Directory.Exists(root)) Directory.Delete(root, true); }
        }
    }
}
