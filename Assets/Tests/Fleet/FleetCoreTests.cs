using System;
using System.IO;
using Crane.Fleet;
using Crane.Fleet.Sim;
using NUnit.Framework;
using UnityEditor.SceneManagement;

namespace Crane.Fleet.Tests {
    public class FleetCoreTests {
        const string SpecDir = "Assets/StreamingAssets/Fleet";
        static FleetSpec Spec() => FleetSpec.LoadDirectory(SpecDir);

        static MecanumGeometry G() => Spec().Describe("scout_1", "scout").Geometry;

        [Test]
        public void InverseThenForwardIsIdentity() {
            var g = G();
            var w = new double[4];
            foreach (var t in new[] { new Twist(0.3, 0, 0), new Twist(0, 0.2, 0), new Twist(0, 0, 1.0), new Twist(0.1, -0.2, 0.5) }) {
                g.Inverse(t, w);
                Twist b = g.Forward(w);
                Assert.AreEqual(t.Vx, b.Vx, 1e-12); Assert.AreEqual(t.Vy, b.Vy, 1e-12); Assert.AreEqual(t.Wz, b.Wz, 1e-12);
            }
        }

        [Test]
        public void PureStrafeSpinsDiagonalPairsOppositely() {
            var g = G();
            var w = new double[4];
            g.Inverse(new Twist(0, 0.2, 0), w);   // left strafe: fl and rr backward, fr and rl forward
            Assert.Less(w[0], 0); Assert.Greater(w[1], 0); Assert.Greater(w[2], 0); Assert.Less(w[3], 0);
            Assert.AreEqual(w[0], w[3], 1e-12); Assert.AreEqual(w[1], w[2], 1e-12);
        }

        [Test]
        public void ZeroSlipAchievedTwistEqualsCommand() {
            var spec = Spec();
            var d = spec.Describe("cargo_1", "cargo");
            var p = PlantParams.From(spec, d, spec.Floor());
            p.LateralEfficiency = 1; p.SlipSigma = 0;
            var plant = new MecanumPlant(d.Geometry, p, new FleetRng(7));
            var cmd = new Twist(0.2, -0.15, 0.4);
            var v = new Twist();
            for (int i = 0; i < 4000; i++) {
                plant.Step(cmd, v, spec.Dt);
                v.Vx += plant.Ax * spec.Dt; v.Vy += plant.Ay * spec.Dt; v.Wz += plant.Alpha * spec.Dt;
            }
            Assert.AreEqual(cmd.Vx, v.Vx, 1e-9); Assert.AreEqual(cmd.Vy, v.Vy, 1e-9); Assert.AreEqual(cmd.Wz, v.Wz, 1e-9);
        }

        [Test]
        public void LateralEfficiencyScalesOnlyStrafe() {
            var spec = Spec();
            var d = spec.Describe("cargo_1", "cargo");
            var floor = spec.Floor();
            var p = PlantParams.From(spec, d, floor);
            p.SlipSigma = 0;
            var plant = new MecanumPlant(d.Geometry, p, new FleetRng(7));
            var v = new Twist();
            for (int i = 0; i < 4000; i++) {
                plant.Step(new Twist(0.1, 0.2, 0), v, spec.Dt);
                v.Vx += plant.Ax * spec.Dt; v.Vy += plant.Ay * spec.Dt; v.Wz += plant.Alpha * spec.Dt;
            }
            Assert.AreEqual(0.1, v.Vx, 1e-9);
            Assert.AreEqual(0.2 * floor.LateralEfficiency, v.Vy, 1e-9);
        }

        [Test]
        public void WheelSpeedLimitPreservesDirection() {
            var g = G();
            var w = new double[4];
            var t = new Twist(0.5, 0, 1.5);   // within body limits, beyond the wheel limit
            double s = g.Saturate(ref t, w);
            Assert.Less(s, 1);
            double m = 0;
            foreach (double x in w) m = Math.Max(m, Math.Abs(x));
            Assert.AreEqual(g.MaxWheelSpeed, m, 1e-9);
            Twist b = g.Forward(w);
            Assert.AreEqual(t.Wz / t.Vx, b.Wz / b.Vx, 1e-9);
        }

        [Test]
        public void NormalLoadsSumToWeightAndFollowCom() {
            var spec = Spec();
            var d = spec.Describe("cargo_1", "cargo");
            var p = PlantParams.From(spec, d, spec.Floor());
            p.ComX = 0.05; p.ComY = 0;
            var plant = new MecanumPlant(d.Geometry, p, new FleetRng(1));
            plant.Step(new Twist(), new Twist(), spec.Dt);
            double sum = 0;
            foreach (double n in plant.NormalLoad) sum += n;
            Assert.AreEqual(p.Mass * p.Gravity, sum, 1e-9);
            Assert.Greater(plant.NormalLoad[0], plant.NormalLoad[2]);   // front carries more
            Assert.AreEqual(plant.NormalLoad[0], plant.NormalLoad[1], 1e-9);
        }

        [Test]
        public void TractionLimitCapsAcceleration() {
            var spec = Spec();
            var d = spec.Describe("cargo_1", "cargo");
            var p = PlantParams.From(spec, d, spec.Floor());
            p.SlipSigma = 0;
            var plant = new MecanumPlant(d.Geometry, p, new FleetRng(1));
            for (int i = 0; i < 200; i++) plant.Step(new Twist(0.5, 0, 0), new Twist(), spec.Dt);  // body held still
            Assert.Less(plant.TractionScale, 1);
            Assert.LessOrEqual(Math.Abs(plant.Ax), p.Mu * p.Gravity + 1e-9);
        }

        [Test]
        public void EncodersQuantizeAndOdometryFollowsIdealWheels() {
            var g = G();
            var enc = new WheelEncoders(g.CountsPerRev);
            enc.Sample(new[] { 2 * Math.PI, -2 * Math.PI, 0.001, 1.0 });
            Assert.AreEqual(g.CountsPerRev, enc.Ticks[0]);
            Assert.AreEqual(-g.CountsPerRev, enc.Ticks[1]);
            Assert.AreEqual((long)Math.Floor(0.001 * g.CountsPerRev / (2 * Math.PI)), enc.Ticks[2]);

            var e2 = new WheelEncoders(g.CountsPerRev);
            var odo = new WheelOdometry(g, e2, 0, 0, 0);
            var w = new double[4];
            var angle = new double[4];
            g.Inverse(new Twist(0.2, 0, 0), w);
            for (int i = 0; i < 1000; i++) {
                for (int k = 0; k < 4; k++) angle[k] += w[k] * 0.01;
                e2.Sample(angle);
                odo.IntegrateEncoders(0.01);
            }
            Assert.AreEqual(2.0, odo.X, 2 * Math.PI * g.Radius / g.CountsPerRev * 2);
            Assert.AreEqual(0.0, odo.Y, 1e-9);
        }

        [Test]
        public void RngStreamsAreDeterministicAndIndependent() {
            var a = FleetRng.For(5, "scout_1/slip");
            var b = FleetRng.For(5, "scout_1/slip");
            var c = FleetRng.For(5, "scout_1/imu");
            for (int i = 0; i < 100; i++) Assert.AreEqual(a.Gaussian(), b.Gaussian());
            Assert.AreNotEqual(FleetRng.For(5, "scout_1/slip").NextU64(), c.NextU64());
        }

        [Test]
        public void EveryRoleResolvesWithAllFrames() {
            var spec = Spec();
            foreach (var (id, role) in spec.FleetDefault()) {
                var d = spec.Describe(id, role);
                Assert.AreEqual(4, d.Wheels.Count, id);
                Assert.AreEqual(4, d.Faces.Count, id);
                Assert.Greater(d.Sensors.Count, 0, id);
                Assert.Greater(d.Mass, spec.P["chassis.base_mass"].Double, id);
                Assert.AreEqual("/fleet/" + id, d.Namespace);
                // base_link at the body-underside centre (asset generator convention): origin rides ground_clearance high
                double clr = spec.P["chassis.ground_clearance"].Double;
                Assert.AreEqual(clr, d.OriginHeight, 1e-12);
                Assert.AreEqual(d.Wheels[0].Radius - clr, d.Wheels[0].Axle.Z, 1e-12);
                Assert.IsNotNull(d.Color);
            }
            Assert.AreEqual("fleet/warehouse_a/v0/qual/3", RunIds.Make(spec.S["scenario_id"].String, "qual", 3));
        }

        [Test]
        public void MatingPosePutsFacesTogether() {
            var spec = Spec();
            var d = spec.Describe("cargo_1", "cargo");
            var a = new Pose2(1, 2, 0.3);
            Pose2 b = CadSwapCheck.MatingPose(a, d.Face("front").Pose, d.Face("rear").Pose);
            FramePose f = d.Face("front").Pose, r = d.Face("rear").Pose;
            double ax = a.X + Math.Cos(a.Yaw) * f.X - Math.Sin(a.Yaw) * f.Y, ay = a.Y + Math.Sin(a.Yaw) * f.X + Math.Cos(a.Yaw) * f.Y;
            double bx = b.X + Math.Cos(b.Yaw) * r.X - Math.Sin(b.Yaw) * r.Y, by = b.Y + Math.Sin(b.Yaw) * r.X + Math.Cos(b.Yaw) * r.Y;
            Assert.AreEqual(ax, bx, 1e-12); Assert.AreEqual(ay, by, 1e-12);
            Assert.AreEqual(Math.PI, Math.Abs(Angles.Wrap(b.Yaw + r.Yaw - (a.Yaw + f.Yaw))), 1e-6);
        }

        [Test]
        public void JsonRoundTrip() {
            object v = FleetJson.Parse("{\"a\": [1, 2.5, -3e-2], \"b\": {\"value\": 0.6, \"provenance\": \"provisional\"}, \"c\": \"x\\\"y\", \"d\": true, \"e\": null}");
            string s = FleetJson.Write(v);
            Assert.AreEqual(s, FleetJson.Write(FleetJson.Parse(s)));
            var n = new JsonNode(v);
            Assert.AreEqual(0.6, n["b"].Double);
            Assert.AreEqual("provisional", n["b"].Provenance);
        }

        [Test]
        public void ObjParserSplitsMaterialsAndKeepsUvsAndNormals() {
            var m = ObjMesh.Parse("v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nvt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nvn 0 0 1\n" +
                "usemtl a\nf 1/1/1 2/2/1 3/3/1\nusemtl b\nf 1/1/1 3/3/1 4/4/1\nusemtl a\nf 1/1/1 2/2/1 4/4/1\n");
            CollectionAssert.AreEqual(new[] { "a", "b" }, m.SubmeshMaterials);
            Assert.AreEqual(6, m.Submeshes[0].Count);
            Assert.AreEqual(3, m.Submeshes[1].Count);
            Assert.AreEqual(4, m.Positions.Count);
            Assert.IsNotNull(m.Uvs[2]); Assert.IsNotNull(m.Normals[2]);
        }

        [Test]
        public void AssetManifestsResolveForEveryRoleAndBuildVisuals() {
            var spec = Spec();
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var root = new UnityEngine.GameObject("t");
            try {
                foreach (var (id, role) in spec.FleetDefault()) {
                    var d = spec.Describe(id, role);
                    Assert.IsTrue(d.MeshManifestPresent, id);
                    Assert.AreEqual("asset_manifest", d.SensorFrameSource, id);
                    Assert.AreEqual(4, d.Asset.Hooks.Count, id);
                    var rig = new RoverRig(spec, d, spec.Floor(), 1, root.transform, new Pose2(0, 0, 0));
                    Assert.IsNotNull(rig.Visual, id);
                    Assert.Greater(rig.Visual.Root.GetComponentsInChildren<UnityEngine.MeshRenderer>().Length, 8, id);
                }
                Assert.AreEqual(1, spec.Describe("cargo_2", "cargo").InstanceIndex);
                Assert.IsNotNull(spec.Describe("lifter_1", "lifter").Asset.LiftPart);
            }
            finally { UnityEngine.Object.DestroyImmediate(root); }
        }

        [Test]
        public void DockingProfileComesFromTheFleetSpec() {
            var p = Crane.Fleet.Docking.DockingProfileBuilder.Build(Spec());
            Assert.AreEqual(Spec().P["docking.hook_lateral_spacing"].Double, p.HookLateralSpacing.Value);
            Assert.AreEqual(4, p.FaceFrames.Count);
            Assert.AreEqual(0, p.Validate().Count);
        }

        [Test]
        public void ConnectorMachineLatchesOnAnIdealApproach() {
            var spec = Spec();
            var model = new Crane.Fleet.Docking.ContactModel(Crane.Fleet.Docking.DockingProfileBuilder.Build(spec), new Crane.Fleet.Docking.DockingAssumptions());
            var mc = new Crane.Fleet.Docking.ConnectorMachine(model, 42, "t");
            double gap = 0.05;
            var cmd = Crane.Fleet.Docking.DockingCommand.Approach;
            for (int i = 0; i < 2000 && mc.State != Crane.Fleet.Docking.ConnectorState.LatchedVerified; i++) {
                mc.Step(0.005, new Crane.Fleet.Docking.ConnectorInputs { Pose = new Crane.Fleet.Docking.FacePose(0.001, gap, 0.002, 0.0005), ClosingSpeed = 0.03, Command = cmd });
                gap = Math.Max(0, gap - 0.03 * 0.005);
                if (mc.State == Crane.Fleet.Docking.ConnectorState.MagnetCaptured) cmd = Crane.Fleet.Docking.DockingCommand.EngageHooks;
                if (mc.SensedA && mc.SensedB) cmd = Crane.Fleet.Docking.DockingCommand.Verify;
            }
            Assert.AreEqual(Crane.Fleet.Docking.ConnectorState.LatchedVerified, mc.State);
        }
    }

    /// <summary>Engine-level checks: the CAD-swap check and a short run's determinism, stepped with Physics.Simulate in edit mode.</summary>
    public class FleetSimTests {
        const string SpecDir = "Assets/StreamingAssets/Fleet";

        [Test]
        public void WarehouseGeometryMatchesScenario() {
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var spec = FleetSpec.LoadDirectory(SpecDir);
            var world = new FleetWorld(spec);
            UnityEngine.Physics.SyncTransforms();
            try {
                double[] size = spec.S["arena.size_xy"].Doubles;
                double margin = spec.O["scene.floor_margin"].Double;
                var floor = world.Root.transform.Find("floor").GetComponent<UnityEngine.Collider>().bounds;
                Assert.AreEqual(-margin, floor.min.x, 1e-4); Assert.AreEqual(size[0] + margin, floor.max.x, 1e-4);   // east = Unity x
                Assert.AreEqual(-margin, floor.min.z, 1e-4); Assert.AreEqual(size[1] + margin, floor.max.z, 1e-4);   // north = Unity z
                Assert.AreEqual(0.0, floor.max.y, 1e-6);
                var divider = world.Root.transform.Find("divider_0").GetComponent<UnityEngine.Collider>().bounds;
                double th = spec.S["walls"].Items[0]["thickness"].Double;
                Assert.AreEqual(6.0 - th / 2, divider.min.x, 1e-4);
                double door = spec.S["doorway.width.default"].Double, cy = spec.S["doorway.centre_y"].Double;
                Assert.AreEqual(cy - door / 2, divider.max.z, 1e-4);
                var shelf = world.Root.transform.Find("shelf_a").GetComponent<UnityEngine.Collider>().bounds;
                Assert.AreEqual(0.5, shelf.size.x, 1e-4); Assert.AreEqual(2.4, shelf.size.z, 1e-4); Assert.AreEqual(2.0, shelf.size.y, 1e-4);
            }
            finally { world.Destroy(); }
        }

        [Test]
        public void CadSwapCheckPasses() {
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var r = CadSwapCheck.Run(FleetSpec.LoadDirectory(SpecDir), null);
            Assert.IsTrue(r.Pass, string.Join("\n", r.Failures));
        }

        [Test]
        public void RunIsIndependentOfWhatRanBeforeInTheProcess() {
            string root = Path.Combine(Path.GetTempPath(), "crane-fleet-hist-" + Guid.NewGuid().ToString("N"));
            try {
                var spec = FleetSpec.LoadDirectory(SpecDir);
                EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
                var a = new FleetRun(spec, new FleetRunOptions { Seed = 5, OutDir = Path.Combine(root, "a"), MaxDuration = 4.0 });
                a.RunToEnd(); a.Dispose();
                CadSwapCheck.Run(spec, null);   // other physics work in between
                EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
                var b = new FleetRun(spec, new FleetRunOptions { Seed = 5, OutDir = Path.Combine(root, "b"), MaxDuration = 4.0 });
                b.RunToEnd(); b.Dispose();
                Assert.AreEqual(File.ReadAllText(Path.Combine(root, "a", "states.jsonl")), File.ReadAllText(Path.Combine(root, "b", "states.jsonl")),
                    "physics scene per run: " + b.Phys?.Describe);
            }
            finally { if (Directory.Exists(root)) Directory.Delete(root, true); }
        }

        [Test]
        public void ShortRunIsDeterministicForASeed() {
            string root = Path.Combine(Path.GetTempPath(), "crane-fleet-det-" + Guid.NewGuid().ToString("N"));
            try {
                string[] text = new string[3];
                long[] seeds = { 11, 11, 12 };
                for (int i = 0; i < 3; i++) {
                    EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
                    var run = new FleetRun(FleetSpec.LoadDirectory(SpecDir), new FleetRunOptions {
                        Seed = seeds[i], OutDir = Path.Combine(root, i.ToString()), MaxDuration = 6.0 });
                    run.RunToEnd();
                    run.Dispose();
                    text[i] = File.ReadAllText(Path.Combine(root, i.ToString(), "states.jsonl"));
                }
                Assert.Greater(text[0].Length, 1000);
                Assert.AreEqual(text[0], text[1], "same seed must reproduce states.jsonl byte for byte");
                Assert.AreNotEqual(text[0], text[2], "a different seed must change the run");
            }
            finally { if (Directory.Exists(root)) Directory.Delete(root, true); }
        }
    }
}
