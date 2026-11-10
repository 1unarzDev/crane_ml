// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using System.Collections.Generic;
using System.IO;
using NUnit.Framework;
using Sim.Physics.Aerial;

namespace Crane.Drones.Autonomy.Tests {
    /// <summary>Fixture tests for the autonomy core (synthetic capsule worlds, flat ground at z = 0). Run with
    /// -testFilter Crane.Drones.Autonomy.Tests or -assemblyNames Crane.Drones.Autonomy.Tests.</summary>
    public class DroneAutonomyTests {
        static ForestManifestWorld Wall(double x, double r = 0.3, double half = 40) {
            var caps = new List<(Vec3, Vec3, double)>();
            for (int i = -10; i <= 10; i++) caps.Add((new Vec3(x, i * 0.5, 0), new Vec3(x, i * 0.5, 6), r));
            return ForestManifestWorld.FromCapsules(caps, 0.0, half);
        }

        static VehicleSpecV2 Spec() {
            string root = Path.GetFullPath(Path.Combine(UnityEngine.Application.dataPath, ".."));
            var p = CraneAerialProfile.LoadFile(CraneAerialProfile.Resolve("aos7_v5_f90_6s", Path.Combine(root, "Tools", "Aerial", "profiles")), true, true);
            var s = p.Vehicle; s.GroundHeight = -1000; return s;
        }

        [Test]
        public void DepthImageSeesWallAtTheRightRange() {
            var w = Wall(6.0);
            var c = new AutonomyConfig { InvalidP = 0, DispSigmaPx = 0 };
            var cam = new DepthRaycastSensor(c, w, 1);
            var f = cam.Tick(0, Pose.FromYaw(new Vec3(0, 0, 2), 0), new Vec3(), new Vec3());
            Assert.NotNull(f);
            float z = f.Depth[(c.DepthHeight / 2) * c.DepthWidth + c.DepthWidth / 2];
            Assert.AreEqual(6.0 - 0.3 - c.DepthMountForward, z, 0.05);
            Assert.AreEqual(f.AcqNs + (long)(c.DepthLatencyS * 1e9), f.AvailNs);
            Assert.IsNull(cam.Tick(5_000_000, Pose.FromYaw(new Vec3(0, 0, 2), 0), new Vec3(), new Vec3()), "15 Hz: no frame 5 ms later");
        }

        [Test]
        public void ThinTwigsDropOutTrunksDoNot() {
            var caps = new List<(Vec3, Vec3, double)> { (new Vec3(10, -0.05, 1), new Vec3(10, -0.05, 3), 0.004), (new Vec3(10, 1.5, 0), new Vec3(10, 1.5, 6), 0.25) };
            var w = ForestManifestWorld.FromCapsules(caps, 0.0, 30);
            var c = new AutonomyConfig { InvalidP = 0 };
            var cam = new DepthRaycastSensor(c, w, 7);
            int twigSeen = 0, frames = 40;
            for (int k = 0; k < frames; k++) {
                var f = cam.Tick(k * 100_000_000L, Pose.FromYaw(new Vec3(0, 0, 2), 0), new Vec3(), new Vec3());
                bool seen = false;
                for (int v = 0; v < c.DepthHeight; v++) for (int u = 0; u < c.DepthWidth; u++) { float z = f.Depth[v * c.DepthWidth + u]; if (z > 9.0f && z < 10.5f && Math.Abs(u - c.DepthWidth / 2) <= 1 && Math.Abs(v - c.DepthHeight / 2) <= 3) seen = true; }
                if (seen) twigSeen++;
            }
            Assert.Less(twigSeen, frames / 4, "an 8 mm twig at 10 m (~0.3 native px) should almost always drop out");
        }

        [Test]
        public void LidarCarriesPerPointTimesAndMotionSkewThatDeskews() {
            var w = Wall(8.0, 0.4);
            var c = new AutonomyConfig { LidarDropP = 0, LidarRangeSigma = 0 };
            var l = new LidarL2Sensor(c, w, 3, 0);
            LidarScan s = null; double vx = 3.0; var poses = new Dictionary<long, Pose>();
            for (long t = 0; s == null; t += c.TickNs) {
                var pose = Pose.FromYaw(new Vec3(vx * t * 1e-9, 0, 2), 0); poses[t] = pose;
                s = l.Tick(t, c.TickNs, pose);
            }
            Assert.Greater(s.Count, 100);
            for (int i = 1; i < s.Count; i++) Assert.GreaterOrEqual(s.OffsetNs[i], s.OffsetNs[i - 1]);
            Assert.Less(s.OffsetNs[s.Count - 1], s.EndNs - s.StartNs);
            double rawSpread = 0, deskewMax = 0; double xmin = 1e9, xmax = -1e9;
            for (int i = 0; i < s.Count; i++) {
                if (s.X[i] < 1.0 || Math.Abs(s.Y[i]) > 3 || s.Z[i] < -1.5) continue;   // wall returns only
                long tq = s.StartNs + s.OffsetNs[i]; tq -= tq % c.TickNs;
                var p = DepthRaycastSensor.Compose(poses[tq], l.Mount);
                var world = p.ToWorld(new Vec3(s.X[i], s.Y[i], s.Z[i]));
                xmin = Math.Min(xmin, s.X[i]); xmax = Math.Max(xmax, s.X[i]);
                deskewMax = Math.Max(deskewMax, Math.Abs(world.X - (8.0 - 0.4)) * (Math.Abs(s.Y[i]) < 0.1 ? 1 : 0));
            }
            rawSpread = xmax - xmin;
            Assert.Greater(rawSpread, 0.15, "a 0.1 s sweep at 3 m/s skews the wall by ~0.3 m in the raw sensor frame");
            Assert.Less(deskewMax, 0.05, "per-point poses remove the skew");
        }

        [Test]
        public void EstimatorDriftGrowsAsRandomWalkAndAlignmentIsSeparate() {
            var c = new AutonomyConfig { AlignOffsetX = 0.7 };
            double sum2 = 0; int n = 60;
            for (int k = 0; k < n; k++) {
                var e = new DriftingTruthEstimator(c);
                e.Reset(Pose.FromYaw(new Vec3(), 0), new Vec3(), 0, (ulong)(k + 1));
                for (int i = 1; i <= 2000; i++) e.Update(i * 5_000_000L, Pose.FromYaw(new Vec3(i * 0.05, 0, 0), 0), new Vec3(10, 0, 0));
                var er = e.LocalError; sum2 += er.X * er.X + er.Y * er.Y;
                Assert.AreEqual(0.7, e.Alignment.Offset.X, 1e-9);
            }
            double rms = Math.Sqrt(sum2 / n), expect = c.LocalDriftSigma * Math.Sqrt(2 * 100.0);   // x and y over 100 m
            Assert.AreEqual(expect, rms, 0.5 * expect);
        }

        [Test]
        public void MapMarksHitsCarvesFreeAndLeavesShadowUnknown() {
            var w = Wall(5.0);
            var c = new AutonomyConfig { InvalidP = 0, DispSigmaPx = 0 };
            var cam = new DepthRaycastSensor(c, w, 1);
            var map = new SparseVoxelMap(c);
            var body = Pose.FromYaw(new Vec3(0, 0, 2), 0);
            for (int k = 0; k < 3; k++) { var f = cam.Tick(k * 70_000_000L, body, new Vec3(), new Vec3()); map.InsertDepth(f, DepthRaycastSensor.Compose(body, f.MountInBody), 0); }
            Assert.IsTrue(map.Occupied(new Vec3(4.75, 0, 2)) || map.Occupied(new Vec3(4.65, 0, 2)), "wall surface occupied");
            Assert.IsTrue(map.Known(new Vec3(2.5, 0, 2)) && !map.Occupied(new Vec3(2.5, 0, 2)), "free in front");
            Assert.IsFalse(map.Known(new Vec3(7.0, 0, 2)), "behind the wall unknown");
        }

        [Test]
        public void StopRuleUsesP99AgeAndAMonotoneSimulatedBrakeTable() {
            var bt = StopRule.SimulateBrakeTable(Spec(), 6.0, 1.0);
            for (int i = 1; i < bt.ds.Length; i++) Assert.Greater(bt.ds[i], bt.ds[i - 1]);
            var r = new StopRule(bt.vs, bt.ds, 0.12, 0.5);
            for (int i = 0; i < 200; i++) r.ObserveAge(i == 0 ? 0.5 : 0.08);
            Assert.AreEqual(0.08, r.Tau, 0.01, "one 0.5 s outlier in 200 sits above p99");
            for (int i = 0; i < 10; i++) r.ObserveAge(0.4);
            Assert.Greater(r.Tau, 0.3);
            Assert.Greater(r.Dist(3.0), 3.0 * r.Tau + 0.5);
        }

        [Test]
        public void ClosedLoopMissionReachesGoalThroughTrunksWithoutCollision() {
            var caps = new List<(Vec3, Vec3, double)>();
            var rng = new Rng(5, 1);
            for (int i = 0; i < 14; i++) {
                double x = 4 + 22 * rng.Uniform(), y = -6 + 12 * rng.Uniform();
                if (Math.Abs(y) < 0.9 && i % 2 == 0) y += 1.6;   // leave some gaps
                caps.Add((new Vec3(x, y, 0), new Vec3(x, y, 8), 0.12 + 0.15 * rng.Uniform()));
            }
            var w = ForestManifestWorld.FromCapsules(caps, 0.0, 40);
            var c = new AutonomyConfig { VLimit = 2.0, RunSeed = 3, PriorUse = "none", LidarPointsPerS = 32000 };
            var spec = Spec();
            var bt = StopRule.SimulateBrakeTable(spec, 6.0, 0.5);
            var ms = new MissionSpec { Name = "fixture", Start = new Vec3(0, 0, 2.2), StartYaw = 0 };
            for (int i = 0; i <= 6; i++) ms.Route.Add(new Vec3(i * 5.0, 0, 2.2));
            var rec = new MemoryRecorder();
            var m = new AutonomousMission(ms, c, spec, w, w, w, null, rec, new StopRule(bt.vs, bt.ds, c.TauPrior, c.Margin));
            var r = m.Run();
            Assert.IsFalse(r.Collision, "collision with " + r.CollisionObject);
            Assert.AreEqual("success", r.Outcome, $"outcome {r.Outcome} progress {r.ProgressFrac:0.00}");
            Assert.Greater(r.MinClearanceHardM, 0.0);
            Assert.Less(r.AgeP99S, 0.25);
            Assert.IsTrue(rec.Events.Exists(e => e.kind == "autonomy_result"));
        }

        sealed class FakeLink : IFlightControllerLink { public TrajectorySetpoint Last; public FcStatus Status { get; } = new FcStatus(); public void Submit(TrajectorySetpoint sp) => Last = sp; }

        [Test]
        public void ArbiterBrakesOnStallNoSafeManeuverAndStaleEstimate() {
            var c = new AutonomyConfig(); var link = new FakeLink(); var rec = new MemoryRecorder();
            var a = new SetpointArbiter(c, link, rec);
            var est = new StateEstimate { Pose = Pose.FromYaw(new Vec3(), 0), InputAgeNs = 10_000_000 };
            a.Tick(0, est, 0.005); Assert.AreEqual(0.0, link.Last.Velocity.Norm, 1e-12, "no command before the first plan");
            a.ApplyPlanner(0, new PlanDecision { FeasibleAny = true, VelocityCmd = new Vec3(2, 0, 0), VCmd = 2, Heading = 0 }, 0);
            a.Tick(100_000_000, est, 0.005); Assert.AreEqual(2.0, link.Last.Velocity.X, 1e-12);
            a.Tick(160_000_000, est, 0.005); Assert.AreEqual(0.0, link.Last.Velocity.Norm, 1e-12, "stall watchdog after 150 ms");
            Assert.AreEqual(1, a.WatchdogTrips);
            a.ApplyPlanner(200_000_000, new PlanDecision { FeasibleAny = false, Heading = 1.0 }, 0);
            a.Tick(200_000_000, est, 0.005); Assert.AreEqual(0.0, link.Last.Velocity.Norm, 1e-12); Assert.AreEqual(1, a.NoSafeStops);
            a.ApplyPlanner(250_000_000, new PlanDecision { FeasibleAny = true, VelocityCmd = new Vec3(1, 0, 0), VCmd = 1 }, 0);
            est.InputAgeNs = 200_000_000; a.Tick(250_000_000, est, 0.005);
            Assert.AreEqual(0.0, link.Last.Velocity.Norm, 1e-12, "stale estimate brakes"); Assert.AreEqual(1, a.EstimateStops);
            Assert.IsTrue(rec.Events.Exists(e => e.kind == "autonomy_arbiter"));
        }

        [Test]
        public void PlannerFindsNoSafePrimitiveFacingACloseWallAtSpeed() {
            var w = Wall(1.6, 0.3); var c = new AutonomyConfig { InvalidP = 0, DispSigmaPx = 0, Headings = 16 };
            var map = new SparseVoxelMap(c); var cam = new DepthRaycastSensor(c, w, 1); var lidar = new LidarL2Sensor(c, w, 1, 0);
            var body = Pose.FromYaw(new Vec3(0, 0, 2), 0);
            for (int k = 0; k < 4; k++) { var f = cam.Tick(k * 70_000_000L, body, new Vec3(), new Vec3()); map.InsertDepth(f, DepthRaycastSensor.Compose(body, f.MountInBody), 0); }
            LidarScan s = null; for (long t = 0; s == null; t += c.TickNs) s = lidar.Tick(t, c.TickNs, body);
            map.InsertLidar(s, t => body, 0);
            var win = new PlanningWindow(); win.Build(map, body.P, c.MapWindowXY, c.MapWindowZDown, c.MapWindowZUp);
            var router = new CorridorRouter(c, new List<Vec3> { body.P, new Vec3(20, 0, 2) }, null) { Window = win };
            var est = new StateEstimate { P = body.P, V = new Vec3(6, 0, 0), Pose = body };
            router.Update(0, est, new MapAlignment(), map);
            var bt = StopRule.SimulateBrakeTable(Spec(), 8.0, 1.0);
            var pl = new PrimitivePlanner(c, false, 6.4) { Window = win, Router = router };
            c.BodyRadius = 0.27;
            var d = pl.Plan(0, est, new Vec3(20, 0, 2), 6.0, map, router, new StopRule(bt.vs, bt.ds, 0.12, 0.5), 2.0);
            Assert.IsFalse(d.FeasibleAny, "at 6 m/s 1.3 m from a wall nothing satisfies the stopping rule");
            var still = new StateEstimate { P = body.P, V = new Vec3(), Pose = body };
            var d2 = pl.Plan(0, still, new Vec3(20, 0, 2), 1.0, map, router, new StopRule(bt.vs, bt.ds, 0.12, 0.5), 2.0);
            Assert.IsTrue(d2.FeasibleAny, "hovering, slow sideways/back primitives are feasible");
            Assert.Less(Math.Cos(d2.Heading), 0.9, "it does not choose to fly into the wall");
        }

        [Test]
        public void ConfigCanonicalIsStableAndCoversSpeed() {
            var a = new AutonomyConfig(); var b = new AutonomyConfig();
            Assert.AreEqual(a.Canonical(), b.Canonical());
            b.VLimit = 3.0; Assert.AreNotEqual(a.Canonical(), b.Canonical());
        }
    }
}
