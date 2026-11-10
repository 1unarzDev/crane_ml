using System;
using System.Collections.Generic;
using System.Linq;
using Crane.Fleet.Autonomy.Tests.Harness;
using Crane.Fleet.Docking;
using Crane.Fleet.Perception;
using Xunit;
using Xunit.Abstractions;

namespace Crane.Fleet.Autonomy.Tests
{
    public class AllocationTests
    {
        private static (WheelAllocator, AssemblyCapability, AssemblyModel) Make(AssemblyModel m)
        {
            var cap = Configs.Cap(m); var al = new WheelAllocator(); al.Bind(m, cap); return (al, cap, m);
        }

        [Fact]
        public void Healthy_EqualsInverseKinematicsWithLateralEfficiency()
        {
            var (al, cap, m) = Make(Configs.Single());
            var w = new double[4]; var xi = new Twist2(0.1, 0.05, 0.2);
            al.Allocate(xi, w);
            var s = m.Modules[0].Spec;
            for (int j = 0; j < 4; j++)
            {
                s.WheelRow(j, out double ax, out double ay, out double aw);
                Assert.Equal(ax * xi.Vx + ay / s.LateralEfficiency * xi.Vy + aw * xi.Wz, w[j], 9);
            }
        }

        [Fact]
        public void Saturation_ScalesUniformly_PreservesDirection()
        {
            var (al, cap, _) = Make(Configs.Line());
            var w = new double[8]; var xi = new Twist2(2.0, 1.0, 3.0);
            var real = al.Allocate(xi, w);
            Assert.True(al.LastScale < 1);
            for (int i = 0; i < 8; i++) Assert.True(Math.Abs(w[i]) <= cap.WheelLimit(i) + 1e-9);
            Assert.Equal(xi.Vy / xi.Vx, real.Vy / real.Vx, 9); Assert.Equal(xi.Wz / xi.Vx, real.Wz / real.Vx, 9);
        }

        [Fact]
        public void DeadFreeWheel_CommandedZero_OthersStillRealiseTheTwist()
        {
            var m = Configs.Single(); m.Modules[0].Health[1] = CapHealth.DeadFree;
            var (al, cap, _) = Make(m);
            var w = new double[4]; var xi = new Twist2(0.15, -0.1, 0.2);
            al.Allocate(xi, w);
            Assert.Equal(0, w[1]);
            // least-squares twist of the three live wheels (with lateral efficiency) reproduces xi
            var rows = KinematicRows.Build(m, true);
            var AtA = new double[9]; var Atb = new double[3];
            foreach (int i in new[] { 0, 2, 3 }) for (int r = 0; r < 3; r++) { Atb[r] += rows[3 * i + r] * w[i]; for (int c = 0; c < 3; c++) AtA[3 * r + c] += rows[3 * i + r] * rows[3 * i + c]; }
            var inv = new double[9]; Lin.Invert(AtA, inv, 3, new double[18]);
            double vx = inv[0] * Atb[0] + inv[1] * Atb[1] + inv[2] * Atb[2], vy = inv[3] * Atb[0] + inv[4] * Atb[1] + inv[5] * Atb[2];
            Assert.Equal(xi.Vx, vx, 9); Assert.Equal(xi.Vy, vy, 9);
        }

        [Fact]
        public void DeadLockedWheel_TwistProjected_LockedWheelStill()
        {
            var m = Configs.Line(); m.Modules[1].Health[0] = CapHealth.DeadLocked;
            var (al, cap, _) = Make(m);
            var w = new double[8]; var real = al.Allocate(new Twist2(0.2, 0, 0), w);
            Assert.Equal(0, w[4]);
            Assert.True(Math.Abs(cap.WheelRate(4, real)) < 1e-9);   // kinematic constraint of the locked wheel holds
            Assert.True(real.Vx > 0);                                  // some forward progress remains (along the subspace)
        }

        [Fact]
        public void WeakWheel_LimitRespected()
        {
            var m = Configs.Single(); m.Modules[0].Health[0] = CapHealth.Weak(0.25);
            var (al, cap, _) = Make(m);
            var w = new double[4]; al.Allocate(new Twist2(0.5, 0, 0), w);
            Assert.True(Math.Abs(w[0]) <= 0.25 * m.Modules[0].Spec.MaxWheelSpeed + 1e-9);
        }
    }

    public class TrackerAndHealthTests
    {
        [Fact]
        public void Tracker_FollowsPlannedPath_OnIdealKinematics_AndArrives()
        {
            var a = new AutonomyAssumptions(); var cap = Configs.Cap(Configs.Single());
            var path = new LatticePlanner(Sim.Warehouse(), a).Plan(new PlanRequest { Start = new Pose2(1.5, 1.5, 0), Goal = new Pose2(3.5, 2.5, 0.5), Capability = cap });
            var tr = new TrajectoryTracker(a); tr.SetPath(path, cap);
            var pose = path.Points[0].Pose; double maxDev = 0;
            for (int k = 0; k < 3000 && tr.Status == TrackStatus.Tracking; k++)
            {
                var cmd = tr.Step(pose, 0.02);
                Assert.True(cap.Feasible(cmd, 1e-6));
                pose = Se2.Advance(pose, cmd, 0.02);
                maxDev = Math.Max(maxDev, tr.CrossTrackError);
            }
            Assert.Equal(TrackStatus.Arrived, tr.Status);
            Assert.True(Se2.Distance(pose, new Pose2(3.5, 2.5, 0.5)) < 0.035);
            Assert.True(maxDev < 0.08, "max deviation " + maxDev);
        }

        [Fact]
        public void Tracker_FlagsDeviation()
        {
            var a = new AutonomyAssumptions(); var cap = Configs.Cap(Configs.Single());
            var path = new LatticePlanner(Sim.Warehouse(), a).Plan(new PlanRequest { Start = new Pose2(1.5, 1.5, 0), Goal = new Pose2(3.5, 1.5, 0), Capability = cap });
            var tr = new TrajectoryTracker(a); tr.SetPath(path, cap);
            tr.Step(new Pose2(1.5, 2.0, 0), 0.02);
            Assert.Equal(TrackStatus.Deviated, tr.Status);
        }

        [Fact]
        public void HealthMonitor_IdentifiesLockedAndWeak_NotHealthy()
        {
            var a = new AutonomyAssumptions(); var mon = new WheelHealthMonitor(a);
            double[] cmd = { 6, 6, 6, 6 }, meas = { 0.0, 6, 3.0, 5.9 }, kin = { 5, 5, 5, 5 };
            bool changed = false;
            for (int k = 0; k < 30; k++) changed |= mon.Update(cmd, meas, kin, 0.02);
            Assert.True(changed);
            Assert.Equal(CapHealthMode.DeadLocked, mon.Identified[0].Mode);
            Assert.Equal(CapHealthMode.Ok, mon.Identified[1].Mode);
            Assert.Equal(CapHealthMode.Weak, mon.Identified[2].Mode);
            Assert.InRange(mon.Identified[2].Gain, 0.45, 0.55);
            Assert.Equal(CapHealthMode.Ok, mon.Identified[3].Mode);
        }

        [Fact]
        public void HealthMonitor_IgnoresShortTransients()
        {
            var a = new AutonomyAssumptions(); var mon = new WheelHealthMonitor(a);
            double[] cmd = { 6, 6, 6, 6 }, lag = { 1, 1, 1, 1 }, ok = { 6, 6, 6, 6 };
            for (int k = 0; k < 5; k++) mon.Update(cmd, lag, ok, 0.02);    // 0.1 s motor lag
            for (int k = 0; k < 50; k++) mon.Update(cmd, ok, ok, 0.02);
            Assert.All(mon.Identified, h => Assert.Equal(CapHealthMode.Ok, h.Mode));
        }
    }

    public class EstimationTests
    {
        private readonly ITestOutputHelper _o;
        public EstimationTests(ITestOutputHelper o) { _o = o; }

        [Fact]
        public void Se2_ExpLogRoundTrip_AndJacobiansMatchFiniteDifferences()
        {
            var t = new Twist2(0.3, -0.2, 0.7);
            var d = Se2.Exp(t, 1.3); var back = Se2.Log(d);
            Assert.Equal(t.Vx * 1.3, back.Vx, 9); Assert.Equal(t.Vy * 1.3, back.Vy, 9); Assert.Equal(t.Wz * 1.3, back.Wz, 9);
            var a = new Pose2(1, 2, 0.4); var b = new Pose2(0.3, -0.2, 1.1);
            var j = new double[9]; Se2.JacComposeLeft(a, b, j);
            double h = 1e-6;
            for (int k = 0; k < 3; k++)
            {
                var ap = new Pose2(a.X + (k == 0 ? h : 0), a.Y + (k == 1 ? h : 0), a.Yaw + (k == 2 ? h : 0));
                var c0 = Se2.Compose(a, b); var c1 = Se2.Compose(ap, b);
                Assert.Equal(j[0 * 3 + k], (c1.X - c0.X) / h, 5); Assert.Equal(j[1 * 3 + k], (c1.Y - c0.Y) / h, 5); Assert.Equal(j[2 * 3 + k], Se2.Wrap(c1.Yaw - c0.Yaw) / h, 5);
            }
        }

        private static BodyEkf NewEkf(out AssemblyModel m, double ps = 0.01)
        {
            m = Configs.Single(); var a = new AutonomyAssumptions();
            return new BodyEkf(m, Configs.Cap(m), a, Pose2.Identity, ps, ps, 0);
        }

        [Fact]
        public void DelayedWorldPose_ReappliedEqualsInOrderProcessing()
        {
            BodyEkf inOrder = NewEkf(out var m), late = NewEkf(out _);
            var s = m.Modules[0].Spec;
            var r3 = new double[] { 1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4 };
            void Feed(BodyEkf e, int k)
            {
                double t = k * 0.01;
                for (int j = 0; j < 4; j++) { s.WheelRow(j, out double ax, out double ay, out double aw); e.WheelRate(0, j, t, ax * 0.2 + aw * 0.1, 0.05); }
                e.Gyro(0, t, 0.1, 0.003);
            }
            for (int k = 1; k <= 100; k++)
            {
                Feed(inOrder, k); Feed(late, k);
                if (k == 50) inOrder.WorldPose(0, 0.5, new Pose2(0.12, 0.01, 0.06), r3);
            }
            late.WorldPose(0, 0.5, new Pose2(0.12, 0.01, 0.06), r3);   // arrives 0.5 s late
            inOrder.PredictTo(1.0); late.PredictTo(1.0);
            Assert.Equal(1, late.DelayedReapplied);
            Assert.Equal(inOrder.Pose.X, late.Pose.X, 9); Assert.Equal(inOrder.Pose.Y, late.Pose.Y, 9); Assert.Equal(inOrder.Pose.Yaw, late.Pose.Yaw, 9);
            Assert.Equal(inOrder.Cov(0, 0), late.Cov(0, 0), 12);
        }

        [Fact]
        public void WorldPoseOutlier_IsGated()
        {
            var e = NewEkf(out _);
            e.WorldPose(0, 0, new Pose2(1.0, 0, 0), new double[] { 1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4 });
            Assert.Equal(1, e.RejectedPoseUpdates);
            Assert.Equal(0, e.Pose.X, 9);
        }

        [Fact]
        public void ZeroVelocityAndRestGyro_EstimateBias()
        {
            var e = NewEkf(out _);
            var rng = new DeterministicRng(5);
            for (int k = 1; k <= 300; k++)
            {
                double t = k * 0.005;
                e.GyroAtRest(0, t, 0.003 + 0.004 * rng.NextGaussian(), 0.0045);
                if (k % 8 == 0) e.ZeroVelocity(t, 0.002, 0.0005);
            }
            Assert.InRange(e.GyroBias(0), 0.0026, 0.0034);
        }

        /// <summary>NEES of the pose estimate on synthetic runs in the kinematic world (slip, quantised encoders, gyro bias): mean of
        /// the 3-dof NEES over seeds must be consistent (chi2 mean 3) or conservative, never overconfident.</summary>
        [Fact]
        public void BodyEkf_PoseNees_IsConsistentOrConservative()
        {
            var nees = new List<double>();
            for (ulong seed = 1; seed <= 24; seed++)
            {
                var s = Missions.SingleNav(seed);
                s.MaxTime = 20;
                var r = Sim.Run(s);
                var ag = r.Autonomy.AgentOf("cargo_1"); var tr = r.World.R("cargo_1").Pose;
                var c = new double[9]; ag.Ekf.PoseCov(c);
                var e = Se2.Between(new Pose2(0, 0, 0), new Pose2(ag.Pose.X - tr.X, ag.Pose.Y - tr.Y, Se2.Wrap(ag.Pose.Yaw - tr.Yaw)));
                nees.Add(Lin.Mahalanobis3(c, e.X, e.Y, e.Yaw));
            }
            double mean = nees.Average();
            _o.WriteLine($"pose NEES mean {mean:0.00} (3 dof), max {nees.Max():0.00}");
            Assert.InRange(mean, 0.2, 4.5);
            Assert.True(nees.Count(x => x > 16.3) <= 1);
        }

        [Fact]
        public void Relative_RoundTripAndMotionPrediction()
        {
            var a = new AutonomyAssumptions(); var s = Configs.S("a");
            var rel = new RelativePoseEstimator(s.Face("front").InBody, s.Face("rear").InBody, a);
            var tRel = new Pose2(0.92, 0.02, 0.05);
            var d = rel.FromBaseRelative(tRel); var back = rel.ToBaseRelative(d);
            Assert.Equal(tRel.X, back.X, 9); Assert.Equal(tRel.Y, back.Y, 9); Assert.Equal(tRel.Yaw, back.Yaw, 9);
            Assert.InRange(d.X, 0.49, 0.51);   // face gap ~0.92 - 0.21 - 0.21
            rel.InitializeFromBaseRelative(tRel, new double[] { 1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4 }, 0);
            rel.PredictDockerMotion(new Pose2(0.1, 0, 0), 0.5);
            Assert.Equal(d.X - 0.1, rel.Gap, 6);
            Assert.True(rel.SigmaGap > 0.01);
        }

        [Fact]
        public void Relative_DelayedMeasurementTransportedThroughBufferedMotion()
        {
            var a = new AutonomyAssumptions(); var s = Configs.S("a");
            var ra = new RelativePoseEstimator(s.Face("front").InBody, s.Face("rear").InBody, a);
            var rb = new RelativePoseEstimator(s.Face("front").InBody, s.Face("rear").InBody, a);
            var prior = new double[] { 4e-4, 0, 0, 0, 4e-4, 0, 0, 0, 4e-3 }; var tRel = new Pose2(0.92, 0.0, 0.0);
            ra.InitializeFromBaseRelative(tRel, prior, 0); rb.InitializeFromBaseRelative(tRel, prior, 0);
            var meas = new Pose2(0.93, 0.01, 0.02); var r = new double[] { 1e-5, 0, 0, 0, 1e-6, 0, 0, 0, 1e-4 };
            ra.UpdateBaseRelative(meas, r, 0.0);               // on time
            for (int k = 1; k <= 10; k++) { ra.PredictDockerMotion(new Pose2(0.01, 0, 0), k * 0.1); rb.PredictDockerMotion(new Pose2(0.01, 0, 0), k * 0.1); }
            rb.UpdateBaseRelative(meas, (double[])r.Clone(), 0.0);   // same measurement delivered 1 s late
            Assert.Equal(ra.Gap, rb.Gap, 3); Assert.Equal(ra.Lateral, rb.Lateral, 3);
        }

        [Fact]
        public void Relative_OwnCameraNees_FromPerceptionAtHalfMetre()
        {
            // synthetic own-camera observations of the target's rear face tag through the Perception detection chain
            var prof = Env.Profile; var a = new AutonomyAssumptions(); var s = Configs.S("a");
            var geom = new RoverGeometry(prof, "t", 1);
            var tag = geom.Tags.First(x => x.Name == "face_rear");
            var cam = CameraMount.InRover(new Vec3(0.20, 0, 0.08), 0);
            var K = prof.Camera.Intrinsics; var vis = new VisibilityParams(); var noise = new TagNoiseParams();
            var nees = new List<double>();
            for (int k = 0; k < 400; k++)
            {
                var rng = Rng.ForStream(7, "nees/" + k);
                var target = new Pose2(0.92 + 0.02 * rng.Gaussian(), 0.03 * rng.Gaussian(), 0.08 * rng.Gaussian());
                var det = TagObserver.Detect(cam, BodyMotion.Still, K, Se2.ToPose(target).Compose(tag.MountInOwner), BodyMotion.Still, tag, new List<Obb>(), vis, noise, 0.03, 0, rng);
                if (!det.Detected || det.Pnp.Ambiguous) continue;
                Pose baseTag = cam.Compose(det.CamTag); var cov = PoseUncertainty.RotateInto(cam.R, det.CovarianceCam);
                Pose mInv = tag.MountInOwner.Inverse();
                var obs = Se2.FromPose(baseTag.Compose(mInv)); var c6 = PoseUncertainty.RigidShift(baseTag, mInv, cov);
                var c3 = new double[9]; FleetAutonomy.Marginal(c6, c3);
                var rel = new RelativePoseEstimator(s.Face("front").InBody, s.Face("rear").InBody, a);
                rel.UpdateBaseRelative(obs, c3, 0);
                var truthD = rel.FromBaseRelative(target);
                var P = new double[9]; rel.Covariance(P);
                nees.Add(Lin.Mahalanobis3(P, rel.Gap - truthD.X, rel.Lateral - truthD.Y, Se2.Wrap(rel.Yaw - truthD.Yaw)));
            }
            double m = nees.Average(), med = nees.OrderBy(x => x).ElementAt(nees.Count / 2);
            _o.WriteLine($"docking-frame NEES over {nees.Count} unflagged frames: mean {m:0.00}, median {med:0.00} (3 dof: mean 3, median 2.37)");
            Assert.True(nees.Count > 100);
            Assert.InRange(med, 1.2, 4.0);
        }

        [Fact]
        public void Relative_CaptureTest_TiedToCovariance()
        {
            var a = new AutonomyAssumptions(); var s = Configs.S("a");
            var rel = new RelativePoseEstimator(s.Face("front").InBody, s.Face("rear").InBody, a);
            rel.InitializeFromBaseRelative(rel.ToBaseRelative(new Pose2(0.1, 0.002, 0.01)), new double[] { 1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-5 }, 0);
            Assert.True(rel.PredictsCapture(2.5, 0.018, 0.087));
            rel.InitializeFromBaseRelative(rel.ToBaseRelative(new Pose2(0.1, 0.002, 0.01)), new double[] { 1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-3 }, 0);
            Assert.False(rel.PredictsCapture(2.5, 0.018, 0.087));
        }
    }
}
