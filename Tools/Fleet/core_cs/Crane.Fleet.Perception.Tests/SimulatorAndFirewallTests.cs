using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Reflection.Emit;
using Crane.Fleet.Perception.Evaluator;
using Crane.Fleet.Perception.Rover;
using Xunit;

namespace Crane.Fleet.Perception.Tests
{
    internal static class Rig
    {
        public static FleetSimulator Make(ulong seed, Action<EvaluatorTruth> place, Action<SimulatorConfig> cfg = null, bool dropoutFree = true)
        {
            var truth = new EvaluatorTruth(); place(truth);
            var c = new SimulatorConfig();
            if (dropoutFree) { c.Noise.BaseDropout = 0; c.Noise.RampMax = 0; }
            cfg?.Invoke(c);
            return new FleetSimulator(TestEnv.Profile(), TestEnv.Scene(), truth, seed, c);
        }
        // Lifter facing cargo_1, scout behind looking at cargo_1's rear face; cargo_2 parked away.
        public static void Standard(EvaluatorTruth t)
        {
            t.SetTrajectory("lifter_1", Trajectories.Static(Pose.FromYaw(1.0, 1.0, 0, 0)));
            t.SetTrajectory("cargo_1", Trajectories.Static(Pose.FromYaw(2.0, 1.0, 0, Math.PI)));
            t.SetTrajectory("scout_1", Trajectories.Static(Pose.FromYaw(0.2, 1.0, 0, 0)));   // sees lifter rear? scout faces +x: lifter's rear face tag
            t.SetTrajectory("cargo_2", Trajectories.Static(Pose.FromYaw(0.8, 5.5, 0, 0)));
        }
    }

    public class SimulatorTests
    {
        [Fact]
        public void OwnCamera_SeesPeerFaceTag_AtCentimetreLevelError()
        {
            var sim = Rig.Make(1, Rig.Standard); var lifter = sim.ProviderFor("lifter_1");
            var obs = lifter.PollTagObservations(2.0).Where(o => o.SourceKind == ObservationSourceKind.OwnCamera && o.FrameChild == "cargo_1/base_link").ToList();
            Assert.True(obs.Count > 50, $"{obs.Count}");
            var errs = obs.Select(o => { var e = ObservationScorer.Error(sim.Truth, o); return Math.Sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]); }).ToList();
            errs.Sort(); Assert.InRange(errs[errs.Count / 2], 0.0003, 0.08);   // base-frame position error includes the 0.21 m tag-to-base lever arm times any frontal-ambiguity rotation error
            Assert.True(errs.Average() < 0.08);
            Assert.All(obs, o => Assert.Equal("lifter_1/base_link", o.FrameParent));
        }

        [Fact]
        public void OwnCamera_RateLimit_And_AcquisitionVersusDeliveryTimes()
        {
            var sim = Rig.Make(2, Rig.Standard); var p = sim.ProviderFor("lifter_1");
            var obs = p.PollTagObservations(3.0).Where(o => o.FrameChild == "cargo_1/base_link" && o.TagId == 10 * 3 + 1 - 1 + 0 || true).Where(o => o.FrameChild == "cargo_1/base_link").ToList();
            var byStamp = obs.Select(o => o.StampAcquired).Distinct().OrderBy(x => x).ToList();
            Assert.InRange(byStamp.Count, 175, 181);     // 60 Hz x 3 s
            for (int i = 1; i < byStamp.Count; i++) Assert.Equal(1.0 / 60, byStamp[i] - byStamp[i - 1], 6);
            Assert.All(obs, o => Assert.True(o.StampDelivered >= o.StampAcquired + 0.02 && o.StampDelivered <= o.StampAcquired + 0.04));
        }

        [Fact]
        public void Poll_ReturnsOnlyDeliveredObservations_AndNeverRepeats()
        {
            var sim = Rig.Make(3, Rig.Standard); var p = sim.ProviderFor("lifter_1");
            var first = p.PollTagObservations(1.0); Assert.All(first, o => Assert.True(o.StampDelivered <= 1.0));
            var again = p.PollTagObservations(1.0); Assert.Empty(again);
            var next = p.PollTagObservations(1.5);
            Assert.All(next, o => Assert.True(o.StampDelivered > 1.0 - 1e-12 && o.StampDelivered <= 1.5));
            Assert.Contains(first.Concat(next).Select(o => o.StampAcquired), t => t > 0.98 && t <= 1.0);   // a frame acquired just before 1.0 is held back by latency, delivered later
        }

        [Fact]
        public void Observation_AgeFieldIsDeliveryMinusAcquisition_AtPollTime()
        {
            var sim = Rig.Make(4, Rig.Standard); var p = sim.ProviderFor("lifter_1");
            var obs = p.PollTagObservations(1.0);
            Assert.All(obs, o => Assert.Equal(1.0 - o.StampAcquired, o.Age, 9));
        }

        [Fact]
        public void Simulation_IsDeterministicUnderSeed_AndIndependentOfPollingCadence()
        {
            var a = Rig.Make(7, Rig.Standard); var b = Rig.Make(7, Rig.Standard); var c = Rig.Make(8, Rig.Standard);
            var oa = a.ProviderFor("lifter_1").PollTagObservations(2.0);
            var pb = b.ProviderFor("lifter_1"); var ob = new List<RelPoseObservation>(); for (double t = 0.1; t < 2.0001; t += 0.1) ob.AddRange(pb.PollTagObservations(t));
            var oc = c.ProviderFor("lifter_1").PollTagObservations(2.0);
            Assert.Equal(oa.Count, ob.Count);
            var ka = oa.OrderBy(o => o.StampAcquired).ThenBy(o => o.TagId).ToList(); var kb = ob.OrderBy(o => o.StampAcquired).ThenBy(o => o.TagId).ToList();
            for (int i = 0; i < ka.Count; i++) { Assert.Equal(ka[i].Pose.P.X, kb[i].Pose.P.X); Assert.Equal(ka[i].StampDelivered, kb[i].StampDelivered); }
            Assert.NotEqual(oa[0].Pose.P.X, oc[0].Pose.P.X);
        }

        [Fact]
        public void PeerObservation_SharedFrameError_EqualsComposition_OfObserverEstimateErrorAndMeasurement()
        {
            // Scout sees cargo_1's rear face at 1 m. Scout's localisation estimate is wrong by a known twist; measurement noise made negligible.
            var sim = Rig.Make(5, t =>
            {
                t.SetTrajectory("scout_1", Trajectories.Static(Pose.FromYaw(0.5, 3.0, 0, 0)));
                t.SetTrajectory("cargo_1", Trajectories.Static(Pose.FromYaw(1.9, 3.0, 0, 0)));
                t.SetTrajectory("lifter_1", Trajectories.Static(Pose.FromYaw(6.5, 5.5, 0, 0)));
                t.SetTrajectory("cargo_2", Trajectories.Static(Pose.FromYaw(8.5, 1.0, 0, 0)));
            }, c => { c.Noise.CornerSigmaPx = 1e-5; c.Noise.ObliquityNoiseGain = 0; c.Noise.ExposureS = 0; });
            var truthScout = sim.Truth.At("scout_1", 0).Pose;
            var xi = new[] { 0.03, -0.02, 0.0, 0.0, 0.0, 0.05 };      // 3 cm, -2 cm, 0.05 rad yaw
            var est = truthScout.Perturbed(xi); var estCov = Matrix.Diag(1e-3, 1e-3, 1e-6, 1e-6, 1e-6, 4e-4);
            sim.ProviderFor("scout_1").ReportSelfEstimate(0.0, est, estCov);
            var peer = sim.ProviderFor("lifter_1").PollTagObservations(1.0).Where(o => o.SourceKind == ObservationSourceKind.PeerObserver && o.FrameChild == "cargo_1/base_link").ToList();
            Assert.NotEmpty(peer);
            foreach (var o in peer)
            {
                Assert.Equal("world", o.FrameParent);
                Pose trueTarget = sim.Truth.At("cargo_1", o.StampAcquired).Pose;
                Pose expected = est.Compose(truthScout.Inverse().Compose(trueTarget));
                Assert.True((o.Pose.P - expected.P).Norm < 1e-3, $"{(o.Pose.P - expected.P).Norm}");
                // independent small-angle check: dp = xi_p + psi * z x (p_target - p_scout)
                Vec3 lever = trueTarget.P - truthScout.P; Vec3 predicted = new Vec3(xi[0] - xi[5] * lever.Y, xi[1] + xi[5] * lever.X, 0);
                var err = ObservationScorer.Error(sim.Truth, o);
                Assert.True(Math.Abs(err[0] - predicted.X) < 3e-3 && Math.Abs(err[1] - predicted.Y) < 3e-3, $"err ({err[0]},{err[1]}) predicted {predicted}");
                Assert.Equal(0.05, err[5], 2);
                // covariance contains at least the observer's own position uncertainty plus the yaw lever term
                Assert.True(o.Covariance[0, 0] >= estCov[0, 0] + lever.Y * lever.Y * estCov[5, 5] - 1e-9);
            }
        }

        [Fact]
        public void PeerObservation_NotProduced_WithoutObserverSelfEstimate()
        {
            var sim = Rig.Make(5, t =>
            {
                t.SetTrajectory("scout_1", Trajectories.Static(Pose.FromYaw(0.5, 3.0, 0, 0))); t.SetTrajectory("cargo_1", Trajectories.Static(Pose.FromYaw(1.9, 3.0, 0, 0)));
                t.SetTrajectory("lifter_1", Trajectories.Static(Pose.FromYaw(6.5, 5.5, 0, 0))); t.SetTrajectory("cargo_2", Trajectories.Static(Pose.FromYaw(8.5, 1.0, 0, 0)));
            });
            Assert.DoesNotContain(sim.ProviderFor("lifter_1").PollTagObservations(1.0), o => o.SourceKind == ObservationSourceKind.PeerObserver);
        }

        [Fact]
        public void PeerObservation_UsesEstimateInForceAtAcquisition_NotALaterOne()
        {
            var sim = Rig.Make(5, t =>
            {
                t.SetTrajectory("scout_1", Trajectories.Static(Pose.FromYaw(0.5, 3.0, 0, 0))); t.SetTrajectory("cargo_1", Trajectories.Static(Pose.FromYaw(1.9, 3.0, 0, 0)));
                t.SetTrajectory("lifter_1", Trajectories.Static(Pose.FromYaw(6.5, 5.5, 0, 0))); t.SetTrajectory("cargo_2", Trajectories.Static(Pose.FromYaw(8.5, 1.0, 0, 0)));
            }, c => { c.Noise.CornerSigmaPx = 1e-5; c.Noise.ObliquityNoiseGain = 0; });
            var s = sim.ProviderFor("scout_1"); var truth = sim.Truth.At("scout_1", 0).Pose;
            s.ReportSelfEstimate(0.0, truth, Matrix.Diag(1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6));
            s.ReportSelfEstimate(0.5, truth.Perturbed(new[] { 0.1, 0.0, 0, 0, 0, 0 }), Matrix.Diag(1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6));
            var obs = sim.ProviderFor("lifter_1").PollTagObservations(1.0).Where(o => o.SourceKind == ObservationSourceKind.PeerObserver && o.FrameChild == "cargo_1/base_link").ToList();
            double early = obs.Where(o => o.StampAcquired < 0.5).Select(o => ObservationScorer.Error(sim.Truth, o)[0]).Average();
            double late = obs.Where(o => o.StampAcquired >= 0.5).Select(o => ObservationScorer.Error(sim.Truth, o)[0]).Average();
            Assert.True(Math.Abs(early) < 5e-3); Assert.InRange(late, 0.09, 0.11);
        }

        [Fact]
        public void PeerObservations_AreNotDeliveredBackToTheObserver_AsPeerObservations()
        {
            var sim = Rig.Make(5, Rig.Standard); var scout = sim.ProviderFor("scout_1");
            sim.ProviderFor("lifter_1").ReportSelfEstimate(0, sim.Truth.At("lifter_1", 0).Pose, Matrix.Diag(1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4));
            Assert.DoesNotContain(scout.PollTagObservations(1.0), o => o.SourceKind == ObservationSourceKind.PeerObserver && o.SourceId.StartsWith("scout_1"));
        }

        private static FleetSimulator OverheadRig(OverheadMode mode, double cornerSigma = 1.0)
        {
            var sim = Rig.Make(11, t =>
            {
                t.SetTrajectory("cargo_1", Trajectories.Static(Pose.FromYaw(3.5, 3.5, 0, 0)));
                t.SetTrajectory("scout_1", Trajectories.Static(Pose.FromYaw(0.5, 0.8, 0, 0))); t.SetTrajectory("lifter_1", Trajectories.Static(Pose.FromYaw(0.5, 6.0, 0, 0)));
                t.SetTrajectory("cargo_2", Trajectories.Static(Pose.FromYaw(9.0, 6.0, 0, 0)));
                t.SetOverheadMode("oh_west", mode); t.SetOverheadMode("oh_east", mode);
            }, c => { c.OverheadResolutionW = 1920; c.OverheadResolutionH = 1080; c.Noise.CornerSigmaPx = cornerSigma; });
            return sim;
        }
        private static List<RelPoseObservation> Overhead(FleetSimulator sim, double T) => sim.ProviderFor("scout_1").PollTagObservations(T).Where(o => o.SourceKind == ObservationSourceKind.Overhead).ToList();

        [Fact]
        public void Overhead_Working_ReportsTopTagsInWorldFrame_FromWestCameraOnly()
        {
            var obs = Overhead(OverheadRig(OverheadMode.Working), 10.0);
            Assert.InRange(obs.Count, 290, 301);      // 30 Hz, 10 s, no dropout (delivery latency 0.15 s trims the tail)
            Assert.All(obs, o => { Assert.Equal("oh_west", o.SourceId); Assert.Equal("world", o.FrameParent); Assert.Equal("cargo_1/base_link", o.FrameChild); });
            Assert.All(obs, o => Assert.True(o.StampDelivered - o.StampAcquired >= 0.14));
        }

        [Fact]
        public void Overhead_Disabled_ProducesNothing()
        {
            Assert.Empty(Overhead(OverheadRig(OverheadMode.Disabled), 10.0));
        }

        [Fact]
        public void Overhead_Occluded_DropsAboutSixtyPercentOfFrames()
        {
            int w = Overhead(OverheadRig(OverheadMode.Working), 20.0).Count, o = Overhead(OverheadRig(OverheadMode.Occluded), 20.0).Count;
            Assert.InRange(o / (double)w, 0.30, 0.50);
        }

        [Fact]
        public void Overhead_SurveyError_IsAConstantBiasPerSeed_ThatAveragingFramesCannotRemove()
        {
            var sim = OverheadRig(OverheadMode.Working, 1e-5); var obs = Overhead(sim, 10.0);
            var g = sim.GeometryOf("cargo_1"); var cam = CameraMount.LookAt(new Vec3(2, 3.5, 2.6), new Vec3(3.5, 3.5, 0)); var est = sim.StationEstimate("oh_west");
            Assert.True((est.P - cam.P).Norm > 1e-4);
            // noise-free prediction of the systematic part: estimated camera pose composed with the TRUE camera-to-tag transform
            var mean = new double[3]; int n = 0;
            foreach (var o in obs)
            {
                var tag = g.Tags.Single(t => t.Id == o.TagId); Pose tagW = sim.Truth.At("cargo_1", o.StampAcquired).Pose.Compose(tag.MountInOwner);
                Pose expected = est.Compose(cam.Inverse().Compose(tagW)).Compose(tag.MountInOwner.Inverse());
                var e = o.Pose.P - expected.P; mean[0] += e.X; mean[1] += e.Y; mean[2] += e.Z; n++;
            }
            Assert.True(Math.Sqrt(mean.Sum(v => v * v)) / n < 0.002, $"{Math.Sqrt(mean.Sum(v => v * v)) / n}");
            // and that prediction differs from truth by the survey bias, which does not average away
            var truthMean = obs.Average(o => ObservationScorer.Error(sim.Truth, o)[0]);
            var predicted = est.Compose(cam.Inverse().Compose(sim.Truth.At("cargo_1", 0).Pose.Compose(g.Tags[2].MountInOwner))).Compose(g.Tags[2].MountInOwner.Inverse()).P.X - sim.Truth.At("cargo_1", 0).Pose.P.X;
            Assert.True(Math.Abs(truthMean - predicted) < 0.015, $"{truthMean} vs {predicted}");
        }

        [Fact]
        public void StaticScenery_Occludes_PalletHidesFaceFromSouth()
        {
            var scene = TestEnv.Scene(); var g = new RoverGeometry(TestEnv.Profile(), "r", 1);
            var cargo = Pose.FromYaw(4.6, 3.3, 0, -Math.PI / 2);        // front face looks south (-y)
            var tag = cargo.Compose(g.Tags[0].MountInOwner);
            var cam = CameraMount.InRover(new Vec3(4.6, 0.7, 0.2), Math.PI / 2);   // south of the pallet looking north
            var K = TestEnv.K(); var vp = new VisibilityParams { MinPixels = 8 };
            Assert.Equal(VisibilityReason.Occluded, TagVisibility.Evaluate(cam, K, tag, 0.05, scene.Boxes, vp).Reason);
            Assert.Equal(VisibilityReason.Visible, TagVisibility.Evaluate(cam, K, tag, 0.05, new List<Obb>(), vp).Reason);
        }

        [Fact]
        public void Beam_HidesGroundPointsFromOverheadCamerasOnlyBeyondDecodeRange()
        {
            // Documented finding: with the cameras at 2.6 m and the beam at z 2.15-2.45 m, the beam hides ground-level points only at long, shallow range (far beyond tag-decode range).
            var scene = TestEnv.Scene(); var beam = scene.Boxes.Single(b => b.Id == "gantry_beam"); int blocked = 0, total = 0; double minBlockedRange = double.PositiveInfinity, maxBlockedFrac = 0;
            foreach (var cam in scene.OverheadCameras)
                for (double x = 0.5; x <= 9.5; x += 0.25) for (double y = 0.5; y <= 6.5; y += 0.25) { total++; if (beam.Blocks(cam.Position, new Vec3(x, y, 0.1), 0.006)) { blocked++; minBlockedRange = Math.Min(minBlockedRange, (new Vec3(x, y, 0.1) - cam.Position).Norm); } }
            maxBlockedFrac = blocked / (double)total;
            Assert.True(total > 1000); Assert.True(maxBlockedFrac < 1.0);
            Assert.True(minBlockedRange > 6.5, $"nearest blocked ground point is {minBlockedRange} m from the camera");
        }

        [Fact]
        public void Encoders_And_Imu_FollowTruthMotion_AtProfileRates()
        {
            var sim = Rig.Make(6, t =>
            {
                t.SetTrajectory("lifter_1", Trajectories.ConstantTwist(Pose.FromYaw(1, 1, 0, 0), 0.3, 0.0, 0.4));
                t.SetTrajectory("scout_1", Trajectories.Static(Pose.FromYaw(5, 5, 0, 0))); t.SetTrajectory("cargo_1", Trajectories.Static(Pose.FromYaw(7, 5, 0, 0))); t.SetTrajectory("cargo_2", Trajectories.Static(Pose.FromYaw(8, 2, 0, 0)));
            });
            var p = sim.ProviderFor("lifter_1"); var enc = p.PollEncoders(5.0); var imu = p.PollImu(5.0);
            Assert.InRange(enc.Count, 500, 502); Assert.InRange(imu.Count, 1000, 1002);
            var e = sim.Profile.Encoder; var last = enc.Last();
            double[] w = last.RatesRadS; MecanumKinematics.BodyTwist(w, e.HalfWheelbaseLx, e.HalfTrackLy, e.WheelRadius, out double vx, out double vy, out double wz);
            Assert.InRange(vx, 0.25, 0.35); Assert.InRange(wz, 0.3, 0.5);
            Assert.InRange(imu.Skip(200).Average(r => r.GyroZ), 0.38, 0.42);
            Assert.True(p.PollEncoders(5.0).Count == 0);
        }

        [Fact]
        public void Scout_Lidar_And_Lifter_Tof_ThroughProvider()
        {
            var sim = Rig.Make(6, t =>
            {
                t.SetTrajectory("scout_1", Trajectories.Static(Pose.FromYaw(1, 5, 0, 0))); t.SetTrajectory("lifter_1", Trajectories.Static(Pose.FromYaw(3, 2, 0, 0)));
                t.SetTrajectory("cargo_1", Trajectories.Static(Pose.FromYaw(3.6, 2, 0, Math.PI))); t.SetTrajectory("cargo_2", Trajectories.Static(Pose.FromYaw(8, 2, 0, 0)));
            });
            Assert.InRange(sim.ProviderFor("scout_1").PollLidar(1.0).Count, 10, 11);
            Assert.Empty(sim.ProviderFor("lifter_1").PollLidar(1.0));
            var tof = sim.ProviderFor("lifter_1").PollTof("tof_front", 1.0); Assert.InRange(tof.Count, 30, 31);
            Assert.All(tof.Where(r => r.Valid), r => Assert.InRange(r.Range, 0.1, 0.25));     // cargo body face at x = 3.6 - 0.21 = 3.39, sensor at 3.21
            Assert.Empty(sim.ProviderFor("scout_1").PollTof("tof_front", 1.0));
        }
    }

    // ------------------------------------------------------------------ truth firewall
    internal static class FirewallChecker
    {
        public const string TruthNamespace = "Crane.Fleet.Perception.Evaluator";
        private static readonly Dictionary<short, OpCode> OpCodes = typeof(OpCodes).GetFields(BindingFlags.Public | BindingFlags.Static).Select(f => (OpCode)f.GetValue(null)).ToDictionary(o => o.Value);
        public static bool IsTruth(Type t)
        {
            while (t.HasElementType) t = t.GetElementType();
            return t.Namespace == TruthNamespace;
        }
        private static bool Mentions(Type t)
        {
            if (t == null) return false;
            if (t.HasElementType) return Mentions(t.GetElementType());
            if (IsTruth(t)) return true;
            if (t.IsGenericType && t.GetGenericArguments().Any(Mentions)) return true;
            return false;
        }
        public static List<string> Violations(IEnumerable<Type> types)
        {
            var v = new List<string>(); const BindingFlags F = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static | BindingFlags.DeclaredOnly;
            foreach (var t in types)
            {
                if (Mentions(t.BaseType)) v.Add($"{t.FullName}: base type");
                foreach (var i in t.GetInterfaces()) if (Mentions(i)) v.Add($"{t.FullName}: interface {i.Name}");
                foreach (var f in t.GetFields(F)) if (Mentions(f.FieldType)) v.Add($"{t.FullName}.{f.Name}: field type");
                foreach (var m in t.GetMethods(F).Cast<MethodBase>().Concat(t.GetConstructors(F)))
                {
                    if (m is MethodInfo mi && Mentions(mi.ReturnType)) v.Add($"{t.FullName}.{m.Name}: return type");
                    foreach (var p in m.GetParameters()) if (Mentions(p.ParameterType)) v.Add($"{t.FullName}.{m.Name}: parameter {p.Name}");
                    foreach (var rt in IlTypes(m, t)) if (IsTruth(rt)) v.Add($"{t.FullName}.{m.Name}: IL references {rt.FullName}");
                }
            }
            return v;
        }

        private static IEnumerable<Type> IlTypes(MethodBase m, Type owner)
        {
            var body = m.GetMethodBody(); if (body == null) yield break;
            foreach (var lv in body.LocalVariables) yield return lv.LocalType;
            byte[] il = body.GetILAsByteArray(); var mod = m.Module; int i = 0;
            Type[] ta = owner.IsGenericType ? owner.GetGenericArguments() : null; Type[] ma = m.IsGenericMethod ? m.GetGenericArguments() : null;
            while (i < il.Length)
            {
                OpCode op; if (il[i] == 0xFE) { op = OpCodes[(short)(0xFE00 | il[i + 1])]; i += 2; } else { op = OpCodes[il[i]]; i++; }
                int size;
                switch (op.OperandType)
                {
                    case OperandType.InlineNone: size = 0; break;
                    case OperandType.ShortInlineBrTarget: case OperandType.ShortInlineI: case OperandType.ShortInlineVar: size = 1; break;
                    case OperandType.InlineVar: size = 2; break;
                    case OperandType.InlineI8: case OperandType.InlineR: size = 8; break;
                    case OperandType.InlineSwitch: size = 4 + 4 * BitConverter.ToInt32(il, i); break;
                    default: size = 4; break;
                }
                if (op.OperandType == OperandType.InlineField || op.OperandType == OperandType.InlineMethod || op.OperandType == OperandType.InlineTok || op.OperandType == OperandType.InlineType)
                {
                    int token = BitConverter.ToInt32(il, i);
                    MemberInfo mem = null;
                    try { mem = mod.ResolveMember(token, ta, ma); } catch (Exception) { }
                    if (mem is Type tt) yield return tt;
                    else if (mem is FieldInfo fi) { yield return fi.DeclaringType; yield return fi.FieldType; }
                    else if (mem is MethodBase mb)
                    {
                        yield return mb.DeclaringType;
                        if (mb is MethodInfo r) yield return r.ReturnType;
                        foreach (var p in mb.GetParameters()) yield return p.ParameterType;
                    }
                }
                i += size;
            }
        }
    }

    public class TruthFirewallTests
    {
        private static IEnumerable<Type> AllTypes() => typeof(FleetProfile).Assembly.GetTypes();
        private static IEnumerable<Type> RoverSide() => AllTypes().Where(t => !FirewallChecker.IsTruth(t));

        [Fact]
        public void NoRoverSideType_ReferencesATruthType_BySignatureOrIl()
        {
            var v = FirewallChecker.Violations(RoverSide());
            Assert.True(v.Count == 0, string.Join("\n", v));
        }

        [Fact]
        public void TruthTypesExist_AndTheRoverNamespaceIsNonEmpty()
        {
            Assert.Contains(AllTypes(), t => t == typeof(EvaluatorTruth));
            Assert.True(AllTypes().Count(t => t.Namespace == "Crane.Fleet.Perception.Rover") >= 3);
            Assert.True(AllTypes().Count(t => FirewallChecker.IsTruth(t)) >= 4);
        }

        [Fact]
        public void Checker_FlagsDeliberateLeaks_SoThePassIsMeaningful()
        {
            var sig = FirewallChecker.Violations(new[] { typeof(LeakBySignature) });
            var il = FirewallChecker.Violations(new[] { typeof(LeakByIlOnly) });
            var field = FirewallChecker.Violations(new[] { typeof(LeakByField) });
            Assert.NotEmpty(sig); Assert.NotEmpty(il); Assert.NotEmpty(field);
        }

        [Fact]
        public void ObservationProviderApi_ReturnsOnlyObservationTypes()
        {
            var allowed = new HashSet<Type> { typeof(string), typeof(double), typeof(void), typeof(SensorSuite), typeof(Pose), typeof(Matrix),
                typeof(RelPoseObservation), typeof(EncoderReading), typeof(ImuReading), typeof(RangeReading), typeof(LidarScan) };
            foreach (var m in typeof(IObservationProvider).GetMethods().Concat(typeof(IObservationProvider).GetProperties().Select(p => p.GetMethod)))
            {
                foreach (var t in new[] { m.ReturnType }.Concat(m.GetParameters().Select(p => p.ParameterType)))
                {
                    var core = t.IsGenericType ? t.GetGenericArguments()[0] : t;
                    Assert.True(allowed.Contains(core), $"{m.Name}: unexpected type {t}");
                }
            }
        }

        [Fact]
        public void ReadingTypes_ExposeNoTruthNamedMembers()
        {
            foreach (var t in new[] { typeof(EncoderReading), typeof(ImuReading), typeof(RangeReading), typeof(LidarScan), typeof(RelPoseObservation), typeof(SensorSuite) })
                foreach (var m in t.GetMembers(BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static))
                    Assert.DoesNotMatch("(?i)true|truth|ground", m.Name);
        }

        [Fact]
        public void RoverProvider_HoldsOnlyTheBackendInterface()
        {
            var fields = typeof(RoverObservationProvider).GetFields(BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public);
            Assert.All(fields, f => Assert.True(f.FieldType == typeof(IObservationBackend) || f.FieldType == typeof(string), f.Name));
        }

        [Fact]
        public void RoverProviderHandle_CannotBeUsedToReachTruth_ViaReflectionOnPublicSurface()
        {
            var sim = Rig.Make(1, Rig.Standard); var p = sim.ProviderFor("lifter_1");
            var all = p.GetType().GetMembers(BindingFlags.Public | BindingFlags.Instance);
            Assert.DoesNotContain(all, m => m is PropertyInfo pi && FirewallChecker.IsTruth(pi.PropertyType));
            Assert.DoesNotContain(all, m => m is MethodInfo mi && (FirewallChecker.IsTruth(mi.ReturnType) || mi.GetParameters().Any(x => FirewallChecker.IsTruth(x.ParameterType))));
        }
    }

    internal sealed class LeakBySignature { public double Peek(EvaluatorTruth t) => 0; }
    internal sealed class LeakByField { private EvaluatorTruth _t; public int N => _t == null ? 0 : 1; }
    internal sealed class LeakByIlOnly { public double Peek() { var t = new EvaluatorTruth(); return t.RoverIds.Count; } }
}
