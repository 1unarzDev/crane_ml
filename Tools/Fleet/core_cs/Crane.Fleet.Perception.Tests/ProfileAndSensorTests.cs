using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Xunit;

namespace Crane.Fleet.Perception.Tests
{
    public class ProfileTests
    {
        [Fact]
        public void LoadsRealProfile_AndKeepsProvenanceOnEveryLeaf()
        {
            var p = TestEnv.Profile();
            Assert.Equal("mecanum_fleet_provisional_v0", p.ProfileId);
            Assert.True(p.Leaves.Count > 80);
            Assert.All(p.Leaves.Values, l => Assert.Contains(l.Provenance, p.ProvenanceLevels));
            Assert.Equal("provisional", p.Prov("sensors.mono_camera_ov9281.hfov"));
        }

        [Fact]
        public void TypedSpecs_CarryProfileValuesAndLabels()
        {
            var p = TestEnv.Profile();
            Assert.Equal(1280, p.Camera.Width); Assert.Equal(800, p.Camera.Height); Assert.Equal(1.40, p.Camera.Hfov); Assert.Equal(60, p.Camera.RateHz); Assert.Equal(0.03, p.Camera.Latency);
            Assert.Equal("global", p.Camera.Shutter); Assert.Equal("mono8", p.Camera.Encoding);
            Assert.Equal("provisional", p.Camera.Provenance["hfov"]);
            Assert.Equal(0.05, p.FaceTagSize); Assert.Equal(0.12, p.TopTagSize); Assert.Equal(1440, p.Encoder.CountsPerRev);
            Assert.Equal(0.15, p.Overhead.Latency); Assert.True(p.Overhead.Optional); Assert.Equal(720, p.Lidar.Points);
            Assert.Equal(new[] { "front", "rear" }, p.ActiveFaces);
        }

        [Fact]
        public void MeasuredLabel_IsPreservedWhenProfileIsEdited()
        {
            string json = File.ReadAllText(TestEnv.ProfilePath).Replace("\"hfov\": {\"value\": 1.40, \"provenance\": \"provisional\"}, \"rate_hz\": {\"value\": 60", "\"hfov\": {\"value\": 1.31, \"provenance\": \"measured\"}, \"rate_hz\": {\"value\": 60");
            var p = FleetProfile.Parse(json);
            Assert.Equal(1.31, p.Camera.Hfov); Assert.Equal("measured", p.Camera.Provenance["hfov"]); Assert.Equal("measured", p.Prov("sensors.mono_camera_ov9281.hfov"));
        }

        [Fact]
        public void Rejects_MissingProvenance()
        {
            string json = File.ReadAllText(TestEnv.ProfilePath).Replace("{\"value\": 60, \"provenance\": \"provisional\"}", "{\"value\": 60}");
            var e = Assert.Throws<ProfileException>(() => FleetProfile.Parse(json));
            Assert.Contains(e.Errors, m => m.Contains("missing provenance"));
        }

        [Fact]
        public void Rejects_UnknownProvenanceLabel()
        {
            string json = File.ReadAllText(TestEnv.ProfilePath).Replace("\"lidar_2d\": {\"max_range\": {\"value\": 12.0, \"provenance\": \"provisional\"}", "\"lidar_2d\": {\"max_range\": {\"value\": 12.0, \"provenance\": \"guess\"}");
            var e = Assert.Throws<ProfileException>(() => FleetProfile.Parse(json));
            Assert.Contains(e.Errors, m => m.Contains("guess"));
        }

        [Fact]
        public void Rejects_NonPhysicalFov()
        {
            string json = File.ReadAllText(TestEnv.ProfilePath).Replace("\"hfov\": {\"value\": 1.40, \"provenance\": \"provisional\"}, \"rate_hz\": {\"value\": 60", "\"hfov\": {\"value\": 3.5, \"provenance\": \"provisional\"}, \"rate_hz\": {\"value\": 60");
            Assert.Throws<ProfileException>(() => FleetProfile.Parse(json));
        }

        [Fact]
        public void Rejects_UnknownSensorTokenAndUnknownRole()
        {
            string raw = File.ReadAllText(TestEnv.ProfilePath);
            Assert.Throws<ProfileException>(() => FleetProfile.Parse(raw.Replace("\"range_tof(front)\"", "\"sonar(front)\"")));
            Assert.Throws<ProfileException>(() => FleetProfile.Parse(raw.Replace("{\"id\": \"cargo_2\", \"role\": \"cargo\"}", "{\"id\": \"cargo_2\", \"role\": \"tug\"}")));
        }

        [Fact]
        public void Rejects_DuplicateRoverIds_AndBadJson()
        {
            string raw = File.ReadAllText(TestEnv.ProfilePath);
            Assert.Throws<ProfileException>(() => FleetProfile.Parse(raw.Replace("\"id\": \"cargo_2\"", "\"id\": \"cargo_1\"")));
            Assert.Throws<ProfileException>(() => FleetProfile.Parse("{ not json"));
            Assert.Throws<ProfileException>(() => FleetProfile.Parse("[]"));
        }

        [Fact]
        public void Scenario_LoadsBoxesIncludingElevatedBeamAndOverheadCameras()
        {
            var s = TestEnv.Scene();
            Assert.Equal("fleet/warehouse_a/v0", s.ScenarioId);
            var beam = s.Boxes.Single(b => b.Id == "gantry_beam");
            Assert.Equal(2.15, beam.ZMin, 9); Assert.Equal(2.45, beam.ZMax, 9);
            Assert.Equal(2, s.OverheadCameras.Count); Assert.Equal(new[] { "working", "occluded", "disabled" }, s.OverheadModes);
            Assert.Contains(s.Boxes, b => b.Id == "divider_lintel");
            Assert.Equal(2, s.Boxes.Count(b => b.Id == "divider_s" || b.Id == "divider_n"));
        }

        [Fact]
        public void Scenario_DoorwayGapIsSizedFromWidth()
        {
            var s = TestEnv.Scene(); var south = s.Boxes.Single(b => b.Id == "divider_s"); var north = s.Boxes.Single(b => b.Id == "divider_n");
            double gap = (north.Centre.Y - north.Half.Y) - (south.Centre.Y + south.Half.Y);
            Assert.Equal(0.80, gap, 9);
        }
    }

    public class SensorTests
    {
        [Fact]
        public void SensorSuites_FollowRoles()
        {
            var p = TestEnv.Profile();
            var scout = SensorSuite.ForRover(p, "scout_1"); var lifter = SensorSuite.ForRover(p, "lifter_1"); var cargo = SensorSuite.ForRover(p, "cargo_2");
            Assert.NotNull(scout.Lidar); Assert.Equal(3, scout.Cameras.Count); Assert.Empty(scout.Tofs);
            Assert.Contains(scout.Cameras, c => c.Name == "stereo_left"); Assert.Contains(scout.Cameras, c => c.Facing == "rear");
            foreach (var r in new[] { lifter, cargo }) { Assert.Null(r.Lidar); Assert.Single(r.Cameras); Assert.Single(r.Tofs); Assert.True(r.HasEncoders && r.HasImu); Assert.Equal("front", r.Cameras[0].Facing); }
            Assert.Throws<KeyNotFoundException>(() => SensorSuite.ForRover(p, "ghost"));
        }

        [Fact]
        public void StereoPair_IsSeparatedByTheProfileBaseline()
        {
            var s = SensorSuite.ForRover(TestEnv.Profile(), "scout_1");
            var l = s.Cameras.Single(c => c.Name == "stereo_left").OpticalInRover.P; var r = s.Cameras.Single(c => c.Name == "stereo_right").OpticalInRover.P;
            Assert.Equal(0.075, (l - r).Norm, 12);
        }

        [Fact]
        public void Camera_ParametersComeFromProfile()
        {
            var c = SensorSuite.ForRover(TestEnv.Profile(), "lifter_1").Cameras[0];
            Assert.Equal(60, c.RateHz); Assert.Equal(0.03, c.LatencyS); Assert.Equal(1.40, c.K.Hfov, 12);
        }

        [Fact]
        public void MecanumKinematics_RoundTrips()
        {
            var w = MecanumKinematics.WheelSpeeds(0.3, -0.1, 0.7, 0.14, 0.175, 0.0485);
            MecanumKinematics.BodyTwist(w, 0.14, 0.175, 0.0485, out double vx, out double vy, out double wz);
            Assert.Equal(0.3, vx, 12); Assert.Equal(-0.1, vy, 12); Assert.Equal(0.7, wz, 12);
        }

        [Fact]
        public void Encoder_CountsAreQuantisedIntegers_WithinOneCountOfTruthWithoutSlip()
        {
            var e = new EncoderModel(1440, 0.0, 1, "x"); double angle = 0;
            for (int i = 1; i <= 100; i++)
            {
                angle += 0.0137; var r = e.Sample(i * 0.01, new[] { angle, angle, -angle, 0.0 });
                Assert.InRange(r.Counts[0] - angle / e.RadPerCount, -1.0, 0.0);
                Assert.InRange(r.Counts[2] - (-angle) / e.RadPerCount, -1.0, 0.0);
            }
            Assert.Equal(2 * Math.PI / 1440, e.RadPerCount, 12);
        }

        [Fact]
        public void Encoder_SlipNoise_ScalesWithDistanceAndFraction()
        {
            double Rms(double frac)
            {
                double s2 = 0; int n = 200;
                for (int k = 0; k < n; k++)
                {
                    var e = new EncoderModel(1440, frac, (ulong)k, "x"); double a = 0; EncoderReading r = null;
                    for (int i = 1; i <= 100; i++) { a += 0.1; r = e.Sample(i * 0.01, new[] { a, a, a, a }); }
                    double err = r.Counts[0] * e.RadPerCount - a; s2 += err * err;
                }
                return Math.Sqrt(s2 / n);
            }
            double r3 = Rms(0.03), r6 = Rms(0.06);
            Assert.InRange(r3, 0.022, 0.038);   // random-walk of per-step slip over 100 steps of 0.1 rad: 0.03*0.1*sqrt(100)=0.03
            Assert.InRange(r6 / r3, 1.7, 2.3);
        }

        [Fact]
        public void Encoder_RateIsCountDifferenceOverTime()
        {
            var e = new EncoderModel(1440, 0.0, 1, "x"); e.Sample(0.0, new double[4]);
            var r = e.Sample(0.01, new[] { 0.05, 0.05, 0.05, 0.05 });
            Assert.Equal(0.05, r.RatesRadS[0] * 0.01, 2);
        }

        [Fact]
        public void Gyro_WhiteNoise_MatchesDensityOverSqrtDt()
        {
            var imu = new ImuModel(0.0003, 0.0, 5, "g"); double dt = 0.005; var v = new List<double>();
            imu.Sample(0, 0.2);
            for (int i = 1; i <= 20000; i++) v.Add(imu.Sample(i * dt, 0.2).GyroZ - 0.2);
            double sd = Math.Sqrt(v.Select(x => x * x).Average());
            Assert.InRange(sd / (0.0003 / Math.Sqrt(dt)), 0.95, 1.05);
        }

        [Fact]
        public void Gyro_BiasRandomWalk_VarianceGrowsLinearlyWithTime()
        {
            double Var(double T)
            {
                var b = new List<double>();
                for (int k = 0; k < 300; k++) { var imu = new ImuModel(0.0, 0.0005, (ulong)(1000 + k), "b"); imu.Sample(0, 0); for (double t = 0.01; t <= T + 1e-9; t += 0.01) imu.Sample(t, 0); b.Add(imu.TrueBias); }
                return b.Select(x => x * x).Average();
            }
            double v10 = Var(10), v40 = Var(40);
            Assert.InRange(v40 / v10, 3.0, 5.2); Assert.InRange(v10 / (0.0005 * 0.0005 * 10), 0.8, 1.25);
        }

        [Fact]
        public void Tof_SeesBoxInsideTheCone_NotOutside_AndRespectsMaxRange()
        {
            var suite = SensorSuite.ForRover(TestEnv.Profile(), "lifter_1"); var tof = new TofModel(suite.Tofs[0]) { SigmaBase = 0, SigmaPerMetre = 0, Dropout = 0 };
            var wall = new Obb("w", new Vec3(1.21, 0, 0.5), new Vec3(0.01, 2, 0.5));
            var res = tof.Sample(Pose.Identity, new[] { wall }, 0, new Rng(1));
            Assert.True(res.Valid); Assert.Equal(0.99, res.Range, 2);                // sensor at x = 0.21, wall face at 1.20
            var side = new Obb("s", new Vec3(1.0, 0.9, 0.06), new Vec3(0.05, 0.05, 0.05));   // 0.9 m left at 0.8 m: 48 deg off axis, cone half angle 0.22 rad = 12.6 deg
            Assert.False(tof.Sample(Pose.Identity, new[] { side }, 0, new Rng(1)).Valid);
            var edge = new Obb("e", new Vec3(1.21, 0.1, 0.06), new Vec3(0.01, 0.05, 0.05));   // 0.1 m lateral at 1.0 m: 5.7 deg inside the cone
            Assert.True(tof.Sample(Pose.Identity, new[] { edge }, 0, new Rng(1)).Valid);
            var far = new Obb("f", new Vec3(3.5, 0, 0.5), new Vec3(0.01, 2, 0.5));
            Assert.False(tof.Sample(Pose.Identity, new[] { far }, 0, new Rng(1)).Valid);
        }

        [Fact]
        public void Tof_NoiseAndQuantisation()
        {
            var suite = SensorSuite.ForRover(TestEnv.Profile(), "cargo_1"); var tof = new TofModel(suite.Tofs[0]) { Dropout = 0 };
            var wall = new Obb("w", new Vec3(1.21, 0, 0.5), new Vec3(0.01, 2, 0.5)); var rng = new Rng(3);
            var r = Enumerable.Range(0, 3000).Select(i => tof.Sample(Pose.Identity, new[] { wall }, 0, rng).Range).ToArray();
            double sd = Math.Sqrt(r.Select(x => (x - r.Average()) * (x - r.Average())).Average());
            Assert.InRange(sd, 0.012, 0.018); Assert.All(r, x => Assert.True(Math.Abs(x * 1000 - Math.Round(x * 1000)) < 1e-6));
        }

        [Fact]
        public void Lidar_RangesToWallAreExactWithoutNoise_AndCoverFullCircle()
        {
            var suite = SensorSuite.ForRover(TestEnv.Profile(), "scout_1"); var lid = new LidarModel(suite.Lidar) { SigmaBase = 0, SigmaPerMetre = 0, Dropout = 0 };
            var wall = new Obb("w", new Vec3(3, 0, 0.5), new Vec3(0.05, 5, 0.5));
            var s = lid.Sample(Pose.Identity, new[] { wall }, 0, new Rng(1));
            Assert.Equal(720, s.Ranges.Length);
            int i0 = (int)Math.Round((0 - s.AngleMin) / s.AngleIncrement);
            Assert.Equal(2.95, s.Ranges[i0], 6);
            Assert.True(double.IsInfinity(s.Ranges[0]));   // pointing -x: nothing there
            Assert.Equal(2 * Math.PI, s.AngleIncrement * 720, 9);
        }

        [Fact]
        public void Lidar_DoesNotSeeElevatedBeam_ButSeesGroundBoxes()
        {
            var suite = SensorSuite.ForRover(TestEnv.Profile(), "scout_1"); var lid = new LidarModel(suite.Lidar) { SigmaBase = 0, SigmaPerMetre = 0, Dropout = 0 };
            var scene = TestEnv.Scene(); var pose = Pose.FromYaw(3.0, 3.4, 0, 0);     // beam is straight ahead at x = 3.3
            var s = lid.Sample(pose, scene.Boxes, 0, new Rng(1)); int i0 = (int)Math.Round((0 - s.AngleMin) / s.AngleIncrement);
            Assert.True(s.Ranges[i0] > 3.0, $"lidar must see through under the beam, got {s.Ranges[i0]}");
            var pose2 = Pose.FromYaw(4.6, 3.5, 0, -Math.PI / 2);   // looking south at pallet_stack (y up to 2.6)
            var s2 = lid.Sample(pose2, scene.Boxes, 0, new Rng(1)); int j0 = (int)Math.Round((0 - s2.AngleMin) / s2.AngleIncrement);
            Assert.InRange(s2.Ranges[j0], 0.85, 0.95);
        }

        [Fact]
        public void Lidar_DropoutRate_And_MaxRange()
        {
            var suite = SensorSuite.ForRover(TestEnv.Profile(), "scout_1"); var lid = new LidarModel(suite.Lidar) { Dropout = 0.2 };
            var ring = new[] { new Obb("a", new Vec3(0, 0, 0.5), new Vec3(5, 5, 0.5)) };
            // origin is inside the box: every ray starts inside -> t=0 hits; use two walls instead
            var walls = new[] { new Obb("n", new Vec3(0, 3, 0.5), new Vec3(10, 0.1, 0.5)), new Obb("s", new Vec3(0, -3, 0.5), new Vec3(10, 0.1, 0.5)) };
            var scan = lid.Sample(Pose.FromYaw(0, 0, 0, 0), walls, 0, new Rng(9));
            int valid = scan.Ranges.Count(r => !double.IsInfinity(r)); int geometric = 0;
            for (int i = 0; i < 720; i++) { double a = -Math.PI + i * scan.AngleIncrement; double s = Math.Abs(Math.Sin(a)); if (s > 1e-6 && 3 / s < 12.0 - 0.1 && 3 / s > 0.05 + 0.1) geometric++; }
            Assert.InRange(valid / (double)geometric, 0.7, 0.9);
        }
    }
}
