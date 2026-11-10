using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Crane.Fleet.Perception.Evaluator;
using Xunit;

namespace Crane.Fleet.Perception.Tests
{
    public class ProfileV1Tests
    {
        [Fact]
        public void V1_Parses_DescriptiveLeavesAreMetadata_NotNumbers()
        {
            var p = TestEnv.ProfileV1();
            Assert.Equal("mecanum_fleet_concept_v1", p.ProfileId);
            Assert.All(p.Leaves.Values, l => Assert.Contains(l.Provenance, p.ProvenanceLevels));
            Assert.All(p.Metadata.Values, l => Assert.Contains(l.Provenance, p.ProvenanceLevels));
            foreach (var path in new[] { "chassis.wheel.motor", "docking.magnet", "docking.partner_acquisition", "roles.scout.compute", "roles.scout.mcu", "roles.lifter.compute", "roles.lifter.lifter_option", "roles.scout.arm_proxy.servo" })
            {
                Assert.True(p.Metadata.ContainsKey(path), path);
                Assert.False(p.Leaves.ContainsKey(path), path + " must not be a numeric leaf");
            }
            Assert.Equal("b", p.Text("roles.lifter.lifter_option"));
            Assert.Equal("bevel_tags_staged", p.Text("docking.partner_acquisition"));
            Assert.StartsWith("Pololu 47:1", p.Text("chassis.wheel.motor"));
            Assert.Equal("https://www.pololu.com/product/4845", p.Meta("chassis.wheel.motor").Source);
            var mag = Assert.IsType<Dictionary<string, object>>(p.Meta("docking.magnet").Value);
            Assert.Equal(0.01, (double)mag["diameter"]);
            Assert.Null(p.Meta("docking.magnet").Text);
            // never silently coerced: numeric access to a descriptive leaf throws
            Assert.Throws<InvalidOperationException>(() => p.Num("roles.lifter.lifter_option"));
            Assert.Throws<InvalidOperationException>(() => p.Vec("docking.magnet"));
        }

        [Fact]
        public void V1_NumericValuesAndSources_AreCarried()
        {
            var p = TestEnv.ProfileV1();
            Assert.Equal(2249, p.Encoder.CountsPerRev); Assert.Equal(0.05, p.StereoBaseline); Assert.Equal(0.0485, p.Encoder.WheelRadius);
            Assert.Equal(1.2217, p.Camera.Hfov); Assert.Equal(0.26, p.BodySizeLwh[1]);
            Assert.Equal("https://www.pololu.com/product/4845", p.Get("chassis.wheel.encoder_counts_per_rev").Source);
        }

        [Fact]
        public void V1_Lidar_IsRplidarC1Datasheet()
        {
            var l = TestEnv.ProfileV1().Lidar;   // https://www.slamtec.com/en/C1/Spec : 0.05-12 m white, 5 kHz, 8-12 Hz (10 typ), 0.72 deg
            Assert.Equal("SLAMTEC RPLIDAR C1", l.Model); Assert.Equal(12.0, l.MaxRange); Assert.Equal(10.0, l.RateHz); Assert.Equal(500, l.Points);
            Assert.Equal(0.72, l.AngularResolution * 180 / Math.PI, 9);
            Assert.Equal("https://www.slamtec.com/en/C1/Spec", l.Sources["max_range"]); Assert.Equal("datasheet", l.Provenance["max_range"]);
        }

        [Fact]
        public void V1_D435_SpecFromDatasheet()
        {
            var d = TestEnv.ProfileV1().D435;
            Assert.NotNull(d);
            Assert.Equal(1280, d.Depth.Width); Assert.Equal(720, d.Depth.Height); Assert.Equal(30, d.Depth.RateHz); Assert.Equal(0.3, d.Depth.MinRange); Assert.Equal(3.0, d.Depth.MaxRange);
            Assert.Equal(87.0, d.Depth.Hfov * 180 / Math.PI, 1); Assert.Equal("16UC1", d.Depth.Encoding); Assert.Equal("global", d.Infra.Shutter); Assert.Equal("mono8", d.Infra.Encoding);
            Assert.Equal(0.05, d.IrBaseline); Assert.Equal("datasheet", d.Depth.Provenance["hfov"]); Assert.StartsWith("https://", d.Depth.Sources["hfov"]);
            Assert.Equal("rolling", d.Color.Shutter); Assert.Equal("provisional", d.Color.Provenance["hfov"]);
            Assert.InRange(d.Depth.Intrinsics.Vfov * 180 / Math.PI, 55.0, 59.0);   // pinhole vertical FOV from 87 deg H on 1280x720 (datasheet 58 +-1; lens distortion makes the pinhole value ~56)
        }

        [Fact]
        public void V1_RoleSensorSets_MatchProfile()
        {
            var p = TestEnv.ProfileV1();
            Assert.Equal(new[] { SensorKind.WheelEncoder, SensorKind.Imu, SensorKind.Lidar2D, SensorKind.RealSenseD435 }, p.Roles["scout"].Sensors.Select(t => t.Kind).ToArray());
            Assert.Equal(new[] { "depth", "infra1", "infra2", "color" }, p.Roles["scout"].Sensors[3].Streams.ToArray());
            Assert.Equal(new[] { SensorKind.WheelEncoder, SensorKind.Imu, SensorKind.MonoCamera }, p.Roles["lifter"].Sensors.Select(t => t.Kind).ToArray());
            Assert.Equal("mast", p.Roles["lifter"].Sensors[2].Facing); Assert.Equal("front", p.Roles["cargo"].Sensors[2].Facing);
            Assert.Equal(new[] { "camera_infra1", "camera_infra2" }, p.Roles["scout"].StereoPair.ToArray());

            foreach (var m in p.Fleet)
            {
                var suite = SensorSuite.ForRover(p, m.Id); var role = p.Roles[m.Role];
                Assert.Equal(role.RequiredSensorFrames.Keys.OrderBy(x => x).ToArray(), suite.SensorIds().OrderBy(x => x).ToArray());
                Assert.True(suite.HasEncoders && suite.HasImu);
            }
            var scout = SensorSuite.ForRover(p, "scout_1");
            Assert.NotNull(scout.Lidar); Assert.Equal(500, scout.Lidar.Points); Assert.Equal(2, scout.Cameras.Count); Assert.Single(scout.Depths); Assert.Empty(scout.Tofs);
            Assert.Equal(new[] { "camera_color" }, scout.Unsimulated.ToArray());
            foreach (var id in new[] { "lifter_1", "cargo_1", "cargo_2" }) { var r = SensorSuite.ForRover(p, id); Assert.Null(r.Lidar); Assert.Single(r.Cameras); Assert.Empty(r.Depths); Assert.Equal("cam_front", r.Cameras[0].SensorId); }
        }

        [Fact]
        public void V1_D435_StereoIsTwoMonoImagersAt50mm_AndDepthSharesTheLeftImager()
        {
            var s = SensorSuite.ForRover(TestEnv.ProfileV1(), "scout_1");
            var l = s.Cameras.Single(c => c.Name == "stereo_left"); var r = s.Cameras.Single(c => c.Name == "stereo_right");
            Assert.Equal(0.05, (l.OpticalInRover.P - r.OpticalInRover.P).Norm, 12);
            Assert.Equal(l.OpticalInRover.P.X, r.OpticalInRover.P.X, 12); Assert.True(l.OpticalInRover.P.Y > r.OpticalInRover.P.Y);   // infra1 left (+y)
            Assert.Equal(1280, l.K.Width); Assert.Equal(720, l.K.Height); Assert.Equal(1.5184, l.K.Hfov, 9); Assert.Equal("camera_infra1", l.SensorId); Assert.Equal("camera_infra2", r.SensorId);
            Assert.Equal(0.05, s.StereoBaseline);
            var dm = s.Depths[0]; Assert.Equal(l.OpticalInRover.P, dm.OpticalInRover.P); Assert.Equal(0.3, dm.MinRange); Assert.Equal(3.0, dm.MaxRange);
        }

        [Fact]
        public void V1_MastCamera_IsOv9281AtTheCadMastPose_WithProfilePitch()
        {
            var p = TestEnv.ProfileV1(); var c = SensorSuite.ForRover(p, "lifter_1").Cameras.Single();
            Assert.Equal("cam_front", c.Name); Assert.Equal("front", c.Facing); Assert.Equal(1280, c.K.Width); Assert.Equal(800, c.K.Height); Assert.Equal(1.2217, c.K.Hfov, 9);
            Assert.Equal(60, c.RateHz); Assert.Equal(0.03, c.LatencyS);
            Assert.Equal(0.09859958, c.OpticalInRover.P.X, 9); Assert.Equal(-0.04, c.OpticalInRover.P.Y, 9); Assert.Equal(0.126210623, c.OpticalInRover.P.Z, 9);
            // optical axis (z column) pitched down by mast_camera_pitch = 0.35 rad
            var axis = c.OpticalInRover.R.Col(2); Assert.Equal(Math.Cos(0.35), axis.X, 9); Assert.Equal(-Math.Sin(0.35), axis.Z, 9);
            var cargo = SensorSuite.ForRover(p, "cargo_1").Cameras.Single(); Assert.Equal(1.0, cargo.OpticalInRover.R.Col(2).X, 9);
        }

        [Fact]
        public void DepthModel_MeasuresAWall_WithStereoErrorAndRangeGate()
        {
            var s = SensorSuite.ForRover(TestEnv.ProfileV1(), "scout_1"); var m = s.Depths[0];
            var dm = new DepthModel(m) { Dropout = 0, SubpixelSigma = 0 };
            var wall = new Obb("w", new Vec3(2.2, 0, 0.5), new Vec3(0.05, 5, 0.5));          // face at x = 2.15; depth origin x = 0.1508 -> z = 2.0 on axis
            var f = dm.Sample(Pose.Identity, new[] { wall }, 1.0, new Rng(1));
            Assert.Equal(1280 / 16, f.Width); Assert.Equal(720 / 16, f.Height);
            double zc = f.Z[(f.Height / 2) * f.Width + f.Width / 2]; Assert.InRange(zc, 1.99, 2.01);   // nearest sampled pixel is near the axis
            Assert.True(f.Z.Where(z => z > 0).All(z => z >= 0.3 && z <= 3.0));
            var near = new Obb("n", new Vec3(0.30, 0, 0.5), new Vec3(0.05, 5, 0.5));        // face at 0.25 -> 0.099 m in front of the sensor: below min range
            Assert.All(dm.Sample(Pose.Identity, new[] { near }, 0, new Rng(1)).Z, z => Assert.Equal(0.0, z));
            var far = new Obb("f", new Vec3(4.5, 0, 0.5), new Vec3(0.05, 5, 0.5));
            Assert.All(dm.Sample(Pose.Identity, new[] { far }, 0, new Rng(1)).Z, z => Assert.Equal(0.0, z));
            var noisy = new DepthModel(m) { Dropout = 0 };
            Assert.Equal(2.0 * 2.0 * 0.08 / (m.K.Fx * 0.05), noisy.SigmaZ(2.0), 12);
            Assert.InRange(noisy.SigmaZ(2.0), 0.005, 0.015);
        }

        [Fact]
        public void V1_Lidar_C1_ScansAt500PointsFromTheCadLaserFrame()
        {
            var s = SensorSuite.ForRover(TestEnv.ProfileV1(), "scout_1");
            Assert.Equal(-0.122, s.Lidar.InRover.P.X, 9); Assert.Equal(-0.072, s.Lidar.InRover.P.Y, 9); Assert.Equal(12.0, s.Lidar.MaxRange);
            var lid = new LidarModel(s.Lidar) { SigmaBase = 0, SigmaPerMetre = 0, Dropout = 0 };
            var wall = new Obb("w", new Vec3(3, 0, 0.15), new Vec3(0.05, 5, 0.5));
            var scan = lid.Sample(Pose.Identity, new[] { wall }, 0, new Rng(1)); Assert.Equal(500, scan.Ranges.Length);
            Assert.Equal(2.95 + 0.122, scan.Ranges[(int)Math.Round(Math.PI / scan.AngleIncrement)], 6);
        }

        [Fact]
        public void V1_FleetSimulator_ConstructsAndDeliversSensorsForEveryRover()
        {
            var p = TestEnv.ProfileV1(); var truth = new EvaluatorTruth(); double x = 1;
            foreach (var m in p.Fleet) truth.SetTrajectory(m.Id, Trajectories.Static(Pose.FromYaw(x++, 1, 0, 0)));
            var sim = new FleetSimulator(p, TestEnv.Scene(), truth, seed: 7);
            foreach (var m in p.Fleet)
            {
                var prov = sim.ProviderFor(m.Id);
                Assert.NotNull(prov.PollEncoders(1.0)); Assert.NotNull(prov.PollImu(1.0));
                prov.PollTagObservations(1.0);
            }
        }

        // ---------------------------------------------------------------- malformed numeric fields fail loudly, with the path
        private static string V1Raw() => File.ReadAllText(TestEnv.ProfileV1Path);

        [Fact]
        public void MalformedNumericField_FailsWithFieldPath_NotCoercedToMetadata()
        {
            // a string where the schema says numeric (encoder counts) -- must not be accepted as descriptive
            string json = V1Raw().Replace("\"encoder_counts_per_rev\": {\n    \"value\": 2249,", "\"encoder_counts_per_rev\": {\n    \"value\": \"2249\",");
            Assert.NotEqual(V1Raw(), json);
            var e = Assert.Throws<ProfileException>(() => FleetProfile.Parse(json));
            Assert.Contains(e.Errors, m => m.StartsWith("chassis.wheel.encoder_counts_per_rev:") && m.Contains("not numeric"));
            Assert.Contains("chassis.wheel.encoder_counts_per_rev", e.Message);
        }

        [Fact]
        public void MalformedNumericArrayElement_FailsWithPathAndIndex()
        {
            string json = System.Text.RegularExpressions.Regex.Replace(V1Raw(), "(\"mono_camera_ov9281\": \\{\\s*\"resolution\": \\{\\s*\"value\": \\[\\s*1280,\\s*)800", "$1\"800\"");
            Assert.NotEqual(V1Raw(), json);
            var e = Assert.Throws<ProfileException>(() => FleetProfile.Parse(json));
            Assert.Contains(e.Errors, m => m.StartsWith("sensors.mono_camera_ov9281.resolution:") && m.Contains("value[1]"));
        }

        [Fact]
        public void MalformedNumeric_InV0_AlsoFailsWithPath()
        {
            string json = File.ReadAllText(TestEnv.ProfilePath).Replace("{\"value\": 60, \"provenance\": \"provisional\"}", "{\"value\": \"fast\", \"provenance\": \"provisional\"}");
            var e = Assert.Throws<ProfileException>(() => FleetProfile.Parse(json));
            Assert.Contains(e.Errors, m => m.Contains(": value is not numeric") && m.Contains("\"fast\""));
        }

        [Fact]
        public void UnknownDescriptiveLookingLeaf_StillFails_WhenNotInSchema()
        {
            string json = V1Raw().Replace("\"hook_actuation_time\": {\n   \"value\": 0.8,", "\"hook_actuation_time\": {\n   \"value\": \"slow\",");
            Assert.NotEqual(V1Raw(), json);
            var e = Assert.Throws<ProfileException>(() => FleetProfile.Parse(json));
            Assert.Contains(e.Errors, m => m.StartsWith("docking.hook_actuation_time:"));
        }

        [Fact]
        public void BadD435Token_AndMissingD435Block_AndBaselineMismatch_Fail()
        {
            string raw = V1Raw();
            Assert.Throws<ProfileException>(() => FleetProfile.Parse(raw.Replace("depth + infra1/infra2 + color", "depth + infra3")));
            Assert.Throws<ProfileException>(() => FleetProfile.Parse(raw.Replace("mono_camera_ov9281(mast)", "mono_camera_ov9281(roof)")));
            var e = Assert.Throws<ProfileException>(() => FleetProfile.Parse(raw.Replace("\"ir_baseline\": {\n    \"value\": 0.05,", "\"ir_baseline\": {\n    \"value\": 0.06,")));
            Assert.Contains(e.Errors, m => m.Contains("ir_baseline"));
        }

        [Fact]
        public void BadProvenanceOnDescriptiveLeaf_StillFails()
        {
            string json = V1Raw().Replace("\"value\": \"b\",\n    \"provenance\": \"provisional\"", "\"value\": \"b\",\n    \"provenance\": \"guess\"");
            Assert.NotEqual(V1Raw(), json);
            var e = Assert.Throws<ProfileException>(() => FleetProfile.Parse(json));
            Assert.Contains(e.Errors, m => m.StartsWith("roles.lifter.lifter_option:") && m.Contains("guess"));
        }
    }
}
