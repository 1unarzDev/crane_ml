using System;
using System.Collections.Generic;
using System.IO;

namespace Crane.Fleet.Perception.Tests
{
    internal static class TestEnv
    {
        public static string FleetDir
        {
            get
            {
                var d = new DirectoryInfo(AppContext.BaseDirectory);
                while (d != null && !File.Exists(Path.Combine(d.FullName, "spec", "fleet_profile_v0.json"))) d = d.Parent;
                if (d == null) throw new FileNotFoundException("Tools/Fleet/spec not found above " + AppContext.BaseDirectory);
                return d.FullName;
            }
        }
        public static string ProfilePath => Path.Combine(FleetDir, "spec", "fleet_profile_v0.json");
        public static string ScenarioPath => Path.Combine(FleetDir, "spec", "scenarios", "warehouse_a_v0.json");
        public static string ParityPath => Path.Combine(FleetDir, "core_cs", "Crane.Fleet.Perception.Tests", "parity", "perception_parity_v0.json");
        public static string ProfileV1Path => Path.Combine(FleetDir, "spec", "fleet_profile_v1.json");
        public static FleetProfile ProfileV1() => FleetProfile.Load(ProfileV1Path);
        public static FleetProfile Profile() => FleetProfile.Load(ProfilePath);
        public static Scenario Scene() => Scenario.Load(ScenarioPath);

        /// <summary>Camera at <paramref name="pos"/> looking along +x (yaw 0) in world, optical frame.</summary>
        public static Pose CamAt(double x, double y, double z, double yaw = 0, double pitchDown = 0) => CameraMount.InRover(new Vec3(x, y, z), yaw, pitchDown);
        public static CameraIntrinsics K() => CameraIntrinsics.FromHfov(1280, 800, 1.40);
        /// <summary>Tag facing -x (towards a camera looking along +x) centred at p.</summary>
        public static Pose TagFacingCamera(double x, double y, double z, double viewAngleRad = 0) => TagSpec.OnVerticalFace(new Vec3(x, y, z), Math.PI + viewAngleRad);
    }
}
