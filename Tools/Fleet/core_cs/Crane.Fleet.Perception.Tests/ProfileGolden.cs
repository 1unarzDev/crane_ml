using System;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using Xunit;

namespace Crane.Fleet.Perception.Tests
{
    /// <summary>Canonical text dump of everything a v0 parse produces (leaves, typed specs, role sensors, per-rover suites). Round-trip doubles ("R").</summary>
    internal static class ProfileDump
    {
        private static string N(double v) => v.ToString("R", CultureInfo.InvariantCulture);
        private static string Pose(Pose p) => $"R=[{string.Join(",", new[] { p.R.M00, p.R.M01, p.R.M02, p.R.M10, p.R.M11, p.R.M12, p.R.M20, p.R.M21, p.R.M22 }.Select(N))}] p=[{N(p.P.X)},{N(p.P.Y)},{N(p.P.Z)}]";

        public static string Dump(FleetProfile p)
        {
            var sb = new StringBuilder();
            sb.AppendLine($"schema={p.Schema} id={p.ProfileId} levels={string.Join("|", p.ProvenanceLevels)}");
            foreach (var kv in p.Leaves.OrderBy(k => k.Key, StringComparer.Ordinal))
                sb.AppendLine($"leaf {kv.Key} [{string.Join(",", kv.Value.Numbers.Select(N))}] {kv.Value.Provenance}");
            var c = p.Camera; sb.AppendLine($"camera {c.Width}x{c.Height} {N(c.Hfov)} {N(c.RateHz)} {N(c.Latency)} {c.Shutter} {c.Encoding} {string.Join(",", c.Provenance.OrderBy(k => k.Key, StringComparer.Ordinal).Select(k => k.Key + "=" + k.Value))}");
            sb.AppendLine($"overhead {N(p.Overhead.Height)} {N(p.Overhead.Hfov)} {N(p.Overhead.RateHz)} {N(p.Overhead.Latency)} {p.Overhead.Optional}");
            sb.AppendLine($"tof {N(p.Tof.MaxRange)} {N(p.Tof.Fov)} {N(p.Tof.RateHz)}");
            sb.AppendLine($"lidar {N(p.Lidar.MaxRange)} {N(p.Lidar.RateHz)} {p.Lidar.Points}");
            sb.AppendLine($"imu {N(p.Imu.RateHz)} {N(p.Imu.GyroNoiseDensity)} {N(p.Imu.GyroBiasInstability)}");
            sb.AppendLine($"encoder {N(p.Encoder.RateHz)} {p.Encoder.CountsPerRev} {N(p.Encoder.WheelRadius)} {N(p.Encoder.HalfWheelbaseLx)} {N(p.Encoder.HalfTrackLy)}");
            sb.AppendLine($"misc {N(p.StereoBaseline)} {N(p.FaceTagSize)} {N(p.TopTagSize)} [{string.Join(",", p.BodySizeLwh.Select(N))}] {N(p.SlipSigmaFracHardTile)}");
            sb.AppendLine($"active {string.Join(",", p.ActiveFaces)}");
            foreach (var f in p.FaceFrames.OrderBy(k => k.Key, StringComparer.Ordinal)) sb.AppendLine($"face {f.Key} {N(f.Value.X)} {N(f.Value.Y)} {N(f.Value.Z)} {N(f.Value.Yaw)} {f.Value.Provenance}");
            foreach (var r in p.Roles.OrderBy(k => k.Key, StringComparer.Ordinal))
                sb.AppendLine($"role {r.Key} mast={N(r.Value.MastHeight)} sensors={string.Join(";", r.Value.Sensors.Select(s => s.Kind + ":" + s.Facing + ":" + s.Raw))}");
            foreach (var m in p.Fleet)
            {
                sb.AppendLine($"fleet {m.Id} {m.Role}");
                var s = SensorSuite.ForRover(p, m.Id);
                sb.AppendLine($" suite enc={s.HasEncoders} imu={s.HasImu} base={N(s.StereoBaseline)}");
                foreach (var cam in s.Cameras) sb.AppendLine($" cam {cam.Name} {cam.Facing} {Pose(cam.OpticalInRover)} {cam.K.Width}x{cam.K.Height} {N(cam.K.Fx)} {N(cam.K.Fy)} {N(cam.K.Cx)} {N(cam.K.Cy)} {N(cam.RateHz)} {N(cam.LatencyS)} {cam.Provenance}");
                foreach (var t in s.Tofs) sb.AppendLine($" tof {t.Name} {Pose(t.InRover)} {N(t.MaxRange)} {N(t.ConeHalfAngle)} {N(t.RateHz)}");
                if (s.Lidar != null) sb.AppendLine($" lidar {s.Lidar.Name} {Pose(s.Lidar.InRover)} {N(s.Lidar.MaxRange)} {N(s.Lidar.RateHz)} {s.Lidar.Points}");
            }
            return sb.ToString();
        }
    }

    public class ProfileGoldenTests
    {
        [Fact]
        public void V0_ParsesToIdenticalValuesAsBeforeV1Support()
        {
            string golden = Path.Combine(TestEnv.FleetDir, "core_cs", "Crane.Fleet.Perception.Tests", "parity", "profile_v0_golden.txt");
            string dump = ProfileDump.Dump(TestEnv.Profile());
            if (Environment.GetEnvironmentVariable("CRANE_WRITE_GOLDEN") == "1") File.WriteAllText(golden, dump);
            Assert.Equal(File.ReadAllText(golden).Replace("\r\n", "\n"), dump.Replace("\r\n", "\n"));
        }
    }
}
