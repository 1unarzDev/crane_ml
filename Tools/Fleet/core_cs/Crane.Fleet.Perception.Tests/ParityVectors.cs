using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using Xunit;

namespace Crane.Fleet.Perception.Tests
{
    /// <summary>
    /// Parity vectors: camera pose, tag pose, boxes -> visible? (+ reason) and, for visible cases, the first-order pose-error scale at
    /// 1 px corner noise. Regenerate with FLEET_WRITE_PARITY=1 dotnet test --filter ParityVectors. Cases whose decision flips under a tiny
    /// perturbation of any threshold or box size are rejected so cross-implementation float noise cannot cause false disagreements.
    /// </summary>
    public class ParityVectors
    {
        private sealed class Case
        {
            public string Id, Category; public int W = 1280, H = 800; public double Hfov = 1.40;
            public Pose Cam, Tag; public double TagSize; public List<Obb> Boxes = new List<Obb>(); public VisibilityParams P = new VisibilityParams();
        }

        private static string D(double v) => v.ToString("R", CultureInfo.InvariantCulture);
        private static string Arr(IEnumerable<double> v) => "[" + string.Join(",", v.Select(D)) + "]";
        private static IEnumerable<double> RowMajor(Mat3 m) { for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) yield return m[i, j]; }

        private static VisibilityResult Eval(Case c, VisibilityParams p, double boxGrow = 0)
        {
            var K = CameraIntrinsics.FromHfov(c.W, c.H, c.Hfov);
            var boxes = c.Boxes.Select(b => new Obb(b.Id, b.Centre, b.Half + new Vec3(boxGrow, boxGrow, boxGrow), b.Yaw)).ToList();
            return TagVisibility.Evaluate(c.Cam, K, c.Tag, c.TagSize, boxes, p);
        }

        private static VisibilityParams Scale(VisibilityParams p, double f) => new VisibilityParams { MinPixels = p.MinPixels * f, MaxViewAngle = p.MaxViewAngle * f, MinRange = p.MinRange * f, NearPlane = p.NearPlane, OcclusionEps = p.OcclusionEps * f };

        private static bool Stable(Case c)
        {
            var r0 = Eval(c, c.P);
            foreach (double f in new[] { 1 - 1e-4, 1 + 1e-4 })
                if (Eval(c, Scale(c.P, f)).Reason != r0.Reason) return false;
            foreach (double g in new[] { -2e-5, 2e-5 })
                if (Eval(c, c.P, g).Reason != r0.Reason) return false;
            // image-edge decisions: shift camera slightly
            foreach (double dy in new[] { -1e-6, 1e-6 })
            {
                var c2 = new Case { Id = c.Id, W = c.W, H = c.H, Hfov = c.Hfov, Cam = new Pose(c.Cam.R, c.Cam.P + new Vec3(0, dy, 0)), Tag = c.Tag, TagSize = c.TagSize, Boxes = c.Boxes, P = c.P };
                if (Eval(c2, c2.P).Reason != r0.Reason) return false;
            }
            return true;
        }

        private static Pose RandomTagPose(Rng r, Vec3 near, double viewDegMax, out double normalYaw)
        {
            normalYaw = r.Uniform(-Math.PI, Math.PI);
            return TagSpec.OnVerticalFace(near, normalYaw);
        }

        private static List<Case> Generate()
        {
            var r = new Rng(20261109UL); var cases = new List<Case>(); int guard = 0;
            void Add(Case c, string cat) { c.Category = cat; if (Stable(c)) { c.Id = $"{cat}-{cases.Count(x => x.Category == cat):D3}"; cases.Add(c); } }
            double[] sizes = { 0.05, 0.12 };

            // A: generic, no boxes. Camera at random pose near rover heights looking roughly at the tag.
            while (cases.Count(x => x.Category == "generic") < 50 && guard++ < 100000)
            {
                var cpos = new Vec3(r.Uniform(0, 2), r.Uniform(0, 2), r.Uniform(0.1, 0.5));
                double range = r.Uniform(0.3, 4.5), bearing = r.Uniform(-0.9, 0.9);
                var tpos = cpos + new Vec3(Math.Cos(bearing), Math.Sin(bearing), r.Uniform(-0.2, 0.2) / range) * range;
                var tag = TagSpec.OnVerticalFace(tpos, Math.PI + bearing + r.Uniform(-1.1, 1.1));
                double sz = sizes[r.NextU64() % 2 == 0 ? 0 : 1]; range = Math.Min(range, 1.6 * TestEnv.K().MaxRangeFrontal(sz, 32.0));
                tpos = cpos + new Vec3(Math.Cos(bearing), Math.Sin(bearing), 0) * range; tag = TagSpec.OnVerticalFace(new Vec3(tpos.X, tpos.Y, cpos.Z + r.Uniform(-0.1, 0.1)), Math.PI + bearing + r.Uniform(-1.1, 1.1));
                Add(new Case { Cam = CameraMount.InRover(cpos, r.Uniform(-0.3, 0.3), r.Uniform(-0.2, 0.2)), Tag = tag, TagSize = sz }, "generic");
            }
            // B: facing tags with random boxes between
            guard = 0;
            while (cases.Count(x => x.Category == "boxes") < 60 && guard++ < 100000)
            {
                var cpos = new Vec3(0, 0, r.Uniform(0.15, 0.5)); double sz = sizes[r.NextU64() % 2 == 0 ? 0 : 1];
                double range = r.Uniform(0.5, 0.95 * TestEnv.K().MaxRangeFrontal(sz, 32.0)), lat = r.Uniform(-0.3, 0.3) * range;
                var tag = TagSpec.OnVerticalFace(new Vec3(range, lat, cpos.Z + r.Uniform(-0.1, 0.1)), Math.PI + r.Uniform(-0.9, 0.9));
                var c = new Case { Cam = CameraMount.InRover(cpos, 0, 0), Tag = tag, TagSize = sz };
                int nb = 1 + (int)(r.NextU64() % 3);
                for (int i = 0; i < nb; i++)
                {
                    double bx = r.Uniform(0.2, range + 0.5), by = r.Uniform(-0.6, 0.6) * Math.Max(0.5, bx / range), bz = r.Uniform(0, 0.8);
                    c.Boxes.Add(new Obb("b" + i, new Vec3(bx, by, bz), new Vec3(r.Uniform(0.01, 0.15), r.Uniform(0.01, 0.3), r.Uniform(0.02, 0.5)), r.Uniform(0, Math.PI)));
                }
                Add(c, "boxes");
            }
            // C: range edge at the pixel-size limit
            foreach (double size in sizes)
                for (int i = 0; i < 20; i++)
                {
                    double rmax = TestEnv.K().MaxRangeFrontal(size, 32.0), range = rmax * (0.85 + 0.3 * i / 19.0);
                    Add(new Case { Cam = CameraMount.InRover(new Vec3(0, 0, 0.3), 0, 0), Tag = TagSpec.OnVerticalFace(new Vec3(range, 0, 0.3), Math.PI), TagSize = size }, "range_edge");
                }
            // D: view-angle edge at 65 deg
            for (int i = 0; i < 30; i++)
            {
                double deg = 50 + 30.0 * i / 29.0;
                Add(new Case { Cam = CameraMount.InRover(new Vec3(0, 0, 0.3), 0, 0), Tag = TagSpec.OnVerticalFace(new Vec3(0.9, 0, 0.3), Math.PI + deg * Math.PI / 180), TagSize = 0.12 }, "view_edge");
            }
            // E: image edge: tag sliding across the left edge
            for (int i = 0; i < 30; i++)
            {
                double z = 1.0, edge = z * Math.Tan(0.7), y = edge - 0.15 + 0.3 * i / 29.0;
                Add(new Case { Cam = CameraMount.InRover(new Vec3(0, 0, 0.3), 0, 0), Tag = TagSpec.OnVerticalFace(new Vec3(z, y, 0.3), Math.PI), TagSize = 0.12 }, "fov_edge");
            }
            // F: elevated beam (z 2.15-2.45) on rays at varying heights, plus ground-height controls
            for (int i = 0; i < 30; i++)
            {
                double zc = 1.0 + 1.9 * i / 29.0;
                var c = new Case { Cam = CameraMount.InRover(new Vec3(2.0, 3.4, zc), 0, 0), Tag = TagSpec.OnVerticalFace(new Vec3(6.5, 3.4, zc), Math.PI), TagSize = 0.4 };
                c.Boxes.Add(new Obb("gantry_beam", new Vec3(4.5, 3.4, 2.3), new Vec3(1.2, 0.15, 0.15)));
                Add(c, "beam");
            }
            // G: partial occlusion: a thin pillar crossing exactly one corner ray
            for (int i = 0; i < 20; i++)
            {
                var tag = TagSpec.OnVerticalFace(new Vec3(1.5, 0, 0.3), Math.PI); var cw = TagSpec.WorldCorners(tag, 0.12); int k = i % 4;
                Vec3 camp = new Vec3(0, 0, 0.3), ray = cw[k] - camp; double s = (0.4 + 0.02 * i) / ray.X; Vec3 hit = camp + ray * s;
                var c = new Case { Cam = CameraMount.InRover(camp, 0, 0), Tag = tag, TagSize = 0.12 };
                c.Boxes.Add(new Obb("pillar", hit, new Vec3(0.01, 0.003, 0.003)));
                Add(c, "partial");
            }
            return cases;
        }

        private static string ToJson(List<Case> cases)
        {
            var sb = new StringBuilder();
            sb.Append("{\n\"schema\":\"crane.fleet.perception_parity/v0\",\n");
            sb.Append("\"generator\":\"Crane.Fleet.Perception.Tests.ParityVectors, seed 20261109\",\n");
            sb.Append("\"conventions\":{\"optical_frame\":\"x right, y down, z forward; camera.R columns are optical axes in world; row-major 9\",");
            sb.Append("\"tag_frame\":\"x right, y up, z out of face; tag.R columns are tag axes in world; corners (-h,-h),(h,-h),(h,h),(-h,h) at z=0\",");
            sb.Append("\"reason_order\":\"TooOblique(back-face or angle) > Behind > OutOfFov(all four corners must be inside) > TooClose > TooSmall(sqrt shoelace area) > Occluded(any corner, last occlusion_eps removed)\",");
            sb.Append("\"boxes\":\"centre, half extents (x,y,z), yaw about +z\",\"error_scale\":\"sqrt(trace) of position / rotation block of (J^T J)^-1 at 1 px corner noise, camera axes, at the true pose\"},\n");
            sb.Append("\"cases\":[\n");
            for (int n = 0; n < cases.Count; n++)
            {
                var c = cases[n]; var res = Eval(c, c.P);
                sb.Append("{\"id\":\"" + c.Id + "\",\"category\":\"" + c.Category + "\",");
                sb.Append("\"camera\":{\"width\":" + c.W + ",\"height\":" + c.H + ",\"hfov\":" + D(c.Hfov) + ",\"position\":" + Arr(new[] { c.Cam.P.X, c.Cam.P.Y, c.Cam.P.Z }) + ",\"R\":" + Arr(RowMajor(c.Cam.R)) + "},");
                sb.Append("\"tag\":{\"size\":" + D(c.TagSize) + ",\"centre\":" + Arr(new[] { c.Tag.P.X, c.Tag.P.Y, c.Tag.P.Z }) + ",\"R\":" + Arr(RowMajor(c.Tag.R)) + "},");
                sb.Append("\"boxes\":[" + string.Join(",", c.Boxes.Select(b => "{\"id\":\"" + b.Id + "\",\"centre\":" + Arr(new[] { b.Centre.X, b.Centre.Y, b.Centre.Z }) + ",\"half\":" + Arr(new[] { b.Half.X, b.Half.Y, b.Half.Z }) + ",\"yaw\":" + D(b.Yaw) + "}")) + "],");
                sb.Append("\"params\":{\"min_pixels\":" + D(c.P.MinPixels) + ",\"max_view_angle\":" + D(c.P.MaxViewAngle) + ",\"min_range\":" + D(c.P.MinRange) + ",\"near_plane\":" + D(c.P.NearPlane) + ",\"occlusion_eps\":" + D(c.P.OcclusionEps) + "},");
                sb.Append("\"expected\":{\"visible\":" + (res.Visible ? "true" : "false") + ",\"reason\":\"" + res.Reason + "\",\"range\":" + D(res.Range) + ",\"view_angle\":" + D(res.ViewAngle) + ",\"blocked_corners\":" + res.BlockedCount);
                if (res.Visible)
                {
                    var K = CameraIntrinsics.FromHfov(c.W, c.H, c.Hfov);
                    var truth = c.Cam.Inverse().Compose(c.Tag); var cov = PlanarPnP.Covariance(K, TagSpec.LocalCorners(c.TagSize), truth, 1.0);
                    sb.Append(",\"pixel_size\":" + D(res.PixelSize) + ",\"sigma_pos_m\":" + D(Math.Sqrt(cov[0, 0] + cov[1, 1] + cov[2, 2])) + ",\"sigma_rot_rad\":" + D(Math.Sqrt(cov[3, 3] + cov[4, 4] + cov[5, 5])));
                }
                sb.Append("}}" + (n + 1 < cases.Count ? ",\n" : "\n"));
            }
            sb.Append("]}\n");
            return sb.ToString();
        }

        [Fact]
        public void Vectors_AreGenerated_AtLeast200_WithBothOutcomesAndAllReasons()
        {
            var cases = Generate();
            Assert.True(cases.Count >= 200, cases.Count.ToString());
            var res = cases.Select(c => Eval(c, c.P)).ToList();
            Assert.True(res.Count(x => x.Visible) > 40 && res.Count(x => !x.Visible) > 40);
            foreach (var reason in new[] { VisibilityReason.Visible, VisibilityReason.Occluded, VisibilityReason.TooSmall, VisibilityReason.TooOblique, VisibilityReason.OutOfFov })
                Assert.Contains(res, x => x.Reason == reason);
            Assert.Contains(res, x => x.PartiallyOccluded);
        }

        [Fact]
        public void CheckedInFile_MatchesCurrentCSharpModel_AndRoundTripsThroughJson()
        {
            var cases = Generate(); string json = ToJson(cases);
            if (Environment.GetEnvironmentVariable("FLEET_WRITE_PARITY") == "1")
            {
                Directory.CreateDirectory(Path.GetDirectoryName(TestEnv.ParityPath)); File.WriteAllText(TestEnv.ParityPath, json);
            }
            Assert.True(File.Exists(TestEnv.ParityPath), "run once with FLEET_WRITE_PARITY=1 to create " + TestEnv.ParityPath);
            var root = (Dictionary<string, object>)MiniJson.Parse(File.ReadAllText(TestEnv.ParityPath));
            var file = (List<object>)root["cases"];
            Assert.Equal(cases.Count, file.Count);
            for (int i = 0; i < cases.Count; i++)
            {
                var exp = (Dictionary<string, object>)((Dictionary<string, object>)file[i])["expected"]; var res = Eval(cases[i], cases[i].P);
                Assert.Equal(res.Reason.ToString(), (string)exp["reason"]); Assert.Equal(res.Visible, (bool)exp["visible"]);
            }
            RoundTrip();
        }

        private static void RoundTrip()
        {
            // the Python side reads exactly this file: re-evaluate from the parsed numbers only (guards serialisation precision)
            var root = (Dictionary<string, object>)MiniJson.Parse(File.ReadAllText(TestEnv.ParityPath)); int n = 0;
            foreach (Dictionary<string, object> c in (List<object>)root["cases"])
            {
                var cam = (Dictionary<string, object>)c["camera"]; var tg = (Dictionary<string, object>)c["tag"]; var pr = (Dictionary<string, object>)c["params"]; var ex = (Dictionary<string, object>)c["expected"];
                Pose Make(Dictionary<string, object> d, string pk) { var p = ((List<object>)d[pk]).Cast<double>().ToArray(); var R = ((List<object>)d["R"]).Cast<double>().ToArray(); return new Pose(new Mat3(R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8]), new Vec3(p[0], p[1], p[2])); }
                var boxes = ((List<object>)c["boxes"]).Cast<Dictionary<string, object>>().Select(b => { var ce = ((List<object>)b["centre"]).Cast<double>().ToArray(); var h = ((List<object>)b["half"]).Cast<double>().ToArray(); return new Obb((string)b["id"], new Vec3(ce[0], ce[1], ce[2]), new Vec3(h[0], h[1], h[2]), (double)b["yaw"]); }).ToList();
                var vp = new VisibilityParams { MinPixels = (double)pr["min_pixels"], MaxViewAngle = (double)pr["max_view_angle"], MinRange = (double)pr["min_range"], NearPlane = (double)pr["near_plane"], OcclusionEps = (double)pr["occlusion_eps"] };
                var K = CameraIntrinsics.FromHfov((int)(double)cam["width"], (int)(double)cam["height"], (double)cam["hfov"]);
                var res = TagVisibility.Evaluate(Make(cam, "position"), K, Make(tg, "centre"), (double)tg["size"], boxes, vp);
                Assert.Equal((string)ex["reason"], res.Reason.ToString()); n++;
            }
            Assert.True(n >= 200);
        }
    }
}
