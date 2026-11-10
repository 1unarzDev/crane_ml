using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace Crane.Fleet.Perception
{
    public sealed class ProfileException : Exception
    {
        public IReadOnlyList<string> Errors { get; }
        public ProfileException(IReadOnlyList<string> errors) : base("fleet profile invalid: " + string.Join("; ", errors)) { Errors = errors; }
    }

    /// <summary>A profile leaf that carries a provenance label (provisional | cad | measured).</summary>
    public sealed class Provenanced
    {
        public string Path { get; internal set; }
        public double[] Numbers { get; internal set; }   // length 1 for scalars
        public string Provenance { get; internal set; }
        /// <summary>The leaf's "source" string (datasheet URL, file, decision), or null when the profile gives none.</summary>
        public string Source { get; internal set; }
        public double Scalar => Numbers[0];
    }

    /// <summary>
    /// A profile leaf whose schema role is descriptive (a part name, a datasheet-style object, an option label). Carried verbatim with its
    /// provenance/source; NEVER converted to numbers. <see cref="Value"/> is a string or a nested Dictionary/List as parsed.
    /// </summary>
    public sealed class DescriptiveLeaf
    {
        public string Path { get; internal set; }
        public object Value { get; internal set; }
        public string Provenance { get; internal set; }
        public string Source { get; internal set; }
        public string Note { get; internal set; }
        /// <summary>The value when it is a plain string, else null.</summary>
        public string Text => Value as string;
    }

    public enum SensorKind { WheelEncoder, Imu, Lidar2D, StereoPair, MonoCamera, RangeTof, RealSenseD435 }

    public sealed class SensorToken
    {
        public SensorKind Kind; public string Facing; public string Raw;
        /// <summary>Free-text qualifier inside the parentheses for kinds that do not use it as a facing (e.g. the lidar model).</summary>
        public string Arg;
        /// <summary>For RealSenseD435: the streams listed in the token (subset of depth, infra1, infra2, color).</summary>
        public List<string> Streams = new List<string>();
        public static readonly string[] D435Streams = { "depth", "infra1", "infra2", "color" };
        public static SensorToken Parse(string raw)
        {
            string s = raw.Trim(); var t = new SensorToken { Raw = raw };
            int p = s.IndexOf('(');
            string head = (p < 0 ? s : s.Substring(0, p)).Trim();
            string arg = p < 0 ? null : s.Substring(p + 1).TrimEnd(')').Trim();
            t.Arg = arg;
            switch (head)
            {
                case "wheel_encoder": t.Kind = SensorKind.WheelEncoder; break;
                case "imu": t.Kind = SensorKind.Imu; break;
                case "lidar_2d": t.Kind = SensorKind.Lidar2D; break;     // v1: "lidar_2d (RPLIDAR C1)": the qualifier is the model name, informational
                case "stereo": t.Kind = SensorKind.StereoPair; t.Facing = "front"; break;
                case "mono_camera_ov9281": t.Kind = SensorKind.MonoCamera; t.Facing = arg; break;
                case "range_tof": t.Kind = SensorKind.RangeTof; t.Facing = arg; break;
                case "realsense_d435":
                    t.Kind = SensorKind.RealSenseD435; t.Facing = "front";
                    if (string.IsNullOrEmpty(arg)) throw new FormatException($"sensor token '{raw}' needs a stream list, e.g. (depth + infra1/infra2 + color)");
                    foreach (var part in arg.Replace("/", "+").Split('+'))
                    {
                        string st = part.Trim();
                        if (Array.IndexOf(D435Streams, st) < 0) throw new FormatException($"sensor token '{raw}': unknown D435 stream '{st}' (allowed: {string.Join(", ", D435Streams)})");
                        if (!t.Streams.Contains(st)) t.Streams.Add(st);
                    }
                    break;
                default: throw new FormatException($"unknown sensor token '{raw}'");
            }
            if (t.Kind == SensorKind.RangeTof && t.Facing != "front" && t.Facing != "rear")
                throw new FormatException($"sensor token '{raw}' needs (front) or (rear)");
            if (t.Kind == SensorKind.MonoCamera && t.Facing != "front" && t.Facing != "rear" && t.Facing != "mast")
                throw new FormatException($"sensor token '{raw}' needs (front), (rear) or (mast)");
            return t;
        }
    }

    public sealed class RoleSpec
    {
        public string Name; public List<SensorToken> Sensors = new List<SensorToken>(); public double MastHeight;
        /// <summary>v1 roles.*.required_sensor_frames: sensor id -> URDF/manifest frame (empty for v0).</summary>
        public Dictionary<string, string> RequiredSensorFrames = new Dictionary<string, string>();
        /// <summary>v1 roles.*.stereo_pair: the two sensor ids that form the stereo pair (empty for v0).</summary>
        public List<string> StereoPair = new List<string>();
    }
    public sealed class FleetMember { public string Id; public string Role; }

    public sealed class CameraSpec
    {
        public int Width, Height; public double Hfov, RateHz, Latency; public string Shutter, Encoding;
        public Dictionary<string, string> Provenance = new Dictionary<string, string>();
        public CameraIntrinsics Intrinsics => CameraIntrinsics.FromHfov(Width, Height, Hfov);
    }
    public sealed class OverheadSpec { public double Height, Hfov, RateHz, Latency; public bool Optional; public Dictionary<string, string> Provenance = new Dictionary<string, string>(); }
    public sealed class TofSpec { public double MaxRange, Fov, RateHz; public Dictionary<string, string> Provenance = new Dictionary<string, string>(); }
    public sealed class LidarSpec
    {
        public double MaxRange, RateHz; public int Points; public Dictionary<string, string> Provenance = new Dictionary<string, string>();
        /// <summary>Part name from sensors.lidar_2d.model (v1: "SLAMTEC RPLIDAR C1"), or null.</summary>
        public string Model;
        /// <summary>Per-field source strings (datasheet URLs), where the profile gives them.</summary>
        public Dictionary<string, string> Sources = new Dictionary<string, string>();
        /// <summary>Angular step of a full 360 degree scan, rad (2 pi / Points; C1: 0.72 deg).</summary>
        public double AngularResolution => 2 * Math.PI / Points;
    }

    /// <summary>One imager of the RealSense D435 (depth, infra1/infra2 or color) as declared by the v1 profile.</summary>
    public class ImagerSpec
    {
        public int Width, Height; public double Hfov, RateHz, Latency; public string Shutter, Encoding;
        public Dictionary<string, string> Provenance = new Dictionary<string, string>();
        public Dictionary<string, string> Sources = new Dictionary<string, string>();
        public CameraIntrinsics Intrinsics => CameraIntrinsics.FromHfov(Width, Height, Hfov);
    }
    public sealed class DepthImagerSpec : ImagerSpec { public double MinRange, MaxRange; }

    /// <summary>
    /// RealSense D435 (sensors.realsense_d435_*): depth + two infrared imagers (global shutter, stereo, <see cref="IrBaseline"/>) + colour.
    /// Datasheets: https://www.intel.com/content/www/us/en/products/sku/128255/intel-realsense-depth-camera-d435/specifications.html and
    /// the D400 launch deck https://www.mouser.com/datasheet/2/612/Intel_3_28_2023_D435if_Launch_Deck-3135151.pdf (the URLs the profile cites).
    /// </summary>
    public sealed class D435Spec
    {
        public DepthImagerSpec Depth; public ImagerSpec Infra, Color; public double IrBaseline; public double[] BodySizeLwh; public double Mass;
        public Dictionary<string, string> Sources = new Dictionary<string, string>();
    }
    public sealed class ImuSpec { public double RateHz, GyroNoiseDensity, GyroBiasInstability; public Dictionary<string, string> Provenance = new Dictionary<string, string>(); }
    public sealed class EncoderSpec { public double RateHz; public int CountsPerRev; public double WheelRadius, HalfWheelbaseLx, HalfTrackLy; public Dictionary<string, string> Provenance = new Dictionary<string, string>(); }
    public sealed class FaceFrame { public string Name; public double X, Y, Z, Yaw; public string Provenance; }

    /// <summary>
    /// Loader/validator for fleet_profile_v0.json. Every {"value":..., "provenance":...} leaf is kept
    /// with its label (see <see cref="Leaves"/>); typed specs expose the same labels per field.
    /// </summary>
    public sealed class FleetProfile
    {
        public string Schema { get; private set; }
        public string ProfileId { get; private set; }
        public IReadOnlyList<string> ProvenanceLevels { get; private set; }
        public IReadOnlyDictionary<string, Provenanced> Leaves => _leaves;
        public IReadOnlyDictionary<string, RoleSpec> Roles => _roles;
        public IReadOnlyList<FleetMember> Fleet => _fleet;
        public IReadOnlyDictionary<string, FaceFrame> FaceFrames => _faces;
        public IReadOnlyList<string> ActiveFaces { get; private set; }
        public CameraSpec Camera { get; private set; }
        public OverheadSpec Overhead { get; private set; }
        public TofSpec Tof { get; private set; }
        public LidarSpec Lidar { get; private set; }
        public ImuSpec Imu { get; private set; }
        public EncoderSpec Encoder { get; private set; }
        /// <summary>RealSense D435 spec (v1 profiles); null when the profile has no sensors.realsense_d435_* block (v0).</summary>
        public D435Spec D435 { get; private set; }
        /// <summary>Descriptive (non-numeric) leaves by path: part names, option labels, datasheet-style objects. Never coerced to numbers.</summary>
        public IReadOnlyDictionary<string, DescriptiveLeaf> Metadata => _meta;
        public double StereoBaseline { get; private set; }
        public double FaceTagSize { get; private set; }
        public double TopTagSize { get; private set; }
        public double[] BodySizeLwh { get; private set; }
        public double SlipSigmaFracHardTile { get; private set; }
        public Dictionary<string, string> Provenance = new Dictionary<string, string>();

        private readonly Dictionary<string, Provenanced> _leaves = new Dictionary<string, Provenanced>();
        private readonly Dictionary<string, DescriptiveLeaf> _meta = new Dictionary<string, DescriptiveLeaf>();
        private readonly Dictionary<string, RoleSpec> _roles = new Dictionary<string, RoleSpec>();
        private readonly List<FleetMember> _fleet = new List<FleetMember>();
        private readonly Dictionary<string, FaceFrame> _faces = new Dictionary<string, FaceFrame>();
        private readonly List<string> _errors = new List<string>();

        public static FleetProfile Load(string path) => Parse(File.ReadAllText(path));

        public static FleetProfile Parse(string json)
        {
            var p = new FleetProfile();
            object root;
            try { root = MiniJson.Parse(json); } catch (Exception e) { throw new ProfileException(new[] { "JSON: " + e.Message }); }
            var d = root as Dictionary<string, object>;
            if (d == null) throw new ProfileException(new[] { "root is not an object" });
            p.Build(d);
            if (p._errors.Count > 0) throw new ProfileException(p._errors);
            return p;
        }

        public double Num(string path) => Get(path).Scalar;
        public double[] Vec(string path) => Get(path).Numbers;
        public string Prov(string path) => Get(path).Provenance;
        public Provenanced Get(string path)
        {
            if (!_leaves.TryGetValue(path, out var v))
            {
                if (_meta.ContainsKey(path)) throw new InvalidOperationException("profile leaf is descriptive metadata, not numeric (use Meta/Text): " + path);
                throw new KeyNotFoundException("profile leaf not found: " + path);
            }
            return v;
        }
        public DescriptiveLeaf Meta(string path)
        {
            if (!_meta.TryGetValue(path, out var v)) throw new KeyNotFoundException("profile descriptive leaf not found: " + path);
            return v;
        }
        public string Text(string path) => Meta(path).Text ?? throw new InvalidCastException("descriptive leaf is not a plain string: " + path);

        /// <summary>
        /// Schema: which {"value","provenance"} leaves are DESCRIPTIVE. Every other leaf is numeric (a number or an array of numbers) and a
        /// non-numeric value there is an error naming the path. '*' matches one path segment. Add a pattern here when a profile version
        /// introduces a new descriptive field; an unlisted non-numeric leaf fails loudly rather than being guessed at.
        /// </summary>
        public static readonly IReadOnlyList<string> DescriptivePathPatterns = new[]
        {
            "chassis.wheel.motor",                  // part name + datasheet URL
            "docking.magnet",                       // object: diameter/thickness/face offsets/polarity text
            "docking.partner_acquisition",          // option label (e.g. bevel_tags_staged)
            "roles.*.compute",                      // single-board computer description
            "roles.*.mcu",                          // microcontroller description
            "roles.*.lifter_option",                // option label (a|b)
            "roles.*.arm_proxy.servo",              // servo part description
        };
        public static bool IsDescriptivePath(string path)
        {
            var seg = path.Split('.');
            foreach (var pat in DescriptivePathPatterns)
            {
                var ps = pat.Split('.');
                if (ps.Length != seg.Length) continue;
                bool ok = true;
                for (int i = 0; i < ps.Length && ok; i++) ok = ps[i] == "*" || ps[i] == seg[i];
                if (ok) return true;
            }
            return false;
        }

        private void Build(Dictionary<string, object> d)
        {
            Schema = d.TryGetValue("schema", out var s) ? s as string : null;
            if (Schema == null || !Schema.StartsWith("crane.fleet_profile/")) _errors.Add("schema must start with crane.fleet_profile/");
            ProfileId = d.TryGetValue("profile_id", out var pid) ? pid as string : null;
            var lv = new List<string>();
            if (d.TryGetValue("provenance_levels", out var lo) && lo is List<object> ll) foreach (var o in ll) lv.Add(o as string);
            if (lv.Count == 0) _errors.Add("provenance_levels missing");
            ProvenanceLevels = lv;
            Walk("", d);
            if (_errors.Count > 0) return;   // typed reads below assume leaves exist
            try { BuildTyped(d); } catch (Exception e) when (e is KeyNotFoundException || e is FormatException || e is InvalidCastException)
            { _errors.Add("typed spec: " + e.Message); }
        }

        private void Walk(string prefix, Dictionary<string, object> d)
        {
            if (d.ContainsKey("value"))
            {
                string prov = d.TryGetValue("provenance", out var po) ? po as string : null;
                if (prov == null) _errors.Add($"{prefix}: missing provenance");
                else if (!ProvenanceLevels.Contains(prov)) _errors.Add($"{prefix}: provenance '{prov}' not in {string.Join("|", ProvenanceLevels)}");
                if (IsDescriptivePath(prefix))
                {
                    _meta[prefix] = new DescriptiveLeaf
                    {
                        Path = prefix, Value = d["value"], Provenance = prov,
                        Source = d.TryGetValue("source", out var so) ? so as string : null, Note = d.TryGetValue("note", out var no) ? no as string : null
                    };
                    return;
                }
                double[] nums = null;
                if (d["value"] is double x) nums = new[] { x };
                else if (d["value"] is List<object> lst && lst.Count > 0)
                {
                    for (int i = 0; i < lst.Count; i++)
                        if (!(lst[i] is double)) { _errors.Add($"{prefix}: value[{i}] is not numeric (schema declares a numeric field here)"); return; }
                    nums = lst.Select(e => (double)e).ToArray();
                }
                if (nums == null) { _errors.Add($"{prefix}: value is not numeric (schema declares a numeric field here; got {Describe(d["value"])})"); return; }
                if (nums.Any(v => double.IsNaN(v) || double.IsInfinity(v))) _errors.Add($"{prefix}: non-finite value");
                _leaves[prefix] = new Provenanced { Path = prefix, Numbers = nums, Provenance = prov, Source = d.TryGetValue("source", out var src) ? src as string : null };
                return;
            }
            foreach (var kv in d)
                if (kv.Value is Dictionary<string, object> sub) Walk(prefix.Length == 0 ? kv.Key : prefix + "." + kv.Key, sub);
        }

        private static string Describe(object o) => o == null ? "null" : o is string str ? "string \"" + (str.Length > 40 ? str.Substring(0, 40) + "..." : str) + "\"" : o.GetType().Name;

        private bool Has(string path) => _leaves.ContainsKey(path);
        private void Positive(string path) { if (Num(path) <= 0) _errors.Add($"{path}: must be > 0"); }

        private ImagerSpec ReadImager(ImagerSpec im, string blockPath, Dictionary<string, object> block)
        {
            string b = blockPath + ".";
            var res = Vec(b + "resolution");
            if (res.Length != 2 || res[0] < 16 || res[1] < 16) _errors.Add(b + "resolution invalid");
            else { im.Width = (int)res[0]; im.Height = (int)res[1]; }
            im.Hfov = Num(b + "hfov"); im.RateHz = Num(b + "rate_hz");
            im.Latency = Has(b + "latency") ? Num(b + "latency") : 0;
            im.Shutter = block.TryGetValue("shutter", out var sh) ? sh as string : null;
            im.Encoding = block.TryGetValue("encoding", out var en) ? en as string : null;
            foreach (var f in new[] { "resolution", "hfov", "rate_hz", "latency" })
                if (Has(b + f)) { im.Provenance[f] = Prov(b + f); var src = Get(b + f).Source; if (src != null) im.Sources[f] = src; }
            if (im.Hfov <= 0 || im.Hfov >= Math.PI) _errors.Add(b + "hfov must be in (0, pi)");
            if (im.RateHz <= 0) _errors.Add(b + "rate_hz must be > 0");
            if (im.Latency < 0) _errors.Add(b + "latency must be >= 0");
            return im;
        }

        private D435Spec BuildD435(Dictionary<string, object> sens)
        {
            foreach (var k in new[] { "realsense_d435_depth", "realsense_d435_infra", "realsense_d435_color", "realsense_d435_body" })
                if (!sens.ContainsKey(k) || !(sens[k] is Dictionary<string, object>)) { _errors.Add("sensors." + k + ": missing (a D435 needs depth, infra, color and body blocks)"); return null; }
            var dd = (Dictionary<string, object>)sens["realsense_d435_depth"];
            var depth = (DepthImagerSpec)ReadImager(new DepthImagerSpec(), "sensors.realsense_d435_depth", dd);
            var rg = Vec("sensors.realsense_d435_depth.range");
            if (rg.Length != 2 || rg[0] <= 0 || rg[1] <= rg[0]) _errors.Add("sensors.realsense_d435_depth.range must be [min, max] with 0 < min < max");
            else { depth.MinRange = rg[0]; depth.MaxRange = rg[1]; }
            var infra = ReadImager(new ImagerSpec(), "sensors.realsense_d435_infra", (Dictionary<string, object>)sens["realsense_d435_infra"]);
            var color = ReadImager(new ImagerSpec(), "sensors.realsense_d435_color", (Dictionary<string, object>)sens["realsense_d435_color"]);
            var spec = new D435Spec { Depth = depth, Infra = infra, Color = color, IrBaseline = Num("sensors.realsense_d435_body.ir_baseline") };
            Positive("sensors.realsense_d435_body.ir_baseline");
            if (Has("sensors.realsense_d435_body.size_lwh")) spec.BodySizeLwh = Vec("sensors.realsense_d435_body.size_lwh");
            if (Has("sensors.realsense_d435_body.mass")) spec.Mass = Num("sensors.realsense_d435_body.mass");
            foreach (var k in new[] { "ir_baseline", "size_lwh", "mass" })
                if (Has("sensors.realsense_d435_body." + k)) { var src = Get("sensors.realsense_d435_body." + k).Source; if (src != null) spec.Sources[k] = src; }
            return spec;
        }

        private void BuildTyped(Dictionary<string, object> d)
        {
            var sens = (Dictionary<string, object>)d["sensors"];
            string cam = "sensors.mono_camera_ov9281.";
            var cd = (Dictionary<string, object>)sens["mono_camera_ov9281"];
            var res = Vec(cam + "resolution");
            Camera = new CameraSpec
            {
                Width = (int)res[0], Height = (int)res[1], Hfov = Num(cam + "hfov"), RateHz = Num(cam + "rate_hz"), Latency = Num(cam + "latency"),
                Shutter = cd["shutter"] as string, Encoding = cd["encoding"] as string
            };
            foreach (var f in new[] { "resolution", "hfov", "rate_hz", "latency" }) Camera.Provenance[f] = Prov(cam + f);
            if (res.Length != 2 || res[0] < 16 || res[1] < 16) _errors.Add(cam + "resolution invalid");
            if (Camera.Hfov <= 0 || Camera.Hfov >= Math.PI) _errors.Add(cam + "hfov must be in (0, pi)");
            if (Camera.RateHz <= 0) _errors.Add(cam + "rate_hz must be > 0");
            if (Camera.Latency < 0) _errors.Add(cam + "latency must be >= 0");

            string oh = "sensors.overhead_camera.";
            Overhead = new OverheadSpec { Height = Num(oh + "height"), Hfov = Num(oh + "hfov"), RateHz = Num(oh + "rate_hz"), Latency = Num(oh + "latency"), Optional = (bool)((Dictionary<string, object>)sens["overhead_camera"])["optional"] };
            foreach (var f in new[] { "height", "hfov", "rate_hz", "latency" }) Overhead.Provenance[f] = Prov(oh + f);
            if (Overhead.Hfov <= 0 || Overhead.Hfov >= Math.PI) _errors.Add(oh + "hfov must be in (0, pi)");

            Tof = new TofSpec { MaxRange = Num("sensors.range_tof.max_range"), Fov = Num("sensors.range_tof.fov"), RateHz = Num("sensors.range_tof.rate_hz") };
            foreach (var f in new[] { "max_range", "fov", "rate_hz" }) Tof.Provenance[f] = Prov("sensors.range_tof." + f);
            Positive("sensors.range_tof.max_range"); Positive("sensors.range_tof.fov");

            Lidar = new LidarSpec { MaxRange = Num("sensors.lidar_2d.max_range"), RateHz = Num("sensors.lidar_2d.rate_hz"), Points = (int)Num("sensors.lidar_2d.points_per_scan") };
            foreach (var f in new[] { "max_range", "rate_hz", "points_per_scan" }) Lidar.Provenance[f] = Prov("sensors.lidar_2d." + f);
            if (Lidar.Points < 8) _errors.Add("lidar points_per_scan too small");
            Lidar.Model = ((Dictionary<string, object>)sens["lidar_2d"]).TryGetValue("model", out var lm) ? lm as string : null;
            foreach (var f in new[] { "max_range", "rate_hz", "points_per_scan" }) { var src = Get("sensors.lidar_2d." + f).Source; if (src != null) Lidar.Sources[f] = src; }
            Positive("sensors.lidar_2d.max_range"); Positive("sensors.lidar_2d.rate_hz");

            Imu = new ImuSpec { RateHz = Num("sensors.imu.rate_hz"), GyroNoiseDensity = Num("sensors.imu.gyro_noise_density"), GyroBiasInstability = Num("sensors.imu.gyro_bias_instability") };
            foreach (var f in new[] { "rate_hz", "gyro_noise_density", "gyro_bias_instability" }) Imu.Provenance[f] = Prov("sensors.imu." + f);
            Positive("sensors.imu.rate_hz");

            string w = "chassis.wheel.";
            Encoder = new EncoderSpec
            {
                RateHz = Num("sensors.wheel_encoder.rate_hz"), CountsPerRev = (int)Num(w + "encoder_counts_per_rev"), WheelRadius = Num(w + "radius"),
                HalfWheelbaseLx = Num(w + "half_wheelbase_lx"), HalfTrackLy = Num(w + "half_track_ly")
            };
            Encoder.Provenance["rate_hz"] = Prov("sensors.wheel_encoder.rate_hz");
            Encoder.Provenance["counts_per_rev"] = Prov(w + "encoder_counts_per_rev");
            Positive(w + "encoder_counts_per_rev");

            StereoBaseline = Num("sensors.stereo_baseline");
            FaceTagSize = Num("sensors.apriltag.face_tag_size"); TopTagSize = Num("sensors.apriltag.top_tag_size");
            Positive("sensors.apriltag.face_tag_size"); Positive("sensors.apriltag.top_tag_size");
            BodySizeLwh = Vec("chassis.body_size_lwh");
            SlipSigmaFracHardTile = Num("floors.hard_tile.slip_sigma_frac");
            foreach (var k in new[] { "sensors.stereo_baseline", "sensors.apriltag.face_tag_size", "sensors.apriltag.top_tag_size", "chassis.body_size_lwh", "floors.hard_tile.slip_sigma_frac" })
                Provenance[k] = Prov(k);

            if (sens.ContainsKey("realsense_d435_depth") || sens.ContainsKey("realsense_d435_infra")) D435 = BuildD435(sens);
            if (D435 != null && Math.Abs(D435.IrBaseline - StereoBaseline) > 1e-9)
                _errors.Add($"sensors.realsense_d435_body.ir_baseline ({D435.IrBaseline}) must equal sensors.stereo_baseline ({StereoBaseline}) when the stereo pair is the D435 infra imagers");

            var ff = (Dictionary<string, object>)((Dictionary<string, object>)((Dictionary<string, object>)d["docking"])["face_frames"]);
            foreach (var kv in ff)
                if (kv.Value is Dictionary<string, object>)
                {
                    var v = Vec("docking.face_frames." + kv.Key);
                    if (v.Length != 4) { _errors.Add($"face frame {kv.Key} needs 4 values"); continue; }
                    _faces[kv.Key] = new FaceFrame { Name = kv.Key, X = v[0], Y = v[1], Z = v[2], Yaw = v[3], Provenance = Prov("docking.face_frames." + kv.Key) };
                }
            var act = (List<object>)((Dictionary<string, object>)d["docking"])["active_faces_default"];
            ActiveFaces = act.Select(o => (string)o).ToList();
            foreach (var a in ActiveFaces) if (!_faces.ContainsKey(a)) _errors.Add("active face '" + a + "' has no frame");

            var roles = (Dictionary<string, object>)d["roles"];
            foreach (var kv in roles)
            {
                var rd = (Dictionary<string, object>)kv.Value;
                var spec = new RoleSpec { Name = kv.Key };
                foreach (var o in (List<object>)rd["sensors"])
                {
                    try { spec.Sensors.Add(SensorToken.Parse((string)o)); } catch (FormatException e) { _errors.Add($"role {kv.Key}: {e.Message}"); }
                }
                if (Has($"roles.{kv.Key}.mast_height")) spec.MastHeight = Num($"roles.{kv.Key}.mast_height");
                if (rd.TryGetValue("required_sensor_frames", out var rsf) && rsf is Dictionary<string, object> rsfd)
                    foreach (var f in rsfd)
                    {
                        if (f.Value is string fv) spec.RequiredSensorFrames[f.Key] = fv;
                        else _errors.Add($"roles.{kv.Key}.required_sensor_frames.{f.Key}: must be a string frame name");
                    }
                if (rd.TryGetValue("stereo_pair", out var sp) && sp is List<object> spl)
                {
                    foreach (var o in spl) if (o is string so) spec.StereoPair.Add(so); else _errors.Add($"roles.{kv.Key}.stereo_pair: entries must be strings");
                    if (spec.StereoPair.Count != 2) _errors.Add($"roles.{kv.Key}.stereo_pair needs exactly 2 sensor ids");
                    foreach (var id in spec.StereoPair) if (!spec.RequiredSensorFrames.ContainsKey(id)) _errors.Add($"roles.{kv.Key}.stereo_pair: '{id}' is not in required_sensor_frames");
                }
                foreach (var t in spec.Sensors)
                    if (t.Kind == SensorKind.RealSenseD435 && D435 == null) _errors.Add($"role {kv.Key}: sensor '{t.Raw}' needs sensors.realsense_d435_depth/_infra/_color in the profile");
                _roles[kv.Key] = spec;
            }
            foreach (var o in (List<object>)d["fleet_default"])
            {
                var m = (Dictionary<string, object>)o;
                var fm = new FleetMember { Id = (string)m["id"], Role = (string)m["role"] };
                if (!_roles.ContainsKey(fm.Role)) _errors.Add($"fleet member {fm.Id}: unknown role {fm.Role}");
                if (_fleet.Any(x => x.Id == fm.Id)) _errors.Add($"duplicate rover id {fm.Id}");
                _fleet.Add(fm);
            }
        }
    }
}
