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
        public double Scalar => Numbers[0];
    }

    public enum SensorKind { WheelEncoder, Imu, Lidar2D, StereoPair, MonoCamera, RangeTof }

    public sealed class SensorToken
    {
        public SensorKind Kind; public string Facing; public string Raw;
        public static SensorToken Parse(string raw)
        {
            string s = raw.Trim(); var t = new SensorToken { Raw = raw };
            int p = s.IndexOf('(');
            string head = p < 0 ? s : s.Substring(0, p);
            string arg = p < 0 ? null : s.Substring(p + 1).TrimEnd(')');
            switch (head)
            {
                case "wheel_encoder": t.Kind = SensorKind.WheelEncoder; break;
                case "imu": t.Kind = SensorKind.Imu; break;
                case "lidar_2d": t.Kind = SensorKind.Lidar2D; break;
                case "stereo": t.Kind = SensorKind.StereoPair; t.Facing = "front"; break;
                case "mono_camera_ov9281": t.Kind = SensorKind.MonoCamera; t.Facing = arg; break;
                case "range_tof": t.Kind = SensorKind.RangeTof; t.Facing = arg; break;
                default: throw new FormatException($"unknown sensor token '{raw}'");
            }
            if ((t.Kind == SensorKind.MonoCamera || t.Kind == SensorKind.RangeTof) && t.Facing != "front" && t.Facing != "rear")
                throw new FormatException($"sensor token '{raw}' needs (front) or (rear)");
            return t;
        }
    }

    public sealed class RoleSpec { public string Name; public List<SensorToken> Sensors = new List<SensorToken>(); public double MastHeight; }
    public sealed class FleetMember { public string Id; public string Role; }

    public sealed class CameraSpec
    {
        public int Width, Height; public double Hfov, RateHz, Latency; public string Shutter, Encoding;
        public Dictionary<string, string> Provenance = new Dictionary<string, string>();
        public CameraIntrinsics Intrinsics => CameraIntrinsics.FromHfov(Width, Height, Hfov);
    }
    public sealed class OverheadSpec { public double Height, Hfov, RateHz, Latency; public bool Optional; public Dictionary<string, string> Provenance = new Dictionary<string, string>(); }
    public sealed class TofSpec { public double MaxRange, Fov, RateHz; public Dictionary<string, string> Provenance = new Dictionary<string, string>(); }
    public sealed class LidarSpec { public double MaxRange, RateHz; public int Points; public Dictionary<string, string> Provenance = new Dictionary<string, string>(); }
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
        public double StereoBaseline { get; private set; }
        public double FaceTagSize { get; private set; }
        public double TopTagSize { get; private set; }
        public double[] BodySizeLwh { get; private set; }
        public double SlipSigmaFracHardTile { get; private set; }
        public Dictionary<string, string> Provenance = new Dictionary<string, string>();

        private readonly Dictionary<string, Provenanced> _leaves = new Dictionary<string, Provenanced>();
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
            if (!_leaves.TryGetValue(path, out var v)) throw new KeyNotFoundException("profile leaf not found: " + path);
            return v;
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
                double[] nums = null;
                if (d["value"] is double x) nums = new[] { x };
                else if (d["value"] is List<object> lst && lst.All(e => e is double)) nums = lst.Select(e => (double)e).ToArray();
                if (nums == null) { _errors.Add($"{prefix}: value is not numeric"); return; }
                if (nums.Any(v => double.IsNaN(v) || double.IsInfinity(v))) _errors.Add($"{prefix}: non-finite value");
                _leaves[prefix] = new Provenanced { Path = prefix, Numbers = nums, Provenance = prov };
                return;
            }
            foreach (var kv in d)
                if (kv.Value is Dictionary<string, object> sub) Walk(prefix.Length == 0 ? kv.Key : prefix + "." + kv.Key, sub);
        }

        private bool Has(string path) => _leaves.ContainsKey(path);
        private void Positive(string path) { if (Num(path) <= 0) _errors.Add($"{path}: must be > 0"); }

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
