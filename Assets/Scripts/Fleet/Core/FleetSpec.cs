using System;
using System.Collections.Generic;
using System.IO;
using System.Security.Cryptography;
using System.Text;

namespace Crane.Fleet {
    /// <summary>A pose in a parent frame: position, yaw about z and pitch about y (radians, FLU / ENU).</summary>
    public struct FramePose {
        public double X, Y, Z, Yaw, Pitch;
        public FramePose(double x, double y, double z, double yaw = 0, double pitch = 0) { X = x; Y = y; Z = z; Yaw = yaw; Pitch = pitch; }
        public override string ToString() => $"({X:0.####}, {Y:0.####}, {Z:0.####}, yaw {Yaw:0.####})";
    }

    /// <summary>One loaded spec file: its parsed tree, its text's sha256 and where it came from.</summary>
    public sealed class SpecFile {
        public readonly JsonNode Root;
        public readonly string Sha256;
        public readonly string Source;

        public SpecFile(string text, string source) {
            Root = JsonNode.FromText(text);
            Source = source;
            using (var sha = SHA256.Create()) {
                var h = sha.ComputeHash(Encoding.UTF8.GetBytes(text));
                var b = new StringBuilder();
                foreach (byte x in h) b.Append(x.ToString("x2"));
                Sha256 = b.ToString();
            }
        }

        public static SpecFile Load(string path) => new SpecFile(File.ReadAllText(path), Path.GetFullPath(path));
    }

    /// <summary>The three inputs a fleet run is built from: the shared fleet profile, a shared scenario and the
    /// CRANE-local overlay (values the profile does not define yet). Nothing else carries numbers.</summary>
    public sealed class FleetSpec {
        public const string ProfileFile = "fleet_profile_v0.json";
        public const string OverlayFile = "crane_fleet_sim_v0.json";
        public const string DefaultScenario = "scenarios/warehouse_a_v0.json";

        public readonly SpecFile Profile, Scenario, Overlay;
        public readonly string Directory;

        public FleetSpec(SpecFile profile, SpecFile scenario, SpecFile overlay, string directory) {
            Profile = profile; Scenario = scenario; Overlay = overlay; Directory = directory;
        }

        public static FleetSpec LoadDirectory(string dir, string scenarioRelative = DefaultScenario) =>
            new FleetSpec(SpecFile.Load(Path.Combine(dir, ProfileFile)), SpecFile.Load(Path.Combine(dir, scenarioRelative)),
                SpecFile.Load(Path.Combine(dir, OverlayFile)), Path.GetFullPath(dir));

        public JsonNode P => Profile.Root;
        public JsonNode S => Scenario.Root;
        public JsonNode O => Overlay.Root;

        public double Dt => O["physics.dt"].Double;
        public double Gravity => O["physics.gravity"].Double;

        public List<(string id, string role)> FleetDefault() {
            var r = new List<(string, string)>();
            foreach (var n in P["fleet_default"].Items) r.Add((n["id"].String, n["role"].String));
            return r;
        }

        public IEnumerable<string> Roles() {
            foreach (var kv in P["roles"].Members) yield return kv.Key;
        }

        /// <summary>The floor table entry the scenario's arena names.</summary>
        public FloorProperties Floor(string name = null) {
            name ??= S["arena.floor"].String;
            var f = P["floors." + name];
            return new FloorProperties(name, f["mu_long"].Double, f["lateral_efficiency"].Double, f["slip_sigma_frac"].Double);
        }

        public RoverDescription Describe(string id, string role) => RoverDescription.Resolve(this, id, role);
    }

    public sealed class FloorProperties {
        public readonly string Name;
        public readonly double MuLong, LateralEfficiency, SlipSigmaFrac;
        public FloorProperties(string name, double mu, double eta, double sigma) { Name = name; MuLong = mu; LateralEfficiency = eta; SlipSigmaFrac = sigma; }
    }

    public enum PrimitiveKind { Box, Cylinder, Sphere }

    /// <summary>A primitive part in the chassis frame. Cylinders have their axis along chassis
    /// <see cref="Axis"/> ('x', 'y' or 'z'); Size is (length x, width y, height z) of the bounding box.</summary>
    public sealed class PrimitivePart {
        public string Name;
        public PrimitiveKind Kind;
        public double[] Centre;  // chassis FLU
        public double[] Size;    // FLU extents
        public char Axis = 'z';
        public double Yaw;       // about chassis z
        public double[] Color;
    }

    public sealed class WheelFrame {
        public string Name;           // fl, fr, rl, rr
        public FramePose Axle;        // axle centre, chassis frame
        public double Radius, Width;
    }

    public sealed class SensorFrame {
        public string Id, Sensor;
        public FramePose Pose;
    }

    public sealed class DockingFace {
        public string Name;
        public FramePose Pose;
        public bool Active;
    }

    /// <summary>
    /// Everything a rover is built from, resolved from the spec with each concern in its own field so one
    /// can be replaced (CAD) without touching the others: render geometry, collision geometry, mass
    /// properties, wheel frames, sensor frames, docking-face frames and payload properties.
    /// </summary>
    public sealed class RoverDescription {
        public string Id, Role, Namespace;
        public double[] Color;

        // render: primitives unless a mesh manifest exists (then the manifest replaces them)
        public List<PrimitivePart> RenderPrimitives = new List<PrimitivePart>();
        public string MeshManifestPath;           // asset manifest (crane.fleet_asset_manifest/v0): resolved path, or the expected path when absent
        public bool MeshManifestPresent;
        public AssetManifest Asset;               // parsed without meshes when present (frames, collision, hooks, lift)
        public string SensorFrameSource = "overlay";
        public string CollisionSource = "profile";
        public int InstanceIndex;                 // index among rovers of the same role (cargo_2 = 1)

        // collision
        public List<PrimitivePart> CollisionParts = new List<PrimitivePart>();  // chassis envelope
        public string WheelContact;                                            // "sphere"

        // mass properties
        public double Mass;
        public double[] InertiaDiag;   // about the CoM, chassis axes (FLU)
        public double[] ComOffset;     // chassis frame

        public List<WheelFrame> Wheels = new List<WheelFrame>();
        public List<SensorFrame> Sensors = new List<SensorFrame>();
        public List<DockingFace> Faces = new List<DockingFace>();
        public Dictionary<string, double> Payload = new Dictionary<string, double>();
        public Dictionary<string, string> Provenance = new Dictionary<string, string>();

        public MecanumGeometry Geometry;
        /// <summary>Height of the chassis origin above a level floor when the rover rests on its wheels
        /// (overlay frame_convention.origin: "body_underside" = ground_clearance, "floor" = 0).</summary>
        public double OriginHeight;

        public DockingFace Face(string name) {
            foreach (var f in Faces) if (f.Name == name) return f;
            throw new KeyNotFoundException($"{Id} has no docking face {name}");
        }

        public static RoverDescription Resolve(FleetSpec spec, string id, string role) {
            JsonNode p = spec.P, o = spec.O;
            JsonNode ch = p["chassis"], wh = p["chassis.wheel"], roleNode = p["roles." + role];
            var d = new RoverDescription { Id = id, Role = role, Namespace = "/fleet/" + id };

            double[] lwh = ch["body_size_lwh"].Doubles;
            double clearance = ch["ground_clearance"].Double;
            double r = wh["radius"].Double, w = wh["width"].Double, lx = wh["half_wheelbase_lx"].Double, ly = wh["half_track_ly"].Double;
            string origin = o.TryGet("frame_convention.origin")?.String ?? "floor";
            double g = origin == "body_underside" ? clearance : origin == "floor" ? 0 : throw new FormatException("frame_convention.origin " + origin);
            d.OriginHeight = g;   // generated geometry below is computed as heights above the floor, minus g
            d.Geometry = new MecanumGeometry(r, lx, ly, wh["max_wheel_speed_rad_s"].Double, wh["max_wheel_torque"].Double,
                (int)wh["encoder_counts_per_rev"].Double, ch["max_speed_xy"].Double, ch["max_yaw_rate"].Double);

            // colour: role colour, shaded per instance index of that role in the fleet
            double[] baseColor = o["visual.role_colors." + role].Doubles;
            int index = 0;
            foreach (var (fid, frole) in spec.FleetDefault()) {
                if (fid == id) break;
                if (frole == role) index++;
            }
            double step = o["visual.instance_shade_step"].Double * index;
            d.Color = new[] { Clamp01(baseColor[0] - step), Clamp01(baseColor[1] - step * 0.3), Clamp01(baseColor[2] + step) };

            // mass properties
            double baseMass = ch["base_mass"].Double;
            d.Mass = baseMass + roleNode["added_mass"].Double;
            var options = o.TryGet("role_options." + role);
            if (options != null && options.Has("arm_fitted") && options["arm_fitted"].Bool && roleNode.Has("arm_proxy"))
                d.Mass += roleNode["arm_proxy.mass"].Double;
            double[] inertia = ch["inertia_diag"].Doubles;
            double scale = d.Mass / baseMass;  // derived: the profile's box inertia scaled by total/base mass
            d.InertiaDiag = new[] { inertia[0] * scale, inertia[1] * scale, inertia[2] * scale };
            d.ComOffset = ch["com_offset_xyz"].Doubles;
            d.Provenance["mass"] = roleNode["added_mass"].Provenance;
            d.Provenance["inertia"] = "derived(" + ch["inertia_diag"].Provenance + ", scaled by mass)";

            // wheels (order fl, fr, rl, rr), axle centre at wheel radius above the floor
            d.Wheels.Add(new WheelFrame { Name = "fl", Axle = new FramePose(lx, ly, r - g), Radius = r, Width = w });
            d.Wheels.Add(new WheelFrame { Name = "fr", Axle = new FramePose(lx, -ly, r - g), Radius = r, Width = w });
            d.Wheels.Add(new WheelFrame { Name = "rl", Axle = new FramePose(-lx, ly, r - g), Radius = r, Width = w });
            d.Wheels.Add(new WheelFrame { Name = "rr", Axle = new FramePose(-lx, -ly, r - g), Radius = r, Width = w });
            d.WheelContact = o["physics.wheel_contact"].String;

            // docking faces
            var active = new HashSet<string>();
            foreach (var n in p["docking.active_faces_default"].Items) active.Add(n.String);
            foreach (var kv in p["docking.face_frames"].Members) {
                if (kv.Key == "note") continue;
                double[] v = kv.Value.Doubles;
                d.Faces.Add(new DockingFace { Name = kv.Key, Pose = new FramePose(v[0], v[1], v[2], v[3]), Active = active.Contains(kv.Key) });
            }

            // sensors (overlay mounting poses; stereo split by the profile baseline; "mast_height" etc. read from the role)
            foreach (var m in o["sensor_mounts." + role].Items) {
                var poseItems = m["pose"].Items;
                var v = new double[5];
                for (int i = 0; i < 5; i++)
                    v[i] = poseItems[i].Unwrapped is string key ? roleNode[key].Double - (i == 2 ? g : 0) : poseItems[i].Double;  // role heights are above ground
                string sensor = m["sensor"].String, sid = m["id"].String;
                if (m.Has("pair_baseline")) {
                    double half = p["sensors." + m["pair_baseline"].String].Double / 2;
                    d.Sensors.Add(new SensorFrame { Id = sid + "_left", Sensor = sensor, Pose = new FramePose(v[0], v[1] + half, v[2], v[3], v[4]) });
                    d.Sensors.Add(new SensorFrame { Id = sid + "_right", Sensor = sensor, Pose = new FramePose(v[0], v[1] - half, v[2], v[3], v[4]) });
                }
                else d.Sensors.Add(new SensorFrame { Id = sid, Sensor = sensor, Pose = new FramePose(v[0], v[1], v[2], v[3], v[4]) });
            }

            // payload / role properties: every numeric leaf of the role except mass
            foreach (var kv in roleNode.Members) {
                if (kv.Key == "added_mass") continue;
                object u = kv.Value.Unwrapped;
                if (u is double x) d.Payload[kv.Key] = x;
                else if (u is List<object> list && list.Count == 2 && list[0] is double a0 && list[1] is double a1) {
                    d.Payload[kv.Key + "_0"] = a0; d.Payload[kv.Key + "_1"] = a1;
                }
            }

            // collision: chassis box (wheel spheres are added from the wheel frames by the simulator)
            double bodyZ = clearance + lwh[2] / 2 - g;
            d.CollisionParts.Add(new PrimitivePart { Name = "chassis", Kind = PrimitiveKind.Box, Centre = new[] { 0, 0, bodyZ }, Size = lwh });

            // render primitives (provisional geometry from the profile dimensions)
            double[] wheelColor = o["visual.wheel_color"].Doubles;
            d.RenderPrimitives.Add(new PrimitivePart { Name = "chassis", Kind = PrimitiveKind.Box, Centre = new[] { 0, 0, bodyZ }, Size = lwh, Color = d.Color });
            foreach (var wf in d.Wheels)
                d.RenderPrimitives.Add(new PrimitivePart { Name = "wheel_" + wf.Name, Kind = PrimitiveKind.Cylinder, Axis = 'y',
                    Centre = new[] { wf.Axle.X, wf.Axle.Y, wf.Axle.Z }, Size = new[] { 2 * r, w, 2 * r }, Color = wheelColor });
            double top = clearance + lwh[2] - g;
            if (role == "scout" && roleNode.Has("mast_height")) {
                double mr = o["visual.mast_radius"].Double, mh = roleNode["mast_height"].Double;
                d.RenderPrimitives.Add(new PrimitivePart { Name = "mast", Kind = PrimitiveKind.Cylinder, Axis = 'z',
                    Centre = new[] { 0, 0, (top + mh - g) / 2 }, Size = new[] { 2 * mr, 2 * mr, mh - g - top }, Color = d.Color });
            }
            if (roleNode.Has("deck_size_lw") && roleNode.Has("deck_height")) {
                double[] deck = roleNode["deck_size_lw"].Doubles;
                double t = o["visual.deck_thickness"].Double, dh = roleNode["deck_height"].Double - g;  // deck top above ground
                var part = new PrimitivePart { Name = "deck", Kind = PrimitiveKind.Box, Centre = new[] { 0, 0, dh - t / 2 },
                    Size = new[] { deck[0], deck[1], t }, Color = Lighten(d.Color, 0.35) };
                d.RenderPrimitives.Add(part);
                d.CollisionParts.Add(new PrimitivePart { Name = "deck", Kind = PrimitiveKind.Box, Centre = part.Centre, Size = part.Size });
            }
            if (roleNode.Has("gripper_opening") && roleNode.Has("lift_travel")) {
                double t = o["visual.lift_fork_thickness"].Double, gw = roleNode["gripper_opening"].Double, lt = roleNode["lift_travel"].Double;
                d.RenderPrimitives.Add(new PrimitivePart { Name = "lift_plate", Kind = PrimitiveKind.Box,
                    Centre = new[] { lwh[0] / 2 + t / 2, 0, clearance - g + (lwh[2] + lt) / 2 }, Size = new[] { t, gw, lwh[2] + lt }, Color = Lighten(d.Color, 0.35) });
            }
            double[] faceColor = o["visual.face_marker_color"].Doubles;
            double spacing = p["docking.hook_lateral_spacing"].Double;
            foreach (var f in d.Faces) {
                // render-only marker: a thin plate lying in the face plane, hook spacing wide
                d.RenderPrimitives.Add(new PrimitivePart { Name = "face_" + f.Name, Kind = PrimitiveKind.Box,
                    Centre = new[] { f.Pose.X, f.Pose.Y, f.Pose.Z }, Size = new[] { 0.004, spacing, 0.02 }, Color = f.Active ? faceColor : Lighten(faceColor, 0.5),
                    Yaw = f.Pose.Yaw });
            }

            string manifest = o["visual.mesh_manifest"].String.Replace("{role}", role);
            d.MeshManifestPath = Path.Combine(spec.Directory, manifest);
            d.MeshManifestPresent = File.Exists(d.MeshManifestPath);
            d.InstanceIndex = index;
            if (d.MeshManifestPresent) {
                var a = d.Asset = AssetManifest.Load(d.MeshManifestPath, loadMeshes: false);
                if (a.Role != role) throw new FormatException($"{d.MeshManifestPath}: role {a.Role} != {role}");
                if (Math.Abs(a.GroundClearance - d.OriginHeight) > 1e-9)
                    throw new FormatException($"{d.MeshManifestPath}: base frame ground clearance {a.GroundClearance} != CRANE origin height {d.OriginHeight}");
                // sensors: the manifest declares them (replaces the overlay placeholders)
                d.Sensors.Clear();
                foreach (var (sname, stype, so) in a.Sensors)
                    d.Sensors.Add(new SensorFrame { Id = sname, Sensor = stype, Pose = new FramePose(so.Xyz[0], so.Xyz[1], so.Xyz[2], so.Rpy[2], so.Rpy[1]) });
                d.SensorFrameSource = "asset_manifest";
                // collision: the manifest's base-parented boxes replace the chassis/deck boxes (wheel contact stays the profile spheres)
                var boxes = new List<PrimitivePart>();
                foreach (var sh in a.Collision)
                    if (sh.Type == "box" && sh.Parent == "base_link")
                        boxes.Add(new PrimitivePart { Name = sh.Name, Kind = PrimitiveKind.Box, Centre = sh.Origin.Xyz, Size = sh.Size, Yaw = sh.Origin.Rpy[2] });
                if (boxes.Count > 0) { d.CollisionParts = boxes; d.CollisionSource = "asset_manifest"; }
            }
            return d;
        }

        static double Clamp01(double v) => v < 0 ? 0 : v > 1 ? 1 : v;
        static double[] Lighten(double[] c, double k) => new[] { c[0] + (1 - c[0]) * k, c[1] + (1 - c[1]) * k, c[2] + (1 - c[2]) * k };
    }
}
