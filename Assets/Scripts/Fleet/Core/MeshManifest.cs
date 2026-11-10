using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;

namespace Crane.Fleet {
    /// <summary>Pose in a parent frame from a manifest: xyz (m) and rpy (rad, R = Rz(yaw) Ry(pitch) Rx(roll), URDF).</summary>
    public struct ManifestPose {
        public double[] Xyz, Rpy;
        public static ManifestPose From(JsonNode n) => new ManifestPose { Xyz = n["xyz"].Doubles, Rpy = n["rpy"].Doubles };
        public static ManifestPose Identity => new ManifestPose { Xyz = new double[3], Rpy = new double[3] };
    }

    /// <summary>
    /// The asset pipeline's per-role manifest (crane.fleet_asset_manifest/v0, Tools/Fleet/assets/ASSET_MANIFEST_SPEC.md):
    /// render parts (OBJ in the part frame, placed by origin under a parent, optional joint), material slots, collision
    /// primitives, docking faces, hooks, lift and gripper. Frames: right-handed x forward, y left, z up, metres, base_link at the
    /// body-underside centre (the overlay frame_convention). Only what CRANE uses is parsed.
    /// </summary>
    public sealed class AssetManifest {
        public sealed class Joint {
            public string Type, ActuatedRef;
            public double[] Axis, Limits;
            public double Authored;
        }

        public sealed class Part {
            public string Name, File, Parent, Kind, Role;
            public ManifestPose Origin;
            public bool Visible = true;
            public Joint Joint;
            public ObjMesh Mesh;
        }

        public sealed class MaterialSlot {
            public string Name, Texture;
            public double[] BaseColor;
            public double Specular, Shininess;
        }

        public sealed class Shape {
            public string Name, Type, Parent;
            public ManifestPose Origin;
            public double[] Size;
            public double Radius, Height;
        }

        public sealed class Face {
            public string Name;
            public ManifestPose Origin;
            public List<string> Hooks = new List<string>();
        }

        public sealed class Hook {
            public string Name, Face, Part;
            public double OpenAngle, ClosedAngle;
        }

        public string Path, Directory, Role, AssetId, Provenance, Schema;
        public double GroundClearance;
        public readonly List<Part> Parts = new List<Part>();
        public readonly Dictionary<string, MaterialSlot> Materials = new Dictionary<string, MaterialSlot>();
        public readonly List<Shape> Collision = new List<Shape>();
        public readonly List<Face> Faces = new List<Face>();
        public readonly List<Hook> Hooks = new List<Hook>();
        public readonly List<(string name, string type, ManifestPose origin)> Sensors = new List<(string, string, ManifestPose)>();
        public string LiftPart;
        public double[] LiftRange;
        public double LiftSpeed = double.NaN, LiftCapacity = double.NaN, LiftDockingValue = double.NaN;
        public readonly List<(string part, double[] axis, double[] travel)> Fingers = new List<(string, double[], double[])>();
        public double[] GripperOpeningRange;
        public double GripperCloseTime = double.NaN;
        public JsonNode Root;

        public Part FindPart(string name) {
            foreach (var p in Parts) if (p.Name == name) return p;
            return null;
        }

        public static AssetManifest Load(string path, bool loadMeshes = true) {
            var root = JsonNode.FromText(File.ReadAllText(path));
            var m = new AssetManifest { Path = System.IO.Path.GetFullPath(path), Root = root };
            m.Directory = System.IO.Path.GetDirectoryName(m.Path);
            m.Schema = root["schema"].String;
            if (!m.Schema.StartsWith("crane.fleet_asset_manifest/")) throw new FormatException($"{path}: schema {m.Schema}");
            m.Role = root["role"].String;
            m.AssetId = root.Has("asset_id") ? root["asset_id"].String : m.Role;
            if (root["units.length"].String != "m") throw new FormatException($"{path}: length unit must be m");
            m.GroundClearance = root["base_frame.ground_clearance"].Double;
            JsonNode render = root["render"];
            m.Provenance = render.Has("provenance") ? render["provenance"].String : null;
            foreach (var kv in render["materials"].Members) {
                var s = new MaterialSlot { Name = kv.Key, BaseColor = kv.Value["base_color"].Doubles };
                if (kv.Value.Has("specular")) s.Specular = kv.Value["specular"].Double;
                if (kv.Value.Has("shininess")) s.Shininess = kv.Value["shininess"].Double;
                if (kv.Value.Has("texture")) s.Texture = System.IO.Path.GetFullPath(System.IO.Path.Combine(m.Directory, kv.Value["texture"].String));
                m.Materials[kv.Key] = s;
            }
            foreach (var pn in render["parts"].Items) {
                var part = new Part {
                    Name = pn["name"].String, File = pn["file"].String, Parent = pn["parent"].String, Origin = ManifestPose.From(pn["origin"]),
                    Kind = pn.Has("kind") ? pn["kind"].String : "solid", Role = pn.Has("role_in_assembly") ? pn["role_in_assembly"].String : "",
                    Visible = !pn.Has("default_visible") || pn["default_visible"].Bool
                };
                if (pn.Has("joint")) {
                    var j = pn["joint"];
                    part.Joint = new Joint { Type = j["type"].String, Axis = j["axis"].Doubles,
                        Limits = j.Has("limits") ? j["limits"].Doubles : null, Authored = j.Has("authored_value") ? j["authored_value"].Double : 0,
                        ActuatedRef = j.Has("actuated_ref") ? j["actuated_ref"].String : null };
                }
                if (loadMeshes) part.Mesh = ObjMesh.Parse(File.ReadAllText(System.IO.Path.Combine(m.Directory, part.File)));
                m.Parts.Add(part);
            }
            if (root.Has("collision"))
                foreach (var sn in root["collision.shapes"].Items) {
                    var s = new Shape { Name = sn["name"].String, Type = sn["type"].String, Parent = sn["parent"].String, Origin = ManifestPose.From(sn["origin"]) };
                    if (sn.Has("size")) s.Size = sn["size"].Doubles;
                    if (sn.Has("radius")) s.Radius = sn["radius"].Double;
                    if (sn.Has("height")) s.Height = sn["height"].Double;
                    m.Collision.Add(s);
                }
            if (root.Has("attachments") && root["attachments"].Has("docking_faces"))
                foreach (var fn in root["attachments.docking_faces"].Items) {
                    var f = new Face { Name = fn["name"].String, Origin = ManifestPose.From(fn["origin"]) };
                    if (fn.Has("hooks")) foreach (var h in fn["hooks"].Items) f.Hooks.Add(h.String);
                    m.Faces.Add(f);
                }
            if (root.Has("sensors"))
                foreach (var sn in root["sensors.frames"].Items)
                    m.Sensors.Add((sn["name"].String, sn["type"].String, ManifestPose.From(sn["origin"])));
            if (root.Has("actuated")) {
                JsonNode a = root["actuated"];
                if (a.Has("hooks"))
                    foreach (var hn in a["hooks"].Items)
                        m.Hooks.Add(new Hook { Name = hn["name"].String, Face = hn["face"].String, Part = hn["part"].String,
                            OpenAngle = hn["open_angle"].Double, ClosedAngle = hn["closed_angle"].Double });
                if (a.Has("lift") && a["lift"].Raw != null) {
                    var l = a["lift"];
                    m.LiftPart = l["part"].String; m.LiftRange = l["range"].Doubles;
                    if (l.Has("speed")) m.LiftSpeed = l["speed"].Double;
                    if (l.Has("capacity")) m.LiftCapacity = l["capacity"].Double;
                    if (l.Has("docking_value")) m.LiftDockingValue = l["docking_value"].Double;
                }
                if (a.Has("gripper") && a["gripper"].Raw != null) {
                    var g = a["gripper"];
                    foreach (var fn in g["fingers"].Items) m.Fingers.Add((fn["part"].String, fn["axis"].Doubles, fn["travel"].Doubles));
                    if (g.Has("opening_range")) m.GripperOpeningRange = g["opening_range"].Doubles;
                    if (g.Has("close_time")) m.GripperCloseTime = g["close_time"].Double;
                }
            }
            return m;
        }
    }

    /// <summary>Wavefront OBJ with v, vt, vn, usemtl: one submesh per material; vertices deduplicated per (v, vt, vn) tuple;
    /// polygons fan-triangulated. Coordinates stay in the file's (right-handed FLU) frame.</summary>
    public sealed class ObjMesh {
        public readonly List<double[]> Positions = new List<double[]>();
        public readonly List<double[]> Normals = new List<double[]>();   // per output vertex (null entries if the file has none)
        public readonly List<double[]> Uvs = new List<double[]>();
        public readonly List<string> SubmeshMaterials = new List<string>();
        public readonly List<List<int>> Submeshes = new List<List<int>>();

        public List<double[]> Vertices => Positions;
        public int TriangleIndexCount { get { int n = 0; foreach (var s in Submeshes) n += s.Count; return n; } }

        public static ObjMesh Parse(string text) {
            var m = new ObjMesh();
            var v = new List<double[]>(); var vt = new List<double[]>(); var vn = new List<double[]>();
            var map = new Dictionary<(int, int, int), int>();
            List<int> cur = null;
            string curMat = "default";
            foreach (string raw in text.Split('\n')) {
                string line = raw.Trim();
                if (line.Length == 0 || line[0] == '#') continue;
                var p = line.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                switch (p[0]) {
                    case "v": v.Add(new[] { D(p[1]), D(p[2]), D(p[3]) }); break;
                    case "vt": vt.Add(new[] { D(p[1]), p.Length > 2 ? D(p[2]) : 0 }); break;
                    case "vn": vn.Add(new[] { D(p[1]), D(p[2]), D(p[3]) }); break;
                    case "usemtl": curMat = p[1]; cur = null; break;
                    case "f": {
                        if (cur == null) {
                            int k = m.SubmeshMaterials.IndexOf(curMat);
                            if (k < 0) { m.SubmeshMaterials.Add(curMat); m.Submeshes.Add(new List<int>()); k = m.Submeshes.Count - 1; }
                            cur = m.Submeshes[k];
                        }
                        var idx = new List<int>();
                        for (int i = 1; i < p.Length; i++) {
                            var t = p[i].Split('/');
                            int a = Idx(t[0], v.Count), b = t.Length > 1 && t[1].Length > 0 ? Idx(t[1], vt.Count) : -1, c = t.Length > 2 && t[2].Length > 0 ? Idx(t[2], vn.Count) : -1;
                            if (!map.TryGetValue((a, b, c), out int o)) {
                                o = m.Positions.Count;
                                m.Positions.Add(v[a]); m.Uvs.Add(b >= 0 ? vt[b] : null); m.Normals.Add(c >= 0 ? vn[c] : null);
                                map[(a, b, c)] = o;
                            }
                            idx.Add(o);
                        }
                        for (int i = 1; i + 1 < idx.Count; i++) { cur.Add(idx[0]); cur.Add(idx[i]); cur.Add(idx[i + 1]); }
                        break;
                    }
                }
            }
            return m;
        }

        static int Idx(string s, int count) {
            int i = int.Parse(s, CultureInfo.InvariantCulture);
            return i < 0 ? count + i : i - 1;
        }

        static double D(string s) => double.Parse(s, NumberStyles.Float, CultureInfo.InvariantCulture);
    }
}
