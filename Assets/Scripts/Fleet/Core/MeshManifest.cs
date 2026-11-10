using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;

namespace Crane.Fleet {
    /// <summary>
    /// The CAD/mesh-generator hook. When <c>meshes/&lt;role&gt;/manifest.json</c> exists next to the profile, its
    /// render meshes replace the role's primitives (and its collision meshes, if given, replace the chassis box);
    /// mass properties, wheel, sensor and docking frames still come from the profile. Expected manifest:
    /// <code>
    /// {"schema": "crane.fleet_mesh_manifest/v0", "role": "scout", "units": "m", "frame": "base_link",
    ///  "render":    [{"obj": "body.obj", "color": [r,g,b]}, ...],
    ///  "collision": [{"obj": "hull_convex.obj"}, ...]}            // optional, each convex
    /// </code>
    /// OBJ vertices are in the chassis frame (FLU, metres, origin = base_link at the centre of the body underside,
    /// overlay frame_convention). The richer crane.fleet_asset_manifest/v0 (Tools/Fleet/assets) needs an adapter (parts tree, origins).
    /// </summary>
    public sealed class MeshManifest {
        public sealed class Entry {
            public string Obj;
            public double[] Color;
            public ObjMesh Mesh;
        }

        public string Role, Frame, Path;
        public readonly List<Entry> Render = new List<Entry>();
        public readonly List<Entry> Collision = new List<Entry>();

        public static MeshManifest Load(string path) {
            var root = JsonNode.FromText(File.ReadAllText(path));
            string dir = System.IO.Path.GetDirectoryName(path) ?? ".";
            var m = new MeshManifest { Role = root["role"].String, Frame = root.Has("frame") ? root["frame"].String : null, Path = path };
            if (root.Has("units") && root["units"].String != "m") throw new FormatException($"{path}: units must be m");
            foreach (var (key, list) in new[] { ("render", m.Render), ("collision", m.Collision) }) {
                if (!root.Has(key)) continue;
                foreach (var e in root[key].Items) {
                    string obj = e["obj"].String;
                    list.Add(new Entry { Obj = obj, Color = e.Has("color") ? e["color"].Doubles : null,
                        Mesh = ObjMesh.Parse(File.ReadAllText(System.IO.Path.Combine(dir, obj))) });
                }
            }
            if (m.Render.Count == 0) throw new FormatException($"{path}: no render meshes");
            return m;
        }
    }

    /// <summary>Minimal Wavefront OBJ: v and f (polygons fan-triangulated; v/vt/vn indices accepted, only v used).</summary>
    public sealed class ObjMesh {
        public readonly List<double[]> Vertices = new List<double[]>();
        public readonly List<int> Triangles = new List<int>();

        public static ObjMesh Parse(string text) {
            var m = new ObjMesh();
            foreach (string raw in text.Split('\n')) {
                string line = raw.Trim();
                if (line.StartsWith("v ")) {
                    var p = line.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
                    m.Vertices.Add(new[] { D(p[1]), D(p[2]), D(p[3]) });
                }
                else if (line.StartsWith("f ")) {
                    var p = line.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
                    var idx = new List<int>();
                    for (int i = 1; i < p.Length; i++) {
                        int v = int.Parse(p[i].Split('/')[0], CultureInfo.InvariantCulture);
                        idx.Add(v < 0 ? m.Vertices.Count + v : v - 1);
                    }
                    for (int i = 1; i + 1 < idx.Count; i++) { m.Triangles.Add(idx[0]); m.Triangles.Add(idx[i]); m.Triangles.Add(idx[i + 1]); }
                }
            }
            return m;
        }

        static double D(string s) => double.Parse(s, NumberStyles.Float, CultureInfo.InvariantCulture);
    }
}
