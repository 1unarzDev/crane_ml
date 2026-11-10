using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

namespace Crane.Fleet.Sim {
    /// <summary>Procedural geometry helpers: coloured primitives that are render-only or collision-only.</summary>
    public static class FleetGeometry {
        static Material s_Template;

        /// <summary>A coloured copy of the pipeline's default Lit material (works in editor and player, no Shader.Find).</summary>
        public static Material Material(Color c) {
            if (s_Template == null) {
                var probe = GameObject.CreatePrimitive(PrimitiveType.Cube);
                s_Template = probe.GetComponent<MeshRenderer>().sharedMaterial;
                UnityEngine.Object.DestroyImmediate(probe);
            }
            var m = new Material(s_Template) { name = "fleet " + ColorUtility.ToHtmlStringRGB(c) };
            if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", c);
            if (m.HasProperty("_Color")) m.SetColor("_Color", c);
            return m;
        }

        public static GameObject Visual(PrimitiveType type, string name, Transform parent, Vector3 localPos, Quaternion localRot, Vector3 scale, Color c) {
            var go = GameObject.CreatePrimitive(type);
            go.name = name;
            UnityEngine.Object.DestroyImmediate(go.GetComponent<Collider>());
            go.transform.SetParent(parent, false);
            go.transform.localPosition = localPos;
            go.transform.localRotation = localRot;
            go.transform.localScale = scale;
            go.GetComponent<MeshRenderer>().sharedMaterial = Material(c);
            return go;
        }

        /// <summary>A static box with both a collider and a visual (walls, floor, obstacles).</summary>
        public static GameObject SolidBox(string name, Transform parent, Vector3 centre, Vector3 size, Color c, PhysicsMaterial material) {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.name = name;
            go.transform.SetParent(parent, false);
            go.transform.localPosition = centre;
            go.transform.localScale = size;
            go.GetComponent<MeshRenderer>().sharedMaterial = Material(c);
            var col = go.GetComponent<BoxCollider>();
            col.sharedMaterial = material;
            col.providesContacts = true;
            return go;
        }

        /// <summary>A primitive part (chassis frame) as a render object under <paramref name="parent"/>.</summary>
        public static GameObject Part(PrimitivePart p, Transform parent) {
            Vector3 pos = FleetFrames.Local(p.Centre);
            Quaternion yaw = FleetFrames.LocalYawPitch(p.Yaw, 0);
            Color c = FleetFrames.Rgb(p.Color);
            switch (p.Kind) {
                case PrimitiveKind.Cylinder: {
                    // Unity cylinder: height 2 along its y, diameter 1
                    double len = p.Axis == 'x' ? p.Size[0] : p.Axis == 'y' ? p.Size[1] : p.Size[2];
                    double dia = p.Axis == 'z' ? p.Size[0] : p.Size[2];
                    Quaternion axis = p.Axis == 'x' ? Quaternion.Euler(90, 0, 0) : p.Axis == 'y' ? Quaternion.Euler(0, 0, 90) : Quaternion.identity;
                    return Visual(PrimitiveType.Cylinder, p.Name, parent, pos, yaw * axis, new Vector3((float)dia, (float)(len / 2), (float)dia), c);
                }
                case PrimitiveKind.Sphere:
                    return Visual(PrimitiveType.Sphere, p.Name, parent, pos, yaw, Vector3.one * (float)p.Size[0], c);
                default:
                    return Visual(PrimitiveType.Cube, p.Name, parent, pos, yaw, FleetFrames.LocalScale(p.Size[0], p.Size[1], p.Size[2]), c);
            }
        }

        public static Mesh ToMesh(ObjMesh obj, string name) {
            var v = new Vector3[obj.Vertices.Count];
            for (int i = 0; i < v.Length; i++) v[i] = FleetFrames.Local(obj.Vertices[i]);
            // FLU -> Unity is a reflection: flip winding so faces stay outward
            var t = obj.Triangles.ToArray();
            for (int i = 0; i + 2 < t.Length; i += 3) { int a = t[i + 1]; t[i + 1] = t[i + 2]; t[i + 2] = a; }
            var m = new Mesh { name = name, indexFormat = v.Length > 65000 ? IndexFormat.UInt32 : IndexFormat.UInt16 };
            m.vertices = v;
            m.triangles = t;
            m.RecalculateNormals();
            m.RecalculateBounds();
            return m;
        }
    }

    /// <summary>
    /// The warehouse built from the scenario JSON at runtime: floor, outer walls, walls with doorways, box
    /// obstacles, coloured zone patches (render only), overhead camera frames, a sun and a fixed exposure.
    /// </summary>
    public sealed class FleetWorld {
        public readonly FleetSpec Spec;
        public readonly GameObject Root;
        public readonly double DoorwayWidth;
        public readonly PhysicsMaterial FloorMaterial, WallMaterial;
        public readonly Dictionary<string, Transform> OverheadCameras = new Dictionary<string, Transform>();

        public FleetWorld(FleetSpec spec, double? doorwayWidth = null, bool scenario = true) {
            Spec = spec;
            JsonNode s = spec.S, o = spec.O;
            Root = new GameObject(scenario ? "fleet world " + s["scenario_id"].String : "fleet flat floor");
            double bodyFriction = o["physics.body_friction"].Double;
            FloorMaterial = new PhysicsMaterial("fleet floor") { dynamicFriction = (float)bodyFriction, staticFriction = (float)bodyFriction };
            WallMaterial = FloorMaterial;

            double[] size = s["arena.size_xy"].Doubles;
            double margin = o["scene.floor_margin"].Double, ft = o["scene.floor_thickness"].Double;
            double wallH = s["arena.wall_height"].Double, wt = o["scene.outer_wall_thickness"].Double;
            Color floorC = FleetFrames.Rgb(o["visual.floor_color"].Doubles), wallC = FleetFrames.Rgb(o["visual.wall_color"].Doubles);
            var t = Root.transform;
            FleetGeometry.SolidBox("floor", t, FleetFrames.World(size[0] / 2, size[1] / 2, -ft / 2),
                FleetFrames.LocalScale(size[0] + 2 * margin, size[1] + 2 * margin, ft), floorC, FloorMaterial);
            if (!scenario) { DoorwayWidth = double.NaN; Light(); return; }

            // outer walls, outside the arena rectangle
            Box("wall_south", size[0] / 2, -wt / 2, wallH / 2, size[0] + 2 * wt, wt, wallH, wallC);
            Box("wall_north", size[0] / 2, size[1] + wt / 2, wallH / 2, size[0] + 2 * wt, wt, wallH, wallC);
            Box("wall_west", -wt / 2, size[1] / 2, wallH / 2, wt, size[1], wallH, wallC);
            Box("wall_east", size[0] + wt / 2, size[1] / 2, wallH / 2, wt, size[1], wallH, wallC);

            // interior walls (axis-aligned), cut by their doorway
            JsonNode door = s.TryGet("doorway");
            DoorwayWidth = doorwayWidth ?? (door != null ? door["width.default"].Double : double.NaN);
            foreach (var w in s["walls"].Items) {
                string id = w["id"].String;
                double[] a = w["from"].Doubles, b = w["to"].Doubles;
                double th = w["thickness"].Double;
                bool vertical = Math.Abs(a[0] - b[0]) < 1e-9;
                if (!vertical && Math.Abs(a[1] - b[1]) > 1e-9) throw new NotSupportedException($"wall {id} is not axis-aligned");
                double lo = vertical ? Math.Min(a[1], b[1]) : Math.Min(a[0], b[0]), hi = vertical ? Math.Max(a[1], b[1]) : Math.Max(a[0], b[0]);
                double fixedCoord = vertical ? a[0] : a[1];
                var segments = new List<(double, double, double, double)>();  // along lo, hi, z lo, z hi
                if (door != null && door["in_wall"].String == id) {
                    double c = door[vertical ? "centre_y" : "centre_x"].Double, dh = door["height"].Double;
                    segments.Add((lo, c - DoorwayWidth / 2, 0, wallH));
                    segments.Add((c + DoorwayWidth / 2, hi, 0, wallH));
                    segments.Add((c - DoorwayWidth / 2, c + DoorwayWidth / 2, dh, wallH));  // lintel
                }
                else segments.Add((lo, hi, 0, wallH));
                int k = 0;
                foreach (var (s0, s1, z0, z1) in segments) {
                    if (s1 - s0 <= 1e-6) continue;
                    double mid = (s0 + s1) / 2, len = s1 - s0, zc = (z0 + z1) / 2, zh = z1 - z0;
                    if (vertical) Box($"{id}_{k++}", fixedCoord, mid, zc, th, len, zh, wallC);
                    else Box($"{id}_{k++}", mid, fixedCoord, zc, len, th, zh, wallC);
                }
            }

            Color obstacleC = FleetFrames.Rgb(o["visual.obstacle_color"].Doubles);
            foreach (var ob in s["obstacles"].Items) {
                if (ob["type"].String != "box") throw new NotSupportedException("obstacle type " + ob["type"].String);
                double[] c = ob["centre"].Doubles, sz = ob["size"].Doubles;
                Box(ob["id"].String, c[0], c[1], c[2], sz[0], sz[1], sz[2], obstacleC);
            }

            double patch = o["scene.zone_patch_thickness"].Double;
            var zoneColors = o["visual.zone_colors"];
            foreach (var kv in s["zones"].Members) {
                double[] r = kv.Value["rect"].Doubles;  // x0, y0, x1, y1
                double[] col = zoneColors.Has(kv.Key) ? zoneColors[kv.Key].Doubles : o["visual.floor_color"].Doubles;
                FleetGeometry.Visual(PrimitiveType.Cube, "zone_" + kv.Key, t, FleetFrames.World((r[0] + r[2]) / 2, (r[1] + r[3]) / 2, patch / 2),
                    Quaternion.identity, FleetFrames.LocalScale(r[2] - r[0], r[3] - r[1], patch), FleetFrames.Rgb(col));
            }

            double marker = o["scene.overhead_marker_size"].Double;
            foreach (var cam in s["overhead_cameras"].Items) {
                double[] p = cam["position"].Doubles, l = cam["look_at"].Doubles;
                var go = new GameObject("overhead_" + cam["id"].String);
                go.transform.SetParent(t, false);
                go.transform.position = FleetFrames.World(p[0], p[1], p[2]);
                go.transform.rotation = Quaternion.LookRotation(FleetFrames.World(l[0], l[1], l[2]) - go.transform.position, Vector3.up);
                FleetGeometry.Visual(PrimitiveType.Cube, "marker", go.transform, Vector3.zero, Quaternion.identity, Vector3.one * (float)marker, Color.black);
                OverheadCameras[cam["id"].String] = go.transform;
            }
            Light();
        }

        void Box(string name, double x, double y, double z, double sx, double sy, double sz, Color c) =>
            FleetGeometry.SolidBox(name, Root.transform, FleetFrames.World(x, y, z), FleetFrames.LocalScale(sx, sy, sz), c, WallMaterial);

        void Light() {
            JsonNode o = Spec.O;
            var sun = new GameObject("sun");
            sun.transform.SetParent(Root.transform, false);
            double[] e = o["scene.sun_euler_deg"].Doubles;
            sun.transform.rotation = Quaternion.Euler((float)e[0], (float)e[1], (float)e[2]);
            var light = sun.AddComponent<Light>();
            light.type = LightType.Directional;
            if (!sun.TryGetComponent<HDAdditionalLightData>(out _)) sun.AddComponent<HDAdditionalLightData>();
            light.lightUnit = LightUnit.Lux;
            light.intensity = (float)o["scene.sun_lux"].Double;
            light.shadows = LightShadows.Soft;

            var volGo = new GameObject("fleet volume");
            volGo.transform.SetParent(Root.transform, false);
            var vol = volGo.AddComponent<Volume>();
            vol.isGlobal = true;
            vol.priority = 100;
            var profile = ScriptableObject.CreateInstance<VolumeProfile>();
            var exposure = profile.Add<Exposure>(false);
            exposure.mode.Override(ExposureMode.Fixed);
            exposure.fixedExposure.Override((float)o["scene.exposure_ev100"].Double);
            var blur = profile.Add<MotionBlur>(false);
            blur.intensity.Override(0f);
            vol.sharedProfile = profile;
        }

        public void Destroy() => UnityEngine.Object.DestroyImmediate(Root);
    }
}
