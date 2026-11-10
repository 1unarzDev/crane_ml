using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.Rendering;

namespace Crane.Fleet.Sim {
    /// <summary>
    /// Render-only rover visuals from the asset pipeline's manifest: one transform per part (origin under its parent, then the joint),
    /// OBJ submeshes per material slot. Movable parts are driven every step from simulation state: wheels from the plant's wheel angles,
    /// hooks from the connector machine's hook positions, lift carriage and fingers from the lifter state. Nothing here has a collider.
    /// </summary>
    public sealed class AssetVisual {
        sealed class Driven {
            public Transform T;
            public Vector3 BasePos;
            public Quaternion BaseRot;
            public AssetManifest.Joint Joint;
        }

        public readonly AssetManifest Manifest;
        readonly Dictionary<string, Driven> parts = new Dictionary<string, Driven>();
        readonly Dictionary<string, Material> materials = new Dictionary<string, Material>();
        static readonly Dictionary<string, Texture2D> s_Textures = new Dictionary<string, Texture2D>();
        public readonly Dictionary<string, Renderer> Leds = new Dictionary<string, Renderer>();
        public Transform Root;

        public AssetVisual(RoverDescription d, Transform parent, FleetSpec spec) {
            Manifest = AssetManifest.Load(d.MeshManifestPath, loadMeshes: true);
            Root = new GameObject("render_asset").transform;
            Root.SetParent(parent, false);
            Build(Manifest, Root, slot => SlotMaterial(slot, d), parts);
            // one state LED per docking face, 4 cm above the face centre (LOOK.md: emissive accent per face)
            var o = spec.O;
            double led = o["visual.led_size"].Double, up = o["visual.led_height"].Double;
            foreach (var f in Manifest.Faces) {
                var go = FleetGeometry.Visual(PrimitiveType.Cube, "led_" + f.Name, Root,
                    FleetFrames.Local(f.Origin.Xyz[0] - Math.Cos(f.Origin.Rpy[2]) * led, f.Origin.Xyz[1] - Math.Sin(f.Origin.Rpy[2]) * led, f.Origin.Xyz[2] + up),
                    Quaternion.identity, Vector3.one * (float)led, Color.black);
                Leds[f.Name] = go.GetComponent<Renderer>();
            }
        }

        /// <summary>A payload (no joints): every visible part of the payload manifest whose name is <paramref name="part"/>.</summary>
        public static Transform BuildPayload(AssetManifest m, string part, Transform parent) {
            var root = new GameObject("render_" + part).transform;
            root.SetParent(parent, false);
            var cache = new Dictionary<string, Material>();
            var all = new Dictionary<string, Driven>();
            Build(m, root, slot => {
                if (cache.TryGetValue(slot, out var mat)) return mat;
                var ms = m.Materials.TryGetValue(slot, out var s) ? s : null;
                return cache[slot] = MakeMaterial(ms, null, slot);
            }, all, only: part);
            return root;
        }

        static void Build(AssetManifest m, Transform root, Func<string, Material> mat, Dictionary<string, Driven> outParts, string only = null) {
            // parents first: iterate until every part is placed (manifest order normally already satisfies this)
            var pending = new List<AssetManifest.Part>();
            foreach (var p in m.Parts) if (only == null || p.Name == only) pending.Add(p);
            int guard = 0;
            while (pending.Count > 0 && guard++ < 100) {
                for (int i = pending.Count - 1; i >= 0; i--) {
                    var p = pending[i];
                    Transform parent = p.Parent == "base_link" || p.Parent == "payload_frame" || only != null ? root
                        : outParts.TryGetValue(p.Parent, out var dp) ? dp.T : null;
                    if (parent == null) continue;
                    pending.RemoveAt(i);
                    var go = new GameObject(p.Name);
                    go.transform.SetParent(parent, false);
                    var dr = new Driven { T = go.transform, BasePos = FleetFrames.Local(p.Origin.Xyz), BaseRot = FleetFrames.Rpy(p.Origin.Rpy), Joint = p.Joint };
                    go.transform.localPosition = dr.BasePos;
                    go.transform.localRotation = dr.BaseRot;
                    outParts[p.Name] = dr;
                    if (p.Mesh != null && p.Mesh.Positions.Count > 0) {
                        go.AddComponent<MeshFilter>().sharedMesh = ToMesh(p.Mesh, p.Name);
                        var mats = new Material[p.Mesh.SubmeshMaterials.Count];
                        for (int k = 0; k < mats.Length; k++) mats[k] = mat(p.Mesh.SubmeshMaterials[k]);
                        go.AddComponent<MeshRenderer>().sharedMaterials = mats;
                    }
                    go.SetActive(p.Visible);
                }
            }
            if (pending.Count > 0) throw new FormatException($"{m.Path}: parts with unknown parents: {pending[0].Name} -> {pending[0].Parent}");
        }

        Material SlotMaterial(string slot, RoverDescription d) {
            if (materials.TryGetValue(slot, out var cached)) return cached;
            Manifest.Materials.TryGetValue(slot, out var ms);
            Color? over = null;
            if (slot == "shell_" + d.Role) over = FleetFrames.Rgb(d.Color);   // role colour coding (overlay) wins over the asset's shell colour
            string tex = ms?.Texture;
            if (tex != null && d.InstanceIndex > 0) {
                // second instance of a role carries tag ids + 10 (asset pipeline convention)
                string name = Path.GetFileNameWithoutExtension(tex);
                int us = name.LastIndexOf('_');
                if (us > 0 && int.TryParse(name.Substring(us + 1), out int id)) {
                    string alt = Path.Combine(Path.GetDirectoryName(tex), name.Substring(0, us + 1) + (id + 10 * d.InstanceIndex).ToString("D5") + Path.GetExtension(tex));
                    if (File.Exists(alt)) tex = alt;
                }
            }
            return materials[slot] = MakeMaterial(ms, over, slot, tex);
        }

        static Material MakeMaterial(AssetManifest.MaterialSlot ms, Color? over, string slot, string texOverride = null) {
            Color c = over ?? (ms != null ? FleetFrames.Rgb(ms.BaseColor) : Color.magenta);
            var m = FleetGeometry.Material(c);
            m.name = "asset " + slot;
            double shin = ms?.Shininess ?? 20;
            if (m.HasProperty("_Smoothness")) m.SetFloat("_Smoothness", Mathf.Clamp((float)(shin / 100.0), 0.15f, 0.8f));
            if (m.HasProperty("_Metallic")) m.SetFloat("_Metallic", slot == "pin" || slot == "axle_cap" || slot == "metal" ? 0.7f : 0f);
            string tex = texOverride ?? ms?.Texture;
            if (tex != null && File.Exists(tex)) {
                if (!s_Textures.TryGetValue(tex, out var t)) {
                    t = new Texture2D(2, 2, TextureFormat.RGBA32, false) { filterMode = FilterMode.Point, wrapMode = TextureWrapMode.Clamp, name = Path.GetFileName(tex) };
                    t.LoadImage(File.ReadAllBytes(tex));
                    s_Textures[tex] = t;
                }
                if (m.HasProperty("_BaseColorMap")) m.SetTexture("_BaseColorMap", t);
                if (m.HasProperty("_MainTex")) m.SetTexture("_MainTex", t);
            }
            return m;
        }

        public static Mesh ToMesh(ObjMesh obj, string name) {
            int n = obj.Positions.Count;
            var v = new Vector3[n];
            var nrm = new Vector3[n];
            var uv = new Vector2[n];
            bool hasNormals = true;
            for (int i = 0; i < n; i++) {
                v[i] = FleetFrames.Local(obj.Positions[i]);
                if (obj.Normals[i] != null) nrm[i] = FleetFrames.Local(obj.Normals[i]); else hasNormals = false;
                if (obj.Uvs[i] != null) uv[i] = new Vector2((float)obj.Uvs[i][0], (float)obj.Uvs[i][1]);
            }
            var m = new Mesh { name = name, indexFormat = n > 65000 ? IndexFormat.UInt32 : IndexFormat.UInt16 };
            m.vertices = v;
            m.uv = uv;
            m.subMeshCount = obj.Submeshes.Count;
            for (int s = 0; s < obj.Submeshes.Count; s++) {
                var t = obj.Submeshes[s].ToArray();
                for (int i = 0; i + 2 < t.Length; i += 3) { int a = t[i + 1]; t[i + 1] = t[i + 2]; t[i + 2] = a; }   // reflection: flip winding
                m.SetTriangles(t, s);
            }
            if (hasNormals) m.normals = nrm; else m.RecalculateNormals();
            m.RecalculateBounds();
            return m;
        }

        /// <summary>Set a joint value (rad or m, the manifest's q); the part pose is origin * J(q - authored).</summary>
        public void SetJoint(string part, double q) {
            if (!parts.TryGetValue(part, out var d) || d.Joint == null) return;
            double dq = q - d.Joint.Authored;
            switch (d.Joint.Type) {
                case "revolute":
                case "continuous":
                    d.T.localRotation = d.BaseRot * FleetFrames.AxisAngle(d.Joint.Axis, dq);
                    d.T.localPosition = d.BasePos;
                    break;
                case "prismatic":
                    d.T.localPosition = d.BasePos + d.BaseRot * (FleetFrames.Local(d.Joint.Axis) * (float)dq);
                    break;
            }
        }

        static readonly string[] s_WheelParts = { "wheel_FL", "wheel_FR", "wheel_RL", "wheel_RR" };

        public void SetWheels(double[] angles) {
            for (int i = 0; i < 4; i++) SetJoint(s_WheelParts[i], angles[i]);
        }

        /// <summary>Hooks of one face: fraction closed (0 open .. 1 closed) per hook A/B (machine HookPosA/B).</summary>
        public void SetFaceHooks(string face, double closedA, double closedB) {
            foreach (var h in Manifest.Hooks) {
                if (h.Face != face) continue;
                bool a = h.Name.EndsWith("_a");
                double f = a ? closedA : closedB;
                SetJoint(h.Part, h.ClosedAngle + (h.OpenAngle - h.ClosedAngle) * (1 - f));
            }
        }

        public void SetLed(string face, Color c, float nits) {
            if (!Leds.TryGetValue(face, out var r)) return;
            var m = r.sharedMaterial;
            m.SetColor("_BaseColor", c * 0.3f);
            if (m.HasProperty("_EmissiveColor")) m.SetColor("_EmissiveColor", c * nits);
        }

        public void SetLift(double q) { if (Manifest.LiftPart != null) SetJoint(Manifest.LiftPart, q); }

        /// <summary>Gripper opening (clear gap, m): each finger moves (open_max - opening)/2 along its axis.</summary>
        public void SetGripper(double opening) {
            if (Manifest.GripperOpeningRange == null) return;
            double move = (Manifest.GripperOpeningRange[1] - opening) / 2;
            foreach (var (part, _, travel) in Manifest.Fingers) SetJoint(part, Math.Max(travel[0], Math.Min(travel[1], move)));
        }
    }
}
