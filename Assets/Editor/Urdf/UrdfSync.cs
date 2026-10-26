// Sync a crane prefab from a URDF: frames from the URDF, components from the profile's rules.
// Menu: Crane/URDF/Sync Selected URDF, Crane/URDF/Check Selected URDF.
// Batch: Unity -batchmode -projectPath . -executeMethod Crane.Urdf.UrdfSync.Batch -urdf "Assets/../x.urdf[;more.urdf]" [-check]
//   exit 0 = ok, 1 = errors (each logged as "URDF_SYNC error <sentence>").
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using System.Text.RegularExpressions;
using System.Xml.Linq;
using Unity.Robotics.UrdfImporter;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Crane.Urdf
{
    public static class UrdfSync
    {
        public const float MaxPosErrorM = 0.001f, MaxAngErrorDeg = 0.1f;

        public class Report
        {
            public readonly List<string> errors = new List<string>(), warnings = new List<string>(), changes = new List<string>();
            public string prefabPath;
            public float worstPosM, worstAngDeg;
            public int instancesChecked;   // scene boats checked as they stand in their scenes
            public bool Ok => errors.Count == 0;
            public void Log()
            {
                foreach (var c in changes) Debug.Log("URDF_SYNC change " + c);
                foreach (var w in warnings) Debug.LogWarning("URDF_SYNC warn " + w);
                foreach (var e in errors) Debug.LogError("URDF_SYNC error " + e);
                Debug.Log($"URDF_SYNC {(Ok ? "ok" : "failed")} prefab={prefabPath} changes={changes.Count} warnings={warnings.Count} errors={errors.Count} worst_frame_mm={worstPosM * 1000:F3} worst_frame_deg={worstAngDeg:F3} scene_instances={instancesChecked}");
            }
        }

        // ---------- URDF model (our own pass: the stock importer drops tags and we need exact frames) ----------
        class UJoint { public string name, type, parent; public Vector3 xyz, axis = Vector3.right; public Quaternion rot = Quaternion.identity; }
        class UModel
        {
            public string name, root, dir;
            public readonly List<string> links = new List<string>();
            public readonly Dictionary<string, UJoint> jointOf = new Dictionary<string, UJoint>();   // child link -> joint
            public readonly Dictionary<string, HashSet<string>> tags = new Dictionary<string, HashSet<string>>();
            public readonly Dictionary<string, string> geomHash = new Dictionary<string, string>();
            public readonly HashSet<string> noInertial = new HashSet<string>();
            public XDocument doc;
        }

        static readonly Dictionary<string, string> SensorTag = new Dictionary<string, string>
        {
            { "ray", "lidar" }, { "gpu_ray", "lidar" }, { "lidar", "lidar" }, { "gpu_lidar", "lidar" },
            { "camera", "camera" }, { "depth", "camera" }, { "depth_camera", "camera" }, { "rgbd_camera", "camera" }, { "multicamera", "camera" },
            { "imu", "imu" }, { "gps", "gps" }, { "navsat", "gps" },
        };

        static Vector3 V(string s, Vector3 dflt)
        {
            if (string.IsNullOrWhiteSpace(s)) return dflt;
            var p = s.Split((char[])null, StringSplitOptions.RemoveEmptyEntries).Select(t => float.Parse(t, System.Globalization.CultureInfo.InvariantCulture)).ToArray();
            return new Vector3(p[0], p[1], p[2]);
        }

        // ROS (x fwd, y left, z up) -> Unity (x right, y up, z fwd)
        static Vector3 M(Vector3 p) => new Vector3(-p.y, p.z, p.x);
        static Quaternion M(Quaternion q) => new Quaternion(q.y, -q.z, -q.x, q.w);
        static Quaternion Rpy(Vector3 rpy)
        {
            Quaternion ax(Vector3 a, float r) => new Quaternion(a.x * Mathf.Sin(r / 2), a.y * Mathf.Sin(r / 2), a.z * Mathf.Sin(r / 2), Mathf.Cos(r / 2));
            return ax(Vector3.forward, rpy.z) * ax(Vector3.up, rpy.y) * ax(Vector3.right, rpy.x);   // ROS fixed-axis XYZ, in ROS axes
        }

        static UModel Parse(string urdfPath, Report r)
        {
            var m = new UModel { dir = Path.GetDirectoryName(urdfPath).Replace('\\', '/') };
            try { m.doc = XDocument.Load(urdfPath); }
            catch (Exception e) { r.errors.Add($"'{urdfPath}' is not readable XML: {e.Message}"); return null; }
            var robot = m.doc.Root;
            if (robot == null || robot.Name.LocalName != "robot") { r.errors.Add($"'{urdfPath}' has no <robot> root element (is it still xacro? expand it with `xacro file.xacro > file.urdf`)."); return null; }
            if (robot.Descendants().Any(e => e.Name.NamespaceName.Contains("xacro"))) { r.errors.Add($"'{urdfPath}' still contains xacro elements; expand it first (`xacro file.xacro > file.urdf`)."); return null; }
            m.name = (string)robot.Attribute("name") ?? Path.GetFileNameWithoutExtension(urdfPath);

            foreach (var l in robot.Elements("link"))
            {
                string n = (string)l.Attribute("name");
                if (m.links.Contains(n)) r.errors.Add($"link '{n}' is defined twice.");
                m.links.Add(n);
                if (l.Element("inertial") == null) m.noInertial.Add(n);
                // mesh paths: resolve by file name under the URDF folder, so package:// and ../ both work
                foreach (var mesh in l.Descendants("mesh"))
                {
                    string f = (string)mesh.Attribute("filename") ?? "";
                    string file = Path.GetFileName(f.Replace("package://", "").Replace("file://", ""));
                    string hit = Directory.GetFiles(m.dir, file, SearchOption.AllDirectories).FirstOrDefault();
                    if (hit == null) { r.errors.Add($"mesh '{file}' (link '{n}') is not under '{m.dir}'; copy it there."); continue; }
                    mesh.SetAttributeValue("filename", "package://" + hit.Replace('\\', '/').Substring(m.dir.Length + 1));
                }
                m.geomHash[n] = Hash(string.Concat(l.Elements("visual").Concat(l.Elements("collision")).Select(e => e.ToString(SaveOptions.DisableFormatting))));
            }
            foreach (var j in robot.Elements("joint"))
            {
                var o = j.Element("origin");
                var uj = new UJoint
                {
                    name = (string)j.Attribute("name"), type = (string)j.Attribute("type"),
                    parent = (string)j.Element("parent")?.Attribute("link"),
                    xyz = V((string)o?.Attribute("xyz"), Vector3.zero), rot = Rpy(V((string)o?.Attribute("rpy"), Vector3.zero)),
                    axis = V((string)j.Element("axis")?.Attribute("xyz"), Vector3.right),
                };
                string child = (string)j.Element("child")?.Attribute("link");
                if (!m.links.Contains(child)) { r.errors.Add($"joint '{uj.name}' has child link '{child}', which is not defined."); continue; }
                if (!m.links.Contains(uj.parent)) { r.errors.Add($"joint '{uj.name}' has parent link '{uj.parent}', which is not defined."); continue; }
                if (m.jointOf.ContainsKey(child)) { r.errors.Add($"link '{child}' is the child of two joints."); continue; }
                m.jointOf[child] = uj;
                if (j.Descendants("param").Any(p => (string)p.Attribute("name") == "thruster" && p.Value.Trim().ToLowerInvariant() == "true")) Tag(m, child, "thruster");
            }
            foreach (var g in robot.Elements("gazebo"))
            {
                string refLink = (string)g.Attribute("reference");
                foreach (var s in g.Descendants("sensor"))
                    if (refLink != null && SensorTag.TryGetValue(((string)s.Attribute("type") ?? "").ToLowerInvariant(), out var t)) Tag(m, refLink, t);
                foreach (var p in g.Descendants("plugin").Where(p => ((string)p.Attribute("filename") ?? "").ToLowerInvariant().Contains("thruster")))
                {
                    string ln = p.Element("link_name")?.Value?.Trim() ?? refLink;
                    if (ln != null) Tag(m, ln, "thruster");
                }
            }
            var roots = m.links.Where(l => !m.jointOf.ContainsKey(l)).ToList();
            if (roots.Count != 1) { r.errors.Add($"the URDF needs exactly one root link; found {roots.Count}: {string.Join(", ", roots)}."); return null; }
            m.root = roots[0];
            return m;
        }

        static void Tag(UModel m, string link, string tag)
        {
            if (!m.tags.TryGetValue(link, out var s)) m.tags[link] = s = new HashSet<string>();
            s.Add(tag);
        }

        static string Hash(string s)
        {
            using var sha = SHA1.Create();
            return BitConverter.ToString(sha.ComputeHash(Encoding.UTF8.GetBytes(s))).Replace("-", "").Substring(0, 12);
        }

        // Expected pose of each link relative to the root link, in Unity, with the profile's body yaw Y on every frame:
        // position Y·M(p), rotation Y·M(R)·Y⁻¹ (the frame carries the correction; geometry children get Y·M(R)).
        static void LocalPose(UModel m, string link, Quaternion y, out Vector3 pos, out Quaternion rot)
        {
            var j = m.jointOf[link];
            pos = y * M(j.xyz);
            rot = y * M(j.rot) * Quaternion.Inverse(y);
        }

        static void RootPose(UModel m, string link, Quaternion y, out Vector3 pos, out Quaternion rot)
        {
            pos = Vector3.zero; rot = Quaternion.identity;
            var chain = new List<string>();
            for (string l = link; l != m.root; l = m.jointOf[l].parent) chain.Add(l);
            chain.Reverse();
            foreach (var l in chain)
            {
                LocalPose(m, l, y, out var lp, out var lr);
                pos += rot * lp; rot *= lr;
            }
        }

        // ---------- entry points ----------
        [MenuItem("Crane/URDF/Sync Selected URDF")] static void MenuSync() => RunMenu(false);
        [MenuItem("Crane/URDF/Check Selected URDF")] static void MenuCheck() => RunMenu(true);
        [MenuItem("Crane/URDF/Sync Selected URDF", true)] [MenuItem("Crane/URDF/Check Selected URDF", true)]
        static bool MenuValid() => SelectedUrdf() != null;
        [MenuItem("Assets/Crane: Sync Robot from URDF")] static void ContextSync() => RunMenu(false);
        [MenuItem("Assets/Crane: Sync Robot from URDF", true)] static bool ContextValid() => SelectedUrdf() != null;

        static string SelectedUrdf()
        {
            string p = Selection.activeObject ? AssetDatabase.GetAssetPath(Selection.activeObject) : null;
            return p != null && p.EndsWith(".urdf", StringComparison.OrdinalIgnoreCase) ? p : null;
        }

        static void RunMenu(bool check)
        {
            var r = Run(SelectedUrdf(), check);
            r.Log();
            EditorUtility.DisplayDialog("URDF " + (check ? "check" : "sync"), r.Ok
                ? $"OK: {r.changes.Count} changes, {r.warnings.Count} warnings, worst frame error {r.worstPosM * 1000:F2} mm / {r.worstAngDeg:F2} deg."
                : string.Join("\n", r.errors.Take(8)), "Close");
        }

        public static void Batch()
        {
            var args = Environment.GetCommandLineArgs();
            int i = Array.IndexOf(args, "-urdf");
            if (i < 0 || i + 1 >= args.Length) { Debug.LogError("URDF_SYNC error pass -urdf <path to .urdf under Assets>."); EditorApplication.Exit(1); return; }
            bool ok = true;
            foreach (var urdf in args[i + 1].Split(';').Where(s => s.Length > 0))   // several run in order
            {
                var r = Run(urdf, args.Contains("-check"));
                r.Log();
                ok &= r.Ok;
            }
            EditorApplication.Exit(ok ? 0 : 1);
        }

        /// <summary>Sync (or with check, only validate) the prefab for this URDF.</summary>
        public static Report Run(string urdfPath, bool check)
        {
            var r = new Report();
            urdfPath = urdfPath?.Replace('\\', '/');
            if (urdfPath == null || !File.Exists(urdfPath)) { r.errors.Add($"URDF '{urdfPath}' does not exist."); return r; }
            var m = Parse(urdfPath, r);
            if (m == null || !r.Ok) return r;
            var profile = FindProfile(m.dir) ?? UrdfImportProfile.CreateDefault();
            r.prefabPath = string.IsNullOrEmpty(profile.prefabPath) ? $"{m.dir}/{m.name}.prefab" : profile.prefabPath;
            var yaw = M(Quaternion.AngleAxis(profile.rootYawDeg, Vector3.forward));   // ROS yaw about z, in ROS axes → Unity
            r.changes.Add($"profile '{profile.name}', body yaw {profile.rootYawDeg} deg, {m.links.Count} links");
            bool exists = File.Exists(r.prefabPath);

            if (check)
            {
                if (!exists) { r.errors.Add($"prefab '{r.prefabPath}' does not exist yet; run the sync first."); return r; }
                var c = PrefabUtility.LoadPrefabContents(r.prefabPath);
                try { CheckFrames(m, c, yaw, r, ""); } finally { PrefabUtility.UnloadPrefabContents(c); }
                CheckUsages(m, r.prefabPath, r);
                CheckSceneInstances(m, r.prefabPath, yaw, r);
                return r;
            }

            GameObject temp = BuildWithStockImporter(m, urdfPath, r);
            if (temp == null) return r;
            GameObject contents = exists ? PrefabUtility.LoadPrefabContents(r.prefabPath) : temp;
            var sidecar = r.prefabPath + ".urdfsync.json";   // where hashes lived before RobotDescription held them
            try
            {
                var oldHashes = exists ? OldHashes(contents, sidecar) : new Dictionary<string, string>();
                if (exists) Transplant(m, temp, contents, profile, oldHashes, yaw, r);
                else FixNewRobot(m, temp, yaw, r);
                FixInertials(m, contents, r);
                ApplyRules(m, contents, profile, r);
                Describe(contents, urdfPath, profile.rootYawDeg, m.geomHash, r);
                CheckFrames(m, contents, yaw, r, "");
                if (exists) CheckUsages(m, r.prefabPath, r);
                if (!r.Ok) { r.errors.Add("nothing saved."); return r; }
                PrefabUtility.SaveAsPrefabAsset(contents, r.prefabPath);
                if (File.Exists(sidecar)) { AssetDatabase.DeleteAsset(sidecar); r.changes.Add($"removed '{sidecar}' (hashes now on RobotDescription)"); }
                r.changes.Add($"saved '{r.prefabPath}'" + (exists ? "" : " (new)"));
                Register(m.name, r.prefabPath, r);
            }
            finally
            {
                if (exists) PrefabUtility.UnloadPrefabContents(contents);
                UnityEngine.Object.DestroyImmediate(temp);
            }
            return r;
        }

        /// <summary>Check the prefab's instances in one scene (open or not): frames and anchors as the boat stands there.</summary>
        public static Report CheckScene(string urdfPath, string scenePath)
        {
            var r = new Report();
            var m = Parse(urdfPath, r);
            if (m == null || !r.Ok) return r;
            var profile = FindProfile(m.dir) ?? UrdfImportProfile.CreateDefault();
            r.prefabPath = string.IsNullOrEmpty(profile.prefabPath) ? $"{m.dir}/{m.name}.prefab" : profile.prefabPath;
            CheckSceneInstances(m, r.prefabPath, M(Quaternion.AngleAxis(profile.rootYawDeg, Vector3.forward)), r, scenePath);
            return r;
        }

        static Dictionary<string, string> OldHashes(GameObject contents, string sidecar)
        {
            var d = new Dictionary<string, string>();
            var desc = contents.GetComponent<Sim.Physics.Core.RobotDescription>();
            if (desc != null && desc.geometryHashes != null && desc.geometryHashes.Length > 0)
            {
                foreach (var e in desc.geometryHashes) { int i = e.LastIndexOf('='); if (i > 0) d[e.Substring(0, i)] = e.Substring(i + 1); }
                return d;
            }
            return File.Exists(sidecar) ? JsonUtility.FromJson<Hashes>(File.ReadAllText(sidecar)).ToDict() : d;
        }

        // The runtime reads the robot's body frame (RoboBoatPhysics, RobotSelection) from this, not from the URDF.
        static void Describe(GameObject robot, string urdfPath, float rootYawDeg, Dictionary<string, string> geomHash, Report r)
        {
            var d = robot.GetComponent<Sim.Physics.Core.RobotDescription>();
            if (d == null)
            {
                d = robot.AddComponent<Sim.Physics.Core.RobotDescription>();
                r.changes.Add($"'{robot.name}' gets RobotDescription (body yaw {rootYawDeg} deg)");
            }
            d.urdf = urdfPath;
            d.rootYawDeg = rootYawDeg;
            d.geometryHashes = geomHash.OrderBy(kv => kv.Key, StringComparer.Ordinal).Select(kv => $"{kv.Key}={kv.Value}@{rootYawDeg}").ToArray();
        }

        // Every shippable robot is spawnable with --crane-robot <URDF robot name>; prefabs under an Editor folder never reach a build.
        static void Register(string robot, string prefabPath, Report r)
        {
            if (prefabPath.Split('/').Contains("Editor")) return;
            var catalog = AssetDatabase.LoadAssetAtPath<Sim.Physics.Core.RobotCatalog>(Sim.Physics.Core.RobotCatalog.AssetPath);
            if (catalog == null)
            {
                string folder = "Assets";
                foreach (var part in Path.GetDirectoryName(Sim.Physics.Core.RobotCatalog.AssetPath).Replace('\\', '/').Split('/').Skip(1))
                {
                    if (!AssetDatabase.IsValidFolder($"{folder}/{part}")) AssetDatabase.CreateFolder(folder, part);
                    folder = $"{folder}/{part}";
                }
                catalog = ScriptableObject.CreateInstance<Sim.Physics.Core.RobotCatalog>();
                AssetDatabase.CreateAsset(catalog, Sim.Physics.Core.RobotCatalog.AssetPath);
            }
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(prefabPath);
            var entry = catalog.robots.FirstOrDefault(e => e.name == robot);
            if (entry != null && entry.prefab == prefab) return;
            if (entry == null) catalog.robots.Add(entry = new Sim.Physics.Core.RobotCatalog.Entry { name = robot });
            entry.prefab = prefab;
            EditorUtility.SetDirty(catalog);
            AssetDatabase.SaveAssetIfDirty(catalog);
            r.changes.Add($"catalog: --crane-robot {robot} spawns '{prefabPath}'");
        }

        static UrdfImportProfile FindProfile(string dir)
        {
            for (string d = dir; !string.IsNullOrEmpty(d) && d.StartsWith("Assets"); d = Path.GetDirectoryName(d)?.Replace('\\', '/'))
            {
                var guid = AssetDatabase.FindAssets("t:UrdfImportProfile", new[] { d })
                    .FirstOrDefault(g => Path.GetDirectoryName(AssetDatabase.GUIDToAssetPath(g))?.Replace('\\', '/') == d);
                if (guid != null) return AssetDatabase.LoadAssetAtPath<UrdfImportProfile>(AssetDatabase.GUIDToAssetPath(guid));
            }
            return null;
        }

        [Serializable]
        class Hashes   // the legacy .urdfsync.json sidecar, read once and then removed
        {
            public float rootYawDeg; public List<string> links = new List<string>(), hashes = new List<string>();
            public Dictionary<string, string> ToDict()
            {
                var d = new Dictionary<string, string>();
                for (int i = 0; i < links.Count; i++) d[links[i]] = hashes[i] + "@" + rootYawDeg;
                return d;
            }
        }

        static GameObject BuildWithStockImporter(UModel m, string urdfPath, Report r)
        {
            // The rewritten copy (resolved mesh paths) sits next to the URDF so package:// resolves there; the dot keeps it out of the AssetDatabase.
            string tmp = $"{m.dir}/.urdfsync-{Path.GetFileName(urdfPath)}";
            m.doc.Save(tmp);
            try
            {
                var it = UrdfRobotExtensions.Create(Path.GetFullPath(tmp), ImportSettings.DefaultSettings());
                GameObject go = null;
                while (it.MoveNext()) go = it.Current ?? go;
                if (go == null) r.errors.Add($"the URDF importer could not build '{urdfPath}' (see the log above).");
                return go;
            }
            finally { File.Delete(tmp); }
        }

        static Dictionary<string, Transform> Links(GameObject robot, Report r)
        {
            var d = new Dictionary<string, Transform>();
            foreach (var l in robot.GetComponentsInChildren<UrdfLink>(true))
                if (!d.ContainsKey(l.name)) d[l.name] = l.transform;
                else r?.errors.Add($"the prefab has two links named '{l.name}'.");
            return d;
        }

        static bool IsGeometry(Transform t) => t.name == "Visuals" || t.name == "Collisions";

        // Geometry children were posed by the importer as M(origin); in a corrected frame they need Y·M(origin).
        static void CorrectGeometry(Transform link, Quaternion y)
        {
            foreach (Transform g in link) if (IsGeometry(g)) foreach (Transform c in g) { c.localPosition = y * c.localPosition; c.localRotation = y * c.localRotation; }
        }

        public const float NoInertialMassKg = 1e-3f, NoInertialTensor = 1e-6f;

        // A link without <inertial> (a sensor frame) is massless in the URDF; the stock importer leaves the ArticulationBody's
        // 1 kg default and an automatic tensor. Give it near-zero mass and a tiny explicit tensor (UrdfInertial re-applies
        // the tensor at Start, so it is written there too). Only the importer's untouched default is replaced.
        static void FixInertials(UModel m, GameObject robot, Report r)
        {
            var links = Links(robot, null);
            foreach (var n in m.noInertial)
            {
                if (!links.TryGetValue(n, out var t)) continue;
                var ab = t.GetComponent<ArticulationBody>();
                var ui = t.GetComponent<UrdfInertial>();
                if (!ab || (ui && ui.useUrdfData) || Mathf.Abs(ab.mass - 1f) > 1e-6f) continue;
                ab.mass = NoInertialMassKg;
                ab.automaticCenterOfMass = false; ab.centerOfMass = Vector3.zero;
                ab.automaticInertiaTensor = false; ab.inertiaTensor = Vector3.one * NoInertialTensor; ab.inertiaTensorRotation = Quaternion.identity;
                if (ui)
                {
                    ui.useUrdfData = true; ui.centerOfMass = Vector3.zero; ui.inertiaTensor = Vector3.one * NoInertialTensor;
                    ui.inertiaTensorRotation = Quaternion.identity; ui.inertialAxisRotation = Quaternion.identity;
                }
                r.changes.Add($"link '{n}' has no <inertial>: mass {NoInertialMassKg} kg, tensor {NoInertialTensor} (was the importer's 1 kg default)");
            }
        }

        // Geometry is replaced in place where the names line up, so its objects (and their fileIDs) survive a re-import:
        // a matched object takes the new transform and component values, unmatched new objects move in, stale ones go.
        static void MergeGeometry(Transform have, Transform want)
        {
            foreach (var g in want.Cast<Transform>().Where(IsGeometry).ToList())
            {
                var old = have.Cast<Transform>().FirstOrDefault(c => c.name == g.name);
                if (old == null) g.SetParent(have, false);
                else MergeInto(old, g);
            }
            foreach (var g in have.Cast<Transform>().Where(c => IsGeometry(c) && want.Find(c.name) == null).ToList()) UnityEngine.Object.DestroyImmediate(g.gameObject);
        }

        static void MergeInto(Transform dst, Transform src)
        {
            dst.localPosition = src.localPosition; dst.localRotation = src.localRotation; dst.localScale = src.localScale;
            dst.gameObject.SetActive(src.gameObject.activeSelf);
            var used = new HashSet<Component>();
            foreach (var sc in src.GetComponents<Component>())
            {
                if (sc == null || sc is Transform) continue;
                var dc = dst.GetComponents(sc.GetType()).FirstOrDefault(c => c.GetType() == sc.GetType() && !used.Contains(c)) ?? dst.gameObject.AddComponent(sc.GetType());
                used.Add(dc);
                UnityEditorInternal.ComponentUtility.CopyComponent(sc);
                UnityEditorInternal.ComponentUtility.PasteComponentValues(dc);
            }
            foreach (var dc in dst.GetComponents<Component>().Where(c => c != null && !(c is Transform) && !used.Contains(c)).ToList()) UnityEngine.Object.DestroyImmediate(dc);
            var claimed = new HashSet<Transform>();
            foreach (var sc in src.Cast<Transform>().ToList())
            {
                var dc = dst.Cast<Transform>().FirstOrDefault(c => c.name == sc.name && !claimed.Contains(c));
                if (dc == null) { sc.SetParent(dst, false); claimed.Add(sc); }
                else { claimed.Add(dc); MergeInto(dc, sc); }
            }
            foreach (var dc in dst.Cast<Transform>().Where(c => !claimed.Contains(c)).ToList()) UnityEngine.Object.DestroyImmediate(dc.gameObject);
            // keep the importer's sibling order, so a merge and a fresh import serialize alike
            int i = 0;
            foreach (var sc in src.Cast<Transform>().ToList()) dst.Find(sc.name)?.SetSiblingIndex(i++);
        }

        static void FixNewRobot(UModel m, GameObject temp, Quaternion y, Report r)
        {
            // Disabled as on Blastoise: the stock Controller holds every joint in position control (propellers never spin),
            // sets friction/damping (profiles own those) and polls the legacy Input this project turns off.
            foreach (var c in temp.GetComponentsInChildren<Unity.Robotics.UrdfImporter.Control.Controller>(true)) c.enabled = false;
            var links = Links(temp, r);
            foreach (var n in m.links)
            {
                var t = links[n];
                CorrectGeometry(t, y);
                if (n == m.root) continue;
                LocalPose(m, n, y, out var p, out var q);
                t.localPosition = p; t.localRotation = q;
                var ab = t.GetComponent<ArticulationBody>();
                if (ab) { ab.anchorRotation = y * ab.anchorRotation; SetParentAnchor(ab); }
            }
        }

        static void SetParentAnchor(ArticulationBody ab)
        {
            ab.matchAnchors = false;
            var t = ab.transform;
            ab.parentAnchorPosition = t.localPosition + t.localRotation * ab.anchorPosition;
            ab.parentAnchorRotation = t.localRotation * ab.anchorRotation;
        }

        // Round anchors to 1e-7 (0.1 um, well below the 1 mm check) so an edit and its revert give a byte-identical prefab.
        // Written through the serialized fields: the ArticulationBody setters convert through native joint frames and add float residue back.
        static void SnapAnchors(ArticulationBody ab)
        {
            var so = new SerializedObject(ab);
            so.FindProperty("m_AnchorPosition").vector3Value = Snap(ab.anchorPosition);
            so.FindProperty("m_ParentAnchorPosition").vector3Value = Snap(ab.parentAnchorPosition);
            so.FindProperty("m_AnchorRotation").quaternionValue = Snap(ab.anchorRotation);
            so.FindProperty("m_ParentAnchorRotation").quaternionValue = Snap(ab.parentAnchorRotation);
            so.ApplyModifiedPropertiesWithoutUndo();
        }

        static float Snap(float v) => (float)(System.Math.Round(v * 1e7) / 1e7) + 0f;   // double keeps it idempotent; + 0f turns -0 into 0
        static Vector3 Snap(Vector3 v) => new Vector3(Snap(v.x), Snap(v.y), Snap(v.z));
        static Quaternion Snap(Quaternion q) => new Quaternion(Snap(q.x), Snap(q.y), Snap(q.z), Snap(q.w));

        static void Transplant(UModel m, GameObject temp, GameObject contents, UrdfImportProfile profile, Dictionary<string, string> oldHashes, Quaternion y, Report r)
        {
            var want = Links(temp, r);
            var have = Links(contents, r);
            if (!have.ContainsKey(m.root)) { r.errors.Add($"the prefab's root link is not '{m.root}' (it has {string.Join(", ", have.Keys.Where(k => have[k].parent == contents.transform))}); renaming the root link is not supported, rename it in the prefab first."); return; }
            var holder = new GameObject("urdfsync-holder").transform;
            holder.SetParent(temp.transform, false);

            // parents before children
            var order = new List<string> { m.root };
            for (int i = 0; i < order.Count; i++) order.AddRange(m.links.Where(l => m.jointOf.TryGetValue(l, out var j) && j.parent == order[i]));

            foreach (var n in order)
            {
                Transform w = want[n];
                bool isNew = !have.TryGetValue(n, out var t);
                bool reparented = false;
                if (isNew)
                {
                    foreach (var child in w.Cast<Transform>().Where(c => c.GetComponent<UrdfLink>()).ToList()) child.SetParent(holder, false);
                    w.SetParent(have[m.jointOf[n].parent], false);
                    have[n] = t = w;
                    CorrectGeometry(t, y);
                    var ab0 = t.GetComponent<ArticulationBody>();
                    if (ab0) ab0.anchorRotation = y * ab0.anchorRotation;
                    r.changes.Add($"added link '{n}'");
                }
                else
                {
                    if (n != m.root && t.parent != have[m.jointOf[n].parent])
                    {
                        t.SetParent(have[m.jointOf[n].parent], false);
                        reparented = true;
                        r.changes.Add($"link '{n}' now hangs from '{m.jointOf[n].parent}'");
                    }
                    string wantType = w.GetComponent<UrdfJoint>()?.GetType().Name, haveType = t.GetComponent<UrdfJoint>()?.GetType().Name;
                    if (n != m.root && wantType != haveType)
                        r.warnings.Add($"joint of '{n}' is {wantType} in the URDF but {haveType} in the prefab; kept {haveType} (joint types are not synced).");
                    oldHashes.TryGetValue(n, out var oldHash);
                    if (oldHash != m.geomHash[n] + "@" + profile.rootYawDeg)
                    {
                        MergeGeometry(t, w);
                        CorrectGeometry(t, y);
                        r.changes.Add($"geometry of '{n}' re-imported" + (oldHash == null ? " (no stored hash)" : ""));
                    }
                }
                if (n == m.root) continue;
                LocalPose(m, n, y, out var p, out var q);
                bool moved = (t.localPosition - p).magnitude > 1e-6f || Quaternion.Angle(t.localRotation, q) > 1e-4f;
                if (moved && !isNew) r.changes.Add($"link '{n}' moved by {(t.localPosition - p).magnitude * 1000:F1} mm / {Quaternion.Angle(t.localRotation, q):F2} deg");
                t.localPosition = p; t.localRotation = q;
                var ab = t.GetComponent<ArticulationBody>();
                if (!ab) continue;
                // matchAnchors makes the Editor recompute the parent anchor from world poses on any change, which leaves float
                // residue, so a synced link always gets explicit local anchors: the prefab is then a function of the URDF alone.
                if (isNew || reparented || ab.matchAnchors) SetParentAnchor(ab);
                else if (moved)
                {
                    // Keep the parent anchor (the scene may override it) and solve the child anchor, so the solver puts the link exactly here.
                    var inv = Quaternion.Inverse(q);
                    ab.anchorPosition = inv * (ab.parentAnchorPosition - p);
                    ab.anchorRotation = inv * ab.parentAnchorRotation;
                }
                SnapAnchors(ab);
            }
            foreach (var n in have.Keys.Where(k => !m.links.Contains(k)).ToList())
            {
                if (!have[n]) continue;   // went with a removed parent
                if (!profile.removeMissingLinks) { r.warnings.Add($"link '{n}' is in the prefab but not in the URDF; kept (removeMissingLinks is off)."); continue; }
                var refs = UsagesOfLink(r.prefabPath, n, m);
                foreach (var u in refs)
                    r.errors.Add($"link '{n}' is not in the URDF, but {u.file}:{u.line} still targets its {u.what} ({u.prop ?? "a reference"}); removing it would orphan that override. Delete the override in that file first.");
                if (refs.Count > 0) continue;
                foreach (var keep in have[n].GetComponentsInChildren<UrdfLink>(true).Where(l => m.links.Contains(l.name)).ToList()) keep.transform.SetParent(holder, false);
                UnityEngine.Object.DestroyImmediate(have[n].gameObject);
                r.changes.Add($"removed link '{n}' (not in the URDF)");
            }
        }

        // ---------- rules ----------
        static Type FindType(string name, Report r, string rule)
        {
            var hits = TypeCache.GetTypesDerivedFrom<Component>().Where(t => t.Name == name || t.FullName == name).ToList();
            if (hits.Count == 0) r.errors.Add($"rule '{rule}' asks for component '{name}', but no such component type exists.");
            else if (hits.Count > 1) r.warnings.Add($"rule '{rule}': component name '{name}' is ambiguous ({string.Join(", ", hits.Select(h => h.FullName))}); using {hits[0].FullName}.");
            return hits.FirstOrDefault();
        }

        static void ApplyRules(UModel m, GameObject robot, UrdfImportProfile profile, Report r)
        {
            var links = Links(robot, null);
            foreach (var rule in profile.rules)
            {
                Regex linkRx = string.IsNullOrEmpty(rule.link) || rule.link == "<root>" ? null : new Regex(rule.link, RegexOptions.IgnoreCase);
                Regex onRx = string.IsNullOrEmpty(rule.on) ? null : new Regex(rule.on, RegexOptions.IgnoreCase);
                var matched = m.links.Where(l =>
                    (rule.link == "<root>" && l == m.root) ||
                    (!string.IsNullOrEmpty(rule.tag) && m.tags.TryGetValue(l, out var tg) && tg.Contains(rule.tag)) ||
                    (linkRx != null && linkRx.IsMatch(l))).ToList();
                // a link already matched through a parent's 'on' target (e.g. the propeller) is not matched again
                if (onRx != null) matched = matched.Where(l => !(m.jointOf.TryGetValue(l, out var j) && matched.Contains(j.parent) && onRx.IsMatch(l))).ToList();
                foreach (var l in matched)
                {
                    string target = l;
                    if (onRx != null)
                        target = m.links.FirstOrDefault(c => m.jointOf.TryGetValue(c, out var j) && j.parent == l && onRx.IsMatch(c)) ?? l;
                    if (rule.tag == "thruster" || rule.components.Contains("Thruster"))
                    {
                        if (m.jointOf.TryGetValue(target, out var tj) && (tj.type == "continuous" || tj.type == "revolute") && Mathf.Abs(Mathf.Abs(tj.axis.normalized.z) - 1) > 1e-3f)
                            r.warnings.Add($"thruster '{target}': thrust acts along the link's z axis, but its joint '{tj.name}' spins about ({tj.axis.x}, {tj.axis.y}, {tj.axis.z}); rotate the link so z is the propeller axis.");
                    }
                    Place(rule, links[target], robot, r);
                }
            }
        }

        static void Place(UrdfRule rule, Transform dst, GameObject robot, Report r)
        {
            Transform srcLink = null; string srcAsset = null;
            if (!string.IsNullOrEmpty(rule.template))
            {
                var parts = rule.template.Split('#');
                var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(parts[0]);
                srcLink = prefab ? prefab.GetComponentsInChildren<Transform>(true).FirstOrDefault(t => parts.Length < 2 || t.name == parts[1]) : null;
                if (srcLink == null) r.warnings.Add($"rule '{rule.name}': template '{rule.template}' not found; using an existing component in the robot or code defaults.");
                else srcAsset = parts[0];
            }
            // non-link children of the template (e.g. the Camera object of a camera link)
            if (srcLink != null)
                foreach (Transform c in srcLink)
                    if (!c.GetComponent<UrdfLink>() && !IsGeometry(c) && dst.Find(c.name) == null)
                    {
                        var copy = UnityEngine.Object.Instantiate(c.gameObject, dst, false);
                        copy.name = c.name;
                        Remap(copy.GetComponentsInChildren<Component>(true), srcLink, srcAsset, dst, robot, rule, r);
                    }
            foreach (var typeName in rule.components)
            {
                var type = FindType(typeName, r, rule.name);
                if (type == null || dst.GetComponent(type)) continue;
                Component src = srcLink ? srcLink.GetComponent(type) : null;
                Transform from = srcLink; string fromAsset = srcAsset;
                if (src == null)
                {
                    src = robot.GetComponentsInChildren(type, true).FirstOrDefault(c => c.transform != dst);
                    from = src ? src.transform : null; fromAsset = null;
                }
                var comp = dst.gameObject.AddComponent(type);
                if (src != null)
                {
                    UnityEditorInternal.ComponentUtility.CopyComponent(src);
                    UnityEditorInternal.ComponentUtility.PasteComponentValues(comp);
                    Remap(new[] { comp }, from, fromAsset, dst, robot, rule, r);
                }
                r.changes.Add($"'{dst.name}' gets {type.Name} (rule '{rule.name}'{(src ? ", values from " + (fromAsset ?? src.name) : ", code defaults")})");
                if (src == null)
                {
                    var so = new SerializedObject(comp);
                    var empty = new List<string>();
                    for (var it = so.GetIterator(); it.NextVisible(true);)
                        if (it.propertyType == SerializedPropertyType.ObjectReference && it.objectReferenceValue == null && it.name != "m_Script") empty.Add(it.propertyPath);
                    if (empty.Count > 0) r.warnings.Add($"'{dst.name}' {type.Name} has code defaults with empty references ({string.Join(", ", empty)}); give rule '{rule.name}' a template or set them in the prefab.");
                }
            }
        }

        // References copied from a template point into the template; send them to the same place in this robot.
        static void Remap(IEnumerable<Component> comps, Transform srcLink, string srcAsset, Transform dstLink, GameObject robot, UrdfRule rule, Report r)
        {
            var links = Links(robot, null);
            foreach (var comp in comps)
            {
                if (comp == null || comp is Transform) continue;
                var so = new SerializedObject(comp);
                var it = so.GetIterator();
                while (it.Next(true))
                {
                    if (it.propertyType != SerializedPropertyType.ObjectReference || it.objectReferenceValue == null || it.name == "m_Script" || it.name == "m_GameObject" || it.name == "m_PrefabAsset") continue;
                    var obj = it.objectReferenceValue;
                    var t = (obj as Component)?.transform ?? (obj as GameObject)?.transform;
                    if (t == null) continue;
                    bool fromTemplate = srcAsset != null ? AssetDatabase.GetAssetPath(obj) == srcAsset : srcLink != null && t.root == srcLink.root;
                    if (!fromTemplate) continue;
                    Transform anchor = srcLink != null && (t == srcLink || t.IsChildOf(srcLink)) ? srcLink : t.GetComponentsInParent<UrdfLink>(true).FirstOrDefault()?.transform;
                    Transform dstAnchor = anchor == srcLink ? dstLink : (anchor != null && links.TryGetValue(anchor.name, out var dl) ? dl : null);
                    Transform hit = dstAnchor != null ? (t == anchor ? dstAnchor : dstAnchor.Find(RelPath(anchor, t))) : null;
                    UnityEngine.Object mapped = hit == null ? null : obj is GameObject ? (UnityEngine.Object)hit.gameObject : hit.GetComponent(obj.GetType());
                    if (mapped == null && srcAsset == null && anchor != srcLink) continue;   // in-robot reference to another link: leave it
                    if (mapped == null) r.warnings.Add($"rule '{rule.name}': '{comp.GetType().Name}.{it.propertyPath}' on '{dstLink.name}' pointed at '{t.name}' in the template, which this robot lacks; left empty.");
                    it.objectReferenceValue = mapped;
                }
                so.ApplyModifiedPropertiesWithoutUndo();
            }
        }

        static string RelPath(Transform root, Transform t)
        {
            var parts = new List<string>();
            for (; t != null && t != root; t = t.parent) parts.Add(t.name);
            parts.Reverse();
            return string.Join("/", parts);
        }

        // ---------- validation ----------
        static void CheckFrames(UModel m, GameObject robot, Quaternion y, Report r, string where)
        {
            var links = Links(robot, r);
            if (!links.TryGetValue(m.root, out var root)) { r.errors.Add($"{where}the prefab has no root link '{m.root}'."); return; }
            foreach (var n in m.links)
            {
                if (!links.TryGetValue(n, out var t)) { r.errors.Add($"{where}link '{n}' is in the URDF but not in the prefab."); continue; }
                JointedRootPose(m, n, y, links, out var p, out var q);
                Vector3 hp = root.InverseTransformPoint(t.position);   // links are unscaled
                Quaternion hq = Quaternion.Inverse(root.rotation) * t.rotation;
                float dp = (hp - p).magnitude, da = Quaternion.Angle(hq, q);
                r.worstPosM = Mathf.Max(r.worstPosM, dp); r.worstAngDeg = Mathf.Max(r.worstAngDeg, da);
                if (dp > MaxPosErrorM || da > MaxAngErrorDeg) r.errors.Add($"{where}link '{n}' is {dp * 1000:F2} mm / {da:F2} deg away from the URDF (limit {MaxPosErrorM * 1000} mm / {MaxAngErrorDeg} deg).");
                if (n != m.root) CheckAnchors(m, n, t, y, r, where);
            }
            foreach (var n in links.Keys.Where(k => !m.links.Contains(k))) r.warnings.Add($"{where}link '{n}' is in the prefab but not in the URDF.");
        }

        // RootPose, except that a revolute joint's own rotation (its joint coordinate: a spinning propeller) is taken from the
        // link as it stands, as a twist about the joint axis through the anchor. Everything else must match the URDF.
        static void JointedRootPose(UModel m, string link, Quaternion y, Dictionary<string, Transform> links, out Vector3 pos, out Quaternion rot)
        {
            pos = Vector3.zero; rot = Quaternion.identity;
            var chain = new List<string>();
            for (string l = link; l != m.root; l = m.jointOf[l].parent) chain.Add(l);
            chain.Reverse();
            foreach (var l in chain)
            {
                LocalPose(m, l, y, out var lp, out var lq);
                if (links.TryGetValue(l, out var t) && t.GetComponent<ArticulationBody>() is ArticulationBody ab && ab && ab.jointType == ArticulationJointType.RevoluteJoint)
                {
                    var tw = Twist(Quaternion.Inverse(lq) * t.localRotation, (ab.anchorRotation * Vector3.right).normalized);
                    lp += lq * ab.anchorPosition - lq * tw * ab.anchorPosition;
                    lq *= tw;
                }
                pos += rot * lp; rot *= lq;
            }
        }

        static Quaternion Twist(Quaternion d, Vector3 axis)
        {
            var v = Vector3.Dot(new Vector3(d.x, d.y, d.z), axis) * axis;
            float n = Mathf.Sqrt(v.sqrMagnitude + d.w * d.w);
            return n < 1e-9f ? Quaternion.identity : new Quaternion(v.x / n, v.y / n, v.z / n, d.w / n);
        }

        // The joint frame the solver uses must put the link where the URDF does: parent anchor = URDF pose ∘ child anchor.
        static void CheckAnchors(UModel m, string n, Transform t, Quaternion y, Report r, string where)
        {
            var ab = t.GetComponent<ArticulationBody>();
            if (!ab) return;
            if (ab.matchAnchors) { r.errors.Add($"{where}link '{n}' has matchAnchors on; the sync writes explicit anchors (re-run the sync)."); return; }
            Quaternion pr = ab.parentAnchorRotation, cr = ab.anchorRotation;
            float pn = Mathf.Sqrt(pr.x * pr.x + pr.y * pr.y + pr.z * pr.z + pr.w * pr.w), cn = Mathf.Sqrt(cr.x * cr.x + cr.y * cr.y + cr.z * cr.z + cr.w * cr.w);
            if (Mathf.Abs(pn - 1) > 1e-3f || Mathf.Abs(cn - 1) > 1e-3f)
            {
                r.errors.Add($"{where}link '{n}' has a non-unit anchor rotation (parent {pr}, |{pn:F3}|; child {cr}, |{cn:F3}|), typically per-component overrides mixed with a synced prefab.");
                return;
            }
            LocalPose(m, n, y, out var p, out var q);
            Vector3 wantPos = p + q * ab.anchorPosition; Quaternion wantRot = q * cr;
            float dp = (ab.parentAnchorPosition - wantPos).magnitude, da = Quaternion.Angle(pr, wantRot);
            r.worstPosM = Mathf.Max(r.worstPosM, dp); r.worstAngDeg = Mathf.Max(r.worstAngDeg, da);
            if (dp > MaxPosErrorM || da > MaxAngErrorDeg)
                r.errors.Add($"{where}link '{n}': its joint's parent anchor is {dp * 1000:F2} mm / {da:F2} deg off the URDF joint frame (limit {MaxPosErrorM * 1000} mm / {MaxAngErrorDeg} deg); the solver would move the link there.");
        }

        class Usage { public string file, prop, what; public int line; public long id; }

        // Every line in a scene or prefab (other than the robot's own) that targets an object of this prefab: overrides
        // (target: {fileID, guid}), stripped corresponding objects and plain references. Text search: closed scenes count too.
        static List<Usage> Usages(string prefabPath)
        {
            var hits = new List<Usage>();
            string guid = AssetDatabase.AssetPathToGUID(prefabPath);
            if (string.IsNullOrEmpty(guid)) return hits;
            var rx = new Regex(@"fileID: (-?\d+), guid: " + guid + @"\b");
            var files = Directory.GetFiles("Assets", "*.unity", SearchOption.AllDirectories).Concat(Directory.GetFiles("Assets", "*.prefab", SearchOption.AllDirectories));
            foreach (var f in files.Select(f => f.Replace('\\', '/')).Where(f => f != prefabPath))
            {
                string text = File.ReadAllText(f);
                if (!text.Contains(guid)) continue;
                var lines = text.Split('\n');
                for (int i = 0; i < lines.Length; i++)
                {
                    var hit = rx.Match(lines[i]);
                    if (!hit.Success) continue;
                    string prop = null;
                    for (int k = i + 1; k <= i + 2 && k < lines.Length && prop == null; k++)
                    {
                        var pm = Regex.Match(lines[k], @"^\s*propertyPath: (\S+)");
                        if (pm.Success) prop = pm.Groups[1].Value;
                        else if (lines[k].TrimStart().StartsWith("-")) break;
                    }
                    hits.Add(new Usage { file = f, line = i + 1, id = long.Parse(hit.Groups[1].Value), prop = prop });
                }
            }
            return hits;
        }

        // fileID -> (object, link it belongs to) for every object in the saved prefab asset.
        static Dictionary<long, (UnityEngine.Object obj, string link)> AssetObjects(string prefabPath)
        {
            var d = new Dictionary<long, (UnityEngine.Object, string)>();
            foreach (var o in AssetDatabase.LoadAllAssetsAtPath(prefabPath))
            {
                if (o == null || !AssetDatabase.TryGetGUIDAndLocalFileIdentifier(o, out string _, out long id)) continue;
                var go = (o as Component)?.gameObject ?? o as GameObject;
                d[id] = (o, go ? go.GetComponentsInParent<UrdfLink>(true).FirstOrDefault()?.name : null);
            }
            return d;
        }

        static string What(UnityEngine.Object o) => $"{o.GetType().Name} on '{((o as Component)?.gameObject ?? o as GameObject)?.name}'";

        // References into a link (and everything under it) that the sync is about to remove.
        static List<Usage> UsagesOfLink(string prefabPath, string link, UModel m)
        {
            var objs = AssetObjects(prefabPath);
            var asset = AssetDatabase.LoadAssetAtPath<GameObject>(prefabPath);
            var lt = asset ? asset.GetComponentsInChildren<UrdfLink>(true).FirstOrDefault(l => l.name == link) : null;
            if (lt == null) return new List<Usage>();
            var under = new HashSet<UnityEngine.Object>(lt.GetComponentsInChildren<Transform>(true).SelectMany(t => t.GetComponents<Component>().Cast<UnityEngine.Object>().Append(t.gameObject)));
            var hits = Usages(prefabPath).Where(u => objs.TryGetValue(u.id, out var o) && under.Contains(o.obj)).ToList();
            foreach (var u in hits) u.what = What(objs[u.id].obj);
            return hits;
        }

        static readonly Regex AnchorProp = new Regex(@"^m_(Parent)?Anchor(Position|Rotation)\b|^m_MatchAnchors$");

        // Scenes and prefabs must not override what the sync writes (joint anchors of URDF links), nor target objects the
        // prefab no longer has: either way the boat in the scene is not the boat the URDF describes.
        static void CheckUsages(UModel m, string prefabPath, Report r)
        {
            var objs = AssetObjects(prefabPath);
            foreach (var u in Usages(prefabPath))
            {
                if (u.id == 100100000) continue;   // the prefab asset itself (m_SourcePrefab)
                if (!objs.TryGetValue(u.id, out var o))
                    r.errors.Add($"{u.file}:{u.line} targets object {u.id} ({u.prop ?? "a reference"}), which '{prefabPath}' no longer has (a removed link?); delete that override.");
                else if (u.prop != null && AnchorProp.IsMatch(u.prop) && o.obj is ArticulationBody && o.link != null && o.link != m.root && m.links.Contains(o.link))
                    r.errors.Add($"{u.file}:{u.line} overrides {u.prop} of link '{o.link}', which the sync writes from the URDF; delete that override (it mixes with the synced joint frame).");
            }
        }

        // Open (additively, if closed) each scene that holds this prefab and check every instance as it stands there.
        static void CheckSceneInstances(UModel m, string prefabPath, Quaternion y, Report r, string onlyScene = null)
        {
            string guid = AssetDatabase.AssetPathToGUID(prefabPath);
            var scenes = onlyScene != null ? new[] { onlyScene }
                : Directory.GetFiles("Assets", "*.unity", SearchOption.AllDirectories).Select(f => f.Replace('\\', '/')).Where(f => File.ReadAllText(f).Contains("guid: " + guid)).ToArray();
            foreach (var path in scenes)
            {
                var scene = SceneManager.GetSceneByPath(path);
                bool opened = !scene.isLoaded;
                if (opened) scene = EditorSceneManager.OpenScene(path, OpenSceneMode.Additive);
                try
                {
                    foreach (var go in scene.GetRootGameObjects().SelectMany(g => g.GetComponentsInChildren<Transform>(true)).Select(t => t.gameObject)
                                 .Where(g => PrefabUtility.IsAnyPrefabInstanceRoot(g) && PrefabUtility.GetPrefabAssetPathOfNearestInstanceRoot(g) == prefabPath))
                    {
                        CheckFrames(m, go, y, r, $"scene '{path}' {go.name}: ");
                        r.instancesChecked++;
                    }
                }
                finally { if (opened) EditorSceneManager.CloseScene(scene, true); }
            }
        }
    }
}
