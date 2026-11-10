// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Crane.Drones.Environment.Editor;
using EnvTreeGen = Crane.Drones.Environment.Editor.TreeGen;

namespace Crane.Drones.TreeGenImport.Editor
{
    /// <summary>Places imported TreeGen library variants in the Drone Forest through BuildDroneForest's seam.
    /// The forest layout (positions, species class, size class, yaw, allometric height) is unchanged; each placement asks
    /// <see cref="BuildDroneForest.ExternalTreeProvider"/> for a tree and this provider answers with the TreeGen variant of
    /// matching species and closest height to the size class. <see cref="BuildDroneForest.ExternalTreeInstantiated"/> then
    /// swaps the placeholder renderer/MeshCollider for the TreeGen prefab (LODGroup + CapsuleColliders) and makes the scale
    /// uniform (= the layout's height scale), so capsules stay exact and the manifest's flight-band segments (built from
    /// TreeResult.segs = TreeGen capsules + soft obstacles) match the colliders.
    ///   -executeMethod Crane.Drones.TreeGenImport.Editor.TreeGenForestProvider.BuildFromCommandLine -treegenLibrary NAME [BuildDroneForest args]
    /// Species map (forest class -> TreeGen preset): Spruce, Pine -> spruce; Oak -> birch; Beech -> beech; SnagConifer, SnagBroadleaf -> snag.</summary>
    public static class TreeGenForestProvider
    {
        static readonly CultureInfo CI = CultureInfo.InvariantCulture;
        static readonly string[] ForestNames = { "Spruce", "Pine", "Oak", "Beech", "SnagConifer", "SnagBroadleaf" };
        static readonly string[] TgSpecies = { "spruce", "spruce", "birch", "beech", "snag", "snag" };

        class Placed { public string name; public TreeGenImporter.Entry e; public Vector3 pos; public Quaternion rot; public float scale; }

        /// <summary>Render-only builds (comparison stills): drop the per-instance capsule GameObjects (about 220k in a full
        /// forest) to cut memory. Ground truth still references colliders.json. Never use for physics/flight runs.</summary>
        public static bool StripColliders;
        static List<TreeGenImporter.Entry> lib;
        static string libName;
        static readonly Dictionary<TreeResult, TreeGenImporter.Entry> issued = new Dictionary<TreeResult, TreeGenImporter.Entry>();
        static readonly List<Placed> placed = new List<Placed>();

        public static void Register(string library)
        {
            lib = TreeGenImporter.Load(library); libName = library; issued.Clear(); placed.Clear();
            if (lib.Count == 0) throw new InvalidOperationException("TreeGen library " + library + " has no prefabs");
            BuildDroneForest.ExternalTreeProvider = Provide;
            BuildDroneForest.ExternalTreeInstantiated = OnInstantiated;
        }

        public static void Unregister() { BuildDroneForest.ExternalTreeProvider = null; BuildDroneForest.ExternalTreeInstantiated = null; }

        /// <summary>Variant choice: same TreeGen species, sorted by |height - nominal class height|, the closest two
        /// (or all within 25%) form the pool and the layout's variant index picks from it. Deterministic.</summary>
        public static TreeGenImporter.Entry Choose(List<TreeGenImporter.Entry> pool, string species, float targetH, int variant)
        {
            var c = pool.Where(e => e.tree.species == species).OrderBy(e => Mathf.Abs(e.tree.metrics.height_m - targetH)).ThenBy(e => e.tree.tree_id).ToList();
            if (c.Count == 0) return null;
            float best = Mathf.Abs(c[0].tree.metrics.height_m - targetH);
            var near = c.Where(e => Mathf.Abs(e.tree.metrics.height_m - targetH) <= Mathf.Max(best, 0.25f * targetH)).ToList();
            if (near.Count < 2) near = c.Take(Mathf.Min(2, c.Count)).ToList();
            return near[Mathf.Abs(variant) % near.Count];
        }

        static TreeResult Provide(int sp, int sc, int variant)
        {
            if (lib == null || sp < 0 || sp >= TgSpecies.Length) return null;
            var e = Choose(lib, TgSpecies[sp], EnvTreeGen.NominalH(sc), variant); if (e == null) return null;
            var tr = new TreeResult
            {
                species = new Species { name = ForestNames[sp], conifer = sp == 0 || sp == 1 || sp == 4, dead = sp >= 4, bark = sp, crownBase = 0.3f, limbLen = 0.25f },
                sizeClass = sc, variant = variant, height = e.tree.metrics.height_m,
                trunkR = e.tree.metrics.dbh_m > 0 ? 0.5f * e.tree.metrics.dbh_m : 0.05f,
                visual = LodMesh(e, 2), collision = null,
            };
            foreach (var c in e.colliders.capsules) tr.segs.Add(new BranchSeg { a = c.a.V, b = c.b.V, r = c.radius, order = 0 });
            foreach (var c in e.colliders.soft_obstacles) if (c.radius >= 0.005f) tr.segs.Add(new BranchSeg { a = c.a.V, b = c.b.V, r = c.radius, order = 9 });
            float low = float.MaxValue; foreach (var c in e.colliders.soft_obstacles) low = Mathf.Min(low, Mathf.Min(c.a.y, c.b.y));
            tr.lowestLiveLimb = low == float.MaxValue ? tr.height : low;
            issued[tr] = e;
            return tr;
        }

        static Mesh LodMesh(TreeGenImporter.Entry e, int lod)
        {
            var mfs = e.prefab.GetComponentsInChildren<MeshFilter>(true);
            var m = mfs.FirstOrDefault(f => f.gameObject.name == "LOD" + lod) ?? mfs.LastOrDefault();
            return m != null ? m.sharedMesh : new Mesh();
        }

        static void OnInstantiated(GameObject g, TreeResult tr)
        {
            TreeGenImporter.Entry e; if (tr == null || !issued.TryGetValue(tr, out e)) return;
            float s = g.transform.localScale.y; g.transform.localScale = Vector3.one * s;
            var mc = g.GetComponent<MeshCollider>(); if (mc != null) UnityEngine.Object.DestroyImmediate(mc);
            var mr = g.GetComponent<MeshRenderer>(); if (mr != null) UnityEngine.Object.DestroyImmediate(mr);
            var mf = g.GetComponent<MeshFilter>(); if (mf != null) UnityEngine.Object.DestroyImmediate(mf);
            var inst = (GameObject)PrefabUtility.InstantiatePrefab(e.prefab, g.transform);
            inst.transform.localPosition = Vector3.zero; inst.transform.localRotation = Quaternion.identity; inst.transform.localScale = Vector3.one;
            if (StripColliders)
            {
                PrefabUtility.UnpackPrefabInstance(inst, PrefabUnpackMode.OutermostRoot, InteractionMode.AutomatedAction);
                var cr = inst.transform.Find("Colliders"); if (cr != null) UnityEngine.Object.DestroyImmediate(cr.gameObject);
            }
            placed.Add(new Placed { name = g.name, e = e, pos = g.transform.position, rot = g.transform.rotation, scale = s });
        }

        // ------------------------------------------------------------------ entry points
        public static void BuildFromCommandLine()
        {
            var a = System.Environment.GetCommandLineArgs(); string lib = TreeGenImporter.Arg(a, "-treegenLibrary") ?? "lib_seed7";
            var stats = BuildTreeGenForest(lib);
            Debug.Log("[TreeGenForest] " + stats);
        }

        /// <summary>Build with TreeGen trees: register, run BuildDroneForest.BuildFromCommandLine (same args), unregister,
        /// merge the TreeGen ground truth into the forest manifest. Returns a JSON stats line.</summary>
        public static string BuildTreeGenForest(string library)
        {
            var t0 = DateTime.Now;
            Register(library);
            try { BuildDroneForest.BuildFromCommandLine(); }
            finally { Unregister(); }
            double build = (DateTime.Now - t0).TotalSeconds;
            var gen = new DirectoryInfo(BuildDroneForest.Root + "/Generated");
            var fol = gen.Exists ? gen.GetFiles("FoliageDiffusion.asset", SearchOption.AllDirectories).OrderByDescending(f => f.LastWriteTimeUtc).FirstOrDefault() : null;
            if (fol != null)
            {
                string rel = "Assets" + fol.FullName.Replace('\\', '/').Substring(Application.dataPath.Length);
                int n = TreeGenImporter.UseDiffusionProfile(AssetDatabase.LoadAssetAtPath<UnityEngine.Rendering.HighDefinition.DiffusionProfileSettings>(rel));
                Debug.Log("[TreeGenForest] leaf materials use the forest diffusion profile " + rel + " (" + n + ")");
            }
            string mp = LatestManifest();
            if (mp != null) MergeManifest(mp);
            return SceneStats("treegen", build);
        }

        static string LatestManifest()
        {
            var d = new DirectoryInfo(BuildDroneForest.Root + "/Manifests"); if (!d.Exists) return null;
            var f = d.GetFiles("forest_seed*_manifest.json").OrderByDescending(x => x.LastWriteTimeUtc).FirstOrDefault();
            return f?.FullName;
        }

        static string F(float v) { return v.ToString("F3", CI); }
        static string V3(Vector3 v) { return "[" + F(v.x) + "," + F(v.y) + "," + F(v.z) + "]"; }

        /// <summary>Append a "treegen" section (appended key; existing keys untouched). Per tree: library source, the
        /// tree-local colliders.json (capsules + soft obstacles, CONTRACT section 6) and the world transform
        /// (world = pos + yaw rotation * scale * local). World-space lists are not inlined: a 1627-tree forest has
        /// ~222k capsules and ~4.7M soft obstacles (inlining gave a 340 MB manifest).</summary>
        public static void MergeManifest(string manifestPath)
        {
            string txt = File.ReadAllText(manifestPath).TrimEnd();
            if (!txt.EndsWith("}")) { Debug.LogWarning("[TreeGenForest] manifest not a JSON object: " + manifestPath); return; }
            var sb = new StringBuilder();
            long nCap = 0, nSoft = 0;
            sb.Append(",\n\"treegen\":{\"note\":\"TreeGen trees placed through BuildDroneForest.ExternalTreeProvider/ExternalTreeInstantiated. Ground truth per tree = colliders_json (tree-local Unity frame, Tools/TreeGen/CONTRACT.md section 6) transformed by world = pos + Euler(0,yaw_deg,0) * (scale * local). Hard capsules exist as CapsuleColliders on layer Environment(9); soft obstacles have no colliders. flight_band_segments above hold TreeGen capsules and soft obstacles with radius >= 5 mm.\",");
            sb.Append("\"library\":\"" + libName + "\",\"trees\":[\n");
            for (int i = 0; i < placed.Count; i++)
            {
                var p = placed[i]; var t = p.e.tree;
                if (i > 0) sb.Append(",\n");
                sb.Append("{\"name\":\"" + p.name + "\",\"treegen_species\":\"" + t.species + "\",\"treegen_variant\":\"" + t.variant + "\",\"treegen_tree_id\":" + t.tree_id + ",\"skeleton_hash\":\"" + t.skeleton_hash + "\"");
                sb.Append(",\"colliders_json\":\"" + p.e.collidersPath + "\",\"pos\":" + V3(p.pos) + ",\"yaw_deg\":" + F(p.rot.eulerAngles.y) + ",\"scale\":" + F(p.scale) + ",\"height_m\":" + F(t.metrics.height_m * p.scale));
                sb.Append(",\"n_capsules\":" + p.e.colliders.capsules.Length + ",\"n_soft_obstacles\":" + p.e.colliders.soft_obstacles.Length + "}");
                nCap += p.e.colliders.capsules.Length; nSoft += p.e.colliders.soft_obstacles.Length;
            }
            sb.Append("\n],\"n_trees\":" + placed.Count + ",\"n_capsules\":" + nCap + ",\"n_soft_obstacles\":" + nSoft + "}\n}\n");
            File.WriteAllText(manifestPath, txt.Substring(0, txt.Length - 1).TrimEnd() + sb);
            Debug.Log("[TreeGenForest] merged " + placed.Count + " trees, " + nCap + " capsules, " + nSoft + " soft obstacles (by reference) into " + manifestPath);
        }

        /// <summary>Counts for the open scene: trees, triangles per LOD level (sum over instances), colliders.</summary>
        public static string SceneStats(string label, double buildSeconds)
        {
            long[] tris = new long[3]; long builtinTris = 0; int lodGroups = 0;
            foreach (var lg in UnityEngine.Object.FindObjectsByType<LODGroup>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            {
                if (lg.GetComponent<TreeGenTree>() == null) continue; lodGroups++;
                var lods = lg.GetLODs();
                for (int l = 0; l < lods.Length && l < 3; l++) foreach (var r in lods[l].renderers) { var mf = r != null ? r.GetComponent<MeshFilter>() : null; if (mf != null && mf.sharedMesh != null) tris[l] += TriCount(mf.sharedMesh); }
            }
            var trees = GameObject.Find("DroneForest/Trees");
            if (trees != null) foreach (var mf in trees.GetComponentsInChildren<MeshFilter>(true)) if (mf.GetComponentInParent<TreeGenTree>() == null && mf.sharedMesh != null) builtinTris += TriCount(mf.sharedMesh);
            int caps = 0, meshCols = 0, other = 0;
            foreach (var c in UnityEngine.Object.FindObjectsByType<Collider>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            {
                if (c is CapsuleCollider && c.GetComponentInParent<TreeGenTree>() != null) caps++; else if (c is MeshCollider) meshCols++; else other++;
            }
            int treeObjs = trees != null ? trees.transform.childCount : 0;
            return string.Format(CI, "{{\"label\":\"{0}\",\"build_s\":{1:F1},\"tree_objects\":{2},\"treegen_lodgroups\":{3},\"treegen_tris_lod0\":{4},\"treegen_tris_lod1\":{5},\"treegen_tris_lod2\":{6},\"builtin_tree_tris\":{7},\"treegen_capsule_colliders\":{8},\"mesh_colliders\":{9},\"other_colliders\":{10}}}",
                label, buildSeconds, treeObjs, lodGroups, tris[0], tris[1], tris[2], builtinTris, caps, meshCols, other);
        }

        static long TriCount(Mesh m) { long n = 0; for (int i = 0; i < m.subMeshCount; i++) n += m.GetIndexCount(i) / 3; return n; }
    }
}
