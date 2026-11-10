// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

namespace Crane.Drones.TreeGenImport.Editor
{
    /// <summary>Imports a TreeGen library or stand output directory (Tools/TreeGen/CONTRACT.md) as prefabs.
    ///   Unity -batchmode -executeMethod Crane.Drones.TreeGenImport.Editor.TreeGenImporter.ImportFromCommandLine -quit
    ///     -treegenSrc /abs/path/OUT -treegenName lib_seed7
    /// Writes Assets/Drones/TreeGen/Imported/NAME/: manifest copy, trees/tree_NNNN/{lod0,lod1,lod2}.tgglb + colliders.json,
    /// and Prefabs/tree_NNNN.prefab (+ Prefabs/stand.prefab for stands). Prefab = LODGroup (LOD0/1/2 at 0.5/0.2/0.01 screen
    /// height), bark material (Poly Haven textures reused from Assets/Drones/Environment/Textures), leaf-card material
    /// (alpha clip, double-sided, translucent), CapsuleColliders from colliders.json on the Environment layer.
    /// Imported/ is git-ignored (large binary meshes); re-run this to regenerate it.</summary>
    public static class TreeGenImporter
    {
        public const string Root = "Assets/Drones/TreeGen";
        public const string ImportRoot = Root + "/Imported";
        public const string MatDir = Root + "/Materials";
        public const string PolyHaven = "Assets/Drones/Environment/Textures/PolyHaven/";
        public const int EnvLayer = 9; // "Environment" (ProjectSettings/TagManager.asset), same as BuildDroneForest.EnvLayer
        public static readonly float[] LodScreenHeights = { 0.5f, 0.2f, 0.01f };

        public static void ImportFromCommandLine()
        {
            var a = System.Environment.GetCommandLineArgs(); string src = Arg(a, "-treegenSrc"), name = Arg(a, "-treegenName");
            if (string.IsNullOrEmpty(src) || string.IsNullOrEmpty(name)) throw new ArgumentException("need -treegenSrc DIR -treegenName NAME");
            // comma-separated lists import several outputs in one Editor session (e.g. a library and a stand)
            var srcs = src.Split(','); var names = name.Split(',');
            if (srcs.Length != names.Length) throw new ArgumentException("-treegenSrc and -treegenName need the same number of entries");
            for (int i = 0; i < srcs.Length; i++)
            {
                var t0 = DateTime.Now; var r = Import(srcs[i], names[i]);
                int caps = r.Sum(e => e.colliders.capsules.Length);
                Debug.Log("[TreeGenImport] " + names[i] + ": " + r.Count + " prefabs, " + caps + " capsules in " + (DateTime.Now - t0).TotalSeconds.ToString("F1") + "s");
            }
        }

        public static string Arg(string[] a, string k) { for (int i = 0; i < a.Length - 1; i++) if (a[i] == k) return a[i + 1]; return null; }

        public class Entry { public TgTreeEntry tree; public GameObject prefab; public string collidersPath; public TgColliders colliders; }

        /// <summary>Copy, import and build prefabs. Returns one entry per manifest tree.</summary>
        public static List<Entry> Import(string srcDir, string name)
        {
            string mfName = File.Exists(Path.Combine(srcDir, "stand_manifest.json")) ? "stand_manifest.json" : "library_manifest.json";
            string mfText = File.ReadAllText(Path.Combine(srcDir, mfName));
            var man = TgJson.Parse<TgManifest>(mfText);
            string dst = ImportRoot + "/" + name; Directory.CreateDirectory(dst);
            EnsureGitIgnore();
            File.WriteAllText(Path.Combine(dst, mfName), mfText);
            foreach (var t in man.trees)
            {
                string sd = Path.Combine(srcDir, t.dir), dd = Path.Combine(dst, t.dir); Directory.CreateDirectory(dd);
                for (int l = 0; l < 3; l++) { string f = Path.Combine(sd, "lod" + l + ".glb"); if (File.Exists(f)) CopyIfChanged(f, Path.Combine(dd, "lod" + l + ".tgglb")); }
                CopyIfChanged(Path.Combine(sd, "colliders.json"), Path.Combine(dd, "colliders.json"));
            }
            AssetDatabase.Refresh();
            var mats = EnsureMaterials();
            string pd = dst + "/Prefabs"; Directory.CreateDirectory(pd); AssetDatabase.Refresh();
            var res = new List<Entry>();
            foreach (var t in man.trees)
            {
                string dd = dst + "/" + t.dir;
                var e = new Entry { tree = t, collidersPath = dd + "/colliders.json" };
                e.colliders = TgJson.Parse<TgColliders>(File.ReadAllText(e.collidersPath));
                var go = BuildTreeObject(t, dd, e.colliders, mats, srcDir);
                e.prefab = PrefabUtility.SaveAsPrefabAsset(go, pd + "/tree_" + t.tree_id.ToString("D4") + ".prefab");
                UnityEngine.Object.DestroyImmediate(go);
                res.Add(e);
            }
            if (mfName == "stand_manifest.json")
            {
                var stand = new GameObject("TreeGenStand_" + name);
                foreach (var e in res)
                {
                    var inst = (GameObject)PrefabUtility.InstantiatePrefab(e.prefab, stand.transform);
                    inst.transform.localPosition = TgJson.V(e.tree.position_unity); inst.transform.localRotation = Quaternion.identity; // stand trees are never rotated
                }
                PrefabUtility.SaveAsPrefabAsset(stand, pd + "/stand.prefab"); UnityEngine.Object.DestroyImmediate(stand);
            }
            AssetDatabase.SaveAssets();
            return res;
        }

        static void CopyIfChanged(string src, string dst)
        {
            if (File.Exists(dst) && new FileInfo(dst).Length == new FileInfo(src).Length && File.GetLastWriteTimeUtc(dst) >= File.GetLastWriteTimeUtc(src)) return;
            File.Copy(src, dst, true);
        }

        static void EnsureGitIgnore()
        {
            Directory.CreateDirectory(ImportRoot);
            string gi = ImportRoot + "/.gitignore";
            if (!File.Exists(gi)) File.WriteAllText(gi, "# Regenerated by TreeGenImporter (large binary meshes); do not commit.\n*\n!.gitignore\n");
        }

        /// <summary>Load the prefab entries of an already imported library/stand (no copying).</summary>
        public static List<Entry> Load(string name)
        {
            string dst = ImportRoot + "/" + name;
            string mf = File.Exists(dst + "/stand_manifest.json") ? dst + "/stand_manifest.json" : dst + "/library_manifest.json";
            if (!File.Exists(mf)) throw new FileNotFoundException("TreeGen import not found (run ImportFromCommandLine first): " + mf);
            var man = TgJson.Parse<TgManifest>(File.ReadAllText(mf)); var res = new List<Entry>();
            foreach (var t in man.trees)
            {
                var e = new Entry { tree = t, collidersPath = dst + "/" + t.dir + "/colliders.json" };
                e.colliders = TgJson.Parse<TgColliders>(File.ReadAllText(e.collidersPath));
                e.prefab = AssetDatabase.LoadAssetAtPath<GameObject>(dst + "/Prefabs/tree_" + t.tree_id.ToString("D4") + ".prefab");
                if (e.prefab != null) res.Add(e);
            }
            return res;
        }

        // ------------------------------------------------------------------ prefab assembly
        public static GameObject BuildTreeObject(TgTreeEntry t, string treeDir, TgColliders col, Mats mats, string source)
        {
            var root = new GameObject("TreeGen_" + t.species + "_" + t.tree_id.ToString("D4")); root.layer = EnvLayer;
            var info = root.AddComponent<TreeGenTree>();
            info.treeId = t.tree_id; info.species = t.species; info.variant = t.variant; info.skeletonHash = t.skeleton_hash; info.source = source;
            info.heightM = t.metrics != null ? t.metrics.height_m : 0f; info.dbhM = t.metrics != null ? t.metrics.dbh_m : -1f;
            if (t.bounds_unity_local != null) { info.boundsMin = TgJson.V(t.bounds_unity_local.min); info.boundsMax = TgJson.V(t.bounds_unity_local.max); }
            var (bark, leaf) = mats.For(t.species, t.variant);
            var lods = new List<LOD>();
            for (int l = 0; l < 3; l++)
            {
                var mesh = AssetDatabase.LoadAssetAtPath<Mesh>(treeDir + "/lod" + l + ".tgglb"); if (mesh == null) continue;
                var g = new GameObject("LOD" + l); g.layer = EnvLayer; g.transform.SetParent(root.transform, false);
                g.AddComponent<MeshFilter>().sharedMesh = mesh; var r = g.AddComponent<MeshRenderer>();
                r.sharedMaterials = mesh.subMeshCount > 1 ? new[] { bark, leaf } : new[] { bark };
                lods.Add(new LOD(LodScreenHeights[Mathf.Min(l, LodScreenHeights.Length - 1)], new Renderer[] { r }));
                info.barkTris[l] = (int)mesh.GetIndexCount(0) / 3; info.leafTris[l] = mesh.subMeshCount > 1 ? (int)mesh.GetIndexCount(1) / 3 : 0;
            }
            var lg = root.AddComponent<LODGroup>(); lg.SetLODs(lods.ToArray()); lg.RecalculateBounds();
            AddCapsules(root.transform, col);
            info.capsuleCount = col.capsules.Length;
            info.softObstacles = col.soft_obstacles.Select(s => new TreeGenSegment { a = s.a.V, b = s.b.V, radius = s.radius, dead = s.dead }).ToArray();
            return root;
        }

        /// <summary>CONTRACT section 6: child at center, localRotation = FromToRotation(up, axis), direction Y, height incl. caps.</summary>
        public static Transform AddCapsules(Transform root, TgColliders col)
        {
            var cr = new GameObject("Colliders"); cr.layer = EnvLayer; cr.transform.SetParent(root, false);
            for (int i = 0; i < col.capsules.Length; i++)
            {
                var c = col.capsules[i];
                var g = new GameObject("Capsule_" + i.ToString("D4")); g.layer = EnvLayer; g.transform.SetParent(cr.transform, false);
                g.transform.localPosition = c.center.V; g.transform.localRotation = Quaternion.FromToRotation(Vector3.up, c.axis.V);
                var cc = g.AddComponent<CapsuleCollider>(); cc.direction = 1; cc.height = c.height; cc.radius = c.radius;
            }
            return cr.transform;
        }

        // ------------------------------------------------------------------ materials
        public class Mats
        {
            public Dictionary<string, Material> bark = new Dictionary<string, Material>(), leaf = new Dictionary<string, Material>();
            public (Material, Material) For(string species, string variant)
            {
                string k = variant == "snag" || species == "snag" ? "snag" : variant == "fallen_log" || species == "fallen_log" ? "fallen_log" : species;
                Material b, l; if (!bark.TryGetValue(k, out b)) b = bark["beech"]; if (!leaf.TryGetValue(species, out l)) l = leaf.ContainsKey(k) ? leaf[k] : leaf["beech"];
                return (b, l);
            }
        }

        public static Mats EnsureMaterials()
        {
            Directory.CreateDirectory(MatDir); AssetDatabase.Refresh();
            var m = new Mats();
            Texture2D brownD = Tex(PolyHaven + "bark_brown_02_diff.jpg", false), brownN = Tex(PolyHaven + "bark_brown_02_nor.jpg", true);
            Texture2D pineD = Tex(PolyHaven + "pine_bark_diff.jpg", false), pineN = Tex(PolyHaven + "pine_bark_nor.jpg", true);
            var atlas = Atlas(Root + "/Textures/treegen_leaf_atlas.png");
            var diff = Diffusion(MatDir + "/TG_FoliageDiffusion.asset");
            m.bark["beech"] = BarkMat("TG_Bark_Beech", brownD, brownN, new Color(0.95f, 0.93f, 0.90f), 0.22f, 0.35f);
            m.bark["birch"] = BarkMat("TG_Bark_Birch", brownD, brownN, new Color(1.9f, 1.9f, 1.85f), 0.25f, 0.25f);
            m.bark["spruce"] = BarkMat("TG_Bark_Spruce", pineD, pineN, new Color(0.62f, 0.50f, 0.44f), 0.12f, 1.1f);
            m.bark["snag"] = BarkMat("TG_Bark_Snag", brownD, brownN, new Color(1.15f, 1.1f, 1.05f), 0.06f, 1.0f);
            m.bark["fallen_log"] = BarkMat("TG_Bark_Log", pineD, pineN, new Color(1.05f, 0.95f, 0.9f), 0.05f, 1.0f);
            m.leaf["beech"] = LeafMat("TG_Leaf_Beech", atlas, new Color(1f, 1f, 1f), diff);
            m.leaf["birch"] = LeafMat("TG_Leaf_Birch", atlas, new Color(1.05f, 1.08f, 0.85f), diff);
            m.leaf["spruce"] = LeafMat("TG_Leaf_Spruce", atlas, new Color(0.85f, 0.95f, 0.88f), diff);
            AssetDatabase.SaveAssets();
            return m;
        }

        static Texture2D Tex(string path, bool normal)
        {
            var ti = AssetImporter.GetAtPath(path) as TextureImporter;
            // same settings BuildDroneForest.ImportTextures applies, so either importer can run first
            if (ti != null && normal && ti.textureType != TextureImporterType.NormalMap) { ti.textureType = TextureImporterType.NormalMap; ti.sRGBTexture = false; ti.maxTextureSize = 2048; ti.mipmapEnabled = true; ti.SaveAndReimport(); }
            return AssetDatabase.LoadAssetAtPath<Texture2D>(path);
        }

        static Texture2D Atlas(string path)
        {
            var ti = AssetImporter.GetAtPath(path) as TextureImporter;
            if (ti != null && (!ti.alphaIsTransparency || ti.wrapMode != TextureWrapMode.Clamp || !ti.mipMapsPreserveCoverage))
            {
                ti.textureType = TextureImporterType.Default; ti.sRGBTexture = true; ti.alphaIsTransparency = true; ti.mipmapEnabled = true;
                ti.mipMapsPreserveCoverage = true; ti.alphaTestReferenceValue = 0.5f; ti.wrapMode = TextureWrapMode.Clamp; ti.maxTextureSize = 1024; ti.SaveAndReimport();
            }
            return AssetDatabase.LoadAssetAtPath<Texture2D>(path);
        }

        static Material NewLit(string name)
        {
            string path = MatDir + "/" + name + ".mat";
            var m = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (m == null) { m = new Material(Shader.Find("HDRP/Lit")) { name = name }; AssetDatabase.CreateAsset(m, path); }
            return m;
        }

        static Material BarkMat(string name, Texture2D d, Texture2D n, Color tint, float smooth, float nrmScale)
        {
            var m = NewLit(name);
            m.SetTexture("_BaseColorMap", d); m.SetColor("_BaseColor", tint); m.SetFloat("_Smoothness", smooth); m.SetFloat("_Metallic", 0f);
            if (n != null) { m.SetTexture("_NormalMap", n); m.SetFloat("_NormalScale", nrmScale); }
            Validate(m); return m;
        }

        static Material LeafMat(string name, Texture2D atlas, Color tint, DiffusionProfileSettings diff)
        {
            var m = NewLit(name);
            m.SetTexture("_BaseColorMap", atlas); m.SetColor("_BaseColor", tint); m.SetFloat("_Smoothness", 0.25f); m.SetFloat("_Metallic", 0f);
            m.SetFloat("_AlphaCutoffEnable", 1f); m.SetFloat("_AlphaCutoff", 0.5f); m.SetFloat("_UseShadowThreshold", 1f); m.SetFloat("_AlphaCutoffShadow", 0.5f);
            m.SetFloat("_AlphaCutoffPrepass", 0.5f); m.SetFloat("_AlphaCutoffPostpass", 0.5f);
            m.SetFloat("_DoubleSidedEnable", 1f); m.SetFloat("_DoubleSidedNormalMode", 1f); m.SetFloat("_CullMode", 0f); m.SetFloat("_CullModeForward", 0f);
            if (diff != null)
            {
                // translucent material type (5) with transmission, as the forest's own foliage materials do
                m.SetFloat("_MaterialID", 5f); m.SetFloat("_TransmissionEnable", 1f); m.SetFloat("_Thickness", 0.6f); m.SetVector("_ThicknessRemap", new Vector4(0.2f, 0.8f, 0, 0));
                try { HDMaterial.SetDiffusionProfile(m, diff); } catch (Exception e) { Debug.LogWarning("[TreeGenImport] SetDiffusionProfile: " + e.Message); }
            }
            Validate(m); return m;
        }

        /// <summary>Point the TreeGen leaf materials at a diffusion profile that the scene's DiffusionProfileList
        /// registers (HDRP only renders transmission for listed profiles). Called after a forest build with the
        /// forest's own foliage profile.</summary>
        public static int UseDiffusionProfile(DiffusionProfileSettings diff)
        {
            int n = 0; if (diff == null) return 0;
            foreach (var g in AssetDatabase.FindAssets("TG_Leaf_ t:Material", new[] { MatDir }))
            {
                var m = AssetDatabase.LoadAssetAtPath<Material>(AssetDatabase.GUIDToAssetPath(g)); if (m == null) continue;
                try { HDMaterial.SetDiffusionProfile(m, diff); EditorUtility.SetDirty(m); n++; } catch (Exception e) { Debug.LogWarning("[TreeGenImport] SetDiffusionProfile: " + e.Message); }
            }
            AssetDatabase.SaveAssets(); return n;
        }

        static void Validate(Material m)
        {
            try { UnityEditor.Rendering.HighDefinition.HDShaderUtils.ResetMaterialKeywords(m); } catch (Exception e) { Debug.LogWarning("[TreeGenImport] material keywords: " + e.Message); }
            EditorUtility.SetDirty(m);
        }

        static DiffusionProfileSettings Diffusion(string path)
        {
            var d = AssetDatabase.LoadAssetAtPath<DiffusionProfileSettings>(path);
            if (d != null) return d;
            d = ScriptableObject.CreateInstance<DiffusionProfileSettings>(); AssetDatabase.CreateAsset(d, path);
            try
            {
                var p = d.GetType().GetField("profile", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Public).GetValue(d);
                SetField(p, "scatteringDistance", new Color(0.25f, 0.55f, 0.12f)); SetField(p, "transmissionTint", new Color(0.55f, 0.8f, 0.2f));
                SetField(p, "thicknessRemap", new Vector2(0f, 1.2f)); SetField(p, "worldScale", 1f);
            }
            catch (Exception e) { Debug.LogWarning("[TreeGenImport] diffusion profile setup: " + e.Message); }
            EditorUtility.SetDirty(d); AssetDatabase.SaveAssets();
            return d;
        }

        static void SetField(object o, string f, object v)
        {
            var fi = o.GetType().GetField(f, System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Public);
            if (fi != null) fi.SetValue(o, v);
        }
    }
}
