using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;
using UnityEngine.SceneManagement;

namespace Crane.Drones.Environment.Editor
{
    /// <summary>
    /// Seeded Drone Forest generator.
    ///   Unity -batchmode -executeMethod Crane.Drones.Environment.Editor.BuildDroneForest.BuildFromCommandLine -quit
    ///     [-forestSpec path.json|asset] [-forestSeed N] [-forestScene Assets/.../Name.unity] [-forestLighting GoldenHour|Overcast|Dusk]
    /// Same spec + seed => identical scene and manifest. Layer: Environment (9) for terrain, trunks, branches, logs, rocks, targets.
    /// </summary>
    public static class BuildDroneForest
    {
        public const string Root = "Assets/Drones/Environment";
        public const int EnvLayer = 9; // "Environment" in ProjectSettings/TagManager.asset
        public const int FarLayer = 14; // "EnvironmentFar": distant visual-only landscape (no colliders, not in manifest)
        static readonly CultureInfo CI = CultureInfo.InvariantCulture;

        // ---------- build context ----------
        class Ctx
        {
            public ForestSpec spec; public string genDir; public System.Random rng;
            public Terrain terrain; public Transform tRoot;
            public Material[] bark = new Material[6]; public Material leaves, needles, fern, litter, rock, moss, wood, shrubStem, skin, jacket, trousers, boots, hood, mound, twigMat;
            public List<Vector2> trail = new List<Vector2>(), corridor = new List<Vector2>();
            public List<Vector3> clearings = new List<Vector3>(); // x,z,radius
            public Vector2 spawn; public float spawnYaw;
            public List<TreeRec> trees = new List<TreeRec>(); public List<string> branchLines = new List<string>();
            public List<PropRec> props = new List<PropRec>(); public List<TargetRec> targets = new List<TargetRec>();
            public Func<float, float, float> H; // world x,z -> world y
            public Dictionary<string, TreeResult> treeCache = new Dictionary<string, TreeResult>();
            public Dictionary<string, Mesh> meshCache = new Dictionary<string, Mesh>();
            public Material[] ferns = new Material[4], leafVar = new Material[6], needleVar = new Material[4], branchMats = new Material[2];
            public Material grass, farBroad, farConifer, farGround; public List<MidRec> mids = new List<MidRec>(); public int nGrass, nBranch, farTrees;
            public Texture2D tMoss, tMossN, tNeedle, tNeedleN, tGrass, tGrassN, tLeaf2, tLeaf2N;
        }
        class TreeRec { public int id; public string species; public int sizeClass, variant; public Vector3 pos; public float yaw, sx, sy, dbhCm, heightM; public bool dead; public int branchCount; }
        class MidRec { public int id; public string kind; public Vector3 pos; public float yaw, scale, height, crownR; public int segCount; public string segs; }
        class PropRec { public string kind; public Vector3 pos; public float yaw, size; public string note; }
        class TargetRec { public int id; public string pose; public Vector3 pos; public float yaw; public float occlusion; public int coverSticks, coverMounds, coverFerns; public bool log; public float visibleFraction = -1; public string name; }

        static Species[] SpeciesTable = {
            new Species{ name="Spruce", conifer=true, bark=0, crownBase=0.20f, limbLen=0.20f },
            new Species{ name="Pine", conifer=true, bark=1, crownBase=0.52f, limbLen=0.14f },
            new Species{ name="Oak", conifer=false, bark=2, crownBase=0.30f, limbLen=0.32f },
            new Species{ name="Beech", conifer=false, bark=3, crownBase=0.36f, limbLen=0.27f },
            new Species{ name="SnagConifer", conifer=true, dead=true, bark=4, crownBase=0.2f, limbLen=0.2f },
            new Species{ name="SnagBroadleaf", conifer=false, dead=true, bark=5, crownBase=0.3f, limbLen=0.3f },
        };

        // ---------- entry points ----------
        [MenuItem("CRANE/Drones/Build Drone Forest (default spec)")]
        public static void BuildMenu() { BuildFromCommandLine(); }

        public static void BuildFromCommandLine()
        {
            var args = System.Environment.GetCommandLineArgs(); string specPath = Arg(args, "-forestSpec"), seedS = Arg(args, "-forestSeed"), scenePath = Arg(args, "-forestScene"), light = Arg(args, "-forestLighting");
            ForestSpec spec;
            if (!string.IsNullOrEmpty(specPath) && specPath.EndsWith(".json")) spec = ForestSpec.FromJson(File.ReadAllText(specPath));
            else if (!string.IsNullOrEmpty(specPath)) spec = AssetDatabase.LoadAssetAtPath<ForestSpec>(specPath);
            else
            {
                string p = Root + "/Specs/ForestSpec_Default.asset"; spec = AssetDatabase.LoadAssetAtPath<ForestSpec>(p);
                if (spec == null) { Directory.CreateDirectory(Root + "/Specs"); spec = ScriptableObject.CreateInstance<ForestSpec>(); AssetDatabase.CreateAsset(spec, p); }
            }
            if (!string.IsNullOrEmpty(seedS)) spec.seed = int.Parse(seedS);
            if (!string.IsNullOrEmpty(light)) spec.lightingPreset = light;
            if (string.IsNullOrEmpty(scenePath)) scenePath = spec.seed == 1337 ? "Assets/Scenes/Drone Forest.unity" : Root + "/Generated/DroneForest_seed" + spec.seed + ".unity";
            Build(spec, scenePath);
        }
        static string Arg(string[] a, string k) { for (int i = 0; i < a.Length - 1; i++) if (a[i] == k) return a[i + 1]; return null; }

        // ---------- main ----------
        public static void Build(ForestSpec spec, string scenePath)
        {
            var t0 = DateTime.Now;
            var c = new Ctx { spec = spec, rng = new System.Random(spec.seed) }; CurrentSeed = spec.seed;
            c.genDir = Root + "/Generated/seed_" + spec.seed;
            if (AssetDatabase.IsValidFolder(c.genDir)) AssetDatabase.DeleteAsset(c.genDir);
            Directory.CreateDirectory(c.genDir); AssetDatabase.Refresh();
            ImportTextures();
            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var root = new GameObject("DroneForest").transform;
            var diff = MakeDiffusionProfile(c);
            MakeMaterials(c, diff);
            Layout(c);
            BuildTerrain(c, root);
            PlaceTrees(c, root);
            PlaceUndergrowth(c, root);
            PlaceTargets(c, root);
            PlaceMidstory(c, root);
            BuildFarLandscape(c, root);
            MarkersAndRigs(c, root);
            var dir = ForestLighting.Build(root, Root + "/Profiles", spec.lightingPreset, diff);
            PostChecks(c);
            Directory.CreateDirectory(Path.GetDirectoryName(scenePath));
            EditorSceneManager.SaveScene(scene, scenePath);
            WriteManifest(c, scenePath);
            AssetDatabase.SaveAssets();
            Debug.Log("[DroneForest] built " + scenePath + " seed=" + spec.seed + " trees=" + c.trees.Count + " in " + (DateTime.Now - t0).TotalSeconds.ToString("F1") + "s");
        }

        // ---------- textures & materials ----------
        static void ImportTextures()
        {
            AssetDatabase.Refresh();
            foreach (var g in AssetDatabase.FindAssets("t:Texture2D", new[] { Root + "/Textures/PolyHaven" }))
            {
                string p = AssetDatabase.GUIDToAssetPath(g); var ti = (TextureImporter)AssetImporter.GetAtPath(p); if (ti == null) continue;
                bool nrm = p.Contains("_nor"); var want = nrm ? TextureImporterType.NormalMap : TextureImporterType.Default;
                if (ti.textureType != want || ti.maxTextureSize != 2048) { ti.textureType = want; ti.maxTextureSize = 2048; ti.sRGBTexture = !nrm; ti.mipmapEnabled = true; ti.SaveAndReimport(); }
            }
        }

        static Texture2D Tex(string rel) { return AssetDatabase.LoadAssetAtPath<Texture2D>(Root + "/Textures/PolyHaven/" + rel); }

        static Texture2D SavePng(Texture2D t, string path, bool alpha, int kind = 0) // kind 0 colour, 1 normal map, 2 linear data
        {
            File.WriteAllBytes(path, t.EncodeToPNG()); UnityEngine.Object.DestroyImmediate(t); AssetDatabase.ImportAsset(path);
            var ti = (TextureImporter)AssetImporter.GetAtPath(path);
            if (kind == 1) { ti.textureType = TextureImporterType.NormalMap; ti.wrapMode = alpha ? TextureWrapMode.Clamp : TextureWrapMode.Repeat; ti.maxTextureSize = 1024; ti.SaveAndReimport(); return AssetDatabase.LoadAssetAtPath<Texture2D>(path); }
            if (kind == 2) { ti.sRGBTexture = false; ti.alphaIsTransparency = false; ti.wrapMode = TextureWrapMode.Clamp; ti.maxTextureSize = 1024; ti.SaveAndReimport(); return AssetDatabase.LoadAssetAtPath<Texture2D>(path); }
            ti.alphaIsTransparency = alpha; ti.mipmapEnabled = true; ti.sRGBTexture = true; ti.maxTextureSize = 1024;
            ti.wrapMode = alpha ? TextureWrapMode.Clamp : TextureWrapMode.Repeat; if (alpha) ti.mipMapsPreserveCoverage = true; if (alpha) ti.alphaTestReferenceValue = 0.5f;
            ti.SaveAndReimport(); return AssetDatabase.LoadAssetAtPath<Texture2D>(path);
        }

        static DiffusionProfileSettings MakeDiffusionProfile(Ctx c)
        {
            string path = c.genDir + "/FoliageDiffusion.asset";
            var d = ScriptableObject.CreateInstance<DiffusionProfileSettings>(); AssetDatabase.CreateAsset(d, path);
            try
            {
                object p = Priv(d, "profile");
                ForestLighting.Try(p, "scatteringDistance", new Color(0.25f, 0.55f, 0.12f)); ForestLighting.Try(p, "transmissionTint", new Color(0.55f, 0.8f, 0.2f));
                ForestLighting.Try(p, "thicknessRemap", new Vector2(0f, 1.2f)); ForestLighting.Try(p, "worldScale", 1f);
                ForestLighting.Try(p, "transmissionMode", "Regular"); ForestLighting.Try(p, "texturingMode", "PreAndPostScatter");
            }
            catch (Exception e) { Debug.LogWarning("[DroneForest] diffusion profile setup: " + e.Message); }
            EditorUtility.SetDirty(d); AssetDatabase.SaveAssets();
            try
            {
                string guid = AssetDatabase.AssetPathToGUID(path); uint hash = uint.Parse(guid.Substring(0, 8), System.Globalization.NumberStyles.HexNumber); if (hash == 0) hash = 1;
                object pr = Priv(d, "profile"); var hf = pr.GetType().GetField("hash", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.NonPublic); hf.SetValue(pr, hash); EditorUtility.SetDirty(d); AssetDatabase.SaveAssets();
            }
            catch (Exception e) { Debug.LogWarning("[DroneForest] diffusion hash: " + e.Message); }
            return d;
        }

        static object Priv(object o, string f) { var fi = o.GetType().GetField(f, System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Public); return fi.GetValue(o); }

        static Material Mat(string path, Texture2D basemap, Texture2D nrm, Color tint, float smooth, float nrmScale = 1f, bool clip = false, bool dbl = false, DiffusionProfileSettings diff = null, Vector2? tiling = null, Texture2D thick = null)
        {
            AssetDatabase.DeleteAsset(path);
            var m = new Material(Shader.Find("HDRP/Lit")); m.name = Path.GetFileNameWithoutExtension(path);
            if (basemap != null) { m.SetTexture("_BaseColorMap", basemap); if (tiling.HasValue) m.SetTextureScale("_BaseColorMap", tiling.Value); }
            m.SetColor("_BaseColor", tint); m.SetFloat("_Smoothness", smooth); m.SetFloat("_Metallic", 0f);
            if (nrm != null) { m.SetTexture("_NormalMap", nrm); m.SetFloat("_NormalScale", nrmScale); }
            if (clip) { m.SetFloat("_AlphaCutoffEnable", 1f); m.SetFloat("_AlphaCutoff", 0.5f); m.SetFloat("_UseShadowThreshold", 1f); m.SetFloat("_AlphaCutoffShadow", 0.5f); m.SetFloat("_AlphaCutoffPrepass", 0.5f); m.SetFloat("_AlphaCutoffPostpass", 0.5f); }
            if (dbl) { m.SetFloat("_DoubleSidedEnable", 1f); m.SetFloat("_DoubleSidedNormalMode", 1f); m.SetFloat("_CullMode", 0f); m.SetFloat("_CullModeForward", 0f); }
            if (diff != null)
            {
                m.SetFloat("_MaterialID", 5f); m.SetFloat("_TransmissionEnable", 1f); m.SetFloat("_Thickness", 1f); m.SetVector("_ThicknessRemap", new Vector4(0.3f, 0.9f, 0, 0)); if (thick != null) m.SetTexture("_ThicknessMap", thick);
            }
            AssetDatabase.CreateAsset(m, path);
            if (diff != null) // must run after CreateAsset: HDRP records the profile as an external asset reference of the saved material
            {
                try { HDMaterial.SetDiffusionProfile(m, diff); EditorUtility.SetDirty(m); } catch (Exception e) { Debug.LogWarning("SetDiffusionProfile: " + e.Message); }
            }
            try { UnityEditor.Rendering.HighDefinition.HDShaderUtils.ResetMaterialKeywords(m); } catch (Exception e) { Debug.LogWarning("ValidateMaterial: " + e.Message); }
            EditorUtility.SetDirty(m); return m;
        }

        static void MakeMaterials(Ctx c, DiffusionProfileSettings diff)
        {
            string d = c.genDir; string tex = d + "/Tex"; Directory.CreateDirectory(tex);
            var pineD = Tex("pine_bark_diff.jpg"); var pineN = Tex("pine_bark_nor.jpg");
            var brkD = Tex("bark_brown_02_diff.jpg"); var brkN = Tex("bark_brown_02_nor.jpg");
            var leafD = Tex("forest_leaves_02_diff.jpg"); var leafN = Tex("forest_leaves_02_nor.jpg");
            var mudD = Tex("brown_mud_leaves_01_diff.jpg"); var mudN = Tex("brown_mud_leaves_01_nor.jpg");
            var rockD = Tex("rock_boulder_cracked_diff.jpg"); var rockN = Tex("rock_boulder_cracked_nor.jpg");
            c.bark[0] = Mat(d + "/Bark_Spruce.mat", pineD, pineN, new Color(0.62f, 0.50f, 0.44f), 0.12f, 1.1f);
            c.bark[1] = Mat(d + "/Bark_Pine.mat", pineD, pineN, new Color(0.95f, 0.62f, 0.45f), 0.14f, 1.0f);
            c.bark[2] = Mat(d + "/Bark_Oak.mat", brkD, brkN, new Color(0.55f, 0.50f, 0.46f), 0.10f, 1.3f);
            c.bark[3] = Mat(d + "/Bark_Beech.mat", brkD, brkN, new Color(0.95f, 0.93f, 0.90f), 0.22f, 0.35f);
            c.bark[4] = Mat(d + "/Bark_SnagConifer.mat", pineD, pineN, new Color(1.2f, 1.15f, 1.1f), 0.06f, 1.0f);
            c.bark[5] = Mat(d + "/Bark_SnagBroadleaf.mat", brkD, brkN, new Color(1.15f, 1.1f, 1.05f), 0.06f, 1.0f);
            c.twigMat = c.bark[4];
            Texture2D bl = TexGen.Broadleaf(c.spec.seed + 11), bln = TexGen.NormalFromAlpha(bl, 4f), blt = TexGen.ThicknessFromAlpha(bl);
            bl = SavePng(bl, tex + "/leaf_broadleaf.png", true); bln = SavePng(bln, tex + "/leaf_broadleaf_n.png", true, 1); blt = SavePng(blt, tex + "/leaf_broadleaf_t.png", true, 2);
            Color[] leafTints = { new Color(1f, 1f, 1f), new Color(0.82f, 1.05f, 0.8f), new Color(1.2f, 1.12f, 0.68f), new Color(0.78f, 0.95f, 0.9f), new Color(1.3f, 0.98f, 0.62f), new Color(0.92f, 1.0f, 0.72f) };
            for (int i = 0; i < 6; i++) c.leafVar[i] = Mat(d + "/Foliage_Broadleaf" + (i == 0 ? "" : "_" + i) + ".mat", bl, bln, leafTints[i], 0.22f, 0.8f, true, true, diff, null, blt);
            c.leaves = c.leafVar[0];
            Texture2D nd = TexGen.Needles(c.spec.seed + 12), ndn = TexGen.NormalFromAlpha(nd, 3f), ndt = TexGen.ThicknessFromAlpha(nd);
            nd = SavePng(nd, tex + "/leaf_needles.png", true); ndn = SavePng(ndn, tex + "/leaf_needles_n.png", true, 1); ndt = SavePng(ndt, tex + "/leaf_needles_t.png", true, 2);
            Color[] needleTints = { new Color(0.9f, 1f, 0.9f), new Color(0.72f, 0.95f, 1.0f), new Color(1.1f, 1.05f, 0.68f), new Color(0.66f, 0.82f, 0.74f) };
            for (int i = 0; i < 4; i++) c.needleVar[i] = Mat(d + "/Foliage_Needles" + (i == 0 ? "" : "_" + i) + ".mat", nd, ndn, needleTints[i], 0.2f, 0.8f, true, true, diff, null, ndt);
            c.needles = c.needleVar[0];
            Color[] fernTints = { Color.white, new Color(1.1f, 1.1f, 0.8f), new Color(0.95f, 1.05f, 0.9f), new Color(0.85f, 0.95f, 0.85f) };
            for (int i = 0; i < 4; i++)
            {
                Texture2D ft = TexGen.FernFrond(c.spec.seed + 13 + i, 512, i), fn = TexGen.NormalFromAlpha(ft, 3f), fth = TexGen.ThicknessFromAlpha(ft);
                ft = SavePng(ft, tex + "/fern_frond" + i + ".png", true); fn = SavePng(fn, tex + "/fern_frond" + i + "_n.png", true, 1); fth = SavePng(fth, tex + "/fern_frond" + i + "_t.png", true, 2);
                c.ferns[i] = Mat(d + "/Understory_Fern" + i + ".mat", ft, fn, fernTints[i], 0.2f, 0.8f, true, true, diff, null, fth);
            }
            c.fern = c.ferns[0];
            Texture2D gt = TexGen.GrassBlades(c.spec.seed + 21), gn = TexGen.NormalFromAlpha(gt, 3f), gth = TexGen.ThicknessFromAlpha(gt);
            gt = SavePng(gt, tex + "/grass_blades.png", true); gn = SavePng(gn, tex + "/grass_blades_n.png", true, 1); gth = SavePng(gth, tex + "/grass_blades_t.png", true, 2);
            c.grass = Mat(d + "/Understory_Grass.mat", gt, gn, new Color(0.72f, 0.78f, 0.58f), 0.15f, 0.6f, true, true, diff, null, gth);
            c.branchMats[0] = Mat(d + "/FallenBranch_A.mat", pineD, pineN, new Color(0.55f, 0.45f, 0.38f), 0.06f, 0.9f);
            c.branchMats[1] = Mat(d + "/FallenBranch_B.mat", brkD, brkN, new Color(0.62f, 0.58f, 0.52f), 0.06f, 0.9f);
            // tileable procedural ground layers (moss, needle litter, grass, oak-leaf litter)
            Texture2D td, tn;
            TexGen.GroundTile("moss", c.spec.seed + 31, out td, out tn); c.tMoss = SavePng(td, tex + "/ground_moss.png", false); c.tMossN = SavePng(tn, tex + "/ground_moss_n.png", false, 1);
            TexGen.GroundTile("needles", c.spec.seed + 32, out td, out tn); c.tNeedle = SavePng(td, tex + "/ground_needles.png", false); c.tNeedleN = SavePng(tn, tex + "/ground_needles_n.png", false, 1);
            TexGen.GroundTile("grass", c.spec.seed + 33, out td, out tn); c.tGrass = SavePng(td, tex + "/ground_grass.png", false); c.tGrassN = SavePng(tn, tex + "/ground_grass_n.png", false, 1);
            TexGen.GroundTile("leaves2", c.spec.seed + 34, out td, out tn); c.tLeaf2 = SavePng(td, tex + "/ground_leaves2.png", false); c.tLeaf2N = SavePng(tn, tex + "/ground_leaves2_n.png", false, 1);
            // distant-forest proxies (visual only)
            TexGen.GroundTile("canopyB", c.spec.seed + 41, out td, out tn); var cb = SavePng(td, tex + "/canopy_broad.png", false); var cbn = SavePng(tn, tex + "/canopy_broad_n.png", false, 1);
            TexGen.GroundTile("canopyC", c.spec.seed + 42, out td, out tn); var cc = SavePng(td, tex + "/canopy_conifer.png", false); var ccn = SavePng(tn, tex + "/canopy_conifer_n.png", false, 1);
            c.farBroad = Mat(d + "/Far_Broadleaf.mat", cb, cbn, new Color(1.05f, 1.05f, 0.9f), 0.08f, 0.8f, false, true); c.farConifer = Mat(d + "/Far_Conifer.mat", cc, ccn, new Color(1f, 1f, 1f), 0.08f, 0.8f, false, true);
            c.farGround = Mat(d + "/Far_Ground.mat", cb, cbn, new Color(0.8f, 0.85f, 0.75f), 0.05f, 0.8f, false, true);
            c.litter = Mat(d + "/Litter_Cards.mat", leafD, leafN, new Color(0.9f, 0.7f, 0.5f), 0.12f, 1f, false, true, null);
            c.rock = Mat(d + "/Rock.mat", rockD, rockN, new Color(0.7f, 0.7f, 0.68f), 0.18f, 1.2f);
            c.moss = Mat(d + "/Moss.mat", mudD, mudN, new Color(0.30f, 0.46f, 0.12f), 0.05f, 1.5f);
            c.wood = Mat(d + "/CutWood.mat", brkD, brkN, new Color(1.2f, 1.0f, 0.8f), 0.08f, 0.6f);
            c.shrubStem = Mat(d + "/ShrubStem.mat", brkD, null, new Color(0.35f, 0.28f, 0.2f), 0.1f);
            c.mound = Mat(d + "/LitterMound.mat", leafD, leafN, new Color(0.85f, 0.65f, 0.45f), 0.08f, 1.0f);
            var weave = SavePng(TexGen.Weave(c.spec.seed + 14), tex + "/weave.png", false);
            c.skin = Mat(d + "/Mannequin_Skin.mat", null, null, new Color(0.55f, 0.38f, 0.30f), 0.35f);
            c.jacket = Mat(d + "/Mannequin_Jacket.mat", weave, null, new Color(0.30f, 0.34f, 0.22f), 0.12f, 1f, false, false, null, new Vector2(1, 1));
            c.trousers = Mat(d + "/Mannequin_Trousers.mat", weave, null, new Color(0.22f, 0.25f, 0.30f), 0.10f);
            c.boots = Mat(d + "/Mannequin_Boots.mat", null, null, new Color(0.10f, 0.08f, 0.07f), 0.30f);
            c.hood = Mat(d + "/Mannequin_Hood.mat", weave, null, new Color(0.45f, 0.20f, 0.14f), 0.08f);
        }

        // ---------- layout: trail, corridor, clearings ----------
        static float Perlin(float x, float z, float off) { return Mathf.PerlinNoise(x + off, z + off * 0.7f + 13.1f); }

        static void Layout(Ctx c)
        {
            var s = c.spec; var r = new System.Random(s.seed * 7919 + 1); float A = s.areaSize, half = A * 0.5f - 8f;
            // trail: smoothed random walk across the map
            Vector2 p = new Vector2(-half * 0.8f, -half * 0.7f); float head = Mathf.Atan2(1f, 1f) + Geo.R(r, -0.4f, 0.4f);
            c.spawn = p; c.trail.Add(p);
            for (int i = 0; i < 30; i++)
            {
                Vector2 target = new Vector2(half * 0.8f, half * 0.7f); float want = Mathf.Atan2(target.y - p.y, target.x - p.x);
                head = Mathf.LerpAngle(head * Mathf.Rad2Deg, want * Mathf.Rad2Deg, 0.12f) * Mathf.Deg2Rad + Geo.R(r, -0.28f, 0.28f);
                p += new Vector2(Mathf.Cos(head), Mathf.Sin(head)) * 6f;
                if (Mathf.Abs(p.x) > half || Mathf.Abs(p.y) > half) break; c.trail.Add(p);
            }
            c.spawnYaw = Mathf.Atan2(c.trail[1].x - c.trail[0].x, c.trail[1].y - c.trail[0].y) * Mathf.Rad2Deg;
            // flight corridor: leaves the trail after ~18 m and threads through the stand
            Vector2 q = c.trail[Mathf.Min(3, c.trail.Count - 1)]; c.corridor.Add(c.spawn); c.corridor.Add(q);
            float hd = head + (r.NextDouble() < 0.5 ? 0.7f : -0.7f);
            hd = Mathf.Atan2(c.trail[3 < c.trail.Count ? 3 : c.trail.Count - 1].y - c.trail[2].y, c.trail[3 < c.trail.Count ? 3 : c.trail.Count - 1].x - c.trail[2].x) + (r.NextDouble() < 0.5 ? 0.65f : -0.65f);
            for (int i = 0; i < 20; i++)
            {
                hd += Geo.R(r, -0.35f, 0.35f); Vector2 n = q + new Vector2(Mathf.Cos(hd), Mathf.Sin(hd)) * 5f;
                if (Mathf.Abs(n.x) > half || Mathf.Abs(n.y) > half) { hd += 1.2f; continue; }
                q = n; c.corridor.Add(q);
            }
            // clearings (not on spawn)
            int tries = 0;
            while (c.clearings.Count < s.clearingCount && tries++ < 200)
            {
                Vector2 cp = new Vector2(Geo.R(r, -half + 12, half - 12), Geo.R(r, -half + 12, half - 12)); float rr = Geo.R(r, s.clearingRadiusRange.x, s.clearingRadiusRange.y);
                if (Vector2.Distance(cp, c.spawn) < rr + 12) continue;
                if (c.clearings.Any(o => Vector2.Distance(cp, new Vector2(o.x, o.y)) < o.z + rr + 12)) continue;
                c.clearings.Add(new Vector3(cp.x, cp.y, rr));
            }
        }

        static float DistPoly(Vector2 p, List<Vector2> poly)
        {
            float best = float.MaxValue;
            for (int i = 0; i < poly.Count - 1; i++)
            {
                Vector2 a = poly[i], b = poly[i + 1], ab = b - a; float t = Mathf.Clamp01(Vector2.Dot(p - a, ab) / Mathf.Max(1e-4f, ab.sqrMagnitude));
                best = Mathf.Min(best, (a + ab * t - p).magnitude);
            }
            return best;
        }
        static float ClearingDepth(Ctx c, Vector2 p) // >0 inside a clearing, value = distance inside edge
        {
            float best = -999f; foreach (var cl in c.clearings) best = Mathf.Max(best, cl.z - Vector2.Distance(p, new Vector2(cl.x, cl.y))); return best;
        }

        // ---------- terrain ----------
        static void BuildTerrain(Ctx c, Transform root)
        {
            var s = c.spec; float A = s.areaSize; int res = 513; float ySize = 60f, yBase = -30f; float off = s.seed * 0.137f;
            Vector2 slopeDir = new Vector2(Mathf.Cos(s.seed * 0.7f), Mathf.Sin(s.seed * 0.7f)); float tilt = Mathf.Tan(s.slopeDeg * Mathf.Deg2Rad);
            Func<float, float, float> hf = (x, z) =>
            {
                float big = (Perlin(x * 0.012f, z * 0.012f, off) - 0.5f) * 14f + (Perlin(x * 0.035f, z * 0.035f, off + 40) - 0.5f) * 4.5f;
                float fine = (Perlin(x * 0.2f, z * 0.2f, off + 7) - 0.5f) * 0.55f + (Perlin(x * 0.7f, z * 0.7f, off + 9) - 0.5f) * 0.14f;
                // erosion-like gullies
                float gully = Mathf.Abs(Perlin(x * 0.02f, z * 0.02f, off + 90) - 0.5f); gully = Mathf.Clamp01(gully * 8f); big -= (1f - gully) * 1.4f;
                Vector2 pp = new Vector2(x, z); float w = 0f;
                foreach (var cl in c.clearings) w = Mathf.Max(w, Mathf.SmoothStep(0, 1, (cl.z + 6f - Vector2.Distance(pp, new Vector2(cl.x, cl.y))) / 9f));
                w = Mathf.Max(w, Mathf.SmoothStep(0, 1, (4f - DistPoly(pp, c.trail)) / 3f) * 0.7f);
                w = Mathf.Max(w, Mathf.SmoothStep(0, 1, (7f - Vector2.Distance(pp, c.spawn)) / 4f));
                return (big * (1f - 0.55f * w) * Mathf.Lerp(0.35f, 1f, Mathf.Clamp01(s.terrainRoughness)) + fine * s.terrainRoughness * (1f - 0.85f * w)) + Vector2.Dot(slopeDir, pp) * tilt;
            };
            var td = new TerrainData { heightmapResolution = res, size = new Vector3(A, ySize, A), alphamapResolution = 512 };
            var h = new float[res, res];
            for (int j = 0; j < res; j++) for (int i = 0; i < res; i++)
                {
                    float x = (float)i / (res - 1) * A - A / 2, z = (float)j / (res - 1) * A - A / 2;
                    h[j, i] = Mathf.Clamp01((hf(x, z) - yBase) / ySize);
                }
            td.SetHeights(0, 0, h);
            // layers
            string ld = c.genDir; TerrainLayer L2(string n, Texture2D d, Texture2D nr, float tile, float sm, float ns = 1.1f) { var t = new TerrainLayer { diffuseTexture = d, normalMapTexture = nr, tileSize = new Vector2(tile, tile), smoothness = sm, normalScale = ns }; AssetDatabase.CreateAsset(t, ld + "/TL_" + n + ".terrainlayer"); return t; }
            td.terrainLayers = new[] {
                L2("Litter", Tinted("forest_leaves_02_diff.jpg", "litter_dark", new Color(0.66f, 0.60f, 0.52f)), Tex("forest_leaves_02_nor.jpg"), 3.0f, 0.1f), L2("MudLeaves", Tinted("brown_mud_leaves_01_diff.jpg", "mud_dark", new Color(0.62f, 0.58f, 0.54f)), Tex("brown_mud_leaves_01_nor.jpg"), 3.0f, 0.2f),
                L2("Rock", Tex("rock_boulder_cracked_diff.jpg"), Tex("rock_boulder_cracked_nor.jpg"), 4.5f, 0.15f), L2("Moss", c.tMoss, c.tMossN, 2.5f, 0.12f, 1.0f),
                L2("NeedleLitter", c.tNeedle, c.tNeedleN, 2.2f, 0.08f, 0.9f), L2("Grass", c.tGrass, c.tGrassN, 2.0f, 0.1f, 0.8f), L2("OakLeaves", c.tLeaf2, c.tLeaf2N, 2.6f, 0.1f, 0.9f) };
            int ar = td.alphamapResolution; var am = new float[ar, ar, 7];
            for (int j = 0; j < ar; j++) for (int i = 0; i < ar; i++)
                {
                    float u = (float)i / (ar - 1), v = (float)j / (ar - 1); float x = u * A - A / 2, z = v * A - A / 2; var pp = new Vector2(x, z);
                    float steep = td.GetSteepness(u, v);
                    float rockW = Mathf.SmoothStep(0, 1, (steep - 24f) / 12f) + (Perlin(x * 0.09f, z * 0.09f, off + 300) > 0.78f ? 0.8f : 0f);
                    float mud = Mathf.SmoothStep(0, 1, (s.trailWidth * 0.9f - DistPoly(pp, c.trail)) / 1.2f) * 0.95f + (Perlin(x * 0.05f, z * 0.05f, off + 500) > 0.66f ? 0.7f : 0f) * 0.7f;
                    // moisture noise (+ damp gully floors) -> moss; open ground / clearings -> grass; conifer stands (same field as tree placement) -> needle litter
                    float moist = Perlin(x * 0.028f, z * 0.028f, off + 700) * 0.75f + Perlin(x * 0.2f, z * 0.2f, off + 710) * 0.25f;
                    float moss = Mathf.SmoothStep(0.52f, 0.72f, moist) + Mathf.SmoothStep(18f, 6f, steep) * 0f; moss = Mathf.Clamp01(moss) * 0.9f;
                    float gnoise = Perlin(x * 0.045f, z * 0.045f, off + 900) * 0.8f + Perlin(x * 0.3f, z * 0.3f, off + 910) * 0.2f; float cd = ClearingDepth(c, pp);
                    float grass = Mathf.SmoothStep(0.6f, 0.76f, gnoise) * 0.9f + Mathf.SmoothStep(-3f, 3f, cd) * 0.8f;
                    float cn = Perlin(x * 0.02f, z * 0.02f, s.seed * 0.31f); float needle = Mathf.SmoothStep(0.45f, 0.62f, cn + (Perlin(x * 0.4f, z * 0.4f, off + 920) - 0.5f) * 0.25f) * 0.9f;
                    float oak = Mathf.SmoothStep(0.5f, 0.35f, cn) * Mathf.SmoothStep(0.45f, 0.62f, Perlin(x * 0.06f, z * 0.06f, off + 930)) * 0.8f;
                    float rem = 1f; var w = new float[7];
                    w[2] = Mathf.Min(rem, Mathf.Clamp01(rockW)); rem -= w[2]; w[1] = Mathf.Min(rem, Mathf.Clamp01(mud)); rem -= w[1];
                    w[3] = Mathf.Min(rem, moss); rem -= w[3]; w[5] = Mathf.Min(rem, Mathf.Clamp01(grass)); rem -= w[5];
                    w[4] = Mathf.Min(rem, needle * 0.9f); rem -= w[4]; w[6] = Mathf.Min(rem, oak); rem -= w[6]; w[0] = rem;
                    for (int k = 0; k < 7; k++) am[j, i, k] = w[k];
                }
            td.SetAlphamaps(0, 0, am);
            AssetDatabase.CreateAsset(td, c.genDir + "/ForestTerrain.asset");
            var go = Terrain.CreateTerrainGameObject(td); go.name = "ForestTerrain"; go.transform.SetParent(root, false);
            go.transform.position = new Vector3(-A / 2, yBase, -A / 2); go.layer = EnvLayer;
            var terrain = go.GetComponent<Terrain>(); terrain.drawInstanced = true; terrain.heightmapPixelError = 3f; terrain.basemapDistance = 90f;
            var tm = new Material(Shader.Find("HDRP/TerrainLit")); AssetDatabase.CreateAsset(tm, c.genDir + "/TerrainLit.mat"); terrain.materialTemplate = tm;
            c.terrain = terrain; c.tRoot = go.transform;
            Physics.SyncTransforms();
            c.H = (x, z) => terrain.SampleHeight(new Vector3(x, 0, z)) + yBase;
        }

        /// <summary>Darkened/tinted copy of a PolyHaven diffuse (the originals are very pale under bright light); saved next to the other generated textures.</summary>
        static Texture2D Tinted(string rel, string name, Color mul)
        {
            string src = Root + "/Textures/PolyHaven/" + rel; string dst = Root + "/Generated/seed_" + CurrentSeed + "/Tex/" + name + ".png";
            var t = new Texture2D(2, 2, TextureFormat.RGBA32, false); t.LoadImage(File.ReadAllBytes(src)); var px = t.GetPixels32();
            for (int i = 0; i < px.Length; i++) px[i] = new Color32((byte)(px[i].r * mul.r), (byte)(px[i].g * mul.g), (byte)(px[i].b * mul.b), 255);
            t.SetPixels32(px); t.Apply(); return SavePng(t, dst, false);
        }
        static int CurrentSeed;

        static Vector3 Normal(Ctx c, float x, float z)
        {
            float e = 0.5f; float hx = c.H(x + e, z) - c.H(x - e, z), hz = c.H(x, z + e) - c.H(x, z - e);
            return new Vector3(-hx / (2 * e), 1, -hz / (2 * e)).normalized;
        }

        // ---------- assets helpers ----------
        static Mesh Save(Ctx c, Mesh m, string name)
        {
            string p = c.genDir + "/Meshes"; if (!AssetDatabase.IsValidFolder(p)) { Directory.CreateDirectory(p); AssetDatabase.Refresh(); }
            AssetDatabase.CreateAsset(m, p + "/" + name + ".asset"); return m;
        }

        static GameObject Obj(string name, Transform parent, Vector3 pos, Quaternion rot, Vector3? scale = null)
        {
            var g = new GameObject(name); g.transform.SetParent(parent, false); g.transform.SetPositionAndRotation(pos, rot); if (scale.HasValue) g.transform.localScale = scale.Value; g.layer = EnvLayer; return g;
        }
        static MeshRenderer Rend(GameObject g, Mesh m, params Material[] mats)
        {
            g.AddComponent<MeshFilter>().sharedMesh = m; var r = g.AddComponent<MeshRenderer>(); r.sharedMaterials = mats; return r;
        }

        // ---------- trees ----------
        /// <summary>Seam for external tree sources (e.g. Tools/TreeGen importer): given (speciesIndex, sizeClass, variant) return a TreeResult
        /// (visual mesh with submesh 0 bark / 1 foliage, collision mesh, flight-band segments, nominal height/trunkR). Null => built-in TreeGen fallback.
        /// Layout and placement (Layout, PlaceTrees) never touch tree geometry except through this call.</summary>
        public static Func<int, int, int, TreeResult> ExternalTreeProvider;
        /// <summary>Optional companion to ExternalTreeProvider (TreeGen, Assets/Drones/TreeGen): called once per placed tree after the default
        /// renderer/MeshCollider are added, before the manifest record; it may replace them and change the scale, which the record then uses.</summary>
        public static Action<GameObject, TreeResult> ExternalTreeInstantiated;

        static TreeResult GetTree(Ctx c, int sp, int sc, int variant)
        {
            string key = sp + "_" + sc + "_" + variant; TreeResult t;
            if (ExternalTreeProvider != null && !c.treeCache.ContainsKey(key)) { var ext = ExternalTreeProvider(sp, sc, variant); if (ext != null) c.treeCache[key] = ext; }
            if (!c.treeCache.TryGetValue(key, out t))
            {
                t = TreeGen.Build(SpeciesTable[sp], c.spec, sc, variant, c.spec.seed * 1009 + sp * 101 + sc * 11 + variant);
                Save(c, t.visual, t.visual.name); Save(c, t.collision, t.collision.name); c.treeCache[key] = t;
            }
            return t;
        }

        static float Hash01(int i) { float f = Mathf.Sin(i * 12.9898f + 78.233f) * 43758.5453f; return f - Mathf.Floor(f); }

        class Stem { public Vector2 p; public float r; }

        static void PlaceTrees(Ctx c, Transform root)
        {
            var s = c.spec; var r = new System.Random(s.seed * 104729 + 3); float A = s.areaSize, half = A * 0.5f - 2f; float ha = A * A / 10000f;
            var group = new GameObject("Trees").transform; group.SetParent(root, false);
            int nTrees = Mathf.RoundToInt(s.stemDensityPerHa * ha), nSap = Mathf.RoundToInt(s.saplingDensityPerHa * ha);
            var dbhs = new List<float>(); for (int i = 0; i < nTrees; i++) dbhs.Add(Mathf.Clamp(s.dbhMeanCm * Mathf.Exp(s.dbhSigma * Geo.Gauss(r) - 0.5f * s.dbhSigma * s.dbhSigma) * 0.01f, 0.08f, 0.85f));
            dbhs.Sort((a, b) => b.CompareTo(a));
            var parents = new List<Vector2>(); int np = Mathf.Max(3, Mathf.RoundToInt(ha * 2.2f)); for (int i = 0; i < np; i++) parents.Add(new Vector2(Geo.R(r, -half, half), Geo.R(r, -half, half)));
            var grid = new Dictionary<long, List<Stem>>(); const float cell = 8f;
            Func<Vector2, long> key = p => ((long)Mathf.FloorToInt((p.x + A) / cell) << 20) + Mathf.FloorToInt((p.y + A) / cell);
            Action<Stem> addStem = st => { long k = key(st.p); List<Stem> l; if (!grid.TryGetValue(k, out l)) grid[k] = l = new List<Stem>(); l.Add(st); };
            Func<Vector2, float, float, bool> free = (p, rad, k0) =>
            {
                int cx = Mathf.FloorToInt((p.x + A) / cell), cz = Mathf.FloorToInt((p.y + A) / cell);
                for (int dx = -1; dx <= 1; dx++) for (int dz = -1; dz <= 1; dz++)
                    { List<Stem> l; if (!grid.TryGetValue(((long)(cx + dx) << 20) + (cz + dz), out l)) continue; foreach (var o in l) if ((o.p - p).magnitude < s.spacingBase * k0 + s.spacingK * k0 * (rad + o.r)) return false; }
                return true;
            };
            Func<Vector2, float, bool> allowed = (p, rad) =>
            {
                if (Mathf.Abs(p.x) > half || Mathf.Abs(p.y) > half) return false;
                if (DistPoly(p, c.trail) < s.trailWidth * 0.5f + rad + 0.5f) return false;
                if (DistPoly(p, c.corridor) < s.corridorClearRadius + rad) return false;
                if (Vector2.Distance(p, c.spawn) < 4f + rad) return false;
                float cd = ClearingDepth(c, p); if (cd > 0f && r.NextDouble() < Mathf.Clamp01(0.15f + cd / 4f)) return false; // soft edge: sparse trees at the margin
                return true;
            };
            int id = 0;
            Action<float, bool> place = (dbh, sapling) =>
            {
                float rad = dbh * 0.5f; Vector2 p = Vector2.zero; bool ok = false;
                for (int a = 0; a < 70 && !ok; a++)
                {
                    if (r.NextDouble() < s.clustering) { var par = parents[r.Next(parents.Count)]; p = par + new Vector2(Geo.Gauss(r), Geo.Gauss(r)) * 11f; } else p = new Vector2(Geo.R(r, -half, half), Geo.R(r, -half, half));
                    if (!allowed(p, rad)) continue; if (!free(p, rad, sapling ? 0.55f : 1f)) continue; ok = true;
                }
                if (!ok) return;
                addStem(new Stem { p = p, r = rad });
                float cn = Perlin(p.x * 0.02f, p.y * 0.02f, s.seed * 0.31f); float pc = Mathf.Clamp01(s.coniferFraction + (cn - 0.5f) * 1.1f);
                bool conifer = r.NextDouble() < pc; bool dead = !sapling && r.NextDouble() < s.deadFraction; int sp;
                if (dead) sp = conifer ? 4 : 5; else sp = conifer ? (r.NextDouble() < 0.6 ? 0 : 1) : (r.NextDouble() < 0.5 ? 2 : 3);
                if (sapling && sp >= 4) sp -= 4 - (conifer ? 0 : 2) + 0; // saplings are never snags
                if (sapling) sp = conifer ? (r.NextDouble() < 0.7 ? 0 : 1) : (r.NextDouble() < 0.5 ? 2 : 3);
                int sc = sapling ? 0 : (dbh < 0.15f ? 0 : (dbh < 0.38f ? 1 : 2));
                float Hh = sapling ? Geo.R(r, 2.5f, 5.2f) : 1.3f + 26f * (1f - Mathf.Exp(-4.5f * dbh)) * Geo.R(r, 0.88f, 1.1f); // dbh in m: 0.3 m -> ~20 m
                float sx = sapling ? Geo.R(r, 0.28f, 0.45f) : Mathf.Clamp(rad / TreeGen.NominalR(sc), 0.6f, 1.6f);
                float sy = sapling ? Hh / TreeGen.NominalH(0) : Mathf.Clamp(Hh / TreeGen.NominalH(sc), 0.7f, 1.4f);
                int variant = r.Next(sapling ? 3 : (dead ? 1 : 2));
                var tr = GetTree(c, sp, sc, variant);
                float y = c.H(p.x, p.y) - 0.04f; float yaw = Geo.R(r, 0, 360f);
                var g = Obj((sapling ? "Sapling_" : "Tree_") + id.ToString("D4") + "_" + tr.species.name, group, new Vector3(p.x, y, p.y), Quaternion.Euler(0, yaw, 0), new Vector3(sx, sy, sx));
                int nv = tr.species.conifer ? c.needleVar.Length : c.leafVar.Length;
                float hue = Mathf.Clamp01(Perlin(p.x * 0.04f, p.y * 0.04f, s.seed * 0.19f) * 0.55f + Hash01(id * 7 + (sapling ? 3 : 0)) * 0.45f); int vi = Mathf.Min(nv - 1, (int)(hue * nv));
                Rend(g, tr.visual, new[] { c.bark[tr.species.bark], tr.species.conifer ? c.needleVar[vi] : c.leafVar[vi] });
                var mc = g.AddComponent<MeshCollider>(); mc.sharedMesh = tr.collision;
                if (ExternalTreeInstantiated != null) { ExternalTreeInstantiated(g, tr); sx = g.transform.localScale.x; sy = g.transform.localScale.y; }
                var rec = new TreeRec { id = id, species = tr.species.name, sizeClass = sc, variant = variant, pos = g.transform.position, yaw = yaw, sx = sx, sy = sy, dbhCm = tr.trunkR * 2 * sx * 100f, heightM = tr.height * sy, dead = tr.species.dead };
                // branch obstacles in the flight band (world space)
                var q = Quaternion.Euler(0, yaw, 0); int nb = 0; var sb = new StringBuilder();
                foreach (var sg in tr.segs)
                {
                    Vector3 a = g.transform.position + q * new Vector3(sg.a.x * sx, sg.a.y * sy, sg.a.z * sx), b = g.transform.position + q * new Vector3(sg.b.x * sx, sg.b.y * sy, sg.b.z * sx);
                    float h0 = (a.y + b.y) * 0.5f - y; float rr = sg.r * sx; if (h0 < 0.3f || h0 > 5.5f || rr < 0.003f) continue;
                    if (nb++ > 0) sb.Append(','); sb.Append(string.Format(CI, "[{0:F2},{1:F2},{2:F2},{3:F2},{4:F2},{5:F2},{6:F4}]", a.x, a.y, a.z, b.x, b.y, b.z, rr));
                }
                rec.branchCount = nb; c.trees.Add(rec); c.branchLines.Add("{\"tree\":" + id + ",\"segments\":[" + sb + "]}"); id++;
            };
            foreach (var d in dbhs) place(d, false);
            for (int i = 0; i < nSap; i++) place(Geo.R(r, 0.03f, 0.07f), true);
        }

        // ---------- understory, logs, rocks ----------
        static Vector2 RandOpen(Ctx c, System.Random r, float minTrail = 1.5f)
        {
            float half = c.spec.areaSize * 0.5f - 3f;
            for (int i = 0; i < 40; i++)
            {
                var p = new Vector2(Geo.R(r, -half, half), Geo.R(r, -half, half));
                if (DistPoly(p, c.trail) < minTrail) continue; if (Vector2.Distance(p, c.spawn) < 2.5f) continue;
                bool nearTree = false; foreach (var t in c.trees) if (Mathf.Abs(t.pos.x - p.x) < 1.5f && Mathf.Abs(t.pos.z - p.y) < 1.5f && Vector2.Distance(new Vector2(t.pos.x, t.pos.z), p) < t.dbhCm * 0.005f + 0.7f) { nearTree = true; break; }
                if (!nearTree) return p;
            }
            return Vector2.zero;
        }

        static Quaternion GroundRot(Ctx c, Vector3 p, float yaw)
        {
            return Quaternion.FromToRotation(Vector3.up, Normal(c, p.x, p.z)) * Quaternion.Euler(0, yaw, 0);
        }

        static void PlaceUndergrowth(Ctx c, Transform root)
        {
            var s = c.spec; var r = new System.Random(s.seed * 15485863 + 5); float ha = s.areaSize * s.areaSize / 10000f;
            var g = new GameObject("Understory").transform; g.SetParent(root, false);
            // shared variant meshes
            var ferns = new List<Mesh>(); for (int i = 0; i < 8; i++) ferns.Add(Save(c, Props.Fern(new System.Random(s.seed + 700 + i), 0.9f, i % 4).ToMesh("Fern" + i, true), "Fern" + i));
            var shrubs = new List<Mesh>(); for (int i = 0; i < 4; i++) shrubs.Add(Save(c, Props.Shrub(new System.Random(s.seed + 800 + i), 1.6f, s.canopyOpenness).ToMesh("Shrub" + i, true), "Shrub" + i));
            var shrubCol = new List<Mesh>(); for (int i = 0; i < 4; i++) shrubCol.Add(null);
            var litter = new List<Mesh>(); for (int i = 0; i < 4; i++) litter.Add(Save(c, Props.LitterPatch(new System.Random(s.seed + 900 + i), 1.2f, 46).ToMesh("Litter" + i, true), "Litter" + i));
            int nFern = Mathf.RoundToInt(s.fernDensity * 420f * ha), nShrub = Mathf.RoundToInt(s.undergrowthDensity * 70f * ha), nLit = 0; // ground-hugging litter cards showed as flat rectangles; litter now comes from terrain layers
            for (int i = 0; i < nFern; i++)
            {
                Vector2 p; if (r.NextDouble() < 0.6) { var tp = c.trees.Count > 0 ? c.trees[r.Next(c.trees.Count)].pos : Vector3.zero; p = new Vector2(tp.x, tp.z) + new Vector2(Geo.Gauss(r), Geo.Gauss(r)) * 2.5f; if (DistPoly(p, c.trail) < 1.2f || Mathf.Abs(p.x) > s.areaSize / 2 - 2 || Mathf.Abs(p.y) > s.areaSize / 2 - 2) continue; } else p = RandOpen(c, r, 1.2f);
                if (p == Vector2.zero) continue; float y = c.H(p.x, p.y); float sc = Geo.R(r, 0.7f, 1.35f);
                int fi = r.Next(ferns.Count); if (Perlin(p.x * 0.03f, p.y * 0.03f, s.seed * 0.5f) > 0.62f && r.NextDouble() < 0.7) fi = 1 + 4 * r.Next(2); // bracken drifts in open patches
                float jx = Geo.R(r, 0.8f, 1.25f), jy = Geo.R(r, 0.7f, 1.3f);
                var go = Obj("Fern", g, new Vector3(p.x, y - 0.02f, p.y), GroundRot(c, new Vector3(p.x, y, p.y), Geo.R(r, 0, 360)) * Quaternion.Euler(Geo.R(r, -6, 6), 0, Geo.R(r, -6, 6)), new Vector3(sc * jx, sc * jy, sc * jx)); go.layer = 0;
                Rend(go, ferns[fi], c.ferns[fi % 4]); c.props.Add(new PropRec { kind = "fern", pos = go.transform.position, size = sc });
            }
            for (int i = 0; i < nShrub; i++)
            {
                var p = RandOpen(c, r, 1.6f); if (p == Vector2.zero) continue; float y = c.H(p.x, p.y); float sc = Geo.R(r, 0.7f, 1.5f);
                var go = Obj("Shrub", g, new Vector3(p.x, y - 0.03f, p.y), Quaternion.Euler(0, Geo.R(r, 0, 360), 0), Vector3.one * sc);
                var m = shrubs[r.Next(shrubs.Count)]; Rend(go, m, c.shrubStem, c.leaves);
                c.props.Add(new PropRec { kind = "shrub", pos = go.transform.position, size = sc });
            }
            for (int i = 0; i < nLit; i++)
            {
                var p = RandOpen(c, r, 0.5f); if (p == Vector2.zero) continue; float y = c.H(p.x, p.y);
                var go = Obj("LeafLitter", g, new Vector3(p.x, y, p.y), GroundRot(c, new Vector3(p.x, y, p.y), Geo.R(r, 0, 360)), Vector3.one * Geo.R(r, 0.8f, 1.6f)); go.layer = 0;
                var rr = Rend(go, litter[r.Next(litter.Count)], c.litter); rr.shadowCastingMode = ShadowCastingMode.Off;
            }
            // grass tufts in grass patches / clearings, fallen-branch scatter
            {
                var gm = new List<Mesh>(); for (int i = 0; i < 4; i++) gm.Add(Save(c, Props.GrassTuft(new System.Random(s.seed + 1500 + i), 0.55f).ToMesh("Grass" + i, true), "Grass" + i));
                var gr = new GameObject("Grass").transform; gr.SetParent(root, false); float goff = s.seed * 0.137f; int nG = Mathf.RoundToInt(s.grassTuftsPerHa * ha), gTries = 0;
                for (int i = 0; i < nG * 3 && c.nGrass < nG; i++)
                {
                    gTries++; var p = RandOpen(c, r, 0.8f); if (p == Vector2.zero) continue;
                    float gn = Perlin(p.x * 0.045f, p.y * 0.045f, goff + 900) * 0.8f + Perlin(p.x * 0.3f, p.y * 0.3f, goff + 910) * 0.2f; float cdp = ClearingDepth(c, p);
                    float pa = Mathf.SmoothStep(0.6f, 0.76f, gn) + Mathf.SmoothStep(-3f, 3f, cdp) * 0.8f; if (r.NextDouble() > pa) continue;
                    float y = c.H(p.x, p.y); float sc = Geo.R(r, 0.6f, 1.5f);
                    var go = Obj("GrassTuft", gr, new Vector3(p.x, y - 0.01f, p.y), GroundRot(c, new Vector3(p.x, y, p.y), Geo.R(r, 0, 360)), new Vector3(sc, sc * Geo.R(r, 0.7f, 1.3f), sc)); go.layer = 0;
                    var gR = Rend(go, gm[r.Next(gm.Count)], c.grass); gR.shadowCastingMode = ShadowCastingMode.Off; c.nGrass++;
                }
                var bm = new List<Mesh>(); for (int i = 0; i < 10; i++) bm.Add(Save(c, Props.FallenBranch(new System.Random(s.seed + 1700 + i), Geo.R(r, 0.5f, 2.4f), Geo.R(r, 0.012f, 0.04f)).ToMesh("FallenBranch" + i, true), "FallenBranch" + i));
                var br = new GameObject("FallenBranches").transform; br.SetParent(root, false); int nB = Mathf.RoundToInt(s.fallenBranchPerHa * ha);
                for (int i = 0; i < nB; i++)
                {
                    Vector2 p; if (r.NextDouble() < 0.5 && c.trees.Count > 0) { var tp = c.trees[r.Next(c.trees.Count)].pos; p = new Vector2(tp.x, tp.z) + new Vector2(Geo.Gauss(r), Geo.Gauss(r)) * 2.2f; if (Mathf.Abs(p.x) > s.areaSize / 2 - 2 || Mathf.Abs(p.y) > s.areaSize / 2 - 2) continue; } else p = RandOpen(c, r, 0.3f);
                    if (p == Vector2.zero) continue; float y = c.H(p.x, p.y);
                    var go = Obj("FallenBranch", br, new Vector3(p.x, y + 0.005f, p.y), GroundRot(c, new Vector3(p.x, y, p.y), Geo.R(r, 0, 360)), Vector3.one * Geo.R(r, 0.8f, 1.3f)); go.layer = 0;
                    Rend(go, bm[r.Next(bm.Count)], c.branchMats[r.Next(2)]); c.nBranch++;
                }
            }
            // logs
            var lg = new GameObject("Logs").transform; lg.SetParent(root, false);
            for (int i = 0; i < s.logCount; i++)
            {
                var p = RandOpen(c, r, 2.5f); if (p == Vector2.zero) continue; float len = Geo.R(r, 3f, 9f), rad = Geo.R(r, 0.09f, 0.34f); bool broad = r.NextDouble() < 0.5; float yaw = Geo.R(r, 0, 360);
                var go = MakeLog(c, lg, "Log_" + i.ToString("D2"), new System.Random(s.seed * 31 + i), len, rad, broad, p, yaw);
                c.props.Add(new PropRec { kind = "log", pos = go.transform.position, yaw = yaw, size = len, note = "radius=" + rad.ToString("F2", CI) });
            }
            var rk = new GameObject("Rocks").transform; rk.SetParent(root, false);
            for (int i = 0; i < s.rockCount; i++)
            {
                var p = RandOpen(c, r, 1.2f); if (p == Vector2.zero) continue; float sz = Mathf.Clamp(0.22f * Mathf.Exp(0.7f * Geo.Gauss(r)), 0.12f, 1.4f);
                var mb = Geo.Rock(new System.Random(s.seed * 17 + i), new Vector3(sz * Geo.R(r, 0.9f, 1.5f), sz * Geo.R(r, 0.6f, 1f), sz * Geo.R(r, 0.9f, 1.3f)), 1f, true, i);
                var mesh = Save(c, mb.ToMesh("Rock" + i, true), "Rock" + i); var col = Save(c, mb.ToCollisionMesh("RockCol" + i, 0), "RockCol" + i);
                float y = c.H(p.x, p.y) + sz * 0.12f;
                var go = Obj("Rock_" + i.ToString("D2"), rk, new Vector3(p.x, y, p.y), Quaternion.Euler(Geo.R(r, -8, 8), Geo.R(r, 0, 360), Geo.R(r, -8, 8)));
                Rend(go, mesh, c.rock, c.moss); go.AddComponent<MeshCollider>().sharedMesh = col;
                c.props.Add(new PropRec { kind = "rock", pos = go.transform.position, size = sz });
            }
        }

        static GameObject MakeLog(Ctx c, Transform parent, string name, System.Random r, float len, float rad, bool broad, Vector2 p, float yaw)
        {
            var mb = Props.Log(r, len, rad, broad);
            var mesh = Save(c, mb.ToMesh(name, true), name); var col = Save(c, mb.ToCollisionMesh(name + "_col", 0, 2), name + "_col");
            var q = Quaternion.Euler(0, yaw, 0); Vector3 fwd = q * Vector3.right; Vector3 a = new Vector3(p.x, 0, p.y), b = a + fwd * len;
            float ya = c.H(a.x, a.z), yb = c.H(b.x, b.z); float pitch = -Mathf.Atan2(yb - ya, len) * Mathf.Rad2Deg;
            var go = Obj(name, parent, new Vector3(p.x, Mathf.Min(ya, yb) + rad * 0.55f + Mathf.Abs(yb - ya) * 0.35f, p.y), q * Quaternion.Euler(0, 0, -pitch));
            Rend(go, mesh, c.bark[broad ? 5 : 4], c.moss, c.wood); go.AddComponent<MeshCollider>().sharedMesh = col; return go;
        }

        // ---------- search targets ----------
        static void PlaceTargets(Ctx c, Transform root)
        {
            var s = c.spec; var r = new System.Random(s.seed * 2750159 + 9); var tg = new GameObject("SearchTargets").transform; tg.SetParent(root, false);
            var poses = new[] { Humanoid.Pose.SupineBentKnee, Humanoid.Pose.SittingAgainstTree, Humanoid.Pose.ProneArmExtended };
            var cover = new GameObject("TargetCover").transform; cover.SetParent(root, false);
            var stickMesh = new List<Mesh>(); for (int i = 0; i < 5; i++) stickMesh.Add(Save(c, Props.Stick(new System.Random(s.seed + 1200 + i), Geo.R(r, 0.9f, 2.0f), 0.012f).ToMesh("TStick" + i, false), "TStick" + i));
            var fernM = AssetDatabase.LoadAllAssetsAtPath(c.genDir + "/Meshes/Fern0.asset").OfType<Mesh>().FirstOrDefault();
            int placed = 0;
            for (int ti = 0; ti < Mathf.Min(3, s.targetCount); ti++)
            {
                var pose = poses[ti % 3]; Vector2 p = Vector2.zero; float yaw = 0; bool ok = false; Vector3 treePos = Vector3.zero; float treeR = 0;
                for (int a = 0; a < 400 && !ok; a++)
                {
                    int ci = r.Next(c.corridor.Count - 1); var cp = c.corridor[ci] + new Vector2(Geo.R(r, -14f, 14f), Geo.R(r, -14f, 14f));
                    float d = Vector2.Distance(cp, c.spawn); if (d < s.targetDistanceFromSpawn.x || d > s.targetDistanceFromSpawn.y) continue;
                    if (Mathf.Abs(cp.x) > s.areaSize / 2 - 5 || Mathf.Abs(cp.y) > s.areaSize / 2 - 5 || DistPoly(cp, c.trail) < 2f || DistPoly(cp, c.corridor) < 1.8f) continue;
                    if (c.targets.Any(o => Vector2.Distance(new Vector2(o.pos.x, o.pos.z), cp) < 15f)) continue;
                    if (pose == Humanoid.Pose.SittingAgainstTree)
                    {
                        TreeRec best = null; float bd = 1e9f; foreach (var t in c.trees) { if (t.dbhCm < 22 || t.dbhCm > 60 || t.dead) continue; float dd = Vector2.Distance(new Vector2(t.pos.x, t.pos.z), cp); if (dd < bd) { bd = dd; best = t; } }
                        if (best == null || bd > 10f) continue; treePos = best.pos; treeR = best.dbhCm * 0.005f * 1.15f; float ang = Geo.R(r, 0, 6.283f);
                        p = new Vector2(treePos.x, treePos.z) + new Vector2(Mathf.Cos(ang), Mathf.Sin(ang)) * (treeR + 0.22f); yaw = Mathf.Atan2(-Mathf.Sin(ang), Mathf.Cos(ang)) * Mathf.Rad2Deg; // local +X points away from the tree
                    }
                    else
                    {
                        bool clear = true; foreach (var t in c.trees) if (Vector2.Distance(new Vector2(t.pos.x, t.pos.z), cp) < t.dbhCm * 0.005f + 1.6f) { clear = false; break; }
                        if (!clear) continue; p = cp; yaw = Geo.R(r, 0, 360);
                    }
                    ok = true;
                }
                if (!ok) { Debug.LogWarning("[DroneForest] could not place target " + ti); continue; }
                var mb = Humanoid.Build(pose, new System.Random(s.seed + 5000 + ti));
                var mesh = Save(c, mb.ToMesh("Target" + ti, true), "Target" + ti); var col = Save(c, mb.ToCollisionMesh("TargetCol" + ti, 0, 1, 2, 3, 4), "TargetCol" + ti);
                float y = c.H(p.x, p.y); Vector3 n = Normal(c, p.x, p.y); Quaternion q = Quaternion.FromToRotation(Vector3.up, Vector3.Lerp(Vector3.up, n, 0.8f)) * Quaternion.Euler(0, yaw + (pose == Humanoid.Pose.SittingAgainstTree ? 0 : 0), 0);
                string nm = "SearchTarget_" + (ti + 1).ToString("D2") + "_" + pose;
                var go = Obj(nm, tg, new Vector3(p.x, y - 0.015f, p.y), q); Rend(go, mesh, c.skin, c.jacket, c.trousers, c.boots, c.hood);
                go.AddComponent<MeshCollider>().sharedMesh = col; var mk = go.AddComponent<DroneForestMarker>(); mk.kind = DroneForestMarker.Kind.Target;
                var rec = new TargetRec { id = ti + 1, pose = pose.ToString(), pos = go.transform.position, yaw = go.transform.eulerAngles.y, occlusion = s.targetOcclusion, name = nm };
                // occluders scale with spec.targetOcclusion
                float o = s.targetOcclusion; int sticks = Mathf.RoundToInt(3 + 16 * o), mounds = o > 0.25f ? Mathf.RoundToInt(o * 3) : 0, fcount = Mathf.RoundToInt(2 + 7 * o);
                for (int k = 0; k < sticks; k++)
                {
                    Vector3 lp = new Vector3(Geo.R(r, -0.7f, 0.9f), Geo.R(r, 0.1f, 0.42f), Geo.R(r, -0.4f, 0.4f)); if (pose == Humanoid.Pose.SittingAgainstTree) lp = new Vector3(Geo.R(r, -0.1f, 1.0f), Geo.R(r, 0.15f, 0.7f), Geo.R(r, -0.4f, 0.4f));
                    var sg = Obj("CoverStick", cover, go.transform.TransformPoint(lp), Quaternion.Euler(Geo.R(r, -50, 25), Geo.R(r, 0, 360), Geo.R(r, -20, 20)));
                    Rend(sg, stickMesh[r.Next(stickMesh.Count)], c.bark[4]); var smc = sg.AddComponent<MeshCollider>(); smc.sharedMesh = stickMesh[0]; smc.convex = false;
                }
                for (int k = 0; k < mounds; k++)
                {
                    var mm = Geo.Rock(new System.Random(s.seed + 6000 + ti * 10 + k), new Vector3(Geo.R(r, 0.45f, 0.8f), Geo.R(r, 0.14f, 0.22f), Geo.R(r, 0.3f, 0.5f)), 0.5f, false, k);
                    var me = Save(c, mm.ToMesh("Mound" + ti + "_" + k, true), "Mound" + ti + "_" + k);
                    Vector3 lp = new Vector3(Geo.R(r, -0.55f, 0.35f), 0.05f, Geo.R(r, -0.2f, 0.2f)); if (pose == Humanoid.Pose.SittingAgainstTree) lp = new Vector3(Geo.R(r, 0.3f, 0.8f), 0.04f, Geo.R(r, -0.2f, 0.2f));
                    var mo = Obj("CoverMound", cover, go.transform.TransformPoint(lp), go.transform.rotation * Quaternion.Euler(0, Geo.R(r, 0, 360), 0)); Rend(mo, me, c.mound);
                    mo.AddComponent<MeshCollider>().sharedMesh = Save(c, mm.ToCollisionMesh("MoundCol" + ti + "_" + k, 0), "MoundCol" + ti + "_" + k);
                }
                for (int k = 0; k < fcount; k++)
                {
                    float ang = (k + Geo.R(r, -0.3f, 0.3f)) / fcount * 6.283f; float dist = Geo.R(r, 0.35f, 0.95f);
                    Vector3 wp = go.transform.TransformPoint(new Vector3(0.2f, 0, 0)) + new Vector3(Mathf.Cos(ang) * dist * 1.3f, 0, Mathf.Sin(ang) * dist);
                    var fg = Obj("CoverFern", cover, new Vector3(wp.x, c.H(wp.x, wp.z) - 0.02f, wp.z), Quaternion.Euler(0, Geo.R(r, 0, 360), 0), Vector3.one * Geo.R(r, 0.9f, 1.4f)); fg.layer = 0;
                    Rend(fg, fernM, c.fern);
                }
                if (o > 0.55f && pose != Humanoid.Pose.SittingAgainstTree)
                {
                    var lgo = MakeLog(c, cover, "CoverLog_" + ti, new System.Random(s.seed + 7000 + ti), 4.5f, 0.15f, ti % 2 == 0, new Vector2(go.transform.position.x - 0.45f * Mathf.Cos(yaw * Mathf.Deg2Rad) + 1.9f * Mathf.Sin(yaw * Mathf.Deg2Rad), go.transform.position.z + 0.45f * Mathf.Sin(yaw * Mathf.Deg2Rad) - 1.9f * Mathf.Cos(yaw * Mathf.Deg2Rad)), yaw + 90f);
                    rec.log = true;
                }
                if (o > 0.4f && pose == Humanoid.Pose.SittingAgainstTree)
                {
                    var sh = Obj("CoverShrub", cover, go.transform.TransformPoint(new Vector3(1.0f, 0, 0.4f)), Quaternion.identity, Vector3.one * 0.9f);
                    var shm = AssetDatabase.LoadAssetAtPath<Mesh>(c.genDir + "/Meshes/Shrub0.asset"); Rend(sh, shm, c.shrubStem, c.leaves);
                }
                rec.coverSticks = sticks; rec.coverMounds = mounds; rec.coverFerns = fcount;
                var mkc = go.GetComponent<DroneForestMarker>(); c.targets.Add(rec); placed++;
            }
        }

        // ---------- mid-story: shrubs and young conifers 1-4 m (stems have colliders and are in the manifest) ----------
        class MidProto { public Mesh visual, collision; public List<BranchSeg> segs; public float height, crownR; public bool conifer; }

        static void PlaceMidstory(Ctx c, Transform root)
        {
            var s = c.spec; var r = new System.Random(s.seed * 193939 + 17); float ha = s.areaSize * s.areaSize / 10000f; float half = s.areaSize * 0.5f - 2f;
            var g = new GameObject("Midstory").transform; g.SetParent(root, false);
            var protos = new List<MidProto>();
            for (int i = 0; i < 12; i++)
            {
                bool con = i >= 6; var segs = new List<BranchSeg>(); float cr; float h = con ? Geo.R(r, 1.6f, 4.0f) : Geo.R(r, 1.6f, 3.8f);
                var mb = con ? Props.YoungConifer(new System.Random(s.seed + 2100 + i), h, segs, out cr) : Props.MidShrub(new System.Random(s.seed + 2200 + i), h, segs, out cr);
                var pr = new MidProto { height = h, crownR = cr, segs = segs, conifer = con };
                pr.visual = Save(c, mb.ToMesh((con ? "YoungConifer" : "MidShrub") + i, true), (con ? "YoungConifer" : "MidShrub") + i);
                pr.collision = Save(c, mb.ToCollisionMesh((con ? "YoungConiferCol" : "MidShrubCol") + i, 0), (con ? "YoungConiferCol" : "MidShrubCol") + i); protos.Add(pr);
            }
            int n = Mathf.RoundToInt(s.midstoryDensityPerHa * ha), id = 0;
            for (int i = 0; i < n * 4 && id < n; i++)
            {
                var p = new Vector2(Geo.R(r, -half, half), Geo.R(r, -half, half));
                float dens = 0.3f + 0.7f * Perlin(p.x * 0.05f, p.y * 0.05f, s.seed * 0.43f); float cd = ClearingDepth(c, p); if (cd > 0f) dens *= 0.15f; if (r.NextDouble() > dens) continue;
                float cn = Perlin(p.x * 0.02f, p.y * 0.02f, s.seed * 0.31f); bool con = r.NextDouble() < Mathf.Clamp01(s.coniferFraction + (cn - 0.5f) * 1.1f);
                var cand = protos.Where(pp => pp.conifer == con).ToList(); var pr = cand[r.Next(cand.Count)]; float sc = Geo.R(r, 0.85f, 1.3f); float crown = pr.crownR * sc;
                if (DistPoly(p, c.corridor) < s.corridorClearRadius + crown) continue; if (DistPoly(p, c.trail) < s.trailWidth * 0.5f + crown * 0.5f + 0.2f) continue;
                if (Vector2.Distance(p, c.spawn) < 3.5f + crown) continue;
                bool bad = false; foreach (var t in c.targets) if (Vector2.Distance(new Vector2(t.pos.x, t.pos.z), p) < 1.8f + crown * 0.5f) { bad = true; break; }
                if (bad) continue;
                foreach (var t in c.trees) { if (Mathf.Abs(t.pos.x - p.x) > 2.5f || Mathf.Abs(t.pos.z - p.y) > 2.5f) continue; if (Vector2.Distance(new Vector2(t.pos.x, t.pos.z), p) < t.dbhCm * 0.005f + 0.45f) { bad = true; break; } }
                if (bad) continue;
                float y = c.H(p.x, p.y) - 0.04f; float yaw = Geo.R(r, 0, 360);
                string kind = pr.conifer ? "young_conifer" : "shrub";
                var go = Obj("Mid_" + id.ToString("D4") + "_" + kind, g, new Vector3(p.x, y, p.y), Quaternion.Euler(0, yaw, 0), Vector3.one * sc);
                int nvar = pr.conifer ? c.needleVar.Length : c.leafVar.Length; float hue = Mathf.Clamp01(Perlin(p.x * 0.04f, p.y * 0.04f, s.seed * 0.19f) * 0.55f + Hash01(id * 13 + 5) * 0.45f); int vi = Mathf.Min(nvar - 1, (int)(hue * nvar));
                Rend(go, pr.visual, pr.conifer ? c.bark[0] : c.shrubStem, pr.conifer ? c.needleVar[vi] : c.leafVar[vi]);
                go.AddComponent<MeshCollider>().sharedMesh = pr.collision;
                var q = Quaternion.Euler(0, yaw, 0); var sb = new StringBuilder(); int nb = 0;
                foreach (var sg in pr.segs)
                {
                    Vector3 a = go.transform.position + q * (sg.a * sc), b = go.transform.position + q * (sg.b * sc); float h0 = (a.y + b.y) * 0.5f - y; float rr = sg.r * sc; if (h0 < 0.3f || h0 > 5.5f || rr < 0.004f) continue;
                    if (nb++ > 0) sb.Append(','); sb.Append(string.Format(CI, "[{0:F2},{1:F2},{2:F2},{3:F2},{4:F2},{5:F2},{6:F4}]", a.x, a.y, a.z, b.x, b.y, b.z, rr));
                }
                c.mids.Add(new MidRec { id = id, kind = kind, pos = go.transform.position, yaw = yaw, scale = sc, height = pr.height * sc, crownR = crown, segCount = nb, segs = sb.ToString() }); id++;
            }
        }

        // ---------- surrounding landscape: hills to the horizon + low-LOD distant forest (visual only, layer EnvironmentFar, no colliders, not in manifest) ----------
        static void EnsureFarLayer()
        {
            try
            {
                var so = new SerializedObject(AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/TagManager.asset")[0]); var l = so.FindProperty("layers").GetArrayElementAtIndex(FarLayer);
                if (string.IsNullOrEmpty(l.stringValue)) { l.stringValue = "EnvironmentFar"; so.ApplyModifiedProperties(); }
                else if (l.stringValue != "EnvironmentFar") Debug.LogWarning("[DroneForest] layer " + FarLayer + " already named " + l.stringValue);
            }
            catch (Exception e) { Debug.LogWarning("[DroneForest] could not name far layer: " + e.Message); }
        }

        static void BuildFarLandscape(Ctx c, Transform root)
        {
            var s = c.spec; if (!s.surroundingLandscape) return; EnsureFarLayer();
            float A = s.areaSize, E = A * 0.5f, R = s.landscapeRadius; float off = s.seed * 0.137f; var rng = new System.Random(s.seed * 99991 + 31);
            var far = new GameObject("SurroundingLandscape").transform; far.SetParent(root, false);
            float edgeMean = 0f; for (int i = 0; i < 16; i++) { float t = i / 15f * A - E; edgeMean += c.H(t, E - 0.5f) + c.H(t, -E + 0.5f); } edgeMean /= 32f;
            Func<float, float, float> dEdge = (x, z) => { float dx = Mathf.Max(0, Mathf.Abs(x) - E), dz = Mathf.Max(0, Mathf.Abs(z) - E); return Mathf.Sqrt(dx * dx + dz * dz); };
            Func<float, float, float> ground = (x, z) =>
            {
                float d = dEdge(x, z); float ex = Mathf.Clamp(x, -E + 0.3f, E - 0.3f), ez = Mathf.Clamp(z, -E + 0.3f, E - 0.3f);
                if (Mathf.Abs(x) < E - 0.01f && Mathf.Abs(z) < E - 0.01f) return -40f;
                float e = Mathf.Lerp(c.H(ex, ez), edgeMean, Mathf.SmoothStep(0, 1, d / 220f));
                float hill = (Perlin(x * 0.0011f, z * 0.0011f, off + 1100) - 0.35f) * 140f * Mathf.SmoothStep(30f, 1400f, d) + (Perlin(x * 0.006f, z * 0.006f, off + 1200) - 0.5f) * 26f * Mathf.SmoothStep(20f, 500f, d)
                    + (Perlin(x * 0.03f, z * 0.03f, off + 1300) - 0.5f) * 4f * Mathf.SmoothStep(20f, 150f, d);
                return e - 0.05f + hill * Mathf.Min(1f, d / 6f) * (hill > 0 ? 1f : 0.6f);
            };
            // non-uniform grid: fine at the patch edge, geometrically coarser outward
            var ds = new List<float> { 0 }; for (float d = 4f; d < R; d += Mathf.Max(4f, d * 0.14f)) ds.Add(d); ds.Add(R);
            var xs = new List<float>(); for (int i = ds.Count - 1; i >= 1; i--) xs.Add(-(E + ds[i])); xs.Add(-E); xs.Add(-E + 6f); xs.Add(0f); xs.Add(E - 6f); xs.Add(E); for (int i = 1; i < ds.Count; i++) xs.Add(E + ds[i]);
            int nx = xs.Count; var vs = new Vector3[nx * nx]; var uvs = new Vector2[nx * nx]; var nrm = new Vector3[nx * nx];
            for (int j = 0; j < nx; j++) for (int i = 0; i < nx; i++) { float x = xs[i], z = xs[j]; vs[j * nx + i] = new Vector3(x, ground(x, z), z); uvs[j * nx + i] = new Vector2(x, z) / 45f; }
            var tris = new List<int>();
            for (int j = 0; j < nx - 1; j++) for (int i = 0; i < nx - 1; i++)
                {
                    int a = j * nx + i, b = a + 1, d2 = a + nx, e2 = d2 + 1; tris.Add(a); tris.Add(d2); tris.Add(e2); tris.Add(a); tris.Add(e2); tris.Add(b);
                }
            var fm = new Mesh { name = "FarGround", indexFormat = IndexFormat.UInt32 }; fm.vertices = vs; fm.uv = uvs; fm.SetTriangles(tris, 0); fm.RecalculateNormals(); fm.RecalculateTangents(); fm.RecalculateBounds(); Save(c, fm, "FarGround");
            var gg = new GameObject("FarGround"); gg.transform.SetParent(far, false); gg.layer = FarLayer; Rend(gg, fm, c.farGround).shadowCastingMode = ShadowCastingMode.Off;

            // distant forest: merged low-poly trees per 160 m tile; broadleaf = lumpy ellipsoid crowns, conifer = stacked cones; trunks as prisms
            float reach = s.distantForestReach; int tileN = 0; float tileSz = 160f; int tilesPerSide = Mathf.CeilToInt((A + 2 * reach) / tileSz);
            var tiles = new Dictionary<int, MB>();
            Func<float, float, MB> tileOf = (x, z) => { int tx = Mathf.FloorToInt((x + E + reach) / tileSz), tz = Mathf.FloorToInt((z + E + reach) / tileSz); int k = tz * tilesPerSide + tx; MB m; if (!tiles.TryGetValue(k, out m)) { m = new MB(3); tiles[k] = m; } return m; };
            for (int pass = 0; pass < 2; pass++)
            {
                float cs = pass == 0 ? 6.0f : 9.5f, d0 = pass == 0 ? 0f : 150f, d1 = pass == 0 ? 150f : reach, crScale = pass == 0 ? 1f : 1.45f; float lim = E + reach;
                for (float gz = -lim; gz < lim; gz += cs) for (float gx = -lim; gx < lim; gx += cs)
                    {
                        float x = gx + Geo.R(rng, 0, cs), z = gz + Geo.R(rng, 0, cs); float d = dEdge(x, z); if (d < d0 + 1.5f * (pass == 0 ? 1 : 0) || d >= d1 || d <= 1.5f) continue;
                        if (Mathf.Abs(x) > lim || Mathf.Abs(z) > lim) continue;
                        float dens = 0.55f + 0.45f * Perlin(x * 0.02f, z * 0.02f, off + 1500); if (rng.NextDouble() > dens) continue;
                        float cn = Perlin(x * 0.012f, z * 0.012f, s.seed * 0.31f); bool con = rng.NextDouble() < Mathf.Clamp01(s.coniferFraction + (cn - 0.5f) * 1.3f);
                        float y = ground(x, z); MB mb = tileOf(x, z); float hgt = Geo.R(rng, 14f, 24f) * (con ? 1.1f : 1f); float cr = Geo.R(rng, 3.2f, 5.0f) * crScale * (con ? 0.7f : 1f); float th = hgt * (con ? 0.18f : 0.35f);
                        Vector2 uvo = new Vector2(Geo.R(rng, 0, 1), Geo.R(rng, 0, 1));
                        // trunk prism
                        { float tr = 0.22f + hgt * 0.008f; int i0 = mb.Count; for (int k = 0; k < 4; k++) { float an = k * 1.5708f + 0.7f; var dir = new Vector3(Mathf.Cos(an), 0, Mathf.Sin(an)); mb.Add(new Vector3(x, y - 0.3f, z) + dir * tr, dir, new Vector2(k * 0.25f, 0)); mb.Add(new Vector3(x, y + th + (con ? hgt * 0.1f : 2f), z) + dir * tr * 0.5f, dir, new Vector2(k * 0.25f, 4)); }
                          for (int k = 0; k < 4; k++) { int a = i0 + k * 2, b = i0 + ((k + 1) % 4) * 2; mb.Tri(2, a, a + 1, b + 1); mb.Tri(2, a, b + 1, b); } }
                        if (con)
                        {
                            int tiersN = 3; for (int t = 0; t < tiersN; t++)
                            {
                                float f0 = t / (float)tiersN; float yb = y + th + (hgt - th) * f0 * 0.82f, yt = yb + (hgt - th) * (0.5f - 0.12f * t), rad = cr * (1.15f - 0.38f * t); int sides = 8; int i0 = mb.Count;
                                for (int k = 0; k < sides; k++) { float an = (k + 0.5f * t) / sides * 6.283f; var dir = new Vector3(Mathf.Cos(an), 0, Mathf.Sin(an)); float rr = rad * Geo.R(rng, 0.85f, 1.1f); mb.Add(new Vector3(x, yb, z) + dir * rr, (dir * 0.8f + Vector3.up * 0.6f).normalized, uvo + new Vector2(dir.x, dir.z) * 0.35f); }
                                int ap = mb.Add(new Vector3(x, yt, z), Vector3.up, uvo + new Vector2(0.5f, 0.5f));
                                int ctr = mb.Add(new Vector3(x, yb - 0.5f, z), Vector3.down, uvo);
                                for (int k = 0; k < sides; k++) { int a = i0 + k, b = i0 + (k + 1) % sides; mb.Tri(1, a, ap, b); mb.Tri(1, a, b, ctr); }
                            }
                        }
                        else
                        {
                            float chh = (hgt - th) * 0.5f; Vector3 cc = new Vector3(x, y + th + chh, z); var ico = Geo.Icosphere(1); int i0 = mb.Count; float nph = Geo.R(rng, 0, 50);
                            foreach (var dv in ico.Item1)
                            {
                                float lump = 0.75f + 0.5f * Mathf.PerlinNoise(nph + dv.x * 2.2f + dv.z * 1.3f, nph + dv.y * 2.6f + dv.z * 0.9f); Vector3 pv = new Vector3(dv.x * cr * lump, dv.y * chh * lump, dv.z * cr * lump);
                                mb.Add(cc + pv, (dv * 0.75f + Vector3.up * 0.5f).normalized, uvo + new Vector2(dv.x + dv.y * 0.3f, dv.z + dv.y * 0.3f) * 0.5f);
                            }
                            for (int k = 0; k < ico.Item2.Length; k += 3) mb.Tri(0, i0 + ico.Item2[k], i0 + ico.Item2[k + 1], i0 + ico.Item2[k + 2]);
                        }
                        c.farTrees++;
                    }
            }
            foreach (var kv in tiles)
            {
                var m = kv.Value.ToMesh("FarForestTile" + tileN, true); Save(c, m, m.name);
                var go = new GameObject("FarForestTile_" + tileN++); go.transform.SetParent(far, false); go.layer = FarLayer;
                Rend(go, m, c.farBroad, c.farConifer, c.bark[0]).shadowCastingMode = ShadowCastingMode.Off;
            }
        }

        // ---------- spawn point, corridor, camera rigs ----------
        static void MarkersAndRigs(Ctx c, Transform root)
        {
            var s = c.spec; float y0 = c.H(c.spawn.x, c.spawn.y);
            var sp = new GameObject("DroneSpawn"); sp.transform.SetParent(root, false); sp.transform.position = new Vector3(c.spawn.x, y0 + 0.05f, c.spawn.y); sp.transform.rotation = Quaternion.Euler(0, c.spawnYaw, 0);
            sp.AddComponent<DroneForestMarker>().kind = DroneForestMarker.Kind.Spawn; sp.tag = "Untagged";
            var fc = new GameObject("FlightCorridor"); fc.transform.SetParent(root, false);
            for (int i = 0; i < c.corridor.Count; i++)
            {
                var w = new GameObject("WP_" + i.ToString("D2")); w.transform.SetParent(fc.transform, false); var p = c.corridor[i];
                w.transform.position = new Vector3(p.x, c.H(p.x, p.y) + 2.2f, p.y);
                if (i < c.corridor.Count - 1) w.transform.rotation = Quaternion.LookRotation(new Vector3(c.corridor[i + 1].x - p.x, 0, c.corridor[i + 1].y - p.y));
                var mk = w.AddComponent<DroneForestMarker>(); mk.kind = DroneForestMarker.Kind.CorridorWaypoint; mk.corridorHalfWidth = s.corridorClearRadius;
            }
            var rigs = new GameObject("CameraRigs"); rigs.transform.SetParent(root, false);
            Action<string, Vector3, Vector3> rig = (n, pos, look) => { var g = new GameObject(n); g.transform.SetParent(rigs.transform, false); g.transform.position = pos; g.transform.rotation = Quaternion.LookRotation((look - pos).normalized); g.AddComponent<DroneForestMarker>().kind = DroneForestMarker.Kind.CameraRig; };
            Vector3 W(int i, float up = 0) { var p = c.corridor[Mathf.Clamp(i, 0, c.corridor.Count - 1)]; return new Vector3(p.x, c.H(p.x, p.y) + 2.2f + up, p.y); }
            Vector3 fwd0 = (W(2) - W(0)).normalized;
            rig("CameraRig_Chase", W(0) - fwd0 * 3.2f + Vector3.up * 0.9f, W(0) + fwd0 * 6f);
            rig("CameraRig_FPV", W(0) + Vector3.up * 0.05f, W(1) + Vector3.up * 0.05f);
            rig("CameraRig_Corridor_Start", W(0) - fwd0 * 2.0f + Vector3.up * 0.1f, W(4));
            rig("CameraRig_Corridor_Mid", W(c.corridor.Count / 2) - (W(c.corridor.Count / 2 + 1) - W(c.corridor.Count / 2)).normalized * 2.5f, W(c.corridor.Count / 2 + 3));
            var trailMid = c.trail[c.trail.Count / 2]; var trailNext = c.trail[Mathf.Min(c.trail.Count - 1, c.trail.Count / 2 + 1)];
            rig("CameraRig_Trail_Low", new Vector3(trailMid.x, c.H(trailMid.x, trailMid.y) + 1.1f, trailMid.y), new Vector3(trailNext.x, c.H(trailNext.x, trailNext.y) + 1.7f, trailNext.y));
            rig("CameraRig_Canopy_Up", new Vector3(c.spawn.x + 2, y0 + 1.6f, c.spawn.y + 2), new Vector3(c.spawn.x + 4, y0 + 20, c.spawn.y + 6));
            if (c.clearings.Count > 0)
            {
                var cl = c.clearings[0]; var e = new Vector3(cl.x - cl.z * 0.8f, 0, cl.y - cl.z * 0.6f); e.y = c.H(e.x, e.z) + 2.5f;
                rig("CameraRig_Clearing_Wide", e, new Vector3(cl.x, c.H(cl.x, cl.y) + 3f, cl.y));
            }
            // high aerial establishing view over the canopy toward the patch edge and the surrounding landscape
            { float hf = s.areaSize * 0.5f - 6f; var ap = new Vector3(-hf * 0.85f, 0, -hf * 0.9f); ap.y = c.H(ap.x, ap.z) + 62f; var at = new Vector3(hf * 0.7f, 0, hf * 0.55f); at.y = c.H(at.x, at.z) + 6f; rig("CameraRig_Aerial_Wide", ap, at); }
            foreach (var t in c.targets)
            {
                Vector3 look; Vector3 cp = PickOrbitCam(c, t, out look);
                rig("CameraRig_Orbit_Target" + t.id.ToString("D2"), cp, look);
            }
        }

        /// <summary>Chooses a camera pose around a target: the body must be partially occluded by colliders (about 0.35-0.8 visible), the camera must not sit in or near geometry, and the target is centred.</summary>
        static Vector3 PickOrbitCam(Ctx c, TargetRec t, out Vector3 look)
        {
            Physics.SyncTransforms(); var go = GameObject.Find(t.name); var mf = go.GetComponent<MeshFilter>().sharedMesh; var vs = mf.vertices; var r = new System.Random(c.spec.seed + 777 + t.id);
            var samples = new List<Vector3>(); for (int i = 0; i < 48; i++) samples.Add(go.transform.TransformPoint(vs[r.Next(vs.Length)]));
            // foliage has no colliders: approximate cover ferns and mid-story crowns as spheres for the view test
            var spheres = new List<Vector4>(); foreach (var m in c.mids) spheres.Add(new Vector4(m.pos.x, m.pos.y + m.height * 0.5f, m.pos.z, m.crownR * (m.kind == "shrub" ? 0.85f : 0.7f)));
            var cov = GameObject.Find("TargetCover"); if (cov != null) foreach (Transform ch in cov.transform) if (ch.name == "CoverFern") spheres.Add(new Vector4(ch.position.x, ch.position.y + 0.35f, ch.position.z, 0.55f * ch.lossyScale.x));
            Func<Vector3, Vector3, bool> soft = (a, b) => { foreach (var sp in spheres) { Vector3 ctr = new Vector3(sp.x, sp.y, sp.z); Vector3 ab = b - a; float tt = Mathf.Clamp01(Vector3.Dot(ctr - a, ab) / Mathf.Max(1e-4f, ab.sqrMagnitude)); if ((a + ab * tt - ctr).sqrMagnitude < sp.w * sp.w) return true; } return false; };
            look = go.GetComponent<MeshRenderer>().bounds.center; Vector3 tp = t.pos; Vector3 back = (new Vector3(c.spawn.x, 0, c.spawn.y) - new Vector3(tp.x, 0, tp.z)).normalized; float baseAng = Mathf.Atan2(back.z, back.x);
            Vector3 best = tp + back * 5f + Vector3.up * 2f; float bestScore = -1e9f; int mask = 1 << EnvLayer;
            foreach (float dist in new[] { 3.6f, 4.4f, 5.2f, 6.2f }) foreach (float hh in new[] { 1.6f, 2.2f, 2.8f, 3.4f }) for (int k = 0; k < 36; k++)
                    {
                        float ang = k * 10f * Mathf.Deg2Rad; Vector3 cp = tp + new Vector3(Mathf.Cos(ang), 0, Mathf.Sin(ang)) * dist; cp.y = c.H(cp.x, cp.z) + hh;
                        if (Mathf.Abs(cp.x) > c.spec.areaSize * 0.5f - 3f || Mathf.Abs(cp.z) > c.spec.areaSize * 0.5f - 3f) continue;
                        if (Physics.CheckSphere(cp, 0.7f, mask)) continue; bool nearFoliage = false; foreach (var sp in spheres) if ((new Vector3(sp.x, sp.y, sp.z) - cp).magnitude < sp.w + 0.8f) { nearFoliage = true; break; }
                        if (nearFoliage) continue;
                        int vis = 0; foreach (var sp in samples) { RaycastHit hit; if ((!Physics.Linecast(cp, sp + (cp - sp).normalized * 0.02f, out hit, mask) || hit.collider.transform == go.transform) && !soft(cp, sp)) vis++; }
                        float vf = vis / (float)samples.Count; if (vf < 0.3f || vf > 0.85f) continue;
                        float d = Mathf.Abs(Mathf.DeltaAngle(ang * Mathf.Rad2Deg, baseAng * Mathf.Rad2Deg)) / 180f;
                        float score = -Mathf.Abs(vf - 0.55f) * 3f - d * 0.5f - Mathf.Abs(dist - 4.4f) * 0.05f - (soft(cp, look) ? 1.5f : 0f);
                        if (score > bestScore) { bestScore = score; best = cp; }
                    }
            return best;
        }

        // ---------- ground-truth visibility ----------
        static void PostChecks(Ctx c)
        {
            try
            {
                Physics.SyncTransforms(); var r = new System.Random(c.spec.seed + 4242);
                foreach (var t in c.targets)
                {
                    var go = GameObject.Find(t.name); if (go == null) continue; var mf = go.GetComponent<MeshFilter>().sharedMesh; var vs = mf.vertices;
                    int vis = 0, tot = 0;
                    for (int i = 0; i < 48; i++)
                    {
                        float az = Geo.R(r, 0, 6.283f), d = Geo.R(r, 5f, 14f); Vector3 vp = t.pos + new Vector3(Mathf.Cos(az) * d, 0, Mathf.Sin(az) * d); vp.y = c.H(vp.x, vp.z) + Geo.R(r, 1.5f, 3.5f);
                        for (int k = 0; k < 10; k++)
                        {
                            Vector3 bp = go.transform.TransformPoint(vs[r.Next(vs.Length)]); tot++;
                            RaycastHit hit; if (!Physics.Linecast(vp, bp + (vp - bp).normalized * 0.02f, out hit, 1 << EnvLayer) || hit.collider.transform == go.transform) vis++;
                        }
                    }
                    t.visibleFraction = tot > 0 ? (float)vis / tot : -1f;
                }
            }
            catch (Exception e) { Debug.LogWarning("[DroneForest] visibility check failed: " + e.Message); }
        }

        // ---------- manifest ----------
        static string V3(Vector3 v) { return string.Format(CI, "[{0:F3},{1:F3},{2:F3}]", v.x, v.y, v.z); }
        static void WriteManifest(Ctx c, string scenePath)
        {
            var s = c.spec; var sb = new StringBuilder();
            sb.Append("{\n\"schema\":\"crane.drone_forest.manifest/1\",\n\"seed\":" + s.seed + ",\n\"scene\":\"" + scenePath + "\",\n\"spec\":" + JsonUtility.ToJson(s) + ",\n");
            sb.Append("\"coordinate_system\":\"Unity world, metres, Y up; terrain centred on origin\",\n\"layer\":\"Environment(9)\",\n");
            sb.Append("\"spawn\":{\"position\":" + V3(new Vector3(c.spawn.x, c.H(c.spawn.x, c.spawn.y), c.spawn.y)) + ",\"yaw_deg\":" + c.spawnYaw.ToString("F1", CI) + "},\n");
            sb.Append("\"corridor_clear_radius_m\":" + s.corridorClearRadius.ToString("F2", CI) + ",\n\"corridor_waypoints\":[" + string.Join(",", c.corridor.Select(p => V3(new Vector3(p.x, c.H(p.x, p.y) + 2.2f, p.y)))) + "],\n");
            sb.Append("\"trail\":[" + string.Join(",", c.trail.Select(p => V3(new Vector3(p.x, c.H(p.x, p.y), p.y)))) + "],\n");
            sb.Append("\"clearings\":[" + string.Join(",", c.clearings.Select(p => string.Format(CI, "{{\"x\":{0:F2},\"z\":{1:F2},\"radius\":{2:F2} }}", p.x, p.y, p.z))) + "],\n");
            sb.Append("\"trees\":[\n" + string.Join(",\n", c.trees.Select(t => string.Format(CI, "{{\"id\":{0},\"species\":\"{1}\",\"dead\":{2},\"pos\":{3},\"yaw_deg\":{4:F1},\"scale_xz\":{5:F3},\"scale_y\":{6:F3},\"dbh_cm\":{7:F1},\"height_m\":{8:F2},\"size_class\":{9},\"variant\":{10},\"flight_band_segments\":{11}}}", t.id, t.species, t.dead ? "true" : "false", V3(t.pos), t.yaw, t.sx, t.sy, t.dbhCm, t.heightM, t.sizeClass, t.variant, t.branchCount))) + "\n],\n");
            sb.Append("\"props\":[\n" + string.Join(",\n", c.props.Where(p => p.kind == "log" || p.kind == "rock").Select(p => string.Format(CI, "{{\"kind\":\"{0}\",\"pos\":{1},\"yaw_deg\":{2:F1},\"size_m\":{3:F2},\"note\":\"{4}\"}}", p.kind, V3(p.pos), p.yaw, p.size, p.note))) + "\n],\n");
            sb.Append("\"understory_counts\":{\"ferns\":" + c.props.Count(p => p.kind == "fern") + ",\"shrubs\":" + c.props.Count(p => p.kind == "shrub") + ",\"grass_tufts\":" + c.nGrass + ",\"fallen_branches\":" + c.nBranch + ",\"note\":\"ferns, grass tufts and fallen branches are ground-level visuals without colliders; shrubs (legacy) have no colliders; see midstory for collidable stems\"},\n");
            sb.Append("\"surrounding_landscape\":{\"note\":\"visual only, layer EnvironmentFar(14), no colliders, excluded from ground truth\",\"enabled\":" + (s.surroundingLandscape ? "true" : "false") + ",\"distant_trees\":" + c.farTrees + "},\n");
            sb.Append("\"midstory\":{\"note\":\"shrubs and young conifers 1.5-4 m, stems have MeshColliders on layer Environment; leaf/needle cards have none. segments are capsules [ax,ay,az,bx,by,bz,radius_m] (mid-height 0.3-5.5 m, radius>=4 mm); crown_radius_m is the visual foliage extent\",\"count\":" + c.mids.Count + ",\"items\":[\n" + string.Join(",\n", c.mids.Select(m => string.Format(CI, "{{\"id\":{0},\"kind\":\"{1}\",\"pos\":{2},\"yaw_deg\":{3:F1},\"scale\":{4:F3},\"height_m\":{5:F2},\"crown_radius_m\":{6:F2},\"segment_count\":{7},\"segments\":[{8}]}}", m.id, m.kind, V3(m.pos), m.yaw, m.scale, m.height, m.crownR, m.segCount, m.segs))) + "\n]},\n");
            sb.Append("\"targets\":[\n" + string.Join(",\n", c.targets.Select(t => string.Format(CI, "{{\"id\":{0},\"name\":\"{1}\",\"pose\":\"{2}\",\"pos\":{3},\"yaw_deg\":{4:F1},\"occlusion_param\":{5:F2},\"cover_sticks\":{6},\"cover_mounds\":{7},\"cover_ferns\":{8},\"cover_log\":{9},\"geometric_visible_fraction\":{10:F3},\"visible_fraction_note\":\"fraction of 480 view-ray/body-point pairs (viewpoints 5-14 m away, 1.5-3.5 m above ground) not blocked by colliders; foliage cards have no colliders\"}}", t.id, t.name, t.pose, V3(t.pos), t.yaw, t.occlusion, t.coverSticks, t.coverMounds, t.coverFerns, t.log ? "true" : "false", t.visibleFraction))) + "\n],\n");
            sb.Append("\"flight_band_branches\":{\"note\":\"capsule segments [ax,ay,az,bx,by,bz,radius_m] with mid-height 0.3-5.5 m above local ground and radius>=3 mm\",\"per_tree\":[\n" + string.Join(",\n", c.branchLines) + "\n]}\n}\n");
            string mp = Root + "/Manifests/forest_seed" + s.seed + "_manifest.json"; Directory.CreateDirectory(Path.GetDirectoryName(mp)); File.WriteAllText(mp, sb.ToString());
        }
    }
}
