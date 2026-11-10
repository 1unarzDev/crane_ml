// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;
using Crane.Drones.Environment;
using Crane.Drones.Environment.Editor;

namespace Crane.Drones.TreeGenImport.Editor
{
    /// <summary>TreeGen vs built-in trees, same seed, spec and rigs, in one Editor session (needs a GPU: unity-drones --render).
    ///   unity-drones --render LOG -quit -executeMethod Crane.Drones.TreeGenImport.Editor.TreeGenCompare.Run \
    ///     -treegenLibrary lib_seed7 -captureOut /abs/dir [-compareDensity 0.5] [-compareRestore 1]
    /// Memory budget (the first full-density attempt reached about 9 GB and was stopped):
    ///  - both passes use a copy of ForestSpec_Default with stem and sapling density scaled by -compareDensity;
    ///  - TreeGen instances drop their capsule GameObjects (TreeGenForestProvider.StripColliders, render only);
    ///  - the 3 rigs are rendered inside the freshly built scene (no reopen), with the shot settings of
    ///    DroneForestCapture (01 Corridor_Start GoldenHour 65, 03 Canopy_Up GoldenHour 70, 06 Corridor_Mid Dusk 65);
    ///  - unused assets are unloaded between passes.
    /// -compareRestore 1 (default) finishes with a normal full-density built-in BuildDroneForest build, so the shared
    /// scene, manifest and Generated/seed_N are left as the forest owner expects. Writes DIR/{treegen,builtin}/*.png + perf.json.</summary>
    public static class TreeGenCompare
    {
        struct Shot { public string rig, preset, file; public float fov; }
        static readonly Shot[] Shots = {
            new Shot { rig = "CameraRig_Corridor_Start", preset = "GoldenHour", file = "01_corridor_start_golden.png", fov = 65 },
            new Shot { rig = "CameraRig_Canopy_Up", preset = "GoldenHour", file = "03_canopy_up_golden.png", fov = 70 },
            new Shot { rig = "CameraRig_Corridor_Mid", preset = "Dusk", file = "06_corridor_mid_dusk.png", fov = 65 },
        };
        static readonly CultureInfo CI = CultureInfo.InvariantCulture;

        public static void Run()
        {
            var a = System.Environment.GetCommandLineArgs();
            string lib = TreeGenImporter.Arg(a, "-treegenLibrary") ?? "lib_seed7", outDir = TreeGenImporter.Arg(a, "-captureOut");
            float dens = float.Parse(TreeGenImporter.Arg(a, "-compareDensity") ?? "0.5", CI);
            bool restore = (TreeGenImporter.Arg(a, "-compareRestore") ?? "1") != "0";
            if (string.IsNullOrEmpty(outDir)) throw new ArgumentException("need -captureOut DIR");
            var baseSpec = AssetDatabase.LoadAssetAtPath<ForestSpec>(BuildDroneForest.Root + "/Specs/ForestSpec_Default.asset");
            var spec = UnityEngine.Object.Instantiate(baseSpec); spec.stemDensityPerHa *= dens; spec.saplingDensityPerHa *= dens;
            string scene = "Assets/Scenes/Drone Forest.unity";
            var lines = new List<string>();
            try
            {
                TreeGenForestProvider.Register(lib); TreeGenForestProvider.StripColliders = true;
                var t0 = DateTime.Now; BuildDroneForest.Build(spec, scene); double b = (DateTime.Now - t0).TotalSeconds;
                TreeGenForestProvider.Unregister();
                UseForestFoliageProfile();
                string st = TreeGenForestProvider.SceneStats("treegen", b);
                t0 = DateTime.Now; Capture(Path.Combine(outDir, "treegen")); double r = (DateTime.Now - t0).TotalSeconds;
                lines.Add(st.TrimEnd('}') + ",\"render_s\":" + r.ToString("F1", CI) + ",\"density_scale\":" + dens.ToString(CI) + ",\"capsules_stripped_for_render\":true}");
            }
            catch (Exception e) { Debug.LogError("[TreeGenCompare] TreeGen pass failed: " + e); }
            finally { TreeGenForestProvider.Unregister(); TreeGenForestProvider.StripColliders = false; EditorUtility.UnloadUnusedAssetsImmediate(); GC.Collect(); }

            try
            {
                var t0 = DateTime.Now; BuildDroneForest.Build(spec, scene); double b = (DateTime.Now - t0).TotalSeconds;
                string st = TreeGenForestProvider.SceneStats("builtin", b);
                t0 = DateTime.Now; Capture(Path.Combine(outDir, "builtin")); double r = (DateTime.Now - t0).TotalSeconds;
                lines.Add(st.TrimEnd('}') + ",\"render_s\":" + r.ToString("F1", CI) + ",\"density_scale\":" + dens.ToString(CI) + "}");
            }
            catch (Exception e) { Debug.LogError("[TreeGenCompare] built-in pass failed: " + e); }
            Directory.CreateDirectory(outDir);
            File.WriteAllText(Path.Combine(outDir, "perf.json"), "[\n" + string.Join(",\n", lines) + "\n]\n");
            Debug.Log("[TreeGenCompare] " + string.Join(" | ", lines));
            if (restore) { EditorUtility.UnloadUnusedAssetsImmediate(); GC.Collect(); BuildDroneForest.Build(baseSpec, scene); Debug.Log("[TreeGenCompare] restored built-in forest " + scene); }
        }

        static void UseForestFoliageProfile()
        {
            var gen = new DirectoryInfo(BuildDroneForest.Root + "/Generated"); if (!gen.Exists) return;
            foreach (var f in gen.GetFiles("FoliageDiffusion.asset", SearchOption.AllDirectories))
            {
                string rel = "Assets" + f.FullName.Replace('\\', '/').Substring(Application.dataPath.Length);
                TreeGenImporter.UseDiffusionProfile(AssetDatabase.LoadAssetAtPath<DiffusionProfileSettings>(rel)); return;
            }
        }

        /// <summary>Same camera setup as DroneForestCapture.Run, without reopening the scene.</summary>
        static void Capture(string dir, int W = 1600, int H = 900)
        {
            Directory.CreateDirectory(dir);
            var director = UnityEngine.Object.FindFirstObjectByType<DroneForestDirector>();
            foreach (var v in UnityEngine.Object.FindObjectsByType<Volume>(FindObjectsSortMode.None))
                if (v.sharedProfile != null && v.sharedProfile.TryGet(out Exposure ex)) { ex.adaptationMode.overrideState = true; ex.adaptationMode.value = AdaptationMode.Fixed; }
            var camGo = new GameObject("CaptureCam"); var cam = camGo.AddComponent<Camera>(); cam.nearClipPlane = 0.1f; cam.farClipPlane = 600f; DroneForestDirector.ConfigureCamera(cam);
            var rt = new RenderTexture(W, H, 24, RenderTextureFormat.ARGB32); rt.Create(); cam.targetTexture = rt;
            try
            {
                foreach (var s in Shots)
                {
                    var rig = GameObject.Find(s.rig); if (rig == null) { Debug.LogWarning("[TreeGenCompare] rig missing " + s.rig); continue; }
                    camGo.transform.SetPositionAndRotation(rig.transform.position, rig.transform.rotation); cam.fieldOfView = s.fov;
                    if (director != null) { director.SetPreset(s.preset); director.SetDepthOfField(false, 8f); director.SetFilmGrain(false); director.SetMotionBlur(false); }
                    for (int i = 0; i < 6; i++) cam.Render();
                    RenderTexture.active = rt; var tex = new Texture2D(W, H, TextureFormat.RGB24, false); tex.ReadPixels(new Rect(0, 0, W, H), 0, 0); tex.Apply(); RenderTexture.active = null;
                    File.WriteAllBytes(Path.Combine(dir, s.file), tex.EncodeToPNG()); UnityEngine.Object.DestroyImmediate(tex);
                    Debug.Log("[TreeGenCompare] wrote " + Path.Combine(dir, s.file));
                }
            }
            finally { cam.targetTexture = null; rt.Release(); UnityEngine.Object.DestroyImmediate(rt); UnityEngine.Object.DestroyImmediate(camGo); }
        }
    }
}
