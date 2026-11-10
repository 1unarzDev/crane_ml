using System;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

namespace Crane.Drones.Environment.Editor
{
    /// <summary>Renders preview stills from the CameraRig_* transforms. Needs a GPU-enabled Editor (no -nographics).
    ///   -executeMethod Crane.Drones.Environment.Editor.DroneForestCapture.Run -forestScene <path> -captureOut <dir></summary>
    public static class DroneForestCapture
    {
        struct Shot { public string rig, preset, file; public bool dof; public float fov; public float focus; }

        public static void Run()
        {
            var a = System.Environment.GetCommandLineArgs(); string scenePath = "Assets/Scenes/Drone Forest.unity", outDir = "Docs/DroneVisuals/environment"; int W = 1600, H = 900; string only = null;
            for (int i = 0; i < a.Length - 1; i++) { if (a[i] == "-forestScene") scenePath = a[i + 1]; if (a[i] == "-captureOut") outDir = a[i + 1]; if (a[i] == "-captureW") W = int.Parse(a[i + 1]); if (a[i] == "-captureH") H = int.Parse(a[i + 1]); if (a[i] == "-captureOnly") only = a[i + 1]; }
            Directory.CreateDirectory(outDir);
            EditorSceneManager.OpenScene(scenePath, OpenSceneMode.Single);
            var dir = UnityEngine.Object.FindFirstObjectByType<DroneForestDirector>();
            var shots = new List<Shot> {
                new Shot{ rig="CameraRig_Corridor_Start", preset="GoldenHour", file="01_corridor_start_golden.png", fov=65 },
                new Shot{ rig="CameraRig_Trail_Low", preset="GoldenHour", file="02_trail_low_golden.png", fov=60 },
                new Shot{ rig="CameraRig_Canopy_Up", preset="GoldenHour", file="03_canopy_up_golden.png", fov=70 },
                new Shot{ rig="CameraRig_Orbit_Target01", preset="GoldenHour", file="04_target1_dof_golden.png", fov=45, dof=true, focus=6.5f },
                new Shot{ rig="CameraRig_Clearing_Wide", preset="Overcast", file="05_clearing_overcast.png", fov=65 },
                new Shot{ rig="CameraRig_Corridor_Mid", preset="Dusk", file="06_corridor_mid_dusk.png", fov=65 },
                new Shot{ rig="CameraRig_Orbit_Target02", preset="Overcast", file="07_target2_overcast.png", fov=45, dof=true, focus=6.5f },
                new Shot{ rig="CameraRig_Orbit_Target03", preset="GoldenHour", file="08_target3_golden.png", fov=45, dof=true, focus=6.5f },
            };
            // instant exposure adaptation for stills
            foreach (var v in UnityEngine.Object.FindObjectsByType<Volume>(FindObjectsSortMode.None))
                if (v.sharedProfile != null && v.sharedProfile.TryGet(out Exposure ex)) { ex.adaptationMode.overrideState = true; ex.adaptationMode.value = AdaptationMode.Fixed; }
            var camGo = new GameObject("CaptureCam"); var cam = camGo.AddComponent<Camera>(); cam.nearClipPlane = 0.1f; cam.farClipPlane = 600f; DroneForestDirector.ConfigureCamera(cam);
            var rt = new RenderTexture(W, H, 24, RenderTextureFormat.ARGB32); rt.Create(); cam.targetTexture = rt;
            foreach (var s in shots)
            {
                if (only != null && !s.file.StartsWith(only)) continue;
                var rig = GameObject.Find(s.rig); if (rig == null) { Debug.LogWarning("rig missing " + s.rig); continue; }
                camGo.transform.SetPositionAndRotation(rig.transform.position, rig.transform.rotation); cam.fieldOfView = s.fov;
                dir.SetPreset(s.preset); dir.SetDepthOfField(s.dof, s.focus > 0 ? s.focus : 8f); dir.SetFilmGrain(false); dir.SetMotionBlur(false);
                for (int i = 0; i < 6; i++) cam.Render();
                RenderTexture.active = rt; var tex = new Texture2D(W, H, TextureFormat.RGB24, false); tex.ReadPixels(new Rect(0, 0, W, H), 0, 0); tex.Apply(); RenderTexture.active = null;
                File.WriteAllBytes(Path.Combine(outDir, s.file), tex.EncodeToPNG()); UnityEngine.Object.DestroyImmediate(tex);
                Debug.Log("[DroneForestCapture] wrote " + s.file);
            }
        }
    }
}
