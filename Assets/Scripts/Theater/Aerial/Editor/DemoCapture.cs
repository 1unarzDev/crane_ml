using System;
using System.Globalization;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace Crane.Theater.Aerial {
    /// <summary>
    /// Headless demo capture. Run through the drone worktree's wrapper (never -quit; the method exits the editor itself):
    ///   unity-drones LOG -executeMethod Crane.Theater.Aerial.DemoCapture.Run -demoSeed 7 -demoSequence Assets/Drones/Demo/Sequences/forest_demo_v1.json
    ///     -demoProfile both|cinematic|onboard [-demoRunId ID] [-demoOut DIR] [-demoFrames A:B] [-demoFfmpeg PATH|none] [-demoNoise SIGMA]
    ///     [-demoReadoutMs MS] [-demoNoStills] [-demoMaxMb 18]
    /// Output: Docs/DroneVisuals/demo/&lt;run_id&gt;/ (demo_cinematic_1080p30.mp4, onboard_oakd_rgb_1080p30.mp4, stills/, frames.jsonl,
    /// render_report.json). Renders in edit mode with Camera.Render(); the GPU must be available (no -nographics).
    /// </summary>
    public static class DemoCapture {
        const string DefaultSequence = "Assets/Drones/Demo/Sequences/forest_demo_v1.json";

        static string Arg(string key, string fallback = null) {
            var a = Environment.GetCommandLineArgs();
            for (int i = 0; i + 1 < a.Length; i++) if (a[i] == key) return a[i + 1];
            return fallback;
        }
        static bool Flag(string key) => Array.IndexOf(Environment.GetCommandLineArgs(), key) >= 0;

        public static void Run() {
            int code = 0;
            try { code = RunInner() ? 0 : 2; }
            catch (Exception e) { Debug.LogError("AERIAL_DEMO_FAILED " + e); code = 3; }
            Debug.Log("AERIAL_DEMO_EXIT code=" + code);
            EditorApplication.Exit(code);
        }

        static bool RunInner() {
            string project = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
            string seqPath = Path.GetFullPath(Path.Combine(project, Arg("-demoSequence", DefaultSequence)));
            var spec = DemoSpec.Parse(File.ReadAllText(seqPath));
            int seed = int.Parse(Arg("-demoSeed", spec.seed.ToString(CultureInfo.InvariantCulture)), CultureInfo.InvariantCulture);
            string profile = Arg("-demoProfile", "both");
            string runId = Arg("-demoRunId", $"{Path.GetFileNameWithoutExtension(seqPath)}_s{seed}_{profile}");
            string outDir = Path.GetFullPath(Path.Combine(project, Arg("-demoOut", Path.Combine("Docs", "DroneVisuals", "demo", runId))));

            var o = new DemoOptions {
                sequencePath = seqPath, outDir = outDir, runId = runId, profile = profile, seed = seed,
                ffmpeg = Arg("-demoFfmpeg", "ffmpeg"), crf = int.Parse(Arg("-demoCrf", "20"), CultureInfo.InvariantCulture),
                maxMegabytes = float.Parse(Arg("-demoMaxMb", "18"), CultureInfo.InvariantCulture),
                noiseSigma = float.Parse(Arg("-demoNoise", "0"), CultureInfo.InvariantCulture),
                readoutS = float.Parse(Arg("-demoReadoutMs", "0"), CultureInfo.InvariantCulture) / 1000f,
                stills = !Flag("-demoNoStills")
            };
            string frames = Arg("-demoFrames");
            if (frames != null) {
                var p = frames.Split(':');
                o.firstFrame = int.Parse(p[0], CultureInfo.InvariantCulture);
                if (p.Length > 1 && p[1].Length > 0) o.lastFrame = int.Parse(p[1], CultureInfo.InvariantCulture);
            }

            string scenePath = Path.Combine(project, spec.scene);
            if (File.Exists(scenePath)) EditorSceneManager.OpenScene(spec.scene, OpenSceneMode.Single);
            else { Debug.LogWarning("AERIAL_DEMO scene missing, using the stub forest: " + spec.scene); EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single); }

            AerialRobot.PrefabLoader = () => AssetDatabase.LoadAssetAtPath<GameObject>(spec.prefab);
            if (AerialRobot.PrefabLoader() == null) Debug.LogWarning("AERIAL_DEMO prefab missing, using the stub quad: " + spec.prefab);
            return new DemoSession(o).Run();
        }
    }
}
