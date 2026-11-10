using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

namespace Crane.Fleet.Sim {
    /// <summary>Command-line options shared by the editor CLI and the player.</summary>
    public static class FleetArgs {
        public static string Get(string key, string fallback = null) {
            string[] a = Environment.GetCommandLineArgs();
            for (int i = 0; i + 1 < a.Length; i++) if (a[i] == key) return a[i + 1];
            return fallback;
        }

        public static bool Has(string key) => Array.IndexOf(Environment.GetCommandLineArgs(), key) >= 0;

        public static string SpecDir(string fallback) => Get("--crane-fleet-spec", fallback);

        public static FleetRunOptions RunOptions(string launcher) {
            var o = new FleetRunOptions { Launcher = launcher };
            o.Seed = long.Parse(Get("--crane-seed", "1"), CultureInfo.InvariantCulture);
            o.Variant = Get("--crane-fleet-variant");
            o.OutRoot = Get("--crane-fleet-out-root");
            o.OutDir = Get("--crane-fleet-out");
            string door = Get("--crane-fleet-doorway");
            if (door != null) o.DoorwayWidth = double.Parse(door, CultureInfo.InvariantCulture);
            string dur = Get("--crane-fleet-duration");
            if (dur != null) o.MaxDuration = double.Parse(dur, CultureInfo.InvariantCulture);
            o.SourceCommit = Get("--crane-source-commit", "");
            o.SourceDirty = string.Equals(Get("--crane-source-dirty", "false"), "true", StringComparison.OrdinalIgnoreCase);
            return o;
        }
    }

    /// <summary>
    /// Player entry: <c>--crane-fleet-run</c> (seeded qualification run, works with -nographics),
    /// <c>--crane-fleet-check REPORT.json</c> (CAD-swap check) or <c>--crane-fleet-still DIR</c> (the same seeded run
    /// with graphics, rendering the overlay's still shots to PNG). The scene is built at runtime from the spec in
    /// StreamingAssets/Fleet (or <c>--crane-fleet-spec DIR</c>); the player quits with 0 on success.
    /// </summary>
    public sealed class FleetBoot : MonoBehaviour {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        static void Install() {
            if (!FleetArgs.Has("--crane-fleet-run") && FleetArgs.Get("--crane-fleet-check") == null && FleetArgs.Get("--crane-fleet-still") == null) return;
            var go = new GameObject("CRANE fleet boot");
            DontDestroyOnLoad(go);
            go.AddComponent<FleetBoot>();
        }

        IEnumerator Start() {
            int code = 0;
            IEnumerator body = Body(c => code = c);
            while (true) {
                object cur;
                try {
                    if (!body.MoveNext()) break;
                    cur = body.Current;
                }
                catch (Exception e) {
                    Debug.LogError("CRANE_FLEET_FAILED " + e);
                    code = 3;
                    break;
                }
                yield return cur;
            }
            Debug.Log($"CRANE_FLEET_EXIT code={code}");
            Application.Quit(code);
        }

        IEnumerator Body(Action<int> exit) {
            yield return null;
            var spec = FleetSpec.LoadDirectory(FleetArgs.SpecDir(Path.Combine(Application.streamingAssetsPath, "Fleet")));
            Debug.Log($"CRANE_FLEET_SPEC profile={spec.Profile.Sha256} scenario={spec.Scenario.Sha256} overlay={spec.Overlay.Sha256} gfx={SystemInfo.graphicsDeviceType}");
            string check = FleetArgs.Get("--crane-fleet-check");
            if (check != null) {
                var r = CadSwapCheck.Run(spec, check);
                Debug.Log($"CRANE_FLEET_CHECK pass={r.Pass} report={check} failures={r.Failures.Count}");
                foreach (string f in r.Failures) Debug.Log("CRANE_FLEET_CHECK_FAIL " + f);
                exit(r.Pass ? 0 : 1);
                yield break;
            }
            string still = FleetArgs.Get("--crane-fleet-still");
            var opt = FleetArgs.RunOptions("player");
            if (still != null && opt.OutDir == null) opt.OutDir = Path.Combine(still, "run");
            var run = new FleetRun(spec, opt);
            run.Setup();
            Debug.Log($"CRANE_FLEET_RUN_START id={run.RunId} dir={run.Dir}");
            var shots = still != null ? FleetStills.Shots(spec) : new List<FleetStills.Shot>();
            int next = 0;
            while (true) {
                while (next < shots.Count && run.T >= shots[next].T - 1e-9) {
                    yield return FleetStills.Render(spec, shots[next], still);
                    next++;
                }
                if (!run.Advance()) break;
                if (run.Step % 4000 == 0) yield return null;
            }
            while (next < shots.Count) { yield return FleetStills.Render(spec, shots[next], still); next++; }
            run.Finish();
            Debug.Log($"CRANE_FLEET_RUN_END id={run.RunId} reason={run.EndReason} t={run.T} steps={run.Step} dir={run.Dir}");
            exit(0);
        }
    }

    /// <summary>Offscreen stills with an HDRP camera into a RenderTexture (needs a graphics device: run the player
    /// batchmode without -nographics, under xvfb-run).</summary>
    public static class FleetStills {
        public sealed class Shot {
            public string Name;
            public double T, Fov;
            public double[] Position, LookAt;
        }

        public static List<Shot> Shots(FleetSpec spec) {
            var r = new List<Shot>();
            foreach (var s in spec.O["stills.shots"].Items)
                r.Add(new Shot { Name = s["name"].String, T = s["t"].Double, Fov = s["fov"].Double, Position = s["position"].Doubles, LookAt = s["look_at"].Doubles });
            r.Sort((a, b) => a.T.CompareTo(b.T));
            return r;
        }

        public static IEnumerator Render(FleetSpec spec, Shot shot, string dir) {
            double[] size = spec.O["stills.size"].Doubles;
            int w = (int)size[0], h = (int)size[1];
            var go = new GameObject("still camera " + shot.Name);
            var cam = go.AddComponent<Camera>();
            var hd = go.AddComponent<HDAdditionalCameraData>();
            hd.antialiasing = HDAdditionalCameraData.AntialiasingMode.SubpixelMorphologicalAntiAliasing;
            hd.allowDynamicResolution = false;
            cam.enabled = false;
            cam.nearClipPlane = 0.05f; cam.farClipPlane = 200f;
            cam.fieldOfView = (float)shot.Fov;
            var rt = new RenderTexture(w, h, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB) { antiAliasing = 1 };
            rt.Create();
            cam.targetTexture = rt;
            cam.aspect = w / (float)h;
            Vector3 p = FleetFrames.World(shot.Position[0], shot.Position[1], shot.Position[2]);
            Vector3 l = FleetFrames.World(shot.LookAt[0], shot.LookAt[1], shot.LookAt[2]);
            go.transform.SetPositionAndRotation(p, Quaternion.LookRotation(l - p, Vector3.up));
            var renderers = UnityEngine.Object.FindObjectsByType<Renderer>(FindObjectsInactive.Exclude);
            Material sample = renderers.Length > 0 ? renderers[0].sharedMaterial : null;
            Debug.Log($"CRANE_FLEET_STILL_DIAG renderers={renderers.Length} template={FleetGeometry.Template?.name} " +
                      $"shader={(sample != null ? sample.shader.name : "none")} supported={(sample != null && sample.shader.isSupported)} " +
                      $"passes={(sample != null ? sample.passCount : 0)} pipeline={UnityEngine.Rendering.GraphicsSettings.currentRenderPipeline?.name} " +
                      $"quality={QualitySettings.names[QualitySettings.GetQualityLevel()]} cam={p} look={l}");
            for (int i = 0; i < 12; i++) { cam.Render(); yield return null; }  // sky, shadows and history warm-up
            cam.Render();
            var tex = new Texture2D(w, h, TextureFormat.RGBA32, false);
            var prev = RenderTexture.active;
            RenderTexture.active = rt;
            tex.ReadPixels(new Rect(0, 0, w, h), 0, 0, false);
            tex.Apply();
            RenderTexture.active = prev;
            Directory.CreateDirectory(dir);
            string path = Path.Combine(dir, shot.Name + ".png");
            File.WriteAllBytes(path, tex.EncodeToPNG());
            Debug.Log($"CRANE_FLEET_STILL {path} {w}x{h} t={shot.T}");
            UnityEngine.Object.Destroy(tex);
            cam.targetTexture = null;
            rt.Release();
            UnityEngine.Object.Destroy(go);
        }
    }
}
