using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using Sim.Utils.Performance;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;
using UnityEngine.SceneManagement;

namespace Sim.Performance {
    /// <summary>
    /// Picks and applies the simulation profile for every player launch.
    /// Precedence: --crane-sim-profile NAME, then a saved manual choice, then auto. Auto runs the
    /// real scene once per device: per tier warm 3 s, measure 6 s capped (must deliver 97% of the
    /// cap), then 4 s uncapped (must sustain 1.05x the cap), with real time held and zero stale
    /// observations in both; steps down a tier on any failure; the result is cached beside the saved choice.
    /// --crane-sim-profile NAME --crane-sim-profile-save stores NAME as the manual choice;
    /// saving auto forgets both the manual choice and the cached measurement.
    /// </summary>
    internal sealed class CraneSimProfileSelector : MonoBehaviour {
        internal static readonly string[] Tiers = { "full", "balanced", "minimal" };
        private const float WarmSeconds = 3f;
        private const float MeasureSeconds = 6f;
        // Measured with the frame cap lifted: the tier must sustain 5% above its own cap, so a
        // pass leaves headroom and a capped run never queues work behind the sensors.
        private const double MinFrameRateMargin = 1.05;
        private const float UncappedSeconds = 4f;
        private const double MinDeliveredFraction = 0.97;
        private const double MinRealTimeFactor = 0.95;
        private const string SavedFileName = "crane-sim-profile.json";

        private static readonly FrameSettingsField[] s_ReducedEffects = {
            FrameSettingsField.Bloom, FrameSettingsField.DepthOfField, FrameSettingsField.MotionBlur,
            FrameSettingsField.ChromaticAberration, FrameSettingsField.FilmGrain,
            FrameSettingsField.LensFlareScreenSpace, FrameSettingsField.LensFlareDataDriven,
            FrameSettingsField.SSAO, FrameSettingsField.SSR, FrameSettingsField.SSGI,
            FrameSettingsField.ContactShadows, FrameSettingsField.Volumetrics,
            FrameSettingsField.VolumetricClouds
        };

        [Serializable] private sealed class Saved {
            public string profile;
            public string source;
            public string device;
        }

        [Serializable] private sealed class Report {
            public string schema = "crane-sim-profile-v1";
            public string profile;
            public string source;
            public string device;
            public string pipeline;
            public int frameRateCap;
            public float cameraResolutionScale;
            public int cameraWidth;
            public int cameraHeight;
            public float cameraHz;
            public float detectionHz;
            public float lidarHz;
            public bool reducedEffects;
            public bool waterDetail;
            public bool spectator;
            public List<string> benchmark;
        }

        private string requestedScene;
        private string tier;
        private string source;
        private CraneSimProfile profile;
        private bool needsBenchmark;
        private bool benchmarking;
        private readonly List<string> benchmarkLines = new();
        private readonly Dictionary<RenderTexture, Vector2Int> baseSizes = new();
        private Report report;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            string[] args = Environment.GetCommandLineArgs();
            if (Array.IndexOf(args, "--crane-list-profiles") >= 0) return;
            string requested = ReadString(args, "--crane-sim-profile", null)?.ToLowerInvariant();
            if (requested == "headless") {
                Fail("--crane-sim-profile headless is deferred: buoyancy and LiDAR water culling " +
                    "query HDRP water, which needs a graphics device. Use minimal.");
                return;
            }
            if (requested != null && requested != "auto" && Array.IndexOf(Tiers, requested) < 0) {
                Fail($"Unknown --crane-sim-profile '{requested}'. Expected auto, full, balanced " +
                    "or minimal.");
                return;
            }
            if (SystemInfo.graphicsDeviceType == GraphicsDeviceType.Null) {
                Debug.Log("CRANE_SIM_PROFILE_SKIPPED graphics-free player (no rendering to scale)");
                return;
            }
            var host = new GameObject("CRANE Sim Profile");
            DontDestroyOnLoad(host);
            host.AddComponent<CraneSimProfileSelector>().Resolve(args, requested);
        }

        private void Resolve(string[] args, string requested) {
            requestedScene = ReadString(args, "--crane-scene", null);
            string path = Path.Combine(Application.persistentDataPath, SavedFileName);
            if (requested != null && Array.IndexOf(args, "--crane-sim-profile-save") >= 0) {
                if (requested == "auto") File.Delete(path);
                else WriteSaved(path, requested, "manual");
            }
            Saved saved = ReadSaved(path);
            if (requested != null && requested != "auto") Select(requested, "cli");
            else if (requested == null && saved?.source == "manual" &&
                     Array.IndexOf(Tiers, saved.profile) >= 0) Select(saved.profile, "saved");
            else if (requested == null && Application.isEditor) Select("full", "editor-default");
            else if (saved?.source == "auto" && saved.device == DeviceSignature() &&
                     Array.IndexOf(Tiers, saved.profile) >= 0) Select(saved.profile, "auto-cache");
            else {
                Select(Tiers[0], "auto-benchmark");
                needsBenchmark = true;
            }
            SceneManager.sceneLoaded += OnSceneLoaded;
        }

        private void Select(string name, string why) {
            tier = name;
            source = why;
            profile = Resources.Load<CraneSimProfile>("SimProfiles/" +
                char.ToUpperInvariant(name[0]) + name.Substring(1));
            if (profile == null) {
                Fail($"Sim profile asset SimProfiles/{name} is missing from Resources.");
                return;
            }
            profile = WithOverrides(profile, ReadString(Environment.GetCommandLineArgs(),
                "--crane-sim-set", null));
            // vSyncCount stays 0 in QualitySettings: changing it at runtime crashes this Vulkan
            // player, so the frame cap is the only frame pacing.
            if (profile.Pipeline != null && QualitySettings.renderPipeline != profile.Pipeline)
                QualitySettings.renderPipeline = profile.Pipeline;
            Application.targetFrameRate = profile.FrameRateCap > 0 ? profile.FrameRateCap : -1;
        }

        /// <summary>--crane-sim-set "CameraHz=10,Spectator=false" adjusts single fields of the
        /// selected tier on a copy (assets stay untouched); FrameRateCap=0 uncaps.</summary>
        private static CraneSimProfile WithOverrides(CraneSimProfile source, string overrides) {
            if (string.IsNullOrWhiteSpace(overrides)) return source;
            CraneSimProfile copy = Instantiate(source);
            copy.name = source.name;
            foreach (string pair in overrides.Split(',', StringSplitOptions.RemoveEmptyEntries)) {
                string[] parts = pair.Split('=', 2);
                PropertyInfo property = parts.Length == 2 ? typeof(CraneSimProfile).GetProperty(
                    parts[0].Trim(), BindingFlags.Instance | BindingFlags.Public |
                    BindingFlags.IgnoreCase) : null;
                try {
                    if (property == null || property.PropertyType == typeof(RenderPipelineAsset))
                        throw new FormatException("unknown field");
                    property.SetValue(copy, Convert.ChangeType(parts[1].Trim(),
                        property.PropertyType, System.Globalization.CultureInfo.InvariantCulture));
                }
                catch (Exception exception) {
                    Fail($"--crane-sim-set '{pair}': {exception.Message}. Fields: FrameRateCap, " +
                        "CameraResolutionScale, CameraHz, DetectionHz, LidarHz, ReducedEffects, " +
                        "WaterDetail, Spectator.");
                    return source;
                }
            }
            return copy;
        }

        private void OnSceneLoaded(Scene scene, LoadSceneMode mode) {
            if (profile == null) return;
            if (!string.IsNullOrEmpty(requestedScene) &&
                !scene.name.Equals(requestedScene, StringComparison.OrdinalIgnoreCase)) return;
            if (needsBenchmark) {
                needsBenchmark = false;
                benchmarking = true;
                StartCoroutine(Benchmark());
            }
            ApplyScene();
        }

        private IEnumerator Benchmark() {
            for (int i = Array.IndexOf(Tiers, tier); ; i++) {
                // Phase 1, as shipped (capped): the cap must actually be delivered, since sensor
                // spikes overrun frames that an uncapped average hides.
                yield return new WaitForSecondsRealtime(WarmSeconds);
                Measurement capped = default;
                yield return Measure(MeasureSeconds, m => capped = m);
                bool pass = capped.Fps >= MinDeliveredFraction * profile.FrameRateCap &&
                    capped.Rtf >= MinRealTimeFactor && capped.Stale == 0;
                // Phase 2, cap lifted: headroom of 5% above the cap.
                Measurement open = default;
                if (pass) {
                    Application.targetFrameRate = -1;
                    yield return new WaitForSecondsRealtime(1f);
                    yield return Measure(UncappedSeconds, m => open = m);
                    pass = open.Fps >= MinFrameRateMargin * profile.FrameRateCap &&
                        open.Rtf >= MinRealTimeFactor && open.Stale == 0;
                }
                string line = $"{Tiers[i]} capped fps={capped.Fps:F1} cap={profile.FrameRateCap} " +
                    $"rtf={capped.Rtf:F3} stale={capped.Stale} uncapped fps={open.Fps:F1} " +
                    $"need={MinFrameRateMargin * profile.FrameRateCap:F1} stale={open.Stale} " +
                    (pass ? "pass" : "fail");
                Debug.Log($"CRANE_SIM_PROFILE_BENCHMARK {line}");
                benchmarkLines.Add(line);
                if (pass || i == Tiers.Length - 1) {
                    if (!pass) Debug.LogWarning("CRANE_SIM_PROFILE no tier holds its frame cap " +
                        "and real time on this device; staying on minimal.");
                    break;
                }
                Select(Tiers[i + 1], "auto-benchmark");
                ApplyScene();
            }
            Application.targetFrameRate = profile.FrameRateCap > 0 ? profile.FrameRateCap : -1;
            WriteSaved(Path.Combine(Application.persistentDataPath, SavedFileName), tier, "auto");
            benchmarking = false;
            EmitReport();
        }

        private struct Measurement {
            public double Fps, Rtf;
            public long Stale;
        }

        private static IEnumerator Measure(float seconds, Action<Measurement> done) {
            long stale = CraneRuntimeMetrics.StaleObservations;
            int frames = Time.frameCount;
            double wall = Time.realtimeSinceStartupAsDouble;
            double simulated = Time.timeAsDouble;
            yield return new WaitForSecondsRealtime(seconds);
            wall = Time.realtimeSinceStartupAsDouble - wall;
            done(new Measurement {
                Fps = (Time.frameCount - frames) / wall,
                Rtf = (Time.timeAsDouble - simulated) / wall / Math.Max(Time.timeScale, 1e-6f),
                Stale = CraneRuntimeMetrics.StaleObservations - stale
            });
        }

        private void ApplyScene() {
            report = new Report {
                profile = tier, source = source, device = DeviceSignature(),
                pipeline = QualitySettings.renderPipeline != null ?
                    QualitySettings.renderPipeline.name : GraphicsSettings.defaultRenderPipeline?.name,
                frameRateCap = profile.FrameRateCap,
                cameraResolutionScale = profile.CameraResolutionScale,
                reducedEffects = profile.ReducedEffects, waterDetail = profile.WaterDetail,
                spectator = profile.Spectator
            };
            var textures = new HashSet<RenderTexture>();
            foreach (MonoBehaviour component in FindObjectsByType<MonoBehaviour>(
                         FindObjectsInactive.Include)) {
                switch (component.GetType().FullName) {
                    case "Sim.Sensors.Vision.ROSCameraAsync":
                    case "Sim.Sensors.Vision.ROSDepthCameraAsync":
                        report.cameraHz = SetRate(component, profile.CameraHz);
                        CollectTextures(component, textures);
                        break;
                    case "Sim.Sensors.Vision.ProcessRenderTexture":
                    case "Sim.Sensors.Vision.CameraDepthBake":
                        CollectTextures(component, textures);
                        break;
                    case "Sim.Sensors.Vision.BoundingBox3D":
                        report.detectionHz = SetRate(component, profile.DetectionHz);
                        break;
                    case "Sim.Sensors.Lidar.Lidar3D":
                        report.lidarHz = SetRate(component, profile.LidarHz);
                        break;
                }
            }
            ResizeTextures(textures);
            if (profile.ReducedEffects)
                foreach (Camera camera in FindObjectsByType<Camera>(FindObjectsInactive.Include))
                    if (camera.TryGetComponent(out HDAdditionalCameraData data)) {
                        data.customRenderingSettings = true;
                        foreach (FrameSettingsField field in s_ReducedEffects) {
                            data.renderingPathCustomFrameSettingsOverrideMask.mask[(uint)field] = true;
                            data.renderingPathCustomFrameSettings.SetEnabled(field, false);
                        }
                    }
            if (!profile.WaterDetail)
                foreach (WaterSurface water in FindObjectsByType<WaterSurface>(
                             FindObjectsInactive.Include)) {
                    water.tessellation = false;
                    water.caustics = false;
                }
            if (!profile.Spectator) CraneRuntimeOptions.DisableSpectatorRendering();
            if (!benchmarking) EmitReport();
        }

        private void EmitReport() {
            report.benchmark = benchmarkLines;
            Debug.Log($"CRANE_SIM_PROFILE_RESOLVED {JsonUtility.ToJson(report)}");
        }

        /// <summary>Sets a sensor's private Hz (and its publisher's, for live changes); returns
        /// the effective rate.</summary>
        private static float SetRate(MonoBehaviour sensor, float hz) {
            const BindingFlags flags = BindingFlags.Instance | BindingFlags.Public |
                BindingFlags.NonPublic;
            Type type = sensor.GetType();
            FieldInfo rate = type.GetField("Hz", flags);
            if (rate?.FieldType != typeof(float)) return 0f;
            if (hz > 0f) {
                rate.SetValue(sensor, hz);
                object publisher = type.GetProperty("publisher", flags)?.GetValue(sensor) ??
                    type.GetField("publisher", flags)?.GetValue(sensor);
                FieldInfo publisherRate = publisher?.GetType().GetField("Hz", flags);
                if (publisherRate != null && publisherRate.GetValue(publisher) is float current &&
                    current > 0f) publisherRate.SetValue(publisher, hz);
            }
            return (float)rate.GetValue(sensor);
        }

        private static void CollectTextures(MonoBehaviour component, HashSet<RenderTexture> into) {
            for (Type type = component.GetType(); type != null && type != typeof(MonoBehaviour);
                 type = type.BaseType)
                foreach (FieldInfo field in type.GetFields(BindingFlags.Instance |
                             BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.DeclaredOnly)) {
                    if (field.GetValue(component) is RenderTexture texture && texture != null)
                        into.Add(texture);
                    else if (field.GetValue(component) is Camera camera && camera != null &&
                             camera.targetTexture != null)
                        into.Add(camera.targetTexture);
                }
        }

        private void ResizeTextures(HashSet<RenderTexture> textures) {
            bool drained = false;
            Camera[] cameras = null;
            foreach (RenderTexture texture in textures) {
                if (!baseSizes.TryGetValue(texture, out Vector2Int size))
                    baseSizes[texture] = size = new Vector2Int(texture.width, texture.height);
                int width = Mathf.Max(16, Mathf.RoundToInt(size.x * profile.CameraResolutionScale));
                int height = Mathf.Max(16, Mathf.RoundToInt(size.y * profile.CameraResolutionScale));
                if (report.cameraWidth == 0) {
                    report.cameraWidth = width;
                    report.cameraHeight = height;
                }
                if (texture.width == width && texture.height == height) continue;
                if (!drained) {
                    // In-flight readbacks of the old size would complete against a released target.
                    AsyncGPUReadback.WaitAllRequests();
                    cameras = FindObjectsByType<Camera>(FindObjectsInactive.Include);
                    drained = true;
                }
                var users = new List<Camera>();
                foreach (Camera camera in cameras)
                    if (camera.targetTexture == texture) {
                        users.Add(camera);
                        camera.targetTexture = null;
                    }
                texture.Release();
                texture.width = width;
                texture.height = height;
                texture.Create();
                foreach (Camera camera in users) camera.targetTexture = texture;
            }
        }

        private static string DeviceSignature() =>
            $"{SystemInfo.processorType}|{SystemInfo.processorCount}|" +
            $"{SystemInfo.graphicsDeviceName}|{SystemInfo.graphicsDeviceType}";

        private static Saved ReadSaved(string path) {
            try {
                return File.Exists(path) ? JsonUtility.FromJson<Saved>(File.ReadAllText(path)) : null;
            }
            catch (Exception exception) {
                Debug.LogWarning($"CRANE_SIM_PROFILE ignoring unreadable {path}: {exception.Message}");
                return null;
            }
        }

        private static void WriteSaved(string path, string name, string why) {
            try {
                Directory.CreateDirectory(Path.GetDirectoryName(path));
                File.WriteAllText(path, JsonUtility.ToJson(new Saved {
                    profile = name, source = why, device = DeviceSignature()
                }, true));
            }
            catch (Exception exception) {
                Debug.LogWarning($"CRANE_SIM_PROFILE could not save {path}: {exception.Message}");
            }
        }

        private static void Fail(string reason) {
            Debug.LogError($"CRANE_SIM_PROFILE_REJECTED {reason}");
            Application.Quit(3);
        }

        private void OnDestroy() => SceneManager.sceneLoaded -= OnSceneLoaded;

        private static string ReadString(string[] args, string key, string fallback) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length ? args[index + 1] : fallback;
        }
    }
}
