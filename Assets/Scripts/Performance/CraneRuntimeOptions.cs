using System;
using System.Collections.Generic;
using System.Reflection;
using System.IO;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

namespace Sim.Performance {
    /// <summary>
    /// Keeps HDRP's global water update loop alive without presentation rendering when an
    /// aquatic worker scene has no enabled task-sensor camera.
    /// </summary>
    internal sealed class CraneWaterUpdateDriver : MonoBehaviour {
        private Camera targetCamera;
        private RenderTexture target;
        private int originalCullingMask;
        private RenderTexture originalTarget;
        private bool configured;

        public void Configure(Camera camera) {
            if (configured) return;
            configured = true;
            targetCamera = camera;
            originalCullingMask = camera.cullingMask;
            originalTarget = camera.targetTexture;
            // HDRP derives several downsampled render-graph textures from the camera target;
            // 1x1 produces zero-sized intermediates, while 64x64 remains presentation-cheap.
            target = new RenderTexture(64, 64, 0, RenderTextureFormat.ARGB32) {
                name = "CRANE Water Update Driver"
            };
            target.Create();
            camera.cullingMask = 0;
            camera.targetTexture = target;
            camera.enabled = true;
            if (camera.TryGetComponent(out AudioListener listener)) listener.enabled = false;
        }

        private void OnDestroy() {
            if (targetCamera != null) {
                targetCamera.cullingMask = originalCullingMask;
                targetCamera.targetTexture = originalTarget;
            }
            if (target != null) {
                target.Release();
                Destroy(target);
            }
        }
    }

    /// <summary>
    /// Parsed worker options and the single place that applies runtime subsystem gates.
    /// This runs before scene Start methods through CraneWorkerOverrides.sceneLoaded.
    /// </summary>
    internal sealed class CraneRuntimeOptions {
        private static ulong s_AppliedSceneHandle = ulong.MaxValue;
        private static string s_AppliedProfile;
        internal readonly struct CameraStatus {
            public readonly int Enabled;
            public readonly int Sensor;
            public readonly int Spectator;
            public readonly int WaterDriver;

            public CameraStatus(int enabled, int sensor, int spectator, int waterDriver) {
                Enabled = enabled;
                Sensor = sensor;
                Spectator = spectator;
                WaterDriver = waterDriver;
            }
        }

        public string ProfileName { get; private set; }
        public bool DisableRosTcp { get; private set; }
        public bool DisableSitl { get; private set; }
        public bool DisableRgb { get; private set; }
        public bool DisableDepth { get; private set; }
        public bool DisableCameraInfo { get; private set; }
        public bool DisableDetections { get; private set; }
        public bool DisableSpectatorCameras { get; private set; }
        public bool LowPresentation { get; private set; }
        public bool StrictGraphicsFree { get; private set; }
        public string DepthBackend { get; private set; }
        public int InteractiveWidth { get; private set; }
        public int InteractiveHeight { get; private set; }
        public string RequestedScene { get; private set; }

        public static CraneRuntimeOptions Parse(string[] args) {
            string fallbackProfile = Array.IndexOf(args, "--crane-replay") >= 0 ?
                "replay-high" : "custom";
            string profile = ReadString(args, "--crane-profile", fallbackProfile).ToLowerInvariant();
            bool trainGpu = profile == "train-gpu";
            bool trainCpu = profile == "train-cpu";
            bool interactiveLow = profile == "interactive-low";
            if (profile != "custom" && profile != "train-gpu" && profile != "train-cpu" &&
                profile != "interactive-low" && profile != "interactive-high" &&
                profile != "replay-high" && profile != "evaluation-high")
                throw new ArgumentException($"Unknown CRANE runtime profile '{profile}'. " +
                    "Expected custom, train-gpu, train-cpu, interactive-low, " +
                    "interactive-high, evaluation-high, or replay-high.");
            bool disableVisual = HasFlag(args, "--crane-disable-visual-sensors");
            string depthBackend = ReadString(args, "--crane-depth-backend",
                trainCpu ? "geometric" : "gpu").ToLowerInvariant();
            if (depthBackend != "gpu" && depthBackend != "geometric" && depthBackend != "off")
                throw new ArgumentException($"Unknown CRANE depth backend '{depthBackend}'. " +
                    "Expected gpu, geometric, or off.");
            string requestedScene = ReadString(args, "--crane-scene", null);
            if (string.IsNullOrEmpty(requestedScene)) {
                if (HasFlag(args, "--crane-land-validation"))
                    requestedScene = "Land Vehicle Validation";
                else if (HasFlag(args, "--crane-aerial-validation"))
                    requestedScene = "Aerial Vehicle Validation";
                else if (HasFlag(args, "--crane-collision-validation"))
                    requestedScene = "Collision Validation";
                else if (HasFlag(args, "--crane-in-place-reset-validation"))
                    requestedScene = "Aerial Vehicle Validation";
            }
            bool disableAllTransport = HasFlag(args, "--crane-disable-ros");
            return new CraneRuntimeOptions {
                ProfileName = profile,
                DisableRosTcp = disableAllTransport || HasFlag(args, "--crane-disable-ros-tcp"),
                DisableSitl = disableAllTransport || HasFlag(args, "--crane-disable-sitl"),
                DisableRgb = trainGpu || trainCpu || disableVisual ||
                    HasFlag(args, "--crane-disable-rgb"),
                DisableDepth = depthBackend == "off" || disableVisual ||
                    HasFlag(args, "--crane-disable-depth"),
                DisableCameraInfo = disableVisual ||
                    HasFlag(args, "--crane-disable-camera-info"),
                DisableDetections = HasFlag(args, "--crane-disable-detections"),
                DisableSpectatorCameras = trainGpu || trainCpu ||
                    HasFlag(args, "--crane-disable-spectator-cameras"),
                LowPresentation = interactiveLow,
                StrictGraphicsFree = trainCpu,
                DepthBackend = depthBackend,
                InteractiveWidth = Math.Max(320,
                    ReadInt(args, "--crane-interactive-width", 960)),
                InteractiveHeight = Math.Max(180,
                    ReadInt(args, "--crane-interactive-height", 540)),
                RequestedScene = requestedScene
            };
        }

        public static string CatalogJson() =>
            "{\"schema\":\"crane-runtime-profile-catalog-v1\",\"profiles\":[" +
            "{\"name\":\"train-gpu\",\"purpose\":\"RGB-free accelerated training with depth, detections and task sensors; graphics-backed\"}," +
            "{\"name\":\"train-cpu\",\"purpose\":\"Strict graphics-free non-aquatic training with geometric 32FC1 depth and camera info\"}," +
            "{\"name\":\"interactive-low\",\"purpose\":\"Weak-hardware Nav2 development with full task physics/sensors and reduced presentation resolution\"}," +
            "{\"name\":\"interactive-high\",\"purpose\":\"Full-fidelity interactive Nav2 and parameter development\"}," +
            "{\"name\":\"evaluation-high\",\"purpose\":\"Full-fidelity evaluation with live physics/controllers\"}," +
            "{\"name\":\"replay-high\",\"purpose\":\"Full-fidelity authoritative playback without live control or physics integration\"}," +
            "{\"name\":\"custom\",\"purpose\":\"Explicit individual subsystem flags\"}]}";

        public void Apply() {
            string activeSceneName = UnityEngine.SceneManagement.SceneManager.GetActiveScene().name;
            if (!string.IsNullOrEmpty(RequestedScene) &&
                !activeSceneName.Equals(RequestedScene, StringComparison.OrdinalIgnoreCase))
                return;
            ulong sceneHandle = UnityEngine.SceneManagement.SceneManager.GetActiveScene()
                .handle.GetRawData();
            if (s_AppliedSceneHandle == sceneHandle && s_AppliedProfile == ProfileName) return;
            s_AppliedSceneHandle = sceneHandle;
            s_AppliedProfile = ProfileName;
            if (StrictGraphicsFree &&
                UnityEngine.Object.FindObjectsByType<WaterSurface>(FindObjectsInactive.Exclude).Length > 0) {
                Debug.LogError("The train-cpu profile cannot run an aquatic scene: HDRP water " +
                    "queries require a graphics-backed camera path. Use train-gpu or a " +
                    "non-aquatic scene; CRANE will not silently disable water physics.");
                Application.Quit(3);
                return;
            }
            foreach (MonoBehaviour component in UnityEngine.Object.FindObjectsByType<MonoBehaviour>(
                         FindObjectsInactive.Include)) {
                string typeName = component.GetType().FullName;
                if ((DisableRosTcp && IsRosTcpTransport(typeName)) ||
                    (DisableSitl && IsSitlTransport(typeName))) {
                    component.enabled = false;
                    continue;
                }

                if ((DisableRgb && typeName == "Sim.Sensors.Vision.ROSCameraAsync") ||
                    ((DisableDepth || DepthBackend == "geometric") &&
                     typeName == "Sim.Sensors.Vision.ROSDepthCameraAsync") ||
                    (DisableDepth &&
                     typeName == "Sim.Sensors.Vision.GeometricDepthCamera") ||
                    (DisableCameraInfo && typeName == "Sim.Sensors.Vision.CameraInfo") ||
                    (DisableDetections && typeName == "Sim.Sensors.Vision.BoundingBox3D")) {
                    component.enabled = false;
                    continue;
                }

                if (typeName == "Sim.Sensors.Vision.ProcessRenderTexture" &&
                    ShouldDisableImageProcessing(component)) {
                    component.enabled = false;
                }
            }

            if (StrictGraphicsFree) DisableAllRenderingCameras();
            else if (DisableSpectatorCameras) DisableSpectatorRendering();
            else if (LowPresentation &&
                     (Screen.width != InteractiveWidth || Screen.height != InteractiveHeight))
                Screen.SetResolution(InteractiveWidth, InteractiveHeight, Screen.fullScreenMode);

            EmitResolvedReport();
        }

        [Serializable] private sealed class ResolvedProfileReport {
            public string schema = "crane-runtime-profile-v1";
            public string profile;
            public string unityVersion;
            public string scene;
            public string graphicsDeviceType;
            public bool graphicsFree;
            public bool strictGraphicsFree;
            public bool rosEnabled;
            public bool sitlEnabled;
            public bool rgbEnabled;
            public bool depthEnabled;
            public string depthBackend;
            public bool cameraInfoEnabled;
            public bool detectionsEnabled;
            public bool spectatorCamerasEnabled;
            public bool lowPresentation;
            public int screenWidth;
            public int screenHeight;
            public int enabledCameras;
            public int enabledSensorCameras;
            public int enabledSpectatorCameras;
            public int enabledWaterDriverCameras;
            public int waterSurfaces;
            public string aquaticGraphicsFreeStatus;
        }

        private void EmitResolvedReport() {
            CameraStatus cameras = GetCameraStatus();
            int waterSurfaces = UnityEngine.Object.FindObjectsByType<WaterSurface>(
                FindObjectsInactive.Exclude).Length;
            bool graphicsFree = SystemInfo.graphicsDeviceType ==
                UnityEngine.Rendering.GraphicsDeviceType.Null;
            var report = new ResolvedProfileReport {
                profile = ProfileName,
                unityVersion = Application.unityVersion,
                scene = UnityEngine.SceneManagement.SceneManager.GetActiveScene().name,
                graphicsDeviceType = SystemInfo.graphicsDeviceType.ToString(),
                graphicsFree = graphicsFree,
                strictGraphicsFree = StrictGraphicsFree,
                rosEnabled = !DisableRosTcp,
                sitlEnabled = !DisableSitl,
                rgbEnabled = !DisableRgb,
                depthEnabled = !DisableDepth,
                depthBackend = DisableDepth ? "off" : DepthBackend,
                cameraInfoEnabled = !DisableCameraInfo,
                detectionsEnabled = !DisableDetections,
                spectatorCamerasEnabled = !DisableSpectatorCameras,
                lowPresentation = LowPresentation,
                screenWidth = Screen.width,
                screenHeight = Screen.height,
                enabledCameras = cameras.Enabled,
                enabledSensorCameras = cameras.Sensor,
                enabledSpectatorCameras = cameras.Spectator,
                enabledWaterDriverCameras = cameras.WaterDriver,
                waterSurfaces = waterSurfaces,
                aquaticGraphicsFreeStatus = waterSurfaces == 0 ? "not-applicable" :
                    graphicsFree ? "blocked-invalid" : "graphics-backed"
            };
            string json = JsonUtility.ToJson(report);
            Debug.Log($"CRANE_RUNTIME_PROFILE_RESOLVED {json}");
            string outputPath = ReadString(Environment.GetCommandLineArgs(),
                "--crane-runtime-report", null);
            if (string.IsNullOrWhiteSpace(outputPath)) return;
            outputPath = Path.GetFullPath(outputPath);
            string directory = Path.GetDirectoryName(outputPath);
            if (!string.IsNullOrEmpty(directory)) Directory.CreateDirectory(directory);
            File.WriteAllText(outputPath, JsonUtility.ToJson(report, true));
        }

        public CameraStatus GetCameraStatus() {
            HashSet<Camera> sensorCameras = CollectEnabledSensorCameras();
            int enabled = 0;
            int sensors = 0;
            int waterDrivers = 0;
            foreach (Camera camera in UnityEngine.Object.FindObjectsByType<Camera>(
                         FindObjectsInactive.Include)) {
                if (!camera.enabled || !camera.gameObject.activeInHierarchy) continue;
                enabled++;
                if (sensorCameras.Contains(camera)) sensors++;
                else if (camera.TryGetComponent<CraneWaterUpdateDriver>(out _)) waterDrivers++;
            }
            return new CameraStatus(enabled, sensors, enabled - sensors - waterDrivers, waterDrivers);
        }

        private static void DisableSpectatorRendering() {
            HashSet<Camera> sensorCameras = CollectEnabledSensorCameras();
            Camera[] allCameras = UnityEngine.Object.FindObjectsByType<Camera>(
                FindObjectsInactive.Include);
            if (sensorCameras.Count == 0 &&
                UnityEngine.Object.FindObjectsByType<WaterSurface>(FindObjectsInactive.Exclude).Length > 0) {
                Camera candidate = Array.Find(allCameras,
                    camera => camera.gameObject.activeInHierarchy && camera.enabled);
                if (candidate == null)
                    candidate = Array.Find(allCameras, camera => camera.gameObject.activeInHierarchy);
                if (candidate != null) {
                    CraneWaterUpdateDriver driver = candidate.GetComponent<CraneWaterUpdateDriver>();
                    if (driver == null) driver = candidate.gameObject.AddComponent<CraneWaterUpdateDriver>();
                    driver.Configure(candidate);
                }
            }
            foreach (Camera camera in allCameras) {
                if (sensorCameras.Contains(camera) ||
                    camera.TryGetComponent<CraneWaterUpdateDriver>(out _)) continue;
                camera.enabled = false;
                if (camera.TryGetComponent(out AudioListener listener)) listener.enabled = false;
            }
        }

        private static void DisableAllRenderingCameras() {
            foreach (Camera camera in UnityEngine.Object.FindObjectsByType<Camera>(
                         FindObjectsInactive.Include)) {
                camera.enabled = false;
                if (camera.TryGetComponent(out AudioListener listener)) listener.enabled = false;
            }
        }

        private static HashSet<Camera> CollectEnabledSensorCameras() {
            var cameras = new HashSet<Camera>();
            foreach (MonoBehaviour component in UnityEngine.Object.FindObjectsByType<MonoBehaviour>(
                         FindObjectsInactive.Include)) {
                if (!component.enabled || !component.gameObject.activeInHierarchy) continue;
                Type type = component.GetType();
                string typeName = type.FullName;
                if (typeName != "Sim.Sensors.Vision.ROSCameraAsync" &&
                    typeName != "Sim.Sensors.Vision.ROSDepthCameraAsync" &&
                    typeName != "Sim.Sensors.Vision.ProcessRenderTexture") continue;
                FieldInfo field = type.GetField("sensorCamera",
                    BindingFlags.Instance | BindingFlags.NonPublic);
                if (field?.FieldType == typeof(Camera) && field.GetValue(component) is Camera camera)
                    cameras.Add(camera);
            }
            return cameras;
        }

        private bool ShouldDisableImageProcessing(MonoBehaviour component) {
            FieldInfo field = component.GetType().GetField("sensorType",
                BindingFlags.Instance | BindingFlags.NonPublic);
            string sensorType = field?.GetValue(component)?.ToString();
            return (DisableRgb && sensorType == "RGB") ||
                   ((DisableDepth || DepthBackend == "geometric") && sensorType == "Depth");
        }

        private static bool IsRosTcpTransport(string typeName) =>
            typeName == "Unity.Robotics.ROSTCPConnector.ROSConnection" ||
            typeName == "Sim.Utils.ROS.ROSClock";

        private static bool IsSitlTransport(string typeName) =>
            typeName == "Sim.Sensors.Nav.MAVROSConnection" ||
            typeName == "Sim.Utils.MAVROS.MAVROSConnection";

        private static bool HasFlag(string[] args, string flag) =>
            Array.IndexOf(args, flag) >= 0;

        private static string ReadString(string[] args, string key, string fallback) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length ? args[index + 1] : fallback;
        }

        private static int ReadInt(string[] args, string key, int fallback) =>
            int.TryParse(ReadString(args, key, null), out int value) ? value : fallback;
    }
}
