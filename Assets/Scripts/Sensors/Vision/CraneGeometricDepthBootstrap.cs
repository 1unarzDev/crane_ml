using System;
using System.Collections;
using System.Diagnostics;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEngine.SceneManagement;
using Debug = UnityEngine.Debug;

namespace Sim.Sensors.Vision {
    /// <summary>Installs render-independent depth replacements and the null-graphics fixture.</summary>
    internal sealed class CraneGeometricDepthBootstrap : MonoBehaviour {
        [Serializable] private sealed class ValidationResult {
            public string schema = "crane-geometric-depth-validation-v1";
            public string unityVersion;
            public string graphicsDevice;
            public int width;
            public int height;
            public int rowStep;
            public string encoding = "32FC1";
            public string distanceConvention = "optical-axis-z-metres";
            public string invalidConvention = "quiet-NaN";
            public float foregroundDepth;
            public float backgroundDepth;
            public float movedDepth;
            public float thinGeometryDepth;
            public bool invalidCorner;
            public bool foregroundValid;
            public bool backgroundValid;
            public bool movingGeometryValid;
            public bool thinGeometryValid;
            public bool graphicsFree;
            public int benchmarkCaptures;
            public int benchmarkWidth;
            public int benchmarkHeight;
            public double meanCaptureMilliseconds;
            public double millionRaysPerSecond;
            public bool valid;
        }

        private bool validate;
        private bool installBenchmarkFixture;
        private string outputPath;

        // Install before the profile/benchmark BeforeSceneLoad hooks so the geometric replacement
        // exists when camera ownership is classified and an aquatic water driver is selected.
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterAssembliesLoaded)]
        private static void Install() {
            string[] args = Environment.GetCommandLineArgs();
            string profile = ReadString(args, "--crane-profile", string.Empty);
            string backend = ReadString(args, "--crane-depth-backend",
                profile.Equals("train-cpu", StringComparison.OrdinalIgnoreCase)
                    ? "geometric" : "gpu");
            bool validate = args.Contains("--crane-geometric-depth-validation");
            bool fixture = args.Contains("--crane-geometric-depth-fixture");
            if (!backend.Equals("geometric", StringComparison.OrdinalIgnoreCase) && !validate &&
                !fixture)
                return;
            var host = new GameObject("CRANE Geometric Depth Bootstrap");
            DontDestroyOnLoad(host);
            host.AddComponent<CraneGeometricDepthBootstrap>();
        }

        private void Awake() {
            string[] args = Environment.GetCommandLineArgs();
            validate = args.Contains("--crane-geometric-depth-validation");
            installBenchmarkFixture = args.Contains("--crane-geometric-depth-fixture");
            outputPath = ReadString(args, "--crane-output",
                Path.Combine(Application.persistentDataPath,
                    "crane-geometric-depth-validation.json"));
            SceneManager.sceneLoaded += OnSceneLoaded;
        }

        private void OnSceneLoaded(Scene scene, LoadSceneMode mode) => ReplaceGpuDepthSensors();

        private IEnumerator Start() {
            string requestedScene = ReadString(Environment.GetCommandLineArgs(),
                "--crane-scene", string.Empty);
            while (!string.IsNullOrEmpty(requestedScene) &&
                   !SceneManager.GetActiveScene().name.Equals(requestedScene,
                       StringComparison.OrdinalIgnoreCase))
                yield return null;
            ReplaceGpuDepthSensors();
            if (installBenchmarkFixture && !validate) CreateBenchmarkFixture();
            if (!validate) yield break;
            yield return RunValidation();
        }

        private static void ReplaceGpuDepthSensors() {
            foreach (ROSDepthCameraAsync gpu in FindObjectsByType<ROSDepthCameraAsync>(
                         FindObjectsInactive.Include)) {
                GeometricDepthCamera geometric = gpu.GetComponent<GeometricDepthCamera>();
                if (geometric == null) geometric = gpu.gameObject.AddComponent<GeometricDepthCamera>();
                geometric.Configure(gpu.SensorCamera, gpu.ImageWidth, gpu.ImageHeight,
                    gpu.TopicName, gpu.FrameId, gpu.PublishRateHz);
                gpu.enabled = false;
            }
        }

        private IEnumerator RunValidation() {
            const int width = 65;
            const int height = 49;
            var root = new GameObject("CRANE Geometric Depth Validation Fixture");
            root.transform.position = new Vector3(1000f, 1000f, 1000f);

            var cameraObject = new GameObject("geometric-depth-optical-frame");
            cameraObject.transform.SetParent(root.transform, false);
            var camera = cameraObject.AddComponent<Camera>();
            camera.enabled = false;
            camera.fieldOfView = 60f;
            camera.nearClipPlane = 0.2f;
            camera.farClipPlane = 20f;
            var sensor = cameraObject.AddComponent<GeometricDepthCamera>();
            sensor.Configure(camera, width, height, "validation/depth",
                "validation_depth_optical", 10f, true);

            GameObject wall = CreateBox(root.transform, "background", new Vector3(0, 0, 10),
                new Vector3(8, 8, 1));
            GameObject foreground = CreateBox(root.transform, "foreground", new Vector3(0, 0, 5),
                new Vector3(2, 2, 1));
            yield return null;
            Physics.SyncTransforms();

            sensor.CaptureNow();
            int centerX = width / 2;
            int centerY = height / 2;
            float foregroundDepth = sensor.GetDepth(centerX, centerY);
            float backgroundDepth = sensor.GetDepth(centerX + 12, centerY);
            bool invalidCorner = float.IsNaN(sensor.GetDepth(0, 0));

            foreground.transform.localPosition = new Vector3(0, 0, 7);
            Physics.SyncTransforms();
            sensor.CaptureNow();
            float movedDepth = sensor.GetDepth(centerX, centerY);

            foreground.SetActive(false);
            GameObject thin = CreateBox(root.transform, "thin-geometry", new Vector3(0, 0, 3),
                new Vector3(0.08f, 2, 0.04f));
            Physics.SyncTransforms();
            sensor.CaptureNow();
            float thinDepth = sensor.GetDepth(centerX, centerY);

            string[] args = Environment.GetCommandLineArgs();
            int benchmarkWidth = ReadPositiveInt(args, "--crane-depth-width", 320);
            int benchmarkHeight = ReadPositiveInt(args, "--crane-depth-height", 180);
            int benchmarkCaptures = ReadPositiveInt(args,
                "--crane-depth-benchmark-captures", 20);
            sensor.Configure(camera, benchmarkWidth, benchmarkHeight, "validation/depth",
                "validation_depth_optical", 10f, true);
            sensor.CaptureNow();
            var stopwatch = Stopwatch.StartNew();
            for (int i = 0; i < benchmarkCaptures; i++) sensor.CaptureNow();
            stopwatch.Stop();
            double meanMilliseconds = stopwatch.Elapsed.TotalMilliseconds / benchmarkCaptures;
            double raysPerSecond = benchmarkWidth * benchmarkHeight /
                (meanMilliseconds / 1000.0);

            var result = new ValidationResult {
                unityVersion = Application.unityVersion,
                graphicsDevice = SystemInfo.graphicsDeviceType.ToString(),
                width = width,
                height = height,
                rowStep = width * sizeof(float),
                foregroundDepth = foregroundDepth,
                backgroundDepth = backgroundDepth,
                movedDepth = movedDepth,
                thinGeometryDepth = thinDepth,
                invalidCorner = invalidCorner,
                foregroundValid = Mathf.Abs(foregroundDepth - 4.5f) < 0.02f,
                backgroundValid = Mathf.Abs(backgroundDepth - 9.5f) < 0.02f,
                movingGeometryValid = Mathf.Abs(movedDepth - 6.5f) < 0.02f,
                thinGeometryValid = Mathf.Abs(thinDepth - 2.98f) < 0.02f,
                graphicsFree = SystemInfo.graphicsDeviceType ==
                    UnityEngine.Rendering.GraphicsDeviceType.Null,
                benchmarkCaptures = benchmarkCaptures,
                benchmarkWidth = benchmarkWidth,
                benchmarkHeight = benchmarkHeight,
                meanCaptureMilliseconds = meanMilliseconds,
                millionRaysPerSecond = raysPerSecond / 1_000_000.0
            };
            result.valid = result.foregroundValid && result.backgroundValid &&
                result.movingGeometryValid && result.thinGeometryValid && result.invalidCorner &&
                result.graphicsFree;

            string fullPath = Path.GetFullPath(outputPath);
            string directory = Path.GetDirectoryName(fullPath);
            if (!string.IsNullOrEmpty(directory)) Directory.CreateDirectory(directory);
            File.WriteAllText(fullPath, JsonUtility.ToJson(result, true));
            Debug.Log($"CRANE_GEOMETRIC_DEPTH_VALIDATION {JsonUtility.ToJson(result)}");
            Destroy(thin);
            Destroy(wall);
            Destroy(root);
            Application.Quit(result.valid ? 0 : 4);
        }

        private static void CreateBenchmarkFixture() {
            string[] args = Environment.GetCommandLineArgs();
            if (args.Contains("--crane-disable-depth") ||
                args.Contains("--crane-disable-visual-sensors")) return;
            int width = ReadPositiveInt(args, "--crane-depth-width", 320);
            int height = ReadPositiveInt(args, "--crane-depth-height", 180);
            float rate = ReadPositiveFloat(args, "--crane-depth-hz", 15f);
            var root = new GameObject("CRANE Geometric Depth Benchmark Fixture");
            root.transform.position = new Vector3(1000f, 1000f, 1000f);

            var cameraObject = new GameObject("geometric-depth-benchmark-optical-frame");
            cameraObject.transform.SetParent(root.transform, false);
            var camera = cameraObject.AddComponent<Camera>();
            camera.enabled = false;
            camera.fieldOfView = 70f;
            camera.nearClipPlane = 0.2f;
            camera.farClipPlane = 50f;
            var sensor = cameraObject.AddComponent<GeometricDepthCamera>();
            sensor.Configure(camera, width, height, "benchmark/depth/image_rect_raw",
                "benchmark_depth_optical", rate);

            CreateBox(root.transform, "benchmark-background", new Vector3(0, 0, 20),
                new Vector3(30, 20, 1));
            for (int y = -2; y <= 2; y++) {
                for (int x = -3; x <= 3; x++) {
                    float z = 5f + ((x + y + 12) % 5) * 2.5f;
                    CreateBox(root.transform, $"benchmark-obstacle-{x}-{y}",
                        new Vector3(x * 2.2f, y * 1.8f, z), new Vector3(1.2f, 1.2f, 0.8f));
                }
            }
            Physics.SyncTransforms();
        }

        private static GameObject CreateBox(Transform parent, string name, Vector3 position,
            Vector3 scale) {
            GameObject box = GameObject.CreatePrimitive(PrimitiveType.Cube);
            box.name = name;
            box.transform.SetParent(parent, false);
            box.transform.localPosition = position;
            box.transform.localScale = scale;
            Renderer renderer = box.GetComponent<Renderer>();
            if (renderer != null) renderer.enabled = false;
            return box;
        }

        private void OnDestroy() => SceneManager.sceneLoaded -= OnSceneLoaded;

        private static string ReadString(string[] args, string key, string fallback) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length ? args[index + 1] : fallback;
        }

        private static int ReadPositiveInt(string[] args, string key, int fallback) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length &&
                   int.TryParse(args[index + 1], out int value) && value > 0
                ? value : fallback;
        }

        private static float ReadPositiveFloat(string[] args, string key, float fallback) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length &&
                   float.TryParse(args[index + 1], out float value) && value > 0
                ? value : fallback;
        }
    }
}
