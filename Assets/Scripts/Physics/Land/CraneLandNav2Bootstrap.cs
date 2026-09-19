using System;
using System.Globalization;
using System.IO;
using System.Reflection;
using Sim.Physics.Contacts;
using Sim.Utils.ReferenceEnvironments;
using Sim.Utils.ROS;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Physics.Land {
    /// <summary>Builds a deterministic, non-aquatic corridor around the validation rover.</summary>
    internal static class CraneLandNav2Bootstrap {
        [Serializable]
        private sealed class EvaluatorTruth {
            public string schema = "crane-land-corridor-truth-v1";
            public int seed;
            public float corridorWidth;
            public float corridorLength;
            public string blocker;
            public float blockerDistance;
            public float blockerWidth;
            public string blockerSemanticId;
            public float blockerRemovalAfterSeconds = -1f;
            public double blockerRemovalScheduledSimulationTime = -1d;
            public double blockerRemovalActualSimulationTime = -1d;
            public bool blockerRemoved;
            public string platform;
            public Vector3 startPosition;
        }

        private static bool enabled;
        private static string[] arguments;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            arguments = Environment.GetCommandLineArgs();
            enabled = Array.IndexOf(arguments, "--crane-land-nav2") >= 0;
            if (enabled) SceneManager.sceneLoaded += OnSceneLoaded;
        }

        private static void OnSceneLoaded(Scene scene, LoadSceneMode mode) {
            if (!enabled) return;
            bool ackermannScene = scene.name.Equals("Land Vehicle Validation",
                StringComparison.OrdinalIgnoreCase);
            bool differentialScene = scene.name.Equals("TurtleBot3 Warehouse Validation",
                StringComparison.OrdinalIgnoreCase);
            if (!ackermannScene && !differentialScene) return;
            AckermannRoverDynamics rover = ackermannScene
                ? UnityEngine.Object.FindAnyObjectByType<AckermannRoverDynamics>(
                    FindObjectsInactive.Exclude)
                : null;
            DifferentialDriveDynamics differential = differentialScene
                ? UnityEngine.Object.FindAnyObjectByType<DifferentialDriveDynamics>(
                    FindObjectsInactive.Exclude)
                : null;
            if (rover == null && differential == null)
                throw new MissingReferenceException("Land Nav2 fixture robot is missing.");

            // The TurtleBot3 scene's recognizable warehouse remains the normal reference scene.
            // Corridor experiments explicitly replace only its generated environment root while
            // retaining the existing robot, dynamics, sensors, ROS integration, and scene setup.
            if (differentialScene) {
                CraneReferenceWarehouse warehouse = UnityEngine.Object.FindAnyObjectByType<
                    CraneReferenceWarehouse>(FindObjectsInactive.Include);
                if (warehouse != null) warehouse.gameObject.SetActive(false);
            }

            // This validation scene does not contain the ROSClock object present in the aquatic
            // scenes. Nav2 uses simulated time, so without /clock progress deadlines and timed
            // recovery behaviors never advance even though sensor and command topics are active.
            if (UnityEngine.Object.FindAnyObjectByType<ROSClock>() == null) {
                var clockHost = new GameObject("CRANE Land ROS Clock");
                clockHost.AddComponent<ROSClock>();
            }

            float width = ReadFloat("--crane-land-corridor-width", 4f);
            float length = ReadFloat("--crane-land-corridor-length", 20f);
            float blockerDistance = ReadFloat("--crane-land-blocker-distance", 6f);
            float blockerWidth = ReadFloat("--crane-land-blocker-width", width);
            float blockerRemoveAfter = ReadFloat("--crane-land-blocker-remove-after", -1f);
            string blocker = ReadString("--crane-land-blocker", "none").ToLowerInvariant();
            if (blocker != "none" && blocker != "partial" && blocker != "full")
                throw new ArgumentException($"Unknown land blocker mode '{blocker}'.");
            if (blocker == "partial") blockerWidth = Mathf.Min(blockerWidth, width * 0.55f);
            if (blocker == "full") blockerWidth = width;

            Rigidbody body = rover != null
                ? rover.GetComponent<Rigidbody>()
                : differential.GetComponent<Rigidbody>();
            body.position = new Vector3(0f, differentialScene ? 0.08f : 0.65f, 0f);
            body.rotation = Quaternion.identity;
            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
            if (rover != null) rover.ResetActuators();
            else differential.SetCommand(0f, 0f);

            const float wallThickness = 0.25f;
            const float wallHeight = 2f;
            if (differentialScene) {
                CreateObstacle("Corridor Floor", "corridor-floor", "traversable-floor",
                    new Vector3(0f, -0.10f, length * 0.5f),
                    new Vector3(width + 2f, 0.20f, length + 4f));
            }
            CreateObstacle("Corridor Left Wall", "corridor-wall-left", "boundary-wall",
                new Vector3(-width * 0.5f - wallThickness * 0.5f, wallHeight * 0.5f,
                    length * 0.5f),
                new Vector3(wallThickness, wallHeight, length + 4f));
            CreateObstacle("Corridor Right Wall", "corridor-wall-right", "boundary-wall",
                new Vector3(width * 0.5f + wallThickness * 0.5f, wallHeight * 0.5f,
                    length * 0.5f),
                new Vector3(wallThickness, wallHeight, length + 4f));
            GameObject blockerObject = null;
            if (blocker != "none") {
                float x = blocker == "partial" ? -width * 0.5f + blockerWidth * 0.5f : 0f;
                blockerObject = CreateObstacle("Corridor Blocker", "corridor-blocker",
                    "controlled-obstacle",
                    new Vector3(x, wallHeight * 0.5f, blockerDistance),
                    new Vector3(blockerWidth, wallHeight, wallThickness));
            }

            if (ackermannScene) {
                var lidarHost = new GameObject("lidar_link");
                lidarHost.transform.SetParent(rover.transform, false);
                lidarHost.transform.localPosition = new Vector3(0f, 0.65f, 0.65f);
                Type lidarType = Type.GetType("Sim.Sensors.Lidar.Lidar2D, SensorsAssembly", true);
                Component lidar = lidarHost.AddComponent(lidarType);
                MethodInfo configure = lidarType.GetMethod("Configure") ??
                    throw new MissingMethodException(lidarType.FullName, "Configure");
                configure.Invoke(lidar, new object[] {
                    -180f, 180f, 1f, 0.1f, 20f, 180, false, "/scan", "lidar_link", 10f
                });
            }

            string truthPath = ReadString("--crane-land-evaluator-output", null);
            var truth = new EvaluatorTruth {
                seed = ReadInt("--crane-seed", 1),
                corridorWidth = width,
                corridorLength = length,
                blocker = blocker,
                blockerDistance = blockerDistance,
                blockerWidth = blocker == "none" ? 0f : blockerWidth,
                blockerSemanticId = blockerObject == null ? string.Empty : "corridor-blocker",
                blockerRemovalAfterSeconds = blockerObject == null ? -1f : blockerRemoveAfter,
                platform = differentialScene ? "turtlebot3-waffle-differential" :
                    "reference-ackermann-rover",
                startPosition = body.position
            };
            if (!string.IsNullOrWhiteSpace(truthPath)) {
                truthPath = Path.GetFullPath(truthPath);
                string directory = Path.GetDirectoryName(truthPath);
                if (!string.IsNullOrEmpty(directory)) Directory.CreateDirectory(directory);
                WriteTruth(truthPath, truth);
            }
            if (blockerObject != null && blockerRemoveAfter >= 0f) {
                var removalHost = new GameObject("CRANE Timed Blocker Intervention");
                var removal = removalHost.AddComponent<CraneTimedBlockerRemoval>();
                removal.Configure(blockerObject, blockerRemoveAfter, actualTime => {
                    truth.blockerRemoved = true;
                    truth.blockerRemovalActualSimulationTime = actualTime;
                    if (!string.IsNullOrWhiteSpace(truthPath)) WriteTruth(truthPath, truth);
                });
                truth.blockerRemovalScheduledSimulationTime = removal.ScheduledSimulationTime;
                if (!string.IsNullOrWhiteSpace(truthPath)) WriteTruth(truthPath, truth);
            }
            Debug.Log($"CRANE_LAND_NAV2_READY width={width:R} length={length:R} " +
                      $"blocker={blocker} blockerRemoveAfter={blockerRemoveAfter:R} " +
                      $"platform={truth.platform} lidar=/scan");
        }

        private static GameObject CreateObstacle(string name, string semanticId,
            string semanticRole, Vector3 position, Vector3 scale) {
            GameObject obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
            obstacle.name = name;
            obstacle.layer = CraneCollisionLayers.Environment;
            obstacle.transform.position = position;
            obstacle.transform.localScale = scale;
            obstacle.AddComponent<CraneSemanticIdentity>().Configure(
                semanticId, semanticRole, "crane-land-corridor-v1");
            return obstacle;
        }

        private static void WriteTruth(string path, EvaluatorTruth truth) =>
            File.WriteAllText(path, JsonUtility.ToJson(truth, true));

        private static string ReadString(string key, string fallback) {
            int index = Array.IndexOf(arguments, key);
            return index >= 0 && index + 1 < arguments.Length ? arguments[index + 1] : fallback;
        }

        private static float ReadFloat(string key, float fallback) =>
            float.TryParse(ReadString(key, null), NumberStyles.Float,
                CultureInfo.InvariantCulture, out float value) ? value : fallback;

        private static int ReadInt(string key, int fallback) =>
            int.TryParse(ReadString(key, null), NumberStyles.Integer,
                CultureInfo.InvariantCulture, out int value) ? value : fallback;
    }
}
