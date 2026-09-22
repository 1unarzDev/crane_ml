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
            public bool blockerActiveInitially;
            public float blockerActivationAfterSeconds = -1f;
            public double blockerActivationScheduledSimulationTime = -1d;
            public double blockerActivationActualSimulationTime = -1d;
            public bool blockerActivated;
            public float blockerRemovalAfterSeconds = -1f;
            public double blockerRemovalScheduledSimulationTime = -1d;
            public double blockerRemovalActualSimulationTime = -1d;
            public bool blockerRemoved;
            public float mobilityHoldAfterSeconds = -1f;
            public float mobilityReleaseAfterSeconds = -1f;
            public double mobilityHoldScheduledSimulationTime = -1d;
            public double mobilityHoldActualSimulationTime = -1d;
            public double mobilityReleaseScheduledSimulationTime = -1d;
            public double mobilityReleaseActualSimulationTime = -1d;
            public bool mobilityHeld;
            public bool mobilityReleased;
            public string environmentId;
            public string scenarioId;
            public bool referenceEnvironmentPreserved;
            public string platform;
            public Vector3 startPosition;
            public string warehouseScenarioId;
            public string warehouseRouteId;
            public int warehouseScenarioSeed = -1;
            public string warehouseScenarioRobot;
            public string warehouseExpectedChallenge;
            public string warehouseExpectedBroadOutcome;
            public string[] warehouseObstacleSemanticIds = Array.Empty<string>();
            public bool[] warehouseObstacleActive = Array.Empty<bool>();
            public double[] warehouseObstacleScheduledActivationSimulationTime =
                Array.Empty<double>();
            public double[] warehouseObstacleActualActivationSimulationTime =
                Array.Empty<double>();
            public double[] warehouseObstacleScheduledRemovalSimulationTime =
                Array.Empty<double>();
            public double[] warehouseObstacleActualRemovalSimulationTime =
                Array.Empty<double>();
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
            bool turtlebotScene = scene.name.Equals("TurtleBot3 Warehouse Validation",
                StringComparison.OrdinalIgnoreCase);
            bool clearpathScene = scene.name.Equals("Clearpath Pipeline Validation",
                StringComparison.OrdinalIgnoreCase);
            bool differentialScene = turtlebotScene || clearpathScene;
            bool preserveRequested = Array.IndexOf(arguments,
                "--crane-preserve-reference-environment") >= 0;
            bool preserveReferenceEnvironment = clearpathScene ||
                (turtlebotScene && preserveRequested);
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
            CraneReferenceWarehouse warehouse = turtlebotScene
                ? UnityEngine.Object.FindAnyObjectByType<CraneReferenceWarehouse>(
                    FindObjectsInactive.Include)
                : null;
            if (turtlebotScene && !preserveReferenceEnvironment) {
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
            float blockerEnableAfter = ReadFloat("--crane-land-blocker-enable-after", -1f);
            float blockerRemoveAfter = ReadFloat("--crane-land-blocker-remove-after", -1f);
            float mobilityHoldAfter = ReadFloat("--crane-land-mobility-hold-after", -1f);
            float mobilityReleaseAfter = ReadFloat("--crane-land-mobility-release-after", -1f);
            string provingGroundLayout =
                ReadString("--crane-land-proving-ground-layout", null);
            string provingGroundCatalog =
                ReadString("--crane-land-proving-ground-catalog",
                    CraneLandProvingGround.DefaultCatalog);
            string blocker = ReadString("--crane-land-blocker", "none").ToLowerInvariant();
            if (blocker != "none" && blocker != "partial" && blocker != "full")
                throw new ArgumentException($"Unknown land blocker mode '{blocker}'.");
            if (blocker == "partial") blockerWidth = Mathf.Min(blockerWidth, width * 0.55f);
            if (blocker == "full") blockerWidth = width;

            Rigidbody body = rover != null
                ? rover.GetComponent<Rigidbody>()
                : differential.GetComponent<Rigidbody>();
            if (!preserveReferenceEnvironment) {
                body.position = new Vector3(0f, differentialScene ? 0.08f : 0.65f, 0f);
                body.rotation = Quaternion.identity;
            }
            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
            if (rover != null) rover.ResetActuators();
            else differential.SetCommand(0f, 0f);

            string truthPath = ReadString("--crane-land-evaluator-output", null);
            if (!string.IsNullOrWhiteSpace(truthPath)) {
                truthPath = Path.GetFullPath(truthPath);
                string directory = Path.GetDirectoryName(truthPath);
                if (!string.IsNullOrEmpty(directory)) Directory.CreateDirectory(directory);
            }
            if (!string.IsNullOrWhiteSpace(provingGroundLayout)) {
                if (!turtlebotScene || preserveReferenceEnvironment)
                    throw new ArgumentException(
                        "The land proving ground requires the replaceable TurtleBot3 scene.");
                if (blocker != "none" || blockerEnableAfter >= 0f ||
                    blockerRemoveAfter >= 0f || mobilityHoldAfter >= 0f ||
                    mobilityReleaseAfter >= 0f)
                    throw new ArgumentException(
                        "Proving-ground layouts cannot be combined with legacy corridor interventions.");
                CraneLandProvingGround.EvaluatorTruth provingTruth =
                    CraneLandProvingGround.Build(provingGroundCatalog, provingGroundLayout,
                        ReadInt("--crane-seed", -1), body, differential, truthPath);
                Debug.Log($"CRANE_LAND_NAV2_READY provingGround={provingTruth.layoutId} " +
                          $"seed={provingTruth.seed} platform=turtlebot3-waffle-differential " +
                          "lidar=/scan");
                return;
            }

            const float wallThickness = 0.25f;
            const float wallHeight = 2f;
            GameObject blockerObject = null;
            if (!preserveReferenceEnvironment) {
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
                if (blocker != "none") {
                    float x = blocker == "partial" ? -width * 0.5f + blockerWidth * 0.5f : 0f;
                    blockerObject = CreateObstacle("Corridor Blocker", "corridor-blocker",
                        "controlled-obstacle",
                        new Vector3(x, wallHeight * 0.5f, blockerDistance),
                        new Vector3(blockerWidth, wallHeight, wallThickness));
                }
            }
            else if (blocker != "none")
                throw new ArgumentException(
                    "Imported reference environments do not accept synthetic corridor blockers.");
            if (blockerObject == null &&
                (blockerEnableAfter >= 0f || blockerRemoveAfter >= 0f))
                throw new ArgumentException(
                    "Timed blocker intervention requires a non-none blocker mode.");

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

            var truth = new EvaluatorTruth {
                seed = ReadInt("--crane-seed", 1),
                corridorWidth = width,
                corridorLength = length,
                blocker = blocker,
                blockerDistance = blockerDistance,
                blockerWidth = blocker == "none" ? 0f : blockerWidth,
                blockerSemanticId = blockerObject == null ? string.Empty : "corridor-blocker",
                blockerActiveInitially = blockerObject != null && blockerEnableAfter < 0f,
                blockerActivationAfterSeconds = blockerObject == null ? -1f : blockerEnableAfter,
                blockerRemovalAfterSeconds = blockerObject == null ? -1f : blockerRemoveAfter,
                mobilityHoldAfterSeconds = mobilityHoldAfter,
                mobilityReleaseAfterSeconds = mobilityReleaseAfter,
                environmentId = clearpathScene ? "clearpath-pipeline-2.9.4-v1" :
                    preserveReferenceEnvironment ? CraneReferenceWarehouse.EnvironmentId :
                    "crane-land-corridor-v1",
                scenarioId = ReadString("--crane-land-scenario-id",
                    preserveReferenceEnvironment ? "unspecified-reference-route" :
                    "controlled-corridor"),
                referenceEnvironmentPreserved = preserveReferenceEnvironment,
                platform = clearpathScene ? "clearpath-jackal-class-differential" :
                    differentialScene ? "turtlebot3-waffle-differential" :
                    "reference-ackermann-rover",
                startPosition = body.position
            };
            string warehouseScenarioId = ReadString("--crane-warehouse-scenario-id", null);
            if (!string.IsNullOrWhiteSpace(warehouseScenarioId)) {
                if (!turtlebotScene || !preserveReferenceEnvironment || warehouse == null)
                    throw new ArgumentException(
                        "Warehouse scenarios require the preserved TurtleBot3 warehouse scene.");
                warehouse.ApplyScenario(warehouseScenarioId, state => {
                    CopyWarehouseScenarioTruth(truth, state);
                    if (!string.IsNullOrWhiteSpace(truthPath)) WriteTruth(truthPath, truth);
                });
            }
            if (!string.IsNullOrWhiteSpace(truthPath)) {
                WriteTruth(truthPath, truth);
            }
            if (blockerObject != null &&
                (blockerEnableAfter >= 0f || blockerRemoveAfter >= 0f)) {
                var removalHost = new GameObject("CRANE Timed Blocker Intervention");
                var removal = removalHost.AddComponent<CraneTimedBlockerRemoval>();
                removal.Configure(blockerObject, blockerEnableAfter, blockerRemoveAfter,
                    actualTime => {
                        truth.blockerActivated = true;
                        truth.blockerActivationActualSimulationTime = actualTime;
                        if (!string.IsNullOrWhiteSpace(truthPath)) WriteTruth(truthPath, truth);
                    }, actualTime => {
                        truth.blockerRemoved = true;
                        truth.blockerRemovalActualSimulationTime = actualTime;
                        if (!string.IsNullOrWhiteSpace(truthPath)) WriteTruth(truthPath, truth);
                    });
                truth.blockerActivationScheduledSimulationTime =
                    removal.ScheduledActivationSimulationTime;
                truth.blockerRemovalScheduledSimulationTime = removal.ScheduledSimulationTime;
                if (!string.IsNullOrWhiteSpace(truthPath)) WriteTruth(truthPath, truth);
            }
            if (mobilityHoldAfter < 0f && mobilityReleaseAfter >= 0f)
                throw new ArgumentException(
                    "Mobility release requires a configured hold boundary.");
            if (mobilityHoldAfter >= 0f) {
                var mobilityHost = new GameObject("CRANE Timed Mobility Hold");
                var mobility = mobilityHost.AddComponent<CraneTimedMobilityHold>();
                mobility.Configure(body, mobilityHoldAfter, mobilityReleaseAfter,
                    actualTime => {
                        truth.mobilityHeld = true;
                        truth.mobilityHoldActualSimulationTime = actualTime;
                        if (!string.IsNullOrWhiteSpace(truthPath)) WriteTruth(truthPath, truth);
                    }, actualTime => {
                        truth.mobilityReleased = true;
                        truth.mobilityReleaseActualSimulationTime = actualTime;
                        if (!string.IsNullOrWhiteSpace(truthPath)) WriteTruth(truthPath, truth);
                    });
                truth.mobilityHoldScheduledSimulationTime =
                    mobility.ScheduledHoldSimulationTime;
                truth.mobilityReleaseScheduledSimulationTime =
                    mobility.ScheduledReleaseSimulationTime;
                if (!string.IsNullOrWhiteSpace(truthPath)) WriteTruth(truthPath, truth);
            }
            Debug.Log($"CRANE_LAND_NAV2_READY width={width:R} length={length:R} " +
                      $"blocker={blocker} blockerEnableAfter={blockerEnableAfter:R} " +
                      $"blockerRemoveAfter={blockerRemoveAfter:R} " +
                      $"mobilityHoldAfter={mobilityHoldAfter:R} " +
                      $"mobilityReleaseAfter={mobilityReleaseAfter:R} " +
                      $"preserveReferenceEnvironment={preserveReferenceEnvironment} " +
                      $"warehouseScenario={warehouseScenarioId ?? "none"} " +
                      $"platform={truth.platform} lidar=/scan");
        }

        private static void CopyWarehouseScenarioTruth(EvaluatorTruth truth,
            CraneReferenceWarehouse.ScenarioRuntimeState state) {
            truth.warehouseScenarioId = state.scenarioId;
            truth.warehouseRouteId = state.routeId;
            truth.warehouseScenarioSeed = state.seed;
            truth.warehouseScenarioRobot = state.robot;
            truth.warehouseExpectedChallenge = state.expectedChallenge;
            truth.warehouseExpectedBroadOutcome = state.expectedBroadOutcome;
            int count = state.obstacles.Length;
            truth.warehouseObstacleSemanticIds = new string[count];
            truth.warehouseObstacleActive = new bool[count];
            truth.warehouseObstacleScheduledActivationSimulationTime = new double[count];
            truth.warehouseObstacleActualActivationSimulationTime = new double[count];
            truth.warehouseObstacleScheduledRemovalSimulationTime = new double[count];
            truth.warehouseObstacleActualRemovalSimulationTime = new double[count];
            for (int index = 0; index < count; index++) {
                CraneReferenceWarehouse.ScenarioObstacleRuntimeState obstacle =
                    state.obstacles[index];
                truth.warehouseObstacleSemanticIds[index] = obstacle.semanticId;
                truth.warehouseObstacleActive[index] = obstacle.active;
                truth.warehouseObstacleScheduledActivationSimulationTime[index] =
                    obstacle.scheduledActivationSimulationTime;
                truth.warehouseObstacleActualActivationSimulationTime[index] =
                    obstacle.actualActivationSimulationTime;
                truth.warehouseObstacleScheduledRemovalSimulationTime[index] =
                    obstacle.scheduledRemovalSimulationTime;
                truth.warehouseObstacleActualRemovalSimulationTime[index] =
                    obstacle.actualRemovalSimulationTime;
            }
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
