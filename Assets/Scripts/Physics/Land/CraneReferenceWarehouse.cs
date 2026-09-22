using System;
using System.Collections.Generic;
using Sim.Physics.Contacts;
using Sim.Utils.ReferenceEnvironments;
using UnityEngine;

namespace Sim.Physics.Land {
    /// <summary>
    /// Builds the authoritative industrial warehouse from a versioned resource manifest.
    /// Collision/semantics are authoritative; presentation is non-colliding and replaceable.
    /// </summary>
    public sealed class CraneReferenceWarehouse : MonoBehaviour {
        public const string EnvironmentId = "crane-industrial-warehouse-v2";
        public const string ManifestResource =
            "ReferenceEnvironments/unity_turtlebot3_industrial_warehouse_v2";

        [Serializable]
        private sealed class Manifest {
            public string schema;
            public string environmentId;
            public string generatorVersion;
            public Canonical canonical;
            public Route[] routes;
            public Scenario[] scenarios;
        }

        [Serializable]
        private sealed class Canonical {
            public float[] dimensionsMeters;
            public Box[] boxes;
            public Region[] regions;
        }

        [Serializable]
        private sealed class Box {
            public string id;
            public string role;
            public float[] center;
            public float[] size;
            public float[] color;
            public float seedJitterX;
        }

        [Serializable]
        private sealed class Region {
            public string id;
            public string role;
            public float[] position;
        }

        [Serializable]
        private sealed class Route {
            public string id;
            public float[] start;
            public float[] goal;
            public float approximateLengthMeters;
            public string[] alternatives;
            public string[] relevantObstacles;
            public string expectedChallenge;
            public string expectedBroadOutcome;
        }

        [Serializable]
        private sealed class Scenario {
            public string id;
            public string routeId;
            public int seed;
            public string robot;
            public ScenarioObstacle[] obstacles;
            public string expectedChallenge;
            public string expectedBroadOutcome;
        }

        [Serializable]
        private sealed class ScenarioObstacle {
            public string id;
            public string role;
            public float[] center;
            public float[] size;
            public float[] color;
            public bool activeInitially;
            public float activationAfterSeconds;
            public float removalAfterSeconds;
        }

        public sealed class ScenarioObstacleRuntimeState {
            public string semanticId;
            public bool active;
            public double scheduledActivationSimulationTime = -1d;
            public double actualActivationSimulationTime = -1d;
            public double scheduledRemovalSimulationTime = -1d;
            public double actualRemovalSimulationTime = -1d;
        }

        public sealed class ScenarioRuntimeState {
            public string scenarioId;
            public string routeId;
            public int seed;
            public string robot;
            public string expectedChallenge;
            public string expectedBroadOutcome;
            public ScenarioObstacleRuntimeState[] obstacles;
        }

        [SerializeField] private int seed = 2001;
        [SerializeField] private bool createVisualLayer = true;
        [SerializeField] private TextAsset manifestAsset;

        private void Awake() {
            if (transform.Find("CanonicalGeometry") == null) Generate();
        }

        public void Configure(int configuredSeed, bool visuals, TextAsset configuredManifest) {
            seed = configuredSeed;
            createVisualLayer = visuals;
            manifestAsset = configuredManifest;
        }

        public void Generate() {
            Manifest manifest = LoadManifest();
            ValidateManifest(manifest);

            Transform oldCanonical = transform.Find("CanonicalGeometry");
            Transform oldVisual = transform.Find("VisualPresentation");
            if (oldCanonical != null) DestroyGeneratedObject(oldCanonical.gameObject);
            if (oldVisual != null) DestroyGeneratedObject(oldVisual.gameObject);
            var canonical = new GameObject("CanonicalGeometry").transform;
            canonical.SetParent(transform, false);
            var visual = new GameObject("VisualPresentation").transform;
            visual.SetParent(transform, false);

            var random = new System.Random(seed);
            foreach (Box value in manifest.canonical.boxes) {
                Vector3 center = ToVector3(value.center, $"box {value.id} center");
                if (value.seedJitterX > 0f)
                    center.x += ((float)random.NextDouble() * 2f - 1f) * value.seedJitterX;
                CreateBox(canonical, visual, value.id, value.role, center,
                    ToVector3(value.size, $"box {value.id} size"), ToColor(value.color));
            }

            foreach (Region value in manifest.canonical.regions)
                AddRegion(canonical, value.id, value.role,
                    ToVector3(value.position, $"region {value.id} position"));

            Debug.Log($"CRANE_REFERENCE_ENVIRONMENT_READY id={EnvironmentId} seed={seed} " +
                      $"boxes={manifest.canonical.boxes.Length} " +
                      $"regions={manifest.canonical.regions.Length} routes={manifest.routes.Length} " +
                      $"scenarios={manifest.scenarios.Length}");
        }

        public ScenarioRuntimeState ApplyScenario(string scenarioId,
            Action<ScenarioRuntimeState> stateChanged = null) {
            if (string.IsNullOrWhiteSpace(scenarioId))
                throw new ArgumentException("Warehouse scenario ID must be non-empty.",
                    nameof(scenarioId));
            Manifest manifest = LoadManifest();
            ValidateManifest(manifest);
            Scenario scenario = Array.Find(manifest.scenarios,
                value => string.Equals(value.id, scenarioId, StringComparison.Ordinal));
            if (scenario == null)
                throw new ArgumentException($"Unknown warehouse scenario '{scenarioId}'.",
                    nameof(scenarioId));

            Transform oldCanonical = transform.Find("ScenarioCanonicalGeometry");
            Transform oldVisual = transform.Find("ScenarioVisualPresentation");
            if (oldCanonical != null) DestroyGeneratedObject(oldCanonical.gameObject);
            if (oldVisual != null) DestroyGeneratedObject(oldVisual.gameObject);
            var canonical = new GameObject("ScenarioCanonicalGeometry").transform;
            canonical.SetParent(transform, false);
            var visual = new GameObject("ScenarioVisualPresentation").transform;
            visual.SetParent(transform, false);

            var runtime = new ScenarioRuntimeState {
                scenarioId = scenario.id,
                routeId = scenario.routeId,
                seed = scenario.seed,
                robot = scenario.robot,
                expectedChallenge = scenario.expectedChallenge,
                expectedBroadOutcome = scenario.expectedBroadOutcome,
                obstacles = new ScenarioObstacleRuntimeState[scenario.obstacles.Length]
            };
            for (int index = 0; index < scenario.obstacles.Length; index++) {
                ScenarioObstacle value = scenario.obstacles[index];
                (GameObject collision, GameObject presentation) = CreateScenarioBox(
                    canonical, visual, value);
                var obstacleState = new ScenarioObstacleRuntimeState {
                    semanticId = value.id,
                    active = value.activeInitially
                };
                runtime.obstacles[index] = obstacleState;
                var timingHost = new GameObject($"timing-{value.id}");
                // Keep the timer inside the scenario-owned canonical root so reapplying a
                // scenario destroys its pending callbacks together with its colliders.
                timingHost.transform.SetParent(canonical, false);
                var timing = timingHost.AddComponent<CraneTimedWarehouseObstacle>();
                timing.Configure(collision, presentation, value.activeInitially,
                    value.activationAfterSeconds, value.removalAfterSeconds,
                    (active, actualTime) => {
                        obstacleState.active = active;
                        if (active) obstacleState.actualActivationSimulationTime = actualTime;
                        else obstacleState.actualRemovalSimulationTime = actualTime;
                        stateChanged?.Invoke(runtime);
                    });
                obstacleState.scheduledActivationSimulationTime =
                    timing.ScheduledActivationSimulationTime;
                obstacleState.scheduledRemovalSimulationTime =
                    timing.ScheduledRemovalSimulationTime;
            }
            stateChanged?.Invoke(runtime);
            Debug.Log($"CRANE_WAREHOUSE_SCENARIO_READY environment={EnvironmentId} " +
                      $"scenario={runtime.scenarioId} route={runtime.routeId} " +
                      $"seed={runtime.seed} obstacles={runtime.obstacles.Length}");
            return runtime;
        }

        private Manifest LoadManifest() {
            TextAsset asset = manifestAsset != null ? manifestAsset :
                Resources.Load<TextAsset>(ManifestResource);
            if (asset == null)
                throw new MissingReferenceException(
                    $"Warehouse manifest resource '{ManifestResource}' is missing.");
            Manifest manifest = JsonUtility.FromJson<Manifest>(asset.text);
            if (manifest == null)
                throw new InvalidOperationException("Warehouse manifest could not be parsed.");
            return manifest;
        }

        private static void ValidateManifest(Manifest manifest) {
            if (manifest.schema != "crane-environment-scenario-catalog-v1")
                throw new InvalidOperationException($"Unsupported warehouse schema '{manifest.schema}'.");
            if (manifest.environmentId != EnvironmentId)
                throw new InvalidOperationException(
                    $"Warehouse environment ID '{manifest.environmentId}' does not match '{EnvironmentId}'.");
            if (manifest.generatorVersion != "2.2.0")
                throw new InvalidOperationException(
                    $"Unsupported warehouse generator version '{manifest.generatorVersion}'.");
            if (manifest.canonical == null || manifest.canonical.boxes == null ||
                manifest.canonical.regions == null || manifest.routes == null ||
                manifest.scenarios == null)
                throw new InvalidOperationException("Warehouse manifest is incomplete.");
            if (manifest.canonical.dimensionsMeters == null ||
                manifest.canonical.dimensionsMeters.Length != 2)
                throw new InvalidOperationException("Warehouse dimensions must contain width and length.");

            var ids = new HashSet<string>(StringComparer.Ordinal);
            foreach (Box value in manifest.canonical.boxes) {
                ValidateIdentity(value.id, value.role, ids);
                ValidateTriplet(value.center, $"box {value.id} center");
                ValidateTriplet(value.size, $"box {value.id} size");
                if (value.size[0] <= 0f || value.size[1] <= 0f || value.size[2] <= 0f)
                    throw new InvalidOperationException($"Box '{value.id}' has non-positive size.");
            }
            foreach (Region value in manifest.canonical.regions) {
                ValidateIdentity(value.id, value.role, ids);
                ValidateTriplet(value.position, $"region {value.id} position");
            }
            foreach (Route value in manifest.routes) {
                ValidateIdentity(value.id, "route", ids);
                ValidateTriplet(value.start, $"route {value.id} start");
                ValidateTriplet(value.goal, $"route {value.id} goal");
                if (value.approximateLengthMeters <= 0f || value.alternatives == null ||
                    value.relevantObstacles == null || string.IsNullOrWhiteSpace(value.expectedChallenge) ||
                    string.IsNullOrWhiteSpace(value.expectedBroadOutcome))
                    throw new InvalidOperationException($"Route '{value.id}' is incomplete.");
            }
            var routeIds = new HashSet<string>(StringComparer.Ordinal);
            foreach (Route value in manifest.routes) routeIds.Add(value.id);
            foreach (Scenario value in manifest.scenarios) {
                ValidateIdentity(value.id, "scenario", ids);
                if (!routeIds.Contains(value.routeId) || value.seed < 0 ||
                    string.IsNullOrWhiteSpace(value.robot) || value.obstacles == null ||
                    value.obstacles.Length == 0 ||
                    string.IsNullOrWhiteSpace(value.expectedChallenge) ||
                    string.IsNullOrWhiteSpace(value.expectedBroadOutcome))
                    throw new InvalidOperationException($"Scenario '{value.id}' is incomplete.");
                foreach (ScenarioObstacle obstacle in value.obstacles) {
                    ValidateIdentity(obstacle.id, obstacle.role, ids);
                    ValidateTriplet(obstacle.center,
                        $"scenario obstacle {obstacle.id} center");
                    ValidateTriplet(obstacle.size,
                        $"scenario obstacle {obstacle.id} size");
                    if (obstacle.size[0] <= 0f || obstacle.size[1] <= 0f ||
                        obstacle.size[2] <= 0f)
                        throw new InvalidOperationException(
                            $"Scenario obstacle '{obstacle.id}' has non-positive size.");
                    if (!obstacle.activeInitially && obstacle.activationAfterSeconds < 0f)
                        throw new InvalidOperationException(
                            $"Initially inactive obstacle '{obstacle.id}' has no activation.");
                    if (obstacle.activeInitially && obstacle.activationAfterSeconds >= 0f)
                        throw new InvalidOperationException(
                            $"Initially active obstacle '{obstacle.id}' also declares activation.");
                    if (obstacle.removalAfterSeconds >= 0f &&
                        obstacle.activationAfterSeconds >= 0f &&
                        obstacle.removalAfterSeconds <= obstacle.activationAfterSeconds)
                        throw new InvalidOperationException(
                            $"Obstacle '{obstacle.id}' removal does not follow activation.");
                }
            }
        }

        private static void ValidateIdentity(string id, string role, HashSet<string> ids) {
            if (string.IsNullOrWhiteSpace(id) || string.IsNullOrWhiteSpace(role))
                throw new InvalidOperationException("Warehouse semantic ID/role must be non-empty.");
            if (!ids.Add(id))
                throw new InvalidOperationException($"Duplicate warehouse semantic ID '{id}'.");
        }

        private static void ValidateTriplet(float[] values, string label) {
            if (values == null || values.Length != 3)
                throw new InvalidOperationException($"Warehouse {label} must contain three values.");
        }

        private void CreateBox(Transform canonical, Transform visual, string id, string role,
            Vector3 center, Vector3 size, Color color) {
            var collision = new GameObject(id);
            collision.layer = CraneCollisionLayers.Environment;
            collision.transform.SetParent(canonical, false);
            collision.transform.localPosition = center;
            var collider = collision.AddComponent<BoxCollider>();
            collider.size = size;
            collision.AddComponent<CraneSemanticIdentity>().Configure(id, role, EnvironmentId);
            if (!createVisualLayer) return;
            GameObject presentation = GameObject.CreatePrimitive(PrimitiveType.Cube);
            presentation.name = id + "-visual";
            presentation.transform.SetParent(visual, false);
            presentation.transform.localPosition = center;
            presentation.transform.localScale = size;
            Collider presentationCollider = presentation.GetComponent<Collider>();
            if (presentationCollider != null) DestroyGeneratedObject(presentationCollider);
            presentation.GetComponent<Renderer>().sharedMaterial = new Material(
                Shader.Find("HDRP/Lit") ?? Shader.Find("Standard")) {
                name = id + "-material",
                color = color
            };
        }

        private (GameObject collision, GameObject presentation) CreateScenarioBox(
            Transform canonical, Transform visual, ScenarioObstacle value) {
            Vector3 center = ToVector3(value.center,
                $"scenario obstacle {value.id} center");
            Vector3 size = ToVector3(value.size,
                $"scenario obstacle {value.id} size");
            var collision = new GameObject(value.id);
            collision.layer = CraneCollisionLayers.Environment;
            collision.transform.SetParent(canonical, false);
            collision.transform.localPosition = center;
            collision.AddComponent<BoxCollider>().size = size;
            collision.AddComponent<CraneSemanticIdentity>().Configure(
                value.id, value.role, EnvironmentId);
            GameObject presentation = null;
            if (createVisualLayer) {
                presentation = GameObject.CreatePrimitive(PrimitiveType.Cube);
                presentation.name = value.id + "-visual";
                presentation.transform.SetParent(visual, false);
                presentation.transform.localPosition = center;
                presentation.transform.localScale = size;
                Collider presentationCollider = presentation.GetComponent<Collider>();
                if (presentationCollider != null) DestroyGeneratedObject(presentationCollider);
                presentation.GetComponent<Renderer>().sharedMaterial = new Material(
                    Shader.Find("HDRP/Lit") ?? Shader.Find("Standard")) {
                    name = value.id + "-material",
                    color = ToColor(value.color)
                };
            }
            return (collision, presentation);
        }

        private static void AddRegion(Transform canonical, string id, string role,
            Vector3 center) {
            var region = new GameObject(id);
            region.transform.SetParent(canonical, false);
            region.transform.localPosition = center;
            region.AddComponent<CraneSemanticIdentity>().Configure(id, role, EnvironmentId);
        }

        private static Vector3 ToVector3(float[] values, string label) {
            ValidateTriplet(values, label);
            return new Vector3(values[0], values[1], values[2]);
        }

        private static Color ToColor(float[] values) {
            if (values == null || values.Length != 3) return Color.gray;
            return new Color(values[0], values[1], values[2]);
        }

        private static void DestroyGeneratedObject(UnityEngine.Object value) {
            if (Application.isPlaying) Destroy(value);
            else DestroyImmediate(value);
        }
    }
}
