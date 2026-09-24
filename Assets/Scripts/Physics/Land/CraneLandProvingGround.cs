using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using Sim.Physics.Contacts;
using Sim.Utils.ReferenceEnvironments;
using UnityEngine;

namespace Sim.Physics.Land {
    /// <summary>
    /// Builds one deterministic land-navigation topology from a versioned manifest. The caller
    /// selects only a layout and seed; this module owns validation, geometry, scheduling, hashes,
    /// semantics, and evaluator-only truth.
    /// </summary>
    internal static class CraneLandProvingGround {
        internal const string DefaultCatalog = "v1";
        private const string ManifestResourcePrefix =
            "ReferenceEnvironments/crane_land_proving_ground_";

        [Serializable]
        private sealed class Manifest {
            public string schema;
            public string environmentId;
            public string generatorVersion;
            public float[] dimensionsMeters;
            public Box[] sharedBoxes;
            public Layout[] layouts;
        }

        [Serializable]
        private sealed class Layout {
            public string id;
            public int seed;
            public string robot;
            public float[] start;
            public float[] goal;
            public float approximateLengthMeters;
            public string[] alternatives;
            public string[] relevantObstacles;
            public string expectedChallenge;
            public string expectedBroadOutcome;
            public string studySplit;
            public string diagnosticMechanism;
            public int generatorSeed;
            public Box[] obstacles;
        }

        [Serializable]
        private sealed class Box {
            public string id;
            public string role;
            public float[] center;
            public float[] size;
            public float[] color;
            public bool activeInitially = true;
            public float activationAfterSeconds = -1f;
            public float removalAfterSeconds = -1f;
        }

        [Serializable]
        internal sealed class EvaluatorTruth {
            public string schema = "crane-land-proving-ground-truth-v1";
            public string environmentId;
            public string generatorVersion;
            public string manifestSha256;
            public string configurationSha256;
            public string layoutId;
            public int seed;
            public string robot;
            public Vector3 startPosition;
            public Vector3 goalPosition;
            public float approximateLengthMeters;
            public string[] alternatives = Array.Empty<string>();
            public string[] relevantObstacles = Array.Empty<string>();
            public string expectedChallenge;
            public string expectedBroadOutcome;
            public string studySplit;
            public string diagnosticMechanism;
            public int generatorSeed;
            public string[] obstacleSemanticIds = Array.Empty<string>();
            public bool[] obstacleActive = Array.Empty<bool>();
            public double[] obstacleScheduledActivationSimulationTime = Array.Empty<double>();
            public double[] obstacleActualActivationSimulationTime = Array.Empty<double>();
            public double[] obstacleScheduledRemovalSimulationTime = Array.Empty<double>();
            public double[] obstacleActualRemovalSimulationTime = Array.Empty<double>();
            public float mobilityHoldAfterSeconds = -1f;
            public float mobilityReleaseAfterSeconds = -1f;
            public double mobilityHoldScheduledSimulationTime = -1d;
            public double mobilityHoldActualSimulationTime = -1d;
            public double mobilityReleaseScheduledSimulationTime = -1d;
            public double mobilityReleaseActualSimulationTime = -1d;
            public bool mobilityHeld;
            public bool mobilityReleased;
        }

        internal static EvaluatorTruth Build(string catalogId, string layoutId, int requestedSeed,
            Rigidbody body, DifferentialDriveDynamics robot, string truthPath,
            float mobilityHoldAfterSeconds = -1f,
            float mobilityReleaseAfterSeconds = -1f) {
            if (string.IsNullOrWhiteSpace(catalogId))
                throw new ArgumentException("Proving-ground catalog ID must be non-empty.",
                    nameof(catalogId));
            if (string.IsNullOrWhiteSpace(layoutId))
                throw new ArgumentException("Proving-ground layout ID must be non-empty.",
                    nameof(layoutId));
            if (body == null) throw new ArgumentNullException(nameof(body));
            if (robot == null) throw new ArgumentNullException(nameof(robot));
            if (mobilityHoldAfterSeconds < 0f && mobilityReleaseAfterSeconds >= 0f)
                throw new ArgumentException(
                    "Proving-ground mobility release requires a configured hold boundary.");
            if (mobilityReleaseAfterSeconds >= 0f &&
                mobilityReleaseAfterSeconds <= mobilityHoldAfterSeconds)
                throw new ArgumentException(
                    "Proving-ground mobility release must occur after the hold begins.");

            string resource = ResolveManifestResource(catalogId);
            TextAsset asset = Resources.Load<TextAsset>(resource);
            if (asset == null)
                throw new MissingReferenceException(
                    $"Proving-ground manifest resource '{resource}' is missing.");
            Manifest manifest = JsonUtility.FromJson<Manifest>(asset.text);
            ValidateManifest(manifest, catalogId);
            Layout layout = Array.Find(manifest.layouts,
                value => string.Equals(value.id, layoutId, StringComparison.Ordinal));
            if (layout == null)
                throw new ArgumentException($"Unknown proving-ground layout '{layoutId}'.",
                    nameof(layoutId));
            int seed = requestedSeed >= 0 ? requestedSeed : layout.seed;
            string manifestHash = Sha256(asset.bytes);
            string configurationIdentity = $"{manifestHash}\n{layout.id}\n{seed}";
            // Preserve the established no-intervention identity. Only an explicitly configured
            // proving-ground execution intervention extends the authenticated configuration.
            if (mobilityHoldAfterSeconds >= 0f)
                configurationIdentity += string.Format(CultureInfo.InvariantCulture,
                    "\nmobility-hold={0:R}\nmobility-release={1:R}",
                    mobilityHoldAfterSeconds, mobilityReleaseAfterSeconds);
            string configurationHash = Sha256(Encoding.UTF8.GetBytes(configurationIdentity));

            GameObject existing = GameObject.Find("Reference Environment");
            if (existing != null) UnityEngine.Object.Destroy(existing);
            // The generic reference validator discovers this active root by name. The scene's
            // authored warehouse root is inactive while the proving ground is selected.
            var root = new GameObject("Reference Environment");
            var canonical = new GameObject("CanonicalGeometry").transform;
            canonical.SetParent(root.transform, false);
            var visual = new GameObject("VisualPresentation").transform;
            visual.SetParent(root.transform, false);

            foreach (Box value in manifest.sharedBoxes)
                CreateBox(canonical, visual, value, true, manifest.environmentId);

            Vector3 start = ToVector3(layout.start, $"layout {layout.id} start");
            Vector3 goal = ToVector3(layout.goal, $"layout {layout.id} goal");
            AddSemanticPoint(canonical, "proving-start", "route-start", start,
                manifest.environmentId);
            AddSemanticPoint(canonical, "proving-goal", "route-goal", goal,
                manifest.environmentId);

            body.position = start;
            body.rotation = Quaternion.identity;
            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
            robot.SetCommand(0f, 0f);

            int count = layout.obstacles.Length;
            var truth = new EvaluatorTruth {
                environmentId = manifest.environmentId,
                generatorVersion = manifest.generatorVersion,
                manifestSha256 = manifestHash,
                configurationSha256 = configurationHash,
                layoutId = layout.id,
                seed = seed,
                robot = layout.robot,
                startPosition = start,
                goalPosition = goal,
                approximateLengthMeters = layout.approximateLengthMeters,
                alternatives = layout.alternatives,
                relevantObstacles = layout.relevantObstacles,
                expectedChallenge = layout.expectedChallenge,
                expectedBroadOutcome = layout.expectedBroadOutcome,
                studySplit = layout.studySplit,
                diagnosticMechanism = layout.diagnosticMechanism,
                generatorSeed = layout.generatorSeed,
                obstacleSemanticIds = new string[count],
                obstacleActive = new bool[count],
                obstacleScheduledActivationSimulationTime = Filled(count, -1d),
                obstacleActualActivationSimulationTime = Filled(count, -1d),
                obstacleScheduledRemovalSimulationTime = Filled(count, -1d),
                obstacleActualRemovalSimulationTime = Filled(count, -1d),
                mobilityHoldAfterSeconds = mobilityHoldAfterSeconds,
                mobilityReleaseAfterSeconds = mobilityReleaseAfterSeconds
            };

            for (int index = 0; index < count; index++) {
                Box value = layout.obstacles[index];
                (GameObject collision, GameObject presentation) =
                    CreateBox(canonical, visual, value, value.activeInitially,
                        manifest.environmentId);
                int capturedIndex = index;
                truth.obstacleSemanticIds[index] = value.id;
                truth.obstacleActive[index] = value.activeInitially;
                if (value.activationAfterSeconds < 0f && value.removalAfterSeconds < 0f)
                    continue;
                var timingHost = new GameObject($"timing-{value.id}");
                timingHost.transform.SetParent(canonical, false);
                var timing = timingHost.AddComponent<CraneTimedWarehouseObstacle>();
                timing.Configure(collision, presentation, value.activeInitially,
                    value.activationAfterSeconds, value.removalAfterSeconds,
                    (active, actualTime) => {
                        truth.obstacleActive[capturedIndex] = active;
                        if (active)
                            truth.obstacleActualActivationSimulationTime[capturedIndex] =
                                actualTime;
                        else
                            truth.obstacleActualRemovalSimulationTime[capturedIndex] = actualTime;
                        WriteTruth(truthPath, truth);
                    });
                truth.obstacleScheduledActivationSimulationTime[index] =
                    timing.ScheduledActivationSimulationTime;
                truth.obstacleScheduledRemovalSimulationTime[index] =
                    timing.ScheduledRemovalSimulationTime;
            }
            if (mobilityHoldAfterSeconds >= 0f) {
                var mobilityHost = new GameObject("CRANE Proving-Ground Timed Mobility Hold");
                mobilityHost.transform.SetParent(canonical, false);
                var mobility = mobilityHost.AddComponent<CraneTimedMobilityHold>();
                mobility.Configure(body, mobilityHoldAfterSeconds, mobilityReleaseAfterSeconds,
                    actualTime => {
                        truth.mobilityHeld = true;
                        truth.mobilityHoldActualSimulationTime = actualTime;
                        WriteTruth(truthPath, truth);
                    }, actualTime => {
                        truth.mobilityReleased = true;
                        truth.mobilityReleaseActualSimulationTime = actualTime;
                        WriteTruth(truthPath, truth);
                    });
                truth.mobilityHoldScheduledSimulationTime =
                    mobility.ScheduledHoldSimulationTime;
                truth.mobilityReleaseScheduledSimulationTime =
                    mobility.ScheduledReleaseSimulationTime;
            }
            WriteTruth(truthPath, truth);
            ConfigureInspection(body.transform, manifest.environmentId, layout);
            UnityEngine.Physics.SyncTransforms();
            Debug.Log($"CRANE_LAND_PROVING_GROUND_READY environment={manifest.environmentId} " +
                      $"layout={layout.id} seed={seed} obstacles={count} " +
                      $"mobilityHoldAfter={mobilityHoldAfterSeconds:R} " +
                      $"mobilityReleaseAfter={mobilityReleaseAfterSeconds:R} " +
                      $"configurationSha256={configurationHash}");
            return truth;
        }

        private static string ResolveManifestResource(string catalogId) {
            if (catalogId != "v1" && catalogId != "v2" && catalogId != "v3" &&
                catalogId != "v4")
                throw new ArgumentException(
                    $"Unsupported proving-ground catalog '{catalogId}'.", nameof(catalogId));
            return ManifestResourcePrefix + catalogId;
        }

        private static void ConfigureInspection(Transform robot, string environmentId,
            Layout layout) {
            GameObject spectator = GameObject.Find("Spectator Camera");
            CraneReferenceInspectionController inspection =
                spectator?.GetComponent<CraneReferenceInspectionController>();
            Camera camera = spectator?.GetComponent<Camera>();
            if (inspection == null || camera == null) {
                Debug.LogWarning(
                    "CRANE proving-ground inspection unavailable: spectator camera/controller missing.");
                return;
            }
            inspection.Configure(camera, robot, environmentId, layout.id,
                layout.relevantObstacles, new Vector3(0f, 0f, 10f));
        }

        private static void ValidateManifest(Manifest manifest, string catalogId) {
            if (manifest == null)
                throw new InvalidOperationException("Proving-ground manifest could not be parsed.");
            if (manifest.schema != "crane-land-proving-ground-catalog-v1" ||
                manifest.environmentId != $"crane-land-proving-ground-{catalogId}" ||
                string.IsNullOrWhiteSpace(manifest.generatorVersion))
                throw new InvalidOperationException("Unsupported proving-ground manifest identity.");
            ValidateTripletPair(manifest.dimensionsMeters, "dimensions", 2);
            if (manifest.sharedBoxes == null || manifest.layouts == null ||
                manifest.layouts.Length == 0)
                throw new InvalidOperationException("Proving-ground manifest is incomplete.");
            var layoutIds = new HashSet<string>(StringComparer.Ordinal);
            foreach (Layout layout in manifest.layouts) {
                if (string.IsNullOrWhiteSpace(layout.id) || !layoutIds.Add(layout.id) ||
                    layout.seed < 0 || string.IsNullOrWhiteSpace(layout.robot) ||
                    layout.approximateLengthMeters <= 0f || layout.alternatives == null ||
                    layout.relevantObstacles == null || layout.obstacles == null ||
                    layout.obstacles.Length == 0 ||
                    string.IsNullOrWhiteSpace(layout.expectedChallenge) ||
                    string.IsNullOrWhiteSpace(layout.expectedBroadOutcome))
                    throw new InvalidOperationException(
                        $"Proving-ground layout '{layout.id}' is incomplete.");
                ValidateTripletPair(layout.start, $"layout {layout.id} start", 3);
                ValidateTripletPair(layout.goal, $"layout {layout.id} goal", 3);
                var semanticIds = new HashSet<string>(StringComparer.Ordinal) {
                    "proving-start", "proving-goal"
                };
                foreach (Box value in manifest.sharedBoxes)
                    ValidateBox(value, semanticIds, "shared");
                foreach (Box value in layout.obstacles)
                    ValidateBox(value, semanticIds, layout.id);
                foreach (string relevant in layout.relevantObstacles)
                    if (!semanticIds.Contains(relevant))
                        throw new InvalidOperationException(
                            $"Layout '{layout.id}' references unknown obstacle '{relevant}'.");
            }
        }

        private static void ValidateBox(Box value, HashSet<string> ids, string scope) {
            if (value == null || string.IsNullOrWhiteSpace(value.id) ||
                string.IsNullOrWhiteSpace(value.role) || !ids.Add(value.id))
                throw new InvalidOperationException(
                    $"Invalid or duplicate proving-ground semantic ID in '{scope}'.");
            ValidateTripletPair(value.center, $"box {value.id} center", 3);
            ValidateTripletPair(value.size, $"box {value.id} size", 3);
            ValidateTripletPair(value.color, $"box {value.id} color", 3);
            if (value.size[0] <= 0f || value.size[1] <= 0f || value.size[2] <= 0f)
                throw new InvalidOperationException($"Box '{value.id}' has non-positive size.");
            if (!value.activeInitially && value.activationAfterSeconds < 0f)
                throw new InvalidOperationException(
                    $"Inactive box '{value.id}' has no activation time.");
            if (value.activeInitially && value.activationAfterSeconds >= 0f)
                throw new InvalidOperationException(
                    $"Active box '{value.id}' also declares an activation time.");
            if (value.removalAfterSeconds >= 0f &&
                value.activationAfterSeconds >= 0f &&
                value.removalAfterSeconds <= value.activationAfterSeconds)
                throw new InvalidOperationException(
                    $"Box '{value.id}' removal must follow activation.");
        }

        private static (GameObject collision, GameObject presentation) CreateBox(
            Transform canonical, Transform visual, Box value, bool active,
            string environmentId) {
            Vector3 center = ToVector3(value.center, $"box {value.id} center");
            Vector3 size = ToVector3(value.size, $"box {value.id} size");
            var collision = new GameObject(value.id);
            collision.layer = CraneCollisionLayers.Environment;
            collision.transform.SetParent(canonical, false);
            collision.transform.localPosition = center;
            collision.AddComponent<BoxCollider>().size = size;
            collision.AddComponent<CraneSemanticIdentity>().Configure(
                value.id, value.role, environmentId);

            GameObject presentation = GameObject.CreatePrimitive(PrimitiveType.Cube);
            presentation.name = value.id + "-visual";
            presentation.transform.SetParent(visual, false);
            presentation.transform.localPosition = center;
            presentation.transform.localScale = size;
            Collider visualCollider = presentation.GetComponent<Collider>();
            if (visualCollider != null) UnityEngine.Object.Destroy(visualCollider);
            presentation.GetComponent<Renderer>().sharedMaterial = new Material(
                Shader.Find("HDRP/Lit") ?? Shader.Find("Standard")) {
                name = value.id + "-material",
                color = ToColor(value.color)
            };
            collision.SetActive(active);
            presentation.SetActive(active);
            return (collision, presentation);
        }

        private static void AddSemanticPoint(Transform canonical, string id, string role,
            Vector3 position, string environmentId) {
            var point = new GameObject(id);
            point.transform.SetParent(canonical, false);
            point.transform.localPosition = position;
            point.AddComponent<CraneSemanticIdentity>().Configure(id, role, environmentId);
        }

        private static void WriteTruth(string path, EvaluatorTruth truth) {
            if (string.IsNullOrWhiteSpace(path)) return;
            string fullPath = Path.GetFullPath(path);
            string directory = Path.GetDirectoryName(fullPath);
            if (!string.IsNullOrEmpty(directory)) Directory.CreateDirectory(directory);
            File.WriteAllText(fullPath, JsonUtility.ToJson(truth, true));
        }

        private static double[] Filled(int count, double value) {
            var result = new double[count];
            for (int index = 0; index < count; index++) result[index] = value;
            return result;
        }

        private static Vector3 ToVector3(float[] values, string label) {
            ValidateTripletPair(values, label, 3);
            return new Vector3(values[0], values[1], values[2]);
        }

        private static Color ToColor(float[] values) {
            ValidateTripletPair(values, "color", 3);
            return new Color(values[0], values[1], values[2], 1f);
        }

        private static void ValidateTripletPair(float[] values, string label, int expected) {
            if (values == null || values.Length != expected)
                throw new InvalidOperationException(
                    $"Proving-ground {label} must contain {expected} values.");
        }

        private static string Sha256(byte[] bytes) {
            using SHA256 algorithm = SHA256.Create();
            byte[] hash = algorithm.ComputeHash(bytes);
            var text = new StringBuilder(hash.Length * 2);
            foreach (byte value in hash) text.Append(value.ToString("x2"));
            return text.ToString();
        }
    }
}
