using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Utils.ReferenceEnvironments {
    [Serializable]
    internal sealed class ReferenceSceneValidationResult {
        public string schema = "crane-reference-scene-validation-v1";
        public string scene;
        public string environmentId;
        public string manifestSha256;
        public string sourceVersion;
        public int sourceObjectCount;
        public int canonicalColliderCount;
        public int meshColliderCount;
        public int visualRendererCount;
        public int lidarCount;
        public int semanticIdentityCount;
        public int duplicateSemanticIdCount;
        public float boundsMinX;
        public float boundsMinY;
        public float boundsMinZ;
        public float boundsMaxX;
        public float boundsMaxY;
        public float boundsMaxZ;
        public string raycastSemanticId;
        public float raycastDistance;
        public float raycastSurfaceHeight;
        public float dropFinalHeight;
        public float dropFinalSpeed;
        public bool dropContactObserved;
        public bool layersValid;
        public bool boundsValid;
        public bool raycastValid;
        public bool collisionValid;
        public bool semanticSensorRayApplicable;
        public bool semanticSensorRayValid;
        public string semanticSensorRayId;
        public float semanticSensorRayDistance;
        public bool semanticHighlightApplicable;
        public int semanticHighlightRendererCount;
        public bool semanticHighlightValid;
        public bool ackermannDynamicsApplicable;
        public int ackermannGroundedWheels;
        public float ackermannDriveDisplacement;
        public float ackermannTurnDegrees;
        public float ackermannUpAlignment;
        public float ackermannBodyHeight;
        public float ackermannVerticalSpeed;
        public bool ackermannDynamicsValid;
        public bool differentialDynamicsApplicable;
        public float differentialDriveDisplacement;
        public float differentialTurnDegrees;
        public float differentialUpAlignment;
        public float differentialBodyHeight;
        public float differentialVerticalSpeed;
        public bool differentialDynamicsValid;
        public string structuralStatus;
        public string physicsStatus;
        public string sensorStatus;
        public string navigationStatus = "NOT_RUN";
        public string interactiveStatus = "NOT_RUN";
        public string headlessStatus;
        public string explanationStatus;
        public string verdict;
        public bool valid;
    }

    internal sealed class CraneReferenceSceneValidationRunner : MonoBehaviour {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            if (!Environment.GetCommandLineArgs().Contains("--crane-reference-validation")) return;
            var host = new GameObject("CRANE Reference Scene Validation Runner");
            DontDestroyOnLoad(host);
            host.AddComponent<CraneReferenceSceneValidationRunner>();
        }

        private IEnumerator Start() {
            string[] args = Environment.GetCommandLineArgs();
            string requestedScene = ReadArgument(args, "--crane-reference-scene");
            string outputPath = ReadArgument(args, "--crane-output") ??
                Path.Combine(Application.persistentDataPath, "crane-reference-validation.json");
            if (string.IsNullOrWhiteSpace(requestedScene))
                throw new ArgumentException("--crane-reference-scene is required");
            if (SceneManager.GetActiveScene().name != requestedScene) {
                AsyncOperation load = SceneManager.LoadSceneAsync(requestedScene, LoadSceneMode.Single);
                while (load != null && !load.isDone) yield return null;
            }
            yield return new WaitForFixedUpdate();
            yield return Validate(outputPath);
        }

        private IEnumerator Validate(string outputPath) {
            string[] args = Environment.GetCommandLineArgs();
            CraneImportedReferenceEnvironment imported =
                FindAnyObjectByType<CraneImportedReferenceEnvironment>();
            Transform environment = imported == null
                ? GameObject.Find("Reference Environment")?.transform
                : imported.transform;
            if (environment == null)
                throw new MissingReferenceException("Reference environment root missing.");
            string environmentId = imported == null
                ? ReadArgument(args, "--crane-reference-environment-id")
                : imported.EnvironmentId;
            string manifestSha256 = imported == null
                ? ReadArgument(args, "--crane-reference-manifest-sha256")
                : imported.ManifestSha256;
            string sourceVersion = imported == null
                ? ReadArgument(args, "--crane-reference-source-version")
                : imported.SourceVersion;
            int sourceObjectCount = imported == null
                ? ReadIntArgument(args, "--crane-reference-source-object-count")
                : imported.SourceObjectCount;
            if (string.IsNullOrWhiteSpace(environmentId) ||
                string.IsNullOrWhiteSpace(manifestSha256))
                throw new ArgumentException("Native reference environments require identity and manifest hash.");
            Transform canonical = environment.Find("CanonicalGeometry");
            Transform visual = environment.Find("VisualPresentation");
            Collider[] colliders = canonical == null ? Array.Empty<Collider>() :
                canonical.GetComponentsInChildren<Collider>(true);
            Renderer[] canonicalRenderers = canonical == null ? Array.Empty<Renderer>() :
                canonical.GetComponentsInChildren<Renderer>(true);
            Renderer[] visualRenderers = visual == null ? Array.Empty<Renderer>() :
                visual.GetComponentsInChildren<Renderer>(true);
            Collider[] visualColliders = visual == null ? Array.Empty<Collider>() :
                visual.GetComponentsInChildren<Collider>(true);
            CraneSemanticIdentity[] identities = canonical == null
                ? Array.Empty<CraneSemanticIdentity>()
                : canonical.GetComponentsInChildren<CraneSemanticIdentity>(true);
            int duplicateSemanticIds = identities.GroupBy(value => value.SemanticId,
                StringComparer.Ordinal).Count(group => string.IsNullOrWhiteSpace(group.Key) ||
                group.Count() > 1);
            var result = new ReferenceSceneValidationResult {
                scene = SceneManager.GetActiveScene().name,
                environmentId = environmentId,
                manifestSha256 = manifestSha256,
                sourceVersion = sourceVersion,
                sourceObjectCount = sourceObjectCount,
                canonicalColliderCount = colliders.Length,
                meshColliderCount = colliders.Count(value => value is MeshCollider),
                visualRendererCount = visualRenderers.Length,
                semanticIdentityCount = identities.Length,
                duplicateSemanticIdCount = duplicateSemanticIds,
                lidarCount = FindObjectsByType<MonoBehaviour>(FindObjectsInactive.Include,
                    FindObjectsSortMode.None).Count(value =>
                    value.GetType().FullName == "Sim.Sensors.Lidar.Lidar2D"),
                layersValid = colliders.Length > 0 && visualRenderers.Length > 0 &&
                    canonicalRenderers.Length == 0 && visualColliders.Length == 0 &&
                    identities.Length > 0 && duplicateSemanticIds == 0 &&
                    colliders.All(value => value.gameObject.layer ==
                        LayerMask.NameToLayer("Environment"))
            };

            if (colliders.Length > 0) {
                Bounds bounds = colliders[0].bounds;
                foreach (Collider collider in colliders.Skip(1)) bounds.Encapsulate(collider.bounds);
                result.boundsMinX = bounds.min.x;
                result.boundsMinY = bounds.min.y;
                result.boundsMinZ = bounds.min.z;
                result.boundsMaxX = bounds.max.x;
                result.boundsMaxY = bounds.max.y;
                result.boundsMaxZ = bounds.max.z;
                result.boundsValid = bounds.size.x > 1f && bounds.size.y > 0.1f && bounds.size.z > 1f;
                result.raycastValid = FindSurface(bounds, out RaycastHit hit);
                if (result.raycastValid) {
                    result.raycastDistance = hit.distance;
                    result.raycastSurfaceHeight = hit.point.y;
                    CraneSemanticIdentity identity = hit.collider.GetComponentInParent<CraneSemanticIdentity>();
                    result.raycastSemanticId = identity == null ? string.Empty : identity.SemanticId;
                    result.raycastValid = !string.IsNullOrEmpty(result.raycastSemanticId);
                    if (result.raycastValid) {
                        var highlighterHost = new GameObject("Reference Evidence Highlighter");
                        var highlighter = highlighterHost.AddComponent<CraneSemanticEvidenceHighlighter>();
                        int colliderCountBefore = FindObjectsByType<Collider>(
                            FindObjectsInactive.Include, FindObjectsSortMode.None).Length;
                        result.semanticHighlightApplicable = true;
                        result.semanticHighlightRendererCount = highlighter.Highlight(
                            new[] { result.raycastSemanticId });
                        int colliderCountAfter = FindObjectsByType<Collider>(
                            FindObjectsInactive.Include, FindObjectsSortMode.None).Length;
                        result.semanticHighlightValid = result.semanticHighlightRendererCount > 0 &&
                            colliderCountBefore == colliderCountAfter;
                        highlighter.Clear();
                        Destroy(highlighterHost);
                        var probe = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                        probe.name = "Reference Collision Probe";
                        probe.layer = LayerMask.NameToLayer("Vehicle");
                        probe.transform.position = hit.point + Vector3.up * 2f;
                        probe.transform.localScale = Vector3.one * 0.4f;
                        var body = probe.AddComponent<Rigidbody>();
                        var contact = probe.AddComponent<CraneReferenceCollisionProbe>();
                        body.mass = 1f;
                        body.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
                        for (int i = 0; i < 150; i++) yield return new WaitForFixedUpdate();
                        result.dropFinalHeight = body.position.y;
                        result.dropFinalSpeed = body.linearVelocity.magnitude;
                        result.dropContactObserved = contact.ContactObserved;
                        result.collisionValid = contact.ContactObserved &&
                            body.position.y > bounds.min.y - 2f;
                    }
                }
            }
            MonoBehaviour lidar = FindObjectsByType<MonoBehaviour>(FindObjectsInactive.Exclude,
                FindObjectsSortMode.None).FirstOrDefault(value =>
                value.GetType().FullName == "Sim.Sensors.Lidar.Lidar2D");
            result.semanticSensorRayApplicable = lidar != null;
            if (lidar != null)
                result.semanticSensorRayValid = FindSemanticSensorHit(lidar.transform,
                    out result.semanticSensorRayId, out result.semanticSensorRayDistance);

            MonoBehaviour ackermann = FindObjectsByType<MonoBehaviour>(FindObjectsInactive.Exclude,
                FindObjectsSortMode.None).FirstOrDefault(value =>
                value.GetType().FullName == "Sim.Physics.Land.AckermannRoverDynamics");
            result.ackermannDynamicsApplicable = ackermann != null;
            if (ackermann != null) {
                Rigidbody robotBody = ackermann.GetComponent<Rigidbody>();
                System.Reflection.MethodInfo setCommand = ackermann.GetType().GetMethod("SetCommand") ??
                    throw new MissingMethodException(ackermann.GetType().FullName, "SetCommand");
                System.Reflection.PropertyInfo groundedWheels = ackermann.GetType().GetProperty(
                    "GroundedWheelCount") ?? throw new MissingMemberException(
                    ackermann.GetType().FullName, "GroundedWheelCount");
                setCommand.Invoke(ackermann, new object[] { 0f, 0f, 1f });
                for (int i = 0; i < 25; i++) {
                    yield return new WaitForFixedUpdate();
                    result.ackermannGroundedWheels = Mathf.Max(result.ackermannGroundedWheels,
                        (int)groundedWheels.GetValue(ackermann));
                }
                Vector3 driveStart = robotBody.position;
                setCommand.Invoke(ackermann, new object[] { 0.30f, 0f, 0f });
                for (int i = 0; i < 50; i++) {
                    yield return new WaitForFixedUpdate();
                    result.ackermannGroundedWheels = Mathf.Max(result.ackermannGroundedWheels,
                        (int)groundedWheels.GetValue(ackermann));
                }
                result.ackermannDriveDisplacement = Vector2.Distance(
                    new Vector2(driveStart.x, driveStart.z),
                    new Vector2(robotBody.position.x, robotBody.position.z));
                Quaternion turnStart = robotBody.rotation;
                setCommand.Invoke(ackermann, new object[] { 0.20f, 0.5f, 0f });
                for (int i = 0; i < 35; i++) {
                    yield return new WaitForFixedUpdate();
                    result.ackermannGroundedWheels = Mathf.Max(result.ackermannGroundedWheels,
                        (int)groundedWheels.GetValue(ackermann));
                }
                setCommand.Invoke(ackermann, new object[] { 0f, 0f, 1f });
                result.ackermannTurnDegrees = Quaternion.Angle(turnStart, robotBody.rotation);
                result.ackermannUpAlignment = Vector3.Dot(robotBody.rotation * Vector3.up,
                    Vector3.up);
                result.ackermannBodyHeight = robotBody.position.y;
                result.ackermannVerticalSpeed = robotBody.linearVelocity.y;
                result.ackermannDynamicsValid = result.ackermannDriveDisplacement > 0.02f &&
                    result.ackermannTurnDegrees > 0.25f && result.ackermannGroundedWheels >= 3 &&
                    result.ackermannUpAlignment > 0.95f;
            }

            MonoBehaviour differential = FindObjectsByType<MonoBehaviour>(FindObjectsInactive.Exclude,
                FindObjectsSortMode.None).FirstOrDefault(value =>
                value.GetType().FullName == "Sim.Physics.Land.DifferentialDriveDynamics");
            result.differentialDynamicsApplicable = differential != null;
            if (differential != null) {
                Rigidbody robotBody = differential.GetComponent<Rigidbody>();
                System.Reflection.MethodInfo setCommand = differential.GetType().GetMethod("SetCommand") ??
                    throw new MissingMethodException(differential.GetType().FullName, "SetCommand");
                setCommand.Invoke(differential, new object[] { 0f, 0f });
                for (int i = 0; i < 25; i++) yield return new WaitForFixedUpdate();
                Vector3 driveStart = robotBody.position;
                setCommand.Invoke(differential, new object[] { 0.15f, 0f });
                for (int i = 0; i < 50; i++) yield return new WaitForFixedUpdate();
                result.differentialDriveDisplacement = Vector2.Distance(
                    new Vector2(driveStart.x, driveStart.z),
                    new Vector2(robotBody.position.x, robotBody.position.z));
                Quaternion turnStart = robotBody.rotation;
                setCommand.Invoke(differential, new object[] { 0f, 0.5f });
                for (int i = 0; i < 35; i++) yield return new WaitForFixedUpdate();
                setCommand.Invoke(differential, new object[] { 0f, 0f });
                result.differentialTurnDegrees = Quaternion.Angle(turnStart, robotBody.rotation);
                result.differentialUpAlignment = Vector3.Dot(robotBody.rotation * Vector3.up,
                    Vector3.up);
                result.differentialBodyHeight = robotBody.position.y;
                result.differentialVerticalSpeed = robotBody.linearVelocity.y;
                result.differentialDynamicsValid = result.differentialDriveDisplacement > 0.05f &&
                    result.differentialTurnDegrees > 5f && result.differentialUpAlignment > 0.95f &&
                    robotBody.position.y > -0.2f;
            }

            result.valid = result.layersValid && result.boundsValid && result.raycastValid &&
                result.collisionValid &&
                (!result.semanticHighlightApplicable || result.semanticHighlightValid) &&
                (!result.semanticSensorRayApplicable || result.semanticSensorRayValid) &&
                (!result.ackermannDynamicsApplicable || result.ackermannDynamicsValid) &&
                (!result.differentialDynamicsApplicable || result.differentialDynamicsValid);
            result.structuralStatus = result.layersValid ? "STRUCTURAL_PASS" : "PARTIAL";
            result.physicsStatus = result.boundsValid && result.raycastValid && result.collisionValid &&
                (!result.ackermannDynamicsApplicable || result.ackermannDynamicsValid) &&
                (!result.differentialDynamicsApplicable || result.differentialDynamicsValid)
                ? "PHYSICS_PASS" : "PARTIAL";
            result.sensorStatus = !result.semanticSensorRayApplicable || result.semanticSensorRayValid
                ? "SENSOR_PASS" : "PARTIAL";
            result.headlessStatus = result.valid ? "HEADLESS_PASS" : "PARTIAL";
            result.explanationStatus = identities.Length > 0 && duplicateSemanticIds == 0 &&
                (!result.semanticHighlightApplicable || result.semanticHighlightValid)
                ? "EXPLANATION_READY" : "PARTIAL";
            result.verdict = result.valid ? "PARTIAL" : "BLOCKED";
            string directory = Path.GetDirectoryName(outputPath);
            if (!string.IsNullOrEmpty(directory)) Directory.CreateDirectory(directory);
            File.WriteAllText(outputPath, JsonUtility.ToJson(result, true));
            Debug.Log($"CRANE_REFERENCE_VALIDATION_COMPLETE {outputPath} valid={result.valid}");
            Application.Quit(result.valid ? 0 : 2);
        }

        private static bool FindSurface(Bounds bounds, out RaycastHit hit) {
            int mask = 1 << LayerMask.NameToLayer("Environment");
            float height = bounds.max.y + 20f;
            for (int z = 1; z <= 7; z++) {
                for (int x = 1; x <= 7; x++) {
                    var origin = new Vector3(
                        Mathf.Lerp(bounds.min.x, bounds.max.x, x / 8f), height,
                        Mathf.Lerp(bounds.min.z, bounds.max.z, z / 8f));
                    if (UnityEngine.Physics.Raycast(origin, Vector3.down, out hit,
                            bounds.size.y + 40f, mask)) return true;
                }
            }
            hit = default;
            return false;
        }

        private static bool FindSemanticSensorHit(Transform sensor, out string semanticId,
            out float distance) {
            int mask = 1 << LayerMask.NameToLayer("Environment");
            for (int angle = 0; angle < 360; angle += 5) {
                Vector3 direction = Quaternion.Euler(0f, angle, 0f) * sensor.forward;
                if (!UnityEngine.Physics.Raycast(sensor.position, direction, out RaycastHit hit,
                        30f, mask, QueryTriggerInteraction.Ignore)) continue;
                CraneSemanticIdentity identity = hit.collider.GetComponentInParent<CraneSemanticIdentity>();
                if (identity == null || identity.SemanticRole == "drivable-track-floor") continue;
                semanticId = identity.SemanticId;
                distance = hit.distance;
                return !string.IsNullOrWhiteSpace(semanticId);
            }
            semanticId = string.Empty;
            distance = 0f;
            return false;
        }

        private static string ReadArgument(string[] args, string key) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length ? args[index + 1] : null;
        }

        private static int ReadIntArgument(string[] args, string key) {
            string value = ReadArgument(args, key);
            return int.TryParse(value, out int parsed) ? parsed : 0;
        }
    }

    internal sealed class CraneReferenceCollisionProbe : MonoBehaviour {
        public bool ContactObserved { get; private set; }

        private void OnCollisionEnter(Collision value) {
            if (value.collider.GetComponentInParent<CraneSemanticIdentity>() != null)
                ContactObserved = true;
        }

        private void OnCollisionStay(Collision value) {
            if (value.collider.GetComponentInParent<CraneSemanticIdentity>() != null)
                ContactObserved = true;
        }
    }
}
