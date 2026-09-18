using RosMessageTypes.Vision;
using RosMessageTypes.Geometry;
using Sim.Utils.ROS;
using Sim.Utils.Performance;
using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;

namespace Sim.Sensors.Vision {
    [System.Serializable]
    public class ObjectEntry {
        public GameObject obj;
        public string id;
    }

    public class BoundingBox3D : MonoBehaviour, ICraneEpisodeResettable {
        [Serializable]
        private sealed class VisibilityValidationResult {
            public string schema = "crane-detection-visibility-validation-v1";
            public float clearFraction;
            public float centerOccludedFraction;
            public float fullyOccludedFraction;
            public bool valid;
        }

        private static bool visibilityValidationInstalled;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void InstallVisibilityValidation() {
            string[] args = Environment.GetCommandLineArgs();
            if (visibilityValidationInstalled ||
                Array.IndexOf(args, "--crane-detection-visibility-validation") < 0) return;
            visibilityValidationInstalled = true;

            var host = new GameObject("CRANE Detection Visibility Validation");
            host.SetActive(false);
            Camera camera = host.AddComponent<Camera>();
            camera.enabled = false;
            var sensor = host.AddComponent<BoundingBox3D>();
            sensor.sensorCamera = camera;
            sensor.maxPartialVisibilitySamples = 4;
            sensor.minimumVisibleFraction = 0.2f;
            sensor.enabled = false;

            var target = new GameObject("Semantic Target");
            for (int i = 0; i < 2; i++) {
                GameObject part = GameObject.CreatePrimitive(PrimitiveType.Cube);
                part.name = $"Target Part {i}";
                part.transform.SetParent(target.transform);
                part.transform.position = new Vector3(i == 0 ? -1f : 1f, 0f, 10f);
            }
            GameObject occluder = GameObject.CreatePrimitive(PrimitiveType.Cube);
            occluder.name = "Visibility Occluder";
            occluder.transform.position = new Vector3(0f, 0f, 5f);

            host.SetActive(true);
            sensor.StartCoroutine(sensor.ValidateVisibility(target, occluder, ReadArgument(args,
                "--crane-detection-visibility-validation-output")));
        }

        private IEnumerator ValidateVisibility(GameObject target, GameObject occluder,
            string outputPath) {
            yield return new WaitForFixedUpdate();
            Physics.SyncTransforms();
            Bounds bounds = ComputeWorldBounds(target);
            occluder.SetActive(false);
            Physics.SyncTransforms();
            float clear = CalculateVisibleFraction(target, bounds.center);
            occluder.SetActive(true);
            occluder.transform.localScale = new Vector3(0.6f, 4f, 0.5f);
            Physics.SyncTransforms();
            float partial = CalculateVisibleFraction(target, bounds.center);
            occluder.transform.localScale = new Vector3(4f, 4f, 0.5f);
            Physics.SyncTransforms();
            float blocked = CalculateVisibleFraction(target, bounds.center);
            var result = new VisibilityValidationResult {
                clearFraction = clear,
                centerOccludedFraction = partial,
                fullyOccludedFraction = blocked,
                valid = clear == 1f && partial > 0f && blocked == 0f
            };
            string json = JsonUtility.ToJson(result, true);
            Debug.Log($"CRANE_DETECTION_VISIBILITY_VALIDATION {json}");
            if (!string.IsNullOrWhiteSpace(outputPath)) {
                string path = Path.GetFullPath(outputPath);
                string directory = Path.GetDirectoryName(path);
                if (!string.IsNullOrEmpty(directory)) Directory.CreateDirectory(directory);
                File.WriteAllText(path, json);
            }
            Application.Quit(result.valid ? 0 : 2);
        }

        private static string ReadArgument(string[] args, string key) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length ? args[index + 1] : null;
        }

        [SerializeField] private string topicName = "/detections";
        [SerializeField] private string frameId = "front_camera_link";
        [SerializeField] private Camera sensorCamera;
        [SerializeField] private float Hz = 10f;

        [SerializeField] private float minDist = 1f;
        [SerializeField] private float maxDist = 20f;
        [SerializeField] private LayerMask occlusionMask = ~0;
        [SerializeField, Min(1)] private int maxPartialVisibilitySamples = 4;
        [SerializeField, Range(0f, 1f)] private float minimumVisibleFraction = 0.2f;

        [SerializeField] private bool drawGizmos = true;
        [SerializeField] private float gizmoScale = 0.05f;

        private ROSPublisher publisher;
        private float timeSincePublish;

        [SerializeField] private List<ObjectEntry> objects = new();
        private Dictionary<GameObject, string> objectDict = new();
        private readonly Dictionary<GameObject, Renderer[]> renderersByObject = new();
        public int ResetPriority => -70;
        public void CaptureEpisodeInitialState() { }
        public void ResetEpisode(in CraneEpisodeResetContext context,
            CraneEpisodeResetPhase phase) {
            if (phase == CraneEpisodeResetPhase.BeforePhysics) timeSincePublish = 0;
        }

        private void Awake() {
            foreach (var entry in objects) {
                if (entry.obj != null && !string.IsNullOrEmpty(entry.id)) {
                    objectDict[entry.obj] = entry.id;
                    renderersByObject[entry.obj] = entry.obj.GetComponentsInChildren<Renderer>();
                }
            }

            if (sensorCamera == null) {
                Debug.LogError("Missing camera reference.");
                enabled = false;
                return;
            }

            publisher = gameObject.AddComponent<ROSPublisher>();
        }

        private void Start() {
            publisher.Initialize(topicName, frameId, CreateMessage, Hz, true);
        }

        private void FixedUpdate() {
            timeSincePublish += Time.fixedDeltaTime;
            if (timeSincePublish >= 1f / Hz) {
                publisher.Publish();
                timeSincePublish -= 1f / Hz;
            }
        }

        private void OnDrawGizmos() {
            if (!drawGizmos) return;

            foreach (var entry in objects) {
                if (entry.obj == null) continue;

                ComputeLocalBounds(entry.obj, out Vector3 localCenter, out Vector3 size);

                Vector3 worldCenter = entry.obj.transform.TransformPoint(localCenter);
                Quaternion rotation = entry.obj.transform.rotation;

                DrawBoundingBox(worldCenter, rotation, size);
            }
        }

        private void DrawBoundingBox(Vector3 center, Quaternion rotation, Vector3 size) {
            Vector3 half = size * 0.5f;

            Vector3[] localOffsets = new Vector3[]
            {
                new(-half.x, -half.y, -half.z),
                new( half.x, -half.y, -half.z),
                new( half.x, -half.y,  half.z),
                new(-half.x, -half.y,  half.z),

                new(-half.x,  half.y, -half.z),
                new( half.x,  half.y, -half.z),
                new( half.x,  half.y,  half.z),
                new(-half.x,  half.y,  half.z),
            };

            Vector3[] corners = new Vector3[8];

            for (int i = 0; i < 8; i++)
                corners[i] = center + rotation * localOffsets[i];

            Gizmos.color = Color.red;

            foreach (var c in corners)
                Gizmos.DrawSphere(c, gizmoScale);

            int[,] edges = {
                {0,1},{1,2},{2,3},{3,0},
                {4,5},{5,6},{6,7},{7,4},
                {0,4},{1,5},{2,6},{3,7}
            };

            for (int i = 0; i < 12; i++)
                Gizmos.DrawLine(corners[edges[i, 0]], corners[edges[i, 1]]);
        }

        private Detection3DArrayMsg CreateMessage() {
            using var marker = CraneProfiler.OtherSensor.Auto();
            using var sensorMarker = CraneProfiler.Detection.Auto();
            List<Detection3DMsg> detections = new();
            Plane[] frustumPlanes = GeometryUtility.CalculateFrustumPlanes(sensorCamera);

            foreach (var kvp in objectDict) {
                GameObject obj = kvp.Key;
                string id = kvp.Value;

                ComputeLocalBounds(obj, out Vector3 localCenter, out Vector3 localSize);

                Vector3 worldCenter = obj.transform.TransformPoint(localCenter);

                float dist = Vector3.Magnitude(worldCenter - sensorCamera.transform.position);
                bool inRange = minDist <= dist && dist <= maxDist;
                Bounds worldBounds = ComputeWorldBounds(obj);
                bool inFrustum = GeometryUtility.TestPlanesAABB(frustumPlanes, worldBounds);

                if (!inFrustum || !inRange) continue;
                float visibleFraction = CalculateVisibleFraction(obj, worldBounds.center);
                if (visibleFraction < minimumVisibleFraction) continue;

                // Transform to camera frame
                Vector3 cameraSpaceCenter =
                    sensorCamera.transform.InverseTransformPoint(worldCenter);

                // Rotation in camera frame
                Quaternion cameraSpaceRotation =
                    Quaternion.Inverse(obj.transform.rotation) *
                    sensorCamera.transform.rotation;

                // Convert to ROS
                Vector3 rosPosition = UnityToROSPosition(cameraSpaceCenter);
                Quaternion rosRotation = UnityToROSRotation(cameraSpaceRotation);

                detections.Add(
                    GenerateDetection(rosPosition, rosRotation, localSize, id,
                        CalculateConfidence(dist, visibleFraction))
                );
            }

            CraneRuntimeMetrics.ReportDetections(detections.Count,
                CraneRuntimeMetrics.SimulationTick);

            return new Detection3DArrayMsg(
                publisher.CreateHeader(),
                detections.ToArray()
            );
        }

        private Detection3DMsg GenerateDetection(
            Vector3 rosPosition,
            Quaternion rosRotation,
            Vector3 size,
            string id,
            float confidence) {
            PoseMsg pose = new(
                new PointMsg(rosPosition.x, rosPosition.y, rosPosition.z),
                new QuaternionMsg(rosRotation.x, rosRotation.y, rosRotation.z, rosRotation.w)
            );

            double[] covariance = new double[36];

            ObjectHypothesisWithPoseMsg hypothesis =
                new(
                    new ObjectHypothesisMsg(id, confidence),
                    new PoseWithCovarianceMsg(pose, covariance)
                );

            BoundingBox3DMsg bbox = new(
                pose,
                new Vector3Msg(size.x, size.y, size.z)
            );

            return new Detection3DMsg(
                publisher.CreateHeader(),
                new[] { hypothesis },
                bbox,
                id
            );
        }

        private bool HasLineOfSight(GameObject target, Vector3 sample) {
            Vector3 origin = sensorCamera.transform.position;
            Vector3 offset = sample - origin;
            float distance = offset.magnitude;
            if (distance <= Mathf.Epsilon) return true;
            return !UnityEngine.Physics.Raycast(origin, offset / distance, out RaycastHit hit,
                       distance, occlusionMask, QueryTriggerInteraction.Ignore) ||
                   hit.distance >= distance - 0.02f ||
                   hit.transform == target.transform || hit.transform.IsChildOf(target.transform);
        }

        private float CalculateVisibleFraction(GameObject target, Vector3 center) {
            if (HasLineOfSight(target, center)) return 1f;

            Renderer[] renderers = GetRenderers(target);
            int sampleCount = Mathf.Min(maxPartialVisibilitySamples, renderers.Length);
            if (sampleCount == 0) return 0f;
            int visibleSamples = 0;
            for (int i = 0; i < sampleCount; i++) {
                int rendererIndex = sampleCount == 1 ? 0 : Mathf.RoundToInt(
                    i * (renderers.Length - 1f) / (sampleCount - 1));
                if (HasLineOfSight(target, renderers[rendererIndex].bounds.center))
                    visibleSamples++;
            }
            return visibleSamples / (float)sampleCount;
        }

        private float CalculateConfidence(float distance, float visibleFraction) {
            float rangeQuality = 1f - Mathf.InverseLerp(minDist, maxDist, distance);
            return Mathf.Clamp01((0.8f + 0.2f * rangeQuality) * visibleFraction);
        }

        private Bounds ComputeWorldBounds(GameObject obj) {
            Renderer[] renderers = GetRenderers(obj);
            if (renderers.Length == 0) return new Bounds(obj.transform.position, Vector3.zero);
            Bounds bounds = renderers[0].bounds;
            for (int i = 1; i < renderers.Length; i++) bounds.Encapsulate(renderers[i].bounds);
            return bounds;
        }

        private Renderer[] GetRenderers(GameObject obj) {
            if (renderersByObject.TryGetValue(obj, out Renderer[] renderers)) return renderers;
            renderers = obj.GetComponentsInChildren<Renderer>();
            renderersByObject[obj] = renderers;
            return renderers;
        }

        private void ComputeLocalBounds(
            GameObject obj,
            out Vector3 center,
            out Vector3 size) {
            Renderer[] renderers = GetRenderers(obj);

            if (renderers.Length == 0) {
                center = Vector3.zero;
                size = Vector3.zero;
                return;
            }

            bool initialized = false;
            Vector3 min = Vector3.zero;
            Vector3 max = Vector3.zero;

            foreach (var r in renderers) {
                Bounds b = r.localBounds;

                Vector3 worldCenter = r.transform.TransformPoint(b.center);
                Vector3 localCenter = obj.transform.InverseTransformPoint(worldCenter);

                Vector3 extents = Vector3.Scale(b.extents, r.transform.lossyScale);

                Vector3 localMin = localCenter - extents;
                Vector3 localMax = localCenter + extents;

                if (!initialized) {
                    min = localMin;
                    max = localMax;
                    initialized = true;
                }
                else {
                    min = Vector3.Min(min, localMin);
                    max = Vector3.Max(max, localMax);
                }
            }

            center = (min + max) * 0.5f;
            size = max - min;
        }

        private Vector3 UnityToROSPosition(Vector3 unityPos) {
            return new Vector3(unityPos.x, unityPos.z, unityPos.y);
        }

        private Quaternion UnityToROSRotation(Quaternion unityRot) {
            return Quaternion.AngleAxis(-90f, Vector3.forward) * Quaternion.AngleAxis(-90f, Vector3.up) * Quaternion.AngleAxis(-90f, Vector3.forward) * unityRot;
        }
    }
}
