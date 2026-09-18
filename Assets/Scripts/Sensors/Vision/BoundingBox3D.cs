using RosMessageTypes.Vision;
using RosMessageTypes.Geometry;
using Sim.Utils.ROS;
using Sim.Utils.Performance;
using UnityEngine;
using System.Collections.Generic;

namespace Sim.Sensors.Vision {
    [System.Serializable]
    public class ObjectEntry {
        public GameObject obj;
        public string id;
    }

    public class BoundingBox3D : MonoBehaviour, ICraneEpisodeResettable {
        [SerializeField] private string topicName = "/detections";
        [SerializeField] private string frameId = "front_camera_link";
        [SerializeField] private Camera sensorCamera;
        [SerializeField] private float Hz = 10f;

        [SerializeField] private float minDist = 1f;
        [SerializeField] private float maxDist = 20f;
        [SerializeField] private LayerMask occlusionMask = ~0;

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
                if (!HasLineOfSight(obj, worldBounds.center)) continue;

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
                        CalculateConfidence(dist))
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

        private float CalculateConfidence(float distance) {
            float rangeQuality = 1f - Mathf.InverseLerp(minDist, maxDist, distance);
            return Mathf.Clamp01(0.8f + 0.2f * rangeQuality);
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
