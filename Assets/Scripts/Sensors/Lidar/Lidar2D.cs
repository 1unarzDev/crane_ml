using System;
using RosMessageTypes.Sensor;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using Sim.Utils.ROS;
using Sim.Utils.Performance;

namespace Sim.Sensors.Lidar {
    public class Lidar2D : MonoBehaviour, IROSSensor<LaserScanMsg> {
        [SerializeField] private float minAngleDegrees = -45.0f;
        [SerializeField] private float maxAngleDegrees = 45.0f;
        [SerializeField] private float angleIncrementDegrees = 1.0f;
        [SerializeField] private float minRange = 0.1f;
        [SerializeField] private float maxRange = 50.0f;
        [SerializeField] private int batchSize = 500;
        [SerializeField] private bool drawRays = false;

        [SerializeField] private string topicName = "scan";
        [SerializeField] private string frameId = "lidar_link";
        [SerializeField] private float Hz = 5.0f;
        public ROSPublisher publisher { get; set; }

        private Vector3[] scanDirVectors;
        private float[] distances;
        private NativeArray<RaycastCommand> commands;
        private NativeArray<RaycastHit> results;

        private void Awake() {
            publisher = gameObject.AddComponent<ROSPublisher>();
        }

        /// <summary>Configures a runtime-created scanner before Start initializes ROS.</summary>
        public void Configure(float minimumAngleDegrees, float maximumAngleDegrees,
            float incrementDegrees, float minimumRange, float maximumRange,
            int configuredBatchSize, bool showRays, string topic, string frame,
            float publishRateHz) {
            minAngleDegrees = minimumAngleDegrees;
            maxAngleDegrees = maximumAngleDegrees;
            angleIncrementDegrees = Mathf.Max(0.01f, incrementDegrees);
            minRange = Mathf.Max(0f, minimumRange);
            maxRange = Mathf.Max(minRange, maximumRange);
            batchSize = Mathf.Max(1, configuredBatchSize);
            drawRays = showRays;
            topicName = topic;
            frameId = frame;
            Hz = Mathf.Max(0.1f, publishRateHz);
        }

        private void Start() {
            publisher.Initialize(topicName, frameId, CreateMessage, Hz);

            scanDirVectors = GenerateScanVectors();
            distances = new float[scanDirVectors.Length + 1];
            commands = new NativeArray<RaycastCommand>(scanDirVectors.Length, Allocator.Persistent);
            results = new NativeArray<RaycastHit>(scanDirVectors.Length, Allocator.Persistent);
        }

        private void OnDestroy() {
            if (commands.IsCreated) commands.Dispose();
            if (results.IsCreated) results.Dispose();
        }

        public LaserScanMsg CreateMessage() {
            using var marker = CraneProfiler.Lidar.Auto();
            float[] dists = PerformScan(scanDirVectors);
            return DistancesToLaserscan(dists);
        }

        private Vector3[] GenerateScanVectors() {
            int numBeams = (int)((maxAngleDegrees - minAngleDegrees) / (angleIncrementDegrees));
            Debug.Assert(numBeams >= 0, "Number of beams is negative. Check min/max angle and angle increment.");
            Vector3[] scanVectors = new Vector3[numBeams];
            float minAngleRad = Mathf.Deg2Rad * minAngleDegrees;
            float angleIncrementRad = Mathf.Deg2Rad * angleIncrementDegrees;
            for (int i = 0; i < numBeams; i++) {
                float hRot = minAngleRad + angleIncrementRad * i;
                float x = -Mathf.Sin(hRot);
                float y = 0;
                float z = Mathf.Cos(hRot);
                scanVectors[i] = new Vector3(x, y, z);
            }
            return scanVectors;
        }

        private float[] PerformScan(Vector3[] dirs) {
            int numPoints = dirs.Length;
            int hitCount = 0;
            float minimumHitRange = float.PositiveInfinity;
            float maximumHitRange = 0;
            double hitRangeSum = 0;
            ulong checksum = 14695981039346656037UL;
            for (int i = 0; i < numPoints; i++) {
                Vector3 origin = transform.position;
                Vector3 direction = transform.rotation * dirs[i];
                commands[i] = new RaycastCommand(origin, direction, QueryParameters.Default, maxRange);
            }

            JobHandle handle = RaycastCommand.ScheduleBatch(commands, results, batchSize, 1);
            handle.Complete();

            for (int i = 0; i < numPoints; i++) {
                var hit = results[i];
                if (hit.collider != null && (transform.position - hit.point).sqrMagnitude > minRange * minRange) {
                    Vector3 beam = transform.InverseTransformPoint(hit.point);
                    distances[i] = hit.distance;
                    hitCount++;
                    minimumHitRange = Mathf.Min(minimumHitRange, hit.distance);
                    maximumHitRange = Mathf.Max(maximumHitRange, hit.distance);
                    hitRangeSum += hit.distance;
                    checksum ^= unchecked((uint)BitConverter.SingleToInt32Bits(hit.distance));
                    checksum *= 1099511628211UL;
                    if (drawRays) {
                        Debug.DrawLine(transform.position, transform.TransformPoint(beam), Color.red);
                    }
                }
                else {
                    distances[i] = float.NaN;
                    checksum ^= uint.MaxValue;
                    checksum *= 1099511628211UL;
                }
            }
            distances[numPoints] = float.NaN;
            CraneRuntimeMetrics.ReportLidarScan(numPoints, batchSize, hitCount,
                numPoints - hitCount,
                minimumHitRange, maximumHitRange, hitRangeSum, checksum,
                CraneRuntimeMetrics.SimulationTick);
            return distances;
        }

        private LaserScanMsg DistancesToLaserscan(float[] dists) {
            LaserScanMsg msg = new() {
                header = publisher.CreateHeader(),

                angle_min = minAngleDegrees * Mathf.Deg2Rad,
                angle_max = maxAngleDegrees * Mathf.Deg2Rad,
                angle_increment = angleIncrementDegrees * Mathf.Deg2Rad,
                scan_time = 1.0f / Hz,
                range_min = minRange,
                range_max = maxRange,
                ranges = dists
            };
            return msg;
        }
    }
}
