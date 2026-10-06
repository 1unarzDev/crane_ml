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

        private System.Random errorRandom;
        private float rangeSigma, dropoutProbability;
        private string responseClass = "off";
        private int delayScans;
        private readonly System.Collections.Generic.Queue<LaserScanMsg> delayedScans = new();
        public long ErrorDropouts { get; private set; }
        long errorScans,geometricReturns,materialReturns,materialDropouts,noiseSamples;
        double noiseSum,noiseSquaredSum;
        [Serializable] sealed class ErrorEvidence {
            public long scans,geometricReturns,injectedDropouts,materialReturns,materialDropouts,noiseSamples;
            public double noiseSumMeters,noiseSquaredSumMetersSquared;
            public float configuredSigmaMeters,configuredDropoutProbability;
            public int latencyScans;public string materialClass;
        }
        public string ErrorDiagnosticsJson()=>JsonUtility.ToJson(new ErrorEvidence{
            scans=errorScans,geometricReturns=geometricReturns,injectedDropouts=ErrorDropouts,
            materialReturns=materialReturns,materialDropouts=materialDropouts,noiseSamples=noiseSamples,
            noiseSumMeters=noiseSum,noiseSquaredSumMetersSquared=noiseSquaredSum,
            configuredSigmaMeters=rangeSigma,configuredDropoutProbability=dropoutProbability,
            latencyScans=delayScans,materialClass=responseClass});
        public void ConfigureErrors(int seed, float sigma, float dropout, int latencyScans, string materialClass) {
            if (!float.IsFinite(sigma) || !float.IsFinite(dropout) || sigma > .1f || sigma < 0 || dropout < 0 || dropout > .5f || latencyScans < 0 || latencyScans > 3)
                throw new ArgumentOutOfRangeException("LiDAR error profile");
            errorRandom = new System.Random(seed); rangeSigma = sigma; dropoutProbability = dropout;
            delayScans = latencyScans; responseClass = materialClass ?? "off";
            delayedScans.Clear(); ErrorDropouts = 0;
            errorScans=geometricReturns=materialReturns=materialDropouts=noiseSamples=0;noiseSum=noiseSquaredSum=0;
        }
        private double Gaussian() { return Math.Sqrt(-2*Math.Log(Math.Max(1e-12,errorRandom.NextDouble()))) * Math.Cos(2*Math.PI*errorRandom.NextDouble()); }

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
            var current = DistancesToLaserscan(dists);
            if (errorRandom == null || delayScans == 0) return current;
            delayedScans.Enqueue(current);
            // Acquisition stamps are preserved. During queue warmup hold the earliest acquisition
            // rather than publishing newer stamps and subsequently jumping backwards.
            if (delayedScans.Count <= delayScans) return delayedScans.Peek();
            return delayedScans.Dequeue();
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
            if(errorRandom!=null)errorScans++;
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
                    if (errorRandom != null) {
                        geometricReturns++;
                        var response = hit.collider.GetComponentInParent<CraneLidarResponse>();
                        bool materialMatch=response!=null&&responseClass!="off"&&response.Class==responseClass;
                        if(materialMatch)materialReturns++;
                        float chance = dropoutProbability;
                        if (response != null && responseClass != "off") {
                            float grazing = 1-Mathf.Abs(Vector3.Dot(hit.normal,(transform.rotation*dirs[i]).normalized));
                            chance += responseClass == "reflective" && response.Class == "reflective" ? .08f + .12f*grazing :
                                      responseClass == "dark" && response.Class == "dark" ? .12f : 0;
                        }
                        if (errorRandom.NextDouble() < chance) { distances[i] = float.NaN; ErrorDropouts++;if(materialMatch)materialDropouts++; }
                        else {
                            distances[i] = Mathf.Clamp(hit.distance+(float)Gaussian()*rangeSigma,minRange,maxRange);
                            double realizedError=distances[i]-hit.distance;noiseSamples++;noiseSum+=realizedError;noiseSquaredSum+=realizedError*realizedError;
                        }
                    }
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
                ranges = delayScans > 0 ? (float[])dists.Clone() : dists
            };
            return msg;
        }
    }
}
