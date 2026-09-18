using System;
using RosMessageTypes.Sensor;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using Sim.Utils.ROS;
using UnityEngine;
using Sim.Utils.Performance;

namespace Sim.Sensors.Lidar {
    public class Lidar3D : MonoBehaviour, IROSSensor<PointCloud2Msg> {
        [SerializeField, Range(0.1f, 200.0f)] private float maxRange = 100.0f;
        [SerializeField, Range(0, 5000)] private int numHorizontalBeams = 500;
        [SerializeField, Range(0, 16)] private int numVerticalBeams = 16;
        [SerializeField, Range(0.1f, 5.0f)] private float minDistance = 0.3f;
        [SerializeField, Range(0.0f, 2.0f * Mathf.PI)] private float horizontalFOV = 60.0f * Mathf.PI;
        [SerializeField, Range(0.0f, Mathf.PI)] private float verticalFOV = 0.5f * Mathf.PI;
        [SerializeField] private int batchSize = 500;
        [SerializeField] private bool drawRays = true;

        [SerializeField] private string topicName = "points";
        [SerializeField] private string frameId = "lidar_link";
        [SerializeField] private float Hz = 10.0f;
        public ROSPublisher publisher { get; set; }
        public int BatchSize => batchSize;

        private Vector3[] scanDirVectors;
        private NativeArray<Vector3> scanPoints;
        private NativeArray<Vector3> nativeScanDirections;
        private NativeArray<RaycastCommand> commands;
        private NativeArray<RaycastHit> results;
        private NativeArray<byte> packedPointBytes;
        private NativeArray<ScanSummary> scanSummary;
        private NativeArray<int> validHitIndices;
        private Transform transformCache;
        private Vector3 transformScale;
        private bool validateCommandJob;
        private bool validatePackJob;
        private bool validateProcessPath;
        private long commandJobValidatedEpisode = -1;
        private long packJobValidatedEpisode = -1;
        private long processPathValidatedEpisode = -1;

        private int previousHorizontalBeams;
        private int previousVerticalBeams;
        private float previousHorizontalFov;
        private float previousVerticalFov;

        [BurstCompile(FloatMode = FloatMode.Strict, FloatPrecision = FloatPrecision.High)]
        private struct PopulateRaycastCommandsJob : IJobParallelFor {
            [ReadOnly] public NativeArray<Vector3> LocalDirections;
            [WriteOnly] public NativeArray<RaycastCommand> Commands;
            public Vector3 Origin;
            public Quaternion Rotation;
            public float MaxRange;

            public void Execute(int index) {
                Commands[index] = new RaycastCommand(Origin,
                    Rotation * LocalDirections[index], QueryParameters.Default, MaxRange);
            }
        }

        [BurstCompile(FloatMode = FloatMode.Strict, FloatPrecision = FloatPrecision.High)]
        private struct PackPointCloudJob : IJobParallelFor {
            [ReadOnly] public NativeArray<Vector3> Points;
            [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<byte> Bytes;
            public Vector3 Scale;

            public void Execute(int index) {
                Vector3 point = Points[index];
                int offset = index * 12;
                WriteSingle(offset, point.z * Scale.z);
                WriteSingle(offset + 4, -point.x * Scale.x);
                WriteSingle(offset + 8, point.y * Scale.y);
            }

            private void WriteSingle(int offset, float value) {
                uint bits = math.asuint(value);
                Bytes[offset] = (byte)bits;
                Bytes[offset + 1] = (byte)(bits >> 8);
                Bytes[offset + 2] = (byte)(bits >> 16);
                Bytes[offset + 3] = (byte)(bits >> 24);
            }
        }

        private struct ScanSummary {
            public int HitCount;
            public float MinimumRange;
            public float MaximumRange;
            public double RangeSum;
            public ulong Checksum;
        }

        [BurstCompile(FloatMode = FloatMode.Strict, FloatPrecision = FloatPrecision.High)]
        private struct ProcessRaycastResultsJob : IJob {
            [ReadOnly] public NativeArray<RaycastHit> Results;
            [WriteOnly] public NativeArray<Vector3> Points;
            [WriteOnly] public NativeArray<ScanSummary> Summary;
            [WriteOnly] public NativeArray<int> ValidHitIndices;
            public Vector3 Origin;
            public float MinimumDistanceSquared;

            public void Execute() {
                var value = new ScanSummary {
                    MinimumRange = float.PositiveInfinity,
                    Checksum = 14695981039346656037UL
                };
                Vector3 nan = new(float.NaN, float.NaN, float.NaN);
                for (int i = 0; i < Results.Length; i++) {
                    RaycastHit hit = Results[i];
                    bool valid = hit.colliderEntityId != EntityId.None &&
                        math.lengthsq((float3)(Origin - hit.point)) > MinimumDistanceSquared;
                    if (valid) {
                        float distance = hit.distance;
                        ValidHitIndices[value.HitCount] = i;
                        value.HitCount++;
                        value.MinimumRange = math.min(value.MinimumRange, distance);
                        value.MaximumRange = math.max(value.MaximumRange, distance);
                        value.RangeSum += distance;
                        value.Checksum ^= math.asuint(distance);
                        value.Checksum *= 1099511628211UL;
                        Points[i] = default;
                    }
                    else {
                        value.Checksum ^= uint.MaxValue;
                        value.Checksum *= 1099511628211UL;
                        Points[i] = nan;
                    }
                }
                Summary[0] = value;
            }
        }

        private void Awake() {
            publisher = gameObject.AddComponent<ROSPublisher>();
            string[] args = Environment.GetCommandLineArgs();
            batchSize = ReadPositiveInt(args, "--crane-lidar-batch-size", batchSize);
            validateCommandJob = Array.IndexOf(args,
                "--crane-lidar-command-validation") >= 0;
            validatePackJob = Array.IndexOf(args,
                "--crane-lidar-pack-validation") >= 0;
            validateProcessPath = Array.IndexOf(args,
                "--crane-lidar-process-validation") >= 0;
        }

        private static int ReadPositiveInt(string[] args, string name, int fallback) {
            int index = Array.IndexOf(args, name);
            if (index < 0 || index + 1 >= args.Length ||
                !int.TryParse(args[index + 1], out int value) || value <= 0)
                return fallback;
            return value;
        }

        private void Start() {
            publisher.Initialize(topicName, frameId, CreateMessage, Hz);

            transformCache = transform;
            RebuildScanPattern();
        }

        private void OnDestroy() {
            if (scanPoints.IsCreated) scanPoints.Dispose();
            if (nativeScanDirections.IsCreated) nativeScanDirections.Dispose();
            if (commands.IsCreated) commands.Dispose();
            if (results.IsCreated) results.Dispose();
            if (packedPointBytes.IsCreated) packedPointBytes.Dispose();
            if (scanSummary.IsCreated) scanSummary.Dispose();
            if (validHitIndices.IsCreated) validHitIndices.Dispose();
        }

        private void FixedUpdate() {
            // dont re-calculate lidar scan vectors if parameters unchanged
            if (numHorizontalBeams != previousHorizontalBeams ||
                numVerticalBeams != previousVerticalBeams ||
                !Mathf.Approximately(horizontalFOV, previousHorizontalFov) ||
                !Mathf.Approximately(verticalFOV, previousVerticalFov))
                RebuildScanPattern();
            // ROSPublisher handles publishing internally, so no extra UpdatePublish() needed
        }

        public PointCloud2Msg CreateMessage() {
            using var marker = CraneProfiler.Lidar.Auto();
            transformScale = transform.lossyScale;
            NativeArray<Vector3> points = PerformScan();
            using (CraneProfiler.LidarPack.Auto()) {
                return PointsToPointCloud2(points);
            }
        }

        private void RebuildScanPattern() {
            if (numHorizontalBeams <= 0 || numVerticalBeams <= 0) {
                scanDirVectors = Array.Empty<Vector3>();
            }
            else {
                scanDirVectors = GenerateScanVectors();
            }
            if (scanPoints.IsCreated) scanPoints.Dispose();
            if (nativeScanDirections.IsCreated) nativeScanDirections.Dispose();
            if (commands.IsCreated) commands.Dispose();
            if (results.IsCreated) results.Dispose();
            if (packedPointBytes.IsCreated) packedPointBytes.Dispose();
            if (scanSummary.IsCreated) scanSummary.Dispose();
            if (validHitIndices.IsCreated) validHitIndices.Dispose();
            scanPoints = new NativeArray<Vector3>(scanDirVectors.Length, Allocator.Persistent);
            nativeScanDirections = new NativeArray<Vector3>(scanDirVectors,
                Allocator.Persistent);
            commands = new NativeArray<RaycastCommand>(scanDirVectors.Length, Allocator.Persistent);
            results = new NativeArray<RaycastHit>(scanDirVectors.Length, Allocator.Persistent);
            packedPointBytes = new NativeArray<byte>(scanDirVectors.Length * 12,
                Allocator.Persistent);
            scanSummary = new NativeArray<ScanSummary>(1, Allocator.Persistent);
            validHitIndices = new NativeArray<int>(scanDirVectors.Length, Allocator.Persistent);
            previousHorizontalBeams = numHorizontalBeams;
            previousVerticalBeams = numVerticalBeams;
            previousHorizontalFov = horizontalFOV;
            previousVerticalFov = verticalFOV;
        }

        private Vector3[] GenerateScanVectors() {
            float fidelityHorizontal = horizontalFOV / numHorizontalBeams;
            float fidelityVertical = verticalFOV / numVerticalBeams;

            Vector3[] scanVectors = new Vector3[numHorizontalBeams * numVerticalBeams];

            for (int i = 0; i < numHorizontalBeams; i++) {
                float hRot = 0.5f * (Mathf.PI - horizontalFOV) + fidelityHorizontal * i;

                for (int j = 0; j < numVerticalBeams; j++) {
                    float vRot = 0.5f * (Mathf.PI - verticalFOV) + (fidelityVertical * j);

                    float x = Mathf.Sin(vRot) * Mathf.Cos(hRot);
                    float y = Mathf.Cos(vRot);
                    float z = Mathf.Sin(vRot) * Mathf.Sin(hRot);

                    scanVectors[i * numVerticalBeams + j] = new Vector3(x, y, z);
                }
            }
            return scanVectors;
        }

        private NativeArray<Vector3> PerformScan() {
            int numPoints = scanPoints.Length;
            Vector3 origin = transformCache.position;
            Quaternion rotation = transformCache.rotation;
            using (CraneProfiler.LidarRaycast.Auto()) {
                JobHandle commandHandle = new PopulateRaycastCommandsJob {
                    LocalDirections = nativeScanDirections,
                    Commands = commands,
                    Origin = origin,
                    Rotation = rotation,
                    MaxRange = maxRange
                }.Schedule(numPoints, batchSize);
                long episodeId = CraneRuntimeMetrics.EpisodeId;
                if (validateCommandJob && commandJobValidatedEpisode != episodeId) {
                    commandHandle.Complete();
                    int mismatches = 0;
                    for (int i = 0; i < numPoints; i++) {
                        var expected = new RaycastCommand(origin,
                            rotation * scanDirVectors[i], QueryParameters.Default, maxRange);
                        RaycastCommand actual = commands[i];
                        QueryParameters expectedQuery = expected.queryParameters;
                        QueryParameters actualQuery = actual.queryParameters;
                        if (actual.from != expected.from || actual.direction != expected.direction ||
                            actual.distance != expected.distance ||
                            actualQuery.layerMask != expectedQuery.layerMask ||
                            actualQuery.hitBackfaces != expectedQuery.hitBackfaces ||
                            actualQuery.hitMultipleFaces != expectedQuery.hitMultipleFaces ||
                            actualQuery.hitTriggers != expectedQuery.hitTriggers)
                            mismatches++;
                    }
                    commandJobValidatedEpisode = episodeId;
                    CraneRuntimeMetrics.ReportLidarCommandValidation(numPoints, mismatches);
                    if (mismatches != 0)
                        Debug.LogError($"CRANE LiDAR command validation found {mismatches} mismatches " +
                                       $"across {numPoints} beams.");
                }
                JobHandle handle = RaycastCommand.ScheduleBatch(commands, results, batchSize, 1,
                    commandHandle);
                handle.Complete();
            }

            using (CraneProfiler.LidarProcess.Auto()) {
                float minimumDistanceSquared = minDistance * minDistance;
                ScanSummary summary = drawRays
                    ? ProcessResultsManaged(origin, minimumDistanceSquared)
                    : ProcessResultsBurst(origin, minimumDistanceSquared);
                CraneRuntimeMetrics.ReportLidarScan(numPoints, batchSize, summary.HitCount,
                    numPoints - summary.HitCount,
                    summary.MinimumRange, summary.MaximumRange, summary.RangeSum, summary.Checksum,
                    CraneRuntimeMetrics.SimulationTick);
            }
            return scanPoints;
        }

        private ScanSummary ProcessResultsBurst(Vector3 origin, float minimumDistanceSquared) {
            new ProcessRaycastResultsJob {
                Results = results,
                Points = scanPoints,
                Summary = scanSummary,
                ValidHitIndices = validHitIndices,
                Origin = origin,
                MinimumDistanceSquared = minimumDistanceSquared
            }.Schedule().Complete();
            ScanSummary summary = scanSummary[0];
            for (int i = 0; i < summary.HitCount; i++) {
                int resultIndex = validHitIndices[i];
                scanPoints[resultIndex] = transformCache.InverseTransformPoint(
                    results[resultIndex].point);
            }
            ValidateProcessPath(origin, minimumDistanceSquared, summary);
            return summary;
        }

        private ScanSummary ProcessResultsManaged(Vector3 origin, float minimumDistanceSquared) {
            var summary = new ScanSummary {
                MinimumRange = float.PositiveInfinity,
                Checksum = 14695981039346656037UL
            };
            Vector3 nan = new(float.NaN, float.NaN, float.NaN);
            for (int i = 0; i < results.Length; i++) {
                RaycastHit hit = results[i];
                bool valid = hit.colliderEntityId != EntityId.None &&
                    (origin - hit.point).sqrMagnitude > minimumDistanceSquared;
                if (valid) {
                    float distance = hit.distance;
                    summary.HitCount++;
                    summary.MinimumRange = Mathf.Min(summary.MinimumRange, distance);
                    summary.MaximumRange = Mathf.Max(summary.MaximumRange, distance);
                    summary.RangeSum += distance;
                    summary.Checksum ^= unchecked((uint)BitConverter.SingleToInt32Bits(distance));
                    summary.Checksum *= 1099511628211UL;
                    Vector3 beam = transformCache.InverseTransformPoint(hit.point);
                    scanPoints[i] = beam;
                    Debug.DrawLine(origin, hit.point, Color.red);
                }
                else {
                    summary.Checksum ^= uint.MaxValue;
                    summary.Checksum *= 1099511628211UL;
                    scanPoints[i] = nan;
                }
            }
            return summary;
        }

        private void ValidateProcessPath(Vector3 origin, float minimumDistanceSquared,
            ScanSummary summary) {
            long episodeId = CraneRuntimeMetrics.EpisodeId;
            if (!validateProcessPath || processPathValidatedEpisode == episodeId) return;
            int mismatches = 0;
            int classificationMismatches = 0;
            int pointMismatches = 0;
            float maximumPointError = 0f;
            var expectedSummary = new ScanSummary {
                MinimumRange = float.PositiveInfinity,
                Checksum = 14695981039346656037UL
            };
            for (int i = 0; i < results.Length; i++) {
                RaycastHit hit = results[i];
                bool hasCollider = hit.colliderEntityId != EntityId.None;
                if (hasCollider != (hit.collider != null)) {
                    classificationMismatches++;
                    mismatches++;
                }
                bool valid = hasCollider &&
                    (origin - hit.point).sqrMagnitude > minimumDistanceSquared;
                if (valid) {
                    float distance = hit.distance;
                    expectedSummary.HitCount++;
                    expectedSummary.MinimumRange = Mathf.Min(expectedSummary.MinimumRange, distance);
                    expectedSummary.MaximumRange = Mathf.Max(expectedSummary.MaximumRange, distance);
                    expectedSummary.RangeSum += distance;
                    expectedSummary.Checksum ^=
                        unchecked((uint)BitConverter.SingleToInt32Bits(distance));
                    expectedSummary.Checksum *= 1099511628211UL;
                    Vector3 expectedPoint = transformCache.InverseTransformPoint(hit.point);
                    if (scanPoints[i] != expectedPoint) {
                        pointMismatches++;
                        mismatches++;
                        maximumPointError = Mathf.Max(maximumPointError,
                            Vector3.Distance(scanPoints[i], expectedPoint));
                    }
                }
                else {
                    expectedSummary.Checksum ^= uint.MaxValue;
                    expectedSummary.Checksum *= 1099511628211UL;
                    if (!float.IsNaN(scanPoints[i].x) || !float.IsNaN(scanPoints[i].y) ||
                        !float.IsNaN(scanPoints[i].z)) mismatches++;
                }
            }
            bool summaryMismatch = summary.HitCount != expectedSummary.HitCount ||
                summary.MinimumRange != expectedSummary.MinimumRange ||
                summary.MaximumRange != expectedSummary.MaximumRange ||
                summary.RangeSum != expectedSummary.RangeSum ||
                summary.Checksum != expectedSummary.Checksum;
            if (summaryMismatch) mismatches++;
            processPathValidatedEpisode = episodeId;
            CraneRuntimeMetrics.ReportLidarProcessValidation(results.Length, mismatches);
            if (mismatches != 0)
                Debug.LogError($"CRANE LiDAR Burst processing validation found {mismatches} " +
                               $"mismatches across {results.Length} beams: " +
                               $"classification={classificationMismatches} " +
                               $"points={pointMismatches} maxPointError={maximumPointError:R} " +
                               $"summary={summaryMismatch} hits={summary.HitCount}/" +
                               $"{expectedSummary.HitCount} min={summary.MinimumRange:R}/" +
                               $"{expectedSummary.MinimumRange:R} max={summary.MaximumRange:R}/" +
                               $"{expectedSummary.MaximumRange:R} sum={summary.RangeSum:R}/" +
                               $"{expectedSummary.RangeSum:R} checksum={summary.Checksum:X16}/" +
                               $"{expectedSummary.Checksum:X16}.");
        }

        private PointCloud2Msg PointsToPointCloud2(NativeArray<Vector3> points) {
            PointCloud2Msg msg = new PointCloud2Msg();
            msg.header = publisher.CreateHeader();

            // publishing as unordered cloud (height = 1). might reconsider later, idk.
            msg.height = 1;
            // currently the size of each message is non-constant, as the number of scan returns varies.
            // could consider having a constant size pointcloud and filling non-hits with NaN values.
            msg.width = (uint)points.Length;

            PointFieldMsg[] fields = new PointFieldMsg[3];
            fields[0] = new PointFieldMsg("x", 0, 7, 1); // "name", offset, datatype (7 = float), number of elements in field
            fields[1] = new PointFieldMsg("y", 4, 7, 1); // 4 byte offset, since float32 uses 4 bytes
            fields[2] = new PointFieldMsg("z", 8, 7, 1); // another 4 bytes as offset
            // theres an option for this field too, but i dont see a use for it currently
            // fields[3] = new PointFieldMsg("intensity", 12, 7, msg.width);
            msg.fields = fields;

            msg.point_step = (uint)fields.Length * 4; // each point needs 12 bytes (3 float32's, when using fields x, y, z)
            msg.row_step = msg.point_step * msg.width;
            msg.is_dense = true;

            // finally, populate the data field, containing the actual points in bytes
            JobHandle packHandle = new PackPointCloudJob {
                Points = points,
                Bytes = packedPointBytes,
                Scale = transformScale
            }.Schedule(points.Length, batchSize);
            packHandle.Complete();

            byte[] data = new byte[packedPointBytes.Length];
            packedPointBytes.CopyTo(data);

            long episodeId = CraneRuntimeMetrics.EpisodeId;
            if (validatePackJob && packJobValidatedEpisode != episodeId) {
                int mismatches = 0;
                byte[] reference = new byte[data.Length];
                for (int i = 0; i < points.Length; i++) {
                    Vector3 point = points[i];
                    int offset = i * 12;
                    BitConverter.TryWriteBytes(reference.AsSpan(offset, 4), point.z * transformScale.z);
                    BitConverter.TryWriteBytes(reference.AsSpan(offset + 4, 4), -point.x * transformScale.x);
                    BitConverter.TryWriteBytes(reference.AsSpan(offset + 8, 4), point.y * transformScale.y);
                }
                for (int i = 0; i < data.Length; i++) {
                    if (data[i] != reference[i]) mismatches++;
                }
                packJobValidatedEpisode = episodeId;
                CraneRuntimeMetrics.ReportLidarPackValidation(data.Length, mismatches);
                if (mismatches != 0)
                    Debug.LogError($"CRANE LiDAR packing validation found {mismatches} mismatches " +
                                   $"across {data.Length} bytes.");
            }
            msg.data = data;
            return msg;
        }
    }
}
