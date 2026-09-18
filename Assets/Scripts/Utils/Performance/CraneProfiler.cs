using System.Threading;
using Unity.Profiling;

namespace Sim.Utils.Performance {
    /// <summary>Stable profiler marker names used by standalone CRANE benchmarks.</summary>
    public static class CraneProfiler {
        public static readonly ProfilerMarker SimulationStep = new("CRANE.Simulation.Step");
        public static readonly ProfilerMarker WaterPrepare = new("CRANE.Water.Prepare");
        public static readonly ProfilerMarker WaterQuery = new("CRANE.Water.Query");
        public static readonly ProfilerMarker WaterBuoyancy = new("CRANE.Water.Buoyancy");
        public static readonly ProfilerMarker VehicleDynamics = new("CRANE.Vehicle.Dynamics");
        public static readonly ProfilerMarker VehicleDynamicsGeneral =
            new("CRANE.Vehicle.Dynamics.General");
        public static readonly ProfilerMarker VehicleDynamicsCurrent =
            new("CRANE.Vehicle.Dynamics.Current");
        public static readonly ProfilerMarker VehicleDynamicsFossen =
            new("CRANE.Vehicle.Dynamics.Fossen");
        public static readonly ProfilerMarker VehicleDynamicsAreaPreparation =
            new("CRANE.Vehicle.Dynamics.AreaPreparation");
        public static readonly ProfilerMarker VehicleDynamicsResistanceCoefficient =
            new("CRANE.Vehicle.Dynamics.ResistanceCoefficient");
        public static readonly ProfilerMarker VehicleDynamicsViscousResistance =
            new("CRANE.Vehicle.Dynamics.ViscousResistance");
        public static readonly ProfilerMarker VehicleDynamicsPressureDrag =
            new("CRANE.Vehicle.Dynamics.PressureDrag");
        public static readonly ProfilerMarker Thruster = new("CRANE.Vehicle.Thruster");
        public static readonly ProfilerMarker Lidar = new("CRANE.Sensor.Lidar");
        public static readonly ProfilerMarker LidarRaycast = new("CRANE.Sensor.Lidar.Raycast");
        public static readonly ProfilerMarker LidarProcess = new("CRANE.Sensor.Lidar.Process");
        public static readonly ProfilerMarker LidarPack = new("CRANE.Sensor.Lidar.Pack");
        public static readonly ProfilerMarker OtherSensor = new("CRANE.Sensor.Other");
        public static readonly ProfilerMarker DepthReadback = new("CRANE.Sensor.Depth.Readback");
        public static readonly ProfilerMarker DepthGeometric = new("CRANE.Sensor.Depth.Geometric");
        public static readonly ProfilerMarker DepthCopy = new("CRANE.Sensor.Depth.Copy");
        public static readonly ProfilerMarker DepthRowFlip = new("CRANE.Sensor.Depth.RowFlip");
        public static readonly ProfilerMarker DepthPublish = new("CRANE.Sensor.Depth.Publish");
        public static readonly ProfilerMarker RgbReadback = new("CRANE.Sensor.RGB.Readback");
        public static readonly ProfilerMarker Detection = new("CRANE.Sensor.Detection");
        public static readonly ProfilerMarker RosCreateMessage = new("CRANE.ROS.CreateMessage");
        public static readonly ProfilerMarker RosPublish = new("CRANE.ROS.Publish");
        public static readonly ProfilerMarker EpisodeReset = new("CRANE.Episode.Reset");
    }

    /// <summary>
    /// Lock-free counters for data quality. Producers report ticks rather than wall time so
    /// accelerated workers can detect stale or cross-episode data.
    /// </summary>
    public static class CraneRuntimeMetrics {
        public static bool CaptureImageSignatures { get; set; }
        public readonly struct LidarSnapshot {
            public readonly long ScanCount;
            public readonly int ConfiguredPointsPerScan;
            public readonly int BatchSize;
            public readonly long TotalPoints;
            public readonly long HitCount;
            public readonly long MissCount;
            public readonly float MinimumRange;
            public readonly float MaximumRange;
            public readonly double MeanRange;
            public readonly ulong Checksum;
            public readonly long AcquisitionTick;

            public LidarSnapshot(long scanCount, int configuredPointsPerScan, int batchSize,
                long totalPoints,
                long hitCount, long missCount, float minimumRange, float maximumRange,
                double meanRange, ulong checksum, long acquisitionTick) {
                ScanCount = scanCount;
                ConfiguredPointsPerScan = configuredPointsPerScan;
                BatchSize = batchSize;
                TotalPoints = totalPoints;
                HitCount = hitCount;
                MissCount = missCount;
                MinimumRange = minimumRange;
                MaximumRange = maximumRange;
                MeanRange = meanRange;
                Checksum = checksum;
                AcquisitionTick = acquisitionTick;
            }
        }

        public readonly struct ImageSnapshot {
            public readonly long AcquisitionCount;
            public readonly int Width;
            public readonly int Height;
            public readonly long TotalBytes;
            public readonly ulong Checksum;
            public readonly long AcquisitionTick;

            public ImageSnapshot(long acquisitionCount, int width, int height, long totalBytes,
                ulong checksum, long acquisitionTick) {
                AcquisitionCount = acquisitionCount;
                Width = width;
                Height = height;
                TotalBytes = totalBytes;
                Checksum = checksum;
                AcquisitionTick = acquisitionTick;
            }
        }

        public readonly struct DetectionSnapshot {
            public readonly long AcquisitionCount;
            public readonly long DetectionCount;
            public readonly long AcquisitionTick;

            public DetectionSnapshot(long acquisitionCount, long detectionCount, long acquisitionTick) {
                AcquisitionCount = acquisitionCount;
                DetectionCount = detectionCount;
                AcquisitionTick = acquisitionTick;
            }
        }

        private static readonly object s_LidarLock = new();
        private static readonly object s_ImageLock = new();
        private static long s_EpisodeId;
        private static long s_SimulationTick;
        private static long s_ObservationTick = -1;
        private static long s_ActionSourceTick = -1;
        private static long s_ActionReceiveTick = -1;
        private static long s_ActionApplicationTick = -1;
        private static long s_ActionSequence = -1;
        private static long s_AcceptedActions;
        private static long s_RejectedActions;
        private static long s_UnknownSourceActions;
        private static long s_CrossEpisodeActions;
        private static long s_DuplicateActions;
        private static long s_StaleObservations;
        private static long s_StaleActions;
        private static long s_FailedObservations;
        private static long s_LidarScanCount;
        private static int s_LidarConfiguredPoints;
        private static int s_LidarBatchSize;
        private static long s_LidarTotalPoints;
        private static long s_LidarHits;
        private static long s_LidarMisses;
        private static float s_LidarMinimumRange = float.PositiveInfinity;
        private static float s_LidarMaximumRange;
        private static double s_LidarRangeSum;
        private static ulong s_LidarChecksum = 14695981039346656037UL;
        private static long s_LidarAcquisitionTick = -1;
        private static long s_LidarCommandValidationCount;
        private static long s_LidarCommandValidationMismatches;
        private static long s_LidarPackValidationBytes;
        private static long s_LidarPackValidationMismatches;
        private static long s_LidarProcessValidationCount;
        private static long s_LidarProcessValidationMismatches;
        private static long s_DepthBufferValidationBytes;
        private static long s_DepthBufferValidationMismatches;
        private static readonly long[] s_ImageAcquisitionCount = new long[2];
        private static readonly int[] s_ImageWidth = new int[2];
        private static readonly int[] s_ImageHeight = new int[2];
        private static readonly long[] s_ImageTotalBytes = new long[2];
        private static readonly ulong[] s_ImageChecksum = { 14695981039346656037UL, 14695981039346656037UL };
        private static readonly long[] s_ImageAcquisitionTick = { -1, -1 };
        private static long s_DetectionAcquisitionCount;
        private static long s_DetectionCount;
        private static long s_DetectionAcquisitionTick = -1;

        public static long EpisodeId => Interlocked.Read(ref s_EpisodeId);
        public static long SimulationTick => Interlocked.Read(ref s_SimulationTick);
        public static long ObservationTick => Interlocked.Read(ref s_ObservationTick);
        public static long ActionSourceTick => Interlocked.Read(ref s_ActionSourceTick);
        public static long ActionReceiveTick => Interlocked.Read(ref s_ActionReceiveTick);
        public static long ActionApplicationTick => Interlocked.Read(ref s_ActionApplicationTick);
        public static long ActionSequence => Interlocked.Read(ref s_ActionSequence);
        public static long AcceptedActions => Interlocked.Read(ref s_AcceptedActions);
        public static long RejectedActions => Interlocked.Read(ref s_RejectedActions);
        public static long UnknownSourceActions => Interlocked.Read(ref s_UnknownSourceActions);
        public static long CrossEpisodeActions => Interlocked.Read(ref s_CrossEpisodeActions);
        public static long DuplicateActions => Interlocked.Read(ref s_DuplicateActions);
        public static long StaleObservations => Interlocked.Read(ref s_StaleObservations);
        public static long StaleActions => Interlocked.Read(ref s_StaleActions);
        public static long FailedObservations => Interlocked.Read(ref s_FailedObservations);
        public static long LidarCommandValidationCount =>
            Interlocked.Read(ref s_LidarCommandValidationCount);
        public static long LidarCommandValidationMismatches =>
            Interlocked.Read(ref s_LidarCommandValidationMismatches);
        public static long LidarPackValidationBytes =>
            Interlocked.Read(ref s_LidarPackValidationBytes);
        public static long LidarPackValidationMismatches =>
            Interlocked.Read(ref s_LidarPackValidationMismatches);
        public static long LidarProcessValidationCount =>
            Interlocked.Read(ref s_LidarProcessValidationCount);
        public static long LidarProcessValidationMismatches =>
            Interlocked.Read(ref s_LidarProcessValidationMismatches);
        public static long DepthBufferValidationBytes =>
            Interlocked.Read(ref s_DepthBufferValidationBytes);
        public static long DepthBufferValidationMismatches =>
            Interlocked.Read(ref s_DepthBufferValidationMismatches);

        public static long AdvanceSimulationTick() => Interlocked.Increment(ref s_SimulationTick);
        public static void ReportObservation(long tick) => Interlocked.Exchange(ref s_ObservationTick, tick);
        public static void ReportActionReceived(long receiveTick, bool sourceUnknown) {
            Interlocked.Exchange(ref s_ActionReceiveTick, receiveTick);
            if (sourceUnknown) Interlocked.Increment(ref s_UnknownSourceActions);
        }
        public static void ReportAction(long sourceTick, long receiveTick, long applicationTick,
            long sequence) {
            Interlocked.Exchange(ref s_ActionSourceTick, sourceTick);
            Interlocked.Exchange(ref s_ActionReceiveTick, receiveTick);
            Interlocked.Exchange(ref s_ActionApplicationTick, applicationTick);
            Interlocked.Exchange(ref s_ActionSequence, sequence);
            Interlocked.Increment(ref s_AcceptedActions);
        }
        public static void ReportRejectedAction(CraneActionRejection rejection) {
            Interlocked.Increment(ref s_RejectedActions);
            if (rejection == CraneActionRejection.Stale)
                Interlocked.Increment(ref s_StaleActions);
            else if (rejection == CraneActionRejection.CrossEpisode)
                Interlocked.Increment(ref s_CrossEpisodeActions);
            else if (rejection == CraneActionRejection.DuplicateOrOutOfOrder)
                Interlocked.Increment(ref s_DuplicateActions);
        }
        public static void ReportStaleObservation() => Interlocked.Increment(ref s_StaleObservations);
        public static void ReportFailedObservation() => Interlocked.Increment(ref s_FailedObservations);

        public static void ReportLidarScan(int configuredPoints, int batchSize, int hits, int misses,
            float minimumRange, float maximumRange, double rangeSum, ulong checksum, long acquisitionTick) {
            lock (s_LidarLock) {
                s_LidarScanCount++;
                s_LidarConfiguredPoints = configuredPoints;
                s_LidarBatchSize = batchSize;
                s_LidarTotalPoints += configuredPoints;
                s_LidarHits += hits;
                s_LidarMisses += misses;
                if (hits > 0) {
                    if (minimumRange < s_LidarMinimumRange) s_LidarMinimumRange = minimumRange;
                    if (maximumRange > s_LidarMaximumRange) s_LidarMaximumRange = maximumRange;
                    s_LidarRangeSum += rangeSum;
                }
                s_LidarChecksum ^= checksum;
                s_LidarChecksum *= 1099511628211UL;
                s_LidarAcquisitionTick = acquisitionTick;
            }
        }

        public static LidarSnapshot GetLidarSnapshot() {
            lock (s_LidarLock) {
                return new LidarSnapshot(s_LidarScanCount, s_LidarConfiguredPoints, s_LidarBatchSize,
                    s_LidarTotalPoints,
                    s_LidarHits, s_LidarMisses,
                    float.IsPositiveInfinity(s_LidarMinimumRange) ? 0 : s_LidarMinimumRange,
                    s_LidarMaximumRange, s_LidarHits == 0 ? 0 : s_LidarRangeSum / s_LidarHits,
                    s_LidarChecksum, s_LidarAcquisitionTick);
            }
        }

        public static void ReportLidarCommandValidation(int compared, int mismatches) {
            Interlocked.Add(ref s_LidarCommandValidationCount, compared);
            Interlocked.Add(ref s_LidarCommandValidationMismatches, mismatches);
        }

        public static void ReportLidarPackValidation(int bytes, int mismatches) {
            Interlocked.Add(ref s_LidarPackValidationBytes, bytes);
            Interlocked.Add(ref s_LidarPackValidationMismatches, mismatches);
        }

        public static void ReportLidarProcessValidation(int compared, int mismatches) {
            Interlocked.Add(ref s_LidarProcessValidationCount, compared);
            Interlocked.Add(ref s_LidarProcessValidationMismatches, mismatches);
        }

        public static void ReportDepthBufferValidation(int bytes, int mismatches) {
            Interlocked.Add(ref s_DepthBufferValidationBytes, bytes);
            Interlocked.Add(ref s_DepthBufferValidationMismatches, mismatches);
        }

        public static void ReportImage(bool depth, int width, int height, byte[] data, long acquisitionTick) {
            int index = depth ? 1 : 0;
            ulong checksum = 14695981039346656037UL;
            if (CaptureImageSignatures) {
                for (int i = 0; i < data.Length; i++) {
                    checksum ^= data[i];
                    checksum *= 1099511628211UL;
                }
            }
            lock (s_ImageLock) {
                s_ImageAcquisitionCount[index]++;
                s_ImageWidth[index] = width;
                s_ImageHeight[index] = height;
                s_ImageTotalBytes[index] += data.Length;
                s_ImageChecksum[index] ^= checksum;
                s_ImageChecksum[index] *= 1099511628211UL;
                s_ImageAcquisitionTick[index] = acquisitionTick;
            }
        }

        public static ImageSnapshot GetImageSnapshot(bool depth) {
            int index = depth ? 1 : 0;
            lock (s_ImageLock) {
                return new ImageSnapshot(s_ImageAcquisitionCount[index], s_ImageWidth[index],
                    s_ImageHeight[index], s_ImageTotalBytes[index], s_ImageChecksum[index],
                    s_ImageAcquisitionTick[index]);
            }
        }

        public static void ReportDetections(int count, long acquisitionTick) {
            Interlocked.Increment(ref s_DetectionAcquisitionCount);
            Interlocked.Add(ref s_DetectionCount, count);
            Interlocked.Exchange(ref s_DetectionAcquisitionTick, acquisitionTick);
        }

        public static DetectionSnapshot GetDetectionSnapshot() => new(
            Interlocked.Read(ref s_DetectionAcquisitionCount),
            Interlocked.Read(ref s_DetectionCount),
            Interlocked.Read(ref s_DetectionAcquisitionTick));

        public static long BeginEpisode() {
            Sim.Utils.ROS.Clock.BeginEpisode();
            Interlocked.Exchange(ref s_SimulationTick, 0);
            Interlocked.Exchange(ref s_ObservationTick, -1);
            Interlocked.Exchange(ref s_ActionSourceTick, -1);
            Interlocked.Exchange(ref s_ActionReceiveTick, -1);
            Interlocked.Exchange(ref s_ActionApplicationTick, -1);
            Interlocked.Exchange(ref s_ActionSequence, -1);
            Interlocked.Exchange(ref s_AcceptedActions, 0);
            Interlocked.Exchange(ref s_RejectedActions, 0);
            Interlocked.Exchange(ref s_UnknownSourceActions, 0);
            Interlocked.Exchange(ref s_CrossEpisodeActions, 0);
            Interlocked.Exchange(ref s_DuplicateActions, 0);
            Interlocked.Exchange(ref s_StaleObservations, 0);
            Interlocked.Exchange(ref s_StaleActions, 0);
            Interlocked.Exchange(ref s_FailedObservations, 0);
            lock (s_LidarLock) {
                s_LidarScanCount = 0;
                s_LidarConfiguredPoints = 0;
                s_LidarBatchSize = 0;
                s_LidarTotalPoints = 0;
                s_LidarHits = 0;
                s_LidarMisses = 0;
                s_LidarMinimumRange = float.PositiveInfinity;
                s_LidarMaximumRange = 0;
                s_LidarRangeSum = 0;
                s_LidarChecksum = 14695981039346656037UL;
                s_LidarAcquisitionTick = -1;
            }
            Interlocked.Exchange(ref s_LidarCommandValidationCount, 0);
            Interlocked.Exchange(ref s_LidarCommandValidationMismatches, 0);
            Interlocked.Exchange(ref s_LidarPackValidationBytes, 0);
            Interlocked.Exchange(ref s_LidarPackValidationMismatches, 0);
            Interlocked.Exchange(ref s_LidarProcessValidationCount, 0);
            Interlocked.Exchange(ref s_LidarProcessValidationMismatches, 0);
            Interlocked.Exchange(ref s_DepthBufferValidationBytes, 0);
            Interlocked.Exchange(ref s_DepthBufferValidationMismatches, 0);
            lock (s_ImageLock) {
                for (int i = 0; i < 2; i++) {
                    s_ImageAcquisitionCount[i] = 0;
                    s_ImageWidth[i] = 0;
                    s_ImageHeight[i] = 0;
                    s_ImageTotalBytes[i] = 0;
                    s_ImageChecksum[i] = 14695981039346656037UL;
                    s_ImageAcquisitionTick[i] = -1;
                }
            }
            Interlocked.Exchange(ref s_DetectionAcquisitionCount, 0);
            Interlocked.Exchange(ref s_DetectionCount, 0);
            Interlocked.Exchange(ref s_DetectionAcquisitionTick, -1);
            return Interlocked.Increment(ref s_EpisodeId);
        }
    }
}
