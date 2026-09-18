using System;
using System.Collections.Generic;
using System.Collections;
using System.Diagnostics;
using System.IO;
using System.Linq;
using Sim.Utils.Performance;
using Unity.Profiling;
using UnityEngine;
using UnityEngine.Profiling;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;
using UnityEngine.SceneManagement;
using Sim.Physics.Contacts;
using Debug = UnityEngine.Debug;

namespace Sim.Performance {
    /// <summary>Standalone benchmark recorder enabled with --crane-benchmark.</summary>
    public sealed class CraneBenchmarkRunner : MonoBehaviour {
        [Serializable] private sealed class MarkerResult {
            public string name;
            public double totalMilliseconds;
            public double meanMillisecondsPerFrame;
            public double maxMillisecondsPerFrame;
            public long sampledFrames;
        }

        [Serializable] private sealed class BodySample {
            public string name;
            public Vector3 position;
            public Quaternion rotation;
            public Vector3 linearVelocity;
            public Vector3 angularVelocity;
        }

        [Serializable] private sealed class WaterSample {
            public string surface;
            public float simulationTime;
            public float timeMultiplier;
            public Vector3 target;
            public Vector3 projected;
            public float error;
            public int steps;
            public bool valid;
        }

        [Serializable] private sealed class ValidationSample {
            public double simulatedSeconds;
            public long monoUsedBytes;
            public long totalAllocatedMemoryBytes;
            public long observationQueueAgeTicks;
            public long staleObservations;
            public long failedObservations;
            public List<BodySample> bodies = new();
            public List<WaterSample> water = new();
        }

        [Serializable] private sealed class LidarResult {
            public long scanCount;
            public int configuredPointsPerScan;
            public int batchSize;
            public long totalPoints;
            public long hitCount;
            public long missCount;
            public float minimumRange;
            public float maximumRange;
            public double meanRange;
            public string checksum;
            public long acquisitionTick;
            public long commandValidationCount;
            public long commandValidationMismatches;
            public long packValidationBytes;
            public long packValidationMismatches;
            public long processValidationCount;
            public long processValidationMismatches;
        }

        [Serializable] private sealed class ImageResult {
            public long acquisitionCount;
            public int width;
            public int height;
            public long totalBytes;
            public string checksum;
            public long acquisitionTick;
        }

        [Serializable] private sealed class DetectionResult {
            public long acquisitionCount;
            public long detectionCount;
            public long acquisitionTick;
        }

        [Serializable] private sealed class DifferenceResult {
            public double rms;
            public double max;
            public int count;
        }

        [Serializable] private sealed class ActionTimingResult {
            public long acceptedActions;
            public long knownSourceActions;
            public double meanSourceToApplicationTicks;
            public long maximumSourceToApplicationTicks;
            public double meanReceiveToApplicationTicks;
            public long maximumReceiveToApplicationTicks;
            public long maximumInterApplicationTicks;
            public long commandTimeouts;
        }

        [Serializable] private sealed class SitlResult {
            public long validServoPackets;
            public long invalidServoPackets;
            public long telemetryPackets;
            public long lastFrame;
            public double lastTelemetrySimulatedSeconds;
        }

        [Serializable] private sealed class ResetProbeResult {
            public bool executed;
            public double reloadWallMilliseconds;
            public DifferenceResult position;
            public DifferenceResult attitudeDegrees;
            public DifferenceResult linearVelocity;
            public DifferenceResult angularVelocity;
            public DifferenceResult waterHeight;
        }

        [Serializable] private sealed class BenchmarkResult {
            public string schema = "crane-benchmark-v1";
            public string scenario;
            public string runtimeProfile;
            public string scene;
            public string unityVersion;
            public string platform;
            public string cpu;
            public string gpu;
            public int workerId;
            public int randomSeed;
            public bool imageSignatures;
            public int logicalCpuCount;
            public double warmupSeconds;
            public double wallSeconds;
            public double simulatedSeconds;
            public double realTimeFactor;
            public float timeScale;
            public double processCpuUtilizationPercent;
            public double meanGpuFrameMilliseconds;
            public long gcAllocatedBytes;
            public long monoUsedBytes;
            public long totalAllocatedMemoryBytes;
            public long episodeId;
            public long simulationTick;
            public long observationTick;
            public long actionSourceTick;
            public long actionReceiveTick;
            public long actionApplicationTick;
            public long actionSequence;
            public long observationQueueAgeTicks;
            public long actionQueueAgeTicks;
            public long actionAgeTicks;
            public long staleObservations;
            public long staleActions;
            public long acceptedActions;
            public long rejectedActions;
            public long unknownSourceActions;
            public long crossEpisodeActions;
            public long duplicateActions;
            public long failedObservations;
            public long depthBufferValidationBytes;
            public long depthBufferValidationMismatches;
            public int loggedErrors;
            public int loggedExceptions;
            public int enabledCameras;
            public int enabledSensorCameras;
            public int enabledSpectatorCameras;
            public int enabledWaterDriverCameras;
            public int screenWidth;
            public int screenHeight;
            public string validationStream;
            public long validationSamplesCaptured;
            public int validationSamplesRetained;
            public long validationSamplesDropped;
            public long invalidWaterSearches;
            public long maximumObservationQueueAgeTicks;
            public ActionTimingResult actionTiming;
            public SitlResult sitl;
            public bool valid;
            public LidarResult lidar;
            public ImageResult rgbCamera;
            public ImageResult depthCamera;
            public DetectionResult detections;
            public ResetProbeResult resetProbe;
            public List<MarkerResult> markers = new();
            public List<ValidationSample> validation = new();
        }

        private sealed class MarkerRecorder : IDisposable {
            public readonly string Name;
            private readonly ProfilerRecorder recorder;
            private double totalNanoseconds;
            private long maxNanoseconds;
            private long frames;

            public MarkerRecorder(string name, ProfilerCategory category) {
                Name = name;
                recorder = ProfilerRecorder.StartNew(category, name, 1);
            }

            public void Accumulate() {
                if (!recorder.Valid) return;
                long value = recorder.LastValue;
                totalNanoseconds += value;
                if (value > maxNanoseconds) maxNanoseconds = value;
                frames++;
            }

            public MarkerResult ToResult() => new() {
                name = Name,
                totalMilliseconds = totalNanoseconds / 1_000_000.0,
                meanMillisecondsPerFrame = frames == 0 ? 0 : totalNanoseconds / frames / 1_000_000.0,
                maxMillisecondsPerFrame = maxNanoseconds / 1_000_000.0,
                sampledFrames = frames
            };

            public void Dispose() => recorder.Dispose();
        }

        private static readonly string[] s_CustomMarkers = {
            "CRANE.Simulation.Step", "CRANE.Water.Prepare", "CRANE.Water.Query",
            "CRANE.Water.Buoyancy", "CRANE.Vehicle.Dynamics",
            "CRANE.Vehicle.Dynamics.General", "CRANE.Vehicle.Dynamics.Current",
            "CRANE.Vehicle.Dynamics.Fossen",
            "CRANE.Vehicle.Dynamics.AreaPreparation",
            "CRANE.Vehicle.Dynamics.ResistanceCoefficient",
            "CRANE.Vehicle.Dynamics.ViscousResistance",
            "CRANE.Vehicle.Dynamics.PressureDrag", "CRANE.Vehicle.Thruster",
            "CRANE.Sensor.Lidar", "CRANE.Sensor.Lidar.Raycast", "CRANE.Sensor.Lidar.Process",
            "CRANE.Sensor.Lidar.Pack", "CRANE.Sensor.Other", "CRANE.Sensor.Depth.Readback",
            "CRANE.Sensor.Depth.Geometric",
            "CRANE.Sensor.Depth.Copy", "CRANE.Sensor.Depth.RowFlip", "CRANE.Sensor.Depth.Publish",
            "CRANE.Sensor.RGB.Readback", "CRANE.Sensor.Detection", "CRANE.ROS.CreateMessage",
            "CRANE.ROS.Publish", "CRANE.Episode.Reset"
        };

        private readonly List<MarkerRecorder> recorders = new();
        private readonly List<ValidationSample> validation = new();
        private readonly Stopwatch wallClock = new();
        private ProfilerRecorder gcAllocRecorder;
        private Process process;
        private TimeSpan processCpuStart;
        private double simulatedStart;
        private double warmupSeconds = 5;
        private double durationSeconds = 30;
        private double validationPeriodSeconds = 1;
        private int validationCapacity = 256;
        private float requestedTimeScale = 1;
        private double nextValidationTime;
        private string outputPath;
        private string validationOutputPath;
        private string scenario = "unspecified";
        private string requestedScene;
        private CraneRuntimeOptions runtimeOptions;
        private string fixture;
        private bool useClassifiedContactLayers = true;
        private bool measuring;
        private int workerId;
        private int randomSeed = 1;
        private bool imageSignatures = true;
        private bool runResetProbe;
        private ResetProbeResult resetProbeResult = new();
        private double gpuMillisecondsTotal;
        private long gpuSampleCount;
        private long gcAllocatedBytes;
        private int loggedErrors;
        private int loggedExceptions;
        private long validationSamplesCaptured;
        private long validationSamplesDropped;
        private long invalidWaterSearches;
        private long maximumObservationQueueAgeTicks = -1;
        private StreamWriter validationWriter;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            if (!HasFlag("--crane-benchmark")) return;
            var host = new GameObject("CRANE Benchmark Runner");
            DontDestroyOnLoad(host);
            host.AddComponent<CraneBenchmarkRunner>();
        }

        private void Awake() {
            ParseArguments();
            Application.runInBackground = true;
            Time.timeScale = 1;
            CraneRuntimeMetrics.BeginEpisode();
            CraneRuntimeMetrics.CaptureImageSignatures = imageSignatures;
            process = Process.GetCurrentProcess();
            Application.logMessageReceived += OnLogMessage;
            SceneManager.sceneLoaded += OnSceneLoaded;
        }

        private IEnumerator Start() {
            if (!string.IsNullOrEmpty(requestedScene) &&
                !SceneManager.GetActiveScene().name.Equals(requestedScene, StringComparison.OrdinalIgnoreCase)) {
                AsyncOperation load = SceneManager.LoadSceneAsync(requestedScene, LoadSceneMode.Single);
                while (load != null && !load.isDone) yield return null;
            }
            ApplyRuntimeOptions();
            if (fixture == "contact-heavy") CreateContactFixture(useClassifiedContactLayers);
            yield return new WaitForSecondsRealtime((float)warmupSeconds);
            if (runResetProbe) yield return RunResetProbe();
            Time.timeScale = requestedTimeScale;
            BeginMeasurement();
        }

        private void Update() {
            if (!measuring) return;
            foreach (MarkerRecorder recorder in recorders) recorder.Accumulate();
            if (gcAllocRecorder.Valid) gcAllocatedBytes += gcAllocRecorder.LastValue;
            CaptureGpuTiming();

            double simulatedElapsed = Time.timeAsDouble - simulatedStart;
            if (simulatedElapsed >= nextValidationTime) {
                RecordValidation(CaptureValidation(simulatedElapsed));
                nextValidationTime += validationPeriodSeconds;
            }
            if (wallClock.Elapsed.TotalSeconds >= durationSeconds) Finish();
        }

        private void BeginMeasurement() {
            // Keep warmup readbacks outside the measured episode. Production resets use the
            // generation check; the benchmark can drain once before its stopwatch starts.
            AsyncGPUReadback.WaitAllRequests();
            CraneRuntimeMetrics.BeginEpisode();
            foreach (string marker in s_CustomMarkers)
                recorders.Add(new MarkerRecorder(marker, ProfilerCategory.Scripts));
            recorders.Add(new MarkerRecorder("Physics.Simulate", ProfilerCategory.Physics));
            gcAllocRecorder = ProfilerRecorder.StartNew(ProfilerCategory.Memory, "GC.Alloc", 1);

            FrameTimingManager.CaptureFrameTimings();
            OpenValidationStream();
            simulatedStart = Time.timeAsDouble;
            nextValidationTime = 0;
            processCpuStart = process.TotalProcessorTime;
            wallClock.Restart();
            measuring = true;
        }

        private void CaptureGpuTiming() {
            FrameTimingManager.CaptureFrameTimings();
            var timings = new FrameTiming[1];
            if (FrameTimingManager.GetLatestTimings(1, timings) == 0) return;
            if (timings[0].gpuFrameTime <= 0) return;
            gpuMillisecondsTotal += timings[0].gpuFrameTime;
            gpuSampleCount++;
        }

        private ValidationSample CaptureValidation(double simulatedElapsed) {
            long observationTick = CraneRuntimeMetrics.ObservationTick;
            var sample = new ValidationSample {
                simulatedSeconds = simulatedElapsed,
                monoUsedBytes = Profiler.GetMonoUsedSizeLong(),
                totalAllocatedMemoryBytes = Profiler.GetTotalAllocatedMemoryLong(),
                observationQueueAgeTicks = observationTick < 0 ? -1 :
                    CraneRuntimeMetrics.SimulationTick - observationTick,
                staleObservations = CraneRuntimeMetrics.StaleObservations,
                failedObservations = CraneRuntimeMetrics.FailedObservations
            };
            foreach (Rigidbody body in FindObjectsByType<Rigidbody>()
                         .OrderBy(body => body.name).Take(64)) {
                sample.bodies.Add(new BodySample {
                    name = body.name,
                    position = body.position,
                    rotation = body.rotation,
                    linearVelocity = body.linearVelocity,
                    angularVelocity = body.angularVelocity
                });
            }
            foreach (ArticulationBody body in FindObjectsByType<ArticulationBody>()
                         .OrderBy(body => body.name).Take(Math.Max(0, 64 - sample.bodies.Count))) {
                sample.bodies.Add(new BodySample {
                    name = body.name,
                    position = body.transform.position,
                    rotation = body.transform.rotation,
                    linearVelocity = body.linearVelocity,
                    angularVelocity = body.angularVelocity
                });
            }

            foreach (WaterSurface surface in FindObjectsByType<WaterSurface>().Take(8)) {
                Vector3 target = sample.bodies.Count > 0 ? sample.bodies[0].position : surface.transform.position;
                var parameters = new WaterSearchParameters {
                    startPositionWS = target,
                    targetPositionWS = target,
                    error = 0.01f,
                    maxIterations = 8,
                    includeDeformation = true,
                    excludeSimulation = false
                };
                bool valid = surface.ProjectPointOnWaterSurface(parameters, out WaterSearchResult result);
                sample.water.Add(new WaterSample {
                    surface = surface.name,
                    simulationTime = surface.simulationTime,
                    timeMultiplier = surface.timeMultiplier,
                    target = target,
                    projected = result.projectedPositionWS,
                    error = result.error,
                    steps = result.numIterations,
                    valid = valid
                });
            }
            return sample;
        }

        private void RecordValidation(ValidationSample sample) {
            validationSamplesCaptured++;
            invalidWaterSearches += sample.water.Count(water => !water.valid);
            maximumObservationQueueAgeTicks = Math.Max(maximumObservationQueueAgeTicks,
                sample.observationQueueAgeTicks);
            validationWriter?.WriteLine(JsonUtility.ToJson(sample));

            if (validationCapacity <= 0) {
                validationSamplesDropped++;
                return;
            }
            if (validation.Count == validationCapacity) {
                validation.RemoveAt(0);
                validationSamplesDropped++;
            }
            validation.Add(sample);
        }

        private void OpenValidationStream() {
            string directory = Path.GetDirectoryName(validationOutputPath);
            if (!string.IsNullOrEmpty(directory)) Directory.CreateDirectory(directory);
            validationWriter = new StreamWriter(validationOutputPath, false) { AutoFlush = true };
        }

        private void Finish() {
            measuring = false;
            wallClock.Stop();
            validationWriter?.Dispose();
            validationWriter = null;
            TimeSpan cpuElapsed = process.TotalProcessorTime - processCpuStart;
            double wallSeconds = wallClock.Elapsed.TotalSeconds;
            double simulatedSeconds = Time.timeAsDouble - simulatedStart;
            long simulationTick = CraneRuntimeMetrics.SimulationTick;
            long observationTick = CraneRuntimeMetrics.ObservationTick;
            long actionTick = CraneRuntimeMetrics.ActionApplicationTick;
            CraneRuntimeMetrics.LidarSnapshot lidar = CraneRuntimeMetrics.GetLidarSnapshot();
            CraneRuntimeMetrics.ImageSnapshot rgb = CraneRuntimeMetrics.GetImageSnapshot(false);
            CraneRuntimeMetrics.ImageSnapshot depth = CraneRuntimeMetrics.GetImageSnapshot(true);
            CraneRuntimeMetrics.DetectionSnapshot detections = CraneRuntimeMetrics.GetDetectionSnapshot();
            CraneRuntimeMetrics.ActionTimingSnapshot actionTiming =
                CraneRuntimeMetrics.GetActionTimingSnapshot();
            CraneRuntimeMetrics.SitlSnapshot sitl = CraneRuntimeMetrics.GetSitlSnapshot();
            CraneRuntimeOptions.CameraStatus cameraStatus = runtimeOptions.GetCameraStatus();
            var result = new BenchmarkResult {
                scenario = scenario,
                runtimeProfile = runtimeOptions.ProfileName,
                scene = SceneManager.GetActiveScene().path,
                unityVersion = Application.unityVersion,
                platform = Application.platform.ToString(),
                cpu = SystemInfo.processorType,
                gpu = SystemInfo.graphicsDeviceName,
                workerId = workerId,
                randomSeed = randomSeed,
                imageSignatures = imageSignatures,
                logicalCpuCount = SystemInfo.processorCount,
                warmupSeconds = warmupSeconds,
                wallSeconds = wallSeconds,
                simulatedSeconds = simulatedSeconds,
                realTimeFactor = wallSeconds > 0 ? simulatedSeconds / wallSeconds : 0,
                timeScale = Time.timeScale,
                processCpuUtilizationPercent = wallSeconds > 0 ?
                    cpuElapsed.TotalSeconds / wallSeconds / Math.Max(1, SystemInfo.processorCount) * 100.0 : 0,
                meanGpuFrameMilliseconds = gpuSampleCount == 0 ? 0 : gpuMillisecondsTotal / gpuSampleCount,
                gcAllocatedBytes = gcAllocatedBytes,
                monoUsedBytes = Profiler.GetMonoUsedSizeLong(),
                totalAllocatedMemoryBytes = Profiler.GetTotalAllocatedMemoryLong(),
                episodeId = CraneRuntimeMetrics.EpisodeId,
                simulationTick = simulationTick,
                observationTick = observationTick,
                actionSourceTick = CraneRuntimeMetrics.ActionSourceTick,
                actionReceiveTick = CraneRuntimeMetrics.ActionReceiveTick,
                actionApplicationTick = actionTick,
                actionSequence = CraneRuntimeMetrics.ActionSequence,
                observationQueueAgeTicks = observationTick < 0 ? -1 : simulationTick - observationTick,
                actionQueueAgeTicks = actionTick < 0 || CraneRuntimeMetrics.ActionReceiveTick < 0 ? -1 :
                    actionTick - CraneRuntimeMetrics.ActionReceiveTick,
                actionAgeTicks = actionTick < 0 ? -1 : simulationTick - actionTick,
                staleObservations = CraneRuntimeMetrics.StaleObservations,
                staleActions = CraneRuntimeMetrics.StaleActions,
                acceptedActions = CraneRuntimeMetrics.AcceptedActions,
                rejectedActions = CraneRuntimeMetrics.RejectedActions,
                unknownSourceActions = CraneRuntimeMetrics.UnknownSourceActions,
                crossEpisodeActions = CraneRuntimeMetrics.CrossEpisodeActions,
                duplicateActions = CraneRuntimeMetrics.DuplicateActions,
                failedObservations = CraneRuntimeMetrics.FailedObservations,
                depthBufferValidationBytes = CraneRuntimeMetrics.DepthBufferValidationBytes,
                depthBufferValidationMismatches = CraneRuntimeMetrics.DepthBufferValidationMismatches,
                loggedErrors = loggedErrors,
                loggedExceptions = loggedExceptions,
                enabledCameras = cameraStatus.Enabled,
                enabledSensorCameras = cameraStatus.Sensor,
                enabledSpectatorCameras = cameraStatus.Spectator,
                enabledWaterDriverCameras = cameraStatus.WaterDriver,
                screenWidth = Screen.width,
                screenHeight = Screen.height,
                validationStream = validationOutputPath,
                validationSamplesCaptured = validationSamplesCaptured,
                validationSamplesRetained = validation.Count,
                validationSamplesDropped = validationSamplesDropped,
                invalidWaterSearches = invalidWaterSearches,
                maximumObservationQueueAgeTicks = Math.Max(maximumObservationQueueAgeTicks,
                    observationTick < 0 ? -1 : simulationTick - observationTick),
                actionTiming = new ActionTimingResult {
                    acceptedActions = actionTiming.AcceptedActions,
                    knownSourceActions = actionTiming.KnownSourceActions,
                    meanSourceToApplicationTicks = actionTiming.KnownSourceActions == 0 ? -1 :
                        (double)actionTiming.SourceToApplicationTicks /
                        actionTiming.KnownSourceActions,
                    maximumSourceToApplicationTicks =
                        actionTiming.MaximumSourceToApplicationTicks,
                    meanReceiveToApplicationTicks = actionTiming.AcceptedActions == 0 ? -1 :
                        (double)actionTiming.ReceiveToApplicationTicks /
                        actionTiming.AcceptedActions,
                    maximumReceiveToApplicationTicks =
                        actionTiming.MaximumReceiveToApplicationTicks,
                    maximumInterApplicationTicks = actionTiming.MaximumInterApplicationTicks,
                    commandTimeouts = actionTiming.CommandTimeouts
                },
                sitl = new SitlResult {
                    validServoPackets = sitl.ValidServoPackets,
                    invalidServoPackets = sitl.InvalidServoPackets,
                    telemetryPackets = sitl.TelemetryPackets,
                    lastFrame = sitl.LastFrame,
                    lastTelemetrySimulatedSeconds =
                        sitl.LastTelemetryTimestampMicroseconds < 0 ? -1 :
                        sitl.LastTelemetryTimestampMicroseconds / 1_000_000.0
                },
                lidar = new LidarResult {
                    scanCount = lidar.ScanCount,
                    configuredPointsPerScan = lidar.ConfiguredPointsPerScan,
                    batchSize = lidar.BatchSize,
                    totalPoints = lidar.TotalPoints,
                    hitCount = lidar.HitCount,
                    missCount = lidar.MissCount,
                    minimumRange = lidar.MinimumRange,
                    maximumRange = lidar.MaximumRange,
                    meanRange = lidar.MeanRange,
                    checksum = lidar.Checksum.ToString("X16"),
                    acquisitionTick = lidar.AcquisitionTick,
                    commandValidationCount = CraneRuntimeMetrics.LidarCommandValidationCount,
                    commandValidationMismatches = CraneRuntimeMetrics.LidarCommandValidationMismatches,
                    packValidationBytes = CraneRuntimeMetrics.LidarPackValidationBytes,
                    packValidationMismatches = CraneRuntimeMetrics.LidarPackValidationMismatches,
                    processValidationCount = CraneRuntimeMetrics.LidarProcessValidationCount,
                    processValidationMismatches = CraneRuntimeMetrics.LidarProcessValidationMismatches
                },
                rgbCamera = ToImageResult(rgb),
                depthCamera = ToImageResult(depth),
                detections = new DetectionResult {
                    acquisitionCount = detections.AcquisitionCount,
                    detectionCount = detections.DetectionCount,
                    acquisitionTick = detections.AcquisitionTick
                },
                resetProbe = resetProbeResult,
                valid = loggedErrors == 0 && loggedExceptions == 0 &&
                    CraneRuntimeMetrics.FailedObservations == 0 &&
                    CraneRuntimeMetrics.StaleObservations == 0 &&
                    CraneRuntimeMetrics.StaleActions == 0 &&
                    CraneRuntimeMetrics.CrossEpisodeActions == 0 &&
                    CraneRuntimeMetrics.DepthBufferValidationMismatches == 0 &&
                    invalidWaterSearches == 0,
                validation = validation
            };
            foreach (MarkerRecorder recorder in recorders) {
                result.markers.Add(recorder.ToResult());
                recorder.Dispose();
            }
            recorders.Clear();
            gcAllocRecorder.Dispose();

            string directory = Path.GetDirectoryName(outputPath);
            if (!string.IsNullOrEmpty(directory)) Directory.CreateDirectory(directory);
            File.WriteAllText(outputPath, JsonUtility.ToJson(result, true));
            Debug.Log($"CRANE_BENCHMARK_COMPLETE {outputPath} RTF={result.realTimeFactor:F3}");
            Application.logMessageReceived -= OnLogMessage;
            SceneManager.sceneLoaded -= OnSceneLoaded;
            Application.Quit(0);
        }

        private static ImageResult ToImageResult(CraneRuntimeMetrics.ImageSnapshot image) => new() {
            acquisitionCount = image.AcquisitionCount,
            width = image.Width,
            height = image.Height,
            totalBytes = image.TotalBytes,
            checksum = image.Checksum.ToString("X16"),
            acquisitionTick = image.AcquisitionTick
        };

        private void ParseArguments() {
            string[] args = Environment.GetCommandLineArgs();
            warmupSeconds = ReadDouble(args, "--crane-warmup", warmupSeconds);
            durationSeconds = ReadDouble(args, "--crane-duration", durationSeconds);
            validationPeriodSeconds = ReadDouble(args, "--crane-validation-period", validationPeriodSeconds);
            validationCapacity = Math.Max(0,
                (int)ReadDouble(args, "--crane-validation-capacity", validationCapacity));
            requestedTimeScale = (float)ReadDouble(args, "--crane-time-scale", requestedTimeScale);
            if (requestedTimeScale <= 0) requestedTimeScale = 1;
            scenario = ReadString(args, "--crane-scenario", scenario);
            requestedScene = ReadString(args, "--crane-scene", null);
            runtimeOptions = CraneRuntimeOptions.Parse(args);
            fixture = ReadString(args, "--crane-fixture", null);
            useClassifiedContactLayers = !HasFlag("--crane-contact-fixture-legacy-layers");
            workerId = (int)ReadDouble(args, "--crane-worker-id", 0);
            randomSeed = (int)ReadDouble(args, "--crane-seed", randomSeed);
            imageSignatures = !HasFlag("--crane-no-signatures");
            runResetProbe = HasFlag("--crane-reset-probe");
            outputPath = ReadString(args, "--crane-output",
                Path.Combine(Application.persistentDataPath, $"crane-benchmark-{DateTime.UtcNow:yyyyMMdd-HHmmss}.json"));
            validationOutputPath = ReadString(args, "--crane-validation-output",
                Path.ChangeExtension(outputPath, ".validation.jsonl"));
        }

        private void OnDestroy() {
            validationWriter?.Dispose();
            validationWriter = null;
        }

        private static bool HasFlag(string flag) => Environment.GetCommandLineArgs().Any(arg => arg == flag);
        private void OnLogMessage(string condition, string stackTrace, LogType type) {
            if (type == LogType.Exception) loggedExceptions++;
            else if (type == LogType.Error || type == LogType.Assert) loggedErrors++;
        }
        private void OnSceneLoaded(Scene scene, LoadSceneMode mode) {
            ApplyRuntimeOptions();
        }
        private void ApplyRuntimeOptions() {
            runtimeOptions.Apply();
        }
        private static void CreateContactFixture(bool classifiedLayers) {
            var root = new GameObject("CRANE Contact Fixture");
            var ground = GameObject.CreatePrimitive(PrimitiveType.Cube);
            ground.name = "contact-ground";
            if (classifiedLayers) ground.layer = CraneCollisionLayers.Environment;
            ground.transform.SetParent(root.transform);
            ground.transform.position = new Vector3(100, 4, 0);
            ground.transform.localScale = new Vector3(30, 1, 30);
            ground.GetComponent<Renderer>().enabled = false;

            const int columns = 12;
            const int rows = 12;
            for (int index = 0; index < columns * rows; index++) {
                int x = index % columns;
                int layer = index / columns;
                var bodyObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
                bodyObject.name = $"contact-body-{index:D3}";
                if (classifiedLayers) bodyObject.layer = CraneCollisionLayers.DynamicObstacle;
                bodyObject.transform.SetParent(root.transform);
                bodyObject.transform.position = new Vector3(94.5f + x, 4.8f + layer * 0.52f,
                    (x % 2) * 0.04f);
                bodyObject.transform.localScale = new Vector3(0.9f, 0.5f, 0.9f);
                bodyObject.GetComponent<Renderer>().enabled = false;
                var body = bodyObject.AddComponent<Rigidbody>();
                body.mass = 2;
                body.collisionDetectionMode = CollisionDetectionMode.Discrete;
                body.sleepThreshold = 0;
            }
        }

        private IEnumerator RunResetProbe() {
            ValidationSample firstA = CaptureValidation(0);
            yield return new WaitForSecondsRealtime(1);

            AsyncGPUReadback.WaitAllRequests();
            CraneRuntimeMetrics.BeginEpisode();
            string sceneName = SceneManager.GetActiveScene().name;
            var reloadClock = Stopwatch.StartNew();
            AsyncOperation reload = SceneManager.LoadSceneAsync(sceneName, LoadSceneMode.Single);
            while (reload != null && !reload.isDone) yield return null;
            reloadClock.Stop();

            ApplyRuntimeOptions();
            if (fixture == "contact-heavy") CreateContactFixture(useClassifiedContactLayers);
            yield return new WaitForSecondsRealtime((float)warmupSeconds);
            ValidationSample secondA = CaptureValidation(0);
            resetProbeResult = CompareResetSamples(firstA, secondA, reloadClock.Elapsed.TotalMilliseconds);
        }

        private static ResetProbeResult CompareResetSamples(ValidationSample first,
            ValidationSample second, double reloadMilliseconds) {
            var position = new List<double>();
            var attitude = new List<double>();
            var linearVelocity = new List<double>();
            var angularVelocity = new List<double>();
            var secondBodies = second.bodies.ToDictionary(body => body.name);
            foreach (BodySample body in first.bodies) {
                if (!secondBodies.TryGetValue(body.name, out BodySample other)) continue;
                position.Add(Vector3.Distance(body.position, other.position));
                attitude.Add(Quaternion.Angle(body.rotation, other.rotation));
                linearVelocity.Add(Vector3.Distance(body.linearVelocity, other.linearVelocity));
                angularVelocity.Add(Vector3.Distance(body.angularVelocity, other.angularVelocity));
            }
            var water = new List<double>();
            var secondWater = second.water.ToDictionary(sample => sample.surface);
            foreach (WaterSample sample in first.water)
                if (secondWater.TryGetValue(sample.surface, out WaterSample other))
                    water.Add(Math.Abs(sample.projected.y - other.projected.y));
            return new ResetProbeResult {
                executed = true,
                reloadWallMilliseconds = reloadMilliseconds,
                position = SummarizeDifferences(position),
                attitudeDegrees = SummarizeDifferences(attitude),
                linearVelocity = SummarizeDifferences(linearVelocity),
                angularVelocity = SummarizeDifferences(angularVelocity),
                waterHeight = SummarizeDifferences(water)
            };
        }

        private static DifferenceResult SummarizeDifferences(List<double> values) => new() {
            count = values.Count,
            rms = values.Count == 0 ? 0 : Math.Sqrt(values.Sum(value => value * value) / values.Count),
            max = values.Count == 0 ? 0 : values.Max()
        };
        private static string ReadString(string[] args, string key, string fallback) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length ? args[index + 1] : fallback;
        }
        private static double ReadDouble(string[] args, string key, double fallback) =>
            double.TryParse(ReadString(args, key, fallback.ToString(System.Globalization.CultureInfo.InvariantCulture)),
                System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out double value)
                ? value : fallback;
    }
}
