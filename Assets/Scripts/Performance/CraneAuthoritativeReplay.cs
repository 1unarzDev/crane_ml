using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using Sim.Utils.Performance;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;
using UnityEngine.SceneManagement;

namespace Sim.Performance {
    internal static class CraneReplayFormat {
        public const int StreamMagic = 0x4352414E; // CRAN
        public const int IndexMagic = 0x43524958;  // CRIX
        public const int FrameMagic = 0x4652414D;  // FRAM
        public const int MinimumReadableVersion = 1;
        public const int Version = 5;
        public const int IndexEntryBytes = sizeof(long) * 3 + sizeof(byte);

        [Serializable] private sealed class RuntimeMetadata {
            public string schema = "crane-replay-runtime-v1";
            public string platform;
            public string operatingSystem;
            public string processor;
            public string graphicsDevice;
            public string graphicsDeviceType;
            public string graphicsDeviceVersion;
            public string physicsBackend = "Unity PhysX via Rigidbody/ArticulationBody";
        }

        public static string ReadArgument(string[] args, string key) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length ? args[index + 1] : null;
        }

        public static string StablePath(Component component) {
            Transform current = component.transform;
            var segments = new Stack<string>();
            while (current != null) {
                segments.Push($"{current.GetSiblingIndex()}:{current.name}");
                current = current.parent;
            }
            return $"{component.gameObject.scene.path}|{string.Join("/", segments)}|{component.GetType().FullName}";
        }

        public static string ConfigurationHash() {
            string commandLine = string.Join("\n", Environment.GetCommandLineArgs());
            using SHA256 sha = SHA256.Create();
            return BitConverter.ToString(sha.ComputeHash(Encoding.UTF8.GetBytes(commandLine)))
                .Replace("-", string.Empty);
        }

        public static string BuildManifestJson() {
            string directory = Path.GetDirectoryName(Application.dataPath);
            string path = Path.Combine(directory ?? string.Empty, "crane-build-manifest.json");
            if (!File.Exists(path)) return string.Empty;
            var info = new FileInfo(path);
            if (info.Length > 64 * 1024 * 1024)
                throw new InvalidDataException($"CRANE build manifest is too large: {info.Length} bytes.");
            return File.ReadAllText(path);
        }

        public static string HashText(string value) {
            if (string.IsNullOrEmpty(value)) return string.Empty;
            using SHA256 sha = SHA256.Create();
            return BitConverter.ToString(sha.ComputeHash(Encoding.UTF8.GetBytes(value)))
                .Replace("-", string.Empty);
        }

        public static string RuntimeMetadataJson() => JsonUtility.ToJson(new RuntimeMetadata {
            platform = Application.platform.ToString(),
            operatingSystem = SystemInfo.operatingSystem,
            processor = SystemInfo.processorType,
            graphicsDevice = SystemInfo.graphicsDeviceName,
            graphicsDeviceType = SystemInfo.graphicsDeviceType.ToString(),
            graphicsDeviceVersion = SystemInfo.graphicsDeviceVersion
        });

        public static void WriteVector(BinaryWriter writer, Vector3 value) {
            writer.Write(value.x); writer.Write(value.y); writer.Write(value.z);
        }

        public static Vector3 ReadVector(BinaryReader reader) =>
            new(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());

        public static void WriteRotation(BinaryWriter writer, Quaternion value) {
            writer.Write(value.x); writer.Write(value.y); writer.Write(value.z); writer.Write(value.w);
        }

        public static Quaternion ReadRotation(BinaryReader reader) =>
            new(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
    }

    public readonly struct CraneReplayAction {
        public readonly string Source;
        public readonly long EpisodeId;
        public readonly long Sequence;
        public readonly long SourceObservationTick;
        public readonly long ReceiveTick;
        public readonly long ApplicationTick;
        public readonly string PayloadEncoding;
        public readonly byte[] Payload;

        internal CraneReplayAction(string source, long episodeId, long sequence,
            long sourceObservationTick, long receiveTick, long applicationTick,
            string payloadEncoding, byte[] payload) {
            Source = source;
            EpisodeId = episodeId;
            Sequence = sequence;
            SourceObservationTick = sourceObservationTick;
            ReceiveTick = receiveTick;
            ApplicationTick = applicationTick;
            PayloadEncoding = payloadEncoding;
            Payload = payload;
        }
    }

    public readonly struct CraneReplayOutcome {
        public readonly string Source;
        public readonly long EpisodeId;
        public readonly long Sequence;
        public readonly long Tick;
        public readonly double Reward;
        public readonly double CumulativeReward;
        public readonly bool Terminated;
        public readonly bool Truncated;
        public readonly string Reason;

        internal CraneReplayOutcome(string source, long episodeId, long sequence, long tick,
            double reward, double cumulativeReward, bool terminated, bool truncated, string reason) {
            Source = source;
            EpisodeId = episodeId;
            Sequence = sequence;
            Tick = tick;
            Reward = reward;
            CumulativeReward = cumulativeReward;
            Terminated = terminated;
            Truncated = truncated;
            Reason = reason;
        }
    }

    public readonly struct CraneReplayObservation {
        public readonly string Source;
        public readonly string Topic;
        public readonly string FrameId;
        public readonly string MessageType;
        public readonly string Encoding;
        public readonly string PayloadPolicy;
        public readonly long EpisodeId;
        public readonly long Sequence;
        public readonly long AcquisitionTick;
        public readonly double AcquisitionTime;
        public readonly long CompletionTick;
        public readonly double CompletionTime;
        public readonly int Width;
        public readonly int Height;
        public readonly int RowStep;
        public readonly int ElementCount;
        public readonly long PayloadBytes;

        internal CraneReplayObservation(in CraneObservationMetadata metadata) {
            Source = metadata.Source;
            Topic = metadata.Topic;
            FrameId = metadata.FrameId;
            MessageType = metadata.MessageType;
            Encoding = metadata.Encoding;
            PayloadPolicy = metadata.PayloadPolicy;
            EpisodeId = metadata.EpisodeId;
            Sequence = metadata.Sequence;
            AcquisitionTick = metadata.AcquisitionTick;
            AcquisitionTime = metadata.AcquisitionTime;
            CompletionTick = metadata.CompletionTick;
            CompletionTime = metadata.CompletionTime;
            Width = metadata.Width;
            Height = metadata.Height;
            RowStep = metadata.RowStep;
            ElementCount = metadata.ElementCount;
            PayloadBytes = metadata.PayloadBytes;
        }
    }

    /// <summary>
    /// Streams authoritative post-physics state. Memory use is bounded by the writer buffer and
    /// one tick's body list; the fixed-size sidecar index permits binary-search seeking.
    /// </summary>
    [DefaultExecutionOrder(32000)]
    internal sealed class CraneEpisodeRecorder : MonoBehaviour {
        private BinaryWriter stream;
        private BinaryWriter index;
        private string outputPath;
        private string requestedScene;
        private int flushEveryTicks = 50;
        private int ticksSinceFlush;
        private long localTick;
        private long previousEpisode = -1;
        private bool forceDiscontinuity;
        private HashSet<string> previousObjects = new();
        private HashSet<string> currentObjects = new();
        private readonly List<string> spawned = new();
        private readonly List<string> despawned = new();
        private readonly Dictionary<EntityId, string> stablePaths = new();
        private readonly object acceptedActionsLock = new();
        private readonly List<CraneAcceptedAction> acceptedActions = new();
        private readonly object outcomesLock = new();
        private readonly List<CraneTaskOutcomeEvent> outcomes = new();
        private readonly object observationsLock = new();
        private readonly List<CraneObservationMetadata> observations = new();
        private Coroutine captureLoop;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            string path = CraneReplayFormat.ReadArgument(Environment.GetCommandLineArgs(), "--crane-record");
            if (string.IsNullOrWhiteSpace(path)) return;
            var host = new GameObject("CRANE Episode Recorder");
            DontDestroyOnLoad(host);
            host.AddComponent<CraneEpisodeRecorder>();
        }

        private void Awake() {
            string[] args = Environment.GetCommandLineArgs();
            outputPath = Path.GetFullPath(CraneReplayFormat.ReadArgument(args, "--crane-record"));
            requestedScene = CraneReplayFormat.ReadArgument(args, "--crane-scene");
            if (int.TryParse(CraneReplayFormat.ReadArgument(args, "--crane-record-flush-ticks"), out int value))
                flushEveryTicks = Math.Max(1, value);
            SceneManager.sceneLoaded += OnSceneLoaded;
        }

        private void OnSceneLoaded(Scene scene, LoadSceneMode mode) {
            if (!string.IsNullOrEmpty(requestedScene) &&
                !scene.name.Equals(requestedScene, StringComparison.OrdinalIgnoreCase)) return;
            if (stream != null && localTick > 0) {
                forceDiscontinuity = true;
                stablePaths.Clear();
                return;
            }
            CloseStreams();
            string directory = Path.GetDirectoryName(outputPath);
            if (!string.IsNullOrEmpty(directory)) Directory.CreateDirectory(directory);
            stream = new BinaryWriter(new BufferedStream(File.Create(outputPath), 64 * 1024));
            index = new BinaryWriter(new BufferedStream(File.Create(outputPath + ".index"), 16 * 1024));
            CraneActionGate.SubscribeAcceptedActions(OnAcceptedAction);
            CraneTaskOutcome.Subscribe(OnTaskOutcome);
            CraneObservationJournal.Subscribe(OnObservation);
            WriteHeader(scene);
            previousObjects.Clear();
            currentObjects.Clear();
            stablePaths.Clear();
            previousEpisode = -1;
            localTick = 0;
            if (captureLoop != null) StopCoroutine(captureLoop);
            captureLoop = StartCoroutine(CaptureAfterPhysics());
        }

        private void WriteHeader(Scene scene) {
            string buildManifest = CraneReplayFormat.BuildManifestJson();
            stream.Write(CraneReplayFormat.StreamMagic);
            stream.Write(CraneReplayFormat.Version);
            stream.Write(Application.unityVersion);
            stream.Write(Application.buildGUID ?? string.Empty);
            stream.Write(scene.path ?? string.Empty);
            stream.Write(CraneReplayFormat.ConfigurationHash());
            stream.Write(Time.fixedDeltaTime);
            stream.Write(DateTime.UtcNow.Ticks);
            stream.Write(CraneReplayFormat.HashText(buildManifest));
            stream.Write(buildManifest);
            stream.Write(CraneReplayFormat.RuntimeMetadataJson());
            index.Write(CraneReplayFormat.IndexMagic);
            index.Write(CraneReplayFormat.Version);
        }

        private IEnumerator CaptureAfterPhysics() {
            while (true) {
                yield return new WaitForFixedUpdate();
                if (stream != null) CaptureFrame();
            }
        }

        private void CaptureFrame() {
            long offset = stream.BaseStream.Position;
            long episode = CraneRuntimeMetrics.EpisodeId;
            long metricTick = CraneRuntimeMetrics.SimulationTick;
            long tick = ++localTick;
            bool discontinuity = forceDiscontinuity ||
                (previousEpisode >= 0 && previousEpisode != episode);
            forceDiscontinuity = false;
            previousEpisode = episode;

            Rigidbody[] rigidbodies = FindObjectsByType<Rigidbody>(FindObjectsInactive.Exclude);
            ArticulationBody[] articulations = FindObjectsByType<ArticulationBody>(FindObjectsInactive.Exclude);
            currentObjects.Clear();
            foreach (Rigidbody body in rigidbodies) currentObjects.Add(GetStablePath(body));
            foreach (ArticulationBody body in articulations) currentObjects.Add(GetStablePath(body));
            spawned.Clear();
            despawned.Clear();
            foreach (string id in currentObjects) if (!previousObjects.Contains(id)) spawned.Add(id);
            foreach (string id in previousObjects) if (!currentObjects.Contains(id)) despawned.Add(id);

            stream.Write(CraneReplayFormat.FrameMagic);
            stream.Write(tick);
            stream.Write(metricTick);
            stream.Write(Time.timeAsDouble);
            stream.Write(episode);
            stream.Write(discontinuity);
            WriteStrings(spawned);
            WriteStrings(despawned);
            stream.Write(rigidbodies.Length + articulations.Length);
            foreach (Rigidbody body in rigidbodies) WriteBody(body);
            foreach (ArticulationBody body in articulations) WriteBody(body);
            WriteWater();
            WriteAcceptedActions();
            WriteOutcomes();
            WriteObservations();

            index.Write(tick);
            index.Write(Time.timeAsDouble);
            index.Write(offset);
            index.Write(discontinuity);
            HashSet<string> swap = previousObjects;
            previousObjects = currentObjects;
            currentObjects = swap;
            if (++ticksSinceFlush >= flushEveryTicks) Flush();
        }

        private void WriteBody(Rigidbody body) {
            stream.Write(GetStablePath(body));
            stream.Write((byte)0);
            CraneReplayFormat.WriteVector(stream, body.position);
            CraneReplayFormat.WriteRotation(stream, body.rotation);
            CraneReplayFormat.WriteVector(stream, body.linearVelocity);
            CraneReplayFormat.WriteVector(stream, body.angularVelocity);
            stream.Write(0);
        }

        private void WriteBody(ArticulationBody body) {
            stream.Write(GetStablePath(body));
            stream.Write((byte)1);
            CraneReplayFormat.WriteVector(stream, body.transform.position);
            CraneReplayFormat.WriteRotation(stream, body.transform.rotation);
            CraneReplayFormat.WriteVector(stream, body.linearVelocity);
            CraneReplayFormat.WriteVector(stream, body.angularVelocity);
            ArticulationReducedSpace joints = body.jointPosition;
            stream.Write(joints.dofCount);
            for (int i = 0; i < joints.dofCount; i++) stream.Write(joints[i]);
        }

        private void WriteWater() {
            WaterSurface[] surfaces = FindObjectsByType<WaterSurface>(FindObjectsInactive.Exclude);
            stream.Write(surfaces.Length);
            foreach (WaterSurface surface in surfaces) {
                stream.Write(GetStablePath(surface));
                stream.Write(surface.simulationTime);
                stream.Write(surface.timeMultiplier);
            }
        }

        private void OnAcceptedAction(CraneAcceptedAction action) {
            lock (acceptedActionsLock) acceptedActions.Add(action);
        }

        private void WriteAcceptedActions() {
            lock (acceptedActionsLock) {
                stream.Write(acceptedActions.Count);
                foreach (CraneAcceptedAction action in acceptedActions) {
                    stream.Write(action.Source ?? string.Empty);
                    stream.Write(action.EpisodeId);
                    stream.Write(action.Sequence);
                    stream.Write(action.SourceObservationTick);
                    stream.Write(action.ReceiveTick);
                    stream.Write(action.ApplicationTick);
                    stream.Write(action.Payload.Encoding ?? string.Empty);
                    byte[] payload = action.Payload.Data ?? Array.Empty<byte>();
                    stream.Write(payload.Length);
                    stream.Write(payload);
                }
                acceptedActions.Clear();
            }
        }

        private void OnTaskOutcome(CraneTaskOutcomeEvent outcome) {
            lock (outcomesLock) outcomes.Add(outcome);
        }

        private void WriteOutcomes() {
            lock (outcomesLock) {
                stream.Write(outcomes.Count);
                foreach (CraneTaskOutcomeEvent outcome in outcomes) {
                    stream.Write(outcome.Source ?? string.Empty);
                    stream.Write(outcome.EpisodeId);
                    stream.Write(outcome.Sequence);
                    stream.Write(outcome.Tick);
                    stream.Write(outcome.Reward);
                    stream.Write(outcome.CumulativeReward);
                    stream.Write(outcome.Terminated);
                    stream.Write(outcome.Truncated);
                    stream.Write(outcome.Reason ?? string.Empty);
                }
                outcomes.Clear();
            }
        }

        private void OnObservation(CraneObservationMetadata observation) {
            lock (observationsLock) observations.Add(observation);
        }

        private void WriteObservations() {
            lock (observationsLock) {
                stream.Write(observations.Count);
                foreach (CraneObservationMetadata observation in observations) {
                    stream.Write(observation.Source);
                    stream.Write(observation.Topic);
                    stream.Write(observation.FrameId);
                    stream.Write(observation.MessageType);
                    stream.Write(observation.Encoding);
                    stream.Write(observation.PayloadPolicy);
                    stream.Write(observation.EpisodeId);
                    stream.Write(observation.Sequence);
                    stream.Write(observation.AcquisitionTick);
                    stream.Write(observation.AcquisitionTime);
                    stream.Write(observation.CompletionTick);
                    stream.Write(observation.CompletionTime);
                    stream.Write(observation.Width);
                    stream.Write(observation.Height);
                    stream.Write(observation.RowStep);
                    stream.Write(observation.ElementCount);
                    stream.Write(observation.PayloadBytes);
                }
                observations.Clear();
            }
        }

        private string GetStablePath(Component component) {
            EntityId id = component.GetEntityId();
            if (!stablePaths.TryGetValue(id, out string path)) {
                path = CraneReplayFormat.StablePath(component);
                stablePaths.Add(id, path);
            }
            return path;
        }

        private void WriteStrings(IReadOnlyList<string> values) {
            stream.Write(values.Count);
            foreach (string value in values) stream.Write(value);
        }

        private void Flush() {
            stream?.Flush();
            index?.Flush();
            ticksSinceFlush = 0;
        }

        private void OnDestroy() {
            SceneManager.sceneLoaded -= OnSceneLoaded;
            CloseStreams();
        }

        private void OnApplicationQuit() => CloseStreams();

        private void CloseStreams() {
            CraneActionGate.UnsubscribeAcceptedActions(OnAcceptedAction);
            CraneTaskOutcome.Unsubscribe(OnTaskOutcome);
            CraneObservationJournal.Unsubscribe(OnObservation);
            lock (acceptedActionsLock) acceptedActions.Clear();
            lock (outcomesLock) outcomes.Clear();
            lock (observationsLock) observations.Clear();
            Flush();
            stream?.Dispose();
            index?.Dispose();
            stream = null;
            index = null;
        }
    }

    /// <summary>Applies recorded states exactly; it never re-simulates or interpolates them.</summary>
    [DefaultExecutionOrder(-32000)]
    public sealed class CraneEpisodePlayer : MonoBehaviour {
        private BinaryReader stream;
        private BinaryReader index;
        private readonly Dictionary<string, Rigidbody> rigidbodies = new();
        private readonly Dictionary<string, ArticulationBody> articulations = new();
        private readonly Dictionary<string, WaterSurface> water = new();
        private readonly Dictionary<WaterSurface, float> originalWaterTimeMultipliers = new();
        private readonly Dictionary<WaterSurface, float> desiredWaterSimulationTimes = new();
        private readonly List<CraneReplayAction> replayActions = new();
        private readonly List<CraneReplayOutcome> replayOutcomes = new();
        private readonly List<CraneReplayObservation> replayObservations = new();
        private string inputPath;
        private float playbackRate = 1;
        private double accumulator;
        private float recordedFixedDelta;
        private bool paused;
        private bool loaded;
        private int streamVersion;

        public string RecordedBuildManifestHash { get; private set; } = string.Empty;
        public string RecordedBuildManifestJson { get; private set; } = string.Empty;
        public string RecordedRuntimeMetadataJson { get; private set; } = string.Empty;

        public bool Paused => paused;
        public float PlaybackRate { get => playbackRate; set => playbackRate = Mathf.Max(0, value); }
        public IReadOnlyList<CraneReplayAction> LastActions => replayActions;
        public IReadOnlyList<CraneReplayOutcome> LastOutcomes => replayOutcomes;
        public IReadOnlyList<CraneReplayObservation> LastObservations => replayObservations;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            string path = CraneReplayFormat.ReadArgument(Environment.GetCommandLineArgs(), "--crane-replay");
            if (string.IsNullOrWhiteSpace(path)) return;
            var host = new GameObject("CRANE Episode Player");
            DontDestroyOnLoad(host);
            host.AddComponent<CraneEpisodePlayer>();
        }

        private IEnumerator Start() {
            inputPath = Path.GetFullPath(CraneReplayFormat.ReadArgument(
                Environment.GetCommandLineArgs(), "--crane-replay"));
            stream = new BinaryReader(new BufferedStream(File.OpenRead(inputPath), 64 * 1024));
            index = new BinaryReader(new BufferedStream(File.OpenRead(inputPath + ".index"), 16 * 1024));
            string scenePath = ReadHeader();
            string sceneName = Path.GetFileNameWithoutExtension(scenePath);
            if (!string.IsNullOrEmpty(sceneName) && SceneManager.GetActiveScene().name != sceneName) {
                AsyncOperation load = SceneManager.LoadSceneAsync(sceneName, LoadSceneMode.Single);
                while (load != null && !load.isDone) yield return null;
            }
            DisableLiveSimulation();
            RebuildObjectMaps();
            loaded = true;
            StepOneTick();
        }

        private string ReadHeader() {
            if (stream.ReadInt32() != CraneReplayFormat.StreamMagic)
                throw new InvalidDataException("Unsupported CRANE replay stream.");
            streamVersion = stream.ReadInt32();
            if (streamVersion < CraneReplayFormat.MinimumReadableVersion ||
                streamVersion > CraneReplayFormat.Version)
                throw new InvalidDataException($"Unsupported CRANE replay stream version {streamVersion}.");
            stream.ReadString(); // Unity version
            stream.ReadString(); // build GUID
            string scenePath = stream.ReadString();
            stream.ReadString(); // configuration hash
            recordedFixedDelta = stream.ReadSingle();
            stream.ReadInt64(); // UTC start
            if (streamVersion >= 4) {
                RecordedBuildManifestHash = stream.ReadString();
                RecordedBuildManifestJson = stream.ReadString();
                RecordedRuntimeMetadataJson = stream.ReadString();
                string actualHash = CraneReplayFormat.HashText(RecordedBuildManifestJson);
                if (!string.Equals(RecordedBuildManifestHash, actualHash,
                        StringComparison.OrdinalIgnoreCase))
                    throw new InvalidDataException("CRANE replay build manifest hash mismatch.");
            }
            if (index.ReadInt32() != CraneReplayFormat.IndexMagic)
                throw new InvalidDataException("Unsupported CRANE replay index.");
            int indexVersion = index.ReadInt32();
            if (indexVersion < CraneReplayFormat.MinimumReadableVersion ||
                indexVersion > CraneReplayFormat.Version)
                throw new InvalidDataException($"Unsupported CRANE replay index version {indexVersion}.");
            return scenePath;
        }

        private void Update() {
            if (!loaded) return;
            ApplyWaterStates();
            if (paused || playbackRate <= 0) return;
            accumulator += Time.unscaledDeltaTime * playbackRate;
            while (accumulator >= recordedFixedDelta) {
                if (!StepOneTick()) { paused = true; break; }
                accumulator -= recordedFixedDelta;
            }
            ApplyWaterStates();
        }

        public void Pause(bool value) => paused = value;
        public bool Step() => paused && StepOneTick();

        public bool SeekTick(long targetTick) {
            if (index == null) return false;
            long count = (index.BaseStream.Length - sizeof(int) * 2) / CraneReplayFormat.IndexEntryBytes;
            long low = 0, high = count - 1, best = -1;
            while (low <= high) {
                long middle = low + (high - low) / 2;
                index.BaseStream.Position = sizeof(int) * 2 + middle * CraneReplayFormat.IndexEntryBytes;
                long tick = index.ReadInt64();
                index.ReadDouble();
                long offset = index.ReadInt64();
                index.ReadBoolean();
                if (tick <= targetTick) { best = offset; low = middle + 1; }
                else high = middle - 1;
            }
            if (best < 0) return false;
            stream.BaseStream.Position = best;
            accumulator = 0;
            return StepOneTick();
        }

        private bool StepOneTick() {
            if (stream.BaseStream.Position >= stream.BaseStream.Length) return false;
            if (stream.ReadInt32() != CraneReplayFormat.FrameMagic)
                throw new InvalidDataException("Corrupt CRANE replay frame.");
            stream.ReadInt64();
            stream.ReadInt64();
            stream.ReadDouble();
            stream.ReadInt64();
            stream.ReadBoolean();
            SkipStrings();
            SkipStrings();
            int bodyCount = stream.ReadInt32();
            for (int i = 0; i < bodyCount; i++) ReadAndApplyBody();
            int waterCount = stream.ReadInt32();
            for (int i = 0; i < waterCount; i++) {
                string id = stream.ReadString();
                float simulationTime = stream.ReadSingle();
                stream.ReadSingle(); // recorded live time multiplier; replay pins exact spectral time
                if (water.TryGetValue(id, out WaterSurface surface)) {
                    desiredWaterSimulationTimes[surface] = simulationTime;
                    surface.timeMultiplier = 0;
                    surface.simulationTime = simulationTime;
                }
            }
            replayActions.Clear();
            if (streamVersion >= 2) ReadActions();
            replayOutcomes.Clear();
            if (streamVersion >= 3) ReadOutcomes();
            replayObservations.Clear();
            if (streamVersion >= 5) ReadObservations();
            return true;
        }

        private void ReadActions() {
            int actionCount = stream.ReadInt32();
            if (actionCount < 0 || actionCount > 1_000_000)
                throw new InvalidDataException($"Invalid replay action count {actionCount}.");
            for (int i = 0; i < actionCount; i++) {
                string source = stream.ReadString();
                long episodeId = stream.ReadInt64();
                long sequence = stream.ReadInt64();
                long sourceObservationTick = stream.ReadInt64();
                long receiveTick = stream.ReadInt64();
                long applicationTick = stream.ReadInt64();
                string encoding = stream.ReadString();
                int payloadLength = stream.ReadInt32();
                if (payloadLength < 0 || payloadLength > 64 * 1024 * 1024)
                    throw new InvalidDataException($"Invalid replay action payload length {payloadLength}.");
                byte[] payload = stream.ReadBytes(payloadLength);
                if (payload.Length != payloadLength)
                    throw new EndOfStreamException("Replay action payload is truncated.");
                replayActions.Add(new CraneReplayAction(source, episodeId, sequence,
                    sourceObservationTick, receiveTick, applicationTick, encoding, payload));
            }
        }

        private void ReadOutcomes() {
            int outcomeCount = stream.ReadInt32();
            if (outcomeCount < 0 || outcomeCount > 1_000_000)
                throw new InvalidDataException($"Invalid replay outcome count {outcomeCount}.");
            for (int i = 0; i < outcomeCount; i++) {
                string source = stream.ReadString();
                long episodeId = stream.ReadInt64();
                long sequence = stream.ReadInt64();
                long tick = stream.ReadInt64();
                double reward = stream.ReadDouble();
                double cumulativeReward = stream.ReadDouble();
                bool terminated = stream.ReadBoolean();
                bool truncated = stream.ReadBoolean();
                string reason = stream.ReadString();
                replayOutcomes.Add(new CraneReplayOutcome(source, episodeId, sequence, tick,
                    reward, cumulativeReward, terminated, truncated, reason));
            }
        }

        private void ReadObservations() {
            int observationCount = stream.ReadInt32();
            if (observationCount < 0 || observationCount > 1_000_000)
                throw new InvalidDataException(
                    $"Invalid replay observation count {observationCount}.");
            for (int i = 0; i < observationCount; i++) {
                var metadata = new CraneObservationMetadata(
                    stream.ReadString(), stream.ReadString(), stream.ReadString(),
                    stream.ReadString(), stream.ReadString(), stream.ReadString(),
                    stream.ReadInt64(), stream.ReadInt64(), stream.ReadInt64(),
                    stream.ReadDouble(), stream.ReadInt64(), stream.ReadDouble(),
                    stream.ReadInt32(), stream.ReadInt32(), stream.ReadInt32(),
                    stream.ReadInt32(), stream.ReadInt64());
                replayObservations.Add(new CraneReplayObservation(metadata));
            }
        }

        private void ReadAndApplyBody() {
            string id = stream.ReadString();
            byte kind = stream.ReadByte();
            Vector3 position = CraneReplayFormat.ReadVector(stream);
            Quaternion rotation = CraneReplayFormat.ReadRotation(stream);
            Vector3 linearVelocity = CraneReplayFormat.ReadVector(stream);
            Vector3 angularVelocity = CraneReplayFormat.ReadVector(stream);
            int jointCount = stream.ReadInt32();
            float[] joints = new float[jointCount];
            for (int i = 0; i < jointCount; i++) joints[i] = stream.ReadSingle();
            if (kind == 0 && rigidbodies.TryGetValue(id, out Rigidbody rigidbody)) {
                rigidbody.position = position;
                rigidbody.rotation = rotation;
                rigidbody.linearVelocity = linearVelocity;
                rigidbody.angularVelocity = angularVelocity;
            }
            else if (kind == 1 && articulations.TryGetValue(id, out ArticulationBody articulation)) {
                if (articulation.isRoot) articulation.TeleportRoot(position, rotation);
                articulation.linearVelocity = linearVelocity;
                articulation.angularVelocity = angularVelocity;
                if (jointCount == articulation.dofCount) {
                    articulation.jointPosition = jointCount switch {
                        1 => new ArticulationReducedSpace(joints[0]),
                        2 => new ArticulationReducedSpace(joints[0], joints[1]),
                        3 => new ArticulationReducedSpace(joints[0], joints[1], joints[2]),
                        _ => articulation.jointPosition
                    };
                }
            }
        }

        private static void DisableLiveSimulation() {
            foreach (MonoBehaviour behaviour in FindObjectsByType<MonoBehaviour>(FindObjectsInactive.Include)) {
                string typeName = behaviour.GetType().FullName ?? string.Empty;
                if (typeName.StartsWith("Sim.Physics.") || typeName.StartsWith("Sim.Actuators.") ||
                    typeName.StartsWith("Sim.Controllers.")) behaviour.enabled = false;
            }
            foreach (Rigidbody body in FindObjectsByType<Rigidbody>(FindObjectsInactive.Include))
                body.isKinematic = true;
            foreach (ArticulationBody body in FindObjectsByType<ArticulationBody>(FindObjectsInactive.Include))
                if (body.isRoot) body.immovable = true;
        }

        private void RebuildObjectMaps() {
            rigidbodies.Clear(); articulations.Clear(); water.Clear();
            originalWaterTimeMultipliers.Clear();
            desiredWaterSimulationTimes.Clear();
            foreach (Rigidbody body in FindObjectsByType<Rigidbody>(FindObjectsInactive.Include))
                rigidbodies[CraneReplayFormat.StablePath(body)] = body;
            foreach (ArticulationBody body in FindObjectsByType<ArticulationBody>(FindObjectsInactive.Include))
                articulations[CraneReplayFormat.StablePath(body)] = body;
            foreach (WaterSurface surface in FindObjectsByType<WaterSurface>(FindObjectsInactive.Include)) {
                water[CraneReplayFormat.StablePath(surface)] = surface;
                originalWaterTimeMultipliers[surface] = surface.timeMultiplier;
            }
        }

        private void ApplyWaterStates() {
            foreach ((WaterSurface surface, float simulationTime) in desiredWaterSimulationTimes) {
                if (surface == null) continue;
                surface.timeMultiplier = 0;
                surface.simulationTime = simulationTime;
            }
        }

        private void SkipStrings() {
            int count = stream.ReadInt32();
            for (int i = 0; i < count; i++) stream.ReadString();
        }

        private void OnDestroy() {
            foreach ((WaterSurface surface, float multiplier) in originalWaterTimeMultipliers)
                if (surface != null) surface.timeMultiplier = multiplier;
            stream?.Dispose();
            index?.Dispose();
        }
    }
}
