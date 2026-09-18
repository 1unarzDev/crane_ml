using RosMessageTypes.Sensor;
using UnityEngine;
using System;
using System.Collections.Generic;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;
using Sim.Utils.ROS;
using Sim.Utils.Performance;

namespace Sim.Sensors.Vision {
    public class ROSDepthCameraAsync : MonoBehaviour, IROSSensor<ImageMsg> {
        private readonly struct ReadbackMetadata {
            public readonly long EpisodeId;
            public readonly long Tick;
            public readonly int Width;
            public readonly int Height;
            public readonly double SimulationTime;

            public ReadbackMetadata(long episodeId, long tick, int width, int height,
                double simulationTime) {
                EpisodeId = episodeId;
                Tick = tick;
                Width = width;
                Height = height;
                SimulationTime = simulationTime;
            }
        }

        private const int MaxPendingReadbacks = 2;
        private static byte[] s_ScratchSpace;

        [SerializeField] private RenderTexture depthRenderTexture;
        [SerializeField] private Camera sensorCamera;

        [SerializeField] private string topicName = "camera/depth/image_rect_raw";
        [SerializeField] private string frameId = "front_camera_link";
        [SerializeField] private float Hz = 15.0f;
        public ROSPublisher publisher { get; set; }
        internal Camera SensorCamera => sensorCamera;
        internal int ImageWidth => depthRenderTexture != null ? depthRenderTexture.width : 320;
        internal int ImageHeight => depthRenderTexture != null ? depthRenderTexture.height : 180;
        internal string TopicName => topicName;
        internal string FrameId => frameId;
        internal float PublishRateHz => Hz;

        private CustomPassVolume customPassVolume;
        private CameraDepthBake depthBakePass = new();
        private byte[] depthData;
        private float timeSincePublish = 0.0f;
        private bool alive;
        private long requestTick;
        private int requestWidth;
        private int requestHeight;
        private double requestSimulationTime;
        private bool validateBufferReuse;
        private long validatedEpisode = -1;
        private readonly Queue<ReadbackMetadata> pendingReadbacks = new(MaxPendingReadbacks);

        private void Awake() {
            alive = true;
            validateBufferReuse = Array.IndexOf(Environment.GetCommandLineArgs(),
                "--crane-depth-buffer-validation") >= 0;
            if (sensorCamera == null) {
                Debug.LogError("Missing a camera reference.");
                enabled = false;
                return;
            }

            publisher = gameObject.AddComponent<ROSPublisher>();
        }

        private void Start() {
            customPassVolume = gameObject.AddComponent<CustomPassVolume>();
            customPassVolume.injectionPoint = CustomPassInjectionPoint.AfterPostProcess;
            customPassVolume.targetCamera = sensorCamera;
            depthBakePass.bakingCamera = sensorCamera;
            depthBakePass.depthTexture = depthRenderTexture;
            customPassVolume.customPasses.Add(depthBakePass);
            publisher.Initialize(topicName, frameId, CreateMessage, Hz, true);
        }

        public ImageMsg CreateMessage() {
            var message = new ImageMsg(publisher.CreateHeader(), (uint)requestHeight,
                (uint)requestWidth, "32FC1", 0, (uint)(requestWidth * 4), depthData);
            CraneRuntimeMetrics.ReportImage(true, requestWidth, requestHeight,
                message.data, requestTick);
            return message;
        }

        private void FixedUpdate() {
            timeSincePublish += Time.fixedDeltaTime;
            if (timeSincePublish >= 1.0f / Hz) {
                if (pendingReadbacks.Count < MaxPendingReadbacks) RequestReadback(depthRenderTexture);
                else if (HasCurrentEpisodeRequest())
                    CraneRuntimeMetrics.ReportStaleObservation();
                timeSincePublish -= 1.0f / Hz;
            }
        }

        private void RequestReadback(RenderTexture targetTexture) {
            pendingReadbacks.Enqueue(new ReadbackMetadata(CraneRuntimeMetrics.EpisodeId,
                CraneRuntimeMetrics.SimulationTick, targetTexture.width, targetTexture.height,
                Time.timeAsDouble));
            AsyncGPUReadback.Request(targetTexture, 0, TextureFormat.RFloat, OnReadbackComplete);
        }

        private void OnReadbackComplete(AsyncGPUReadbackRequest request) {
            using var marker = CraneProfiler.OtherSensor.Auto();
            using var sensorMarker = CraneProfiler.DepthReadback.Auto();
            if (pendingReadbacks.Count == 0) return;
            ReadbackMetadata metadata = pendingReadbacks.Dequeue();
            if (!alive || metadata.EpisodeId != CraneRuntimeMetrics.EpisodeId) {
                return;
            }
            if (request.hasError) {
                Debug.LogError("Failed to read back texture once");
                CraneRuntimeMetrics.ReportFailedObservation();
                return;
            }

            requestTick = metadata.Tick;
            requestWidth = metadata.Width;
            requestHeight = metadata.Height;
            requestSimulationTime = metadata.SimulationTime;
            var requestBytes = request.GetData<byte>();
            using (CraneProfiler.DepthCopy.Auto()) {
                int requiredBytes = requestWidth * requestHeight * 4;
                // ROS-TCP queues message objects and serializes their arrays asynchronously, so
                // ROS-enabled frames must retain distinct payloads. With transport suppressed,
                // Publish consumes the data synchronously for metrics and the buffer is reusable.
                if (!ROSPublisher.TransportSuppressed || depthData == null ||
                    depthData.Length != requiredBytes) {
                    depthData = new byte[requiredBytes];
                }
                requestBytes.CopyTo(depthData);
            }
            using (CraneProfiler.DepthRowFlip.Auto()) {
                ReverseInBlocks(depthData, requestWidth * 4, requestHeight);
            }

            long episode = CraneRuntimeMetrics.EpisodeId;
            if (validateBufferReuse && validatedEpisode != episode) {
                int rowBytes = requestWidth * 4;
                int mismatches = 0;
                for (int row = 0; row < requestHeight; row++) {
                    int destinationOffset = row * rowBytes;
                    int sourceOffset = (requestHeight - 1 - row) * rowBytes;
                    for (int column = 0; column < rowBytes; column++) {
                        if (depthData[destinationOffset + column] != requestBytes[sourceOffset + column])
                            mismatches++;
                    }
                }
                CraneRuntimeMetrics.ReportDepthBufferValidation(depthData.Length, mismatches);
                validatedEpisode = episode;
            }

            // Publish via ROSPublisher
            using (CraneProfiler.DepthPublish.Auto()) {
                if (publisher != null) publisher.Publish(requestTick, requestSimulationTime);
            }
            CraneRuntimeMetrics.ReportObservation(requestTick);
        }

        private void OnDestroy() => alive = false;

        private bool HasCurrentEpisodeRequest() {
            long episode = CraneRuntimeMetrics.EpisodeId;
            foreach (ReadbackMetadata metadata in pendingReadbacks)
                if (metadata.EpisodeId == episode) return true;
            return false;
        }

        private void ReverseInBlocks(byte[] array, int blockSize, int numBlocks) {
            if (blockSize * numBlocks > array.Length) {
                Debug.LogError($"Invalid ReverseInBlocks, array length is {array.Length}, should be at least {blockSize * numBlocks}");
                return;
            }

            if (s_ScratchSpace == null || s_ScratchSpace.Length < blockSize)
                s_ScratchSpace = new byte[blockSize];

            int startBlockIndex = 0;
            int endBlockIndex = ((int)numBlocks - 1) * blockSize;

            while (startBlockIndex < endBlockIndex) {
                Buffer.BlockCopy(array, startBlockIndex, s_ScratchSpace, 0, blockSize);
                Buffer.BlockCopy(array, endBlockIndex, array, startBlockIndex, blockSize);
                Buffer.BlockCopy(s_ScratchSpace, 0, array, endBlockIndex, blockSize);
                startBlockIndex += blockSize;
                endBlockIndex -= blockSize;
            }
        }
    }
}
