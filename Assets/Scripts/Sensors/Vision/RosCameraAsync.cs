using RosMessageTypes.Sensor;
using UnityEngine;
using UnityEngine.Rendering;
using Sim.Utils.ROS;
using Sim.Utils.Performance;
using System;
using System.Collections.Generic;

namespace Sim.Sensors.Vision {
    public class ROSCameraAsync : MonoBehaviour, IROSSensor<ImageMsg> {
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
        [SerializeField] private RenderTexture rgbRenderTexture;
        [SerializeField] private Camera sensorCamera;

        [SerializeField] private string topicName = "camera/image_raw";
        [SerializeField] private string frameId = "front_camera_link";
        [SerializeField] private float Hz = 15.0f;
        public ROSPublisher publisher { get; set; }

        private static byte[] s_ScratchSpace;
        private byte[] rgbData;
        private float timeSincePublish = 0.0f;
        private bool alive;
        private long requestTick;
        private int requestWidth;
        private int requestHeight;
        private double requestSimulationTime;
        private readonly Queue<ReadbackMetadata> pendingReadbacks = new(MaxPendingReadbacks);

        private void Awake() {
            alive = true;
            if (sensorCamera == null) {
                Debug.LogError("Missing a camera reference.");
                enabled = false;
                return;
            }

            publisher = gameObject.AddComponent<ROSPublisher>();
        }

        private void Start() {
            sensorCamera.targetTexture = rgbRenderTexture;
            publisher.Initialize(topicName, frameId, CreateMessage, Hz, true);
        }

        public ImageMsg CreateMessage() {
            var message = new ImageMsg(publisher.CreateHeader(), (uint)requestHeight,
                (uint)requestWidth, "rgb8", 0, (uint)(requestWidth * 3), rgbData);
            CraneRuntimeMetrics.ReportImage(false, requestWidth, requestHeight,
                message.data, requestTick);
            return message;
        }

        private void FixedUpdate() {
            timeSincePublish += Time.fixedDeltaTime;
            if (timeSincePublish >= 1.0f / Hz) {
                if (pendingReadbacks.Count < MaxPendingReadbacks) RequestReadback(rgbRenderTexture);
                else if (HasCurrentEpisodeRequest())
                    CraneRuntimeMetrics.ReportStaleObservation();
                timeSincePublish -= 1.0f / Hz;
            }
        }

        private void RequestReadback(RenderTexture targetTexture) {
            pendingReadbacks.Enqueue(new ReadbackMetadata(CraneRuntimeMetrics.EpisodeId,
                CraneRuntimeMetrics.SimulationTick, targetTexture.width, targetTexture.height,
                Time.timeAsDouble));
            AsyncGPUReadback.Request(targetTexture, 0, TextureFormat.RGB24, OnReadbackComplete);
        }

        private void OnReadbackComplete(AsyncGPUReadbackRequest request) {
            using var marker = CraneProfiler.OtherSensor.Auto();
            using var sensorMarker = CraneProfiler.RgbReadback.Auto();
            if (pendingReadbacks.Count == 0) return;
            ReadbackMetadata metadata = pendingReadbacks.Dequeue();
            if (!alive || metadata.EpisodeId != CraneRuntimeMetrics.EpisodeId) {
                return;
            }
            if (request.hasError) {
                Debug.LogWarning("Failed to read back texture once");
                CraneRuntimeMetrics.ReportFailedObservation();
                return;
            }

            requestTick = metadata.Tick;
            requestWidth = metadata.Width;
            requestHeight = metadata.Height;
            requestSimulationTime = metadata.SimulationTime;
            rgbData = new byte[requestWidth * requestHeight * 3];
            request.GetData<byte>().CopyTo(rgbData);
            ReverseInBlocks(rgbData, requestWidth * 3, requestHeight);

            if (publisher != null) publisher.Publish(requestTick, requestSimulationTime);
            CraneRuntimeMetrics.ReportObservation(requestTick);
        }

        private void OnDestroy() => alive = false;

        private bool HasCurrentEpisodeRequest() {
            long episode = CraneRuntimeMetrics.EpisodeId;
            foreach (ReadbackMetadata metadata in pendingReadbacks)
                if (metadata.EpisodeId == episode) return true;
            return false;
        }

        private static void ReverseInBlocks(byte[] array, int blockSize, int numBlocks) {
            if (s_ScratchSpace == null || s_ScratchSpace.Length < blockSize)
                s_ScratchSpace = new byte[blockSize];
            int start = 0;
            int end = (numBlocks - 1) * blockSize;
            while (start < end) {
                Buffer.BlockCopy(array, start, s_ScratchSpace, 0, blockSize);
                Buffer.BlockCopy(array, end, array, start, blockSize);
                Buffer.BlockCopy(s_ScratchSpace, 0, array, end, blockSize);
                start += blockSize;
                end -= blockSize;
            }
        }
    }
}
