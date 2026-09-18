using System;
using System.Reflection;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using Sim.Utils.Performance;
using System.Linq;

namespace Sim.Utils.ROS {
    [AddComponentMenu("")]
    public class ROSPublisher : MonoBehaviour, ICraneEpisodeResettable {
        public static bool TransportSuppressed =>
            Environment.GetCommandLineArgs().Contains("--crane-disable-ros");
        public string topicName { get; set; }
        public string frameId { get; set; }
        public float Hz;
        public bool manualPublish;

        private ROSConnection ros;
        private float time;

        private Func<object> createMessage;
        private Action<string, object> publishTyped;
        private long observationSequence;
        private long observationEpisode = -1;
        public int ResetPriority => -100;

        public void CaptureEpisodeInitialState() { }

        public void ResetEpisode(in CraneEpisodeResetContext context,
            CraneEpisodeResetPhase phase) {
            if (phase != CraneEpisodeResetPhase.BeforePhysics) return;
            time = 0;
            observationSequence = 0;
            observationEpisode = context.EpisodeId;
        }

        private static void PublishWrapper<T>(ROSConnection ros, string topic, object msg)
        where T : Unity.Robotics.ROSTCPConnector.MessageGeneration.Message {
            ros.Publish(topic, (T)msg);
        }

        private void OnEnable() { hideFlags = HideFlags.HideInInspector; }

        public void Initialize<T>(string topicName, string frameId, Func<T> createMessage, float Hz=10f, bool manualPublish=false)
        where T : Unity.Robotics.ROSTCPConnector.MessageGeneration.Message {
            this.topicName = topicName;
            this.frameId = frameId;
            this.Hz = Hz;
            this.manualPublish = manualPublish;

            this.createMessage = () => createMessage();

            if (TransportSuppressed) return;

            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<T>(topicName);

            var wrapperMethod = typeof(ROSPublisher)
                .GetMethod(nameof(PublishWrapper), BindingFlags.Static | BindingFlags.NonPublic)
                .MakeGenericMethod(typeof(T));

            publishTyped = (Action<string, object>)
                Delegate.CreateDelegate(
                    typeof(Action<string, object>),
                    ros,
                    wrapperMethod
                );
        }

        private void FixedUpdate() {
            if (manualPublish) return;
            if (createMessage == null) return;

            time += Time.fixedDeltaTime;
            if (time >= 1f / Hz) {
                Publish();
                // Preserve the fractional remainder so configured sensor rates do not
                // drift down when their period is not a multiple of fixedDeltaTime.
                time -= 1f / Hz;
            }
        }

        public void Publish() => Publish(CraneRuntimeMetrics.SimulationTick,
            Clock.time);

        /// <summary>
        /// Publishes an observation while preserving when it was acquired. Async sensors must use
        /// this overload because completion can occur several simulation ticks after acquisition.
        /// </summary>
        public void Publish(long acquisitionTick, double acquisitionTime) {
            object message;
            using (CraneProfiler.RosCreateMessage.Auto()) {
                message = createMessage();
            }
            ReportObservation(message, acquisitionTick, acquisitionTime);
            using (CraneProfiler.RosPublish.Auto()) {
                if (!TransportSuppressed) publishTyped(topicName, message);
            }
        }

        private void ReportObservation(object message, long acquisitionTick,
            double acquisitionTime) {
            long episode = CraneRuntimeMetrics.EpisodeId;
            if (observationEpisode != episode) {
                observationEpisode = episode;
                observationSequence = 0;
            }

            int width = 0;
            int height = 0;
            int rowStep = 0;
            int elements = 0;
            long payloadBytes = 0;
            string encoding = string.Empty;
            switch (message) {
                case ImageMsg image:
                    width = checked((int)image.width);
                    height = checked((int)image.height);
                    rowStep = checked((int)image.step);
                    elements = checked(width * height);
                    payloadBytes = image.data?.LongLength ?? 0;
                    encoding = image.encoding ?? string.Empty;
                    break;
                case PointCloud2Msg cloud:
                    width = checked((int)cloud.width);
                    height = checked((int)cloud.height);
                    rowStep = checked((int)cloud.row_step);
                    elements = checked(width * height);
                    payloadBytes = cloud.data?.LongLength ?? 0;
                    encoding = "sensor_msgs/PointCloud2";
                    break;
                case LaserScanMsg scan:
                    elements = scan.ranges?.Length ?? 0;
                    payloadBytes = (long)elements * sizeof(float);
                    encoding = "sensor_msgs/LaserScan:float32";
                    break;
            }
            if (message?.GetType().FullName == "RosMessageTypes.Vision.Detection3DArrayMsg") {
                FieldInfo detectionsField = message.GetType().GetField("detections");
                if (detectionsField?.GetValue(message) is Array detectionArray)
                    elements = detectionArray.Length;
                encoding = "vision_msgs/Detection3DArray";
            }

            var metadata = new CraneObservationMetadata(
                StableSourcePath(), topicName, frameId,
                message?.GetType().FullName ?? string.Empty, encoding,
                "metadata-only:regenerate-from-authoritative-state",
                episode, ++observationSequence, acquisitionTick, acquisitionTime,
                CraneRuntimeMetrics.SimulationTick, Clock.time,
                width, height, rowStep, elements, payloadBytes);
            CraneObservationJournal.Report(metadata);
        }

        private string StableSourcePath() {
            Transform current = transform;
            string path = string.Empty;
            while (current != null) {
                string segment = $"{current.GetSiblingIndex()}:{current.name}";
                path = string.IsNullOrEmpty(path) ? segment : segment + "/" + path;
                current = current.parent;
            }
            return $"{gameObject.scene.path}|{path}|{GetType().FullName}";
        }

        public HeaderMsg CreateHeader() {
            var header = new HeaderMsg { frame_id=frameId };
            var publishTime = Clock.Now;
            var sec = publishTime;
            var nanosec = (publishTime - Math.Floor(publishTime)) * Clock.k_NanoSecondsInSeconds;
            header.stamp.sec = (int)sec;
            header.stamp.nanosec = (uint)nanosec;
            return header;
        }
    }
}
