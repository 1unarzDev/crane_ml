using System;
using System.Collections;
using System.IO;
using System.Reflection;
using NUnit.Framework;
using RosMessageTypes.Sensor;
using Sim.Sensors.Lidar;
using Sim.Sensors.Vision;
using Sim.Utils.Performance;
using Sim.Utils.ROS;
using UnityEngine;

namespace Sim.Tests.Editor {
    public sealed class SensorAcquisitionTimingTests {
        private const BindingFlags PrivateInstance = BindingFlags.NonPublic | BindingFlags.Instance;

        [TestCase(0.0, 0, 0u)]
        [TestCase(1.25, 1, 250000000u)]
        [TestCase(42.875, 42, 875000000u)]
        public void ExplicitHeaderUsesAcquisitionTime(double time, int seconds, uint nanoseconds) {
            var host = new GameObject("acquisition-header-test");
            try {
                var publisher = host.AddComponent<ROSPublisher>();
                publisher.frameId = "sensor_frame";
                var header = publisher.CreateHeader(time);
                Assert.That(header.stamp.sec, Is.EqualTo(seconds));
                Assert.That(header.stamp.nanosec, Is.EqualTo(nanoseconds));
                Assert.That(header.frame_id, Is.EqualTo("sensor_frame"));
                Assert.Throws<ArgumentOutOfRangeException>(() => publisher.CreateHeader(double.NaN));
            }
            finally { UnityEngine.Object.DestroyImmediate(host); }
        }

        [TestCase(typeof(ROSCameraAsync), 3)]
        [TestCase(typeof(ROSDepthCameraAsync), 4)]
        public void AsyncImageHeaderUsesCapturedTime(Type sensorType, int bytesPerPixel) {
            var host = new GameObject("async-acquisition-test");
            host.SetActive(false);
            try {
                var sensor = host.AddComponent(sensorType);
                var publisher = host.AddComponent<ROSPublisher>();
                sensorType.GetProperty("publisher").SetValue(sensor, publisher);
                sensorType.GetField("requestSimulationTime", PrivateInstance).SetValue(sensor, 1.25);
                sensorType.GetField("requestWidth", PrivateInstance).SetValue(sensor, 1);
                sensorType.GetField("requestHeight", PrivateInstance).SetValue(sensor, 1);
                sensorType.GetField(bytesPerPixel == 3 ? "rgbData" : "depthData", PrivateInstance)
                    .SetValue(sensor, new byte[bytesPerPixel]);
                var message = (ImageMsg)sensorType.GetMethod("CreateMessage").Invoke(sensor, null);
                Assert.That(message.header.stamp.sec, Is.EqualTo(1));
                Assert.That(message.header.stamp.nanosec, Is.EqualTo(250000000u));
            }
            finally { UnityEngine.Object.DestroyImmediate(host); }
        }

        [TestCase(typeof(ROSCameraAsync))]
        [TestCase(typeof(ROSDepthCameraAsync))]
        public void OldCameraCallbackCannotConsumeNewEpisodeMetadata(Type sensorType) {
            var host = new GameObject("async-reset-test");
            host.SetActive(false);
            try {
                var sensor = host.AddComponent(sensorType);
                Type metadataType = sensorType.GetNestedType("ReadbackMetadata", BindingFlags.NonPublic);
                object oldMetadata = Activator.CreateInstance(metadataType, 1L, 1L, 1, 1, 0.02);
                object newMetadata = Activator.CreateInstance(metadataType, 2L, 1L, 1, 1, 0.02);
                var pending = (IList)sensorType.GetField("pendingReadbacks", PrivateInstance).GetValue(sensor);
                pending.Add(oldMetadata);
                pending.Clear(); // The reset seam discards this episode's pending work.
                pending.Add(newMetadata);
                // No GPU data should be accessed for an old request whose identity was cleared.
                sensorType.GetMethod("OnReadbackComplete", PrivateInstance).Invoke(sensor,
                    new[] { (object)default(UnityEngine.Rendering.AsyncGPUReadbackRequest), oldMetadata });
                Assert.That(pending.Count, Is.EqualTo(1));
                Assert.That(pending[0], Is.SameAs(newMetadata));
            }
            finally { UnityEngine.Object.DestroyImmediate(host); }
        }

        [Test]
        public void DelayedLidarJournalTracksReturnedPayloadAndClearsOnReset() {
            var host = new GameObject("delayed-lidar-test");
            CraneObservationMetadata reported = default;
            Action<CraneObservationMetadata> observer = metadata => {
                if (metadata.Topic == "/timing-test/scan") reported = metadata;
            };
            CraneObservationJournal.Subscribe(observer);
            try {
                var sensor = host.AddComponent<Lidar2D>();
                // EditMode does not invoke the runtime MonoBehaviour lifecycle.
                typeof(Lidar2D).GetMethod("Awake", PrivateInstance).Invoke(sensor, null);
                sensor.Configure(-1, 1, 1, 0.1f, 10, 1, false, "/timing-test/scan", "lidar", 10);
                sensor.ConfigureErrors(7, 0, 0, 1, "off");
                typeof(Lidar2D).GetMethod("Start", PrivateInstance).Invoke(sensor, null);
                sensor.publisher.manualPublish = true;
                CraneRuntimeMetrics.BeginEpisode();
                CraneRuntimeMetrics.AdvanceSimulationTick();
                sensor.publisher.Publish();
                long firstAcquisition = reported.AcquisitionTick;
                CraneRuntimeMetrics.AdvanceSimulationTick();
                sensor.publisher.Publish();
                Assert.That(reported.AcquisitionTick, Is.EqualTo(firstAcquisition));
                Assert.That(reported.CompletionTick, Is.GreaterThan(reported.AcquisitionTick));

                // Only this phase reads the context, which Lidar2D does not need.
                sensor.ResetEpisode(default, CraneEpisodeResetPhase.BeforePhysics);
                CraneRuntimeMetrics.BeginEpisode();
                CraneRuntimeMetrics.AdvanceSimulationTick();
                CraneRuntimeMetrics.AdvanceSimulationTick();
                CraneRuntimeMetrics.AdvanceSimulationTick();
                sensor.publisher.Publish();
                Assert.That(reported.AcquisitionTick, Is.EqualTo(3),
                    "The previous episode's delayed scan must not survive reset");
            }
            finally {
                CraneObservationJournal.Unsubscribe(observer);
                UnityEngine.Object.DestroyImmediate(host);
            }
        }
    }
}
