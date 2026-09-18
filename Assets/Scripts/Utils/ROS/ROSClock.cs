using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Rosgraph;

namespace Sim.Utils.ROS {
    public class ROSClock : MonoBehaviour, Sim.Utils.Performance.ICraneEpisodeResettable {
        private static ROSClock s_Owner;
        [SerializeField] private Clock.ClockMode clockMode;

        [SerializeField, HideInInspector] private Clock.ClockMode lastSetClockMode;

        [SerializeField] private double publishRateHz = 100f;

        private double lastPublishTimeSeconds;

        ROSConnection ros;

        private double PublishPeriodSeconds => 1.0f / publishRateHz;
        public int ResetPriority => -90;

        public void CaptureEpisodeInitialState() { }

        public void ResetEpisode(in Sim.Utils.Performance.CraneEpisodeResetContext context,
            Sim.Utils.Performance.CraneEpisodeResetPhase phase) {
            if (phase == Sim.Utils.Performance.CraneEpisodeResetPhase.BeforePhysics)
                lastPublishTimeSeconds = -PublishPeriodSeconds;
        }

        private bool ShouldPublishMessage => Clock.FrameStartTimeInSeconds - PublishPeriodSeconds > lastPublishTimeSeconds;

        private void OnValidate() {
            var clocks = FindObjectsByType<ROSClock>(FindObjectsSortMode.None);
            if (clocks.Length > 1) {
                Debug.LogWarning("Found too many clock publishers in the scene, there should only be one!");
            }

            if (Application.isPlaying && lastSetClockMode != clockMode) {
                Debug.LogWarning("Can't change ClockMode during simulation! Setting it back...");
                clockMode = lastSetClockMode;
            }

            SetClockMode(clockMode);
        }

        private void SetClockMode(Clock.ClockMode mode) {
            Clock.Mode = mode;
            lastSetClockMode = mode;
        }

        // Start is called before the first frame update
        private void Start() {
            if (s_Owner != null && s_Owner != this) {
                Debug.LogWarning($"Disabling duplicate ROSClock on {name}; {s_Owner.name} " +
                                 "already owns the process simulation clock.");
                enabled = false;
                return;
            }
            s_Owner = this;
            SetClockMode(clockMode);
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<ClockMsg>("clock");
        }

        private void PublishMessage() {
            var publishTime = Clock.time;
            var clockMsg = new TimeMsg {
                sec = (int)publishTime,
                nanosec = (uint)((publishTime - Math.Floor(publishTime)) * Clock.k_NanoSecondsInSeconds)
            };
            lastPublishTimeSeconds = publishTime;
            ros.Publish("clock", clockMsg);
        }

        private void Update() {
            if (ShouldPublishMessage) {
                PublishMessage();
            }
        }


        private void OnDestroy() {
            if (s_Owner == this) s_Owner = null;
        }
    }
}
