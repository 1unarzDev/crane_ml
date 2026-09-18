using UnityEngine;
using Sim.Utils.ROS;
using RosMessageTypes.Std;
using Sim.Utils.Performance;
using System.Threading;

namespace Sim.Actuators.Motors {
    public class ROSThruster : Thruster {
        [SerializeField] private string topicName;

        private ROSSubscriber ros;
        private readonly CraneQueuedAction<float> pendingCommand =
            new(CraneActionPayloadEncoding.Float32);
        private System.Action<float> applyCommand;
        private long sequence;

        public override void ResetEpisode(in CraneEpisodeResetContext context,
            CraneEpisodeResetPhase phase) {
            if (phase == CraneEpisodeResetPhase.BeforePhysics) pendingCommand.Clear();
            base.ResetEpisode(context, phase);
        }

        protected override void Awake() {
            base.Awake();
            applyCommand = ApplyCommand;
            ros = gameObject.AddComponent<ROSSubscriber>();
            ros.Initialize<Float32Msg>(topicName, CommandCallback);
        }

        private void CommandCallback(Float32Msg msg) {
            pendingCommand.Receive(msg.data, $"ros:{topicName}",
                Interlocked.Increment(ref sequence), -1);
        }

        protected override void FixedUpdate() {
            pendingCommand.TryApply(applyCommand, out _);
            base.FixedUpdate();
        }

        private void ApplyCommand(float value) => base.SetCommand(value);
    }
}
