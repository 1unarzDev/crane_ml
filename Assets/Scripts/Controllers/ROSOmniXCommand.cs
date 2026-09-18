using System;
using System.Globalization;
using System.Threading;
using RosMessageTypes.Geometry;
using Sim.Utils.Performance;
using Sim.Utils.ROS;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Controllers {
    /// <summary>
    /// Fixed-step ROS command adapter for the production Omni-X aquatic controller. Transport
    /// callbacks only enqueue immutable data; actuator commands change in FixedUpdate.
    /// </summary>
    public sealed class ROSOmniXCommand : MonoBehaviour, ICraneEpisodeResettable {
        private readonly struct Command {
            public readonly float Forward;
            public readonly float Lateral;
            public readonly float Yaw;

            public Command(float forward, float lateral, float yaw) {
                Forward = forward;
                Lateral = lateral;
                Yaw = yaw;
            }
        }

        private readonly CraneQueuedAction<Command> pending = new(command =>
            CraneActionPayloadEncoding.Float32Array(new[] {
                command.Forward, command.Lateral, command.Yaw
            }));
        private OmniXController controller;
        private string topic;
        private float linearScale;
        private float yawScale;
        private long sequence;
        private long lastApplicationTick = -1;
        private long timeoutTicks;
        private bool stoppedForTimeout;

        public int ResetPriority => -40;

        public void Initialize(OmniXController target, string commandTopic,
            float maximumLinearVelocity, float maximumYawRate, long commandTimeoutTicks) {
            controller = target ?? throw new ArgumentNullException(nameof(target));
            topic = commandTopic;
            linearScale = Mathf.Max(0.0001f, maximumLinearVelocity);
            yawScale = Mathf.Max(0.0001f, maximumYawRate);
            timeoutTicks = Math.Max(1, commandTimeoutTicks);
            var subscriber = gameObject.AddComponent<ROSSubscriber>();
            subscriber.Initialize<TwistStampedMsg>(topic, Receive);
            Debug.Log($"CRANE_ROS_COMMAND_READY topic={topic} controller={controller.name} " +
                      $"linearScale={linearScale:R} yawScale={yawScale:R} " +
                      $"timeoutTicks={timeoutTicks}");
        }

        public void CaptureEpisodeInitialState() { }

        public void ResetEpisode(in CraneEpisodeResetContext context,
            CraneEpisodeResetPhase phase) {
            if (phase != CraneEpisodeResetPhase.BeforePhysics) return;
            pending.Clear();
            sequence = 0;
            lastApplicationTick = -1;
            stoppedForTimeout = false;
            Apply(new Command(0f, 0f, 0f));
        }

        private void Receive(TwistStampedMsg message) {
            if (message?.twist == null || message.header?.stamp == null) return;
            double seconds = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9;
            long sourceTick = seconds < 0 ? -1 :
                (long)Math.Round(seconds / Time.fixedDeltaTime, MidpointRounding.AwayFromZero);
            var command = new Command(
                Mathf.Clamp((float)message.twist.linear.x / linearScale, -1f, 1f),
                Mathf.Clamp((float)message.twist.linear.y / linearScale, -1f, 1f),
                Mathf.Clamp((float)message.twist.angular.z / yawScale, -1f, 1f));
            pending.Receive(command, $"ros:{topic}", Interlocked.Increment(ref sequence),
                sourceTick);
        }

        private void FixedUpdate() {
            if (controller == null) return;
            if (controller.movementOverride) {
                pending.Clear();
                return;
            }
            if (pending.TryApply(Apply, out _)) {
                lastApplicationTick = CraneRuntimeMetrics.SimulationTick;
                stoppedForTimeout = false;
            }
            else if (!stoppedForTimeout && lastApplicationTick >= 0 &&
                     CraneRuntimeMetrics.SimulationTick - lastApplicationTick > timeoutTicks) {
                Apply(new Command(0f, 0f, 0f));
                stoppedForTimeout = true;
                CraneRuntimeMetrics.ReportCommandTimeout();
                Debug.LogWarning($"CRANE_ROS_COMMAND_TIMEOUT topic={topic} " +
                                 $"lastApplicationTick={lastApplicationTick}");
            }
        }

        private void Apply(Command command) {
            // ROS FLU x/y maps to the controller's forward/lateral convention.
            controller.SetMotion(new Vector3(command.Lateral, command.Forward, 0f),
                new Vector3(0f, 0f, command.Yaw));
        }
    }

    internal static class ROSOmniXCommandBootstrap {
        private static string topic;
        private static float linearScale;
        private static float yawScale;
        private static long timeoutTicks;
        private static bool enabled;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Initialize() {
            string[] args = Environment.GetCommandLineArgs();
            int flag = Array.IndexOf(args, "--crane-ros-cmd-vel");
            enabled = flag >= 0 && !ROSPublisher.TransportSuppressed;
            if (!enabled) return;
            topic = flag + 1 < args.Length && !args[flag + 1].StartsWith("-") ?
                args[flag + 1] : "/crane/cmd_vel_stamped";
            linearScale = ReadFloat(args, "--crane-cmd-vel-linear-scale", 1f);
            yawScale = ReadFloat(args, "--crane-cmd-vel-yaw-scale", 1f);
            timeoutTicks = ReadLong(args, "--crane-command-timeout-ticks", 25);
            SceneManager.sceneLoaded += OnSceneLoaded;
        }

        private static void OnSceneLoaded(Scene scene, LoadSceneMode mode) {
            if (!enabled) return;
            OmniXController controller = UnityEngine.Object.FindAnyObjectByType<OmniXController>(
                FindObjectsInactive.Exclude);
            if (controller == null) {
                Debug.LogWarning($"CRANE_ROS_COMMAND_UNAVAILABLE scene={scene.name} " +
                                 "reason=no-enabled-omni-x-controller");
                return;
            }
            var adapter = controller.gameObject.GetComponent<ROSOmniXCommand>() ??
                controller.gameObject.AddComponent<ROSOmniXCommand>();
            adapter.Initialize(controller, topic, linearScale, yawScale, timeoutTicks);
        }

        private static float ReadFloat(string[] args, string key, float fallback) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length && float.TryParse(args[index + 1],
                NumberStyles.Float, CultureInfo.InvariantCulture, out float value) ? value : fallback;
        }

        private static long ReadLong(string[] args, string key, long fallback) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length && long.TryParse(args[index + 1],
                NumberStyles.Integer, CultureInfo.InvariantCulture, out long value) ? value : fallback;
        }
    }
}
