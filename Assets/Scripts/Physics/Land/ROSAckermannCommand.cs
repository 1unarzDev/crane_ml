using System;
using System.Globalization;
using System.Threading;
using RosMessageTypes.Geometry;
using Sim.Utils.Performance;
using Sim.Utils.ROS;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Physics.Land {
    /// <summary>Applies stamped ROS velocity commands to the PhysX Ackermann rover.</summary>
    public sealed class ROSAckermannCommand : MonoBehaviour, ICraneEpisodeResettable {
        private readonly struct Command {
            public readonly float Linear;
            public readonly float Angular;
            public Command(float linear, float angular) { Linear = linear; Angular = angular; }
        }

        private readonly CraneQueuedAction<Command> pending = new(command =>
            CraneActionPayloadEncoding.Float32Array(new[] { command.Linear, command.Angular }));
        private AckermannRoverDynamics rover;
        private string topic;
        private float maximumLinearSpeed;
        private long timeoutTicks;
        private long sequence;
        private long lastApplicationTick = -1;
        private bool stoppedForTimeout;

        public int ResetPriority => -40;

        public void Initialize(AckermannRoverDynamics target, string commandTopic,
            float linearSpeed, long commandTimeoutTicks) {
            rover = target ?? throw new ArgumentNullException(nameof(target));
            topic = commandTopic;
            maximumLinearSpeed = Mathf.Max(0.01f, linearSpeed);
            timeoutTicks = Math.Max(1, commandTimeoutTicks);
            var subscriber = gameObject.AddComponent<ROSSubscriber>();
            subscriber.Initialize<TwistStampedMsg>(topic, Receive);
            Debug.Log($"CRANE_ROS_ACKERMANN_COMMAND_READY topic={topic} rover={rover.name} " +
                      $"maximumLinearSpeed={maximumLinearSpeed:R} timeoutTicks={timeoutTicks}");
        }

        public void CaptureEpisodeInitialState() { }

        public void ResetEpisode(in CraneEpisodeResetContext context,
            CraneEpisodeResetPhase phase) {
            if (phase != CraneEpisodeResetPhase.BeforePhysics) return;
            pending.Clear();
            sequence = 0;
            lastApplicationTick = -1;
            stoppedForTimeout = false;
            Stop();
        }

        private void Receive(TwistStampedMsg message) {
            if (message?.twist == null || message.header?.stamp == null) return;
            double seconds = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9;
            long sourceTick = seconds < 0 ? -1 :
                (long)Math.Round(seconds / Time.fixedDeltaTime, MidpointRounding.AwayFromZero);
            pending.Receive(new Command((float)message.twist.linear.x,
                    (float)message.twist.angular.z), $"ros:{topic}",
                Interlocked.Increment(ref sequence), sourceTick);
        }

        private void FixedUpdate() {
            if (rover == null) return;
            if (pending.TryApply(Apply, out _)) {
                lastApplicationTick = CraneRuntimeMetrics.SimulationTick;
                stoppedForTimeout = false;
            }
            else if (!stoppedForTimeout && lastApplicationTick >= 0 &&
                     CraneRuntimeMetrics.SimulationTick - lastApplicationTick > timeoutTicks) {
                Stop();
                stoppedForTimeout = true;
                CraneRuntimeMetrics.ReportCommandTimeout();
                Debug.LogWarning($"CRANE_ROS_ACKERMANN_COMMAND_TIMEOUT topic={topic} " +
                                 $"lastApplicationTick={lastApplicationTick}");
            }
        }

        private void Apply(Command command) {
            float linear = Mathf.Clamp(command.Linear, -maximumLinearSpeed, maximumLinearSpeed);
            if (Mathf.Abs(linear) < 0.01f) {
                Stop();
                return;
            }
            float steeringRadians = Mathf.Atan(rover.Wheelbase * command.Angular / linear);
            float steering = steeringRadians * Mathf.Rad2Deg / rover.MaximumSteeringAngle;
            rover.SetCommand(linear / maximumLinearSpeed, steering, 0f);
        }

        private void Stop() => rover.SetCommand(0f, 0f, 1f);
    }

    internal static class ROSAckermannCommandBootstrap {
        private static bool enabled;
        private static string topic;
        private static float maximumLinearSpeed;
        private static long timeoutTicks;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            string[] args = Environment.GetCommandLineArgs();
            int flag = Array.IndexOf(args, "--crane-ros-ackermann-cmd-vel");
            enabled = flag >= 0 && !ROSPublisher.TransportSuppressed;
            if (!enabled) return;
            topic = flag + 1 < args.Length && !args[flag + 1].StartsWith("-")
                ? args[flag + 1] : "/crane/cmd_vel_stamped";
            maximumLinearSpeed = ReadFloat(args, "--crane-ackermann-linear-scale", 2f);
            timeoutTicks = ReadLong(args, "--crane-command-timeout-ticks", 25);
            SceneManager.sceneLoaded += OnSceneLoaded;
        }

        private static void OnSceneLoaded(Scene scene, LoadSceneMode mode) {
            if (!enabled) return;
            AckermannRoverDynamics rover = UnityEngine.Object.FindAnyObjectByType<
                AckermannRoverDynamics>(FindObjectsInactive.Exclude);
            if (rover == null) {
                Debug.LogWarning($"CRANE_ROS_ACKERMANN_COMMAND_UNAVAILABLE scene={scene.name} " +
                                 "reason=no-enabled-ackermann-rover");
                return;
            }
            var adapter = rover.GetComponent<ROSAckermannCommand>() ??
                          rover.gameObject.AddComponent<ROSAckermannCommand>();
            adapter.Initialize(rover, topic, maximumLinearSpeed, timeoutTicks);
        }

        private static float ReadFloat(string[] args, string key, float fallback) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length && float.TryParse(args[index + 1],
                NumberStyles.Float, CultureInfo.InvariantCulture, out float value)
                ? value : fallback;
        }

        private static long ReadLong(string[] args, string key, long fallback) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length && long.TryParse(args[index + 1],
                NumberStyles.Integer, CultureInfo.InvariantCulture, out long value)
                ? value : fallback;
        }
    }
}
