using System;
using System.Globalization;
using System.Threading;
using RosMessageTypes.Geometry;
using Sim.Utils.Performance;
using Sim.Utils.ROS;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Physics.Land {
    /// <summary>Applies stamped ROS velocity commands to a differential-drive base.</summary>
    public sealed class ROSDifferentialCommand : MonoBehaviour, ICraneEpisodeResettable {
        private readonly struct Command {
            public readonly float Linear;
            public readonly float Angular;
            public Command(float linear, float angular) { Linear = linear; Angular = angular; }
        }

        private readonly CraneQueuedAction<Command> pending = new(command =>
            CraneActionPayloadEncoding.Float32Array(new[] { command.Linear, command.Angular }));
        private DifferentialDriveDynamics robot;
        private string topic;
        private long timeoutTicks;
        private long sequence;
        private long lastApplicationTick = -1;
        private bool stoppedForTimeout;
        public int ResetPriority => -40;

        public void Initialize(DifferentialDriveDynamics target, string commandTopic,
            long commandTimeoutTicks) {
            robot = target ?? throw new ArgumentNullException(nameof(target));
            topic = commandTopic;
            timeoutTicks = Math.Max(1, commandTimeoutTicks);
            var subscriber = gameObject.AddComponent<ROSSubscriber>();
            subscriber.Initialize<TwistStampedMsg>(topic, Receive);
            Debug.Log($"CRANE_ROS_DIFFERENTIAL_COMMAND_READY topic={topic} robot={robot.name} " +
                      $"timeoutTicks={timeoutTicks}");
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
            if (robot == null) return;
            if (pending.TryApply(command => robot.SetCommand(command.Linear, command.Angular), out _)) {
                lastApplicationTick = CraneRuntimeMetrics.SimulationTick;
                stoppedForTimeout = false;
            }
            else if (!stoppedForTimeout && lastApplicationTick >= 0 &&
                     CraneRuntimeMetrics.SimulationTick - lastApplicationTick > timeoutTicks) {
                robot.SetCommand(0f, 0f);
                stoppedForTimeout = true;
                CraneRuntimeMetrics.ReportCommandTimeout();
            }
        }

        public void CaptureEpisodeInitialState() { }
        public void ResetEpisode(in CraneEpisodeResetContext context,
            CraneEpisodeResetPhase phase) {
            if (phase != CraneEpisodeResetPhase.BeforePhysics) return;
            pending.Clear();
            sequence = 0;
            lastApplicationTick = -1;
            stoppedForTimeout = false;
            if (robot != null) robot.SetCommand(0f, 0f);
        }
    }

    internal static class ROSDifferentialCommandBootstrap {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            string[] args = Environment.GetCommandLineArgs();
            int flag = Array.IndexOf(args, "--crane-ros-differential-cmd-vel");
            if (flag < 0 || ROSPublisher.TransportSuppressed) return;
            string topic = flag + 1 < args.Length && !args[flag + 1].StartsWith("-")
                ? args[flag + 1] : "/crane/cmd_vel_stamped";
            long timeout = ReadLong(args, "--crane-command-timeout-ticks", 25);
            SceneManager.sceneLoaded += (_, _) => {
                DifferentialDriveDynamics robot = UnityEngine.Object.FindAnyObjectByType<
                    DifferentialDriveDynamics>(FindObjectsInactive.Exclude);
                if (robot == null) return;
                var adapter = robot.GetComponent<ROSDifferentialCommand>() ??
                              robot.gameObject.AddComponent<ROSDifferentialCommand>();
                adapter.Initialize(robot, topic, timeout);
            };
        }

        private static long ReadLong(string[] args, string key, long fallback) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length && long.TryParse(args[index + 1],
                NumberStyles.Integer, CultureInfo.InvariantCulture, out long value)
                ? value : fallback;
        }
    }
}
