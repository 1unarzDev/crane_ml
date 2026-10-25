using System;
using System.Globalization;
using System.Threading;
using RosMessageTypes.Std;
using Sim.Actuators.Motors;
using Sim.Utils.Performance;
using Sim.Utils.ROS;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Controllers {
    /// <summary>
    /// Drives the four RoboBoat thrusters from per-output ESC pulse widths, the same signal the
    /// boat's ODROID puts on its PWM pins. mhseals_hardware's thruster_pwm_node (output_backend
    /// topic) publishes it after its mixer, channel map and calibration, so the simulator tests
    /// the boat's real allocation instead of the simulator's own mixer. Transport callbacks only
    /// enqueue; thrusters change in FixedUpdate.
    /// </summary>
    public sealed class ROSThrusterPwmCommand : MonoBehaviour, ICraneEpisodeResettable {
        private readonly CraneQueuedAction<ushort[]> pending =
            new(CraneActionPayloadEncoding.UInt16Array);
        private OmniXController controller;
        private Thruster[] byOutput;
        private string topic;
        private float deadbandPulse;
        private long sequence;
        private long lastApplicationTick = -1;
        private long timeoutTicks;
        private bool stoppedForTimeout;
        // Plant variant: pulses reach the thrusters this many physics ticks late (0 = at once).
        internal static int DelayTicks;
        private readonly System.Collections.Generic.Queue<ushort[]> delayed = new();
        private ushort[] held;

        public int ResetPriority => -40;

        /// <param name="outputs">Physical output number (1-4) wired to FL, FR, RR, RL.</param>
        public void Initialize(OmniXController target, string pwmTopic, int[] outputs,
            float deadband, long commandTimeoutTicks) {
            controller = target ?? throw new ArgumentNullException(nameof(target));
            byOutput = ThrustersByOutput(controller, outputs);
            topic = pwmTopic;
            deadbandPulse = Mathf.Max(0f, deadband);
            timeoutTicks = Math.Max(1, commandTimeoutTicks);
            var subscriber = gameObject.AddComponent<ROSSubscriber>();
            subscriber.Initialize<UInt16MultiArrayMsg>(topic, Receive);
            Debug.Log($"CRANE_ROS_PWM_READY topic={topic} controller={controller.name} " +
                      $"outputs={string.Join(",", outputs)} deadband={deadbandPulse:R} " +
                      $"timeoutTicks={timeoutTicks}");
        }

        /// <summary>Thruster driven by each physical output, index 0 = output 1.</summary>
        internal static Thruster[] ThrustersByOutput(OmniXController controller, int[] outputs) {
            if (outputs == null || outputs.Length != 4)
                throw new ArgumentException("Exactly four physical outputs (FL,FR,RR,RL) are required.");
            Thruster[] positions = {
                controller.frontLeft, controller.frontRight, controller.rearRight, controller.rearLeft
            };
            var result = new Thruster[4];
            for (int position = 0; position < 4; position++) {
                int output = outputs[position];
                if (output < 1 || output > 4 || result[output - 1] != null)
                    throw new ArgumentException(
                        "Physical outputs must be a permutation of 1,2,3,4.");
                result[output - 1] = positions[position];
            }
            return result;
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

        private void Receive(UInt16MultiArrayMsg message) {
            if (message?.data == null || message.data.Length != 4) return;
            pending.Receive((ushort[])message.data.Clone(), $"ros:{topic}",
                Interlocked.Increment(ref sequence));
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
                // The publisher repeats at 20 Hz, so silence means a lost link or a dead node:
                // fail neutral like the boat's ESCs without a signal.
                Stop();
                stoppedForTimeout = true;
                CraneRuntimeMetrics.ReportCommandTimeout();
                Debug.LogWarning($"CRANE_ROS_PWM_TIMEOUT topic={topic} " +
                                 $"lastApplicationTick={lastApplicationTick}");
            }
            if (DelayTicks > 0 && held != null) {
                delayed.Enqueue(held);
                while (delayed.Count > DelayTicks)
                    ApplyPulses(byOutput, delayed.Dequeue(), controller.config.GetMaxCommand(), deadbandPulse);
            }
        }

        private void Apply(ushort[] pulses) {
            if (DelayTicks > 0) {
                held = pulses;
                return;
            }
            ApplyPulses(byOutput, pulses, controller.config.GetMaxCommand(), deadbandPulse);
        }

        internal static void ApplyPulses(Thruster[] byOutput, ushort[] pulses, float maximum,
            float deadband = 0f) {
            for (int output = 0; output < 4; output++)
                byOutput[output].SetCommand(EscPwm.Bidirectional(pulses[output], maximum,
                    deadbandPulse: deadband));
        }

        private void Stop() {
            held = null;
            delayed.Clear();
            if (byOutput == null) return;
            foreach (Thruster thruster in byOutput) thruster.SetCommand(0f);
        }
    }

    internal static class ROSThrusterPwmCommandBootstrap {
        internal const string Flag = "--crane-ros-thruster-pwm";
        private static string topic;
        private static int[] outputs;
        private static float deadband;
        private static long timeoutTicks;
        private static bool enabled;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Initialize() {
            string[] args = Environment.GetCommandLineArgs();
            int flag = Array.IndexOf(args, Flag);
            enabled = flag >= 0 && !ROSPublisher.TransportSuppressed;
            if (!enabled) return;
            string conflict = Conflict(args);
            if (conflict != null) {
                Debug.LogError($"CRANE_ROS_PWM_UNAVAILABLE reason={conflict}");
                enabled = false;
                return;
            }
            topic = flag + 1 < args.Length && !args[flag + 1].StartsWith("-") ?
                args[flag + 1] : "/thrusters/pwm";
            outputs = ParseOutputs(ReadString(args, "--crane-pwm-outputs", "1,2,3,4"));
            deadband = ReadFloat(args, "--crane-pwm-deadband", 0f);
            timeoutTicks = ReadLong(args, "--crane-command-timeout-ticks", 25);
            SceneManager.sceneLoaded += OnSceneLoaded;
        }

        /// <summary>
        /// Two thruster sources would fight every tick, so the PWM input refuses to start beside
        /// the velocity input or the ArduPilot SITL bridge (which applies its own servo outputs).
        /// </summary>
        internal static string Conflict(string[] args) {
            if (Array.IndexOf(args, "--crane-ros-cmd-vel") >= 0)
                return "--crane-ros-cmd-vel-also-drives-the-thrusters";
            if (Array.IndexOf(args, "--crane-disable-sitl") < 0)
                return "sitl-bridge-enabled;pass---crane-disable-sitl";
            return null;
        }

        internal static int[] ParseOutputs(string raw) {
            string[] parts = raw.Split(',');
            var result = new int[parts.Length];
            for (int i = 0; i < parts.Length; i++)
                result[i] = int.Parse(parts[i].Trim(), NumberStyles.Integer,
                    CultureInfo.InvariantCulture);
            return result;
        }

        private static void OnSceneLoaded(Scene scene, LoadSceneMode mode) {
            if (!enabled) return;
            OmniXController controller = UnityEngine.Object.FindAnyObjectByType<OmniXController>(
                FindObjectsInactive.Exclude);
            if (controller == null) {
                Debug.LogWarning($"CRANE_ROS_PWM_UNAVAILABLE scene={scene.name} " +
                                 "reason=no-enabled-omni-x-controller");
                return;
            }
            var adapter = controller.gameObject.GetComponent<ROSThrusterPwmCommand>() ??
                controller.gameObject.AddComponent<ROSThrusterPwmCommand>();
            adapter.Initialize(controller, topic, outputs, deadband, timeoutTicks);
        }

        private static string ReadString(string[] args, string key, string fallback) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length && !args[index + 1].StartsWith("-")
                ? args[index + 1] : fallback;
        }

        private static float ReadFloat(string[] args, string key, float fallback) =>
            float.TryParse(ReadString(args, key, null), NumberStyles.Float,
                CultureInfo.InvariantCulture, out float value) ? value : fallback;

        private static long ReadLong(string[] args, string key, long fallback) =>
            long.TryParse(ReadString(args, key, null), NumberStyles.Integer,
                CultureInfo.InvariantCulture, out long value) ? value : fallback;
    }
}
