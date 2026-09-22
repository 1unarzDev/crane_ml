using System;
using System.Globalization;
using System.Threading;
using RosMessageTypes.Geometry;
using Sim.Utils;
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
        private IPhysicsBody body;
        private string topic;
        private float linearScale;
        private float yawScale;
        private float linearFeedForward;
        private float yawFeedForward;
        private float linearProportionalGain;
        private float yawProportionalGain;
        private float linearIntegralGain;
        private float linearIntegralEffortLimit;
        private float forwardIntegralEffort;
        private float lateralIntegralEffort;
        private float previousForwardDesired;
        private float previousLateralDesired;
        private Command desiredVelocity;
        private bool hasDesiredVelocity;
        private long sequence;
        private long lastApplicationTick = -1;
        private long timeoutTicks;
        private bool stoppedForTimeout;

        public int ResetPriority => -40;

        public void Initialize(OmniXController target, string commandTopic,
            float maximumLinearVelocity, float maximumYawRate, long commandTimeoutTicks,
            float linearVelocityFeedForward, float yawRateFeedForward,
            float linearVelocityProportionalGain, float yawRateProportionalGain,
            float linearVelocityIntegralGain, float linearVelocityIntegralEffortLimit) {
            controller = target ?? throw new ArgumentNullException(nameof(target));
            var articulation = controller.GetComponent<ArticulationBody>() ??
                               controller.GetComponentInParent<ArticulationBody>();
            var rigidbody = controller.GetComponent<Rigidbody>() ??
                            controller.GetComponentInParent<Rigidbody>();
            body = articulation != null ? new ArticulationBodyAdapter(articulation) :
                   rigidbody != null ? new RigidbodyAdapter(rigidbody) :
                   throw new MissingComponentException(
                       $"{controller.name} requires a physics body for velocity feedback");
            topic = commandTopic;
            linearScale = Mathf.Max(0.0001f, maximumLinearVelocity);
            yawScale = Mathf.Max(0.0001f, maximumYawRate);
            linearFeedForward = Mathf.Max(0f, linearVelocityFeedForward);
            yawFeedForward = Mathf.Max(0f, yawRateFeedForward);
            linearProportionalGain = Mathf.Max(0f, linearVelocityProportionalGain);
            yawProportionalGain = Mathf.Max(0f, yawRateProportionalGain);
            linearIntegralGain = Mathf.Max(0f, linearVelocityIntegralGain);
            linearIntegralEffortLimit = Mathf.Clamp01(linearVelocityIntegralEffortLimit);
            timeoutTicks = Math.Max(1, commandTimeoutTicks);
            var subscriber = gameObject.AddComponent<ROSSubscriber>();
            subscriber.Initialize<TwistStampedMsg>(topic, Receive);
            Debug.Log($"CRANE_ROS_COMMAND_READY topic={topic} controller={controller.name} " +
                      $"linearScale={linearScale:R} yawScale={yawScale:R} " +
                      $"linearFeedForward={linearFeedForward:R} " +
                      $"yawFeedForward={yawFeedForward:R} " +
                      $"linearKp={linearProportionalGain:R} yawKp={yawProportionalGain:R} " +
                      $"linearKi={linearIntegralGain:R} " +
                      $"linearIntegralLimit={linearIntegralEffortLimit:R} " +
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
            ResetLinearIntegral();
            Stop();
        }

        private void Receive(TwistStampedMsg message) {
            if (message?.twist == null || message.header?.stamp == null) return;
            double seconds = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9;
            long sourceTick = seconds < 0 ? -1 :
                (long)Math.Round(seconds / Time.fixedDeltaTime, MidpointRounding.AwayFromZero);
            var command = new Command(
                Mathf.Clamp((float)message.twist.linear.x, -linearScale, linearScale),
                Mathf.Clamp((float)message.twist.linear.y, -linearScale, linearScale),
                Mathf.Clamp((float)message.twist.angular.z, -yawScale, yawScale));
            pending.Receive(command, $"ros:{topic}", Interlocked.Increment(ref sequence),
                sourceTick);
        }

        private void FixedUpdate() {
            if (controller == null) return;
            if (controller.movementOverride) {
                pending.Clear();
                hasDesiredVelocity = false;
                ResetLinearIntegral();
                return;
            }
            if (pending.TryApply(SetDesiredVelocity, out _)) {
                lastApplicationTick = CraneRuntimeMetrics.SimulationTick;
                stoppedForTimeout = false;
            }
            else if (!stoppedForTimeout && lastApplicationTick >= 0 &&
                     CraneRuntimeMetrics.SimulationTick - lastApplicationTick > timeoutTicks) {
                Stop();
                stoppedForTimeout = true;
                CraneRuntimeMetrics.ReportCommandTimeout();
                Debug.LogWarning($"CRANE_ROS_COMMAND_TIMEOUT topic={topic} " +
                                 $"lastApplicationTick={lastApplicationTick}");
            }
            if (hasDesiredVelocity) ApplyVelocityControl();
        }

        private void SetDesiredVelocity(Command command) {
            desiredVelocity = command;
            hasDesiredVelocity = true;
        }

        private void ApplyVelocityControl() {
            Vector3 localLinear = RoboBoatRosFrame.ToRosLocalUnity(
                body.transform.InverseTransformDirection(body.linearVelocity));
            Vector3 localAngular = RoboBoatRosFrame.ToRosLocalUnity(
                body.transform.InverseTransformDirection(body.angularVelocity));
            float measuredForward = localLinear.z;
            float measuredLateral = -localLinear.x;
            float measuredYaw = -localAngular.y;
            UpdateNormalizedLinearIntegral(desiredVelocity.Forward, measuredForward, linearScale,
                linearIntegralGain, linearIntegralEffortLimit, Time.fixedDeltaTime,
                ref forwardIntegralEffort, ref previousForwardDesired);
            UpdateNormalizedLinearIntegral(desiredVelocity.Lateral, measuredLateral, linearScale,
                linearIntegralGain, linearIntegralEffortLimit, Time.fixedDeltaTime,
                ref lateralIntegralEffort, ref previousLateralDesired);
            var effort = new Command(
                Mathf.Clamp(CalculateNormalizedLinearEffort(
                    desiredVelocity.Forward, measuredForward, linearScale,
                    linearFeedForward, linearProportionalGain) + forwardIntegralEffort, -1f, 1f),
                Mathf.Clamp(CalculateNormalizedLinearEffort(
                    desiredVelocity.Lateral, measuredLateral, linearScale,
                    linearFeedForward, linearProportionalGain) + lateralIntegralEffort, -1f, 1f),
                CalculateNormalizedYawEffort(desiredVelocity.Yaw, measuredYaw, yawScale,
                    yawFeedForward, yawProportionalGain));
            ToControllerMotion(effort.Forward, effort.Lateral, effort.Yaw,
                out Vector3 linear, out Vector3 angular);
            controller.SetMotion(linear, angular);
        }

        private void Stop() {
            desiredVelocity = new Command(0f, 0f, 0f);
            hasDesiredVelocity = false;
            ResetLinearIntegral();
            controller.SetMotion(Vector3.zero, Vector3.zero);
        }

        private void ResetLinearIntegral() {
            forwardIntegralEffort = 0f;
            lateralIntegralEffort = 0f;
            previousForwardDesired = 0f;
            previousLateralDesired = 0f;
        }

        internal static float CalculateNormalizedLinearEffort(float desired, float measured,
            float scale, float feedForward, float proportionalGain) {
            float normalizedDesired = desired / Mathf.Max(0.0001f, scale);
            float normalizedError = (desired - measured) / Mathf.Max(0.0001f, scale);
            float modelEffort = feedForward * normalizedDesired;
            return Mathf.Clamp(modelEffort + proportionalGain * normalizedError, -1f, 1f);
        }

        internal static float CalculateNormalizedYawEffort(float desired, float measured,
            float scale, float feedForward, float proportionalGain) {
            float normalizedDesired = desired / Mathf.Max(0.0001f, scale);
            float normalizedError = (desired - measured) / Mathf.Max(0.0001f, scale);
            float modelEffort = Mathf.Sign(normalizedDesired) * feedForward *
                                Mathf.Sqrt(Mathf.Abs(normalizedDesired));
            return Mathf.Clamp(modelEffort + proportionalGain * normalizedError, -1f, 1f);
        }

        internal static void UpdateNormalizedLinearIntegral(float desired, float measured,
            float scale, float integralGain, float integralEffortLimit, float deltaTime,
            ref float integralEffort, ref float previousDesired) {
            if (Mathf.Abs(desired) < 0.0001f || desired * previousDesired < 0f) {
                integralEffort = 0f;
            }
            else {
                float normalizedError = (desired - measured) / Mathf.Max(0.0001f, scale);
                integralEffort = Mathf.Clamp(
                    integralEffort + integralGain * normalizedError * Mathf.Max(0f, deltaTime),
                    -Mathf.Abs(integralEffortLimit), Mathf.Abs(integralEffortLimit));
            }
            previousDesired = desired;
        }

        internal static void ToControllerMotion(float forward, float lateral, float yaw,
            out Vector3 linear, out Vector3 angular) {
            // The imported catamaran's visible bow is body-local -X (the chase camera is aft at
            // +X), while OmniX controller X realizes body-local +Z and controller Y realizes
            // body-local -X. Keep this correction at the ROS boundary so manual input, the mixer,
            // and the physical model retain their established contract.
            linear = new Vector3(-lateral, forward, 0f);
            angular = new Vector3(0f, 0f, -yaw);
        }
    }

    /// <summary>
    /// Defines the ROS FLU body frame from the imported RoboBoat physics transform. The model's
    /// visible bow is physics-local -X, so the ROS-frame Unity-forward axis is rotated -90 degrees
    /// about Unity up relative to the ArticulationBody transform.
    /// </summary>
    internal static class RoboBoatRosFrame {
        private static readonly Quaternion BodyFromRos = Quaternion.Euler(0f, -90f, 0f);
        private static readonly Quaternion RosFromBody = Quaternion.Inverse(BodyFromRos);

        internal static Quaternion Rotation(Quaternion bodyRotation) =>
            bodyRotation * BodyFromRos;

        internal static Vector3 ToRosLocalUnity(Vector3 bodyLocal) =>
            RosFromBody * bodyLocal;

        internal static Vector3 WorldPointToRosLocal(Vector3 bodyPosition,
            Quaternion bodyRotation, Vector3 worldPoint) =>
            Quaternion.Inverse(Rotation(bodyRotation)) * (worldPoint - bodyPosition);
    }

    internal static class ROSOmniXCommandBootstrap {
        private static string topic;
        private static float linearScale;
        private static float yawScale;
        private static long timeoutTicks;
        private static float linearFeedForward;
        private static float yawFeedForward;
        private static float linearProportionalGain;
        private static float yawProportionalGain;
        private static float linearIntegralGain;
        private static float linearIntegralEffortLimit;
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
            linearFeedForward = ReadFloat(args, "--crane-cmd-vel-linear-feed-forward", 0.21f);
            yawFeedForward = ReadFloat(args, "--crane-cmd-vel-yaw-feed-forward", 0.24f);
            linearProportionalGain = ReadFloat(args, "--crane-cmd-vel-linear-kp", 0.1f);
            yawProportionalGain = ReadFloat(args, "--crane-cmd-vel-yaw-kp", 0.05f);
            linearIntegralGain = ReadFloat(args, "--crane-cmd-vel-linear-ki", 0.1f);
            linearIntegralEffortLimit = ReadFloat(
                args, "--crane-cmd-vel-linear-integral-limit", 0.2f);
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
            adapter.Initialize(controller, topic, linearScale, yawScale, timeoutTicks,
                linearFeedForward, yawFeedForward, linearProportionalGain,
                yawProportionalGain, linearIntegralGain, linearIntegralEffortLimit);
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
