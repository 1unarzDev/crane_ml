using System;
using System.Globalization;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using Sim.Utils;
using Sim.Utils.Performance;
using Sim.Utils.ROS;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Controllers {
    /// <summary>
    /// Publishes the authoritative vehicle pose and body-frame twist used by ROS navigation.
    /// This component is opt-in so existing scene-owned odometry publishers are not duplicated.
    /// </summary>
    public sealed class CraneROSNavigationState : MonoBehaviour, ICraneEpisodeResettable {
        private IPhysicsBody body;
        private Component bodyComponent;
        private ROSConnection ros;
        private string odometryTopic;
        private string transformTopic;
        private string odometryFrame;
        private string baseFrame;
        private double publishPeriod;
        private double nextPublishTime;

        public int ResetPriority => -80;

        public void Initialize(Component target, string odomTopic, string tfTopic,
            string parentFrame, string childFrame, float publishRateHz) {
            bodyComponent = target != null ? target : throw new ArgumentNullException(nameof(target));
            body = target switch {
                Rigidbody rigidbody => new RigidbodyAdapter(rigidbody),
                ArticulationBody articulation => new ArticulationBodyAdapter(articulation),
                _ => throw new ArgumentException("Navigation state requires a PhysX body.",
                    nameof(target))
            };
            odometryTopic = odomTopic;
            transformTopic = tfTopic;
            odometryFrame = parentFrame;
            baseFrame = childFrame;
            publishPeriod = 1.0 / Math.Max(0.1f, publishRateHz);
            nextPublishTime = 0;

            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<OdometryMsg>(odometryTopic);
            ros.RegisterPublisher<TFMessageMsg>(transformTopic);
            Debug.Log($"CRANE_ROS_NAV_STATE_READY body={bodyComponent.name} " +
                      $"bodyType={bodyComponent.GetType().Name} odom={odometryTopic} " +
                      $"tf={transformTopic} frames={odometryFrame}->{baseFrame} " +
                      $"rateHz={publishRateHz:R}");
        }

        public void CaptureEpisodeInitialState() { }

        public void ResetEpisode(in CraneEpisodeResetContext context,
            CraneEpisodeResetPhase phase) {
            if (phase == CraneEpisodeResetPhase.BeforePhysics) nextPublishTime = 0;
        }

        private void FixedUpdate() {
            if (body == null || ros == null) return;
            double simulationTime = Clock.time;
            if (simulationTime + 1e-9 < nextPublishTime) return;
            // Advance by periods instead of assigning time + period, retaining fractional sensor
            // cadence at rates that do not divide the fixed 0.02-second physics interval.
            do nextPublishTime += publishPeriod;
            while (nextPublishTime <= simulationTime + 1e-9);
            Publish(simulationTime);
        }

        private void Publish(double simulationTime) {
            HeaderMsg header = CreateHeader(simulationTime, odometryFrame);
            PointMsg position = ToRosPoint(body.position);
            QuaternionMsg orientation = body.rotation.To<FLU>();
            Vector3 localLinear = body.transform.InverseTransformDirection(body.linearVelocity);
            Vector3 localAngular = body.transform.InverseTransformDirection(body.angularVelocity);
            Vector3Msg linear = ToRosVector(localLinear);
            Vector3Msg angular = ToRosVector(localAngular);

            var odometry = new OdometryMsg {
                header = header,
                child_frame_id = baseFrame
            };
            odometry.pose.pose.position = position;
            odometry.pose.pose.orientation = orientation;
            odometry.twist.twist.linear = linear;
            odometry.twist.twist.angular = angular;

            var transform = new TransformStampedMsg {
                header = CreateHeader(simulationTime, odometryFrame),
                child_frame_id = baseFrame
            };
            transform.transform.translation = new Vector3Msg(position.x, position.y, position.z);
            transform.transform.rotation = orientation;

            ros.Publish(odometryTopic, odometry);
            ros.Publish(transformTopic, new TFMessageMsg(new[] { transform }));
        }

        internal static PointMsg ToRosPoint(Vector3 unity) =>
            new(unity.z, -unity.x, unity.y);

        internal static Vector3Msg ToRosVector(Vector3 unity) =>
            new(unity.z, -unity.x, unity.y);

        private static HeaderMsg CreateHeader(double seconds, string frame) {
            double integral = Math.Floor(Math.Max(0, seconds));
            uint nanoseconds = (uint)Math.Round((Math.Max(0, seconds) - integral) * 1e9);
            if (nanoseconds >= 1_000_000_000) {
                integral += 1;
                nanoseconds = 0;
            }
            return new HeaderMsg { frame_id = frame, stamp = new TimeMsg((int)integral, nanoseconds) };
        }
    }

    internal static class CraneROSNavigationStateBootstrap {
        private static bool enabled;
        private static string odometryTopic;
        private static string transformTopic;
        private static string odometryFrame;
        private static string baseFrame;
        private static float publishRate;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            string[] args = Environment.GetCommandLineArgs();
            enabled = Array.IndexOf(args, "--crane-ros-nav-state") >= 0 &&
                      Array.IndexOf(args, "--crane-disable-ros") < 0;
            if (!enabled) return;
            odometryTopic = ReadString(args, "--crane-ros-odom-topic", "/crane/odom");
            transformTopic = ReadString(args, "--crane-ros-tf-topic", "/tf");
            odometryFrame = ReadString(args, "--crane-ros-odom-frame", "odom");
            baseFrame = ReadString(args, "--crane-ros-base-frame", "base_link");
            publishRate = ReadFloat(args, "--crane-ros-nav-state-hz", 50f);
            SceneManager.sceneLoaded += OnSceneLoaded;
        }

        private static void OnSceneLoaded(Scene scene, LoadSceneMode mode) {
            if (!enabled) return;
            OmniXController controller = UnityEngine.Object.FindAnyObjectByType<OmniXController>(
                FindObjectsInactive.Exclude);
            Component body = null;
            if (controller != null) {
                body = controller.GetComponent<ArticulationBody>();
                body ??= controller.GetComponentInParent<ArticulationBody>();
                body ??= controller.GetComponent<Rigidbody>();
                body ??= controller.GetComponentInParent<Rigidbody>();
            }
            if (body == null) {
                Debug.LogWarning($"CRANE_ROS_NAV_STATE_UNAVAILABLE scene={scene.name} " +
                                 "reason=no-enabled-omni-x-rigidbody");
                return;
            }
            var state = body.GetComponent<CraneROSNavigationState>() ??
                        body.gameObject.AddComponent<CraneROSNavigationState>();
            state.Initialize(body, odometryTopic, transformTopic, odometryFrame, baseFrame,
                publishRate);
        }

        private static string ReadString(string[] args, string key, string fallback) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length && !args[index + 1].StartsWith("-")
                ? args[index + 1] : fallback;
        }

        private static float ReadFloat(string[] args, string key, float fallback) {
            string raw = ReadString(args, key, null);
            return float.TryParse(raw, NumberStyles.Float, CultureInfo.InvariantCulture,
                out float value) ? value : fallback;
        }
    }
}
