using System;
using System.Collections.Generic;
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
        private readonly List<Transform> childFrames = new();
        private string[] requestedChildFrameNames = Array.Empty<string>();

        public int ResetPriority => -80;

        public void Initialize(Component target, string odomTopic, string tfTopic,
            string parentFrame, string childFrame, float publishRateHz,
            IReadOnlyList<string> requestedChildFrames) {
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
            requestedChildFrameNames = requestedChildFrames == null
                ? Array.Empty<string>()
                : new List<string>(requestedChildFrames).ToArray();
            CacheChildFrames(requestedChildFrames);

            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<OdometryMsg>(odometryTopic);
            ros.RegisterPublisher<TFMessageMsg>(transformTopic);
            Debug.Log($"CRANE_ROS_NAV_STATE_READY body={bodyComponent.name} " +
                      $"bodyType={bodyComponent.GetType().Name} odom={odometryTopic} " +
                      $"tf={transformTopic} frames={odometryFrame}->{baseFrame} " +
                      $"rateHz={publishRateHz:R} childFrames={childFrames.Count}");
        }

        private void CacheChildFrames(IReadOnlyList<string> requestedFrames) {
            childFrames.Clear();
            if (requestedFrames == null || requestedFrames.Count == 0) return;
            Transform[] descendants = body.transform.GetComponentsInChildren<Transform>(true);
            foreach (string requested in requestedFrames) {
                Transform match = null;
                foreach (Transform candidate in descendants) {
                    if (candidate != body.transform && candidate.name == requested) {
                        match = candidate;
                        break;
                    }
                }
                if (match != null) childFrames.Add(match);
                else Debug.LogWarning($"CRANE_ROS_NAV_CHILD_FRAME_UNAVAILABLE " +
                                      $"body={bodyComponent.name} frame={requested}");
            }
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
            // Runtime fixtures may mount sensors from another sceneLoaded callback. Retry until
            // every requested static child exists instead of depending on callback order.
            if (childFrames.Count < requestedChildFrameNames.Length)
                CacheChildFrames(requestedChildFrameNames);
            HeaderMsg header = CreateHeader(simulationTime, odometryFrame);
            PointMsg position = ToRosPoint(body.position);
            QuaternionMsg orientation = body.rotation.To<FLU>();
            Vector3 localLinear = body.transform.InverseTransformDirection(body.linearVelocity);
            Vector3 localAngular = body.transform.InverseTransformDirection(body.angularVelocity);
            Vector3Msg linear = ToRosVector(localLinear);
            Vector3Msg angular = ToRosAngularVector(localAngular);

            var odometry = new OdometryMsg {
                header = header,
                child_frame_id = baseFrame
            };
            odometry.pose.pose.position = position;
            odometry.pose.pose.orientation = orientation;
            odometry.twist.twist.linear = linear;
            odometry.twist.twist.angular = angular;

            var rootTransform = new TransformStampedMsg {
                header = CreateHeader(simulationTime, odometryFrame),
                child_frame_id = baseFrame
            };
            rootTransform.transform.translation = new Vector3Msg(position.x, position.y, position.z);
            rootTransform.transform.rotation = orientation;

            var transforms = new TransformStampedMsg[childFrames.Count + 1];
            transforms[0] = rootTransform;
            for (int i = 0; i < childFrames.Count; i++) {
                Transform child = childFrames[i];
                Vector3 localPosition = body.transform.InverseTransformPoint(child.position);
                Quaternion localRotation = Quaternion.Inverse(body.rotation) * child.rotation;
                var childTransform = new TransformStampedMsg {
                    header = CreateHeader(simulationTime, baseFrame),
                    child_frame_id = child.name
                };
                childTransform.transform.translation = ToRosVector(localPosition);
                childTransform.transform.rotation = localRotation.To<FLU>();
                transforms[i + 1] = childTransform;
            }

            ros.Publish(odometryTopic, odometry);
            ros.Publish(transformTopic, new TFMessageMsg(transforms));
        }

        internal static PointMsg ToRosPoint(Vector3 unity) =>
            new(unity.z, -unity.x, unity.y);

        internal static Vector3Msg ToRosVector(Vector3 unity) =>
            new(unity.z, -unity.x, unity.y);

        // Unity-to-FLU is a handedness-changing reflection. Angular velocity is an axial vector,
        // so it gains the determinant sign that polar position/linear-velocity vectors do not.
        internal static Vector3Msg ToRosAngularVector(Vector3 unity) =>
            new(-unity.z, unity.x, -unity.y);

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
        private static string[] childFrames;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            string[] args = Environment.GetCommandLineArgs();
            enabled = Array.IndexOf(args, "--crane-ros-nav-state") >= 0 &&
                      !ROSPublisher.TransportSuppressed;
            if (!enabled) return;
            odometryTopic = ReadString(args, "--crane-ros-odom-topic", "/crane/odom");
            transformTopic = ReadString(args, "--crane-ros-tf-topic", "/tf");
            odometryFrame = ReadString(args, "--crane-ros-odom-frame", "odom");
            baseFrame = ReadString(args, "--crane-ros-base-frame", "base_link");
            publishRate = ReadFloat(args, "--crane-ros-nav-state-hz", 50f);
            string rawChildFrames = ReadString(args, "--crane-ros-nav-child-frames",
                "lidar_link,front_camera_link,imu_link,gps_link");
            childFrames = string.Equals(rawChildFrames, "none", StringComparison.OrdinalIgnoreCase)
                ? Array.Empty<string>()
                : rawChildFrames.Split(',', StringSplitOptions.RemoveEmptyEntries);
            for (int i = 0; i < childFrames.Length; i++) childFrames[i] = childFrames[i].Trim();
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
                foreach (MonoBehaviour candidate in UnityEngine.Object.FindObjectsByType<MonoBehaviour>(
                             FindObjectsInactive.Exclude)) {
                    string typeName = candidate.GetType().FullName;
                    if (typeName != "Sim.Physics.Land.AckermannRoverDynamics" &&
                        typeName != "Sim.Physics.Land.DifferentialDriveDynamics") continue;
                    body = candidate.GetComponent<Rigidbody>() ??
                           candidate.GetComponentInParent<Rigidbody>();
                    if (body != null) break;
                }
            }
            if (body == null) {
                Debug.LogWarning($"CRANE_ROS_NAV_STATE_UNAVAILABLE scene={scene.name} " +
                                 "reason=no-supported-enabled-physics-body");
                return;
            }
            var state = body.GetComponent<CraneROSNavigationState>() ??
                        body.gameObject.AddComponent<CraneROSNavigationState>();
            state.Initialize(body, odometryTopic, transformTopic, odometryFrame, baseFrame,
                publishRate, childFrames);
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
