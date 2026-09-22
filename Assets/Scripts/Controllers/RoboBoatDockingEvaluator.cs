using System;
using System.Globalization;
using RosMessageTypes.Std;
using Sim.Utils;
using Sim.Utils.Performance;
using Sim.Utils.ROS;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Controllers {
    [Serializable]
    internal sealed class RoboBoatDockingEvaluationMessage {
        public string schema = "crane-roboboat-docking-evaluation-v1";
        public double simulationTime;
        public float x;
        public float y;
        public float yaw;
        public float xyError;
        public float yawError;
        public float bodySpeed;
        public float bodyYawRate;
        public float longitudinalOffset;
        public float lateralOffset;
        public float longitudinalClearance;
        public float lateralClearance;
        public float minimumRegionClearance;
        public bool hullInsideDockRegion;
        public bool poseWithinTolerance;
        public bool stopped;
        public int prohibitedContactCount;
        public float continuousQualifiedSeconds;
        public float requiredSettleSeconds;
        public bool success;
    }

    /// <summary>
    /// Opt-in, command-independent physical docking predicate. The berth rectangle and hull
    /// envelope are explicit development-fixture inputs; no scene object or collider is changed.
    /// </summary>
    public sealed class RoboBoatDockingEvaluator : MonoBehaviour, ICraneEpisodeResettable {
        private IPhysicsBody body;
        private float goalX;
        private float goalY;
        private float goalYaw;
        private float xyTolerance;
        private float yawTolerance;
        private float speedTolerance;
        private float yawRateTolerance;
        private float settleSeconds;
        private float dockWidth;
        private float dockDepth;
        private float hullLength;
        private float hullBeam;
        private double qualifiedSince = -1;
        private int prohibitedContactCount;

        public int ResetPriority => -30;

        public void Initialize(OmniXController controller, string topic,
            float targetX, float targetY, float targetYaw,
            float positionTolerance, float headingTolerance,
            float stoppedSpeed, float stoppedYawRate, float requiredSettleSeconds,
            float regionWidth, float regionDepth, float physicalHullLength,
            float physicalHullBeam) {
            ArticulationBody articulation = controller.GetComponent<ArticulationBody>() ??
                                           controller.GetComponentInParent<ArticulationBody>();
            Rigidbody rigidbody = controller.GetComponent<Rigidbody>() ??
                                  controller.GetComponentInParent<Rigidbody>();
            body = articulation != null ? new ArticulationBodyAdapter(articulation) :
                   rigidbody != null ? new RigidbodyAdapter(rigidbody) :
                   throw new MissingComponentException("Dock evaluator requires a physics body");
            goalX = targetX;
            goalY = targetY;
            goalYaw = targetYaw;
            xyTolerance = Mathf.Max(0f, positionTolerance);
            yawTolerance = Mathf.Max(0f, headingTolerance);
            speedTolerance = Mathf.Max(0f, stoppedSpeed);
            yawRateTolerance = Mathf.Max(0f, stoppedYawRate);
            settleSeconds = Mathf.Max(0f, requiredSettleSeconds);
            dockWidth = Mathf.Max(0.01f, regionWidth);
            dockDepth = Mathf.Max(0.01f, regionDepth);
            hullLength = Mathf.Max(0.01f, physicalHullLength);
            hullBeam = Mathf.Max(0.01f, physicalHullBeam);

            foreach (Collider collider in controller.GetComponentsInChildren<Collider>(true)) {
                var probe = collider.gameObject.GetComponent<RoboBoatDockContactProbe>() ??
                            collider.gameObject.AddComponent<RoboBoatDockContactProbe>();
                probe.Owner = this;
            }
            var publisher = gameObject.AddComponent<ROSPublisher>();
            publisher.Initialize<StringMsg>(topic, "odom", CreateMessage, 10f);
            Debug.Log($"CRANE_ROBOBOAT_DOCK_EVALUATOR_READY topic={topic} " +
                      $"goal=({goalX:R},{goalY:R},{goalYaw:R}) region=({dockDepth:R},{dockWidth:R}) " +
                      $"hull=({hullLength:R},{hullBeam:R}) settle={settleSeconds:R}");
        }

        public void CaptureEpisodeInitialState() { }

        public void ResetEpisode(in CraneEpisodeResetContext context,
            CraneEpisodeResetPhase phase) {
            if (phase != CraneEpisodeResetPhase.BeforePhysics) return;
            qualifiedSince = -1;
            prohibitedContactCount = 0;
        }

        internal void ReportContact(Collision collision) {
            if (!IsMarina(collision.gameObject.transform)) return;
            prohibitedContactCount++;
            Debug.LogWarning($"CRANE_ROBOBOAT_DOCK_CONTACT count={prohibitedContactCount} " +
                             $"other={collision.gameObject.name}");
        }

        private static bool IsMarina(Transform value) {
            for (Transform current = value; current != null; current = current.parent)
                if (current.name.IndexOf("Marina", StringComparison.OrdinalIgnoreCase) >= 0 ||
                    current.name.IndexOf("Dock", StringComparison.OrdinalIgnoreCase) >= 0)
                    return true;
            return false;
        }

        private StringMsg CreateMessage() {
            Vector3 position = body.position;
            Quaternion frameRotation = RoboBoatRosFrame.Rotation(body.rotation);
            Vector3 worldForward = frameRotation * Vector3.forward;
            float x = position.z;
            float y = -position.x;
            float yaw = Mathf.Atan2(-worldForward.x, worldForward.z);
            Vector3 localLinear = RoboBoatRosFrame.ToRosLocalUnity(
                body.transform.InverseTransformDirection(body.linearVelocity));
            Vector3 localAngular = RoboBoatRosFrame.ToRosLocalUnity(
                body.transform.InverseTransformDirection(body.angularVelocity));
            float surge = localLinear.z;
            float sway = -localLinear.x;
            float yawRate = -localAngular.y;
            RoboBoatDockingEvaluationMessage evaluation = Evaluate(
                x, y, yaw, surge, sway, yawRate, Time.fixedTimeAsDouble,
                prohibitedContactCount, ref qualifiedSince,
                goalX, goalY, goalYaw, xyTolerance, yawTolerance,
                speedTolerance, yawRateTolerance, settleSeconds,
                dockWidth, dockDepth, hullLength, hullBeam);
            return new StringMsg(JsonUtility.ToJson(evaluation));
        }

        internal static RoboBoatDockingEvaluationMessage Evaluate(
            float x, float y, float yaw, float surge, float sway, float yawRate,
            double simulationTime, int contactCount, ref double qualifiedSince,
            float targetX, float targetY, float targetYaw,
            float positionTolerance, float headingTolerance,
            float stoppedSpeed, float stoppedYawRate, float requiredSettleSeconds,
            float regionWidth, float regionDepth, float physicalHullLength,
            float physicalHullBeam) {
            float dx = x - targetX;
            float dy = y - targetY;
            float cos = Mathf.Cos(targetYaw);
            float sin = Mathf.Sin(targetYaw);
            float longitudinal = cos * dx + sin * dy;
            float lateral = -sin * dx + cos * dy;
            float yawError = Mathf.DeltaAngle(targetYaw * Mathf.Rad2Deg,
                                              yaw * Mathf.Rad2Deg) * Mathf.Deg2Rad;
            float errorCos = Mathf.Abs(Mathf.Cos(yawError));
            float errorSin = Mathf.Abs(Mathf.Sin(yawError));
            float halfAlong = errorCos * physicalHullLength * 0.5f +
                              errorSin * physicalHullBeam * 0.5f;
            float halfAcross = errorSin * physicalHullLength * 0.5f +
                               errorCos * physicalHullBeam * 0.5f;
            float longitudinalClearance = regionDepth * 0.5f -
                                          (Mathf.Abs(longitudinal) + halfAlong);
            float lateralClearance = regionWidth * 0.5f -
                                     (Mathf.Abs(lateral) + halfAcross);
            bool inside = longitudinalClearance >= 0f && lateralClearance >= 0f;
            float xyError = Mathf.Sqrt(dx * dx + dy * dy);
            float speed = Mathf.Sqrt(surge * surge + sway * sway);
            bool poseValid = xyError <= positionTolerance &&
                             Mathf.Abs(yawError) <= headingTolerance;
            bool stopped = speed <= stoppedSpeed && Mathf.Abs(yawRate) <= stoppedYawRate;
            bool qualified = inside && poseValid && stopped && contactCount == 0;
            if (qualified) {
                if (qualifiedSince < 0) qualifiedSince = simulationTime;
            }
            else qualifiedSince = -1;
            float held = qualifiedSince < 0 ? 0f :
                (float)Math.Max(0.0, simulationTime - qualifiedSince);
            return new RoboBoatDockingEvaluationMessage {
                simulationTime = simulationTime,
                x = x,
                y = y,
                yaw = yaw,
                xyError = xyError,
                yawError = yawError,
                bodySpeed = speed,
                bodyYawRate = yawRate,
                longitudinalOffset = longitudinal,
                lateralOffset = lateral,
                longitudinalClearance = longitudinalClearance,
                lateralClearance = lateralClearance,
                minimumRegionClearance = Mathf.Min(longitudinalClearance, lateralClearance),
                hullInsideDockRegion = inside,
                poseWithinTolerance = poseValid,
                stopped = stopped,
                prohibitedContactCount = contactCount,
                continuousQualifiedSeconds = held,
                requiredSettleSeconds = requiredSettleSeconds,
                success = qualified && held >= requiredSettleSeconds,
            };
        }
    }

    internal sealed class RoboBoatDockContactProbe : MonoBehaviour {
        internal RoboBoatDockingEvaluator Owner { get; set; }
        private void OnCollisionEnter(Collision collision) => Owner?.ReportContact(collision);
    }

    internal static class RoboBoatDockingEvaluatorBootstrap {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            string[] args = Environment.GetCommandLineArgs();
            if (Array.IndexOf(args, "--crane-roboboat-docking-evaluator") < 0 ||
                ROSPublisher.TransportSuppressed) return;
            SceneManager.sceneLoaded += (_, _) => Configure(args);
        }

        private static void Configure(string[] args) {
            OmniXController controller = UnityEngine.Object.FindAnyObjectByType<OmniXController>(
                FindObjectsInactive.Exclude);
            if (controller == null) {
                Debug.LogWarning("CRANE_ROBOBOAT_DOCK_EVALUATOR_UNAVAILABLE reason=no-controller");
                return;
            }
            var evaluator = controller.gameObject.GetComponent<RoboBoatDockingEvaluator>() ??
                            controller.gameObject.AddComponent<RoboBoatDockingEvaluator>();
            evaluator.Initialize(controller,
                ReadString(args, "--crane-dock-evaluator-topic", "/crane/docking_evaluator"),
                ReadFloat(args, "--crane-dock-goal-x", 0f),
                ReadFloat(args, "--crane-dock-goal-y", 0f),
                ReadFloat(args, "--crane-dock-goal-yaw", 0f),
                ReadFloat(args, "--crane-dock-xy-tolerance", 0.4f),
                ReadFloat(args, "--crane-dock-yaw-tolerance", 0.35f),
                ReadFloat(args, "--crane-dock-speed-tolerance", 0.05f),
                ReadFloat(args, "--crane-dock-yaw-rate-tolerance", 0.05f),
                ReadFloat(args, "--crane-dock-settle-seconds", 5f),
                ReadFloat(args, "--crane-dock-width", 2f),
                ReadFloat(args, "--crane-dock-depth", 3f),
                ReadFloat(args, "--crane-dock-hull-length", 1.063f),
                ReadFloat(args, "--crane-dock-hull-beam", 0.895f));
        }

        private static string ReadString(string[] args, string key, string fallback) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length ? args[index + 1] : fallback;
        }

        private static float ReadFloat(string[] args, string key, float fallback) =>
            float.TryParse(ReadString(args, key, null), NumberStyles.Float,
                CultureInfo.InvariantCulture, out float value) ? value : fallback;
    }
}
