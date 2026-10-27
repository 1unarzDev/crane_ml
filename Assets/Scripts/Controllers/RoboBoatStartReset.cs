using System.Collections.Generic;
using System.Globalization;
using System.Threading.Tasks;
using RosMessageTypes.Std;
using Sim.Physics.Core;
using Sim.Utils.ROS;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace Sim.Controllers {
    /// <summary>
    /// Sim-only /crane/reset_to_start (std_srvs/Trigger): puts the boat back on the course
    /// truth start pose (startX, startY, startYaw: the pose the course was laid out from), at
    /// rest, so current drift before a mission starts cannot move the start. Added by the course
    /// bootstrap once a course is built, so a real stack has no such service. The response
    /// message carries the resulting world-ROS pose of /crane/odom (the course truth frame).
    /// A request is served at the start of the next physics tick, before the hull's submersion
    /// and plant read the body, so the whole tick sees the boat at the start.
    /// </summary>
    [DefaultExecutionOrder(-1000)]
    public sealed class RoboBoatStartReset : MonoBehaviour {
        internal const string Service = "/crane/reset_to_start";
        internal const string ProbeFlag = "--crane-roboboat-reset-probe";

        private Transform body;
        private Vector2 start;
        private float startYaw;
        private Vector3 startPosition;
        private Quaternion startRotation;
        private readonly List<TaskCompletionSource<TriggerResponse>> pending = new();

        internal Vector2 StartPosition => start;
        internal float StartYaw => startYaw;
        internal Transform Body => body;

        /// <summary>Records the boat body and the course start pose (world-ROS position and
        /// yaw). The body keeps its height, roll and pitch; only its position and heading are
        /// moved onto the start.</summary>
        public void Capture(Transform boatBody, Vector2 courseStart, float courseStartYaw) {
            start = courseStart;
            startYaw = courseStartYaw;
            body = boatBody;
            if (body == null) return;
            startPosition = RoboBoatCourse.RosToUnity(start, body.position.y);
            float yaw = RoboBoatCourse.RosYawOfUnityDirection(
                RoboBoatRosFrame.Rotation(body.rotation) * Vector3.forward);
            // ROS yaw is counter-clockwise; a Unity turn about +y is clockwise.
            startRotation = Quaternion.AngleAxis(-(startYaw - yaw) * Mathf.Rad2Deg, Vector3.up) *
                            body.rotation;
        }

        private void Start() {
            // --crane-robot spawns its boat after the course is built: capture it now instead.
            if (body == null) Capture(FindBoatBody(), start, startYaw);
            if (System.Array.IndexOf(System.Environment.GetCommandLineArgs(), ProbeFlag) >= 0)
                gameObject.AddComponent<RoboBoatStartResetProbe>();
            if (ROSPublisher.TransportSuppressed) return;
            ROSConnection.GetOrCreateInstance()
                .ImplementService<TriggerRequest, TriggerResponse>(Service, _ => Request());
        }

        /// <summary>Queues a reset for the start of the next physics tick (a service call
        /// arrives in Update, between ticks).</summary>
        internal Task<TriggerResponse> Request() {
            var reply = new TaskCompletionSource<TriggerResponse>();
            pending.Add(reply);
            return reply.Task;
        }

        private void FixedUpdate() {
            if (pending.Count == 0) return;
            TriggerResponse response = Handle();
            foreach (TaskCompletionSource<TriggerResponse> reply in pending) reply.TrySetResult(response);
            pending.Clear();
        }

        private void OnDestroy() {
            foreach (TaskCompletionSource<TriggerResponse> reply in pending)
                reply.TrySetResult(new TriggerResponse(false, "reset_to_start went away before the next physics tick"));
            pending.Clear();
        }

        internal TriggerResponse Handle() {
            if (body == null)
                return new TriggerResponse(false, "no boat body to reset");
            Teleport(body, startPosition, startRotation);
            foreach (PlantBase plant in body.GetComponentsInChildren<PlantBase>()) plant.ResetDynamics();
            string pose = DescribeRosPose(body.position, body.rotation);
            Debug.Log($"CRANE_ROBOBOAT_RESET_TO_START {pose}");
            return new TriggerResponse(true, pose);
        }

        /// <summary>Moves a boat root to a pose with zero linear, angular and joint velocity.
        /// Articulations use TeleportRoot, as the episode reset coordinator does. TeleportRoot
        /// moves only the physics pose until the next step, so the transform is moved with it:
        /// the hull's submersion, body frame and plant read the transform, and a stale one put
        /// the buoyancy centre at the old pose (a lever arm of the whole drift) and blew up.</summary>
        internal static void Teleport(Transform body, Vector3 position, Quaternion rotation) {
            if (body.TryGetComponent(out ArticulationBody articulation) && articulation.isRoot) {
                articulation.TeleportRoot(position, rotation);
                body.SetPositionAndRotation(position, rotation);
                foreach (ArticulationBody link in body.GetComponentsInChildren<ArticulationBody>()) {
                    link.linearVelocity = Vector3.zero;
                    link.angularVelocity = Vector3.zero;
                    ArticulationReducedSpace still = link.jointVelocity;
                    for (int i = 0; i < still.dofCount; i++) still[i] = 0f;
                    link.jointVelocity = still;
                    link.WakeUp();
                }
            }
            else if (body.TryGetComponent(out Rigidbody rigidbody)) {
                rigidbody.position = position;
                rigidbody.rotation = rotation;
                body.SetPositionAndRotation(position, rotation);
                rigidbody.linearVelocity = Vector3.zero;
                rigidbody.angularVelocity = Vector3.zero;
            }
            else body.SetPositionAndRotation(position, rotation);
            UnityEngine.Physics.SyncTransforms();
        }

        /// <summary>"x=.. y=.. yaw=.." in the world-ROS frame of /crane/odom (metres, radians).
        /// </summary>
        internal static string DescribeRosPose(Vector3 position, Quaternion rotation) {
            Vector2 ros = RoboBoatCourse.UnityToRos(position);
            float yaw = RoboBoatCourse.RosYawOfUnityDirection(
                RoboBoatRosFrame.Rotation(rotation) * Vector3.forward);
            return string.Format(CultureInfo.InvariantCulture, "x={0:R} y={1:R} yaw={2:R}",
                ros.x, ros.y, yaw);
        }

        public static Transform FindBoatBody() {
            OmniXController boat = FindAnyObjectByType<OmniXController>(FindObjectsInactive.Exclude);
            if (boat == null) return null;
            return boat.GetComponentInParent<ArticulationBody>()?.transform ??
                   boat.GetComponentInParent<Rigidbody>()?.transform ?? boat.transform;
        }
    }
}
