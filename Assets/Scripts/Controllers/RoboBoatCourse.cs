using System;
using System.Globalization;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.Rendering.HighDefinition;

namespace Sim.Controllers {
    /// <summary>
    /// World-ROS frame and water-current helpers for the RoboBoat scene, and the boat's start
    /// pose. The course itself is built by Sim.Course (Assets/Scripts/Course), which depends on
    /// this assembly, not the other way round.
    /// </summary>
    public static class RoboBoatCourse {
        public const string CurrentFlag = "--crane-water-current";

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            if (Array.IndexOf(Environment.GetCommandLineArgs(), CurrentFlag) < 0) return;
            SceneManager.sceneLoaded += (_, _) => {
                try { ApplyCurrentFromArgs(); }
                catch (Exception error) { Debug.LogError($"CRANE_ROBOBOAT_COURSE_FAILED {error}"); }
            };
        }

        /// <summary>Applies <c>--crane-water-current</c> if given; harmless to call twice.</summary>
        public static void ApplyCurrentFromArgs() {
            string[] args = Environment.GetCommandLineArgs();
            int index = Array.IndexOf(args, CurrentFlag);
            if (index < 0) return;
            string value = index + 1 < args.Length ? args[index + 1] : null;
            if (!ParseCurrent(value, out float speed, out float toward))
                throw new ArgumentException($"{CurrentFlag} expects speed,toward_deg; got {value}");
            foreach (var surface in UnityEngine.Object.FindObjectsByType<WaterSurface>(FindObjectsSortMode.None)) {
                surface.largeCurrentSpeedValue = speed;
                surface.largeOrientationValue = HdrpOrientation(toward);
            }
            Debug.Log($"CRANE_ROBOBOAT_CURRENT speed={speed:R} toward_deg={toward:R}");
        }

        /// <summary>"speed,toward_deg": speed in m/s, direction counter-clockwise from the
        /// world +x of /crane/odom.</summary>
        public static bool ParseCurrent(string value, out float speed, out float towardDeg) {
            speed = towardDeg = 0f;
            string[] parts = (value ?? "").Split(',');
            return parts.Length == 2 &&
                   float.TryParse(parts[0], NumberStyles.Float, CultureInfo.InvariantCulture, out speed) &&
                   float.TryParse(parts[1], NumberStyles.Float, CultureInfo.InvariantCulture, out towardDeg) &&
                   speed >= 0f;
        }

        /// <summary>HDRP current orientation for a world-ROS direction. HDRP's direction is
        /// (cos, sin) over Unity (x, z); ROS x is Unity z and ROS y is -Unity x.</summary>
        public static float HdrpOrientation(float towardDeg) => towardDeg + 90f;

        public static (float speed, float toward) CurrentNow() {
            var surface = UnityEngine.Object.FindAnyObjectByType<WaterSurface>();
            if (surface == null) return (0f, 0f);
            return (surface.largeCurrentSpeedValue, (surface.largeOrientationValue - 90f) * Mathf.Deg2Rad);
        }

        /// <summary>The boat's start pose in world-ROS (/crane/odom): position and yaw. With
        /// --crane-robot the scene boat is replaced, so the spawned robot's start pose
        /// (RobotSelection.StartPose) is used. False with neither (a bootstrap scene).</summary>
        public static bool BoatStart(out Vector2 start, out float yaw) {
            start = Vector2.zero;
            yaw = 0f;
            Vector3 startAt, bow;
            var boat = UnityEngine.Object.FindAnyObjectByType<OmniXController>(FindObjectsInactive.Exclude);
            if (boat != null) {
                Transform body = boat.GetComponentInParent<ArticulationBody>()?.transform ??
                                 boat.GetComponentInParent<Rigidbody>()?.transform ?? boat.transform;
                startAt = body.position;
                bow = RoboBoatRosFrame.Rotation(body.rotation) * Vector3.forward;
            }
            else if (Sim.Physics.Core.RobotSelection.StartPose != null) {
                startAt = Sim.Physics.Core.RobotSelection.StartPose.position;
                bow = Sim.Physics.Core.RobotSelection.StartPose.forward;
            }
            else return false;
            start = UnityToRos(startAt);
            yaw = RosYawOfUnityDirection(bow);
            return true;
        }

        /// <summary>Course frame (start pose) to world-ROS: rotate by the start heading, then
        /// translate to the start position.</summary>
        public static Vector2 ToWorld(Vector2 start, float startYaw, float x, float y) {
            float c = Mathf.Cos(startYaw), s = Mathf.Sin(startYaw);
            return new Vector2(start.x + c * x - s * y, start.y + s * x + c * y);
        }

        public static Vector3 RosToUnity(Vector2 ros, float height) => new(-ros.y, height, ros.x);

        public static Vector2 UnityToRos(Vector3 unity) => new(unity.z, -unity.x);

        // A ROS yaw (counter-clockwise from x) as a Unity heading (clockwise about +y).
        public static float RosYawOfUnityDirection(Vector3 direction) => Mathf.Atan2(-direction.x, direction.z);
    }
}
