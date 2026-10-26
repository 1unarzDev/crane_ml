using UnityEngine;

namespace Sim.Physics.Core {
    /// <summary>
    /// What a robot prefab says about itself at runtime: the URDF it was synced from and the yaw
    /// between that URDF's body frame (REP-103 FLU) and the frame crane's runtime uses. The URDF
    /// sync writes it on the prefab root. A robot without one is taken as RoboBoat (yaw 90: bow =
    /// hull-local −X).
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class RobotDescription : MonoBehaviour {
        public const float LegacyRootYawDeg = 90f;

        [Tooltip("URDF the prefab was synced from")] public string urdf = "";
        [Tooltip("deg. ROS yaw from the URDF body frame to the runtime frame: 0 for REP-103, 90 for RoboBoat")]
        public float rootYawDeg;
        [Tooltip("Written by the URDF sync: link=hash@yaw of each link's URDF geometry, so a re-sync keeps unchanged geometry")]
        public string[] geometryHashes = System.Array.Empty<string>();

        /// <summary>Hull-local rotation of body FLU (Unity axes: forward = bow, up = up).</summary>
        public static Quaternion BodyFromFlu(float rootYawDeg) => Quaternion.Euler(0f, -rootYawDeg, 0f);

        /// <summary>The body frame of the robot that carries <paramref name="part"/>.</summary>
        public static Quaternion BodyFromFlu(Component part) {
            RobotDescription description = part != null ? part.GetComponentInParent<RobotDescription>(true) : null;
            return BodyFromFlu(description != null ? description.rootYawDeg : LegacyRootYawDeg);
        }
    }
}
