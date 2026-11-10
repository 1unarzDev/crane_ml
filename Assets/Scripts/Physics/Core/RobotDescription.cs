using UnityEngine;

namespace Sim.Physics.Core {
    /// <summary>
    /// What a robot prefab says about itself at runtime: the URDF it was synced from and the yaw
    /// between that URDF's body frame (REP-103 FLU) and the frame crane's runtime uses. The URDF
    /// sync writes it on the prefab root. It is the only source of a robot's body frame; a robot
    /// without one is REP-103 (yaw 0).
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class RobotDescription : MonoBehaviour {
        [Tooltip("URDF the prefab was synced from")] public string urdf = "";
        [Tooltip("deg. ROS yaw from the URDF body frame to the runtime frame: 0 for REP-103, 90 for a hull whose bow is local -X")]
        public float rootYawDeg;
        [Tooltip("Written by the URDF sync: link=hash@yaw of each link's URDF geometry, so a re-sync keeps unchanged geometry")]
        public string[] geometryHashes = System.Array.Empty<string>();

        /// <summary>Hull-local rotation of body FLU (Unity axes: forward = bow, up = up).</summary>
        public static Quaternion BodyFromFlu(float rootYawDeg) => Quaternion.Euler(0f, -rootYawDeg, 0f);

        /// <summary>The body frame of the robot that carries <paramref name="part"/>.</summary>
        public static Quaternion BodyFromFlu(Component part) {
            RobotDescription description = part != null ? part.GetComponentInParent<RobotDescription>(true) : null;
            return BodyFromFlu(description != null ? description.rootYawDeg : 0f);
        }
    }
}
