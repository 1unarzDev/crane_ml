using UnityEngine;

namespace Sim.Physics.Core {
    /// <summary>The world-ROS plane (REP 103 ENU, the /crane/odom frame) in Unity's world
    /// (x right, y up, z forward): ROS x is Unity z and ROS y is -Unity x.</summary>
    public static class RosFrame {
        public static Vector3 ToUnity(Vector2 ros, float height) => new(-ros.y, height, ros.x);

        public static Vector2 ToRos(Vector3 unity) => new(unity.z, -unity.x);

        /// <summary>The ROS yaw (counter-clockwise from x) of a Unity direction (whose heading
        /// turns clockwise about +y).</summary>
        public static float YawOf(Vector3 direction) => Mathf.Atan2(-direction.x, direction.z);
    }
}
