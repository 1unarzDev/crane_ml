using System;
using System.Collections.Generic;
using UnityEngine;

namespace Crane.Urdf
{
    /// <summary>One component-placement rule: a URDF link (by name pattern or tag) gets these components.</summary>
    [Serializable]
    public class UrdfRule
    {
        [Tooltip("Shown in messages.")] public string name;
        [Tooltip("Regex on the link name; \"<root>\" is the URDF root link. Empty = match by tag only.")] public string link;
        [Tooltip("thruster | lidar | camera | imu | gps, read from <ros2_control> thruster params and <gazebo><sensor type=..>. Empty = match by name only.")] public string tag;
        [Tooltip("Regex on a child link name to place the components on instead (e.g. _propeller_link$). Falls back to the matched link.")] public string on;
        [Tooltip("Component type names (class name or full name).")] public string[] components = Array.Empty<string>();
        [Tooltip("Copy values (and non-link children such as a camera object) from \"Assets/..prefab#link_name\". Empty = an existing component of the same type in the robot, else code defaults.")] public string template;
    }

    /// <summary>How a URDF becomes a crane prefab. Put one next to the URDF (or in a parent folder);
    /// without one the built-in generic rules apply. Mass, inertia and hydrodynamics are not here:
    /// vehicle profiles apply them at runtime.</summary>
    [CreateAssetMenu(menuName = "Robots/URDF Import Profile")]
    public class UrdfImportProfile : ScriptableObject
    {
        [field: SerializeField, Tooltip("Prefab to create or sync. Empty = <URDF folder>/<robot name>.prefab.")] public string prefabPath { get; set; } = "";
        [field: SerializeField, Tooltip("deg. ROS yaw from the URDF body frame to the frame crane's runtime expects for every link. 0 for REP-103 consumers; 90 for RoboBoat (bow = local -X, RoboBoatPhysics.BodyFromFlu).")] public float rootYawDeg { get; set; }
        [field: SerializeField, Tooltip("Delete prefab links the URDF no longer has.")] public bool removeMissingLinks { get; set; } = true;
        [field: SerializeField] public List<UrdfRule> rules { get; set; } = new List<UrdfRule>();

        /// <summary>Generic rules: name patterns or tags onto crane's sensor and thruster components, with values from
        /// an existing component of the same type in the robot, else code defaults (the sync warns about empty references).
        /// Domain rules (water, reference templates) live in profile assets, e.g. Assets/Robot Descriptions/Surface.urdfprofile.asset.</summary>
        public static UrdfImportProfile CreateDefault()
        {
            var p = CreateInstance<UrdfImportProfile>();
            p.name = "built-in generic";
            p.rules = new List<UrdfRule>
            {
                new UrdfRule { name = "thruster", tag = "thruster", link = "thruster", on = "propeller", components = new[] { "Thruster" } },
                new UrdfRule { name = "lidar", tag = "lidar", link = "(lidar|laser|velodyne|ouster)", components = new[] { "Lidar3D" } },
                new UrdfRule { name = "camera", tag = "camera", link = "camera(_link)?$", components = new[] { "CameraInfo", "ROSCameraAsync", "ROSDepthCameraAsync" } },
                new UrdfRule { name = "imu", tag = "imu", link = "(^|_)imu(_|$)", components = new[] { "Imu" } },
                new UrdfRule { name = "gps", tag = "gps", link = "(gps|gnss|navsat)", components = new[] { "GPS" } },
            };
            return p;
        }
    }
}
