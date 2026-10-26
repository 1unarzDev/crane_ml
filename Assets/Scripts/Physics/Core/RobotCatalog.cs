using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Sim.Physics.Core {
    /// <summary>
    /// The robot prefabs a player can spawn with <c>--crane-robot NAME</c>. The URDF sync registers
    /// every robot it writes, so a new URDF needs no other step; the asset sits in a Resources
    /// folder so builds carry the prefabs.
    /// </summary>
    public sealed class RobotCatalog : ScriptableObject {
        public const string ResourceName = "RobotCatalog";
        public const string AssetPath = "Assets/Config/Robots/Resources/RobotCatalog.asset";

        [Serializable]
        public sealed class Entry {
            public string name = "";
            public GameObject prefab;
        }

        public List<Entry> robots = new();

        public GameObject Find(string robot) =>
            robots.FirstOrDefault(e => string.Equals(e.name, robot?.Trim(), StringComparison.OrdinalIgnoreCase))?.prefab;

        public string Names => string.Join(",", robots.Select(e => e.name));

        public static RobotCatalog Load() => Resources.Load<RobotCatalog>(ResourceName);
    }
}
