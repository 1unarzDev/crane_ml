using System;
using System.Linq;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Physics.Core {
    /// <summary>
    /// <c>--crane-robot NAME</c>: when a scene with a robot loads, that robot is replaced by the
    /// <see cref="RobotCatalog"/> prefab NAME, placed with its root link where the scene robot's
    /// was and its bow along the same heading. The scene stays as authored. Installed before the
    /// other scene bootstraps (plant, course) so they see the selected robot.
    /// </summary>
    public static class RobotSelection {
        public const string Flag = "--crane-robot";

        /// <summary>The replaced robot's root link pose, bow along +z (null without a swap).</summary>
        public static Transform StartPose { get; private set; }
        public static GameObject Spawned { get; private set; }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        private static void Install() {
            StartPose = null;
            Spawned = null;
            string robot = ProfileBootstrap.Read(Environment.GetCommandLineArgs(), Flag);
            if (string.IsNullOrEmpty(robot)) return;
            SceneManager.sceneLoaded += (_, _) => {
                try { Swap(robot); }
                catch (Exception error) { Debug.LogError($"CRANE_ROBOT_FAILED robot={robot} {error}"); }
            };
        }

        /// <summary>Replace the scene's robot with catalog entry <paramref name="robot"/>.</summary>
        public static GameObject Swap(string robot) {
            // The scene robot: one that describes itself, else the largest articulation (a prefab synced before RobotDescription).
            GameObject old = UnityEngine.Object.FindObjectsByType<RobotDescription>(FindObjectsInactive.Exclude)
                .Select(d => d.gameObject).FirstOrDefault(g => Spawned == null || g != Spawned);
            if (old == null)
                old = UnityEngine.Object.FindObjectsByType<ArticulationBody>(FindObjectsInactive.Exclude)
                    .Where(a => a.isRoot && (Spawned == null || !a.transform.IsChildOf(Spawned.transform)))
                    .OrderByDescending(a => a.GetComponentsInChildren<ArticulationBody>().Length)
                    .Select(a => a.transform.root.gameObject).FirstOrDefault();
            if (old == null) return null;   // a scene without a robot (bootstrap)
            RobotCatalog catalog = RobotCatalog.Load();
            GameObject prefab = catalog != null ? catalog.Find(robot) : null;
            if (prefab == null) {
                Debug.LogError($"CRANE_ROBOT_UNKNOWN robot={robot} catalog={(catalog != null ? catalog.Names : "missing")}");
                return null;
            }
            ArticulationBody oldBody = old.GetComponentInChildren<ArticulationBody>();
            Transform oldLink = oldBody != null ? oldBody.transform : old.transform;
            var start = new GameObject($"Robot start ({old.name})").transform;
            start.SetPositionAndRotation(oldLink.position, oldLink.rotation * RobotDescription.BodyFromFlu(oldLink));

            // Root pose that puts the new root link at the start, bow along the start's +z.
            ArticulationBody newBody = prefab.GetComponentInChildren<ArticulationBody>(true);
            Transform link = newBody != null ? newBody.transform : prefab.transform;
            Quaternion linkInRoot = Quaternion.Inverse(prefab.transform.rotation) * link.rotation;
            Vector3 linkAt = prefab.transform.InverseTransformPoint(link.position);
            Quaternion want = start.rotation * Quaternion.Inverse(RobotDescription.BodyFromFlu(link));
            Quaternion rootRotation = want * Quaternion.Inverse(linkInRoot);
            Vector3 rootPosition = start.position - rootRotation * linkAt;

            old.SetActive(false);
            UnityEngine.Object.Destroy(old);
            GameObject spawned = UnityEngine.Object.Instantiate(prefab, rootPosition, rootRotation);
            spawned.name = prefab.name;
            StartPose = start;
            Spawned = spawned;
            Vector3 e = start.rotation.eulerAngles;
            Debug.Log($"CRANE_ROBOT_SPAWNED robot={robot} prefab={prefab.name} replaced={old.name} " +
                      $"position=({start.position.x:F3},{start.position.y:F3},{start.position.z:F3}) heading_deg={e.y:F1}");
            return spawned;
        }
    }
}
