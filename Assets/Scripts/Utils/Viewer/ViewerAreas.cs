using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Utils.Viewer {
    /// <summary>Named world areas a shot can frame.</summary>
    public interface IViewerAreas {
        /// <summary>The union of every area (the whole course).</summary>
        bool TryAll(out Bounds bounds);
        /// <summary>The area closest to <paramref name="point"/> in the horizontal plane.</summary>
        bool TryNearest(Vector3 point, out Bounds bounds);
    }

    /// <summary>A fixed list of areas.</summary>
    public sealed class ViewerAreaList : IViewerAreas {
        private readonly List<Bounds> areas;

        public ViewerAreaList(IEnumerable<Bounds> areas) => this.areas = new List<Bounds>(areas);

        public int Count => areas.Count;

        public bool TryAll(out Bounds bounds) {
            bounds = default;
            for (int i = 0; i < areas.Count; i++) {
                if (i == 0) bounds = areas[0];
                else bounds.Encapsulate(areas[i]);
            }
            return areas.Count > 0;
        }

        public bool TryNearest(Vector3 point, out Bounds bounds) {
            bounds = default;
            float best = float.PositiveInfinity;
            foreach (Bounds area in areas) {
                Vector3 closest = area.ClosestPoint(new Vector3(point.x, area.center.y, point.z));
                float distance = new Vector2(closest.x - point.x, closest.z - point.z).sqrMagnitude;
                if (distance >= best) continue;
                best = distance;
                bounds = area;
            }
            return areas.Count > 0;
        }
    }

    /// <summary>
    /// The scene's task areas. A course generator registers ground truth with
    /// <see cref="Set"/>; without it the areas are the active scene's root objects whose
    /// rendered bounds are smaller than a size limit (prefab groups such as a channel or a
    /// marina), excluding the robot.
    /// </summary>
    public static class ViewerAreas {
        private static readonly Dictionary<string, Bounds> registered = new();
        private static int registeredScene = -1;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        private static void ResetStatics() {
            registered.Clear();
            registeredScene = -1;
        }

        /// <summary>Registers ground truth for the active scene, replacing an area of the same name.</summary>
        public static void Set(string name, Bounds bounds) {
            int scene = SceneManager.GetActiveScene().handle.GetHashCode();
            if (scene != registeredScene) registered.Clear();
            registeredScene = scene;
            registered[name] = bounds;
        }

        /// <summary>The registered areas of the active scene, else the discovered ones.</summary>
        public static ViewerAreaList ForActiveScene(Transform robot, float maxAreaSize) {
            if (registeredScene == SceneManager.GetActiveScene().handle.GetHashCode() && registered.Count > 0)
                return new ViewerAreaList(registered.Values);
            var found = new List<Bounds>();
            foreach (GameObject root in SceneManager.GetActiveScene().GetRootGameObjects()) {
                if (!root.activeInHierarchy || (robot != null && robot.IsChildOf(root.transform))) continue;
                bool any = false;
                Bounds bounds = default;
                foreach (Renderer renderer in root.GetComponentsInChildren<Renderer>()) {
                    if (!renderer.enabled) continue;
                    if (any) bounds.Encapsulate(renderer.bounds);
                    else bounds = renderer.bounds;
                    any = true;
                }
                if (any && bounds.size.magnitude <= maxAreaSize) found.Add(bounds);
            }
            return new ViewerAreaList(found);
        }
    }
}
