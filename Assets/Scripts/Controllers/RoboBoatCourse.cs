using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using Sim.Sensors.Vision;
using Sim.Utils.ROS;
using Sim.Utils.Viewer;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.Rendering.HighDefinition;

namespace Sim.Controllers {
    /// <summary>One element of a RoboBoat course, in the course frame: origin at the
    /// boat's start, x along its starting heading, y to port, metres and radians.</summary>
    [Serializable]
    internal sealed class RoboBoatCourseElement {
        public string task;       // which task it belongs to, for the referee
        public string template;   // scene object cloned for it, e.g. "RedBuoy0" or "Dock0"
        public string label;      // detection class; empty means not detected
        public float x;
        public float y;
        public float yaw;         // for a dock: the direction its bays open toward
        public float scale = 1f;
        public string color;      // optional recolor, "#rrggbb"
        public string[] bays;     // dock only: banner label per bay, e.g. "red_circle"
        public int[] occupied;    // dock only: bays holding a stationary vessel
        public string vessel = "YellowDeliveryBoat";
        public string vesselLabel = "yellow_target_boat";
    }

    [Serializable]
    internal sealed class RoboBoatCourseSpec {
        public string name;
        public string[] hide;     // scene groups replaced by the course
        public float jitter;      // metres of seeded random offset per element
        public RoboBoatCourseElement[] elements;
    }

    [Serializable]
    internal sealed class RoboBoatCourseTruthElement {
        public string task;
        public string label;
        public string template;
        public float x;
        public float y;
        public float radius;
    }

    [Serializable]
    internal sealed class RoboBoatCourseTruthBay {
        public int dock;
        public int index;
        public float x;           // bay centre, between the tines
        public float y;
        public float yaw;         // direction the bay opens toward
        public float width;
        public float depth;
        public string banner;
        public bool occupied;
    }

    [Serializable]
    internal sealed class RoboBoatCourseTruth {
        public string schema = "crane-roboboat-course-truth-v1";
        public string name;
        public string frame = "world";  // the frame of /crane/odom
        public float startX;
        public float startY;
        public float startYaw;
        public float currentSpeed;
        public float currentToward;     // radians, counter-clockwise from +x
        public List<RoboBoatCourseTruthElement> elements = new();
        public List<RoboBoatCourseTruthBay> bays = new();
    }

    /// <summary>
    /// Builds a RoboBoat course at load time from a JSON spec, by cloning the scene's own
    /// buoys, docks and vessels, so a layout changes without touching the scene. Every
    /// element is registered with the detection sensor under its label, and the truth
    /// (what was placed, in the /crane/odom frame) is written for the referee.
    /// </summary>
    internal static class RoboBoatCourse {
        internal const string Flag = "--crane-roboboat-course";
        internal const string CurrentFlag = "--crane-water-current";
        private const float BannerSize = 0.61f;    // 24 in
        private const float BannerHeight = 0.8f;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            string[] args = Environment.GetCommandLineArgs();
            bool course = Array.IndexOf(args, Flag) >= 0;
            bool current = Array.IndexOf(args, CurrentFlag) >= 0;
            if (!course && !current) return;
            SceneManager.sceneLoaded += (_, _) => {
                try {
                    if (current) ApplyCurrent(ReadString(args, CurrentFlag, null));
                    if (course) Build(args);
                }
                catch (Exception error) {
                    Debug.LogError($"CRANE_ROBOBOAT_COURSE_FAILED {error}");
                }
            };
        }

        /// <summary>"speed,toward_deg": speed in m/s, direction counter-clockwise from the
        /// world +x of /crane/odom.</summary>
        internal static bool ParseCurrent(string value, out float speed, out float towardDeg) {
            speed = towardDeg = 0f;
            string[] parts = (value ?? "").Split(',');
            return parts.Length == 2 &&
                   float.TryParse(parts[0], NumberStyles.Float, CultureInfo.InvariantCulture,
                       out speed) &&
                   float.TryParse(parts[1], NumberStyles.Float, CultureInfo.InvariantCulture,
                       out towardDeg) && speed >= 0f;
        }

        /// <summary>HDRP current orientation for a world-ROS direction. HDRP's direction is
        /// (cos, sin) over Unity (x, z); ROS x is Unity z and ROS y is -Unity x.</summary>
        internal static float HdrpOrientation(float towardDeg) => towardDeg + 90f;

        private static void ApplyCurrent(string value) {
            if (!ParseCurrent(value, out float speed, out float toward))
                throw new ArgumentException($"{CurrentFlag} expects speed,toward_deg; got {value}");
            foreach (var surface in UnityEngine.Object.FindObjectsByType<WaterSurface>(
                         FindObjectsSortMode.None)) {
                surface.largeCurrentSpeedValue = speed;
                surface.largeOrientationValue = HdrpOrientation(toward);
            }
            Debug.Log($"CRANE_ROBOBOAT_CURRENT speed={speed:R} toward_deg={toward:R}");
        }

        internal static (float speed, float toward) CurrentNow() {
            var surface = UnityEngine.Object.FindAnyObjectByType<WaterSurface>();
            if (surface == null) return (0f, 0f);
            return (surface.largeCurrentSpeedValue,
                (surface.largeOrientationValue - 90f) * Mathf.Deg2Rad);
        }

        internal static RoboBoatCourseSpec Parse(string json) {
            var spec = JsonUtility.FromJson<RoboBoatCourseSpec>(json);
            if (spec?.elements == null || spec.elements.Length == 0)
                throw new ArgumentException("course has no elements");
            foreach (var element in spec.elements)
                if (string.IsNullOrEmpty(element.template))
                    throw new ArgumentException("every course element needs a template");
            return spec;
        }

        /// <summary>Course frame (start pose) to world-ROS: rotate by the start heading, then
        /// translate to the start position.</summary>
        internal static Vector2 ToWorld(Vector2 start, float startYaw, float x, float y) {
            float c = Mathf.Cos(startYaw), s = Mathf.Sin(startYaw);
            return new Vector2(start.x + c * x - s * y, start.y + s * x + c * y);
        }

        internal static Vector3 RosToUnity(Vector2 ros, float height) =>
            new(-ros.y, height, ros.x);

        internal static Vector2 UnityToRos(Vector3 unity) => new(unity.z, -unity.x);

        // A ROS yaw (counter-clockwise from x) as a Unity heading (clockwise about +y).
        internal static float RosYawOfUnityDirection(Vector3 direction) =>
            Mathf.Atan2(-direction.x, direction.z);

        private static void Build(string[] args) {
            string path = ReadString(args, Flag, null);
            RoboBoatCourseSpec spec = Parse(File.ReadAllText(path));
            OmniXController boat = UnityEngine.Object.FindAnyObjectByType<OmniXController>(
                FindObjectsInactive.Exclude);
            Vector3 startAt, bow;
            if (boat != null) {
                Transform body = boat.GetComponentInParent<ArticulationBody>()?.transform ??
                                 boat.GetComponentInParent<Rigidbody>()?.transform ?? boat.transform;
                startAt = body.position;
                bow = RoboBoatRosFrame.Rotation(body.rotation) * Vector3.forward;
            }
            else if (Sim.Physics.Core.RobotSelection.StartPose != null) {
                // --crane-robot replaced the scene boat: the course is laid out from where it stood.
                startAt = Sim.Physics.Core.RobotSelection.StartPose.position;
                bow = Sim.Physics.Core.RobotSelection.StartPose.forward;
            }
            else return;  // a bootstrap scene, before the course scene loads
            Vector2 start = UnityToRos(startAt);
            float startYaw = RosYawOfUnityDirection(bow);

            var templates = new Dictionary<string, GameObject>();
            foreach (var element in spec.elements) {
                foreach (string name in new[] { element.template,
                             element.occupied?.Length > 0 ? element.vessel : null }) {
                    if (name == null || templates.ContainsKey(name)) continue;
                    templates[name] = GameObject.Find(name) ??
                        throw new ArgumentException($"no scene object named {name} to clone");
                }
            }

            var random = new System.Random(ReadInt(args, "--crane-seed", 0));
            float Jitter() => spec.jitter * (float)(random.NextDouble() * 2.0 - 1.0);
            var truth = new RoboBoatCourseTruth {
                name = spec.name, startX = start.x, startY = start.y, startYaw = startYaw
            };
            (truth.currentSpeed, truth.currentToward) = CurrentNow();
            var targets = new List<KeyValuePair<GameObject, string>>();
            var root = new GameObject($"RoboBoat Course {spec.name}").transform;
            int docks = 0;
            var taskAreas = new Dictionary<string, Bounds>();  // framed by the viewer's Task view

            foreach (var element in spec.elements) {
                GameObject template = templates[element.template];
                Vector2 world = ToWorld(start, startYaw, element.x + Jitter(), element.y + Jitter());
                float yaw = startYaw + element.yaw;
                bool dock = element.bays != null && element.bays.Length > 0;
                // Docks are turned so their bays open toward ``yaw``; buoys keep their pose.
                Quaternion rotation = dock
                    ? Quaternion.AngleAxis(-(yaw - OpeningYaw(template)) * Mathf.Rad2Deg,
                        Vector3.up) * template.transform.rotation
                    : template.transform.rotation;
                GameObject clone = UnityEngine.Object.Instantiate(template,
                    RosToUnity(world, template.transform.position.y), rotation, root);
                clone.name = $"{template.name} {element.label}".Trim();
                clone.transform.localScale = template.transform.lossyScale * element.scale;
                if (!string.IsNullOrEmpty(element.color)) Recolor(clone, element.color);
                MakeSolid(clone, element.template.Contains("Buoy"));
                if (!string.IsNullOrEmpty(element.label))
                    targets.Add(new KeyValuePair<GameObject, string>(clone, element.label));
                truth.elements.Add(new RoboBoatCourseTruthElement {
                    task = element.task, label = element.label, template = element.template,
                    x = world.x, y = world.y, radius = HorizontalRadius(clone)
                });
                if (dock) BuildBays(clone, element, docks++, templates, targets, truth, root);
                if (!string.IsNullOrEmpty(element.task)) {
                    Bounds bounds = WorldBounds(clone);
                    if (taskAreas.TryGetValue(element.task, out Bounds area)) bounds.Encapsulate(area);
                    taskAreas[element.task] = bounds;
                }
            }
            foreach (var area in taskAreas) ViewerAreas.Set(area.Key, area.Value);

            foreach (string name in spec.hide ?? Array.Empty<string>()) {
                GameObject group = GameObject.Find(name);
                if (group != null) group.SetActive(false);
            }
            foreach (var sensor in UnityEngine.Object.FindObjectsByType<BoundingBox3D>(
                         FindObjectsSortMode.None))
                sensor.ReplaceTargets(targets);

            string truthPath = ReadString(args, "--crane-roboboat-course-truth", null);
            if (truthPath != null) File.WriteAllText(truthPath, JsonUtility.ToJson(truth, true));
            Debug.Log($"CRANE_ROBOBOAT_COURSE_READY name={spec.name} elements={truth.elements.Count} " +
                      $"bays={truth.bays.Count} start=({start.x:R},{start.y:R},{startYaw:R}) " +
                      $"truth={truthPath ?? "none"}");
        }

        private static IEnumerable<Transform> Dividers(GameObject dock) =>
            dock.GetComponentsInChildren<Transform>()
                .Where(child => child.name.StartsWith("DockDivider", StringComparison.Ordinal));

        private static Bounds WorldBounds(GameObject value) {
            var renderers = value.GetComponentsInChildren<Renderer>();
            if (renderers.Length == 0) return new Bounds(value.transform.position, Vector3.zero);
            Bounds bounds = renderers[0].bounds;
            foreach (var renderer in renderers.Skip(1)) bounds.Encapsulate(renderer.bounds);
            return bounds;
        }

        private static Vector3 Center(Transform value) => WorldBounds(value.gameObject).center;

        /// <summary>World-ROS yaw the bays open toward: from the spine to the tines.</summary>
        private static float OpeningYaw(GameObject dock) {
            Transform spine = dock.GetComponentsInChildren<Transform>()
                .First(child => child.name == "DockBody");
            Vector3 tines = Dividers(dock).Select(Center).Aggregate(Vector3.zero, (a, b) => a + b) /
                            Dividers(dock).Count();
            Vector3 opening = tines - Center(spine);
            opening.y = 0f;
            return RosYawOfUnityDirection(opening);
        }

        private static void BuildBays(GameObject dock, RoboBoatCourseElement element, int index,
            Dictionary<string, GameObject> templates,
            List<KeyValuePair<GameObject, string>> targets, RoboBoatCourseTruth truth,
            Transform root) {
            float openingYaw = OpeningYaw(dock);
            var opening = new Vector3(-Mathf.Sin(openingYaw), 0f, Mathf.Cos(openingYaw));
            var across = Vector3.Cross(Vector3.up, opening);
            Vector3 spine = Center(dock.GetComponentsInChildren<Transform>()
                .First(child => child.name == "DockBody"));
            var tines = Dividers(dock).Select(divider => (divider, center: Center(divider)))
                .OrderBy(item => Vector3.Dot(item.center, across)).ToList();
            for (int bay = 0; bay + 1 < tines.Count && bay < element.bays.Length; bay++) {
                Vector3 middle = (tines[bay].center + tines[bay + 1].center) * 0.5f;
                Bounds tine = WorldBounds(tines[bay].divider.gameObject);
                float depth = Mathf.Abs(Vector3.Dot(tine.size, opening));
                float width = Mathf.Abs(Vector3.Dot(tines[bay + 1].center - tines[bay].center,
                    across));
                // The banner stands on the spine behind the bay, facing out of it.
                Vector3 back = middle - opening * Vector3.Dot(middle - spine, opening);
                GameObject banner = Banner(element.bays[bay],
                    back + Vector3.up * BannerHeight, opening, root);
                targets.Add(new KeyValuePair<GameObject, string>(banner, element.bays[bay]));
                bool occupied = element.occupied != null && element.occupied.Contains(bay);
                if (occupied) {
                    GameObject vessel = UnityEngine.Object.Instantiate(templates[element.vessel],
                        new Vector3(middle.x, templates[element.vessel].transform.position.y,
                            middle.z),
                        Quaternion.LookRotation(opening) , root);
                    vessel.name = $"{element.vessel} bay {bay}";
                    targets.Add(new KeyValuePair<GameObject, string>(vessel, element.vesselLabel));
                }
                Vector2 center = UnityToRos(middle);
                truth.bays.Add(new RoboBoatCourseTruthBay {
                    dock = index, index = bay, x = center.x, y = center.y, yaw = openingYaw,
                    width = width, depth = depth, banner = element.bays[bay],
                    occupied = occupied
                });
            }
        }

        private static GameObject Banner(string label, Vector3 position, Vector3 facing,
            Transform root) {
            GameObject banner = GameObject.CreatePrimitive(PrimitiveType.Quad);
            banner.name = $"Banner {label}";
            banner.transform.SetParent(root, true);
            banner.transform.position = position;
            // A quad shows its face toward -z, so point +z into the dock.
            banner.transform.rotation = Quaternion.LookRotation(-facing, Vector3.up);
            banner.transform.localScale = Vector3.one * BannerSize;
            var renderer = banner.GetComponent<Renderer>();
            var material = new Material(TemplateMaterial());
            Texture2D texture = BannerTexture(label);
            material.SetTexture("_BaseColorMap", texture);
            material.SetTexture("_MainTex", texture);
            material.SetColor("_BaseColor", Color.white);
            renderer.sharedMaterial = material;
            return banner;
        }

        private static Material TemplateMaterial() {
            foreach (var renderer in UnityEngine.Object.FindObjectsByType<MeshRenderer>(
                         FindObjectsSortMode.None))
                if (renderer.sharedMaterial != null &&
                    renderer.sharedMaterial.HasProperty("_BaseColorMap"))
                    return renderer.sharedMaterial;
            throw new InvalidOperationException("no HDRP Lit material to copy for banners");
        }

        internal static Color BannerColor(string label) =>
            label.StartsWith("red", StringComparison.Ordinal) ? new Color(0.85f, 0.1f, 0.1f) :
            label.StartsWith("green", StringComparison.Ordinal) ? new Color(0.1f, 0.65f, 0.2f) :
            label.StartsWith("blue", StringComparison.Ordinal) ? new Color(0.1f, 0.3f, 0.9f) :
            Color.black;

        /// <summary>Is texel (u, v) in [0, 1] inside the banner's shape?</summary>
        internal static bool InShape(string shape, float u, float v) {
            float x = u - 0.5f, y = v - 0.5f;
            switch (shape) {
                case "circle": return x * x + y * y <= 0.35f * 0.35f;
                case "square": return Mathf.Abs(x) <= 0.3f && Mathf.Abs(y) <= 0.3f;
                case "triangle": return y >= -0.3f && Mathf.Abs(x) <= (0.35f - y) * 0.55f;
                case "plus":
                case "cross":
                    return (Mathf.Abs(x) <= 0.1f && Mathf.Abs(y) <= 0.35f) ||
                           (Mathf.Abs(y) <= 0.1f && Mathf.Abs(x) <= 0.35f);
                default: return false;
            }
        }

        private static Texture2D BannerTexture(string label) {
            const int size = 64;
            string shape = label.Contains('_') ? label[(label.IndexOf('_') + 1)..] : label;
            Color color = BannerColor(label);
            var texture = new Texture2D(size, size, TextureFormat.RGBA32, false);
            for (int row = 0; row < size; row++)
                for (int column = 0; column < size; column++)
                    texture.SetPixel(column, row, InShape(shape, (column + 0.5f) / size,
                        (row + 0.5f) / size) ? color : Color.white);
            texture.Apply();
            return texture;
        }

        /// <summary>Real buoys are solid: a LiDAR sees them and a hull bumps them. Give an
        /// element without colliders one per mesh, on the obstacle layer. A buoy's own
        /// colliders are replaced by its meshes (pole buoys had none, the black buoy's sphere
        /// was half its mesh), and kept non-convex where the body is kinematic, so a pole
        /// stays a pole rather than a cone.</summary>
        private static void MakeSolid(GameObject value, bool buoy) {
            Collider[] existing = value.GetComponentsInChildren<Collider>()
                .Where(collider => !collider.isTrigger).ToArray();
            if (existing.Length > 0 && !buoy) return;
            foreach (Collider collider in existing) UnityEngine.Object.DestroyImmediate(collider);
            int added = 0;
            foreach (var filter in value.GetComponentsInChildren<MeshFilter>()) {
                if (filter.sharedMesh == null) continue;
                var collider = filter.gameObject.AddComponent<MeshCollider>();
                collider.sharedMesh = filter.sharedMesh;
                Rigidbody body = filter.GetComponentInParent<Rigidbody>();
                collider.convex = body != null && !body.isKinematic;
                filter.gameObject.layer = Sim.Physics.Contacts.CraneCollisionLayers.DynamicObstacle;
                added++;
            }
            if (buoy) {
                Bounds bounds = new(value.transform.position, Vector3.zero);
                foreach (Collider collider in value.GetComponentsInChildren<Collider>())
                    bounds.Encapsulate(collider.bounds);
                Debug.Log($"CRANE_ROBOBOAT_SOLID {value.name} replaced={existing.Length} " +
                          $"mesh_colliders={added} size=({bounds.size.x:F2},{bounds.size.y:F2}," +
                          $"{bounds.size.z:F2})");
            }
        }

        private static void Recolor(GameObject value, string hex) {
            if (!ColorUtility.TryParseHtmlString(hex, out Color color))
                throw new ArgumentException($"bad color {hex}");
            foreach (var renderer in value.GetComponentsInChildren<Renderer>()) {
                var materials = renderer.materials;  // instanced copies
                foreach (var material in materials) {
                    if (material.HasProperty("_BaseColor")) material.SetColor("_BaseColor", color);
                    if (material.HasProperty("_Color")) material.SetColor("_Color", color);
                }
                renderer.materials = materials;
            }
        }

        private static float HorizontalRadius(GameObject value) {
            Vector3 size = WorldBounds(value).size;
            return 0.5f * Mathf.Max(size.x, size.z);
        }

        private static string ReadString(string[] args, string key, string fallback) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length && !args[index + 1].StartsWith("--")
                ? args[index + 1] : fallback;
        }

        private static int ReadInt(string[] args, string key, int fallback) =>
            int.TryParse(ReadString(args, key, null), out int value) ? value : fallback;
    }
}
