using System;
using System.Collections.Generic;
using Sim.Controllers;
using Sim.Course.RoboBoat;
using Sim.Sensors.Vision;
using Sim.Utils.Viewer;
using UnityEngine;

namespace Sim.Course {
    /// <summary>Applies a world-frame truth to the scene: each element from stream A's prefab,
    /// else the scene template during the transition, else a placeholder primitive behind the
    /// same <see cref="CourseObjectAnchor"/>; state components attached; detection targets with
    /// state-following labels and opaque ids. No scene edits.</summary>
    public static class CourseBuilder {
        public sealed class Built {
            public Transform Root;
            public readonly Dictionary<string, IStatefulCourseObject> States = new Dictionary<string, IStatefulCourseObject>();
            public readonly List<DetectionTarget> Targets = new List<DetectionTarget>();
            public int Placeholders;
        }

        public static Built Build(JObj truth, CourseObjectCatalog catalog, LabelMode mode, ulong seed, IEnumerable<string> hide) {
            var built = new Built { Root = new GameObject($"Course {truth.Str("name")}").transform };
            var stateKinds = new Dictionary<string, string>();
            foreach (JObj s in truth.Arr("states")) stateKinds[s.Str("id")] = s.Str("kind");

            // Resolve scene templates before their groups are hidden (Find skips inactive objects).
            var templates = new Dictionary<string, GameObject>();
            foreach (var e in catalog.Entries)
                if (e.prefab == null && e.template.Length > 0 && !templates.ContainsKey(e.template))
                    templates[e.template] = GameObject.Find(e.template);
            float waterline = 0f;
            foreach (var t in templates.Values) if (t != null) { waterline = t.transform.position.y; break; }

            var taskAreas = new Dictionary<string, Bounds>();  // framed by the viewer's Task and Course views
            foreach (JObj e in truth.Arr("elements")) {
                var entry = catalog.Get(e.Str("kind"));
                GameObject go = Instantiate(entry, e, templates, waterline, built.Root, out bool placeholder);
                go.name = e.Str("id");
                string task = e.Str("task");
                if (!string.IsNullOrEmpty(task)) {
                    Bounds bounds = JsonCourseSource.WorldBounds(go);
                    if (taskAreas.TryGetValue(task, out Bounds area)) bounds.Encapsulate(area);
                    taskAreas[task] = bounds;
                }
                if (placeholder) built.Placeholders++;
                IStatefulCourseObject stateful = null;
                string stateId = e.Str("state");
                if (stateId != null && stateKinds.TryGetValue(stateId, out string kind)) {
                    StatefulCourseObject c = kind == "light_sign" ? go.AddComponent<LightSign>()
                        : kind == "sign_face" ? (StatefulCourseObject)go.AddComponent<SignFace>() : go.AddComponent<IndicatorLight>();
                    c.Id = stateId;
                    built.States[stateId] = stateful = c;
                }
                string elementKind = e.Str("kind");
                if (catalog.Label(elementKind, "x", mode).Length == 0) continue;
                var source = stateful;
                built.Targets.Add(new DetectionTarget(go, () => catalog.Label(elementKind, source?.State, mode),
                    CourseTruth.OpaqueId(seed, e.Str("id"))));
            }
            foreach (var area in taskAreas) ViewerAreas.Set(area.Key, area.Value);
            foreach (JObj s in truth.Arr("states"))
                if (s.Str("kind") == "horn") {
                    var horn = new GameObject(s.Str("id")).AddComponent<HornProgram>();
                    horn.transform.SetParent(built.Root, false);
                    horn.Id = s.Str("id");
                    built.States[horn.Id] = horn;
                }

            foreach (string name in hide ?? Array.Empty<string>()) {
                GameObject group = GameObject.Find(name);
                if (group != null) group.SetActive(false);
            }
            return built;
        }

        static Quaternion Heading(double rosYaw) =>
            Quaternion.LookRotation(new Vector3(-(float)Math.Sin(rosYaw), 0f, (float)Math.Cos(rosYaw)), Vector3.up);

        static GameObject Instantiate(CourseObjectCatalog.Entry entry, JObj e, Dictionary<string, GameObject> templates,
            float waterline, Transform root, out bool placeholder) {
            placeholder = false;
            var ros = new Vector2((float)e.Num("x"), (float)e.Num("y"));
            Quaternion heading = Heading(e.Num("yaw"));
            float z = (float)e.Num("z", 0);
            if (entry.prefab != null) {
                var anchor = entry.prefab.GetComponent<CourseObjectAnchor>();
                float lift = anchor != null ? anchor.WaterlineOffset : 0f;
                return UnityEngine.Object.Instantiate(entry.prefab, RoboBoatCourse.RosToUnity(ros, waterline + lift + z), heading, root);
            }
            if (entry.template.Length > 0 && templates.TryGetValue(entry.template, out var template) && template != null) {
                var clone = UnityEngine.Object.Instantiate(template, RoboBoatCourse.RosToUnity(ros, template.transform.position.y + z),
                    heading * template.transform.rotation, root);
                clone.transform.localScale = template.transform.lossyScale;
                float r = JsonCourseSource.HorizontalRadius(clone);
                if (r > 1e-3f) clone.transform.localScale *= entry.radius / r;
                if (entry.color.Length > 0) JsonCourseSource.Recolor(clone, entry.color);
                JsonCourseSource.MakeSolid(clone, entry.template.Contains("Buoy"));
                return clone;
            }
            placeholder = true;
            return Placeholder(entry, e, RoboBoatCourse.RosToUnity(ros, waterline + z), heading, root);
        }

        /// <summary>A primitive standing in for a missing prefab (design Appendix B).</summary>
        static GameObject Placeholder(CourseObjectCatalog.Entry entry, JObj e, Vector3 at, Quaternion heading, Transform root) {
            var go = new GameObject(entry.kind);
            go.transform.SetParent(root, false);
            go.transform.SetPositionAndRotation(at, heading);
            var anchor = go.AddComponent<CourseObjectAnchor>();
            anchor.Placeholder = true;
            anchor.FootprintRadius = entry.radius;
            float h = entry.height, r = entry.radius;
            Color color = entry.color.Length > 0 && ColorUtility.TryParseHtmlString(entry.color, out var c) ? c : new Color(0.9f, 0.9f, 0.9f);
            switch (entry.placeholder) {
                case PlaceholderShape.Box:
                    anchor.StateSlot = Part(go, PrimitiveType.Cube, new Vector3(0, h / 2, 0),
                        new Vector3((float)e.Num("width", 2 * r), h, (float)e.Num("length", 2 * r)), color);
                    break;
                case PlaceholderShape.Hull:
                    anchor.StateSlot = Part(go, PrimitiveType.Cube, new Vector3(0, h / 2, 0), new Vector3(0.41f, h, 1.07f), color);
                    break;
                case PlaceholderShape.Panel:
                    Part(go, PrimitiveType.Cylinder, new Vector3(0, h / 2, 0), new Vector3(0.05f, h / 2, 0.05f), Color.gray);
                    // A quad shows its face toward -z; the panel faces along the element's yaw.
                    var face = Part(go, PrimitiveType.Quad, new Vector3(0, h, 0.03f), new Vector3(0.61f, 0.61f, 1f), Color.white);
                    face.transform.localRotation = Quaternion.Euler(0f, 180f, 0f);
                    anchor.StateSlot = face;
                    break;
                default:
                    anchor.StateSlot = Part(go, PrimitiveType.Cylinder, new Vector3(0, h / 2, 0), new Vector3(2 * r, h / 2, 2 * r), color);
                    break;
            }
            return go;
        }

        static Renderer Part(GameObject parent, PrimitiveType type, Vector3 local, Vector3 scale, Color color) {
            var part = GameObject.CreatePrimitive(type);
            part.transform.SetParent(parent.transform, false);
            part.transform.localPosition = local;
            part.transform.localScale = scale;
            part.layer = Sim.Physics.Contacts.CraneCollisionLayers.DynamicObstacle;
            var renderer = part.GetComponent<Renderer>();
            try { renderer.sharedMaterial = new Material(JsonCourseSource.TemplateMaterial()); }
            catch (InvalidOperationException) { }
            renderer.material.SetColor("_BaseColor", color);
            return renderer;
        }
    }
}
