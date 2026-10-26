using System;
using System.Linq;
using UnityEngine;

namespace Sim.Course {
    /// <summary>Scene helpers shared by the course builder, sign faces and the legacy JSON
    /// source: bounds, recolouring, colliders and the banner shapes.</summary>
    internal static class CourseVisuals {
        internal static Bounds WorldBounds(GameObject value) {
            var renderers = value.GetComponentsInChildren<Renderer>();
            if (renderers.Length == 0) return new Bounds(value.transform.position, Vector3.zero);
            Bounds bounds = renderers[0].bounds;
            foreach (var renderer in renderers.Skip(1)) bounds.Encapsulate(renderer.bounds);
            return bounds;
        }

        internal static Material TemplateMaterial() {
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

        /// <summary>Real buoys are solid: a LiDAR sees them and a hull bumps them. Give an
        /// element without colliders one per mesh, on the obstacle layer. A buoy's own
        /// colliders are replaced by its meshes (pole buoys had none, the black buoy's sphere
        /// was half its mesh), and kept non-convex where the body is kinematic, so a pole
        /// stays a pole rather than a cone.</summary>
        internal static void MakeSolid(GameObject value, bool buoy) {
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

        internal static void Recolor(GameObject value, string hex) {
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

        internal static float HorizontalRadius(GameObject value) {
            Vector3 size = WorldBounds(value).size;
            return 0.5f * Mathf.Max(size.x, size.z);
        }
    }
}
