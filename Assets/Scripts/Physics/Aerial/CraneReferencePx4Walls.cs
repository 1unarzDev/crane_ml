using Sim.Physics.Contacts;
using Sim.Utils.ReferenceEnvironments;
using UnityEngine;

namespace Sim.Physics.Aerial {
    /// <summary>
    /// Canonical primitive reconstruction of PX4-gazebo-models/worlds/walls.sdf. Gazebo ENU
    /// (x, y, z) maps to Unity (x, z, y). Collision and presentation remain separate.
    /// </summary>
    public sealed class CraneReferencePx4Walls : MonoBehaviour {
        public const string EnvironmentId = "px4-gazebo-walls-bb0b9cf-v1";
        [SerializeField] private bool createVisualLayer = true;

        private static readonly WallDefinition[] Walls = {
            new("wall-box-01", new Vector3(5f, 7.5f, 0f), new Vector3(1f, 15f, 20f)),
            new("wall-box-02", new Vector3(-3f, 7.5f, 5f), new Vector3(10f, 15f, 1f)),
            new("wall-box-03", new Vector3(13f, 7.5f, -10f), new Vector3(17f, 15f, 1f)),
            new("wall-box-04", new Vector3(12f, 7.5f, 0f), new Vector3(1f, 15f, 20f))
        };

        private void Awake() {
            if (transform.Find("CanonicalGeometry") == null) Generate();
        }

        public void Configure(bool visuals) => createVisualLayer = visuals;

        public void Generate() {
            DestroyChild("CanonicalGeometry");
            DestroyChild("VisualPresentation");
            var canonical = new GameObject("CanonicalGeometry").transform;
            canonical.SetParent(transform, false);
            var visual = new GameObject("VisualPresentation").transform;
            visual.SetParent(transform, false);

            // SDF planes are infinite. This finite 100 m test envelope matches the source visual
            // extent and contains every benchmark object; the approximation is explicit in the
            // provenance manifest.
            CreateBox(canonical, visual, "ground-plane", "traversable-floor",
                new Vector3(0f, -0.05f, 0f), new Vector3(100f, 0.1f, 100f),
                new Color(0.65f, 0.65f, 0.65f));
            foreach (WallDefinition wall in Walls)
                CreateBox(canonical, visual, wall.Id, "aerial-obstacle-wall", wall.Center,
                    wall.Size, new Color(0.42f, 0.42f, 0.42f));

            Debug.Log($"CRANE_REFERENCE_ENVIRONMENT_READY id={EnvironmentId} walls={Walls.Length} " +
                      "sourceAxes=ENU unityMapping=x,z,y");
        }

        public bool ValidateCanonicalGeometry(out string message) {
            Transform canonical = transform.Find("CanonicalGeometry");
            if (canonical == null) {
                message = "canonical layer missing";
                return false;
            }
            foreach (WallDefinition expected in Walls) {
                Transform value = canonical.Find(expected.Id);
                BoxCollider collider = value == null ? null : value.GetComponent<BoxCollider>();
                CraneSemanticIdentity identity = value == null ? null :
                    value.GetComponent<CraneSemanticIdentity>();
                if (value == null || collider == null || identity == null ||
                    Vector3.Distance(value.localPosition, expected.Center) > 0.0001f ||
                    Vector3.Distance(collider.size, expected.Size) > 0.0001f ||
                    identity.EnvironmentId != EnvironmentId || identity.SemanticId != expected.Id) {
                    message = $"invalid wall {expected.Id}";
                    return false;
                }
            }
            message = "ok";
            return true;
        }

        private void CreateBox(Transform canonical, Transform visual, string id, string role,
            Vector3 center, Vector3 size, Color color) {
            var collision = new GameObject(id);
            collision.layer = CraneCollisionLayers.Environment;
            collision.transform.SetParent(canonical, false);
            collision.transform.localPosition = center;
            collision.AddComponent<BoxCollider>().size = size;
            collision.AddComponent<CraneSemanticIdentity>().Configure(id, role, EnvironmentId);
            if (!createVisualLayer) return;

            GameObject presentation = GameObject.CreatePrimitive(PrimitiveType.Cube);
            presentation.name = id + "-visual";
            presentation.transform.SetParent(visual, false);
            presentation.transform.localPosition = center;
            presentation.transform.localScale = size;
            Collider presentationCollider = presentation.GetComponent<Collider>();
            if (presentationCollider != null) DestroyGeneratedObject(presentationCollider);
            presentation.GetComponent<Renderer>().sharedMaterial = new Material(
                Shader.Find("HDRP/Lit") ?? Shader.Find("Standard")) { color = color };
        }

        private void DestroyChild(string childName) {
            Transform value = transform.Find(childName);
            if (value != null) DestroyGeneratedObject(value.gameObject);
        }

        private static void DestroyGeneratedObject(Object value) {
            if (Application.isPlaying) Destroy(value);
            else DestroyImmediate(value);
        }

        private readonly struct WallDefinition {
            public readonly string Id;
            public readonly Vector3 Center;
            public readonly Vector3 Size;

            public WallDefinition(string id, Vector3 center, Vector3 size) {
                Id = id;
                Center = center;
                Size = size;
            }
        }
    }
}
