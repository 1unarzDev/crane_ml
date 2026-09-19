using Sim.Physics.Contacts;
using Sim.Utils.ReferenceEnvironments;
using UnityEngine;

namespace Sim.Physics.Aerial {
    /// <summary>
    /// Render-only reconstruction of the PX4 ArUco world. The landmark deliberately has no
    /// collider; canonical collision contains only the documented finite ground envelope.
    /// </summary>
    public sealed class CraneReferencePx4Aruco : MonoBehaviour {
        public const string EnvironmentId = "px4-gazebo-aruco-bb0b9cf-v1";

        private void Awake() {
            if (transform.Find("CanonicalGeometry") == null) Generate();
        }

        public void Generate() {
            DestroyChild("CanonicalGeometry");
            DestroyChild("SimulationSemantics");
            DestroyChild("VisualPresentation");

            var canonical = new GameObject("CanonicalGeometry").transform;
            canonical.SetParent(transform, false);
            var semantics = new GameObject("SimulationSemantics").transform;
            semantics.SetParent(transform, false);
            var visual = new GameObject("VisualPresentation").transform;
            visual.SetParent(transform, false);

            var ground = new GameObject("ground-plane");
            ground.layer = CraneCollisionLayers.Environment;
            ground.transform.SetParent(canonical, false);
            ground.transform.localPosition = new Vector3(0f, -0.05f, 0f);
            ground.AddComponent<BoxCollider>().size = new Vector3(100f, 0.1f, 100f);
            ground.AddComponent<CraneSemanticIdentity>().Configure("ground-plane",
                "traversable-floor", EnvironmentId);

            GameObject groundVisual = GameObject.CreatePrimitive(PrimitiveType.Cube);
            groundVisual.name = "ground-plane-visual";
            groundVisual.transform.SetParent(visual, false);
            groundVisual.transform.localPosition = new Vector3(0f, -0.05f, 0f);
            groundVisual.transform.localScale = new Vector3(100f, 0.1f, 100f);
            DestroyGeneratedObject(groundVisual.GetComponent<Collider>());
            groundVisual.GetComponent<Renderer>().sharedMaterial = CreateColorMaterial(
                new Color(0.65f, 0.65f, 0.65f));

            var landmark = new GameObject("aruco-tag-01");
            landmark.transform.SetParent(semantics, false);
            landmark.transform.localPosition = new Vector3(0f, 0.001f, 0f);
            landmark.AddComponent<CraneSemanticIdentity>().Configure("aruco-tag-01",
                "render-only-fiducial", EnvironmentId);

            GameObject tag = GameObject.CreatePrimitive(PrimitiveType.Quad);
            tag.name = "aruco-tag-01-visual";
            tag.transform.SetParent(visual, false);
            tag.transform.localPosition = new Vector3(0f, 0.001f, 0f);
            tag.transform.localRotation = Quaternion.Euler(-90f, 0f, 0f);
            tag.transform.localScale = new Vector3(0.5f, 0.5f, 1f);
            DestroyGeneratedObject(tag.GetComponent<Collider>());
            tag.GetComponent<Renderer>().sharedMaterial = CreateTagMaterial();

            Debug.Log($"CRANE_REFERENCE_ENVIRONMENT_READY id={EnvironmentId} " +
                      "landmark=aruco-tag-01 collision=render-only");
        }

        public bool ValidateLayers(out string message) {
            Transform canonical = transform.Find("CanonicalGeometry");
            Transform semantics = transform.Find("SimulationSemantics/aruco-tag-01");
            Transform visual = transform.Find("VisualPresentation/aruco-tag-01-visual");
            if (canonical == null || semantics == null || visual == null) {
                message = "required layer or landmark missing";
                return false;
            }
            if (semantics.GetComponent<Collider>() != null || visual.GetComponent<Collider>() != null) {
                message = "render-only landmark owns collision";
                return false;
            }
            BoxCollider ground = canonical.Find("ground-plane")?.GetComponent<BoxCollider>();
            CraneSemanticIdentity identity = semantics.GetComponent<CraneSemanticIdentity>();
            if (ground == null || identity == null || identity.SemanticId != "aruco-tag-01" ||
                identity.EnvironmentId != EnvironmentId) {
                message = "ground or semantic identity invalid";
                return false;
            }
            message = "ok";
            return true;
        }

        private static Material CreateColorMaterial(Color color) => new(
            Shader.Find("HDRP/Lit") ?? Shader.Find("Standard")) { color = color };

        private static Material CreateTagMaterial() {
            // Exact 6x6 black/white cell pattern sampled from the pinned 354x354 upstream PNG.
            string[] rows = { "000000", "010110", "001010", "000110", "000100", "000000" };
            const int cell = 32;
            var texture = new Texture2D(6 * cell, 6 * cell, TextureFormat.RGBA32, false) {
                name = "PX4 ArUco Tag 01",
                filterMode = FilterMode.Point,
                wrapMode = TextureWrapMode.Clamp
            };
            var pixels = new Color32[texture.width * texture.height];
            for (int y = 0; y < texture.height; y++) {
                int sourceRow = 5 - y / cell;
                for (int x = 0; x < texture.width; x++) {
                    byte value = rows[sourceRow][x / cell] == '1' ? (byte)255 : (byte)0;
                    pixels[y * texture.width + x] = new Color32(value, value, value, 255);
                }
            }
            texture.SetPixels32(pixels);
            texture.Apply(false, true);
            Shader shader = Shader.Find("HDRP/Unlit") ?? Shader.Find("Unlit/Texture") ??
                Shader.Find("Standard");
            var material = new Material(shader) { name = "PX4 ArUco Tag Material" };
            material.mainTexture = texture;
            return material;
        }

        private void DestroyChild(string childName) {
            Transform value = transform.Find(childName);
            if (value != null) DestroyGeneratedObject(value.gameObject);
        }

        private static void DestroyGeneratedObject(Object value) {
            if (value == null) return;
            if (Application.isPlaying) Destroy(value);
            else DestroyImmediate(value);
        }
    }
}
