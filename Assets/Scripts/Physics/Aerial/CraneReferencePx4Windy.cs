using Sim.Physics.Contacts;
using Sim.Utils.ReferenceEnvironments;
using UnityEngine;

namespace Sim.Physics.Aerial {
    /// <summary>CRANE realization of the pinned PX4 windy world parameter and ground plane.</summary>
    public sealed class CraneReferencePx4Windy : MonoBehaviour {
        public const string EnvironmentId = "px4-gazebo-windy-bb0b9cf-v1";
        public static readonly Vector3 UnityWindVelocity = new(5f, 0f, 2f);

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
            groundVisual.GetComponent<Renderer>().sharedMaterial = new Material(
                Shader.Find("HDRP/Lit") ?? Shader.Find("Standard")) {
                color = new Color(0.65f, 0.65f, 0.65f)
            };

            var field = new GameObject("wind-field-01");
            field.transform.SetParent(semantics, false);
            field.AddComponent<CraneSemanticIdentity>().Configure("wind-field-01",
                "configured-environment-field", EnvironmentId);
            Debug.Log($"CRANE_REFERENCE_ENVIRONMENT_READY id={EnvironmentId} " +
                      $"windUnity={UnityWindVelocity}");
        }

        public bool ValidateLayers(out string message) {
            Transform ground = transform.Find("CanonicalGeometry/ground-plane");
            Transform field = transform.Find("SimulationSemantics/wind-field-01");
            if (ground?.GetComponent<BoxCollider>() == null ||
                field?.GetComponent<CraneSemanticIdentity>() == null ||
                field.GetComponent<Collider>() != null) {
                message = "ground or wind semantic layer invalid";
                return false;
            }
            message = "ok";
            return true;
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
