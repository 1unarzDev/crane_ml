using System.Collections.Generic;
using System.Linq;
using UnityEngine.Rendering.HighDefinition;
using Unity.Collections;
using UnityEngine;
using Sim.Utils;

namespace Sim.Physics.Processing {
    public class Submersion : MonoBehaviour {
        [ReadOnly] public Submerged submerged;

        [SerializeField, Tooltip("HDRP water surface used for height querying")]
        private WaterSurface waterSurface = null;
        [SerializeField, Tooltip("A simplified mesh for physics calculations")]
        private Mesh simplifiedMesh;
        [SerializeField, Tooltip("Side length of square water surface approximation patch. Much be large enough to fit entire vessel")]
        private float patchSize = 10;
        [SerializeField, Tooltip("Higher number gives a better approximation of water surface")]
        private int patchResolution = 4;

        private Patch patch;
        //public bool drawWaterLine;

        [SerializeField] private bool drawPatch;
        [SerializeField] private bool drawSubmerged;
        private float displacedVolume;

        private void Awake() {
            // A robot spawned at runtime (a URDF-synced hull) has neither: the scene's water and a
            // hull from its own colliders.
            if (waterSurface == null) waterSurface = FindAnyObjectByType<WaterSurface>();
            if (waterSurface == null) {
                // No water in this scene: no hydrostatics rather than a null patch every FixedUpdate.
                Debug.LogWarning($"CRANE_SUBMERSION_NO_WATER link={name}: no WaterSurface in the scene; Submersion and Buoyancy disabled");
                if (TryGetComponent(out Sim.Physics.Water.Statics.Buoyancy buoyancy)) buoyancy.enabled = false;
                enabled = false;
                return;
            }
            if (simplifiedMesh == null) {
                simplifiedMesh = ColliderHull(transform);
                Debug.Log($"CRANE_SUBMERSION_HULL link={name} from=colliders vertices={simplifiedMesh.vertexCount}");
            }
            Vector3 gridOrigin = new(-patchSize / 2, 0, patchSize / 2);
            patch = new Patch(waterSurface, patchSize, patchResolution, gridOrigin);
            submerged = new Submerged(simplifiedMesh, debug: true); // set up submersion by providing the simplified hull mesh
            patch.Update(transform); // updates the patch to follow the boat and queried water height
        }

        private void FixedUpdate() {
            patch.Update(transform); // updates the patch to follow the boat and queried water height
            submerged.Update(patch, transform);

            displacedVolume = submerged.data.volume;

            if (drawPatch) DebugPatch();
            if (drawSubmerged) DebugSubmerged();
        }


        /// <summary>A closed mesh in <paramref name="link"/>'s local space from the colliders of that
        /// link (not of child links): boxes as boxes, mesh colliders as their mesh, other shapes
        /// as their bounding box.</summary>
        public static Mesh ColliderHull(Transform link) {
            var vertices = new List<Vector3>();
            var triangles = new List<int>();
            foreach (Collider c in link.GetComponentsInChildren<Collider>()) {
                if (c.isTrigger || c.attachedArticulationBody != null && c.attachedArticulationBody.transform != link) continue;
                Matrix4x4 toLink = link.worldToLocalMatrix * c.transform.localToWorldMatrix;
                if (c is MeshCollider mc && mc.sharedMesh != null) {
                    int b0 = vertices.Count;
                    vertices.AddRange(mc.sharedMesh.vertices.Select(v => toLink.MultiplyPoint3x4(v)));
                    triangles.AddRange(mc.sharedMesh.triangles.Select(i => b0 + i));
                    continue;
                }
                Vector3 centre, size;
                switch (c) {
                    case BoxCollider box: centre = box.center; size = box.size; break;
                    case SphereCollider sphere: centre = sphere.center; size = Vector3.one * 2f * sphere.radius; break;
                    case CapsuleCollider capsule:
                        centre = capsule.center;
                        size = Vector3.one * 2f * capsule.radius;
                        size[capsule.direction] = Mathf.Max(capsule.height, 2f * capsule.radius);
                        break;
                    default: continue;
                }
                Vector3 Corner(int k) => toLink.MultiplyPoint3x4(centre + Vector3.Scale(size * 0.5f,
                    new Vector3((k & 1) == 0 ? -1 : 1, (k & 2) == 0 ? -1 : 1, (k & 4) == 0 ? -1 : 1)));
                // Outward faces (Unity winds clockwise seen from outside); own vertices per face for flat normals.
                int[] faces = { 0,2,3, 0,3,1,  4,5,7, 4,7,6,  0,1,5, 0,5,4,  2,6,7, 2,7,3,  0,4,6, 0,6,2,  1,3,7, 1,7,5 };
                foreach (int k in faces) {
                    triangles.Add(vertices.Count);
                    vertices.Add(Corner(k));
                }
            }
            var mesh = new Mesh { name = $"{link.name} collider hull" };
            mesh.SetVertices(vertices);
            mesh.SetTriangles(triangles, 0);
            mesh.RecalculateNormals();
            return mesh;
        }

        private void DebugPatch() {
            int[] tris = patch.patchTriangles;
            Vector3[] verts = patch.patchVertices;
            for (var i = 0; i < tris.Length; i += 3) {
                Vector3[] tri = new Vector3[] { verts[tris[i]], verts[tris[i + 1]], verts[tris[i + 2]] };
                CommonUtils.DebugDrawTriangle(tri, Color.red);
            }
        }


        private void DebugSubmerged() {
            int[] tris = submerged.data.triangles;
            Vector3[] verts = submerged.data.vertices;

            for (int i = 0; i < submerged.data.maxTriangleIndex - 2; i += 3) {
                Vector3[] tri = new Vector3[]
                {
                transform.TransformPoint(verts[tris[i]]),
                transform.TransformPoint(verts[tris[i + 1]]),
                transform.TransformPoint(verts[tris[i + 2]])
                };

                CommonUtils.DebugDrawTriangle(tri, Color.green);
            }
        }

        private void OnDestroy() {
            patch.Dispose();
        }
    }
}
