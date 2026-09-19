using System;
using System.Collections.Generic;
using Sim.Physics.Contacts;
using Sim.Utils.ReferenceEnvironments;
using UnityEngine;

namespace Sim.Physics.Land {
    /// <summary>
    /// Deterministic canonical warehouse geometry adapted from the layout pattern used by the
    /// Unity Robotics Warehouse Nav2 example. Collision and semantic layers are authoritative;
    /// presentation objects are non-colliding and may be replaced without changing feasibility.
    /// </summary>
    public sealed class CraneReferenceWarehouse : MonoBehaviour {
        public const string EnvironmentId = "unity-turtlebot3-simple-warehouse-v1";
        [SerializeField] private int seed = 1000;
        [SerializeField] private float width = 12f;
        [SerializeField] private float length = 18f;
        [SerializeField] private int shelfRows = 3;
        [SerializeField] private int shelfColumns = 2;
        [SerializeField] private bool createVisualLayer = true;

        private void Awake() {
            if (transform.Find("CanonicalGeometry") == null) Generate();
        }

        public void Configure(int configuredSeed, float configuredWidth, float configuredLength,
            int rows, int columns, bool visuals) {
            seed = configuredSeed;
            width = Mathf.Max(4f, configuredWidth);
            length = Mathf.Max(6f, configuredLength);
            shelfRows = Mathf.Max(1, rows);
            shelfColumns = Mathf.Max(1, columns);
            createVisualLayer = visuals;
        }

        public void Generate() {
            Transform oldCanonical = transform.Find("CanonicalGeometry");
            Transform oldVisual = transform.Find("VisualPresentation");
            if (oldCanonical != null) DestroyGeneratedObject(oldCanonical.gameObject);
            if (oldVisual != null) DestroyGeneratedObject(oldVisual.gameObject);
            var canonical = new GameObject("CanonicalGeometry").transform;
            canonical.SetParent(transform, false);
            var visual = new GameObject("VisualPresentation").transform;
            visual.SetParent(transform, false);

            CreateBox(canonical, visual, "floor", "traversable-floor",
                new Vector3(0f, -0.1f, length * 0.5f), new Vector3(width, 0.2f, length),
                new Color(0.34f, 0.36f, 0.38f));
            const float wallThickness = 0.20f;
            const float wallHeight = 2.5f;
            CreateBox(canonical, visual, "wall-west", "boundary-wall",
                new Vector3(-width * 0.5f, wallHeight * 0.5f, length * 0.5f),
                new Vector3(wallThickness, wallHeight, length), Color.gray);
            CreateBox(canonical, visual, "wall-east", "boundary-wall",
                new Vector3(width * 0.5f, wallHeight * 0.5f, length * 0.5f),
                new Vector3(wallThickness, wallHeight, length), Color.gray);
            CreateBox(canonical, visual, "wall-north", "boundary-wall",
                new Vector3(0f, wallHeight * 0.5f, length),
                new Vector3(width, wallHeight, wallThickness), Color.gray);

            float rowSpacing = length / (shelfRows + 1f);
            float columnSpacing = width / (shelfColumns + 1f);
            for (int column = 1; column <= shelfColumns; column++) {
                for (int row = 1; row <= shelfRows; row++) {
                    string id = $"shelf-c{column:D2}-r{row:D2}";
                    Vector3 center = new(column * columnSpacing - width * 0.5f,
                        0.9f, row * rowSpacing);
                    CreateBox(canonical, visual, id, "shelving-obstacle", center,
                        new Vector3(1.35f, 1.8f, 3.0f), new Color(0.18f, 0.32f, 0.48f));
                }
            }

            // A seed-controlled pallet provides independently variable but reproducible clutter.
            var random = new System.Random(seed);
            float palletX = (float)(random.NextDouble() * (width - 3f) - (width - 3f) * 0.5f);
            CreateBox(canonical, visual, "pallet-01", "movable-clutter",
                new Vector3(palletX, 0.25f, length * 0.78f), new Vector3(1.0f, 0.5f, 1.2f),
                new Color(0.42f, 0.23f, 0.08f));

            AddRegion(canonical, "aisle-west", "navigation-corridor",
                new Vector3(-width * 0.33f, 0f, length * 0.5f));
            AddRegion(canonical, "aisle-center", "navigation-corridor",
                new Vector3(0f, 0f, length * 0.5f));
            AddRegion(canonical, "aisle-east", "navigation-corridor",
                new Vector3(width * 0.33f, 0f, length * 0.5f));
            Debug.Log($"CRANE_REFERENCE_ENVIRONMENT_READY id={EnvironmentId} seed={seed} " +
                      $"width={width:R} length={length:R} shelves={shelfRows * shelfColumns}");
        }

        private void CreateBox(Transform canonical, Transform visual, string id, string role,
            Vector3 center, Vector3 size, Color color) {
            var collision = new GameObject(id);
            collision.layer = CraneCollisionLayers.Environment;
            collision.transform.SetParent(canonical, false);
            collision.transform.localPosition = center;
            var collider = collision.AddComponent<BoxCollider>();
            collider.size = size;
            collision.AddComponent<CraneSemanticIdentity>().Configure(id, role, EnvironmentId);
            if (!createVisualLayer) return;
            GameObject presentation = GameObject.CreatePrimitive(PrimitiveType.Cube);
            presentation.name = id + "-visual";
            presentation.transform.SetParent(visual, false);
            presentation.transform.localPosition = center;
            presentation.transform.localScale = size;
            Collider presentationCollider = presentation.GetComponent<Collider>();
            if (presentationCollider != null) DestroyGeneratedObject(presentationCollider);
            presentation.GetComponent<Renderer>().material.color = color;
        }

        private static void AddRegion(Transform canonical, string id, string role,
            Vector3 center) {
            var region = new GameObject(id);
            region.transform.SetParent(canonical, false);
            region.transform.localPosition = center;
            region.AddComponent<CraneSemanticIdentity>().Configure(id, role, EnvironmentId);
        }

        private static void DestroyGeneratedObject(UnityEngine.Object value) {
            if (Application.isPlaying) Destroy(value);
            else DestroyImmediate(value);
        }
    }
}
