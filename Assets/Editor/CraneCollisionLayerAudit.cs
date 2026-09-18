#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using Sim.Physics.Contacts;

public static class CraneCollisionLayerAudit {
    [Serializable] private sealed class AuditResult {
        public string schema = "crane-collision-layer-audit-v2";
        public string unityVersion;
        public string generatedUtc;
        public int misclassifiedColliders;
        public bool valid;
        public List<SceneAudit> scenes = new();
    }

    [Serializable] private sealed class SceneAudit {
        public string scene;
        public int gameObjects;
        public int colliders;
        public int triggers;
        public int rigidbodyColliders;
        public int articulationColliders;
        public int misclassifiedColliders;
        public bool valid;
        public List<LayerCount> objectLayers = new();
        public List<ColliderAudit> colliderDetails = new();
    }

    [Serializable] private sealed class LayerCount {
        public int layer;
        public string name;
        public int gameObjects;
        public int colliders;
    }

    [Serializable] private sealed class ColliderAudit {
        public string path;
        public string colliderType;
        public int layer;
        public string layerName;
        public bool trigger;
        public string bodyType;
        public string bodyPath;
        public string tag;
        public string prefabAsset;
        public List<string> ancestorBehaviours;
    }

    private static readonly string[] ProductionScenes = {
        "Assets/Scenes/Robosub Pool.unity",
        "Assets/Scenes/Roboboat Course.unity"
    };

    public static void MigrateProduction() {
        Dictionary<string, int> prefabLayers = DiscoverPrefabLayers();
        foreach (KeyValuePair<string, int> entry in prefabLayers)
            MigratePrefab(entry.Key, entry.Value);
        foreach (string scenePath in ProductionScenes)
            MigrateScene(scenePath);
        AssetDatabase.SaveAssets();
        Run();
        Debug.Log($"CRANE_COLLISION_LAYER_MIGRATION_COMPLETE prefabs={prefabLayers.Count}");
    }

    public static void Run() {
        string output = ReadArgument("--crane-output") ??
            "PerformanceResults/collision-layer-audit.json";
        var result = new AuditResult {
            unityVersion = Application.unityVersion,
            generatedUtc = DateTime.UtcNow.ToString("O")
        };

        foreach (string scenePath in ProductionScenes)
            result.scenes.Add(AuditScene(scenePath));
        foreach (SceneAudit scene in result.scenes)
            result.misclassifiedColliders += scene.misclassifiedColliders;
        result.valid = result.misclassifiedColliders == 0;

        string absoluteOutput = Path.GetFullPath(output);
        Directory.CreateDirectory(Path.GetDirectoryName(absoluteOutput) ?? ".");
        File.WriteAllText(absoluteOutput, JsonUtility.ToJson(result, true));
        Debug.Log($"CRANE_COLLISION_LAYER_AUDIT_COMPLETE path={absoluteOutput} " +
                  $"valid={result.valid} misclassified={result.misclassifiedColliders}");
        if (!result.valid)
            throw new InvalidOperationException(
                $"Production collision-layer audit found {result.misclassifiedColliders} " +
                "misclassified colliders.");
    }

    private static SceneAudit AuditScene(string scenePath) {
        Scene scene = EditorSceneManager.OpenScene(scenePath, OpenSceneMode.Single);
        var audit = new SceneAudit { scene = scenePath };
        var objectCounts = new int[32];
        var colliderCounts = new int[32];

        foreach (GameObject root in scene.GetRootGameObjects()) {
            foreach (Transform transform in root.GetComponentsInChildren<Transform>(true)) {
                GameObject gameObject = transform.gameObject;
                audit.gameObjects++;
                objectCounts[gameObject.layer]++;

                foreach (Collider collider in gameObject.GetComponents<Collider>()) {
                    audit.colliders++;
                    colliderCounts[gameObject.layer]++;
                    if (collider.isTrigger) audit.triggers++;
                    if (gameObject.layer != Classify(collider)) audit.misclassifiedColliders++;

                    Component body = collider.attachedRigidbody;
                    if (body != null) audit.rigidbodyColliders++;
                    else {
                        body = collider.GetComponentInParent<ArticulationBody>(true);
                        if (body != null) audit.articulationColliders++;
                    }

                    audit.colliderDetails.Add(new ColliderAudit {
                        path = HierarchyPath(transform),
                        colliderType = collider.GetType().Name,
                        layer = gameObject.layer,
                        layerName = LayerMask.LayerToName(gameObject.layer),
                        trigger = collider.isTrigger,
                        bodyType = body == null ? "Static" : body.GetType().Name,
                        bodyPath = body == null ? "" : HierarchyPath(body.transform),
                        tag = gameObject.tag,
                        prefabAsset = PrefabUtility.GetPrefabAssetPathOfNearestInstanceRoot(gameObject),
                        ancestorBehaviours = AncestorBehaviours(transform)
                    });
                }
            }
        }

        for (int layer = 0; layer < 32; layer++) {
            if (objectCounts[layer] == 0 && colliderCounts[layer] == 0) continue;
            audit.objectLayers.Add(new LayerCount {
                layer = layer,
                name = LayerMask.LayerToName(layer),
                gameObjects = objectCounts[layer],
                colliders = colliderCounts[layer]
            });
        }
        audit.valid = audit.misclassifiedColliders == 0;
        return audit;
    }

    private static Dictionary<string, int> DiscoverPrefabLayers() {
        var layers = new Dictionary<string, int>(StringComparer.Ordinal);
        foreach (string scenePath in ProductionScenes) {
            Scene scene = EditorSceneManager.OpenScene(scenePath, OpenSceneMode.Single);
            foreach (Collider collider in SceneColliders(scene)) {
                string prefabPath = PrefabUtility.GetPrefabAssetPathOfNearestInstanceRoot(
                    collider.gameObject);
                if (!prefabPath.EndsWith(".prefab", StringComparison.OrdinalIgnoreCase)) continue;
                int layer = Classify(collider);
                if (layers.TryGetValue(prefabPath, out int existing) && existing != layer)
                    throw new InvalidOperationException(
                        $"Collision prefab {prefabPath} has conflicting layer ownership " +
                        $"{LayerMask.LayerToName(existing)} and {LayerMask.LayerToName(layer)}.");
                layers[prefabPath] = layer;
            }
        }
        return layers;
    }

    private static void MigratePrefab(string prefabPath, int layer) {
        GameObject root = PrefabUtility.LoadPrefabContents(prefabPath);
        try {
            bool changed = false;
            foreach (Collider collider in root.GetComponentsInChildren<Collider>(true)) {
                if (collider.gameObject.layer == layer) continue;
                collider.gameObject.layer = layer;
                changed = true;
            }
            if (changed) PrefabUtility.SaveAsPrefabAsset(root, prefabPath);
        }
        finally {
            PrefabUtility.UnloadPrefabContents(root);
        }
    }

    private static void MigrateScene(string scenePath) {
        Scene scene = EditorSceneManager.OpenScene(scenePath, OpenSceneMode.Single);
        bool changed = false;
        foreach (Collider collider in SceneColliders(scene)) {
            int layer = Classify(collider);
            if (collider.gameObject.layer == layer) continue;
            collider.gameObject.layer = layer;
            EditorUtility.SetDirty(collider.gameObject);
            PrefabUtility.RecordPrefabInstancePropertyModifications(collider.gameObject);
            changed = true;
        }
        if (changed) EditorSceneManager.SaveScene(scene);
    }

    private static IEnumerable<Collider> SceneColliders(Scene scene) {
        foreach (GameObject root in scene.GetRootGameObjects())
            foreach (Collider collider in root.GetComponentsInChildren<Collider>(true))
                yield return collider;
    }

    private static int Classify(Collider collider) {
        if (collider.GetComponentInParent<ArticulationBody>(true) != null)
            return CraneCollisionLayers.Vehicle;
        if (collider.isTrigger)
            return CraneCollisionLayers.SimulationTrigger;
        if (collider.attachedRigidbody != null ||
            HasAncestorBehaviour(collider.transform, "Sim.Physics.Misc.SimpleFloat"))
            return CraneCollisionLayers.DynamicObstacle;
        return CraneCollisionLayers.Environment;
    }

    private static bool HasAncestorBehaviour(Transform transform, string fullName) {
        for (Transform current = transform; current != null; current = current.parent) {
            foreach (MonoBehaviour behaviour in current.GetComponents<MonoBehaviour>()) {
                if (behaviour != null && behaviour.GetType().FullName == fullName) return true;
            }
        }
        return false;
    }

    private static string HierarchyPath(Transform transform) {
        var names = new Stack<string>();
        for (Transform current = transform; current != null; current = current.parent)
            names.Push(current.name);
        return string.Join("/", names);
    }

    private static List<string> AncestorBehaviours(Transform transform) {
        var names = new SortedSet<string>(StringComparer.Ordinal);
        for (Transform current = transform; current != null; current = current.parent) {
            foreach (MonoBehaviour behaviour in current.GetComponents<MonoBehaviour>()) {
                if (behaviour != null) names.Add(behaviour.GetType().FullName);
            }
        }
        return new List<string>(names);
    }

    private static string ReadArgument(string key) {
        string[] args = Environment.GetCommandLineArgs();
        int index = Array.IndexOf(args, key);
        return index >= 0 && index + 1 < args.Length ? args[index + 1] : null;
    }
}
#endif
