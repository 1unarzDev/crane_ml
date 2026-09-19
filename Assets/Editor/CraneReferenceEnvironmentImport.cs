#if UNITY_EDITOR
using System;
using System.IO;
using System.Security.Cryptography;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using Sim.Physics.Contacts;
using Sim.Physics.Land;
using Sim.Sensors.Lidar;
using Sim.Utils.ReferenceEnvironments;
using Sim.Utils.ROS;

public static class CraneReferenceEnvironmentImport {
    [Serializable] private sealed class ImportManifest {
        public string environmentId;
        public ImportSource source;
        public ImportObject[] objects;
    }

    [Serializable] private sealed class ImportSource {
        public string version;
    }

    [Serializable] private sealed class ImportObject {
        public string semanticId;
        public string semanticRole;
        public ImportPose unityPose;
        public ImportGeometry[] collisions;
        public ImportGeometry[] visuals;
    }

    [Serializable] private sealed class ImportPose {
        public float[] position;
        public float[] eulerDegrees;
    }

    [Serializable] private sealed class ImportGeometry {
        public string name;
        public ImportAsset asset;
        public float[] unityScale;
    }

    [Serializable] private sealed class ImportAsset {
        public string unityAssetPath;
    }

    public static void CreateImportedReferenceScene() {
        string manifestPath = ReadArgument("--crane-reference-manifest");
        string output = ReadArgument("--crane-reference-output-scene") ??
            "Assets/Generated/ReferenceEnvironments/Imported Reference Validation.unity";
        if (string.IsNullOrWhiteSpace(manifestPath) || !File.Exists(manifestPath))
            throw new FileNotFoundException("Reference manifest not found", manifestPath);
        if (!output.StartsWith("Assets/", StringComparison.Ordinal))
            throw new ArgumentException("Output scene must be below Assets/", nameof(output));

        AssetDatabase.Refresh(ImportAssetOptions.ForceSynchronousImport);
        var manifest = JsonUtility.FromJson<ImportManifest>(File.ReadAllText(manifestPath));
        if (manifest == null || string.IsNullOrWhiteSpace(manifest.environmentId) ||
            manifest.objects == null || manifest.objects.Length == 0)
            throw new InvalidDataException("Reference manifest is empty or invalid.");

        const string sourceScene = "Assets/Scenes/Land Vehicle Validation.unity";
        Scene scene = EditorSceneManager.OpenScene(sourceScene, OpenSceneMode.Single);
        foreach (AckermannRoverDynamics rover in UnityEngine.Object.FindObjectsByType<
                     AckermannRoverDynamics>(FindObjectsInactive.Include))
            UnityEngine.Object.DestroyImmediate(rover.gameObject);
        foreach (Collider collider in UnityEngine.Object.FindObjectsByType<Collider>(
                     FindObjectsInactive.Include)) {
            if (collider.gameObject.name.Contains("Ground", StringComparison.OrdinalIgnoreCase))
                UnityEngine.Object.DestroyImmediate(collider.gameObject);
        }

        var environment = new GameObject("Imported Reference Environment");
        environment.AddComponent<CraneImportedReferenceEnvironment>().Configure(
            manifest.environmentId, HashFile(manifestPath), manifest.source?.version ?? string.Empty,
            manifest.objects.Length);
        var canonical = new GameObject("CanonicalGeometry").transform;
        canonical.SetParent(environment.transform, false);
        var visual = new GameObject("VisualPresentation").transform;
        visual.SetParent(environment.transform, false);
        var material = new Material(Shader.Find("HDRP/Lit") ?? Shader.Find("Standard")) {
            name = "CRANE Imported Reference Neutral Material",
            color = new Color(0.42f, 0.46f, 0.40f)
        };

        foreach (ImportObject value in manifest.objects) {
            if (value.collisions != null && value.collisions.Length > 0) {
                Transform model = CreateModelRoot(canonical, value);
                foreach (ImportGeometry geometry in value.collisions)
                    InstantiateGeometry(model, geometry, true, material);
            }
            if (value.visuals != null && value.visuals.Length > 0) {
                Transform model = CreateModelRoot(visual, value);
                foreach (ImportGeometry geometry in value.visuals)
                    InstantiateGeometry(model, geometry, false, material);
            }
        }
        CreateJackalReference(manifest.environmentId);
        if (UnityEngine.Object.FindAnyObjectByType<ROSClock>() == null)
            new GameObject("CRANE ROS Clock").AddComponent<ROSClock>();

        Directory.CreateDirectory(Path.GetDirectoryName(output) ?? "Assets");
        EditorSceneManager.SaveScene(scene, output);
        AssetDatabase.SaveAssets();
        Debug.Log($"CRANE_IMPORTED_REFERENCE_SCENE_CREATED path={output} " +
                  $"id={manifest.environmentId} objects={manifest.objects.Length}");
    }

    private static Transform CreateModelRoot(Transform parent, ImportObject value) {
        var model = new GameObject(value.semanticId);
        model.transform.SetParent(parent, false);
        model.transform.localPosition = Vector(value.unityPose.position);
        model.transform.localRotation = Quaternion.Euler(Vector(value.unityPose.eulerDegrees));
        CraneImportedReferenceEnvironment environment =
            parent.GetComponentInParent<CraneImportedReferenceEnvironment>();
        model.AddComponent<CraneSemanticIdentity>().Configure(value.semanticId,
            value.semanticRole, environment.EnvironmentId);
        return model.transform;
    }

    private static void InstantiateGeometry(Transform parent, ImportGeometry geometry,
        bool collision, Material material) {
        GameObject source = AssetDatabase.LoadAssetAtPath<GameObject>(geometry.asset.unityAssetPath);
        if (source == null)
            throw new FileNotFoundException("Unity failed to import reference mesh",
                geometry.asset.unityAssetPath);
        var instance = (GameObject)PrefabUtility.InstantiatePrefab(source);
        instance.name = geometry.name;
        instance.transform.SetParent(parent, false);
        instance.transform.localScale = Vector(geometry.unityScale);
        if (collision) {
            foreach (Renderer renderer in instance.GetComponentsInChildren<Renderer>(true))
                UnityEngine.Object.DestroyImmediate(renderer);
            foreach (Collider oldCollider in instance.GetComponentsInChildren<Collider>(true))
                UnityEngine.Object.DestroyImmediate(oldCollider);
            foreach (MeshFilter filter in instance.GetComponentsInChildren<MeshFilter>(true)) {
                filter.gameObject.layer = CraneCollisionLayers.Environment;
                var collider = filter.gameObject.AddComponent<MeshCollider>();
                collider.sharedMesh = filter.sharedMesh;
                collider.convex = false;
            }
        } else {
            foreach (Collider collider in instance.GetComponentsInChildren<Collider>(true))
                UnityEngine.Object.DestroyImmediate(collider);
            foreach (Renderer renderer in instance.GetComponentsInChildren<Renderer>(true))
                renderer.sharedMaterial = material;
        }
    }

    private static void CreateJackalReference(string environmentId) {
        var robot = new GameObject("Clearpath Jackal-class Reference");
        robot.layer = CraneCollisionLayers.Vehicle;
        robot.transform.position = new Vector3(0f, 5f, 0f);
        var body = robot.AddComponent<Rigidbody>();
        body.mass = 17f;
        body.linearDamping = 0.05f;
        body.angularDamping = 0.15f;
        body.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
        body.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        var collider = robot.AddComponent<BoxCollider>();
        collider.size = new Vector3(0.43f, 0.20f, 0.508f);
        collider.material = new PhysicsMaterial("Jackal Chassis Slip") {
            dynamicFriction = 0f,
            staticFriction = 0f,
            frictionCombine = PhysicsMaterialCombine.Minimum
        };
        robot.AddComponent<DifferentialDriveDynamics>().Configure(0.37559f, 0.098f, 2f, 2f);
        robot.AddComponent<CraneSemanticIdentity>().Configure("robot-clearpath-jackal-class",
            "outdoor-differential-mobile-robot", environmentId);
        CreateVisual(robot.transform, "chassis-visual", PrimitiveType.Cube,
            new Vector3(0f, 0f, 0f), new Vector3(0.43f, 0.20f, 0.508f),
            new Color(0.95f, 0.48f, 0.05f), Quaternion.identity);
        var lidarHost = new GameObject("base_scan");
        lidarHost.transform.SetParent(robot.transform, false);
        lidarHost.transform.localPosition = new Vector3(0f, 0.22f, 0f);
        lidarHost.AddComponent<Lidar2D>().Configure(-180f, 180f, 0.5f, 0.12f, 20f,
            180, false, "/scan", "base_scan", 10f);
        CreateVisual(lidarHost.transform, "lidar-visual", PrimitiveType.Cylinder, Vector3.zero,
            new Vector3(0.09f, 0.04f, 0.09f), Color.black, Quaternion.identity);
    }

    private static void CreateVisual(Transform parent, string name, PrimitiveType primitive,
        Vector3 position, Vector3 scale, Color color, Quaternion rotation) {
        GameObject value = GameObject.CreatePrimitive(primitive);
        value.name = name;
        value.transform.SetParent(parent, false);
        value.transform.localPosition = position;
        value.transform.localRotation = rotation;
        value.transform.localScale = scale;
        UnityEngine.Object.DestroyImmediate(value.GetComponent<Collider>());
        value.GetComponent<Renderer>().sharedMaterial = new Material(
            Shader.Find("HDRP/Lit") ?? Shader.Find("Standard")) { color = color };
    }

    private static Vector3 Vector(float[] values) {
        if (values == null || values.Length != 3) throw new InvalidDataException("Expected vector3.");
        return new Vector3(values[0], values[1], values[2]);
    }

    private static string HashFile(string path) {
        using SHA256 sha = SHA256.Create();
        using FileStream stream = File.OpenRead(path);
        return BitConverter.ToString(sha.ComputeHash(stream)).Replace("-", string.Empty);
    }

    private static string ReadArgument(string key) {
        string[] args = Environment.GetCommandLineArgs();
        int index = Array.IndexOf(args, key);
        return index >= 0 && index + 1 < args.Length ? args[index + 1] : null;
    }
}
#endif
