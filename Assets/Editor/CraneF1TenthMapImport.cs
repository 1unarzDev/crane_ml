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

/// <summary>Constructs a Unity scene offline from a canonical F1TENTH occupancy-map manifest.</summary>
public static class CraneF1TenthMapImport {
    [Serializable] private sealed class Manifest {
        public string environmentId;
        public Source source;
        public Collision canonicalCollision;
        public Spawn robotSpawn;
    }

    [Serializable] private sealed class Source {
        public string upstreamVersion;
    }

    [Serializable] private sealed class Collision {
        public BoxRecord floor;
        public BoxRecord[] segments;
    }

    [Serializable] private sealed class BoxRecord {
        public string semanticId;
        public string semanticRole;
        public UnityCollider unityCollider;
    }

    [Serializable] private sealed class UnityCollider {
        public float[] center;
        public float[] size;
        public float yawDegrees;
    }

    [Serializable] private sealed class Spawn {
        public UnitySpawn unity;
    }

    [Serializable] private sealed class UnitySpawn {
        public float[] position;
        public float yawDegrees;
    }

    public static void CreateScene() {
        string manifestPath = ReadArgument("--crane-f1tenth-manifest");
        string output = ReadArgument("--crane-f1tenth-output-scene") ??
            "Assets/Generated/ReferenceEnvironments/F1TENTH Validation.unity";
        if (string.IsNullOrWhiteSpace(manifestPath) || !File.Exists(manifestPath))
            throw new FileNotFoundException("F1TENTH canonical manifest not found", manifestPath);
        if (!output.StartsWith("Assets/", StringComparison.Ordinal))
            throw new ArgumentException("Output scene must be below Assets/", nameof(output));

        Manifest manifest = JsonUtility.FromJson<Manifest>(File.ReadAllText(manifestPath));
        if (manifest == null || string.IsNullOrWhiteSpace(manifest.environmentId) ||
            manifest.canonicalCollision?.floor == null ||
            manifest.canonicalCollision.segments == null)
            throw new InvalidDataException("Incomplete crane-f1tenth-map-v1 manifest.");
        CranePerformanceBuild.ConfigureCollisionLayers();
        Directory.CreateDirectory(Path.GetDirectoryName(output) ?? "Assets/Generated");
        Scene scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

        var root = new GameObject("F1TENTH Reference Environment");
        root.AddComponent<CraneImportedReferenceEnvironment>().Configure(
            manifest.environmentId, HashFile(manifestPath),
            manifest.source?.upstreamVersion ?? "unversioned",
            manifest.canonicalCollision.segments.Length + 1);
        Transform canonical = new GameObject("CanonicalGeometry").transform;
        canonical.SetParent(root.transform, false);
        Transform visual = new GameObject("VisualPresentation").transform;
        visual.SetParent(root.transform, false);
        var floorMaterial = CreateMaterial("F1TENTH Floor", new Color(0.18f, 0.19f, 0.20f));
        var wallMaterial = CreateMaterial("F1TENTH Walls", new Color(0.75f, 0.13f, 0.10f));
        CreateBox(manifest.canonicalCollision.floor, canonical, visual, floorMaterial,
            manifest.environmentId);
        foreach (BoxRecord segment in manifest.canonicalCollision.segments)
            CreateBox(segment, canonical, visual, wallMaterial, manifest.environmentId);

        CreateF1TenthRobot(manifest.robotSpawn, manifest.environmentId);
        CreateLightingAndCamera(manifest.canonicalCollision.floor.unityCollider);
        EditorSceneManager.SaveScene(scene, output);
        AssetDatabase.SaveAssets();
        Debug.Log($"CRANE_F1TENTH_SCENE_CREATED path={output} " +
                  $"environment={manifest.environmentId} walls={manifest.canonicalCollision.segments.Length}");
    }

    private static void CreateBox(BoxRecord record, Transform canonical, Transform visual,
        Material material, string environmentId) {
        if (record?.unityCollider == null)
            throw new InvalidDataException("Canonical box record lacks unityCollider.");
        Vector3 center = Vector(record.unityCollider.center);
        Vector3 size = Vector(record.unityCollider.size);
        Quaternion rotation = Quaternion.Euler(0f, record.unityCollider.yawDegrees, 0f);
        var collision = new GameObject(record.semanticId);
        collision.layer = CraneCollisionLayers.Environment;
        collision.transform.SetParent(canonical, false);
        collision.transform.localPosition = center;
        collision.transform.localRotation = rotation;
        collision.AddComponent<BoxCollider>().size = size;
        collision.AddComponent<CraneSemanticIdentity>().Configure(record.semanticId,
            record.semanticRole, environmentId);

        GameObject presentation = GameObject.CreatePrimitive(PrimitiveType.Cube);
        presentation.name = record.semanticId + "-visual";
        presentation.transform.SetParent(visual, false);
        presentation.transform.localPosition = center;
        presentation.transform.localRotation = rotation;
        presentation.transform.localScale = size;
        UnityEngine.Object.DestroyImmediate(presentation.GetComponent<Collider>());
        presentation.GetComponent<Renderer>().sharedMaterial = material;
    }

    private static void CreateF1TenthRobot(Spawn spawn, string environmentId) {
        Vector3 position = spawn?.unity?.position?.Length == 3
            ? Vector(spawn.unity.position) : new Vector3(0f, 0.16f, 0f);
        position.y = Mathf.Max(position.y, 0.20f);
        float yaw = spawn?.unity == null ? 0f : spawn.unity.yawDegrees;
        var robot = new GameObject("F1TENTH Ackermann Reference");
        robot.layer = CraneCollisionLayers.Vehicle;
        robot.transform.SetPositionAndRotation(position, Quaternion.Euler(0f, yaw, 0f));
        var body = robot.AddComponent<Rigidbody>();
        body.mass = 3.47f;
        body.linearDamping = 0.02f;
        body.angularDamping = 0.12f;
        body.centerOfMass = new Vector3(0f, -0.04f, 0f);
        // The source benchmark is a planar occupancy-map simulator. Roll and pitch are outside
        // that contract and destabilize a small WheelCollider chassis without adding fidelity.
        body.constraints = RigidbodyConstraints.FreezeRotationX |
            RigidbodyConstraints.FreezeRotationZ;
        body.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        var chassis = robot.AddComponent<BoxCollider>();
        chassis.size = new Vector3(0.30f, 0.12f, 0.45f);
        robot.AddComponent<CraneSemanticIdentity>().Configure("robot-f1tenth-ackermann",
            "ackermann-mobile-robot", environmentId);

        const float wheelbase = 0.32f;
        const float track = 0.24f;
        WheelCollider fl = CreateWheel(robot.transform, "front-left-wheel", -track * 0.5f,
            wheelbase * 0.5f);
        WheelCollider fr = CreateWheel(robot.transform, "front-right-wheel", track * 0.5f,
            wheelbase * 0.5f);
        WheelCollider rl = CreateWheel(robot.transform, "rear-left-wheel", -track * 0.5f,
            -wheelbase * 0.5f);
        WheelCollider rr = CreateWheel(robot.transform, "rear-right-wheel", track * 0.5f,
            -wheelbase * 0.5f);
        robot.AddComponent<AckermannRoverDynamics>().Configure(fl, fr, rl, rr, wheelbase, track);
        CreateVisual(robot.transform, "chassis-visual", PrimitiveType.Cube, Vector3.zero,
            new Vector3(0.30f, 0.12f, 0.45f), new Color(0.08f, 0.25f, 0.65f), Quaternion.identity);

        var lidarHost = new GameObject("base_scan");
        lidarHost.transform.SetParent(robot.transform, false);
        lidarHost.transform.localPosition = new Vector3(0f, 0.10f, 0.04f);
        lidarHost.AddComponent<Lidar2D>().Configure(-135f, 135f, 0.25f, 0.05f, 30f,
            1080, false, "/scan", "base_scan", 10f);
        CreateVisual(lidarHost.transform, "lidar-visual", PrimitiveType.Cylinder, Vector3.zero,
            new Vector3(0.055f, 0.025f, 0.055f), Color.black, Quaternion.identity);
    }

    private static WheelCollider CreateWheel(Transform parent, string name, float x, float z) {
        var host = new GameObject(name);
        host.layer = CraneCollisionLayers.Vehicle;
        host.transform.SetParent(parent, false);
        host.transform.localPosition = new Vector3(x, -0.07f, z);
        var wheel = host.AddComponent<WheelCollider>();
        wheel.mass = 0.25f;
        wheel.radius = 0.05f;
        wheel.suspensionDistance = 0.08f;
        JointSpring spring = wheel.suspensionSpring;
        spring.spring = 300f;
        spring.damper = 40f;
        spring.targetPosition = 0.5f;
        wheel.suspensionSpring = spring;
        WheelFrictionCurve forward = wheel.forwardFriction;
        forward.stiffness = 1.5f;
        wheel.forwardFriction = forward;
        WheelFrictionCurve sideways = wheel.sidewaysFriction;
        sideways.stiffness = 1.8f;
        wheel.sidewaysFriction = sideways;
        CreateVisual(host.transform, "wheel-visual", PrimitiveType.Cylinder, Vector3.zero,
            new Vector3(0.10f, 0.025f, 0.10f), Color.black, Quaternion.Euler(0f, 0f, 90f));
        return wheel;
    }

    private static void CreateLightingAndCamera(UnityCollider floor) {
        var lightHost = new GameObject("Directional Light");
        Light light = lightHost.AddComponent<Light>();
        light.type = LightType.Directional;
        light.intensity = 1.2f;
        lightHost.transform.rotation = Quaternion.Euler(50f, -35f, 0f);
        Vector3 center = Vector(floor.center);
        Vector3 size = Vector(floor.size);
        float extent = Mathf.Max(size.x, size.z);
        var cameraHost = new GameObject("Spectator Camera");
        Camera camera = cameraHost.AddComponent<Camera>();
        cameraHost.tag = "MainCamera";
        cameraHost.transform.position = center + new Vector3(0f, extent * 0.65f, -extent * 0.25f);
        cameraHost.transform.LookAt(center);
        camera.farClipPlane = Mathf.Max(200f, extent * 2f);
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
        value.GetComponent<Renderer>().sharedMaterial = CreateMaterial(name + " material", color);
    }

    private static Material CreateMaterial(string name, Color color) => new(
        Shader.Find("HDRP/Lit") ?? Shader.Find("Standard")) { name = name, color = color };

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
