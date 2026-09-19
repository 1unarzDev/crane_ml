#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using Sim.Physics.Land;
using Sim.Physics.Aerial;
using Sim.Physics.Contacts;
using Sim.Sensors.Lidar;
using Sim.Utils.ReferenceEnvironments;
using Sim.Utils.ROS;

public static class CranePerformanceBuild {
    [Serializable] private sealed class BuildAssetHash {
        public string path;
        public string sha256;
        public long bytes;
    }

    [Serializable] private sealed class BuildManifest {
        public string schema = "crane-build-manifest-v1";
        public string unityVersion;
        public string buildGuid;
        public string packageManifestHash;
        public string packageLockHash;
        public string projectVersionHash;
        public string projectSettingsHash;
        public string assetSetHash;
        public string[] scenes;
        public List<BuildAssetHash> assets = new();
    }

    private static readonly string[] Scenes = {
        "Assets/Scenes/Robosub Pool.unity",
        "Assets/Scenes/Roboboat Course.unity",
        "Assets/Scenes/Land Vehicle Validation.unity",
        "Assets/Scenes/TurtleBot3 Warehouse Validation.unity",
        "Assets/Scenes/Aerial Vehicle Validation.unity",
        "Assets/Scenes/PX4 Walls Validation.unity",
        "Assets/Scenes/PX4 ArUco Validation.unity",
        "Assets/Scenes/PX4 Windy Validation.unity",
        "Assets/Scenes/Collision Validation.unity"
    };

    public static void CreateTurtleBot3WarehouseScene() {
        const string source = "Assets/Scenes/Land Vehicle Validation.unity";
        const string output = "Assets/Scenes/TurtleBot3 Warehouse Validation.unity";
        Scene scene = EditorSceneManager.OpenScene(source, OpenSceneMode.Single);
        foreach (AckermannRoverDynamics rover in UnityEngine.Object.FindObjectsByType<
                     AckermannRoverDynamics>(FindObjectsInactive.Include))
            UnityEngine.Object.DestroyImmediate(rover.gameObject);

        foreach (Collider collider in UnityEngine.Object.FindObjectsByType<Collider>(
                     FindObjectsInactive.Include)) {
            if (collider.gameObject.name.Contains("Ground", StringComparison.OrdinalIgnoreCase))
                UnityEngine.Object.DestroyImmediate(collider.gameObject);
        }

        var environment = new GameObject("Reference Environment");
        environment.AddComponent<CraneReferenceWarehouse>().Configure(1000, 12f, 18f, 3, 2, true);
        environment.GetComponent<CraneReferenceWarehouse>().Generate();

        var robot = new GameObject("TurtleBot3 Waffle Reference");
        robot.transform.position = new Vector3(0f, 0.08f, 0.8f);
        var body = robot.AddComponent<Rigidbody>();
        body.mass = 1.3729096f;
        body.linearDamping = 0.05f;
        body.angularDamping = 0.15f;
        body.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
        var baseCollider = robot.AddComponent<BoxCollider>();
        baseCollider.center = new Vector3(0f, 0.047f, -0.064f);
        baseCollider.size = new Vector3(0.266f, 0.094f, 0.266f);
        baseCollider.material = new PhysicsMaterial("TurtleBot3 Chassis Slip") {
            dynamicFriction = 0f,
            staticFriction = 0f,
            frictionCombine = PhysicsMaterialCombine.Minimum
        };
        robot.AddComponent<DifferentialDriveDynamics>().Configure(0.287f, 0.033f, 0.26f, 1.82f);
        robot.AddComponent<CraneSemanticIdentity>().Configure("robot-turtlebot3-waffle",
            "mobile-robot", CraneReferenceWarehouse.EnvironmentId);

        CreateRobotVisual(robot.transform, "base-visual", PrimitiveType.Cube,
            new Vector3(0f, 0.075f, -0.064f), new Vector3(0.266f, 0.12f, 0.266f),
            new Color(0.08f, 0.10f, 0.12f), Quaternion.identity);
        CreateRobotVisual(robot.transform, "deck-visual", PrimitiveType.Cube,
            new Vector3(0f, 0.22f, -0.04f), new Vector3(0.22f, 0.025f, 0.20f),
            new Color(0.05f, 0.20f, 0.50f), Quaternion.identity);
        CreateRobotVisual(robot.transform, "wheel-left-visual", PrimitiveType.Cylinder,
            new Vector3(-0.144f, 0.033f, 0f), new Vector3(0.066f, 0.018f, 0.066f),
            Color.black, Quaternion.Euler(0f, 0f, 90f));
        CreateRobotVisual(robot.transform, "wheel-right-visual", PrimitiveType.Cylinder,
            new Vector3(0.144f, 0.033f, 0f), new Vector3(0.066f, 0.018f, 0.066f),
            Color.black, Quaternion.Euler(0f, 0f, 90f));

        var lidarHost = new GameObject("base_scan");
        lidarHost.transform.SetParent(robot.transform, false);
        lidarHost.transform.localPosition = new Vector3(0f, 0.23f, 0f);
        lidarHost.AddComponent<Lidar2D>().Configure(-180f, 180f, 1f, 0.12f, 3.5f,
            180, false, "/scan", "base_scan", 5f);
        CreateRobotVisual(lidarHost.transform, "lidar-visual", PrimitiveType.Cylinder,
            Vector3.zero, new Vector3(0.07f, 0.035f, 0.07f), Color.black,
            Quaternion.identity);

        if (UnityEngine.Object.FindAnyObjectByType<ROSClock>() == null)
            new GameObject("CRANE ROS Clock").AddComponent<ROSClock>();
        EditorSceneManager.SaveScene(scene, output);
        AssetDatabase.SaveAssets();
        Debug.Log($"CRANE_REFERENCE_SCENE_CREATED path={output}");
    }

    private static void CreateRobotVisual(Transform parent, string name, PrimitiveType primitive,
        Vector3 localPosition, Vector3 localScale, Color color, Quaternion localRotation) {
        GameObject value = GameObject.CreatePrimitive(primitive);
        value.name = name;
        value.transform.SetParent(parent, false);
        value.transform.localPosition = localPosition;
        value.transform.localRotation = localRotation;
        value.transform.localScale = localScale;
        Collider collider = value.GetComponent<Collider>();
        if (collider != null) UnityEngine.Object.DestroyImmediate(collider);
        value.GetComponent<Renderer>().sharedMaterial = new Material(
            Shader.Find("HDRP/Lit") ?? Shader.Find("Standard")) { color = color };
    }

    public static void BuildLinuxWorker() {
        string output = ReadArgument("--crane-build-output", "Builds/CRANE-Worker/CRANE.x86_64");
        var buildScenes = new List<string>(Scenes);
        string extraScene = ReadArgument("--crane-extra-scene", string.Empty);
        if (!string.IsNullOrWhiteSpace(extraScene)) {
            if (!File.Exists(extraScene)) throw new FileNotFoundException("Extra scene not found", extraScene);
            buildScenes.Add(extraScene.Replace('\\', '/'));
        }
        Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(output)) ?? ".");
        var options = new BuildPlayerOptions {
            scenes = buildScenes.ToArray(),
            locationPathName = output,
            target = BuildTarget.StandaloneLinux64,
            options = BuildOptions.Development
        };
        BuildReport report = BuildPipeline.BuildPlayer(options);
        if (report.summary.result != BuildResult.Succeeded)
            throw new Exception($"CRANE worker build failed: {report.summary.result}");
        WriteBuildManifest(output, report, buildScenes.ToArray());
        Debug.Log($"CRANE_BUILD_COMPLETE path={output} bytes={report.summary.totalSize}");
    }

    private static void WriteBuildManifest(string output, BuildReport report, string[] buildScenes) {
        var manifest = new BuildManifest {
            unityVersion = Application.unityVersion,
            buildGuid = report.summary.guid.ToString(),
            packageManifestHash = HashFile("Packages/manifest.json"),
            packageLockHash = HashFile("Packages/packages-lock.json"),
            projectVersionHash = HashFile("ProjectSettings/ProjectVersion.txt"),
            projectSettingsHash = HashFiles(Directory.GetFiles("ProjectSettings", "*",
                SearchOption.AllDirectories)),
            scenes = (string[])buildScenes.Clone()
        };

        var dependencies = new SortedSet<string>(StringComparer.Ordinal);
        foreach (string scene in buildScenes) {
            foreach (string dependency in AssetDatabase.GetDependencies(scene, true)) {
                if (dependency.StartsWith("Assets/", StringComparison.Ordinal) && File.Exists(dependency))
                    dependencies.Add(dependency.Replace('\\', '/'));
            }
        }
        foreach (string dependency in dependencies) {
            var info = new FileInfo(dependency);
            manifest.assets.Add(new BuildAssetHash {
                path = dependency,
                sha256 = HashFile(dependency),
                bytes = info.Length
            });
        }
        manifest.assetSetHash = HashAssetEntries(manifest.assets);

        string outputDirectory = Path.GetDirectoryName(Path.GetFullPath(output)) ?? ".";
        string manifestPath = Path.Combine(outputDirectory, "crane-build-manifest.json");
        File.WriteAllText(manifestPath, JsonUtility.ToJson(manifest, true));
        Debug.Log($"CRANE_BUILD_MANIFEST path={manifestPath} assets={manifest.assets.Count} " +
                  $"assetSetHash={manifest.assetSetHash}");
    }

    private static string HashFile(string path) {
        if (!File.Exists(path)) return string.Empty;
        using SHA256 sha = SHA256.Create();
        using FileStream stream = File.OpenRead(path);
        return ToHex(sha.ComputeHash(stream));
    }

    private static string HashFiles(IEnumerable<string> paths) {
        var sorted = new List<string>(paths);
        sorted.Sort(StringComparer.Ordinal);
        var canonical = new StringBuilder();
        foreach (string path in sorted) {
            string normalized = path.Replace('\\', '/');
            canonical.Append(normalized).Append('\0').Append(HashFile(path)).Append('\n');
        }
        return HashText(canonical.ToString());
    }

    private static string HashAssetEntries(IEnumerable<BuildAssetHash> entries) {
        var canonical = new StringBuilder();
        foreach (BuildAssetHash entry in entries)
            canonical.Append(entry.path).Append('\0').Append(entry.sha256).Append('\0')
                .Append(entry.bytes).Append('\n');
        return HashText(canonical.ToString());
    }

    private static string HashText(string value) {
        using SHA256 sha = SHA256.Create();
        return ToHex(sha.ComputeHash(Encoding.UTF8.GetBytes(value)));
    }

    private static string ToHex(byte[] value) =>
        BitConverter.ToString(value).Replace("-", string.Empty);

    public static void CreateLandValidationScene() {
        ConfigureCollisionLayers();
        const string scenePath = "Assets/Scenes/Land Vehicle Validation.unity";
        var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

        var ground = GameObject.CreatePrimitive(PrimitiveType.Cube);
        ground.name = "Validation Ground";
        ground.layer = CraneCollisionLayers.Environment;
        ground.transform.position = new Vector3(0f, -0.5f, 0f);
        ground.transform.localScale = new Vector3(100f, 1f, 100f);

        var roverObject = new GameObject("Reference Ackermann Rover");
        roverObject.layer = CraneCollisionLayers.Vehicle;
        roverObject.transform.position = new Vector3(0f, 0.65f, 0f);
        var body = roverObject.AddComponent<Rigidbody>();
        body.mass = 60f;
        body.linearDamping = 0.02f;
        body.angularDamping = 0.15f;
        body.centerOfMass = new Vector3(0f, -0.22f, 0f);
        body.collisionDetectionMode = CollisionDetectionMode.Discrete;
        var chassisCollider = roverObject.AddComponent<BoxCollider>();
        chassisCollider.size = new Vector3(1.35f, 0.45f, 2f);

        var chassisVisual = GameObject.CreatePrimitive(PrimitiveType.Cube);
        chassisVisual.name = "Chassis Visual";
        UnityEngine.Object.DestroyImmediate(chassisVisual.GetComponent<Collider>());
        chassisVisual.transform.SetParent(roverObject.transform, false);
        chassisVisual.transform.localScale = new Vector3(1.3f, 0.4f, 1.9f);

        const float wheelbase = 1.6f;
        const float track = 1.2f;
        WheelCollider fl = CreateWheel(roverObject.transform, "Front Left Wheel", -track * 0.5f, wheelbase * 0.5f);
        WheelCollider fr = CreateWheel(roverObject.transform, "Front Right Wheel", track * 0.5f, wheelbase * 0.5f);
        WheelCollider rl = CreateWheel(roverObject.transform, "Rear Left Wheel", -track * 0.5f, -wheelbase * 0.5f);
        WheelCollider rr = CreateWheel(roverObject.transform, "Rear Right Wheel", track * 0.5f, -wheelbase * 0.5f);
        var dynamics = roverObject.AddComponent<AckermannRoverDynamics>();
        dynamics.Configure(fl, fr, rl, rr, wheelbase, track);

        var lightObject = new GameObject("Directional Light");
        var light = lightObject.AddComponent<Light>();
        light.type = LightType.Directional;
        light.intensity = 1.2f;
        lightObject.transform.rotation = Quaternion.Euler(45f, -35f, 0f);

        var cameraObject = new GameObject("Spectator Camera");
        var camera = cameraObject.AddComponent<Camera>();
        cameraObject.tag = "MainCamera";
        cameraObject.transform.position = new Vector3(8f, 5f, -10f);
        cameraObject.transform.LookAt(Vector3.zero);
        camera.farClipPlane = 200f;

        EditorSceneManager.SaveScene(scene, scenePath);
        AssetDatabase.SaveAssets();
        Debug.Log($"CRANE_LAND_SCENE_CREATED {scenePath}");
    }

    public static void CreateAerialValidationScene() {
        ConfigureCollisionLayers();
        const string scenePath = "Assets/Scenes/Aerial Vehicle Validation.unity";
        var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

        var ground = GameObject.CreatePrimitive(PrimitiveType.Cube);
        ground.name = "Validation Ground";
        ground.layer = CraneCollisionLayers.Environment;
        ground.transform.position = new Vector3(0f, -0.5f, 0f);
        ground.transform.localScale = new Vector3(50f, 1f, 50f);

        var aircraft = new GameObject("Reference Quadrotor");
        aircraft.layer = CraneCollisionLayers.Vehicle;
        aircraft.transform.position = new Vector3(0f, 5f, 0f);
        var body = aircraft.AddComponent<Rigidbody>();
        body.mass = 1.5f;
        body.linearDamping = 0f;
        body.angularDamping = 0f;
        body.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        var bodyCollider = aircraft.AddComponent<BoxCollider>();
        bodyCollider.size = new Vector3(0.5f, 0.16f, 0.5f);

        var bodyVisual = GameObject.CreatePrimitive(PrimitiveType.Cube);
        bodyVisual.name = "Airframe Visual";
        UnityEngine.Object.DestroyImmediate(bodyVisual.GetComponent<Collider>());
        bodyVisual.transform.SetParent(aircraft.transform, false);
        bodyVisual.transform.localScale = new Vector3(0.5f, 0.16f, 0.5f);

        const float arm = 0.23f;
        Transform fl = CreateRotor(aircraft.transform, "Front Left Rotor", -arm, arm);
        Transform fr = CreateRotor(aircraft.transform, "Front Right Rotor", arm, arm);
        Transform rr = CreateRotor(aircraft.transform, "Rear Right Rotor", arm, -arm);
        Transform rl = CreateRotor(aircraft.transform, "Rear Left Rotor", -arm, -arm);
        var dynamics = aircraft.AddComponent<MultirotorDynamics>();
        dynamics.Configure(fl, fr, rr, rl);

        var lightObject = new GameObject("Directional Light");
        var light = lightObject.AddComponent<Light>();
        light.type = LightType.Directional;
        light.intensity = 1.2f;
        lightObject.transform.rotation = Quaternion.Euler(45f, -35f, 0f);

        var cameraObject = new GameObject("Spectator Camera");
        var camera = cameraObject.AddComponent<Camera>();
        cameraObject.tag = "MainCamera";
        cameraObject.transform.position = new Vector3(8f, 6f, -10f);
        cameraObject.transform.LookAt(new Vector3(0f, 3f, 0f));
        camera.farClipPlane = 200f;

        EditorSceneManager.SaveScene(scene, scenePath);
        AssetDatabase.SaveAssets();
        Debug.Log($"CRANE_AERIAL_SCENE_CREATED {scenePath}");
    }

    public static void CreatePx4WallsScene() {
        const string source = "Assets/Scenes/Aerial Vehicle Validation.unity";
        const string output = "Assets/Scenes/PX4 Walls Validation.unity";
        Scene scene = EditorSceneManager.OpenScene(source, OpenSceneMode.Single);
        foreach (Collider collider in UnityEngine.Object.FindObjectsByType<Collider>(
                     FindObjectsInactive.Include)) {
            if (collider.gameObject.name.Contains("Ground", StringComparison.OrdinalIgnoreCase))
                UnityEngine.Object.DestroyImmediate(collider.gameObject);
        }

        var environment = new GameObject("PX4 Walls Reference Environment");
        var reference = environment.AddComponent<CraneReferencePx4Walls>();
        reference.Configure(true);
        reference.Generate();

        MultirotorDynamics multirotor = UnityEngine.Object.FindAnyObjectByType<MultirotorDynamics>();
        if (multirotor == null) throw new MissingReferenceException("Aerial multirotor is missing.");
        multirotor.gameObject.name = "PX4 x500-class Reference";
        multirotor.gameObject.AddComponent<CraneSemanticIdentity>().Configure("robot-px4-x500-class",
            "aerial-mobile-robot", CraneReferencePx4Walls.EnvironmentId);

        EditorSceneManager.SaveScene(scene, output);
        AssetDatabase.SaveAssets();
        Debug.Log($"CRANE_REFERENCE_SCENE_CREATED path={output}");
    }

    public static void CreatePx4ArucoScene() {
        const string source = "Assets/Scenes/Aerial Vehicle Validation.unity";
        const string output = "Assets/Scenes/PX4 ArUco Validation.unity";
        Scene scene = EditorSceneManager.OpenScene(source, OpenSceneMode.Single);
        foreach (Collider collider in UnityEngine.Object.FindObjectsByType<Collider>(
                     FindObjectsInactive.Include)) {
            if (collider.gameObject.name.Contains("Ground", StringComparison.OrdinalIgnoreCase))
                UnityEngine.Object.DestroyImmediate(collider.gameObject);
        }

        var environment = new GameObject("PX4 ArUco Reference Environment");
        var reference = environment.AddComponent<CraneReferencePx4Aruco>();
        reference.Generate();
        MultirotorDynamics multirotor = UnityEngine.Object.FindAnyObjectByType<MultirotorDynamics>();
        if (multirotor == null) throw new MissingReferenceException("Aerial multirotor is missing.");
        multirotor.gameObject.name = "PX4 x500-class Reference";
        multirotor.gameObject.AddComponent<CraneSemanticIdentity>().Configure("robot-px4-x500-class",
            "aerial-mobile-robot", CraneReferencePx4Aruco.EnvironmentId);

        EditorSceneManager.SaveScene(scene, output);
        AssetDatabase.SaveAssets();
        Debug.Log($"CRANE_REFERENCE_SCENE_CREATED path={output}");
    }

    public static void CreatePx4WindyScene() {
        const string source = "Assets/Scenes/Aerial Vehicle Validation.unity";
        const string output = "Assets/Scenes/PX4 Windy Validation.unity";
        Scene scene = EditorSceneManager.OpenScene(source, OpenSceneMode.Single);
        foreach (Collider collider in UnityEngine.Object.FindObjectsByType<Collider>(
                     FindObjectsInactive.Include)) {
            if (collider.gameObject.name.Contains("Ground", StringComparison.OrdinalIgnoreCase))
                UnityEngine.Object.DestroyImmediate(collider.gameObject);
        }
        var environment = new GameObject("PX4 Windy Reference Environment");
        environment.AddComponent<CraneReferencePx4Windy>().Generate();
        MultirotorDynamics multirotor = UnityEngine.Object.FindAnyObjectByType<MultirotorDynamics>();
        if (multirotor == null) throw new MissingReferenceException("Aerial multirotor is missing.");
        multirotor.gameObject.name = "PX4 x500-class Reference";
        multirotor.gameObject.AddComponent<CraneSemanticIdentity>().Configure("robot-px4-x500-class",
            "aerial-mobile-robot", CraneReferencePx4Windy.EnvironmentId);
        EditorSceneManager.SaveScene(scene, output);
        AssetDatabase.SaveAssets();
        Debug.Log($"CRANE_REFERENCE_SCENE_CREATED path={output}");
    }

    public static void CreateCollisionValidationScene() {
        ConfigureCollisionLayers();
        const string scenePath = "Assets/Scenes/Collision Validation.unity";
        var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        EditorSceneManager.SaveScene(scene, scenePath);
        AssetDatabase.SaveAssets();
        Debug.Log($"CRANE_COLLISION_SCENE_CREATED {scenePath}");
    }

    public static void ConfigureCollisionLayers() {
        UnityEngine.Object tagManagerAsset =
            AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/TagManager.asset")[0];
        var tagManager = new SerializedObject(tagManagerAsset);
        SerializedProperty layers = tagManager.FindProperty("layers");
        layers.GetArrayElementAtIndex(CraneCollisionLayers.Environment).stringValue = "Environment";
        layers.GetArrayElementAtIndex(CraneCollisionLayers.Vehicle).stringValue = "Vehicle";
        layers.GetArrayElementAtIndex(CraneCollisionLayers.DynamicObstacle).stringValue = "DynamicObstacle";
        layers.GetArrayElementAtIndex(CraneCollisionLayers.SensorQuery).stringValue = "SensorQuery";
        layers.GetArrayElementAtIndex(CraneCollisionLayers.SimulationTrigger).stringValue = "SimulationTrigger";
        tagManager.ApplyModifiedPropertiesWithoutUndo();

        int[] classifiedLayers = {
            CraneCollisionLayers.Environment,
            CraneCollisionLayers.Vehicle,
            CraneCollisionLayers.DynamicObstacle,
            CraneCollisionLayers.SensorQuery,
            CraneCollisionLayers.SimulationTrigger
        };
        foreach (int classifiedLayer in classifiedLayers)
            for (int otherLayer = 0; otherLayer < 32; otherLayer++)
                UnityEngine.Physics.IgnoreLayerCollision(classifiedLayer, otherLayer, true);

        SetLayerPair(CraneCollisionLayers.Environment, 0, true);
        SetLayerPair(CraneCollisionLayers.Environment, CraneCollisionLayers.Vehicle, true);
        SetLayerPair(CraneCollisionLayers.Environment, CraneCollisionLayers.DynamicObstacle, true);
        SetLayerPair(CraneCollisionLayers.Vehicle, 0, true);
        SetLayerPair(CraneCollisionLayers.Vehicle, CraneCollisionLayers.Vehicle, true);
        SetLayerPair(CraneCollisionLayers.Vehicle, CraneCollisionLayers.DynamicObstacle, true);
        SetLayerPair(CraneCollisionLayers.DynamicObstacle, 0, true);
        SetLayerPair(CraneCollisionLayers.DynamicObstacle, CraneCollisionLayers.DynamicObstacle, true);
        SetLayerPair(CraneCollisionLayers.SimulationTrigger, 0, true);
        SetLayerPair(CraneCollisionLayers.SimulationTrigger, CraneCollisionLayers.Vehicle, true);
        SetLayerPair(CraneCollisionLayers.SimulationTrigger, CraneCollisionLayers.DynamicObstacle, true);
        AssetDatabase.SaveAssets();
        Debug.Log("CRANE_COLLISION_LAYERS_CONFIGURED");
    }

    private static void SetLayerPair(int first, int second, bool enabled) {
        UnityEngine.Physics.IgnoreLayerCollision(first, second, !enabled);
    }

    private static Transform CreateRotor(Transform parent, string name, float x, float z) {
        var rotor = new GameObject(name);
        rotor.transform.SetParent(parent, false);
        rotor.transform.localPosition = new Vector3(x, 0f, z);
        var visual = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        visual.name = "Rotor Visual";
        UnityEngine.Object.DestroyImmediate(visual.GetComponent<Collider>());
        visual.transform.SetParent(rotor.transform, false);
        visual.transform.localScale = new Vector3(0.18f, 0.015f, 0.18f);
        return rotor.transform;
    }

    private static WheelCollider CreateWheel(Transform parent, string name, float x, float z) {
        var wheelObject = new GameObject(name);
        wheelObject.layer = parent.gameObject.layer;
        wheelObject.transform.SetParent(parent, false);
        wheelObject.transform.localPosition = new Vector3(x, -0.3f, z);
        var wheel = wheelObject.AddComponent<WheelCollider>();
        wheel.mass = 3f;
        wheel.radius = 0.25f;
        wheel.wheelDampingRate = 0.25f;
        wheel.suspensionDistance = 0.2f;
        JointSpring spring = wheel.suspensionSpring;
        spring.spring = 18000f;
        spring.damper = 2200f;
        spring.targetPosition = 0.5f;
        wheel.suspensionSpring = spring;
        WheelFrictionCurve forward = wheel.forwardFriction;
        forward.extremumSlip = 0.35f;
        forward.extremumValue = 1f;
        forward.asymptoteSlip = 0.8f;
        forward.asymptoteValue = 0.65f;
        forward.stiffness = 1.35f;
        wheel.forwardFriction = forward;
        WheelFrictionCurve sideways = wheel.sidewaysFriction;
        sideways.extremumSlip = 0.2f;
        sideways.extremumValue = 1f;
        sideways.asymptoteSlip = 0.5f;
        sideways.asymptoteValue = 0.75f;
        sideways.stiffness = 1.6f;
        wheel.sidewaysFriction = sideways;
        wheel.ConfigureVehicleSubsteps(5f, 5, 10);

        var visual = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        visual.name = "Wheel Visual";
        UnityEngine.Object.DestroyImmediate(visual.GetComponent<Collider>());
        visual.transform.SetParent(wheelObject.transform, false);
        visual.transform.localRotation = Quaternion.Euler(0f, 0f, 90f);
        visual.transform.localScale = new Vector3(0.5f, 0.12f, 0.5f);
        return wheel;
    }

    private static string ReadArgument(string key, string fallback) {
        string[] args = Environment.GetCommandLineArgs();
        int index = Array.IndexOf(args, key);
        return index >= 0 && index + 1 < args.Length ? args[index + 1] : fallback;
    }
}
#endif
