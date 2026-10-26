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
using Sim.Physics.Contacts;

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
        public string sourceCommit;
        public bool sourceDirty;
        public string packageManifestHash;
        public string packageLockHash;
        public string projectVersionHash;
        public string projectSettingsHash;
        public string assetSetHash;
        public string[] scenes;
        public List<BuildAssetHash> assets = new();
    }

    private static readonly string[] Scenes = {
        "Assets/Scenes/Roboboat Course.unity"
    };

    public static void BuildLinuxWorker() => BuildLinuxWorkerWithInitialScene(null);

    /// <summary>Choose a product-specific startup scene without editing shared scene settings.</summary>
    public static void BuildLinuxWorkerWithInitialScene(string initialScene) {
        string output = ReadArgument("--crane-build-output", "Builds/CRANE-Worker/CRANE.x86_64");
        var buildScenes = new List<string>(Scenes);
        if (!string.IsNullOrEmpty(initialScene)) {
            if (!buildScenes.Remove(initialScene))
                throw new ArgumentException("Initial scene is not a worker scene: " + initialScene);
            buildScenes.Insert(0, initialScene);
        }
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
        BuildReport report;
        string versionFile = WriteCourseGeneratorVersion();
        try { report = BuildPipeline.BuildPlayer(options); }
        finally {
            AssetDatabase.DeleteAsset("Assets/Generated");
            File.Delete("Assets/Generated.meta");
        }
        if (report.summary.result != BuildResult.Succeeded)
            throw new Exception($"CRANE worker build failed: {report.summary.result}");
        WriteBuildManifest(output, report, buildScenes.ToArray());
        Debug.Log($"CRANE_BUILD_COMPLETE path={output} bytes={report.summary.totalSize}");
    }

    /// <summary>The course generator version the player stamps into every truth (CourseIdentity),
    /// from this checkout; removed again after the build.</summary>
    private static string WriteCourseGeneratorVersion() {
        string commit = ReadArgument("--crane-source-commit", string.Empty);
        var version = Sim.Course.CourseIdentity.FromCheckout(Directory.GetCurrentDirectory(), commit.Length > 0 ? commit : null);
        Directory.CreateDirectory(Sim.Course.CourseGeneratorVersionHost.ResourceDir);
        string path = Sim.Course.CourseGeneratorVersionHost.ResourceDir + "/" + Sim.Course.CourseGeneratorVersionHost.ResourceName + ".json";
        File.WriteAllText(path, Sim.Course.Json.Write(version.ToJson(), true) + "\n");
        AssetDatabase.ImportAsset(path, ImportAssetOptions.ForceSynchronousImport);
        Debug.Log($"CRANE_COURSE_GENERATOR commit={version.CraneCommit} dirty={version.CraneDirty} source_sha256={version.SourceSha256}");
        return path;
    }

    private static void WriteBuildManifest(string output, BuildReport report, string[] buildScenes) {
        var manifest = new BuildManifest {
            unityVersion = Application.unityVersion,
            buildGuid = report.summary.guid.ToString(),
            sourceCommit = ReadArgument("--crane-source-commit", string.Empty),
            sourceDirty = string.Equals(
                ReadArgument("--crane-source-dirty", "false"), "true",
                StringComparison.OrdinalIgnoreCase),
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

    private static string ReadArgument(string key, string fallback) {
        string[] args = Environment.GetCommandLineArgs();
        int index = Array.IndexOf(args, key);
        return index >= 0 && index + 1 < args.Length ? args[index + 1] : fallback;
    }
}
#endif
