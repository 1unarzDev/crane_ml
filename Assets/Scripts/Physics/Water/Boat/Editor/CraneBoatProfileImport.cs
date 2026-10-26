using System;
using System.IO;
using UnityEditor;
using UnityEngine;

namespace Sim.Physics.Water.Boat.Editor {
    /// <summary>
    /// Turns a profile JSON from Tools/Characterization into a MarineSurfaceProfile asset.
    /// Batch: <c>-executeMethod Sim.Physics.Water.Boat.Editor.CraneBoatProfileImport.ImportFromCommandLine
    /// --crane-profile-json Tools/Characterization/profiles/ideal.json
    /// --crane-profile-asset Assets/Config/Boats/Resources/Ideal.asset</c>.
    /// </summary>
    public static class CraneBoatProfileImport {
        public const string DefaultJson = "Tools/Characterization/profiles/ideal.json";
        public const string DefaultAsset = "Assets/Config/Boats/Resources/Ideal.asset";

        public static MarineSurfaceProfile Import(string jsonPath, string assetPath) {
            string json = File.ReadAllText(jsonPath);
            var profile = AssetDatabase.LoadAssetAtPath<MarineSurfaceProfile>(assetPath);
            bool created = profile == null;
            if (created) profile = ScriptableObject.CreateInstance<MarineSurfaceProfile>();
            profile.FromJson(json);
            var validation = profile.Validate(profile.Parameters.fixedDeltaTime);
            if (!validation.Ok) throw new InvalidDataException($"{jsonPath}: {validation}");
            if (created) {
                Directory.CreateDirectory(Path.GetDirectoryName(assetPath) ?? ".");
                AssetDatabase.CreateAsset(profile, assetPath);
            }
            EditorUtility.SetDirty(profile);
            AssetDatabase.SaveAssets();
            Debug.Log($"CRANE_PROFILE_IMPORTED asset={assetPath} json={jsonPath} profile={profile.ProfileName} " +
                      $"status={profile.Provenance.summary} warnings={validation.Warnings.Count}");
            return profile;
        }

        public static void ImportFromCommandLine() {
            string[] args = Environment.GetCommandLineArgs();
            Import(Read(args, "--crane-profile-json") ?? DefaultJson, Read(args, "--crane-profile-asset") ?? DefaultAsset);
        }

        private static string Read(string[] args, string key) {
            int i = Array.IndexOf(args, key);
            return i >= 0 && i + 1 < args.Length ? args[i + 1] : null;
        }
    }
}
