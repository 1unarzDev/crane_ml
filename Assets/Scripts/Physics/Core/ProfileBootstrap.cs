using System;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using UnityEngine;

namespace Sim.Physics.Core {
    /// <summary>
    /// Profile selection shared by every vehicle bootstrap: <c>--crane-boat-profile
    /// ideal|characterized|&lt;asset name&gt;</c> (default ideal) picks an asset from a Resources
    /// folder, and <c>--crane-boat-profile-json &lt;path&gt;</c> overrides its numbers with the same
    /// JSON schema. Parsed here, not in CraneRuntimeOptions.
    /// </summary>
    public static class ProfileBootstrap {
        public const string ProfileFlag = "--crane-boat-profile";
        public const string JsonFlag = "--crane-boat-profile-json";
        public const string DefaultProfile = "Ideal";

        public static bool Requested(string[] args) =>
            Array.IndexOf(args, ProfileFlag) >= 0 || Array.IndexOf(args, JsonFlag) >= 0;

        /// <summary>Asset name for a <see cref="ProfileFlag"/> value.</summary>
        public static string AssetName(string value) => value?.Trim().ToLowerInvariant() switch {
            null or "" or "ideal" => DefaultProfile,
            "characterized" => "Characterized",
            _ => value.Trim()
        };

        /// <summary>Loads the selected profile as a fresh instance (the asset itself is never
        /// modified) with the JSON override applied.</summary>
        public static T Load<T>(string[] args) where T : VehicleProfile {
            string name = AssetName(Read(args, ProfileFlag));
            T asset = Resources.Load<T>(name);
            if (asset == null)
                throw new FileNotFoundException($"no {typeof(T).Name} named '{name}' in a Resources folder");
            T profile = UnityEngine.Object.Instantiate(asset);
            profile.name = asset.name;
            string json = Read(args, JsonFlag);
            if (!string.IsNullOrEmpty(json)) profile.FromJson(File.ReadAllText(json));
            return profile;
        }

        public static string Sha256(string text) {
            using var hash = SHA256.Create();
            return BitConverter.ToString(hash.ComputeHash(Encoding.UTF8.GetBytes(text)))
                .Replace("-", "").ToLowerInvariant();
        }

        /// <summary>The one log line every applied profile writes.</summary>
        public static string AppliedLine(VehicleProfile profile, string extra = "") =>
            $"CRANE_PLANT_APPLIED profile={profile.ProfileName} sha256={Sha256(profile.ToJson())} " +
            $"status={profile.Provenance?.summary ?? "unknown"}{(extra.Length > 0 ? " " + extra : "")}";

        public static string Read(string[] args, string key) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length && !args[index + 1].StartsWith("--")
                ? args[index + 1] : null;
        }
    }
}
