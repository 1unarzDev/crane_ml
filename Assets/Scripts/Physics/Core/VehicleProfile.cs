using System.Collections.Generic;
using UnityEngine;

namespace Sim.Physics.Core {
    /// <summary>Errors reject a profile; warnings are logged and the profile still applies.</summary>
    public sealed class ProfileValidation {
        public readonly List<string> Errors = new();
        public readonly List<string> Warnings = new();
        public bool Ok => Errors.Count == 0;

        public void Require(bool condition, string error) {
            if (!condition) Errors.Add(error);
        }

        public void Warn(bool condition, string warning) {
            if (condition) Warnings.Add(warning);
        }

        public override string ToString() =>
            $"errors=[{string.Join("; ", Errors)}] warnings=[{string.Join("; ", Warnings)}]";
    }

    /// <summary>
    /// A vehicle's physical description: mass layout, actuator specs (by name), force-model
    /// configs and provenance. Concrete profiles serialize to the JSON schema Tools/Characterization
    /// emits, so a profile can be overridden at runtime with JsonUtility.
    /// </summary>
    public abstract class VehicleProfile : ScriptableObject {
        public abstract string ProfileName { get; }
        public abstract Provenance Provenance { get; }
        public abstract string ToJson();
        public abstract void FromJson(string json);
        /// <summary>Checks the numbers for a fixed step <paramref name="dt"/> [s].</summary>
        public abstract ProfileValidation Validate(float dt);
    }
}
