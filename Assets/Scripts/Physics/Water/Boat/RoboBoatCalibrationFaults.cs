using System;
using System.IO;
using Sim.Physics.Core;
using UnityEngine;

namespace Sim.Physics.Water.Boat {
    /// <summary>Opt-in physical faults for held-out characterization tests:
    /// <c>--crane-calibration-faults &lt;json&gt;</c> becomes an <see cref="ActuatorErrorProfile"/>
    /// applied after the profile. Default worker physics is unchanged.</summary>
    public static class RoboBoatCalibrationFaults {
        public const string Flag = "--crane-calibration-faults";

        /// <summary>The error layer and its JSON sha256, or (null, "none") without the flag.</summary>
        public static (ActuatorErrorProfile errors, string sha256) Load(string[] args) {
            if (Array.IndexOf(args, Flag) < 0) return (null, "none");
            string path = ProfileBootstrap.Read(args, Flag) ?? throw new ArgumentException("fault JSON path required");
            string text = File.ReadAllText(path);
            ActuatorErrorProfile errors = ActuatorErrorProfile.Parse(text);
            string sha = ProfileBootstrap.Sha256(text);
            Debug.Log($"CRANE_CALIBRATION_FAULTS sha256={sha} {JsonUtility.ToJson(errors)}");
            return (errors, sha);
        }
    }
}
