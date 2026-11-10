using System;

namespace Sim.Physics.Aerial {
    /// <summary>
    /// Command-line gates for the aerial SITL bridge and rotor model. Accepts both
    /// "--key value" (the style of CraneRuntimeOptions) and "--key=value".
    /// </summary>
    public static class CraneAerialOptions {
        public const string BridgeKey = "--crane-sitl-bridge";
        public const string BridgeArduPilotJsonV2 = "ardupilot-json-v2";
        public const string AllowLegacyBridgeFlag = "--crane-allow-legacy-sitl-bridge";
        public const string AerialProfileKey = "--crane-aerial-profile";

        public static string ReadValue(string[] args, string key) {
            if (args == null) return null;
            string prefix = key + "=";
            for (int i = 0; i < args.Length; i++) {
                if (args[i] == key) return i + 1 < args.Length ? args[i + 1] : null;
                if (args[i].StartsWith(prefix, StringComparison.Ordinal))
                    return args[i].Substring(prefix.Length);
            }
            return null;
        }

        public static bool HasFlag(string[] args, string flag) =>
            args != null && Array.IndexOf(args, flag) >= 0;

        public enum BridgeMode { LegacyRefused, LegacyAllowed, LegacyNonAerial, ArduPilotJsonV2 }

        /// <summary>Decides which bridge path may run. Throws on an unknown bridge name.</summary>
        public static BridgeMode ResolveBridge(string[] args, bool aerial = true) {
            string bridge = ReadValue(args, BridgeKey);
            if (bridge == null) {
                // D16: the refusal protects only the multirotor path; boats keep legacy behaviour.
                if (!aerial) return BridgeMode.LegacyNonAerial;
                return HasFlag(args, AllowLegacyBridgeFlag) ?
                    BridgeMode.LegacyAllowed : BridgeMode.LegacyRefused;
            }
            if (string.Equals(bridge, BridgeArduPilotJsonV2, StringComparison.OrdinalIgnoreCase))
                return BridgeMode.ArduPilotJsonV2;
            throw new ArgumentException(
                $"Unknown {BridgeKey} value '{bridge}'; supported: {BridgeArduPilotJsonV2}.");
        }

        public static string LegacyRefusalMessage =>
            "The legacy ArduPilot-JSON SITL bridge is NOT_SUPPORTED (sign/frame errors and servo " +
            "order/spin mismatch, decision D14). Use " + BridgeKey + "=" + BridgeArduPilotJsonV2 +
            ", or pass " + AllowLegacyBridgeFlag + " to run the unvalidated legacy path knowingly.";

        public static string LegacyNonAerialWarning =>
            "Legacy ArduPilot-JSON SITL bridge on a non-aerial vehicle: telemetry has known frame " +
            "defects (A1: NED down sign, reversed roll/pitch, world-frame gyro, constant accel_body, " +
            "float32 timestamp). Behaviour unchanged; use " + BridgeKey + "=" + BridgeArduPilotJsonV2 + ".";

        public static string ReadAerialProfile(string[] args) => ReadValue(args, AerialProfileKey);
    }
}
