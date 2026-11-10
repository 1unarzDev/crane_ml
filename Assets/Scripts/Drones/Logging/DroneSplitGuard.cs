using System;
using System.IO;
using UnityEngine;

namespace Crane.Drones.Logging {
    [Serializable] public sealed class DroneSeedRange { public string split; public int start; public int end; }
    [Serializable] public sealed class DroneDisturbanceSet { public string family; public string split; public string[] values; }
    [Serializable] public sealed class DroneSplits {
        public string schema;
        public DroneSeedRange[] seed_ranges;
        public DroneDisturbanceSet[] disturbance;
    }

    public sealed class DroneHeldoutException : Exception {
        public DroneHeldoutException(string message) : base(message) { }
    }

    /// <summary>
    /// Refuses held-out (and unassigned) forest seeds and held-out disturbance values unless
    /// <c>--allow-heldout</c> is given. Call <see cref="EnforceFromCommandLine"/> from every capture or
    /// logging entry point BEFORE building the scene. Fails closed: if splits.json cannot be found the
    /// run is refused. Python twin: Tools/DroneData/splits.py (same file, same rules).
    /// </summary>
    public static class DroneSplitGuard {
        public const string Heldout = "heldout";
        public const string AllowFlag = "--allow-heldout";

        public static DroneSplits Parse(string json) {
            var s = JsonUtility.FromJson<DroneSplits>(json);
            if (s == null || s.seed_ranges == null || s.schema != "crane-drone-splits/1")
                throw new InvalidDataException("not a crane-drone-splits/1 file");
            return s;
        }

        public static string FindSplitsFile(string[] args) {
            string arg = ReadArg(args, "--drone-splits");
            if (!string.IsNullOrEmpty(arg)) return arg;
            string dir = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
            for (int i = 0; i < 4 && dir != null; i++, dir = Path.GetDirectoryName(dir)) {
                string p = Path.Combine(dir, "Tools", "DroneData", "splits.json");
                if (File.Exists(p)) return p;
            }
            return null;
        }

        public static string SplitOfSeed(DroneSplits s, int seed) {
            string found = null;
            foreach (var r in s.seed_ranges) {
                if (seed < r.start || seed > r.end) continue;
                if (found != null && found != r.split) throw new InvalidDataException($"seed {seed} in overlapping splits");
                found = r.split;
            }
            return found ?? "unassigned";
        }

        public static string SplitOfDisturbance(DroneSplits s, string family, string value) {
            string found = "unassigned";
            if (s.disturbance != null)
                foreach (var d in s.disturbance)
                    if (d.family == family && Array.IndexOf(d.values, value) >= 0) {
                        if (d.split == Heldout) return Heldout; // held-out wins over any other listing
                        if (found == "unassigned") found = d.split;
                    }
            return found;
        }

        /// <summary>Returns the seed's split; throws <see cref="DroneHeldoutException"/> if refused.</summary>
        public static string Enforce(DroneSplits splits, int seed, string wind, string lighting, bool allowHeldout) {
            string split = SplitOfSeed(splits, seed);
            string w = wind == null ? null : SplitOfDisturbance(splits, "wind", wind);
            string l = lighting == null ? null : SplitOfDisturbance(splits, "lighting", lighting);
            string bad = null;
            if (split == Heldout) bad = $"seed {seed} is HELD-OUT";
            else if (split == "unassigned") bad = $"seed {seed} is in no split";
            else if (w == Heldout) bad = $"wind '{wind}' is HELD-OUT";
            else if (l == Heldout) bad = $"lighting '{lighting}' is HELD-OUT";
            if (bad != null) {
                if (!allowHeldout)
                    throw new DroneHeldoutException(bad + $"; refusing (pass {AllowFlag} for the final evaluation only)");
                Debug.LogWarning($"CRANE_DRONE_HELDOUT_ALLOWED {bad}");
            }
            return split;
        }

        public static string EnforceFromCommandLine(string[] args) {
            string path = FindSplitsFile(args);
            if (path == null) throw new DroneHeldoutException("splits.json not found; refusing (fail closed)");
            var splits = Parse(File.ReadAllText(path));
            if (!int.TryParse(ReadArg(args, "--drone-seed"), out int seed))
                throw new DroneHeldoutException("--drone-seed missing; refusing (seed must be declared)");
            return Enforce(splits, seed, ReadArg(args, "--drone-wind"), ReadArg(args, "--drone-lighting"),
                Array.IndexOf(args, AllowFlag) >= 0);
        }

        public static string ReadArg(string[] args, string name) {
            for (int i = 0; i < args.Length - 1; i++) if (args[i] == name) return args[i + 1];
            return null;
        }
    }
}
