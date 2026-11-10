using System;
using System.Globalization;
using System.IO;
using UnityEngine;

namespace Crane.Drones.Logging {
    /// <summary>
    /// Batch entry: <c>-executeMethod Crane.Drones.Logging.DroneReproCli.Run --drone-seed N --drone-out DIR
    /// [--drone-duration S] [--drone-repeat K] [--allow-heldout]</c>. Writes DIR/run_0..run_{K-1}.
    /// Held-out seeds are refused unless --allow-heldout.
    /// </summary>
    public static class DroneReproCli {
        public static void Run() {
            string[] args = Environment.GetCommandLineArgs();
            try {
                DroneSplitGuard.EnforceFromCommandLine(args);
                var splits = DroneSplitGuard.Parse(File.ReadAllText(DroneSplitGuard.FindSplitsFile(args)));
                int seed = int.Parse(DroneSplitGuard.ReadArg(args, "--drone-seed"), CultureInfo.InvariantCulture);
                string outDir = DroneSplitGuard.ReadArg(args, "--drone-out") ?? throw new ArgumentException("--drone-out required");
                double dur = double.TryParse(DroneSplitGuard.ReadArg(args, "--drone-duration"), NumberStyles.Float, CultureInfo.InvariantCulture, out double d) ? d : 6.0;
                int repeat = int.TryParse(DroneSplitGuard.ReadArg(args, "--drone-repeat"), out int r) ? r : 1;
                bool allow = Array.IndexOf(args, DroneSplitGuard.AllowFlag) >= 0;
                for (int i = 0; i < repeat; i++)
                    DroneReferenceFlight.Run(Path.Combine(outDir, "run_" + i), seed, dur, splits, allow);
                Debug.Log($"CRANE_DRONE_REPRO_DONE out={outDir} repeat={repeat}");
            }
            catch (Exception e) {
                Debug.LogError("CRANE_DRONE_REPRO_FAILED " + e);
                if (Application.isBatchMode) UnityEditorExit(1);
                throw;
            }
        }

        private static void UnityEditorExit(int code) => Environment.Exit(code);
    }
}
