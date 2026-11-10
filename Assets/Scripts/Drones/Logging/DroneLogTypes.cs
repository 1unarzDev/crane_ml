using System;
using System.Collections.Generic;

namespace Crane.Drones.Logging {
    public static class DroneLogFormat {
        public const string Schema = "crane-drone-log/1";
        public const string StateFile = "state.csv";
        public const string ImuFile = "imu.csv";
        public const string EventsFile = "events.jsonl";
        public const string ObstaclesFile = "obstacles.json";
    }

    /// <summary>
    /// The one sim clock. A tick is one Unity fixed step (the CraneSimulationClock convention). The
    /// physics plant may sub-step each tick; time is exact integer nanoseconds:
    /// t_ns = tick * FixedDtNs + sub * FixedDtNs / Substeps (FixedDtNs divisible by Substeps).
    /// Every stream carries this t_ns; no stream uses wall-clock time.
    /// </summary>
    public readonly struct DroneSimClock {
        public readonly long FixedDtNs;
        public readonly int Substeps;
        public DroneSimClock(long fixedDtNs, int substeps) {
            if (fixedDtNs <= 0 || substeps <= 0 || fixedDtNs % substeps != 0)
                throw new ArgumentException("FixedDtNs must be positive and divisible by Substeps");
            FixedDtNs = fixedDtNs; Substeps = substeps;
        }
        public long SubstepNs => FixedDtNs / Substeps;
        public long TimeNs(long tick, int sub = 0) => tick * FixedDtNs + sub * SubstepNs;
    }

    /// <summary>Ground-truth state, world ENU (x east, y north, z up), body FLU, quaternion w,x,y,z world-from-body.</summary>
    public sealed class DroneStateSample {
        public double[] Pos = new double[3];
        public double[] Quat = { 1, 0, 0, 0 };
        public double[] Vel = new double[3];          // world, m/s
        public double[] AngRateBody = new double[3];  // rad/s
        public double[] AccBody = { double.NaN, double.NaN, double.NaN }; // coordinate accel in body, m/s^2 (NaN: recorder differentiates Vel)
        public double[] RotorRpm;
        public double[] MotorCmd;                     // normalised 0..1
        public double BattVolt = double.NaN, BattAmp = double.NaN, BattSoc = double.NaN;
        public DroneStateSample(int rotors) { RotorRpm = new double[rotors]; MotorCmd = new double[rotors]; }
    }

    /// <summary>Plant adapter. The physics module implements this; the stub plant implements it for tests.</summary>
    public interface IDroneStateSource {
        int RotorCount { get; }
        void Sample(DroneStateSample into);
    }

    /// <summary>IMU sample. Flags bit0 white noise, bit1 bias random walk, bit2 saturation, bit3 quantisation; 0 = ideal.</summary>
    public struct DroneImuSample {
        public double Gx, Gy, Gz;   // rad/s, body FLU
        public double Ax, Ay, Az;   // m/s^2 specific force, body FLU
        public int NoiseFlags;
    }

    public interface IDroneImuSource { DroneImuSample Sample(); }

    public enum DroneCameraKind { Rgb, StereoLeft, StereoRight, Depth }

    /// <summary>One frame. Rgb8 is width*height*3; Depth16 is width*height millimetres (0 = invalid).</summary>
    public sealed class DroneCameraFrame {
        public string Camera;             // e.g. "oakd_rgb", "oakd_left", "oakd_right", "oakd_depth"
        public DroneCameraKind Kind;
        public int Width, Height;
        public byte[] Rgb8;               // Rgb, StereoLeft/Right (gray replicated to RGB allowed) 
        public ushort[] Depth16;
        public long AcquisitionNs;        // exposure/capture instant on the sim clock (NOT completion)
        public long CompletionNs;         // readback done
    }

    public sealed class DroneLidarScan {
        public string Sensor = "lidar";
        public long AcquisitionNs;        // start of the sweep
        public float[] X, Y, Z, Intensity; // sensor frame, metres
        public int[] TimeOffsetNs;        // per point, relative to AcquisitionNs, >= 0, non-decreasing
        public int Count => X.Length;
    }

    public sealed class DroneRunManifest {
        public string RunId;
        public string CraneGitSha = "unknown";
        public int SceneSeed;
        public string Scene = "";
        public string ForestManifestSha256 = "";
        public string PhysicsProfile = "";
        public string PhysicsParamsSha256 = "";
        public string Controller = "";
        public Dictionary<string, object> TrajectorySpec = new();
        public string UnityVersion = "";
        public string WallClockStartUtc = "";
        public long FixedDtNs = 5_000_000;
        public int Substeps = 5;
        public int StateEverySubsteps = 1;
        public int RotorCount = 4;
        public string Disturbance_Wind = "calm";
        public string Disturbance_Lighting = "GoldenHour";
        public string Split = "dev";
        public double ImuRateHz;
        public Dictionary<string, object> ImuNoise = new();
        public List<Dictionary<string, object>> Cameras = new();   // name, kind, width, height, rate_hz, profile
        public List<Dictionary<string, object>> Lidars = new();    // name, rate_hz, points
        public Dictionary<string, object> Extra = new();
    }
}
