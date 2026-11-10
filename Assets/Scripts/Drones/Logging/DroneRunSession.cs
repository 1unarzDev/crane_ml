using System;
using System.Collections.Generic;

namespace Crane.Drones.Logging {
    /// <summary>
    /// Drives one logged flight: call <see cref="OnSubstep"/> after every physics substep (sub-step
    /// index 0..Substeps-1 within a Unity fixed tick) and push sensor frames as they are acquired.
    /// Entry points call the split guard first, so held-out seeds cannot be logged by accident.
    /// </summary>
    public sealed class DroneRunSession : IDisposable {
        public readonly DroneRunWriter Writer;
        private readonly IDroneStateSource plant;
        private readonly IDroneImuSource imuSource;
        private readonly int imuEvery;
        private readonly DroneStateSample sample;
        private readonly double[] prevVel = new double[3];
        private long prevT = -1;
        private long lastTick;
        public readonly DroneClearanceTracker Clearance;

        public static DroneRunSession Begin(string dir, DroneRunManifest m, DroneSplits splits, bool allowHeldout,
                IDroneStateSource plant, IDroneImuSource imu = null) {
            m.Split = DroneSplitGuard.Enforce(splits, m.SceneSeed, m.Disturbance_Wind, m.Disturbance_Lighting, allowHeldout);
            return new DroneRunSession(dir, m, plant, imu);
        }

        /// <summary>Capture/logging entry point: reads --drone-seed/--drone-wind/--drone-lighting/--allow-heldout.</summary>
        public static DroneRunSession BeginFromCommandLine(string[] args, string dir, DroneRunManifest m,
                IDroneStateSource plant, IDroneImuSource imu = null) {
            DroneSplitGuard.EnforceFromCommandLine(args);
            var splits = DroneSplitGuard.Parse(System.IO.File.ReadAllText(DroneSplitGuard.FindSplitsFile(args)));
            return Begin(dir, m, splits, Array.IndexOf(args, DroneSplitGuard.AllowFlag) >= 0, plant, imu);
        }

        private DroneRunSession(string dir, DroneRunManifest m, IDroneStateSource plant, IDroneImuSource imu) {
            this.plant = plant; imuSource = imu;
            m.RotorCount = plant.RotorCount;
            if (m.ImuRateHz > 0) {
                double every = m.ImuRateHz > 0 ? 1e9 * m.Substeps / (m.FixedDtNs * m.ImuRateHz) : 0;
                imuEvery = (int)Math.Round(every);
                if (imuEvery < 1 || Math.Abs(every - imuEvery) > 1e-9)
                    throw new ArgumentException("IMU rate must be an integer divisor of the substep rate");
            }
            Writer = new DroneRunWriter(dir, m);
            sample = new DroneStateSample(plant.RotorCount);
            Clearance = new DroneClearanceTracker();
        }

        public long TimeNs(long tick, int sub) => Writer.Clock.TimeNs(tick, sub);

        public void OnSubstep(long tick, int sub) {
            lastTick = tick;
            var m = Writer.Manifest;
            long gs = tick * m.Substeps + sub; // global substep index
            long t = TimeNs(tick, sub);
            if (gs % m.StateEverySubsteps == 0) {
                plant.Sample(sample);
                if (double.IsNaN(sample.AccBody[0])) FillAcceleration(t);
                Writer.WriteState(tick, sub, sample);
            }
            if (imuSource != null && gs % imuEvery == 0) Writer.WriteImu(t, imuSource.Sample());
        }

        private void FillAcceleration(long t) {
            double dt = prevT < 0 ? 0 : (t - prevT) * 1e-9;
            double[] a = new double[3];
            if (dt > 0) for (int i = 0; i < 3; i++) a[i] = (sample.Vel[i] - prevVel[i]) / dt;
            RotateInverse(sample.Quat, a, sample.AccBody);
            Array.Copy(sample.Vel, prevVel, 3);
            prevT = t;
        }

        /// <summary>out = R(q)^T v for q = (w,x,y,z).</summary>
        public static void RotateInverse(double[] q, double[] v, double[] o) {
            double w = q[0], x = -q[1], y = -q[2], z = -q[3]; // conjugate
            double tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
            o[0] = v[0] + w * tx + (y * tz - z * ty);
            o[1] = v[1] + w * ty + (z * tx - x * tz);
            o[2] = v[2] + w * tz + (x * ty - y * tx);
        }

        public void PushCamera(DroneCameraFrame f) => Writer.WriteCameraFrame(f);
        public void PushLidar(DroneLidarScan s) => Writer.WriteLidarScan(s);

        public void Collision(long tNs, string other, double impactSpeed, double[] normalEnu) =>
            Writer.WriteEvent(tNs, "collision", new Dictionary<string, object> {
                ["other"] = other, ["impact_speed_mps"] = impactSpeed, ["normal_enu"] = normalEnu });

        public void Detection(long tNs, Dictionary<string, object> d) => Writer.WriteEvent(tNs, "detection", d);

        public void Close() {
            Clearance.Flush(TimeNs(lastTick, 0), Writer);
            Writer.Close(lastTick);
        }
        public void Dispose() => Close();
    }
}
