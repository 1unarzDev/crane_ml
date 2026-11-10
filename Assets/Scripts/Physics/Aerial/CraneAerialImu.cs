using System;
using System.Collections.Generic;

namespace Sim.Physics.Aerial {
    /// <summary>One IMU sample. Body FLU, SI units, plant time [s].</summary>
    public struct CraneImuSample {
        public double Time;
        public double[] GyroFlu;            // body angular velocity [rad/s]
        public double[] SpecificForceFlu;   // accelerometer reading (a - g) in body [m/s^2]; +9.81 z at rest
        public double[] OrientationEnuFlu;  // truth attitude (w, x, y, z) at the sample
        public double[] GyroFrd => CraneAerialFrames.FluToFrd(GyroFlu);
        public double[] SpecificForceFrd => CraneAerialFrames.FluToFrd(SpecificForceFlu);
    }

    /// <summary>
    /// IMU stub sampled from the sub-stepped plant at RateHz (default 500 Hz; must be &gt;= 250 Hz and
    /// divide the sub-step rate). Fixes the v1 defects: gyro is the body-frame rate (not the world
    /// rate converted as a polar vector), accelerometer is the measured specific force
    /// R^T (dv/dt - g) integrated over the sample interval (not a constant (0,0,-g)), timestamps
    /// are double-precision plant time. Optional white noise and bias random walk are seeded and
    /// reset with the episode. Samples go to a bounded queue (oldest dropped, counted).
    /// </summary>
    public sealed class CraneAerialImu {
        public readonly double RateHz;
        public double GyroNoiseDensity, AccelNoiseDensity;   // [rad/s/sqrt(Hz)], [m/s^2/sqrt(Hz)]
        public double GyroBiasWalk, AccelBiasWalk;           // [rad/s^2/sqrt(Hz)], [m/s^3/sqrt(Hz)]
        public int Capacity = 512;
        public long Dropped { get; private set; }
        public CraneImuSample Latest { get; private set; }
        private readonly Queue<CraneImuSample> queue = new Queue<CraneImuSample>();
        private readonly CraneAerialRng rng;
        private readonly double[] gyroBias = new double[3], accelBias = new double[3];
        private readonly double[] sumF = new double[3], sumW = new double[3];
        private double sumT;
        private readonly double gravity;

        public CraneAerialImu(double rateHz, double gravity, ulong seed) {
            if (!(rateHz >= 250.0)) throw new ArgumentOutOfRangeException(nameof(rateHz), "IMU rate must be >= 250 Hz");
            RateHz = rateHz; this.gravity = gravity; rng = new CraneAerialRng(seed);
        }

        public void Reset(ulong seed) {
            rng.Reset(seed); queue.Clear(); Dropped = 0; sumT = 0;
            Array.Clear(gyroBias, 0, 3); Array.Clear(accelBias, 0, 3); Array.Clear(sumF, 0, 3); Array.Clear(sumW, 0, 3);
            Latest = default;
        }

        /// <summary>Called by the plant after every sub-step (prev -> next over h).</summary>
        public void Accumulate(StateV2 prev, StateV2 next, double h, bool sampleNow) {
            double[,] r = CraneRotorModelV2.QuatToMatrix(next.Q);
            double[] aMinusG = { (next.V[0] - prev.V[0]) / h, (next.V[1] - prev.V[1]) / h, (next.V[2] - prev.V[2]) / h + gravity };
            double[] fb = CraneRotorModelV2.MatTVec(r, aMinusG);
            for (int i = 0; i < 3; i++) { sumF[i] += fb[i] * h; sumW[i] += next.Omega[i] * h; }
            sumT += h;
            if (!sampleNow) return;
            double dt = sumT;
            var g = new double[3]; var f = new double[3];
            for (int i = 0; i < 3; i++) {
                gyroBias[i] += GyroBiasWalk * Math.Sqrt(dt) * rng.NextGaussian();
                accelBias[i] += AccelBiasWalk * Math.Sqrt(dt) * rng.NextGaussian();
                g[i] = sumW[i] / dt + gyroBias[i] + GyroNoiseDensity * Math.Sqrt(RateHz) * rng.NextGaussian();
                f[i] = sumF[i] / dt + accelBias[i] + AccelNoiseDensity * Math.Sqrt(RateHz) * rng.NextGaussian();
            }
            Array.Clear(sumF, 0, 3); Array.Clear(sumW, 0, 3); sumT = 0;
            var s = new CraneImuSample { Time = next.T, GyroFlu = g, SpecificForceFlu = f, OrientationEnuFlu = (double[])next.Q.Clone() };
            Latest = s;
            if (queue.Count >= Capacity) { queue.Dequeue(); Dropped++; }
            queue.Enqueue(s);
        }

        public int Count => queue.Count;
        public bool TryDequeue(out CraneImuSample s) {
            if (queue.Count == 0) { s = default; return false; }
            s = queue.Dequeue(); return true;
        }
    }
}
