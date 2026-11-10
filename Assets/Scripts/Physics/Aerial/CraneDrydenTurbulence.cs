using System;

namespace Sim.Physics.Aerial {
    /// <summary>Deterministic SplitMix64 + Box-Muller normal generator (no UnityEngine.Random).</summary>
    public sealed class CraneAerialRng {
        private ulong state;
        private double spare; private bool hasSpare;
        public CraneAerialRng(ulong seed) { Reset(seed); }
        public void Reset(ulong seed) { state = seed ^ 0x9E3779B97F4A7C15UL; hasSpare = false; spare = 0; }
        public ulong NextU64() {
            ulong z = (state += 0x9E3779B97F4A7C15UL);
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
            return z ^ (z >> 31);
        }
        public double NextUniform() => ((NextU64() >> 11) + 0.5) * (1.0 / 9007199254740992.0);
        public double NextGaussian() {
            if (hasSpare) { hasSpare = false; return spare; }
            double u1 = NextUniform(), u2 = NextUniform();
            double r = Math.Sqrt(-2.0 * Math.Log(u1)), a = 2.0 * Math.PI * u2;
            spare = r * Math.Sin(a); hasSpare = true;
            return r * Math.Cos(a);
        }
    }

    /// <summary>
    /// Dryden low-altitude turbulence (MIL-F-8785C / MIL-HDBK-1797), seeded and reset-safe.
    /// Scales (h in ft, below 1000 ft): L_w = h, L_u = L_v = h / (0.177 + 0.000823 h)^1.2,
    /// sigma_w = 0.1 W20, sigma_u = sigma_v = sigma_w / (0.177 + 0.000823 h)^0.4.
    /// Shaping filters (T = L / V): u is first order 1/(1+Ts), discretised exactly (AR(1));
    /// v and w are (1 + sqrt(3) T s)/(1 + T s)^2 integrated at the plant sub-step, gain chosen so
    /// the stationary standard deviation equals sigma (analytic variance of that filter is g^2/T).
    /// Frozen-turbulence speed V = max(|mean wind - vehicle velocity|, MinAdvectionSpeed): a hovering
    /// vehicle sees turbulence advected by the mean wind. Axes: u along the mean-wind heading
    /// (world +x when calm), v horizontal-lateral, w vertical (ENU). Reset(seed) restores the
    /// exact sequence, so episodes replay bit-identically.
    /// </summary>
    public sealed class CraneDrydenTurbulence {
        public double W20;                       // wind speed at 20 ft [m/s]
        public double MinAdvectionSpeed = 1.0;   // [m/s]
        public double MinAltitude = 1.0;         // [m] clamp for the scale lengths
        private readonly CraneAerialRng rng;
        private double xu, v1, v2, w1, w2;
        public ulong Seed { get; private set; }

        public CraneDrydenTurbulence(double w20, ulong seed) { W20 = w20; rng = new CraneAerialRng(seed); Reset(seed); }

        public void Reset(ulong seed) {
            Seed = seed; rng.Reset(seed);
            xu = v1 = v2 = w1 = w2 = 0.0;
        }

        public static void Scales(double altitudeM, double w20, out double lu, out double lw,
            out double sigmaU, out double sigmaW) {
            const double ft = 0.3048;
            double h = Math.Min(1000.0, Math.Max(altitudeM, 1e-3) / ft);
            double d = 0.177 + 0.000823 * h;
            lw = h * ft;
            lu = h / Math.Pow(d, 1.2) * ft;
            sigmaW = 0.1 * w20;
            sigmaU = sigmaW / Math.Pow(d, 0.4);
        }

        /// <summary>Advances the filters by dt and returns the gust velocity in world ENU [m/s].</summary>
        public double[] Step(double dt, double altitude, double[] meanWindWorld, double[] vehicleVelocityWorld) {
            Scales(Math.Max(altitude, MinAltitude), W20, out double lu, out double lw, out double su, out double sw);
            double ax = meanWindWorld[0] - vehicleVelocityWorld[0], ay = meanWindWorld[1] - vehicleVelocityWorld[1],
                az = meanWindWorld[2] - vehicleVelocityWorld[2];
            double speed = Math.Max(MinAdvectionSpeed, Math.Sqrt(ax * ax + ay * ay + az * az));
            double tu = lu / speed, tw = lw / speed;
            // u: exact AR(1) with stationary variance su^2
            double a = Math.Exp(-dt / tu);
            xu = a * xu + su * Math.Sqrt(1.0 - a * a) * rng.NextGaussian();
            double v = SecondOrder(ref v1, ref v2, dt, tu, su);
            double w = SecondOrder(ref w1, ref w2, dt, tw, sw);
            double mx = meanWindWorld[0], my = meanWindWorld[1];
            double mh = Math.Sqrt(mx * mx + my * my);
            double cx = mh > 1e-6 ? mx / mh : 1.0, cy = mh > 1e-6 ? my / mh : 0.0;
            return new[] { xu * cx - v * cy, xu * cy + v * cx, w };
        }

        private double SecondOrder(ref double z1, ref double z2, double dt, double t, double sigma) {
            double g = sigma * Math.Sqrt(t);
            double n = rng.NextGaussian();
            double z1n = z1 + dt / t * (-z1) + g / t * Math.Sqrt(dt) * n;
            double z2n = z2 + dt / t * (z1 - z2);
            z1 = z1n; z2 = z2n;
            return z2 + Math.Sqrt(3.0) * (z1 - z2);
        }
    }
}
