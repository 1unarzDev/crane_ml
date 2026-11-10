using System;

namespace Sim.Physics.Aerial {
    /// <summary>
    /// Linear multirotor mixer in thrust-fraction space (each output in [0, 1]) with
    /// priority-preserving desaturation. Priority: roll/pitch (attitude) first, then collective,
    /// then yaw. Order of resolution: (1) if roll/pitch alone span more than the available
    /// range, scale roll/pitch together (ratio preserved) and drop yaw; (2) scale yaw down to the
    /// largest value that still fits; (3) shift collective into the remaining window. Attitude
    /// torque directions are therefore never altered by saturation; collective gives way.
    /// Axis coefficients come from the rotor geometry and spin: roll ~ y, pitch ~ -x, yaw ~ -spin
    /// (torque about +x, +y, +z of FLU body), each normalized to unit maximum magnitude.
    /// </summary>
    public sealed class CraneRotorMixerV2 {
        public readonly double[] RollCoeff, PitchCoeff, YawCoeff;
        public int Count => RollCoeff.Length;

        public CraneRotorMixerV2(RotorSpecV2[] rotors) {
            int n = rotors.Length;
            RollCoeff = new double[n]; PitchCoeff = new double[n]; YawCoeff = new double[n];
            double my = 0, mx = 0;
            foreach (var r in rotors) {
                my = Math.Max(my, Math.Abs(r.Position[1]));
                mx = Math.Max(mx, Math.Abs(r.Position[0]));
            }
            for (int i = 0; i < n; i++) {
                RollCoeff[i] = my > 0 ? rotors[i].Position[1] / my : 0.0;
                PitchCoeff[i] = mx > 0 ? -rotors[i].Position[0] / mx : 0.0;
                YawCoeff[i] = -rotors[i].Spin;
            }
        }

        public struct Result {
            public double[] Output;
            public double Collective;       // applied collective after shifting
            public double YawScale;         // fraction of requested yaw delivered
            public double AttitudeScale;    // fraction of requested roll/pitch delivered
            public bool Saturated;
        }

        public Result Mix(double collective, double roll, double pitch, double yaw) {
            int n = Count;
            double[] rp = new double[n];
            double lo = double.MaxValue, hi = double.MinValue;
            for (int i = 0; i < n; i++) {
                rp[i] = roll * RollCoeff[i] + pitch * PitchCoeff[i];
                lo = Math.Min(lo, rp[i]); hi = Math.Max(hi, rp[i]);
            }
            double span = hi - lo, attScale = 1.0;
            if (span > 1.0) {
                attScale = 1.0 / span;
                for (int i = 0; i < n; i++) rp[i] *= attScale;
                yaw = 0.0;
            }
            double yawScale = 1.0;
            if (yaw != 0.0 && Span(rp, yaw, 1.0) > 1.0) {
                double a = 0.0, b = 1.0;
                for (int it = 0; it < 80; it++) {
                    double m = 0.5 * (a + b);
                    if (Span(rp, yaw, m) <= 1.0) a = m; else b = m;
                }
                yawScale = a;
            }
            double[] t = new double[n];
            lo = double.MaxValue; hi = double.MinValue;
            for (int i = 0; i < n; i++) {
                t[i] = rp[i] + yaw * yawScale * YawCoeff[i];
                lo = Math.Min(lo, t[i]); hi = Math.Max(hi, t[i]);
            }
            double cMin = -lo, cMax = 1.0 - hi;
            double c = Math.Max(cMin, Math.Min(cMax, collective));
            double[] u = new double[n];
            for (int i = 0; i < n; i++) u[i] = Math.Max(0.0, Math.Min(1.0, c + t[i]));
            return new Result {
                Output = u, Collective = c, YawScale = yaw == 0.0 ? 1.0 : yawScale,
                AttitudeScale = attScale,
                Saturated = attScale < 1.0 || (yaw != 0.0 && yawScale < 1.0) || c != collective
            };
        }

        private double Span(double[] rp, double yaw, double s) {
            double lo = double.MaxValue, hi = double.MinValue;
            for (int i = 0; i < rp.Length; i++) {
                double x = rp[i] + yaw * s * YawCoeff[i];
                lo = Math.Min(lo, x); hi = Math.Max(hi, x);
            }
            return hi - lo;
        }
    }
}
