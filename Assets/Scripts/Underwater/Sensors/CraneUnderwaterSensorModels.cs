using System;
using static Sim.Underwater.CraneLinAlg;

namespace Sim.Underwater {
    /// <summary>What a pressure sensor publishes. Contains ONLY measured quantities and provenance (no ground truth).</summary>
    public readonly struct CranePressureObservation {
        public readonly long PhysicsTick;      // fixed-step index since episode start (never wall clock)
        public readonly double TimeSeconds;    // PhysicsTick * fixedDeltaTime
        public readonly long Sequence;         // 1-based count of published samples this episode; 0 = none yet
        public readonly double PressurePa;     // absolute pressure, includes noise and quantisation
        public readonly double DepthMeters;    // depth derived from PressurePa with the sensor's configured density
        public CranePressureObservation(long tick, double time, long seq, double pa, double depth) {
            PhysicsTick = tick; TimeSeconds = time; Sequence = seq; PressurePa = pa; DepthMeters = depth;
        }
    }

    /// <summary>What a DVL publishes. Velocity is in the DVL SENSOR frame (FRD-aligned axes unless SensorToBody is set).</summary>
    public readonly struct CraneDvlObservation {
        public readonly long PhysicsTick;
        public readonly double TimeSeconds;
        public readonly long Sequence;
        public readonly bool BottomLock;               // false: dropped out (out of range or &lt; 3 valid beams); Velocity is zero then
        public readonly bool WaterTrack;
        public readonly double VelX, VelY, VelZ;       // m/s, sensor frame, relative to the bottom (or water layer)
        public readonly double AltitudeMeters;         // from valid slant ranges; NaN without lock
        public readonly int ValidBeamMask;             // bit i = beam i valid
        public CraneDvlObservation(long tick, double time, long seq, bool lockOk, bool water, double[] v, double alt, int mask) {
            PhysicsTick = tick; TimeSeconds = time; Sequence = seq; BottomLock = lockOk; WaterTrack = water;
            VelX = lockOk ? v[0] : 0; VelY = lockOk ? v[1] : 0; VelZ = lockOk ? v[2] : 0; AltitudeMeters = alt; ValidBeamMask = mask;
        }
    }

    /// <summary>Port of sensor_models.PressureSensor: p = p_atm + rho g depth at a body-fixed port (NED/FRD).</summary>
    public sealed class CranePressureSensor {
        public const double PAtm = 101325.0;
        public double[] PortBody = new double[3];
        public double Rho = 997.0;
        public double SigmaPa, ResolutionPa, SurfaceZ;
        public double Gravity = CraneVehicleParams.StandardGravity;

        public double Measure(double[] posN, double[] q, CraneRng rng = null) {
            double zPort = posN[2] + Mul(QuatToRot(q), PortBody)[2];
            double depth = Math.Max(zPort - SurfaceZ, 0.0);
            double p = PAtm + Rho * Gravity * depth;
            if (rng != null && SigmaPa > 0) p += SigmaPa * rng.NextGaussian();
            if (ResolutionPa > 0) p = Math.Round(p / ResolutionPa, MidpointRounding.ToEven) * ResolutionPa;
            return p;
        }

        public double DepthFromPressure(double p) => (p - PAtm) / (Rho * Gravity);
    }

    public sealed class CraneDvlMeasurement {
        public double[] VelocitySensor;    // null without lock
        public double[] Radial;            // per-beam radial velocity (noisy)
        public bool[] Valid;
        public double[] Ranges;            // true slant ranges (core-internal, NOT part of any published observation)
        public bool Lock => VelocitySensor != null;
    }

    /// <summary>
    /// Port of sensor_models.DVL (Janus, bottom/water track). Beams are defined in the SENSOR frame; SensorToBody (R_bs) maps
    /// sensor axes to body FRD axes (identity = FRD-aligned). Output velocity is expressed in the sensor frame.
    /// </summary>
    public sealed class CraneDvl {
        public double[,] BeamsSensor = JanusBeams();
        public double[,] SensorToBody = Identity(3);
        public double[] MountBody = new double[3];
        public double SigmaBeam, MinRange = 0.05, MaxRange = 50.0;

        public static double[,] JanusBeams(double tiltDeg = 22.5, double yawOffsetDeg = 45.0) {
            double t = tiltDeg * Math.PI / 180.0;
            var b = new double[4, 3];
            for (int k = 0; k < 4; k++) {
                double a = (yawOffsetDeg + 90.0 * k) * Math.PI / 180.0;
                b[k, 0] = Math.Sin(t) * Math.Cos(a); b[k, 1] = Math.Sin(t) * Math.Sin(a); b[k, 2] = Math.Cos(t);
            }
            return b;
        }

        int N => BeamsSensor.GetLength(0);
        double[] Beam(int i) => new[] { BeamsSensor[i, 0], BeamsSensor[i, 1], BeamsSensor[i, 2] };

        public double[] BeamRanges(double[] posN, double[] q, double seabedZ) {
            var r = QuatToRot(q);
            var p = Add(posN, Mul(r, MountBody));
            double dz = seabedZ - p[2];
            var rt = Transpose(r);
            var ranges = new double[N];
            for (int i = 0; i < N; i++) {
                var beamNed = Mul(r, Mul(SensorToBody, Beam(i)));
                double down = beamNed[2];
                double range = down > 1e-6 ? dz / down : double.PositiveInfinity;
                ranges[i] = range > 0 ? range : double.PositiveInfinity;
            }
            return ranges;
        }

        public CraneDvlMeasurement Measure(double[] posN, double[] q, double[] nu, double seabedZ, double[] currentN = null, bool waterTrack = false, CraneRng rng = null) {
            var r = QuatToRot(q);
            var vMount = Add(Slice(nu, 0, 3), Cross(Slice(nu, 3, 3), MountBody));           // body frame
            if (waterTrack && currentN != null) vMount = Sub(vMount, Mul(Transpose(r), currentN));
            var vSensor = Mul(Transpose(SensorToBody), vMount);                               // body -> sensor axes
            var ranges = BeamRanges(posN, q, seabedZ);
            var valid = new bool[N];
            int nv = 0;
            for (int i = 0; i < N; i++) { valid[i] = waterTrack || (ranges[i] >= MinRange && ranges[i] <= MaxRange); if (valid[i]) nv++; }
            var radial = new double[N];
            for (int i = 0; i < N; i++) {
                radial[i] = Dot(Beam(i), vSensor);
                if (rng != null && SigmaBeam > 0) radial[i] += SigmaBeam * rng.NextGaussian();
            }
            var m = new CraneDvlMeasurement { Radial = radial, Valid = valid, Ranges = ranges };
            if (nv < 3) return m;
            var a = new double[nv, 3]; var y = new double[nv]; int k = 0;
            for (int i = 0; i < N; i++) if (valid[i]) { for (int j = 0; j < 3; j++) a[k, j] = BeamsSensor[i, j]; y[k++] = radial[i]; }
            var at = Transpose(a);
            m.VelocitySensor = Mul(Mul(Inverse(Mul(at, a)), at), y);
            return m;
        }

        /// <summary>Altitude of the mount above a flat bottom from the valid slant ranges: mean(range_i * beam_i_down).</summary>
        public double Altitude(CraneDvlMeasurement m, double[] qBodyToNed) {
            var rn = Mul(QuatToRot(qBodyToNed), SensorToBody);
            double sum = 0; int n = 0;
            for (int i = 0; i < N; i++) if (m.Valid[i] && !double.IsInfinity(m.Ranges[i])) {
                sum += m.Ranges[i] * Mul(rn, Beam(i))[2]; n++;
            }
            return n == 0 ? double.NaN : sum / n;
        }
    }
}
