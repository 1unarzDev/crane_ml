using System;

namespace Crane.Fleet {
    /// <summary>Planar body twist in the chassis frame: vx forward, vy left (m/s), wz CCW (rad/s).</summary>
    public struct Twist {
        public double Vx, Vy, Wz;
        public Twist(double vx, double vy, double wz) { Vx = vx; Vy = vy; Wz = wz; }
        public override string ToString() => $"({Vx:0.####}, {Vy:0.####}, {Wz:0.####})";
    }

    /// <summary>Deterministic RNG (SplitMix64 + Box-Muller); identical streams in and out of Unity.</summary>
    public sealed class FleetRng {
        ulong state;
        double spare;
        bool hasSpare;

        public FleetRng(ulong seed) { state = seed; }

        /// <summary>An independent stream for a named consumer (rover id, sensor), derived from the run seed.</summary>
        public static FleetRng For(long seed, string stream) {
            ulong h = 1469598103934665603UL;
            foreach (char c in stream) { h ^= c; h *= 1099511628211UL; }
            return new FleetRng((ulong)seed * 0x9E3779B97F4A7C15UL ^ h);
        }

        public ulong NextU64() {
            ulong z = state += 0x9E3779B97F4A7C15UL;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
            return z ^ (z >> 31);
        }

        /// <summary>Uniform in [0, 1).</summary>
        public double Uniform() => (NextU64() >> 11) * (1.0 / 9007199254740992.0);
        public double Uniform(double a, double b) => a + (b - a) * Uniform();

        public double Gaussian() {
            if (hasSpare) { hasSpare = false; return spare; }
            double u, v, s;
            do { u = 2 * Uniform() - 1; v = 2 * Uniform() - 1; s = u * u + v * v; } while (s >= 1 || s == 0);
            double m = Math.Sqrt(-2 * Math.Log(s) / s);
            spare = v * m; hasSpare = true;
            return u * m;
        }
    }

    /// <summary>Wheel layout and limits of a four-wheel 45-degree mecanum base (X roller pattern seen from above).
    /// Wheel order everywhere: fl, fr, rl, rr.</summary>
    public sealed class MecanumGeometry {
        public readonly double Radius, Lx, Ly, MaxWheelSpeed, MaxWheelTorque, MaxSpeedXy, MaxYawRate;
        public readonly int CountsPerRev;

        public MecanumGeometry(double radius, double lx, double ly, double maxWheelSpeed, double maxWheelTorque, int countsPerRev,
            double maxSpeedXy, double maxYawRate) {
            Radius = radius; Lx = lx; Ly = ly; MaxWheelSpeed = maxWheelSpeed; MaxWheelTorque = maxWheelTorque;
            CountsPerRev = countsPerRev; MaxSpeedXy = maxSpeedXy; MaxYawRate = maxYawRate;
        }

        public double K => Lx + Ly;

        /// <summary>Standard mecanum inverse kinematics: body twist to wheel angular speeds (rad/s).</summary>
        public void Inverse(Twist t, double[] w) {
            double k = K, r = Radius;
            w[0] = (t.Vx - t.Vy - k * t.Wz) / r;  // fl
            w[1] = (t.Vx + t.Vy + k * t.Wz) / r;  // fr
            w[2] = (t.Vx + t.Vy - k * t.Wz) / r;  // rl
            w[3] = (t.Vx - t.Vy + k * t.Wz) / r;  // rr
        }

        /// <summary>Forward kinematics (least squares of the inverse): wheel speeds to body twist.</summary>
        public Twist Forward(double[] w) {
            double r = Radius / 4;
            return new Twist(r * (w[0] + w[1] + w[2] + w[3]), r * (-w[0] + w[1] + w[2] - w[3]), r / K * (-w[0] + w[1] - w[2] + w[3]));
        }

        /// <summary>Weighted least-squares forward kinematics (weights 0 drop a wheel); with three or four weighted wheels the twist is determined.</summary>
        public Twist ForwardWeighted(double[] w, double[] weight) {
            double k = K, r = Radius;
            double[,] J = { { 1, -1, -k }, { 1, 1, k }, { 1, 1, -k }, { 1, -1, k } };   // w_i = J_i . twist / r
            var A = new double[3, 3]; var b = new double[3];
            for (int i = 0; i < 4; i++)
                for (int p = 0; p < 3; p++) {
                    b[p] += weight[i] * J[i, p] * w[i] * r;
                    for (int q = 0; q < 3; q++) A[p, q] += weight[i] * J[i, p] * J[i, q];
                }
            // 3x3 solve (Cramer); a singular system (fewer than three wheels) gives zero motion
            double det = A[0, 0] * (A[1, 1] * A[2, 2] - A[1, 2] * A[2, 1]) - A[0, 1] * (A[1, 0] * A[2, 2] - A[1, 2] * A[2, 0]) + A[0, 2] * (A[1, 0] * A[2, 1] - A[1, 1] * A[2, 0]);
            if (Math.Abs(det) < 1e-12) return new Twist();
            double D(int col) {
                var m = (double[,])A.Clone();
                for (int i = 0; i < 3; i++) m[i, col] = b[i];
                return m[0, 0] * (m[1, 1] * m[2, 2] - m[1, 2] * m[2, 1]) - m[0, 1] * (m[1, 0] * m[2, 2] - m[1, 2] * m[2, 0]) + m[0, 2] * (m[1, 0] * m[2, 1] - m[1, 1] * m[2, 0]);
            }
            return new Twist(D(0) / det, D(1) / det, D(2) / det);
        }

        /// <summary>Clamp a command to the body limits, then scale all four wheels together if one exceeds its limit
        /// (preserves the direction of motion). Returns the scale applied (1 = unsaturated).</summary>
        public double Saturate(ref Twist t, double[] w) {
            double sp = Math.Sqrt(t.Vx * t.Vx + t.Vy * t.Vy);
            if (sp > MaxSpeedXy) { t.Vx *= MaxSpeedXy / sp; t.Vy *= MaxSpeedXy / sp; }
            if (Math.Abs(t.Wz) > MaxYawRate) t.Wz = Math.Sign(t.Wz) * MaxYawRate;
            Inverse(t, w);
            double m = 0;
            for (int i = 0; i < 4; i++) m = Math.Max(m, Math.Abs(w[i]));
            if (m <= MaxWheelSpeed) return 1;
            double s = MaxWheelSpeed / m;
            for (int i = 0; i < 4; i++) w[i] *= s;
            return s;
        }
    }

    /// <summary>Per-wheel actuator health (CONFIG_SPACE.md A9): ok, dead_free (motor off, wheel rolls freely), dead_locked (wheel cannot turn),
    /// weak (torque and speed limits scaled by <see cref="MecanumPlant.WeakFraction"/>).</summary>
    public enum WheelHealth { Ok, DeadFree, DeadLocked, Weak }

    /// <summary>Plant parameters that are not geometry: floor, motor lag, tracking compliance, mass properties.</summary>
    public sealed class PlantParams {
        public double MotorTimeConstant, ContactTrackingTime, SlipCorrelationTime;
        public double LateralEfficiency, SlipSigma, Mu;
        public double Mass, Izz, ComX, ComY, Gravity;
        public double ComZ;   // CoM height above the floor (load transfer lever)

        public static PlantParams From(FleetSpec spec, RoverDescription d, FloorProperties floor) => new PlantParams {
            MotorTimeConstant = spec.O["plant.motor_time_constant"].Double,
            ContactTrackingTime = spec.O["plant.contact_tracking_time"].Double,
            SlipCorrelationTime = spec.O["plant.slip_noise_correlation_time"].Double,
            LateralEfficiency = floor.LateralEfficiency, SlipSigma = floor.SlipSigmaFrac, Mu = floor.MuLong,
            Mass = d.Mass, Izz = d.InertiaDiag[2], ComX = d.ComOffset[0], ComY = d.ComOffset[1], ComZ = d.ComOffset[2] + d.OriginHeight,  // CoM height above the floor
            Gravity = spec.Gravity
        };
    }

    /// <summary>
    /// Body-level mecanum plant (engine-free). Per step: commanded twist -> limits -> inverse kinematics ->
    /// per-wheel first-order motor lag -> per-wheel slip noise (Gauss-Markov, floor sigma) -> forward
    /// kinematics with the floor's lateral efficiency -> the body acceleration that pulls the measured
    /// twist toward the wheel-implied twist within <see cref="PlantParams.ContactTrackingTime"/>, limited
    /// per wheel by traction mu*N_i (N_i from the centre of mass, with load transfer from the last
    /// acceleration) and by motor torque / radius. The simulator applies the acceleration to a rigid body.
    /// </summary>
    public sealed class MecanumPlant {
        public readonly MecanumGeometry G;
        public readonly PlantParams P;
        readonly FleetRng rng;

        public readonly double[] WheelCommand = new double[4];   // rad/s after limits
        public readonly double[] WheelSpeed = new double[4];     // rad/s, motor output (what encoders see)
        public readonly double[] WheelAngle = new double[4];     // rad, integrated motor output
        public readonly double[] SlipFactor = new double[4];     // ground speed / wheel speed, 1 + noise
        public readonly double[] NormalLoad = new double[4];     // N
        public readonly double[] WheelForce = new double[4];     // N, magnitude after limiting
        public Twist Target;                                     // wheel-implied twist after slip and lateral efficiency
        public double Ax, Ay, Alpha;                             // body-frame acceleration to apply
        public double TractionScale = 1;                         // 1 = no traction or torque limit hit
        public readonly WheelHealth[] Health = { WheelHealth.Ok, WheelHealth.Ok, WheelHealth.Ok, WheelHealth.Ok };
        public double WeakFraction = 0.3;
        public bool Healthy => Health[0] == WheelHealth.Ok && Health[1] == WheelHealth.Ok && Health[2] == WheelHealth.Ok && Health[3] == WheelHealth.Ok;
        public double SpeedScale = 1;                            // 1 = no wheel-speed limit hit
        double lastAx, lastAy;

        public MecanumPlant(MecanumGeometry g, PlantParams p, FleetRng rng) {
            G = g; P = p; this.rng = rng;
            for (int i = 0; i < 4; i++) SlipFactor[i] = 1;
        }

        public void Step(Twist command, Twist measured, double dt) {
            SpeedScale = G.Saturate(ref command, WheelCommand);
            double a = 1 - Math.Exp(-dt / P.MotorTimeConstant);
            double rho = Math.Exp(-dt / P.SlipCorrelationTime), q = P.SlipSigma * Math.Sqrt(1 - rho * rho);
            var ground = new double[4];
            var measuredWheel = new double[4];
            G.Inverse(measured, measuredWheel);
            for (int i = 0; i < 4; i++) {
                double cmd = WheelCommand[i];
                if (Health[i] == WheelHealth.Weak) cmd = Math.Max(-G.MaxWheelSpeed * WeakFraction, Math.Min(G.MaxWheelSpeed * WeakFraction, cmd));
                switch (Health[i]) {
                    case WheelHealth.DeadLocked: WheelSpeed[i] = 0; break;
                    case WheelHealth.DeadFree: WheelSpeed[i] = measuredWheel[i]; break;   // rolls with the ground
                    default: WheelSpeed[i] += (cmd - WheelSpeed[i]) * a; break;
                }
                WheelAngle[i] += WheelSpeed[i] * dt;
                if (P.SlipSigma > 0) SlipFactor[i] = 1 + (SlipFactor[i] - 1) * rho + q * rng.Gaussian();
                ground[i] = WheelSpeed[i] * SlipFactor[i];
            }
            // wheel-implied twist: least squares over the wheels that constrain the body (a dead-free wheel does not)
            Twist t;
            if (Healthy) t = G.Forward(ground);
            else {
                var w = new double[4];
                for (int i = 0; i < 4; i++) w[i] = Health[i] == WheelHealth.DeadFree ? 0 : 1;
                t = G.ForwardWeighted(ground, w);
            }
            t.Vy *= P.LateralEfficiency;
            Target = t;

            double tau = P.ContactTrackingTime;
            double ax = (t.Vx - measured.Vx) / tau, ay = (t.Vy - measured.Vy) / tau, al = (t.Wz - measured.Wz) / tau;

            // normal loads from the CoM, shifted by the previous step's acceleration (avoids an algebraic loop)
            double cx = P.ComX - P.ComZ * lastAx / P.Gravity, cy = P.ComY - P.ComZ * lastAy / P.Gravity;
            double fx = 0.5 + cx / (2 * G.Lx), fy = 0.5 + cy / (2 * G.Ly);
            fx = Math.Min(1, Math.Max(0, fx)); fy = Math.Min(1, Math.Max(0, fy));
            double mg = P.Mass * P.Gravity;
            NormalLoad[0] = mg * fx * fy;             // fl: front (+x), left (+y)
            NormalLoad[1] = mg * fx * (1 - fy);       // fr
            NormalLoad[2] = mg * (1 - fx) * fy;       // rl
            NormalLoad[3] = mg * (1 - fx) * (1 - fy); // rr

            // share of the required body force and yaw moment among the wheels that can push (all but dead-free);
            // one common scale keeps the direction
            int active = 0;
            for (int i = 0; i < 4; i++) if (Health[i] != WheelHealth.DeadFree) active++;
            double Fx = P.Mass * ax, Fy = P.Mass * ay, Mz = P.Izz * al, R2 = G.Lx * G.Lx + G.Ly * G.Ly;
            double torqueLimit = G.MaxWheelTorque / G.Radius, k = active == 0 ? 0 : 1;
            for (int i = 0; i < 4; i++) {
                WheelForce[i] = 0;
                if (Health[i] == WheelHealth.DeadFree || active == 0) continue;
                double rx = i < 2 ? G.Lx : -G.Lx, ry = (i % 2 == 0) ? G.Ly : -G.Ly;
                double fxi = Fx / active - Mz * ry / (active * R2), fyi = Fy / active + Mz * rx / (active * R2);
                double f = Math.Sqrt(fxi * fxi + fyi * fyi);
                double limit = Math.Min(P.Mu * NormalLoad[i], torqueLimit * (Health[i] == WheelHealth.Weak ? WeakFraction : 1));
                WheelForce[i] = f;
                if (f > limit && f > 0) k = Math.Min(k, limit / f);
            }
            TractionScale = k;
            for (int i = 0; i < 4; i++) WheelForce[i] *= k;
            Ax = ax * k; Ay = ay * k; Alpha = al * k;
            lastAx = Ax; lastAy = Ay;
        }
    }

    /// <summary>Quantized incremental encoders on the four wheels (counts per revolution from the profile).</summary>
    public sealed class WheelEncoders {
        readonly int cpr;
        public readonly long[] Ticks = new long[4];
        public WheelEncoders(int countsPerRev) { cpr = countsPerRev; }

        public void Sample(double[] wheelAngle) {
            for (int i = 0; i < 4; i++) Ticks[i] = (long)Math.Floor(wheelAngle[i] * cpr / (2 * Math.PI));
        }
        public double TicksToRad(long ticks) => ticks * 2 * Math.PI / cpr;
    }

    /// <summary>Gyro z: truth + bias (initial offset, random walk) + white noise.</summary>
    public sealed class YawGyro {
        readonly FleetRng rng;
        readonly double noiseDensity, biasWalk;
        public double Bias, Last;

        public YawGyro(FleetRng rng, double noiseDensity, double biasInstability, double initialBiasSigma) {
            this.rng = rng; this.noiseDensity = noiseDensity; biasWalk = biasInstability;
            Bias = initialBiasSigma * rng.Gaussian();
        }

        public double Sample(double trueRate, double dt) {
            Bias += biasWalk * Math.Sqrt(dt) * rng.Gaussian();
            Last = trueRate + Bias + noiseDensity / Math.Sqrt(dt) * rng.Gaussian();
            return Last;
        }
    }

    /// <summary>The rover's own pose estimate: wheel-encoder translation (ideal kinematics, no knowledge of slip or
    /// lateral efficiency) and gyro-integrated heading. Never reads truth after initialisation.</summary>
    public sealed class WheelOdometry {
        readonly MecanumGeometry g;
        readonly WheelEncoders enc;
        readonly long[] last = new long[4];
        public double X, Y, Yaw;
        public Twist LastTwist;

        public WheelOdometry(MecanumGeometry g, WheelEncoders enc, double x, double y, double yaw) {
            this.g = g; this.enc = enc; X = x; Y = y; Yaw = yaw;
        }

        public void IntegrateGyro(double rate, double dt) { Yaw += rate * dt; }

        public void IntegrateEncoders(double dt) {
            var dw = new double[4];
            for (int i = 0; i < 4; i++) { dw[i] = enc.TicksToRad(enc.Ticks[i] - last[i]); last[i] = enc.Ticks[i]; }
            Twist d = g.Forward(dw);   // displacement in the body frame over the interval
            LastTwist = new Twist(d.Vx / dt, d.Vy / dt, d.Wz / dt);
            double c = Math.Cos(Yaw), s = Math.Sin(Yaw);
            X += c * d.Vx - s * d.Vy;
            Y += s * d.Vx + c * d.Vy;
        }
    }
}
