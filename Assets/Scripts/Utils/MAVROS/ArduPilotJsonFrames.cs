using System;

namespace Sim.Sensors.Nav {
    /// <summary>Double-precision 3-vector (no UnityEngine dependency).</summary>
    public readonly struct Vec3d {
        public readonly double X, Y, Z;
        public Vec3d(double x, double y, double z) { X = x; Y = y; Z = z; }
        public static Vec3d operator +(Vec3d a, Vec3d b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        public static Vec3d operator -(Vec3d a, Vec3d b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        public static Vec3d operator *(Vec3d a, double s) => new(a.X * s, a.Y * s, a.Z * s);
        public double[] ToArray() => new[] { X, Y, Z };
    }

    /// <summary>Unity quaternion components (x, y, z, w) as stored by UnityEngine.Quaternion.</summary>
    public readonly struct QuatU {
        public readonly double X, Y, Z, W;
        public QuatU(double x, double y, double z, double w) { X = x; Y = y; Z = z; W = w; }
    }

    /// <summary>
    /// Pure frame conversions between Unity (left-handed RUF: x right, y up, z forward) and the
    /// ArduPilot JSON SITL backend (libraries/SITL/examples/JSON/readme.md @9c412ba):
    /// position/velocity in NED earth frame, gyro and accel_body in the FRD body frame,
    /// attitude as aerospace roll/pitch/yaw (ZYX) of the FRD body in NED, timestamp = absolute
    /// physics time in seconds.
    /// Mapping M (Unity -> NED/FRD): N = z, E = x, D = -y; M has det -1, so polar vectors map
    /// by M and pseudovectors (angular velocity) by det(M) M = -M.
    /// </summary>
    public static class ArduPilotJsonFrames {
        public const int ServoChannels = 4;

        /// <summary>Unity world position or velocity to NED.</summary>
        public static Vec3d UnityToNed(Vec3d u) => new(u.Z, u.X, -u.Y);

        /// <summary>Rotation matrix (row-major) of a Unity quaternion, raw Hamilton formulas.</summary>
        public static double[,] RotationMatrix(QuatU q) {
            double n = Math.Sqrt(q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W);
            double x = q.X / n, y = q.Y / n, z = q.Z / n, w = q.W / n;
            return new[,] {
                { 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y) },
                { 2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x) },
                { 2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y) },
            };
        }

        /// <summary>World vector to body axes (R^T v), still in Unity axes.</summary>
        public static Vec3d WorldToBodyUnity(QuatU q, Vec3d v) {
            double[,] r = RotationMatrix(q);
            return new Vec3d(r[0, 0] * v.X + r[1, 0] * v.Y + r[2, 0] * v.Z,
                r[0, 1] * v.X + r[1, 1] * v.Y + r[2, 1] * v.Z,
                r[0, 2] * v.X + r[1, 2] * v.Y + r[2, 2] * v.Z);
        }

        /// <summary>Unity body axes (right, up, forward) polar vector to FRD (forward, right, down).</summary>
        public static Vec3d BodyUnityToFrd(Vec3d b) => new(b.Z, b.X, -b.Y);

        /// <summary>
        /// Body-frame FRD gyro from Rigidbody.angularVelocity (world, Unity). Pseudovector:
        /// -(M R^T w). Positive yaw about Unity +y (clockwise from above) is +z (yaw right).
        /// </summary>
        public static Vec3d GyroFrd(QuatU rotation, Vec3d angularVelocityWorldUnity) {
            Vec3d b = WorldToBodyUnity(rotation, angularVelocityWorldUnity);
            Vec3d f = BodyUnityToFrd(b);
            return new Vec3d(-f.X, -f.Y, -f.Z);
        }

        /// <summary>
        /// Accelerometer specific force in FRD: R^T(a_world - g_world), mapped to FRD. At rest
        /// level this is (0, 0, -|g|). Not a constant: callers pass the measured world
        /// acceleration (velocity difference over the physics tick).
        /// </summary>
        public static Vec3d AccelBodyFrd(QuatU rotation, Vec3d accelerationWorldUnity,
            Vec3d gravityWorldUnity) {
            Vec3d specific = accelerationWorldUnity - gravityWorldUnity;
            return BodyUnityToFrd(WorldToBodyUnity(rotation, specific));
        }

        /// <summary>Finite-difference world acceleration over one physics tick.</summary>
        public static Vec3d AccelerationFromVelocities(Vec3d previous, Vec3d current, double dt) {
            if (!(dt > 0.0)) throw new ArgumentOutOfRangeException(nameof(dt));
            return (current - previous) * (1.0 / dt);
        }

        /// <summary>Aerospace (roll, pitch, yaw) in radians of the FRD body in NED, R = Rz Ry Rx.</summary>
        public static Vec3d AttitudeEuler(QuatU rotation) {
            double[,] r = RotationMatrix(rotation);
            // R_ned_frd = M R_u M^T with M rows (0,0,1), (1,0,0), (0,-1,0).
            int[] src = { 2, 0, 1 };
            double[] sgn = { 1, 1, -1 };
            double[,] m = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    m[i, j] = sgn[i] * sgn[j] * r[src[i], src[j]];
            double pitch = -Math.Asin(Math.Max(-1.0, Math.Min(1.0, m[2, 0])));
            double roll = Math.Atan2(m[2, 1], m[2, 2]);
            double yaw = Math.Atan2(m[1, 0], m[0, 0]);
            return new Vec3d(roll, pitch, yaw);
        }

        /// <summary>Absolute physics time of a fixed tick, in double precision.</summary>
        public static double Timestamp(long tick, double fixedDeltaTime) => tick * fixedDeltaTime;

        // ArduCopter MOTOR_FRAME_TYPE_X (QUAD), AP_MotorsMatrix.cpp @9c412ba:
        // servo output i -> FR, RL, FL, RR, yaw factors CCW, CCW, CW, CW.
        public static readonly string[] QuadXChannelNames = { "front_right", "rear_left", "front_left", "rear_right" };
        /// <summary>Spin of servo channel i seen from above: +1 CCW, -1 CW.</summary>
        public static readonly int[] QuadXSpin = { 1, 1, -1, -1 };

        /// <summary>CRANE MultirotorDynamics rotor order: FL, FR, RR, RL.</summary>
        public static readonly string[] CraneRotorNames = { "front_left", "front_right", "rear_right", "rear_left" };
        /// <summary>Servo channel -> index into the CRANE FL, FR, RR, RL rotor order.</summary>
        public static readonly int[] QuadXChannelToCraneRotor = { 1, 3, 0, 2 };

        /// <summary>Quadrant name of a rotor at FLU position (x forward, y left).</summary>
        public static string QuadrantName(double xForward, double yLeft) =>
            (xForward >= 0 ? "front_" : "rear_") + (yLeft >= 0 ? "left" : "right");

        /// <summary>
        /// Maps servo channels 0..3 to rotor indices of a profile by FLU quadrant and reports
        /// whether the profile's spin pattern equals ArduCopter QUAD/X.
        /// </summary>
        public static int[] ChannelToRotorByGeometry(double[][] rotorPositionsFlu, int[] spins,
            out bool spinMatches) {
            if (rotorPositionsFlu.Length != ServoChannels || spins.Length != ServoChannels)
                throw new ArgumentException("QUAD/X mapping needs exactly four rotors.");
            int[] map = new int[ServoChannels];
            spinMatches = true;
            for (int ch = 0; ch < ServoChannels; ch++) {
                int found = -1;
                for (int r = 0; r < ServoChannels; r++) {
                    if (QuadrantName(rotorPositionsFlu[r][0], rotorPositionsFlu[r][1]) == QuadXChannelNames[ch]) {
                        found = r; break;
                    }
                }
                if (found < 0) throw new ArgumentException($"No rotor in quadrant {QuadXChannelNames[ch]}.");
                map[ch] = found;
                if (spins[found] != QuadXSpin[ch]) spinMatches = false;
            }
            return map;
        }

        /// <summary>Normalizes a servo PWM (microseconds) to [0, 1].</summary>
        public static double NormalizePwm(double pwm, double pwmMin, double pwmMax) {
            if (!(pwmMax > pwmMin)) throw new ArgumentException("pwmMax must exceed pwmMin.");
            return (Math.Max(pwmMin, Math.Min(pwmMax, pwm)) - pwmMin) / (pwmMax - pwmMin);
        }
    }
}
