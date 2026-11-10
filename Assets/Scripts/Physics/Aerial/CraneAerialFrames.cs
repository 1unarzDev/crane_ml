using System;

namespace Sim.Physics.Aerial {
    /// <summary>
    /// Pure frame conversions for the aerial plant (no UnityEngine dependency).
    /// Plant: world ENU (x east, y north, z up), body FLU (x forward, y left, z up), Hamilton
    /// quaternion (w, x, y, z) rotating body to world.
    /// Unity: left-handed RUF (x right, y up, z forward); world mapping E = x, N = z, U = y;
    /// body mapping F = z, L = -x, U = y. Both maps have det -1, so a polar vector maps by M and
    /// the composite rotation M_w R_u M_b^T is proper. Angular velocity (a pseudovector) must not
    /// be converted with the polar map: that was the v1 Imu.cs defect (To&lt;FLU&gt;() on omega).
    /// PX4/ArduPilot SITL: world NED, body FRD (ArduPilot libraries/SITL/examples/JSON/readme.md).
    /// </summary>
    public static class CraneAerialFrames {
        public static double[] EnuToUnity(double[] p) => new[] { p[0], p[2], p[1] };
        public static double[] UnityToEnu(double[] u) => new[] { u[0], u[2], u[1] };
        public static double[] FluToUnityBody(double[] b) => new[] { -b[1], b[2], b[0] };
        public static double[] UnityBodyToFlu(double[] u) => new[] { u[2], -u[0], u[1] };

        /// <summary>ENU position/velocity to NED.</summary>
        public static double[] EnuToNed(double[] p) => new[] { p[1], p[0], -p[2] };
        /// <summary>FLU body vector to FRD. Valid for polar vectors and for angular velocity,
        /// because FLU->FRD is a proper rotation (pi about x).</summary>
        public static double[] FluToFrd(double[] b) => new[] { b[0], -b[1], -b[2] };

        /// <summary>Unity body-frame forward and up axes expressed in Unity world, for LookRotation.</summary>
        public static void UnityLookAxes(double[] qEnuFlu, out double[] forwardUnity, out double[] upUnity) {
            double[,] r = CraneRotorModelV2.QuatToMatrix(qEnuFlu);
            forwardUnity = EnuToUnity(new[] { r[0, 0], r[1, 0], r[2, 0] });
            upUnity = EnuToUnity(new[] { r[0, 2], r[1, 2], r[2, 2] });
        }

        /// <summary>Hamilton quaternion from a proper rotation matrix (w &gt;= 0).</summary>
        public static double[] MatrixToQuat(double[,] r) {
            double t = r[0, 0] + r[1, 1] + r[2, 2];
            double[] q = new double[4];
            if (t > 0) {
                double s = Math.Sqrt(t + 1.0) * 2;
                q[0] = 0.25 * s; q[1] = (r[2, 1] - r[1, 2]) / s; q[2] = (r[0, 2] - r[2, 0]) / s; q[3] = (r[1, 0] - r[0, 1]) / s;
            } else {
                int i = r[1, 1] > r[0, 0] ? (r[2, 2] > r[1, 1] ? 2 : 1) : (r[2, 2] > r[0, 0] ? 2 : 0);
                int j = (i + 1) % 3, k = (i + 2) % 3;
                double s = Math.Sqrt(1.0 + r[i, i] - r[j, j] - r[k, k]) * 2;
                q[0] = (r[k, j] - r[j, k]) / s; q[1 + i] = 0.25 * s;
                q[1 + j] = (r[j, i] + r[i, j]) / s; q[1 + k] = (r[k, i] + r[i, k]) / s;
            }
            double sign = q[0] >= 0 ? 1 : -1;
            double n = Math.Sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]) * sign;
            for (int a = 0; a < 4; a++) q[a] /= n;
            return q;
        }

        /// <summary>ENU/FLU quaternion from Unity forward/up axes (Unity world coordinates).</summary>
        public static double[] QuatFromUnityAxes(double[] forwardUnity, double[] upUnity) {
            double[] f = UnityToEnu(forwardUnity), u = UnityToEnu(upUnity);
            double[] l = CraneRotorModelV2.Cross(u, f);
            double ln = CraneRotorModelV2.Norm(l);
            l = CraneRotorModelV2.Scale(l, 1.0 / ln);
            f = CraneRotorModelV2.Scale(f, 1.0 / CraneRotorModelV2.Norm(f));
            u = CraneRotorModelV2.Cross(f, l);
            var r = new[,] { { f[0], l[0], u[0] }, { f[1], l[1], u[1] }, { f[2], l[2], u[2] } };
            return MatrixToQuat(r);
        }

        /// <summary>Aerospace (roll, pitch, yaw) [rad] of the FRD body in NED, from an ENU/FLU quaternion.</summary>
        public static double[] NedFrdEuler(double[] qEnuFlu) {
            double[,] r = CraneRotorModelV2.QuatToMatrix(qEnuFlu);
            // R_ned_frd = T R D, T = [[0,1,0],[1,0,0],[0,0,-1]], D = diag(1,-1,-1)
            int[] row = { 1, 0, 2 }; double[] rs = { 1, 1, -1 }; double[] cs = { 1, -1, -1 };
            var m = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    m[i, j] = rs[i] * r[row[i], j] * cs[j];
            double pitch = -Math.Asin(Math.Max(-1.0, Math.Min(1.0, m[2, 0])));
            return new[] { Math.Atan2(m[2, 1], m[2, 2]), pitch, Math.Atan2(m[1, 0], m[0, 0]) };
        }

        /// <summary>Yaw (heading) about ENU +z of the body x axis, CCW from east [rad].</summary>
        public static double YawEnu(double[] qEnuFlu) {
            double[,] r = CraneRotorModelV2.QuatToMatrix(qEnuFlu);
            return Math.Atan2(r[1, 0], r[0, 0]);
        }
    }
}
