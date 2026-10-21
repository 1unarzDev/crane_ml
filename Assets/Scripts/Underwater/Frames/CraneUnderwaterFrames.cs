using static Sim.Underwater.CraneLinAlg;

namespace Sim.Underwater {
    /// <summary>
    /// Explicit frame maps between Unity (left-handed, +X right, +Y up, +Z forward) and
    /// NED/FRD (right-handed, X north/forward, Y east/starboard, Z down) or ROS FLU (X forward, Y left, Z up).
    ///
    /// Polar vectors:   NED/FRD = ( Uz,  Ux, -Uy);   FLU = ( Uz, -Ux,  Uy).
    /// Axial vectors (angular velocity, torque): the maps have det = -1, so
    ///                  NED/FRD = (-Uz, -Ux,  Uy);   FLU = (-Uz,  Ux, -Uy).
    /// Check: a Unity +Y (yaw right, clockwise from above) rotation is +r about Down in NED, -wz in FLU.
    /// The same map applies to world and body frames (Unity body: x right, y up, z forward = FRD starboard/down/forward).
    /// </summary>
    public static class CraneUnderwaterFrames {
        static readonly double[,] P = { { 0, 0, 1 }, { 1, 0, 0 }, { 0, -1, 0 } };   // Unity -> NED/FRD (det -1)

        public static double[] UnityToNed(double x, double y, double z) => new[] { z, x, -y };
        public static double[] NedToUnity(double[] n) => new[] { n[1], -n[2], n[0] };
        public static double[] UnityToFlu(double x, double y, double z) => new[] { z, -x, y };
        public static double[] FluToUnity(double[] f) => new[] { -f[1], f[2], f[0] };
        public static double[] NedToFlu(double[] n) => new[] { n[0], -n[1], -n[2] };
        public static double[] FluToNed(double[] f) => new[] { f[0], -f[1], -f[2] };

        public static double[] UnityAxialToNed(double x, double y, double z) => new[] { -z, -x, y };
        public static double[] NedAxialToUnity(double[] n) => new[] { -n[1], n[2], -n[0] };
        public static double[] UnityAxialToFlu(double x, double y, double z) => new[] { -z, x, -y };
        public static double[] FluAxialToUnity(double[] f) => new[] { f[1], -f[2], -f[0] };
        public static double[] NedAxialToFlu(double[] n) => new[] { n[0], -n[1], -n[2] };   // proper rotation: same as polar

        /// <summary>Unity quaternion (x,y,z,w) body-to-world to [w x y z] FRD-body to NED-world.</summary>
        public static double[] UnityQuatToNedFrd(double x, double y, double z, double w) {
            var ru = QuatToRot(new[] { w, x, y, z });
            return RotToQuat(Mul(Mul(P, ru), Transpose(P)));
        }

        /// <summary>[w x y z] FRD-to-NED back to Unity (x, y, z, w) body-to-world.</summary>
        public static double[] NedFrdQuatToUnity(double[] q) {
            var rn = QuatToRot(q);
            var ru = Mul(Mul(Transpose(P), rn), P);
            var wxyz = RotToQuat(ru);
            return new[] { wxyz[1], wxyz[2], wxyz[3], wxyz[0] };
        }

        public static double Determinant(double[,] m) =>
            m[0, 0] * (m[1, 1] * m[2, 2] - m[1, 2] * m[2, 1]) - m[0, 1] * (m[1, 0] * m[2, 2] - m[1, 2] * m[2, 0])
            + m[0, 2] * (m[1, 0] * m[2, 1] - m[1, 1] * m[2, 0]);

        public static double[,] UnityToNedMatrix() => (double[,])P.Clone();
    }
}
