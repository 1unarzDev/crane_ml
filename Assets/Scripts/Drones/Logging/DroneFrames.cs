using UnityEngine;

namespace Crane.Drones.Logging {
    /// <summary>Unity (x right, y up, z forward) to world ENU (x = Unity z, y = -Unity x, z = Unity y), as CraneTrajRecorder.</summary>
    public static class DroneFrames {
        public static Vector3 Enu(Vector3 u) => new Vector3(u.z, -u.x, u.y);

        /// <summary>World-from-body quaternion (w,x,y,z) in ENU/FLU for a Unity rotation whose forward/left/up are FLU x/y/z.</summary>
        public static void QuatEnuFlu(Quaternion rot, double[] q) {
            Vector3 bx = Enu(rot * Vector3.forward), by = Enu(rot * Vector3.left), bz = Enu(rot * Vector3.up);
            double m00 = bx.x, m10 = bx.y, m20 = bx.z, m01 = by.x, m11 = by.y, m21 = by.z, m02 = bz.x, m12 = bz.y, m22 = bz.z;
            double tr = m00 + m11 + m22, w, x, y, z;
            if (tr > 0) { double s = System.Math.Sqrt(tr + 1) * 2; w = s / 4; x = (m21 - m12) / s; y = (m02 - m20) / s; z = (m10 - m01) / s; }
            else if (m00 > m11 && m00 > m22) { double s = System.Math.Sqrt(1 + m00 - m11 - m22) * 2; w = (m21 - m12) / s; x = s / 4; y = (m01 + m10) / s; z = (m02 + m20) / s; }
            else if (m11 > m22) { double s = System.Math.Sqrt(1 + m11 - m00 - m22) * 2; w = (m02 - m20) / s; x = (m01 + m10) / s; y = s / 4; z = (m12 + m21) / s; }
            else { double s = System.Math.Sqrt(1 + m22 - m00 - m11) * 2; w = (m10 - m01) / s; x = (m02 + m20) / s; y = (m12 + m21) / s; z = s / 4; }
            if (w < 0) { w = -w; x = -x; y = -y; z = -z; } // canonical sign: w >= 0
            q[0] = w; q[1] = x; q[2] = y; q[3] = z;
        }
    }
}
