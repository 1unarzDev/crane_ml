using System;
using UnityEngine;

namespace Crane.Fleet.Sim {
    /// <summary>
    /// The one place frames are converted. World: scenario ENU (x east, y north, z up) maps to Unity (x, z, y);
    /// Unity yaw (deg, clockwise about +y) = 90 - ENU yaw (deg). Chassis: FLU (x forward, y left, z up) maps to
    /// Unity local (-y, z, x), so a rover's Unity +z is its forward. Both maps are reflections, so a right-handed
    /// rotation by a about axis n becomes Unity's AngleAxis(-a, map(n)).
    /// </summary>
    public static class FleetFrames {
        public static Vector3 World(double x, double y, double z) => new Vector3((float)x, (float)z, (float)y);
        public static Vector3 Local(double f, double l, double u) => new Vector3((float)-l, (float)u, (float)f);
        public static Vector3 Local(double[] flu) => Local(flu[0], flu[1], flu[2]);
        /// <summary>Unity scale of a chassis-frame box with extents (length along x forward, width along y left, height).</summary>
        public static Vector3 LocalScale(double l, double w, double h) => new Vector3((float)w, (float)h, (float)l);
        /// <summary>Unity scale of a world box with ENU extents (east, north, up).</summary>
        public static Vector3 WorldScale(double east, double north, double up) => new Vector3((float)east, (float)up, (float)north);

        public static Quaternion WorldYaw(double yawRad) => Quaternion.Euler(0f, (float)(90.0 - yawRad * 180.0 / Math.PI), 0f);
        public static Quaternion LocalYawPitch(double yawRad, double pitchRad) =>
            Quaternion.AngleAxis((float)(-yawRad * 180.0 / Math.PI), Vector3.up) *
            Quaternion.AngleAxis((float)(pitchRad * 180.0 / Math.PI), Vector3.right);

        public static Quaternion Local(FramePose p) => LocalYawPitch(p.Yaw, p.Pitch);

        /// <summary>Unity local rotation of a manifest rpy (R = Rz(yaw) Ry(pitch) Rx(roll), right-handed FLU).</summary>
        public static Quaternion Rpy(double[] rpy) =>
            Quaternion.AngleAxis((float)(-rpy[2] * 180.0 / Math.PI), Vector3.up) *
            Quaternion.AngleAxis((float)(rpy[1] * 180.0 / Math.PI), Vector3.right) *
            Quaternion.AngleAxis((float)(-rpy[0] * 180.0 / Math.PI), Vector3.forward);

        /// <summary>Unity rotation of a right-handed rotation by <paramref name="angle"/> rad about FLU axis <paramref name="axis"/>.</summary>
        public static Quaternion AxisAngle(double[] axis, double angle) =>
            Quaternion.AngleAxis((float)(-angle * 180.0 / Math.PI), Local(axis));

        /// <summary>Planar pose (ENU x, y, yaw) of a world transform whose local +z is its forward.</summary>
        public static Pose2 Planar(Transform t) {
            ToEnu(t.position, out double x, out double y, out _);
            Attitude(t.rotation, out double yaw, out _, out _);
            return new Pose2(x, y, yaw);
        }
        public static Vector3 LocalPosition(FramePose p) => Local(p.X, p.Y, p.Z);

        /// <summary>ENU position of a Unity point.</summary>
        public static void ToEnu(Vector3 u, out double x, out double y, out double z) { x = u.x; y = u.z; z = u.y; }

        /// <summary>ENU yaw, pitch (nose down +), roll (left side up +) of a Unity rotation whose local +z is forward.</summary>
        public static void Attitude(Quaternion q, out double yaw, out double pitch, out double roll) {
            Vector3 f = q * Vector3.forward, l = q * Vector3.left, u = q * Vector3.up;
            yaw = Math.Atan2(f.z, f.x);
            pitch = -Math.Asin(Mathf.Clamp(f.y, -1f, 1f));
            roll = Math.Atan2(l.y, u.y);
        }

        /// <summary>Planar body twist (FLU) of a rigid body: velocity in its own frame and yaw rate.</summary>
        public static Twist BodyTwist(Rigidbody rb) {
            Vector3 v = Quaternion.Inverse(rb.rotation) * rb.linearVelocity;
            Vector3 w = Quaternion.Inverse(rb.rotation) * rb.angularVelocity;
            return new Twist(v.z, -v.x, -w.y);
        }

        public static Color Rgb(double[] c) => new Color((float)c[0], (float)c[1], (float)c[2], 1f);
    }
}
