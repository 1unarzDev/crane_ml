using UnityEngine;

namespace Sim.Physics.Core {
    /// <summary>Where a model's wrench goes: <see cref="Hydro"/> through the plant's
    /// <see cref="IInertiaCoupling"/> (so it sees the added mass), <see cref="Direct"/> straight
    /// to Unity.</summary>
    public enum WrenchRoute { Hydro, Direct }

    /// <summary>Force [N] and torque [N m] in the body FLU frame, about the centre of mass.
    /// Also used as a 6-vector of per-DOF values (x, y, z, roll, pitch, yaw), for example the
    /// diagonal of a damping Jacobian.</summary>
    public struct Wrench {
        public Vector3 Force;
        public Vector3 Torque;

        public Wrench(Vector3 force, Vector3 torque) {
            Force = force;
            Torque = torque;
        }

        /// <summary>Adds <paramref name="force"/> acting at <paramref name="arm"/> (body FLU, m,
        /// from the centre of mass).</summary>
        public void AddAt(Vector3 force, Vector3 arm) {
            Force += force;
            Torque += Vector3.Cross(arm, force);
        }

        public bool IsFinite => Finite(Force) && Finite(Torque);

        public static Wrench operator +(Wrench a, Wrench b) =>
            new(a.Force + b.Force, a.Torque + b.Torque);

        public override string ToString() => $"F={Force:F3} T={Torque:F3}";

        internal static bool Finite(Vector3 v) =>
            float.IsFinite(v.x) && float.IsFinite(v.y) && float.IsFinite(v.z);
    }
}
