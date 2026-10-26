using UnityEngine;

namespace Sim.Physics.Core {
    /// <summary>
    /// The single conversion point between Unity (left-handed, y up) and the body FLU frame at the
    /// centre of mass, with world ENU. Linear vectors use the polar map FLU(x,y,z) = (z,-x,y);
    /// angular vectors are axial and take the reflection's sign, -FLU(w).
    /// </summary>
    public interface IBodyFrame {
        /// <summary>Unity world vector (velocity, force) to body FLU.</summary>
        Vector3 BodyLinear(Vector3 unityWorld);
        /// <summary>Unity world angular vector (rate, torque) to body FLU.</summary>
        Vector3 BodyAngular(Vector3 unityWorld);
        Vector3 UnityLinear(Vector3 body);
        Vector3 UnityAngular(Vector3 body);
        /// <summary>Unity world point to world ENU [m].</summary>
        Vector3 WorldEnu(Vector3 unityWorldPoint);
        /// <summary>Heading of body +x, rad CCW from ENU east.</summary>
        float Yaw { get; }
    }

    /// <summary>
    /// <see cref="IBodyFrame"/> of a transform whose visible bow is
    /// <c>rotation * bodyFromFlu * Vector3.forward</c>. Identity for a body authored bow along
    /// local +Z; the RoboBoat hull uses Euler(0,-90,0) (visible bow = local -X), the same offset as
    /// the controllers' RoboBoatRosFrame.
    /// </summary>
    public sealed class BodyFrame : IBodyFrame {
        private readonly Transform transform;
        private readonly Quaternion bodyFromFlu;

        public BodyFrame(Transform transform, Quaternion bodyFromFlu) {
            this.transform = transform;
            this.bodyFromFlu = bodyFromFlu;
        }

        public Quaternion Rotation => transform.rotation * bodyFromFlu;

        public Vector3 BodyLinear(Vector3 unityWorld) => Flu(Quaternion.Inverse(Rotation) * unityWorld);
        public Vector3 BodyAngular(Vector3 unityWorld) => -Flu(Quaternion.Inverse(Rotation) * unityWorld);
        public Vector3 UnityLinear(Vector3 body) => Rotation * Ruf(body);
        public Vector3 UnityAngular(Vector3 body) => Rotation * -Ruf(body);
        public Vector3 WorldEnu(Vector3 p) => Flu(p);

        public float Yaw {
            get {
                Vector3 bow = Flu(Rotation * Vector3.forward);
                return Mathf.Atan2(bow.y, bow.x);
            }
        }

        public static Vector3 Flu(Vector3 ruf) => new(ruf.z, -ruf.x, ruf.y);
        public static Vector3 Ruf(Vector3 flu) => new(-flu.y, flu.z, flu.x);
    }
}
