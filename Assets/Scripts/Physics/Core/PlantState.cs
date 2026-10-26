using UnityEngine;

namespace Sim.Physics.Core {
    /// <summary>One tick's snapshot, computed once by <see cref="PlantBase"/> and passed
    /// <c>in</c> to every actuator and force model. Body vectors are FLU at the centre of mass.</summary>
    public readonly struct PlantState {
        public readonly IBodyFrame Frame;
        /// <summary>Centre of mass, Unity world [m] (for arms of world-located forces).</summary>
        public readonly Vector3 CenterOfMass;
        /// <summary>Centre of mass in world ENU [m] and heading [rad CCW from east].</summary>
        public readonly Vector3 Position;
        public readonly float Yaw;
        /// <summary>ν: body linear [m/s] and angular [rad/s] velocity.</summary>
        public readonly Vector3 Linear, Angular;
        /// <summary>Linear velocity of the medium at the centre of mass, body FLU [m/s].</summary>
        public readonly Vector3 MediumLinear;
        public readonly float Dt;
        public readonly double Time;

        public PlantState(IBodyFrame frame, Vector3 centerOfMass, Vector3 linear, Vector3 angular,
            Vector3 mediumLinear, float dt, double time) {
            Frame = frame;
            CenterOfMass = centerOfMass;
            Position = frame.WorldEnu(centerOfMass);
            Yaw = frame.Yaw;
            Linear = linear;
            Angular = angular;
            MediumLinear = mediumLinear;
            Dt = dt;
            Time = time;
        }

        /// <summary>ν_r = ν − ν_medium (linear part; the medium does not rotate).</summary>
        public Vector3 RelativeLinear => Linear - MediumLinear;

        public bool IsFinite => Wrench.Finite(CenterOfMass) && Wrench.Finite(Linear) &&
                                Wrench.Finite(Angular) && Wrench.Finite(MediumLinear) &&
                                float.IsFinite(Dt) && Dt > 0f;
    }
}
