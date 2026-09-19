using UnityEngine;
using Sim.Utils.Performance;

namespace Sim.Physics.Land {
    /// <summary>
    /// Force-driven planar differential base. Commands are body linear/angular velocities;
    /// movement remains subject to PhysX mass, contact, friction, and canonical colliders.
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    public sealed class DifferentialDriveDynamics : MonoBehaviour, ICraneEpisodeResettable {
        [SerializeField] private float wheelSeparation = 0.287f;
        [SerializeField] private float wheelRadius = 0.033f;
        [SerializeField] private float maximumLinearSpeed = 0.26f;
        [SerializeField] private float maximumAngularSpeed = 1.82f;
        [SerializeField] private float maximumLinearAcceleration = 10.0f;
        [SerializeField] private float maximumAngularAcceleration = 6.0f;

        private Rigidbody body;
        private float commandedLinear;
        private float commandedAngular;

        public int ResetPriority => -40;
        public float WheelSeparation => wheelSeparation;
        public float WheelRadius => wheelRadius;
        public float MaximumLinearSpeed => maximumLinearSpeed;
        public float MaximumAngularSpeed => maximumAngularSpeed;

        private void Awake() => body = GetComponent<Rigidbody>();

        public void Configure(float separation, float radius, float linearSpeed,
            float angularSpeed) {
            wheelSeparation = Mathf.Max(0.01f, separation);
            wheelRadius = Mathf.Max(0.001f, radius);
            maximumLinearSpeed = Mathf.Max(0.01f, linearSpeed);
            maximumAngularSpeed = Mathf.Max(0.01f, angularSpeed);
            maximumLinearAcceleration = 10.0f;
            maximumAngularAcceleration = 6.0f;
        }

        public void SetCommand(float linear, float angular) {
            commandedLinear = Mathf.Clamp(linear, -maximumLinearSpeed, maximumLinearSpeed);
            commandedAngular = Mathf.Clamp(angular, -maximumAngularSpeed, maximumAngularSpeed);
        }

        private void FixedUpdate() {
            Vector3 localVelocity = transform.InverseTransformDirection(body.linearVelocity);
            float linearError = commandedLinear - localVelocity.z;
            float linearDelta = Mathf.Clamp(linearError,
                -maximumLinearAcceleration * Time.fixedDeltaTime,
                maximumLinearAcceleration * Time.fixedDeltaTime);
            body.AddForce(transform.forward * linearDelta, ForceMode.VelocityChange);
            float angularError = commandedAngular - body.angularVelocity.y;
            float angularDelta = Mathf.Clamp(angularError,
                -maximumAngularAcceleration * Time.fixedDeltaTime,
                maximumAngularAcceleration * Time.fixedDeltaTime);
            body.AddTorque(Vector3.up * angularDelta, ForceMode.VelocityChange);
            // The physical platform is non-holonomic; cancel lateral numerical drift without
            // changing forward or vertical dynamics.
            float lateralDelta = Mathf.Clamp(-localVelocity.x,
                -maximumLinearAcceleration * Time.fixedDeltaTime,
                maximumLinearAcceleration * Time.fixedDeltaTime);
            body.AddForce(transform.right * lateralDelta, ForceMode.VelocityChange);
        }

        public void CaptureEpisodeInitialState() { }

        public void ResetEpisode(in CraneEpisodeResetContext context,
            CraneEpisodeResetPhase phase) {
            if (phase == CraneEpisodeResetPhase.BeforePhysics) SetCommand(0f, 0f);
        }
    }
}
