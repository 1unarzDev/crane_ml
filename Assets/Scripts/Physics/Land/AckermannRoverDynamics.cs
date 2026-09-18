using System;
using UnityEngine;
using Sim.Utils.Performance;

namespace Sim.Physics.Land {
    /// <summary>
    /// Four-wheel Ackermann rover backed by PhysX WheelColliders. Commands are normalized;
    /// ordinary motion is produced only through wheel torque, steering, braking, and contact.
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    public sealed class AckermannRoverDynamics : MonoBehaviour, ICraneEpisodeResettable {
        [Header("Wheel references")]
        [SerializeField] private WheelCollider frontLeft;
        [SerializeField] private WheelCollider frontRight;
        [SerializeField] private WheelCollider rearLeft;
        [SerializeField] private WheelCollider rearRight;

        [Header("Geometry and limits")]
        [SerializeField] private float wheelbase = 1.6f;
        [SerializeField] private float trackWidth = 1.2f;
        [SerializeField] private float maximumSteeringAngle = 32f;
        [SerializeField] private float maximumMotorTorque = 6f;
        [SerializeField] private float maximumBrakeTorque = 650f;
        [SerializeField] private float maximumSpeed = 12f;

        [Header("Actuator and resistance")]
        [SerializeField] private float motorTimeConstant = 0.18f;
        [SerializeField] private float steeringRateDegreesPerSecond = 90f;
        [SerializeField, Range(0f, 0.1f)] private float rollingResistanceCoefficient = 0.015f;

        private Rigidbody body;
        private float throttleCommand;
        private float steeringCommand;
        private float brakeCommand;
        private float appliedTorque;
        private float leftSteeringAngle;
        private float rightSteeringAngle;
        private WheelCollider[] wheels;
        public int ResetPriority => -40;
        public void CaptureEpisodeInitialState() { }
        public void ResetEpisode(in CraneEpisodeResetContext context,
            CraneEpisodeResetPhase phase) {
            if (phase == CraneEpisodeResetPhase.BeforePhysics) ResetActuators();
        }

        public float Speed => Vector3.Dot(body.linearVelocity, transform.forward);
        public float ThrottleCommand => throttleCommand;
        public float SteeringCommand => steeringCommand;
        public float BrakeCommand => brakeCommand;
        public float AppliedWheelTorque => appliedTorque;
        public float LeftSteeringAngle => leftSteeringAngle;
        public float RightSteeringAngle => rightSteeringAngle;
        public float Wheelbase => wheelbase;
        public float TrackWidth => trackWidth;
        public float MaximumSteeringAngle => maximumSteeringAngle;
        public int GroundedWheelCount {
            get {
                int count = 0;
                foreach (WheelCollider wheel in wheels)
                    if (wheel.isGrounded) count++;
                return count;
            }
        }
        public float AverageWheelRpm {
            get {
                float rpm = 0f;
                foreach (WheelCollider wheel in wheels) rpm += Mathf.Abs(wheel.rpm);
                return rpm / wheels.Length;
            }
        }

        private void Awake() {
            body = GetComponent<Rigidbody>();
            ValidateWheels();
            CacheWheels();
        }

        public void Configure(WheelCollider fl, WheelCollider fr, WheelCollider rl,
            WheelCollider rr, float configuredWheelbase, float configuredTrackWidth) {
            frontLeft = fl;
            frontRight = fr;
            rearLeft = rl;
            rearRight = rr;
            wheelbase = configuredWheelbase;
            trackWidth = configuredTrackWidth;
            CacheWheels();
        }

        public void SetCommand(float throttle, float steering, float brake) {
            throttleCommand = Mathf.Clamp(throttle, -1f, 1f);
            steeringCommand = Mathf.Clamp(steering, -1f, 1f);
            brakeCommand = Mathf.Clamp01(brake);
        }

        private void FixedUpdate() {
            UpdateSteering();
            UpdateMotor();
            ApplyWheelCommands();
        }

        private void UpdateSteering() {
            float targetLeft = 0f;
            float targetRight = 0f;
            float targetCenter = steeringCommand * maximumSteeringAngle;
            if (Mathf.Abs(targetCenter) > 0.01f) {
                float sign = Mathf.Sign(targetCenter);
                float centerRadians = Mathf.Abs(targetCenter) * Mathf.Deg2Rad;
                float centerRadius = wheelbase / Mathf.Tan(centerRadians);
                float inner = Mathf.Atan(wheelbase / Mathf.Max(0.05f,
                    centerRadius - trackWidth * 0.5f)) * Mathf.Rad2Deg;
                float outer = Mathf.Atan(wheelbase / (centerRadius + trackWidth * 0.5f)) * Mathf.Rad2Deg;
                if (sign > 0) { targetLeft = inner; targetRight = outer; }
                else { targetLeft = -outer; targetRight = -inner; }
            }
            float maxChange = steeringRateDegreesPerSecond * Time.fixedDeltaTime;
            leftSteeringAngle = Mathf.MoveTowards(leftSteeringAngle, targetLeft, maxChange);
            rightSteeringAngle = Mathf.MoveTowards(rightSteeringAngle, targetRight, maxChange);
        }

        private void UpdateMotor() {
            float targetTorque = throttleCommand * maximumMotorTorque;
            if (Mathf.Abs(Speed) >= maximumSpeed && Mathf.Sign(targetTorque) == Mathf.Sign(Speed))
                targetTorque = 0f;
            float response = motorTimeConstant <= 0f ? 1f :
                1f - Mathf.Exp(-Time.fixedDeltaTime / motorTimeConstant);
            appliedTorque = Mathf.Lerp(appliedTorque, targetTorque, response);
        }

        private void ApplyWheelCommands() {
            frontLeft.steerAngle = leftSteeringAngle;
            frontRight.steerAngle = rightSteeringAngle;
            float rollingTorque = rollingResistanceCoefficient * body.mass *
                UnityEngine.Physics.gravity.magnitude * frontLeft.radius / wheels.Length;
            float brakeTorque = brakeCommand * maximumBrakeTorque;
            if (Mathf.Abs(throttleCommand) < 0.001f && brakeCommand <= 0f)
                brakeTorque = rollingTorque;
            foreach (WheelCollider wheel in wheels) {
                wheel.motorTorque = appliedTorque;
                wheel.brakeTorque = brakeTorque;
            }
        }

        public float ExpectedCenterTurnRadius(float normalizedSteering) {
            float angle = Mathf.Abs(normalizedSteering) * maximumSteeringAngle * Mathf.Deg2Rad;
            return angle <= 0.0001f ? float.PositiveInfinity : wheelbase / Mathf.Tan(angle);
        }

        public void ResetActuators() {
            throttleCommand = steeringCommand = brakeCommand = appliedTorque = 0f;
            leftSteeringAngle = rightSteeringAngle = 0f;
            ApplyWheelCommands();
        }

        private void ValidateWheels() {
            if (frontLeft == null || frontRight == null || rearLeft == null || rearRight == null)
                throw new MissingReferenceException($"{name} requires four WheelColliders.");
        }

        private void CacheWheels() {
            wheels = new[] { frontLeft, frontRight, rearLeft, rearRight };
        }
    }
}
