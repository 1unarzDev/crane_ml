using UnityEngine;
using Sim.Utils.Performance;

namespace Sim.Physics.Aerial {
    /// <summary>
    /// Four-rotor rigid-body dynamics. Normal motion is produced only by rotor forces,
    /// reaction torque, aerodynamic drag, gravity, and PhysX contact.
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    public sealed class MultirotorDynamics : MonoBehaviour, ICraneEpisodeResettable {
        [Header("Rotor locations: FL, FR, RR, RL")]
        [SerializeField] private Transform frontLeft;
        [SerializeField] private Transform frontRight;
        [SerializeField] private Transform rearRight;
        [SerializeField] private Transform rearLeft;

        [Header("Rotor model")]
        [SerializeField] private float maximumThrustPerRotor = 8f;
        [SerializeField] private float reactionTorquePerThrust = 0.015f;
        [SerializeField] private float motorTimeConstant = 0.06f;
        [SerializeField, Range(0f, 0.5f)] private float maximumAxisMix = 0.2f;

        [Header("Air model")]
        [SerializeField] private Vector3 linearDrag = new(0.18f, 0.25f, 0.18f);
        [SerializeField] private Vector3 quadraticDrag = new(0.08f, 0.12f, 0.08f);
        [SerializeField] private Vector3 angularDrag = new(0.025f, 0.025f, 0.025f);
        [SerializeField] private Vector3 windVelocity;
        [SerializeField] private Vector3 gustAmplitude;
        [SerializeField] private float gustFrequencyHz = 0.35f;

        private readonly float[] targetMotorSpeed = new float[4];
        private readonly float[] motorSpeed = new float[4];
        private readonly float[] rotorThrust = new float[4];
        private readonly float[] yawSigns = { 1f, -1f, 1f, -1f };
        private Transform[] rotors;
        private Rigidbody body;
        private Vector3 initialWindVelocity;
        private Vector3 initialGustAmplitude;
        public int ResetPriority => -40;

        public void CaptureEpisodeInitialState() {
            initialWindVelocity = windVelocity;
            initialGustAmplitude = gustAmplitude;
        }

        public void ResetEpisode(in CraneEpisodeResetContext context,
            CraneEpisodeResetPhase phase) {
            if (phase != CraneEpisodeResetPhase.BeforePhysics) return;
            ResetActuators();
            windVelocity = initialWindVelocity;
            gustAmplitude = initialGustAmplitude;
        }

        public float MaximumThrustPerRotor => maximumThrustPerRotor;
        public float AverageMotorSpeed =>
            (motorSpeed[0] + motorSpeed[1] + motorSpeed[2] + motorSpeed[3]) * 0.25f;
        public float TotalThrust => rotorThrust[0] + rotorThrust[1] + rotorThrust[2] + rotorThrust[3];
        public float HoverMotorSpeed => Mathf.Sqrt(body.mass * UnityEngine.Physics.gravity.magnitude /
            (4f * maximumThrustPerRotor));

        private void Awake() {
            body = GetComponent<Rigidbody>();
            ValidateRotors();
            CacheRotors();
        }

        public void Configure(Transform fl, Transform fr, Transform rr, Transform rl) {
            frontLeft = fl;
            frontRight = fr;
            rearRight = rr;
            rearLeft = rl;
            CacheRotors();
        }

        public void SetCommand(float collective, float roll, float pitch, float yaw) {
            float axisScale = maximumAxisMix;
            float r = Mathf.Clamp(roll, -1f, 1f) * axisScale;
            float p = Mathf.Clamp(pitch, -1f, 1f) * axisScale;
            float y = Mathf.Clamp(yaw, -1f, 1f) * axisScale;
            collective = Mathf.Clamp01(collective);
            targetMotorSpeed[0] = Mathf.Clamp01(collective - r - p + y);
            targetMotorSpeed[1] = Mathf.Clamp01(collective + r - p - y);
            targetMotorSpeed[2] = Mathf.Clamp01(collective + r + p + y);
            targetMotorSpeed[3] = Mathf.Clamp01(collective - r + p - y);
        }

        /// <summary>Applies normalized direct rotor commands in FL, FR, RR, RL order.</summary>
        public void SetMotorCommands(float frontLeftCommand, float frontRightCommand,
            float rearRightCommand, float rearLeftCommand) {
            targetMotorSpeed[0] = Mathf.Clamp01(frontLeftCommand);
            targetMotorSpeed[1] = Mathf.Clamp01(frontRightCommand);
            targetMotorSpeed[2] = Mathf.Clamp01(rearRightCommand);
            targetMotorSpeed[3] = Mathf.Clamp01(rearLeftCommand);
        }

        public void SetWind(Vector3 steadyWind, Vector3 gust) {
            windVelocity = steadyWind;
            gustAmplitude = gust;
        }

        private void FixedUpdate() {
            UpdateMotors();
            ApplyRotorForces();
            ApplyAerodynamics();
        }

        private void UpdateMotors() {
            float response = motorTimeConstant <= 0f ? 1f :
                1f - Mathf.Exp(-Time.fixedDeltaTime / motorTimeConstant);
            for (int i = 0; i < motorSpeed.Length; i++)
                motorSpeed[i] = Mathf.Lerp(motorSpeed[i], targetMotorSpeed[i], response);
        }

        private void ApplyRotorForces() {
            Vector3 up = transform.up;
            for (int i = 0; i < rotors.Length; i++) {
                float thrust = maximumThrustPerRotor * motorSpeed[i] * motorSpeed[i];
                rotorThrust[i] = thrust;
                body.AddForceAtPosition(up * thrust, rotors[i].position, ForceMode.Force);
                body.AddTorque(up * (yawSigns[i] * thrust * reactionTorquePerThrust),
                    ForceMode.Force);
            }
        }

        private void ApplyAerodynamics() {
            float phase = Time.fixedTime * gustFrequencyHz * Mathf.PI * 2f;
            Vector3 gust = gustAmplitude * Mathf.Sin(phase);
            Vector3 relativeWorldVelocity = body.linearVelocity - windVelocity - gust;
            Vector3 relativeLocalVelocity = transform.InverseTransformDirection(relativeWorldVelocity);
            Vector3 dragLocal = new(
                DragAxis(relativeLocalVelocity.x, linearDrag.x, quadraticDrag.x),
                DragAxis(relativeLocalVelocity.y, linearDrag.y, quadraticDrag.y),
                DragAxis(relativeLocalVelocity.z, linearDrag.z, quadraticDrag.z));
            body.AddForce(transform.TransformDirection(dragLocal), ForceMode.Force);

            Vector3 localAngularVelocity = transform.InverseTransformDirection(body.angularVelocity);
            Vector3 angularTorque = -Vector3.Scale(localAngularVelocity, angularDrag);
            body.AddTorque(transform.TransformDirection(angularTorque), ForceMode.Force);
        }

        private static float DragAxis(float speed, float linear, float quadratic) {
            return -linear * speed - quadratic * speed * Mathf.Abs(speed);
        }

        public void ResetActuators(float initialMotorSpeed = 0f) {
            initialMotorSpeed = Mathf.Clamp01(initialMotorSpeed);
            for (int i = 0; i < motorSpeed.Length; i++) {
                targetMotorSpeed[i] = initialMotorSpeed;
                motorSpeed[i] = initialMotorSpeed;
                rotorThrust[i] = maximumThrustPerRotor * initialMotorSpeed * initialMotorSpeed;
            }
        }

        private void ValidateRotors() {
            if (frontLeft == null || frontRight == null || rearRight == null || rearLeft == null)
                throw new MissingReferenceException($"{name} requires four rotor transforms.");
        }

        private void CacheRotors() {
            rotors = new[] { frontLeft, frontRight, rearRight, rearLeft };
        }
    }
}
