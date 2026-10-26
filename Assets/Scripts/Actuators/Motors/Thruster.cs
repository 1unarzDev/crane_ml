using System;
using UnityEngine;
using Sim.Utils;
using UnityEngine.Rendering.HighDefinition;
using Sim.Physics.Processing;
using Sim.Utils.Performance;

namespace Sim.Actuators.Motors {
    public class Thruster : MotorBase<ThrusterConfig> {
        [SerializeField] private WaterSurface waterSurface; // Can't put GameObjects into ScriptableObjects
        [SerializeField] private bool debug;
        private IPhysicsBody rootBody;
        private float thrustK, backK, height;
        // With a measured thrust curve the thrust follows the commanded speed through a
        // first-order spin-up, instead of the simulated rotor joint's speed.
        private bool thrustFromCommand;
        private float spinUpTime, spool, thrustYawDegrees;

        protected override void Awake() {
            base.Awake();

            thrustK = config.thrustK;
            backK = config.backK;
            height = config.height;

            var parentRb = transform.root.GetComponent<Rigidbody>();
            var parentAb = transform.root.GetComponent<ArticulationBody>();

            if (parentRb != null) rootBody = new RigidbodyAdapter(parentRb);
            else if (parentAb != null) rootBody = new ArticulationBodyAdapter(parentAb);
            else rootBody = body;
        }

        protected override void FixedUpdate() {
            using var marker = CraneProfiler.Thruster.Auto();
            base.FixedUpdate();

            float waterHeight = WaterUtils.Search(waterSurface, body.transform.position).projectedPositionWS.y;
            float depth = waterHeight - body.transform.position.y; // positive if below water
            float submersionFraction = Mathf.Clamp01(depth / height);

            float velocity = base.GetVelocity();
            if (thrustFromCommand) {
                spool += (command - spool) * Mathf.Clamp01(Time.fixedDeltaTime / spinUpTime);
                velocity = spool;
            }
            Vector3 forceDirection = rootBody.transform.TransformDirection(transform.localRotation * localAxis);
            if (thrustYawDegrees != 0)
                // The adapter may point to the rotor, whose up axis spins with it.
                // A marine mounting yaw error is about the water's vertical.
                forceDirection = Quaternion.AngleAxis(thrustYawDegrees, Vector3.up) * forceDirection;
            Vector3 force = Mathf.Sign(velocity) * (velocity * velocity) * thrustK * forceDirection;
            force *= Mathf.Sign(velocity) < 0 ? backK : 1;
            force *= submersionFraction;
            rootBody.AddForceAtPosition(force, body.transform.position);

            if (debug) Debug.DrawLine(body.position, body.position - force * 0.01f, Color.red, Time.fixedDeltaTime);
        }

        /// <summary>Replace the thrust curve with a measured one: full forward and full
        /// reverse thrust in newtons at full command, reached with a first-order spin-up
        /// of ``spinUp`` seconds. Thrust then goes with the square of the command.</summary>
        public void SetThrustCurve(float forwardNewtons, float reverseNewtons, float spinUp,
            float yawDegrees = 0) {
            float speed = config.GetMaxCommand();
            thrustK = Mathf.Sign(thrustK == 0 ? config.thrustK : thrustK) * forwardNewtons / (speed * speed);
            backK = forwardNewtons > 0 ? reverseNewtons / forwardNewtons : 1;  // a dead thruster: no thrust either way
            spinUpTime = Mathf.Max(spinUp, Time.fixedDeltaTime);
            thrustYawDegrees = yawDegrees;
            thrustFromCommand = true;
        }

        public float GetMaxThrust() => config.GetMaxThrust();
        public float GetMaxBackThrust() => config.GetMaxBackThrust();
    }
}
