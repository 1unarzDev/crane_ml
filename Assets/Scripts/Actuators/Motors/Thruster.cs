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

        /// <summary>True when a plant owns this thruster through a <see cref="ThrusterActuator"/>:
        /// the thruster then keeps its command and rotor but applies no force itself.</summary>
        public bool PlantDriven { get; set; }

        protected override void Awake() {
            base.Awake();

            thrustK = config.thrustK;
            backK = config.backK;
            height = config.height;
            // Legacy path only (the plant applies thrust on the hull itself): the scene as authored.
            var parentRb = transform.root.GetComponent<Rigidbody>();
            var parentAb = transform.root.GetComponent<ArticulationBody>();
            if (parentRb != null) rootBody = new RigidbodyAdapter(parentRb);
            else if (parentAb != null) rootBody = new ArticulationBodyAdapter(parentAb);
            else rootBody = body;
        }

        /// <summary>The ESC pulse [µs] equivalent of the command: ±max maps onto 1500 ± 400, the
        /// inverse of <see cref="EscPwm.Bidirectional"/> with no deadband.</summary>
        public float CommandPulse => ThrusterModel.Neutral + ThrusterModel.HalfRange * command / GetMaxCommand();

        public void SetPulse(float pulse) =>
            SetCommand((pulse - ThrusterModel.Neutral) / ThrusterModel.HalfRange * GetMaxCommand());

        /// <summary>Submerged fraction of the rotor, clamp01(depth / fullDepth).</summary>
        public float SubmersionFraction(float fullDepth) {
            if (waterSurface == null) return 1f;
            float waterHeight = WaterUtils.Search(waterSurface, body.transform.position).projectedPositionWS.y;
            return Mathf.Clamp01((waterHeight - body.transform.position.y) / fullDepth);
        }

        protected override void FixedUpdate() {
            using var marker = CraneProfiler.Thruster.Auto();
            base.FixedUpdate();
            if (PlantDriven) return;

            float submersionFraction = SubmersionFraction(height);
            float velocity = base.GetVelocity();
            Vector3 forceDirection = rootBody.transform.TransformDirection(transform.localRotation * localAxis);
            Vector3 force = Mathf.Sign(velocity) * (velocity * velocity) * thrustK * forceDirection;
            force *= Mathf.Sign(velocity) < 0 ? backK : 1;
            force *= submersionFraction;
            rootBody.AddForceAtPosition(force, body.transform.position);

            if (debug) Debug.DrawLine(body.position, body.position - force * 0.01f, Color.red, Time.fixedDeltaTime);
        }

        public float GetMaxThrust() => config.GetMaxThrust();
        public float GetMaxBackThrust() => config.GetMaxBackThrust();
    }
}
