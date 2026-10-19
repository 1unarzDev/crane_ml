// EXPLORATORY actual physics body access, never an independent-motion sensor.
using System;
using UnityEngine;

namespace Crane.MainVenueV8 {
    public sealed class PhysicalBodyV8 {
        public Rigidbody rigid { get; private set; }
        public ArticulationBody articulation { get; private set; }
        public Component component => rigid != null ? (Component)rigid : articulation;
        public Transform transform => component.transform;
        public Vector3 position => rigid != null ? rigid.position : articulation.transform.position;
        public Quaternion rotation => rigid != null ? rigid.rotation : articulation.transform.rotation;
        public Vector3 linearVelocity => rigid != null ? rigid.linearVelocity : articulation.linearVelocity;
        // An articulation link's mass is not the assembly mass. Until a contact
        // sensor model is qualified, withhold that entire channel for ASV bodies.
        public bool contactMassQualified => rigid != null;
        public float contactMass {
            get { if(rigid==null)throw new InvalidOperationException("Articulation contact mass unqualified"); return rigid.mass; }
        }
        public static PhysicalBodyV8 Resolve(GameObject owner) {
            var rigid=owner.GetComponent<Rigidbody>();var articulation=owner.GetComponent<ArticulationBody>();
            if((rigid==null)==(articulation==null))throw new ArgumentException("Exactly one physical body binding required");
            if(rigid!=null&&rigid.isKinematic)throw new ArgumentException("Dynamic Rigidbody unavailable");
            if(articulation!=null&&(!articulation.isRoot||articulation.immovable))throw new ArgumentException("Movable root ArticulationBody required");
            return new PhysicalBodyV8 {rigid=rigid,articulation=articulation};
        }
        public void AddForce(Vector3 force,ForceMode mode=ForceMode.Force) {
            if(rigid!=null)rigid.AddForce(force,mode);else articulation.AddForce(force,mode);
        }
    }
}
