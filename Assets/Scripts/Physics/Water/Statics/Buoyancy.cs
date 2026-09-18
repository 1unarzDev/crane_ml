using UnityEngine;
using Sim.Utils.Performance;
using Sim.Physics.Processing;
using Sim.Utils;

namespace Sim.Physics.Water.Statics {
    [RequireComponent(typeof(Submersion))]
    public class Buoyancy : MonoBehaviour {
        public bool buoyancyForceActive = true;
        private Vector3 buoyancyCenter = new();
        private Submersion submersion;
        private IPhysicsBody body;

        void OnValidate() {
            if (GetComponent<Rigidbody>() == null && GetComponent<ArticulationBody>() == null)
                Debug.LogWarning($"{name} should have either a Rigidbody or an ArticulationBody attached.");
        }

        void Awake() {
            var rb = GetComponent<Rigidbody>();
            var ab = GetComponent<ArticulationBody>();

            if (rb != null) body = new RigidbodyAdapter(rb);
            else if (ab != null) body = new ArticulationBodyAdapter(ab);
            else throw new MissingComponentException($"{name} requires a Rigidbody or ArticulationBody!");
        }

        private void Start() {
            submersion = GetComponent<Submersion>();
        }


        private void FixedUpdate() {
            using var marker = CraneProfiler.WaterBuoyancy.Auto();
            if (!buoyancyForceActive) return;
            ApplyBuoyancyVolume();
        }


        private void ApplyBuoyancyVolume() {
            buoyancyCenter = submersion.submerged.data.centroid;
            float displacedVolume = submersion.submerged.data.volume;
            float buoyancyForce = Constants.waterDensity * Constants.gravity * displacedVolume;
            Vector3 forceVector = new(0f, buoyancyForce, 0f);
            body.AddForceAtPosition(forceVector, buoyancyCenter);
        }
    }
}
