using UnityEngine;
using Sim.Utils.Performance;
using Sim.Physics.Core;
using Sim.Physics.Processing;
using Sim.Utils;

namespace Sim.Physics.Water.Statics {
    /// <summary>
    /// Hydrostatic buoyancy ρ g V up through the submerged centroid, from <see cref="Submersion"/>
    /// (wave-aware). Standalone it applies itself; with <see cref="PlantDriven"/> a plant calls it as
    /// an <see cref="IForceModel"/> routed Hydro: its body-frame planar part (X, Y and N from wave
    /// slope) then sees the added mass, while heave, roll and pitch pass through to Unity.
    /// </summary>
    [RequireComponent(typeof(Submersion))]
    public class Buoyancy : MonoBehaviour, IForceModel {
        public bool buoyancyForceActive = true;
        private Vector3 buoyancyCenter = new();
        private Submersion submersion;
        private IPhysicsBody body;

        public bool PlantDriven { get; set; }
        public WrenchRoute Route => WrenchRoute.Hydro;

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
            if (!buoyancyForceActive || PlantDriven) return;
            ApplyBuoyancyVolume();
        }

        /// <summary>World force [N, Unity] through <paramref name="centre"/> (Unity world).</summary>
        private Vector3 Force(out Vector3 centre) {
            centre = submersion.submerged.data.centroid;
            float displacedVolume = submersion.submerged.data.volume;
            return new Vector3(0f, Constants.waterDensity * Constants.gravity * displacedVolume, 0f);
        }

        private void ApplyBuoyancyVolume() {
            Vector3 forceVector = Force(out buoyancyCenter);
            body.AddForceAtPosition(forceVector, buoyancyCenter);
        }

        public void Accumulate(in PlantState state, ref Wrench wrench) {
            if (!buoyancyForceActive || submersion == null || submersion.submerged == null) return;
            Vector3 force = Force(out buoyancyCenter);
            wrench.AddAt(state.Frame.BodyLinear(force), state.Frame.BodyLinear(buoyancyCenter - state.CenterOfMass));
        }
    }
}
