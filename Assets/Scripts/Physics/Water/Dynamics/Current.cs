using UnityEngine.Rendering.HighDefinition;
using UnityEngine;
using Sim.Utils.Performance;
using Sim.Physics.Processing;
using Sim.Utils;

namespace Sim.Physics.Water.Dynamics {
    [RequireComponent(typeof(Submersion))]
    public class Current : MonoBehaviour {
        [SerializeField] private WaterSurface waterSurface;
        [SerializeField] private bool debugCurrent = false;
        [SerializeField] private float Cd = 1.0f;
        private Submerged submerged;
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

        private void Start() { submerged = GetComponent<Submersion>().submerged; }

        private void FixedUpdate() {
            using var marker = CraneProfiler.VehicleDynamics.Auto();
            using var componentMarker = CraneProfiler.VehicleDynamicsCurrent.Auto();
            ApplyCurrent();
        }

        private void ApplyCurrent() {
            if (submerged.data == null) return;

            Vector3 bodyVel = body.linearVelocity;
            Vector3 bodyOmega = body.angularVelocity;
            Vector3 bodyPos = body.position;
            int faceCount = submerged.data.maxTriangleIndex / 3;
            if (faceCount == 0) return;

            // HDRP only rotates the spectrum's base current direction by position when a
            // large- or ripple-current map is active. Avoid repeating the full iterative water
            // projection for every face when both spatial current features are disabled.
            bool hasSpatialCurrent = waterSurface.supportLargeCurrent || waterSurface.supportRipplesCurrent;
            Vector3 uniformWaterVel = hasSpatialCurrent
                ? default
                : GetCurrentAtPoint(submerged.data.faceCentersWorld[0]);

            for (int i = 0; i < faceCount; i++) {
                Vector3 faceCenter = submerged.data.faceCentersWorld[i];
                Vector3 pointVel = bodyVel + Vector3.Cross(bodyOmega, faceCenter - bodyPos);
                Vector3 waterVel = hasSpatialCurrent ? GetCurrentAtPoint(faceCenter) : uniformWaterVel;
                Vector3 relVel = pointVel - waterVel;

                float rho = Constants.waterDensity;
                float faceArea = submerged.data.triangleAreas[i];

                Vector3 force = -0.5f * rho * Cd * faceArea * relVel.magnitude * relVel;
                force.y = 0.0f;

                body.AddForceAtPosition(force, faceCenter);

                if (debugCurrent) Debug.DrawRay(faceCenter, force, Color.cyan);
            }
        }

        private Vector3 GetCurrentAtPoint(Vector3 point) { return WaterUtils.Search(waterSurface, point).currentDirectionWS * waterSurface.largeCurrentSpeedValue; }
    }
}
