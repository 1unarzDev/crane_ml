using NUnit.Framework;
using Sim.Physics.Water.Dynamics;
using UnityEngine;

namespace Sim.Tests.Physics {
    /// <summary>
    /// A side-on hit at 0.5 m/s. Against a fixed dock a rigid contact sets the rebound by
    /// restitution alone, Δv = (1 + e)·v0, whatever the mass, so added mass cannot show there (in
    /// Unity or in the impulse model with m + a22). It shows against a movable body: a 10 kg buoy
    /// takes Δv_boat = (1 + e)·M/(m_eff + M)·v0 with m_eff = m + a22 physically, but Unity uses m
    /// unless the contact mass is scaled. Both are measured with and without the scale.
    /// </summary>
    public sealed class DockBumpTests {
        private const float Mass = 30f, A22 = 15f, Izz = 4.1f, A66 = 1.23f, Speed = 0.5f, Buoy = 10f;

        /// <summary>(boat Δv, restitution e) of one side hit on a dock (fixed) or a free buoy.</summary>
        private static (float dv, float e) Bump(bool scaled, bool movable) {
            SimulationMode mode = UnityEngine.Physics.simulationMode;
            float threshold = UnityEngine.Physics.bounceThreshold;
            var boat = new GameObject("boat");
            var other = new GameObject("other");
            try {
                UnityEngine.Physics.simulationMode = SimulationMode.Script;
                UnityEngine.Physics.bounceThreshold = 0.1f;
                var material = new PhysicsMaterial { bounciness = 0.5f, bounceCombine = PhysicsMaterialCombine.Maximum,
                    dynamicFriction = 0, staticFriction = 0, frictionCombine = PhysicsMaterialCombine.Minimum };
                var hull = boat.AddComponent<BoxCollider>();
                hull.size = new Vector3(0.9f, 0.4f, 1.2f);  // beam along x, bow along +z
                hull.sharedMaterial = material;
                var body = boat.AddComponent<ArticulationBody>();
                body.useGravity = false;
                body.linearDamping = body.angularDamping = 0;
                body.mass = Mass;
                body.automaticInertiaTensor = false;
                body.inertiaTensor = new Vector3(2.2f, Izz, 2.0f);
                var wall = other.AddComponent<BoxCollider>();
                wall.size = new Vector3(0.5f, 2f, 4f);
                wall.sharedMaterial = material;
                other.transform.position = new Vector3(0.9f, 0, 0);  // 0.2 m gap to starboard (+x)
                Rigidbody buoy = null;
                if (movable) {
                    buoy = other.AddComponent<Rigidbody>();
                    buoy.mass = Buoy;
                    buoy.useGravity = false;
                    buoy.linearDamping = buoy.angularDamping = 0;
                    buoy.constraints = RigidbodyConstraints.FreezeRotation;
                }
                if (scaled) {
                    AddedMassContacts.Register(body, new Collider[] { hull }, Mass, Izz, 0f, A22, A66);
                    AddedMassContacts.SetHeading(body, Vector3.forward, Vector3.left);
                }
                UnityEngine.Physics.Simulate(0.02f);
                body.linearVelocity = new Vector3(Speed, 0, 0);
                for (int k = 0; k < 75; k++) UnityEngine.Physics.Simulate(0.02f);
                float boatAfter = body.linearVelocity.x, otherAfter = movable ? buoy.linearVelocity.x : 0f;
                return (Speed - boatAfter, (otherAfter - boatAfter) / Speed);
            }
            finally {
                AddedMassContacts.Unregister(boat.GetComponent<ArticulationBody>());
                UnityEngine.Physics.simulationMode = mode;
                UnityEngine.Physics.bounceThreshold = threshold;
                Object.DestroyImmediate(boat);
                Object.DestroyImmediate(other);
            }
        }

        [Test]
        public void SideHitWithAndWithoutContactMassScale() {
            string line = "PHYS_DOCK_BUMP";
            foreach (bool scaled in new[] { false, true }) {
                var (dv, e) = Bump(scaled, movable: false);
                float model = (1 + e) * Speed;
                line += $" dock scaled={scaled} dv={dv:F4} e={e:F3} dv_over_model={dv / model:F3};";
                Assert.That(dv / model, Is.EqualTo(1f).Within(0.05f), "a fixed dock sets Δv by restitution");
                var (dvBuoy, eBuoy) = Bump(scaled, movable: true);
                float modelBuoy = (1 + eBuoy) * Buoy / (Mass + A22 + Buoy) * Speed;
                float ratio = dvBuoy / modelBuoy;
                line += $" buoy scaled={scaled} dv={dvBuoy:F4} e={eBuoy:F3} dv_over_model={ratio:F3};";
                if (!scaled)
                    Assert.That(ratio, Is.EqualTo((Mass + A22 + Buoy) / (Mass + Buoy)).Within(0.05f),
                        "without the scale the boat's contact mass is M_RB only");
                Assert.That(float.IsFinite(ratio), Is.True);
            }
            UnityEngine.Debug.Log(line);
        }

        [Test]
        public void MassScaleFollowsTheNormal() {
            Assert.That(AddedMassContacts.MassScale(Mass, 1.5f, A22, Vector3.forward, Vector3.left, Vector3.left),
                Is.EqualTo(Mass / (Mass + A22)).Within(1e-6f));
            Assert.That(AddedMassContacts.MassScale(Mass, 1.5f, A22, Vector3.forward, Vector3.left, Vector3.forward),
                Is.EqualTo(Mass / (Mass + 1.5f)).Within(1e-6f));
            Assert.That(AddedMassContacts.MassScale(Mass, 1.5f, A22, Vector3.forward, Vector3.left, Vector3.up),
                Is.EqualTo(1f), "no added mass on vertical contacts");
        }
    }
}
