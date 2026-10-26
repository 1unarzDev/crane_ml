using System.IO;
using NUnit.Framework;
using Sim.Physics.Core;
using Sim.Physics.Water.Boat;
using Sim.Physics.Water.Boat.Editor;
using UnityEngine;

namespace Sim.Tests.Physics {
    public sealed class ProfileImportTests {
        private static string Physical(MarineSurfaceParameters p) {
            MarineSurfaceParameters copy = p.Clone();
            copy.provenance = new Provenance();
            copy.name = "";
            return JsonUtility.ToJson(copy);
        }

        private static MarineSurfaceParameters IdealJson() =>
            JsonUtility.FromJson<MarineSurfaceParameters>(File.ReadAllText(CraneBoatProfileImport.DefaultJson));

        [Test]
        public void CodeDefaultsEqualTheIdealJson() {
            Assert.That(Physical(IdealJson()), Is.EqualTo(Physical(Kit.Ideal())));
            Assert.That(IdealJson().provenance.parameters, Is.Not.Empty, "every Ideal parameter carries its prior status");
        }

        [Test]
        public void IdealAssetLoadsFromResourcesAndMatches() {
            var asset = Resources.Load<MarineSurfaceProfile>(ProfileBootstrap.DefaultProfile);
            Assert.That(asset, Is.Not.Null, "Assets/Config/Boats/Resources/Ideal.asset");
            Assert.That(Physical(asset.Parameters), Is.EqualTo(Physical(IdealJson())));
            Assert.That(asset.Provenance.kind, Is.EqualTo("ideal"));
            Assert.That(ProfileBootstrap.AssetName("characterized"), Is.EqualTo("Characterized"));
            Assert.That(ProfileBootstrap.AssetName(null), Is.EqualTo("Ideal"));
        }

        [Test]
        public void JsonOverrideRoundTripsAndHashesStably() {
            var profile = ScriptableObject.CreateInstance<MarineSurfaceProfile>();
            try {
                string json = profile.ToJson();
                profile.FromJson(json);
                Assert.That(profile.ToJson(), Is.EqualTo(json));
                Assert.That(ProfileBootstrap.AppliedLine(profile), Does.StartWith("CRANE_PLANT_APPLIED profile=Ideal sha256="));
            }
            finally {
                Object.DestroyImmediate(profile);
            }
        }

        [Test]
        public void ErrorLayerIsKeyedByNameAndLeavesTheProfileAlone() {
            MarineSurfaceParameters nominal = Kit.Ideal();
            string before = JsonUtility.ToJson(nominal);
            ActuatorErrorProfile errors = ActuatorErrorProfile.Parse(
                "{\"strengths\":[1,0.5,1,1],\"angles\":[0,0,10,0],\"reverse\":[1,1,1,0.5],\"deadtime_ticks\":2,\"mass\":40}");
            MarineSurfaceParameters p = errors.Apply(nominal, 0.02f);
            Assert.That(JsonUtility.ToJson(nominal), Is.EqualTo(before), "an error profile never changes nominal parameters");
            Assert.That(p.thrusters.Find(t => t.name == "front_right").kForward, Is.EqualTo(25.75f));
            Assert.That(p.thrusters.Find(t => t.name == "front_left").kForward, Is.EqualTo(51.5f));
            Assert.That(p.thrusters.Find(t => t.name == "rear_right").alphaDeg, Is.EqualTo(-55f),
                "+10° Unity (clockwise) is −10° FLU");
            Assert.That(p.thrusters.Find(t => t.name == "rear_left").kReverse, Is.EqualTo(25.75f));
            Assert.That(p.thrusters.TrueForAll(t => Mathf.Approximately(t.deadTime, 0.04f)), Is.True);
            Assert.That(p.mass, Is.EqualTo(40f));
            Assert.That(p.izz, Is.EqualTo(nominal.izz), "absent fields keep the profile");
            Assert.That(errors.DragModel(p.PlanarModel()), Is.Null);
            Assert.Throws<System.ArgumentException>(() => ActuatorErrorProfile.Parse("{\"strengths\":[1,1,1]}"));
            Assert.Throws<System.ArgumentException>(() => ActuatorErrorProfile.Parse("{\"mass\":200}"));
        }

        [Test]
        public void MassLayoutScalesLinksToTheTotal() {
            float[] masses = RoboBoatPhysics.ScaledMasses(new[] { 3f, 8f, 8f, 1f }, 30f);
            Assert.That(masses[0] + masses[1] + masses[2] + masses[3], Is.EqualTo(30f).Within(1e-4f));
            Assert.That(masses[1] / masses[0], Is.EqualTo(8f / 3f).Within(1e-4f));
        }

        [Test]
        public void DisturbanceWaitsThenRampsIn() {
            RoboBoatPhysics.DisturbanceAfter = 30f;
            try {
                Assert.That(RoboBoatPhysics.Disturbance(20f), Is.EqualTo(0f));
                Assert.That(RoboBoatPhysics.Disturbance(35f), Is.EqualTo(0.5f).Within(1e-6f));
                Assert.That(RoboBoatPhysics.Disturbance(60f), Is.EqualTo(1f));
            }
            finally {
                RoboBoatPhysics.DisturbanceAfter = -10f;
            }
        }
    }
}
