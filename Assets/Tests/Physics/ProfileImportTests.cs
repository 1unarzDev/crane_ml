using System.IO;
using NUnit.Framework;
using Sim.Physics.Core;
using Sim.Physics.Water.Boat;
using Sim.Physics.Water.Boat.Editor;
using Sim.Physics.Water.Dynamics;
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
        public void CharacterizedAssetEqualsIdealUntilDataPassesTheGate() {
            var asset = Resources.Load<MarineSurfaceProfile>(ProfileBootstrap.AssetName("characterized"));
            Assert.That(asset, Is.Not.Null, "Assets/Config/Boats/Resources/Characterized.asset");
            Assert.That(Physical(asset.Parameters), Is.EqualTo(Physical(IdealJson())), "no field data has passed the gate");
            Assert.That(asset.Provenance.kind, Is.EqualTo("characterized"));
            Assert.That(asset.Provenance.summary, Does.StartWith("prior").And.Contain("SIM_DERIVED"));
            Assert.That(asset.Provenance.parameters.Find(p => p.name == "surge+").reason, Is.Not.Empty,
                "the rejection reasons are recorded per parameter");
        }

        /// <summary>End to end: a characterization-v2 survey fixture went through the adapter and the
        /// gate into profiles/fixture-survey.json (test_mhseals_physics.py keeps it current); here it is
        /// imported into an asset, selected the way a run selects it, validated, and the plant model it
        /// builds reaches the steady speeds of the imported D.</summary>
        [Test]
        public void SurveyFixtureImportsAndThePlantUsesIt() {
            const string json = "Tools/Characterization/profiles/fixture-survey.json";
            const string assetPath = "Assets/Config/Boats/FixtureSurvey.test.asset";
            try {
                MarineSurfaceProfile asset = CraneBoatProfileImport.Import(json, assetPath);
                Assert.That(asset.Provenance.summary, Does.StartWith("characterized"));
                MarineSurfaceProfile run = ProfileBootstrap.Load<MarineSurfaceProfile>(
                    new[] { ProfileBootstrap.ProfileFlag, "characterized", ProfileBootstrap.JsonFlag, json });
                Assert.That(Physical(run.Parameters), Is.EqualTo(Physical(asset.Parameters)));
                Assert.That(ProfileBootstrap.AppliedLine(run), Does.Contain("profile=FixtureSurvey").And.Contain("status=characterized"));
                MarineSurfaceParameters p = run.Parameters;
                Assert.That(p.Validate(p.fixedDeltaTime).Ok, Is.True);
                Assert.That(p.surge.d1Pos, Is.Not.EqualTo(IdealJson().surge.d1Pos), "the imported D, not the prior");
                MarinePlanarModel m = p.PlanarModel();
                var thrust = new Planar(Kit.FullAxisThrust, 0, 0);
                double got = Kit.Run(m, default, _ => thrust, p.fixedDeltaTime, 60).Body[0];
                Assert.That(got, Is.EqualTo(MarinePlanarModel.SteadySpeed(p.surge.d1Pos, p.surge.d2Pos, Kit.FullAxisThrust))
                    .Within(0.01).Percent);
                UnityEngine.Object.DestroyImmediate(run);
            }
            finally {
                UnityEditor.AssetDatabase.DeleteAsset(assetPath);
            }
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
