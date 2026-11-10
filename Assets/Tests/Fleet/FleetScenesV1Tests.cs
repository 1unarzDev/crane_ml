using System.IO;
using System.Linq;
using Crane.Fleet.Perception;
using NUnit.Framework;

namespace Crane.Fleet.Tests {
    /// <summary>Scenes v1 compiled scenarios (Tools/Fleet/scenes/crane_out, from scenes.crane_compile --library): the v0 loaders
    /// accept them unchanged and the x_v1 extension parses. Written without a Unity run (scenes v1 handoff): the CRANE integration
    /// agent compiles and runs these first.</summary>
    public class FleetScenesV1Tests {
        const string SpecDir = "Assets/StreamingAssets/Fleet";
        const string Out = "Tools/Fleet/scenes/crane_out/scenarios";

        static string[] Compiled() => Directory.GetFiles(Out, "*.json").OrderBy(p => p).ToArray();

        [Test]
        public void EveryCompiledScenarioLoadsThroughFleetSpecAndThePerceptionScenario() {
            var files = Compiled();
            Assert.GreaterOrEqual(files.Length, 9, "one compiled scene per family at least");
            foreach (var f in files) {
                var spec = FleetSpec.LoadDirectory(SpecDir, Path.GetFullPath(f));
                Assert.IsNotNull(spec.Floor(), f);
                var sc = Scenario.Parse(File.ReadAllText(f));
                int obstacles = spec.S["obstacles"].Items.Count;
                Assert.AreEqual(4 + obstacles, sc.Boxes.Count, f + ": outer walls + obstacles (walls are lowered into obstacles)");
                Assert.IsTrue(spec.S.Has("zones") && spec.S["zones"].Has("delivery"), f + ": zones.delivery (AutonomyMission's truth check)");
            }
        }

        [Test]
        public void ExtensionBlockParsesDoorsThresholdsPatchesAndTags() {
            foreach (var f in Compiled()) {
                var root = JsonNode.FromText(File.ReadAllText(f));
                var v1 = ScenarioV1.From(root);
                Assert.IsTrue(v1.Present, f);
                Assert.AreEqual(root["x_v1.doors"].Items.Count, v1.Doors.Count, f);
                Assert.AreEqual(root["x_v1.thresholds"].Items.Count, v1.Thresholds.Count, f);
                Assert.AreEqual(root["x_v1.floor_patches"].Items.Count, v1.FloorPatches.Count, f);
                Assert.AreEqual(root["x_v1.tags"].Items.Count, v1.Tags.Count, f);
                foreach (var t in v1.Thresholds) { Assert.IsTrue(t.Traversable); Assert.Greater(t.Sz, 0); }
                foreach (var d in v1.Doors) Assert.Greater(d.Width, 0.3, f + " " + d.Id);
            }
        }

        [Test]
        public void FloorAtReturnsThePatchFloorInsideAndTheArenaFloorOutside() {
            var f = Compiled().First(p => Path.GetFileName(p).StartsWith("friction_patches_train"));
            var spec = FleetSpec.LoadDirectory(SpecDir, Path.GetFullPath(f));
            var p0 = spec.V1().FloorPatches[0];
            string inside = spec.FloorAt((p0.X0 + p0.X1) / 2, (p0.Y0 + p0.Y1) / 2).Name;
            Assert.AreEqual(p0.ProfileFloor, inside);
            Assert.AreEqual(spec.S["arena.floor"].String, spec.FloorAt(0.2, 0.2).Name);
        }

        [Test]
        public void PolishedPatchFallsBackToTheArenaFloor() {
            var f = Compiled().First(p => Path.GetFileName(p).StartsWith("friction_patches_heldout"));
            var spec = FleetSpec.LoadDirectory(SpecDir, Path.GetFullPath(f));
            var p0 = spec.V1().FloorPatches[0];
            Assert.AreEqual("polished", p0.FrictionClass);
            Assert.IsNull(p0.ProfileFloor, "polished is not a fleet_profile_v0 floor");
            Assert.AreEqual(spec.S["arena.floor"].String, spec.FloorAt((p0.X0 + p0.X1) / 2, (p0.Y0 + p0.Y1) / 2).Name);
        }

        [Test]
        public void PlainV0ScenarioHasNoExtension() {
            var spec = FleetSpec.LoadDirectory(SpecDir);
            Assert.IsFalse(spec.V1().Present);
            Assert.AreEqual(spec.Floor().Name, spec.FloorAt(1.0, 1.0).Name);
        }

        [Test]
        public void TagsBecomeLandmarksOnDoorsAndBays() {
            var f = Compiled().First(p => Path.GetFileName(p).StartsWith("narrow_door_train"));
            var v1 = ScenarioV1.From(JsonNode.FromText(File.ReadAllText(f)));
            Assert.AreEqual(8, v1.Landmarks().Count(), "two doors x two faces x two jambs");
        }
    }
}
