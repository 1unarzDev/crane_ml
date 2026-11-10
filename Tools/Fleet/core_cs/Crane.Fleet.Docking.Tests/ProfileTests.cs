using System;
using System.Linq;
using System.Text.Json.Nodes;
using Crane.Fleet.Docking;
using Crane.Fleet.Profile;
using Xunit;

namespace Crane.Fleet.Docking.Tests
{
    public class ProfileTests
    {
        private static ProfileLoadResult LoadMutated(Action<JsonNode> mutate)
        {
            var n = JsonNode.Parse(Fx.ProfileJson());
            mutate(n);
            return ProfileLoader.Load(n.ToJsonString());
        }

        [Fact]
        public void Loads_real_profile_without_errors()
        {
            var r = ProfileLoader.Load(Fx.ProfileJson());
            Assert.True(r.Ok, string.Join("; ", r.Errors));
            Assert.Equal("mecanum_fleet_provisional_v0", r.Profile.ProfileId);
        }

        [Fact]
        public void Values_come_from_file_in_file_units()
        {
            var p = Fx.Profile();
            var d = JsonNode.Parse(Fx.ProfileJson())["docking"];
            Assert.Equal((double)d["hook_lateral_spacing"]["value"], p.HookLateralSpacing.Value);
            Assert.Equal((double)d["guide_acceptance_half_angle"]["value"], p.GuideAcceptanceHalfAngle.Value);
            Assert.Equal((double)d["hook_rated_load"]["value"], p.HookRatedLoad.Value);
            Assert.Equal(p.HookLateralSpacing.Value / 2, p.HookHalfSpan);
        }

        [Fact]
        public void Provenance_label_is_carried_through()
        {
            var r = LoadMutated(n => n["docking"]["hook_rated_load"]["provenance"] = "measured");
            Assert.True(r.Ok);
            Assert.Equal(Provenance.Measured, r.Profile.HookRatedLoad.Provenance);
            Assert.Equal(Provenance.Provisional, r.Profile.HookActuationTime.Provenance);
            var all = r.Profile.AllProvenance().ToDictionary(k => k.Key, k => k.Value);
            Assert.Equal(Provenance.Measured, all["docking.hook_rated_load"]);
            Assert.Equal(Provenance.Provisional, all["docking.face_frames.front"]);
        }

        [Fact]
        public void Face_frames_loaded_with_all_four_faces()
        {
            var p = Fx.Profile();
            Assert.Equal(new[] { "front", "left", "rear", "right" }, p.FaceFrames.Keys.OrderBy(k => k).ToArray());
            Assert.Equal(0.21, p.FaceFrames["front"].X);
            Assert.Equal(new[] { "front", "rear" }, p.ActiveFaces);
        }

        [Fact]
        public void Missing_field_is_reported_with_path()
        {
            var r = LoadMutated(n => ((JsonObject)n["docking"]).Remove("hook_actuation_time"));
            Assert.False(r.Ok);
            Assert.Contains(r.Errors, e => e.Path == "docking.hook_actuation_time");
        }

        [Fact]
        public void Non_positive_tolerance_is_rejected()
        {
            var r = LoadMutated(n => n["docking"]["hook_position_tolerance"]["value"] = 0.0);
            Assert.Contains(r.Errors, e => e.Path == "docking.hook_position_tolerance" && e.Message.Contains("> 0"));
            r = LoadMutated(n => n["docking"]["hook_yaw_tolerance"]["value"] = -0.01);
            Assert.Contains(r.Errors, e => e.Path == "docking.hook_yaw_tolerance");
        }

        [Fact]
        public void Unknown_provenance_is_rejected()
        {
            var r = LoadMutated(n => n["docking"]["magnet_hold_force"]["provenance"] = "guessed");
            Assert.Contains(r.Errors, e => e.Path == "docking.magnet_hold_force.provenance" && e.Message.Contains("unknown provenance"));
        }

        [Fact]
        public void Missing_provenance_is_rejected()
        {
            var r = LoadMutated(n => ((JsonObject)n["docking"]["guide_half_width"]).Remove("provenance"));
            Assert.Contains(r.Errors, e => e.Path == "docking.guide_half_width.provenance");
        }

        [Fact]
        public void Unknown_state_name_is_rejected()
        {
            var r = LoadMutated(n => ((JsonArray)n["docking"]["states"]).Add("TELEPORTED"));
            Assert.Contains(r.Errors, e => e.Path == "docking.states" && e.Message.Contains("TELEPORTED"));
        }

        [Fact]
        public void Wrong_units_and_hook_count_are_rejected()
        {
            var r = LoadMutated(n => n["units"]["length"] = "mm");
            Assert.Contains(r.Errors, e => e.Path == "units.length");
            r = LoadMutated(n => n["docking"]["hook_count"]["value"] = 3);
            Assert.Contains(r.Errors, e => e.Path == "docking.hook_count");
        }

        [Fact]
        public void Invalid_json_and_LoadOrThrow()
        {
            Assert.False(ProfileLoader.Load("{ nope").Ok);
            var ex = Assert.Throws<ProfileException>(() => ProfileLoader.LoadOrThrow("{}"));
            Assert.NotEmpty(ex.Errors);
        }

        [Fact]
        public void State_names_round_trip_with_profile_vocabulary()
        {
            var p = Fx.Profile();
            foreach (string name in p.States.Concat(p.FaultStates))
            {
                string pascal = ConnectorStateNames.FromProfileName(name);
                Assert.True(Enum.TryParse(pascal, out ConnectorState s), name);
                Assert.Equal(name, ConnectorStateNames.ToProfileName(s));
            }
            Assert.Equal(14, Enum.GetValues(typeof(ConnectorState)).Length);
        }

        [Fact]
        public void No_profile_value_is_duplicated_in_library_sources()
        {
            string dir = AppContext.BaseDirectory;
            while (dir != null && !System.IO.Directory.Exists(System.IO.Path.Combine(dir, "Crane.Fleet.Docking"))) dir = System.IO.Path.GetDirectoryName(dir);
            Assert.NotNull(dir);
            var src = string.Join("\n", System.IO.Directory.GetFiles(System.IO.Path.Combine(dir, "Crane.Fleet.Docking"), "*.cs", System.IO.SearchOption.AllDirectories)
                .Where(f => !f.Contains("/obj/") && !f.Contains("/bin/")).Select(System.IO.File.ReadAllText));
            // distinctive profile values: must not appear as literals anywhere in the library
            foreach (string lit in new[] { "0.087", "0.035", "0.12", "60.0", "0.21", "25.0" })
                Assert.DoesNotContain(lit, src);
        }
    }
}
