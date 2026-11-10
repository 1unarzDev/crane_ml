using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using Crane.Fleet.Autonomy.Tests.Harness;
using Crane.Fleet.Docking;
using Xunit;
using Xunit.Abstractions;

namespace Crane.Fleet.Autonomy.Tests
{
    public class MissionFormatTests
    {
        private const string Head = "{\"schema\":\"crane.fleet_mission/v0\",\"tasks\":[";

        [Fact]
        public void Parse_RejectsBadSchema_UnknownType_DuplicateIds_UnknownDependency()
        {
            Assert.Throws<FormatException>(() => MissionSpec.Parse("{\"schema\":\"x\",\"tasks\":[]}"));
            Assert.Throws<FormatException>(() => MissionSpec.Parse(Head + "{\"id\":\"a\",\"type\":\"teleport\"}]}"));
            Assert.Throws<FormatException>(() => MissionSpec.Parse(Head + "{\"id\":\"a\",\"type\":\"wait\"},{\"id\":\"a\",\"type\":\"wait\"}]}"));
            Assert.Throws<FormatException>(() => MissionSpec.Parse(Head + "{\"id\":\"a\",\"type\":\"wait\",\"after\":[\"zz\"]}]}"));
        }

        [Fact]
        public void Parse_AcceptsHarnessAliases_AndInfersPerRoverOrder()
        {
            var m = MissionSpec.Parse(Head +
                "{\"id\":\"t1\",\"kind\":\"goto\",\"actors\":[\"cargo_1\"],\"route_m\":[[2,1],[3,1]]}," +
                "{\"id\":\"t2\",\"kind\":\"goto\",\"actors\":[\"cargo_2\"],\"reference_member\":\"cargo_2\",\"route_m\":[[2,2]]}," +
                "{\"id\":\"t3\",\"kind\":\"dock\",\"follower\":\"cargo_2\",\"leader\":\"cargo_1\",\"face_follower\":\"front\",\"face_leader\":\"rear\"}," +
                "{\"id\":\"t4\",\"kind\":\"undock\",\"coupling\":\"cargo_1.rear-cargo_2.front\"}]}");
            Assert.Equal("goto", m.Tasks[0].Type); Assert.Equal("cargo_1", m.Tasks[0].Str("agent"));
            Assert.Empty(m.Tasks[0].After); Assert.Empty(m.Tasks[1].After);
            Assert.Equal(new[] { "t2" }, m.Tasks[2].After);          // latest earlier task sharing a rover
            Assert.Equal("cargo_2", m.Tasks[2].Str("docker")); Assert.Equal("rear", m.Tasks[2].Str("target_face"));
            Assert.Equal("separate", m.Tasks[3].Type); Assert.Equal("cargo_2", m.Tasks[3].Str("docker")); Assert.Equal("front", m.Tasks[3].Str("docker_face"));
            Assert.Equal(new[] { "t3" }, m.Tasks[3].After);
        }

        [Fact]
        public void Parse_RejectsHarnessKindsWithoutEquivalent()
        {
            var ex = Assert.Throws<FormatException>(() => MissionSpec.Parse(Head + "{\"id\":\"t1\",\"kind\":\"load\",\"actors\":[\"lifter_1\"]}]}"));
            Assert.Contains("supported", ex.Message);
        }

        [Fact]
        public void SchemaFile_ListsEveryTaskType()
        {
            var path = Path.Combine(Env.FleetDir, "core_cs", "Crane.Fleet.Autonomy", "Mission", "mission_schema_v0.json");
            var txt = File.ReadAllText(path);
            foreach (var t in TaskSpec.KnownTypes) Assert.Contains("\"" + t + "\"", txt);
            Assert.NotNull(Crane.Fleet.Perception.MiniJson.Parse(txt));
        }

        [Fact]
        public void PromotedHarnessMissions_ParseInFull()
        {
            var dir = Path.Combine(Env.FleetDir, "harness", "out", "promoted");
            if (!Directory.Exists(dir)) return;
            int n = 0;
            foreach (var f in Directory.GetFiles(dir, "mission_*.json", SearchOption.AllDirectories))
            {
                var m = MissionSpec.Parse(File.ReadAllText(f));
                Assert.NotEmpty(m.Tasks);
                n++;
            }
            Assert.True(n > 0);
        }
    }

    public class ExecutiveTests
    {
        private static HarnessSetup TwoRovers(string tasks, double maxTime = 60)
        {
            var s = new HarnessSetup { Map = Sim.Warehouse(), MaxTime = maxTime };
            s.Rovers.Add(new RoverSetup { Id = "cargo_1", Role = "cargo", Truth = new Pose2(1.5, 1.5, 0), Believed = new Pose2(1.5, 1.5, 0) });
            s.Rovers.Add(new RoverSetup { Id = "cargo_2", Role = "cargo", Truth = new Pose2(1.5, 3.0, 0), Believed = new Pose2(1.5, 3.0, 0) });
            s.MissionJson = "{\"schema\":\"crane.fleet_mission/v0\",\"tasks\":[" + tasks + "]}";
            return s;
        }

        [Fact]
        public void FailedTask_IsRetried_ThenDependantsSkipped_WithClass()
        {
            var r = Sim.Run(TwoRovers("{\"id\":\"bad\",\"type\":\"goto\",\"agent\":\"cargo_1\",\"goal\":[6.0,1.0,0],\"retries\":2},{\"id\":\"next\",\"type\":\"goto\",\"agent\":\"cargo_1\",\"goal\":[2,2,0],\"after\":[\"bad\"]}"));
            var ex = r.Autonomy.Executive;
            Assert.True(ex.Done); Assert.False(ex.Succeeded);
            Assert.Equal(TaskState.Failed, ex.Record("bad").State); Assert.Equal(3, ex.Record("bad").Attempts);
            Assert.Equal(FailureClass.Geometry, ex.Record("bad").Class);
            Assert.Equal(TaskState.Skipped, ex.Record("next").State);
            Assert.Equal(FailureClass.Geometry, ex.FirstFailureClass);
        }

        [Fact]
        public void IndependentTasks_RunConcurrently_SharedRoverSerialised()
        {
            var r = Sim.Run(TwoRovers(
                "{\"id\":\"a\",\"type\":\"goto\",\"agent\":\"cargo_1\",\"goal\":[2.5,1.5,0]}," +
                "{\"id\":\"b\",\"type\":\"goto\",\"agent\":\"cargo_2\",\"goal\":[2.5,3.0,0]}," +
                "{\"id\":\"c\",\"type\":\"goto\",\"agent\":\"cargo_1\",\"goal\":[1.5,1.5,0],\"after\":[]}"));
            var ex = r.Autonomy.Executive;
            Assert.True(ex.Succeeded);
            Assert.Equal(ex.Record("a").Started, ex.Record("b").Started, 6);         // parallel
            Assert.True(ex.Record("c").Started >= ex.Record("a").Ended - 1e-9);       // exclusive use of cargo_1
        }

        [Fact]
        public void Timeout_IsAFailureClass()
        {
            var r = Sim.Run(TwoRovers("{\"id\":\"slow\",\"type\":\"goto\",\"agent\":\"cargo_1\",\"goal\":[8.5,3.5,0],\"timeout\":3}"));
            Assert.Equal(FailureClass.Timeout, r.Autonomy.Executive.Record("slow").Class);
        }

        [Fact]
        public void Wait_AndEventLogTransitions()
        {
            var r = Sim.Run(TwoRovers("{\"id\":\"w\",\"type\":\"wait\",\"agents\":[\"cargo_1\"],\"duration\":0.5}"));
            var kinds = r.Autonomy.Events.Select(e => e.Kind).ToList();
            Assert.Equal(new[] { "task_started", "task_succeeded", "mission_believed_complete", "mission_confirmed" }, kinds.Where(k => k.StartsWith("task_") || k.StartsWith("mission_")).ToArray());
        }

        [Fact]
        public void LibraryNeverReportsSuccessWithoutHost()
        {
            var s = TwoRovers("{\"id\":\"w\",\"type\":\"wait\",\"agents\":[\"cargo_1\"],\"duration\":0.2}");
            s.HostCheck = (w, m) => "operator says no";
            var r = Sim.Run(s);
            Assert.True(r.BelievedComplete); Assert.False(r.Success);
            Assert.Equal(FailureClass.HostRejected, r.Autonomy.Executive.FirstFailureClass);
        }

        [Fact]
        public void ScanMatchFixes_RelocaliseAWrongButConfidentEstimate()
        {
            var s = Missions.SingleNav(3);
            var t = s.Rovers[0].Truth; s.Rovers[0].Believed = new Pose2(t.X + 0.4, t.Y - 0.3, t.Yaw);   // confident (2 cm) and 0.5 m wrong
            s.WorldInit = w => w.ScanMatchRovers.Add("cargo_1");
            var r = Sim.Run(s);
            Assert.Contains(r.Autonomy.Events, e => e.Kind == "relocalised");
            Assert.True(r.Success, r.Failure);
        }

        [Fact]
        public void Explore_FindsTaggedPayload_ThenLiftAndHandoffOntoDeck()
        {
            var s = new HarnessSetup { Map = Sim.Warehouse(), MaxTime = 200 };
            s.Rovers.Add(new RoverSetup { Id = "lifter_1", Role = "lifter", Truth = new Pose2(3.4, 4.0, Math.PI / 2), Believed = new Pose2(3.4, 4.0, Math.PI / 2) });
            s.Rovers.Add(new RoverSetup { Id = "cargo_1", Role = "cargo", Truth = new Pose2(4.8, 3.0, 0), Believed = new Pose2(4.8, 3.0, 0) });
            s.WorldInit = w => w.AddPayload("box_s", "box_small", new Pose2(4.3, 5.6, 0));
            // cargo deck centre at (4.8, 3.0): the lifter holds the box 0.30 m ahead -> place pose (4.5, 3.0, 0)
            s.MissionJson = @"{""schema"":""crane.fleet_mission/v0"",""tasks"":[
 {""id"":""find"",""type"":""explore"",""agent"":""lifter_1"",""region"":[3.6,5.0,5.0,6.2],""target"":""box_s"",""viewpoints"":[3.5,5.6,0.0]},
 {""id"":""pick"",""type"":""lift"",""agent"":""lifter_1"",""object"":""box_s""},
 {""id"":""give"",""type"":""handoff"",""from"":""lifter_1"",""supporters"":[""cargo_1""],""place_on"":""cargo_1"",""place_offset"":[0,0,0],""payload"":""box_s"",""payload_type"":""box_small""}]}";
            var r = Sim.Run(s);
            Assert.True(r.Success, r.Failure + " | " + string.Join("\n", r.Autonomy.Events.Select(e => e.ToString())));
            Assert.Contains("box_s", r.Autonomy.Belief.Payloads.Keys);
            Assert.Equal(new[] { "cargo_1" }, r.World.Payloads[0].Supporters.ToArray());
        }
    }

    public class DockingBehaviourTests
    {
        private readonly ITestOutputHelper _o;
        public DockingBehaviourTests(ITestOutputHelper o) { _o = o; }

        [Fact]
        public void Obstructed_Aborts_WithDockingClass()
        {
            var s = Missions.TwoRoverDock(3);
            s.WorldInit = w => w.Pairs[0].Obstruction = true;
            var r = Sim.Run(s);
            Assert.False(r.Success);
            Assert.Equal(FailureClass.Docking, r.Class);
            Assert.Contains("obstructed", r.Failure);
            Assert.Equal(0, r.UnsafeReleases);
        }

        [Fact]
        public void NoTagView_FailsAsPerception()
        {
            var s = Missions.TwoRoverDock(4);
            s.WorldInit = w => w.R("cargo_2").Cameras.Clear();
            var r = Sim.Run(s);
            Assert.Equal(FailureClass.Perception, r.Class);
        }

        [Fact]
        public void HookFaults_RetryThenAbandon_ReleaseBeforeEachRetry()
        {
            var s = Missions.TwoRoverDock(5);
            s.WorldInit = w =>
            {
                var a = w.DockAssume.Clone(); a.HookMissCatchProbability = 1.0;   // hooks never catch: PARTIAL / hook timeout every attempt
                w.Pairs[0].Machine = new ConnectorMachine(new ContactModel(w.DockingProfile, a), 11, w.Pairs[0].Machine.Id);
            };
            var r = Sim.Run(s);
            Assert.False(r.Success);
            Assert.Equal(FailureClass.Docking, r.Class);
            var m = r.World.Pairs[0].Machine;
            Assert.True(m.FailedAttempts > m.A.MaxRetries);
            Assert.Contains(r.Autonomy.Events, e => e.Kind == "connector_cmd" && e.Detail == "Release");
            Assert.True(r.Autonomy.Events.Count(e => e.Kind == "connector_cmd" && e.Detail == "Approach") >= 2);
            Assert.Equal(0, r.UnsafeReleases);
        }

        [Fact]
        public void UncertainEstimate_SlowsOrBacksOff_NeverDrivesBlind()
        {
            var rs = new List<HarnessResult>();
            for (ulong seed = 1; seed <= 6; seed++)
            {
                var s = Missions.TwoRoverDock(seed);
                s.WorldInit = w => { w.Noise.CornerSigmaPx = 6.0; };   // six times the assumed corner noise
                rs.Add(Sim.Run(s));
            }
            Sim.Report("docking_corner_noise_6px", rs);
            int backoffs = rs.Sum(r => r.Autonomy.Events.Count(e => e.Kind == "dock_uncertain"));
            int jams = rs.Sum(r => r.World.Pairs[0].Machine.Log.Count(e => e.Kind == EventKind.Jam));
            _o.WriteLine($"uncertain back-offs {backoffs}, jams {jams}");
            Assert.True(backoffs > 0 || rs.All(r => r.Success));
            Assert.All(rs, r => Assert.Equal(0, r.UnsafeReleases));
        }

        [Fact]
        public void OverheadCamera_IsFusedIntoBothEstimators()
        {
            var r = Sim.Run(Missions.TwoRoverDock(2, overhead: true));
            Assert.True(r.Success, r.Failure);
            _o.WriteLine($"overhead self-observations offered {r.Autonomy.WorldPoseOffered}, accepted {r.Autonomy.WorldPoseAccepted}");
            Assert.True(r.Autonomy.WorldPoseAccepted > 0);
        }

        [Fact]
        public void LargerStartError_SixCmTenDegrees_Reported()
        {
            var rs = new List<HarnessResult>();
            for (ulong seed = 1; seed <= 12; seed++) rs.Add(Sim.Run(Missions.TwoRoverDock(seed, false, 0.06, 10)));
            Sim.Report("docking_start_error_6cm_10deg", rs);
            Sim.Tally(rs, out int ok, out int n);
            Assert.True(ok >= n / 2);
        }
    }

    public class IntegrationContractTests
    {
        private static Assembly Lib => typeof(FleetAutonomy).Assembly;

        [Fact]
        public void Determinism_SameSeedSameRun_DifferentSeedDiffers()
        {
            var a = Sim.Run(Missions.TwoRoverDock(9)); var b = Sim.Run(Missions.TwoRoverDock(9)); var c = Sim.Run(Missions.TwoRoverDock(10));
            Assert.Equal(string.Join("\n", a.Autonomy.Events.Select(e => e.ToString())), string.Join("\n", b.Autonomy.Events.Select(e => e.ToString())));
            Assert.Equal(a.World.R("cargo_2").Pose.X, b.World.R("cargo_2").Pose.X);
            Assert.Equal(a.World.R("cargo_2").Pose.Yaw, b.World.R("cargo_2").Pose.Yaw);
            Assert.NotEqual(a.World.R("cargo_2").Pose.X, c.World.R("cargo_2").Pose.X);
        }

        [Fact]
        public void Determinism_NavigationWithPlanning()
        {
            var a = Sim.Run(Missions.LoadedLine(2, false)); var b = Sim.Run(Missions.LoadedLine(2, false));
            Assert.Equal(a.Time, b.Time);
            Assert.Equal(a.World.R("cargo_1").Pose.X, b.World.R("cargo_1").Pose.X);
        }

        [Fact]
        public void NoEngineOrTruthReferences()
        {
            foreach (var r in Lib.GetReferencedAssemblies()) Assert.DoesNotContain("UnityEngine", r.Name);
            var bytes = File.ReadAllBytes(Lib.Location);
            var text = Encoding.ASCII.GetString(bytes);
            Assert.DoesNotContain("Perception.Evaluator", text);   // the truth namespace never appears in the metadata
            Assert.DoesNotContain("EvaluatorTruth", text);
            Assert.DoesNotContain("KinematicWorld", text);
        }

        /// <summary>Public type names of CRANE's Crane.Fleet.Core (Assets/Scripts/Fleet/Core, research/fleet worktree, 2026-11-10) and the
        /// sibling libraries; the autonomy library shares the Crane.Fleet namespace with them in one Unity assembly set.</summary>
        private static readonly string[] Reserved =
        {
            "Twist", "FleetRng", "MecanumGeometry", "PlantParams", "MecanumPlant", "WheelEncoders", "YawGyro", "WheelOdometry", "GoToController", "Angles",
            "StepEvent", "QualificationScript", "RunIds", "SpecFile", "FleetSpec", "FloorProperties", "PrimitivePart", "WheelFrame", "SensorFrame",
            "DockingFace", "RoverDescription", "FleetJson", "JsonNode", "AssetManifest", "Joint", "Part", "MaterialSlot", "Shape", "Face", "Hook", "ObjMesh",
            "Vec2", "Pose2", "Hull2", "DeterministicRng", "Rover", "Payload", "Coupling", "ModuleGraph",
        };

        [Fact]
        public void TypeNames_DoNotCollideWithCraneCoreOrSiblings()
        {
            var mine = Lib.GetTypes().Where(t => t.IsPublic && t.Namespace == "Crane.Fleet").Select(t => t.Name).ToList();
            foreach (var n in mine) Assert.DoesNotContain(n, Reserved);
            var siblings = new[] { typeof(Pose2).Assembly, typeof(CapWheel).Assembly, typeof(Crane.Fleet.Perception.Pose).Assembly }
                .SelectMany(a => a.GetTypes()).Where(t => t.IsPublic && t.Namespace == "Crane.Fleet").Select(t => t.Name).ToHashSet();
            foreach (var n in mine) Assert.DoesNotContain(n, siblings);
        }

        [Fact]
        public void AssumptionRegistry_DocumentsReplacements()
        {
            Assert.True(AutonomyAssumptions.Registry.Count >= 10);
            Assert.All(AutonomyAssumptions.Registry, e => Assert.False(string.IsNullOrWhiteSpace(e.ReplacedBy)));
        }

        [Fact]
        public void RoverSpec_ReadsProfileWithoutHardCodedNumbers()
        {
            var s = Env.Spec("cargo_1", "cargo");
            var p = Env.Profile;
            Assert.Equal(p.Num("chassis.wheel.radius"), s.WheelRadius);
            Assert.Equal(p.Num("floors.hard_tile.lateral_efficiency"), s.LateralEfficiency);
            Assert.True(s.HasDeck); Assert.True(s.HasFrontCamera);
            Assert.True(Env.Spec("lifter_1", "lifter").HasLift);
            Assert.Equal(4, s.Faces.Count);
        }
    }
}
