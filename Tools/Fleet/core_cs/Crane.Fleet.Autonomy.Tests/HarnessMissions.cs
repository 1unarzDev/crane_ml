using System;
using System.Collections.Generic;
using System.Linq;
using Crane.Fleet.Autonomy.Tests.Harness;
using Crane.Fleet.Docking;
using Xunit;
using Xunit.Abstractions;

namespace Crane.Fleet.Autonomy.Tests
{
    /// <summary>Scenario builders shared by the whole-mission tests.</summary>
    public static class Missions
    {
        public static HarnessSetup SingleNav(ulong seed)
        {
            var rng = new DeterministicRng(seed * 31 + 7);
            var s = new HarnessSetup { Seed = seed, Map = Sim.Warehouse(), MaxTime = 120 };
            var start = new Pose2(1.5 + 0.3 * (rng.NextDouble() - 0.5), 1.2 + 0.3 * (rng.NextDouble() - 0.5), 0.4 * (rng.NextDouble() - 0.5));
            s.Rovers.Add(new RoverSetup { Id = "cargo_1", Role = "cargo", Truth = start, Believed = start });
            s.MissionJson = Sim.GotoMission("cargo_1", new Pose2(8.5, 3.5, 0));
            return s;
        }

        /// <summary>Two-rover docking from the pre-dock pose (0.5 m) with a start error (lateral +-3 cm, range +-2 cm, yaw +-5 deg)
        /// that the docker does not know: its believed pose is the nominal pre-dock pose. Tag-only feedback (no overhead).</summary>
        public static HarnessSetup TwoRoverDock(ulong seed, bool overhead = false, double lat = 0.03, double yawDeg = 5)
        {
            var rng = new DeterministicRng(seed * 7919 + 3);
            var s = new HarnessSetup { Seed = seed, Map = Sim.Warehouse(), MaxTime = 120, Overhead = overhead };
            var target = new Pose2(4.5, 3.4, 0);
            var spec = Env.Spec("cargo_2", "cargo");
            // nominal pre-dock: docker front face 0.5 m behind the target's rear face, facing +x
            var tRear = spec.Face("rear").InBody; var dFront = spec.Face("front").InBody;
            var mate = Se2.Compose(Se2.Compose(target, tRear), new Pose2(0, 0, Math.PI));
            var faceA = Se2.Compose(mate, new Pose2(-0.5, 0, 0));
            var nominal = Se2.Compose(faceA, Se2.Inverse(dFront));
            double ey = lat * (2 * rng.NextDouble() - 1), ex = 0.02 * (2 * rng.NextDouble() - 1), eth = yawDeg * Math.PI / 180 * (2 * rng.NextDouble() - 1);
            var truth = Se2.Compose(nominal, new Pose2(ex, ey, eth));
            s.Rovers.Add(new RoverSetup { Id = "cargo_1", Role = "cargo", Truth = target, Believed = target });
            s.Rovers.Add(new RoverSetup { Id = "cargo_2", Role = "cargo", Truth = truth, Believed = nominal });
            s.InitialPosSigma = 0.03; s.InitialYawSigma = 0.09;
            s.Pairs.Add(new[] { "cargo_2", "front", "cargo_1", "rear" });
            s.MissionJson = "{\"schema\":\"crane.fleet_mission/v0\",\"mission_id\":\"dock2\",\"tasks\":[{\"id\":\"dock\",\"type\":\"dock\",\"docker\":\"cargo_2\",\"docker_face\":\"front\",\"target\":\"cargo_1\",\"target_face\":\"rear\",\"retries\":0}]}";
            return s;
        }

        /// <summary>Line assembly from separate start poses (rendezvous + dock), then transport of the line through the 0.8 m doorway.</summary>
        public static HarnessSetup LineAssemblyTransport(ulong seed)
        {
            var rng = new DeterministicRng(seed * 104729 + 11);
            var s = new HarnessSetup { Seed = seed, Map = Sim.Warehouse(), MaxTime = 260 };
            var p1 = new Pose2(2.2 + 0.2 * (rng.NextDouble() - 0.5), 3.0 + 0.2 * (rng.NextDouble() - 0.5), 0.3 * (rng.NextDouble() - 0.5));
            var p2 = new Pose2(1.2 + 0.2 * (rng.NextDouble() - 0.5), 1.5 + 0.2 * (rng.NextDouble() - 0.5), 0.3 * (rng.NextDouble() - 0.5));
            s.Rovers.Add(new RoverSetup { Id = "cargo_1", Role = "cargo", Truth = p1, Believed = p1 });
            s.Rovers.Add(new RoverSetup { Id = "cargo_2", Role = "cargo", Truth = p2, Believed = p2 });
            s.Pairs.Add(new[] { "cargo_2", "front", "cargo_1", "rear" });
            s.MissionJson = @"{""schema"":""crane.fleet_mission/v0"",""mission_id"":""line_transport"",""tasks"":[
 {""id"":""meet"",""type"":""rendezvous"",""target"":""cargo_1"",""docker"":""cargo_2"",""docker_face"":""front"",""target_face"":""rear"",""meeting_pose"":[4.5,3.5,0]},
 {""id"":""asm"",""type"":""assemble"",""after"":[""meet""],""sequence"":[{""docker"":""cargo_2"",""docker_face"":""front"",""target"":""cargo_1"",""target_face"":""rear""}]},
 {""id"":""move"",""type"":""transport"",""after"":[""asm""],""agent"":""cargo_1"",""goal"":[8.6,3.5,0]}]}";
            return s;
        }

        /// <summary>Pre-assembled side-by-side pair (0.80 m wide, 0.42 m deep) crossing a 0.65 m doorway: only crabbing fits.</summary>
        public static HarnessSetup CrabPair(ulong seed, double door = 0.65)
        {
            var rng = new DeterministicRng(seed * 15485863 + 5);
            var s = new HarnessSetup { Seed = seed, Map = Sim.Warehouse(door), MaxTime = 150 };
            var a = new Pose2(4.3 + 0.2 * (rng.NextDouble() - 0.5), 3.0 + 0.2 * (rng.NextDouble() - 0.5), Math.PI / 2 + 0.1 * (rng.NextDouble() - 0.5));
            var spec = Env.Spec("cargo_1", "cargo");
            var b = AssemblyModel.MatedPose(a, spec.Face("left"), spec.Face("right"));
            s.Rovers.Add(new RoverSetup { Id = "cargo_1", Role = "cargo", Truth = a, Believed = a });
            s.Rovers.Add(new RoverSetup { Id = "cargo_2", Role = "cargo", Truth = b, Believed = b });
            s.PreLatched.Add(new[] { "cargo_2", "right", "cargo_1", "left" });
            s.MissionJson = @"{""schema"":""crane.fleet_mission/v0"",""mission_id"":""crab"",
 ""initial_couplings"":[{""docker"":""cargo_2"",""docker_face"":""right"",""target"":""cargo_1"",""target_face"":""left""}],
 ""tasks"":[{""id"":""cross"",""type"":""transport"",""agent"":""cargo_1"",""goal"":[7.6,3.3,1.5707963]}]}";
            return s;
        }

        /// <summary>Pre-assembled line with a box_long payload spanning both decks, transported through the doorway; optionally a
        /// wheel of cargo_2 fails (locks) mid-mission.</summary>
        public static HarnessSetup LoadedLine(ulong seed, bool failWheel, CapHealthMode mode = CapHealthMode.DeadLocked, double failAt = 8.0)
        {
            var rng = new DeterministicRng(seed * 32452843 + 1);
            var s = new HarnessSetup { Seed = seed, Map = Sim.Warehouse(), MaxTime = 300 };
            var a = new Pose2(3.4 + 0.2 * (rng.NextDouble() - 0.5), 3.4 + 0.2 * (rng.NextDouble() - 0.5), 0.2 * (rng.NextDouble() - 0.5));
            var spec = Env.Spec("cargo_1", "cargo");
            var b = AssemblyModel.MatedPose(a, spec.Face("rear"), spec.Face("front"));
            s.Rovers.Add(new RoverSetup { Id = "cargo_1", Role = "cargo", Truth = a, Believed = a });
            s.Rovers.Add(new RoverSetup { Id = "cargo_2", Role = "cargo", Truth = b, Believed = b });
            s.PreLatched.Add(new[] { "cargo_2", "front", "cargo_1", "rear" });
            var com = Se2.Compose(a, new Pose2(-0.21, 0, 0));
            s.WorldInit = w => w.AddPayload("box1", "box_long", com, "cargo_1", "cargo_2");
            s.MissionJson = @"{""schema"":""crane.fleet_mission/v0"",""mission_id"":""loaded_line"",
 ""initial_couplings"":[{""docker"":""cargo_2"",""docker_face"":""front"",""target"":""cargo_1"",""target_face"":""rear""}],
 ""initial_payloads"":[{""id"":""box1"",""type"":""box_long"",""supporters"":[""cargo_1"",""cargo_2""],""com"":[" + Sim.F(com.X) + "," + Sim.F(com.Y) + @"]}],
 ""tasks"":[{""id"":""move"",""type"":""transport"",""agent"":""cargo_1"",""payload"":""box1"",""goal"":[8.4,3.5,0],""retries"":2}]}";
            if (failWheel)
                s.Hook = (w, fa, t) => { if (Math.Abs(t - failAt) < 1e-6 || (t > failAt && w.R("cargo_2").Health[0].Mode == CapHealthMode.Ok)) w.R("cargo_2").Health[0] = mode == CapHealthMode.Weak ? CapHealth.Weak(0.2) : new CapHealth(mode); };
            return s;
        }

        /// <summary>Single rover crossing the open hall; one wheel locks mid-way (rank 3 -> 2): identification, capability rebuild, replanning.</summary>
        public static HarnessSetup LockedSingle(ulong seed, double failAt = 4.0)
        {
            var rng = new DeterministicRng(seed * 86028121 + 3);
            var s = new HarnessSetup { Seed = seed, Map = Sim.Warehouse(), MaxTime = 120 };
            var a = new Pose2(1.5 + 0.2 * (rng.NextDouble() - 0.5), 1.5 + 0.2 * (rng.NextDouble() - 0.5), 0.2 * (rng.NextDouble() - 0.5));
            s.Rovers.Add(new RoverSetup { Id = "cargo_1", Role = "cargo", Truth = a, Believed = a });
            s.MissionJson = @"{""schema"":""crane.fleet_mission/v0"",""tasks"":[{""id"":""go"",""type"":""goto"",""agent"":""cargo_1"",""goal"":[3.6,3.4,0],""retries"":2}]}";
            int wheel = (int)(seed % 4);
            s.Hook = (w, fa, t) => { if (t >= failAt && w.R("cargo_1").Health[wheel].Mode == CapHealthMode.Ok) w.R("cargo_1").Health[wheel] = CapHealth.DeadLocked; };
            return s;
        }

        public static readonly (string, Pose2)[] DoorJambs = { ("jamb_s", new Pose2(5.92, 2.95, Math.PI)), ("jamb_n", new Pose2(5.92, 4.05, Math.PI)) };

        /// <summary>CRANE phase-2 failure 1: the loaded line is shoved sideways (by another rover; the wheels do not see it), then drives
        /// into the doorway jamb; its wheels keep turning while the body is stuck, so odometry integrates motion that never happened.</summary>
        public static HarnessSetup WallJam(ulong seed, bool safeguards, bool landmarks)
        {
            var s = LoadedLine(seed, false);
            s.MaxTime = 120;
            s.Hook = (w, fa, t) => { if (Math.Abs(t - 5.0) < 1e-6) w.Shove("cargo_1", 0, 0.5, 0.7); };
            if (landmarks) s.Landmarks.AddRange(DoorJambs);
            if (!safeguards) s.ConfigHook = c => { c.StallDetection = false; c.Relocalisation = false; };
            return s;
        }

        /// <summary>CRANE phase-2 failure 2: the lifter's odometry is biased (here +0.15 m, +0.17 m, believed with 2 cm sigma) when it
        /// hands a box over to a cargo deck.</summary>
        public static HarnessSetup HandoffBias(ulong seed, bool safeguards, bool loadCells = false)
        {
            var rng = new DeterministicRng(seed * 7 + 1);
            var s = new HarnessSetup { Seed = seed, Map = Sim.Warehouse(), MaxTime = 120 };
            var lt = new Pose2(3.2, 3.0 + 0.1 * (rng.NextDouble() - 0.5), 0);
            s.Rovers.Add(new RoverSetup { Id = "lifter_1", Role = "lifter", Truth = lt, Believed = new Pose2(lt.X + 0.15, lt.Y + 0.17, lt.Yaw) });
            s.Rovers.Add(new RoverSetup { Id = "cargo_1", Role = "cargo", Truth = new Pose2(4.8, 3.0, 0), Believed = new Pose2(4.8, 3.0, 0) });
            s.WorldInit = w =>
            {
                var p = w.AddPayload("box_s", "box_small", Se2.Compose(lt, new Pose2(0.30, 0, 0)));
                var l = w.R("lifter_1"); p.CarriedBy = "lifter_1"; p.InCarrier = new Pose2(0.30, 0, 0); l.Holding = "box_s"; l.GripClosed = true; l.GripCmd = true;
                w.DeckLoadCells = loadCells;
            };
            if (!safeguards) s.ConfigHook = c => { c.HandoffTagAlignment = false; c.PostReleaseSupportCheck = false; };
            s.MissionJson = @"{""schema"":""crane.fleet_mission/v0"",""tasks"":[{""id"":""give"",""type"":""handoff"",""from"":""lifter_1"",""supporters"":[""cargo_1""],
 ""place_on"":""cargo_1"",""place_offset"":[0,0,0],""place_pose"":[4.5,3.0,0],""payload"":""box_s"",""payload_type"":""box_small"",""retries"":0}]}";
            return s;
        }

        /// <summary>Pre-assembled line; separate (release + clear). With a payload spanning both decks the release must be refused.</summary>
        public static HarnessSetup Separation(ulong seed, bool payloadAcross)
        {
            var rng = new DeterministicRng(seed * 49979687 + 9);
            var s = new HarnessSetup { Seed = seed, Map = Sim.Warehouse(), MaxTime = 60 };
            var a = new Pose2(4.0 + 0.2 * (rng.NextDouble() - 0.5), 3.2 + 0.2 * (rng.NextDouble() - 0.5), 0.3 * (rng.NextDouble() - 0.5));
            var spec = Env.Spec("cargo_1", "cargo");
            var b = AssemblyModel.MatedPose(a, spec.Face("rear"), spec.Face("front"));
            s.Rovers.Add(new RoverSetup { Id = "cargo_1", Role = "cargo", Truth = a, Believed = a });
            s.Rovers.Add(new RoverSetup { Id = "cargo_2", Role = "cargo", Truth = b, Believed = b });
            s.PreLatched.Add(new[] { "cargo_2", "front", "cargo_1", "rear" });
            string payload = "";
            if (payloadAcross)
            {
                var com = Se2.Compose(a, new Pose2(-0.21, 0, 0));
                s.WorldInit = w => w.AddPayload("box1", "box_long", com, "cargo_1", "cargo_2");
                payload = @",""initial_payloads"":[{""id"":""box1"",""type"":""box_long"",""supporters"":[""cargo_1"",""cargo_2""],""com"":[" + Sim.F(com.X) + "," + Sim.F(com.Y) + "]}]";
            }
            s.MissionJson = @"{""schema"":""crane.fleet_mission/v0"",""mission_id"":""separate"",
 ""initial_couplings"":[{""docker"":""cargo_2"",""docker_face"":""front"",""target"":""cargo_1"",""target_face"":""rear""}]" + payload + @",
 ""tasks"":[{""id"":""sep"",""type"":""separate"",""docker"":""cargo_2"",""docker_face"":""front"",""target"":""cargo_1"",""target_face"":""rear"",""clear_distance"":0.3}]}";
            return s;
        }
    }

    public class HarnessMissionTests
    {
        [Fact]
        public void LineAssembly_ThenTransportThroughDoorway_Seeds()
        {
            var rs = new List<HarnessResult>();
            for (ulong seed = 1; seed <= 8; seed++) rs.Add(Sim.Run(Missions.LineAssemblyTransport(seed)));
            Sim.Report("line_assembly_transport", rs, r => $"t={r.Time:0.0} s, line front truth x={r.World.R("cargo_1").Pose.X:0.000}");
            Sim.Tally(rs, out int ok, out int n);
            Assert.True(ok >= 7, Sim.Tally(rs, out _, out _));
        }

        [Fact]
        public void SideBySidePair_CrabsThroughNarrowDoorway_Seeds()
        {
            var rs = new List<HarnessResult>();
            for (ulong seed = 1; seed <= 8; seed++) rs.Add(Sim.Run(Missions.CrabPair(seed)));
            Sim.Report("crab_pair_door_0.65", rs, r => $"t={r.Time:0.0} s, final yaw {r.World.R("cargo_1").Pose.Yaw:0.000}");
            Sim.Tally(rs, out int ok, out int n);
            Assert.True(ok >= 7, Sim.Tally(rs, out _, out _));
            Assert.All(rs.Where(r => r.Success), r => Assert.InRange(r.World.R("cargo_1").Pose.Yaw, Math.PI / 2 - 0.2, Math.PI / 2 + 0.2));
        }

        [Fact]
        public void Separation_ReleasesAndClears_OrRefusesUnderLoad()
        {
            var free = new List<HarnessResult>(); var loaded = new List<HarnessResult>();
            for (ulong seed = 1; seed <= 6; seed++) { free.Add(Sim.Run(Missions.Separation(seed, false))); loaded.Add(Sim.Run(Missions.Separation(seed, true))); }
            Sim.Report("separation_unloaded", free);
            Sim.Report("separation_payload_across_coupling", loaded);
            Assert.All(free, r => { Assert.True(r.Success, r.Failure); Assert.NotSame(r.Autonomy.AgentOf("cargo_1"), r.Autonomy.AgentOf("cargo_2")); Assert.True(Se2.Distance(r.World.R("cargo_1").Pose, r.World.R("cargo_2").Pose) > 0.6); });
            Assert.All(loaded, r =>
            {
                Assert.Equal(FailureClass.Support, r.Class);
                Assert.Equal(ConnectorState.LatchedVerified, r.World.Pairs[0].Machine.State == ConnectorState.LoadValidated ? ConnectorState.LatchedVerified : r.World.Pairs[0].Machine.State);
                Assert.Equal(0, r.UnsafeReleases);
                Assert.DoesNotContain(r.Autonomy.Events, e => e.Kind == "connector_cmd" && e.Detail == "Release");
            });
        }

        [Fact]
        public void LoadedLine_WeakWheelMidMission_Replans_AndDelivers()
        {
            var rs = new List<HarnessResult>();
            for (ulong seed = 1; seed <= 6; seed++) rs.Add(Sim.Run(Missions.LoadedLine(seed, true, CapHealthMode.Weak)));
            Sim.Report("loaded_line_weak_wheel_midmission", rs, r => $"t={r.Time:0.0} s, identified={r.Autonomy.Events.Count(e => e.Kind == "wheel_health_identified")} replans={r.Autonomy.Events.Count(e => e.Kind == "plan") - 1}");
            Sim.Tally(rs, out int ok, out int n);
            Assert.True(ok >= 5, Sim.Tally(rs, out _, out _));
            Assert.All(rs, r => Assert.Contains(r.Autonomy.Events, e => e.Kind == "wheel_health_identified" && e.Detail.Contains("Weak")));
        }

        /// <summary>Hard case (q3: "the doorway then needs a planned zig-zag"): a loaded two-module line loses a wheel to dead_locked before
        /// the 0.8 m doorway. Reported, not required: the outcome must be either delivery or a classified failure, never unsafe.</summary>
        [Fact]
        public void LoadedLine_LockedWheelBeforeDoorway_DeliversOrReportsInfeasible()
        {
            var rs = new List<HarnessResult>();
            for (ulong seed = 1; seed <= 2; seed++) { var s = Missions.LoadedLine(seed, true, CapHealthMode.DeadLocked); s.MaxTime = 160; s.ConfigHook = c => c.Assume.PlanMaxExpansionsReduced = 5000; rs.Add(Sim.Run(s)); }
            Sim.Report("loaded_line_locked_wheel_before_doorway", rs, r => "t=" + r.Time.ToString("0.0") + " s, plans=" + r.Autonomy.Events.Count(e => e.Kind == "plan") + ", success=" + r.Success + ", class=" + r.Class);
            Assert.All(rs, r => Assert.Equal(0, r.UnsafeReleases));
            Assert.All(rs, r => Assert.True(r.Success || r.Class != FailureClass.None));
            Assert.All(rs, r => Assert.Contains(r.Autonomy.Events, e => e.Kind == "wheel_health_identified"));
        }

        [Fact]
        public void SingleRover_LockedWheelMidMission_RankDrops_Replans()
        {
            var rs = new List<HarnessResult>();
            for (ulong seed = 1; seed <= 4; seed++) rs.Add(Sim.Run(Missions.LockedSingle(seed)));
            Sim.Report("single_rover_locked_wheel_midmission", rs, r => $"t={r.Time:0.0} s, final truth error {Se2.Distance(r.World.R("cargo_1").Pose, new Pose2(3.6, 3.4, 0)):0.000} m, ranks={string.Join(",", r.Autonomy.Events.Where(e => e.Kind == "plan").Select(e => e.Detail.Contains("rank=2") ? "2" : "3"))}");
            Assert.All(rs, r => Assert.Contains(r.Autonomy.Events, e => e.Kind == "wheel_health_identified" && e.Detail.Contains("DeadLocked")));
            Assert.All(rs, r => Assert.Contains(r.Autonomy.Events, e => e.Kind == "plan" && e.Detail.Contains("rank=2")));
            Sim.Tally(rs, out int ok, out int n);
            Assert.True(ok >= 3, Sim.Tally(rs, out _, out _));
        }

        private readonly ITestOutputHelper _o;
        public HarnessMissionTests(ITestOutputHelper o) { _o = o; }

        private static double TruthError(HarnessResult r, string rover, Pose2 goal) => Se2.Distance(r.World.R(rover).Pose, goal);

        [Fact]
        public void SingleRoverNavigation_ThroughDoorway_Seeds()
        {
            var rs = new List<HarnessResult>();
            var errs = new List<double>();
            for (ulong seed = 1; seed <= 10; seed++)
            {
                var r = Sim.Run(Missions.SingleNav(seed)); rs.Add(r);
                errs.Add(TruthError(r, "cargo_1", new Pose2(8.5, 3.5, 0)));
            }
            Sim.Report("single_rover_navigation", rs, r => $"truth error {TruthError(r, "cargo_1", new Pose2(8.5, 3.5, 0)):0.000} m, t={r.Time:0.0} s");
            Sim.Tally(rs, out int ok, out int n);
            _o.WriteLine($"nav ok {ok}/{n}, median truth error {errs.OrderBy(x => x).ElementAt(errs.Count / 2):0.000} m");
            Assert.True(ok >= 9, Sim.Tally(rs, out _, out _));
        }

        [Fact]
        public void TwoRoverDocking_TagOnly_FromHalfMetre_WithStartError_Seeds()
        {
            var rs = new List<HarnessResult>();
            for (ulong seed = 1; seed <= 30; seed++) rs.Add(Sim.Run(Missions.TwoRoverDock(seed)));
            Sim.Report("two_rover_docking_tag_only", rs, r =>
            {
                var p = r.World.Pairs[0];
                return $"state={p.Machine.State} failed_attempts={p.Machine.FailedAttempts} t={r.Time:0.0}";
            });
            Sim.Tally(rs, out int ok, out int n);
            _o.WriteLine($"docking ok {ok}/{n}");
            Assert.True(ok >= 27, Sim.Tally(rs, out _, out _));
            Assert.All(rs, r => Assert.Equal(0, r.UnsafeReleases));
        }
    }
}

namespace Crane.Fleet.Autonomy.Tests
{
    /// <summary>Reproductions of the two CRANE phase-2 failures (estimator overconfidence under real contact). Each runs once with the
    /// safeguards OFF (must reproduce the failure: library believes complete, host rejects) and once with them ON (must not).</summary>
    public class ContactFailureReproductionTests
    {
        private readonly ITestOutputHelper _o;
        public ContactFailureReproductionTests(ITestOutputHelper o) { _o = o; }
        private void Dump(HarnessResult r) { foreach (var e in r.Autonomy.Events) if (e.Kind != "phase" && e.Kind != "connector_cmd") _o.WriteLine(e.ToString()); _o.WriteLine($"believed={r.BelievedComplete} success={r.Success} class={r.Class} {r.Failure}"); }

        [Fact]
        public void WallJam_WithoutSafeguards_LibraryBelievesCompleteButHostRejects()
        {
            var r = Sim.Run(Missions.WallJam(1, safeguards: false, landmarks: false));
            Dump(r);
            Assert.True(r.BelievedComplete);
            Assert.False(r.Success);
            Assert.Equal(FailureClass.HostRejected, r.Class);
            Assert.Contains(r.Collisions, c => c.Contains("blocked"));
        }

        [Fact]
        public void WallJam_WithSafeguardsAndDoorTags_DetectsRelocalisesAndDelivers()
        {
            var rs = new List<HarnessResult>();
            for (ulong seed = 1; seed <= 3; seed++) rs.Add(Sim.Run(Missions.WallJam(seed, safeguards: true, landmarks: true)));
            Dump(rs[0]);
            Sim.Report("wall_jam_safeguards_landmarks", rs, r => $"stall={r.Autonomy.Events.Count(e => e.Kind == "stall_detected")} reloc={r.Autonomy.Events.Count(e => e.Kind == "relocalised")} t={r.Time:0.0}");
            Assert.All(rs, r => Assert.Contains(r.Autonomy.Events, e => e.Kind == "stall_detected"));
            Assert.All(rs, r => Assert.True(r.Success, r.Failure));
        }

        [Fact]
        public void WallJam_WithSafeguardsNoExternalFix_NeverFalselyComplete()
        {
            var r = Sim.Run(Missions.WallJam(2, safeguards: true, landmarks: false));
            Dump(r);
            Assert.Contains(r.Autonomy.Events, e => e.Kind == "stall_detected");
            Assert.True(r.Success || !r.BelievedComplete, "library claimed completion while the world disagrees");
        }

        [Fact]
        public void HandoffBias_WithoutSafeguards_PayloadFallsButLibraryBelievesComplete()
        {
            var r = Sim.Run(Missions.HandoffBias(1, safeguards: false));
            Dump(r);
            Assert.True(r.World.Payloads[0].Dropped);
            Assert.True(r.BelievedComplete);
            Assert.False(r.Success);
        }

        [Fact]
        public void HandoffBias_WithTagAlignmentAndSupportCheck_PlacesOnDeck()
        {
            var rs = new List<HarnessResult>();
            for (ulong seed = 1; seed <= 3; seed++) rs.Add(Sim.Run(Missions.HandoffBias(seed, safeguards: true)));
            Dump(rs[0]);
            Sim.Report("handoff_bias_aligned", rs);
            Assert.All(rs, r => { Assert.True(r.Success, r.Failure); Assert.False(r.World.Payloads[0].Dropped); Assert.Contains(r.Autonomy.Events, e => e.Kind == "support_confirmed"); });
        }

        [Fact]
        public void HandoffBias_LoadCellConfirmsSupport()
        {
            var r = Sim.Run(Missions.HandoffBias(2, safeguards: true, loadCells: true));
            Dump(r);
            Assert.True(r.Success, r.Failure);
            Assert.Contains(r.Autonomy.Events, e => e.Kind == "support_confirmed" && e.Detail.Contains("deck load"));
        }

        [Fact]
        public void HandoffWithoutAlignment_ButSupportCheck_ReportsSupportFailureNotSuccess()
        {
            var s = Missions.HandoffBias(1, safeguards: true);
            s.ConfigHook = c => c.HandoffTagAlignment = false;
            var r = Sim.Run(s);
            Dump(r);
            Assert.True(r.World.Payloads[0].Dropped);
            Assert.False(r.BelievedComplete);
            Assert.Equal(FailureClass.Support, r.Class);
        }
    }
}

namespace Crane.Fleet.Autonomy.Tests
{
    /// <summary>CRANE v2 seed 2: "goal_in_collision" twice at the lift. CRANE adds the payload (box_long 0.80 x 0.25 m at (3.65, 6.0, pi))
    /// to the planning map; the old lift task planned a fixed 0.6 m pre-grasp pose and a 0.237 m grasp pose with the payload box as an
    /// obstacle, so both goals overlapped the object itself (pre-grasp by 1 cm plus the 3 cm margin; grasp by design).</summary>
    public class LiftGeometryTests
    {
        private static PlanMap CraneLikeMap()
        {
            var m = Sim.Warehouse();
            m.AddBox("payload_box_long", 3.65, 6.0, 0.80, 0.25, Math.PI);
            return m;
        }

        [Fact]
        public void OldLiftGoals_AreInCollisionWithThePayloadItself()
        {
            var map = CraneLikeMap(); var cap = Configs.Cap(AssemblyModel.Single(Env.Spec("lifter_1", "lifter")));
            var op = new Pose2(3.65, 6.0, Math.PI);
            var pl = new LatticePlanner(map, new AutonomyAssumptions());
            var start = new Pose2(4.6, 4.6, Math.PI / 2);
            Assert.Equal("goal_in_collision", pl.Plan(new PlanRequest { Start = start, Goal = Se2.Compose(op, new Pose2(-0.6, 0, 0)), Capability = cap }).Failure);
            Assert.Equal("goal_in_collision", pl.Plan(new PlanRequest { Start = start, Goal = Se2.Compose(op, new Pose2(-0.237, 0, 0)), Capability = cap }).Failure);
            // fixed behaviour: goal search finds a nearby free pre-grasp; the grasp move ignores the grasped object
            var pre = pl.Plan(new PlanRequest { Start = start, Goal = Se2.Compose(op, new Pose2(-0.6, 0, 0)), Capability = cap, GoalSearch = true });
            Assert.True(pre.Ok && pre.GoalAdjusted, pre.Failure);
            var grasp = pl.Plan(new PlanRequest { Start = pre.GoalUsed, Goal = Se2.Compose(op, new Pose2(-0.237, 0, 0)), Capability = cap, IgnoreObstacleIds = new[] { "payload_box_long" } });
            Assert.True(grasp.Ok, grasp.Failure);
        }

        /// <summary>CRANE seed 4: after the lift the carry plan started with the lifter overlapping the payload's static map box (still at
        /// its pick pose): start_in_collision. Now the held object leaves the map and inflates the carrier's footprint until release.</summary>
        [Fact]
        public void CarryAfterLift_FromThePickPose_PlansWithHeldObjectInFootprint()
        {
            var s = new HarnessSetup { Map = Sim.Warehouse(), MaxTime = 150 };
            s.PlannerOnlyBoxes.Add(new Box2("payload_box_long", 3.65, 6.0, Math.PI, 0.40, 0.125));
            s.Rovers.Add(new RoverSetup { Id = "lifter_1", Role = "lifter", Truth = new Pose2(4.6, 4.6, Math.PI / 2), Believed = new Pose2(4.6, 4.6, Math.PI / 2) });
            s.WorldInit = w => w.AddPayload("payload_box_long", "box_long", new Pose2(3.65, 6.0, Math.PI)).Mass = 4.5;   // CRANE geometry; mass within the 5 kg lift capacity
            s.MissionJson = @"{""schema"":""crane.fleet_mission/v0"",""tasks"":[
 {""id"":""pick"",""type"":""lift"",""agent"":""lifter_1"",""object"":""payload_box_long"",""object_pose"":[3.65,6.0,3.14159265],""grasp_offset"":0.237,""height"":0.15,""retries"":0},
 {""id"":""carry"",""type"":""goto"",""agent"":""lifter_1"",""goal"":[4.8,4.2,0.0],""retries"":0}]}";
            s.HostCheck = (w, m) => w.R("lifter_1").Holding != "payload_box_long" ? "not holding (truth)" : Sim.DefaultHostCheck(w, m, 0.12);
            var r = Sim.Run(s);
            Assert.True(r.Success, r.Failure + " | " + string.Join("\n", r.Autonomy.Events.Select(e => e.ToString())));
            Assert.Contains(r.Autonomy.Events, e => e.Kind == "object_picked" && e.Detail.Contains("map box removed"));
            Assert.DoesNotContain(r.Autonomy.Config.Map.Obstacles, o => o.Id == "payload_box_long");
            Assert.Contains(r.Autonomy.AgentOf("lifter_1").Cap.Footprint, b => b.Id == "held:payload_box_long");
            Assert.Empty(r.Collisions);
        }

        private static HarnessSetup PickSetup(string payloadType, string liftExtra, int retries)
        {
            var s = new HarnessSetup { Map = Sim.Warehouse(), MaxTime = 150 };
            s.PlannerOnlyBoxes.Add(new Box2("payload", 3.65, 6.0, Math.PI, 0.40, 0.125));
            s.Rovers.Add(new RoverSetup { Id = "lifter_1", Role = "lifter", Truth = new Pose2(4.6, 4.6, Math.PI / 2), Believed = new Pose2(4.6, 4.6, Math.PI / 2) });
            s.WorldInit = w => w.AddPayload("payload", payloadType, new Pose2(3.65, 6.0, Math.PI));
            s.MissionJson = "{\"schema\":\"crane.fleet_mission/v0\",\"tasks\":[{\"id\":\"pick\",\"type\":\"lift\",\"agent\":\"lifter_1\",\"object\":\"payload\","
                + "\"object_pose\":[3.65,6.0,3.14159265],\"grasp_offset\":0.237,\"height\":0.12" + liftExtra + ",\"retries\":" + retries + "}]}";
            s.HostCheck = (w, m) => w.R("lifter_1").Holding == "payload" ? null : "not holding (truth)";
            return s;
        }

        /// <summary>CRANE seed 4 (over-capacity batch): an unconfirmed lift leaves the lifter at the grasp pose inside the object's map box;
        /// the retry plan used to fail with start_in_collision. Now the retry first escapes (ignoring only the contacted object).</summary>
        [Fact]
        public void LiftStallAtGraspPose_RetryEscapesAndSucceeds()
        {
            var s = PickSetup("box_small", "", 1);
            s.Hook = (w, fa, t) => { if (Math.Abs(t - 1.0) < 1e-6) w.R("lifter_1").LiftJamUntil = 30; };   // transient lift jam on the first attempt
            var r = Sim.Run(s);
            var ev = string.Join("\n", r.Autonomy.Events.Select(e => e.ToString()));
            Assert.True(r.Success, r.Failure + " | " + ev);
            Assert.Equal(2, r.Autonomy.Executive.Record("pick").Attempts);
            Assert.Contains(r.Autonomy.Events, e => e.Kind == "escape_move" && e.Detail.Contains("payload"));
            Assert.DoesNotContain(r.Autonomy.Events, e => e.Detail != null && e.Detail.Contains("start_in_collision"));
        }

        [Fact]
        public void OverCapacityLift_FailsFastAsCapacity_NoRetry()
        {
            var r = Sim.Run(PickSetup("box_long", "", 2));          // 6 kg > 5 kg: the lift stalls on its force limit
            Assert.Equal(FailureClass.Capacity, r.Class);
            Assert.Equal(1, r.Autonomy.Executive.Record("pick").Attempts);
            Assert.Contains("over capacity", r.Failure);
            Assert.False(r.World.R("lifter_1").GripCmd);            // gripper commanded open: released, not dragged
            var r2 = Sim.Run(PickSetup("box_long", ",\"payload_type\":\"box_long\"", 2));   // known mass: refused before trying
            Assert.Equal(FailureClass.Capacity, r2.Class);
            Assert.True(r2.Time < 3.0);
        }

        [Fact]
        public void ReleasedObject_LeavesFootprint_AndCanBeReinserted()
        {
            var s = new HarnessSetup { Map = Sim.Warehouse(), MaxTime = 5 };
            s.Rovers.Add(new RoverSetup { Id = "lifter_1", Role = "lifter", Truth = new Pose2(4.6, 4.6, 0), Believed = new Pose2(4.6, 4.6, 0) });
            s.MissionJson = "{\"schema\":\"crane.fleet_mission/v0\",\"tasks\":[{\"id\":\"w\",\"type\":\"wait\",\"duration\":0.1}]}";
            var r = Sim.Run(s); var f = r.Autonomy;
            f.Config.Map.AddBox("obj", 5.0, 4.6, 0.3, 0.2);
            f.ObjectPicked("obj", "lifter_1", new Pose2(0.3, 0, 0));
            Assert.Equal(2, f.AgentOf("lifter_1").Cap.Footprint.Count);
            Assert.DoesNotContain(f.Config.Map.Obstacles, o => o.Id == "obj");
            f.ObjectReleased("obj", new Pose2(5.5, 4.0, 0.2));
            Assert.Single(f.AgentOf("lifter_1").Cap.Footprint);
            Assert.Contains(f.Config.Map.Obstacles, o => o.Id == "obj" && Math.Abs(o.Cx - 5.5) < 1e-9);
        }

        [Fact]
        public void LiftOfLongPayloadInCraneGeometry_Succeeds()
        {
            var s = new HarnessSetup { Map = Sim.Warehouse(), MaxTime = 120 };
            s.PlannerOnlyBoxes.Add(new Box2("payload_box_long", 3.65, 6.0, Math.PI, 0.40, 0.125));
            s.Rovers.Add(new RoverSetup { Id = "lifter_1", Role = "lifter", Truth = new Pose2(4.6, 4.6, Math.PI / 2), Believed = new Pose2(4.6, 4.6, Math.PI / 2) });
            s.WorldInit = w => w.AddPayload("payload_box_long", "box_long", new Pose2(3.65, 6.0, Math.PI)).Mass = 4.5;   // CRANE geometry; mass within the 5 kg lift capacity
            s.MissionJson = @"{""schema"":""crane.fleet_mission/v0"",""tasks"":[{""id"":""pick"",""type"":""lift"",""agent"":""lifter_1"",""object"":""payload_box_long"",
 ""object_pose"":[3.65,6.0,3.14159265],""grasp_offset"":0.237,""height"":0.15,""retries"":0}]}";
            s.HostCheck = (w, m) => w.R("lifter_1").Holding == "payload_box_long" ? null : "lifter not holding the payload (truth)";
            var r = Sim.Run(s);
            Assert.True(r.Success, r.Failure + " | " + string.Join("\n", r.Autonomy.Events.Select(e => e.ToString())));
        }
    }
}
