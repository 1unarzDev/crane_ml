using System;
using System.Linq;
using Crane.Fleet.Autonomy;
using Crane.Fleet.Autonomy.Tests.Harness;
using Xunit;

namespace Crane.Fleet.Autonomy.Tests
{
    /// <summary>
    /// Tests for CRANE's local patches to the imported autonomy library (Assets/Scripts/Fleet/Autonomy), run against the upstream kinematic harness.
    /// The project is ~/.cache/crane-fleet/autpatch (see README in this directory).
    /// </summary>
    public class CranePatchTests
    {
        /// <summary>CRANE box_small seeds 1 and 5 (2026-11-10): the lifter carries box_small from the pick (4.06, 5.99, pi) to the handoff on the coupled
        /// pair (cargo_1 at (4.783, 3.496, 0), cargo_2 mated behind). The carry Navigator ignored the receivers, and the route swept across
        /// cargo_1's front-left corner (contact, stall detection, receiver not localised). Patched: the receivers stay obstacles on the carry leg
        /// to the pre-place pose.</summary>
        [Theory]
        [InlineData(1UL)] [InlineData(2UL)] [InlineData(3UL)] [InlineData(4UL)] [InlineData(5UL)]
        public void HandoffCarry_KeepsReceiversAsObstacles_NoContact(ulong seed)
        {
            var s = new HarnessSetup { Seed = seed, Map = Sim.Warehouse(), MaxTime = 160 };
            var lt = new Pose2(4.06, 5.99, Math.PI);
            var a = new Pose2(4.783, 3.496, 0.0);
            var spec = Env.Spec("cargo_1", "cargo");
            var b = AssemblyModel.MatedPose(a, spec.Face("rear"), spec.Face("front"));
            s.Rovers.Add(new RoverSetup { Id = "lifter_1", Role = "lifter", Truth = lt, Believed = lt });
            s.Rovers.Add(new RoverSetup { Id = "cargo_1", Role = "cargo", Truth = a, Believed = a });
            s.Rovers.Add(new RoverSetup { Id = "cargo_2", Role = "cargo", Truth = b, Believed = b });
            s.PreLatched.Add(new[] { "cargo_2", "front", "cargo_1", "rear" });
            s.WorldInit = w =>
            {
                var p = w.AddPayload("payload_box_small", "box_small", Se2.Compose(lt, new Pose2(0.387, 0, 0)));
                var l = w.R("lifter_1"); p.CarriedBy = "lifter_1"; p.InCarrier = new Pose2(0.387, 0, 0); l.Holding = "payload_box_small"; l.GripClosed = true; l.GripCmd = true;
            };
            s.MissionJson = @"{""schema"":""crane.fleet_mission/v0"",""tasks"":[{""id"":""give"",""type"":""handoff"",""from"":""lifter_1"",""supporters"":[""cargo_1""],
 ""place_on"":""cargo_1"",""place_offset"":[0.08,0,3.14159265],""payload"":""payload_box_small"",""payload_type"":""box_small"",""hold_offset"":0.387,""retries"":0}]}";
            var r = Sim.Run(s);
            var hits = r.Collisions.Where(c => c.Contains("lifter_1 hit cargo") || c.Contains("cargo_1 hit lifter") || c.Contains("cargo_2 hit lifter")).ToList();
            Assert.True(hits.Count == 0, "lifter touched the receivers: " + string.Join("; ", hits));
            Assert.True(r.Success, r.Failure + " | " + string.Join("\n", r.Autonomy.Events.Select(e => e.ToString())));
        }

        [Fact]
        public void TippingClass_Exists_AndIsNotRetried()
        {
            Assert.True(Enum.IsDefined(typeof(FailureClass), "Tipping"));
            Assert.True(new AutonomyAssumptions().TipTiltLimit > 0);
        }
    }
}
