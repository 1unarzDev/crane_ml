using System;
using Crane.Fleet;
using Xunit;

namespace Crane.Fleet.Capability.Tests
{
    public class MotionTests
    {
        private static CapAssembly Mecanum(double lx = 0.15, double ly = 0.12, double r = 0.05, double vmax = 20)
        {
            var a = new CapAssembly { Id = "mec" };
            a.Modules.Add(CapabilityBuilders.MecanumRover("r", lx, ly, r, vmax));
            return a;
        }

        [Fact]
        public void MecanumX_IsHolonomicWithTextbookInverseKinematics()
        {
            var c = MotionAnalyzer.Analyze(Mecanum());
            Assert.Equal(0, c.RankC); Assert.Equal(3, c.Mobility); Assert.Equal(3, c.ActuatedRank); Assert.True(c.Holonomic);
            // standard IK: fl = (vx - vy - (lx+ly) wz)/r ; fr = (vx + vy + (lx+ly) wz)/r ; rl = (vx + vy - (lx+ly) wz)/r ; rr = (vx - vy + (lx+ly) wz)/r
            double k = 0.15 + 0.12, r = 0.05;
            var w = c.WheelSpeeds(new[] { 0.3, 0.2, 0.5 });
            Assert.Equal((0.3 - 0.2 - k * 0.5) / r, w[0], 9);
            Assert.Equal((0.3 + 0.2 + k * 0.5) / r, w[1], 9);
            Assert.Equal((0.3 + 0.2 - k * 0.5) / r, w[2], 9);
            Assert.Equal((0.3 - 0.2 + k * 0.5) / r, w[3], 9);
            // round trip through B
            var xi = c.TwistFromWheelSpeeds(w);
            Assert.Equal(0.3, xi[0], 9); Assert.Equal(0.2, xi[1], 9); Assert.Equal(0.5, xi[2], 9);
        }

        [Fact]
        public void MecanumAxisBoundsMatchClosedForm()
        {
            var c = MotionAnalyzer.Analyze(Mecanum(vmax: 20));
            var b = c.AxisBounds();
            Assert.Equal(20 * 0.05, b[0], 9);               // pure forward: |vx|/r <= vmax
            Assert.Equal(20 * 0.05, b[2], 9);               // pure lateral
            Assert.Equal(20 * 0.05 / (0.15 + 0.12), b[4], 9); // pure yaw
        }

        [Fact]
        public void MecanumDeadFreeWheel_StaysHolonomic_ButLosesRedundancy()
        {
            var c = MotionAnalyzer.Analyze(Mecanum().WithHealth("r.fl", CapHealth.DeadFree));
            Assert.Equal(3, c.Mobility); Assert.Equal(3, c.ActuatedRank); Assert.Equal(3, c.DrivenWheelIds.Length);
            Assert.True(c.AxisBounds()[0] > 0);
        }

        [Fact]
        public void MecanumDeadLocked_RemovesOneDegreeOfMobility_DeadFreeDoesNot()
        {
            var locked = MotionAnalyzer.Analyze(Mecanum().WithHealth("r.fl", CapHealth.DeadLocked));
            Assert.Equal(1, locked.RankC); Assert.Equal(2, locked.Mobility); Assert.Equal(2, locked.ActuatedRank);
            Assert.False(locked.Holonomic);
            var free = MotionAnalyzer.Analyze(Mecanum().WithHealth("r.fl", CapHealth.DeadFree));
            Assert.Equal(0, free.RankC);
            // the probe that separates the hypotheses: some twist is feasible under dead_free but not under dead_locked
            var probe = new[] { 0.0, 0.0, 1.0 };
            Assert.True(free.MaxAlong(probe) > 0);
            // the locked wheel's direction (a.v_c = 0): twist violating it is infeasible
            double[] bad = null;
            foreach (var u in new[] { new[] { 1.0, 0, 0 }, new[] { 0, 1.0, 0 }, new[] { 0, 0, 1.0 } })
                if (locked.MaxAlong(u) == 0) bad = u;
            Assert.NotNull(bad);
        }

        [Fact]
        public void MecanumTwoWheelsDead_LeavesTwoRowsAndLosesHolonomy()
        {
            var a = Mecanum().WithHealth("r.fl", CapHealth.DeadFree).WithHealth("r.rr", CapHealth.DeadFree);
            var c = MotionAnalyzer.Analyze(a);
            Assert.Equal(3, c.Mobility); Assert.Equal(2, c.ActuatedRank); Assert.Equal(1, c.Uncontrolled);
        }

        [Fact]
        public void WeakWheel_ScalesBoundsAndNeverRaisesThem()
        {
            var full = MotionAnalyzer.Analyze(Mecanum());
            var weak = MotionAnalyzer.Analyze(Mecanum().WithHealth("r.fr", CapHealth.Weak(0.5)));
            var bf = full.AxisBounds(); var bw = weak.AxisBounds();
            for (int i = 0; i < 6; i++) Assert.True(bw[i] <= bf[i] + 1e-12);
            Assert.True(bw[0] < bf[0]); // forward needs fr at full speed
            Assert.Equal(3, weak.Mobility);
            var zero = MotionAnalyzer.Analyze(Mecanum().WithHealth("r.fr", CapHealth.Weak(0.0)));
            Assert.Equal(0.0, zero.AxisBounds()[0], 12); // gain 0 pins the wheel: forward needs it
        }

        [Fact]
        public void DifferentialPod_HardConstraint_IsNonholonomic()
        {
            var a = new CapAssembly();
            a.Modules.Add(CapabilityBuilders.DifferentialPod("p", 0, 0, 0));
            var c = MotionAnalyzer.Analyze(a);
            Assert.Equal(1, c.RankC); Assert.Equal(2, c.Mobility); Assert.Equal(2, c.ActuatedRank);
            Assert.Equal(0.0, c.MaxAlong(new[] { 0.0, 1.0, 0 }), 12); // no sideways motion
            var b = c.AxisBounds();
            Assert.Equal(24 * 0.055, b[0], 9);
            Assert.Equal(24 * 0.055 / 0.07, b[4], 9);   // pure spin: |wz| * 0.07 / r <= vmax
            // vx and wz are coupled: full speed forward AND full spin is infeasible
            Assert.False(c.Feasible(new[] { 24 * 0.055, 0, 24 * 0.055 / 0.07 }));
        }

        [Fact]
        public void DifferentialPod_DeadFreeWheel_LeavesOneCommandableDirection()
        {
            var a = new CapAssembly();
            a.Modules.Add(CapabilityBuilders.DifferentialPod("p", 0, 0, 0));
            var c = MotionAnalyzer.Analyze(a.WithHealth("p.l", CapHealth.DeadFree));
            Assert.Equal(1, c.RankC); Assert.Equal(2, c.Mobility); Assert.Equal(1, c.ActuatedRank); Assert.Equal(1, c.Uncontrolled);
            var l = MotionAnalyzer.Analyze(a.WithHealth("p.l", CapHealth.DeadLocked));
            Assert.Equal(2, l.RankC); Assert.Equal(1, l.Mobility); // pivot about the locked wheel: one dof (arc)
            Assert.Equal(1, l.ActuatedRank);
        }

        [Fact]
        public void SixPodGazeboPlatforms_HardNoSlipPredictsNoTurning_SkidDoes()
        {
            foreach (var name in new[] { "compact_diff", "narrow_tandem" })
            {
                var hard = MotionAnalyzer.Analyze(CapabilityBuilders.GazeboSixPod(name, CapLateralMode.Hard));
                Assert.Equal(2, hard.RankC);                  // rows vy + wz*px for distinct px
                Assert.Equal(1, hard.Mobility);               // forward/back only
                Assert.Equal(0.0, hard.MaxAlong(new[] { 0.0, 0, 1 }), 12);
                var skid = MotionAnalyzer.Analyze(CapabilityBuilders.GazeboSixPod(name, CapLateralMode.Skid));
                Assert.Equal(0, skid.RankC); Assert.Equal(2, skid.ActuatedRank); // vx and wz commandable, vy uncontrolled
                Assert.True(skid.MaxAlong(new[] { 0.0, 0, 1 }) > 0);
                Assert.Equal(12, skid.Soft.Length);
                // yaw scrub torque grows with lever arm: the narrow platform scrubs far more at the same yaw rate
                var w = skid.ScrubWrench(new[] { 0.0, 0, 0.25 });
                Assert.True(Math.Abs(w[2]) > 0);
            }
            var cN = MotionAnalyzer.Analyze(CapabilityBuilders.GazeboSixPod("narrow_tandem", CapLateralMode.Skid));
            var cC = MotionAnalyzer.Analyze(CapabilityBuilders.GazeboSixPod("compact_diff", CapLateralMode.Skid));
            Assert.True(Math.Abs(cN.ScrubWrench(new[] { 0.0, 0, 0.25 })[2]) > 2 * Math.Abs(cC.ScrubWrench(new[] { 0.0, 0, 0.25 })[2]));
            // kinematic limits: spin bound is set by the farthest wheel pair of the same side lever (0.20 +- 0.07)
            Assert.Equal(24 * 0.055 / 0.27, cC.AxisBounds()[4], 9);
        }

        [Fact]
        public void Caster_ImposesNothing_AndDoesNotAppearInActuation()
        {
            var a = new CapAssembly();
            var m = CapabilityBuilders.DifferentialPod("p", 0, 0, 0);
            m.Wheels.Add(new CapWheel { Id = "p.c", Kind = CapWheelKind.Caster, X = 0.2, Y = 0 });
            a.Modules.Add(m);
            var c = MotionAnalyzer.Analyze(a);
            Assert.Equal(2, c.DrivenWheelIds.Length); Assert.Equal(1, c.RankC);
        }

        [Fact]
        public void ModulePoseComposition_RotatingTheAssemblyRotatesTheCapability()
        {
            var a = Mecanum(); a.Modules[0].Yaw = Math.PI / 2;
            var c = MotionAnalyzer.Analyze(a);
            var b = c.AxisBounds();
            Assert.Equal(1.0, b[0], 9); // vx in assembly frame equals module vy: 20*0.05 = 1.0
            var d = new CapAssembly();
            d.Modules.Add(CapabilityBuilders.DifferentialPod("p", 0, 0, Math.PI / 2));
            var cd = MotionAnalyzer.Analyze(d);
            Assert.Equal(0.0, cd.MaxAlong(new[] { 1.0, 0, 0 }), 12);   // pod points along +y of the assembly
            Assert.True(cd.MaxAlong(new[] { 0.0, 1.0, 0 }) > 0);
        }

        [Fact]
        public void JsonDescription_IsWellFormedAndNamesProvenance()
        {
            var c = MotionAnalyzer.Analyze(Mecanum().WithHealth("r.fl", CapHealth.DeadLocked));
            var json = c.ToJson();
            using var doc = System.Text.Json.JsonDocument.Parse(json);
            Assert.Equal(2, doc.RootElement.GetProperty("degree_of_mobility").GetInt32());
            Assert.Equal("analytic", doc.RootElement.GetProperty("provenance").GetString());
            Assert.Equal("locked", doc.RootElement.GetProperty("hard_constraints")[0].GetProperty("role").GetString());
        }

        [Fact]
        public void IntermittentWheel_ExpectedAuthorityVersusWorstCase()
        {
            var a = Mecanum().WithHealth("r.fl", CapHealth.Intermittent(0.5));
            var e = MotionAnalyzer.Analyze(a);
            var w = MotionAnalyzer.Analyze(a, new CapOptions { IntermittentWorstCase = true });
            Assert.Equal(4, e.DrivenWheelIds.Length); Assert.Equal(3, w.DrivenWheelIds.Length);
            Assert.True(e.AxisBounds()[0] <= MotionAnalyzer.Analyze(Mecanum()).AxisBounds()[0] + 1e-12);
        }
    }
}
