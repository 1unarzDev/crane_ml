using System;
using System.Collections.Generic;
using System.Linq;
using Crane.Fleet.Autonomy.Tests.Harness;
using Xunit;

namespace Crane.Fleet.Autonomy.Tests
{
    public static class Configs
    {
        public static RoverSpec S(string id) => Env.Spec(id, "cargo");
        public static AssemblyModel Single() => AssemblyModel.Single(S("a"));
        public static AssemblyModel Line() => AssemblyModel.Merge(AssemblyModel.Single(S("a")), "a", "rear", AssemblyModel.Single(S("b")), "b", "front", "line");
        public static AssemblyModel Pair() => AssemblyModel.Merge(AssemblyModel.Single(S("a")), "a", "left", AssemblyModel.Single(S("b")), "b", "right", "pair");
        public static AssemblyModel L() => AssemblyModel.Merge(Line(), "a", "left", AssemblyModel.Single(S("c")), "c", "rear", "L");
        public static AssemblyModel Get(string name, string fail = "ok")
        {
            var m = name switch { "single" => Single(), "line" => Line(), "pair" => Pair(), "L" => L(), _ => throw new ArgumentException(name) };
            if (fail == "locked") m.Modules[m.Modules.Count - 1].Health[0] = CapHealth.DeadLocked;
            if (fail == "free") m.Modules[0].Health[1] = CapHealth.DeadFree;
            if (fail == "weak") m.Modules[0].Health[2] = CapHealth.Weak(0.5);
            return m;
        }
        public static AutonomyAssumptions A => new AutonomyAssumptions();
        public static AssemblyCapability Cap(AssemblyModel m) => (AssemblyCapability)new CapabilityLibraryBuilder().Build(m, A);
    }

    public class CapabilityTests
    {
        [Fact]
        public void SingleRoverRows_MatchCraneInverseKinematics()
        {
            var s = Configs.S("a"); var cap = Configs.Cap(Configs.Single());
            double k = s.Lx + s.Ly, r = s.WheelRadius; var xi = new Twist2(0.2, -0.1, 0.3);
            double[] expect = { (xi.Vx - xi.Vy - k * xi.Wz) / r, (xi.Vx + xi.Vy + k * xi.Wz) / r, (xi.Vx + xi.Vy - k * xi.Wz) / r, (xi.Vx - xi.Vy + k * xi.Wz) / r };
            for (int i = 0; i < 4; i++) Assert.Equal(expect[i], cap.WheelRate(i, xi), 9);
        }

        [Theory]
        [InlineData("single", "ok")] [InlineData("single", "locked")] [InlineData("single", "free")] [InlineData("line", "ok")]
        [InlineData("line", "locked")] [InlineData("pair", "weak")] [InlineData("L", "ok")] [InlineData("L", "locked")]
        public void LibraryAndAnalyticBuilders_Agree(string cfg, string fail)
        {
            var m = Configs.Get(cfg, fail);
            var a = (AssemblyCapability)new CapabilityLibraryBuilder().Build(m, Configs.A);
            var b = (AssemblyCapability)new AnalyticCapabilityBuilder().Build(m, Configs.A);
            Assert.Equal(a.ActuatedRank, b.ActuatedRank);
            for (int i = 0; i < 9; i++) Assert.Equal(a.P[i], b.P[i], 6);
            for (int i = 0; i < a.WheelCount; i++) Assert.Equal(a.WheelLimit(i), b.WheelLimit(i), 9);
            foreach (var u in new[] { new Twist2(1, 0, 0), new Twist2(0, 1, 0), new Twist2(0.3, 0.2, 1) })
                Assert.Equal(a.MaxAlong(u), b.MaxAlong(u), 6);
        }

        [Theory]
        [InlineData("single", "ok", 3)] [InlineData("single", "free", 3)] [InlineData("single", "locked", 2)] [InlineData("line", "locked", 2)] [InlineData("pair", "ok", 3)]
        public void Rank_FollowsHealth(string cfg, string fail, int rank) => Assert.Equal(rank, Configs.Cap(Configs.Get(cfg, fail)).ActuatedRank);

        [Fact]
        public void TwoLockedWheels_SingleRover_LeaveOneDof()
        {
            var m = Configs.Single(); m.Modules[0].Health[0] = CapHealth.DeadLocked; m.Modules[0].Health[3] = CapHealth.DeadLocked;
            Assert.Equal(1, Configs.Cap(m).ActuatedRank);
        }

        [Fact]
        public void WeakWheel_ScalesBound_BodyLimitCapsSpeed()
        {
            var ok = Configs.Cap(Configs.Single());
            Assert.Equal(ok.MaxSpeedXy, ok.MaxAlong(new Twist2(1, 0, 0)), 9);          // body limit 0.5 m/s binds before wheels (0.58)
            var m = Configs.Single(); m.Modules[0].Health[0] = CapHealth.Weak(0.3);
            var w = Configs.Cap(m);
            Assert.True(w.MaxAlong(new Twist2(1, 0, 0)) < 0.5 * ok.MaxAlong(new Twist2(1, 0, 0)));
        }

        [Fact]
        public void LockedWheel_FeasibleRejectsConstraintViolation_ProjectionSatisfiesIt()
        {
            var cap = Configs.Cap(Configs.Get("single", "locked"));
            var bad = new Twist2(0.2, 0, 0);   // fl must turn for pure vx
            Assert.False(cap.Feasible(bad));
            var p = cap.Project(bad);
            Assert.True(Math.Abs(cap.WheelRate(4 - 4, p)) < 1e-9 || cap.WheelMode(0) != CapHealthMode.DeadLocked);
            Assert.True(cap.Feasible(p * Math.Min(1, cap.MaxAlong(p))));
        }

        [Fact]
        public void Merge_PlacesModulesAtMatedPoses()
        {
            var line = Configs.Line(); var pair = Configs.Pair();
            Assert.Equal(-0.42, line.Modules[1].InAssembly.X, 6); Assert.Equal(0, line.Modules[1].InAssembly.Y, 6);
            Assert.Equal(0.40, pair.Modules[1].InAssembly.Y, 6); Assert.Equal(0, Se2.Wrap(pair.Modules[1].InAssembly.Yaw), 6);
            var l = Configs.L(); Assert.Equal(3, l.Modules.Count);
            Assert.Equal(Math.PI / 2, Se2.Wrap(l.Modules[2].InAssembly.Yaw), 6);
        }

        [Fact]
        public void Without_RebasesRemainder()
        {
            var line = Configs.Line();
            var rest = line.Without(new[] { "a" }, "rest", out var frame);
            Assert.Single(rest.Modules); Assert.Equal("b", rest.Modules[0].Spec.Id);
            Assert.Equal(0, rest.Modules[0].InAssembly.X, 9); Assert.Equal(-0.42, frame.X, 9);
        }

        [Fact]
        public void Footprint_IncludesPayloadAndModules()
        {
            var m = Configs.Line(); m.ExtraFootprint.Add(new Box2("payload", -0.21, 0, 0, 0.4, 0.125));
            var cap = Configs.Cap(m);
            Assert.Equal(3, cap.Footprint.Count);
        }
    }

    public class PlannerTests
    {
        private static LatticePlanner P(PlanMap m) => new LatticePlanner(m, new AutonomyAssumptions());

        [Theory]
        [InlineData("single", "ok")] [InlineData("single", "locked")] [InlineData("line", "ok")] [InlineData("line", "locked")] [InlineData("pair", "ok")] [InlineData("L", "weak")]
        public void EveryPrimitive_IsInsideTheTwistSet(string cfg, string fail)
        {
            var cap = Configs.Cap(Configs.Get(cfg, fail));
            var prims = P(Sim.Warehouse()).Primitives(cap, 0.6);
            Assert.NotEmpty(prims);
            foreach (var p in prims) Assert.True(cap.Feasible(p.Twist, 1e-6), $"{cfg}/{fail}: {p.Twist}");
            if (cap.ActuatedRank < 3) Assert.True(prims.Count >= 4);
        }

        [Fact]
        public void PlannedPath_IsCollisionFree_AndEverySegmentFeasible()
        {
            var map = Sim.Warehouse(); var cap = Configs.Cap(Configs.Single());
            var path = P(map).Plan(new PlanRequest { Start = new Pose2(1.5, 1.2, 0), Goal = new Pose2(8.5, 3.5, 0), Capability = cap });
            Assert.True(path.Ok, path.Failure);
            foreach (var pt in path.Points)
            {
                Assert.False(map.Collides(pt.Pose, cap.Footprint, 0.0));
                Assert.True(cap.Feasible(pt.Twist, 1e-6), pt.Twist.ToString());
            }
            Assert.True(Se2.Distance(path.Points.Last().Pose, new Pose2(8.5, 3.5, 0)) < 0.07);
        }

        [Fact]
        public void SideBySidePair_CrabsThroughNarrowDoor_KeepingHeading()
        {
            var map = Sim.Warehouse(0.65); var cap = Configs.Cap(Configs.Pair());
            var path = P(map).Plan(new PlanRequest { Start = new Pose2(4.3, 3.0, Math.PI / 2), Goal = new Pose2(7.6, 3.3, Math.PI / 2), Capability = cap });
            Assert.True(path.Ok, path.Failure);
            double maxYawDev = path.Points.Max(p => Math.Abs(Se2.Wrap(p.Pose.Yaw - Math.PI / 2)));
            Assert.True(maxYawDev < 0.25, "yaw deviation " + maxYawDev);
            // crossing the wall plane: motion is lateral in the pair's frame
            var cross = path.Points.Where(p => Math.Abs(p.Pose.X - 6.0) < 0.3).ToList();
            Assert.NotEmpty(cross);
            Assert.All(cross, p => Assert.True(Math.Abs(p.Twist.Vy) >= Math.Abs(p.Twist.Vx) - 1e-9 || p.Twist.LinearSpeed < 1e-6));
        }

        [Fact]
        public void SideBySidePair_HeadingForward_CannotPassWithoutTurning_AndForwardOnlyCannot()
        {
            var map = Sim.Warehouse(0.65);
            var pair = Configs.Pair();
            // forward/backward only (two locked wheels per module leave one dof): no route through the doorway
            foreach (var m in pair.Modules) { m.Health[0] = CapHealth.DeadLocked; m.Health[3] = CapHealth.DeadLocked; }
            var cap = Configs.Cap(pair);
            Assert.True(cap.ActuatedRank <= 1);
            var path = new LatticePlanner(map, new AutonomyAssumptions { PlanMaxExpansions = 8000 }).Plan(new PlanRequest { Start = new Pose2(4.3, 3.0, 0), Goal = new Pose2(7.6, 3.3, 0), Capability = cap });
            Assert.False(path.Ok);
        }

        [Fact]
        public void Line_FitsHalfMetreDoor_NotA44cmDoor()
        {
            var cap = Configs.Cap(Configs.Line());
            Assert.True(P(Sim.Warehouse(0.50)).Plan(new PlanRequest { Start = new Pose2(4.0, 3.5, 0), Goal = new Pose2(8.0, 3.5, 0), Capability = cap }).Ok);
            var narrow = new LatticePlanner(Sim.Warehouse(0.44), new AutonomyAssumptions { PlanMaxExpansions = 8000 }).Plan(new PlanRequest { Start = new Pose2(4.0, 3.5, 0), Goal = new Pose2(8.0, 3.5, 0), Capability = cap });
            Assert.False(narrow.Ok);
        }

        [Fact]
        public void GoalOrStartInCollision_Reported()
        {
            var map = Sim.Warehouse(); var cap = Configs.Cap(Configs.Single());
            Assert.Equal("goal_in_collision", P(map).Plan(new PlanRequest { Start = new Pose2(1, 1, 0), Goal = new Pose2(6.0, 1.0, 0), Capability = cap }).Failure);
            Assert.Equal("start_in_collision", P(map).Plan(new PlanRequest { Start = new Pose2(4.6, 1.6, 0), Goal = new Pose2(1, 1, 0), Capability = cap }).Failure);
        }

        [Fact]
        public void LockedWheel_PlanStillReachesGoal_WithinRelaxedTolerance()
        {
            var cap = Configs.Cap(Configs.Get("single", "locked"));
            var path = P(Sim.Warehouse()).Plan(new PlanRequest { Start = new Pose2(1.5, 1.5, 0), Goal = new Pose2(3.5, 3.0, 0), Capability = cap });
            Assert.True(path.Ok, path.Failure);
            foreach (var pt in path.Points) Assert.True(cap.Feasible(pt.Twist, 1e-6));
            Assert.True(Se2.Distance(path.Points.Last().Pose, new Pose2(3.5, 3.0, 0)) < 0.15);
        }

        [Fact]
        public void Planner_IsDeterministic()
        {
            var cap = Configs.Cap(Configs.Line());
            var req = new PlanRequest { Start = new Pose2(2, 2, 0.3), Goal = new Pose2(8.4, 3.5, 0), Capability = cap };
            var a = P(Sim.Warehouse()).Plan(req); var b = P(Sim.Warehouse()).Plan(req);
            Assert.Equal(a.Points.Count, b.Points.Count);
            for (int i = 0; i < a.Points.Count; i++) { Assert.Equal(a.Points[i].Pose.X, b.Points[i].Pose.X); Assert.Equal(a.Points[i].Pose.Yaw, b.Points[i].Pose.Yaw); }
        }
    }
}
