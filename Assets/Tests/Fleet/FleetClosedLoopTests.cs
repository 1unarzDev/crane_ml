using System;
using System.Collections.Generic;
using Crane.Fleet;
using Crane.Fleet.Autonomy;
using Crane.Fleet.Docking;
using Crane.Fleet.Perception;
using Crane.Fleet.Perception.Evaluator;
using Crane.Fleet.Perception.Rover;
using NUnit.Framework;

namespace Crane.Fleet.Tests {
    public class FleetClosedLoopTests {
        const string SpecDir = "Assets/StreamingAssets/Fleet";

        [Test]
        public void WeightedForwardKinematicsRecoversTwistFromThreeWheels() {
            var g = FleetSpec.LoadDirectory(SpecDir).Describe("cargo_1", "cargo").Geometry;
            var w = new double[4];
            var t = new Twist(0.2, 0.1, 0.3);
            g.Inverse(t, w);
            for (int drop = 0; drop < 4; drop++) {
                var weight = new double[] { 1, 1, 1, 1 }; weight[drop] = 0;
                Twist b = g.ForwardWeighted(w, weight);
                Assert.AreEqual(t.Vx, b.Vx, 1e-9); Assert.AreEqual(t.Vy, b.Vy, 1e-9); Assert.AreEqual(t.Wz, b.Wz, 1e-9);
            }
        }

        static Twist Drive(WheelHealth h, int wheel) {
            var spec = FleetSpec.LoadDirectory(SpecDir);
            var d = spec.Describe("cargo_1", "cargo");
            var p = PlantParams.From(spec, d, spec.Floor()); p.SlipSigma = 0; p.LateralEfficiency = 1;
            var plant = new MecanumPlant(d.Geometry, p, new FleetRng(1));
            plant.Health[wheel] = h;
            var v = new Twist();
            for (int i = 0; i < 4000; i++) { plant.Step(new Twist(0.2, 0, 0), v, 0.005); v.Vx += plant.Ax * 0.005; v.Vy += plant.Ay * 0.005; v.Wz += plant.Alpha * 0.005; }
            return v;
        }

        [Test]
        public void DeadFreeWheelKeepsControlDeadLockedWheelDoesNot() {
            var free = Drive(WheelHealth.DeadFree, 0);
            Assert.AreEqual(0.2, free.Vx, 1e-6); Assert.AreEqual(0.0, free.Wz, 1e-6);
            var locked = Drive(WheelHealth.DeadLocked, 0);
            Assert.Greater(Math.Abs(locked.Wz), 0.05, "a locked wheel must turn the body under a straight command");
        }

        sealed class MockIO : IRoverIO {
            public string Id { get; set; } public string Role => "cargo"; public RoverDescription Description { get; set; } public IObservationProvider Sensors { get; set; }
            public double X, Y, Yaw; public Twist Cmd; public string State = "FREE"; public DockingCommand Last; public bool Coupled;
            public OdometryEstimate Odometry => new OdometryEstimate { X = X + 0.03, Y = Y - 0.02, Yaw = Yaw };
            public double LiftPosition => 0; public double GripperOpening => 0; public bool GripperHolding => false;
            public IReadOnlyList<string> Faces => new[] { "front", "rear" };
            public ConnectorStatus Connector(string f) => new ConnectorStatus { Face = f, State = State, SensedA = State == "HOOK_A_ENGAGED", SensedB = State == "HOOK_A_ENGAGED", Coupled = Coupled };
            public void CommandTwist(Twist t) { Cmd = t; } public void CommandLift(double q) { } public void CommandGripper(double o) { }
            public void CommandConnector(string f, DockingCommand c) { Last = c; } public void Report(string c, string p) { }
        }

        [Test]
        public void TagDockingControllerReachesTheCaptureEnvelopeFromObservationsOnly() {
            var spec = FleetSpec.LoadDirectory(SpecDir);
            var lead = spec.Describe("cargo_1", "cargo");
            double lx = 4.8, ly = 3.52, lyaw = 0.03;
            var io = new MockIO { Id = "cargo_2", Description = spec.Describe("cargo_2", "cargo"), X = 3.71, Y = 3.45, Yaw = -0.05 };
            var truth = new EvaluatorTruth();
            truth.SetTrajectory("cargo_1", t => new RoverTruthState { Pose = Pose.FromYaw(lx, ly, 0, lyaw) });
            truth.SetTrajectory("cargo_2", t => new RoverTruthState { Pose = Pose.FromYaw(io.X, io.Y, 0, io.Yaw) });
            foreach (var id in new[] { "scout_1", "lifter_1" }) truth.SetTrajectory(id, t => new RoverTruthState { Pose = Pose.FromYaw(1, 1, 0, 0) });
            var sim = new FleetSimulator(FleetProfile.Parse(System.IO.File.ReadAllText(SpecDir + "/fleet_profile_v0.json")),
                Scenario.Parse(System.IO.File.ReadAllText(SpecDir + "/scenarios/warehouse_a_v0.json")), truth, 7);
            io.Sensors = sim.ProviderFor("cargo_2");
            var ctl = new TagDockingController(io, "cargo_1", lead, "front", "rear", spec.O["demo.cargo.tag_docking"]);
            bool inside = false; double dt = 0.005;
            for (int i = 0; i < 8000 && !inside; i++) {
                ctl.Step(i * dt);
                double c = Math.Cos(io.Yaw), s = Math.Sin(io.Yaw);
                io.X += (c * io.Cmd.Vx - s * io.Cmd.Vy) * dt; io.Y += (s * io.Cmd.Vx + c * io.Cmd.Vy) * dt; io.Yaw += io.Cmd.Wz * dt;
                if (io.Last == DockingCommand.Approach && io.State == "FREE") io.State = "APPROACH";
                double fx = io.X + Math.Cos(io.Yaw) * 0.21, fy = io.Y + Math.Sin(io.Yaw) * 0.21, rx = lx - Math.Cos(lyaw) * 0.21, ry = ly - Math.Sin(lyaw) * 0.21;
                double ca = Math.Cos(lyaw + Math.PI), sa = Math.Sin(lyaw + Math.PI), dx = fx - rx, dy = fy - ry;
                double gap = ca * dx + sa * dy, lat = -sa * dx + ca * dy, yerr = Angles.Wrap(io.Yaw - lyaw);
                if (io.State == "APPROACH" && gap <= 0.008) { Assert.Less(Math.Abs(lat), 0.01, "lateral at capture"); Assert.Less(Math.Abs(yerr), 0.087, "yaw at capture"); inside = true; }
            }
            Assert.IsTrue(inside, $"reached capture range (phase {ctl.State}, observations {ctl.ObservationsUsed})");
        }
    }
}
