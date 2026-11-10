using System;
using System.IO;
using Crane.Drones.Logging;
using NUnit.Framework;
using Sim.Physics.Aerial;
using UnityEngine;

namespace Crane.Drones.Integration.Tests {
    /// <summary>The real aerial plant drives the logger: state rows at 1 kHz, IMU from the plant at 500 Hz, monotonic stamps.</summary>
    public class AerialPlantLoggingTests {
        string dir;

        [SetUp] public void SetUp() => dir = Path.Combine(Path.GetTempPath(), "crane_aerial_log_" + Guid.NewGuid().ToString("N"));
        [TearDown] public void TearDown() { if (Directory.Exists(dir)) Directory.Delete(dir, true); }

        static DroneSplits Splits() => new DroneSplits {
            schema = "crane-drone-splits/1",
            seed_ranges = new[] { new DroneSeedRange { split = "dev", start = 0, end = 1999 } },
            disturbance = new[] {
                new DroneDisturbanceSet { family = "wind", split = "dev", values = new[] { "calm" } },
                new DroneDisturbanceSet { family = "lighting", split = "dev", values = new[] { "GoldenHour" } } }
        };

        static (CraneAerialPlantCore, CraneAerialProfile, CraneMultirotorController) Plant(int seed) {
            string profDir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "Tools", "Aerial", "profiles"));
            var prof = CraneAerialProfile.LoadFile(CraneAerialProfile.Resolve("aos7_v5_f90_6s", profDir), true, true);
            var plant = new CraneAerialPlantCore(prof.Vehicle, 1000.0, 500.0);
            plant.AttachImu(new CraneAerialImu(500.0, prof.Vehicle.Gravity, (ulong)seed));
            var ctl = new CraneMultirotorController(prof.Vehicle, plant.ControlDt);
            ctl.Follower = new CraneTrajectoryFollower();
            plant.CommandSource = ctl;
            var s0 = CraneAerialPlantCore.HoverState(prof.Vehicle, new[] { 0.0, 0.0, 3.0 });
            plant.Reset(s0, (ulong)seed);
            ctl.HoldPosition(s0.P, 0);
            ctl.Follower.SetTrajectory(new[] { s0.P, new[] { 4.0, 1.0, 3.5 } }, new[] { 2.0 });
            return (plant, prof, ctl);
        }

        static string[] Lines(string p) => File.ReadAllText(p).TrimEnd('\n').Split('\n');

        [Test]
        public void StateRowsAre1kHzFromTheRealPlantAndMonotonic() {
            var (plant, prof, ctl) = Plant(1337);
            var src = new AerialPlantLogSource(plant);
            var m = new DroneRunManifest { RunId = "aerial_test", SceneSeed = 1337, FixedDtNs = 5_000_000, Disturbance_Wind = "calm", Disturbance_Lighting = "GoldenHour" };
            src.FillManifest(m, prof, ctl);
            Assert.AreEqual(5, m.Substeps);
            Assert.AreEqual(500.0, m.ImuRateHz);
            Assert.AreEqual(64, m.PhysicsParamsSha256.Length);
            const int ticks = 100; // 0.5 s
            using (var session = DroneRunSession.Begin(dir, m, Splits(), false, src, src)) {
                var flight = new AerialLoggedFlight(session, src);
                while (flight.Tick < ticks) flight.StepTick();
            }

            var state = Lines(Path.Combine(dir, "state.csv"));
            Assert.AreEqual(ticks * 5 + 1, state.Length, "header + one row per 1 ms sub-step");
            long prev = -1; double prevPx = double.NaN; bool moved = false;
            for (int i = 1; i < state.Length; i++) {
                var c = state[i].Split(',');
                long t = long.Parse(c[2]);
                Assert.AreEqual(i * 1_000_000L, t, "row " + i + " is at i ms");
                Assert.Greater(t, prev); prev = t;
                Assert.AreEqual(t, long.Parse(c[0]) * 5_000_000L + int.Parse(c[1]) * 1_000_000L);
                double px = double.Parse(c[3], System.Globalization.CultureInfo.InvariantCulture);
                if (!double.IsNaN(prevPx) && px != prevPx) moved = true;
                prevPx = px;
            }
            Assert.IsTrue(moved, "positions come from the integrating plant");
            // last row equals the plant state (the rows are the real plant, not a copy of something else)
            var last = state[state.Length - 1].Split(',');
            Assert.AreEqual(plant.State.P[0], double.Parse(last[3], System.Globalization.CultureInfo.InvariantCulture));
            Assert.AreEqual(plant.State.T * 1e9, long.Parse(last[2]), 1e-3);

            var imu = Lines(Path.Combine(dir, "imu.csv"));
            Assert.AreEqual(ticks * 5 / 2 + 1, imu.Length, "plant IMU at 500 Hz");
            prev = -1;
            for (int i = 1; i < imu.Length; i++) {
                long t = long.Parse(imu[i].Split(',')[0]);
                Assert.AreEqual(i * 2_000_000L, t);
                Assert.Greater(t, prev); prev = t;
            }
            var az = double.Parse(imu[1].Split(',')[6], System.Globalization.CultureInfo.InvariantCulture);
            Assert.AreEqual(9.81, az, 1.5, "specific force near +g at hover");
            Assert.AreEqual(0, plant.Imu.Count, "every plant IMU sample was logged");
        }

        [Test]
        public void SameSeedIsBitwiseIdentical() {
            string[] Run(string sub) {
                var (plant, prof, ctl) = Plant(7);
                var src = new AerialPlantLogSource(plant);
                var m = new DroneRunManifest { RunId = sub, SceneSeed = 7, FixedDtNs = 5_000_000, Disturbance_Wind = "calm", Disturbance_Lighting = "GoldenHour" };
                src.FillManifest(m, prof, ctl);
                string d = Path.Combine(dir, sub);
                using (var session = DroneRunSession.Begin(d, m, Splits(), false, src, src)) {
                    var f = new AerialLoggedFlight(session, src);
                    while (f.Tick < 60) f.StepTick();
                }
                return Lines(Path.Combine(d, "state.csv"));
            }
            CollectionAssert.AreEqual(Run("a"), Run("b"));
        }
    }
}
