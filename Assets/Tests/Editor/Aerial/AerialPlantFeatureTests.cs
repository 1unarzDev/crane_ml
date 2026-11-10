using System;
using System.Collections.Generic;
using System.IO;
using NUnit.Framework;
using UnityEngine;
using Sim.Physics.Aerial;

namespace Sim.Tests.Aerial {
    /// <summary>Turbulence, IMU frames/signs, controller + trajectory follower, AOS 7 profile provenance.</summary>
    [Category("AerialPlantFeatures")]
    public class AerialPlantFeatureTests {
        static string Root => Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
        static string ProfilePath => Path.Combine(Root, "Tools", "Aerial", "profiles", "aos7_v5_f90_6s.json");
        static VehicleSpecV2 Aos(bool battery = true, bool ground = true) => CraneAerialProfile.LoadFile(ProfilePath, battery, ground).Vehicle;

        // ---- profile -------------------------------------------------------------------------
        [Test] public void Aos7ProfileHasProvenanceOnEveryParameterAndPlausibleValues() {
            var root = (Dictionary<string, object>)MiniJsonAccess.Parse(File.ReadAllText(ProfilePath));
            var prm = (Dictionary<string, object>)root["parameters"];
            var allowed = new HashSet<string> { "MEASURED_DATASHEET", "LITERATURE", "ESTIMATED" };
            foreach (var kv in prm) {
                var e = (Dictionary<string, object>)kv.Value;
                Assert.IsTrue(allowed.Contains((string)e["status"]), kv.Key + " status");
                Assert.IsFalse(string.IsNullOrEmpty((string)e["locator"]), kv.Key + " locator");
            }
            foreach (var item in (List<object>)root["massBudget"])
                Assert.IsTrue(allowed.Contains((string)((Dictionary<string, object>)item)["status"]));
            var v = Aos();
            Assert.That(v.Mass, Is.InRange(1.0, 1.5));
            Assert.AreEqual(4, v.Rotors.Length);
            double tw = 4 * v.Rotors[0].Kf * v.Rotors[0].OmegaMax * v.Rotors[0].OmegaMax / (v.Mass * v.Gravity);
            Assert.Greater(tw, 2.5, "thrust-to-weight");
            Assert.That(v.Rotors[0].TauUp, Is.InRange(0.010, 0.050));
            CollectionAssert.AreEqual(new[] { 1, 1, -1, -1 }, Array.ConvertAll(v.Rotors, r => r.Spin), "QUAD-X FR,RL CCW; FL,RR CW");
            Assert.Less(v.Inertia[0, 0], v.Inertia[1, 1]); Assert.Less(v.Inertia[1, 1], v.Inertia[2, 2]);
        }

        // ---- turbulence ----------------------------------------------------------------------
        [Test] public void DrydenIsSeededResetSafeAndHasTheSpecifiedVariance() {
            var mean = new[] { 5.0, 0.0, 0.0 }; var vel = new double[3];
            var a = new CraneDrydenTurbulence(10.0, 42); var b = new CraneDrydenTurbulence(10.0, 42);
            var first = new List<double[]>();
            for (int i = 0; i < 1000; i++) { var x = a.Step(0.001, 30.0, mean, vel); first.Add(x); CollectionAssert.AreEqual(x, b.Step(0.001, 30.0, mean, vel)); }
            a.Reset(42);
            for (int i = 0; i < 1000; i++) CollectionAssert.AreEqual(first[i], a.Step(0.001, 30.0, mean, vel), "reset replays");
            var c = new CraneDrydenTurbulence(10.0, 43);
            Assert.AreNotEqual(first[10][0], c.Step(0.001, 30.0, mean, vel)[0]);
            CraneDrydenTurbulence.Scales(30.0, 10.0, out double lu, out double lw, out double su, out double sw);
            Assert.AreEqual(1.0, sw, 1e-12, "sigma_w = 0.1 W20");
            Assert.Greater(su, sw); Assert.AreEqual(30.0, lw, 1e-9); Assert.Greater(lu, lw);
            // stationary std over a long run (many correlation times) within 15 %
            var d = new CraneDrydenTurbulence(10.0, 7);
            double s0 = 0, s2 = 0, w2 = 0; int n = 0;
            for (int i = 0; i < 600000; i++) {
                var g = d.Step(0.002, 30.0, mean, vel);
                if (i < 20000) continue;
                s0 += g[0] * g[0]; s2 += g[1] * g[1]; w2 += g[2] * g[2]; n++;
            }
            Assert.AreEqual(su, Math.Sqrt(s0 / n), 0.15 * su, "u");
            Assert.AreEqual(su, Math.Sqrt(s2 / n), 0.15 * su, "v");
            Assert.AreEqual(sw, Math.Sqrt(w2 / n), 0.15 * sw, "w");
        }

        [Test] public void PlantResetReplaysTurbulentEpisodeExactly() {
            var v = Aos();
            var plant = new CraneAerialPlantCore(v) { MeanWindWorld = new[] { 3.0, 1.0, 0.0 }, Turbulence = new CraneDrydenTurbulence(8.0, 5) };
            var ctrl = new CraneMultirotorController(v, plant.ControlDt);
            plant.CommandSource = ctrl;
            var s0 = CraneAerialPlantCore.HoverState(v, new[] { 0.0, 0.0, 5.0 });
            double[] Run() {
                plant.Reset(s0, 99); ctrl.ResetIntegrators(); ctrl.HoldPosition(s0.P, 0.0);
                for (int i = 0; i < 100; i++) plant.Advance(0.02);
                return (double[])plant.State.P.Clone();
            }
            var p1 = Run(); var p2 = Run();
            CollectionAssert.AreEqual(p1, p2);
            Assert.Greater(Math.Abs(p1[0]) + Math.Abs(p1[1]), 1e-4, "gusts move the vehicle");
        }

        // ---- IMU -----------------------------------------------------------------------------
        static CraneAerialPlantCore ImuPlant(VehicleSpecV2 v, StateV2 s0, out CraneAerialImu imu) {
            var plant = new CraneAerialPlantCore(v) { Contact = false };
            imu = new CraneAerialImu(500.0, v.Gravity, 1);
            plant.AttachImu(imu);
            plant.Reset(s0, 1);
            return plant;
        }

        [Test] public void ImuRateAndSpecificForceSignsAtHoverAndFreeFall() {
            var v = Aos(battery: false, ground: false);
            var s0 = CraneAerialPlantCore.HoverState(v, new[] { 0.0, 0.0, 50.0 });
            var plant = ImuPlant(v, s0, out var imu);
            plant.RotorCommand = (double[])s0.RotorSpeed.Clone();
            plant.Advance(0.1);
            Assert.AreEqual(50, imu.Count, "500 Hz over 0.1 s");
            var h = imu.Latest;
            Assert.AreEqual(v.Gravity, h.SpecificForceFlu[2], 1e-6, "hover: +g on body z (FLU)");
            Assert.AreEqual(-v.Gravity, h.SpecificForceFrd[2], 1e-6, "hover: -g on body z (FRD)");
            Assert.AreEqual(0.0, h.SpecificForceFlu[0], 1e-4);
            Assert.Throws<ArgumentOutOfRangeException>(() => new CraneAerialImu(200.0, 9.81, 1));
            // free fall: motors off, no drag at zero airspeed initially -> ~0 specific force
            var s1 = CraneAerialPlantCore.HoverState(v, new[] { 0.0, 0.0, 50.0 });
            for (int i = 0; i < 4; i++) s1.RotorSpeed[i] = 0;
            plant = ImuPlant(v, s1, out imu);
            plant.Advance(0.004);
            Assert.Less(CraneRotorModelV2.Norm(imu.Latest.SpecificForceFlu), 0.05);
        }

        [Test] public void ImuGyroIsBodyFrameAndPseudovectorCorrect() {
            var v = Aos(battery: false, ground: false);
            // pitched 90 deg nose-up about body y... use a 90 deg roll: body y points up.
            var s0 = CraneAerialPlantCore.HoverState(v, new[] { 0.0, 0.0, 50.0 });
            s0.Q = new[] { Math.Cos(Math.PI / 4), Math.Sin(Math.PI / 4), 0, 0 };
            s0.Omega = new[] { 0.0, 0.0, 0.5 };   // body yaw rate about body z (which is world -y now)
            for (int i = 0; i < 4; i++) s0.RotorSpeed[i] = 0;
            var plant = ImuPlant(v, s0, out var imu);
            plant.Advance(0.002);
            var g = imu.Latest.GyroFlu;
            Assert.AreEqual(0.5, g[2], 1e-3, "gyro reports BODY rate, not world rate");
            Assert.AreEqual(0.0, g[0], 1e-3); Assert.AreEqual(0.0, g[1], 1e-3);
            Assert.AreEqual(-0.5, imu.Latest.GyroFrd[2], 1e-3, "FRD yaw rate = -FLU");
            // CCW yaw seen from above (positive ENU z rotation) is positive FLU gyro z
            var s1 = CraneAerialPlantCore.HoverState(v, new[] { 0.0, 0.0, 50.0 });
            s1.Omega = new[] { 0.0, 0.0, 0.3 };
            plant = ImuPlant(v, s1, out imu);
            plant.RotorCommand = (double[])s1.RotorSpeed.Clone();
            plant.Advance(0.1);
            Assert.Greater(CraneAerialFrames.YawEnu(plant.State.Q), 0.0);
            Assert.Greater(imu.Latest.GyroFlu[2], 0.0);
        }

        [Test] public void ImuQueueIsBounded() {
            var v = Aos(battery: false, ground: false);
            var s0 = CraneAerialPlantCore.HoverState(v, new[] { 0.0, 0.0, 50.0 });
            var plant = ImuPlant(v, s0, out var imu);
            imu.Capacity = 16;
            plant.RotorCommand = (double[])s0.RotorSpeed.Clone();
            plant.Advance(0.1);
            Assert.AreEqual(16, imu.Count); Assert.AreEqual(34, imu.Dropped);
        }

        [Test] public void FramesRoundTripAndNedConventions() {
            var q = CraneRotorModelV2.QuatMul(new[] { Math.Cos(0.3), 0, 0, Math.Sin(0.3) }, new[] { Math.Cos(0.1), Math.Sin(0.1), 0, 0 });
            CraneAerialFrames.UnityLookAxes(q, out var f, out var u);
            var back = CraneAerialFrames.QuatFromUnityAxes(f, u);
            for (int i = 0; i < 4; i++) Assert.AreEqual(q[i], back[i], 1e-12);
            CollectionAssert.AreEqual(new[] { 2.0, 1.0, -3.0 }, CraneAerialFrames.EnuToNed(new[] { 1.0, 2.0, 3.0 }));
            // ENU yaw 0 (facing east) = NED yaw +90 deg; level -> roll = pitch = 0
            var e = CraneAerialFrames.NedFrdEuler(new[] { 1.0, 0, 0, 0 });
            Assert.AreEqual(0.0, e[0], 1e-12); Assert.AreEqual(0.0, e[1], 1e-12); Assert.AreEqual(Math.PI / 2, e[2], 1e-12);
            // FLU nose-up pitch (negative rotation about +y_FLU) is positive NED pitch
            var up = CraneAerialFrames.NedFrdEuler(new[] { Math.Cos(-0.1), 0, Math.Sin(-0.1), 0 });
            Assert.AreEqual(0.2, up[1], 1e-9);
            // positive rotation about +x_FLU lifts the left side (y -> z): right side down = positive NED roll
            var roll = CraneAerialFrames.NedFrdEuler(new[] { Math.Cos(0.1), Math.Sin(0.1), 0, 0 });
            Assert.AreEqual(0.2, roll[0], 1e-9);
        }

        // ---- controller / trajectory -----------------------------------------------------------
        static (CraneAerialPlantCore plant, CraneMultirotorController ctrl) Flying(double[] p) {
            var v = Aos();
            var plant = new CraneAerialPlantCore(v);
            var ctrl = new CraneMultirotorController(v, plant.ControlDt);
            plant.CommandSource = ctrl;
            var s0 = CraneAerialPlantCore.HoverState(v, p);
            plant.Reset(s0, 1);
            ctrl.HoldPosition(p, 0.0);
            return (plant, ctrl);
        }

        [Test] public void ControllerHoldsHoverAndRejectsAStepInPosition() {
            var (plant, ctrl) = Flying(new[] { 0.0, 0.0, 3.0 });
            for (int i = 0; i < 100; i++) plant.Advance(0.02);
            Assert.Less(CraneRotorModelV2.Norm(CraneRotorModelV2.Sub(plant.State.P, new[] { 0.0, 0.0, 3.0 })), 0.02);
            ctrl.HoldPosition(new[] { 4.0, -2.0, 5.0 }, 1.0);
            for (int i = 0; i < 400; i++) plant.Advance(0.02);
            var s = plant.State;
            Assert.Less(CraneRotorModelV2.Norm(CraneRotorModelV2.Sub(s.P, new[] { 4.0, -2.0, 5.0 })), 0.05);
            Assert.AreEqual(1.0, CraneAerialFrames.YawEnu(s.Q), 0.02);
            Assert.Less(CraneRotorModelV2.Norm(s.Omega), 0.05);
        }

        [Test] public void ControllerTracksVelocitySetpointWithTurbulence() {
            var (plant, ctrl) = Flying(new[] { 0.0, 0.0, 10.0 });
            plant.MeanWindWorld = new[] { -3.0, 0.0, 0.0 };
            plant.Turbulence = new CraneDrydenTurbulence(6.0, 11);
            ctrl.SetVelocity(new[] { 6.0, 2.0, 0.0 }, 0.2);
            for (int i = 0; i < 250; i++) plant.Advance(0.02);
            double ev = 0; int n = 0;
            for (int i = 0; i < 100; i++) {
                plant.Advance(0.02);
                ev += CraneRotorModelV2.Norm(CraneRotorModelV2.Sub(plant.State.V, new[] { 6.0, 2.0, 0.0 })); n++;
            }
            Assert.Less(ev / n, 0.6, "mean speed error in gusts");
            Assert.AreEqual(0.2, plant.State.Omega[2], 0.1, "yaw rate");
        }

        [Test] public void TrajectoryFollowerFliesWaypointsAndStops() {
            var (plant, ctrl) = Flying(new[] { 0.0, 0.0, 2.0 });
            var fol = new CraneTrajectoryFollower();
            ctrl.Follower = fol;
            var wps = new[] { new[] { 0.0, 0.0, 2.0 }, new[] { 10.0, 0.0, 3.0 }, new[] { 10.0, 8.0, 3.0 }, new[] { 0.0, 8.0, 2.0 } };
            fol.SetTrajectory(wps, new[] { 6.0, 4.0, 8.0 });
            double maxDev = 0;
            for (int i = 0; i < 1500 && !(fol.Finished && CraneRotorModelV2.Norm(plant.State.V) < 0.05); i++) {
                plant.Advance(0.02);
                double y = plant.State.P[1], x = plant.State.P[0];
                if (x > 1 && x < 9 && y < 4) maxDev = Math.Max(maxDev, Math.Abs(y));
            }
            Assert.IsTrue(fol.Finished);
            for (int i = 0; i < 150; i++) plant.Advance(0.02);   // 3 s settle on the final hold
            Assert.Less(CraneRotorModelV2.Norm(CraneRotorModelV2.Sub(plant.State.P, wps[3])), 0.1);
            Assert.Less(maxDev, 0.5, "cross-track deviation on the first leg");
        }

        [Test] public void MixerGivesUpYawFirstUnderSaturation() {
            var v = Aos();
            var mixer = new CraneRotorMixerV2(v.Rotors);
            var r = mixer.Mix(0.5, 0.3, 0.0, 0.9);
            Assert.AreEqual(1.0, r.AttitudeScale); Assert.Less(r.YawScale, 1.0); Assert.IsTrue(r.Saturated);
        }
    }

    /// <summary>Test-side access to the profile JSON reader (MiniJson is internal to the plant assembly).</summary>
    static class MiniJsonAccess {
        public static object Parse(string json) {
            var t = typeof(CraneAerialProfile).Assembly.GetType("Sim.Physics.Aerial.MiniJson");
            return t.GetMethod("Parse").Invoke(null, new object[] { json });
        }
    }
}
