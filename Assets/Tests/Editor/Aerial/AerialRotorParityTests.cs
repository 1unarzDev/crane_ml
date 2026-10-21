using System;
using System.IO;
using NUnit.Framework;
using UnityEngine;
using Sim.Physics.Aerial;

namespace Sim.Tests.Aerial {
    [Serializable] public class ParityState {
        public double[] p, v, q, omega, rotorSpeed; public double soc;
    }
    [Serializable] public class ParityCommand { public int steps; public double[] command; }
    [Serializable] public class ParitySample {
        public int step; public double[] p, v, q, omega, rotorSpeed, specificForce; public double soc;
    }
    [Serializable] public class ParityScenario {
        public string name, profile, profileSha256;
        public bool battery, groundEffect, contact;
        public double dt; public double[] wind;
        public ParityState initial; public ParityCommand[] commands; public ParitySample[] samples;
    }
    [Serializable] public class ParityFixture { public string schema; public ParityScenario[] scenarios; }

    /// <summary>
    /// Parity of CraneRotorModelV2 against Tools/Aerial/aerial_model.py trajectories (exported by
    /// Tools/Aerial/export_parity_trajectories.py) plus physical invariants of the model.
    /// </summary>
    [Category("AerialRotorParity")]
    public class AerialRotorParityTests {
        static string ProfileDir => Path.GetFullPath(Path.Combine(Application.dataPath, "..", "Tools", "Aerial", "profiles"));
        static string FixturePath => Path.Combine(Application.dataPath, "Tests", "Editor", "AerialParity", "rotor_parity_trajectories.json");
        static CraneAerialProfile Load(string name, bool battery = false, bool ground = false) =>
            CraneAerialProfile.LoadFile(Path.Combine(ProfileDir, name + ".json"), battery, ground);

        static double Err(double a, double b) => Math.Abs(a - b) / (1.0 + Math.Abs(b));
        static double MaxErr(double[] a, double[] b) {
            Assert.AreEqual(b.Length, a.Length);
            double m = 0;
            for (int i = 0; i < a.Length; i++) m = Math.Max(m, Err(a[i], b[i]));
            return m;
        }

        // ---- parity ------------------------------------------------------------------------
        [Test] public void TrajectoriesMatchPythonOracle() {
            var fx = JsonUtility.FromJson<ParityFixture>(File.ReadAllText(FixturePath));
            Assert.AreEqual("crane-aerial-rotor-parity-v1", fx.schema);
            Assert.GreaterOrEqual(fx.scenarios.Length, 5);
            double worst = 0;
            foreach (var sc in fx.scenarios) {
                var prof = Load(sc.profile, sc.battery, sc.groundEffect);
                Assert.AreEqual(sc.profileSha256, prof.Sha256, sc.name + ": profile hash differs from the oracle's");
                var v = prof.Vehicle;
                var s = new StateV2(v.Rotors.Length) {
                    P = (double[])sc.initial.p.Clone(), V = (double[])sc.initial.v.Clone(),
                    Q = (double[])sc.initial.q.Clone(), Omega = (double[])sc.initial.omega.Clone(),
                    RotorSpeed = (double[])sc.initial.rotorSpeed.Clone(), Soc = sc.initial.soc
                };
                int step = 0, next = 0;
                double scenarioWorst = 0;
                foreach (var seg in sc.commands) {
                    for (int k = 0; k < seg.steps; k++) {
                        s = CraneRotorModelV2.Step(v, s, seg.command, sc.dt, sc.wind, sc.contact);
                        step++;
                        if (step % 10 != 0) continue;
                        var r = sc.samples[next++];
                        Assert.AreEqual(r.step, step);
                        double e = Math.Max(Math.Max(MaxErr(s.P, r.p), MaxErr(s.V, r.v)),
                            Math.Max(MaxErr(s.Q, r.q), MaxErr(s.Omega, r.omega)));
                        e = Math.Max(e, Math.Max(MaxErr(s.RotorSpeed, r.rotorSpeed), Err(s.Soc, r.soc)));
                        e = Math.Max(e, MaxErr(CraneRotorModelV2.SpecificForceBody(v, s, sc.wind), r.specificForce));
                        scenarioWorst = Math.Max(scenarioWorst, e);
                    }
                }
                Assert.AreEqual(sc.samples.Length, next, sc.name);
                TestContext.WriteLine($"AERIAL_PARITY scenario={sc.name} maxRelErr={scenarioWorst:E3}");
                Debug.Log($"AERIAL_PARITY scenario={sc.name} maxRelErr={scenarioWorst:E3}");
                Assert.Less(scenarioWorst, 1e-8, sc.name);
                worst = Math.Max(worst, scenarioWorst);
            }
            Debug.Log($"AERIAL_PARITY_MAX_ERR={worst:E3}");
        }

        [Test] public void ProfilesLoadWithHashAndExpectedShape() {
            var x = Load("x500_px4_gz");
            Assert.AreEqual(4, x.Vehicle.Rotors.Length);
            Assert.AreEqual(64, x.Sha256.Length);
            Assert.AreEqual(8.54858e-06, x.Vehicle.Rotors[0].Kf, 1e-15);
            Assert.IsNull(x.Vehicle.Battery);
            var cf = Load("crazyflie21", battery: true);
            Assert.IsNotNull(cf.Vehicle.Battery);
            Assert.AreEqual(2.2183102415114744e-08, cf.Vehicle.Rotors[0].Kf, 1e-20, "D15: measured kf is primary");
            Assert.AreEqual(6, Load("rotors_firefly_hex").Vehicle.Rotors.Length);
            Assert.Throws<FileNotFoundException>(() => CraneAerialProfile.Resolve("nope", ProfileDir));
            Assert.Throws<System.Collections.Generic.KeyNotFoundException>(() =>
                CraneAerialProfile.Parse("{\"schema\":\"crane-aerial-vehicle-profile-v1\"}", false, false));
            Assert.Throws<InvalidDataException>(() => CraneAerialProfile.Parse("{\"schema\":\"x\"}", false, false));
        }

        // ---- invariants --------------------------------------------------------------------
        static StateV2 Hover(VehicleSpecV2 v, double z = 100.0) {
            var s = new StateV2(v.Rotors.Length);
            s.P[2] = z;
            Array.Fill(s.RotorSpeed, CraneRotorModelV2.HoverRotorSpeed(v));
            return s;
        }

        [Test] public void HoverIsForceAndTorqueBalancedAndDriftFree() {
            var v = Load("x500_px4_gz").Vehicle;
            var s = Hover(v, 10.0);
            CraneRotorModelV2.Wrench(v, s, new double[3], out var f, out var t);
            Assert.AreEqual(v.Mass * v.Gravity, f[2], 1e-9);
            for (int i = 0; i < 3; i++) Assert.AreEqual(0.0, t[i], 1e-12);
            double w = s.RotorSpeed[0];
            var cmd = new[] { w, w, w, w };
            for (int i = 0; i < 500; i++) s = CraneRotorModelV2.Step(v, s, cmd, 0.002);
            Assert.AreEqual(10.0, s.P[2], 1e-9);
            Assert.Less(CraneRotorModelV2.Norm(s.Omega), 1e-12);
        }

        [Test] public void MotorStepFollowsUpAndDownTimeConstants() {
            var v = Load("x500_px4_gz").Vehicle;
            var r = v.Rotors[0];
            var s = new StateV2(4); Array.Fill(s.RotorSpeed, 500.0);
            double dt = r.TauUp / 50;
            for (int i = 0; i < 50; i++) CraneRotorModelV2.UpdateMotors(v, s, new[] { 800.0, 800, 800, 800 }, dt);
            Assert.AreEqual(500 + 300 * (1 - Math.Exp(-1)), s.RotorSpeed[0], 1e-9);
            s = new StateV2(4); Array.Fill(s.RotorSpeed, 800.0);
            dt = r.TauDown / 50;
            for (int i = 0; i < 50; i++) CraneRotorModelV2.UpdateMotors(v, s, new[] { 500.0, 500, 500, 500 }, dt);
            Assert.AreEqual(800 - 300 * (1 - Math.Exp(-1)), s.RotorSpeed[0], 1e-9);
        }

        [Test] public void ThrustIsQuadraticInRotorSpeed() {
            var v = Load("x500_px4_gz").Vehicle;
            var s1 = Hover(v); var s2 = Hover(v);
            for (int i = 0; i < 4; i++) s2.RotorSpeed[i] *= 2;
            CraneRotorModelV2.Wrench(v, s1, new double[3], out var f1, out _);
            CraneRotorModelV2.Wrench(v, s2, new double[3], out var f2, out _);
            Assert.AreEqual(4.0, f2[2] / f1[2], 1e-12);
        }

        [Test] public void ReactionTorqueSignFollowsSpin() {
            // Spinning up the CCW (+1) rotors produces NEGATIVE body yaw torque (-spin km T).
            var v = Load("x500_px4_gz").Vehicle;
            var s = Hover(v);
            for (int i = 0; i < 4; i++) s.RotorSpeed[i] += v.Rotors[i].Spin * 50.0;
            CraneRotorModelV2.Wrench(v, s, new double[3], out _, out var t);
            Assert.Less(t[2], 0.0);
            double expected = 0;
            for (int i = 0; i < 4; i++) expected += -v.Rotors[i].Spin * v.Rotors[i].Km * v.Rotors[i].Kf * s.RotorSpeed[i] * s.RotorSpeed[i];
            Assert.AreEqual(expected, t[2], 1e-12);
        }

        [Test] public void GroundEffectFollowsCheesemanBennett() {
            var v = Load("x500_px4_gz", ground: true).Vehicle;
            var r = v.Rotors[0];
            double R = r.Radius;
            Assert.AreEqual(1.0 / (1.0 - 1.0 / 16.0), CraneRotorModelV2.GroundEffectFactor(v, r, R), 1e-12);
            Assert.AreEqual(1.0 / (1.0 - 1.0 / 64.0), CraneRotorModelV2.GroundEffectFactor(v, r, 2 * R), 1e-12);
            Assert.AreEqual(1.0 / (1.0 - 1.0 / 256.0), CraneRotorModelV2.GroundEffectFactor(v, r, 4 * R), 1e-12);
            Assert.AreEqual(1.0 / (1.0 - 1.0 / 4.0), CraneRotorModelV2.GroundEffectFactor(v, r, 0.0), 1e-12, "clamped at h = R/2");
            v.GroundEffect = false;
            Assert.AreEqual(1.0, CraneRotorModelV2.GroundEffectFactor(v, r, R));
            double prev = double.MaxValue;
            v.GroundEffect = true;
            for (double h = 0.5 * R; h < 10 * R; h += 0.25 * R) {
                double f = CraneRotorModelV2.GroundEffectFactor(v, r, h);
                Assert.LessOrEqual(f, prev + 1e-15); Assert.GreaterOrEqual(f, 1.0); prev = f;
            }
        }

        [Test] public void RotorDragOpposesMotionAndScalesWithRotorSpeed() {
            var v = Load("x500_px4_gz").Vehicle;
            v.DragLinear = new double[3]; v.DragQuadratic = 0;
            var s = Hover(v); s.V = new[] { 3.0, 0, 0 };
            CraneRotorModelV2.Wrench(v, s, new double[3], out var f1, out _);
            Assert.Less(f1[0], 0.0, "lateral force opposes +x motion");
            double expected = -4 * s.RotorSpeed[0] * v.Rotors[0].RotorDrag * 3.0;
            Assert.AreEqual(expected, f1[0], 1e-12);
            for (int i = 0; i < 4; i++) s.RotorSpeed[i] *= 2;
            CraneRotorModelV2.Wrench(v, s, new double[3], out var f2, out _);
            Assert.AreEqual(2.0, f2[0] / f1[0], 1e-12);
            // Galilean: relative air velocity, not ground velocity, drives the drag.
            s = Hover(v); s.V = new[] { 3.0, 0, 0 };
            CraneRotorModelV2.Wrench(v, s, new[] { 3.0, 0, 0 }, out var f3, out _);
            Assert.AreEqual(0.0, f3[0], 1e-12);
        }

        [Test] public void BatterySagLowersRotorSpeedLimitAndDrainsCharge() {
            var v = Load("crazyflie21", battery: true).Vehicle;
            var s = new StateV2(4);
            double idle = CraneRotorModelV2.RotorSpeedLimit(v, v.Rotors[0], s);
            Array.Fill(s.RotorSpeed, v.Rotors[0].OmegaMax);
            double loaded = CraneRotorModelV2.RotorSpeedLimit(v, v.Rotors[0], s);
            Assert.Less(loaded, idle);
            Assert.Less(loaded, v.Rotors[0].OmegaMax);
            s.Soc = 0.2;
            Assert.Less(CraneRotorModelV2.RotorSpeedLimit(v, v.Rotors[0], s), loaded);
            s = new StateV2(4); Array.Fill(s.RotorSpeed, CraneRotorModelV2.HoverRotorSpeed(v));
            double cmd = s.RotorSpeed[0];
            var n = CraneRotorModelV2.Step(v, s, new[] { cmd, cmd, cmd, cmd }, 0.01);
            Assert.Less(n.Soc, 1.0);
            Assert.Greater(CraneRotorModelV2.ElectricalCurrent(v, s), 0.0);
        }

        [Test] public void FreeFallConservesEnergyWithoutAerodynamics() {
            var v = Load("x500_px4_gz").Vehicle;
            v.DragLinear = new double[3]; v.DragQuadratic = 0;
            foreach (var r in v.Rotors) { r.RotorDrag = 0; r.RollingMoment = 0; }
            var s = new StateV2(4) { Omega = new[] { 3.0, 2.0, 1.0 }, Q = new[] { 1.0, 0, 0, 0 } };
            s.P[2] = 50;
            double e0 = CraneRotorModelV2.MechanicalEnergy(v, s);
            var cmd = new double[4];
            for (int i = 0; i < 1000; i++) s = CraneRotorModelV2.Step(v, s, cmd, 0.002);
            Assert.AreEqual(e0, CraneRotorModelV2.MechanicalEnergy(v, s), 1e-4);
        }

        [Test] public void GyroscopicTorqueIsMinusOmegaCrossH() {
            var v = Load("x500_px4_gz").Vehicle;
            foreach (var r in v.Rotors) r.RotorInertia = 2e-5;
            var s = Hover(v);
            s.Omega = new[] { 0.0, 1.5, 0.0 };
            CraneRotorModelV2.Wrench(v, s, new double[3], out _, out var withJ);
            foreach (var r in v.Rotors) r.RotorInertia = 0;
            CraneRotorModelV2.Wrench(v, s, new double[3], out _, out var noJ);
            // Balanced spin pattern (2 CCW, 2 CW) at equal speed: net rotor angular momentum cancels.
            for (int i = 0; i < 3; i++) Assert.AreEqual(noJ[i], withJ[i], 1e-12);
            // Unbalance: all rotors one way -> h = sum J spin w axis, tau = -omega x h.
            foreach (var r in v.Rotors) { r.RotorInertia = 2e-5; r.Spin = 1; }
            CraneRotorModelV2.Wrench(v, s, new double[3], out _, out var tj);
            foreach (var r in v.Rotors) r.RotorInertia = 0;
            CraneRotorModelV2.Wrench(v, s, new double[3], out _, out var t0);
            double h = 4 * 2e-5 * s.RotorSpeed[0];
            var expected = CraneRotorModelV2.Scale(CraneRotorModelV2.Cross(s.Omega, new[] { 0, 0, h }), -1.0);
            for (int i = 0; i < 3; i++) Assert.AreEqual(expected[i], tj[i] - t0[i], 1e-12);
            Assert.Greater(Math.Abs(expected[0]), 1e-4, "pitch rate couples into roll torque");
        }

        // ---- mixer: wrench round trip through the plant, priority desaturation -------------
        static readonly string[] Quads = { "x500_px4_gz", "crazyflie21", "rotors_firefly_hex", "crane_reference_quad" };

        static double[] MixedWrench(VehicleSpecV2 v, double[] u) {
            var s = new StateV2(v.Rotors.Length); s.P[2] = 100.0;
            for (int i = 0; i < u.Length; i++) s.RotorSpeed[i] = v.Rotors[i].OmegaMax * Math.Sqrt(u[i]);
            CraneRotorModelV2.Wrench(v, s, new double[3], out var f, out var t);
            return new[] { f[2], t[0], t[1], t[2] };
        }

        [Test] public void UnsaturatedMixerOutputProducesRequestedTorqueSignsOnEachAxis() {
            // Ties mixer coefficients to the plant's own wrench (kills sign-flip mutants in either).
            foreach (string name in Quads) {
                var v = Load(name).Vehicle;
                var mixer = new CraneRotorMixerV2(v.Rotors);
                double tMax = v.Rotors[0].Kf * v.Rotors[0].OmegaMax * v.Rotors[0].OmegaMax;
                double ymax = 0, xmax = 0;
                foreach (var r in v.Rotors) { ymax = Math.Max(ymax, Math.Abs(r.Position[1])); xmax = Math.Max(xmax, Math.Abs(r.Position[0])); }
                double sy2 = 0, sx2 = 0;
                foreach (var r in v.Rotors) { sy2 += r.Position[1] * r.Position[1] / ymax; sx2 += r.Position[0] * r.Position[0] / xmax; }
                double km = v.Rotors[0].Km;
                double[] hover = MixedWrench(v, mixer.Mix(0.4, 0, 0, 0).Output);
                Assert.AreEqual(0, hover[1], 1e-12, name); Assert.AreEqual(0, hover[2], 1e-12, name); Assert.AreEqual(0, hover[3], 1e-12, name);
                foreach (double sign in new[] { 1.0, -1.0 }) {
                    double d = 0.1 * sign;
                    var res = mixer.Mix(0.4, d, 0, 0);
                    Assert.IsFalse(res.Saturated);
                    var w = MixedWrench(v, res.Output);
                    Assert.AreEqual(d * tMax * sy2, w[1], 1e-9 * tMax, name + " roll");          // +roll = +torque about x
                    Assert.AreEqual(0, w[2], 1e-9 * tMax, name + " roll cross pitch");
                    w = MixedWrench(v, mixer.Mix(0.4, 0, d, 0).Output);
                    Assert.AreEqual(d * tMax * sx2, w[2], 1e-9 * tMax, name + " pitch");         // +pitch = +torque about y
                    Assert.AreEqual(0, w[1], 1e-9 * tMax, name + " pitch cross roll");
                    w = MixedWrench(v, mixer.Mix(0.4, 0, 0, d).Output);
                    Assert.AreEqual(d * tMax * km * v.Rotors.Length, w[3], 1e-9 * tMax, name + " yaw");   // +yaw = +torque about z
                    Assert.AreEqual(0, w[1], 1e-9 * tMax, name + " yaw cross roll");
                    Assert.AreEqual(0, w[2], 1e-9 * tMax, name + " yaw cross pitch");
                    // collective is unchanged in thrust
                    Assert.AreEqual(0.4 * tMax * v.Rotors.Length, w[0], 1e-9 * tMax, name + " thrust");
                }
            }
        }

        [Test] public void MixerSpinPatternMatchesProfile() {
            var v = Load("x500_px4_gz").Vehicle;
            var m = new CraneRotorMixerV2(v.Rotors);
            for (int i = 0; i < 4; i++) Assert.AreEqual(-v.Rotors[i].Spin, m.YawCoeff[i]);
            CollectionAssert.AreEqual(new[] { 1, 1, -1, -1 }, new[] { v.Rotors[0].Spin, v.Rotors[1].Spin, v.Rotors[2].Spin, v.Rotors[3].Spin },
                "x500 profile order is ArduCopter QUAD/X: FR, RL CCW; FL, RR CW");
        }

        [Test] public void DesaturationPreservesAttitudeOverCollective() {
            var v = Load("x500_px4_gz").Vehicle;
            var mixer = new CraneRotorMixerV2(v.Rotors);
            // Legacy counterexample A6: collective 0.95 with roll 0.2 clipped each motor independently.
            var res = mixer.Mix(0.95, 0.2, 0, 0);
            Assert.IsTrue(res.Saturated);
            Assert.Less(res.Collective, 0.95);
            Assert.AreEqual(1.0, res.AttitudeScale);
            foreach (double u in res.Output) { Assert.GreaterOrEqual(u, 0.0); Assert.LessOrEqual(u, 1.0); }
            double tMax = v.Rotors[0].Kf * v.Rotors[0].OmegaMax * v.Rotors[0].OmegaMax;
            var w = MixedWrench(v, res.Output);
            double ymax = 0.174, sy2 = 4 * 0.174 * 0.174 / ymax;
            Assert.AreEqual(0.2 * tMax * sy2, w[1], 1e-9 * tMax, "full requested roll delivered");

            // Over-demand: roll and pitch beyond authority keep their ratio.
            res = mixer.Mix(0.5, 0.9, 0.45, 0.3);
            Assert.Less(res.AttitudeScale, 1.0);
            Assert.AreEqual(0.0, MixedWrench(v, res.Output)[3] / tMax, 1e-9, "yaw dropped first");
            var wr = MixedWrench(v, res.Output);
            Assert.AreEqual(2.0, wr[1] / wr[2], 1e-6, "roll:pitch ratio preserved");
            foreach (double u in res.Output) { Assert.GreaterOrEqual(u, -1e-12); Assert.LessOrEqual(u, 1.0 + 1e-12); }

            // Yaw gives way before roll when only yaw does not fit.
            res = mixer.Mix(0.5, 0.45, 0, 0.9);
            Assert.AreEqual(1.0, res.AttitudeScale);
            Assert.Less(res.YawScale, 1.0);
            Assert.Greater(res.YawScale, 0.0);
            double span(double[] o) { double lo = 9, hi = -9; foreach (double x in o) { lo = Math.Min(lo, x); hi = Math.Max(hi, x); } return hi - lo; }
            Assert.AreEqual(1.0, span(res.Output), 1e-9, "yaw scaled to exactly fit");
        }

        [Test] public void MixerOutputsAlwaysInRangeFuzz() {
            var rng = new System.Random(3);
            foreach (string name in Quads) {
                var mixer = new CraneRotorMixerV2(Load(name).Vehicle.Rotors);
                for (int i = 0; i < 2000; i++) {
                    var res = mixer.Mix(rng.NextDouble(), rng.NextDouble() * 2 - 1, rng.NextDouble() * 2 - 1, rng.NextDouble() * 2 - 1);
                    foreach (double u in res.Output) { Assert.GreaterOrEqual(u, 0.0); Assert.LessOrEqual(u, 1.0); }
                }
            }
        }
    }
}
