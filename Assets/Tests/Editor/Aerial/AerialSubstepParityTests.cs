using System;
using System.IO;
using NUnit.Framework;
using UnityEngine;
using Sim.Physics.Aerial;

namespace Sim.Tests.Aerial {
    [Serializable] public class SubstepSample { public double t; public double[] p, v, q, omega, rotorSpeed; public double soc; }
    [Serializable] public class SubstepScenario {
        public string name; public double duration, controlDt, hoverRotorSpeed, omegaMax;
        public double[] p0, wind; public bool groundEffect, contact, battery;
        public SubstepSample[] reference, plant1kHz;
    }
    [Serializable] public class SubstepFixture {
        public string schema, profile, profileSha256; public double referenceDt, plantDt, unityDt, sampleDt;
        public SubstepScenario[] scenarios;
    }

    /// <summary>
    /// CraneAerialPlantCore (1 kHz RK4 sub-steps inside a 50 Hz Unity step) against
    /// Tools/Aerial/aerial_model.py: (a) identity with the oracle at the same 1 kHz step,
    /// (b) RMS error against the 10 kHz reference on step / chirp / saturation / pitch-flip.
    /// Fixture: Tools/Aerial/parity/aos7_substep_parity.json (export_substep_parity.py).
    /// </summary>
    [Category("AerialSubstepParity")]
    public class AerialSubstepParityTests {
        static string Root => Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
        static string FixturePath => Path.Combine(Root, "Tools", "Aerial", "parity", "aos7_substep_parity.json");
        static string ProfilePath => Path.Combine(Root, "Tools", "Aerial", "profiles", "aos7_v5_f90_6s_v1.json");

        // RMS bounds vs the 10 kHz reference: (pos m, att rad, rate rad/s, rotor rad/s). Observed
        // values are in Docs/DroneVisuals/PHYSICS.md; bounds are ~3x observed (non-smooth scenarios)
        // or 1e-6-level (smooth scenarios, where only round-off and O(dt^4) remain).
        static readonly (string name, double pos, double att, double rate, double rotor)[] Bounds = {
            ("step", 2e-4, 1e-3, 6e-3, 1e-6),          // ground contact at t = 1.73 s
            ("chirp", 1e-6, 1e-6, 1e-6, 1e-6),
            ("saturation", 4e-3, 4e-3, 1.2e-2, 0.5),    // battery-limit clamp switching
            ("pitch_flip", 1e-6, 1e-6, 1e-6, 1e-6),
        };

        sealed class Schedule : IAerialRotorCommandSource {
            readonly string name; readonly double wh, wmax;
            public Schedule(string n, double h, double m) { name = n; wh = h; wmax = m; }
            public double[] RotorCommands(StateV2 s, double t) {
                switch (name) {
                    case "step":
                        if (t < 0.5) return new[] { wh, wh, wh, wh };
                        if (t < 1.0) return new[] { 1.15 * wh, 1.15 * wh, 1.15 * wh, 1.15 * wh };
                        if (t < 1.3) return new[] { 1.06 * wh, 0.94 * wh, 0.94 * wh, 1.06 * wh };
                        return new[] { wh, wh, wh, wh };
                    case "chirp": {
                        double f0 = 0.5, f1 = 10.0, T = 3.0, tt = Math.Min(t, T);
                        double phase = 2 * Math.PI * (f0 * tt + 0.5 * (f1 - f0) / T * tt * tt);
                        double d = 0.04 * wh * Math.Sin(phase);
                        return new[] { wh + d, wh - d, wh + d, wh - d };
                    }
                    case "saturation":
                        if (t < 0.6) return new[] { 1.3 * wmax, 1.3 * wmax, 1.3 * wmax, 1.3 * wmax };
                        if (t < 0.9) return new[] { wmax, wmax, 0.2 * wmax, 0.2 * wmax };
                        if (t < 1.2) return new double[4];
                        return new[] { wh, wh, wh, wh };
                    case "pitch_flip":
                        if (t < 0.3) return new[] { wh, wh, wh, wh };
                        if (t < 0.38) return new[] { 0.35 * wmax, 0.9 * wmax, 0.35 * wmax, 0.9 * wmax };
                        if (t < 0.62) return new[] { 0.45 * wmax, 0.45 * wmax, 0.45 * wmax, 0.45 * wmax };
                        if (t < 0.70) return new[] { 0.9 * wmax, 0.35 * wmax, 0.9 * wmax, 0.35 * wmax };
                        return new[] { 1.25 * wh, 1.25 * wh, 1.25 * wh, 1.25 * wh };
                }
                throw new ArgumentException(name);
            }
        }

        static double QuatAngle(double[] a, double[] b) {
            double d = Math.Abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
            return 2.0 * Math.Acos(Math.Min(1.0, d));
        }
        static double Dist(double[] a, double[] b) {
            double s = 0; for (int i = 0; i < a.Length; i++) s += (a[i] - b[i]) * (a[i] - b[i]);
            return Math.Sqrt(s);
        }
        static double RelMax(double[] a, double[] b) {
            double m = 0; for (int i = 0; i < a.Length; i++) m = Math.Max(m, Math.Abs(a[i] - b[i]) / (1.0 + Math.Abs(b[i])));
            return m;
        }

        [Test] public void SubsteppedPlantMatchesHighRateOracle() {
            var fx = JsonUtility.FromJson<SubstepFixture>(File.ReadAllText(FixturePath));
            Assert.AreEqual("crane-aerial-substep-parity-v1", fx.schema);
            Assert.AreEqual(4, fx.scenarios.Length);
            foreach (var sc in fx.scenarios) {
                var prof = CraneAerialProfile.LoadFile(ProfilePath, sc.battery, sc.groundEffect);
                Assert.AreEqual(fx.profileSha256, prof.Sha256, "profile changed since export; rerun export_substep_parity.py");
                var plant = new CraneAerialPlantCore(prof.Vehicle, 1.0 / fx.plantDt, 1.0 / sc.controlDt) {
                    Contact = sc.contact, MeanWindWorld = (double[])sc.wind.Clone(),
                    CommandSource = new Schedule(sc.name, sc.hoverRotorSpeed, sc.omegaMax)
                };
                var s0 = new StateV2(4);
                Array.Copy(sc.p0, s0.P, 3);
                for (int i = 0; i < 4; i++) s0.RotorSpeed[i] = sc.hoverRotorSpeed;
                plant.Reset(s0, 1);
                int unitySteps = (int)Math.Round(sc.duration / fx.unityDt);
                double ep = 0, ea = 0, ew = 0, er = 0, identity = 0; int n = 0;
                for (int k = 0; k < unitySteps; k++) {
                    // one Unity FixedUpdate = 20 sub-steps; compare at every 10 ms sample inside it
                    for (int half = 0; half < 2; half++) {
                        plant.Advance(fx.sampleDt);   // 10 sub-steps (two per Unity FixedUpdate of 20 ms)
                        var st = plant.State;
                        var r = sc.reference[n]; var p1 = sc.plant1kHz[n];
                        Assert.AreEqual(r.t, st.T, 1e-9);
                        ep += Math.Pow(Dist(st.P, r.p), 2); ea += Math.Pow(QuatAngle(st.Q, r.q), 2);
                        ew += Math.Pow(Dist(st.Omega, r.omega), 2); er += Math.Pow(Dist(st.RotorSpeed, r.rotorSpeed) / 2.0, 2);
                        identity = Math.Max(identity, Math.Max(Math.Max(RelMax(st.P, p1.p), RelMax(st.V, p1.v)),
                            Math.Max(Math.Max(RelMax(st.Q, p1.q), RelMax(st.Omega, p1.omega)), RelMax(st.RotorSpeed, p1.rotorSpeed))));
                        n++;
                    }
                }
                Assert.AreEqual(sc.reference.Length, n);
                double rp = Math.Sqrt(ep / n), ra = Math.Sqrt(ea / n), rw = Math.Sqrt(ew / n), rr = Math.Sqrt(er / n);
                string line = $"AERIAL_SUBSTEP_PARITY scenario={sc.name} posRms={rp:E2} attRms={ra:E2} rateRms={rw:E2} rotorRms={rr:E2} identityMaxRel={identity:E2}";
                TestContext.WriteLine(line); Debug.Log(line);
                Assert.Less(identity, 1e-7, sc.name + ": C# 1 kHz differs from the oracle at the same step");
                var b = Array.Find(Bounds, x => x.name == sc.name);
                Assert.Less(rp, b.pos, sc.name + " pos"); Assert.Less(ra, b.att, sc.name + " att");
                Assert.Less(rw, b.rate, sc.name + " rate"); Assert.Less(rr, b.rotor, sc.name + " rotor");
            }
        }

        [Test] public void FlipScenarioActuallyInverts() {
            var fx = JsonUtility.FromJson<SubstepFixture>(File.ReadAllText(FixturePath));
            var sc = Array.Find(fx.scenarios, x => x.name == "pitch_flip");
            double minUp = 1;
            foreach (var r in sc.reference) minUp = Math.Min(minUp, CraneRotorModelV2.QuatToMatrix(r.q)[2, 2]);
            Assert.Less(minUp, -0.9, "body z must point down during the flip");
        }

        [Test] public void AdvanceIsIndependentOfUnityFixedStep() {
            var v = CraneAerialProfile.LoadFile(ProfilePath, true, false).Vehicle;
            StateV2 Run(double unityDt) {
                var plant = new CraneAerialPlantCore(v) { Contact = false, CommandSource = new Schedule("pitch_flip", CraneRotorModelV2.HoverRotorSpeed(v), v.Rotors[0].OmegaMax) };
                var s0 = CraneAerialPlantCore.HoverState(v, new[] { 0.0, 0.0, 10.0 });
                plant.Reset(s0, 1);
                int steps = (int)Math.Round(1.2 / unityDt);
                for (int i = 0; i < steps; i++) plant.Advance(unityDt);
                return plant.State;
            }
            var a = Run(0.02); var b = Run(0.01); var c = Run(0.005);  // Unity at 50, 100, 200 Hz
            Assert.Less(Dist(a.P, b.P), 1e-12); Assert.Less(Dist(a.Q, b.Q), 1e-12);
            Assert.Less(Dist(a.P, c.P), 1e-12);
        }
    }
}
