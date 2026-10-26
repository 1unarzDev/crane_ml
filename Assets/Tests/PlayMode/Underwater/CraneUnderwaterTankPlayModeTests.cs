#if UNITY_EDITOR
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using NUnit.Framework;
using Sim.Underwater;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using static Sim.Underwater.CraneLinAlg;

namespace Sim.Tests.PlayMode.Underwater {
    /// <summary>
    /// UW Tank: the CraneUnderwaterBody adapter drives one PhysX Rigidbody (drag 0, no gravity) and the measured
    /// trajectory is compared with the RK4 oracle (Tools/Underwater/fossen6dof.py) at the same fixed dt.
    ///
    /// Why the two do not agree to round-off: PhysX integrates semi-implicitly (v += F/m dt; x += v dt) and the adapter holds
    /// each step's wrench constant (explicit evaluation at t_k). Both are FIRST order in dt; the oracle is RK4. Local
    /// velocity error per step is O(dt^2), accumulated O(dt). Position error adds the semi-implicit offset ~ v dt / 2.
    /// Unity's PhysX applies no gyroscopic torque; the adapter therefore commands only I_cg*omega_dot about the CG (see CraneUnderwaterBody).
    /// Float32 PhysX state adds ~1e-6 relative noise. Evidence required by these tests: (a) errors below the stated absolute
    /// bounds at dt = 0.02 (project fixed step), (b) errors shrink when dt is halved (ratio below 0.6; ideal first order 0.5).
    /// </summary>
    [TestFixture, Category("UnderwaterTankPlayMode")]
    public sealed class CraneUnderwaterTankPlayModeTests {
        const string ScenePath = "Assets/Scenes/Underwater/UW Tank.unity";
        static readonly string GoldenPath = Path.Combine(Application.dataPath, "Tests/Editor/Underwater/Golden/parity_reference.json");

        // Absolute bounds at dt = 0.02 = 1.5 x the errors measured on 2026-10-21 (units: m, rad, m/s or rad/s). Measured errors halve
        // exactly when dt halves (ratio 0.50 in all cases), which is the first-order-scheme justification; do not loosen without re-measuring.
        public struct Bounds { public double Pos, Att, Nu; }
        static readonly Dictionary<string, Bounds> BoundsAt002 = new Dictionary<string, Bounds> {
            { "hover",         new Bounds { Pos = 1e-5, Att = 1e-5, Nu = 1e-5 } },     // measured 0
            { "surge_step",    new Bounds { Pos = 6.5e-3, Att = 1.5e-2, Nu = 1.3e-2 } },   // measured 4.2e-3 / 1.0e-2 / 8.8e-3 (1.5x)
            { "roll_release",  new Bounds { Pos = 8.5e-4, Att = 8e-3, Nu = 5e-3 } },       // measured 5.6e-4 / 5.3e-3 / 3.3e-3 (1.5x)
            { "yaw_spin_down", new Bounds { Pos = 1e-5, Att = 6.2e-2, Nu = 3.3e-2 } },     // measured 0 / 4.1e-2 / 2.2e-2 (1.5x)
            // real CG/CB offset: r_g = (0.01, -0.005, 0.02), BG = 0.03
            { "roll_release_cg",    new Bounds { Pos = 1.4e-3, Att = 2.4e-2, Nu = 2.7e-2 } },   // measured 9.1e-4 / 1.5e-2 / 1.7e-2 (1.5x)
            { "pitch_release_cg",   new Bounds { Pos = 1.1e-3, Att = 2.2e-2, Nu = 2.6e-2 } },   // measured 7.2e-4 / 1.4e-2 / 1.7e-2 (1.5x)
            { "pitch_release_bg3",  new Bounds { Pos = 1e-5, Att = 8e-4, Nu = 1.0e-3 } },       // measured 0 / 5.1e-4 / 6.7e-4 (1.5x)
            { "ascent_surplus",     new Bounds { Pos = 4.3e-3, Att = 1e-5, Nu = 1.8e-3 } },     // measured 2.8e-3 / 0 / 1.2e-3 (1.5x)
        };

        static double MaxAbs(double[] a) { double m = 0; foreach (var x in a) m = Math.Max(m, Math.Abs(x)); return m; }
        static double[] Arr(object o) => ((List<object>)o).Select(x => (double)x).ToArray();
        static Dictionary<string, object> D(object o) => (Dictionary<string, object>)o;

        struct Errors { public double Pos, Att, Nu, Thrust; }
        CraneUnderwaterProfile lastProfile;
        // full-rate traces of the last Run (one entry per physics step)
        readonly List<double> trT = new List<double>(), trPitch = new List<double>(), trHeave = new List<double>();
        double centerOfMassError;

        static double PitchOf(double[] q) => -Math.Asin(Math.Max(-1.0, Math.Min(1.0, QuatToRot(q)[2, 0])));

        IEnumerator Run(string baseName, double dt, Action<Errors> done) {
            trT.Clear(); trPitch.Clear(); trHeave.Clear();
            var golden = (Dictionary<string, object>)CraneMiniJson.Parse(File.ReadAllText(GoldenPath));
            string key = $"tank_{baseName}_dt{dt.ToString("0.0##", System.Globalization.CultureInfo.InvariantCulture)}";
            var c = ((List<object>)golden["cases"]).Select(D).First(x => (string)x["name"] == key);
            var profile = CraneUnderwaterProfile.LoadFromResources("bluerov2_heavy");
            Assert.AreEqual((string)golden["profileSha256"], profile.Sha256);
            if (c["variant"] != null) {     // real CG/CB (and buoyancy) offsets, identical to the oracle case
                var v = D(c["variant"]);
                var prm = profile.Params.Clone(); prm.RG = Arr(v["rG"]); prm.RB = Arr(v["rB"]); prm.Buoyancy = (double)v["buoyancyN"];
                profile = new CraneUnderwaterProfile { Name = profile.Name, EvidenceLabel = profile.EvidenceLabel, Sha256 = profile.Sha256, Params = prm, Thrusters = profile.Thrusters };
            }
            lastProfile = profile;

            yield return EditorSceneManager.LoadSceneAsyncInPlayMode(ScenePath, new LoadSceneParameters(LoadSceneMode.Single));
            var body = UnityEngine.Object.FindFirstObjectByType<CraneUnderwaterBody>();
            Assert.IsNotNull(body, "scene must contain the BlueROV2 proxy");
            var rb = body.GetComponent<Rigidbody>();
            Assert.AreEqual(0f, rb.linearDamping); Assert.AreEqual(0f, rb.angularDamping); Assert.IsFalse(rb.useGravity);

            float oldFixed = Time.fixedDeltaTime, oldCapture = Time.captureDeltaTime;
            Time.fixedDeltaTime = (float)dt;
            Time.captureDeltaTime = 0.1f;      // run faster than real time; 0.1 s of simulated time per frame
            try {
                // initial condition (NED/FRD -> Unity), then activate: no physics step happens in between
                var pos0 = Arr(c["pos0"]); var q0 = Arr(c["q0"]); var nu0 = Arr(c["nu0"]);
                var uq = CraneUnderwaterFrames.NedFrdQuatToUnity(q0);
                var rot = new Quaternion((float)uq[0], (float)uq[1], (float)uq[2], (float)uq[3]);
                var up = CraneUnderwaterFrames.NedToUnity(pos0);
                body.transform.SetPositionAndRotation(new Vector3((float)up[0], (float)up[1], (float)up[2]), rot);
                rb.position = body.transform.position; rb.rotation = rot;
                body.Configure(profile);
                var w = CraneUnderwaterFrames.NedAxialToUnity(new[] { nu0[3], nu0[4], nu0[5] });
                rb.angularVelocity = rot * new Vector3((float)w[0], (float)w[1], (float)w[2]);
                rb.linearVelocity = Vector3.zero;
                body.SetExternalWrench(Arr(c["tauExt"]));
                if (c["fCmd"] != null) body.SetThrusterCommands(Arr(c["fCmd"]).Select(x => (float)x).ToArray());

                int steps = (int)(double)c["steps"], every = (int)(double)c["sampleEvery"];
                var samples = ((List<object>)c["samples"]).Select(D).ToList();
                var err = new Errors();
                bool finished = false;
                // PhysX centre of mass must equal the profile CG exactly (set explicitly, not derived from colliders)
                var cgU = CraneUnderwaterFrames.NedToUnity(profile.Params.RG);
                centerOfMassError = (rb.centerOfMass - new Vector3((float)cgU[0], (float)cgU[1], (float)cgU[2])).magnitude;
                body.PreStep += (k, s) => {
                    if (k <= steps) { trT.Add(k * dt); trPitch.Add(PitchOf(s.Q)); trHeave.Add(s.Nu[2]); }
                    if (finished || k > steps || k % every != 0) return;
                    var r = samples[(int)(k / every)];
                    err.Pos = Math.Max(err.Pos, Norm(Sub(s.Pos, Arr(r["pos"]))));
                    err.Nu = Math.Max(err.Nu, Math.Max(Norm(Sub(Slice(s.Nu, 0, 3), Slice(Arr(r["nu"]), 0, 3))), Norm(Sub(Slice(s.Nu, 3, 3), Slice(Arr(r["nu"]), 3, 3)))));
                    var qr = Arr(r["q"]);
                    err.Att = Math.Max(err.Att, 2.0 * Math.Acos(Math.Min(1.0, Math.Abs(Dot(s.Q, qr)))));   // geodesic angle, rad
                    if (k == steps) finished = true;
                };
                int guard = 0;
                while (!finished && guard++ < 2000) yield return null;
                body.enabled = false;
                Assert.IsTrue(finished, $"{key}: did not reach step {steps} (StepCount={body.StepCount})");
                done(err);
            } finally {
                Time.fixedDeltaTime = oldFixed; Time.captureDeltaTime = oldCapture;
            }
        }

        IEnumerator Case(string name, Func<double, double> analyticErr = null) {
            Errors e02 = default, e01 = default; Action analytic = null;
            double a02 = 0, a01 = 0;
            yield return Run(name, 0.02, e => e02 = e);
            if (analyticErr != null) a02 = analyticErr(0.02);
            yield return Run(name, 0.01, e => e01 = e);
            if (analyticErr != null) a01 = analyticErr(0.01);
            if (analyticErr != null) {
                Debug.Log($"TANK_ANALYTIC {name} relErr dt0.02={a02:E3} dt0.01={a01:E3}");
                try { File.AppendAllText("/tmp/crane-underwater/tank_errors.txt", $"{name}\tanalytic relErr dt0.02={a02:E3} dt0.01={a01:E3}\n"); } catch (Exception) { }
                var ab = AnalyticBound[name];
                analytic = () => {
                    Assert.That(a02, Is.LessThan(ab), "analytic comparison at dt=0.02");
                    if (a02 > 1e-4) Assert.That(a01, Is.LessThan(0.6 * a02), "analytic error must shrink with dt");
                };
            }
            string line = $"{name}\tdt0.02 pos={e02.Pos:E3} att={e02.Att:E3} nu={e02.Nu:E3}\tdt0.01 pos={e01.Pos:E3} att={e01.Att:E3} nu={e01.Nu:E3}";
            Debug.Log("TANK_ERROR " + line);
            try { Directory.CreateDirectory("/tmp/crane-underwater"); File.AppendAllText("/tmp/crane-underwater/tank_errors.txt", line + "\n"); } catch (Exception) { }
            Assert.That(centerOfMassError, Is.LessThan(1e-6), "PhysX centerOfMass must be the profile CG");
            var b = BoundsAt002[name];
            Assert.That(e02.Pos, Is.LessThan(b.Pos), "position error (m) at dt=0.02");
            Assert.That(e02.Att, Is.LessThan(b.Att), "attitude error (rad) at dt=0.02");
            Assert.That(e02.Nu, Is.LessThan(b.Nu), "velocity error at dt=0.02");
            if (e02.Nu > 1e-3) Assert.That(e01.Nu, Is.LessThan(0.6 * e02.Nu), "velocity error must shrink with dt (first-order scheme)");
            if (e02.Pos > 1e-3) Assert.That(e01.Pos, Is.LessThan(0.6 * e02.Pos), "position error must shrink with dt");
            if (analytic != null) analytic();
        }

        [UnityTest] public IEnumerator Tank_Hover_Matches_Oracle() => Case("hover");
        [UnityTest] public IEnumerator Tank_SurgeStep_Matches_Oracle() => Case("surge_step");
        [UnityTest] public IEnumerator Tank_RollRelease_Matches_Oracle() => Case("roll_release");
        [UnityTest] public IEnumerator Tank_YawSpinDown_Matches_Oracle() => Case("yaw_spin_down");
        [UnityTest] public IEnumerator Tank_RollRelease_RealCgCb_Matches_Oracle() => Case("roll_release_cg");
        [UnityTest] public IEnumerator Tank_PitchRelease_RealCgCb_Matches_Oracle() => Case("pitch_release_cg");
        [UnityTest] public IEnumerator Tank_PitchRelease_Matches_Oracle_And_Analytic_Period() => Case("pitch_release_bg3", PitchPeriodRelativeError);
        [UnityTest] public IEnumerator Tank_BuoyancySurplusAscent_Matches_Oracle_And_Terminal_Velocity() => Case("ascent_surplus", TerminalVelocityRelativeError);

        // Relative error bounds against closed forms (dt = 0.02), see the convergence note on the class.
        static readonly Dictionary<string, double> AnalyticBound = new Dictionary<string, double> {
            { "pitch_release_bg3", 1.5e-2 },   // measured 9.8e-3 at dt 0.02, 4.8e-3 at 0.01 (halves)
            { "ascent_surplus", 1e-5 },        // measured 8e-7: the fixed point of a=0 is integrator independent
        };

        /// <summary>
        /// Small-angle pitch release, CG = CO, neutral, BG = 0.03 m, 1 deg. Linearised: (I_y + M_qdot) th'' + M_q th' + W BG th = 0.
        /// Undamped T0 = 2 pi sqrt((I+A)/(W BG)); with the profile's linear damping the free period is T_d = 2 pi / sqrt(k/J - (c/2J)^2).
        /// The quadratic term (0.47 |q| q) is ~3 % of the linear one at this amplitude and ignored; this is part of the measured error.
        /// Returns |T_measured - T_d| / T_d from successive downward zero crossings of the PhysX pitch.
        /// </summary>
        double PitchPeriodRelativeError(double dt) {
            var p = lastProfile.Params;
            double k = p.Weight * p.BG, J = p.InertiaCg[1, 1] + p.AddedMass[4, 4], c = p.LinearDamping[4, 4];
            double t0 = 2 * Math.PI * Math.Sqrt(J / k);
            double td = 2 * Math.PI / Math.Sqrt(k / J - (c / (2 * J)) * (c / (2 * J)));
            var cross = new List<double>();
            for (int i = 1; i < trPitch.Count; i++)
                if (trPitch[i - 1] > 0 && trPitch[i] <= 0) cross.Add(trT[i - 1] + dt * trPitch[i - 1] / (trPitch[i - 1] - trPitch[i]));
            Assert.GreaterOrEqual(cross.Count, 3, "need >= 3 downward crossings in 12 s");
            double measured = (cross[cross.Count - 1] - cross[0]) / (cross.Count - 1);
            Debug.Log($"TANK_PERIOD dt={dt} measured={measured:F5} analyticDamped={td:F5} analyticUndamped={t0:F5}");
            try { File.AppendAllText("/tmp/crane-underwater/tank_errors.txt", $"pitch period dt={dt} measured={measured:F5} T_d={td:F5} T0={t0:F5}\n"); } catch (Exception) { }
            return Math.Abs(measured - td) / td;
        }

        /// <summary>Terminal heave speed of a +10 N buoyancy surplus: Z_w|w| w^2 + Z_w w = S. Returns |w_mean(last 2 s) + w_term| / w_term (NED w is negative upward).</summary>
        double TerminalVelocityRelativeError(double dt) {
            var p = lastProfile.Params;
            double surplus = p.Buoyancy - p.Weight, lin = p.LinearDamping[2, 2], quad = p.QuadraticDamping[2];
            double wTerm = (-lin + Math.Sqrt(lin * lin + 4 * quad * surplus)) / (2 * quad);
            int n = (int)Math.Round(2.0 / dt); double sum = 0;
            for (int i = trHeave.Count - n; i < trHeave.Count; i++) sum += trHeave[i];
            double mean = sum / n;
            Debug.Log($"TANK_TERMINAL dt={dt} meanW={mean:F5} analytic={-wTerm:F5}");
            try { File.AppendAllText("/tmp/crane-underwater/tank_errors.txt", $"ascent terminal dt={dt} meanW={mean:F5} analytic={-wTerm:F5}\n"); } catch (Exception) { }
            return Math.Abs(mean + wTerm) / wTerm;
        }
    }
}
#endif
