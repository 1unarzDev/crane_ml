using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using NUnit.Framework;
using Sim.Underwater;
using Sim.Utils.Performance;
using UnityEngine;
using static Sim.Underwater.CraneLinAlg;

namespace Sim.Tests.Editor.Underwater {
    /// <summary>
    /// C# core vs Tools/Underwater/fossen6dof.py reference trajectories plus physical invariants.
    /// Parity tolerance is 1e-8 (absolute, on position m, quaternion, velocity m/s or rad/s, thrust N); both sides are
    /// float64 RK4 with the same operation order, so the observed error is expected near 1e-12.
    /// </summary>
    [TestFixture, Category("UnderwaterParity")]
    public sealed class CraneUnderwaterParityTests {
        const double ParityTol = 1e-8;
        static readonly string Root = Path.Combine(Application.dataPath, "Tests/Editor/Underwater/Golden/parity_reference.json");
        static readonly Dictionary<string, double> MaxErr = new Dictionary<string, double>();

        Dictionary<string, object> golden;
        CraneUnderwaterProfile profile;

        [OneTimeSetUp]
        public void Load() {
            golden = (Dictionary<string, object>)CraneMiniJson.Parse(File.ReadAllText(Root));
            profile = CraneUnderwaterProfile.LoadFromResources("bluerov2_heavy");
        }

        [OneTimeTearDown]
        public void Report() {
            double worst = MaxErr.Count == 0 ? 0 : MaxErr.Values.Max();
            string text = string.Join("\n", MaxErr.Select(kv => $"{kv.Key}\t{kv.Value:E3}")) + $"\nMAX\t{worst:E3}\n";
            Debug.Log("PARITY_MAX_ERROR\n" + text);
            try { Directory.CreateDirectory("/tmp/crane-underwater"); File.WriteAllText("/tmp/crane-underwater/parity_max_error.txt", text); } catch (Exception) { }
        }

        // ---- helpers ---------------------------------------------------------------------
        static double[] Arr(object o) => ((List<object>)o).Select(x => (double)x).ToArray();
        static double[,] Mat(object o) {
            var rows = ((List<object>)o).Select(Arr).ToArray();
            var m = new double[rows.Length, rows[0].Length];
            for (int i = 0; i < rows.Length; i++) for (int j = 0; j < rows[0].Length; j++) m[i, j] = rows[i][j];
            return m;
        }
        static Dictionary<string, object> D(object o) => (Dictionary<string, object>)o;
        static double MaxAbsDiff(double[] a, double[] b) {
            Assert.AreEqual(a.Length, b.Length);
            double m = 0; for (int i = 0; i < a.Length; i++) m = Math.Max(m, Math.Abs(a[i] - b[i])); return m;
        }
        static void Track(string key, double e) { MaxErr[key] = Math.Max(MaxErr.TryGetValue(key, out var v) ? v : 0, e); }

        Dictionary<string, object> Case(string name) =>
            ((List<object>)golden["cases"]).Select(D).First(c => (string)c["name"] == name);

        void RunTrajectory(string name) {
            var c = Case(name);
            bool useThr = (bool)c["useThrusters"];
            var sim = new CraneFossenSimulator(profile.Params, Arr(c["current"]), useThr ? profile.Thrusters : null);
            var s = new CraneState { Pos = Arr(c["pos0"]), Q = Arr(c["q0"]), Nu = Arr(c["nu0"]), Thrust = Arr(c["thrust0"]) };
            var tau = Arr(c["tauExt"]);
            var fCmd = c["fCmd"] == null ? null : Arr(c["fCmd"]);
            double dt = (double)c["dt"];
            int steps = (int)(double)c["steps"], every = (int)(double)c["sampleEvery"];
            var samples = ((List<object>)c["samples"]).Select(D).ToList();
            double worst = 0;
            int si = 0;
            for (int k = 0; k <= steps; k++) {
                if (k % every == 0) {
                    var r = samples[si++];
                    double e = Math.Max(Math.Max(MaxAbsDiff(s.Pos, Arr(r["pos"])), MaxAbsDiff(s.Q, Arr(r["q"]))),
                                        Math.Max(MaxAbsDiff(s.Nu, Arr(r["nu"])), s.Thrust.Length == 0 ? 0 : MaxAbsDiff(s.Thrust, Arr(r["thrust"]))));
                    worst = Math.Max(worst, e);
                    Assert.That(e, Is.LessThan(ParityTol), $"{name} t={(double)r["t"]:F2}");
                }
                if (k < steps) s = sim.Step(s, dt, tau, fCmd);
            }
            Assert.AreEqual(samples.Count, si, "sample count");
            Track("trajectory:" + name, worst);
        }

        // ---- parity ---------------------------------------------------------------------
        [Test] public void Profile_Resource_Hash_Matches_Python_Source() {
            Assert.AreEqual((string)golden["profileSha256"], profile.Sha256);
            Assert.AreEqual("UNKNOWN", profile.EvidenceLabel);
            Assert.AreEqual(13.5, profile.Params.Mass, 0);
            Assert.AreEqual(8, profile.Thrusters.Count);
        }
        [Test] public void Parity_Hover() => RunTrajectory("hover");
        [Test] public void Parity_SurgeStep_WithThrusterLag() => RunTrajectory("surge_step");
        [Test] public void Parity_RollRelease() => RunTrajectory("roll_release");
        [Test] public void Parity_YawSpinDown() => RunTrajectory("yaw_spin_down");
        [Test] public void Parity_CurrentDrift() => RunTrajectory("current_drift");
        [Test] public void Parity_CoupledFree_Coriolis() => RunTrajectory("coupled_free");

        [Test] public void Parity_ThrustAllocation_Cases() {
            double worst = 0; int n = 0, saturated = 0;
            foreach (var a in ((List<object>)golden["allocation"]).Select(D)) {
                var idx = Arr(a["indices"]).Select(x => (int)x).ToArray();
                var bank = new CraneThrusterBank(idx.Select(i => profile.Thrusters[i]).ToList());
                var Tref = Mat(a["T"]);
                for (int i = 0; i < 6; i++) for (int j = 0; j < idx.Length; j++) worst = Math.Max(worst, Math.Abs(bank.T[i, j] - Tref[i, j]));
                var r = bank.Allocate(Arr(a["tau"]));
                worst = Math.Max(worst, MaxAbsDiff(r.Unconstrained, Arr(a["fUnconstrained"])));
                worst = Math.Max(worst, MaxAbsDiff(r.Forces, Arr(a["f"])));
                worst = Math.Max(worst, MaxAbsDiff(r.AllocatedWrench, Arr(a["wrench"])));
                if (r.Saturated) saturated++;
                n++;
            }
            Assert.AreEqual(20, n);
            Assert.Greater(saturated, 0, "saturating cases must be exercised");
            Assert.That(worst, Is.LessThan(1e-9));
            Track("allocation", worst);
        }

        [Test] public void Allocation_Reports_Requested_Allocated_Applied_And_Flags() {
            var bank = new CraneThrusterBank(profile.Thrusters);
            Assert.AreEqual(6, bank.Rank);
            var r = bank.Allocate(new double[] { 200, 0, 0, 0, 0, 0 });
            Assert.IsTrue(r.Saturated);
            Assert.Less(r.Scale, 1.0);
            Assert.IsTrue(r.SaturatedThrusters.Any(b => b));
            Assert.AreEqual(200, r.RequestedWrench[0]);
            Assert.AreEqual(r.Scale * 200, r.AllocatedWrench[0], 1e-9);   // direction preserved
            Assert.AreEqual(0, r.AppliedWrench[0], 0);                     // nothing applied before the thrusters run
            bank.Step(0.5, r.Forces);                                      // 25 tau
            Assert.Greater(bank.Last.AppliedWrench[0], 0);
            var horiz = new CraneThrusterBank(profile.Thrusters.Take(4).ToList());
            Assert.AreEqual(3, horiz.Rank);
            var u = horiz.Allocate(new double[] { 0, 0, 10, 0, 0, 0 });    // heave unreachable
            Assert.Greater(u.ResidualNorm, 9.9);
        }

        // ---- invariants ---------------------------------------------------------------
        [Test] public void Coriolis_Is_Skew_NuT_C_Nu_Is_Zero() {
            var rng = new System.Random(7);
            var p = profile.Params;
            var mrb = p.MRb();
            double worst = 0;
            for (int k = 0; k < 200; k++) {
                var nu = Enumerable.Range(0, 6).Select(_ => rng.NextDouble() * 4 - 2).ToArray();
                foreach (var M in new[] { mrb, p.AddedMass, Add(mrb, p.AddedMass) }) {
                    var C = CraneFossen.CoriolisFromMass(M, nu);
                    worst = Math.Max(worst, Math.Abs(Dot(nu, Mul(C, nu))));
                    for (int i = 0; i < 6; i++) for (int j = 0; j < 6; j++) worst = Math.Max(worst, Math.Abs(C[i, j] + C[j, i]));
                }
            }
            Assert.That(worst, Is.LessThan(1e-12));
        }

        [Test] public void Mass_Matrices_Are_Symmetric_Positive_Definite_With_Cg_Offset() {
            var p = profile.Params;
            p = new CraneVehicleParams {
                Mass = p.Mass, InertiaCg = p.InertiaCg, RG = new[] { 0.01, -0.02, 0.03 }, RB = p.RB, Buoyancy = p.Buoyancy,
                AddedMass = p.AddedMass, LinearDamping = p.LinearDamping, QuadraticDamping = p.QuadraticDamping, Gravity = p.Gravity
            };
            var m = p.MRb();
            for (int i = 0; i < 6; i++) for (int j = 0; j < 6; j++) Assert.AreEqual(m[i, j], m[j, i], 1e-12);
            Assert.Greater(MinSymmetricEigenvalue(m), 0);
        }

        [Test] public void Energy_Decays_Monotonically_With_Zero_Thrust() {
            var sim = new CraneFossenSimulator(profile.Params, new double[] { 0.2, -0.1, 0.0 });
            var s = new CraneState { Pos = new double[] { 0, 0, 2 }, Q = QuatNormalize(new[] { 0.9, 0.2, -0.1, 0.3 }), Nu = new[] { 0.4, -0.2, 0.1, 0.5, -0.4, 0.6 } };
            double e0 = sim.TotalEnergy(s), prev = e0;
            for (int k = 0; k < 2000; k++) {
                s = sim.Step(s, 0.005);
                double e = sim.TotalEnergy(s);
                Assert.LessOrEqual(e, prev + 1e-9, $"energy rose at step {k}");
                prev = e;
            }
            Assert.Less(prev, e0);
        }

        [Test] public void Energy_Is_Conserved_Without_Damping_Even_With_Coupled_Added_Mass() {
            var p = profile.Params;
            var am = (double[,])p.AddedMass.Clone(); am[0, 4] = am[4, 0] = 0.3; am[1, 3] = am[3, 1] = -0.2;
            var q = new CraneVehicleParams { Mass = p.Mass, InertiaCg = p.InertiaCg, RG = new[] { 0.0, 0.0, 0.02 }, RB = p.RB, Buoyancy = p.Buoyancy - 2.0,
                AddedMass = am, LinearDamping = new double[6, 6], QuadraticDamping = new double[6], Gravity = p.Gravity };
            var sim = new CraneFossenSimulator(q, damping: false);
            var s = new CraneState { Pos = new double[] { 0, 0, 3 }, Q = QuatNormalize(new[] { 0.95, 0.1, 0.2, -0.2 }), Nu = new[] { 0.3, 0.1, -0.2, 0.4, -0.3, 0.2 } };
            double e0 = sim.TotalEnergy(s);
            for (int k = 0; k < 1000; k++) s = sim.Step(s, 0.002);
            Assert.AreEqual(e0, sim.TotalEnergy(s), 1e-6 * Math.Abs(e0) + 1e-6);
        }

        [Test] public void Rotation_Equivariance_About_Down_Axis() {
            double alpha = 0.9;
            var rz = QuatFromAxisAngle(new[] { 0.0, 0.0, 1.0 }, alpha);
            var rot = QuatToRot(rz);
            var cur = new[] { 0.3, -0.2, 0.05 };
            var q0 = QuatNormalize(new[] { 0.9, 0.2, -0.1, 0.3 });
            var nu0 = new[] { 0.3, 0.1, -0.1, 0.2, -0.15, 0.3 };
            // Limited to 3 s: the oracle's RK4 re-normalises q to w >= 0 inside each stage, which injects an error
            // (1e-4..1e-2) once |w| approaches 0 (rotation near 180 deg). Documented, replicated for parity, avoided here.
            var tau = new[] { 5.0, -3.0, -1.0, 0.2, 0.1, -0.3 };
            var a = new CraneFossenSimulator(profile.Params, cur);
            var b = new CraneFossenSimulator(profile.Params, Mul(rot, cur));
            var sa = new CraneState { Pos = new double[] { 1, 2, 3 }, Q = q0, Nu = nu0 };
            var sb = new CraneState { Pos = Mul(rot, new double[] { 1, 2, 3 }), Q = QuatNormalize(QuatMul(rz, q0)), Nu = nu0 };
            double worst = 0;
            double minW = 1;
            for (int k = 0; k < 300; k++) {
                sa = a.Step(sa, 0.01, tau); sb = b.Step(sb, 0.01, tau);
                minW = Math.Min(minW, Math.Min(sa.Q[0], sb.Q[0]));
                worst = Math.Max(worst, MaxAbsDiff(sa.Nu, sb.Nu));
                worst = Math.Max(worst, MaxAbsDiff(Mul(rot, sa.Pos), sb.Pos));
                // quaternions agree up to the hemisphere sign
                var expected = QuatNormalize(QuatMul(rz, sa.Q));
                worst = Math.Max(worst, MaxAbsDiff(expected, sb.Q));
            }
            Assert.Greater(minW, 0.3, "scenario must stay away from the w=0 hemisphere boundary");
            Assert.That(worst, Is.LessThan(1e-9));
            Track("equivariance", worst);
        }

        [Test] public void Current_Relative_Velocity_And_Galilean_Drift() {
            var q = QuatNormalize(new[] { 0.8, 0.1, 0.4, -0.3 });
            var vc = new[] { 0.5, -0.1, 0.2 };
            var nuR = CraneFossen.RelativeVelocity(q, new double[6], vc);
            var expect = Mul(Transpose(QuatToRot(q)), vc);
            for (int i = 0; i < 3; i++) Assert.AreEqual(-expect[i], nuR[i], 1e-12);
            // a body moving with the current (upright, no wrench beyond hover) feels no drag
            var s = new CraneState { Q = new[] { 1.0, 0, 0, 0 }, Nu = new[] { 0.5, -0.1, 0.2, 0, 0, 0 } };
            var sim = new CraneFossenSimulator(profile.Params, vc);
            var acc = sim.Acceleration(s, new[] { 0.0, 0, -(profile.Params.Weight - profile.Params.Buoyancy), 0, 0, 0 });
            Assert.That(Norm(acc), Is.LessThan(1e-9));
        }

        // ---- thrusters ------------------------------------------------------------------
        [Test] public void Thruster_FirstOrderLag_63Percent_Deadband_Saturation_AsymmetricReverse() {
            var th = new CraneThruster("t", new double[] { 0, 0, 0 }, new double[] { 1, 0, 0 }, 30, 10, 0.02, 1.0);
            var bank = new CraneThrusterBank(new List<CraneThruster> { th });
            bank.Step(0.02, new[] { 20.0 });
            Assert.AreEqual(20 * (1 - Math.Exp(-1)), bank.Thrust[0], 1e-12);
            bank.Reset();
            bank.Step(100, new[] { 0.5 });  Assert.AreEqual(0, bank.Thrust[0], 0, "deadband");
            bank.Step(100, new[] { 99.0 }); Assert.AreEqual(30, bank.Thrust[0], 1e-9, "forward saturation");
            bank.Step(100, new[] { -99.0 }); Assert.AreEqual(-10, bank.Thrust[0], 1e-9, "asymmetric reverse limit");
        }

        // ---- frames -----------------------------------------------------------------------
        [Test] public void Frames_UnityNedFlu_Mappings_Are_Explicit_And_Consistent() {
            Assert.AreEqual(-1, CraneUnderwaterFrames.Determinant(CraneUnderwaterFrames.UnityToNedMatrix()), 0);
            AssertVec(new double[] { 1, 0, 0 }, CraneUnderwaterFrames.UnityToNed(0, 0, 1));    // Unity forward = north
            AssertVec(new double[] { 0, 1, 0 }, CraneUnderwaterFrames.UnityToNed(1, 0, 0));    // Unity right = east
            AssertVec(new double[] { 0, 0, -1 }, CraneUnderwaterFrames.UnityToNed(0, 1, 0));   // Unity up = -down
            AssertVec(new double[] { 1, 0, 0 }, CraneUnderwaterFrames.UnityToFlu(0, 0, 1));
            AssertVec(new double[] { 0, 1, 0 }, CraneUnderwaterFrames.UnityToFlu(-1, 0, 0));   // Unity left = FLU +y
            AssertVec(new double[] { 0, 0, 1 }, CraneUnderwaterFrames.UnityToFlu(0, 1, 0));
            // yaw right (Unity +Y rotation, clockwise from above): NED +r about Down, FLU -wz
            AssertVec(new double[] { 0, 0, 1 }, CraneUnderwaterFrames.UnityAxialToNed(0, 1, 0));
            AssertVec(new double[] { 0, 0, -1 }, CraneUnderwaterFrames.UnityAxialToFlu(0, 1, 0));
            // Unity rotation about -Z (x toward -y: right wing down) is NED +p (roll right)
            AssertVec(new double[] { 1, 0, 0 }, CraneUnderwaterFrames.UnityAxialToNed(0, 0, -1));
            var rng = new System.Random(3);
            for (int i = 0; i < 20; i++) {
                double x = rng.NextDouble() - .5, y = rng.NextDouble() - .5, z = rng.NextDouble() - .5;
                AssertVec(new[] { x, y, z }, CraneUnderwaterFrames.NedToUnity(CraneUnderwaterFrames.UnityToNed(x, y, z)));
                AssertVec(new[] { x, y, z }, CraneUnderwaterFrames.NedAxialToUnity(CraneUnderwaterFrames.UnityAxialToNed(x, y, z)));
                AssertVec(new[] { x, y, z }, CraneUnderwaterFrames.FluToUnity(CraneUnderwaterFrames.UnityToFlu(x, y, z)));
                AssertVec(new[] { x, y, z }, CraneUnderwaterFrames.FluAxialToUnity(CraneUnderwaterFrames.UnityAxialToFlu(x, y, z)));
                AssertVec(CraneUnderwaterFrames.UnityToFlu(x, y, z), CraneUnderwaterFrames.NedToFlu(CraneUnderwaterFrames.UnityToNed(x, y, z)));
                AssertVec(CraneUnderwaterFrames.UnityAxialToFlu(x, y, z), CraneUnderwaterFrames.NedAxialToFlu(CraneUnderwaterFrames.UnityAxialToNed(x, y, z)));
            }
        }

        [Test] public void Frames_Quaternion_Conversion_Commutes_With_Vector_Rotation() {
            var rng = new System.Random(11);
            for (int i = 0; i < 25; i++) {
                var uq = UnityEngine.Random.rotationUniform;
                if (i > 0) uq = Quaternion.Euler((float)(rng.NextDouble() * 360), (float)(rng.NextDouble() * 360), (float)(rng.NextDouble() * 360));
                var v = new Vector3((float)rng.NextDouble() - .5f, (float)rng.NextDouble() - .5f, (float)rng.NextDouble() - .5f);
                var qn = CraneUnderwaterFrames.UnityQuatToNedFrd(uq.x, uq.y, uq.z, uq.w);
                var rotatedUnity = uq * v;
                var viaNed = Mul(QuatToRot(qn), CraneUnderwaterFrames.UnityToNed(v.x, v.y, v.z));
                AssertVec(CraneUnderwaterFrames.UnityToNed(rotatedUnity.x, rotatedUnity.y, rotatedUnity.z), viaNed, 1e-6);
                var back = CraneUnderwaterFrames.NedFrdQuatToUnity(qn);
                double dot = Math.Abs(back[0] * uq.x + back[1] * uq.y + back[2] * uq.z + back[3] * uq.w);
                Assert.That(dot, Is.EqualTo(1.0).Within(1e-6));
            }
            // Unity yaw-right of 30 deg => NED yaw +30 deg (rotation about Down by +30)
            var yaw = Quaternion.AngleAxis(30f, Vector3.up);
            var q = CraneUnderwaterFrames.UnityQuatToNedFrd(yaw.x, yaw.y, yaw.z, yaw.w);
            AssertVec(QuatFromAxisAngle(new[] { 0.0, 0, 1 }, 30 * Math.PI / 180), q, 1e-6);
        }

        static void AssertVec(double[] expected, double[] actual, double tol = 1e-12) {
            Assert.AreEqual(expected.Length, actual.Length);
            for (int i = 0; i < expected.Length; i++) Assert.AreEqual(expected[i], actual[i], tol, $"component {i}");
        }

        // ---- reset / isolation ---------------------------------------------------------
        [Test] public void Reset_Clears_Thruster_State_Core() {
            var bank = new CraneThrusterBank(profile.Thrusters);
            bank.Allocate(new double[] { 30, 0, 0, 0, 0, 0 });
            bank.Step(0.1, new double[] { 20, 20, -20, -20, 5, 5, 5, 5 });
            Assert.IsTrue(bank.Thrust.Any(t => t != 0));
            bank.Reset();
            Assert.IsTrue(bank.Thrust.All(t => t == 0));
            Assert.IsTrue(bank.Command.All(t => t == 0));
            Assert.IsNull(bank.Last);
        }

        [Test] public void Adapter_Is_Inert_Without_Flag_And_Reset_Clears_All_State() {
            Assert.IsNull(CraneUnderwaterBody.ReadProfileName(new[] { "Unity", "-batchmode" }));
            Assert.AreEqual("bluerov2_heavy", CraneUnderwaterBody.ReadProfileName(new[] { "x", "--crane-underwater-profile=bluerov2_heavy" }));
            Assert.AreEqual("a", CraneUnderwaterBody.ReadProfileName(new[] { "x", "--crane-underwater-profile", "a" }));

            var go = new GameObject("uw");
            try {
                var rb = go.AddComponent<Rigidbody>();
                go.transform.position = new Vector3(1, -2, 3);
                var body = go.AddComponent<CraneUnderwaterBody>();     // Awake does not run in EditMode: invoke the gate explicitly
                body.InitializeFromCommandLine(new[] { "Unity", "-batchmode" });
                Assert.IsFalse(body.enabled, "must be inert without --crane-underwater-profile");
                Assert.IsFalse(body.Active);
                Assert.AreEqual(1f, rb.mass, "inert adapter must not touch the Rigidbody");
                Assert.IsTrue(rb.useGravity);

                body.Configure(profile);
                Assert.IsTrue(body.Active);
                Assert.AreEqual(profile.Params.Mass, rb.mass, 1e-5);
                Assert.IsFalse(rb.useGravity);
                body.CaptureEpisodeInitialState();
                body.SetThrusterCommands(new float[] { 20, 20, -20, -20, 3, 3, 3, 3 });
                rb.linearVelocity = new Vector3(0.3f, 0.1f, 0.4f);
                rb.angularVelocity = new Vector3(0.1f, -0.2f, 0.05f);
                for (int i = 0; i < 25; i++) body.StepFixed(0.02f);
                Assert.AreEqual(25, body.StepCount);
                Assert.IsTrue(body.Bank.Thrust.Any(t => Math.Abs(t) > 1e-6));
                Assert.IsTrue(body.LastWrenchFrd.Any(w => Math.Abs(w) > 1e-6));
                go.transform.position = new Vector3(9, 9, 9);

                body.ResetEpisode(default, CraneEpisodeResetPhase.BeforePhysics);
                Assert.AreEqual(0, body.StepCount);
                Assert.IsTrue(body.Bank.Thrust.All(t => t == 0));
                Assert.IsTrue(body.Bank.Command.All(t => t == 0));
                Assert.IsTrue(body.LastWrenchFrd.All(w => w == 0));
                Assert.IsTrue(body.LastNu.All(w => w == 0));
                Assert.AreEqual(Vector3.zero, rb.linearVelocity);
                Assert.AreEqual(Vector3.zero, rb.angularVelocity);
                Assert.AreEqual(new Vector3(1, -2, 3), go.transform.position);
                // commands were cleared too: a stepped adapter now only applies hydrostatics
                body.StepFixed(0.02f);
                Assert.IsTrue(body.Bank.Thrust.All(t => t == 0));
            } finally { UnityEngine.Object.DestroyImmediate(go); }
        }

        // ---- architecture ------------------------------------------------------------------
        [Test] public void Assembly_Has_No_Hdrp_Reference() {
            var asm = typeof(CraneUnderwaterBody).Assembly;
            var refs = asm.GetReferencedAssemblies().Select(a => a.Name).ToList();
            Assert.AreEqual("CraneUnderwater", asm.GetName().Name);
            Assert.IsFalse(refs.Any(r => r.IndexOf("HighDefinition", StringComparison.OrdinalIgnoreCase) >= 0 || r.IndexOf("RenderPipelines", StringComparison.OrdinalIgnoreCase) >= 0),
                string.Join(",", refs));
        }
    }
}
