// Regression gate for the corrected hydrodynamics profile (DECISIONS D8/D9, I1-defect-verification V1-V3).
// These tests state CORRECT physics. Tagged KnownDefectVn tests are EXPECTED TO FAIL on the legacy
// FossenDynamics / GeneralDynamics. Tests tagged PhysicsInvariantControl are expected to pass on legacy
// code and guard against the harness itself being wrong. No production code is modified or extracted:
// components are built on bare GameObjects, a recording IPhysicsBody replaces the Rigidbody through
// reflection, and the private FixedUpdate is invoked directly. No scene or ROS connection is used.
using System;
using System.Reflection;
using NUnit.Framework;
using Sim.Physics.Processing;
using Sim.Physics.Water.Dynamics;
using Sim.Utils;
using Sim.Utils.Performance;
using UnityEngine;

namespace Sim.Tests.PhysicsInvariants {
    internal sealed class RecordingBody : IPhysicsBody {
        private readonly Transform t;
        public Vector3 RelForce, WorldForce, Torque, TorqueFromForces;
        public RecordingBody(Transform tr) { t = tr; }
        public void Clear() { RelForce = WorldForce = Torque = TorqueFromForces = Vector3.zero; }
        public Vector3 position { get => t.position; set => t.position = value; }
        public Quaternion rotation { get => t.rotation; set => t.rotation = value; }
        public Transform transform => t;
        public Vector3 centerOfMass { get; set; }
        public Vector3 linearVelocity { get; set; }
        public Vector3 angularVelocity { get; set; }
        public Vector3 inertiaTensor { get; set; }
        public Quaternion inertiaTensorRotation { get; set; }
        public Vector3 GetAccumulatedTorque() => Torque;
        public Vector3 GetAccumulatedTorque(float step) => Torque;
        public float maxLinearVelocity { get; set; }
        public float maxAngularVelocity { get; set; }
        public void AddForce(Vector3 f, ForceMode m = ForceMode.Force) { WorldForce += f; }
        public void AddRelativeForce(Vector3 f, ForceMode m = ForceMode.Force) { RelForce += f; WorldForce += t.rotation * f; }
        public void AddForceAtPosition(Vector3 f, Vector3 p, ForceMode m = ForceMode.Force) {
            WorldForce += f; TorqueFromForces += Vector3.Cross(p - t.position, f);
        }
        public void AddTorque(Vector3 tq, ForceMode m = ForceMode.Force) { Torque += tq; }
        public void AddRelativeTorque(Vector3 tq, ForceMode m = ForceMode.Force) { Torque += t.rotation * tq; }
    }

    internal sealed class FossenRig : IDisposable {
        const BindingFlags F = BindingFlags.NonPublic | BindingFlags.Public | BindingFlags.Instance;
        public readonly GameObject Go;
        public readonly FossenDynamics Fossen;
        public readonly RecordingBody Body;
        readonly MethodInfo start, fixedUpdate;

        public FossenRig(bool damping, bool addedMass, bool coriolis, float yawDeg = 0f) {
            Go = new GameObject("fossen-rig") { hideFlags = HideFlags.HideAndDontSave };
            Go.transform.rotation = Quaternion.Euler(0f, yawDeg, 0f);
            Fossen = Go.AddComponent<FossenDynamics>();
            Body = new RecordingBody(Go.transform);
            Set("Damping", damping); Set("AddedMass", addedMass); Set("Coriolis", coriolis);
            typeof(FossenDynamics).GetField("body", F).SetValue(Fossen, Body);
            start = typeof(FossenDynamics).GetMethod("Start", F);
            fixedUpdate = typeof(FossenDynamics).GetMethod("FixedUpdate", F);
        }
        public void Set(string field, object v) {
            var fi = typeof(FossenDynamics).GetField(field, F);
            Assert.NotNull(fi, "missing field " + field);
            fi.SetValue(Fossen, v);
        }
        public void Start() { start.Invoke(Fossen, null); }
        public void Step() { Body.Clear(); fixedUpdate.Invoke(Fossen, null); }
        public void Dispose() { UnityEngine.Object.DestroyImmediate(Go); }
    }

    public class PhysicsInvariantTests {
        const float Mass = 23.468f;          // Blastoise.prefab rigid-body tree mass (I1-defect-verification V1)
        const float Inertia = 2.0f;          // isotropic stand-in, kg m^2
        const float MaSurge = 6f, MaSway = 8f, MaHeave = 2f, MaRoll = 0.15f, MaPitch = 0.25f, MaYaw = 0.35f;

        // FossenDynamics.FixedUpdate divides by Time.deltaTime. The test reads the very same static
        // immediately before each Step(), so the dt used for integration is the component's own dt.
        static float StepDt() {
            float dt = Time.deltaTime;
            Assert.Greater(dt, 1e-5f, "Time.deltaTime is not usable in this EditMode run: " + dt);
            return dt;
        }

        // Added mass the component implies from one velocity jump: |F| * dt / dv. Compared against the
        // coefficient the test set explicitly, so a wrong magnitude, a wrong DOF or a dt-unit error shows up.
        static void AssertImpliedAddedMass(string axis, float setMa, float expectedMa) {
            using var rig = new FossenRig(false, true, false);
            rig.Set(axis == "surge" ? "XdotU" : "YdotV", setMa);
            rig.Start();
            rig.Step();
            const float dv = 0.5f;
            float dt = StepDt();
            Vector3 dir = axis == "surge" ? new Vector3(0, 0, 1) : new Vector3(1, 0, 0);
            rig.Body.linearVelocity = dir * dv;
            rig.Step();
            float implied = Vector3.Dot(rig.Body.RelForce, dir) * dt / dv;   // sign-blind below on purpose
            Assert.AreEqual(expectedMa, Mathf.Abs(implied), 0.01f * expectedMa,
                $"{axis}: implied added mass {Mathf.Abs(implied):F4} vs expected {expectedMa:F4}");
        }

        // Unity-frame velocities to Fossen state (surge=local z, sway=-local x, heave=local y; angular negated FLU).
        static float[] ToEta(Vector3 v, Vector3 w) => new[] { v.z, -v.x, v.y, -w.z, w.x, -w.y };

        static float Energy(Vector3 v, Vector3 w) {
            float[] e = ToEta(v, w);
            float ke = 0.5f * Mass * v.sqrMagnitude + 0.5f * Inertia * w.sqrMagnitude;
            float[] ma = { MaSurge, MaSway, MaHeave, MaRoll, MaPitch, MaYaw };
            for (int i = 0; i < 6; i++) ke += 0.5f * ma[i] * e[i] * e[i];
            return ke;
        }

        // ---------------- V1 ----------------

        [TestCase("surge", 6f, Category = "KnownDefectV1")]
        [TestCase("surge", 12f, Category = "KnownDefectV1")]
        [TestCase("sway", 9f, Category = "KnownDefectV1")]
        public void AddedMass_StepResponse_IsForceOverMassPlusAddedMass(string axis, float ma) {
            using var rig = new FossenRig(false, true, false);
            rig.Set(axis == "surge" ? "XdotU" : "YdotV", ma);   // set explicitly, never rely on prefab defaults
            rig.Start();
            Vector3 dir = axis == "surge" ? new Vector3(0, 0, 1) : new Vector3(1, 0, 0);
            const float Fext = 10f;
            float vel = 0, vAt20 = 0, vAt60 = 0, dtSum = 0;
            for (int k = 0; k < 60; k++) {
                rig.Body.linearVelocity = dir * vel;
                float dt = StepDt();
                rig.Step();
                float a = (Fext + Vector3.Dot(rig.Body.WorldForce, dir)) / Mass;
                vel += a * dt;
                if (k == 19) { vAt20 = vel; dtSum = 0; }
                if (k > 19) dtSum += dt;
                if (k == 59) vAt60 = vel;
            }
            float measured = (vAt60 - vAt20) / dtSum;
            float expected = Fext / (Mass + ma);
            Assert.AreEqual(expected, measured, 0.03f * expected,
                $"{axis} MA={ma}: a={measured:F4} m/s^2, correct F/(M+MA)={expected:F4} (Fossen 2011, MA>0). Legacy applies +MA*nuDot (V1).");
        }

        [TestCase("surge", 6f), TestCase("surge", 12f), TestCase("sway", 9f)]
        [Category("PhysicsInvariantControl")]
        public void Control_AddedMass_ImpliedMagnitudeMatchesCoefficient(string axis, float ma) {
            AssertImpliedAddedMass(axis, ma, ma);
        }

        [Test, Category("PhysicsInvariantControl")]
        public void Mutant_DoubledAddedMass_IsDetected() {
            // Component built with 2x the coefficient the test expects must fail the magnitude check.
            Assert.Throws<AssertionException>(() => AssertImpliedAddedMass("surge", 12f, 6f));
            Assert.Throws<AssertionException>(() => AssertImpliedAddedMass("sway", 16f, 8f));
        }

        [Test, Category("KnownDefectV1")]
        public void AddedMass_Force_OpposesAcceleration() {
            using var rig = new FossenRig(false, true, false);
            rig.Start();
            rig.Step();
            rig.Body.linearVelocity = new Vector3(0, 0, 0.1f);   // acceleration along +surge
            rig.Step();
            Assert.Less(rig.Body.RelForce.z, 0f, "hydrodynamic added-mass force must be -MA*nuDot (opposes acceleration)");
        }

        [Test, Category("KnownDefectV1")]
        public void Reset_ClearsStatePrev_NoImpulseAfterReset() {
            using var rig = new FossenRig(false, true, false);
            rig.Start();
            rig.Step();
            rig.Body.linearVelocity = new Vector3(0, 0, 1.5f);
            for (int i = 0; i < 5; i++) rig.Step();              // statePrev now tracks 1.5 m/s
            // What CraneEpisodeResetCoordinator does to the body:
            rig.Body.linearVelocity = Vector3.zero;
            rig.Body.angularVelocity = Vector3.zero;
            // ... and what it does to ICraneEpisodeResettable implementers.
            if (rig.Fossen is ICraneEpisodeResettable r) {
                var ctor = typeof(CraneEpisodeResetContext).GetConstructor(
                    BindingFlags.NonPublic | BindingFlags.Public | BindingFlags.Instance, null,
                    new[] { typeof(long), typeof(int) }, null);
                var ctx = (CraneEpisodeResetContext)ctor.Invoke(new object[] { 1L, 0 });
                r.ResetEpisode(in ctx, CraneEpisodeResetPhase.BeforePhysics);
                r.ResetEpisode(in ctx, CraneEpisodeResetPhase.AfterPhysics);
            }
            rig.Step();
            Assert.Less(rig.Body.RelForce.magnitude, 0.01f,
                $"reset impulse {rig.Body.RelForce.z:F1} N; FossenDynamics must implement ICraneEpisodeResettable and clear statePrev");
        }

        [Test, Category("KnownDefectV1")]
        public void SpawnMoving_FirstTickHasNoImpulse() {
            using var rig = new FossenRig(false, true, false);
            rig.Body.linearVelocity = new Vector3(0, 0, 1.0f);
            rig.Start();                                          // statePrev starts at zero in legacy
            rig.Step();
            Assert.Less(rig.Body.RelForce.magnitude, 0.01f, $"first-tick impulse {rig.Body.RelForce.z:F1} N");
        }

        // ---------------- V2 ----------------

        // Expected hydrodynamic wrench -C_A(nu)nu for diagonal M_A (Fossen 2011), body FLU:
        //   force  = -(w x (A1 v)),  moment = -(v x (A1 v) + w x (A2 w)).
        static void ExpectedCoriolis(float[] eta, float[] ma, out Vector3 force, out Vector3 moment) {
            var v = new Vector3(eta[0], eta[1], eta[2]);
            var w = new Vector3(eta[3], eta[4], eta[5]);
            var a1v = new Vector3(ma[0] * v.x, ma[1] * v.y, ma[2] * v.z);
            var a2w = new Vector3(ma[3] * w.x, ma[4] * w.y, ma[5] * w.z);
            force = -Vector3.Cross(w, a1v);
            moment = -(Vector3.Cross(v, a1v) + Vector3.Cross(w, a2w));
        }

        // Inverse of FossenDynamics.GetState and its output axis swaps (F -> (-y,z,x), T -> (y,-z,-x)).
        static void EtaToUnity(float[] e, out Vector3 v, out Vector3 w) {
            v = new Vector3(-e[1], e[2], e[0]);
            w = new Vector3(e[4], -e[5], -e[3]);
        }
        static Vector3 FossenForceToUnity(Vector3 f) => new Vector3(-f.y, f.z, f.x);
        static Vector3 FossenMomentToUnity(Vector3 t) => new Vector3(t.y, -t.z, -t.x);

        static readonly float[][] MaSets = {
            new[] { 6f, 8f, 2f, 0.15f, 0.25f, 0.35f },
            new[] { 9f, 4f, 3f, 0.5f, 0.1f, 0.7f },     // a11 != a22 in the opposite order
        };
        static readonly string[] MaNames = { "XdotU", "YdotV", "ZdotW", "KdotP", "MdotQ", "NdotR" };

        static FossenRig CoriolisRig(float[] ma) {
            var rig = new FossenRig(false, false, true);
            for (int i = 0; i < 6; i++) rig.Set(MaNames[i], ma[i]);
            return rig;
        }

        // Returns (unity force, unity torque) produced for a Fossen-frame state.
        static (Vector3 f, Vector3 t) RunCoriolis(float[] ma, float[] eta) {
            using var rig = CoriolisRig(ma);
            EtaToUnity(eta, out var v, out var w);
            rig.Body.linearVelocity = v; rig.Body.angularVelocity = w;
            rig.Start(); rig.Step();
            return (rig.Body.RelForce, rig.Body.Torque);
        }

        static void AssertSignedCoriolis(float[] ma, float[] eta, string label) {
            ExpectedCoriolis(eta, ma, out var f, out var m);
            var (fu, tu) = RunCoriolis(ma, eta);
            Vector3 fe = FossenForceToUnity(f), te = FossenMomentToUnity(m);
            Assert.AreEqual(0f, (fu - fe).magnitude, 1e-3f, $"{label}: force {fu} vs expected -C_A nu {fe}");
            Assert.AreEqual(0f, (tu - te).magnitude, 1e-3f, $"{label}: torque {tu} vs expected -C_A nu {te}");
        }

        [TestCase(1f, 0.2f, 0.5f), TestCase(-0.7f, 0.3f, -0.4f), TestCase(0.5f, -0.6f, 0.8f)]
        [Category("KnownDefectV2")]
        public void Coriolis_Planar_SignedWrenchMatchesFossen(float u, float v, float r) {
            foreach (var ma in MaSets)
                AssertSignedCoriolis(ma, new[] { u, v, 0f, 0f, 0f, r }, $"planar u={u} v={v} r={r} a11={ma[0]} a22={ma[1]}");
        }

        [Test, Category("KnownDefectV2")]
        public void Coriolis_Random6Dof_SignedWrenchMatchesFossen_AndIsPowerFreeAndQuadratic() {
            var rng = new System.Random(20261021);
            float R() => (float)(rng.NextDouble() * 2.0 - 1.0);
            foreach (var ma in MaSets) {
                for (int n = 0; n < 50; n++) {
                    var eta = new[] { R(), 0.7f * R(), 0.5f * R(), 0.6f * R(), 0.6f * R(), 0.8f * R() };
                    AssertSignedCoriolis(ma, eta, $"seed state {n} ma={ma[0]}/{ma[1]}");
                    // Power-free: nu . tau = 0 (Unity-frame form, frame independent).
                    EtaToUnity(eta, out var v, out var w);
                    var (fu, tu) = RunCoriolis(ma, eta);
                    Assert.AreEqual(0f, Vector3.Dot(fu, v) + Vector3.Dot(tu, w), 2e-3f, $"power nonzero at state {n}");
                    // Quadratic homogeneity: tau(2 nu) = 4 tau(nu).
                    var eta2 = Array.ConvertAll(eta, x => 2f * x);
                    var (f2, t2) = RunCoriolis(ma, eta2);
                    Assert.AreEqual(0f, (f2 - 4f * fu).magnitude + (t2 - 4f * tu).magnitude, 5e-3f, $"tau(2nu) != 4 tau(nu) at {n}");
                }
            }
        }

        [Test, Category("KnownDefectV2")]
        public void Coriolis_PowerIsZero_General6Dof() {
            using var rig = new FossenRig(false, false, true);
            var v = new Vector3(0.3f, -0.1f, 0.9f);
            var w = new Vector3(0.1f, 0.2f, -0.15f);
            rig.Body.linearVelocity = v; rig.Body.angularVelocity = w;
            rig.Start(); rig.Step();
            float p = Vector3.Dot(rig.Body.RelForce, v) + Vector3.Dot(rig.Body.Torque, w);
            Assert.AreEqual(0f, p, 1e-3f, "nu^T C(nu) nu must be 0 for general 6-DOF motion");
            Assert.Greater(rig.Body.RelForce.magnitude + rig.Body.Torque.magnitude, 0.1f, "Coriolis wrench is vacuously zero");
        }

        [Test, Category("KnownDefectV2")]
        public void Damping_IsRotationEquivariant_OverHeading() {
            Vector3 vLocal = new Vector3(0.8f, 0.1f, -0.4f);
            Vector3 wLocal = new Vector3(0.2f, 0.3f, -0.5f);      // includes roll/pitch components
            using var r0 = new FossenRig(true, false, false, 0f);
            r0.Body.linearVelocity = vLocal; r0.Body.angularVelocity = wLocal;
            r0.Start(); r0.Step();
            Vector3 f0 = r0.Body.RelForce, t0 = r0.Body.Torque;
            foreach (float yaw in new[] { 35f, 90f, 200f }) {
                Quaternion q = Quaternion.Euler(0, yaw, 0);
                using var r = new FossenRig(true, false, false, yaw);
                r.Body.linearVelocity = q * vLocal; r.Body.angularVelocity = q * wLocal;
                r.Start(); r.Step();
                Vector3 f = r.Body.RelForce, t = Quaternion.Inverse(q) * r.Body.Torque;
                Assert.AreEqual(0f, (f - f0).magnitude, 1e-3f, $"force differs at heading {yaw}");
                Assert.AreEqual(0f, (t - t0).magnitude, 1e-3f, $"body-frame torque differs at heading {yaw}: {t} vs {t0}");
            }
        }

        [Test, Category("PhysicsInvariantControl")]
        public void Control_FossenQuadraticDrag_ScalesWithVAbsV() {
            Vector3 Drag(Vector3 v) {
                using var rig = new FossenRig(true, false, false);
                foreach (var n in new[] { "Xu", "Yv", "Zw", "Kp", "Mq", "Nr" }) rig.Set(n, 0f);
                rig.Body.linearVelocity = v;
                rig.Start(); rig.Step();
                return rig.Body.RelForce;
            }
            foreach (var dir in new[] { new Vector3(0, 0, 1), new Vector3(-1, 0, 0) }) {
                Vector3 f1 = Drag(0.5f * dir), f2 = Drag(1.0f * dir), fm = Drag(-0.5f * dir);
                Assert.Greater(f1.magnitude, 0.1f);
                Assert.AreEqual(4f, f2.magnitude / f1.magnitude, 0.02f, "F(2v)/F(v) must be 4");
                Assert.AreEqual(0f, (f1 + fm).magnitude, 1e-4f, "drag must be odd in v (v|v|)");
            }
        }

        [Test, Category("KnownDefectV2")]
        public void ZeroCommand_EnergyDoesNotIncrease_FullFossenStack() {
            using var rig = new FossenRig(true, true, true);
            Vector3 v = new Vector3(-0.3f, 0.05f, 1.0f), w = new Vector3(0.05f, -0.4f, 0.03f);
            rig.Body.linearVelocity = v; rig.Body.angularVelocity = w;
            rig.Start();
            float e0 = Energy(v, w), prev = e0, worst = 0f;
            for (int k = 0; k < 400; k++) {
                rig.Body.linearVelocity = v; rig.Body.angularVelocity = w;
                float dt = StepDt();
                rig.Step();
                v += rig.Body.WorldForce / Mass * dt;
                w += rig.Body.Torque / Inertia * dt;
                float e = Energy(v, w);
                if (float.IsNaN(e)) Assert.Fail("energy is NaN at step " + k);
                worst = Mathf.Max(worst, e - prev);
                prev = e;
            }
            Assert.LessOrEqual(worst, 1e-4f * e0, $"energy rose by {worst:E2} J in one step (E0={e0:F3} J)");
            Assert.Less(prev, e0, "energy must decay with zero command");
        }

        // ---------------- V3 ----------------

        static (GameObject go, RecordingBody body, GeneralDynamics gd) BuildGeneral(float yawDeg) {
            var go = new GameObject("general-rig") { hideFlags = HideFlags.HideAndDontSave };
            go.transform.rotation = Quaternion.Euler(0, yawDeg, 0);
            var subType = typeof(GeneralDynamics).Assembly.GetType("Sim.Physics.Processing.Submersion");
            Assert.NotNull(subType, "Submersion type not found");
            go.AddComponent(subType);
            var gd = go.AddComponent<GeneralDynamics>();
            var body = new RecordingBody(go.transform);
            const BindingFlags F = BindingFlags.NonPublic | BindingFlags.Instance;
            typeof(GeneralDynamics).GetField("body", F).SetValue(gd, body);
            var data = new Data(3) {
                vertices = new[] { new Vector3(1, 0, 0), new Vector3(1, 1, 0), new Vector3(1, 0, 1) },
                triangles = new[] { 0, 1, 2 },
                normals = new[] { new Vector3(1, 0, 0) },    // mesh-local, as SubmersionLib stores them
                transform = go.transform,
                maxTriangleIndex = 3,
            };
            data.faceCentersWorld[0] = go.transform.TransformPoint(new Vector3(1, 1f / 3f, 1f / 3f));
            var sm = new Submerged(new Mesh()) { data = data };
            typeof(GeneralDynamics).GetField("submerged", F).SetValue(gd, sm);
            gd.pressureDragActive = true; gd.viscousResistActive = false;
            return (go, body, gd);
        }

        static Vector3 PressureForce(float yawDeg, Vector3 localVelocity, Action<GeneralDynamics> cfg = null) {
            var (go, body, gd) = BuildGeneral(yawDeg);
            try {
                cfg?.Invoke(gd);
                body.linearVelocity = Quaternion.Euler(0, yawDeg, 0) * localVelocity;
                typeof(GeneralDynamics).GetMethod("FixedUpdate", BindingFlags.NonPublic | BindingFlags.Instance).Invoke(gd, null);
                return Quaternion.Inverse(Quaternion.Euler(0, yawDeg, 0)) * body.WorldForce;   // expressed in body frame
            } finally { UnityEngine.Object.DestroyImmediate(go); }
        }

        [Test, Category("KnownDefectV3")]
        public void PressureDrag_IsRotationEquivariant_OverHeading() {
            Vector3 vLocal = new Vector3(-0.6f, 0.3f, 0.2f);
            Vector3 f0 = PressureForce(0f, vLocal);
            Assert.Greater(f0.magnitude, 0.1f, "reference force vacuously zero");
            foreach (float yaw in new[] { 30f, 90f, 160f }) {
                Vector3 f = PressureForce(yaw, vLocal);
                Assert.AreEqual(0f, (f - f0).magnitude, 1e-3f * f0.magnitude + 1e-4f,
                    $"body-frame pressure force differs at heading {yaw}: {f} vs {f0} (normals double-transformed, V3)");
            }
        }

        [Test, Category("KnownDefectV3")]
        public void PressureDrag_QuadraticTerm_ScalesWithVSquared() {
            void Cfg(GeneralDynamics g) {
                g.pressureDragLinearCoefficient = 0f; g.suctionDragLinearCoefficient = 0f;
                g.pressureDragQuadraticCoefficient = 30f; g.suctionDragQuadraticCoefficient = 30f;
                g.pressureDragVelocityRef = 1f;
            }
            float f1 = PressureForce(0f, new Vector3(-0.5f, 0, 0), Cfg).magnitude;
            float f2 = PressureForce(0f, new Vector3(-1.0f, 0, 0), Cfg).magnitude;
            Assert.Greater(f2, 0f, "force vacuously zero");
            Assert.AreEqual(4f, f2 / f1, 0.05f, $"F(2v)/F(v)={f2 / f1:F3}; quadratic term is (v^2)/(v^2) in legacy (V3)");
        }
    }
}
