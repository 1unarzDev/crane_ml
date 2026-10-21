using System;
using System.Collections.Generic;
using System.Globalization;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEditor.SceneManagement;

namespace Sim.Tests.PhysXGyro
{
    // Independent verification of the claim "Unity Rigidbody applies no w x Iw (gyroscopic) term".
    // Torque-free asymmetric body, I = diag(1,2,3), spun near the intermediate (y) axis.
    // Real rigid-body dynamics (Euler's equations) gives the Dzhanibekov flip and a conserved world-frame L.
    // Without the gyroscopic term the world-frame w is constant: the body spins steadily and L drifts.
    // Results are logged as "GYRO ..." lines; the assertions only check that the runs happened.
    [TestFixture, Category("PhysXGyroVerify")]
    public class PhysXGyroVerifyTests
    {
        static string F(double v) { return v.ToString("F4", CultureInfo.InvariantCulture); }
        static string V(Vector3 v) { return "(" + F(v.x) + "," + F(v.y) + "," + F(v.z) + ")"; }
        static readonly Vector3 I = new Vector3(1, 2, 3);
        const float Duration = 20f;

        sealed class Rig : IDisposable
        {
            public Scene scene; public PhysicsScene ps; public readonly List<GameObject> roots = new List<GameObject>();
            public Rig() { scene = EditorSceneManager.NewPreviewScene(); ps = scene.GetPhysicsScene(); }
            public GameObject Make(string n) { var go = new GameObject(n); SceneManager.MoveGameObjectToScene(go, scene); roots.Add(go); return go; }
            public void Dispose() { foreach (var g in roots) if (g != null) UnityEngine.Object.DestroyImmediate(g); EditorSceneManager.ClosePreviewScene(scene); }
        }

        public struct Result
        {
            public string label; public float firstFlipTime; public float minWy; public float maxLDrift; public float maxEDrift; public int flips; public float maxPerp;
        }

        static Vector3 Mul(Vector3 a, Vector3 b) { return new Vector3(a.x * b.x, a.y * b.y, a.z * b.z); }

        // Tracks body-frame w, world-frame L = R (I wb), kinetic energy.
        sealed class Tracker
        {
            readonly string label; readonly float t0Ly; Vector3 L0; float E0; bool init; float lastSign = 1;
            public Result r;
            public Tracker(string label) { this.label = label; r.label = label; r.firstFlipTime = -1; r.minWy = float.MaxValue; }
            public void Sample(float t, Quaternion q, Vector3 wWorld)
            {
                Vector3 wb = Quaternion.Inverse(q) * wWorld;
                Vector3 L = q * Mul(I, wb);
                float E = 0.5f * Vector3.Dot(wb, Mul(I, wb));
                if (!init) { L0 = L; E0 = E; init = true; }
                r.maxLDrift = Mathf.Max(r.maxLDrift, (L - L0).magnitude / L0.magnitude);
                r.maxEDrift = Mathf.Max(r.maxEDrift, Mathf.Abs(E - E0) / E0);
                r.minWy = Mathf.Min(r.minWy, wb.y);
                r.maxPerp = Mathf.Max(r.maxPerp, new Vector2(wb.x, wb.z).magnitude);
                float s = Mathf.Sign(wb.y);
                if (s != lastSign) { r.flips++; if (r.firstFlipTime < 0) r.firstFlipTime = t; lastSign = s; }
                if (Mathf.Abs(t - Mathf.Round(t)) < 1e-4f && ((int)Mathf.Round(t)) % 2 == 0)
                    Debug.Log("GYRO trace " + label + " t=" + F(t) + " wb=" + V(wb) + " Lworld=" + V(L) + " E=" + F(E));
            }
            public Result Done()
            {
                Debug.Log("GYRO result " + label + " firstFlip_s=" + F(r.firstFlipTime) + " flips=" + r.flips + " min_wb_y=" + F(r.minWy)
                    + " max_perp_w=" + F(r.maxPerp) + " max|dL|/|L0|=" + F(r.maxLDrift) + " max|dE|/E0=" + F(r.maxEDrift));
                return r;
            }
        }

        static Vector3 W0 = new Vector3(0.02f, 2.0f, 0.02f);

        // Reference: Euler's equations for torque-free body, RK4 in body frame + quaternion integration.
        static Result Reference(float dt)
        {
            var tr = new Tracker("reference_RK4_dt" + F(dt));
            double[] w = { W0.x, W0.y, W0.z };
            Quaternion q = Quaternion.identity;
            Func<double[], double[]> f = x => new double[] {
                (I.y - I.z) * x[1] * x[2] / I.x, (I.z - I.x) * x[2] * x[0] / I.y, (I.x - I.y) * x[0] * x[1] / I.z };
            int n = Mathf.RoundToInt(Duration / dt);
            tr.Sample(0, q, q * new Vector3((float)w[0], (float)w[1], (float)w[2]));
            for (int i = 1; i <= n; i++)
            {
                var k1 = f(w); var a = new double[3]; for (int j = 0; j < 3; j++) a[j] = w[j] + 0.5 * dt * k1[j];
                var k2 = f(a); for (int j = 0; j < 3; j++) a[j] = w[j] + 0.5 * dt * k2[j];
                var k3 = f(a); for (int j = 0; j < 3; j++) a[j] = w[j] + dt * k3[j];
                var k4 = f(a);
                var wm = new Vector3((float)w[0], (float)w[1], (float)w[2]);
                for (int j = 0; j < 3; j++) w[j] += dt / 6 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]);
                var wb = 0.5f * (wm + new Vector3((float)w[0], (float)w[1], (float)w[2]));
                q = (q * Quaternion.AngleAxis(wb.magnitude * dt * Mathf.Rad2Deg, wb.normalized)).normalized;
                tr.Sample(i * dt, q, q * new Vector3((float)w[0], (float)w[1], (float)w[2]));
            }
            return tr.Done();
        }

        static Result RunRigidbody(float dt, bool manualGyro, bool tensorViaCollider)
        {
            string label = "Rigidbody_dt" + F(dt) + (manualGyro ? "_manualGyroTorque" : "") + (tensorViaCollider ? "_colliderTensor" : "");
            using (var rig = new Rig())
            {
                var go = rig.Make(label);
                var col = go.AddComponent<BoxCollider>();
                // Box with I = diag(1,2,3) at m = 1? Only used for the collider-derived variant (I proportional to (b^2+c^2)).
                // Solve: Iy+Iz-Ix etc. For a box: Ix=(y^2+z^2)/12 m. Want (1,2,3): x^2=12*(2+3-1)/2=24 -> x=4.899, y^2=12*(1+3-2)/2=12, z^2=12*(1+2-3)/2=0.
                // z=0 is degenerate, so the collider variant uses a thin plate z=0.01 and normalises the tensor below.
                col.size = tensorViaCollider ? new Vector3(Mathf.Sqrt(24), Mathf.Sqrt(12), 0.01f) : Vector3.one * 0.1f;
                var rb = go.AddComponent<Rigidbody>();
                rb.useGravity = false; rb.mass = 1; rb.linearDamping = 0; rb.angularDamping = 0;
                rb.maxAngularVelocity = 1000; rb.sleepThreshold = 0; rb.detectCollisions = false;
                if (!tensorViaCollider) { rb.automaticInertiaTensor = false; rb.inertiaTensor = I; rb.inertiaTensorRotation = Quaternion.identity; }
                rb.automaticCenterOfMass = false; rb.centerOfMass = Vector3.zero;
                Debug.Log("GYRO setup " + label + " inertiaTensor=" + V(rb.inertiaTensor) + " rot=" + rb.inertiaTensorRotation.eulerAngles);
                rb.angularVelocity = W0;
                var tr = new Tracker(label);
                Vector3 Ib = rb.inertiaTensor;
                tr.Sample(0, rb.rotation, rb.angularVelocity);
                int n = Mathf.RoundToInt(Duration / dt);
                for (int i = 1; i <= n; i++)
                {
                    if (manualGyro)
                    {
                        // tau = -w x (I_world w), applied in world frame.
                        Quaternion q = rb.rotation * rb.inertiaTensorRotation;
                        Vector3 w = rb.angularVelocity;
                        Vector3 Lw = q * Mul(Ib, Quaternion.Inverse(q) * w);
                        rb.AddTorque(-Vector3.Cross(w, Lw), ForceMode.Force);
                    }
                    rig.ps.Simulate(dt);
                    tr.Sample(i * dt, rb.rotation, rb.angularVelocity);
                }
                return tr.Done();
            }
        }

        static Result RunArticulation(float dt)
        {
            string label = "ArticulationBodyRoot_dt" + F(dt);
            using (var rig = new Rig())
            {
                var go = rig.Make(label);
                var col = go.AddComponent<BoxCollider>(); col.size = Vector3.one * 0.1f;
                var ab = go.AddComponent<ArticulationBody>();
                ab.useGravity = false; ab.mass = 1; ab.linearDamping = 0; ab.angularDamping = 0;
                ab.maxAngularVelocity = 1000; ab.sleepThreshold = 0; ab.immovable = false;
                ab.automaticInertiaTensor = false; ab.inertiaTensor = I; ab.inertiaTensorRotation = Quaternion.identity;
                ab.automaticCenterOfMass = false; ab.centerOfMass = Vector3.zero;
                Debug.Log("GYRO setup " + label + " isRoot=" + ab.isRoot + " inertiaTensor=" + V(ab.inertiaTensor));
                rig.ps.Simulate(1e-6f);
                ab.angularVelocity = W0;
                var tr = new Tracker(label);
                tr.Sample(0, go.transform.rotation, ab.angularVelocity);
                int n = Mathf.RoundToInt(Duration / dt);
                for (int i = 1; i <= n; i++)
                {
                    rig.ps.Simulate(dt);
                    tr.Sample(i * dt, go.transform.rotation, ab.angularVelocity);
                }
                Debug.Log("GYRO articulation " + label + " final w=" + V(ab.angularVelocity));
                return tr.Done();
            }
        }

        // Control: spin near the major axis (z, I=3). Stable in both dynamics; checks the harness does not invent flips.
        static Result RunRigidbodyStableAxis(float dt)
        {
            string label = "Rigidbody_majorAxisControl_dt" + F(dt);
            using (var rig = new Rig())
            {
                var go = rig.Make(label);
                go.AddComponent<BoxCollider>().size = Vector3.one * 0.1f;
                var rb = go.AddComponent<Rigidbody>();
                rb.useGravity = false; rb.mass = 1; rb.linearDamping = 0; rb.angularDamping = 0; rb.maxAngularVelocity = 1000; rb.sleepThreshold = 0; rb.detectCollisions = false;
                rb.automaticInertiaTensor = false; rb.inertiaTensor = I; rb.inertiaTensorRotation = Quaternion.identity;
                rb.angularVelocity = new Vector3(0.02f, 0.02f, 2f);
                var tr = new Tracker(label); tr.Sample(0, rb.rotation, rb.angularVelocity);
                int n = Mathf.RoundToInt(Duration / dt);
                float minWz = float.MaxValue;
                for (int i = 1; i <= n; i++) { rig.ps.Simulate(dt); tr.Sample(i * dt, rb.rotation, rb.angularVelocity); minWz = Mathf.Min(minWz, (Quaternion.Inverse(rb.rotation) * rb.angularVelocity).z); }
                Debug.Log("GYRO control " + label + " min_wb_z=" + F(minWz));
                return tr.Done();
            }
        }

        [Test]
        public void IntermediateAxis_RigidbodyVsArticulationVsReference()
        {
            Debug.Log("GYRO env unity=" + Application.unityVersion + " defaultSolverIterations=" + Physics.defaultSolverIterations);
            var results = new List<Result>();
            foreach (float dt in new[] { 0.02f, 0.005f })
            {
                results.Add(Reference(dt));
                results.Add(RunRigidbody(dt, false, false));
                results.Add(RunRigidbody(dt, true, false));
                results.Add(RunArticulation(dt));
            }
            results.Add(RunRigidbody(0.005f, false, true));
            results.Add(RunRigidbodyStableAxis(0.005f));
            foreach (var r in results)
                Debug.Log("GYRO summary " + r.label + " flip=" + (r.flips > 0) + " firstFlip_s=" + F(r.firstFlipTime) + " minWy=" + F(r.minWy) + " Ldrift=" + F(r.maxLDrift) + " Edrift=" + F(r.maxEDrift));
            Assert.AreEqual(10, results.Count);
            Assert.Greater(results[0].flips, 0, "reference must flip");
        }
    }
}
