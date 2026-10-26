using System;
using System.Collections.Generic;
using System.Globalization;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEditor.SceneManagement;

namespace Sim.Tests.Terrain
{
    // Independent verification of I1-council-patch-friction. Method differs from the council: a slowly increasing
    // horizontal force on flat ground (no ramp angle, no bisection), sleeping disabled, explicit combine modes.
    // Results are logged as "VERIFY ..." lines; assertions are only sanity checks.
    [TestFixture, Category("PatchFrictionVerify")]
    public class PatchFrictionVerifyTests
    {
        static string F(double v) { return v.ToString("F4", CultureInfo.InvariantCulture); }
        const float Dt = 0.005f, R = 0.033f, W = 0.018f, Mass = 1.5f;
        bool saved;
        [SetUp] public void Save() { saved = UnityEngine.Physics.improvedPatchFriction; }
        [TearDown] public void Restore() { UnityEngine.Physics.improvedPatchFriction = saved; }

        static PhysicsMaterial Mat(double mu, PhysicsMaterialCombine c)
        {
            return new PhysicsMaterial("verify") { staticFriction = (float)mu, dynamicFriction = (float)mu, frictionCombine = c, bounciness = 0, bounceCombine = PhysicsMaterialCombine.Minimum };
        }

        sealed class Rig : IDisposable
        {
            public Scene scene; public PhysicsScene ps; public readonly List<GameObject> roots = new List<GameObject>();
            public Rig() { scene = EditorSceneManager.NewPreviewScene(); ps = scene.GetPhysicsScene(); }
            public GameObject Add(GameObject go) { SceneManager.MoveGameObjectToScene(go, scene); roots.Add(go); return go; }
            public void Dispose() { foreach (var g in roots) if (g != null) UnityEngine.Object.DestroyImmediate(g); EditorSceneManager.ClosePreviewScene(scene); }
        }

        static Mesh CampusTire(float radius, float width)
        {
            const int sides = 64; var vertices = new Vector3[sides * 3]; var triangles = new List<int>();
            for (int ring = 0; ring < 3; ring++) for (int i = 0; i < sides; i++)
                {
                    float angle = i * Mathf.PI * 2 / sides; float r = ring == 1 ? radius : radius - .002f;
                    vertices[ring * sides + i] = new Vector3((ring - 1) * width / 2, Mathf.Cos(angle) * r, Mathf.Sin(angle) * r);
                }
            for (int ring = 0; ring < 2; ring++) for (int i = 0; i < sides; i++) { int next = (i + 1) % sides; int a = ring * sides; int b = (ring + 1) * sides; triangles.AddRange(new[] { a + i, a + next, b + i, a + next, b + next, b + i }); }
            for (int i = 1; i < sides - 1; i++) triangles.AddRange(new[] { 0, i + 1, i, 2 * sides, 2 * sides + i, 2 * sides + i + 1 });
            var mesh = new Mesh(); mesh.vertices = vertices; mesh.triangles = triangles.ToArray(); mesh.RecalculateNormals(); mesh.RecalculateBounds(); return mesh;
        }

        public enum S { Box05, Box001, Sphere, Capsule, MeshTire }

        // Adds the collider; returns rest height of the body origin above ground.
        static float AddShape(GameObject go, S s, PhysicsMaterial mat, float sx = 0, float sz = 0)
        {
            Collider c; float h;
            switch (s)
            {
                case S.Box05: { var b = go.AddComponent<BoxCollider>(); b.size = new Vector3(sx > 0 ? sx : 0.5f, 0.2f, sz > 0 ? sz : 0.5f); c = b; h = 0.1f; break; }
                case S.Box001: { var b = go.AddComponent<BoxCollider>(); b.size = new Vector3(sx > 0 ? sx : 0.01f, 0.02f, sz > 0 ? sz : 0.01f); c = b; h = 0.01f; break; }
                case S.Sphere: { var sp = go.AddComponent<SphereCollider>(); sp.radius = R; c = sp; h = R; break; }
                case S.Capsule: { var cap = go.AddComponent<CapsuleCollider>(); cap.radius = R; cap.height = 2 * R + W; cap.direction = 0; c = cap; h = R; break; }
                default: { var mc = go.AddComponent<MeshCollider>(); mc.sharedMesh = CampusTire(R, W); mc.convex = true; c = mc; h = R; break; }
            }
            c.sharedMaterial = mat; return h;
        }

        static GameObject Ground(PhysicsMaterial mat)
        {
            var g = GameObject.CreatePrimitive(PrimitiveType.Cube);
            g.transform.localScale = new Vector3(40, 0.2f, 40); g.transform.position = new Vector3(0, -0.1f, 0);
            g.GetComponent<Collider>().sharedMaterial = mat; return g;
        }

        // Slowly increasing +x force (rate = fraction of expected breakaway per second). Rotation frozen, sleep disabled.
        // flipAtFraction >= 0: toggles the global flag to flipTo once the applied force passes that fraction of m*g.
        static double Breakaway(S s, double mu, PhysicsMaterialCombine comb, bool improved, float sx = 0, float sz = 0, double flipAtMg = -1, bool flipTo = false)
        {
            UnityEngine.Physics.improvedPatchFriction = improved;
            double expect = comb == PhysicsMaterialCombine.Multiply ? mu * mu : mu;
            using (var rig = new Rig())
            {
                var mat = Mat(mu, comb);
                rig.Add(Ground(mat));
                var go = new GameObject("body");
                float h = AddShape(go, s, mat, sx, sz);
                go.transform.position = new Vector3(0, h + 0.0002f, 0);
                var rb = go.AddComponent<Rigidbody>(); rb.mass = Mass; rb.constraints = RigidbodyConstraints.FreezeRotation; rb.sleepThreshold = 0; rb.linearDamping = 0;
                rig.Add(go);
                for (int i = 0; i < 100; i++) rig.ps.Simulate(Dt);
                Vector3 p0 = rb.position; double mg = Mass * 9.81;
                double rate = 0.08 * expect * mg; // N/s
                bool flipped = false;
                for (int i = 1; i <= 9000; i++)
                {
                    double f = rate * i * Dt;
                    if (!flipped && flipAtMg >= 0 && f >= flipAtMg * mg) { UnityEngine.Physics.improvedPatchFriction = flipTo; flipped = true; }
                    rb.AddForce(new Vector3((float)f, 0, 0), ForceMode.Force);
                    rig.ps.Simulate(Dt);
                    if (rb.position.x - p0.x > 0.0015) return f / mg;
                }
                return double.PositiveInfinity;
            }
        }

        static double Expect(double mu, PhysicsMaterialCombine c) { return c == PhysicsMaterialCombine.Multiply ? mu * mu : mu; }

        [Test]
        public void ForceRamp_FlatGround_AllShapes_BothFlags_BothCombines()
        {
            foreach (var comb in new[] { PhysicsMaterialCombine.Average, PhysicsMaterialCombine.Multiply })
                foreach (bool improved in new[] { false, true })
                    foreach (double mu in new[] { 0.3, 0.6 })
                        foreach (S s in Enum.GetValues(typeof(S)))
                        {
                            double r = Breakaway(s, mu, comb, improved);
                            double e = Expect(mu, comb);
                            Debug.Log("VERIFY ramp combine=" + comb + " improved=" + improved + " mu=" + mu + " shape=" + s + " F/mg=" + F(r) + " expected=" + F(e) + " ratio=" + F(r / e));
                        }
        }

        [Test]
        public void ForceRamp_FootprintThreshold()
        {
            foreach (bool improved in new[] { false, true })
            {
                foreach (float side in new[] { 0.004f, 0.010f, 0.0155f, 0.016f, 0.0165f, 0.017f, 0.0175f, 0.018f, 0.020f, 0.022f, 0.025f, 0.030f, 0.040f, 0.060f })
                {
                    double r = Breakaway(S.Box001, 0.4, PhysicsMaterialCombine.Average, improved, side, side);
                    Debug.Log("VERIFY footprint improved=" + improved + " square=" + F(side) + " diag=" + F(side * Math.Sqrt(2)) + " ratio=" + F(r / 0.4));
                }
                // Long thin strips, pushed across and along the strip.
                foreach (float thin in new[] { 0.004f, 0.01f, 0.018f, 0.025f })
                {
                    double across = Breakaway(S.Box001, 0.4, PhysicsMaterialCombine.Average, improved, thin, 0.5f);
                    double along = Breakaway(S.Box001, 0.4, PhysicsMaterialCombine.Average, improved, 0.5f, thin);
                    Debug.Log("VERIFY strip improved=" + improved + " thin=" + F(thin) + " long=0.5 push_across_thin_dim_ratio=" + F(across / 0.4) + " push_along_long_dim_ratio=" + F(along / 0.4));
                }
            }
        }

        [Test]
        public void RuntimeToggle_MidSimulation()
        {
            // mu .3: flag off flat box breaks at ~.6 mg, on at ~.3 mg.
            double offThenOn = Breakaway(S.Box05, 0.3, PhysicsMaterialCombine.Average, false, 0, 0, 0.45, true);
            double onThenOff = Breakaway(S.Box05, 0.3, PhysicsMaterialCombine.Average, true, 0, 0, 0.2, false);
            double offFresh = Breakaway(S.Box05, 0.3, PhysicsMaterialCombine.Average, false);
            double onFresh = Breakaway(S.Box05, 0.3, PhysicsMaterialCombine.Average, true);
            Debug.Log("VERIFY toggle offFresh=" + F(offFresh) + " onFresh=" + F(onFresh) + " off_then_on_at_0.45mg=" + F(offThenOn) + " on_then_off_at_0.2mg=" + F(onThenOff));
        }

        // ---------- rolling traction under drive torque (articulation, frictionless chassis, 4 driven wheels) ----------
        static bool Debug_ = false;
        [Test, Ignore("debug trace")] public void DriveTrace() { Debug_ = true; double a, b, c, d; DriveRun(S.Sphere, 0.6, true, out a, out b, out c, out d); Debug_ = false; }
        static void DriveRun(S s, double mu, bool improved, out double peakTractionG, out double slipOnsetTractionG, out double slidingTractionG, out double slipOnsetT)
        {
            UnityEngine.Physics.improvedPatchFriction = improved;
            var comb = PhysicsMaterialCombine.Average;
            using (var rig = new Rig())
            {
                var mat = Mat(mu, comb);
                var frictionless = new PhysicsMaterial("fl") { staticFriction = 0, dynamicFriction = 0, frictionCombine = PhysicsMaterialCombine.Minimum };
                rig.Add(Ground(mat));
                var root = new GameObject("chassis");
                root.transform.position = new Vector3(0, R, 0);
                var cb = root.AddComponent<BoxCollider>(); cb.size = new Vector3(0.2f, 0.02f, 0.25f); cb.sharedMaterial = frictionless;
                var ab = root.AddComponent<ArticulationBody>(); ab.mass = 1.4f; ab.linearDamping = 0; ab.angularDamping = 0; ab.sleepThreshold = 0;
                var wheels = new List<ArticulationBody>();
                foreach (float x in new[] { -0.13f, 0.13f }) foreach (float z in new[] { -0.1f, 0.1f })
                    {
                        var w = new GameObject("wheel"); w.transform.SetParent(root.transform, false);
                        // Wheel shape: for the box case use a 0.018 x 2R x 2R box (flat face) via S.Box001 sizes.
                        float h;
                        if (s == S.Box001) { var b = w.AddComponent<BoxCollider>(); b.size = new Vector3(W, 2 * R, 2 * R); b.sharedMaterial = mat; h = R; }
                        else h = AddShape(w, s, mat);
                        w.transform.position = new Vector3(x, h + 0.0002f, z);
                        var wa = w.AddComponent<ArticulationBody>();
                        wa.jointType = ArticulationJointType.RevoluteJoint; wa.twistLock = ArticulationDofLock.LimitedMotion; wa.twistLock = ArticulationDofLock.FreeMotion;
                        wa.mass = 0.0285f; wa.linearDamping = 0; wa.angularDamping = 0; wa.jointFriction = 0; wa.maxAngularVelocity = 1000; wa.maxJointVelocity = 10000; wa.sleepThreshold = 0;
                        wa.anchorRotation = Quaternion.identity; wa.anchorPosition = Vector3.zero;
                        wheels.Add(wa);
                    }
                rig.Add(root);
                double m = 1.4 + 4 * 0.0285, g = 9.81;
                for (int i = 0; i < 200; i++) rig.ps.Simulate(Dt);
                double expectF = mu * m * g;
                double Tfull = expectF * R / 4.0;            // torque per wheel that just saturates mu*N at 1x
                double Tmax = 2.0 * Tfull, dur = 12.0; int n = (int)(dur / Dt);
                peakTractionG = 0; slipOnsetTractionG = double.NaN; slipOnsetT = double.NaN; slidingTractionG = 0;
                double prevV = ab.linearVelocity.z; bool slipped = false; double slidSum = 0; int slidN = 0;
                var accHist = new Queue<double>();
                for (int i = 1; i <= n; i++)
                {
                    double T = Tmax * i / n;
                    foreach (var wa in wheels) wa.jointForce = new ArticulationReducedSpace((float)T);
                    rig.ps.Simulate(Dt);
                    double v = ab.linearVelocity.z; double a = (v - prevV) / Dt; prevV = v;
                    double wR = 0; foreach (var wa in wheels) wR += wa.jointVelocity[0] * R / 4.0;
                    double slip = (wR - v) / Math.Max(Math.Abs(wR), 1e-3);
                    double aG = a / g;
                    if (Debug_ && i % 400 == 0) Debug.Log("VERIFY trace i=" + i + " T=" + F(T) + " v=" + F(v) + " wR=" + F(wR) + " y=" + F(ab.transform.position.y) + " pitch=" + F(ab.transform.eulerAngles.x) + " aG=" + F(aG));
                    if (!slipped) { if (aG > peakTractionG) peakTractionG = aG; if (wR > 0.05 && slip > 0.15 && wR - v > 0.05) { slipped = true; slipOnsetTractionG = aG; slipOnsetT = T; } }
                    else if (i > n - 400) { slidSum += aG; slidN++; }
                }
                slidingTractionG = slidN > 0 ? slidSum / slidN : double.NaN;
            }
        }

        [Test, Ignore("Rig defect: torque->force calibration at mu=3 fails by ~35x and the vehicle drives off the ground; see I1-verify-patch-friction")]
        public void DriveTorque_RollingTraction_Articulation()
        {
            foreach (bool improved in new[] { false, true })
                foreach (double mu in new[] { 0.3, 0.6, 3.0 })
                    foreach (S s in new[] { S.Sphere, S.Capsule, S.MeshTire, S.Box001 })
                    {
                        double peak, onset, sliding, T;
                        DriveRun(s, mu, improved, out peak, out onset, out sliding, out T);
                        Debug.Log("VERIFY drive improved=" + improved + " mu=" + mu + " wheel=" + (s == S.Box001 ? "BoxWheel" : s.ToString()) + " peakA/g_preSlip=" + F(peak) + " ratio=" + F(peak / mu) + " slipOnsetA/g=" + F(onset) + " slidingA/g=" + F(sliding) + " sliding_ratio=" + F(sliding / mu) + " slipOnsetT=" + F(T));
                    }
        }
    }
}
