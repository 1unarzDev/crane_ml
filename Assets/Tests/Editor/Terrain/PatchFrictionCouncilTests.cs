using System;
using System.Collections.Generic;
using System.Globalization;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEditor.SceneManagement;

namespace Sim.Tests.Terrain
{
    // Council I1-patch-friction characterisation. It measures the effective Coulomb coefficient
    // mu_eff = F_onset / N for each contact geometry under both UnityEngine.Physics.improvedPatchFriction settings,
    // using isolated preview PhysicsScenes. Everything is logged as "COUNCIL ..." lines. Assertions only
    // check the analytic reference with improvedPatchFriction on, plus sanity checks. The project-default
    // behaviour is recorded, not frozen.
    [TestFixture, Category("PatchFrictionCouncil")]
    public class PatchFrictionCouncilTests
    {
        static readonly double[] Mus = { 0.2, 0.4, 0.6, 0.8 };
        static string F(double v) { return v.ToString("F3", CultureInfo.InvariantCulture); }

        bool saved;
        [SetUp] public void Save() { saved = UnityEngine.Physics.improvedPatchFriction; }
        [TearDown] public void Restore() { UnityEngine.Physics.improvedPatchFriction = saved; }

        static PhysicsMaterial Mat(double mu, PhysicsMaterialCombine c = PhysicsMaterialCombine.Average)
        {
            return new PhysicsMaterial("council") { staticFriction = (float)mu, dynamicFriction = (float)mu, frictionCombine = c, bounciness = 0, bounceCombine = PhysicsMaterialCombine.Minimum };
        }
        static PhysicsMaterial Frictionless() { return Mat(0, PhysicsMaterialCombine.Minimum); }

        sealed class Rig : IDisposable
        {
            public Scene scene; public PhysicsScene ps; public readonly List<GameObject> roots = new List<GameObject>();
            public Rig() { scene = EditorSceneManager.NewPreviewScene(); ps = scene.GetPhysicsScene(); }
            public GameObject Add(GameObject go) { SceneManager.MoveGameObjectToScene(go, scene); roots.Add(go); return go; }
            public void Dispose()
            {
                foreach (var g in roots) if (g != null) UnityEngine.Object.DestroyImmediate(g);
                EditorSceneManager.ClosePreviewScene(scene);
            }
        }

        // Box on ramp (face-on-face patch).
        static double BoxTravel(double mu, double thetaDeg, Rig existing = null)
        {
            var rig = existing ?? new Rig();
            try
            {
                var mat = Mat(mu);
                Quaternion tilt = Quaternion.Euler(0, 0, (float)thetaDeg);
                var ramp = GameObject.CreatePrimitive(PrimitiveType.Cube);
                ramp.transform.localScale = new Vector3(60, 0.2f, 10); ramp.transform.rotation = tilt;
                ramp.GetComponent<Collider>().sharedMaterial = mat;
                var box = GameObject.CreatePrimitive(PrimitiveType.Cube);
                box.transform.localScale = new Vector3(1f, 0.2f, 1f); box.transform.rotation = tilt;
                box.transform.position = tilt * new Vector3(0, 0.2005f, 0);
                box.GetComponent<Collider>().sharedMaterial = mat;
                var rb = box.AddComponent<Rigidbody>(); rb.mass = 5f;
                rig.Add(ramp); rig.Add(box);
                Vector3 start = rb.position;
                for (int i = 0; i < 150; i++) rig.ps.Simulate(0.02f);
                return (rb.position - start).magnitude;
            }
            finally { if (existing == null) rig.Dispose(); }
        }

        static double Bisect(Func<double, bool> slides, double lo, double hi, int iters)
        {
            if (slides(lo)) return double.NegativeInfinity;
            if (!slides(hi)) return double.PositiveInfinity;
            for (int i = 0; i < iters; i++) { double m = 0.5 * (lo + hi); if (slides(m)) hi = m; else lo = m; }
            return 0.5 * (lo + hi);
        }

        [Test]
        public void BoxOnRamp_BothSettings_EffectiveMu()
        {
            var fails = new List<string>();
            foreach (bool improved in new[] { false, true })
            {
                UnityEngine.Physics.improvedPatchFriction = improved;
                foreach (double mu in Mus)
                {
                    double expect = Math.Atan(mu) * 180 / Math.PI;
                    double onset = Bisect(th => BoxTravel(mu, th) > 0.05, Math.Max(0.5, expect - 4), 76, 11);
                    double eff = Math.Tan(onset * Math.PI / 180);
                    Debug.Log("COUNCIL box-ramp improved=" + improved + " mu=" + F(mu) + " atanMuDeg=" + F(expect) + " onsetDeg=" + F(onset) + " muEff=" + F(eff) + " ratio=" + F(eff / mu));
                    if (double.IsInfinity(onset)) fails.Add("onset not bracketed improved=" + improved + " mu=" + mu);
                    else if (improved && Math.Abs(expect - onset) > 0.5) fails.Add("improved mu=" + mu + " onset=" + onset);
                    else if (!improved && onset < expect - 0.5) fails.Add("default slid below atan mu=" + mu);
                }
            }
            Assert.IsEmpty(fails, string.Join("; ", fails));
        }

        // Is the flag read live by every scene, or captured when the scene is created?
        [Test]
        public void RuntimeToggle_AppliesToAlreadyCreatedScene()
        {
            const double mu = 0.4, theta = 30; // atan(0.4)=21.8 deg; the default holds at 30 deg.
            UnityEngine.Physics.improvedPatchFriction = false;
            double offHeld = BoxTravel(mu, theta);
            UnityEngine.Physics.improvedPatchFriction = true;
            double onSlid = BoxTravel(mu, theta);
            // The scene was created while the flag was off, then the flag was flipped before the first step.
            UnityEngine.Physics.improvedPatchFriction = false;
            double flippedBeforeStep;
            using (var rig = new Rig()) { UnityEngine.Physics.improvedPatchFriction = true; flippedBeforeStep = BoxTravel(mu, theta, rig); }
            // The scene was created while the flag was on, then the flag was turned off before the first step.
            UnityEngine.Physics.improvedPatchFriction = true;
            double flippedOff;
            using (var rig = new Rig()) { UnityEngine.Physics.improvedPatchFriction = false; flippedOff = BoxTravel(mu, theta, rig); }
            Debug.Log("COUNCIL toggle offFresh=" + F(offHeld) + " onFresh=" + F(onSlid) + " createdOff_thenOn=" + F(flippedBeforeStep) + " createdOn_thenOff=" + F(flippedOff));
            Assert.Less(offHeld, 0.05); Assert.Greater(onSlid, 0.05);
        }

        // ---------- Wheels: lateral push on flat ground ----------
        public enum Shape { Box, Sphere, CapsuleAxle, PrismFacetDown, PrismEdgeDown, CampusRoundedTire }

        const float R = 0.033f, W = 0.018f, Dt = 0.01f;

        static Mesh Prism(float radius, float width, int sides, float phase)
        {
            var v = new Vector3[sides * 2]; var t = new List<int>();
            for (int ring = 0; ring < 2; ring++) for (int i = 0; i < sides; i++)
                {
                    float a = phase + i * Mathf.PI * 2 / sides;
                    v[ring * sides + i] = new Vector3((ring == 0 ? -1 : 1) * width / 2, Mathf.Cos(a) * radius, Mathf.Sin(a) * radius);
                }
            for (int i = 0; i < sides; i++) { int n = (i + 1) % sides; t.AddRange(new[] { i, n, sides + i, n, sides + n, sides + i }); }
            for (int i = 1; i < sides - 1; i++) t.AddRange(new[] { 0, i + 1, i, sides, sides + i, sides + i + 1 });
            var m = new Mesh { vertices = v, triangles = t.ToArray() }; m.RecalculateNormals(); m.RecalculateBounds(); return m;
        }

        // Verbatim geometry of CraneCampusRobotModel.WheelCylinder (HEAD) for --crane-campus-convex-tires.
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

        // Adds the wheel collider to go (axle along local x). Returns the height of the wheel centre above ground at rest.
        static float AddWheelCollider(GameObject go, Shape s, PhysicsMaterial mat)
        {
            Collider c; float h = R;
            const int sides = 64;
            switch (s)
            {
                case Shape.Box: { var b = go.AddComponent<BoxCollider>(); b.size = new Vector3(W, 2 * R, 2 * R); c = b; break; }
                case Shape.Sphere: { var sp = go.AddComponent<SphereCollider>(); sp.radius = R; c = sp; break; }
                case Shape.CapsuleAxle: { var cap = go.AddComponent<CapsuleCollider>(); cap.radius = R; cap.height = 2 * R + W; cap.direction = 0; c = cap; break; }
                case Shape.PrismFacetDown:
                    {
                        // Vertex angles measured from +y; offsetting by half a step puts a flat facet at the bottom.
                        var mc = go.AddComponent<MeshCollider>(); mc.sharedMesh = Prism(R, W, sides, Mathf.PI / sides); mc.convex = true; c = mc;
                        h = R * Mathf.Cos(Mathf.PI / sides); break;
                    }
                case Shape.PrismEdgeDown:
                    {
                        var mc = go.AddComponent<MeshCollider>(); mc.sharedMesh = Prism(R, W, sides, 0); mc.convex = true; c = mc; break;
                    }
                default:
                    {
                        var mc = go.AddComponent<MeshCollider>(); mc.sharedMesh = CampusTire(R, W); mc.convex = true; c = mc; break;
                    }
            }
            c.sharedMaterial = mat; c.contactOffset = 0.001f;
            return h;
        }

        static GameObject Ground(PhysicsMaterial mat)
        {
            var g = GameObject.CreatePrimitive(PrimitiveType.Cube);
            g.transform.localScale = new Vector3(40, 0.2f, 40); g.transform.position = new Vector3(0, -0.1f, 0);
            g.GetComponent<Collider>().sharedMaterial = mat; return g;
        }

        // (a) A single wheel body with all rotation frozen (a locked wheel), pushed along its axle.
        static bool SingleWheelSlides(Shape s, double mu, double forceOverWeight, out double lateralDrift)
        {
            using (var rig = new Rig())
            {
                var mat = Mat(mu);
                rig.Add(Ground(mat));
                var w = new GameObject("wheel");
                float h = AddWheelCollider(w, s, mat);
                w.transform.position = new Vector3(0, h + 0.0002f, 0);
                var rb = w.AddComponent<Rigidbody>(); rb.mass = 1.5f; rb.constraints = RigidbodyConstraints.FreezeRotation;
                rb.solverIterations = 6;
                rig.Add(w);
                for (int i = 0; i < 50; i++) rig.ps.Simulate(Dt);
                Vector3 p0 = rb.position;
                float f = (float)(forceOverWeight * rb.mass * 9.81f);
                for (int i = 0; i < 200; i++) { rb.AddForce(new Vector3(f, 0, 0), ForceMode.Force); rig.ps.Simulate(Dt); }
                lateralDrift = rb.position.x - p0.x;
                return lateralDrift > 0.005;
            }
        }

        // (b) A campus-like 4-wheel rig: a frictionless chassis with four wheel Rigidbodies on HingeJoints about x.
        // A lateral (axle-direction) force on the chassis; the wheels are free to spin about their axles.
        static bool HingeRigSlides(Shape s, double mu, double forceOverWeight)
        {
            using (var rig = new Rig())
            {
                var mat = Mat(mu);
                rig.Add(Ground(mat));
                var chassis = new GameObject("chassis");
                var cb = chassis.AddComponent<BoxCollider>(); cb.size = new Vector3(0.2f, 0.02f, 0.25f); cb.sharedMaterial = Frictionless();
                chassis.transform.position = new Vector3(0, R, 0);
                var body = chassis.AddComponent<Rigidbody>(); body.mass = 1.4f; body.solverIterations = 12;
                rig.Add(chassis);
                var wheels = new List<Rigidbody>(); var w0 = new List<Vector3>();
                foreach (float x in new[] { -0.13f, 0.13f }) foreach (float z in new[] { -0.1f, 0.1f })
                    {
                        var w = new GameObject("wheel");
                        float h = AddWheelCollider(w, s, mat);
                        w.transform.position = new Vector3(x, h + 0.0002f, z);
                        var wr = w.AddComponent<Rigidbody>(); wr.mass = 0.0285f; wr.maxAngularVelocity = 1000; wr.solverIterations = 12;
                        var j = w.AddComponent<HingeJoint>(); j.connectedBody = body; j.axis = Vector3.right; j.anchor = Vector3.zero;
                        rig.Add(w); wheels.Add(wr);
                    }
                double m = body.mass + 4 * 0.0285;
                for (int i = 0; i < 80; i++) rig.ps.Simulate(Dt);
                foreach (var wr in wheels) w0.Add(wr.position);
                float f = (float)(forceOverWeight * m * 9.81);
                for (int i = 0; i < 200; i++) { body.AddForce(new Vector3(f, 0, 0), ForceMode.Force); rig.ps.Simulate(Dt); }
                double mean = 0; for (int i = 0; i < wheels.Count; i++) mean += (wheels[i].position.x - w0[i].x) / wheels.Count;
                return mean > 0.005;
            }
        }

        // (c) An ArticulationBody rig: a frictionless root chassis with four revolute (about x) wheel links.
        static bool ArticulationRigSlides(Shape s, double mu, double forceOverWeight)
        {
            using (var rig = new Rig())
            {
                var mat = Mat(mu);
                rig.Add(Ground(mat));
                var root = new GameObject("artic-chassis");
                root.transform.position = new Vector3(0, R, 0);
                var cb = root.AddComponent<BoxCollider>(); cb.size = new Vector3(0.2f, 0.02f, 0.25f); cb.sharedMaterial = Frictionless();
                var ab = root.AddComponent<ArticulationBody>(); ab.mass = 1.4f; ab.linearDamping = 0; ab.angularDamping = 0;
                foreach (float x in new[] { -0.13f, 0.13f }) foreach (float z in new[] { -0.1f, 0.1f })
                    {
                        var w = new GameObject("wheel");
                        w.transform.SetParent(root.transform, false);
                        float h = AddWheelCollider(w, s, mat);
                        w.transform.position = new Vector3(x, h + 0.0002f, z);
                        var wa = w.AddComponent<ArticulationBody>();
                        wa.jointType = ArticulationJointType.RevoluteJoint; wa.twistLock = ArticulationDofLock.FreeMotion;
                        wa.mass = 0.0285f; wa.linearDamping = 0; wa.angularDamping = 0; wa.jointFriction = 0;
                        wa.anchorRotation = Quaternion.identity; wa.anchorPosition = Vector3.zero;
                    }
                rig.Add(root);
                double m = 1.4 + 4 * 0.0285;
                for (int i = 0; i < 80; i++) rig.ps.Simulate(Dt);
                Vector3 p0 = ab.transform.position;
                float f = (float)(forceOverWeight * m * 9.81);
                for (int i = 0; i < 200; i++) { ab.AddForce(new Vector3(f, 0, 0), ForceMode.Force); rig.ps.Simulate(Dt); }
                return ab.transform.position.x - p0.x > 0.005;
            }
        }

        static void Sweep(string rigName, Func<Shape, double, double, bool> slides, Shape[] shapes, bool assertImproved = true)
        {
            var fails = new List<string>();
            foreach (bool improved in new[] { false, true })
            {
                UnityEngine.Physics.improvedPatchFriction = improved;
                foreach (var s in shapes) foreach (double mu in Mus)
                    {
                        double eff = Bisect(k => slides(s, mu, k), 0.02, 2.4, 10);
                        Debug.Log("COUNCIL " + rigName + " shape=" + s + " improved=" + improved + " mu=" + F(mu) + " muEff=" + F(eff) + " ratio=" + F(eff / mu));
                        if (double.IsNegativeInfinity(eff)) fails.Add(rigName + " " + s + " slid at the minimum force improved=" + improved + " mu=" + mu);
                        else if (assertImproved && improved && Math.Abs(mu - eff) > 0.15 * mu + 0.02) fails.Add(rigName + " " + s + " improved mu=" + mu + " eff=" + F(eff));
                    }
            }
            Assert.IsEmpty(fails, string.Join("; ", fails));
        }

        static readonly Shape[] AllShapes = { Shape.Box, Shape.Sphere, Shape.CapsuleAxle, Shape.PrismFacetDown, Shape.PrismEdgeDown, Shape.CampusRoundedTire };

        [Test] public void LockedWheel_LateralPush_EffectiveMu() { double d; Sweep("locked-wheel", (s, mu, k) => SingleWheelSlides(s, mu, k, out d), AllShapes); }

        // Rig defect: run 1 (high chassis) gave a mu-independent onset (1.367 = the tip ratio; capsule 0.147; convex slid at 2 %).
        // Run 2 (axle-height chassis, wheel displacement measured) never slid up to 2.4 g. Neither run is friction evidence.
        // Kept only for a later fix. Contact-level answers come from the locked-wheel and articulation rigs.
        [Test, Ignore("HingeJoint rig not validated; see council handoff I1-council-patch-friction")] public void HingeWheelRig_LateralPush_EffectiveMu() { Sweep("hinge-rig", HingeRigSlides, new[] { Shape.Sphere, Shape.CapsuleAxle, Shape.PrismFacetDown, Shape.CampusRoundedTire }, false); }

        [Test] public void ArticulationWheelRig_LateralPush_EffectiveMu() { Sweep("artic-rig", ArticulationRigSlides, new[] { Shape.Sphere, Shape.CapsuleAxle, Shape.PrismFacetDown, Shape.CampusRoundedTire }, false); }

        // Mechanism probe: PhysX patch friction adds a second friction anchor only when the patch spans more than
        // frictionOffsetThreshold (0.04 m at the default tolerance scale). A locked box with a 0.066 m long footprint
        // and variable width is pushed along its width with the project default (off) and on.
        [Test]
        public void LockedBox_FootprintSweep_LocatesInflationThreshold()
        {
            foreach (bool improved in new[] { false, true })
            {
                UnityEngine.Physics.improvedPatchFriction = improved;
                foreach (float side in new[] { 0.010f, 0.018f, 0.025f, 0.030f, 0.035f, 0.040f, 0.045f, 0.050f, 0.080f, 0.200f })
                {
                    const double mu = 0.4;
                    Func<double, bool> slides = k =>
                    {
                        using (var rig = new Rig())
                        {
                            var mat = Mat(mu); rig.Add(Ground(mat));
                            var go = new GameObject("box"); var b = go.AddComponent<BoxCollider>(); b.size = new Vector3(side, 0.02f, side); b.sharedMaterial = mat; b.contactOffset = 0.001f;
                            go.transform.position = new Vector3(0, 0.0102f, 0);
                            var rb = go.AddComponent<Rigidbody>(); rb.mass = 1.5f; rb.constraints = RigidbodyConstraints.FreezeRotation; rig.Add(go);
                            for (int i = 0; i < 50; i++) rig.ps.Simulate(Dt);
                            Vector3 p0 = rb.position; float f = (float)(k * rb.mass * 9.81f);
                            for (int i = 0; i < 200; i++) { rb.AddForce(new Vector3(f, 0, 0)); rig.ps.Simulate(Dt); }
                            return rb.position.x - p0.x > 0.005;
                        }
                    };
                    double eff = Bisect(slides, 0.02, 2.4, 10);
                    Debug.Log("COUNCIL footprint improved=" + improved + " squareSide=" + F(side) + " diag=" + F(side * Math.Sqrt(2)) + " mu=" + F(mu) + " muEff=" + F(eff) + " ratio=" + F(eff / mu));
                }
            }
        }
    }
}
