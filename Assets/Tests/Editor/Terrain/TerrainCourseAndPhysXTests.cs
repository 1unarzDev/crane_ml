using System;
using System.IO;
using NUnit.Framework;
using Sim.Physics.LandTerrain;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Tests.Terrain
{
    [TestFixture, Category("TerrainSlope")]
    public class TerrainCourseTests
    {
        static string Fixture(string name) { return Path.Combine(Application.dataPath, "Tests/Editor/Terrain/Fixtures", name); }

        [Test]
        public void Build_Ramp10_VerifiesHashHeightsAndSlope()
        {
            var res = CraneLandTerrainCourse.BuildFromFiles(Fixture("ramp-10deg.json"), Fixture("ramp-10deg.r16"));
            try
            {
                Assert.AreEqual(res.manifest.heightmapSha256, res.computedSha256);
                StringAssert.Contains(res.computedSha256, res.ToRunHeaderJson());
                StringAssert.Contains("\"hashVerified\":true", res.ToRunHeaderJson());
                var td = res.terrain.terrainData;
                int r = res.manifest.heightmapResolution;
                Assert.AreEqual(r, td.heightmapResolution);
                byte[] raw = File.ReadAllBytes(Fixture("ramp-10deg.r16"));
                double lo = res.manifest.heightRangeMeters[0], span = res.manifest.heightRangeMeters[1] - lo;
                double worst = 0;
                for (int z = 0; z < r; z += 7)
                    for (int x = 0; x < r; x += 11)
                    {
                        int i = (z * r + x) * 2;
                        double expect = (raw[i] | (raw[i + 1] << 8)) / 65535.0 * span;
                        worst = Math.Max(worst, Math.Abs(td.GetHeight(x, z) - expect));
                    }
                Debug.Log("MAXERR terrainHeight = " + worst.ToString("E3") + " m");
                Assert.Less(worst, 2e-4);
                // Mid-ramp slope along +z from the built terrain, compared with the manifest's 10 degrees.
                int zi = r * 3 / 4;
                double dh = td.GetHeight(r / 2, zi + 4) - td.GetHeight(r / 2, zi - 4);
                double slopeDeg = Math.Atan2(dh, 8 * res.manifest.sizeMeters / (r - 1)) * 180 / Math.PI;
                Debug.Log("MAXERR terrainSlopeDeg = " + Math.Abs(slopeDeg - 10).ToString("E3"));
                Assert.AreEqual(10.0, slopeDeg, 0.1);
                Assert.IsNotNull(res.collider);
            }
            finally { Cleanup(res); }
        }

        [Test]
        public void Build_FailsClosed_OnTamperedHeightmap_Schema_AndLength()
        {
            string json = File.ReadAllText(Fixture("ramp-10deg.json"));
            byte[] raw = File.ReadAllBytes(Fixture("ramp-10deg.r16"));
            byte[] bad = (byte[])raw.Clone(); bad[1000] ^= 0x01;
            var ex = Assert.Throws<CraneLandTerrainCourse.CourseIntegrityException>(() => CraneLandTerrainCourse.Build(json, bad));
            StringAssert.Contains("SHA-256 mismatch", ex.Message);
            Assert.Throws<CraneLandTerrainCourse.CourseIntegrityException>(() => CraneLandTerrainCourse.Build(json.Replace("crane-land-terrain-course-v1", "other"), raw));
            Assert.Throws<CraneLandTerrainCourse.CourseIntegrityException>(() => CraneLandTerrainCourse.Build(json, new byte[raw.Length - 2]));
        }

        static void Cleanup(CraneLandTerrainCourse.BuildResult r)
        {
            if (r == null) return;
            var td = r.terrain != null ? r.terrain.terrainData : null;
            UnityEngine.Object.DestroyImmediate(r.root);
            if (td != null) UnityEngine.Object.DestroyImmediate(td);
        }
    }

    // Box on an inclined plane in an isolated PhysicsScene. Checks that PhysX Coulomb friction with the
    // same material on both bodies starts sliding at tan(theta) = mu, the physical reference for the
    // flagged traction law's static hold limit. It does not exercise the robot model.
    [TestFixture, Category("TerrainSlope")]
    public class PhysXRampSweepTests
    {
        static PhysicsMaterialCombine combineMode = PhysicsMaterialCombine.Average;
        static double Travel(double mu, double thetaDeg)
        {
            var scene = UnityEditor.SceneManagement.EditorSceneManager.NewPreviewScene();
            var created = new System.Collections.Generic.List<GameObject>();
            try
            {
                var mat = new UnityEngine.PhysicsMaterial("t") { staticFriction = (float)mu, dynamicFriction = (float)mu, frictionCombine = combineMode };
                Quaternion tilt = Quaternion.Euler(0, 0, (float)thetaDeg);
                var ramp = GameObject.CreatePrimitive(PrimitiveType.Cube);
                ramp.transform.localScale = new Vector3(60, 0.2f, 10);
                ramp.transform.rotation = tilt;
                ramp.GetComponent<Collider>().sharedMaterial = mat;
                var box = GameObject.CreatePrimitive(PrimitiveType.Cube);
                box.transform.localScale = new Vector3(1f, 0.2f, 1f);
                box.transform.rotation = tilt;
                box.transform.position = tilt * new Vector3(0, 0.1f + 0.1f + 0.0005f, 0);
                box.GetComponent<Collider>().sharedMaterial = mat;
                var rb = box.AddComponent<Rigidbody>();
                rb.mass = 5f;
                created.Add(ramp); created.Add(box);
                foreach (var go in created) SceneManager.MoveGameObjectToScene(go, scene);
                PhysicsScene ps = scene.GetPhysicsScene();
                Vector3 start = rb.position;
                for (int i = 0; i < 150; i++) ps.Simulate(0.02f);
                return (rb.position - start).magnitude;
            }
            finally
            {
                foreach (var go in created) if (go != null) UnityEngine.Object.DestroyImmediate(go);
                UnityEditor.SceneManagement.EditorSceneManager.ClosePreviewScene(scene);
            }
        }

        // Onset of sliding: smallest scanned angle at which the box travels > 5 cm in 3 s.
        static double SlideOnset(double mu, double startDeg, double endDeg, double stepDeg)
        {
            for (double th = startDeg; th <= endDeg; th += stepDeg)
                if (Travel(mu, th) > 0.05) return th;
            return double.NaN;
        }

        [Test]
        public void BoxOnRamp_ImprovedPatchFriction_SlidesAtAtanMu()
        {
            bool old = UnityEngine.Physics.improvedPatchFriction;
            UnityEngine.Physics.improvedPatchFriction = true;
            try
            {
                foreach (double mu in new[] { 0.3, 0.5, 0.7 })
                {
                    double expect = Math.Atan(mu) * 180 / Math.PI;
                    double onset = SlideOnset(mu, expect - 3.0, expect + 3.0, 0.25);
                    Debug.Log("PHYSX improvedPatchFriction=true mu=" + mu + " analytic=" + expect.ToString("F2") + " onset=" + onset.ToString("F2") + " err=" + (onset - expect).ToString("F2"));
                    Assert.IsFalse(double.IsNaN(onset), "never slid, mu=" + mu);
                    Assert.AreEqual(expect, onset, 1.0, "mu=" + mu);
                    Assert.Less(Travel(mu, expect - 3.0), 0.05, "must hold 3 deg below the limit");
                }
            }
            finally { UnityEngine.Physics.improvedPatchFriction = old; }
        }

        // Characterisation of the project default (improvedPatchFriction off): face-on-face patch friction
        // holds well above atan(mu). The test asserts only that sliding never starts BELOW the analytic limit
        // and logs the measured onset, so the quirk is recorded without freezing it as a requirement.
        [Test]
        public void BoxOnRamp_ProjectDefaultPatchFriction_NeverSlidesBelowAtanMu()
        {
            Assert.IsFalse(UnityEngine.Physics.improvedPatchFriction, "project default changed; revisit this characterisation");
            foreach (double mu in new[] { 0.3, 0.5 })
            {
                double expect = Math.Atan(mu) * 180 / Math.PI;
                double onset = SlideOnset(mu, expect - 3.0, 70.0, 1.0);
                Debug.Log("PHYSX improvedPatchFriction=false mu=" + mu + " analytic=" + expect.ToString("F2") + " onset=" + onset.ToString("F2") + " effectiveMu=" + Math.Tan(onset * Math.PI / 180).ToString("F2"));
                Assert.IsFalse(double.IsNaN(onset));
                Assert.GreaterOrEqual(onset, expect - 1.0);
            }
        }
    }
}
