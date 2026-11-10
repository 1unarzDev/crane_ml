using System.Collections.Generic;
using System.IO;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

namespace Crane.Theater.Aerial.Tests {
    public class RenderProfileTests {
        [Test]
        public void OnboardProfileHasNoCinematicOverrides() {
            // a scene profile mixing cinematic effects with lighting components: only the latter may survive
            var scene = ScriptableObject.CreateInstance<VolumeProfile>();
            scene.Add<Bloom>(true); scene.Add<DepthOfField>(true); scene.Add<MotionBlur>(true); scene.Add<Vignette>(true);
            scene.Add<ColorAdjustments>(true); scene.Add<FilmGrain>(true); scene.Add<ChromaticAberration>(true); scene.Add<LiftGammaGain>(true);
            scene.Add<Fog>(true); scene.Add<Exposure>(true); scene.Add<Tonemapping>(true);
            var onboard = RenderProfiles.BuildOnboard(new[] { scene });
            Assert.IsEmpty(RenderProfiles.CinematicOverrides(onboard), "cinematic effect leaked into the onboard profile");
            foreach (var c in onboard.components) Assert.IsFalse(RenderProfiles.IsCinematicEffect(c), c.GetType().Name);
            Assert.IsTrue(onboard.Has<Fog>(), "non-cinematic scene components are carried over");
            Assert.IsTrue(onboard.TryGet(out Exposure e));
            Assert.AreEqual(ExposureMode.Automatic, e.mode.value);
            Assert.AreEqual(0f, e.compensation.value, "no creative exposure compensation on the sensor");
            Assert.IsTrue(onboard.TryGet(out Tonemapping tm));
            Assert.AreEqual(TonemappingMode.Neutral, tm.mode.value);
        }

        [Test]
        public void CinematicProfileHasTheFilmLook() {
            var p = RenderProfiles.BuildCinematic();
            foreach (var t in new[] { typeof(Bloom), typeof(DepthOfField), typeof(MotionBlur), typeof(ColorAdjustments), typeof(Tonemapping), typeof(Exposure), typeof(Vignette), typeof(FilmGrain) })
                Assert.IsTrue(p.Has(t), t.Name);
            p.TryGet(out Exposure e); Assert.AreEqual(ExposureMode.Automatic, e.mode.value);
            Assert.IsNotEmpty(RenderProfiles.CinematicOverrides(p));
        }

        [Test]
        public void ProfilesUseSeparateVolumeLayersAndCameras() {
            var go = new GameObject("cam"); var cam = go.AddComponent<Camera>();
            var go2 = new GameObject("cam2"); var cam2 = go2.AddComponent<Camera>();
            try {
                RenderProfiles.ConfigureOnboardCamera(cam, OakD.RgbHfovDeg, OakD.RgbW, OakD.RgbH);
                RenderProfiles.ConfigureCinematicCamera(cam2, 1920, 1080);
                var on = cam.GetComponent<HDAdditionalCameraData>(); var ci = cam2.GetComponent<HDAdditionalCameraData>();
                Assert.AreNotSame(on, ci);
                Assert.AreEqual(0, on.volumeLayerMask.value & ((1 << RenderProfiles.CinematicLayer) | 1), "onboard camera sees neither the cinematic layer nor the default layer");
                Assert.AreNotEqual(0, on.volumeLayerMask.value & (1 << RenderProfiles.OnboardLayer));
                Assert.AreEqual(0, ci.volumeLayerMask.value & (1 << RenderProfiles.OnboardLayer), "cinematic camera must not see the onboard volume");
                Assert.AreNotEqual(0, ci.volumeLayerMask.value & (1 << RenderProfiles.CinematicLayer));
                foreach (var f in new[] { "DepthOfField", "MotionBlur", "Bloom", "Vignette", "ColorGrading", "ChromaticAberration" })
                    Assert.IsFalse(RenderProfiles.FrameSettingOn(on, f), "onboard frame setting " + f + " must be off");
                Assert.AreEqual(HDAdditionalCameraData.AntialiasingMode.None, on.antialiasing);
                Assert.IsFalse(cam.usePhysicalProperties);
                Assert.AreEqual(OakD.RgbVfov, cam.fieldOfView, 1e-3f);
            } finally { Object.DestroyImmediate(go); Object.DestroyImmediate(go2); }
        }

        [Test]
        public void OakDIntrinsicsFollowTheDatasheet() {
            Assert.AreEqual(1920, OakD.RgbW); Assert.AreEqual(1080, OakD.RgbH);
            Assert.AreEqual(69f, OakD.RgbHfovDeg); Assert.AreEqual(80f, OakD.StereoHfovDeg);
            Assert.AreEqual(1280, OakD.StereoW); Assert.AreEqual(800, OakD.StereoH);
            Assert.AreEqual(55f, OakD.StereoVfov, 0.6f, "stereo VFOV 55 deg on the datasheet");
            Assert.AreEqual(55f, OakD.VerticalFov(69f, 4056, 3040), 0.7f, "full 4:3 sensor VFOV 55 deg on the datasheet");
            Assert.AreEqual(1920 * 0.5f / Mathf.Tan(34.5f * Mathf.Deg2Rad), OakD.FocalPx(69f, 1920), 0.01f);
            Assert.AreEqual(0.075f, OakD.BaselineM);
        }
    }

    public class DirectorTests {
        static string SequencePath => Path.Combine(Application.dataPath, "Drones/Demo/Sequences/forest_demo_v1.json");

        [Test]
        public void SequenceSpecCoversTheRequiredShots() {
            var s = DemoSpec.Parse(File.ReadAllText(SequencePath));
            Assert.That(s.Duration, Is.InRange(45f, 90f));
            var types = new HashSet<string>(); foreach (var h in s.shots) types.Add(h.type);
            foreach (var t in new[] { "establishing", "descent", "chase", "fpv_split", "orbit", "pullout" }) Assert.IsTrue(types.Contains(t), t);
            Assert.AreEqual(30f, s.fps); Assert.AreEqual(1920, s.width);
            var orbit = s.shots.Find(h => h.type == "orbit"); Assert.IsTrue(orbit.detection);
            Assert.AreEqual("split", s.shots.Find(h => h.type == "fpv_split").inset);
            Assert.IsFalse(s.shots[0].avoidTrees, "aerial path shots must not pull the camera in on canopy colliders");
        }

        [Test]
        public void SplineHitsItsControlPoints() {
            var pts = new List<Vector3> { Vector3.zero, new Vector3(5, 2, 1), new Vector3(9, 0, 8), new Vector3(2, 4, 12) };
            var sp = new SplinePath(pts);
            for (int i = 0; i < pts.Count; i++) Assert.Less(Vector3.Distance(sp.At(sp.PointS[i]), pts[i]), 1e-3f);
            float prev = 0; for (int i = 1; i <= 100; i++) { var p = sp.AtNorm(i / 100f); Assert.Less(Vector3.Distance(p, sp.AtNorm((i - 1) / 100f)), 1.5f); prev += 1; }
        }

        [Test]
        public void ScheduleIsMonotoneAndStopsAtTheEnds() {
            var sch = new MonotoneSchedule(new[] { 0.0, 5, 6, 12 }, new[] { 0.0, 10, 10, 40 });
            double last = -1;
            for (double t = 0; t <= 12; t += 0.05) { double v = sch.Eval(t); Assert.GreaterOrEqual(v, last - 1e-9); last = v; }
            Assert.AreEqual(0.0, sch.Eval(0), 1e-9); Assert.AreEqual(40.0, sch.Eval(12), 1e-9);
            Assert.Less(sch.Eval(0.05) - sch.Eval(0), 0.05, "starts from rest");
        }

        static TrajectoryTable StubFlight() {
            var w = new List<TrajectoryBaker.Waypoint> {
                new TrajectoryBaker.Waypoint { pos = new Vector3(0, 30, -40), t = 0, mark = "top" },
                new TrajectoryBaker.Waypoint { pos = new Vector3(0, 28, -10), t = 8 },
                new TrajectoryBaker.Waypoint { pos = new Vector3(0, 6, 0), t = 14, mark = "descent" },
                new TrajectoryBaker.Waypoint { pos = new Vector3(1, 2, 10), t = 20, mark = "corridor" },
                new TrajectoryBaker.Waypoint { pos = new Vector3(0, 2, 36), t = 40 },
                new TrajectoryBaker.Waypoint { pos = new Vector3(0, 1.7f, 37), t = 44, mark = "hover" },
                new TrajectoryBaker.Waypoint { pos = new Vector3(0.1f, 1.8f, 37.1f), t = 90 },
            };
            return TrajectoryBaker.Bake(w, new Vector3(0, 1, 40), 1.0 / 120, "test");
        }

        [Test]
        public void BakedFlightIsPhysicallySane() {
            var tr = StubFlight();
            Assert.AreEqual(90.0, tr.Duration, 0.02);
            Assert.AreEqual(44.0, tr.Marks["hover"], 1e-6);
            for (double t = 0; t <= tr.Duration; t += 0.25) {
                var s = tr.Sample(t);
                Assert.IsFalse(float.IsNaN(s.pos.x + s.pos.y + s.pos.z));
                Assert.Less(s.vel.magnitude, 14f);
                Assert.Less(Vector3.Angle(s.rot * Vector3.up, Vector3.up), 36f, "tilt clamp");
                foreach (float w in s.rotorRadS) Assert.That(w, Is.InRange(150f, 1000f));
            }
        }

        [Test]
        public void DirectorIsAPureFunctionOfTheFrameIndex() {
            var spec = DemoSpec.Parse(File.ReadAllText(SequencePath));
            var fl = StubFlight();
            var mk = new ForestMarkers { isStubScene = true };
            var d1 = new DemoDirector(spec, fl, new Vector3(0, 0, 40), mk);
            var d2 = new DemoDirector(spec, fl, new Vector3(0, 0, 40), mk);
            Assert.AreEqual(spec.FrameCount, d1.FrameCount);
            for (int f = 0; f < d1.FrameCount; f += 37) {
                var a = d1.Evaluate(f); var b = d2.Evaluate(f);
                Assert.AreEqual(a.cam.pos, b.cam.pos); Assert.AreEqual(a.cam.rot, b.cam.rot); Assert.AreEqual(a.fade, b.fade);
            }
            Assert.AreEqual(1f, d1.Evaluate(0).fade, 1e-6f, "opens from black");
            Assert.AreEqual(1f, d1.Evaluate(d1.FrameCount - 1).fade, 0.1f, "closes to black");
            // camera motion is smooth inside shots (no pops between consecutive frames except at cuts)
            for (int s = 0; s < spec.shots.Count; s++) {
                int f0 = d1.FirstFrame(s), n = d1.ShotFrames(s);
                for (int f = f0 + 1; f < f0 + n; f++)
                    Assert.Less(Vector3.Distance(d1.Evaluate(f).cam.pos, d1.Evaluate(f - 1).cam.pos), 2.5f, $"shot {spec.shots[s].id} frame {f}");
            }
        }

        [Test]
        public void RecordedCsvConventionsMapToUnity() {
            string csv = "# frame=enu_flu\nt,px,py,pz,qx,qy,qz,qw,w1,w2,w3,w4\n0,0,0,2,0,0,0,1,500,500,500,500\n1,10,0,2,0,0,0,1,500,500,500,500\n";
            var tr = TrajectoryTable.FromCsv(csv, "t");
            var s = tr.Sample(1.0);
            Assert.Less(Vector3.Distance(s.pos, new Vector3(10, 2, 0)), 1e-3f, "ENU x east -> Unity x, z up -> y");
            Assert.Less(Vector3.Distance(s.rot * Vector3.forward, new Vector3(1, 0, 0)), 1e-3f, "FLU forward at identity points east");
            Assert.Less(Vector3.Distance(s.rot * Vector3.up, Vector3.up), 1e-3f);
            Assert.AreEqual(500f, s.rotorRadS[2], 1e-3f);
        }

        [Test]
        public void PlantBakeUsesTheInterfaceAndStaysCloseToTheReference() {
            var tr = StubFlight();
            var baked = TrajectoryTable.BakeThroughPlant(tr, new KinematicStubPlant(), 1);
            Assert.AreEqual(tr.Duration, baked.Duration, 0.01);
            Assert.Less(Vector3.Distance(baked.Sample(60).pos, tr.Sample(60).pos), 1.0f);
        }

        [Test]
        public void RealAerialPlantTracksTheReferenceAndDrivesRotors() {
            var plant = CranePlantAdapter.TryCreate();
            Assert.IsNotNull(plant, "aerial-v2 plant must load from Tools/Aerial/profiles");
            var tr = StubFlight();
            var baked = TrajectoryTable.BakeThroughPlant(tr, plant, 3);
            Assert.AreEqual(tr.Duration, baked.Duration, 0.01);
            Assert.Less(Vector3.Distance(baked.Sample(80).pos, tr.Sample(80).pos), 2.0f, "hover tracking");
            for (double t = 0; t <= baked.Duration; t += 1.0) {
                var s = baked.Sample(t);
                Assert.IsFalse(float.IsNaN(s.pos.x + s.pos.y + s.pos.z));
                foreach (float w in s.rotorRadS) Assert.That(w, Is.InRange(50f, 2000f));
            }
            var again = TrajectoryTable.BakeThroughPlant(tr, CranePlantAdapter.TryCreate(), 3);
            Assert.AreEqual(baked.Sample(50).pos, again.Sample(50).pos, "deterministic for a seed");
        }
    }
}
