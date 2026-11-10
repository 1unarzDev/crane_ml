using System.Collections.Generic;
using System.IO;
using Crane.Theater;
using NUnit.Framework;
using UnityEngine;

namespace Crane.Theater.Tests {
    /// <summary>The single-episode demo options (hud, hero appearance, fixed cameras): their defaults are the fleet
    /// look every earlier render had, they never change a frame's state digest, and the fixed shots hold their camera.</summary>
    public class TheaterDemoOptionsTests {
        static string Dir => Path.Combine(Application.dataPath, "Tests", "Theater", "Fixtures");
        static TheaterManifest Golden() => TheaterManifest.Load(Path.Combine(Dir, "golden_manifest.json"));

        static TheaterState State(TheaterManifest m, TheaterScript s) {
            var world = new PlaneWorld();
            var e = world.Prepare(m);
            while (e.MoveNext()) { }
            return new TheaterState(m, s, world);
        }

        static TheaterScript Parse(string json) => TheaterScript.Parse(TJson.Parse(json));

        const string Compare = @"{""type"": ""compare"", ""duration_s"": 2, ""speed"": 8, ""trails"": ""top"", ""camera"": {""shot"": ""follow""}";

        [Test]
        public void DefaultsAreTheFleetLook() {
            var s = Parse(@"{""fps"": 30, ""acts"": [" + Compare + "}]}");
            Assert.AreEqual("fleet", s.Hud);
            Assert.IsFalse(s.HudSet);
            Assert.AreEqual("", s.Backend);
            var a = s.Acts[0];
            Assert.AreEqual(0.55f, a.heroTint);
            Assert.IsTrue(a.heroRing);
            Assert.IsTrue(a.heroTrail);
            Assert.AreEqual(0.34f, a.trailWidth);
            Assert.AreEqual(0.28f, a.previousTrailWidth);
            Assert.IsFalse(a.trailColour.HasValue);
        }

        [Test]
        public void HudModesParse() {
            Assert.AreEqual("minimal", Parse(@"{""hud"": ""minimal"", ""acts"": [{}]}").Hud);
            Assert.AreEqual("none", Parse(@"{""hud"": ""none"", ""acts"": [{}]}").Hud);
            Assert.AreEqual("fleet", Parse(@"{""hud"": ""bogus"", ""acts"": [{}]}").Hud);
            var s = Parse(@"{""hud"": ""minimal"", ""backend"": ""Unity lockstep"", ""acts"": [{""hero_tint"": 0, ""hero_ring"": false,
                ""hero_trail"": true, ""trail_width"": 0.08, ""trail_colour"": [1, 0.5, 0.25, 0.8]}]}");
            Assert.IsTrue(s.HudSet);
            Assert.AreEqual("Unity lockstep", s.Backend);
            Assert.AreEqual(0f, s.Acts[0].heroTint);
            Assert.IsFalse(s.Acts[0].heroRing);
            Assert.AreEqual(0.08f, s.Acts[0].trailWidth, 1e-6f);
            Assert.AreEqual(new Color(1f, 0.5f, 0.25f, 0.8f), s.Acts[0].trailColour.Value);
        }

        [Test]
        public void MinimalCaptionNamesBackendRunAndTime() {
            var m = new HudModel { backend = "Unity lockstep", goldId = "run-a", runTime = 43.08f };
            Assert.AreEqual("Unity lockstep   ·   run-a   ·   t 43.1 s", HudRenderer.MinimalCaption(m));
            m.caption = "unity-ls-x";
            Assert.AreEqual("Unity lockstep   ·   unity-ls-x   ·   t 43.1 s", HudRenderer.MinimalCaption(m));
            Assert.AreEqual("t 0.0 s", HudRenderer.MinimalCaption(new HudModel { runTime = 0f }));
        }

        [Test]
        public void AppearanceOptionsDoNotChangeTheFrameState() {
            var plain = State(Golden(), Parse(@"{""fps"": 30, ""size"": [640, 360], ""acts"": [" + Compare + "}]}"));
            var demo = State(Golden(), Parse(@"{""fps"": 30, ""size"": [640, 360], ""hud"": ""minimal"", ""acts"": [" + Compare +
                @", ""hero_tint"": 0, ""hero_ring"": false, ""trail_width"": 0.1, ""trail_colour"": [1, 1, 1, 1]}]}"));
            Assert.AreEqual(plain.FrameCount, demo.FrameCount);
            for (int f = 0; f < plain.FrameCount; f += 7) Assert.AreEqual(plain.Evaluate(f).Digest(), demo.Evaluate(f).Digest(), $"frame {f}");
        }

        static void AssertLooksAt(FrameState fs, Vector3 target) {
            var want = (target - fs.camPos).normalized;
            Assert.Greater(Vector3.Dot(fs.camRot * Vector3.forward, want), 0.9999f, "camera looks at the target");
        }

        [Test]
        public void DockShotHoldsAFixedTarget() {
            // PlaneWorld: Unity = (x, z, y). Target (5, 3) on the water; camera 4 m east of it and 2 m up.
            var s = State(Golden(), Parse(@"{""fps"": 30, ""acts"": [{""type"": ""spotlight"", ""duration_s"": 1, ""speed"": 4,
                ""camera"": {""shot"": ""dock"", ""target"": [5, 3, 0], ""bearing"": 0, ""distance"": 4, ""height"": 2}}]}"));
            var first = s.Evaluate(0); var last = s.Evaluate(s.FrameCount - 1);
            Assert.AreEqual(new Vector3(9f, 2f, 3f), first.camPos);
            Assert.AreEqual(first.camPos, last.camPos, "the camera does not follow the boat");
            Assert.AreEqual(first.camRot, last.camRot);
            AssertLooksAt(first, new Vector3(5f, 0f, 3f));
            // bearing 90 is north of the target
            var n = State(Golden(), Parse(@"{""acts"": [{""camera"": {""shot"": ""dock"", ""target"": [5, 3], ""bearing"": 90, ""distance"": 4, ""height"": 2}}]}"));
            var p = n.Evaluate(0).camPos;
            Assert.AreEqual(5f, p.x, 1e-4f); Assert.AreEqual(7f, p.z, 1e-4f);
            // explicit position
            var e = State(Golden(), Parse(@"{""acts"": [{""camera"": {""shot"": ""dock"", ""target"": [5, 3, 0], ""pos"": [1, 1, 3]}}]}"));
            Assert.AreEqual(new Vector3(1f, 3f, 1f), e.Evaluate(0).camPos);
            AssertLooksAt(e.Evaluate(0), new Vector3(5f, 0f, 3f));
        }

        [Test]
        public void WideShotUsesItsPositionAndLookAt() {
            var s = State(Golden(), Parse(@"{""acts"": [{""camera"": {""shot"": ""wide"", ""pos"": [0, -10, 12], ""look"": [4, 2, 0], ""fov"": 40}}]}"));
            var fs = s.Evaluate(0);
            Assert.AreEqual(new Vector3(0f, 12f, -10f), fs.camPos);
            Assert.AreEqual(40f, fs.fov);
            AssertLooksAt(fs, new Vector3(4f, 0f, 2f));
            var u = State(Golden(), Parse(@"{""acts"": [{""camera"": {""shot"": ""wide"", ""space"": ""unity"", ""pos"": [1, 9, 2], ""look"": [3, 0, 4]}}]}"));
            Assert.AreEqual(new Vector3(1f, 9f, 2f), u.Evaluate(0).camPos);
            AssertLooksAt(u.Evaluate(0), new Vector3(3f, 0f, 4f));
            // the 1 m floor above the water still holds unless the shot lowers it
            var low = State(Golden(), Parse(@"{""acts"": [{""camera"": {""shot"": ""wide"", ""pos"": [0, 0, 0.3], ""look"": [4, 0, 0]}}]}"));
            Assert.AreEqual(1f, low.Evaluate(0).camPos.y, 1e-5f);
            var lower = State(Golden(), Parse(@"{""acts"": [{""camera"": {""shot"": ""wide"", ""pos"": [0, 0, 0.3], ""look"": [4, 0, 0], ""min_height"": 0.2}}]}"));
            Assert.AreEqual(0.3f, lower.Evaluate(0).camPos.y, 1e-5f);
        }
    }
}
