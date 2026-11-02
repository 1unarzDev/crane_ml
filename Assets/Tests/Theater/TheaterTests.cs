using System;
using System.Collections.Generic;
using System.IO;
using Crane.Theater;
using NUnit.Framework;
using UnityEngine;

namespace Crane.Theater.Tests {
    /// <summary>The cross-language golden (theater/tests/fixture.py writes it) and the determinism contract:
    /// a frame's state is a pure function of (manifest, script, frame index).</summary>
    public class TheaterTests {
        static string Dir => Path.Combine(Application.dataPath, "Tests", "Theater", "Fixtures");

        static TheaterManifest Golden() => TheaterManifest.Load(Path.Combine(Dir, "golden_manifest.json"));

        static TheaterState State(TheaterManifest m, TheaterScript s) {
            var world = new PlaneWorld();
            var e = world.Prepare(m);
            while (e.MoveNext()) { }
            return new TheaterState(m, s, world);
        }

        static TheaterScript Script() => TheaterScript.Parse(TJson.Parse(@"{""fps"": 30, ""size"": [640, 360], ""title"": ""golden"", ""acts"": [
            {""type"": ""fleet"", ""duration_s"": 2, ""speed"": 6, ""stagger_s"": 1.5, ""colour"": ""iteration"", ""camera"": {""shot"": ""pack""}},
            {""type"": ""evolution"", ""duration_s"": 1.5, ""speed"": 10, ""per_iteration_s"": 0.5, ""camera"": {""shot"": ""top""}},
            {""type"": ""heat"", ""duration_s"": 1, ""speed"": 20, ""heat"": true, ""colour"": ""outcome"", ""camera"": {""shot"": ""orbit""}},
            {""type"": ""compare"", ""duration_s"": 2, ""speed"": 8, ""trails"": ""top"", ""camera"": {""shot"": ""follow""}}]}"));

        [Test]
        public void PackReadsWhatPythonWrote() {
            var m = Golden();
            Assert.AreEqual(9, m.Pack.RunCount);
            Assert.AreEqual(5f, m.Pack.Hz);
            var expect = (List<object>)TJson.Parse(File.ReadAllText(Path.Combine(Dir, "golden_expect.json")));
            foreach (var o in expect) {
                int run = (int)TJson.Num(o, "run"), k = (int)TJson.Num(o, "k");
                var p = m.Pack.Sample(run, k);
                Assert.AreEqual(TJson.Num(o, "x"), p.x, 1e-4, $"x run {run} k {k}");
                Assert.AreEqual(TJson.Num(o, "y"), p.y, 1e-4, $"y run {run} k {k}");
                Assert.AreEqual(TJson.Num(o, "yaw"), p.yaw, 1e-4, $"yaw run {run} k {k}");
                Assert.AreEqual(TJson.Num(o, "roll"), p.roll, 1e-4, $"roll run {run} k {k}");
                Assert.AreEqual(TJson.Num(o, "speed"), p.speed, 1e-4, $"speed run {run} k {k}");
                Assert.AreEqual(TJson.Num(o, "progress"), p.progress, 1e-4, $"progress run {run} k {k}");
            }
            Assert.AreEqual("g0", m.Runs[0].id);
            Assert.AreEqual("failed", m.Runs[2].outcome);
            Assert.AreEqual(2, m.MaxIteration);
        }

        [Test]
        public void CorruptPackIsRefused() {
            var bytes = File.ReadAllBytes(Path.Combine(Dir, "golden.pack"));
            bytes[100] ^= 0x10;
            Assert.Throws<InvalidDataException>(() => TrackPack.Parse(bytes));
        }

        [Test]
        public void InterpolationStaysOnSamples() {
            var m = Golden();
            var a = m.Pack.Sample(3, 10);
            var at = m.Pack.At(3, 10 / 5.0);
            Assert.AreEqual(a.x, at.x, 1e-4); Assert.AreEqual(a.y, at.y, 1e-4);
            var end = m.Pack.At(3, 1e6);
            Assert.AreEqual(m.Pack.Sample(3, m.Pack.Samples(3) - 1).x, end.x, 1e-5);
        }

        [Test]
        public void FrameStateIsAPureFunctionOfTheFrame() {
            var m = Golden();
            var s1 = State(m, Script());
            int n = s1.FrameCount;
            var forward = new string[n];
            for (int f = 0; f < n; f++) forward[f] = s1.Evaluate(f).Digest();
            var s2 = State(Golden(), Script());
            var order = new List<int>();
            for (int f = 0; f < n; f++) order.Add(f);
            var rng = new System.Random(5);
            for (int i = n - 1; i > 0; i--) { int j = rng.Next(i + 1); (order[i], order[j]) = (order[j], order[i]); }
            foreach (int f in order) Assert.AreEqual(forward[f], s2.Evaluate(f).Digest(), $"frame {f}");
            Assert.AreNotEqual(forward[0], forward[n - 1]);
        }

        [Test]
        public void ActsSelectAndLabelAsScripted() {
            var s = State(Golden(), Script());
            Assert.AreEqual(4, s.Plans.Count);
            Assert.AreEqual(195, s.FrameCount);
            var compare = s.Plans[3];
            var records = s.M.Records();
            Assert.AreEqual(records[records.Count - 1].index, compare.hero);
            Assert.AreEqual(records[records.Count - 2].index, compare.previous);
            int lastIteration = int.MinValue;
            for (int f = s.Plans[1].firstFrame; f < s.Plans[1].firstFrame + s.Plans[1].frames; f++) {
                var fs = s.Evaluate(f);
                Assert.GreaterOrEqual(fs.iteration, lastIteration, "the evolution counter only climbs");
                lastIteration = fs.iteration;
                foreach (var g in fs.ghosts) Assert.AreEqual(fs.iteration, s.M.Runs[g.run].iteration);
            }
            Assert.AreEqual(2, lastIteration);
            var cmp = s.Evaluate(compare.firstFrame + 5);
            Assert.AreEqual(2, cmp.heroes.Count);
            Assert.IsTrue(cmp.hud.paramDiff.Count > 0);
            var fleet0 = s.Evaluate(0);
            Assert.Less(fleet0.ghosts.Count, 9, "staggered starts: not every ghost is out at frame 0");
        }

        [Test]
        public void DefaultScriptCoversTheStudy() {
            var m = Golden();
            var sc = TheaterScript.Default(m, 15f);
            Assert.AreEqual(15f, sc.Duration, 1e-3);
            var s = State(m, sc);
            for (int f = 0; f < s.FrameCount; f += 37) {
                var fs = s.Evaluate(f);
                foreach (var g in fs.ghosts) Assert.IsFalse(float.IsNaN(g.pos.x));
                Assert.IsFalse(float.IsNaN(fs.camPos.x));
            }
        }
    }
}
