using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Crane.Drones.Logging;
using NUnit.Framework;

namespace Sim.Tests.Editor {
    public sealed class DroneLoggingTests {
        private string tmp;

        [SetUp] public void SetUp() { tmp = Path.Combine(Path.GetTempPath(), "dronelog-" + Guid.NewGuid().ToString("N")); }
        [TearDown] public void TearDown() { if (Directory.Exists(tmp)) Directory.Delete(tmp, true); }

        private static DroneSplits RealSplits() {
            string path = DroneSplitGuard.FindSplitsFile(new string[0]);
            Assert.That(path, Is.Not.Null, "Tools/DroneData/splits.json must be found from the project");
            return DroneSplitGuard.Parse(File.ReadAllText(path));
        }

        [Test]
        public void ClockIsExactIntegerNanoseconds() {
            var c = new DroneSimClock(5_000_000, 5);
            Assert.That(c.TimeNs(3, 2), Is.EqualTo(3 * 5_000_000L + 2_000_000L));
            Assert.Throws<ArgumentException>(() => new DroneSimClock(5_000_001, 5));
        }

        [Test]
        public void WriterRejectsNonMonotonicStreams() {
            var m = DroneReferenceFlight.MakeManifest("mono", 5);
            using var w = new DroneRunWriter(tmp, m);
            var s = new DroneStateSample(4);
            w.WriteState(1, 0, s);
            Assert.Throws<InvalidDataException>(() => w.WriteState(1, 0, s));
            Assert.Throws<InvalidDataException>(() => w.WriteState(0, 4, s));
            w.WriteImu(10, default);
            Assert.Throws<InvalidDataException>(() => w.WriteImu(10, default));
            var scan = new DroneLidarScan { X = new float[2], Y = new float[2], Z = new float[2], Intensity = new float[2], TimeOffsetNs = new[] { 5, 3 } };
            Assert.Throws<InvalidDataException>(() => w.WriteLidarScan(scan));
            var f = new DroneCameraFrame { Camera = "c", Kind = DroneCameraKind.Depth, Width = 1, Height = 1, Depth16 = new ushort[1], AcquisitionNs = 10, CompletionNs = 5 };
            Assert.Throws<InvalidDataException>(() => w.WriteCameraFrame(f));
        }

        [Test]
        public void DepthPngIsSixteenBitBigEndian() {
            byte[] png = DronePng.EncodeGray16(new ushort[] { 0x1234, 65535, 0, 1 }, 2, 2);
            Assert.That(png.Take(8), Is.EqualTo(new byte[] { 0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A }));
            Assert.That(png[24], Is.EqualTo(16), "IHDR bit depth");
            Assert.That(png[25], Is.EqualTo(0), "IHDR colour type = grayscale");
        }

        [Test]
        public void SplitFileIsDisjointAndHeldOutExists() {
            var s = RealSplits();
            var rs = s.seed_ranges.OrderBy(r => r.start).ToArray();
            for (int i = 1; i < rs.Length; i++) Assert.That(rs[i].start, Is.GreaterThan(rs[i - 1].end));
            Assert.That(rs.Any(r => r.split == DroneSplitGuard.Heldout));
            Assert.That(DroneSplitGuard.SplitOfSeed(s, 1337), Is.EqualTo("dev"), "ForestSpec default seed is a dev seed");
        }

        [TestCase(95000)]
        [TestCase(90000)]
        [TestCase(99999)]
        public void HeldoutSeedRefusedWithoutFlag(int seed) {
            var s = RealSplits();
            Assert.Throws<DroneHeldoutException>(() => DroneSplitGuard.Enforce(s, seed, null, null, false));
            UnityEngine.TestTools.LogAssert.Expect(UnityEngine.LogType.Warning, new System.Text.RegularExpressions.Regex("HELDOUT_ALLOWED"));
            Assert.That(DroneSplitGuard.Enforce(s, seed, null, null, true), Is.EqualTo("heldout"));
        }

        [Test]
        public void UnassignedAndHeldoutDisturbanceRefused() {
            var s = RealSplits();
            Assert.Throws<DroneHeldoutException>(() => DroneSplitGuard.Enforce(s, 100000, null, null, false));
            Assert.Throws<DroneHeldoutException>(() => DroneSplitGuard.Enforce(s, 500, "storm_gust", null, false));
            Assert.Throws<DroneHeldoutException>(() => DroneSplitGuard.Enforce(s, 500, "calm", "HeavyFog", false));
            Assert.That(DroneSplitGuard.Enforce(s, 500, "calm", "GoldenHour", false), Is.EqualTo("dev"));
            Assert.That(DroneSplitGuard.Enforce(s, 75000, "moderate_gusty", "Dusk", false), Is.EqualTo("val"));
        }

        [Test]
        public void EntryPointRefusesHeldoutAndMissingSeed() {
            string splits = DroneSplitGuard.FindSplitsFile(new string[0]);
            Assert.Throws<DroneHeldoutException>(() => DroneSplitGuard.EnforceFromCommandLine(
                new[] { "--drone-splits", splits, "--drone-seed", "91234" }));
            Assert.Throws<DroneHeldoutException>(() => DroneSplitGuard.EnforceFromCommandLine(new[] { "--drone-splits", splits }));
            Assert.That(DroneSplitGuard.EnforceFromCommandLine(new[] { "--drone-splits", splits, "--drone-seed", "1337" }), Is.EqualTo("dev"));
        }

        [Test]
        public void FlightEntryRefusesHeldoutBeforeCreatingAnything() {
            Assert.Throws<DroneHeldoutException>(() => DroneReferenceFlight.Run(tmp, 95000, 0.1, RealSplits()));
            Assert.That(Directory.Exists(tmp), Is.False);
        }

        [Test]
        public void ClearanceTrackerEmitsOneNearMissWithMinimum() {
            var m = DroneReferenceFlight.MakeManifest("nm", 5);
            using var w = new DroneRunWriter(tmp, m);
            var t = new DroneClearanceTracker(0.5);
            double[] d = { 2, 0.4, 0.2, 0.3, 1.0, 2.0 };
            for (int i = 0; i < d.Length; i++) t.Update(i * 10L, d[i], "tree", w);
            t.Flush(100, w);
            w.Close(0);
            var lines = File.ReadAllLines(Path.Combine(tmp, "events.jsonl"));
            Assert.That(lines.Length, Is.EqualTo(1));
            StringAssert.Contains("\"distance_m\":0.2", lines[0]);
            StringAssert.Contains("\"t_min_ns\":20", lines[0]);
        }
    }
}
