using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using Crane.Drones.Logging;
using NUnit.Framework;

namespace Sim.Tests.Editor {
    public sealed class DroneLoggingReproTests {
        // Documented tolerance, see Docs/DroneVisuals/LOG_FORMAT.md ("Reproducibility").
        private const double PosTol = 1e-6;
        private string tmp;

        [SetUp] public void SetUp() { tmp = Path.Combine(Path.GetTempPath(), "dronerepro-" + Guid.NewGuid().ToString("N")); }
        [TearDown] public void TearDown() { if (Directory.Exists(tmp)) Directory.Delete(tmp, true); }

        private static DroneSplits Splits() =>
            DroneSplitGuard.Parse(File.ReadAllText(DroneSplitGuard.FindSplitsFile(new string[0])));

        private static List<double[]> ReadState(string dir, out string[] head) {
            string[] lines = File.ReadAllLines(Path.Combine(dir, "state.csv"));
            head = lines[0].Split(',');
            return lines.Skip(1).Select(l => l.Split(',').Select(x => x == "nan" ? double.NaN : double.Parse(x, CultureInfo.InvariantCulture)).ToArray()).ToList();
        }

        [Test]
        public void SameSeedReproducesStateWithinTolerance_DifferentSeedDiffers() {
            var splits = Splits();
            DroneReferenceFlight.Run(Path.Combine(tmp, "a"), 1337, 3.0, splits);
            DroneReferenceFlight.Run(Path.Combine(tmp, "b"), 1337, 3.0, splits);
            DroneReferenceFlight.Run(Path.Combine(tmp, "c"), 1338, 3.0, splits);
            var a = ReadState(Path.Combine(tmp, "a"), out var head);
            var b = ReadState(Path.Combine(tmp, "b"), out _);
            var c = ReadState(Path.Combine(tmp, "c"), out _);
            Assert.That(b.Count, Is.EqualTo(a.Count));
            int ip = Array.IndexOf(head, "px");
            double worst = 0, worstOther = 0;
            for (int i = 0; i < a.Count; i++) {
                Assert.That(b[i][2], Is.EqualTo(a[i][2]), "t_ns must match exactly");
                for (int k = 0; k < 3; k++) {
                    worst = Math.Max(worst, Math.Abs(a[i][ip + k] - b[i][ip + k]));
                    worstOther = Math.Max(worstOther, Math.Abs(a[i][ip + k] - c[i][ip + k]));
                }
            }
            UnityEngine.Debug.Log($"CRANE_DRONE_REPRO in-process max|dpos|={worst:R} rows={a.Count} diff-seed max|dpos|={worstOther:R}");
            Assert.That(worst, Is.LessThanOrEqualTo(PosTol));
            Assert.That(worstOther, Is.GreaterThan(1000 * PosTol), "a different seed must produce a different flight");
        }

        [Test]
        public void StreamsShareOneClockAndAlign() {
            DroneReferenceFlight.Run(tmp, 1337, 3.0, Splits());
            var st = ReadState(tmp, out var head);
            long[] t = st.Select(r => (long)r[2]).ToArray();
            for (int i = 1; i < t.Length; i++) Assert.That(t[i], Is.GreaterThan(t[i - 1]));
            Assert.That(t[1] - t[0], Is.EqualTo(1_000_000), "state at the 1 kHz substep rate (>= 200 Hz)");
            long t0 = t[0], t1 = t[t.Length - 1];
            var stateSet = new HashSet<long>(t);

            long[] imu = File.ReadAllLines(Path.Combine(tmp, "imu.csv")).Skip(1).Select(l => long.Parse(l.Split(',')[0])).ToArray();
            for (int i = 1; i < imu.Length; i++) Assert.That(imu[i] - imu[i - 1], Is.EqualTo(4_000_000));
            foreach (long x in imu) Assert.That(stateSet.Contains(x), "IMU stamps sit on state stamps");

            foreach (string cam in new[] { "oakd_rgb", "oakd_depth" }) {
                var rows = File.ReadAllLines(Path.Combine(tmp, "camera", cam + ".csv")).Skip(1).Select(l => l.Split(',')).ToArray();
                Assert.That(rows.Length, Is.GreaterThan(5));
                long prev = -1;
                foreach (var r in rows) {
                    long acq = long.Parse(r[1]), done = long.Parse(r[2]);
                    Assert.That(acq, Is.GreaterThan(prev)); prev = acq;
                    Assert.That(done, Is.GreaterThan(acq), "completion is after acquisition");
                    Assert.That(stateSet.Contains(acq), "frame acquisition lies on the state clock");
                    Assert.That(File.Exists(Path.Combine(tmp, r[6])));
                }
            }
            foreach (var r in File.ReadAllLines(Path.Combine(tmp, "lidar", "lidar.csv")).Skip(1).Select(l => l.Split(','))) {
                long acq = long.Parse(r[1]), first = long.Parse(r[3]), last = long.Parse(r[4]);
                Assert.That(first, Is.GreaterThanOrEqualTo(acq)); Assert.That(last, Is.GreaterThan(first));
                Assert.That(last, Is.LessThanOrEqualTo(t1)); Assert.That(acq, Is.GreaterThanOrEqualTo(t0));
            }
            long[] ev = File.ReadAllLines(Path.Combine(tmp, "events.jsonl"))
                .Select(l => long.Parse(System.Text.RegularExpressions.Regex.Match(l, "\"t_ns\":(\\d+)").Groups[1].Value)).ToArray();
            for (int i = 1; i < ev.Length; i++) Assert.That(ev[i], Is.GreaterThanOrEqualTo(ev[i - 1]));
            string events = File.ReadAllText(Path.Combine(tmp, "events.jsonl"));
            StringAssert.Contains("near_miss", events);
            StringAssert.Contains("collision", events);
            StringAssert.Contains("detection", events);
            Assert.That(File.ReadAllText(Path.Combine(tmp, "manifest.json")), Does.Contain("\"end_tick\""));
        }
    }
}
