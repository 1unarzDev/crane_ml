using System.Collections.Generic;
using System.IO;
using NUnit.Framework;
using Sim.Performance;
using UnityEngine;

namespace Sim.Tests.Performance {
    /// <summary>The C# crane-traj/1 codec against the Python one: theater/tests/golden/small.ctraj is
    /// written by theater/traj.py (values in small.json), so reading it here keeps the two in step.</summary>
    public class CraneTrajTests {
        private static string Golden => Path.GetFullPath(Path.Combine(Application.dataPath, "..", "theater", "tests", "golden", "small.ctraj"));

        private static void Near(double expected, double actual, string name) =>
            Assert.That(actual, Is.EqualTo(expected).Within(CraneTraj.ScaleOf(name) / 2 + 1e-9), name);

        [Test]
        public void ReadsThePythonGoldenFile() {
            CraneTraj.Data d = CraneTraj.Decode(File.ReadAllBytes(Golden));
            Assert.AreEqual(700, d.Ticks);
            Assert.AreEqual(600, d.BlockTicks);
            Assert.AreEqual(20.0, d.Hz);
            CollectionAssert.AreEqual(new[] { "boat.x", "boat.y", "boat.z", "boat.yaw", "boat.pitch", "boat.roll", "boat.speed" },
                d.Columns.ConvertAll(c => c.Name));
            Near(-12.5, d["boat.x"].Values[0], "boat.x");
            Near(0.8176, d["boat.yaw"].Values[0], "boat.yaw");
            Near(-0.03, d["boat.roll"].Values[0], "boat.roll");
            Near(32.5, d["boat.x"].Values[600], "boat.x");
            Near(37.76, d["boat.y"].Values[600], "boat.y");
            Near(-0.0198, d["boat.pitch"].Values[600], "boat.pitch");
            Near(1.401, d["boat.speed"].Values[600], "boat.speed");
            Near(39.93, d["boat.x"].Values[699], "boat.x");
            Near(0.04, d["boat.z"].Values[699], "boat.z");
            Near(0.6815, d["boat.yaw"].Values[699], "boat.yaw");
            Assert.AreEqual(2, d.EventLines.Count);
            StringAssert.Contains("\"kind\":\"gate\"", d.EventLines[0]);
            StringAssert.Contains("\"id\":\"buoy-3\"", d.EventLines[1]);
            StringAssert.Contains("\"schema\":\"crane-traj/1\"", d.HeaderJson);
        }

        [Test]
        public void RoundTripsItsOwnFiles() {
            var columns = new List<CraneTraj.Column>();
            foreach (string name in new[] { "r.x", "r.y", "r.yaw" })
                columns.Add(new CraneTraj.Column { Name = name, Scale = CraneTraj.ScaleOf(name) });
            for (int i = 0; i < 1500; i++) {
                columns[0].Values.Add(-300 + 0.07 * i);
                columns[1].Values.Add(12.34 * System.Math.Sin(i / 50.0));
                columns[2].Values.Add(System.Math.Atan2(System.Math.Sin(i / 9.0), System.Math.Cos(i / 9.0)));
            }
            byte[] bytes = CraneTraj.Encode("{\"hz\":10,\"schema\":\"crane-traj/1\"}", 10, columns,
                new[] { "{\"kind\":\"note\",\"t\":1.5}" });
            CraneTraj.Data d = CraneTraj.Decode(bytes);
            Assert.AreEqual(1500, d.Ticks);
            Assert.AreEqual(3, d.Columns.Count);
            for (int c = 0; c < 3; c++)
                for (int i = 0; i < 1500; i += 37) Near(columns[c].Values[i], d.Columns[c].Values[i], columns[c].Name);
            Assert.AreEqual("{\"kind\":\"note\",\"t\":1.5}", d.EventLines[0]);
            bytes[bytes.Length / 2] ^= 0x5A;
            Assert.Throws<InvalidDataException>(() => CraneTraj.Decode(bytes));
        }
    }
}
