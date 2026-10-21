using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Threading;
using NUnit.Framework;
using Sim.Underwater;
using Sim.Utils.Performance;
using UnityEngine;
using static Sim.Underwater.CraneLinAlg;

namespace Sim.Tests.Editor.Underwater {
    /// <summary>Pressure and DVL: parity with Tools/Underwater/sensor_models.py, seeding/reset, timestamps, no ground-truth leakage.</summary>
    [TestFixture, Category("UnderwaterParity")]
    public sealed class CraneUnderwaterSensorTests {
        static readonly string Path_ = Path.Combine(Application.dataPath, "Tests/Editor/Underwater/Golden/sensor_reference.json");
        Dictionary<string, object> g;
        CraneUnderwaterProfile profile;

        [OneTimeSetUp] public void Load() {
            g = (Dictionary<string, object>)CraneMiniJson.Parse(File.ReadAllText(Path_));
            profile = CraneUnderwaterProfile.LoadFromResources("bluerov2_heavy");
        }

        static double[] Arr(object o) => ((List<object>)o).Select(x => x == null ? double.PositiveInfinity : (double)x).ToArray();
        static Dictionary<string, object> D(object o) => (Dictionary<string, object>)o;
        static double MaxDiff(double[] a, double[] b) { double m = 0; for (int i = 0; i < a.Length; i++) m = Math.Max(m, Math.Abs(a[i] - b[i])); return m; }

        [Test] public void Pressure_Parity_With_Python() {
            double worst = 0;
            foreach (var c in ((List<object>)g["pressure"]).Select(D)) {
                var s = new CranePressureSensor { PortBody = Arr(c["port"]), Rho = (double)c["rho"], ResolutionPa = (double)c["resolution"], SurfaceZ = (double)c["surfaceZ"], Gravity = (double)g["g"] };
                double p = s.Measure(Arr(c["pos"]), Arr(c["q"]));
                worst = Math.Max(worst, Math.Abs(p - (double)c["pressure"]) / (double)c["pressure"]);
                Assert.AreEqual((double)c["depth"], s.DepthFromPressure(p), 1e-8);
            }
            Assert.That(worst, Is.LessThan(1e-12));
            Debug.Log("SENSOR_PARITY pressure max relative error " + worst.ToString("E3"));
        }

        [Test] public void Dvl_Parity_With_Python_Including_Dropouts_And_Water_Track() {
            double worst = 0; int lost = 0, partial = 0, water = 0, n = 0;
            var beams = Mat(g["janusBeams"]);
            foreach (var c in ((List<object>)g["dvl"]).Select(D)) {
                var d = new CraneDvl { MountBody = Arr(c["mount"]), MinRange = (double)c["minRange"], MaxRange = (double)c["maxRange"] };
                for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) Assert.AreEqual(beams[i, j], d.BeamsSensor[i, j], 1e-15);
                bool wt = (string)c["mode"] == "water";
                var m = d.Measure(Arr(c["pos"]), Arr(c["q"]), Arr(c["nu"]), (double)c["seabedZ"], Arr(c["current"]), wt);
                var valid = ((List<object>)c["valid"]).Select(x => (bool)x).ToArray();
                CollectionAssert.AreEqual(valid, m.Valid);
                var ranges = Arr(c["ranges"]);
                for (int i = 0; i < 4; i++) {
                    if (double.IsInfinity(ranges[i])) Assert.IsTrue(double.IsInfinity(m.Ranges[i])); else worst = Math.Max(worst, Math.Abs(ranges[i] - m.Ranges[i]));
                }
                worst = Math.Max(worst, MaxDiff(m.Radial, Arr(c["radial"])));
                if (c["v"] == null) { Assert.IsFalse(m.Lock); lost++; } else { Assert.IsTrue(m.Lock); worst = Math.Max(worst, MaxDiff(m.VelocitySensor, Arr(c["v"]))); }
                if (valid.Any(x => !x) && valid.Any(x => x)) partial++;
                if (wt) water++;
                n++;
            }
            Assert.AreEqual(60, n); Assert.Greater(lost, 5); Assert.GreaterOrEqual(partial, 3); Assert.Greater(water, 10);
            Assert.That(worst, Is.LessThan(1e-9));
            Debug.Log($"SENSOR_PARITY dvl max abs error {worst:E3} (lockLost={lost}, partial={partial}, water={water})");
        }

        static double[,] Mat(object o) {
            var rows = ((List<object>)o).Select(Arr).ToArray();
            var m = new double[rows.Length, rows[0].Length];
            for (int i = 0; i < rows.Length; i++) for (int j = 0; j < rows[0].Length; j++) m[i, j] = rows[i][j];
            return m;
        }

        [Test] public void Dvl_Reports_In_Sensor_Frame_And_Drops_Out_Beyond_Max_Range() {
            // sensor yawed +90 deg about body Z relative to body: sensor x = body y, sensor y = -body x
            var rbs = new double[,] { { 0, -1, 0 }, { 1, 0, 0 }, { 0, 0, 1 } };
            var d = new CraneDvl { SensorToBody = rbs, MaxRange = 20 };
            var q = new double[] { 1, 0, 0, 0 };
            var nu = new double[] { 0.4, 0.1, 0.0, 0, 0, 0 };
            var m = d.Measure(new double[] { 0, 0, 1 }, q, nu, 6.0);
            Assert.IsTrue(m.Lock);
            var expect = Mul(Transpose(rbs), new[] { 0.4, 0.1, 0.0 });
            for (int i = 0; i < 3; i++) Assert.AreEqual(expect[i], m.VelocitySensor[i], 1e-12);
            Assert.AreEqual(5.0, d.Altitude(m, q), 1e-9);
            // altitude beyond max range (slant 25 / cos 22.5 > 20): drop out
            var far = d.Measure(new double[] { 0, 0, 1 }, q, nu, 1.0 + 20.0);
            Assert.IsFalse(far.Lock);
            Assert.IsTrue(far.Valid.All(v => !v));
        }

        [Test] public void Dvl_Bottom_Track_Ignores_Current_Water_Track_Subtracts_It() {
            var d = new CraneDvl();
            var q = QuatNormalize(new[] { 0.9, 0.1, 0.2, 0.3 });
            var nu = new double[] { 0.3, -0.1, 0.05, 0.02, 0.01, -0.03 };
            var b1 = d.Measure(new double[] { 0, 0, 1 }, q, nu, 5, new[] { 0.5, 0.0, 0.0 });
            var b2 = d.Measure(new double[] { 0, 0, 1 }, q, nu, 5, new[] { -0.2, 0.7, 0.1 });
            for (int i = 0; i < 3; i++) Assert.AreEqual(b1.VelocitySensor[i], b2.VelocitySensor[i], 0);
            var w = d.Measure(new double[] { 0, 0, 1 }, q, nu, 5, new[] { 0.5, 0.0, 0.0 }, true);
            Assert.That(Math.Abs(b1.VelocitySensor[0] - w.VelocitySensor[0]), Is.GreaterThan(1e-3));
        }

        [Test] public void Rng_Is_Deterministic_And_Gaussian() {
            var a = new CraneRng(123); var b = new CraneRng(123); var c = new CraneRng(124);
            bool differs = false;
            for (int i = 0; i < 100; i++) { double x = a.NextGaussian(); Assert.AreEqual(x, b.NextGaussian(), 0); if (x != c.NextGaussian()) differs = true; }
            Assert.IsTrue(differs);
            var r = new CraneRng(5); double s = 0, s2 = 0; int n = 200000;
            for (int i = 0; i < n; i++) { double x = r.NextGaussian(); s += x; s2 += x * x; }
            Assert.AreEqual(0.0, s / n, 0.01); Assert.AreEqual(1.0, s2 / n, 0.02);
        }

        [Test] public void Dvl_Noise_Covariance_Matches_Janus_Geometry_Like_Python() {
            var d = new CraneDvl { SigmaBeam = 0.01 };
            var rng = new CraneRng(1);
            int n = 4000; var vs = new double[n, 3];
            for (int k = 0; k < n; k++) {
                var m = d.Measure(new double[] { 0, 0, 0 }, new double[] { 1, 0, 0, 0 }, new double[6], 5.0, null, false, rng);
                for (int j = 0; j < 3; j++) vs[k, j] = m.VelocitySensor[j];
            }
            var cov = Scale(Inverse(Mul(Transpose(d.BeamsSensor), d.BeamsSensor)), 0.01 * 0.01);
            var sd = new double[3];
            for (int j = 0; j < 3; j++) {
                double mean = 0; for (int k = 0; k < n; k++) mean += vs[k, j]; mean /= n;
                double v = 0; for (int k = 0; k < n; k++) v += (vs[k, j] - mean) * (vs[k, j] - mean);
                sd[j] = Math.Sqrt(v / n);
                Assert.AreEqual(Math.Sqrt(cov[j, j]), sd[j], 0.06 * Math.Sqrt(cov[j, j]), "axis " + j);
            }
            Assert.Greater(sd[0], sd[2]);
        }

        [Test] public void Pressure_Noise_Is_Seeded_Zero_Mean_With_Configured_Sigma() {
            var s = new CranePressureSensor { SigmaPa = 50 };
            var q = new double[] { 1, 0, 0, 0 };
            var r = new CraneRng(9); double sum = 0, sum2 = 0; int n = 20000;
            double clean = new CranePressureSensor().Measure(new double[] { 0, 0, 3 }, q);
            for (int i = 0; i < n; i++) { double e = s.Measure(new double[] { 0, 0, 3 }, q, r) - clean; sum += e; sum2 += e * e; }
            Assert.AreEqual(0, sum / n, 2.0); Assert.AreEqual(50, Math.Sqrt(sum2 / n), 1.5);
        }

        // ---- adapter -----------------------------------------------------------------
        GameObject MakeVehicle(out CraneUnderwaterBody body, out CraneUnderwaterSensors sensors) {
            var go = new GameObject("uwsensors");
            go.AddComponent<Rigidbody>();
            body = go.AddComponent<CraneUnderwaterBody>();
            body.InitializeFromCommandLine(new[] { "Unity" });
            sensors = go.AddComponent<CraneUnderwaterSensors>();
            sensors.GetType().GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance).Invoke(sensors, null);   // Awake does not run in EditMode
            body.Configure(profile);
            go.transform.position = new Vector3(0, -3, 0);     // 3 m deep (NED z = 3)
            go.GetComponent<Rigidbody>().position = go.transform.position;   // EditMode: no transform->physics sync
            return go;
        }

        static (CranePressureObservation, CraneDvlObservation) Run(CraneUnderwaterSensors s, int ticks, double dt) {
            for (int i = 0; i < ticks; i++) s.Tick(dt);
            return (s.LatestPressure, s.LatestDvl);
        }

        [Test] public void Adapter_Is_Inert_When_Body_Inactive() {
            var go = new GameObject("inert"); go.AddComponent<Rigidbody>();
            var body = go.AddComponent<CraneUnderwaterBody>(); body.InitializeFromCommandLine(new[] { "Unity" });
            var s = go.AddComponent<CraneUnderwaterSensors>();
            try { s.Tick(0.02); s.Tick(0.02); Assert.AreEqual(0, s.LatestPressure.Sequence); Assert.AreEqual(0, s.LatestDvl.Sequence); }
            finally { UnityEngine.Object.DestroyImmediate(go); }
        }

        [Test] public void Adapter_Stamps_Physics_Ticks_Not_Wall_Clock_Seeds_Per_Episode_And_Reset_Clears() {
            var go = MakeVehicle(out var body, out var s);
            try {
                s.PressureModel.SigmaPa = 30; s.DvlModel.SigmaBeam = 0.01;
                s.Configure(77, 8.0, Vector3.zero, false);
                const double dt = 0.02;
                var (p1, d1) = Run(s, 11, dt);
                Thread.Sleep(60);                                       // wall time passes; ticks must not care
                Assert.AreEqual(10, p1.PhysicsTick); Assert.AreEqual(10 * dt, p1.TimeSeconds, 0);
                Assert.AreEqual(10, d1.PhysicsTick); Assert.AreEqual(3, p1.Sequence); Assert.AreEqual(2, d1.Sequence);
                Assert.IsTrue(d1.BottomLock);
                Assert.AreEqual(5.0, d1.AltitudeMeters, 1e-6);          // 8 m bottom - 3 m depth
                Assert.AreEqual(0.0, d1.VelX, 0.2);
                double depth = p1.DepthMeters;
                Assert.AreNotEqual(0.0, p1.PressurePa - (CranePressureSensor.PAtm + 997.0 * CraneVehicleParams.StandardGravity * 3.0), "noise must be present");

                // same seed after reset reproduces the stream exactly
                s.ResetEpisode(new CraneEpisodeResetContext(), CraneEpisodeResetPhase.BeforePhysics);   // Seed 0
                Assert.AreEqual(0, s.TickCount); Assert.AreEqual(0, s.LatestPressure.Sequence); Assert.AreEqual(0, s.LatestDvl.Sequence);
                Assert.AreEqual(default(CranePressureObservation).PressurePa, s.LatestPressure.PressurePa);
                var (pA, dA) = Run(s, 11, dt);
                s.ResetEpisode(new CraneEpisodeResetContext(), CraneEpisodeResetPhase.BeforePhysics);
                var (pB, dB) = Run(s, 11, dt);
                Assert.AreEqual(pA.PressurePa, pB.PressurePa, 0); Assert.AreEqual(dA.VelX, dB.VelX, 0); Assert.AreEqual(dA.VelZ, dB.VelZ, 0);
                Assert.AreNotEqual(p1.PressurePa, pA.PressurePa, "a different episode seed must give a different noise stream");
                Assert.AreEqual(10, pB.PhysicsTick);
            } finally { UnityEngine.Object.DestroyImmediate(go); }
        }

        [Test] public void Adapter_Dvl_Drops_Out_When_Altitude_Exceeds_Max_Range() {
            var go = MakeVehicle(out var body, out var s);
            try {
                s.DvlModel.MaxRange = 6.0;
                s.Configure(1, 3.0 + 40.0, Vector3.zero, false);       // 40 m below the vehicle
                var (_, d) = Run(s, 1, 0.02);
                Assert.IsFalse(d.BottomLock); Assert.AreEqual(0, d.ValidBeamMask); Assert.IsTrue(double.IsNaN(d.AltitudeMeters));
                Assert.AreEqual(0.0, d.VelX); Assert.AreEqual(1, d.Sequence, "a dropout is still a published (lockless) sample");
            } finally { UnityEngine.Object.DestroyImmediate(go); }
        }

        // ---- no ground-truth leakage -----------------------------------------------------
        static readonly string[] Banned = { "truth", "true", "ground", "seabed", "bottomz", "current", "seed", "pos", "rng", "range", "waterdepth" };

        static IEnumerable<(string, Type)> PublicData(Type t) =>
            t.GetFields(BindingFlags.Public | BindingFlags.Instance | BindingFlags.DeclaredOnly).Select(f => (f.Name, f.FieldType))
             .Concat(t.GetProperties(BindingFlags.Public | BindingFlags.Instance | BindingFlags.DeclaredOnly).Select(p => (p.Name, p.PropertyType)));

        [Test] public void Observation_Structs_Contain_Only_Measured_Fields() {
            foreach (var t in new[] { typeof(CranePressureObservation), typeof(CraneDvlObservation) }) {
                Assert.IsTrue(t.IsValueType);
                foreach (var (name, type) in PublicData(t)) {
                    Assert.IsTrue(type.IsPrimitive, $"{t.Name}.{name} must be a primitive (no arrays/objects that could alias truth): {type}");
                    string n = name.ToLowerInvariant();
                    foreach (var b in Banned) Assert.IsFalse(n.Contains(b), $"{t.Name}.{name} looks like ground truth");
                }
            }
        }

        [Test] public void Sensor_Adapter_Public_Surface_Exposes_Only_Observations_And_Counters() {
            var allowed = new HashSet<Type> { typeof(CranePressureObservation), typeof(CraneDvlObservation), typeof(long), typeof(int), typeof(bool),
                                             typeof(CranePressureSensor), typeof(CraneDvl) };   // models carry configuration only (noise, mount, limits)
            foreach (var (name, type) in PublicData(typeof(CraneUnderwaterSensors))) {
                Assert.IsTrue(allowed.Contains(type), $"unexpected public data {name}: {type}");
                foreach (var b in Banned) Assert.IsFalse(name.ToLowerInvariant().Contains(b), $"{name} looks like ground truth");
            }
            foreach (var f in typeof(CraneUnderwaterSensors).GetFields(BindingFlags.Public | BindingFlags.Instance | BindingFlags.DeclaredOnly)) Assert.Fail("no public fields allowed: " + f.Name);
        }

        [Test] public void Observation_Does_Not_Change_With_Hidden_Truth_That_Cannot_Be_Measured() {
            // Bottom-track velocity must be independent of the (hidden) water current; two vehicles differing only in current publish identical samples.
            var go1 = MakeVehicle(out _, out var s1); var go2 = MakeVehicle(out _, out var s2);
            try {
                s1.Configure(5, 9.0, new Vector3(0.5f, 0.1f, 0f), false); s2.Configure(5, 9.0, new Vector3(-0.3f, 0.9f, 0.2f), false);
                var (p1, d1) = Run(s1, 11, 0.02); var (p2, d2) = Run(s2, 11, 0.02);
                Assert.AreEqual(p1.PressurePa, p2.PressurePa, 0);
                Assert.AreEqual(d1.VelX, d2.VelX, 0); Assert.AreEqual(d1.VelY, d2.VelY, 0); Assert.AreEqual(d1.VelZ, d2.VelZ, 0);
            } finally { UnityEngine.Object.DestroyImmediate(go1); UnityEngine.Object.DestroyImmediate(go2); }
        }
    }
}
