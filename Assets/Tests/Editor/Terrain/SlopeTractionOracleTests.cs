using System;
using System.Collections.Generic;
using System.IO;
using NUnit.Framework;
using Sim.Physics.LandTerrain;
using UnityEngine;

namespace Sim.Tests.Terrain
{
    [Serializable] class HoldRef { public double mu, angle; }
    [Serializable] class TipRef { public double lateralRoll, lateralRollNoOffset, longitudinalPitch; }
    [Serializable] class ClimbTorqueRef { public double totalTorque, crr, angle; }
    [Serializable] class ClimbTractionRef { public double mu, crr, angle, cgFromRear; public string driven; }
    [Serializable] class LoadRef { public string kind; public double slope, acc, front, rear, left, right; }
    [Serializable] class CreepRef { public double slope, vy, slopeDeg, crr, creep; }
    [Serializable] class CrossRef { public double limit; public CreepRef[] creep; }
    [Serializable] class ZeroRef { public double holdLimit; public CreepRef[] creep; }
    [Serializable] class YawRef { public double slopeDeg, creep, planeForce; public int[] headingsDeg; }
    [Serializable] class RollRef { public double slopeDeg, force; }
    [Serializable]
    class GeomRef { public double wheelbase, track, cgFromRear, cgHeight, cgLateral, wheelRadius, mass; }
    [Serializable]
    class OracleRefs
    {
        public double gravity, mu, cKappa, cAlpha, velocityEpsilon;
        public GeomRef geometry;
        public HoldRef[] holdLimit;
        public TipRef tip;
        public ClimbTorqueRef[] climbTorque;
        public ClimbTractionRef[] climbTraction;
        public LoadRef[] loadTransfer;
        public CrossRef crossSlopeHold;
        public ZeroRef zeroCommand;
        public YawRef yawEquivariance;
        public RollRef[] rollingResistance;
    }

    // Pure-math checks of SlopeTractionMath against values exported from Tools/Terrain/terrain_reference.py
    // by Tools/Terrain/export_unity_references.py. Not a calibrated-vehicle claim: FIXTURE_VALIDATED only.
    [TestFixture, Category("TerrainSlope")]
    public class SlopeTractionOracleTests
    {
        static OracleRefs refs;
        static readonly Dictionary<string, double> maxErr = new Dictionary<string, double>();

        [OneTimeSetUp]
        public void Load()
        {
            string path = Path.Combine(Application.dataPath, "Tests/Editor/Terrain/Fixtures/terrain_oracle_references.json");
            refs = JsonUtility.FromJson<OracleRefs>(File.ReadAllText(path));
            Assert.IsNotNull(refs.geometry);
            maxErr.Clear();
        }

        [OneTimeTearDown]
        public void Report()
        {
            foreach (var kv in maxErr) Debug.Log("MAXERR " + kv.Key + " = " + kv.Value.ToString("E3"));
        }

        static void Track(string key, double err)
        {
            err = Math.Abs(err);
            if (!maxErr.TryGetValue(key, out double cur) || err > cur) maxErr[key] = err;
        }

        static VehicleGeometry Geo(double cgFromRear, double cgLat)
        {
            var g = refs.geometry;
            double hx = g.track / 2;
            return new VehicleGeometry
            {
                mass = g.mass, wheelRadius = g.wheelRadius, cg = new V3(cgFromRear, cgLat, g.cgHeight),
                supports = new[]
                {
                    new Support { x = 0, y = hx, driven = true },              // 0 rear left
                    new Support { x = 0, y = -hx, driven = true },             // 1 rear right
                    new Support { x = g.wheelbase, y = hx, driven = true },    // 2 front left
                    new Support { x = g.wheelbase, y = -hx, driven = true },   // 3 front right
                },
            };
        }

        static TractionParams Params(double mu, double crr, double torque = double.PositiveInfinity)
        {
            return new TractionParams
            {
                mu = mu, rollingResistance = crr, wheelTorqueLimit = torque, gravity = refs.gravity,
                longitudinalStiffness = refs.cKappa, lateralStiffness = refs.cAlpha, velocityEpsilon = refs.velocityEpsilon,
            };
        }

        // Steady sliding speed (m/s) of a fully braked vehicle on a plane, or NaN if friction cannot hold it.
        static double Creep(VehicleGeometry geo, TractionParams p, double slope, double heading, out double planeForce)
        {
            var model = new SlopeTractionModel(geo, p);
            V3 g = SlopeTractionMath.GravityInBody(slope, heading, p.gravity);
            double gt = Math.Sqrt(g.x * g.x + g.y * g.y);
            double dx = g.x / gt, dy = g.y / gt;               // downhill unit vector
            double need = geo.mass * gt;
            var zero = new double[geo.supports.Length];
            Func<double, double> resist = s =>
            {
                var f = model.Step(g, new BodyMotion { vx = s * dx, vy = s * dy }, zero, 0.01);
                return -(f.totalFx * dx + f.totalFy * dy);
            };
            planeForce = need;
            if (resist(1.0) < need) return double.NaN;
            double lo = 0, hi = 1.0;
            for (int i = 0; i < 200; i++) { double mid = 0.5 * (lo + hi); if (resist(mid) < need) lo = mid; else hi = mid; }
            return 0.5 * (lo + hi);
        }

        static double Bisect(Func<double, bool> predicate, double lo, double hi)
        {
            for (int i = 0; i < 200; i++) { double mid = 0.5 * (lo + hi); if (predicate(mid)) lo = mid; else hi = mid; }
            return 0.5 * (lo + hi);
        }

        [Test]
        public void SlopeHoldLimit_IsAtanMu()
        {
            foreach (var h in refs.holdLimit)
            {
                var geo = Geo(refs.geometry.cgFromRear, 0);
                double a = Bisect(th => !double.IsNaN(Creep(geo, Params(h.mu, 0), th, 0, out _)), 0.0, 1.45);
                Track("holdLimit", a - h.angle);
                Assert.AreEqual(h.angle, a, 1e-8, "mu=" + h.mu);
            }
        }

        [Test]
        public void TipOverAngle_Lateral_And_Longitudinal()
        {
            var t = refs.tip;
            foreach (var (lat, expect) in new[] { (refs.geometry.cgLateral, t.lateralRoll), (0.0, t.lateralRollNoOffset) })
            {
                var geo = Geo(refs.geometry.cgFromRear, lat);
                var loads = new double[4];
                double a = Bisect(th => SlopeTractionMath.SolveLoads(geo, SlopeTractionMath.GravityInBody(th, Math.PI / 2), 0, 0, loads), 0.0, 1.5);
                Track("tipLateral", a - expect);
                Assert.AreEqual(expect, a, 1e-9, "lateral cgLat=" + lat);
            }
            var g2 = Geo(refs.geometry.cgFromRear, 0);
            var l2 = new double[4];
            double p = Bisect(th => SlopeTractionMath.SolveLoads(g2, SlopeTractionMath.GravityInBody(th, 0), 0, 0, l2), 0.0, 1.5);
            Track("tipLongitudinal", p - t.longitudinalPitch);
            Assert.AreEqual(t.longitudinalPitch, p, 1e-9);
        }

        [Test]
        public void ClimbLimit_TorqueLimited()
        {
            foreach (var c in refs.climbTorque)
            {
                var geo = Geo(refs.geometry.cgFromRear, 0);
                double a = SlopeTractionMath.MaxClimbAngle(geo, Params(10.0, c.crr, c.totalTorque / 4));
                Track("climbTorque", a - c.angle);
                Assert.AreEqual(c.angle, a, 1e-9, "T=" + c.totalTorque + " crr=" + c.crr);
            }
        }

        [Test]
        public void ClimbLimit_TractionLimited_AllFrontRear()
        {
            foreach (var c in refs.climbTraction)
            {
                var geo = Geo(c.cgFromRear, 0);
                Func<Support, bool> filter = c.driven == "all" ? (Func<Support, bool>)null
                    : c.driven == "front" ? (s => s.x > 0.1) : (s => s.x < 0.1);
                double a = SlopeTractionMath.MaxClimbAngle(geo, Params(c.mu, c.crr), filter);
                Track("climbTraction", a - c.angle);
                Assert.AreEqual(c.angle, a, 1e-9, c.driven + " mu=" + c.mu + " crr=" + c.crr + " xr=" + c.cgFromRear);
            }
        }

        [Test]
        public void LoadTransfer_MatchesAxleAndSideLoads()
        {
            var geo = Geo(refs.geometry.cgFromRear, refs.geometry.cgLateral);
            double W = refs.geometry.mass * refs.gravity;
            var n = new double[4];
            foreach (var r in refs.loadTransfer)
            {
                bool lon = r.kind == "long";
                V3 g = SlopeTractionMath.GravityInBody(r.slope, lon ? 0 : Math.PI / 2);
                Assert.IsTrue(SlopeTractionMath.SolveLoads(geo, g, lon ? r.acc : 0, lon ? 0 : r.acc, n));
                if (lon)
                {
                    Track("loadFront", (n[2] + n[3] - r.front) / W); Track("loadRear", (n[0] + n[1] - r.rear) / W);
                    Assert.AreEqual(r.front, n[2] + n[3], 1e-8 * W); Assert.AreEqual(r.rear, n[0] + n[1], 1e-8 * W);
                }
                else
                {
                    Track("loadLeft", (n[0] + n[2] - r.left) / W); Track("loadRight", (n[1] + n[3] - r.right) / W);
                    Assert.AreEqual(r.left, n[0] + n[2], 1e-8 * W); Assert.AreEqual(r.right, n[1] + n[3], 1e-8 * W);
                }
            }
        }

        [Test]
        public void CrossSlopeHold_LimitAndCreep()
        {
            var geo = Geo(refs.geometry.cgFromRear, 0);
            double a = Bisect(th => !double.IsNaN(Creep(geo, Params(refs.mu, 0), th, Math.PI / 2, out _)), 0.0, 1.45);
            Track("crossHoldLimit", a - refs.crossSlopeHold.limit);
            Assert.AreEqual(refs.crossSlopeHold.limit, a, 1e-8);
            foreach (var c in refs.crossSlopeHold.creep)
            {
                double s = Creep(geo, Params(refs.mu, 0), c.slope, Math.PI / 2, out _);
                Track("crossCreep", (s - c.vy) / c.vy);
                Assert.AreEqual(c.vy, s, c.vy * 1e-6, "slope=" + c.slope);
            }
        }

        [Test]
        public void ZeroCommand_HoldsBelowLimit_CreepsAsPredicted_SlidesAbove()
        {
            var geo = Geo(refs.geometry.cgFromRear, 0);
            foreach (var c in refs.zeroCommand.creep)
            {
                double s = Creep(geo, Params(refs.mu, c.crr), c.slopeDeg * Math.PI / 180, 0, out _);
                Track("zeroCreep", (s - c.creep) / c.creep);
                Assert.AreEqual(c.creep, s, c.creep * 1e-6, "slope=" + c.slopeDeg + " crr=" + c.crr);
                Assert.Less(s, 0.01, "creep should be sub-cm/s well below the hold limit");
            }
            double lim = refs.zeroCommand.holdLimit;
            Assert.IsFalse(double.IsNaN(Creep(geo, Params(refs.mu, 0), lim - 0.01, 0, out _)));
            Assert.IsTrue(double.IsNaN(Creep(geo, Params(refs.mu, 0), lim + 0.01, 0, out _)));
        }

        [Test]
        public void YawRotationEquivariance()
        {
            var y = refs.yawEquivariance;
            var geo = Geo(refs.geometry.cgFromRear, 0);
            foreach (int deg in y.headingsDeg)
            {
                double s = Creep(geo, Params(refs.mu, 0), y.slopeDeg * Math.PI / 180, deg * Math.PI / 180, out double need);
                Track("yawCreep", (s - y.creep) / y.creep);
                Assert.AreEqual(y.creep, s, y.creep * 1e-6, "heading=" + deg);
                Assert.AreEqual(y.planeForce, need, 1e-9 * y.planeForce);
            }
        }

        [Test]
        public void RollingResistance_AppliedUnderThrottle_OnSlope()
        {
            var geo = Geo(refs.geometry.cgFromRear, 0);
            foreach (var r in refs.rollingResistance)
            {
                double th = r.slopeDeg * Math.PI / 180;
                var model = new SlopeTractionModel(geo, Params(refs.mu, 0.02));
                double v = 1.0;   // rolling at command speed: zero slip, so only rolling resistance acts
                var speeds = new[] { v, v, v, v };
                var f = model.Step(SlopeTractionMath.GravityInBody(th, 0), new BodyMotion { vx = v }, speeds, 0.01);
                Track("rolling", -f.totalFx - r.force);
                Assert.AreEqual(r.force, -f.totalFx, 1e-9, "slope=" + r.slopeDeg);
            }
        }

        [Test]
        public void FrictionCone_NeverExceeded()
        {
            var p = Params(0.6, 0.0);
            double worst = 0;
            for (double n = 10; n <= 400; n += 90)
                for (double vx = -2; vx <= 2; vx += 0.37)
                    for (double vy = -2; vy <= 2; vy += 0.41)
                        for (double ws = -3; ws <= 3; ws += 0.7)
                        {
                            var w = SlopeTractionMath.DrivenWheel(n, vx, vy, ws, p);
                            double mag = Math.Sqrt(w.fx * w.fx + w.fy * w.fy);
                            worst = Math.Max(worst, mag / (p.mu * n));
                            Assert.LessOrEqual(mag, p.mu * n * (1 + 1e-12));
                        }
            Track("coneRatioMinus1", worst - 1.0);
            Assert.Greater(worst, 0.999, "saturation should be reached somewhere in the sweep");
        }

        [Test]
        public void SlipCurve_IsMonotoneThenSaturates()
        {
            var p = Params(0.7, 0.0);
            double prev = 0;
            for (double kappa = 0.0; kappa <= 1.0; kappa += 0.02)
            {
                var w = SlopeTractionMath.DrivenWheel(100, 1.0, 0, 1.0 * (1 + kappa), p);
                Assert.GreaterOrEqual(w.fx, prev - 1e-12);
                prev = w.fx;
            }
            Assert.AreEqual(0.7 * 100, prev, 1e-9);
        }

        [Test]
        public void CasterFriction_OpposesMotionBoundedByMuN()
        {
            var p = Params(0.7, 0.0);
            SlopeTractionMath.CasterForce(80, 1.0, 0.0, p, out double fx, out double fy);
            Assert.AreEqual(-p.casterFriction * 80, fx, 1e-12); Assert.AreEqual(0, fy, 1e-12);
            SlopeTractionMath.CasterForce(80, 0.0, 0.0, p, out fx, out fy);
            Assert.AreEqual(0, fx); Assert.AreEqual(0, fy);
        }

        [Test]
        public void TorqueCap_LimitsTractiveForce()
        {
            var geo = Geo(refs.geometry.cgFromRear, 0);
            var model = new SlopeTractionModel(geo, Params(1.0, 0.0, 5.0));
            var f = model.Step(SlopeTractionMath.GravityInBody(0, 0), new BodyMotion(), new[] { 5.0, 5.0, 5.0, 5.0 }, 0.01);
            double cap = 5.0 / geo.wheelRadius;
            for (int i = 0; i < 4; i++) Assert.LessOrEqual(Math.Abs(f.fx[i]), cap + 1e-12);
            Assert.AreEqual(cap, f.fx[0], 1e-9);
        }

        [Test]
        public void Reset_RestoresFreshBehavior()
        {
            var geo = Geo(refs.geometry.cgFromRear, 0);
            var p = Params(0.7, 0.02); p.loadFilterSeconds = 0.5;
            var zero = new double[4];
            var fresh = new SlopeTractionModel(geo, p);
            var a = fresh.Step(SlopeTractionMath.GravityInBody(0.2, 0), new BodyMotion(), zero, 0.02);
            var used = new SlopeTractionModel(geo, p);
            for (int i = 0; i < 20; i++) used.Step(SlopeTractionMath.GravityInBody(0.5, 1.0), new BodyMotion { vx = 0.3 }, zero, 0.02);
            used.Reset();
            var b = used.Step(SlopeTractionMath.GravityInBody(0.2, 0), new BodyMotion(), zero, 0.02);
            for (int i = 0; i < 4; i++) { Assert.AreEqual(a.loads[i], b.loads[i], 1e-12); Assert.AreEqual(a.fx[i], b.fx[i], 1e-12); }
        }

        [Test]
        public void Adapter_IsInertWithoutFlag_AndActivatesWhenForced()
        {
            Assert.IsFalse(CraneSlopeAwareTraction.FlagRequested());
            var go = new GameObject("traction-flag-test");
            try
            {
                go.AddComponent<Rigidbody>();
                var c = go.AddComponent<CraneSlopeAwareTraction>();
                var awake = typeof(CraneSlopeAwareTraction).GetMethod("Awake", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic);
                awake.Invoke(c, null);
                Assert.IsFalse(c.Active); Assert.IsFalse(c.enabled);
                CraneSlopeAwareTraction.ForceEnabledForTests = true;
                var go2 = new GameObject("traction-flag-test-2");
                try
                {
                    go2.AddComponent<Rigidbody>();
                    var c2 = go2.AddComponent<CraneSlopeAwareTraction>();
                    awake.Invoke(c2, null);
                    Assert.IsTrue(c2.Active);
                    c2.ResetState();
                }
                finally { CraneSlopeAwareTraction.ForceEnabledForTests = false; UnityEngine.Object.DestroyImmediate(go2); }
            }
            finally { UnityEngine.Object.DestroyImmediate(go); }
        }
    }
}
