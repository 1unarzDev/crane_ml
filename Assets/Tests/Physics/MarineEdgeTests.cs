using System;
using System.Linq;
using NUnit.Framework;
using Sim.Physics.Core;
using Sim.Physics.Water.Boat;
using Sim.Physics.Water.Dynamics;
using UnityEngine;

namespace Sim.Tests.Physics {
    public sealed class MarineEdgeTests {
        [SetUp]
        public void ClearEvents() => PlantEvents.Clear();

        [Test]
        public void ValidationRejectsUnusableNumbers() {
            Action<MarineSurfaceParameters>[] bad = {
                p => p.mass = 0, p => p.izz = -1, p => p.surge.d1Pos = -1, p => p.yaw.d2Neg = -0.1f,
                p => p.mass = float.NaN, p => p.addedMass.a22 = float.PositiveInfinity,
                p => p.thrusters[0].kForward = -1, p => p.thrusters[1].name = p.thrusters[0].name,
                p => p.addedMass.a26 = 1000, p => p.thrusters[2].kappa = 0.2f
            };
            foreach (var edit in bad) {
                MarineSurfaceParameters p = Kit.Ideal();
                edit(p);
                Assert.That(p.Validate(0.02f).Ok, Is.False, p.Validate(0.02f).ToString());
            }
            Assert.That(Kit.Ideal().Validate(0.02f).Ok, Is.True);
        }

        [Test]
        public void ValidationWarnsPastTheStepAccuracyLimit() {
            Assert.That(Kit.Ideal().Validate(0.02f).Warnings, Is.Empty, "Ideal is accurate at the default step");
            Assert.That(Kit.Ideal().Validate(0.04f).Warnings.Any(w => w.Contains("axis 2")), Is.True,
                "yaw at 0.04 s: dt·J(1.5 r_max)/M_eff ≈ 0.9");
            MarineSurfaceParameters free = Kit.Ideal();
            free.sway.d1Pos = free.sway.d1Neg = free.sway.d2Pos = free.sway.d2Neg = 0;
            Assert.That(free.Validate(0.02f).Warnings.Any(w => w.Contains("no damping")), Is.True);
        }

        [Test]
        public void LargeAddedMassStaysStableAndPassive([ValueSource(typeof(Kit), nameof(Kit.Dts))] double dt) {
            MarinePlanarModel m = Kit.Model();
            m.A11 = 10 * m.Mass; m.A22 = 10 * m.Mass; m.A66 = 10 * m.Izz; m.A26 = 0.5 * m.Izz;
            var s = new MarinePlanarModel.State { Vx = 3, Vy = -2, R = 2 };
            double e0 = m.Energy(s.Body);
            s = Kit.Run(m, s, _ => default, dt, 30);
            Assert.That(s.IsFinite && m.Energy(s.Body) <= e0, Is.True);
        }

        [Test]
        public void SignFlipEveryTickStaysBoundedWithNoEnergyGain([ValueSource(typeof(Kit), nameof(Kit.Dts))] double dt) {
            MarineSurfaceParameters p = Kit.Ideal();
            foreach (var t in p.thrusters) t.tauT = 0.1f;
            MarinePlanarModel m = p.PlanarModel();
            var thrusters = Kit.Thrusters(p);
            int k = 0;
            var s = Kit.Run(m, default, _ => Kit.Thrust(thrusters, Kit.Pulses(k++ % 2 == 0 ? 1 : -1, 0, 0), (float)dt), dt, 20);
            double vmax = MarinePlanarModel.SteadySpeed(10, 19.4, Kit.FullAxisThrust);
            Assert.That(s.IsFinite, Is.True);
            Assert.That(Math.Abs(s.Body.X), Is.LessThan(0.2 * vmax), "alternating full thrust averages out");
        }

        [Test]
        public void FullReverseMirrorsForwardWithEqualGains() {
            MarineSurfaceParameters p = Kit.Ideal();
            MarinePlanarModel m = p.PlanarModel();
            var fwd = Kit.Thrusters(p);
            var rev = Kit.Thrusters(p);
            double f = Kit.Run(m, default, _ => Kit.Thrust(fwd, Kit.Pulses(1, 0, 0), 0.02f), 0.02, 40).Body.X;
            double r = Kit.Run(m, default, _ => Kit.Thrust(rev, Kit.Pulses(-1, 0, 0), 0.02f), 0.02, 40).Body.X;
            Assert.That(f, Is.EqualTo(2.5).Within(0.05));
            Assert.That(r, Is.EqualTo(-f).Within(1e-9));
        }

        [Test]
        public void MirroredCommandsGiveMirroredTrajectories() {
            MarineSurfaceParameters p = Kit.Ideal();
            MarinePlanarModel m = p.PlanarModel();
            var a = Kit.Thrusters(p);
            var b = Kit.Thrusters(p);
            var left = Kit.Run(m, default, _ => Kit.Thrust(a, Kit.Pulses(0.6f, 0.3f, 0.2f), 0.02f), 0.02, 15);
            var right = Kit.Run(m, default, _ => Kit.Thrust(b, Kit.Pulses(0.6f, -0.3f, -0.2f), 0.02f), 0.02, 15);
            Assert.That(right.X, Is.EqualTo(left.X).Within(1e-9));
            Assert.That(right.Y, Is.EqualTo(-left.Y).Within(1e-9));
            Assert.That(right.Psi, Is.EqualTo(-left.Psi).Within(1e-9));
        }

        [Test]
        public void OneDeadThrusterTurnsTheBoatAndSaturationIsClamped() {
            MarineSurfaceParameters p = Kit.Ideal();
            p.thrusters[0].kForward = p.thrusters[0].kReverse = 0;
            var dead = Kit.Thrusters(p);
            Planar w = Kit.Thrust(dead, Kit.Pulses(1, 0, 0), 0.02f);
            Assert.That(Math.Abs(w.N), Is.GreaterThan(1), "a dead thruster leaves a yaw moment");
            var all = Kit.Thrusters(Kit.Ideal());
            Planar saturated = Kit.Thrust(all, new[] { 2500f, 2500f, 2500f, 2500f }, 0.02f);
            Planar full = Kit.Thrust(Kit.Thrusters(Kit.Ideal()), new[] { 1900f, 1900f, 1900f, 1900f }, 0.02f);
            Assert.That(saturated.X, Is.EqualTo(full.X).Within(1e-6));
            Assert.That(all.All(t => t.Saturated), Is.True);
        }

        [Test]
        public void OutOfEnvelopeIsReported() {
            var hydro = new MarineHydrodynamics(Kit.Model(), default, default, default, new Vector3(2.5f, 1.7f, 1.9f));
            var frame = new BodyFrame(new GameObject("frame").transform, Quaternion.identity);
            var state = new PlantState(frame, Vector3.zero, new Vector3(5.2f, 0, 0), Vector3.zero, Vector3.zero, 0.02f, 0);
            Wrench w = default;
            hydro.Accumulate(state, ref w);
            Assert.That(PlantEvents.Count("OUT_OF_ENVELOPE"), Is.EqualTo(1));
            UnityEngine.Object.DestroyImmediate(GameObject.Find("frame"));
        }

        [Test]
        public void NonFiniteStateAppliesNoWrench() {
            var go = new GameObject("plant");
            try {
                var root = go.AddComponent<ArticulationBody>();
                var plant = go.AddComponent<TestPlant>();
                plant.Setup(root, new BodyFrame(go.transform, Quaternion.identity),
                    new FlowMedium { Flow = new Vector3(float.NaN, 0, 0) }, new NoCoupling(),
                    new ConstantForce { Value = new Wrench(Vector3.one, Vector3.one) });
                Assert.That(plant.Step(0.02f, 0), Is.False);
                Assert.That(plant.LastWrench.Force, Is.EqualTo(Vector3.zero));
                Assert.That(PlantEvents.Count("NONFINITE_STATE"), Is.EqualTo(1));
            }
            finally {
                UnityEngine.Object.DestroyImmediate(go);
            }
        }
    }
}
