using System;
using NUnit.Framework;
using Sim.Physics.Water.Boat;
using Sim.Physics.Water.Dynamics;

namespace Sim.Tests.Physics {
    /// <summary>Design §7 analytic checks on the model the plant and Unity step together
    /// (MarinePlanarModel.Step), against closed forms and RK4 at 1 ms.</summary>
    public sealed class MarineAnalyticTests {
        private static void Report(string what, double dt, double error) =>
            UnityEngine.Debug.Log($"PHYS_ANALYTIC {what} dt={dt} rel_error={error:E3}");

        /// <summary>
        /// Transient tolerance. With the exponential step factor (MarinePlanarModel.StepTerm) the
        /// linearized decay is exact, so design §7's 2 % holds up to the 0.5 accuracy limit of
        /// dt·J/M_eff. Past it only the first-order bound dt·J/M_eff is asserted (both logged).
        /// </summary>
        private static double TransientTolerance(MarinePlanarModel m, int axis, double speed, double dt) {
            double bound = dt * m.Axis(axis).Jacobian(Math.Abs(speed)) / m.EffectiveMass(axis);
            UnityEngine.Debug.Log($"PHYS_BOUND axis={axis} dt={dt} dtJ_over_Meff={bound:F4}");
            return bound <= 0.5 ? 0.02 : Math.Max(0.02, bound);
        }

        [Test]
        public void ImplicitDampingFormIsPinned() {
            // Stiff surge: dt·J/M_eff = 0.04·1000/15 = 2.7, where explicit damping overshoots
            // zero every tick and grows. The step factor gives the exact linear decay
            // u1 = u0·e^{−dt·D1/M_eff} at any dt.
            var m = new MarinePlanarModel { Mass = 10, Izz = 1, A11 = 5, Surge = AxisDamping.Symmetric(1000, 0),
                Sway = AxisDamping.Symmetric(1, 0), Yaw = AxisDamping.Symmetric(1, 0) };
            const double dt = 0.04;
            var s = new MarinePlanarModel.State { Vx = 1 };
            m.Step(ref s, default, 0, 0, dt);
            Assert.That(s.Vx, Is.EqualTo(Math.Exp(-dt * 1000 / 15.0)).Within(1e-12));
            // F_U = τ_h − (M_A + c·J)·M̃⁻¹(τ_h − C_RB ν): the c·J term must sit next to M_A.
            var nu = new Planar(1, 0, 0);
            Planar tauH = m.Hydrodynamic(nu, default);
            Planar f = m.UnityForce(nu, tauH, m.Jacobian(nu), dt);
            double meff = 15, cj = MarinePlanarModel.StepTerm(meff, 1000, dt);
            Assert.That(f.X, Is.EqualTo(tauH.X - (5 + cj) * tauH.X / (meff + cj)).Within(1e-9));
            // c·J is the review's h = (M_eff/J)(e^x − 1) rescaled from step h to Unity's dt.
            double x = dt * 1000 / meff, h = meff / 1000 * (Math.Exp(x) - 1);
            Assert.That(meff + cj, Is.EqualTo(dt / h * (meff + h * 1000)).Within(1e-9));
            Assert.That(MarinePlanarModel.StepTerm(meff, 1e-3, dt) / (dt * 1e-3), Is.EqualTo(0.5).Within(1e-6),
                "small dt·J/M_eff: half the plain dt·J");
            // Yaw at the Ideal profile: one tick of linear decay is exact at every dt.
            var ideal = Kit.Model();
            foreach (double step in Kit.Dts) {
                var y = new MarinePlanarModel.State { R = 1e-9 };
                ideal.Step(ref y, default, 0, 0, step);
                double yawMeff = ideal.EffectiveMass(2);
                Assert.That(y.R / 1e-9, Is.EqualTo(Math.Exp(-step * ideal.Yaw.D1Pos / yawMeff)).Within(1e-6), $"dt={step}");
            }
            for (int k = 0; k < 200; k++) {
                double before = s.Vx;
                m.Step(ref s, default, 0, 0, dt);
                Assert.That(s.Vx, Is.GreaterThanOrEqualTo(0).And.LessThan(before), "monotone decay, no overshoot");
            }
        }

        [Test]
        public void SteadySpeedsMatchTheClosedForm([ValueSource(typeof(Kit), nameof(Kit.Dts))] double dt) {
            MarinePlanarModel m = Kit.Model();
            double[] force = { Kit.FullAxisThrust, Kit.FullAxisThrust, Kit.FullYawMoment };
            for (int j = 0; j < 3; j++) {
                AxisDamping d = m.Axis(j);
                double expected = MarinePlanarModel.SteadySpeed(d.D1Pos, d.D2Pos, force[j]);
                var thrust = new Planar(j == 0 ? force[0] : 0, j == 1 ? force[1] : 0, j == 2 ? force[2] : 0);
                double got = Kit.Run(m, default, _ => thrust, dt, 60).Body[j];
                double error = Math.Abs(got / expected - 1);
                Report($"steady axis={j}", dt, error);
                Assert.That(error, Is.LessThan(0.01), $"axis {j}: {got} vs {expected}");
            }
            Assert.That(MarinePlanarModel.SteadySpeed(10, 19.4, Kit.FullAxisThrust), Is.EqualTo(2.5).Within(0.05),
                "four T200 reach the boat's measured 2.5 m/s");
            Assert.That(MarinePlanarModel.SteadySpeed(15, 41.7, Kit.FullAxisThrust), Is.LessThan(2.0),
                "a catamaran is draggier abeam");
        }

        [Test]
        public void DecayMatchesClosedFormsAndRk4([ValueSource(typeof(Kit), nameof(Kit.Dts))] double dt) {
            for (int j = 0; j < 3; j += 2) {
                MarinePlanarModel linear = Kit.Model(), quadratic = Kit.Model();
                double d1 = Kit.Model().Axis(j).D1Pos, d2 = Kit.Model().Axis(j).D2Pos;
                if (j == 0) { linear.Surge = AxisDamping.Symmetric(d1, 0); quadratic.Surge = AxisDamping.Symmetric(0, d2); }
                else { linear.Yaw = AxisDamping.Symmetric(d1, 0); quadratic.Yaw = AxisDamping.Symmetric(0, d2); }
                double meff = linear.EffectiveMass(j), v0 = 1.5;
                var start = new MarinePlanarModel.State { Vx = j == 0 ? v0 : 0, R = j == 2 ? v0 : 0 };
                double tl = meff / d1, tq = meff / (d2 * v0);
                double gotL = Kit.Run(linear, start, _ => default, dt, tl).Body[j];
                double refL = Kit.Run(linear, start, _ => default, Kit.ReferenceDt, tl, rk4: true).Body[j];
                double gotQ = Kit.Run(quadratic, start, _ => default, dt, tq).Body[j];
                double refQ = Kit.Run(quadratic, start, _ => default, Kit.ReferenceDt, tq, rk4: true).Body[j];
                Assert.That(refL, Is.EqualTo(v0 * Math.Exp(-1)).Within(1e-3 * v0), "RK4 vs v0·e^(−D1 t/M)");
                Assert.That(refQ, Is.EqualTo(v0 / 2).Within(1e-3 * v0), "RK4 vs v0/(1 + D2 v0 t/M)");
                // Kit.Run steps round(t/dt) ticks; judge the stepper against RK4 at that same time,
                // not at t (yaw tq is 4.6 ticks at dt 0.04, a 2-4 % sampling offset on its own).
                refL = Kit.Run(linear, start, _ => default, Kit.ReferenceDt, Math.Round(tl / dt) * dt, rk4: true).Body[j];
                refQ = Kit.Run(quadratic, start, _ => default, Kit.ReferenceDt, Math.Round(tq / dt) * dt, rk4: true).Body[j];
                double errL = Math.Abs(gotL / refL - 1), errQ = Math.Abs(gotQ / refQ - 1);
                Report($"decay_linear axis={j}", dt, errL);
                Report($"decay_quadratic axis={j}", dt, errQ);
                Assert.That(errL, Is.LessThan(TransientTolerance(linear, j, v0, dt)));
                Assert.That(errQ, Is.LessThan(TransientTolerance(quadratic, j, v0, dt)));
            }
        }

        [Test]
        public void StepRiseT95MatchesRk4([ValueSource(typeof(Kit), nameof(Kit.Dts))] double dt) {
            MarinePlanarModel m = Kit.Model();
            MarineSurfaceParameters p = Kit.Ideal();
            double[] force = { Kit.FullAxisThrust, Kit.FullAxisThrust, Kit.FullYawMoment };
            for (int j = 0; j < 3; j++) {
                var thrust = new Planar(j == 0 ? force[0] : 0, j == 1 ? force[1] : 0, j == 2 ? force[2] : 0);
                double target = MarinePlanarModel.SteadySpeed(m.Axis(j).D1Pos, m.Axis(j).D2Pos, force[j]);
                double got = Kit.RiseTime(m, thrust, j, target, 0.95, dt);
                double reference = Kit.RiseTime(m, thrust, j, target, 0.95, Kit.ReferenceDt, rk4: true);
                double error = Math.Abs(got / reference - 1);
                double ratio = p.StepRatio(j, (float)dt);
                Report($"t95 axis={j} step_ratio={ratio:F2}", dt, error);
                if (ratio <= 0.5)
                    Assert.That(error, Is.LessThan(TransientTolerance(m, j, target, dt)), $"axis {j}: t95 {got} vs RK4 {reference}");
                else Assert.That(double.IsFinite(got), $"axis {j}: past the accuracy limit it must stay finite");
            }
        }

        [Test]
        public void SmallStepRiseTimeIsMeffOverD1() {
            MarinePlanarModel m = Kit.Model();
            const double force = 0.01;
            double target = MarinePlanarModel.SteadySpeed(10, 19.4, force);
            double t63 = Kit.RiseTime(m, new Planar(force, 0, 0), 0, target, 1 - Math.Exp(-1), 0.005);
            Assert.That(t63, Is.EqualTo(m.EffectiveMass(0) / 10).Within(0.02 * m.EffectiveMass(0) / 10));
        }

        [Test]
        public void SteadyTurnMatchesRk4([ValueSource(typeof(Kit), nameof(Kit.Dts))] double dt) {
            MarinePlanarModel m = Kit.Model();
            var thrust = new Planar(100, 0, 20);
            Planar got = Kit.Run(m, default, _ => thrust, dt, 60).Body;
            Planar reference = Kit.Run(m, default, _ => thrust, Kit.ReferenceDt, 60, rk4: true).Body;
            double radius = got.X / got.N, refRadius = reference.X / reference.N;
            double errR = Math.Abs(radius / refRadius - 1), errV = Math.Abs(got.Y / reference.Y - 1);
            Report("turn_radius", dt, errR);
            Report("turn_sideslip", dt, errV);
            Assert.That(Math.Abs(reference.Y), Is.GreaterThan(1e-3), "a turn with sway added mass slips sideways");
            Assert.That(errR, Is.LessThan(0.02));
            Assert.That(errV, Is.LessThan(0.02));
        }

        [Test]
        public void AddedCoriolisDoesNoWork() {
            var rng = new Random(7);
            MarinePlanarModel m = Kit.Model();
            m.A26 = 0.7;
            for (int k = 0; k < 1000; k++) {
                var nu = new Planar(rng.NextDouble() * 6 - 3, rng.NextDouble() * 4 - 2, rng.NextDouble() * 4 - 2);
                Planar c = m.AddedCoriolis(nu);
                Assert.That(nu.X * c.X + nu.Y * c.Y + nu.N * c.N, Is.EqualTo(0).Within(1e-9));
            }
        }

        [Test]
        public void UnforcedEnergyNeverGrows([ValueSource(typeof(Kit), nameof(Kit.Dts))] double dt,
            [Values(0.0, 0.4)] double current) {
            MarinePlanarModel m = Kit.Model();
            m.A26 = 0.5;
            var s = new MarinePlanarModel.State { Vx = 2, Vy = 1, R = 1.5, Psi = 0.3 };
            double cx = current, cy = -current / 2;
            double Energy(MarinePlanarModel.State x) =>
                m.Energy(x.Body - MarinePlanarModel.MediumVelocity(x.Psi, cx, cy));
            double e0 = Energy(s), worst = 0;
            int n = (int)Math.Round(10 / dt);
            for (int k = 0; k < n; k++) {
                double before = Energy(s);
                m.Step(ref s, default, cx, cy, dt);
                worst = Math.Max(worst, Energy(s) - before);
            }
            Report($"energy_growth current={current}", dt, worst / e0);
            Assert.That(worst, Is.LessThanOrEqualTo(1e-6 * e0), "per-step growth");
            Assert.That(Energy(s), Is.LessThanOrEqualTo(1.001 * e0), "over 10 s");
        }

        [Test]
        public void FossenAddedCoriolisNumbers() {
            var m = new MarinePlanarModel { A11 = 6, A22 = 8 };
            Planar c = m.AddedCoriolis(new Planar(1, 0.2, 0.5));
            Assert.That(-c.X, Is.EqualTo(0.8).Within(1e-12));
            Assert.That(-c.Y, Is.EqualTo(-3.0).Within(1e-12));
            Assert.That(-c.N, Is.EqualTo(-0.4).Within(1e-12));
        }

        [Test]
        public void AddedMassAloneAcceleratesAtForceOverEffectiveMass() {
            MarinePlanarModel m = Kit.Model();
            m.Surge = m.Sway = m.Yaw = AxisDamping.Symmetric(0, 0);
            var s = new MarinePlanarModel.State();
            m.Step(ref s, new Planar(10, 0, 0), 0, 0, 0.02);
            Assert.That(s.Vx / 0.02, Is.EqualTo(10 / (m.Mass + m.A11)).Within(1e-12), "not today's 1.69×");
        }
    }
}
