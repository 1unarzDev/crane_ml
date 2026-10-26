using System;
using NUnit.Framework;
using Sim.Actuators.Motors;

namespace Sim.Tests.Physics {
    public sealed class ThrusterModelTests {
        private static ThrusterModel Model(Action<ThrusterSpec> edit = null) {
            var spec = new ThrusterSpec { name = "t", kForward = 51.5f, kReverse = 51.5f };
            edit?.Invoke(spec);
            return new ThrusterModel(spec);
        }

        [Test]
        public void DeadbandIsNeutralWithoutRescale() {
            ThrusterModel m = Model(s => { s.deadbandUs = 25; s.offsetUs = 10; });
            Assert.That(m.Step(1535f, 0.02f), Is.EqualTo(0f), "|1535 − 1500 − 10| = 25 is inside");
            Assert.That(m.Step(1485f, 0.02f), Is.EqualTo(0f), "|1485 − 1500 − 10| = 25 is inside");
            float q = (1600f - 1510f) / 400f;
            Assert.That(m.Step(1600f, 0.02f), Is.EqualTo(51.5f * q * q).Within(1e-4f), "no range rescale");
        }

        [Test]
        public void LagReaches63PercentAtTau() {
            ThrusterModel m = Model(s => s.tauT = 0.2f);
            const float dt = 0.01f;
            for (int k = 0; k < 20; k++) m.Step(1900f, dt);
            Assert.That(m.Thrust / 51.5f, Is.EqualTo(1 - Math.Exp(-1)).Within(1e-4));
        }

        [Test]
        public void DeadTimeRoundsToTicksAndIsApplied() {
            ThrusterModel m = Model(s => s.deadTime = 0.045f);
            Assert.That(m.Step(1900f, 0.02f), Is.EqualTo(0f));
            Assert.That(m.DelayTicks, Is.EqualTo(2), "round(0.045/0.02) = round(2.25) = 2");
            Assert.That(m.DelayRoundingError, Is.EqualTo(0.005f).Within(1e-6f));
            Assert.That(m.Step(1900f, 0.02f), Is.EqualTo(0f));
            Assert.That(m.Step(1900f, 0.02f), Is.EqualTo(51.5f), "the first pulse arrives exactly 2 ticks late");
        }

        [Test]
        public void ReverseUsesItsOwnGain() {
            ThrusterModel m = Model(s => s.kReverse = 40.2f);
            Assert.That(m.Step(1100f, 0.02f) / Model().Step(1900f, 0.02f), Is.EqualTo(-40.2f / 51.5f).Within(1e-6f));
        }

        [Test]
        public void PulsesClampTo1100And1900() {
            Assert.That(Model().Step(2300f, 0.02f), Is.EqualTo(Model().Step(1900f, 0.02f)));
            Assert.That(Model().Step(700f, 0.02f), Is.EqualTo(Model().Step(1100f, 0.02f)));
            ThrusterModel m = Model();
            m.Step(1950f, 0.02f);
            Assert.That(m.Saturated, Is.True);
        }

        [Test]
        public void NonFinitePulseIsNeutral() {
            ThrusterModel m = Model();
            Assert.That(m.Step(float.NaN, 0.02f), Is.EqualTo(0f));
            Assert.That(m.Step(float.PositiveInfinity, 0.02f), Is.EqualTo(0f));
            Assert.That(m.NonFinitePulses, Is.EqualTo(2));
        }

        [Test]
        public void ResetClearsLagAndDelay() {
            ThrusterModel m = Model(s => { s.tauT = 0.5f; s.deadTime = 0.04f; });
            for (int k = 0; k < 50; k++) m.Step(1900f, 0.02f);
            Assert.That(m.Thrust, Is.GreaterThan(1f));
            m.Reset();
            Assert.That(m.Thrust, Is.EqualTo(0f));
            Assert.That(m.Step(1500f, 0.02f), Is.EqualTo(0f), "no stale effort left in the delay buffer");
            Assert.That(m.Step(1500f, 0.02f), Is.EqualTo(0f));
        }

        [Test]
        public void KappaIsRefusedUntilMeasured() {
            Assert.Throws<NotSupportedException>(() => Model(s => s.kappa = 0.1f));
        }
    }
}
