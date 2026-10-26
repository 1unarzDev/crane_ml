using System;

namespace Sim.Actuators.Motors {
    /// <summary>
    /// A thruster from ESC pulse to thrust, pure math so tests and the Python twin pin it.
    /// Per tick: pulse clamp to [1100, 1900] µs (non-finite → neutral); effort
    /// q = clamp((p − 1500 − o)/400, ±1), 0 within ±db (no range rescale, as the ESC does); a dead
    /// time of round(t_d/dt) ticks; T_ss = K_f q² forward, −K_r q² reverse, times (V/V_ref)^n_V;
    /// lag T += (1 − e^(−dt/τ_T))(T_ss − T), the exact discretization.
    /// </summary>
    public sealed class ThrusterModel {
        public const float Neutral = 1500f, HalfRange = 400f;
        public readonly ThrusterSpec Spec;
        private float[] delay = Array.Empty<float>();
        private int head;
        private float dt = -1f, lagGain = 1f;

        public float Thrust { get; private set; }
        public float Effort { get; private set; }
        public bool Saturated { get; private set; }
        public int DelayTicks => delay.Length;
        /// <summary>t_d − n_d·dt [s], logged when the buffer is sized.</summary>
        public float DelayRoundingError { get; private set; }
        public int NonFinitePulses { get; private set; }

        public ThrusterModel(ThrusterSpec spec) {
            if (spec.kappa != 0) throw new NotSupportedException("kappa must be 0 until a bollard pull gives k");
            Spec = spec;
        }

        public static float EffortOf(float pulse, float offset, float deadband, out bool saturated) {
            float e = Math.Clamp(pulse, Neutral - HalfRange, Neutral + HalfRange) - Neutral - offset;
            saturated = Math.Abs(e) >= HalfRange;
            return Math.Abs(e) <= deadband ? 0f : Math.Clamp(e / HalfRange, -1f, 1f);
        }

        /// <summary>Thrust [N] along the mount direction after this tick.</summary>
        public float Step(float pulse, float dt, float voltage = float.NaN) {
            if (!float.IsFinite(pulse)) {
                NonFinitePulses++;
                pulse = Neutral;
            }
            if (dt != this.dt) Size(dt);
            float q = EffortOf(pulse, Spec.offsetUs, Spec.deadbandUs, out bool saturated);
            Saturated = saturated;
            if (delay.Length > 0) {
                (delay[head], q) = (q, delay[head]);
                head = (head + 1) % delay.Length;
            }
            Effort = q;
            float target = q > 0 ? Spec.kForward * q * q : q < 0 ? -Spec.kReverse * q * q : 0f;
            if (Spec.nV != 0 && float.IsFinite(voltage) && voltage > 0)
                target *= (float)Math.Pow(voltage / Spec.vRef, Spec.nV);
            Thrust += lagGain * (target - Thrust);
            return Thrust;
        }

        private void Size(float step) {
            dt = step;
            int ticks = Math.Max(0, (int)Math.Round((double)Spec.deadTime / step));
            DelayRoundingError = Spec.deadTime - ticks * step;
            delay = new float[ticks];
            head = 0;
            lagGain = Spec.tauT > 0 ? 1f - (float)Math.Exp(-step / Spec.tauT) : 1f;
        }

        public void Reset() {
            Array.Clear(delay, 0, delay.Length);
            head = 0;
            Thrust = 0;
            Effort = 0;
        }
    }
}
