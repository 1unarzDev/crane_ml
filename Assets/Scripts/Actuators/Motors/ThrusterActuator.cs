using Sim.Physics.Core;
using UnityEngine;

namespace Sim.Actuators.Motors {
    /// <summary>
    /// A <see cref="Thruster"/> as a plant <see cref="IActuator"/>: the thruster keeps its command
    /// (from the PWM or velocity input) and rotor visuals; this adapter turns the command into the
    /// ESC pulse, steps the <see cref="ThrusterModel"/> and adds τ_i = [d_i T_i; r_i × d_i T_i] to
    /// the Hydro wrench, with β = submerged fraction and r_i from the CG (spec mount minus the CG
    /// offset). The plant applies it on the hull, never on the rotor or transform.root.
    /// </summary>
    public sealed class ThrusterActuator : IActuator {
        private readonly Thruster thruster;
        public readonly ThrusterModel Model;
        /// <summary>CG offset from the nominal CG, body FLU [m] (mass-layout errors).</summary>
        public Vector3 CenterOfMassOffset;
        private int loggedTicks = -1;
        private int reportedNonFinite;
        private float pulse = ThrusterModel.Neutral;

        public ThrusterActuator(Thruster thruster, ThrusterSpec spec) {
            this.thruster = thruster;
            Model = new ThrusterModel(spec);
        }

        public string Name => Model.Spec.name;
        public ActuatorReport Report { get; private set; }

        /// <summary>Overrides the thruster's command with a pulse [µs] (tests, direct PWM).</summary>
        public void SetCommand(float pulseMicroseconds) {
            thruster.SetPulse(pulseMicroseconds);
        }

        public void Accumulate(in PlantState s, ref Wrench w) {
            pulse = thruster.CommandPulse;
            float thrust = Model.Step(pulse, s.Dt);
            if (Model.NonFinitePulses != reportedNonFinite) {
                reportedNonFinite = Model.NonFinitePulses;
                PlantEvents.Report("NONFINITE_PULSE", $"thruster={Name} neutral applied");
            }
            if (Model.DelayTicks != loggedTicks) {
                loggedTicks = Model.DelayTicks;
                Debug.Log($"CRANE_THRUSTER_DELAY thruster={Name} ticks={loggedTicks} " +
                          $"rounding_error_s={Model.DelayRoundingError:R}");
            }
            float beta = thruster.SubmersionFraction(Model.Spec.submersionDepth);
            Vector2 d = Model.Spec.Direction;
            var force = new Vector3(d.x, d.y, 0f) * (beta * thrust);
            var arm = new Vector3(Model.Spec.x, Model.Spec.y, Model.Spec.z) - CenterOfMassOffset;
            w.AddAt(force, arm);
            Report = new ActuatorReport { Commanded = pulse, Applied = beta * thrust, Saturated = Model.Saturated };
        }

        public void ResetState() => Model.Reset();
    }
}
