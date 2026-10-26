namespace Sim.Physics.Core {
    /// <summary>What an actuator was asked for and what it delivered this tick.</summary>
    public struct ActuatorReport {
        /// <summary>Command in the actuator's input unit (a thruster: PWM µs).</summary>
        public float Commanded;
        /// <summary>Delivered output in SI (a thruster: N along its axis).</summary>
        public float Applied;
        public bool Saturated;
    }

    /// <summary>A commanded force source owned by a plant, addressed by name (never by index).
    /// Its wrench is routed <see cref="WrenchRoute.Hydro"/>.</summary>
    public interface IActuator {
        string Name { get; }
        void SetCommand(float command);
        void Accumulate(in PlantState state, ref Wrench wrench);
        ActuatorReport Report { get; }
        void ResetState();
    }
}
