using System;

namespace Sim.Actuators.Motors {
    /// <summary>
    /// The ESC pulse model shared by every PWM input (the ArduPilot SITL bridge and the ROS
    /// thruster PWM input): a pulse width maps linearly onto a motor command range.
    /// </summary>
    public static class EscPwm {
        /// <summary>Boat ESC convention used by mhseals_hardware: 1500 us neutral, +/-400 us.</summary>
        public const float BoatMinimumPulse = 1100f;
        public const float BoatMaximumPulse = 1900f;

        /// <summary>
        /// Maps <paramref name="pulse"/>, clamped to [minimumPulse, maximumPulse], linearly onto
        /// [minimumCommand, maximumCommand].
        /// </summary>
        public static float Linear(float pulse, float minimumPulse, float maximumPulse,
            float minimumCommand, float maximumCommand) {
            if (!(maximumPulse > minimumPulse))
                throw new ArgumentException("PWM range must satisfy maximum > minimum.");
            pulse = Math.Clamp(pulse, minimumPulse, maximumPulse);
            return minimumCommand + (pulse - minimumPulse) *
                (maximumCommand - minimumCommand) / (maximumPulse - minimumPulse);
        }

        /// <summary>
        /// Bidirectional ESC: the range midpoint is zero thrust, the ends are +/-maximumCommand,
        /// and pulses within <paramref name="deadbandPulse"/> of the midpoint are neutral.
        /// </summary>
        public static float Bidirectional(float pulse, float maximumCommand,
            float minimumPulse = BoatMinimumPulse, float maximumPulse = BoatMaximumPulse,
            float deadbandPulse = 0f) {
            float neutral = 0.5f * (minimumPulse + maximumPulse);
            if (Math.Abs(pulse - neutral) <= Math.Max(0f, deadbandPulse)) return 0f;
            return Linear(pulse, minimumPulse, maximumPulse, -maximumCommand, maximumCommand);
        }
    }
}
