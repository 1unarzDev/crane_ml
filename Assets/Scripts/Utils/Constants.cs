namespace Sim.Utils {
    public static class Constants {
        /// <summary>Standard gravity [m/s²].</summary>
        public const float gravity = 9.80665f;
        /// <summary>Fresh water at about 25 °C [kg/m³].</summary>
        public const float waterDensity = 997;
        /// <summary>Numerically water's dynamic viscosity in mPa·s (1.0016e-3 Pa·s at 20 °C).
        /// SubmersionLib's Reynolds number divides by it as if it were the kinematic viscosity
        /// [m²/s] (water: about 1.0e-6), so Re there is about 1e6 too small; only GeneralDynamics'
        /// viscous resistance uses it, and the boat plant does not.</summary>
        public const float waterViscosity = 1.0016f;
    }
}
