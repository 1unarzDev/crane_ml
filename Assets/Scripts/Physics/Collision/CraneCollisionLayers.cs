namespace Sim.Physics.Contacts {
    /// <summary>Stable physics-layer ownership used by validation scenes and future migrations.</summary>
    public static class CraneCollisionLayers {
        public const int Environment = 9;
        public const int Vehicle = 10;
        public const int DynamicObstacle = 11;
        public const int SensorQuery = 12;
        public const int SimulationTrigger = 13;
    }
}
