namespace Sim.Physics.Core {
    /// <summary>A force on the plant body (drag, buoyancy, wind, ...). Config is a [Serializable]
    /// class with Tooltip/Range fields; models never re-add rigid-body terms Unity integrates
    /// (M_RB, C_RB, gravity, contacts).</summary>
    public interface IForceModel {
        WrenchRoute Route { get; }
        void Accumulate(in PlantState state, ref Wrench wrench);
        /// <summary>Clears any per-episode state (filters, buffers).</summary>
        void ResetState() { }
    }

    /// <summary>A damping model that also gives the diagonal of its Jacobian, ∂(−τ_D)/∂ν ≥ 0
    /// per DOF, so the coupling can step it implicitly.</summary>
    public interface IDampingModel {
        void AccumulateJacobian(in PlantState state, ref Wrench diagonal);
    }
}
