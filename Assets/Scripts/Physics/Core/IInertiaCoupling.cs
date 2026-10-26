namespace Sim.Physics.Core {
    /// <summary>
    /// Maps the summed Hydro wrench to the wrench Unity must apply so that Unity's rigid-body step
    /// (which knows only M_RB) realizes the dynamics with added mass, stepping damping implicitly
    /// with its Jacobian diagonal. Stateless: nothing persists across ticks or resets.
    /// </summary>
    public interface IInertiaCoupling {
        Wrench Map(in PlantState state, in Wrench hydro, in Wrench dampingJacobian);
    }

    /// <summary>The default coupling: the Hydro wrench goes to Unity unchanged.</summary>
    public sealed class NoCoupling : IInertiaCoupling {
        public Wrench Map(in PlantState state, in Wrench hydro, in Wrench dampingJacobian) => hydro;
    }
}
