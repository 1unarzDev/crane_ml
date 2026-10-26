using Sim.Physics.Core;
using UnityEngine;

namespace Sim.Physics.Water.Dynamics {
    /// <summary>
    /// Planar added-mass coupling: maps the Hydro wrench's (X, Y, N) to the force Unity must apply
    /// with <see cref="MarinePlanarModel.UnityForce"/> (implicit damping), so Unity's M_RB step
    /// realizes M = M_RB + M_A. Heave, roll and pitch pass through unchanged. Stateless.
    /// </summary>
    public sealed class AddedMassCoupling : IInertiaCoupling {
        public readonly MarinePlanarModel Planar;
        public AddedMassCoupling(MarinePlanarModel planar) => Planar = planar;

        public Wrench Map(in PlantState s, in Wrench hydro, in Wrench jacobian) {
            var tauH = new Planar(hydro.Force.x, hydro.Force.y, hydro.Torque.z);
            var j = new Planar(jacobian.Force.x, jacobian.Force.y, jacobian.Torque.z);
            Planar f = Planar.UnityForce(MarineHydrodynamics.Nu(s), tauH, j, s.Dt);
            return new Wrench(new Vector3((float)f.X, (float)f.Y, hydro.Force.z),
                new Vector3(hydro.Torque.x, hydro.Torque.y, (float)f.N));
        }
    }
}
