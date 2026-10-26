using System;
using Sim.Physics.Core;
using UnityEngine;

namespace Sim.Physics.Water.Dynamics {
    /// <summary>
    /// Marine surface hydrodynamics as one force model (routed Hydro). Planar: M_A ν̇_c −
    /// C_A(ν_r)ν_r − D(ν_r) on (u, v, r) from <see cref="MarinePlanarModel"/>, with its Jacobian for
    /// the implicit step. Vertical: body-frame damping −(D1 + D2|s|)s on heave, roll and pitch (no
    /// added mass there; <see cref="AddedMassCoupling"/> passes those DOFs through), so roll and
    /// pitch damp about the hull's own axes. Replaces FossenDynamics.
    /// </summary>
    [Serializable]
    public sealed class MarineHydrodynamics : IForceModel, IDampingModel {
        public readonly MarinePlanarModel Planar;
        public AxisDamping Heave, Roll, Pitch;
        /// <summary>Validity envelope (surge, sway [m/s], yaw [rad/s]); twice it logs OUT_OF_ENVELOPE.</summary>
        public Vector3 Envelope;

        public MarineHydrodynamics(MarinePlanarModel planar, AxisDamping heave, AxisDamping roll,
            AxisDamping pitch, Vector3 envelope) {
            Planar = planar;
            Heave = heave;
            Roll = roll;
            Pitch = pitch;
            Envelope = envelope;
        }

        public WrenchRoute Route => WrenchRoute.Hydro;

        public static Planar Nu(in PlantState s) => new(s.Linear.x, s.Linear.y, s.Angular.z);
        public static Planar Medium(in PlantState s) => new(s.MediumLinear.x, s.MediumLinear.y, 0);

        public void Accumulate(in PlantState s, ref Wrench w) {
            Planar nu = Nu(s);
            Planar tau = Planar.Hydrodynamic(nu, Medium(s));
            w.Force += new Vector3((float)tau.X, (float)tau.Y, -(float)Heave.Force(s.Linear.z));
            w.Torque += new Vector3(-(float)Roll.Force(s.Angular.x), -(float)Pitch.Force(s.Angular.y), (float)tau.N);
            Planar vr = nu - Medium(s);
            for (int j = 0; j < 3; j++) {
                double limit = j == 0 ? Envelope.x : j == 1 ? Envelope.y : Envelope.z;
                if (limit > 0 && Math.Abs(vr[j]) > 2 * limit)
                    PlantEvents.Report("OUT_OF_ENVELOPE", $"axis={j} value={vr[j]:F3} envelope={limit:F3}");
            }
        }

        public void AccumulateJacobian(in PlantState s, ref Wrench diagonal) {
            Planar j = Planar.Jacobian(Nu(s) - Medium(s));
            diagonal.Force.x += (float)j.X;
            diagonal.Force.y += (float)j.Y;
            diagonal.Torque.z += (float)j.N;
        }
    }

}
