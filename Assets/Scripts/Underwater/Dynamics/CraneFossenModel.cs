using System;
using System.Collections.Generic;
using static Sim.Underwater.CraneLinAlg;

namespace Sim.Underwater {
    /// <summary>
    /// Pure-math port of Tools/Underwater/fossen6dof.py (Fossen 2011). No UnityEngine dependency.
    /// Inertial frame NED, body frame FRD, nu = [u v w p q r], q = [w x y z] body-to-NED.
    /// Added mass / damping are stored as positive matrices; added-mass force is -M_A * nudot_r.
    /// </summary>
    public sealed class CraneVehicleParams {
        public const double StandardGravity = 9.80665;
        public double Mass;
        public double[,] InertiaCg = new double[3, 3];
        public double[] RG = new double[3];
        public double[] RB = new double[3];
        public double Buoyancy;
        public double[,] AddedMass = new double[6, 6];
        public double[,] LinearDamping = new double[6, 6];
        public double[] QuadraticDamping = new double[6];
        public double Gravity = StandardGravity;
        public string Name = "vehicle";

        public double Weight => Mass * Gravity;
        /// <summary>CG-below-CB distance (FRD z of CG minus z of CB).</summary>
        public double BG => RG[2] - RB[2];

        public void Validate() {
            if (!(Mass > 0.0)) throw new ArgumentException("mass must be positive");
            if (MaxAsym(InertiaCg) > 1e-9) throw new ArgumentException("inertia_cg must be symmetric");
            if (MinSymmetricEigenvalue(InertiaCg) <= 0.0) throw new ArgumentException("inertia_cg must be positive definite");
            if (MaxAsym(AddedMass) > 1e-12) throw new ArgumentException("added_mass must be symmetric");
            if (MinSymmetricEigenvalue(AddedMass) < -1e-12) throw new ArgumentException("added_mass must be positive semi-definite");
            var symD = Scale(Add(LinearDamping, Transpose(LinearDamping)), 0.5);
            if (MinSymmetricEigenvalue(symD) < -1e-12) throw new ArgumentException("linear_damping must be dissipative");
            foreach (double d in QuadraticDamping) if (d < 0.0) throw new ArgumentException("quadratic_damping must be non-negative");
            if (!(Buoyancy >= 0.0)) throw new ArgumentException("buoyancy must be non-negative");
        }

        static double MaxAsym(double[,] a) {
            double m = 0.0;
            for (int i = 0; i < a.GetLength(0); i++) for (int j = 0; j < a.GetLength(1); j++) m = Math.Max(m, Math.Abs(a[i, j] - a[j, i]));
            return m;
        }

        /// <summary>Rigid-body mass matrix about CO (Fossen eq. 3.44), parallel-axis shifted.</summary>
        public double[,] MRb() {
            var s = Skew(RG);
            var ss = Mul(s, s);
            var io = Add(InertiaCg, Scale(ss, -Mass)); // I_cg - m S S
            var m = new double[6, 6];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) {
                    m[i, j] = i == j ? Mass : 0.0;
                    m[i, 3 + j] = -Mass * s[i, j];
                    m[3 + i, j] = Mass * s[i, j];
                    m[3 + i, 3 + j] = io[i, j];
                }
            return m;
        }
    }

    public sealed class CraneState {
        public double[] Pos = new double[3];
        public double[] Q = { 1, 0, 0, 0 };
        public double[] Nu = new double[6];
        public double[] Thrust = new double[0];

        public CraneState Clone() => new CraneState {
            Pos = (double[])Pos.Clone(), Q = (double[])Q.Clone(), Nu = (double[])Nu.Clone(), Thrust = (double[])Thrust.Clone()
        };
    }

    public static class CraneFossen {
        /// <summary>Skew-symmetric C(nu) for a symmetric mass matrix (Fossen Thm 3.2).</summary>
        public static double[,] CoriolisFromMass(double[,] M, double[] nu) {
            var nu1 = Slice(nu, 0, 3); var nu2 = Slice(nu, 3, 3);
            var a = new double[3]; var b = new double[3];
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    a[i] += M[i, j] * nu1[j] + M[i, 3 + j] * nu2[j];
                    b[i] += M[3 + i, j] * nu1[j] + M[3 + i, 3 + j] * nu2[j];
                }
            }
            var sa = Skew(a); var sb = Skew(b);
            var c = new double[6, 6];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) {
                    c[i, 3 + j] = -sa[i, j];
                    c[3 + i, j] = -sa[i, j];
                    c[3 + i, 3 + j] = -sb[i, j];
                }
            return c;
        }

        /// <summary>g(eta): restoring generalised force on the LHS (Fossen eq. 4.6).</summary>
        public static double[] Restoring(CraneVehicleParams p, double[] q) {
            var rt = Transpose(QuatToRot(q));
            var fg = Mul(rt, new[] { 0.0, 0.0, p.Weight });
            var fb = Mul(rt, new[] { 0.0, 0.0, -p.Buoyancy });
            var f = Add(fg, fb);
            var m = Add(Cross(p.RG, fg), Cross(p.RB, fb));
            return new[] { -f[0], -f[1], -f[2], -m[0], -m[1], -m[2] };
        }

        /// <summary>D(nu_r) nu_r: full linear + diagonal quadratic.</summary>
        public static double[] DampingForce(CraneVehicleParams p, double[] nuR) {
            var lin = Mul(p.LinearDamping, nuR);
            for (int i = 0; i < 6; i++) lin[i] += p.QuadraticDamping[i] * Math.Abs(nuR[i]) * nuR[i];
            return lin;
        }

        public static double PotentialEnergy(CraneVehicleParams p, double z, double[] q) {
            var r = QuatToRot(q);
            double zg = z + Mul(r, p.RG)[2];
            double zb = z + Mul(r, p.RB)[2];
            return -p.Weight * zg + p.Buoyancy * zb;
        }

        /// <summary>Body-frame current-relative velocity nu_r = nu - [R^T v_c^n; 0].</summary>
        public static double[] RelativeVelocity(double[] q, double[] nu, double[] currentN) {
            var vcb = Mul(Transpose(QuatToRot(q)), currentN);
            return new[] { nu[0] - vcb[0], nu[1] - vcb[1], nu[2] - vcb[2], nu[3], nu[4], nu[5] };
        }
    }

    public sealed class CraneFossenSimulator {
        public readonly CraneVehicleParams Params;
        public double[] CurrentN = new double[3];
        public IReadOnlyList<CraneThruster> Thrusters;
        public readonly bool EnableAddedMass, EnableCoriolis, EnableDamping, EnableRestoring;
        readonly double[,] mRb, mA, mInv, tMat;

        public CraneFossenSimulator(CraneVehicleParams p, double[] currentN = null, IReadOnlyList<CraneThruster> thrusters = null,
            bool addedMass = true, bool coriolis = true, bool damping = true, bool restoring = true) {
            Params = p; p.Validate();
            if (currentN != null) CurrentN = (double[])currentN.Clone();
            Thrusters = thrusters ?? new List<CraneThruster>();
            EnableAddedMass = addedMass; EnableCoriolis = coriolis; EnableDamping = damping; EnableRestoring = restoring;
            mRb = p.MRb();
            mA = addedMass ? p.AddedMass : new double[6, 6];
            mInv = Inverse(Add(mRb, mA));
            tMat = Thrusters.Count > 0 ? CraneThrusterBank.AllocationMatrix(Thrusters) : new double[6, 0];
        }

        public double[,] MRb => mRb;
        public double[,] MA => mA;

        public double[] Acceleration(CraneState s, double[] tau) {
            var vcb = Mul(Transpose(QuatToRot(s.Q)), CurrentN);
            var nuR = CraneFossen.RelativeVelocity(s.Q, s.Nu, CurrentN);
            var wxv = Cross(Slice(s.Nu, 3, 3), vcb);
            var nudotC = new[] { -wxv[0], -wxv[1], -wxv[2], 0.0, 0.0, 0.0 };
            var rhs = (double[])tau.Clone();
            if (EnableCoriolis) {
                rhs = Sub(rhs, Mul(CraneFossen.CoriolisFromMass(mRb, s.Nu), s.Nu));
                rhs = Sub(rhs, Mul(CraneFossen.CoriolisFromMass(mA, nuR), nuR));
            }
            if (EnableDamping) rhs = Sub(rhs, CraneFossen.DampingForce(Params, nuR));
            if (EnableRestoring) rhs = Sub(rhs, CraneFossen.Restoring(Params, s.Q));
            rhs = Add(rhs, Mul(mA, nudotC));
            return Mul(mInv, rhs);
        }

        void Deriv(CraneState s, double[] tauExt, double[] fCmd, out double[] posdot, out double[] qdot, out double[] nudot, out double[] fdot) {
            var tau = (double[])tauExt.Clone();
            fdot = new double[s.Thrust.Length];
            if (Thrusters.Count > 0) {
                tau = Add(tau, Mul(tMat, s.Thrust));
                if (fCmd != null) {
                    for (int i = 0; i < Thrusters.Count; i++) {
                        double target = Thrusters[i].Target(fCmd[i]);
                        if (Thrusters[i].TimeConstant > 0.0) fdot[i] = (target - s.Thrust[i]) / Thrusters[i].TimeConstant;
                    }
                }
            }
            posdot = Mul(QuatToRot(s.Q), Slice(s.Nu, 0, 3));
            qdot = QuatRate(s.Q, Slice(s.Nu, 3, 3));
            nudot = Acceleration(s, tau);
        }

        // Stages keep the sign of the step's initial quaternion (a per-stage w>=0 flip flips that stage's qdot and
        // corrupts RK4 near w=0); the hemisphere is canonicalised only after the complete step.
        static CraneState Axpy(CraneState b, double[][] k, double h, bool canonical = false) => new CraneState {
            Pos = Add(b.Pos, Scale(k[0], h)),
            Q = canonical ? QuatNormalize(Add(b.Q, Scale(k[1], h))) : QuatUnit(Add(b.Q, Scale(k[1], h))),
            Nu = Add(b.Nu, Scale(k[2], h)),
            Thrust = Add(b.Thrust, Scale(k[3], h))
        };

        /// <summary>One classical RK4 step; zero-lag thrusters snap to the (limited) command.</summary>
        public CraneState Step(CraneState s, double dt, double[] tauExt = null, double[] fCmd = null) {
            tauExt = tauExt ?? new double[6];
            if (fCmd != null && Thrusters.Count > 0) {
                s = s.Clone();
                for (int i = 0; i < Thrusters.Count; i++)
                    if (Thrusters[i].TimeConstant <= 0.0) s.Thrust[i] = Thrusters[i].Target(fCmd[i]);
            }
            var k1 = Eval(s, tauExt, fCmd);
            var k2 = Eval(Axpy(s, k1, dt / 2), tauExt, fCmd);
            var k3 = Eval(Axpy(s, k2, dt / 2), tauExt, fCmd);
            var k4 = Eval(Axpy(s, k3, dt), tauExt, fCmd);
            var comb = new double[4][];
            for (int j = 0; j < 4; j++) {
                comb[j] = new double[k1[j].Length];
                for (int i = 0; i < comb[j].Length; i++) comb[j][i] = (k1[j][i] + 2 * k2[j][i] + 2 * k3[j][i] + k4[j][i]) / 6.0;
            }
            return Axpy(s, comb, dt, true);
        }

        double[][] Eval(CraneState s, double[] tauExt, double[] fCmd) {
            Deriv(s, tauExt, fCmd, out var p, out var q, out var n, out var f);
            return new[] { p, q, n, f };
        }

        public double KineticEnergy(CraneState s) {
            var nuR = CraneFossen.RelativeVelocity(s.Q, s.Nu, CurrentN);
            return 0.5 * Dot(s.Nu, Mul(mRb, s.Nu)) + 0.5 * Dot(nuR, Mul(mA, nuR));
        }

        public double TotalEnergy(CraneState s) =>
            KineticEnergy(s) + (EnableRestoring ? CraneFossen.PotentialEnergy(Params, s.Pos[2], s.Q) : 0.0);

        public double DissipatedPower(CraneState s) {
            if (!EnableDamping) return 0.0;
            var nuR = CraneFossen.RelativeVelocity(s.Q, s.Nu, CurrentN);
            return Dot(nuR, CraneFossen.DampingForce(Params, nuR));
        }
    }
}
