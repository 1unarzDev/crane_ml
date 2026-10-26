using System;

namespace Sim.Physics.Water.Dynamics {
    /// <summary>A planar (surge, sway, yaw) triple in body FLU: forces [N] / N m or velocities.</summary>
    public struct Planar {
        public double X, Y, N;
        public Planar(double x, double y, double n) { X = x; Y = y; N = n; }
        public double this[int j] => j == 0 ? X : j == 1 ? Y : N;
        public static Planar operator +(Planar a, Planar b) => new(a.X + b.X, a.Y + b.Y, a.N + b.N);
        public static Planar operator -(Planar a, Planar b) => new(a.X - b.X, a.Y - b.Y, a.N - b.N);
        public static Planar operator *(double k, Planar a) => new(k * a.X, k * a.Y, k * a.N);
        public bool IsFinite => double.IsFinite(X) && double.IsFinite(Y) && double.IsFinite(N);
        public override string ToString() => $"({X:G6}, {Y:G6}, {N:G6})";
    }

    /// <summary>Sign-split damping of one axis: D(s) = (D1^σ + D2^σ |s|) s, σ = sgn s.
    /// D1 [N s/m or N m s/rad], D2 [N s²/m² or N m s²/rad²].</summary>
    [Serializable]
    public struct AxisDamping {
        public double D1Pos, D1Neg, D2Pos, D2Neg;
        public AxisDamping(double d1Pos, double d1Neg, double d2Pos, double d2Neg) {
            D1Pos = d1Pos; D1Neg = d1Neg; D2Pos = d2Pos; D2Neg = d2Neg;
        }
        public static AxisDamping Symmetric(double d1, double d2) => new(d1, d1, d2, d2);

        /// <summary>Damping force, signed like s (the hull feels −Force).</summary>
        public double Force(double s) => s > 0 ? (D1Pos + D2Pos * s) * s :
                                         s < 0 ? (D1Neg - D2Neg * s) * s : 0;

        /// <summary>∂Force/∂s. At s = 0 it takes max(D1⁺, D1⁻) so a zero crossing never
        /// under-damps the implicit step.</summary>
        public double Jacobian(double s) => s > 0 ? D1Pos + 2 * D2Pos * s :
                                            s < 0 ? D1Neg - 2 * D2Neg * s : Math.Max(D1Pos, D1Neg);
    }

    /// <summary>
    /// The marine surface maneuvering model in body FLU at the CG, pure double math (no Unity), so
    /// EditMode tests and the Python twin (Tools/Characterization/plant.py) can pin it.
    /// M = M_RB + M_A with M_RB = diag(m, m, I_z), M_A = [[a11,0,0],[0,a22,a26],[0,a26,a66]];
    /// M ν̇ = τ_h − C_RB(ν)ν, τ_h = τ + M_A ν̇_c − C_A(ν_r)ν_r − D(ν_r).
    /// </summary>
    public sealed class MarinePlanarModel {
        public double Mass, Izz;
        public double A11, A22, A26, A66;
        public AxisDamping Surge, Sway, Yaw;

        public AxisDamping Axis(int j) => j == 0 ? Surge : j == 1 ? Sway : Yaw;
        /// <summary>M_eff,j = M_RB,jj + M_A,jj.</summary>
        public double EffectiveMass(int j) => j == 0 ? Mass + A11 : j == 1 ? Mass + A22 : Izz + A66;

        public static Planar MediumVelocity(double yaw, double cx, double cy) {
            double c = Math.Cos(yaw), s = Math.Sin(yaw);
            return new Planar(c * cx + s * cy, -s * cx + c * cy, 0);
        }

        /// <summary>ν̇_c of a constant world current seen from a body turning at r.</summary>
        public static Planar MediumAcceleration(Planar mediumVelocity, double r) =>
            new(r * mediumVelocity.Y, -r * mediumVelocity.X, 0);

        /// <summary>C_A(ν_r)ν_r: skew, does no work.</summary>
        public Planar AddedCoriolis(Planar vr) {
            double sway = A22 * vr.Y + A26 * vr.N;
            return new Planar(-sway * vr.N, A11 * vr.X * vr.N, sway * vr.X - A11 * vr.X * vr.Y);
        }

        public Planar RigidCoriolis(Planar nu) => new(-Mass * nu.Y * nu.N, Mass * nu.X * nu.N, 0);

        public Planar Damping(Planar vr) => new(Surge.Force(vr.X), Sway.Force(vr.Y), Yaw.Force(vr.N));

        public Planar Jacobian(Planar vr) =>
            new(Surge.Jacobian(vr.X), Sway.Jacobian(vr.Y), Yaw.Jacobian(vr.N));

        public Planar AddedMassTimes(Planar a) => new(A11 * a.X, A22 * a.Y + A26 * a.N, A26 * a.Y + A66 * a.N);

        /// <summary>τ_h without the actuator term: M_A ν̇_c − C_A(ν_r)ν_r − D(ν_r).</summary>
        public Planar Hydrodynamic(Planar nu, Planar mediumVelocity) {
            Planar vr = nu - mediumVelocity;
            return AddedMassTimes(MediumAcceleration(mediumVelocity, nu.N)) - AddedCoriolis(vr) - Damping(vr);
        }

        /// <summary>
        /// The term that stands where dt·J_j stood in M̃: c_j·J_j = M_eff·(x/(1 − e^{−x}) − 1),
        /// x = dt·J_j/M_eff. It is the review's step h = (M_eff/J)(e^{x} − 1) of the implicit update
        /// with the acceleration scaled by h/dt, because Unity integrates over dt, not h:
        /// M_eff + c·J = (dt/h)(M_eff + h·J). Linearized decay is then exactly e^{−x} per tick at any
        /// dt (plain dt·J gave 1/(1 + x), lagging by about x/2), and the steady state stays unbiased.
        /// </summary>
        public static double StepTerm(double meff, double jacobian, double dt) {
            double x = dt * jacobian / meff;
            if (!(x > 0)) return 0;
            return meff * (x < 1e-4 ? x / 2 + x * x / 12 : x / (1 - Math.Exp(-x)) - 1);
        }

        /// <summary>
        /// The force Unity must apply (it solves M_RB ν̇ + C_RB ν = F_U) for the exponential step
        /// of the linearized damping:
        ///   M̃ = M + diag(c·J),  F_U = τ_h − (M_A + diag(c·J)) · M̃⁻¹ (τ_h − C_RB ν),
        /// with c·J from <see cref="StepTerm"/> (≈ dt·J/2 for small dt·J/M_eff).
        /// PINNED FORM: c·J sits next to M_A in the correction. Dropping it from either place
        /// silently reverts to explicit damping, which diverges at dt·J/M_eff ≳ 2
        /// (MarineAnalyticTests.ImplicitDampingFormIsPinned). With J = 0 this is exactly Fossen.
        /// </summary>
        public Planar UnityForce(Planar nu, Planar tauH, Planar jacobian, double dt) {
            Planar x = tauH - RigidCoriolis(nu);
            double j0 = StepTerm(Mass + A11, jacobian.X, dt), j1 = StepTerm(Mass + A22, jacobian.Y, dt),
                j2 = StepTerm(Izz + A66, jacobian.N, dt);
            // M̃ is block diagonal: surge alone, sway-yaw coupled through a26.
            double m00 = Mass + A11 + j0, m11 = Mass + A22 + j1, m22 = Izz + A66 + j2, m12 = A26;
            double det = m11 * m22 - m12 * m12;
            var accel = new Planar(x.X / m00, (m22 * x.Y - m12 * x.N) / det, (m11 * x.N - m12 * x.Y) / det);
            var correction = new Planar((A11 + j0) * accel.X, (A22 + j1) * accel.Y + A26 * accel.N,
                A26 * accel.Y + (A66 + j2) * accel.N);
            return tauH - correction;
        }

        /// <summary>Planar state: world ENU position, heading and world velocity, as Unity holds it.</summary>
        public struct State {
            public double X, Y, Psi, Vx, Vy, R;
            public Planar Body => new(Math.Cos(Psi) * Vx + Math.Sin(Psi) * Vy,
                -Math.Sin(Psi) * Vx + Math.Cos(Psi) * Vy, R);
            public bool IsFinite => double.IsFinite(X + Y + Psi + Vx + Vy + R);
        }

        /// <summary>
        /// One tick as the plant and Unity do it together: τ_h and J from the state, F_U from
        /// <see cref="UnityForce"/>, then Unity's symplectic Euler in the world frame (velocity, then
        /// position). <paramref name="thrust"/> is the body actuator wrench; (cx, cy) a constant world
        /// current [m/s].
        /// </summary>
        public void Step(ref State s, Planar thrust, double cx, double cy, double dt) {
            Planar nu = s.Body;
            Planar nuc = MediumVelocity(s.Psi, cx, cy);
            Planar tauH = thrust + Hydrodynamic(nu, nuc);
            Planar f = UnityForce(nu, tauH, Jacobian(nu - nuc), dt);
            double c = Math.Cos(s.Psi), sn = Math.Sin(s.Psi);
            s.Vx += dt * (c * f.X - sn * f.Y) / Mass;
            s.Vy += dt * (sn * f.X + c * f.Y) / Mass;
            s.R += dt * f.N / Izz;
            s.X += dt * s.Vx;
            s.Y += dt * s.Vy;
            s.Psi += dt * s.R;
        }

        /// <summary>Continuous body acceleration ν̇ = M⁻¹(τ − C_RB ν − C_A ν_r − D ν_r + M_A ν̇_c).</summary>
        public Planar Acceleration(Planar nu, Planar thrust, Planar mediumVelocity) {
            Planar rhs = thrust + Hydrodynamic(nu, mediumVelocity) - RigidCoriolis(nu);
            double m00 = Mass + A11, m11 = Mass + A22, m22 = Izz + A66, det = m11 * m22 - A26 * A26;
            return new Planar(rhs.X / m00, (m22 * rhs.Y - A26 * rhs.N) / det, (m11 * rhs.N - A26 * rhs.Y) / det);
        }

        /// <summary>Reference: classic RK4 of the continuous ODE (body ν, world η).</summary>
        public void Rk4Step(ref State s, Planar thrust, double cx, double cy, double dt) {
            double[] y = { s.X, s.Y, s.Psi, s.Body.X, s.Body.Y, s.R };
            double[] k1 = Derivative(y, thrust, cx, cy);
            double[] k2 = Derivative(Axpy(y, k1, dt / 2), thrust, cx, cy);
            double[] k3 = Derivative(Axpy(y, k2, dt / 2), thrust, cx, cy);
            double[] k4 = Derivative(Axpy(y, k3, dt), thrust, cx, cy);
            for (int i = 0; i < 6; i++) y[i] += dt / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
            double c = Math.Cos(y[2]), sn = Math.Sin(y[2]);
            s = new State { X = y[0], Y = y[1], Psi = y[2], Vx = c * y[3] - sn * y[4], Vy = sn * y[3] + c * y[4], R = y[5] };
        }

        private double[] Derivative(double[] y, Planar thrust, double cx, double cy) {
            var nu = new Planar(y[3], y[4], y[5]);
            Planar a = Acceleration(nu, thrust, MediumVelocity(y[2], cx, cy));
            double c = Math.Cos(y[2]), sn = Math.Sin(y[2]);
            return new[] { c * nu.X - sn * nu.Y, sn * nu.X + c * nu.Y, nu.N, a.X, a.Y, a.N };
        }

        private static double[] Axpy(double[] y, double[] k, double h) {
            var r = new double[y.Length];
            for (int i = 0; i < y.Length; i++) r[i] = y[i] + h * k[i];
            return r;
        }

        /// <summary>Steady speed under a constant force T: (−D1 + √(D1² + 4 D2 T)) / (2 D2), or T/D1.</summary>
        public static double SteadySpeed(double d1, double d2, double force) =>
            d2 == 0 ? force / d1 : (-d1 + Math.Sqrt(d1 * d1 + 4 * d2 * force)) / (2 * d2);

        /// <summary>½ ν_rᵀ M ν_r (with ν_r = ν when there is no current).</summary>
        public double Energy(Planar vr) =>
            0.5 * ((Mass + A11) * vr.X * vr.X + (Mass + A22) * vr.Y * vr.Y + (Izz + A66) * vr.N * vr.N
                   + 2 * A26 * vr.Y * vr.N);
    }
}
