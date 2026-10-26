using System;

namespace Sim.Physics.Aerial {
    /// <summary>One rotor of the V2 model. Body frame FLU, spin +1 = CCW seen from above.</summary>
    public sealed class RotorSpecV2 {
        public string Name;
        public double[] Position = new double[3];
        public int Spin = 1;
        public double Kf, Km, OmegaMax, OmegaMin;
        public double TauUp = 0.02, TauDown = 0.02;
        public double RotorDrag, RollingMoment, Radius;
        public double RotorInertia;                  // kg m^2 about the shaft; 0 disables gyroscopics
        public double[] Axis = { 0, 0, 1 };
    }

    public sealed class BatterySpecV2 {
        public double VFull, VEmpty, VNominal, Resistance, CapacityAh, Efficiency = 0.7;
        public double OpenCircuit(double soc) =>
            VEmpty + (VFull - VEmpty) * Math.Max(0.0, Math.Min(1.0, soc));
    }

    public sealed class VehicleSpecV2 {
        public string Name;
        public double Mass;
        public double[,] Inertia = new double[3, 3];
        public RotorSpecV2[] Rotors;
        public double Gravity = 9.81;
        public double[] DragLinear = new double[3];
        public double DragQuadratic;
        public bool GroundEffect;
        public double GroundEffectMinHeightRatio = 0.5;
        public double GroundHeight;
        public BatterySpecV2 Battery;
    }

    /// <summary>World ENU z-up state, body FLU, quaternion Hamilton (w, x, y, z).</summary>
    public sealed class StateV2 {
        public double[] P = new double[3], V = new double[3], Q = { 1, 0, 0, 0 }, Omega = new double[3];
        public double[] RotorSpeed;
        public double Soc = 1.0, T;
        public StateV2(int rotors) { RotorSpeed = new double[rotors]; }
        public StateV2 Copy() {
            var s = new StateV2(RotorSpeed.Length) { Soc = Soc, T = T };
            Array.Copy(P, s.P, 3); Array.Copy(V, s.V, 3); Array.Copy(Q, s.Q, 4);
            Array.Copy(Omega, s.Omega, 3); Array.Copy(RotorSpeed, s.RotorSpeed, RotorSpeed.Length);
            return s;
        }
    }

    /// <summary>
    /// Pure-math opt-in rotor/rigid-body model, a 1:1 port of Tools/Aerial/aerial_model.py:
    /// thrust kf*w^2, asymmetric first-order motor lag (exact ZOH), reaction (drag) torque,
    /// rotor drag and rolling moment proportional to |w| v_perp, rotation-consistent body drag,
    /// Cheeseman-Bennett ground effect, Thevenin battery sag, RK4 rigid-body integration.
    /// Extension beyond the oracle: motor gyroscopic torque -omega x h (zero when RotorInertia == 0).
    /// It never touches the legacy MultirotorDynamics plant.
    /// </summary>
    public static class CraneRotorModelV2 {
        // ---- small vector math -------------------------------------------------------------
        public static double Dot(double[] a, double[] b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        public static double Norm(double[] a) => Math.Sqrt(Dot(a, a));
        public static double[] Cross(double[] a, double[] b) => new[] {
            a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] };
        public static double[] Add(double[] a, double[] b) => new[] { a[0] + b[0], a[1] + b[1], a[2] + b[2] };
        public static double[] Sub(double[] a, double[] b) => new[] { a[0] - b[0], a[1] - b[1], a[2] - b[2] };
        public static double[] Scale(double[] a, double s) => new[] { a[0] * s, a[1] * s, a[2] * s };
        public static double[] MatVec(double[,] m, double[] v) => new[] {
            m[0, 0] * v[0] + m[0, 1] * v[1] + m[0, 2] * v[2],
            m[1, 0] * v[0] + m[1, 1] * v[1] + m[1, 2] * v[2],
            m[2, 0] * v[0] + m[2, 1] * v[1] + m[2, 2] * v[2] };
        public static double[] MatTVec(double[,] m, double[] v) => new[] {
            m[0, 0] * v[0] + m[1, 0] * v[1] + m[2, 0] * v[2],
            m[0, 1] * v[0] + m[1, 1] * v[1] + m[2, 1] * v[2],
            m[0, 2] * v[0] + m[1, 2] * v[1] + m[2, 2] * v[2] };

        public static double[] QuatMul(double[] a, double[] b) => new[] {
            a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0] };

        public static double[,] QuatToMatrix(double[] q) {
            double n = Math.Sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
            double w = q[0] / n, x = q[1] / n, y = q[2] / n, z = q[3] / n;
            return new[,] {
                { 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y) },
                { 2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x) },
                { 2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y) } };
        }

        public static double[] SolveSymmetric3(double[,] a, double[] b) {
            double det = a[0, 0] * (a[1, 1] * a[2, 2] - a[1, 2] * a[2, 1])
                - a[0, 1] * (a[1, 0] * a[2, 2] - a[1, 2] * a[2, 0])
                + a[0, 2] * (a[1, 0] * a[2, 1] - a[1, 1] * a[2, 0]);
            double D(int c) {
                var m = (double[,])a.Clone();
                for (int r = 0; r < 3; r++) m[r, c] = b[r];
                return m[0, 0] * (m[1, 1] * m[2, 2] - m[1, 2] * m[2, 1])
                    - m[0, 1] * (m[1, 0] * m[2, 2] - m[1, 2] * m[2, 0])
                    + m[0, 2] * (m[1, 0] * m[2, 1] - m[1, 1] * m[2, 0]);
            }
            return new[] { D(0) / det, D(1) / det, D(2) / det };
        }

        // ---- model -------------------------------------------------------------------------
        public static double HoverRotorSpeed(VehicleSpecV2 v) {
            double kf = 0.0;
            foreach (var r in v.Rotors) kf += r.Kf;
            return Math.Sqrt(v.Mass * v.Gravity / kf);
        }

        public static double GroundEffectFactor(VehicleSpecV2 v, RotorSpecV2 r, double height) {
            if (!v.GroundEffect || r.Radius <= 0.0) return 1.0;
            double h = Math.Max(height, v.GroundEffectMinHeightRatio * r.Radius);
            double ratio = r.Radius / (4.0 * h);
            return 1.0 / (1.0 - ratio * ratio);
        }

        public static double ElectricalCurrent(VehicleSpecV2 v, StateV2 s) {
            var b = v.Battery;
            if (b == null) return 0.0;
            double power = 0.0;
            for (int i = 0; i < v.Rotors.Length; i++)
                power += v.Rotors[i].Km * v.Rotors[i].Kf * Math.Pow(Math.Abs(s.RotorSpeed[i]), 3);
            power /= b.Efficiency;
            double voc = b.OpenCircuit(s.Soc);
            double disc = voc * voc - 4.0 * b.Resistance * power;
            if (disc < 0.0) return voc / (2.0 * b.Resistance);
            return (voc - Math.Sqrt(disc)) / (2.0 * b.Resistance);
        }

        /// <summary>Terminal voltage ratio cap on omega_max (battery sag).</summary>
        public static double RotorSpeedLimit(VehicleSpecV2 v, RotorSpecV2 r, StateV2 s) {
            var b = v.Battery;
            if (b == null) return r.OmegaMax;
            double current = ElectricalCurrent(v, s);
            double volts = Math.Max(0.0, b.OpenCircuit(s.Soc) - current * b.Resistance);
            return r.OmegaMax * Math.Min(1.0, volts / b.VNominal);
        }

        /// <summary>Body-frame (FLU) force and torque excluding gravity, given the rotation matrix.</summary>
        public static void Wrench(VehicleSpecV2 v, double[,] rWb, double[] p, double[] vel,
            double[] omega, double[] rotorSpeed, double[] windWorld, out double[] force, out double[] torque) {
            double[] vAirB = MatTVec(rWb, Sub(vel, windWorld));
            force = new double[3];
            torque = new double[3];
            double[] angMomentum = new double[3];
            for (int i = 0; i < v.Rotors.Length; i++) {
                RotorSpecV2 r = v.Rotors[i];
                double speed = rotorSpeed[i];
                double[] axis = Scale(r.Axis, 1.0 / Norm(r.Axis));
                double[] hubWorld = Add(p, MatVec(rWb, r.Position));
                double height = hubWorld[2] - v.GroundHeight;
                double thrust = r.Kf * speed * speed * GroundEffectFactor(v, r, height);
                double[] f = Scale(axis, thrust);
                double[] vHub = Add(vAirB, Cross(omega, r.Position));
                double[] vPerp = Sub(vHub, Scale(axis, Dot(vHub, axis)));
                f = Sub(f, Scale(vPerp, Math.Abs(speed) * r.RotorDrag));
                force = Add(force, f);
                torque = Add(torque, Cross(r.Position, f));
                torque = Add(torque, Scale(axis, -r.Spin * r.Km * thrust));
                torque = Add(torque, Scale(vPerp, -Math.Abs(speed) * r.RollingMoment));
                if (r.RotorInertia != 0.0)
                    angMomentum = Add(angMomentum, Scale(axis, r.RotorInertia * r.Spin * speed));
            }
            double airSpeed = Norm(vAirB);
            for (int k = 0; k < 3; k++)
                force[k] += -v.DragLinear[k] * vAirB[k] - v.DragQuadratic * airSpeed * vAirB[k];
            // Motor gyroscopic torque on the body: -omega x h (Euler: I w' + w x (I w + h) = tau).
            if (angMomentum[0] != 0.0 || angMomentum[1] != 0.0 || angMomentum[2] != 0.0)
                torque = Sub(torque, Cross(omega, angMomentum));
        }

        public static void Wrench(VehicleSpecV2 v, StateV2 s, double[] windWorld,
            out double[] force, out double[] torque) =>
            Wrench(v, QuatToMatrix(s.Q), s.P, s.V, s.Omega, s.RotorSpeed, windWorld, out force, out torque);

        /// <summary>
        /// Rigid-body gyroscopic torque -w x (I w) for a diagonal inertia, all in the same body axes.
        /// UnityEngine.Rigidbody does not apply it (ArticulationBody does), so a plant driving a
        /// Rigidbody must add it explicitly. Separate from the rotor-angular-momentum term in Wrench.
        /// </summary>
        public static double[] RigidBodyGyroTorque(double[] inertiaDiag, double[] omegaBody) {
            var iw = new[] { inertiaDiag[0] * omegaBody[0], inertiaDiag[1] * omegaBody[1], inertiaDiag[2] * omegaBody[2] };
            return Scale(Cross(omegaBody, iw), -1.0);
        }

        public static double[] SpecificForceBody(VehicleSpecV2 v, StateV2 s, double[] windWorld) {
            Wrench(v, s, windWorld, out double[] f, out _);
            return Scale(f, 1.0 / v.Mass);
        }

        /// <summary>Advances rotor speeds exactly over dt (in place, rotor order, as the oracle).</summary>
        public static void UpdateMotors(VehicleSpecV2 v, StateV2 s, double[] command, double dt) {
            for (int i = 0; i < v.Rotors.Length; i++) {
                RotorSpecV2 r = v.Rotors[i];
                double limit = RotorSpeedLimit(v, r, s);
                double target = Math.Min(Math.Max(command[i], r.OmegaMin), limit);
                double tau = target >= s.RotorSpeed[i] ? r.TauUp : r.TauDown;
                double alpha = tau <= 0.0 ? 1.0 : 1.0 - Math.Exp(-dt / tau);
                s.RotorSpeed[i] += (target - s.RotorSpeed[i]) * alpha;
            }
        }

        private struct Deriv { public double[] Dp, Dv, Dq, Dw; }

        private static Deriv Derivative(VehicleSpecV2 v, StateV2 s, double[] wind) {
            Wrench(v, s, wind, out double[] fb, out double[] tb);
            double[,] r = QuatToMatrix(s.Q);
            double[] acc = Add(Scale(MatVec(r, fb), 1.0 / v.Mass), new[] { 0.0, 0.0, -v.Gravity });
            double[] iw = MatVec(v.Inertia, s.Omega);
            double[] wDot = SolveSymmetric3(v.Inertia, Sub(tb, Cross(s.Omega, iw)));
            double[] qDot = new double[4];
            double[] prod = QuatMul(s.Q, new[] { 0.0, s.Omega[0], s.Omega[1], s.Omega[2] });
            for (int k = 0; k < 4; k++) qDot[k] = 0.5 * prod[k];
            return new Deriv { Dp = (double[])s.V.Clone(), Dv = acc, Dq = qDot, Dw = wDot };
        }

        private static StateV2 Shifted(StateV2 s0, Deriv k, double h) {
            StateV2 s = s0.Copy();
            for (int i = 0; i < 3; i++) {
                s.P[i] = s0.P[i] + h * k.Dp[i];
                s.V[i] = s0.V[i] + h * k.Dv[i];
                s.Omega[i] = s0.Omega[i] + h * k.Dw[i];
            }
            for (int i = 0; i < 4; i++) s.Q[i] = s0.Q[i] + h * k.Dq[i];
            NormalizeQuat(s.Q);
            return s;
        }

        private static void NormalizeQuat(double[] q) {
            double n = Math.Sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
            for (int i = 0; i < 4; i++) q[i] /= n;
        }

        /// <summary>One fixed step; command is per-rotor target speed [rad/s]. Returns a new state.</summary>
        public static StateV2 Step(VehicleSpecV2 v, StateV2 state, double[] command, double dt,
            double[] windWorld = null, bool contact = false) {
            windWorld ??= new double[3];
            UpdateMotors(v, state, command, dt);
            StateV2 s0 = state;
            Deriv k1 = Derivative(v, s0, windWorld);
            Deriv k2 = Derivative(v, Shifted(s0, k1, dt / 2), windWorld);
            Deriv k3 = Derivative(v, Shifted(s0, k2, dt / 2), windWorld);
            Deriv k4 = Derivative(v, Shifted(s0, k3, dt), windWorld);
            StateV2 n = s0.Copy();
            for (int i = 0; i < 3; i++) {
                n.P[i] = s0.P[i] + dt / 6 * (k1.Dp[i] + 2 * k2.Dp[i] + 2 * k3.Dp[i] + k4.Dp[i]);
                n.V[i] = s0.V[i] + dt / 6 * (k1.Dv[i] + 2 * k2.Dv[i] + 2 * k3.Dv[i] + k4.Dv[i]);
                n.Omega[i] = s0.Omega[i] + dt / 6 * (k1.Dw[i] + 2 * k2.Dw[i] + 2 * k3.Dw[i] + k4.Dw[i]);
            }
            for (int i = 0; i < 4; i++)
                n.Q[i] = s0.Q[i] + dt / 6 * (k1.Dq[i] + 2 * k2.Dq[i] + 2 * k3.Dq[i] + k4.Dq[i]);
            NormalizeQuat(n.Q);
            if (v.Battery != null)
                n.Soc = s0.Soc - ElectricalCurrent(v, s0) * dt / (3600.0 * v.Battery.CapacityAh);
            if (contact && n.P[2] < v.GroundHeight) {
                n.P[2] = v.GroundHeight;
                n.V[2] = Math.Max(0.0, n.V[2]);
            }
            n.T = s0.T + dt;
            return n;
        }

        public static double MechanicalEnergy(VehicleSpecV2 v, StateV2 s) {
            double[] iw = MatVec(v.Inertia, s.Omega);
            return 0.5 * v.Mass * Dot(s.V, s.V) + 0.5 * Dot(s.Omega, iw) + v.Mass * v.Gravity * s.P[2];
        }
    }
}
