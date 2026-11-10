using System;
using System.Collections.Generic;

namespace Crane.Fleet {
    /// <summary>Wheel kinds of the capplan module library (library.py ACT_KINDS).</summary>
    public enum WheelKind { Mecanum, Conventional, Omni, Caster }

    /// <summary>One wheel of a module in the chassis frame (FLU, origin = base_link). For mecanum wheels <see cref="Hand"/> is the roller
    /// hand of library.py <c>mecanum_set</c>: the roller axle is a = R(yaw) (1, hand)/sqrt 2 (fl -1, fr +1, rl +1, rr -1 for the X pattern).</summary>
    public sealed class WheelUnit {
        public string Name;
        public WheelKind Kind;
        public double X, Y, Yaw;
        public int Hand;
        public double Radius, MaxSpeed, MaxTorque;
        public bool Driven => Kind != WheelKind.Caster;

        /// <summary>Unit direction (FLU) along which the wheel's rotation constrains the contact velocity, and c = heading . a
        /// (the share of the surface speed r*omega that appears along a). Casters have none.</summary>
        public void Constraint(out double ax, out double ay, out double c) {
            double ch = Math.Cos(Yaw), sh = Math.Sin(Yaw);
            if (Kind == WheelKind.Mecanum) {
                double s = 1 / Math.Sqrt(2);
                ax = s * (ch - Hand * sh); ay = s * (sh + Hand * ch); c = s;
            }
            else { ax = ch; ay = sh; c = Kind == WheelKind.Caster ? 0 : 1; }
        }
    }

    /// <summary>What the simulator needs from a wheel plant, whichever model it is (wheel arrays in the plant's wheel order).</summary>
    public interface IWheelPlant {
        int WheelCount { get; }
        double[] WheelCommand { get; }
        double[] WheelAngle { get; }
        WheelHealth[] Health { get; }
        /// <summary>weak(g) gain per wheel; writable for the tractive plant (the body-level mecanum plant has one shared fraction).</summary>
        double[] HealthGain { get; }
        double TractionScale { get; }
        /// <summary>Driven-wheel set-points (rad/s) from an autonomy layer, replacing the twist command when set.</summary>
        double[] WheelCommandOverride { get; set; }
        void Step(Twist command, Twist measured, double dt);
        /// <summary>Indices of the wheels with a motor and an encoder, in command/encoder order.</summary>
        int[] Driven { get; }
        /// <summary>Ideal-kinematics body twist from driven-wheel speeds (order <see cref="Driven"/>), for encoder odometry.</summary>
        Twist ForwardDriven(double[] w);
    }

    /// <summary>Contact and actuator parameters of the tractive plant (overlay <c>plant.tractive</c> plus the floor); every value comes from the
    /// spec, nothing here is a default.</summary>
    public sealed class TractionParams {
        public double MuLong, MuLatRatio;        // conventional-wheel friction: longitudinal mu (floor), lateral = ratio * mu
        public double SlipVelocity;              // m/s, slip at which the tanh friction curve reaches 76 % of mu N
        public double RollerDrag;                // mecanum/omni free direction: drag coefficient (fraction of N at full slip)
        public double RollingResistance;         // casters and every wheel's rolling direction (fraction of N)
        public double MotorGain;                 // N m per rad/s of speed error (velocity P loop)
        public double MotorTimeConstant;         // s, first-order lag of the set-point
        public double BackdriveTorque;           // N m, Coulomb torque of an unpowered (dead_free) motor/gearbox
        public double BackdriveSpeed;            // rad/s, smoothing of that Coulomb torque
        public double Mass, Izz, ComX, ComY, ComZ, Gravity;
        public double MaxSpeedXy, MaxYawRate;    // body limits for twist commands
        public double DefaultWeakGain;           // weak without an explicit gain
        /// <summary>"clamp": torque clamp(K (w_ref - w), +-tau_max) (v0). "linear_dc": a DC gearmotor whose available torque in the motoring
        /// direction falls linearly from stall (tau_max) to zero at the no-load speed (the wheel MaxSpeed, datasheet: Pololu 25D 47:1 for v1);
        /// braking torque up to stall.</summary>
        public string MotorCurve = "clamp";

        /// <summary>From the overlay <c>plant.tractive</c>, <c>plant.motor_time_constant</c>, the floor and the rover description.</summary>
        public static TractionParams From(FleetSpec spec, RoverDescription d, FloorProperties floor) {
            JsonNode t = spec.O["plant.tractive"];
            return new TractionParams {
                MuLong = floor.MuLong, MuLatRatio = t["mu_lat_ratio"].Double, SlipVelocity = t["slip_velocity"].Double, RollerDrag = t["roller_drag"].Double,
                RollingResistance = t["rolling_resistance"].Double, MotorGain = t["motor_gain"].Double, MotorTimeConstant = spec.O["plant.motor_time_constant"].Double,
                BackdriveTorque = t["backdrive_torque"].Double, BackdriveSpeed = t["backdrive_speed"].Double, DefaultWeakGain = t["weak_gain_default"].Double,
                MotorCurve = spec.O.TryGet("profiles." + spec.ProfileKey + ".motor_curve")?.String ?? t["motor_curve"].String,
                Mass = d.Mass, Izz = d.InertiaDiag[2], ComX = d.ComOffset[0], ComY = d.ComOffset[1], ComZ = d.ComOffset[2] + d.OriginHeight, Gravity = spec.Gravity,
                MaxSpeedXy = d.MaxSpeedXy, MaxYawRate = d.MaxYawRate
            };
        }
    }

    /// <summary>
    /// Per-wheel tractive plant for N wheels of any kind (CRANE_MODULAR_PLAN item P1; models after capplan/truth/README.md, engine-free,
    /// independent code). Each step:
    /// <list type="bullet">
    /// <item>normal loads: minimum-norm static split over all wheels (casters included) of the weight with the centre of mass shifted by the
    /// last acceleration (load transfer), negative loads clipped and re-solved;</item>
    /// <item>driven wheels: massless; the wheel speed solves motor torque = r c F_a (bisection; the left side decreases and the right side
    /// increases in omega, so the root is unique). Motor: clamp(K (w_ref - w), +-tau_max), w_ref a lagged, clamped set-point. dead_free:
    /// Coulomb back-drive torque; dead_locked: w = 0; weak(g): tau_max and speed limit scaled by g;</item>
    /// <item>contact: smooth tanh slip curve. Conventional: combined slip on an anisotropic friction ellipse (mu_long, mu_lat).
    /// Mecanum/omni: slip only along the roller axle a, roller drag along the free direction, capped by the friction circle. Casters:
    /// quasi-static (aligned), rolling resistance only;</item>
    /// <item>linearly implicit: the per-wheel forces are corrected by their finite-difference Jacobian with respect to the body twist so that
    /// the explicit rigid-body step of the engine equals an implicit Euler step (the slip curve is stiff at dt = 5 ms).</item>
    /// </list>
    /// Outputs <see cref="Fx"/>, <see cref="Fy"/> per wheel (chassis FLU, N) to apply at each wheel's floor contact point.
    /// </summary>
    public sealed class TractivePlant : IWheelPlant {
        public readonly WheelUnit[] W;
        public readonly TractionParams P;
        public readonly int N;
        public readonly int[] Drive;                      // indices of driven wheels (command / encoder order)

        public readonly double[] WheelCommand, WheelRef, WheelSpeed, WheelAngle, NormalLoad, Fx, Fy;
        public readonly WheelHealth[] Health;
        public readonly double[] Gain;                   // weak(g) per wheel
        /// <summary>Wheel-speed set-points (rad/s) for the driven wheels (order <see cref="Drive"/>); replace the twist command when set.</summary>
        public double[] WheelCommandOverride;
        public double SpeedScale = 1, TractionScale = 1; // 1 = no wheel-speed limit / no wheel at its friction limit
        public double Ax, Ay, Alpha;                      // body acceleration (CoM, chassis frame) implied by the applied forces
        double lastAx, lastAy;

        public TractivePlant(IList<WheelUnit> wheels, TractionParams p) {
            W = new WheelUnit[wheels.Count]; wheels.CopyTo(W, 0);
            P = p; N = W.Length;
            var drive = new List<int>();
            for (int i = 0; i < N; i++) if (W[i].Driven) drive.Add(i);
            Drive = drive.ToArray();
            WheelCommand = new double[N]; WheelRef = new double[N]; WheelSpeed = new double[N]; WheelAngle = new double[N];
            NormalLoad = new double[N]; Fx = new double[N]; Fy = new double[N];
            Health = new WheelHealth[N]; Gain = new double[N];
            for (int i = 0; i < N; i++) Gain[i] = 1;
        }

        int IWheelPlant.WheelCount => N;
        double[] IWheelPlant.WheelCommand => WheelCommand;
        double[] IWheelPlant.WheelAngle => WheelAngle;
        WheelHealth[] IWheelPlant.Health => Health;
        double[] IWheelPlant.HealthGain => Gain;
        double IWheelPlant.TractionScale => TractionScale;
        double[] IWheelPlant.WheelCommandOverride { get => WheelCommandOverride; set => WheelCommandOverride = value; }
        int[] IWheelPlant.Driven => Drive;

        public Twist ForwardDriven(double[] w) {
            var full = new double[N];
            for (int k = 0; k < Drive.Length; k++) full[Drive[k]] = w[k];
            return Forward(full);
        }

        // ------------------------------------------------------------------ kinematics (base firmware: ideal rolling)
        /// <summary>Ideal-rolling inverse kinematics: body twist at base_link to wheel speeds (casters 0).</summary>
        public void Inverse(Twist t, double[] w) {
            for (int i = 0; i < N; i++) {
                var u = W[i];
                if (!u.Driven) { w[i] = 0; continue; }
                u.Constraint(out double ax, out double ay, out double c);
                double vx = t.Vx - t.Wz * u.Y, vy = t.Vy + t.Wz * u.X;
                w[i] = (ax * vx + ay * vy) / (u.Radius * c);
            }
        }

        /// <summary>Weighted least-squares forward kinematics over the driven wheels (weights 0 drop a wheel). Directions the wheels do not
        /// constrain (a differential pod's lateral) come out 0 (small ridge), which is what an encoder-only base can know.</summary>
        public Twist Forward(double[] w, double[] weight = null) {
            var A = new double[3, 3]; var b = new double[3];
            for (int i = 0; i < N; i++) {
                var u = W[i];
                if (!u.Driven) continue;
                double wt = weight == null ? 1 : weight[i];
                if (wt == 0) continue;
                u.Constraint(out double ax, out double ay, out double c);
                double[] row = { ax, ay, ay * u.X - ax * u.Y };
                double y = u.Radius * c * w[i];
                for (int p = 0; p < 3; p++) { b[p] += wt * row[p] * y; for (int q = 0; q < 3; q++) A[p, q] += wt * row[p] * row[q]; }
            }
            for (int p = 0; p < 3; p++) A[p, p] += 1e-9;
            var x = Solve3(A, b);
            return x == null ? new Twist() : new Twist(x[0], x[1], x[2]);
        }

        /// <summary>Body limits, then one common scale if any driven wheel exceeds its speed limit (keeps the direction).</summary>
        public double Saturate(ref Twist t, double[] w) {
            double sp = Math.Sqrt(t.Vx * t.Vx + t.Vy * t.Vy);
            if (sp > P.MaxSpeedXy) { t.Vx *= P.MaxSpeedXy / sp; t.Vy *= P.MaxSpeedXy / sp; }
            if (Math.Abs(t.Wz) > P.MaxYawRate) t.Wz = Math.Sign(t.Wz) * P.MaxYawRate;
            Inverse(t, w);
            double s = 1;
            for (int i = 0; i < N; i++) if (W[i].Driven && Math.Abs(w[i]) > W[i].MaxSpeed) s = Math.Min(s, W[i].MaxSpeed / Math.Abs(w[i]));
            if (s < 1) for (int i = 0; i < N; i++) w[i] *= s;
            return s;
        }

        // ------------------------------------------------------------------ dynamics
        /// <summary>One step. <paramref name="measured"/> is the body twist at base_link (chassis frame).</summary>
        public void Step(Twist command, Twist measured, double dt) {
            if (WheelCommandOverride != null) {
                SpeedScale = 1;
                Array.Clear(WheelCommand, 0, N);
                for (int k = 0; k < Drive.Length && k < WheelCommandOverride.Length; k++) {
                    int i = Drive[k];
                    WheelCommand[i] = Math.Max(-W[i].MaxSpeed, Math.Min(W[i].MaxSpeed, WheelCommandOverride[k]));
                }
            }
            else SpeedScale = Saturate(ref command, WheelCommand);
            double a = 1 - Math.Exp(-dt / P.MotorTimeConstant);
            for (int i = 0; i < N; i++) WheelRef[i] += (WheelCommand[i] - WheelRef[i]) * a;

            Loads();

            // twist at the CoM (the mass matrix is diagonal there)
            var V = new[] { measured.Vx - measured.Wz * P.ComY, measured.Vy + measured.Wz * P.ComX, measured.Wz };
            var f0x = new double[N]; var f0y = new double[N]; var om = new double[N];
            Forces(V, f0x, f0y, om);
            // per-wheel Jacobians by forward differences
            var jx = new double[3, N]; var jy = new double[3, N];
            var fx = new double[N]; var fy = new double[N]; var dummy = new double[N];
            double[] eps = { 1e-4, 1e-4, 1e-4 };
            for (int k = 0; k < 3; k++) {
                var Vk = (double[])V.Clone(); Vk[k] += eps[k];
                Forces(Vk, fx, fy, dummy);
                for (int i = 0; i < N; i++) { jx[k, i] = (fx[i] - f0x[i]) / eps[k]; jy[k, i] = (fy[i] - f0y[i]) / eps[k]; }
            }
            // wrench about the CoM and its Jacobian
            var W0 = new double[3]; var J = new double[3, 3];
            for (int i = 0; i < N; i++) {
                double rx = W[i].X - P.ComX, ry = W[i].Y - P.ComY;
                W0[0] += f0x[i]; W0[1] += f0y[i]; W0[2] += rx * f0y[i] - ry * f0x[i];
                for (int k = 0; k < 3; k++) { J[0, k] += jx[k, i]; J[1, k] += jy[k, i]; J[2, k] += rx * jy[k, i] - ry * jx[k, i]; }
            }
            double[] Minv = { 1 / P.Mass, 1 / P.Mass, 1 / P.Izz };
            // (I - dt J M^-1) W* = W0 ;  dV = dt M^-1 W*
            var A = new double[3, 3];
            for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) A[r, c] = (r == c ? 1 : 0) - dt * J[r, c] * Minv[c];
            var Ws = Solve3(A, W0) ?? W0;
            var dV = new[] { dt * Minv[0] * Ws[0], dt * Minv[1] * Ws[1], dt * Minv[2] * Ws[2] };
            for (int i = 0; i < N; i++) {
                Fx[i] = f0x[i] + jx[0, i] * dV[0] + jx[1, i] * dV[1] + jx[2, i] * dV[2];
                Fy[i] = f0y[i] + jy[0, i] * dV[0] + jy[1, i] * dV[1] + jy[2, i] * dV[2];
                WheelSpeed[i] = om[i];
                WheelAngle[i] += om[i] * dt;
            }
            Ax = Ws[0] * Minv[0]; Ay = Ws[1] * Minv[1]; Alpha = Ws[2] * Minv[2];
            lastAx = Ax; lastAy = Ay;
            // traction scale: the worst wheel's force / its friction capacity (reported, not applied: the slip curve already limits)
            double worst = 0;
            for (int i = 0; i < N; i++) {
                if (NormalLoad[i] <= 0 || !W[i].Driven) continue;
                worst = Math.Max(worst, Math.Sqrt(Fx[i] * Fx[i] + Fy[i] * Fy[i]) / (P.MuLong * NormalLoad[i]));
            }
            TractionScale = worst > 0.95 ? 0.95 / worst : 1;
        }

        void Loads() {
            double g = P.Gravity, mg = P.Mass * g;
            double cx = P.ComX - P.ComZ * lastAx / g, cy = P.ComY - P.ComZ * lastAy / g;
            var on = new bool[N];
            for (int i = 0; i < N; i++) on[i] = true;
            for (int pass = 0; pass < N; pass++) {
                // minimum-norm N with sum N = mg, sum N x = mg cx, sum N y = mg cy over the active wheels
                var A = new double[3, 3];
                for (int i = 0; i < N; i++) {
                    if (!on[i]) continue;
                    double[] r = { 1, W[i].X, W[i].Y };
                    for (int p = 0; p < 3; p++) for (int q = 0; q < 3; q++) A[p, q] += r[p] * r[q];
                }
                for (int p = 0; p < 3; p++) A[p, p] += 1e-9;
                var lam = Solve3(A, new[] { mg, mg * cx, mg * cy }) ?? new[] { 0.0, 0, 0 };
                bool clipped = false;
                for (int i = 0; i < N; i++) {
                    NormalLoad[i] = on[i] ? lam[0] + lam[1] * W[i].X + lam[2] * W[i].Y : 0;
                    if (NormalLoad[i] < 0) { on[i] = false; NormalLoad[i] = 0; clipped = true; }
                }
                if (!clipped) break;
            }
        }

        /// <summary>Contact forces (chassis frame) for a CoM twist V, solving each driven wheel's speed.</summary>
        void Forces(double[] V, double[] fx, double[] fy, double[] om) {
            double v0 = P.SlipVelocity;
            for (int i = 0; i < N; i++) {
                var u = W[i];
                double n = NormalLoad[i];
                double rx = u.X - P.ComX, ry = u.Y - P.ComY;
                double vcx = V[0] - V[2] * ry, vcy = V[1] + V[2] * rx;   // contact-point velocity
                fx[i] = fy[i] = 0; om[i] = 0;
                if (u.Kind == WheelKind.Caster) {
                    double sp = Math.Sqrt(vcx * vcx + vcy * vcy);
                    if (sp > 1e-12 && n > 0) {
                        double f = P.RollingResistance * n * Math.Tanh(sp / v0);
                        fx[i] = -f * vcx / sp; fy[i] = -f * vcy / sp;
                    }
                    continue;
                }
                u.Constraint(out double ax, out double ay, out double c);
                double px = -ay, py = ax;                                // free (mecanum/omni) or lateral (conventional) direction
                double va = ax * vcx + ay * vcy, vp = px * vcx + py * vcy;
                double rc = u.Radius * c;
                double mu = P.MuLong, muP = u.Kind == WheelKind.Conventional ? P.MuLatRatio * P.MuLong : P.RollerDrag;
                // force along a for a given wheel speed (combined slip for a conventional wheel; the free direction is separate for rollers)
                double Fa(double w) {
                    double sa = (rc * w - va) / v0;
                    if (u.Kind == WheelKind.Conventional) {
                        double sb = vp / v0, s = Math.Sqrt(sa * sa + sb * sb);
                        return s < 1e-12 ? mu * n * sa : mu * n * Math.Tanh(s) * sa / s;
                    }
                    return mu * n * Math.Tanh(sa);
                }
                double w;
                var h = Health[i];
                if (h == WheelHealth.DeadLocked) w = 0;
                else {
                    double g = h == WheelHealth.Weak ? Gain[i] : 1;
                    double tauMax = u.MaxTorque * g, wmax = u.MaxSpeed * g;
                    double wref = Math.Max(-wmax, Math.Min(wmax, WheelRef[i]));
                    bool dc = P.MotorCurve == "linear_dc";
                    double Tm(double x) {
                        if (h == WheelHealth.DeadFree) return -P.BackdriveTorque * Math.Tanh(x / P.BackdriveSpeed);
                        double tau = P.MotorGain * (wref - x);
                        if (!dc) return Math.Max(-tauMax, Math.Min(tauMax, tau));
                        // torque available in the direction of tau: reduced linearly with speed when motoring (tau and x same sign), stall when braking
                        double avail = tau * x > 0 ? tauMax * Math.Max(0, 1 - Math.Abs(x) / wmax) : tauMax;
                        return Math.Max(-avail, Math.Min(avail, tau));
                    }
                    // residual Tm(w) - r c Fa(w) is strictly decreasing: bisection on a bracket that contains both the free-rolling and set-point speeds
                    double span = Math.Abs(va / rc) + Math.Abs(wref) + u.MaxSpeed + 1;
                    double lo = -span, hi = span;
                    for (int it = 0; it < 80; it++) {
                        double mid = 0.5 * (lo + hi);
                        if (Tm(mid) - rc * Fa(mid) > 0) lo = mid; else hi = mid;
                    }
                    w = 0.5 * (lo + hi);
                }
                om[i] = w;
                double fa = Fa(w);
                double fp;
                if (u.Kind == WheelKind.Conventional) {
                    double sa = (rc * w - va) / v0, sb = vp / v0, s = Math.Sqrt(sa * sa + sb * sb);
                    fp = s < 1e-12 ? -muP * n * sb : -muP * n * Math.Tanh(s) * sb / s;
                }
                else {
                    fp = -muP * n * Math.Tanh(vp / v0);
                    double mag = Math.Sqrt(fa * fa + fp * fp), cap = mu * n;
                    if (mag > cap && mag > 0) { fa *= cap / mag; fp *= cap / mag; }
                }
                // rolling resistance along the heading (all driven wheels)
                double hx = Math.Cos(u.Yaw), hy = Math.Sin(u.Yaw), vh = hx * vcx + hy * vcy;
                double frr = -P.RollingResistance * n * Math.Tanh(vh / v0);
                fx[i] = fa * ax + fp * px + frr * hx;
                fy[i] = fa * ay + fp * py + frr * hy;
            }
        }

        static double[] Solve3(double[,] A, double[] b) {
            double Det(double[,] m) => m[0, 0] * (m[1, 1] * m[2, 2] - m[1, 2] * m[2, 1]) - m[0, 1] * (m[1, 0] * m[2, 2] - m[1, 2] * m[2, 0]) + m[0, 2] * (m[1, 0] * m[2, 1] - m[1, 1] * m[2, 0]);
            double d = Det(A);
            if (Math.Abs(d) < 1e-30) return null;
            var x = new double[3];
            for (int c = 0; c < 3; c++) {
                var m = (double[,])A.Clone();
                for (int r = 0; r < 3; r++) m[r, c] = b[r];
                x[c] = Det(m) / d;
            }
            return x;
        }
    }
}
