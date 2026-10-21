using System;
using System.Collections.Generic;

namespace Sim.Physics.LandTerrain
{
    // Pure-math core of the flagged slope-aware traction law (--crane-terrain-traction=slope-v1).
    // No UnityEngine types: EditMode tests compare it directly with Tools/Terrain/terrain_reference.py.
    //
    // Body frame: x forward, y left, z up (the plane normal). Angles in radians, SI units.
    // Tire model: isotropic brush model (H. B. Pacejka, "Tire and Vehicle Dynamics", 3rd ed., ch. 3.3):
    //   F = mu N (3u - 3u^2 + u^3) for u < 1, mu N otherwise, with u = |K sigma| / (3 mu N),
    //   applied along the slip vector. Cone |F| <= mu N holds by construction.
    // Load distribution: equal-stiffness support springs, i.e. the minimum-norm solution of the
    // three planar equilibrium equations (N_i = a + b x_i + c y_i), with an active set for lift-off.
    // This is a model, not a calibrated vehicle: every coefficient is a prior until measured.
    public struct V3
    {
        public double x, y, z;
        public V3(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }
    }

    public struct Support
    {
        public double x, y;          // contact point in the body plane (m)
        public bool driven;          // powered wheel with a brush tire; false = passive caster
    }

    public sealed class VehicleGeometry
    {
        public Support[] supports;
        public V3 cg;                // body-frame CG: x, y offsets and z = height above the contact plane
        public double mass;
        public double wheelRadius = 0.033;
    }

    public sealed class TractionParams
    {
        public double mu = 0.7;                    // tire-surface friction coefficient (cone slope)
        public double longitudinalStiffness = 12.0; // C_kappa: dimensionless force per unit normal load per unit slip
        public double lateralStiffness = 8.0;       // C_alpha
        public double rollingResistance = 0.02;     // C_rr; applied whether or not throttle is on
        public double casterFriction = 0.02;        // Coulomb-like caster drag coefficient
        public double wheelTorqueLimit = double.PositiveInfinity; // per driven wheel (N m)
        public double velocityEpsilon = 0.05;       // m/s: slip regularisation and drag smoothing
        public double loadFilterSeconds = 0.0;      // 0 disables load filtering
        public double gravity = 9.81;
    }

    public struct WheelOutput
    {
        public double normalLoad, fx, fy, rollingForce, slipRatio, slipAngle;
    }

    public static class SlopeTractionMath
    {
        // Gravity acceleration in the body frame for a plane of inclination `slope` whose uphill
        // direction is `uphillHeading` radians from body +x (0 = nose up, pi/2 = left side up).
        public static V3 GravityInBody(double slope, double uphillHeading, double g = 9.81)
        {
            return new V3(-g * Math.Sin(slope) * Math.Cos(uphillHeading),
                          -g * Math.Sin(slope) * Math.Sin(uphillHeading),
                          -g * Math.Cos(slope));
        }

        // Quasi-static support loads. gBody includes only gravity; (ax, ay) is the body acceleration
        // along the plane (D'Alembert). Returns false if fewer than three supports can carry load
        // (tipped or collinear); loads are then a best effort with the rest zero.
        public static bool SolveLoads(VehicleGeometry geo, V3 gBody, double ax, double ay, double[] loads)
        {
            int n = geo.supports.Length;
            double m = geo.mass;
            double fex = m * (gBody.x - ax), fey = m * (gBody.y - ay), fez = m * gBody.z;
            double total = -fez;
            double sumX = geo.cg.z * fex - geo.cg.x * fez;   // sum N_i x_i
            double sumY = geo.cg.y * fez * -1.0 + geo.cg.z * fey; // sum N_i y_i  (see derivation in the docs)
            sumY = geo.cg.z * fey - geo.cg.y * fez;
            var active = new bool[n];
            for (int i = 0; i < n; i++) { active[i] = true; loads[i] = 0; }
            for (int pass = 0; pass < n; pass++)
            {
                int cnt = 0;
                for (int i = 0; i < n; i++) if (active[i]) cnt++;
                if (cnt < 3) return false;
                if (!MinNorm(geo, active, total, sumX, sumY, loads)) return false;
                bool clipped = false;
                for (int i = 0; i < n; i++)
                    if (active[i] && loads[i] < 0) { active[i] = false; loads[i] = 0; clipped = true; }
                if (!clipped) return true;
            }
            return false;
        }

        static bool MinNorm(VehicleGeometry geo, bool[] active, double total, double sumX, double sumY, double[] loads)
        {
            // N = A^T lambda, (A A^T) lambda = b, rows of A = [1], [x], [y].
            var s = new double[3, 3];
            for (int i = 0; i < geo.supports.Length; i++)
            {
                if (!active[i]) continue;
                double[] r = { 1.0, geo.supports[i].x, geo.supports[i].y };
                for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++) s[a, b] += r[a] * r[b];
            }
            double[] rhs = { total, sumX, sumY };
            double[] lam = Solve3(s, rhs);
            if (lam == null) return false;
            for (int i = 0; i < geo.supports.Length; i++)
                loads[i] = active[i] ? lam[0] + lam[1] * geo.supports[i].x + lam[2] * geo.supports[i].y : 0.0;
            return true;
        }

        static double[] Solve3(double[,] a, double[] b)
        {
            var m = new double[3, 4];
            for (int i = 0; i < 3; i++) { for (int j = 0; j < 3; j++) m[i, j] = a[i, j]; m[i, 3] = b[i]; }
            for (int c = 0; c < 3; c++)
            {
                int p = c;
                for (int r = c + 1; r < 3; r++) if (Math.Abs(m[r, c]) > Math.Abs(m[p, c])) p = r;
                if (Math.Abs(m[p, c]) < 1e-12) return null;
                if (p != c) for (int j = 0; j < 4; j++) { double t = m[c, j]; m[c, j] = m[p, j]; m[p, j] = t; }
                for (int r = 0; r < 3; r++)
                {
                    if (r == c) continue;
                    double f = m[r, c] / m[c, c];
                    for (int j = c; j < 4; j++) m[r, j] -= f * m[c, j];
                }
            }
            return new[] { m[0, 3] / m[0, 0], m[1, 3] / m[1, 1], m[2, 3] / m[2, 2] };
        }

        // Brush-model force magnitude as a fraction of mu N for a linear force magnitude `linear` (N).
        public static double BrushMagnitude(double linear, double muN)
        {
            if (muN <= 0) return 0;
            double u = linear / (3.0 * muN);
            return u >= 1.0 ? muN : muN * (3 * u - 3 * u * u + u * u * u);
        }

        // Smooth sign with a linear region |v| < eps (Coulomb with regularisation).
        public static double SmoothSign(double v, double eps)
        {
            if (eps <= 0) return Math.Sign(v);
            double r = v / eps;
            return r > 1 ? 1 : r < -1 ? -1 : r;
        }

        // Driven wheel. (vx, vy) = contact-point velocity in the wheel frame, surfaceSpeed = omega r.
        public static WheelOutput DrivenWheel(double n, double vx, double vy, double surfaceSpeed, TractionParams p)
        {
            var o = new WheelOutput { normalLoad = n };
            if (n <= 0) return o;
            double den = Math.Max(Math.Max(Math.Abs(vx), Math.Abs(surfaceSpeed)), p.velocityEpsilon);
            double kappa = (surfaceSpeed - vx) / den;
            double alpha = Math.Atan2(vy, Math.Max(Math.Abs(vx), p.velocityEpsilon));
            double qx = p.longitudinalStiffness * n * kappa;
            double qy = -p.lateralStiffness * n * Math.Tan(alpha);
            double q = Math.Sqrt(qx * qx + qy * qy);
            double f = BrushMagnitude(q, p.mu * n);
            if (q > 1e-12) { o.fx = f * qx / q; o.fy = f * qy / q; }
            o.slipRatio = kappa; o.slipAngle = alpha;
            o.rollingForce = -p.rollingResistance * n * SmoothSign(vx, p.velocityEpsilon);
            return o;
        }

        // Passive caster: planar Coulomb-like drag opposing velocity, bounded by casterFriction N.
        public static void CasterForce(double n, double vx, double vy, TractionParams p, out double fx, out double fy)
        {
            fx = fy = 0;
            if (n <= 0) return;
            double speed = Math.Sqrt(vx * vx + vy * vy);
            if (speed < 1e-12) return;
            double k = SmoothSign(speed, p.velocityEpsilon);
            fx = -p.casterFriction * n * k * vx / speed;
            fy = -p.casterFriction * n * k * vy / speed;
        }

        // Tractive force a driven wheel can deliver in steady state: min(torque/r, mu N) (cone and actuator).
        public static double SteadyTraction(double n, TractionParams p, double radius)
        {
            return Math.Max(0, Math.Min(p.wheelTorqueLimit / radius, p.mu * n));
        }

        // Steepest uphill (heading 0) steady climb angle: the sum over driven wheels of
        // min(T/r, mu N_i) must cover W (sin + Crr cos). Bisection on the body-frame load distribution.
        public static double MaxClimbAngle(VehicleGeometry geo, TractionParams p, Func<Support, bool> drivenFilter = null)
        {
            var loads = new double[geo.supports.Length];
            double W = geo.mass * p.gravity;
            Func<double, double> margin = th =>
            {
                SolveLoads(geo, GravityInBody(th, 0, p.gravity), 0, 0, loads);
                double cap = 0;
                for (int i = 0; i < loads.Length; i++)
                {
                    var s = geo.supports[i];
                    if (!s.driven || (drivenFilter != null && !drivenFilter(s))) continue;
                    cap += SteadyTraction(loads[i], p, geo.wheelRadius);
                }
                return cap - W * (Math.Sin(th) + p.rollingResistance * Math.Cos(th));
            };
            if (margin(0) <= 0) return 0;
            double lo = 0, hi = Math.PI / 2 - 1e-9;
            if (margin(hi) >= 0) return hi;
            for (int k = 0; k < 200; k++)
            {
                double mid = 0.5 * (lo + hi);
                if (margin(mid) > 0) lo = mid; else hi = mid;
            }
            return 0.5 * (lo + hi);
        }
    }

    public struct BodyMotion
    {
        public double vx, vy, wz;     // planar velocity and yaw rate in the body frame
        public double ax, ay;         // planar body acceleration (for load transfer)
    }

    public struct VehicleForces
    {
        public double[] loads;        // per support (N)
        public double[] fx, fy;       // per support force in the body frame (N)
        public double totalFx, totalFy, yawMoment;
        public bool tipped;           // fewer than three supports carrying load
    }

    // Stateful wrapper: owns the filtered load state so Reset() restores a clean episode start.
    public sealed class SlopeTractionModel
    {
        readonly VehicleGeometry geo;
        readonly TractionParams p;
        double[] filtered;
        bool primed;

        public SlopeTractionModel(VehicleGeometry geometry, TractionParams parameters)
        {
            geo = geometry; p = parameters;
            filtered = new double[geo.supports.Length];
        }

        public void Reset()
        {
            Array.Clear(filtered, 0, filtered.Length);
            primed = false;
        }

        // wheelSurfaceSpeeds: commanded omega r per support (ignored for casters).
        public VehicleForces Step(V3 gBody, BodyMotion m, double[] wheelSurfaceSpeeds, double dt)
        {
            int n = geo.supports.Length;
            var raw = new double[n];
            var f = new VehicleForces { loads = new double[n], fx = new double[n], fy = new double[n] };
            f.tipped = !SlopeTractionMath.SolveLoads(geo, gBody, m.ax, m.ay, raw);
            if (!primed || p.loadFilterSeconds <= 0) { Array.Copy(raw, filtered, n); primed = true; }
            else
            {
                double a = Math.Min(1.0, dt / (p.loadFilterSeconds + dt));
                for (int i = 0; i < n; i++) filtered[i] += a * (raw[i] - filtered[i]);
            }
            for (int i = 0; i < n; i++)
            {
                var s = geo.supports[i];
                double N = Math.Max(0, filtered[i]);
                f.loads[i] = N;
                double vx = m.vx - m.wz * s.y, vy = m.vy + m.wz * s.x;
                double fx, fy;
                if (s.driven)
                {
                    var w = SlopeTractionMath.DrivenWheel(N, vx, vy, wheelSurfaceSpeeds[i], p);
                    fx = w.fx; fy = w.fy;
                    double cap = p.wheelTorqueLimit / geo.wheelRadius;
                    if (Math.Abs(fx) > cap) fx = Math.Sign(fx) * cap;   // actuator limit on the tractive force
                    fx += w.rollingForce;                                // rolling resistance is outside the cone
                }
                else
                {
                    SlopeTractionMath.CasterForce(N, vx, vy, p, out fx, out fy);
                }
                f.fx[i] = fx; f.fy[i] = fy;
                f.totalFx += fx; f.totalFy += fy;
                f.yawMoment += s.x * fy - s.y * fx;
            }
            return f;
        }
    }
}
