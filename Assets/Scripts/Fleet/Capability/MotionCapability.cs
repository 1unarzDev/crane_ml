using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace Crane.Fleet
{
    /// <summary>One kinematic constraint row (acting on the twist xi = [vx, vy, wz]) with the wheel that contributes it.</summary>
    public sealed class CapConstraintRow
    {
        public string WheelId;
        /// <summary>lateral | locked | locked_rolling</summary>
        public string Role;
        public double[] Row;
        public double NormalLoad, LateralFriction;
    }

    /// <summary>
    /// MOTION part of the capability descriptor for one rigid assembly. All twists are [vx, vy, wz] in the assembly frame at the
    /// assembly origin. Wheel speeds follow phidot = A xi (rows only for driven, live wheels); xi = B phidot with B = T pinv(A T), T the commandable basis.
    /// </summary>
    public sealed class MotionCapability
    {
        public string AssemblyId;
        public string[] DrivenWheelIds;      // order of rows of A
        public double[][] A;                 // m x 3, wheel speed (rad/s) per unit twist
        public double[] SpeedLimit;          // m, effective rad/s after health gains
        public double[][] B;                 // 3 x m, T pinv(A T) with T the commandable basis (matches capplan Q pinv(J Q))
        public CapConstraintRow[] Hard;      // hard constraints C xi = 0
        public CapConstraintRow[] Soft;      // skid-steer lateral rows (Coulomb scrub), not part of the hard set
        public int RankC;                    // rank of the hard constraint rows
        public int Mobility;                 // delta_m = 3 - RankC (Campion et al. 1996)
        public int ActuatedRank;             // dimension of the commandable twist subspace inside null(C)
        public int Uncontrolled => Mobility - ActuatedRank; // free dof whose motion is set by dynamics, not by wheel commands
        public bool Holonomic => ActuatedRank == 3;
        public double[,] ProjectorMobility;  // 3x3 orthogonal projector onto null(C)
        public double[,] ProjectorControlled;// 3x3 orthogonal projector onto the commandable subspace
        public double[][] ControlledBasis;   // d x 3 orthonormal rows
        public double[][] Vertices;          // vertices of the feasible twist polytope (twists), inside the controlled subspace
        public string[] Warnings = new string[0];

        // ------------------------------------------------------------------ queries

        /// <summary>Largest s >= 0 with s*u feasible (u need not be unit). 0 if u leaves the commandable subspace; +inf if u = 0.</summary>
        public double MaxAlong(double[] u)
        {
            double un = Math.Sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
            if (un == 0) return double.PositiveInfinity;
            var pu = Project(ProjectorControlled, u);
            double res = Math.Sqrt(Sq(u[0] - pu[0]) + Sq(u[1] - pu[1]) + Sq(u[2] - pu[2]));
            if (res > 1e-9 * un) return 0.0;
            double s = double.PositiveInfinity;
            for (int i = 0; i < A.Length; i++)
            {
                double au = Math.Abs(A[i][0] * u[0] + A[i][1] * u[1] + A[i][2] * u[2]);
                if (au > 1e-12) s = Math.Min(s, SpeedLimit[i] / au);
            }
            return s;
        }

        /// <summary>Support function max over the polytope of u . xi.</summary>
        public double Support(double[] u)
        {
            double h = 0;
            foreach (var v in Vertices) h = Math.Max(h, v[0] * u[0] + v[1] * u[1] + v[2] * u[2]);
            return h;
        }

        /// <summary>Pure-axis ray bounds: +vx, -vx, +vy, -vy, +wz, -wz.</summary>
        public double[] AxisBounds()
        {
            return new[]
            {
                MaxAlong(new[] {1.0, 0, 0}), MaxAlong(new[] {-1.0, 0, 0}),
                MaxAlong(new[] {0, 1.0, 0}), MaxAlong(new[] {0, -1.0, 0}),
                MaxAlong(new[] {0, 0, 1.0}), MaxAlong(new[] {0, 0, -1.0}),
            };
        }

        public double[] WheelSpeeds(double[] xi)
        {
            var w = new double[A.Length];
            for (int i = 0; i < A.Length; i++) w[i] = A[i][0] * xi[0] + A[i][1] * xi[1] + A[i][2] * xi[2];
            return w;
        }

        /// <summary>Twist realised by a wheel-speed vector (order of DrivenWheelIds): xi = B phidot.</summary>
        public double[] TwistFromWheelSpeeds(double[] phidot)
        {
            var xi = new double[3];
            for (int r = 0; r < 3; r++) for (int i = 0; i < phidot.Length; i++) xi[r] += B[r][i] * phidot[i];
            return xi;
        }

        /// <summary>Norm of the hard-constraint residual C xi (0 means kinematically admissible).</summary>
        public double HardResidual(double[] xi)
        {
            double s = 0;
            foreach (var c in Hard) s += Sq(c.Row[0] * xi[0] + c.Row[1] * xi[1] + c.Row[2] * xi[2]);
            return Math.Sqrt(s);
        }

        /// <summary>True if xi satisfies the hard constraints, lies in the commandable subspace and respects every wheel-speed limit.</summary>
        public bool Feasible(double[] xi, double tol = 1e-9)
        {
            if (HardResidual(xi) > tol) return false;
            var pc = Project(ProjectorControlled, xi);
            if (Math.Sqrt(Sq(xi[0] - pc[0]) + Sq(xi[1] - pc[1]) + Sq(xi[2] - pc[2])) > tol) return false;
            for (int i = 0; i < A.Length; i++)
                if (Math.Abs(A[i][0] * xi[0] + A[i][1] * xi[1] + A[i][2] * xi[2]) > SpeedLimit[i] + tol) return false;
            return true;
        }

        /// <summary>Coulomb scrub wrench [Fx, Fy, Mz] (N, N, Nm; assembly frame, about the origin) resisting twist xi through the soft
        /// (skid) lateral rows. Smoothed with a lateral-speed scale (m/s) so the function is continuous.</summary>
        public double[] ScrubWrench(double[] xi, double smoothing = 1e-3)
        {
            var w = new double[3];
            foreach (var s in Soft)
            {
                double vlat = s.Row[0] * xi[0] + s.Row[1] * xi[1] + s.Row[2] * xi[2];
                double f = -s.NormalLoad * s.LateralFriction * Math.Tanh(vlat / smoothing);
                // row = n^T [I | perp(p)], so the wrench of a lateral force f along n at p is f * row
                w[0] += f * s.Row[0]; w[1] += f * s.Row[1]; w[2] += f * s.Row[2];
            }
            return w;
        }

        // ------------------------------------------------------------------ serialisation

        /// <summary>JSON for the "motion" field of the runtime capability description (descriptor v0.1). Provenance: analytic.</summary>
        public string ToJson()
        {
            var sb = new StringBuilder();
            var ic = CultureInfo.InvariantCulture;
            sb.Append("{\"assembly\":\"").Append(AssemblyId).Append("\",\"provenance\":\"analytic\"");
            sb.Append(",\"degree_of_mobility\":").Append(Mobility);
            sb.Append(",\"rank_hard_constraints\":").Append(RankC);
            sb.Append(",\"actuated_rank\":").Append(ActuatedRank);
            sb.Append(",\"uncontrolled_dof\":").Append(Uncontrolled);
            sb.Append(",\"holonomic\":").Append(Holonomic ? "true" : "false");
            sb.Append(",\"axis_bounds\":{");
            var ab = AxisBounds();
            string[] names = { "vx_pos", "vx_neg", "vy_pos", "vy_neg", "wz_pos", "wz_neg" };
            for (int i = 0; i < 6; i++)
            {
                if (i > 0) sb.Append(',');
                sb.Append('"').Append(names[i]).Append("\":").Append(Num(ab[i]));
            }
            sb.Append("},\"driven_wheels\":[");
            for (int i = 0; i < DrivenWheelIds.Length; i++)
            {
                if (i > 0) sb.Append(',');
                sb.Append("{\"id\":\"").Append(DrivenWheelIds[i]).Append("\",\"limit_radps\":").Append(Num(SpeedLimit[i]))
                  .Append(",\"row\":[").Append(Num(A[i][0])).Append(',').Append(Num(A[i][1])).Append(',').Append(Num(A[i][2])).Append("]}");
            }
            sb.Append("],\"hard_constraints\":[");
            for (int i = 0; i < Hard.Length; i++)
            {
                if (i > 0) sb.Append(',');
                sb.Append("{\"wheel\":\"").Append(Hard[i].WheelId).Append("\",\"role\":\"").Append(Hard[i].Role).Append("\"}");
            }
            sb.Append("],\"soft_constraints\":[");
            for (int i = 0; i < Soft.Length; i++)
            {
                if (i > 0) sb.Append(',');
                sb.Append("{\"wheel\":\"").Append(Soft[i].WheelId).Append("\",\"role\":\"").Append(Soft[i].Role).Append("\"}");
            }
            sb.Append("],\"vertices\":[");
            for (int i = 0; i < Vertices.Length; i++)
            {
                if (i > 0) sb.Append(',');
                sb.Append('[').Append(Num(Vertices[i][0])).Append(',').Append(Num(Vertices[i][1])).Append(',').Append(Num(Vertices[i][2])).Append(']');
            }
            sb.Append("]}");
            return sb.ToString();
            string Num(double d) => double.IsInfinity(d) ? "null" : d.ToString("R", ic);
        }

        private static double Sq(double x) => x * x;

        private static double[] Project(double[,] p, double[] u)
        {
            var r = new double[3];
            for (int i = 0; i < 3; i++) r[i] = p[i, 0] * u[0] + p[i, 1] * u[1] + p[i, 2] * u[2];
            return r;
        }
    }
}
