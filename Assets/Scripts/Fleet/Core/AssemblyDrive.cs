using System;
using System.Collections.Generic;

namespace Crane.Fleet {
    /// <summary>
    /// Group control of rigidly coupled modules (CRANE_MODULAR_PLAN M2, command side). An assembly is a set of modules with fixed poses in
    /// one assembly frame. A group twist at the assembly origin maps to every driven wheel's speed by ideal rolling (B), and wheel speeds
    /// map back to the group twist by weighted least squares (the pseudo-inverse of A). Nothing here knows about the coupling hardware
    /// (hooks, pins, magnets): a coupling is only a fixed relative pose, so a hook + pin face changes nothing.
    /// <para>Hard kinematic constraints: a conventional wheel forbids motion along its axle at its contact point (no-slip lateral row). The
    /// feasible group twists are the null space of those rows. A "skid" assembly can still be driven outside it, but only through scrub,
    /// which the tractive plant models.</para>
    /// </summary>
    public sealed class AssemblyDrive {
        public sealed class Member {
            public string Id;
            public Pose2 InAssembly;                  // module base_link pose in the assembly frame
            public IList<WheelUnit> Wheels;           // module frame
            public int[] Driven;                      // indices of the driven wheels (command order of that module)
        }

        public readonly List<Member> Members = new List<Member>();
        public double MaxSpeedXy = double.PositiveInfinity, MaxYawRate = double.PositiveInfinity;

        public Member Add(string id, Pose2 inAssembly, IList<WheelUnit> wheels) {
            var d = new List<int>();
            for (int i = 0; i < wheels.Count; i++) if (wheels[i].Driven) d.Add(i);
            var m = new Member { Id = id, InAssembly = inAssembly, Wheels = wheels, Driven = d.ToArray() };
            Members.Add(m);
            return m;
        }

        /// <summary>Contact position and constraint direction of a wheel in the assembly frame.</summary>
        static void WheelInAssembly(Member m, WheelUnit u, out double px, out double py, out double ax, out double ay, out double c) {
            double cy = Math.Cos(m.InAssembly.Yaw), sy = Math.Sin(m.InAssembly.Yaw);
            px = m.InAssembly.X + cy * u.X - sy * u.Y; py = m.InAssembly.Y + sy * u.X + cy * u.Y;
            u.Constraint(out double lax, out double lay, out c);
            ax = cy * lax - sy * lay; ay = sy * lax + cy * lay;
        }

        /// <summary>Row of A for a driven wheel: r c w = row . (vx, vy, wz) (group twist at the assembly origin).</summary>
        static double[] Row(double px, double py, double ax, double ay) => new[] { ax, ay, ay * px - ax * py };

        /// <summary>Ideal-rolling wheel speeds for a group twist, per member in its driven-wheel order.</summary>
        public Dictionary<string, double[]> Inverse(Twist t) {
            var r = new Dictionary<string, double[]>();
            foreach (var m in Members) {
                var w = new double[m.Driven.Length];
                for (int k = 0; k < m.Driven.Length; k++) {
                    var u = m.Wheels[m.Driven[k]];
                    WheelInAssembly(m, u, out double px, out double py, out double ax, out double ay, out double c);
                    var row = Row(px, py, ax, ay);
                    w[k] = (row[0] * t.Vx + row[1] * t.Vy + row[2] * t.Wz) / (u.Radius * c);
                }
                r[m.Id] = w;
            }
            return r;
        }

        /// <summary>Body limits, then one common scale over every driven wheel of the assembly if any exceeds its limit (keeps the direction).</summary>
        public double Saturate(ref Twist t, out Dictionary<string, double[]> wheels) {
            double sp = Math.Sqrt(t.Vx * t.Vx + t.Vy * t.Vy);
            if (sp > MaxSpeedXy) { t.Vx *= MaxSpeedXy / sp; t.Vy *= MaxSpeedXy / sp; }
            if (Math.Abs(t.Wz) > MaxYawRate) t.Wz = Math.Sign(t.Wz) * MaxYawRate;
            wheels = Inverse(t);
            double s = 1;
            foreach (var m in Members)
                for (int k = 0; k < m.Driven.Length; k++) {
                    double lim = m.Wheels[m.Driven[k]].MaxSpeed, w = Math.Abs(wheels[m.Id][k]);
                    if (w > lim) s = Math.Min(s, lim / w);
                }
            if (s < 1) foreach (var kv in wheels) for (int k = 0; k < kv.Value.Length; k++) kv.Value[k] *= s;
            return s;
        }

        /// <summary>Weighted least-squares group twist from wheel speeds (per member, driven order); weight 0 drops a wheel. Directions the
        /// wheels do not observe come out 0 (small ridge).</summary>
        public Twist Forward(Dictionary<string, double[]> w, Dictionary<string, double[]> weight = null) {
            var A = new double[3, 3]; var b = new double[3];
            foreach (var m in Members) {
                if (!w.TryGetValue(m.Id, out var wm)) continue;
                for (int k = 0; k < m.Driven.Length; k++) {
                    double wt = weight != null && weight.TryGetValue(m.Id, out var wg) ? wg[k] : 1;
                    if (wt == 0) continue;
                    var u = m.Wheels[m.Driven[k]];
                    WheelInAssembly(m, u, out double px, out double py, out double ax, out double ay, out double c);
                    var row = Row(px, py, ax, ay);
                    double y = u.Radius * c * wm[k];
                    for (int p = 0; p < 3; p++) { b[p] += wt * row[p] * y; for (int q = 0; q < 3; q++) A[p, q] += wt * row[p] * row[q]; }
                }
            }
            for (int p = 0; p < 3; p++) A[p, p] += 1e-9;
            var x = Lin3.Solve(A, b);
            return x == null ? new Twist() : new Twist(x[0], x[1], x[2]);
        }

        /// <summary>Hard no-slip lateral rows of the conventional wheels: row . twist = 0 for a feasible group twist.</summary>
        public List<double[]> HardLateralRows() {
            var rows = new List<double[]>();
            foreach (var m in Members)
                foreach (var u in m.Wheels) {
                    if (u.Kind != WheelKind.Conventional) continue;
                    WheelInAssembly(m, u, out double px, out double py, out double ax, out double ay, out double c);
                    double lx = -ay, ly = ax;   // axle (lateral) direction
                    rows.Add(Row(px, py, lx, ly));
                }
            return rows;
        }

        /// <summary>Degree of mobility under the hard constraints: 3 - rank(lateral rows), and whether a twist is feasible (|C t| below tol
        /// relative to |t|).</summary>
        public int Mobility(double tol = 1e-9) => 3 - Lin3.Rank(HardLateralRows(), tol);

        public bool Feasible(Twist t, double tol = 1e-6) {
            double n = Math.Sqrt(t.Vx * t.Vx + t.Vy * t.Vy + t.Wz * t.Wz);
            if (n == 0) return true;
            foreach (var r in HardLateralRows()) if (Math.Abs(r[0] * t.Vx + r[1] * t.Vy + r[2] * t.Wz) > tol * n) return false;
            return true;
        }

        /// <summary>Rank of the drive rows A (how many group-twist directions the driven wheels command and observe).</summary>
        public int DriveRank(double tol = 1e-9) {
            var rows = new List<double[]>();
            foreach (var m in Members)
                foreach (int i in m.Driven) {
                    WheelInAssembly(m, m.Wheels[i], out double px, out double py, out double ax, out double ay, out _);
                    rows.Add(Row(px, py, ax, ay));
                }
            return Lin3.Rank(rows, tol);
        }

        /// <summary>Pose of a face of <paramref name="b"/> mated to a face of <paramref name="a"/>: face frames coincident, facing each other
        /// (relative yaw pi). Faces are (x, y, yaw) in their module frame; hardware-agnostic.</summary>
        public static Pose2 MatedPose(Pose2 a, (double x, double y, double yaw) faceA, (double x, double y, double yaw) faceB) {
            // world face A frame, flip by pi, then remove face B's offset in module B
            var fa = Compose(a, new Pose2(faceA.x, faceA.y, faceA.yaw));
            var fbWorld = Compose(fa, new Pose2(0, 0, Math.PI));
            return Compose(fbWorld, Inverse(new Pose2(faceB.x, faceB.y, faceB.yaw)));
        }

        public static Pose2 Compose(Pose2 a, Pose2 b) {
            double c = Math.Cos(a.Yaw), s = Math.Sin(a.Yaw);
            return new Pose2(a.X + c * b.X - s * b.Y, a.Y + s * b.X + c * b.Y, Angles.Wrap(a.Yaw + b.Yaw));
        }

        public static Pose2 Inverse(Pose2 a) {
            double c = Math.Cos(a.Yaw), s = Math.Sin(a.Yaw);
            return new Pose2(-(c * a.X + s * a.Y), -(-s * a.X + c * a.Y), Angles.Wrap(-a.Yaw));
        }
    }

    /// <summary>Small dense 3-column linear algebra (engine-free).</summary>
    public static class Lin3 {
        public static double[] Solve(double[,] A, double[] b) {
            double Det(double[,] m) => m[0, 0] * (m[1, 1] * m[2, 2] - m[1, 2] * m[2, 1]) - m[0, 1] * (m[1, 0] * m[2, 2] - m[1, 2] * m[2, 0]) + m[0, 2] * (m[1, 0] * m[2, 1] - m[1, 1] * m[2, 0]);
            double d = Det(A);
            if (Math.Abs(d) < 1e-30) return null;
            var x = new double[3];
            for (int c = 0; c < 3; c++) { var m = (double[,])A.Clone(); for (int r = 0; r < 3; r++) m[r, c] = b[r]; x[c] = Det(m) / d; }
            return x;
        }

        /// <summary>Rank of a set of 3-vectors (Gram-Schmidt with a relative tolerance).</summary>
        public static int Rank(IList<double[]> rows, double tol = 1e-9) {
            var basis = new List<double[]>();
            double scale = 0;
            foreach (var r in rows) scale = Math.Max(scale, Math.Sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]));
            if (scale == 0) return 0;
            foreach (var r in rows) {
                var v = (double[])r.Clone();
                foreach (var e in basis) { double d = v[0] * e[0] + v[1] * e[1] + v[2] * e[2]; for (int i = 0; i < 3; i++) v[i] -= d * e[i]; }
                double n = Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
                if (n > Math.Max(tol, 1e-12) * scale) { for (int i = 0; i < 3; i++) v[i] /= n; basis.Add(v); if (basis.Count == 3) break; }
            }
            return basis.Count;
        }
    }
}
