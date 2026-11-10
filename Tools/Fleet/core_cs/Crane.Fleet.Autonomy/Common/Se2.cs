using System;

namespace Crane.Fleet
{
    /// <summary>Planar body twist: vx forward, vy left (m/s), wz CCW (rad/s), expressed in the body (or assembly) frame.
    /// Named Twist2 so it does not collide with CRANE's Crane.Fleet.Twist when both live in one Unity assembly.</summary>
    public readonly struct Twist2
    {
        public readonly double Vx, Vy, Wz;
        public Twist2(double vx, double vy, double wz) { Vx = vx; Vy = vy; Wz = wz; }
        public static readonly Twist2 Zero = new Twist2(0, 0, 0);
        public static Twist2 operator +(Twist2 a, Twist2 b) => new Twist2(a.Vx + b.Vx, a.Vy + b.Vy, a.Wz + b.Wz);
        public static Twist2 operator -(Twist2 a, Twist2 b) => new Twist2(a.Vx - b.Vx, a.Vy - b.Vy, a.Wz - b.Wz);
        public static Twist2 operator *(Twist2 a, double s) => new Twist2(a.Vx * s, a.Vy * s, a.Wz * s);
        public double this[int i] => i == 0 ? Vx : i == 1 ? Vy : Wz;
        public double LinearSpeed => Math.Sqrt(Vx * Vx + Vy * Vy);
        /// <summary>Euclidean norm with yaw weighted by a characteristic radius (m) so the three components are commensurate.</summary>
        public double Norm(double radius = 0.3) => Math.Sqrt(Vx * Vx + Vy * Vy + radius * radius * Wz * Wz);
        public override string ToString() => $"({Vx:0.####}, {Vy:0.####}, {Wz:0.####})";
    }

    /// <summary>SE(2) helpers on Pose2 (Crane.Fleet.Docking geometry). Angles are wrapped to (-pi, pi].</summary>
    public static class Se2
    {
        public static double Wrap(double a)
        {
            a = Math.IEEERemainder(a, 2 * Math.PI);
            if (a <= -Math.PI) a += 2 * Math.PI;
            return a;
        }

        public static Pose2 Normalize(Pose2 p) => new Pose2(p.X, p.Y, Wrap(p.Yaw));

        /// <summary>a^-1 * b (b expressed in a's frame), yaw wrapped.</summary>
        public static Pose2 Between(Pose2 a, Pose2 b)
        {
            double c = Math.Cos(a.Yaw), s = Math.Sin(a.Yaw), dx = b.X - a.X, dy = b.Y - a.Y;
            return new Pose2(c * dx + s * dy, -s * dx + c * dy, Wrap(b.Yaw - a.Yaw));
        }

        public static Pose2 Compose(Pose2 a, Pose2 b)
        {
            double c = Math.Cos(a.Yaw), s = Math.Sin(a.Yaw);
            return new Pose2(a.X + c * b.X - s * b.Y, a.Y + s * b.X + c * b.Y, Wrap(a.Yaw + b.Yaw));
        }

        public static Pose2 Inverse(Pose2 a)
        {
            double c = Math.Cos(a.Yaw), s = Math.Sin(a.Yaw);
            return new Pose2(-(c * a.X + s * a.Y), -(-s * a.X + c * a.Y), Wrap(-a.Yaw));
        }

        /// <summary>Exact pose increment of a constant body twist applied for dt (exponential map of se(2)).</summary>
        public static Pose2 Exp(Twist2 t, double dt)
        {
            double th = t.Wz * dt, vx = t.Vx * dt, vy = t.Vy * dt;
            double a, b;
            if (Math.Abs(th) < 1e-9) { a = 1 - th * th / 6; b = th / 2; }
            else { a = Math.Sin(th) / th; b = (1 - Math.Cos(th)) / th; }
            return new Pose2(a * vx - b * vy, b * vx + a * vy, th);
        }

        /// <summary>Constant body twist (per unit time) that moves from identity to <paramref name="d"/> in unit time (log map).</summary>
        public static Twist2 Log(Pose2 d)
        {
            double th = Wrap(d.Yaw), a, b;
            if (Math.Abs(th) < 1e-9) { a = 1 - th * th / 6; b = th / 2; }
            else { a = Math.Sin(th) / th; b = (1 - Math.Cos(th)) / th; }
            double det = a * a + b * b;
            return new Twist2((a * d.X + b * d.Y) / det, (-b * d.X + a * d.Y) / det, th);
        }

        public static Pose2 Advance(Pose2 p, Twist2 t, double dt) => Compose(p, Exp(t, dt));

        public static double Distance(Pose2 a, Pose2 b) => Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));

        /// <summary>Planar part of a Perception pose (x, y, yaw).</summary>
        public static Pose2 FromPose(Perception.Pose p) => new Pose2(p.P.X, p.P.Y, p.Yaw);

        public static Perception.Pose ToPose(Pose2 p, double z = 0) => Perception.Pose.FromYaw(p.X, p.Y, z, p.Yaw);

        /// <summary>
        /// Jacobian (3x3, row-major into <paramref name="j"/>) of c = a*b with respect to an additive parent-axes perturbation of a
        /// (p_a += dp, yaw_a += dth), i.e. the Perception error convention restricted to the plane: dc = J da.
        /// </summary>
        public static void JacComposeLeft(Pose2 a, Pose2 b, double[] j)
        {
            double c = Math.Cos(a.Yaw), s = Math.Sin(a.Yaw);
            double rx = c * b.X - s * b.Y, ry = s * b.X + c * b.Y;   // R_a b.p
            j[0] = 1; j[1] = 0; j[2] = -ry;
            j[3] = 0; j[4] = 1; j[5] = rx;
            j[6] = 0; j[7] = 0; j[8] = 1;
        }

        /// <summary>Jacobian of c = a*b with respect to an additive perturbation of b expressed in a's axes (b's parent): dc = J db.</summary>
        public static void JacComposeRight(Pose2 a, double[] j)
        {
            double c = Math.Cos(a.Yaw), s = Math.Sin(a.Yaw);
            j[0] = c; j[1] = -s; j[2] = 0;
            j[3] = s; j[4] = c; j[5] = 0;
            j[6] = 0; j[7] = 0; j[8] = 1;
        }
    }

    /// <summary>Allocation-free dense helpers on row-major double[] buffers. Callers own all storage.</summary>
    public static class Lin
    {
        /// <summary>C = A (r x k) * B (k x c).</summary>
        public static void Mul(double[] a, double[] b, double[] c, int r, int k, int cc)
        {
            for (int i = 0; i < r; i++)
                for (int j = 0; j < cc; j++)
                {
                    double s = 0;
                    for (int t = 0; t < k; t++) s += a[i * k + t] * b[t * cc + j];
                    c[i * cc + j] = s;
                }
        }

        /// <summary>C = A (r x k) * B^T (B is c x k).</summary>
        public static void MulBt(double[] a, double[] b, double[] c, int r, int k, int cc)
        {
            for (int i = 0; i < r; i++)
                for (int j = 0; j < cc; j++)
                {
                    double s = 0;
                    for (int t = 0; t < k; t++) s += a[i * k + t] * b[j * k + t];
                    c[i * cc + j] = s;
                }
        }

        /// <summary>P = J P J^T for square n, using scratch (n*n).</summary>
        public static void Congruence(double[] j, double[] p, double[] scratch, int n)
        {
            Mul(j, p, scratch, n, n, n);
            MulBt(scratch, j, p, n, n, n);
        }

        public static void Symmetrize(double[] p, int n)
        {
            for (int i = 0; i < n; i++) for (int j = i + 1; j < n; j++) { double m = 0.5 * (p[i * n + j] + p[j * n + i]); p[i * n + j] = m; p[j * n + i] = m; }
        }

        /// <summary>In-place inverse of a small SPD-ish matrix by Gauss-Jordan with partial pivoting. Returns false if singular.</summary>
        public static bool Invert(double[] a, double[] inv, int n, double[] work)
        {
            // work: n*2n
            int w = 2 * n;
            for (int i = 0; i < n; i++)
                for (int j = 0; j < w; j++)
                    work[i * w + j] = j < n ? a[i * n + j] : (j - n == i ? 1 : 0);
            for (int c = 0; c < n; c++)
            {
                int piv = c; double best = Math.Abs(work[c * w + c]);
                for (int r = c + 1; r < n; r++) { double v = Math.Abs(work[r * w + c]); if (v > best) { best = v; piv = r; } }
                if (best < 1e-300) return false;
                if (piv != c) for (int j = 0; j < w; j++) { double t = work[c * w + j]; work[c * w + j] = work[piv * w + j]; work[piv * w + j] = t; }
                double d = work[c * w + c];
                for (int j = 0; j < w; j++) work[c * w + j] /= d;
                for (int r = 0; r < n; r++)
                {
                    if (r == c) continue;
                    double f = work[r * w + c];
                    if (f == 0) continue;
                    for (int j = 0; j < w; j++) work[r * w + j] -= f * work[c * w + j];
                }
            }
            for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) inv[i * n + j] = work[i * w + n + j];
            return true;
        }

        /// <summary>Quadratic form x^T S^-1 x for a 3x3 S (closed form). Returns +inf if singular.</summary>
        public static double Mahalanobis3(double[] s, double x0, double x1, double x2)
        {
            double a = s[0], b = s[1], c = s[2], d = s[3], e = s[4], f = s[5], g = s[6], h = s[7], i = s[8];
            double A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
            double det = a * A + b * B + c * C;
            if (Math.Abs(det) < 1e-300) return double.PositiveInfinity;
            double D = -(b * i - c * h), E = a * i - c * g, F = -(a * h - b * g);
            double G = b * f - c * e, H = -(a * f - c * d), I = a * e - b * d;
            double y0 = (A * x0 + D * x1 + G * x2) / det, y1 = (B * x0 + E * x1 + H * x2) / det, y2 = (C * x0 + F * x1 + I * x2) / det;
            return x0 * y0 + x1 * y1 + x2 * y2;
        }
    }
}
