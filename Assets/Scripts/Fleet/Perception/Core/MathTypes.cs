using System;
using System.Collections.Generic;

namespace Crane.Fleet.Perception
{
    /// <summary>Double-precision 3-vector (engine independent).</summary>
    public readonly struct Vec3
    {
        public readonly double X, Y, Z;
        public Vec3(double x, double y, double z) { X = x; Y = y; Z = z; }
        public static readonly Vec3 Zero = new Vec3(0, 0, 0);
        public static readonly Vec3 UnitX = new Vec3(1, 0, 0);
        public static readonly Vec3 UnitY = new Vec3(0, 1, 0);
        public static readonly Vec3 UnitZ = new Vec3(0, 0, 1);
        public double this[int i] => i == 0 ? X : i == 1 ? Y : Z;
        public static Vec3 operator +(Vec3 a, Vec3 b) => new Vec3(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        public static Vec3 operator -(Vec3 a, Vec3 b) => new Vec3(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        public static Vec3 operator -(Vec3 a) => new Vec3(-a.X, -a.Y, -a.Z);
        public static Vec3 operator *(Vec3 a, double s) => new Vec3(a.X * s, a.Y * s, a.Z * s);
        public static Vec3 operator *(double s, Vec3 a) => a * s;
        public static Vec3 operator /(Vec3 a, double s) => new Vec3(a.X / s, a.Y / s, a.Z / s);
        public double Dot(Vec3 b) => X * b.X + Y * b.Y + Z * b.Z;
        public Vec3 Cross(Vec3 b) => new Vec3(Y * b.Z - Z * b.Y, Z * b.X - X * b.Z, X * b.Y - Y * b.X);
        public double Norm => Math.Sqrt(Dot(this));
        public Vec3 Normalized() { double n = Norm; return n > 0 ? this / n : this; }
        public override string ToString() => $"({X:G6}, {Y:G6}, {Z:G6})";
    }

    /// <summary>3x3 rotation/linear map stored row-major.</summary>
    public readonly struct Mat3
    {
        public readonly double M00, M01, M02, M10, M11, M12, M20, M21, M22;
        public Mat3(double a, double b, double c, double d, double e, double f, double g, double h, double i)
        { M00 = a; M01 = b; M02 = c; M10 = d; M11 = e; M12 = f; M20 = g; M21 = h; M22 = i; }
        public static readonly Mat3 Identity = new Mat3(1, 0, 0, 0, 1, 0, 0, 0, 1);
        public static Mat3 FromColumns(Vec3 c0, Vec3 c1, Vec3 c2) => new Mat3(c0.X, c1.X, c2.X, c0.Y, c1.Y, c2.Y, c0.Z, c1.Z, c2.Z);
        public Vec3 Col(int j) => j == 0 ? new Vec3(M00, M10, M20) : j == 1 ? new Vec3(M01, M11, M21) : new Vec3(M02, M12, M22);
        public double this[int i, int j]
        {
            get
            {
                switch (i * 3 + j)
                {
                    case 0: return M00; case 1: return M01; case 2: return M02;
                    case 3: return M10; case 4: return M11; case 5: return M12;
                    case 6: return M20; case 7: return M21; default: return M22;
                }
            }
        }
        public Mat3 T => new Mat3(M00, M10, M20, M01, M11, M21, M02, M12, M22);
        public static Vec3 operator *(Mat3 m, Vec3 v) => new Vec3(
            m.M00 * v.X + m.M01 * v.Y + m.M02 * v.Z,
            m.M10 * v.X + m.M11 * v.Y + m.M12 * v.Z,
            m.M20 * v.X + m.M21 * v.Y + m.M22 * v.Z);
        public static Mat3 operator *(Mat3 a, Mat3 b) => new Mat3(
            a.M00 * b.M00 + a.M01 * b.M10 + a.M02 * b.M20, a.M00 * b.M01 + a.M01 * b.M11 + a.M02 * b.M21, a.M00 * b.M02 + a.M01 * b.M12 + a.M02 * b.M22,
            a.M10 * b.M00 + a.M11 * b.M10 + a.M12 * b.M20, a.M10 * b.M01 + a.M11 * b.M11 + a.M12 * b.M21, a.M10 * b.M02 + a.M11 * b.M12 + a.M12 * b.M22,
            a.M20 * b.M00 + a.M21 * b.M10 + a.M22 * b.M20, a.M20 * b.M01 + a.M21 * b.M11 + a.M22 * b.M21, a.M20 * b.M02 + a.M21 * b.M12 + a.M22 * b.M22);
        public static Mat3 operator +(Mat3 a, Mat3 b) => new Mat3(a.M00 + b.M00, a.M01 + b.M01, a.M02 + b.M02, a.M10 + b.M10, a.M11 + b.M11, a.M12 + b.M12, a.M20 + b.M20, a.M21 + b.M21, a.M22 + b.M22);
        public static Mat3 operator *(Mat3 a, double s) => new Mat3(a.M00 * s, a.M01 * s, a.M02 * s, a.M10 * s, a.M11 * s, a.M12 * s, a.M20 * s, a.M21 * s, a.M22 * s);
        public double Trace => M00 + M11 + M22;
        public static Mat3 Skew(Vec3 v) => new Mat3(0, -v.Z, v.Y, v.Z, 0, -v.X, -v.Y, v.X, 0);
        public static Mat3 Rx(double a) { double c = Math.Cos(a), s = Math.Sin(a); return new Mat3(1, 0, 0, 0, c, -s, 0, s, c); }
        public static Mat3 Ry(double a) { double c = Math.Cos(a), s = Math.Sin(a); return new Mat3(c, 0, s, 0, 1, 0, -s, 0, c); }
        public static Mat3 Rz(double a) { double c = Math.Cos(a), s = Math.Sin(a); return new Mat3(c, -s, 0, s, c, 0, 0, 0, 1); }

        /// <summary>Rodrigues: rotation by |w| about w.</summary>
        public static Mat3 Exp(Vec3 w)
        {
            double th = w.Norm;
            Mat3 K = Skew(w);
            if (th < 1e-9) return Identity + K;
            double a = Math.Sin(th) / th, b = (1 - Math.Cos(th)) / (th * th);
            return Identity + K * a + (K * K) * b;
        }
        /// <summary>Rotation vector (valid for angles below ~pi).</summary>
        public Vec3 Log()
        {
            Vec3 v = new Vec3(M21 - M12, M02 - M20, M10 - M01) * 0.5;
            double s = v.Norm, c = (Trace - 1) * 0.5;
            double th = Math.Atan2(s, c);
            if (s < 1e-12) return v;
            return v * (th / s);
        }
        /// <summary>Nearest rotation by polar iteration.</summary>
        public Mat3 Orthonormalized()
        {
            Mat3 r = this;
            for (int i = 0; i < 25; i++) r = (r * 1.5) + ((r * r.T * r) * -0.5);
            return r;
        }
        /// <summary>Rotation taking unit vector a to unit vector b (minimal).</summary>
        public static Mat3 Between(Vec3 a, Vec3 b)
        {
            Vec3 ax = a.Cross(b); double s = ax.Norm, c = a.Dot(b);
            if (s < 1e-12) return Identity;
            return Exp(ax * (Math.Atan2(s, c) / s));
        }
    }

    /// <summary>Rigid transform: x_parent = R * x_child + P.</summary>
    public readonly struct Pose
    {
        public readonly Mat3 R; public readonly Vec3 P;
        public Pose(Mat3 r, Vec3 p) { R = r; P = p; }
        public static readonly Pose Identity = new Pose(Mat3.Identity, Vec3.Zero);
        public static Pose FromYaw(double x, double y, double z, double yaw) => new Pose(Mat3.Rz(yaw), new Vec3(x, y, z));
        public Vec3 Apply(Vec3 v) => R * v + P;
        public Pose Compose(Pose child) => new Pose(R * child.R, R * child.P + P);
        public Pose Inverse() { Mat3 rt = R.T; return new Pose(rt, rt * (-P)); }
        public double Yaw => Math.Atan2(R.M10, R.M00);
        public static double WrapAngle(double a) { a = Math.IEEERemainder(a, 2 * Math.PI); return a; }
        /// <summary>Apply an error twist xi = [dp(parent), dtheta(parent, fixed axes)].</summary>
        public Pose Perturbed(double[] xi) => new Pose(Mat3.Exp(new Vec3(xi[3], xi[4], xi[5])) * R, P + new Vec3(xi[0], xi[1], xi[2]));
        /// <summary>Error twist of this pose relative to a reference such that this = reference.Perturbed(xi).</summary>
        public double[] ErrorFrom(Pose reference)
        {
            Vec3 dp = P - reference.P; Vec3 dth = (R * reference.R.T).Log();
            return new[] { dp.X, dp.Y, dp.Z, dth.X, dth.Y, dth.Z };
        }
    }

    /// <summary>Small dense matrix (covariances, normal equations).</summary>
    public sealed class Matrix
    {
        public readonly int Rows, Cols; public readonly double[] A;
        public Matrix(int r, int c) { Rows = r; Cols = c; A = new double[r * c]; }
        public double this[int i, int j] { get => A[i * Cols + j]; set => A[i * Cols + j] = value; }
        public static Matrix Identity(int n) { var m = new Matrix(n, n); for (int i = 0; i < n; i++) m[i, i] = 1; return m; }
        public static Matrix Diag(params double[] d) { var m = new Matrix(d.Length, d.Length); for (int i = 0; i < d.Length; i++) m[i, i] = d[i]; return m; }
        public static Matrix FromRowMajor(double[] a, int r, int c) { var m = new Matrix(r, c); Array.Copy(a, m.A, r * c); return m; }
        public Matrix T { get { var m = new Matrix(Cols, Rows); for (int i = 0; i < Rows; i++) for (int j = 0; j < Cols; j++) m[j, i] = this[i, j]; return m; } }
        public static Matrix operator *(Matrix a, Matrix b)
        {
            if (a.Cols != b.Rows) throw new ArgumentException("dim");
            var m = new Matrix(a.Rows, b.Cols);
            for (int i = 0; i < a.Rows; i++) for (int k = 0; k < a.Cols; k++) { double v = a[i, k]; if (v == 0) continue; for (int j = 0; j < b.Cols; j++) m[i, j] += v * b[k, j]; }
            return m;
        }
        public static Matrix operator +(Matrix a, Matrix b) { var m = new Matrix(a.Rows, a.Cols); for (int i = 0; i < m.A.Length; i++) m.A[i] = a.A[i] + b.A[i]; return m; }
        public static Matrix operator *(Matrix a, double s) { var m = new Matrix(a.Rows, a.Cols); for (int i = 0; i < m.A.Length; i++) m.A[i] = a.A[i] * s; return m; }
        public Matrix Clone() => FromRowMajor(A, Rows, Cols);
        public double Trace() { double t = 0; for (int i = 0; i < Math.Min(Rows, Cols); i++) t += this[i, i]; return t; }
        /// <summary>Inverse by Gauss-Jordan with partial pivoting; throws if singular.</summary>
        public Matrix Inverse()
        {
            int n = Rows; var a = Clone(); var inv = Identity(n);
            for (int c = 0; c < n; c++)
            {
                int p = c; for (int r = c + 1; r < n; r++) if (Math.Abs(a[r, c]) > Math.Abs(a[p, c])) p = r;
                if (Math.Abs(a[p, c]) < 1e-300) throw new InvalidOperationException("singular matrix");
                if (p != c) for (int j = 0; j < n; j++) { (a[c, j], a[p, j]) = (a[p, j], a[c, j]); (inv[c, j], inv[p, j]) = (inv[p, j], inv[c, j]); }
                double d = 1.0 / a[c, c];
                for (int j = 0; j < n; j++) { a[c, j] *= d; inv[c, j] *= d; }
                for (int r = 0; r < n; r++) if (r != c) { double f = a[r, c]; if (f == 0) continue; for (int j = 0; j < n; j++) { a[r, j] -= f * a[c, j]; inv[r, j] -= f * inv[c, j]; } }
            }
            return inv;
        }
        /// <summary>Solve A x = b (square) via Gauss-Jordan.</summary>
        public double[] Solve(double[] b)
        {
            var inv = Inverse(); var x = new double[Rows];
            for (int i = 0; i < Rows; i++) for (int j = 0; j < Cols; j++) x[i] += inv[i, j] * b[j];
            return x;
        }
        /// <summary>Lower Cholesky factor; tiny negative pivots clipped (covariance sampling).</summary>
        public Matrix Cholesky()
        {
            int n = Rows; var L = new Matrix(n, n);
            for (int i = 0; i < n; i++)
                for (int j = 0; j <= i; j++)
                {
                    double s = this[i, j]; for (int k = 0; k < j; k++) s -= L[i, k] * L[j, k];
                    if (i == j) L[i, i] = Math.Sqrt(Math.Max(s, 1e-30)); else L[i, j] = s / L[j, j];
                }
            return L;
        }
        public double[] Mul(double[] v) { var r = new double[Rows]; for (int i = 0; i < Rows; i++) for (int j = 0; j < Cols; j++) r[i] += this[i, j] * v[j]; return r; }
        /// <summary>x^T A^-1 x (e.g. NEES).</summary>
        public double Mahalanobis(double[] x) { var s = Solve(x); double q = 0; for (int i = 0; i < x.Length; i++) q += x[i] * s[i]; return q; }
        public static Matrix Block6(Mat3 a, Mat3 b, Mat3 c, Mat3 d)
        {
            var m = new Matrix(6, 6);
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) { m[i, j] = a[i, j]; m[i, j + 3] = b[i, j]; m[i + 3, j] = c[i, j]; m[i + 3, j + 3] = d[i, j]; }
            return m;
        }
    }

    /// <summary>Deterministic platform-independent PRNG (SplitMix64 core) with Gaussian draws.</summary>
    public sealed class Rng
    {
        private ulong _s; private bool _have; private double _spare;
        public Rng(ulong seed) { _s = seed; }
        /// <summary>Independent named substream (FNV-1a of the name: string.GetHashCode is randomized per process).</summary>
        public static Rng ForStream(ulong seed, string name)
        {
            ulong h = 14695981039346656037UL;
            foreach (char c in name) { h ^= c; h *= 1099511628211UL; }
            var r = new Rng(seed ^ (h * 0x9E3779B97F4A7C15UL)); r.NextU64(); return r;
        }
        public ulong NextU64()
        {
            _s += 0x9E3779B97F4A7C15UL; ulong z = _s;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL; z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL; return z ^ (z >> 31);
        }
        public double NextDouble() => (NextU64() >> 11) * (1.0 / 9007199254740992.0);
        public double Uniform(double a, double b) => a + (b - a) * NextDouble();
        public bool Bernoulli(double p) => NextDouble() < p;
        public double Gaussian()
        {
            if (_have) { _have = false; return _spare; }
            double u1 = 0; while (u1 <= 1e-300) u1 = NextDouble();
            double u2 = NextDouble(), r = Math.Sqrt(-2 * Math.Log(u1));
            _spare = r * Math.Sin(2 * Math.PI * u2); _have = true; return r * Math.Cos(2 * Math.PI * u2);
        }
        public double[] SampleGaussian(Matrix cov)
        {
            var L = cov.Cholesky(); var z = new double[cov.Rows]; for (int i = 0; i < z.Length; i++) z[i] = Gaussian();
            return L.Mul(z);
        }
    }
}
