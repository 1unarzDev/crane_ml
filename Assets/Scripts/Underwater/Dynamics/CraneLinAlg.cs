using System;

namespace Sim.Underwater {
    /// <summary>
    /// Minimal dense linear algebra on double[,] / double[] for the Fossen core.
    /// No UnityEngine dependency. Quaternions are [w, x, y, z] (same as fossen6dof.py).
    /// </summary>
    public static class CraneLinAlg {
        public static double[,] Zeros(int r, int c) => new double[r, c];

        public static double[,] Identity(int n) {
            var m = new double[n, n];
            for (int i = 0; i < n; i++) m[i, i] = 1.0;
            return m;
        }

        public static double[,] Diag(params double[] d) {
            var m = new double[d.Length, d.Length];
            for (int i = 0; i < d.Length; i++) m[i, i] = d[i];
            return m;
        }

        public static double[,] Mul(double[,] a, double[,] b) {
            int n = a.GetLength(0), k = a.GetLength(1), m = b.GetLength(1);
            if (k != b.GetLength(0)) throw new ArgumentException("shape mismatch");
            var r = new double[n, m];
            for (int i = 0; i < n; i++)
                for (int j = 0; j < m; j++) {
                    double s = 0.0;
                    for (int l = 0; l < k; l++) s += a[i, l] * b[l, j];
                    r[i, j] = s;
                }
            return r;
        }

        public static double[] Mul(double[,] a, double[] x) {
            int n = a.GetLength(0), k = a.GetLength(1);
            if (k != x.Length) throw new ArgumentException("shape mismatch");
            var r = new double[n];
            for (int i = 0; i < n; i++) {
                double s = 0.0;
                for (int l = 0; l < k; l++) s += a[i, l] * x[l];
                r[i] = s;
            }
            return r;
        }

        public static double[,] Transpose(double[,] a) {
            int n = a.GetLength(0), m = a.GetLength(1);
            var r = new double[m, n];
            for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) r[j, i] = a[i, j];
            return r;
        }

        public static double[,] Add(double[,] a, double[,] b) {
            int n = a.GetLength(0), m = a.GetLength(1);
            var r = new double[n, m];
            for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) r[i, j] = a[i, j] + b[i, j];
            return r;
        }

        public static double[,] Scale(double[,] a, double s) {
            int n = a.GetLength(0), m = a.GetLength(1);
            var r = new double[n, m];
            for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) r[i, j] = a[i, j] * s;
            return r;
        }

        public static double Dot(double[] a, double[] b) {
            double s = 0.0;
            for (int i = 0; i < a.Length; i++) s += a[i] * b[i];
            return s;
        }

        public static double Norm(double[] a) => Math.Sqrt(Dot(a, a));

        public static double[] Cross(double[] a, double[] b) => new[] {
            a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]
        };

        public static double[] Add(double[] a, double[] b) {
            var r = new double[a.Length];
            for (int i = 0; i < r.Length; i++) r[i] = a[i] + b[i];
            return r;
        }

        public static double[] Sub(double[] a, double[] b) {
            var r = new double[a.Length];
            for (int i = 0; i < r.Length; i++) r[i] = a[i] - b[i];
            return r;
        }

        public static double[] Scale(double[] a, double s) {
            var r = new double[a.Length];
            for (int i = 0; i < r.Length; i++) r[i] = a[i] * s;
            return r;
        }

        public static double[] Slice(double[] a, int start, int count) {
            var r = new double[count];
            Array.Copy(a, start, r, 0, count);
            return r;
        }

        /// <summary>S(a) with S(a) b = a x b.</summary>
        public static double[,] Skew(double[] a) => new[,] {
            { 0.0, -a[2], a[1] }, { a[2], 0.0, -a[0] }, { -a[1], a[0], 0.0 }
        };

        public static double[,] Inverse(double[,] a) {
            int n = a.GetLength(0);
            if (n != a.GetLength(1)) throw new ArgumentException("square matrix required");
            var m = new double[n, 2 * n];
            for (int i = 0; i < n; i++) { for (int j = 0; j < n; j++) m[i, j] = a[i, j]; m[i, n + i] = 1.0; }
            for (int c = 0; c < n; c++) {
                int p = c;
                for (int r = c + 1; r < n; r++) if (Math.Abs(m[r, c]) > Math.Abs(m[p, c])) p = r;
                if (Math.Abs(m[p, c]) < 1e-300) throw new InvalidOperationException("singular matrix");
                if (p != c) for (int j = 0; j < 2 * n; j++) { double t = m[c, j]; m[c, j] = m[p, j]; m[p, j] = t; }
                double d = m[c, c];
                for (int j = 0; j < 2 * n; j++) m[c, j] /= d;
                for (int r = 0; r < n; r++) {
                    if (r == c) continue;
                    double f = m[r, c];
                    if (f == 0.0) continue;
                    for (int j = 0; j < 2 * n; j++) m[r, j] -= f * m[c, j];
                }
            }
            var inv = new double[n, n];
            for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) inv[i, j] = m[i, n + j];
            return inv;
        }

        /// <summary>Smallest eigenvalue of a symmetric matrix (cyclic Jacobi).</summary>
        public static double MinSymmetricEigenvalue(double[,] a) {
            int n = a.GetLength(0);
            var m = (double[,])a.Clone();
            for (int sweep = 0; sweep < 100; sweep++) {
                double off = 0.0;
                for (int i = 0; i < n; i++) for (int j = i + 1; j < n; j++) off += m[i, j] * m[i, j];
                if (off < 1e-30) break;
                for (int p = 0; p < n - 1; p++)
                    for (int q = p + 1; q < n; q++) {
                        if (Math.Abs(m[p, q]) < 1e-300) continue;
                        double theta = (m[q, q] - m[p, p]) / (2.0 * m[p, q]);
                        double t = Math.Sign(theta == 0.0 ? 1.0 : theta) / (Math.Abs(theta) + Math.Sqrt(theta * theta + 1.0));
                        double c = 1.0 / Math.Sqrt(t * t + 1.0), s = t * c;
                        for (int k = 0; k < n; k++) {
                            double kp = m[k, p], kq = m[k, q];
                            m[k, p] = c * kp - s * kq; m[k, q] = s * kp + c * kq;
                        }
                        for (int k = 0; k < n; k++) {
                            double pk = m[p, k], qk = m[q, k];
                            m[p, k] = c * pk - s * qk; m[q, k] = s * pk + c * qk;
                        }
                    }
            }
            double min = double.MaxValue;
            for (int i = 0; i < n; i++) min = Math.Min(min, m[i, i]);
            return min;
        }

        /// <summary>
        /// Moore-Penrose pseudo-inverse via one-sided Jacobi (Hestenes) SVD. Handles rank-deficient
        /// and wide matrices. rcond = max(rows, cols) * eps, like numpy.linalg.pinv.
        /// </summary>
        public static double[,] Pinv(double[,] a, out int rank) {
            int m = a.GetLength(0), n = a.GetLength(1);
            var u = (double[,])a.Clone();      // columns are rotated until orthogonal: U*Sigma
            var v = Identity(n);
            for (int sweep = 0; sweep < 60; sweep++) {
                bool rotated = false;
                for (int p = 0; p < n - 1; p++)
                    for (int q = p + 1; q < n; q++) {
                        double alpha = 0, beta = 0, gamma = 0;
                        for (int i = 0; i < m; i++) { alpha += u[i, p] * u[i, p]; beta += u[i, q] * u[i, q]; gamma += u[i, p] * u[i, q]; }
                        if (Math.Abs(gamma) <= 1e-15 * Math.Sqrt(alpha * beta) || gamma == 0.0) continue;
                        rotated = true;
                        double zeta = (beta - alpha) / (2.0 * gamma);
                        double t = Math.Sign(zeta == 0.0 ? 1.0 : zeta) / (Math.Abs(zeta) + Math.Sqrt(1.0 + zeta * zeta));
                        double c = 1.0 / Math.Sqrt(1.0 + t * t), s = c * t;
                        for (int i = 0; i < m; i++) { double up = u[i, p], uq = u[i, q]; u[i, p] = c * up - s * uq; u[i, q] = s * up + c * uq; }
                        for (int i = 0; i < n; i++) { double vp = v[i, p], vq = v[i, q]; v[i, p] = c * vp - s * vq; v[i, q] = s * vp + c * vq; }
                    }
                if (!rotated) break;
            }
            var sigma = new double[n];
            double smax = 0.0;
            for (int j = 0; j < n; j++) {
                double s2 = 0.0;
                for (int i = 0; i < m; i++) s2 += u[i, j] * u[i, j];
                sigma[j] = Math.Sqrt(s2);
                smax = Math.Max(smax, sigma[j]);
            }
            double tol = Math.Max(m, n) * 2.220446049250313e-16 * smax;
            var pinv = new double[n, m];
            rank = 0;
            for (int j = 0; j < n; j++) {
                if (sigma[j] <= tol) continue;
                rank++;
                // pinv += v_j * (u_j / sigma_j)^T / sigma_j   where u_j (column of U*Sigma) = sigma_j * U_j
                double inv2 = 1.0 / (sigma[j] * sigma[j]);
                for (int r = 0; r < n; r++)
                    for (int c = 0; c < m; c++) pinv[r, c] += v[r, j] * u[c, j] * inv2;
            }
            return pinv;
        }

        // ---- quaternions [w, x, y, z] ------------------------------------------------------
        public static double[] QuatNormalize(double[] q) {
            double n = Norm(q);
            if (n == 0.0) throw new ArgumentException("zero quaternion");
            var r = Scale(q, 1.0 / n);
            return r[0] >= 0.0 ? r : Scale(r, -1.0);
        }

        /// <summary>Normalise without touching the sign (safe inside RK4 stages).</summary>
        public static double[] QuatUnit(double[] q) {
            double n = Norm(q);
            if (n == 0.0) throw new ArgumentException("zero quaternion");
            return Scale(q, 1.0 / n);
        }

        public static double[] QuatMul(double[] a, double[] b) => new[] {
            a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]
        };

        public static double[] QuatFromAxisAngle(double[] axis, double angle) {
            double n = Norm(axis), h = 0.5 * angle, s = Math.Sin(h) / n;
            return new[] { Math.Cos(h), axis[0] * s, axis[1] * s, axis[2] * s };
        }

        /// <summary>R_b^n for q = [w,x,y,z].</summary>
        public static double[,] QuatToRot(double[] q) {
            double w = q[0], x = q[1], y = q[2], z = q[3];
            return new[,] {
                { 1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w) },
                { 2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w) },
                { 2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y) }
            };
        }

        /// <summary>Rotation matrix to unit quaternion (Shepperd), hemisphere w &gt;= 0.</summary>
        public static double[] RotToQuat(double[,] r) {
            double tr = r[0, 0] + r[1, 1] + r[2, 2];
            double w, x, y, z;
            if (tr > 0) {
                double s = Math.Sqrt(tr + 1.0) * 2;
                w = 0.25 * s; x = (r[2, 1] - r[1, 2]) / s; y = (r[0, 2] - r[2, 0]) / s; z = (r[1, 0] - r[0, 1]) / s;
            } else if (r[0, 0] > r[1, 1] && r[0, 0] > r[2, 2]) {
                double s = Math.Sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2;
                w = (r[2, 1] - r[1, 2]) / s; x = 0.25 * s; y = (r[0, 1] + r[1, 0]) / s; z = (r[0, 2] + r[2, 0]) / s;
            } else if (r[1, 1] > r[2, 2]) {
                double s = Math.Sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2;
                w = (r[0, 2] - r[2, 0]) / s; x = (r[0, 1] + r[1, 0]) / s; y = 0.25 * s; z = (r[1, 2] + r[2, 1]) / s;
            } else {
                double s = Math.Sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2;
                w = (r[1, 0] - r[0, 1]) / s; x = (r[0, 2] + r[2, 0]) / s; y = (r[1, 2] + r[2, 1]) / s; z = 0.25 * s;
            }
            return QuatNormalize(new[] { w, x, y, z });
        }

        /// <summary>qdot = T(q) omega_b (Fossen eq. 2.72).</summary>
        public static double[] QuatRate(double[] q, double[] omegaBody) {
            double w = q[0], x = q[1], y = q[2], z = q[3];
            double a = omegaBody[0], b = omegaBody[1], c = omegaBody[2];
            return new[] {
                0.5 * (-x * a - y * b - z * c), 0.5 * (w * a - z * b + y * c),
                0.5 * (z * a + w * b - x * c), 0.5 * (-y * a + x * b + w * c)
            };
        }
    }
}
