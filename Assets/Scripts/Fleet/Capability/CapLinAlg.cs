using System;
using System.Collections.Generic;

namespace Crane.Fleet
{
    /// <summary>Tiny dense linear algebra for 3-dimensional twist spaces (internal; avoids clashing with other Crane.Fleet math types).</summary>
    internal static class CapLinAlg
    {
        /// <summary>Jacobi eigen-decomposition of a symmetric n x n matrix. Returns eigenvalues (ascending) and eigenvectors as columns.</summary>
        public static void SymEig(double[,] a, out double[] values, out double[,] vectors)
        {
            int n = a.GetLength(0);
            var A = (double[,])a.Clone();
            var V = new double[n, n];
            for (int i = 0; i < n; i++) V[i, i] = 1;
            for (int sweep = 0; sweep < 100; sweep++)
            {
                double off = 0;
                for (int p = 0; p < n; p++) for (int q = p + 1; q < n; q++) off += A[p, q] * A[p, q];
                if (off < 1e-30) break;
                for (int p = 0; p < n; p++)
                {
                    for (int q = p + 1; q < n; q++)
                    {
                        if (Math.Abs(A[p, q]) < 1e-300) continue;
                        double theta = (A[q, q] - A[p, p]) / (2 * A[p, q]);
                        double t = Math.Sign(theta == 0 ? 1 : theta) / (Math.Abs(theta) + Math.Sqrt(theta * theta + 1));
                        double c = 1 / Math.Sqrt(t * t + 1), s = t * c;
                        for (int k = 0; k < n; k++)
                        {
                            double akp = A[k, p], akq = A[k, q];
                            A[k, p] = c * akp - s * akq; A[k, q] = s * akp + c * akq;
                        }
                        for (int k = 0; k < n; k++)
                        {
                            double apk = A[p, k], aqk = A[q, k];
                            A[p, k] = c * apk - s * aqk; A[q, k] = s * apk + c * aqk;
                        }
                        for (int k = 0; k < n; k++)
                        {
                            double vkp = V[k, p], vkq = V[k, q];
                            V[k, p] = c * vkp - s * vkq; V[k, q] = s * vkp + c * vkq;
                        }
                    }
                }
            }
            var idx = new int[n];
            for (int i = 0; i < n; i++) idx[i] = i;
            Array.Sort(idx, (i, j) => A[i, i].CompareTo(A[j, j]));
            values = new double[n];
            vectors = new double[n, n];
            for (int c = 0; c < n; c++)
            {
                values[c] = A[idx[c], idx[c]];
                for (int r = 0; r < n; r++) vectors[r, c] = V[r, idx[c]];
            }
        }

        /// <summary>Gram matrix rows^T rows for rows of length n.</summary>
        public static double[,] Gram(IList<double[]> rows, int n)
        {
            var g = new double[n, n];
            foreach (var r in rows)
                for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) g[i, j] += r[i] * r[j];
            return g;
        }

        /// <summary>Solve a small dense system by Gaussian elimination with partial pivoting; false if singular.</summary>
        public static bool Solve(double[,] a, double[] b, out double[] x)
        {
            int n = b.Length;
            var m = (double[,])a.Clone();
            var r = (double[])b.Clone();
            x = new double[n];
            double scale = 0;
            for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) scale = Math.Max(scale, Math.Abs(m[i, j]));
            if (scale == 0) return false;
            for (int c = 0; c < n; c++)
            {
                int piv = c;
                for (int i = c + 1; i < n; i++) if (Math.Abs(m[i, c]) > Math.Abs(m[piv, c])) piv = i;
                if (Math.Abs(m[piv, c]) < 1e-10 * scale) return false;
                if (piv != c)
                {
                    for (int j = 0; j < n; j++) { double t = m[c, j]; m[c, j] = m[piv, j]; m[piv, j] = t; }
                    double tb = r[c]; r[c] = r[piv]; r[piv] = tb;
                }
                for (int i = c + 1; i < n; i++)
                {
                    double f = m[i, c] / m[c, c];
                    for (int j = c; j < n; j++) m[i, j] -= f * m[c, j];
                    r[i] -= f * r[c];
                }
            }
            for (int i = n - 1; i >= 0; i--)
            {
                double s = r[i];
                for (int j = i + 1; j < n; j++) s -= m[i, j] * x[j];
                x[i] = s / m[i, i];
            }
            return true;
        }
    }
}
