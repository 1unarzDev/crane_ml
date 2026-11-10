using System;

namespace Crane.Fleet.Perception
{
    /// <summary>Result of a planar-target pose solve. Pose is T_cam_tag (tag frame points -> optical frame).</summary>
    public sealed class PnPResult
    {
        public bool Ok; public Pose CamTag; public double Cost;      // sum of squared reprojection residuals (px^2)
        public bool HasAlternate; public Pose AlternateCamTag; public double AlternateCost;
        /// <summary>True when the second local minimum has a reprojection cost within <see cref="PlanarPnP.AmbiguityRatio"/> of the best one.</summary>
        public bool Ambiguous;
        /// <summary>Gauss-Newton covariance of [dp(3), dtheta(3)] in camera axes (Pose.Perturbed convention), px noise sigma applied.</summary>
        public Matrix Covariance;
    }

    /// <summary>
    /// Pose of a planar square from four image corners. Method: closed-form DLT homography -> rigid decomposition (solution A), the
    /// mirrored "flip" initial guess (solution B, the classic planar ambiguity), each refined by Levenberg-Marquardt on the
    /// reprojection error; the lower-cost minimum is returned and the other kept as the alternate. Pose error therefore emerges
    /// from geometry (range, obliquity, ambiguity) rather than from a hand-tuned noise curve.
    /// </summary>
    public static class PlanarPnP
    {
        /// <summary>Alternate counts as ambiguous when its cost exceeds the best by less than this many 2*sigma^2 (about 2 chi-square degrees of freedom of corner noise).</summary>
        public static double AmbiguityRatio = 8.0;

        public static PnPResult Solve(CameraIntrinsics K, Vec3[] tagCorners, double[] uv, double sigmaPx)
        {
            var res = new PnPResult();
            var xn = new double[8];
            for (int i = 0; i < 4; i++) { xn[2 * i] = (uv[2 * i] - K.Cx) / K.Fx; xn[2 * i + 1] = (uv[2 * i + 1] - K.Cy) / K.Fy; }
            Pose a;
            if (!HomographyInit(tagCorners, xn, out a)) return res;
            Pose b = FlipInit(a);
            double ca = Refine(K, tagCorners, uv, ref a);
            double cb = Refine(K, tagCorners, uv, ref b);
            bool distinct = (a.P - b.P).Norm > 1e-3 * a.P.Norm || (a.R * b.R.T).Log().Norm > 1e-2;
            res.Ok = true;
            if (cb < ca) { var tp = a; a = b; b = tp; var tc = ca; ca = cb; cb = tc; }
            res.CamTag = a; res.Cost = ca;
            if (distinct && cb < 1e12 && b.P.Z > 0) { res.HasAlternate = true; res.AlternateCamTag = b; res.AlternateCost = cb; res.Ambiguous = (cb - ca) < AmbiguityRatio * 2 * sigmaPx * sigmaPx; }
            res.Covariance = Covariance(K, tagCorners, a, sigmaPx);
            return res;
        }

        /// <summary>Gauss-Newton covariance (JᵀJ)^-1 σ² of the pose perturbation [dt; w] in camera axes.</summary>
        public static Matrix Covariance(CameraIntrinsics K, Vec3[] X, Pose camTag, double sigmaPx)
        {
            Matrix J = Jacobian(K, X, camTag);
            Matrix H = J.T * J;
            return H.Inverse() * (sigmaPx * sigmaPx);
        }

        /// <summary>8x6 Jacobian of corner pixels w.r.t. [dp; w] in camera axes under the <see cref="Pose.Perturbed"/> convention (p' = p + dp, R' = Exp(w) R).</summary>
        public static Matrix Jacobian(CameraIntrinsics K, Vec3[] X, Pose camTag)
        {
            var J = new Matrix(8, 6);
            for (int i = 0; i < 4; i++)
            {
                Vec3 pc = camTag.Apply(X[i]); double z = pc.Z, iz = 1 / z;
                double[,] jp = { { K.Fx * iz, 0, -K.Fx * pc.X * iz * iz }, { 0, K.Fy * iz, -K.Fy * pc.Y * iz * iz } };
                Mat3 sk = Mat3.Skew(camTag.R * X[i]);   // rotation about the tag origin (Pose.Perturbed convention): d(pc) = dp + w x (R X)
                for (int r = 0; r < 2; r++)
                    for (int c = 0; c < 3; c++)
                    {
                        J[2 * i + r, c] = jp[r, c];
                        double s = 0; for (int k = 0; k < 3; k++) s += jp[r, k] * -sk[k, c];
                        J[2 * i + r, 3 + c] = s;
                    }
            }
            return J;
        }

        public static double[] Reproject(CameraIntrinsics K, Vec3[] X, Pose camTag)
        {
            var o = new double[8];
            for (int i = 0; i < 4; i++) { Vec3 pc = camTag.Apply(X[i]); K.Project(pc, out o[2 * i], out o[2 * i + 1]); }
            return o;
        }

        private static bool HomographyInit(Vec3[] X, double[] xn, out Pose pose)
        {
            pose = Pose.Identity;
            var A = new Matrix(8, 8); var b = new double[8];
            for (int i = 0; i < 4; i++)
            {
                double Xx = X[i].X, Yy = X[i].Y, x = xn[2 * i], y = xn[2 * i + 1];
                double[] r1 = { Xx, Yy, 1, 0, 0, 0, -x * Xx, -x * Yy }, r2 = { 0, 0, 0, Xx, Yy, 1, -y * Xx, -y * Yy };
                for (int j = 0; j < 8; j++) { A[2 * i, j] = r1[j]; A[2 * i + 1, j] = r2[j]; }
                b[2 * i] = x; b[2 * i + 1] = y;
            }
            double[] h;
            try { h = A.Solve(b); } catch (InvalidOperationException) { return false; }
            Vec3 c1 = new Vec3(h[0], h[3], h[6]), c2 = new Vec3(h[1], h[4], h[7]), c3 = new Vec3(h[2], h[5], 1);
            double lam = 2.0 / (c1.Norm + c2.Norm);
            if (double.IsNaN(lam) || double.IsInfinity(lam)) return false;
            Vec3 t = c3 * lam;
            if (t.Z < 0) { lam = -lam; t = -t; }
            Vec3 r1v = c1 * lam, r2v = c2 * lam;
            Mat3 R = Mat3.FromColumns(r1v, r2v, r1v.Cross(r2v)).Orthonormalized();
            pose = new Pose(R, t);
            return true;
        }

        /// <summary>Second candidate: tag normal reflected about the line of sight to the tag centre (weak-perspective ambiguity).</summary>
        private static Pose FlipInit(Pose a)
        {
            Vec3 n = a.R.Col(2), los = a.P.Normalized();
            Vec3 nb = (los * (2 * n.Dot(los)) - n).Normalized();
            Mat3 d = Mat3.Between(n, nb);
            return new Pose(d * a.R, a.P);
        }

        private static double Cost(CameraIntrinsics K, Vec3[] X, double[] uv, Pose p, double[] r)
        {
            double c = 0;
            for (int i = 0; i < 4; i++)
            {
                Vec3 pc = p.Apply(X[i]);
                if (pc.Z <= 1e-6) { r[2 * i] = r[2 * i + 1] = 1e6; c += 2e12; continue; }
                r[2 * i] = K.Fx * pc.X / pc.Z + K.Cx - uv[2 * i]; r[2 * i + 1] = K.Fy * pc.Y / pc.Z + K.Cy - uv[2 * i + 1];
                c += r[2 * i] * r[2 * i] + r[2 * i + 1] * r[2 * i + 1];
            }
            return c;
        }

        private static double Refine(CameraIntrinsics K, Vec3[] X, double[] uv, ref Pose p)
        {
            var r = new double[8]; var rn = new double[8];
            double cost = Cost(K, X, uv, p, r), lambda = 1e-3;
            for (int it = 0; it < 60; it++)
            {
                if (cost < 1e-16) break;
                Matrix J = Jacobian(K, X, p); Matrix JT = J.T; Matrix H = JT * J;
                var g = JT.Mul(r);
                bool improved = false;
                for (int tries = 0; tries < 12; tries++)
                {
                    var Hl = H.Clone(); for (int i = 0; i < 6; i++) Hl[i, i] += lambda * Math.Max(H[i, i], 1e-12);
                    double[] d;
                    try { d = Hl.Solve(Array.ConvertAll(g, v => -v)); } catch (InvalidOperationException) { lambda *= 10; continue; }
                    var cand = new Pose(Mat3.Exp(new Vec3(d[3], d[4], d[5])) * p.R, p.P + new Vec3(d[0], d[1], d[2]));
                    double cn = Cost(K, X, uv, cand, rn);
                    if (cn < cost)
                    {
                        double rel = (cost - cn) / Math.Max(cost, 1e-30);
                        p = cand; cost = cn; Array.Copy(rn, r, 8); lambda = Math.Max(lambda / 5, 1e-9); improved = true;
                        if (rel < 1e-12) return cost;
                        break;
                    }
                    lambda *= 5;
                }
                if (!improved) break;
            }
            return cost;
        }
    }
}
