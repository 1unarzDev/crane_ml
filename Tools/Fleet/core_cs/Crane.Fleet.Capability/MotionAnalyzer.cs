using System;
using System.Collections.Generic;

namespace Crane.Fleet
{
    /// <summary>
    /// Builds the motion capability of a rigid wheeled assembly from its wheel kinematic constraints.
    ///
    /// Formulation (Campion, Bastin and D'Andrea-Novel, IEEE Trans. Robotics and Automation 12(1), 1996, "Structural properties and
    /// classification of kinematic and dynamic models of wheeled mobile robots"): every wheel contact point has velocity
    /// v_c = [vx - wz*py, vy + wz*px]. A fixed standard wheel with rolling direction d and lateral direction n imposes
    /// d.v_c = r*phidot (rolling) and n.v_c = 0 (no lateral slip). A Swedish (mecanum) wheel whose rollers have axis a
    /// (rollers roll freely perpendicular to a) imposes only a.v_c = r*phidot*(a.d). An omni wheel is the case a = d. A passive
    /// castor imposes no constraint on the body twist (its swivel absorbs it). Constraint rows that do not involve a driven wheel
    /// speed form C (hard, C xi = 0); the mobility is 3 - rank C. Driven rows give phidot = A xi.
    /// Failure modes (descriptor v0.1): dead_free removes the driven row (the lateral no-slip row of a conventional wheel stays);
    /// dead_locked (phidot = 0) turns the driven row into a hard row; weak(g) and intermittent(p) scale the wheel speed limit.
    /// </summary>
    public static class MotionAnalyzer
    {
        public static MotionCapability Analyze(CapAssembly asm, CapOptions opt = null)
        {
            opt = opt ?? CapOptions.Default;
            var drivenIds = new List<string>();
            var A = new List<double[]>();
            var L = new List<double>();
            var hard = new List<CapConstraintRow>();
            var soft = new List<CapConstraintRow>();
            var warnings = new List<string>();

            foreach (var m in asm.Modules)
            {
                double cm = Math.Cos(m.Yaw), sm = Math.Sin(m.Yaw);
                foreach (var w in m.Wheels)
                {
                    if (w.Kind == CapWheelKind.Caster) continue;
                    double px = m.X + cm * w.X - sm * w.Y, py = m.Y + sm * w.X + cm * w.Y;
                    double alpha = m.Yaw + w.DriveYaw;
                    double dx = Math.Cos(alpha), dy = Math.Sin(alpha);
                    double nx = -dy, ny = dx;

                    var mode = w.Health.Mode;
                    double gain = 1.0;
                    if (!w.Driven) mode = CapHealthMode.DeadFree;
                    if (mode == CapHealthMode.Intermittent)
                    {
                        if (opt.IntermittentWorstCase) mode = CapHealthMode.DeadFree; else gain = Clamp01(w.Health.Gain);
                    }
                    else if (mode == CapHealthMode.Weak) gain = Clamp01(w.Health.Gain);

                    // rolling / roller-axis direction a and the gear k = r * (a.d)
                    double ax, ay, k;
                    if (w.Kind == CapWheelKind.Mecanum)
                    {
                        double beta = alpha + w.RollerAngle;
                        ax = Math.Cos(beta); ay = Math.Sin(beta);
                        k = w.Radius * Math.Cos(w.RollerAngle);
                    }
                    else
                    {
                        ax = dx; ay = dy; k = w.Radius;
                    }
                    var rowA = Row(ax, ay, px, py);

                    if (mode == CapHealthMode.DeadLocked)
                    {
                        hard.Add(new CapConstraintRow { WheelId = w.Id, Role = w.Kind == CapWheelKind.Conventional ? "locked_rolling" : "locked", Row = rowA });
                    }
                    else if (mode != CapHealthMode.DeadFree)
                    {
                        var r = new[] { rowA[0] / k, rowA[1] / k, rowA[2] / k };
                        drivenIds.Add(w.Id); A.Add(r); L.Add(w.MaxSpeed * gain);
                    }

                    if (w.Kind == CapWheelKind.Conventional)
                    {
                        var lat = Row(nx, ny, px, py);
                        var c = new CapConstraintRow { WheelId = w.Id, Role = "lateral", Row = lat, NormalLoad = w.NormalLoad, LateralFriction = w.LateralFriction };
                        if (w.Lateral == CapLateralMode.Skid) soft.Add(c); else hard.Add(c);
                    }
                }
            }

            var res = new MotionCapability { AssemblyId = asm.Id };
            res.DrivenWheelIds = drivenIds.ToArray();
            res.A = A.ToArray();
            res.SpeedLimit = L.ToArray();
            res.Hard = hard.ToArray();
            res.Soft = soft.ToArray();

            // ---- rank and null space of the hard constraints
            var hardRows = new List<double[]>();
            foreach (var h in hard) hardRows.Add(h.Row);
            double[,] pn; double[][] nBasis; int rankC;
            NullSpace(hardRows, opt.RankTolerance, out pn, out nBasis, out rankC);
            res.RankC = rankC;
            res.Mobility = 3 - rankC;
            res.ProjectorMobility = pn;

            // ---- commandable subspace: row space of A restricted to null(C)
            int delta = nBasis.Length;
            var mrows = new List<double[]>(); // rows of A N (m x delta)
            foreach (var a in res.A)
            {
                var r = new double[delta];
                for (int j = 0; j < delta; j++) r[j] = a[0] * nBasis[j][0] + a[1] * nBasis[j][1] + a[2] * nBasis[j][2];
                mrows.Add(r);
            }
            var ctrl = new List<double[]>(); // orthonormal basis (3-vectors) of the commandable subspace
            if (delta > 0 && mrows.Count > 0)
            {
                var g = CapLinAlg.Gram(mrows, delta);
                double[] ev; double[,] evec;
                CapLinAlg.SymEig(g, out ev, out evec);
                double top = ev[delta - 1];
                for (int c = delta - 1; c >= 0; c--)
                {
                    if (top <= 0 || ev[c] <= opt.RankTolerance * top) continue;
                    var v = new double[3];
                    for (int j = 0; j < delta; j++) for (int t = 0; t < 3; t++) v[t] += evec[j, c] * nBasis[j][t];
                    ctrl.Add(v);
                }
            }
            res.ActuatedRank = ctrl.Count;
            res.ControlledBasis = ctrl.ToArray();
            var pc = new double[3, 3];
            foreach (var v in ctrl) for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) pc[i, j] += v[i] * v[j];
            res.ProjectorControlled = pc;

            // ---- B = T pinv(A T): wheel speeds -> twist, always inside the commandable subspace (so it also respects C)
            res.B = ActuationInverse(res.A, res.ControlledBasis, opt.RankTolerance);

            // ---- feasible polytope vertices: |A T z| <= L, xi = T z
            res.Vertices = Vertices(res.A, res.SpeedLimit, res.ControlledBasis);

            if (res.Uncontrolled > 0) warnings.Add("uncontrolled_dof:" + res.Uncontrolled);
            if (soft.Count > 0) warnings.Add("soft_lateral_rows:" + soft.Count);
            res.Warnings = warnings.ToArray();
            return res;
        }

        private static double[] Row(double ax, double ay, double px, double py)
        {
            // a . [vx - wz*py, vy + wz*px] = ax*vx + ay*vy + wz*(ay*px - ax*py)
            return new[] { ax, ay, ay * px - ax * py };
        }

        private static double Clamp01(double x) => x < 0 ? 0 : (x > 1 ? 1 : x);

        private static void NullSpace(List<double[]> rows, double tol, out double[,] projector, out double[][] basis, out int rank)
        {
            var g = CapLinAlg.Gram(rows, 3);
            double[] ev; double[,] evec;
            CapLinAlg.SymEig(g, out ev, out evec);
            double top = ev[2];
            var nb = new List<double[]>();
            rank = 0;
            for (int c = 0; c < 3; c++)
            {
                if (top > 0 && ev[c] > tol * top) { rank++; continue; }
                nb.Add(new[] { evec[0, c], evec[1, c], evec[2, c] });
            }
            projector = new double[3, 3];
            foreach (var v in nb) for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) projector[i, j] += v[i] * v[j];
            basis = nb.ToArray();
        }

        private static double[][] ActuationInverse(double[][] A, double[][] T, double tol)
        {
            int m = A.Length, d = T.Length;
            var B = new double[3][];
            for (int i = 0; i < 3; i++) B[i] = new double[m];
            if (m == 0 || d == 0) return B;
            var H = new double[m][]; // A T  (m x d)
            for (int i = 0; i < m; i++)
            {
                H[i] = new double[d];
                for (int j = 0; j < d; j++) H[i][j] = A[i][0] * T[j][0] + A[i][1] * T[j][1] + A[i][2] * T[j][2];
            }
            var g = CapLinAlg.Gram(H, d);
            double[] ev; double[,] v;
            CapLinAlg.SymEig(g, out ev, out v);
            double top = ev[d - 1];
            var P = new double[d][]; // pinv(H) = V diag(1/ev) V^T H^T  (d x m)
            for (int i = 0; i < d; i++) P[i] = new double[m];
            for (int c = 0; c < d; c++)
            {
                if (top <= 0 || ev[c] <= tol * top) continue;
                for (int j = 0; j < m; j++)
                {
                    double sj = 0;
                    for (int t = 0; t < d; t++) sj += v[t, c] * H[j][t];
                    for (int i = 0; i < d; i++) P[i][j] += v[i, c] * sj / ev[c];
                }
            }
            for (int r = 0; r < 3; r++)
                for (int j = 0; j < m; j++)
                    for (int i = 0; i < d; i++) B[r][j] += T[i][r] * P[i][j];
            return B;
        }

        private static double[][] Vertices(double[][] A, double[] L, double[][] T)
        {
            int d = T.Length, m = A.Length;
            if (d == 0) return new[] { new double[3] };
            // constraint rows in z coordinates: c_i = T A_i^T  (A_i . T_j)
            var C = new double[m][];
            for (int i = 0; i < m; i++)
            {
                C[i] = new double[d];
                for (int j = 0; j < d; j++) C[i][j] = A[i][0] * T[j][0] + A[i][1] * T[j][1] + A[i][2] * T[j][2];
            }
            var verts = new List<double[]>();
            var pick = new int[d];
            Recurse(0, 0);
            return verts.ToArray();

            void Recurse(int depth, int start)
            {
                if (depth == d)
                {
                    for (int signMask = 0; signMask < (1 << d); signMask++)
                    {
                        var mat = new double[d, d]; var rhs = new double[d];
                        for (int r = 0; r < d; r++)
                        {
                            double sgn = ((signMask >> r) & 1) == 0 ? 1 : -1;
                            for (int c = 0; c < d; c++) mat[r, c] = C[pick[r]][c];
                            rhs[r] = sgn * L[pick[r]];
                        }
                        double[] z;
                        if (!CapLinAlg.Solve(mat, rhs, out z)) continue;
                        bool ok = true;
                        for (int i = 0; i < m && ok; i++)
                        {
                            double s = 0;
                            for (int c = 0; c < d; c++) s += C[i][c] * z[c];
                            if (Math.Abs(s) > L[i] + 1e-9 * Math.Max(1.0, L[i])) ok = false;
                        }
                        if (!ok) continue;
                        var xi = new double[3];
                        for (int c = 0; c < d; c++) for (int t = 0; t < 3; t++) xi[t] += z[c] * T[c][t];
                        bool dup = false;
                        foreach (var q in verts)
                            if (Math.Abs(q[0] - xi[0]) + Math.Abs(q[1] - xi[1]) + Math.Abs(q[2] - xi[2]) < 1e-9) { dup = true; break; }
                        if (!dup) verts.Add(xi);
                    }
                    return;
                }
                for (int i = start; i < m; i++) { pick[depth] = i; Recurse(depth + 1, i + 1); }
            }
        }
    }
}
