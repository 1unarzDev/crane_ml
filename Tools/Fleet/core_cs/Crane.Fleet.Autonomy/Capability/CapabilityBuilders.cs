using System;
using System.Collections.Generic;

namespace Crane.Fleet
{
    /// <summary>
    /// Default builder: the Crane.Fleet.Capability library's MotionAnalyzer (Campion et al. constraint formulation with the
    /// descriptor v0.1 health modes) supplies the commandable subspace, actuated rank and effective wheel limits. Wheel rows for
    /// every wheel (including dead ones, which the estimator still uses) come from the same mecanum geometry.
    /// </summary>
    public sealed class CapabilityLibraryBuilder : ICapabilityBuilder
    {
        public ICapability Build(AssemblyModel model, AutonomyAssumptions assume)
        {
            var c = AssemblyCapability.Skeleton(model, assume, "capability_library");
            var asm = new CapAssembly { Id = model.Id };
            for (int m = 0; m < model.Modules.Count; m++)
            {
                var mod = model.Modules[m]; var s = mod.Spec;
                var cm = CapabilityBuilders.MecanumRover("m" + m, s.Lx, s.Ly, s.WheelRadius, s.MaxWheelSpeed, mod.InAssembly.X, mod.InAssembly.Y, mod.InAssembly.Yaw);
                for (int j = 0; j < 4; j++) cm.Wheels[j].Health = mod.Health[j];
                asm.Modules.Add(cm);
            }
            var mc = MotionAnalyzer.Analyze(asm);
            for (int i = 0; i < 9; i++) c.P[i] = mc.ProjectorControlled[i / 3, i % 3];
            c.ActuatedRank = mc.ActuatedRank;
            for (int k = 0; k < mc.ControlledBasis.Length && k < 3; k++) for (int t = 0; t < 3; t++) c.BasisRows[3 * k + t] = mc.ControlledBasis[k][t];
            // limits from the library (by wheel id); dead wheels absent there stay 0
            var lim = new Dictionary<string, double>();
            for (int i = 0; i < mc.DrivenWheelIds.Length; i++) lim[mc.DrivenWheelIds[i]] = mc.SpeedLimit[i];
            string[] names = { ".fl", ".fr", ".rl", ".rr" };
            for (int m = 0; m < model.Modules.Count; m++)
                for (int j = 0; j < 4; j++)
                    c.Limits[4 * m + j] = lim.TryGetValue("m" + m + names[j], out var l) ? l : 0;
            return c;
        }
    }

    /// <summary>Engine-free analytic fallback with the same contract (locked wheels are hard rows, dead-free rows are dropped,
    /// weak wheels scale limits). Used when the capability library is absent and as a cross-check in tests.</summary>
    public sealed class AnalyticCapabilityBuilder : ICapabilityBuilder
    {
        public ICapability Build(AssemblyModel model, AutonomyAssumptions assume)
        {
            var c = AssemblyCapability.Skeleton(model, assume, "analytic_fallback");
            int n = c.WheelCount;
            // orthonormal basis Q of locked rows (Gram-Schmidt)
            var q = new List<double[]>();
            for (int i = 0; i < n; i++)
            {
                if (c.Modes[i] != CapHealthMode.DeadLocked) continue;
                var v = new[] { c.Rows[3 * i], c.Rows[3 * i + 1], c.Rows[3 * i + 2] };
                foreach (var b in q) { double d = v[0] * b[0] + v[1] * b[1] + v[2] * b[2]; for (int t = 0; t < 3; t++) v[t] -= d * b[t]; }
                double nv = Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
                double n0 = Math.Sqrt(c.Rows[3 * i] * c.Rows[3 * i] + c.Rows[3 * i + 1] * c.Rows[3 * i + 1] + c.Rows[3 * i + 2] * c.Rows[3 * i + 2]);
                if (nv > 1e-9 * n0) q.Add(new[] { v[0] / nv, v[1] / nv, v[2] / nv });
            }
            var pn = new double[9];
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) { double s = i == j ? 1 : 0; foreach (var b in q) s -= b[i] * b[j]; pn[3 * i + j] = s; }
            // M = PN A^T A PN over driven rows
            var m = new double[9];
            for (int w = 0; w < n; w++)
            {
                if (!c.Driven(w)) continue;
                var r = new double[3];
                for (int i = 0; i < 3; i++) r[i] = pn[3 * i] * c.Rows[3 * w] + pn[3 * i + 1] * c.Rows[3 * w + 1] + pn[3 * i + 2] * c.Rows[3 * w + 2];
                for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) m[3 * i + j] += r[i] * r[j];
            }
            Jacobi3(m, out var ev, out var vec);
            double top = Math.Max(ev[0], Math.Max(ev[1], ev[2]));
            int rank = 0;
            Array.Clear(c.P, 0, 9);
            for (int k = 0; k < 3; k++)
            {
                if (top <= 0 || ev[k] <= 1e-10 * top) continue;
                for (int t = 0; t < 3; t++) c.BasisRows[3 * rank + t] = vec[3 * t + k];
                for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) c.P[3 * i + j] += vec[3 * i + k] * vec[3 * j + k];
                rank++;
            }
            c.ActuatedRank = rank;
            return c;
        }

        /// <summary>Cyclic Jacobi eigen-decomposition of a symmetric 3x3; eigenvectors are the columns of vec (row-major).</summary>
        internal static void Jacobi3(double[] a0, out double[] ev, out double[] vec)
        {
            var a = (double[])a0.Clone();
            vec = new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
            for (int sweep = 0; sweep < 50; sweep++)
            {
                double off = a[1] * a[1] + a[2] * a[2] + a[5] * a[5];
                if (off < 1e-30) break;
                for (int p = 0; p < 2; p++)
                    for (int qq = p + 1; qq < 3; qq++)
                    {
                        double apq = a[3 * p + qq];
                        if (Math.Abs(apq) < 1e-300) continue;
                        double theta = (a[3 * qq + qq] - a[3 * p + p]) / (2 * apq);
                        double t = Math.Sign(theta) / (Math.Abs(theta) + Math.Sqrt(theta * theta + 1));
                        if (theta == 0) t = 1;
                        double cs = 1 / Math.Sqrt(t * t + 1), sn = t * cs;
                        for (int k = 0; k < 3; k++)
                        {
                            double akp = a[3 * k + p], akq = a[3 * k + qq];
                            a[3 * k + p] = cs * akp - sn * akq; a[3 * k + qq] = sn * akp + cs * akq;
                        }
                        for (int k = 0; k < 3; k++)
                        {
                            double apk = a[3 * p + k], aqk = a[3 * qq + k];
                            a[3 * p + k] = cs * apk - sn * aqk; a[3 * qq + k] = sn * apk + cs * aqk;
                        }
                        for (int k = 0; k < 3; k++)
                        {
                            double vkp = vec[3 * k + p], vkq = vec[3 * k + qq];
                            vec[3 * k + p] = cs * vkp - sn * vkq; vec[3 * k + qq] = sn * vkp + cs * vkq;
                        }
                    }
            }
            ev = new[] { a[0], a[4], a[8] };
        }
    }
}
