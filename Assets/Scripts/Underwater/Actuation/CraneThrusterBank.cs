using System;
using System.Collections.Generic;
using static Sim.Underwater.CraneLinAlg;

namespace Sim.Underwater {
    /// <summary>One thruster: mount pose in FRD body frame, force limits, lag and deadband (all forces in N).</summary>
    public sealed class CraneThruster {
        public string Name = "";
        public double[] Position = new double[3];   // m, FRD, relative to CO
        public double[] Direction = { 1, 0, 0 };    // unit thrust direction for a positive command
        public double MaxForward;                   // N
        public double MaxReverse;                   // N (positive); may differ from MaxForward
        public double TimeConstant;                 // s, first-order lag (0 = instantaneous)
        public double Deadband;                     // N; |command| below this produces zero thrust

        public CraneThruster(string name, double[] position, double[] direction, double maxForward, double maxReverse,
            double timeConstant = 0.0, double deadband = 0.0) {
            Name = name; Position = (double[])position.Clone();
            Direction = Scale(direction, 1.0 / Norm(direction));
            MaxForward = maxForward; MaxReverse = maxReverse; TimeConstant = timeConstant; Deadband = deadband;
        }

        /// <summary>Command (N) after deadband and asymmetric saturation.</summary>
        public double Target(double command) {
            if (Math.Abs(command) < Deadband) command = 0.0;
            return Math.Min(Math.Max(command, -MaxReverse), MaxForward);
        }
    }

    /// <summary>Requested/allocated wrench report for one allocation call.</summary>
    public sealed class CraneAllocationResult {
        public double[] RequestedWrench;      // tau requested
        public double[] Unconstrained;        // pinv(T) tau, per-thruster N
        public double[] Forces;               // after uniform scaling into limits, per-thruster N
        public double Scale;                  // 1 = no scaling needed
        public bool[] SaturatedThrusters;     // unconstrained command exceeded this thruster's limit
        public double[] AllocatedWrench;      // T * Forces
        public double[] AppliedWrench;        // T * actual (lagged) thrust; filled by CraneThrusterBank.Apply
        public double ResidualNorm;           // |requested - allocated| (unreachable directions or scaling)
        public int Rank;                      // rank of T
        public bool Saturated => Scale < 1.0;
    }

    public sealed class CraneThrusterBank {
        public readonly IReadOnlyList<CraneThruster> Thrusters;
        public readonly double[,] T;
        readonly double[,] tPinv;
        public readonly int Rank;
        /// <summary>Actual per-thruster force (N) after the lag.</summary>
        public double[] Thrust;
        /// <summary>Last commanded force (N) per thruster.</summary>
        public double[] Command;
        public CraneAllocationResult Last { get; private set; }

        public CraneThrusterBank(IReadOnlyList<CraneThruster> thrusters) {
            Thrusters = thrusters;
            T = AllocationMatrix(thrusters);
            tPinv = Pinv(T, out Rank);
            Thrust = new double[thrusters.Count];
            Command = new double[thrusters.Count];
        }

        /// <summary>T (6 x n): tau = T f, column = [d; r x d].</summary>
        public static double[,] AllocationMatrix(IReadOnlyList<CraneThruster> th) {
            var t = new double[6, th.Count];
            for (int j = 0; j < th.Count; j++) {
                var m = Cross(th[j].Position, th[j].Direction);
                for (int i = 0; i < 3; i++) { t[i, j] = th[j].Direction[i]; t[3 + i, j] = m[i]; }
            }
            return t;
        }

        /// <summary>Least-norm allocation, then uniform scaling into [-MaxReverse, MaxForward] (keeps wrench direction).</summary>
        public CraneAllocationResult Allocate(double[] tau) {
            var f = Mul(tPinv, tau);
            double scale = 1.0;
            var flags = new bool[f.Length];
            for (int i = 0; i < f.Length; i++) {
                double hi = Thrusters[i].MaxForward, lo = -Thrusters[i].MaxReverse;
                if (f[i] > hi && hi > 0) { scale = Math.Min(scale, hi / f[i]); flags[i] = true; }
                else if (f[i] < lo && lo < 0) { scale = Math.Min(scale, lo / f[i]); flags[i] = true; }
            }
            var fs = Scale(f, scale);
            var alloc = Mul(T, fs);
            var res = new CraneAllocationResult {
                RequestedWrench = (double[])tau.Clone(), Unconstrained = f, Forces = fs, Scale = scale,
                SaturatedThrusters = flags, AllocatedWrench = alloc, AppliedWrench = Mul(T, Thrust),
                ResidualNorm = Norm(Sub(tau, alloc)), Rank = Rank
            };
            Last = res;
            return res;
        }

        /// <summary>
        /// Advance the lag by dt (exact exponential for a held command). Returns the wrench T*mean(thrust) over the step
        /// (exact interval mean of the first-order response), so a force-held-per-step integrator conserves impulse.
        /// Thrust holds the end-of-step value.
        /// </summary>
        public double[] Step(double dt, double[] commandNewtons) {
            var mean = new double[Thrusters.Count];
            for (int i = 0; i < Thrusters.Count; i++) {
                Command[i] = commandNewtons[i];
                double target = Thrusters[i].Target(commandNewtons[i]);
                double tc = Thrusters[i].TimeConstant;
                double f0 = Thrust[i];
                if (tc > 0.0 && dt > 0.0) {
                    double e = Math.Exp(-dt / tc);
                    Thrust[i] = target + (f0 - target) * e;
                    mean[i] = target + (f0 - target) * (tc / dt) * (1.0 - e);
                } else { Thrust[i] = target; mean[i] = target; }
            }
            var applied = Mul(T, mean);
            if (Last != null) Last.AppliedWrench = Mul(T, Thrust);
            return applied;
        }

        public void Reset() {
            Array.Clear(Thrust, 0, Thrust.Length);
            Array.Clear(Command, 0, Command.Length);
            Last = null;
        }
    }
}
