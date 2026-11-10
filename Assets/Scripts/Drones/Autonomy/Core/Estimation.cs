// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using System.Collections.Generic;

namespace Crane.Drones.Autonomy {
    /// <summary>
    /// M2 simulated estimator: ground truth plus VIO/LIO-like drift (NavPilot experiments/e2_episode.py conditions).
    ///  * local drift e (the odometry frame wanders from truth): random walk stepped every DriftStepM of travel,
    ///    e += sigma sqrt(ds) N(0, I) in x/y (LocalDriftSigma, NavPilot 0.03 m/sqrt(m)), z and yaw with their own sigmas;
    ///  * global-alignment drift delta (odometry frame vs prior map / mission frame), an independent process: constant
    ///    AlignOffset plus AlignRampPerM along travelled distance. The stack only ever sees the believed alignment,
    ///    so a mission goal lands at (goal - e - delta) relative to truth;
    ///  * velocity: truth rotated by the yaw error plus white noise;
    ///  * output latency EstLatencyS at EstRateHz; pose history (2 s) for frame registration and LiDAR deskew.
    /// Replace with a real estimator behind <see cref="IStateEstimator"/>; nothing else in the stack changes.
    /// </summary>
    public sealed class DriftingTruthEstimator : IStateEstimator {
        private readonly AutonomyConfig c; private Rng rng;
        private Vec3 e; private double eyaw, dist, sinceStep;
        private Vec3 lastP; private long periodNs, nextNs, latNs;
        private readonly Queue<StateEstimate> pending = new Queue<StateEstimate>();
        private readonly List<(long t, Pose p)> hist = new List<(long, Pose)>();
        public StateEstimate Current { get; private set; }
        public MapAlignment Alignment { get; private set; }
        public Vec3 LocalError => -e;   // truth - estimate
        public double YawError => -eyaw;
        public double Travelled => dist;

        public DriftingTruthEstimator(AutonomyConfig cfg) { c = cfg; }

        public void Reset(Pose truth0, Vec3 v0, long tNs, ulong seed) {
            rng = new Rng(seed, 21); e = new Vec3(); eyaw = 0; dist = 0; sinceStep = 0; lastP = truth0.P;
            periodNs = (long)Math.Round(1e9 / c.EstRateHz); nextNs = tNs; latNs = (long)Math.Round(c.EstLatencyS * 1e9);
            pending.Clear(); hist.Clear();
            Current = Make(tNs, truth0, v0); Current.InputAgeNs = 0;
            Alignment = new MapAlignment { Offset = new Vec3(c.AlignOffsetX, c.AlignOffsetY, 0), Sigma = 0.1, TNs = tNs };
            hist.Add((tNs, Current.Pose));
        }

        StateEstimate Make(long tNs, Pose truth, Vec3 v) {
            double cy = Math.Cos(eyaw), sy = Math.Sin(eyaw);
            var vr = new Vec3(cy * v.X - sy * v.Y, sy * v.X + cy * v.Y, v.Z);
            var vn = vr + new Vec3(rng.Normal(), rng.Normal(), rng.Normal()) * c.VelNoiseSigma;
            var dq = Pose.FromYaw(new Vec3(), eyaw);
            var q = DepthRaycastSensor.Compose(dq, new Pose { P = new Vec3(), Qw = truth.Qw, Qx = truth.Qx, Qy = truth.Qy, Qz = truth.Qz });
            q.P = truth.P + e;
            return new StateEstimate { TNs = tNs, P = q.P, V = vn, Pose = q, SigmaP = c.LocalDriftSigma * Math.Sqrt(Math.Max(dist, 1e-6)), SigmaYaw = c.YawDriftSigma * Math.Sqrt(Math.Max(dist, 1e-6)), Valid = true, InputAgeNs = latNs };
        }

        public void Update(long tNs, Pose truth, Vec3 v) {
            double ds = (truth.P - lastP).Norm; lastP = truth.P; dist += ds; sinceStep += ds;
            if (sinceStep >= c.DriftStepM) {
                double s = Math.Sqrt(sinceStep);
                e = e + new Vec3(c.LocalDriftSigma * s * rng.Normal(), c.LocalDriftSigma * s * rng.Normal(), c.LocalDriftSigmaZ * s * rng.Normal());
                eyaw += c.YawDriftSigma * s * rng.Normal();
                sinceStep = 0;
            }
            // pose history at every tick (estimate of the pose at that instant, used for registration and deskew)
            var now = Make(tNs, truth, v);
            hist.Add((tNs, now.Pose));
            while (hist.Count > 2 && hist[0].t < tNs - 2_000_000_000L) hist.RemoveAt(0);
            if (tNs >= nextNs) { pending.Enqueue(now); nextNs += periodNs; }
            while (pending.Count > 0 && pending.Peek().TNs + latNs <= tNs) {
                var s = pending.Dequeue(); s.InputAgeNs = tNs - s.TNs; Current = s;
            }
            Alignment = new MapAlignment { Offset = new Vec3(c.AlignOffsetX + c.AlignRampPerM * dist, c.AlignOffsetY, 0), Sigma = 0.1 + 0.01 * dist * Math.Abs(c.AlignRampPerM) * 100, TNs = tNs };
        }

        public bool PoseAt(long tNs, out Pose pose) {
            pose = default;
            if (hist.Count == 0 || tNs < hist[0].t || tNs > hist[hist.Count - 1].t + 10_000_000) return false;
            int lo = 0, hi = hist.Count - 1;
            if (tNs >= hist[hi].t) { pose = hist[hi].p; return true; }
            while (hi - lo > 1) { int m = (lo + hi) / 2; if (hist[m].t <= tNs) lo = m; else hi = m; }
            double u = (double)(tNs - hist[lo].t) / Math.Max(1, hist[hi].t - hist[lo].t);
            pose = Pose.Interpolate(hist[lo].p, hist[hi].p, u); return true;
        }
    }
}
