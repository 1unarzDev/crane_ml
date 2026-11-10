using System;
using System.Collections.Generic;

namespace Crane.Fleet
{
    /// <summary>
    /// Wheel-speed allocation for an assembly: phidot = A' xi with A' the per-module kinematic rows compensated for the floor's
    /// lateral efficiency, after projecting xi onto the commandable subspace (locked wheels stay at zero), with one common scale so
    /// every live wheel respects its (health-scaled) limit. Dead-free and dead-locked wheels are commanded 0. Allocation-free.
    /// </summary>
    public sealed class WheelAllocator
    {
        private AssemblyCapability _cap;
        private double[] _rows;
        public double LastScale { get; private set; } = 1;
        public int WheelCount => _cap.WheelCount;

        public void Bind(AssemblyModel model, ICapability cap)
        {
            _cap = (AssemblyCapability)cap;
            _rows = KinematicRows.Build(model, true);
        }

        /// <summary>Allocate twist <paramref name="xi"/> (assembly frame) into <paramref name="wheels"/> (length WheelCount). Returns the realised (scaled) twist.</summary>
        public Twist2 Allocate(in Twist2 xi, double[] wheels)
        {
            var p = _cap.Project(xi);
            double s = 1;
            int n = _cap.WheelCount;
            for (int i = 0; i < n; i++)
            {
                if (!_cap.Driven(i)) { wheels[i] = 0; continue; }
                double w = _rows[3 * i] * p.Vx + _rows[3 * i + 1] * p.Vy + _rows[3 * i + 2] * p.Wz;
                wheels[i] = w;
                double lim = _cap.WheelLimit(i);
                if (Math.Abs(w) > lim && Math.Abs(w) > 0) s = Math.Min(s, lim / Math.Abs(w));
            }
            if (s < 1) for (int i = 0; i < n; i++) wheels[i] *= s;
            LastScale = s;
            return p * s;
        }
    }

    /// <summary>Per-wheel kinematic rows (assembly twist -> wheel rad/s), optionally dividing the module-lateral term by the floor's
    /// lateral efficiency (what the wheels must turn to achieve a ground lateral speed).</summary>
    public static class KinematicRows
    {
        public static double[] Build(AssemblyModel model, bool lateralEfficiency)
        {
            int n = model.WheelCount;
            var rows = new double[3 * n];
            for (int m = 0; m < model.Modules.Count; m++)
            {
                var mod = model.Modules[m]; var s = mod.Spec;
                double c = Math.Cos(mod.InAssembly.Yaw), sn = Math.Sin(mod.InAssembly.Yaw), px = mod.InAssembly.X, py = mod.InAssembly.Y;
                for (int j = 0; j < 4; j++)
                {
                    s.WheelRow(j, out double ax, out double ay, out double aw);
                    if (lateralEfficiency) ay /= Math.Max(1e-3, s.LateralEfficiency);
                    int i = 4 * m + j;
                    rows[3 * i] = ax * c - ay * sn;
                    rows[3 * i + 1] = ax * sn + ay * c;
                    rows[3 * i + 2] = ax * (-c * py + sn * px) + ay * (sn * py + c * px) + aw;
                }
            }
            return rows;
        }
    }

    public enum TrackStatus { Idle, Tracking, Arrived, Deviated }

    /// <summary>
    /// Path tracking for holonomic and reduced-mobility assemblies: feedforward of the planned body twist (speed-profiled for the
    /// remaining distance) plus pose feedback to the nearest path point, projected onto the commandable subspace and scaled into
    /// the twist set, with acceleration limits. Error components the assembly cannot command are left to replanning (Deviated).
    /// </summary>
    public sealed class TrajectoryTracker
    {
        private readonly AutonomyAssumptions _a;
        private PlannedPath _path;
        private AssemblyCapability _cap;
        private int _i;
        private Twist2 _last;
        public TrackStatus Status { get; private set; } = TrackStatus.Idle;
        public double CrossTrackError { get; private set; }
        public double SpeedFraction = 1.0;
        public double PosTol, YawTol;

        public TrajectoryTracker(AutonomyAssumptions a) { _a = a; }

        public void SetPath(PlannedPath path, ICapability cap)
        {
            _path = path; _cap = (AssemblyCapability)cap; _i = 0; Status = path != null && path.Ok && path.Points.Count > 0 ? TrackStatus.Tracking : TrackStatus.Idle;
            PosTol = _a.GoalPosTol; YawTol = _a.GoalYawTol;
            if (cap.ActuatedRank < 3) { PosTol = Math.Max(PosTol, 0.10); YawTol = Math.Max(YawTol, 0.25); }
        }

        public void UpdateCapability(ICapability cap) { _cap = (AssemblyCapability)cap; }

        public void Stop() { Status = TrackStatus.Idle; _path = null; }

        public Twist2 LastCommand => _last;

        public Twist2 Step(Pose2 est, double dt)
        {
            if (Status != TrackStatus.Tracking || _path == null) { _last = RateLimit(Twist2.Zero, dt); return _last; }
            var pts = _path.Points;
            // nearest point, monotone search ahead
            double best = double.PositiveInfinity; int bi = _i;
            int lim = Math.Min(pts.Count, _i + 40);
            for (int k = _i; k < lim; k++)
            {
                double d = Se2.Distance(est, pts[k].Pose) + 0.1 * Math.Abs(Se2.Wrap(pts[k].Pose.Yaw - est.Yaw));
                if (d < best) { best = d; bi = k; }
            }
            _i = bi;
            CrossTrackError = Se2.Distance(est, pts[_i].Pose);
            var goal = pts[pts.Count - 1].Pose;
            double dGoal = Se2.Distance(est, goal), yGoal = Math.Abs(Se2.Wrap(goal.Yaw - est.Yaw));
            double remaining = Math.Max(0, pts[pts.Count - 1].S - pts[_i].S) + dGoal * 0.0;
            if (dGoal < PosTol && yGoal < YawTol && _last.LinearSpeed < 0.03 && Math.Abs(_last.Wz) < 0.1)
            { Status = TrackStatus.Arrived; _last = Twist2.Zero; return _last; }
            double dev = _cap.ActuatedRank < 3 ? _a.ReducedMobilityReplanDeviation : _a.ReplanDeviation;
            if (CrossTrackError > dev) { Status = TrackStatus.Deviated; _last = RateLimit(Twist2.Zero, dt); return _last; }

            Twist2 cmd;
            bool terminal = _i >= pts.Count - 2 || remaining < 0.08;
            if (!terminal)
            {
                // reference a little ahead of the nearest point
                int j = _i; while (j < pts.Count - 1 && pts[j].S - pts[_i].S < _a.TrackLookahead && pts[j].T - pts[_i].T < 0.4) j++;
                var e = Se2.Between(est, pts[j].Pose);
                var ff = pts[_i].Twist;
                double vff = Math.Max(ff.LinearSpeed, 1e-6);
                double vAllowed = Math.Sqrt(2 * _cap.MaxLinAccel * 0.5 * Math.Max(remaining, 0.0)) + 0.02;
                double sc = Math.Min(1.0, vAllowed / vff) * SpeedFraction;
                // feedback on the lookahead error scaled by the time to reach it
                double tl = Math.Max(0.15, (pts[j].T - pts[_i].T));
                cmd = ff * sc + new Twist2(_a.TrackKpos * (e.X - ff.Vx * sc * tl) * 0.5, _a.TrackKpos * (e.Y - ff.Vy * sc * tl) * 0.5, _a.TrackKyaw * (e.Yaw - ff.Wz * sc * tl) * 0.5);
            }
            else
            {
                var e = Se2.Between(est, goal);
                cmd = new Twist2(_a.TrackKpos * e.X, _a.TrackKpos * e.Y, _a.TrackKyaw * e.Yaw);
                double v = cmd.LinearSpeed, vmax = Math.Max(0.03, Math.Min(0.15, Math.Sqrt(2 * _cap.MaxLinAccel * 0.5 * dGoal)));
                if (v > vmax) cmd = new Twist2(cmd.Vx * vmax / v, cmd.Vy * vmax / v, cmd.Wz);
                if (Math.Abs(cmd.Wz) > 0.6) cmd = new Twist2(cmd.Vx, cmd.Vy, Math.Sign(cmd.Wz) * 0.6);
            }
            cmd = _cap.Saturate(cmd, out _);
            _last = RateLimit(cmd, dt);
            return _last;
        }

        private Twist2 RateLimit(Twist2 target, double dt)
        {
            double al = _cap != null ? _cap.MaxLinAccel : _a.MaxLinAccel, aw = _cap != null ? _cap.MaxYawAccel : _a.MaxYawAccel;
            double dvx = target.Vx - _last.Vx, dvy = target.Vy - _last.Vy, dw = target.Wz - _last.Wz;
            double dv = Math.Sqrt(dvx * dvx + dvy * dvy), s = 1;
            if (dv > al * dt) s = Math.Min(s, al * dt / dv);
            if (Math.Abs(dw) > aw * dt) s = Math.Min(s, aw * dt / Math.Abs(dw));
            var r = new Twist2(_last.Vx + dvx * s, _last.Vy + dvy * s, _last.Wz + dw * s);
            return _cap != null ? _cap.Project(r) : r;
        }

        /// <summary>Directly regulate to the rate-limited target twist (used by the docking servo).</summary>
        public Twist2 Hold(Twist2 target, double dt) { _last = RateLimit(_cap != null ? _cap.Saturate(target, out _) : target, dt); return _last; }
        public void ResetCommand() { _last = Twist2.Zero; }
    }

    /// <summary>
    /// Online wheel-health identification from commanded versus measured wheel rates (descriptor v0.1: dead_free vs dead_locked vs
    /// weak are distinguishable by motion): a wheel commanded above a threshold that persistently reads ~0 is dead_locked when the
    /// body still moves consistently with a constraint (its rate stays ~0 while neighbours turn), dead_free when its rate follows
    /// the body's kinematics but not its command, weak when it follows the command with a gain below the weak fraction.
    /// </summary>
    public sealed class WheelHealthMonitor
    {
        private readonly AutonomyAssumptions _a;
        private readonly double[] _timer = new double[4], _gainAcc = new double[4];
        private readonly int[] _mode = new int[4]; // 0 ok, 1 locked, 2 free, 3 weak candidate
        public readonly CapHealth[] Identified = { CapHealth.Ok, CapHealth.Ok, CapHealth.Ok, CapHealth.Ok };
        public WheelHealthMonitor(AutonomyAssumptions a) { _a = a; }

        /// <summary>Feed one interval: commanded rates (rad/s), measured encoder rates, and the kinematic rates implied by the body
        /// twist estimated from the OTHER wheels (or the EKF velocity). Returns true when an identification changed.</summary>
        public bool Update(double[] cmd, double[] meas, double[] kin, double dt)
        {
            bool changed = false;
            for (int i = 0; i < 4; i++)
            {
                if (Identified[i].Mode != CapHealthMode.Ok) continue;
                double c = cmd[i], m = meas[i], k = kin[i];
                int cand = 0;
                if (Math.Abs(c) > _a.HealthCmdThreshold)
                {
                    double rt = m / c;
                    if (Math.Abs(m) < _a.HealthLockedFrac * Math.Abs(c)) cand = 1;
                    else if (rt < _a.HealthWeakFrac && Math.Abs(m - k) < 0.35 * Math.Abs(c) && Math.Abs(k - c) > 0.3 * Math.Abs(c)) cand = 2;
                    else if (rt < _a.HealthWeakFrac && rt > _a.HealthLockedFrac) cand = 3;
                }
                if (cand != 0 && cand == _mode[i]) { _timer[i] += dt; if (cand == 3) _gainAcc[i] += ratio(c, m) * dt; }
                else { _mode[i] = cand; _timer[i] = cand != 0 ? dt : 0; _gainAcc[i] = cand == 3 ? ratio(c, m) * dt : 0; }
                if (cand != 0 && _timer[i] >= _a.HealthWindow)
                {
                    Identified[i] = cand == 1 ? CapHealth.DeadLocked : cand == 2 ? CapHealth.DeadFree : CapHealth.Weak(Math.Max(0.1, _gainAcc[i] / _timer[i]));
                    changed = true;
                }
            }
            return changed;
            static double ratio(double c, double m) => Math.Abs(c) > 1e-9 ? Math.Abs(m / c) : 1;
        }
    }
}
