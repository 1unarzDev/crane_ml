// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using System.Collections.Generic;
using Sim.Physics.Aerial;

namespace Crane.Drones.Autonomy {
    /// <summary>
    /// Prior map from the forest manifest (M3): 2D flight-band columns (0.5 m) and a 3D hard-surface query. Optional
    /// degradation: PriorDropFraction of the trees/stems removed (incomplete or stale survey) and a rigid shift
    /// (PriorShiftX/Y, georeferencing error). Thin twigs (&lt; HardRadius) are never in the prior.
    /// </summary>
    public sealed class ManifestPriorMap : IPriorMap {
        private readonly ForestManifestWorld w; private readonly bool[] dropOwner; private readonly double sx, sy;
        private readonly double res = 0.5, x0, y0; private readonly int nx, ny; private readonly bool[] col;
        public double Resolution => res;
        public string Describe { get; }

        public ManifestPriorMap(ForestManifestWorld world, AutonomyConfig c, ulong seed) {
            w = world; sx = c.PriorShiftX; sy = c.PriorShiftY;
            dropOwner = new bool[world.OwnerNames.Length];
            var rng = new Rng(seed, 31); int dropped = 0;
            for (int i = 0; i < dropOwner.Length; i++) if (rng.Uniform() < c.PriorDropFraction) { dropOwner[i] = true; dropped++; }
            x0 = world.MinX; y0 = world.MinY; nx = (int)Math.Ceiling((world.MaxX - x0) / res) + 1; ny = (int)Math.Ceiling((world.MaxY - y0) / res) + 1;
            col = new bool[nx * ny];
            for (int s = 0; s < world.Count; s++) {
                if (world.R[s] < 0.03 || dropOwner[world.Owner[s]]) continue;   // stems and limbs only (diameter >= 6 cm)
                double mz = 0.5 * (world.Az[s] + world.Bz[s]); double g = world.GroundHeight(0.5 * (world.Ax[s] + world.Bx[s]), 0.5 * (world.Ay[s] + world.By[s]));
                if (mz < g + 0.5 || mz > g + 4.5) continue;
                double r = world.R[s];
                int i0 = (int)Math.Floor((Math.Min(world.Ax[s], world.Bx[s]) - r + sx - x0) / res), i1 = (int)Math.Floor((Math.Max(world.Ax[s], world.Bx[s]) + r + sx - x0) / res);
                int j0 = (int)Math.Floor((Math.Min(world.Ay[s], world.By[s]) - r + sy - y0) / res), j1 = (int)Math.Floor((Math.Max(world.Ay[s], world.By[s]) + r + sy - y0) / res);
                for (int i = Math.Max(0, i0); i <= Math.Min(nx - 1, i1); i++) for (int j = Math.Max(0, j0); j <= Math.Min(ny - 1, j1); j++) col[i * ny + j] = true;
            }
            Describe = $"manifest prior: stems/limbs d>=6 cm in the 0.5-4.5 m AGL band, {dropped}/{dropOwner.Length} owners dropped, shift=({sx},{sy})";
        }

        public bool BlockedColumn(double x, double y) {
            int i = (int)Math.Floor((x - x0) / res), j = (int)Math.Floor((y - y0) / res);
            return i >= 0 && j >= 0 && i < nx && j < ny && col[i * ny + j];
        }

        public bool Occupied(Vec3 p) {
            double d = w.Clearance(new Vec3(p.X - sx, p.Y - sy, p.Z), 0.3, true, out int id, out double r);
            return id >= 0 && r >= 0.03 && !dropOwner[w.Owner[id]] && d < 0.1;
        }
    }

    /// <summary>
    /// M2b seam, in-process implementation: TrajectorySetpoint -> CraneMultirotorController velocity mode (velocity in
    /// m/s, acceleration feed-forward in m/s^2, yaw target). The controller's own kinematic gains and its mass/inertia
    /// force conversion are untouched. A PX4 offboard / SITL implementation replaces this class only.
    /// </summary>
    public sealed class InProcessControllerLink : IFlightControllerLink {
        private readonly CraneMultirotorController ctl; private readonly Func<double[]> position;
        private bool holding;
        public FcStatus Status { get; } = new FcStatus();
        public TrajectorySetpoint Last { get; private set; }
        /// <param name="currentPosition">FC-side position (the controller's own frame) latched when a Hold setpoint arrives</param>
        public InProcessControllerLink(CraneMultirotorController controller, Func<double[]> currentPosition = null) { ctl = controller; position = currentPosition; }
        public void Submit(TrajectorySetpoint sp) {
            Last = sp; Status.LastSetpointNs = sp.TNs;
            if (sp.Hold && position != null) {
                if (!holding) { ctl.HoldPosition((double[])position().Clone(), double.IsNaN(sp.Yaw) ? ctl.YawSetpoint : sp.Yaw); holding = true; }
                return;
            }
            holding = false;
            ctl.SetVelocity(sp.Velocity.ToArray(), 0.0);
            ctl.AccelFeedforward[0] = sp.Acceleration.X; ctl.AccelFeedforward[1] = sp.Acceleration.Y; ctl.AccelFeedforward[2] = sp.Acceleration.Z;
            if (!double.IsNaN(sp.Yaw)) ctl.YawSetpoint = sp.Yaw;
        }
    }

    /// <summary>
    /// Setpoint arbiter and supervisor (M4b): RC &gt; supervisor (brake/hold) &gt; planner.
    ///  * stall watchdog: no fresh planner command for WatchdogS while moving -> brake to hover (zero velocity);
    ///  * estimator stale (input age &gt; EstimateStaleS) or invalid -> brake;
    ///  * planner reports no feasible primitive ("neither source supports a safe maneuver") -> brake to hover;
    ///  * belief near miss (ESDF distance at the estimate &lt; r_body + NearMissRecoverM) -> recovery: speed cap
    ///    RecoverySpeedFrac x v for RecoveryHoldS and an immediate cost-to-go recompute.
    /// Yaw follows the commanded horizontal heading, rate-limited, so the forward depth camera looks where it flies.
    /// </summary>
    public sealed class SetpointArbiter {
        private readonly AutonomyConfig c; private readonly IFlightControllerLink fc; private readonly IAutonomyRecorder rec;
        public long LastPlannerApplyNs = long.MinValue;
        public Vec3 PlannerVel; public double PlannerYaw = double.NaN;
        public string Mode { get; private set; } = "init";
        public int WatchdogTrips, NoSafeStops, EstimateStops, Recoveries;
        public long RecoveryUntilNs = long.MinValue;
        private double yawCmd = double.NaN;
        private long lastSensorAcq = -1;
        // path-following brake state
        private PlanDecision lastSafe, brakePath; private long lastSafeNs = long.MinValue, brakeT0; private double brakeV0; private int brakeIdx;
        private Vec3 lastV;
        public int PathBrakes, StraightBrakes;
        public bool BrakingAlongPath => brakePath != null;

        public SetpointArbiter(AutonomyConfig cfg, IFlightControllerLink link, IAutonomyRecorder recorder) { c = cfg; fc = link; rec = recorder; }

        public void ApplyPlanner(long tNs, PlanDecision d, long sensorAcqNs) {
            LastPlannerApplyNs = tNs; lastSensorAcq = sensorAcqNs;
            if (!d.FeasibleAny) {
                if (Mode != "no_safe_maneuver") {
                    NoSafeStops++; rec.Event(tNs, "autonomy_arbiter", new Dictionary<string, object> { ["action"] = "brake", ["reason"] = "no_safe_maneuver", ["d_free_best"] = d.DFree });
                    if (Mode != "watchdog" || brakePath == null) BeginBrake(tNs, "no_safe_maneuver");
                }
                Mode = "no_safe_maneuver"; PlannerVel = new Vec3();
                // yaw: keep facing along the validated path while braking on it; otherwise toward the most open direction
                if (brakePath == null) PlannerYaw = d.Heading;
            } else {
                Mode = "planner"; PlannerVel = d.VelocityCmd; if (d.VCmd > 0.2) PlannerYaw = d.Heading;
                if (d.Path != null) { lastSafe = d; lastSafeNs = tNs; }
                brakePath = null;
            }
        }

        /// <summary>Start a brake: along the last validated path if it is recent (BrakeAlongPath), else straight (zero velocity).</summary>
        void BeginBrake(long tNs, string why) {
            brakePath = null;
            if (c.BrakeAlongPath && lastSafe != null && lastSafe.Path != null && lastSafe.Path.Length >= 2 && tNs - lastSafeNs <= (long)(c.SafePathMaxAgeS * 1e9)) {
                brakePath = lastSafe; brakeT0 = tNs; brakeV0 = lastV.Norm; brakeIdx = 0; PathBrakes++;
                rec.Event(tNs, "autonomy_arbiter", new Dictionary<string, object> { ["action"] = "brake_along_path", ["reason"] = why, ["v0"] = brakeV0,
                    ["safe_arc_m"] = brakePath.SafeArc, ["path_age_s"] = (tNs - lastSafeNs) * 1e-9 });
            } else StraightBrakes++;
        }

        /// <summary>Velocity [m/s] that follows the validated path: tangent x ramped speed min(v0 - a t, sqrt(2 a (SafeArc - s)))
        /// plus a bounded cross-track correction toward the path. Kinematic only.</summary>
        Vec3 PathBrakeVelocity(long tNs, StateEstimate est, out Vec3 accFF) {
            accFF = new Vec3();
            var P = brakePath.Path; var A = brakePath.PathArc; var p = est.P;
            int lo = Math.Max(0, brakeIdx - 2), hi = Math.Min(P.Length - 2, brakeIdx + 60);
            double best = double.MaxValue, sBest = 0; Vec3 foot = P[0]; int iBest = brakeIdx;
            for (int i = lo; i <= hi; i++) {
                var ab = P[i + 1] - P[i]; double L2 = ab.Dot(ab); if (L2 < 1e-12) continue;
                double u = Math.Max(0, Math.Min(1, (p - P[i]).Dot(ab) / L2)); var f = P[i] + ab * u; double d = (p - f).Norm;
                if (d < best) { best = d; foot = f; sBest = A[i] + u * (A[i + 1] - A[i]); iBest = i; }
            }
            brakeIdx = iBest;
            double t = (tNs - brakeT0) * 1e-9;
            double vb = Math.Max(0, brakeV0 - c.BrakeDecel * t);
            vb = Math.Min(vb, Math.Sqrt(2 * c.BrakeDecel * Math.Max(0, brakePath.SafeArc - sBest)));
            if (vb <= 0.05) return new Vec3();
            // pure pursuit on a lookahead of one velocity-loop time constant, plus centripetal feed-forward (m/s^2)
            var tan0 = TangentAt(sBest, out _); var tan1 = TangentAt(sBest + 0.5, out _);
            var look = PointAt(sBest + Math.Max(0.3, vb * c.TrackTau));
            var dir = (look - p).Normalized; if (dir.Norm < 0.5) dir = tan0;
            var kappa = (tan1 - tan0) * (1.0 / 0.5);
            accFF = kappa * (vb * vb);   // centripetal only: a tangential decel feed-forward winds up the velocity integrator
            var lat = (foot - p) * c.PathTrackGain; double ln = lat.Norm;
            if (ln > c.PathTrackMaxLat) lat = lat * (c.PathTrackMaxLat / ln);
            return dir * vb + lat * 0.5;
        }

        Vec3 PointAt(double sArc) {
            var P = brakePath.Path; var A = brakePath.PathArc;
            if (sArc >= A[A.Length - 1]) { var d = (P[P.Length - 1] - P[P.Length - 2]).Normalized; return P[P.Length - 1] + d * (sArc - A[A.Length - 1]); }
            int i = Math.Max(0, brakeIdx - 2); while (i + 2 < A.Length && A[i + 1] < sArc) i++;
            double u = (sArc - A[i]) / Math.Max(1e-9, A[i + 1] - A[i]); return P[i] + (P[i + 1] - P[i]) * Math.Max(0, Math.Min(1, u));
        }

        Vec3 TangentAt(double sArc, out int seg) {
            var P = brakePath.Path; var A = brakePath.PathArc;
            int i = Math.Max(0, brakeIdx - 2); while (i + 2 < A.Length && A[i + 1] < sArc) i++;
            seg = i; return (P[i + 1] - P[i]).Normalized;
        }

        public void BeginRecovery(long tNs, double beliefDist) {
            if (tNs < RecoveryUntilNs) return;
            Recoveries++; RecoveryUntilNs = tNs + (long)(c.RecoveryHoldS * 1e9);
            rec.Event(tNs, "autonomy_arbiter", new Dictionary<string, object> { ["action"] = "recover", ["reason"] = "belief_near_miss", ["esdf_m"] = beliefDist });
        }

        public bool InRecovery(long tNs) => tNs < RecoveryUntilNs;

        /// <summary>Every tick: decide the setpoint and submit it.</summary>
        public void Tick(long tNs, StateEstimate est, double dtS) {
            var v = PlannerVel; string reason = null;
            if (LastPlannerApplyNs == long.MinValue) { v = new Vec3(); reason = "init"; }
            else if (tNs - LastPlannerApplyNs > (long)(c.WatchdogS * 1e9) && v.Norm > 1e-6) {
                if (Mode != "watchdog") { WatchdogTrips++; rec.Event(tNs, "autonomy_arbiter", new Dictionary<string, object> { ["action"] = "brake", ["reason"] = "stall_watchdog", ["since_cmd_s"] = (tNs - LastPlannerApplyNs) * 1e-9 }); }
                Mode = "watchdog"; PlannerVel = v = new Vec3(); reason = "watchdog";
                BeginBrake(tNs, "stall_watchdog");
            }
            var acc = new Vec3(); bool hold = false;
            if (brakePath != null && (Mode == "watchdog" || Mode == "no_safe_maneuver")) {
                v = PathBrakeVelocity(tNs, est, out acc); reason = "brake_along_path";
                if (v.Norm < 1e-9 && est.V.Norm < 0.3) { hold = true; reason = "brake_hold"; }   // brake-to-hover ends in a position hold
            }
            if (!est.Valid || est.InputAgeNs > (long)(c.EstimateStaleS * 1e9)) {
                if (Mode != "estimate_stale") { EstimateStops++; rec.Event(tNs, "autonomy_arbiter", new Dictionary<string, object> { ["action"] = "brake", ["reason"] = "estimate_stale", ["age_s"] = est.InputAgeNs * 1e-9 }); }
                Mode = "estimate_stale"; v = new Vec3(); acc = new Vec3(); hold = false; reason = "estimate";
            }
            lastV = est.V;
            // yaw: toward the planner heading, rate-limited 1.5 rad/s
            double yawNow = est.Pose.Yaw;
            if (double.IsNaN(yawCmd)) yawCmd = yawNow;
            if (!double.IsNaN(PlannerYaw)) {
                double e = PrimitivePlanner.Wrap(PlannerYaw - yawCmd), lim = 1.5 * dtS;
                yawCmd = PrimitivePlanner.Wrap(yawCmd + Math.Max(-lim, Math.Min(lim, e)));
            }
            fc.Submit(new TrajectorySetpoint { TNs = tNs, ValidUntilNs = tNs + (long)(c.WatchdogS * 1e9), Velocity = v, Acceleration = acc, Hold = hold, Yaw = yawCmd, Source = reason ?? Mode, SensorAcqNs = lastSensorAcq });
        }
    }
}
