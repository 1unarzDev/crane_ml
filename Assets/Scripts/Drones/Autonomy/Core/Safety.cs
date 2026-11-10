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
        private readonly CraneMultirotorController ctl;
        public FcStatus Status { get; } = new FcStatus();
        public TrajectorySetpoint Last { get; private set; }
        public InProcessControllerLink(CraneMultirotorController controller) { ctl = controller; }
        public void Submit(TrajectorySetpoint sp) {
            Last = sp; Status.LastSetpointNs = sp.TNs;
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

        public SetpointArbiter(AutonomyConfig cfg, IFlightControllerLink link, IAutonomyRecorder recorder) { c = cfg; fc = link; rec = recorder; }

        public void ApplyPlanner(long tNs, PlanDecision d, long sensorAcqNs) {
            LastPlannerApplyNs = tNs; lastSensorAcq = sensorAcqNs;
            if (!d.FeasibleAny) {
                if (Mode != "no_safe_maneuver") { NoSafeStops++; rec.Event(tNs, "autonomy_arbiter", new Dictionary<string, object> { ["action"] = "brake", ["reason"] = "no_safe_maneuver", ["d_free_best"] = d.DFree }); }
                Mode = "no_safe_maneuver"; PlannerVel = new Vec3();
                PlannerYaw = d.Heading;   // turn toward the most open direction while braking (sensor coverage)
            } else { Mode = "planner"; PlannerVel = d.VelocityCmd; if (d.VCmd > 0.2) PlannerYaw = d.Heading; }
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
            }
            if (!est.Valid || est.InputAgeNs > (long)(c.EstimateStaleS * 1e9)) {
                if (Mode != "estimate_stale") { EstimateStops++; rec.Event(tNs, "autonomy_arbiter", new Dictionary<string, object> { ["action"] = "brake", ["reason"] = "estimate_stale", ["age_s"] = est.InputAgeNs * 1e-9 }); }
                Mode = "estimate_stale"; v = new Vec3(); reason = "estimate";
            }
            // yaw: toward the planner heading, rate-limited 1.5 rad/s
            double yawNow = est.Pose.Yaw;
            if (double.IsNaN(yawCmd)) yawCmd = yawNow;
            if (!double.IsNaN(PlannerYaw)) {
                double e = PrimitivePlanner.Wrap(PlannerYaw - yawCmd), lim = 1.5 * dtS;
                yawCmd = PrimitivePlanner.Wrap(yawCmd + Math.Max(-lim, Math.Min(lim, e)));
            }
            fc.Submit(new TrajectorySetpoint { TNs = tNs, ValidUntilNs = tNs + (long)(c.WatchdogS * 1e9), Velocity = v, Yaw = yawCmd, Source = reason ?? Mode, SensorAcqNs = lastSensorAcq });
        }
    }
}
