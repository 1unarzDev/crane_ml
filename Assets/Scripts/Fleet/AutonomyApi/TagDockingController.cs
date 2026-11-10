using System;
using System.Collections.Generic;
using Crane.Fleet.Docking;
using Crane.Fleet.Perception;

namespace Crane.Fleet.Autonomy {
    /// <summary>
    /// CRANE's fallback closed-loop docking controller (until the autonomy library lands). Uses ONLY the rover's own I/O: tag observations
    /// of the target rover from its own cameras (perception core: noise, latency, dropout, ambiguity flag) and its odometry estimate.
    /// The target is assumed stationary while docking. Each unflagged observation T_self_target (acquired at stamp t_a) is composed with the
    /// odometry pose at t_a, giving the target pose in the odometry frame; those are averaged. The controller drives the odometry pose to
    /// (1) a stand-off pose where the tag is in decode range, (2) after enough fresh observations, straight onto the mating pose
    /// (2 mm past it) at a slow approach speed, issuing connector Approach, then EngageHooks once captured, Verify when both hooks read
    /// engaged. The last stretch is below the camera's minimum range, so it runs on odometry only. A failed attempt (connector back to
    /// FREE) returns to the stand-off pose and re-observes.
    /// </summary>
    public sealed class TagDockingController {
        public enum Phase { Search, Standoff, Observe, Approach, Hooks, Latched, Abandoned }

        readonly IRoverIO io;
        readonly string targetId, myFace, targetFace;
        readonly Pose2 mateRel;        // desired pose of the target base in my base frame at mating
        readonly double standoff, speed, approachSpeed, kp, kyaw, wmax, posTol, yawTol;
        readonly int minObs;
        readonly List<(double t, Pose2 pose)> odoHistory = new List<(double, Pose2)>();
        double sx, sy, sc, ss; int n;   // running mean of the target pose in the odometry frame (yaw by its unit vector)
        double sxx, syy, sxy;            // second moments for the covariance of the mean (x, y, yaw about the first sample)
        double yaw0;
        /// <summary>Covariance of the windowed mean target pose [xx, xy, yy, yawyaw] (sample covariance / n); NaN with fewer than 2 samples.
        /// Excludes odometry drift since acquisition and the observations' own (flagged) covariance: a lower bound.</summary>
        public double[] TargetCovariance {
            get {
                if (n < 2) return new[] { double.NaN, double.NaN, double.NaN, double.NaN };
                double mx = sx / n, my = sy / n;
                double cxx = (sxx / n - mx * mx) * n / (n - 1), cyy = (syy / n - my * my) * n / (n - 1), cxy = (sxy / n - mx * my) * n / (n - 1);
                double cyaw = (yawSq / n - yawSum / n * (yawSum / n)) * n / (n - 1);
                return new[] { cxx / n, cxy / n, cyy / n, cyaw / n };
            }
        }
        double yawSum, yawSq;
        public Phase State { get; private set; } = Phase.Search;
        public int ObservationsUsed => n;
        public int ObservationsRejected;
        Pose2 lastTarget = new Pose2(double.NaN, double.NaN, double.NaN);
        /// <summary>Mean target pose (odometry frame) over the current window; the previous window's mean while the new one is empty.</summary>
        public Pose2 TargetEstimate => n == 0 ? lastTarget : new Pose2(sx / n, sy / n, Math.Atan2(ss, sc));
        public string LastNote = "";
        int ambiguousOrForeign;
        double phaseStart;
        int attempt;
        public int Attempts => attempt;
        public double LastObservationRange = double.NaN;

        public TagDockingController(IRoverIO io, string targetId, RoverDescription target, string myFace, string targetFace, JsonNode p) {
            this.io = io; this.targetId = targetId; this.myFace = myFace; this.targetFace = targetFace;
            // mate: my face frame and the target's face frame coincide with opposite normals
            var mine = io.Description.Face(myFace).Pose; var theirs = target.Face(targetFace).Pose;
            double yaw = Angles.Wrap(mine.Yaw + Math.PI - theirs.Yaw);
            double c = Math.Cos(yaw), s = Math.Sin(yaw);
            mateRel = new Pose2(mine.X - (c * theirs.X - s * theirs.Y), mine.Y - (s * theirs.X + c * theirs.Y), yaw);
            standoff = p["standoff"].Double; speed = p["speed"].Double; approachSpeed = p["approach_speed"].Double;
            kp = p["kp_pos"].Double; kyaw = p["kp_yaw"].Double; wmax = p["max_yaw_rate"].Double;
            posTol = p["pos_tolerance"].Double; yawTol = p["yaw_tolerance"].Double; minObs = (int)p["min_observations"].Double;
        }

        static Pose2 Compose(Pose2 a, Pose2 b) {
            double c = Math.Cos(a.Yaw), s = Math.Sin(a.Yaw);
            return new Pose2(a.X + c * b.X - s * b.Y, a.Y + s * b.X + c * b.Y, Angles.Wrap(a.Yaw + b.Yaw));
        }
        static Pose2 Inverse(Pose2 a) {
            double c = Math.Cos(a.Yaw), s = Math.Sin(a.Yaw);
            return new Pose2(-(c * a.X + s * a.Y), -(-s * a.X + c * a.Y), -a.Yaw);
        }

        Pose2 OdoAt(double t) {
            Pose2 best = odoHistory.Count > 0 ? odoHistory[0].pose : new Pose2(0, 0, 0);
            for (int i = odoHistory.Count - 1; i >= 0; i--) if (odoHistory[i].t <= t + 1e-9) { best = odoHistory[i].pose; break; }
            return best;
        }

        /// <summary>One control step at time t. Returns true when latched (or abandoned).</summary>
        public bool Step(double t) {
            var odo = io.Odometry;
            var me = new Pose2(odo.X, odo.Y, odo.Yaw);
            odoHistory.Add((t, me));
            if (odoHistory.Count > 2000) odoHistory.RemoveRange(0, 500);

            foreach (var o in io.Sensors.PollTagObservations(t)) {
                if (o.SourceKind != ObservationSourceKind.OwnCamera || o.FrameChild != targetId + "/base_link") continue;
                if (o.AmbiguousPose) { ambiguousOrForeign++; ObservationsRejected++; continue; }
                var rel = new Pose2(o.Pose.P.X, o.Pose.P.Y, o.Pose.Yaw);
                LastObservationRange = Math.Sqrt(rel.X * rel.X + rel.Y * rel.Y);
                if (State == Phase.Approach || State == Phase.Hooks || State == Phase.Latched) continue;   // too close to trust; odometry only
                var w = Compose(OdoAt(o.StampAcquired), rel);
                if (n == 0) yaw0 = w.Yaw;
                double dyaw = Angles.Wrap(w.Yaw - yaw0);
                sx += w.X; sy += w.Y; sc += Math.Cos(w.Yaw); ss += Math.Sin(w.Yaw); n++;
                sxx += w.X * w.X; syy += w.Y * w.Y; sxy += w.X * w.Y; yawSum += dyaw; yawSq += dyaw * dyaw;
            }

            var conn = io.Connector(myFace);
            switch (State) {
                case Phase.Search:
                    io.CommandTwist(new Twist());
                    if (n > 0) Enter(Phase.Standoff, t, "target tag seen");
                    return false;
                case Phase.Standoff:
                case Phase.Observe: {
                    var goal = Compose(TargetEstimate, Inverse(Shift(mateRel, standoff)));
                    var cmd = GoToController.Command(me, goal.X, goal.Y, goal.Yaw, kp, kyaw, speed, wmax, out double d, out double ey);
                    io.CommandTwist(cmd);
                    if (State == Phase.Standoff && d < posTol && Math.Abs(ey) < yawTol) { Enter(Phase.Observe, t, "at stand-off"); ResetMean(); }
                    if (State == Phase.Observe && n >= minObs && d < posTol && Math.Abs(ey) < yawTol) {
                        Enter(Phase.Approach, t, $"{n} observations averaged");
                        io.CommandConnector(myFace, DockingCommand.Approach);
                        attempt++;
                    }
                    return false;
                }
                case Phase.Approach:
                case Phase.Hooks: {
                    var goal = Compose(TargetEstimate, Inverse(Shift(mateRel, -0.002)));
                    double v = conn.State == "APPROACH" ? approachSpeed : approachSpeed * 0.3;
                    io.CommandTwist(GoToController.Command(me, goal.X, goal.Y, goal.Yaw, kp, kyaw, v, wmax, out _, out _));
                    if (conn.State == "MAGNET_CAPTURED" && State == Phase.Approach) { io.CommandConnector(myFace, DockingCommand.EngageHooks); Enter(Phase.Hooks, t, "captured"); }
                    if ((conn.State == "HOOK_A_ENGAGED" || conn.State == "HOOK_B_ENGAGED") && conn.SensedA && conn.SensedB) io.CommandConnector(myFace, DockingCommand.Verify);
                    if (conn.State == "PARTIAL" || conn.State == "FALSE_ENGAGEMENT") io.CommandConnector(myFace, DockingCommand.Release);
                    if (conn.Coupled) { io.CommandTwist(new Twist()); Enter(Phase.Latched, t, "latched"); return true; }
                    if ((conn.State == "FREE" || conn.State == "RELEASED") && t - phaseStart > 0.5) {
                        if (conn.RetriesExhausted) { Enter(Phase.Abandoned, t, "retries exhausted"); io.CommandTwist(new Twist()); return true; }
                        io.CommandConnector(myFace, DockingCommand.None);
                        Enter(Phase.Standoff, t, "attempt failed: back to stand-off"); ResetMean();
                    }
                    if (t - phaseStart > 30) { io.CommandConnector(myFace, DockingCommand.Release); Enter(Phase.Standoff, t, "approach timeout"); ResetMean(); }
                    return false;
                }
                default:
                    io.CommandTwist(new Twist());
                    return true;
            }
        }

        /// <summary>The mating relative pose shifted so I stand <paramref name="gap"/> further back along my own heading.</summary>
        static Pose2 Shift(Pose2 rel, double gap) => new Pose2(rel.X + gap, rel.Y, rel.Yaw);

        void ResetMean() { if (n > 0) lastTarget = TargetEstimate; sx = sy = sc = ss = 0; sxx = syy = sxy = yawSum = yawSq = 0; n = 0; }

        void Enter(Phase p, double t, string note) {
            State = p; phaseStart = t; LastNote = note;
            io.Report("closed_loop_tag_docking", p.ToString());
        }
    }
}
