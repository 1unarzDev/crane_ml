// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using System.Collections.Generic;
using Sim.Physics.Aerial;

namespace Crane.Drones.Autonomy {
    public sealed class MissionSpec {
        public string Name = "mission";
        public int ForestSeed;
        /// <summary>Global (mission/prior) frame ENU. Route ends at the goal; Route[0] should be the start.</summary>
        public Vec3 Start; public double StartYaw;
        public List<Vec3> Route = new List<Vec3>();
        public Vec3 Goal => Route[Route.Count - 1];
    }

    public sealed class MissionResult {
        public string Name, Planner, Outcome = "running", FailureCause = "";
        public int ForestSeed; public ulong RunSeed; public double VLimit;
        public bool Success, Collision;
        public double TimeS, PathLengthM, RouteLengthM, ProgressM, ProgressFrac, MeanSpeed, MaxSpeed;
        public double MinClearanceHardM = double.PositiveInfinity, MinClearanceAllM = double.PositiveInfinity;
        public string MinClearanceObject = "";
        public int TwigContacts;
        public double CollisionSpeed; public string CollisionObject = ""; public double CollisionTimeS;
        public double AgeP50S, AgeP99S, AgeMaxS, ComputeP99S, TauFinalS;
        public int Cycles, DeadlineMisses, AgeGateMisses, WatchdogTrips, NoSafeStops, EstimateStops, Recoveries, MarginViolations, MarginTicks;
        public double FinalLocalErrorM, FinalGoalErrorM;
        public long DepthFrames, LidarScans, LidarPoints;
        public double WallS;

        public Dictionary<string, object> ToDict() {
            var d = new Dictionary<string, object>();
            foreach (var f in typeof(MissionResult).GetFields()) {
                var v = f.GetValue(this);
                if (v is double x && (double.IsInfinity(x) || double.IsNaN(x))) v = null;
                if (v is ulong u) v = (double)u;
                d[f.Name] = v;
            }
            return d;
        }
    }

    /// <summary>
    /// One closed-loop autonomous mission, pure C#, lockstep with the plant (M5). Per 5 ms tick:
    /// estimator update -> sensors acquire (truth pose, per-point LiDAR timing) -> planner cycle if due (consume frames
    /// available by now, insert at the estimated poses, window ESDF, cost-to-go, primitive planning; the command takes
    /// effect after a sampled compute time) -> arbiter / watchdog -> plant step (1 kHz RK4 sub-steps through
    /// CraneMultirotorController) -> ground-truth metrics from the manifest (clearance, collision, stopping margin).
    /// </summary>
    public sealed class AutonomousMission {
        public readonly MissionSpec Spec; public readonly AutonomyConfig C; public readonly VehicleSpecV2 Vehicle;
        public readonly CraneAerialPlantCore Plant; public readonly CraneMultirotorController Ctl;
        public readonly DriftingTruthEstimator Estimator; public readonly SparseVoxelMap Map; public readonly SparseVoxelMap EphemeralMap;
        public readonly PlanningWindow Window = new PlanningWindow();
        public readonly CorridorRouter Router; public readonly PrimitivePlanner Planner; public readonly StopRule Rule;
        public readonly SetpointArbiter Arbiter; public readonly InProcessControllerLink Link;
        public readonly IDepthSensor Depth; public readonly ILidarSensor Lidar;
        public readonly IRayWorld SensorWorld; public readonly IGroundTruthWorld Truth; public readonly IRayWorld TruthRays;
        public readonly IAutonomyRecorder Rec;
        public readonly MissionResult Result = new MissionResult();
        public readonly double BodyRadius;
        /// <summary>Advances the plant by one tick; default Plant.Advance(tick). Unity replaces it with the logged flight step.</summary>
        public Action StepPlant;
        public Action<DepthFrame> DepthAcquired; public Action<LidarScan> LidarCompleted;
        public long Tick { get; private set; }
        public long TNs => Tick * C.TickNs;
        public bool Done { get; private set; }

        private readonly List<DepthFrame> depthQ = new List<DepthFrame>(); private readonly List<LidarScan> lidarQ = new List<LidarScan>();
        private long nextPlanNs, newestAcqNs = -1, planPeriodNs;
        private (long apply, PlanDecision d, long acq)? pending;
        private readonly ComputeTimeModel compute;
        private readonly List<double> ages = new List<double>(), comps = new List<double>();
        private double bestArc, bestArcT; private Vec3 prevP; private double speedSum; private long speedN;
        private readonly HashSet<int> twigs = new HashSet<int>();
        private readonly double timeLimit;
        private readonly CorridorRouter truthRoute;   // global-frame route geometry for progress metrics
        private readonly System.Diagnostics.Stopwatch wall = new System.Diagnostics.Stopwatch();

        public AutonomousMission(MissionSpec spec, AutonomyConfig cfg, VehicleSpecV2 vehicle, IRayWorld sensorWorld, IGroundTruthWorld truth,
                IRayWorld truthRays, IPriorMap prior, IAutonomyRecorder rec, StopRule rule) {
            Spec = spec; C = cfg; Vehicle = vehicle; SensorWorld = sensorWorld; Truth = truth; TruthRays = truthRays; Rec = rec ?? NullRecorder.Instance;
            double br = 0;
            foreach (var r in vehicle.Rotors) br = Math.Max(br, Math.Sqrt(r.Position[0] * r.Position[0] + r.Position[1] * r.Position[1]) + r.Radius);
            BodyRadius = br; if (C.BodyRadius <= 0) C.BodyRadius = br;
            Plant = new CraneAerialPlantCore(vehicle, 1000.0, 500.0);
            Plant.AttachImu(new CraneAerialImu(500.0, vehicle.Gravity, cfg.RunSeed));
            Ctl = new CraneMultirotorController(vehicle, Plant.ControlDt);
            Plant.CommandSource = Ctl;
            var s0 = CraneAerialPlantCore.HoverState(vehicle, spec.Start.ToArray(), spec.StartYaw);
            Plant.Reset(s0, cfg.RunSeed); Ctl.ResetIntegrators(); Ctl.HoldPosition(s0.P, spec.StartYaw);
            StepPlant = () => Plant.Advance(C.TickNs * 1e-9);
            Estimator = new DriftingTruthEstimator(cfg);
            Estimator.Reset(Pose.FromQuat(spec.Start, s0.Q), new Vec3(), 0, cfg.RunSeed);
            Map = new SparseVoxelMap(cfg); EphemeralMap = new SparseVoxelMap(cfg);
            Router = new CorridorRouter(cfg, spec.Route, cfg.PriorUse == "none" ? null : prior) { Window = Window };
            truthRoute = new CorridorRouter(cfg, spec.Route, null);
            truthRoute.Update(0, Estimator.Current, new MapAlignment(), null);
            double aMax = 0.8 * Math.Min(Ctl.G.MaxAccelXY, vehicle.Gravity * Math.Tan(Ctl.G.MaxTilt));
            Planner = new PrimitivePlanner(cfg, cfg.Planner == "reactive", aMax) { Window = Window, Router = Router };
            Rule = rule;
            Link = new InProcessControllerLink(Ctl);
            Arbiter = new SetpointArbiter(cfg, Link, Rec);
            Depth = cfg.UseDepth ? new DepthRaycastSensor(cfg, sensorWorld, cfg.RunSeed, 0) : null;
            Lidar = cfg.UseLidar ? new LidarL2Sensor(cfg, sensorWorld, cfg.RunSeed) : null;
            compute = new ComputeTimeModel(new Rng(cfg.RunSeed, 41), cfg.PlanMedianS, cfg.PlanSigma, cfg.PlanPStall, cfg.PlanStallMinS, cfg.PlanStallAlpha);
            planPeriodNs = (long)Math.Round(1e9 / cfg.PlanRateHz); nextPlanNs = 60_000_000;
            Result.Name = spec.Name; Result.ForestSeed = spec.ForestSeed; Result.RunSeed = cfg.RunSeed; Result.VLimit = cfg.VLimit; Result.Planner = Planner.Name;
            Result.RouteLengthM = truthRoute.RouteLength;
            timeLimit = cfg.TimeLimitS > 0 ? cfg.TimeLimitS : Result.RouteLengthM / 0.5 + 30.0;
            prevP = spec.Start;
            Rec.Event(0, "autonomy_start", new Dictionary<string, object> {
                ["mission"] = spec.Name, ["forest_seed"] = spec.ForestSeed, ["run_seed"] = (double)cfg.RunSeed, ["planner"] = Planner.Name, ["v_limit"] = cfg.VLimit,
                ["body_radius_m"] = BodyRadius, ["route_length_m"] = Result.RouteLengthM, ["start"] = spec.Start.ToArray(), ["goal"] = spec.Goal.ToArray(),
                ["brake_table_v"] = rule.Vs, ["brake_table_d"] = rule.Ds, ["config"] = cfg.Canonical(), ["prior"] = prior?.Describe ?? "none" });
        }

        static Pose TruthPose(StateV2 s) => Pose.FromQuat(Vec3.FromArray(s.P), s.Q);

        public MissionResult Run() { while (!Done) Step(); return Result; }

        public void Step() {
            if (Done) return;
            if (!wall.IsRunning) wall.Start();
            phase.Restart();
            long t = TNs; var st = Plant.State; var tp = TruthPose(st); var tv = Vec3.FromArray(st.V);
            Estimator.Update(t, tp, tv);
            // ---- sensors
            if (Depth != null) { var f = Depth.Tick(t, tp, tv, Vec3.FromArray(st.Omega)); if (f != null) { depthQ.Add(f); Result.DepthFrames++; DepthAcquired?.Invoke(f);
                    Rec.Event(t, "autonomy_sensor", new Dictionary<string, object> { ["sensor"] = f.Sensor, ["acq_ns"] = f.AcqNs, ["avail_ns"] = f.AvailNs, ["hits"] = f.TruthHits, ["invalid"] = f.InvalidPixels, ["dropped_objects"] = f.DroppedObjects }); } }
            if (Lidar != null) { var s = Lidar.Tick(t, C.TickNs, tp); if (s != null) { lidarQ.Add(s); Result.LidarScans++; Result.LidarPoints += s.Count; LidarCompleted?.Invoke(s);
                    Rec.Event(t, "autonomy_sensor", new Dictionary<string, object> { ["sensor"] = s.Sensor, ["start_ns"] = s.StartNs, ["end_ns"] = s.EndNs, ["avail_ns"] = s.AvailNs, ["points"] = s.Count, ["misses"] = s.MissX.Length }); } }
            var est = Estimator.Current;
            Lap(0);
            // ---- planner cycle
            if (t >= nextPlanNs && pending == null) PlanCycle(t, est);
            if (pending.HasValue && t >= pending.Value.apply) { Arbiter.ApplyPlanner(t, pending.Value.d, pending.Value.acq); pending = null; }
            Arbiter.Tick(t, est, C.TickNs * 1e-9);
            phase.Restart();
            // ---- plant
            StepPlant();
            Tick++;
            Metrics();
            Lap(5);
        }

        /// <summary>Wall-clock seconds per phase (diagnostics only; never fed back into the simulation).</summary>
        public readonly double[] PhaseWall = new double[6];
        public static readonly string[] PhaseNames = { "sensors", "insert", "window", "router", "plan", "plant+metrics" };
        private readonly System.Diagnostics.Stopwatch phase = new System.Diagnostics.Stopwatch();
        void Lap(int i) { PhaseWall[i] += phase.Elapsed.TotalSeconds; phase.Restart(); }

        void PlanCycle(long t, StateEstimate est) {
            bool reactive = Planner.Name == "reactive";
            var target = reactive ? EphemeralMap : Map;
            if (reactive) EphemeralMap.Clear();
            DepthFrame lastD = null; LidarScan lastL = null;
            int nd = 0, nl = 0;
            for (int i = depthQ.Count - 1; i >= 0; i--) if (depthQ[i].AvailNs <= t) {
                    var f = depthQ[i]; depthQ.RemoveAt(i);
                    if (nd++ >= 3) continue;   // bounded queue: newest 3 per sensor
                    if (Estimator.PoseAt(f.AcqNs, out Pose bp)) target.InsertDepth(f, DepthRaycastSensor.Compose(bp, f.MountInBody), t);
                    newestAcqNs = Math.Max(newestAcqNs, f.AcqNs); lastD = f;
                }
            for (int i = lidarQ.Count - 1; i >= 0; i--) if (lidarQ[i].AvailNs <= t) {
                    var s = lidarQ[i]; lidarQ.RemoveAt(i);
                    if (nl++ >= 3) continue;
                    target.InsertLidar(s, tq => Estimator.PoseAt(tq, out Pose p) ? p : (Pose?)null, t);
                    newestAcqNs = Math.Max(newestAcqNs, s.EndNs); lastL = s;
                }
            Lap(1);
            var align = Estimator.Alignment;
            Window.Build(target, est.P, C.MapWindowXY, C.MapWindowZDown, C.MapWindowZUp, C.PriorUse == "belief" ? PriorForBelief : null, align);
            Lap(2);
            Router.Update(t, est, align, target);
            Lap(3);
            double zT = Router.RouteAltitude(est.P, out _, out _);
            Window.Index(est.P, out int w0);
            double beliefDist = w0 >= 0 ? Window.Dist[w0] : double.PositiveInfinity;
            if (beliefDist < BodyRadius + C.NearMissRecoverM) { Arbiter.BeginRecovery(t, beliefDist); Router.ForceRecompute(); }
            double vCap = C.VLimit * (Arbiter.InRecovery(t) ? C.RecoverySpeedFrac : 1.0);
            var dec = Planner.Plan(t, est, Router.GoalLocal, vCap, target, Router, Rule, zT);
            Lap(4);
            double cS = compute.Sample(); comps.Add(cS);
            long apply = t + (long)Math.Round(cS * 1e9);
            dec.ApplyNs = apply;
            Result.Cycles++;
            if (cS * 1e9 > planPeriodNs) Result.DeadlineMisses++;
            if (newestAcqNs >= 0) {
                double age = (apply - newestAcqNs) * 1e-9; ages.Add(age); Rule.ObserveAge(age);
                if (age > 0.12) Result.AgeGateMisses++;
            }
            pending = (apply, dec, newestAcqNs);
            nextPlanNs = t + Math.Max(planPeriodNs, apply - t);
            if (Result.Cycles % 2 == 0 || !dec.FeasibleAny)
                Rec.Event(t, "autonomy_plan", new Dictionary<string, object> {
                    ["planner"] = dec.Planner, ["apply_ns"] = apply, ["feasible"] = dec.FeasibleCount, ["candidates"] = dec.Candidates, ["v_cmd"] = dec.VCmd,
                    ["heading"] = dec.Heading, ["climb"] = dec.Climb, ["d_free"] = dec.DFree, ["stop_dist"] = dec.StopDist, ["tau_age"] = dec.TauAge,
                    ["progress_g"] = dec.Progress, ["esdf_here"] = double.IsInfinity(beliefDist) ? -1 : beliefDist, ["est_p"] = est.P.ToArray(), ["z_target"] = zT,
                    ["newest_acq_ns"] = newestAcqNs, ["compute_s"] = cS, ["note"] = dec.Note, ["recovery"] = Arbiter.InRecovery(t) });
        }

        /// <summary>Latest ground-truth hard clearance (body radius subtracted), refreshed every 4 ticks (ClearanceUpdated).</summary>
        public double LastHardClearance = double.PositiveInfinity; public string LastHardObject = ""; public bool ClearanceUpdated;
        /// <summary>Prior used inside the belief window (PriorUse = belief).</summary>
        public IPriorMap PriorForBelief;

        void Metrics() {
            var st = Plant.State; var p = Vec3.FromArray(st.P); var v = Vec3.FromArray(st.V); double t = TNs * 1e-9;
            double sp = v.Norm; speedSum += sp; speedN++; Result.MaxSpeed = Math.Max(Result.MaxSpeed, sp);
            Result.PathLengthM += (p - prevP).Norm; prevP = p;
            if (Tick % 4 == 0) {
                double hard = Truth.Clearance(p, 1.5, true, out int hid, out _) - BodyRadius;
                double all = Truth.Clearance(p, 1.0, false, out int aid, out _) - BodyRadius;
                LastHardClearance = hard; LastHardObject = Truth.ObjectName(hid); ClearanceUpdated = true;
                if (hard < Result.MinClearanceHardM) { Result.MinClearanceHardM = hard; Result.MinClearanceObject = Truth.ObjectName(hid); }
                Result.MinClearanceAllM = Math.Min(Result.MinClearanceAllM, all);
                if (all < 0 && aid >= 0 && twigs.Add(aid)) { Result.TwigContacts++; Rec.Event(TNs, "autonomy_twig_contact", new Dictionary<string, object> { ["object"] = Truth.ObjectName(aid), ["speed"] = sp }); }
                if (hard < 0) {
                    Result.Collision = true; Result.CollisionSpeed = sp; Result.CollisionObject = Truth.ObjectName(hid); Result.CollisionTimeS = t;
                    Rec.Event(TNs, "collision", new Dictionary<string, object> { ["other"] = Result.CollisionObject, ["impact_speed_mps"] = sp, ["source"] = "manifest_clearance" });
                    Finish("collision", "collision"); return;
                }
                // true stopping margin along the current velocity (NavPilot n_margin_viol), belief tau and brake table
                if (sp > 0.5 && TruthRays != null) {
                    Result.MarginTicks++;
                    var d = v * (1.0 / sp);
                    double dTrue = TruthRays.Raycast(p, d, 15.0, 0.0, -2, out RayHit h) ? h.Distance - BodyRadius : 15.0;
                    if (dTrue < Rule.Dist(sp) - Rule.Margin) Result.MarginViolations++;
                }
                truthRoute.RouteAltitude(p, out double arc, out _);
                if (arc > bestArc + C.StuckProgressM) { bestArc = arc; bestArcT = t; }
                Result.ProgressM = Math.Max(Result.ProgressM, arc);
                if (Tick % 20 == 0) {
                    var e = Estimator.LocalError;
                    Rec.Event(TNs, "autonomy_estimate", new Dictionary<string, object> { ["truth_p"] = p.ToArray(), ["est_p"] = Estimator.Current.P.ToArray(), ["err_m"] = e.Norm, ["speed"] = sp, ["mode"] = Arbiter.Mode, ["map_occ"] = Map.OccupiedVoxels, ["map_known"] = Map.KnownVoxels });
                }
            }
            var g = Spec.Goal;
            if (Math.Sqrt((p.X - g.X) * (p.X - g.X) + (p.Y - g.Y) * (p.Y - g.Y)) < C.GoalRadius && Math.Abs(p.Z - g.Z) < 2.0) { Finish("success", ""); return; }
            if (t - bestArcT > C.StuckS) { Finish("stuck", "local_minimum_or_blocked"); return; }
            if (t > timeLimit) { Finish("timeout", "time_limit"); return; }
        }

        void Finish(string outcome, string cause) {
            Done = true; wall.Stop();
            var r = Result; r.Outcome = outcome; r.FailureCause = cause; r.Success = outcome == "success"; r.TimeS = TNs * 1e-9;
            r.ProgressFrac = r.RouteLengthM > 0 ? Math.Min(1.0, r.ProgressM / r.RouteLengthM) : 0; if (r.Success) r.ProgressFrac = 1.0;
            r.MeanSpeed = speedN > 0 ? speedSum / speedN : 0;
            r.AgeP50S = Q(ages, 0.5); r.AgeP99S = Q(ages, 0.99); r.AgeMaxS = ages.Count > 0 ? Max(ages) : double.NaN; r.ComputeP99S = Q(comps, 0.99); r.TauFinalS = Rule.Tau;
            r.WatchdogTrips = Arbiter.WatchdogTrips; r.NoSafeStops = Arbiter.NoSafeStops; r.EstimateStops = Arbiter.EstimateStops; r.Recoveries = Arbiter.Recoveries;
            r.FinalLocalErrorM = Estimator.LocalError.Norm;
            var p = Vec3.FromArray(Plant.State.P); r.FinalGoalErrorM = (p - Spec.Goal).NormXY;
            r.WallS = wall.Elapsed.TotalSeconds;
            Rec.Event(TNs, "autonomy_result", r.ToDict());
        }

        static double Max(List<double> a) { double m = double.MinValue; foreach (var x in a) m = Math.Max(m, x); return m; }
        public static double Q(List<double> a, double q) {
            if (a.Count == 0) return double.NaN; var s = a.ToArray(); Array.Sort(s);
            double pos = q * (s.Length - 1); int i = (int)Math.Floor(pos); double u = pos - i;
            return i + 1 < s.Length ? s[i] * (1 - u) + s[i + 1] * u : s[i];
        }
    }
}
