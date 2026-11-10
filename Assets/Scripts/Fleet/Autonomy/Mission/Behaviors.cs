using System;
using System.Collections.Generic;
using Crane.Fleet.Docking;
using Crane.Fleet.Perception;

namespace Crane.Fleet
{
    /// <summary>Plan-track-replan loop for one agent toward a goal of one of its member rovers. Replans on capability change
    /// (failure, attach, detach), on deviation, and on a failed plan (bounded).</summary>
    public sealed class Navigator
    {
        private readonly FleetAutonomy _f; private readonly string _rover, _task;
        public Pose2 Goal;            // goal of the member rover's base (world)
        public double Cruise;
        public ICollection<string> Ignore;   // rovers not treated as obstacles
        public int Replans, MaxReplans = 8;
        public string Failure; public FailureClass FailClass;
        private int _capVersion = -1, _relocEpoch; private Agent _agent; private bool _planned;
        public PlannedPath Path { get; private set; }
        public double PosTol = double.NaN, YawTol = double.NaN;
        public ICollection<string> IgnoreObstacles; public bool GoalSearch;

        public Navigator(FleetAutonomy f, string rover, Pose2 goal, double cruise, string task)
        { _f = f; _rover = rover; Goal = goal; Cruise = cruise; _task = task; }

        private bool Plan()
        {
            var ag = _f.AgentOf(_rover);
            _agent = ag;
            var goalAgent = Se2.Compose(Goal, Se2.Inverse(ag.Model.ModulePose(_rover)));
            var req = new PlanRequest { Start = ag.Pose, Goal = goalAgent, Capability = ag.Cap, CruiseFraction = Cruise, ExtraObstacles = _f.OtherFootprints(ag, Ignore), IgnoreObstacleIds = IgnoreObstacles, GoalSearch = GoalSearch };
            var p = _f.Planner.Plan(req);
            if (!p.Ok)
            {
                // retry once ignoring other rovers (they may be the reason the start is "in collision" after an estimate jump)
                req.ExtraObstacles = null;
                var p2 = _f.Planner.Plan(req);
                if (p2.Ok) p = p2;
            }
            Path = p; _capVersion = ag.Cap.Version;
            if (p.Ok && p.Escaped) _f.Log("escape_move", _task, ag.Id, "start in contact with " + string.Join(",", p.EscapedFrom) + ": escape first");
            if (p.Ok && p.GoalAdjusted) { Goal = Se2.Compose(p.GoalUsed, ag.Model.ModulePose(_rover)); _f.Log("goal_adjusted", _task, ag.Id, $"goal in collision; using ({Goal.X:0.000}, {Goal.Y:0.000}, {Goal.Yaw:0.000})"); }
            _f.Log("plan", _task, ag.Id, p.Ok ? $"ok {p.Points.Count} pts {p.Duration:0.0}s rank={ag.Cap.ActuatedRank} exp={p.Expansions}" : "failed " + p.Failure + $" rank={ag.Cap.ActuatedRank}");
            if (!p.Ok) { Failure = p.Failure; FailClass = p.Failure == "no_feasible_primitive" ? FailureClass.Traction : FailureClass.Geometry; return false; }
            ag.Tracker.SetPath(p, ag.Cap);
            if (!double.IsNaN(PosTol)) { ag.Tracker.PosTol = Math.Max(ag.Tracker.PosTol, PosTol); ag.Tracker.YawTol = Math.Max(ag.Tracker.YawTol, YawTol); }
            _planned = true;
            return true;
        }

        private int _stallPhase, _stalls; private double _stallT; private Twist2 _backDir;
        public int Stalls => _stalls;

        /// <summary>Stall response: back off along the reverse of the last commanded motion, hold still so absolute fixes can
        /// re-localise the (now uncertain) estimate, then replan from the new estimate.</summary>
        private TaskState HandleStall(Agent ag, double dt)
        {
            if (_stallPhase == 0)
            {
                _stalls++; _stallPhase = 1; _stallT = 0;
                if (_stalls > _f.A.MaxStallsPerTask) { Failure = "stalled " + _stalls + " times (" + ag.StallReason + ")"; FailClass = FailureClass.Traction; return TaskState.Failed; }
                var lc = ag.LastCommand; double n = lc.LinearSpeed;
                _backDir = n > 1e-6 ? new Twist2(-lc.Vx / n, -lc.Vy / n, 0) : Twist2.Zero;
                ag.Tracker.Stop(); ag.Tracker.ResetCommand();
                _f.Log("stall_response", _task, ag.Id, "back off and re-localise");
            }
            _stallT += dt;
            if (_stallPhase == 1)
            {
                double dur = _f.A.StallBackoffDistance / _f.A.StallBackoffSpeed;
                ag.Command = ag.Cap.Saturate(_backDir * _f.A.StallBackoffSpeed, out _); ag.CommandSet = true;
                if (_stallT >= dur) { _stallPhase = 2; _stallT = 0; }
                return TaskState.Running;
            }
            ag.Command = Twist2.Zero; ag.CommandSet = true;
            if (_stallT >= _f.A.StallHoldTime) { _f.ClearStall(ag); _stallPhase = 0; _planned = false; }
            return TaskState.Running;
        }

        public TaskState Tick(double dt)
        {
            var ag = _f.AgentOf(_rover);
            if (ag.Stalled || _stallPhase != 0) return HandleStall(ag, dt);
            if (ag.Cap.ActuatedRank < 3) MaxReplans = Math.Min(MaxReplans, 4);   // reduced-mobility plans are expensive: fail fast and let the executive retry
            if (!_planned || ag != _agent || ag.Cap.Version != _capVersion || ag.RelocEpoch != _relocEpoch)
            {
                if (_planned) Replans++;
                _relocEpoch = ag.RelocEpoch;
                if (Replans > MaxReplans) { Failure = "too many replans"; FailClass = FailureClass.Planner; return TaskState.Failed; }
                if (!Plan()) return TaskState.Failed;
            }
            var cmd = ag.Tracker.Step(ag.Pose, dt);
            ag.Command = cmd; ag.CommandSet = true;
            switch (ag.Tracker.Status)
            {
                case TrackStatus.Arrived: return TaskState.Succeeded;
                case TrackStatus.Deviated:
                    Replans++;
                    _f.Log("replan", _task, ag.Id, $"deviation {ag.Tracker.CrossTrackError:0.000} m");
                    if (Replans > MaxReplans) { Failure = "too many replans (deviation)"; FailClass = FailureClass.Traction; return TaskState.Failed; }
                    if (!Plan()) return TaskState.Failed;
                    return TaskState.Running;
                default: return TaskState.Running;
            }
        }
    }

    // ================================================================================================ goto
    /// <summary>Go to a goal pose, optionally through "route" waypoints ([x, y] corner points; the agent keeps its heading at
    /// intermediate points). Without "goal" the last route point is the goal (heading kept).</summary>
    public sealed class GotoBehavior : TaskBehavior
    {
        private Navigator _nav; private readonly List<Pose2> _wps = new List<Pose2>(); private int _k;
        public override IEnumerable<string> Rovers() { yield return S.Str("agent"); }
        public override void Start()
        {
            string rover = S.Str("agent");
            double yaw0 = F.AgentOf(rover).MemberPose(rover).Yaw;
            if (S.Raw.TryGetValue("route", out var rv) && rv is List<object> rl)
                foreach (var o in rl) if (o is List<object> pt && pt.Count >= 2) _wps.Add(new Pose2(Convert.ToDouble(pt[0]), Convert.ToDouble(pt[1]), pt.Count > 2 ? Convert.ToDouble(pt[2]) : yaw0));
            if (S.Nums("goal") != null) _wps.Add(S.PoseOf("goal"));
            if (_wps.Count == 0) throw new FormatException($"task {S.Id}: goto needs goal or route");
            Next();
        }
        private void Next()
        {
            bool last = _k == _wps.Count - 1;
            _nav = new Navigator(F, S.Str("agent"), _wps[_k], S.Num("cruise", F.A.CruiseFraction), S.Id);
            if (!last) { _nav.PosTol = 0.15; _nav.YawTol = Math.PI; }
            else { var tol = S.Nums("tol"); if (tol != null) { _nav.PosTol = tol[0]; _nav.YawTol = tol.Length > 1 ? tol[1] : F.A.GoalYawTol; } }
            SetPhase("navigate" + _k);
        }
        public override TaskState Tick(double dt)
        {
            var s = _nav.Tick(dt);
            if (s == TaskState.Failed) return Fail(_nav.FailClass, _nav.Failure);
            if (s == TaskState.Succeeded && _k < _wps.Count - 1) { _k++; Next(); return TaskState.Running; }
            return s;
        }
    }

    public sealed class WaitBehavior : TaskBehavior
    {
        private double _t0;
        public override IEnumerable<string> Rovers() { foreach (var a in S.Strs("agents")) yield return a; }
        public override void Start() { _t0 = F.Now; }
        public override TaskState Tick(double dt) => F.Now - _t0 >= S.Num("duration", 1.0) ? TaskState.Succeeded : TaskState.Running;
    }

    // ================================================================================================ explore / find
    /// <summary>Visit viewpoints covering a region until the target object's tag has been seen by the agent's own camera.</summary>
    public sealed class ExploreBehavior : TaskBehavior
    {
        private readonly List<Pose2> _views = new List<Pose2>(); private int _k; private Navigator _nav; private double _dwell;
        public override IEnumerable<string> Rovers() { yield return S.Str("agent"); }
        public override void Start()
        {
            var r = S.Nums("region"); double cx = (r[0] + r[2]) / 2, cy = (r[1] + r[3]) / 2;
            var vp = S.Nums("viewpoints");
            if (vp != null) for (int i = 0; i + 2 < vp.Length + 1; i += 3) _views.Add(new Pose2(vp[i], vp[i + 1], vp[i + 2]));
            else
            {
                double sx = (r[2] - r[0]) / 2, sy = (r[3] - r[1]) / 2;
                foreach (var (ox, oy) in new[] { (-1.6, 0.0), (0.0, -1.6), (1.6, 0.0), (0.0, 1.6) })
                {
                    double x = cx + ox * sx, y = cy + oy * sy;
                    _views.Add(new Pose2(x, y, Math.Atan2(cy - y, cx - x)));
                }
            }
            SetPhase("view0");
        }
        public override TaskState Tick(double dt)
        {
            string target = S.Str("target");
            if (F.Blackboard.ContainsKey(target)) { F.Log("found", S.Id, S.Str("agent"), target); return TaskState.Succeeded; }
            if (_k >= _views.Count) return Fail(FailureClass.Perception, "target " + target + " not observed from any viewpoint");
            if (_nav == null) _nav = new Navigator(F, S.Str("agent"), _views[_k], F.A.CruiseFraction, S.Id) { PosTol = 0.10, YawTol = 0.15 };
            if (_dwell > 0)
            {
                _dwell -= dt;
                if (_dwell <= 0) { _k++; _nav = null; SetPhase("view" + _k); }
                return TaskState.Running;
            }
            var s = _nav.Tick(dt);
            if (s == TaskState.Succeeded) _dwell = S.Num("dwell", 1.5);
            if (s == TaskState.Failed) { _k++; _nav = null; }
            return TaskState.Running;
        }
    }

    // ================================================================================================ lift
    /// <summary>Approach a located object, align at the grasp pose, close the gripper and lift.</summary>
    public sealed class LiftBehavior : TaskBehavior
    {
        private Navigator _nav; private double _t; private Pose2 _graspObj;
        public override IEnumerable<string> Rovers() { yield return S.Str("agent"); }
        public override TaskState Tick(double dt)
        {
            string rover = S.Str("agent"), obj = S.Str("object");
            var ch = F.Channel(rover);
            if (Phase == "start")
            {
                Pose2 op;
                if (!F.Blackboard.TryGetValue(obj, out op))
                {
                    var given = S.Nums("object_pose");
                    if (given == null) return Fail(FailureClass.Perception, "object " + obj + " not located");
                    op = new Pose2(given[0], given[1], given.Length > 2 ? given[2] : 0);
                }
                // capacity: known payload mass above the lift's rated capacity is a capability failure, not something to retry
                var lspec = F.Channel(rover).Spec;
                string ptype = S.Str("payload_type");
                if (ptype != null && F.Config.DockingProfile != null && F.Config.DockingProfile.Payloads.TryGetValue(ptype, out var pspec) && lspec.HasLift && pspec.Mass.Value > lspec.LiftCapacity)
                    return Fail(FailureClass.Capacity, $"payload over capacity: {ptype} {pspec.Mass.Value:0.0} kg > {rover} lift capacity {lspec.LiftCapacity:0.0} kg");
                // pre-grasp view in front of the object, clear of it (the close view refines the object's pose on the blackboard).
                // The standoff accounts for the object's own extent when the map carries it as an obstacle box.
                double halfObj = 0;
                foreach (var o in F.Config.Map.Obstacles) if (o.Id == obj) halfObj = o.Hx;
                double standoff = Math.Max(0.6, halfObj + F.Channel(rover).Spec.BodyLength / 2 + F.A.PlanMargin + 0.12);
                _nav = new Navigator(F, rover, Se2.Compose(op, new Pose2(-standoff, 0, 0)), F.A.CruiseFraction * 0.7, S.Id) { PosTol = 0.05, YawTol = 0.08, GoalSearch = true };
                SetPhase("pregrasp");
            }
            if (Phase == "pregrasp")
            {
                var s = _nav.Tick(dt);
                if (s == TaskState.Failed) return Fail(_nav.FailClass, _nav.Failure);
                if (s == TaskState.Succeeded) { _t = 0; SetPhase("look"); }
                return TaskState.Running;
            }
            if (Phase == "look")
            {
                _t += dt; var ag = F.AgentOf(rover); ag.Command = ag.Tracker.Hold(Twist2.Zero, dt); ag.CommandSet = true;
                if (_t < 1.0) return TaskState.Running;
                if (!F.Blackboard.TryGetValue(obj, out var op2)) { var g = S.Nums("object_pose"); op2 = new Pose2(g[0], g[1], g.Length > 2 ? g[2] : 0); }
                _graspObj = op2;
                double off = S.Num("grasp_offset", 0.30);
                // the grasp pose overlaps the object by design (gripper envelope): the object is not an obstacle for this move
                _nav = new Navigator(F, rover, Se2.Compose(op2, new Pose2(-off, 0, 0)), F.A.CruiseFraction * 0.4, S.Id) { PosTol = 0.02, YawTol = 0.04, Ignore = new[] { obj }, IgnoreObstacles = new[] { obj } };
                SetPhase("approach");
            }
            if (Phase == "approach")
            {
                var s = _nav.Tick(dt);
                if (s == TaskState.Failed) return Fail(_nav.FailClass, _nav.Failure);
                if (s == TaskState.Succeeded) { ch.SetLift(0, true); _t = 0; SetPhase("grip"); }
                return TaskState.Running;
            }
            _t += dt;
            if (Phase == "grip")
            {
                if (ch.Lift.GripperClosed) { if (!ch.Lift.HoldingObject) return Fail(FailureClass.Geometry, "gripper closed on nothing"); ch.SetLift(S.Num("height", 0.12), true); SetPhase("lift"); _t = 0; }
                else if (_t > 5) return Fail(FailureClass.Timeout, "gripper did not close");
                return TaskState.Running;
            }
            if (Phase == "lift")
            {
                if (ch.Lift.Height >= S.Num("height", 0.12) - 0.01)
                {
                    if (!ch.Lift.HoldingObject) return Fail(FailureClass.Geometry, "object lost while lifting");
                    // held: the object now travels with the carrier (its map box leaves the map, the carrier's footprint grows)
                    var rel = new Pose2(S.Num("grasp_offset", 0.30), 0, Se2.Wrap(_graspObj.Yaw - F.AgentOf(rover).MemberPose(rover).Yaw));
                    F.ObjectPicked(obj, rover, rel, S.Str("payload_type"));
                    return TaskState.Succeeded;
                }
                if (ch.Lift.ForceLimited) _forceT += dt; else _forceT = 0;
                if (_forceT > 0.5)
                {
                    var ls = F.Channel(rover).Spec;
                    return Fail(FailureClass.Capacity, $"payload over capacity: {rover} lift stalled at its force limit at {ch.Lift.Height:0.000} m (rated {ls.LiftCapacity:0.0} kg)");
                }
                if (_t > 10) return Fail(FailureClass.Timeout, "lift did not reach height");
            }
            return TaskState.Running;
        }

        private double _forceT;

        /// <summary>An unconfirmed lift leaves the object where it is: open the gripper and lower before any retry moves the lifter.</summary>
        public override void Abort()
        {
            var ch = F.Channel(S.Str("agent"));
            if (Phase == "grip" || Phase == "lift") ch.SetLift(0, false);
        }
    }

    // ================================================================================================ handoff
    /// <summary>
    /// The lifter places the held object onto the receivers' decks. Placement is defined in the RECEIVER's frame (place_on,
    /// place_offset = payload centre on that deck), not in world coordinates, because the lifter's and receiver's world estimates can
    /// each be off by centimetres: the lifter stops behind the place pose, aligns on the receiver's tags with a relative-pose
    /// estimate, requires that estimate to pass a covariance gate, makes the final (camera-blind) move on the relative estimate,
    /// lowers and releases, backs off, and only reports done once support is CONFIRMED (deck load cells, or the payload's tag seen
    /// resting inside the deck area); otherwise it fails with class Support.
    /// </summary>
    public sealed class HandoffBehavior : TaskBehavior
    {
        private Navigator _nav; private double _t, _releasedAt; private RelativePoseEstimator _rel;
        private Pose2 _q; private bool _aligned; private Pose2 _payInLifter; private double _payObsT = double.NegativeInfinity;
        private readonly double[] _ca = new double[9], _cb = new double[9], _c3 = new double[9];
        public override IEnumerable<string> Rovers() { yield return S.Str("from"); foreach (var r in S.Strs("supporters")) yield return r; }
        private string PlaceOn => S.Str("place_on") ?? S.Strs("supporters")[0];

        private Pose2 Offset() { var o = S.Nums("place_offset"); return o == null ? Pose2.Identity : new Pose2(o[0], o[1], o.Length > 2 ? o[2] : 0); }

        private void Hold(string rover, double dt) { var ag = F.AgentOf(rover); ag.Command = ag.Tracker.Hold(Twist2.Zero, dt); ag.CommandSet = true; }

        private void FeedRelative(double dt)
        {
            string from = S.Str("from"); var ag = F.AgentOf(from);
            _rel.PredictDockerMotion(ag.MemberIncrement(from, dt), F.Now);
            string self = from + "/base_link", child = PlaceOn + "/base_link", pay = S.Str("payload") + "/base_link";
            foreach (var o in F.Channel(from).Tags)
            {
                if (o.FrameParent != self || o.AmbiguousPose) continue;
                if (o.FrameChild == child) { FleetAutonomy.Marginal(o.Covariance, _c3); _rel.UpdateBaseRelative(Se2.FromPose(o.Pose), _c3, o.StampAcquired); }
                else if (o.FrameChild == pay && o.StampAcquired > _releasedAt) { _payInLifter = Se2.FromPose(o.Pose); _payObsT = o.StampAcquired; }
            }
        }

        public override TaskState Tick(double dt)
        {
            string from = S.Str("from"); var ch = F.Channel(from);
            foreach (var r in S.Strs("supporters")) Hold(r, dt);
            double hold = S.Num("hold_offset", 0.30);
            if (Phase == "start")
            {
                if (!ch.Lift.HoldingObject) return Fail(FailureClass.Configuration, from + " holds nothing");
                _q = Se2.Compose(Offset(), new Pose2(-hold, 0, 0));           // lifter base in the receiver frame at release
                var sw = F.AgentOf(PlaceOn).MemberPose(PlaceOn);
                var goal = F.Config.HandoffTagAlignment ? Se2.Compose(sw, Se2.Compose(_q, new Pose2(-F.A.HandoffPrePlaceBack, 0, 0))) : Se2.Compose(sw, _q);
                if (!F.Config.HandoffTagAlignment && S.Nums("place_pose") != null) goal = S.PoseOf("place_pose");
                _nav = new Navigator(F, from, goal, F.A.CruiseFraction * 0.5, S.Id) { PosTol = 0.03, YawTol = 0.05, Ignore = S.Strs("supporters") };
                // relative estimate: state = T(lifter base -> receiver base) (face frames identity / pi)
                _rel = new RelativePoseEstimator(Pose2.Identity, new Pose2(0, 0, Math.PI), F.A);
                SetPhase("carry");
            }
            if (_rel != null && _rel.Initialized) FeedRelative(dt);
            if (Phase == "carry")
            {
                var s = _nav.Tick(dt);
                if (s == TaskState.Failed) return Fail(_nav.FailClass, _nav.Failure);
                if (s == TaskState.Succeeded)
                {
                    if (F.Config.HandoffTagAlignment) { InitPrior(); SetPhase("observe"); _t = 0; }
                    else { ch.SetLift(0, false); _t = 0; SetPhase("release"); }
                }
                return TaskState.Running;
            }
            _t += dt;
            if (Phase == "observe")
            {
                Hold(from, dt);
                if (_t > 0.6 && _rel.Accepted >= 5 && Math.Max(_rel.SigmaGap, _rel.SigmaLateral) < F.A.HandoffAlignPosSigma && _rel.SigmaYaw < F.A.HandoffAlignYawSigma) { SetPhase("align"); _t = 0; }
                else if (_t > 5)
                    return Fail(_rel.Accepted == 0 ? FailureClass.Perception : FailureClass.Geometry,
                        $"receiver {PlaceOn} not localised for handoff (accepted {_rel.Accepted}, sigma {_rel.SigmaLateral:0.000} m / {_rel.SigmaYaw:0.000} rad)");
                return TaskState.Running;
            }
            if (Phase == "align")
            {
                // current lifter pose in the receiver frame = inv(T); error to the target pose Q in the lifter frame
                var cur = Se2.Inverse(_rel.Estimate);
                var e = Se2.Between(cur, _q);
                var ag = F.AgentOf(from);
                if (Math.Sqrt(e.X * e.X + e.Y * e.Y) < F.A.HandoffAlignPosTol && Math.Abs(e.Yaw) < F.A.HandoffAlignYawTol && ag.Ekf.Velocity.LinearSpeed < 0.01)
                { Hold(from, dt); _aligned = true; ch.SetLift(0, false); _t = 0; SetPhase("release"); return TaskState.Running; }
                var cmd = new Twist2(Clamp(1.2 * e.X, 0.05), Clamp(1.2 * e.Y, 0.05), Clamp(1.5 * e.Yaw, 0.2));
                ag.Command = ag.Tracker.Hold(cmd, dt); ag.CommandSet = true; ag.SuppressStallUntil = F.Now + 0.3;
                if (_t > 20) return Fail(FailureClass.Geometry, "handoff alignment did not converge");
                return TaskState.Running;
            }
            if (Phase == "release")
            {
                Hold(from, dt);
                if (!ch.Lift.HoldingObject)
                {
                    _releasedAt = F.Now;
                    F.ObjectReleased(S.Str("payload"), null);   // leaves the lifter's footprint; the decks' footprint takes it on confirmation
                    if (!F.Config.PostReleaseSupportCheck) { AddBelief(Offset()); }
                    var back = Se2.Compose(F.AgentOf(from).MemberPose(from), new Pose2(-S.Num("clear_distance", 0.55), 0, 0));   // far enough to see the payload tag over the deck
                    _nav = new Navigator(F, from, back, F.A.CruiseFraction * 0.5, S.Id) { PosTol = 0.06, YawTol = 0.1, Ignore = S.Strs("supporters") };
                    SetPhase("clear");
                }
                else if (_t > 8) return Fail(FailureClass.Timeout, "gripper did not release");
                return TaskState.Running;
            }
            if (Phase == "clear")
            {
                var s = _nav.Tick(dt);
                if (s == TaskState.Failed) return Fail(_nav.FailClass, _nav.Failure);
                if (s == TaskState.Succeeded) { if (!F.Config.PostReleaseSupportCheck) return TaskState.Succeeded; SetPhase("confirm"); _t = 0; }
                return TaskState.Running;
            }
            if (Phase == "confirm")
            {
                Hold(from, dt);
                string pid = S.Str("payload"); double mass = F.Config.DockingProfile != null && F.Config.DockingProfile.Payloads.TryGetValue(S.Str("payload_type", "box_small"), out var ps) ? ps.Mass.Value : 1.0;
                // (1) deck load cells
                double load = 0; bool anyCell = false;
                foreach (var r in S.Strs("supporters")) { var l = F.Channel(r).Lift; if (l.DeckLoadValid) { anyCell = true; load += l.DeckLoadKg; } }
                if (anyCell)
                {
                    if (load >= 0.7 * mass) { AddBelief(Offset()); F.Log("support_confirmed", S.Id, from, $"deck load {load:0.00} kg"); return TaskState.Succeeded; }
                    if (_t > 1.0) return Fail(FailureClass.Support, $"payload {pid} not on the decks: deck load {load:0.00} kg of {mass:0.00} kg");
                    return TaskState.Running;
                }
                // (2) the payload's tag seen after release, expressed in the receiver frame
                if (_payObsT > _releasedAt)
                {
                    Pose2 recvInLifter = _rel != null && _rel.Initialized ? _rel.Estimate : Se2.Between(F.AgentOf(from).MemberPose(from), F.AgentOf(PlaceOn).MemberPose(PlaceOn));
                    var pIn = Se2.Between(recvInLifter, _payInLifter);
                    var spec = F.Channel(PlaceOn).Spec;
                    double mx = Math.Abs(pIn.X), my = Math.Abs(pIn.Y);
                    bool onDeck = S.Strs("supporters").Count > 1 ? CheckMulti(pIn) : (mx <= spec.DeckLength / 2 - 0.03 && my <= spec.DeckWidth / 2 - 0.02);
                    if (onDeck) { AddBelief(pIn); F.Log("support_confirmed", S.Id, from, $"payload seen at ({pIn.X:0.000}, {pIn.Y:0.000}) on {PlaceOn}"); return TaskState.Succeeded; }
                    F.ReinsertObstacle(pid, Se2.Compose(F.AgentOf(from).MemberPose(from), _payInLifter));
                    return Fail(FailureClass.Support, $"payload {pid} observed off the decks at ({pIn.X:0.000}, {pIn.Y:0.000}) in {PlaceOn} frame");
                }
                if (_t > F.A.HandoffConfirmTimeout) return Fail(FailureClass.Support, $"support of {pid} not confirmed after release (no deck load signal, payload tag not seen)");
                return TaskState.Running;
            }
            return TaskState.Running;
        }

        private bool CheckMulti(Pose2 pIn)
        {
            if (F.Belief == null) return true;
            string pid = S.Str("payload") + "#check";
            var world = Se2.Compose(F.AgentOf(PlaceOn).MemberPose(PlaceOn), pIn);
            F.Belief.AddPayload(pid, S.Str("payload_type", "box_small"), new Vec2(world.X, world.Y), S.Strs("supporters").ToArray());
            var ok = F.Belief.CheckSupport().Ok;
            F.Belief.Payloads.Remove(pid);
            return ok;
        }

        private void AddBelief(Pose2 inReceiver)
        {
            var com = Se2.Compose(F.AgentOf(PlaceOn).MemberPose(PlaceOn), inReceiver);
            var sup = new List<string>(S.Strs("supporters")); sup.Remove(PlaceOn); sup.Insert(0, PlaceOn);
            F.AddPayload(S.Str("payload"), S.Str("payload_type", "box_small"), sup, com);
        }

        private void InitPrior()
        {
            string from = S.Str("from");
            var al = F.AgentOf(from); var ar = F.AgentOf(PlaceOn);
            al.Ekf.PoseCov(_ca); ar.Ekf.PoseCov(_cb);
            var cov = new double[9];
            for (int i = 0; i < 9; i++) cov[i] = _ca[i] + _cb[i] + (i == 0 || i == 4 ? 0.15 * 0.15 : i == 8 ? 0.2 * 0.2 : 0);   // world estimates are only a coarse prior
            _rel.InitializeFromBaseRelative(Se2.Between(al.MemberPose(from), ar.MemberPose(PlaceOn)), cov, F.Now);
        }

        private static double Clamp(double v, double m) => v > m ? m : v < -m ? -m : v;
    }

    // ================================================================================================ rendezvous
    /// <summary>Agree on a meeting pose and faces: the target goes to the meeting pose, then the docker goes to the pre-dock pose
    /// in front of the target's face (computed from the target's own estimate once it has arrived).</summary>
    public sealed class RendezvousBehavior : TaskBehavior
    {
        private Navigator _nt, _nd;
        public override IEnumerable<string> Rovers() { yield return S.Str("target"); yield return S.Str("docker"); }
        public override TaskState Tick(double dt)
        {
            string t = S.Str("target"), d = S.Str("docker");
            if (Phase == "start") { _nt = new Navigator(F, t, S.PoseOf("meeting_pose"), F.A.CruiseFraction, S.Id); SetPhase("target"); }
            if (Phase == "target")
            {
                var s = _nt.Tick(dt);
                if (s == TaskState.Failed) return Fail(_nt.FailClass, "target: " + _nt.Failure);
                if (s == TaskState.Succeeded)
                {
                    var pre = DockBehavior.PreDockPose(F, d, S.Str("docker_face", "front"), t, S.Str("target_face", "rear"), F.A.PreDockStandoff);
                    _nd = new Navigator(F, d, pre, F.A.CruiseFraction, S.Id) { Ignore = new[] { t }, PosTol = 0.04, YawTol = 0.06 };
                    SetPhase("docker");
                }
                return TaskState.Running;
            }
            var s2 = _nd.Tick(dt);
            if (s2 == TaskState.Failed) return Fail(_nd.FailClass, "docker: " + _nd.Failure);
            return s2;
        }
    }

    // ================================================================================================ dock
    /// <summary>
    /// Closed-loop docking of the docker's face onto the (holding) target face, servoing on the fused relative-pose estimate:
    /// pre-dock waypoint -> observe -> servo (speed tied to the estimate covariance: slow, hold, or back off and re-observe) ->
    /// connector sequence (Approach, magnets, EngageHooks, Verify, TestLoad) with retry on Jam / PARTIAL / FALSE_ENGAGEMENT and
    /// abort on OBSTRUCTED or retries exhausted. On LATCHED_VERIFIED/LOAD_VALIDATED the two agents merge into one assembly.
    /// </summary>
    public sealed class DockBehavior : TaskBehavior
    {
        private CouplingSpec _c;
        private RelativePoseEstimator _rel;
        private Navigator _nav;
        private double _t, _phaseT, _holdT;
        private int _backoffs;
        public int Backoffs => _backoffs;
        public RelativePoseEstimator Relative => _rel;
        private readonly double[] _ca = new double[9], _cb = new double[9], _c3 = new double[9];

        public DockBehavior() { }
        internal DockBehavior(CouplingSpec c, FleetAutonomy f, TaskSpec s) { _c = c; Bind(f, s); }

        public override IEnumerable<string> Rovers() { var c = Spec(); yield return c.Docker; yield return c.Target; }
        private CouplingSpec Spec() => _c ?? (_c = new CouplingSpec { Docker = S.Str("docker"), DockerFace = S.Str("docker_face", "front"), Target = S.Str("target"), TargetFace = S.Str("target_face", "rear") });

        /// <summary>World pose of the docker's base when its face is <paramref name="standoff"/> in front of the target face, aligned.</summary>
        public static Pose2 PreDockPose(FleetAutonomy f, string docker, string dockerFace, string target, string targetFace, double standoff)
        {
            var tb = f.AgentOf(target).MemberPose(target);
            var fb = f.Channel(target).Spec.Face(targetFace).InBody;
            var fa = f.Channel(docker).Spec.Face(dockerFace).InBody;
            var mateB = Se2.Compose(Se2.Compose(tb, fb), new Pose2(0, 0, Math.PI));     // target mating frame (world)
            var faceA = Se2.Compose(mateB, Se2.Inverse(new Pose2(standoff, 0, 0)));      // docker face frame such that D = (standoff, 0, 0)
            return Se2.Compose(faceA, Se2.Inverse(fa));
        }

        public override void Start()
        {
            var c = Spec();
            _rel = new RelativePoseEstimator(F.Channel(c.Docker).Spec.Face(c.DockerFace).InBody, F.Channel(c.Target).Spec.Face(c.TargetFace).InBody, F.A);
            SetPhase("predock");
            _handledFails = Conn().FailedAttempts;
        }

        private void InitPrior()
        {
            var c = Spec();
            var ad = F.AgentOf(c.Docker); var at = F.AgentOf(c.Target);
            ad.Ekf.PoseCov(_ca); at.Ekf.PoseCov(_cb);
            // Between(wA, wB) covariance
            var wa = ad.MemberPose(c.Docker); var wb = at.MemberPose(c.Target);
            _rel = new RelativePoseEstimator(_rel.FaceA, Se2.Compose(_rel.MateB, new Pose2(0, 0, Math.PI)), F.A);
            var tRel = Se2.Between(wa, wb);
            double cs = Math.Cos(wa.Yaw), sn = Math.Sin(wa.Yaw);
            var ja = new[] { -cs, -sn, tRel.Y, sn, -cs, -tRel.X, 0, 0, -1 }; var jb = new[] { cs, sn, 0, -sn, cs, 0, 0, 0, 1.0 };
            var t1 = new double[9]; var t2 = new double[9]; var cov = new double[9];
            Lin.Mul(ja, _ca, t1, 3, 3, 3); Lin.MulBt(t1, ja, cov, 3, 3, 3);
            Lin.Mul(jb, _cb, t1, 3, 3, 3); Lin.MulBt(t1, jb, t2, 3, 3, 3);
            for (int i = 0; i < 9; i++) cov[i] += t2[i];
            // inflate: world estimates are only a coarse prior for docking
            cov[0] += 0.02 * 0.02; cov[4] += 0.02 * 0.02; cov[8] += 0.05 * 0.05;
            _rel.InitializeFromBaseRelative(tRel, cov, F.Now);
        }

        private void FeedObservations(double dt)
        {
            var c = Spec();
            var ad = F.AgentOf(c.Docker);
            _rel.PredictDockerMotion(ad.MemberIncrement(c.Docker, dt), F.Now);
            var ch = F.Channel(c.Docker);
            string childB = c.Target + "/base_link", selfB = c.Docker + "/base_link";
            // own camera
            foreach (var o in ch.Tags)
            {
                if (o.FrameParent != selfB || o.FrameChild != childB) continue;
                if (o.AmbiguousPose && F.A.AmbiguousInflation <= 0) { _rel.Ambiguous++; continue; }
                FleetAutonomy.Marginal(o.Covariance, _c3);
                if (o.AmbiguousPose) for (int i = 0; i < 9; i++) _c3[i] *= F.A.AmbiguousInflation;
                _rel.UpdateBaseRelative(Se2.FromPose(o.Pose), _c3, o.StampAcquired);
            }
            // world-frame pairs (peer / overhead) seen in the docker's stream
            for (int i = 0; i < ch.Tags.Count; i++)
            {
                var ob = ch.Tags[i];
                if (ob.FrameParent != "world" || ob.FrameChild != childB || ob.AmbiguousPose) continue;
                RelPoseObservation oa = null;
                for (int k = 0; k < ch.Tags.Count; k++)
                {
                    var x = ch.Tags[k];
                    if (x.FrameParent == "world" && x.FrameChild == selfB && x.SourceId == ob.SourceId && Math.Abs(x.StampAcquired - ob.StampAcquired) < 1e-6 && !x.AmbiguousPose) { oa = x; break; }
                }
                FleetAutonomy.Marginal(ob.Covariance, _cb);
                if (oa != null) { FleetAutonomy.Marginal(oa.Covariance, _ca); _rel.UpdateWorldPair(Se2.FromPose(oa.Pose), _ca, Se2.FromPose(ob.Pose), _cb, ob.StampAcquired); }
                else { ad.Ekf.PoseCov(_ca); _rel.UpdateWorldPair(ad.MemberPose(c.Docker), _ca, Se2.FromPose(ob.Pose), _cb, ob.StampAcquired); }
            }
        }

        private void CommandFace(Twist2 faceTwist, double dt)
        {
            var c = Spec(); var ad = F.AgentOf(c.Docker);
            var faceInAgent = Se2.Compose(ad.Model.ModulePose(c.Docker), F.Channel(c.Docker).Spec.Face(c.DockerFace).InBody);
            var body = Agent.BodyTwistForFrame(faceInAgent, faceTwist);
            ad.SuppressStallUntil = F.Now + 0.6; F.AgentOf(c.Target).SuppressStallUntil = F.Now + 0.6;   // deliberate face contact
            ad.Command = ad.Tracker.Hold(body, dt); ad.CommandSet = true;
            var at = F.AgentOf(c.Target); at.Command = at.Tracker.Hold(Twist2.Zero, dt); at.CommandSet = true;
        }

        private ConnectorReport Conn() { var c = Spec(); return F.Channel(c.Docker).Connector(c.DockerFace); }
        private void Pulse(DockingCommand cmd) { var c = Spec(); F.Channel(c.Docker).Pulse(c.DockerFace, cmd); F.Log("connector_cmd", S.Id, c.Docker, cmd.ToString()); }

        public override TaskState Tick(double dt)
        {
            var c = Spec();
            _t += dt; _phaseT += dt;
            if (_t > F.A.DockPhaseTimeout * (1 + _backoffs)) return Fail(FailureClass.Timeout, "docking timeout in phase " + Phase);
            if (F.AgentOf(c.Docker) == F.AgentOf(c.Target)) return TaskState.Succeeded;
            var conn = Conn();
            if (Phase != "predock") FeedObservations(dt);
            if (conn.RetriesExhausted || conn.Action == RequestedAction.Abandon) { CommandFace(Twist2.Zero, dt); return Fail(FailureClass.Docking, "connector retries exhausted (" + conn.FailedAttempts + " failed attempts)"); }

            switch (Phase)
            {
                case "predock":
                {
                    var pre = PreDockPose(F, c.Docker, c.DockerFace, c.Target, c.TargetFace, F.A.PreDockStandoff);
                    var cur = F.AgentOf(c.Docker).MemberPose(c.Docker);
                    if (_nav == null && Se2.Distance(cur, pre) < 0.12 && Math.Abs(Se2.Wrap(cur.Yaw - pre.Yaw)) < 0.15) { Go("observe"); InitPrior(); return TaskState.Running; }
                    if (_nav == null) _nav = new Navigator(F, c.Docker, pre, F.A.CruiseFraction, S.Id) { Ignore = new[] { c.Target }, PosTol = 0.04, YawTol = 0.06 };
                    var s = _nav.Tick(dt);
                    var at = F.AgentOf(c.Target); at.Command = at.Tracker.Hold(Twist2.Zero, dt); at.CommandSet = true;
                    if (s == TaskState.Failed) return Fail(_nav.FailClass, "predock: " + _nav.Failure);
                    if (s == TaskState.Succeeded) { Go("observe"); InitPrior(); }
                    return TaskState.Running;
                }
                case "observe":
                {
                    CommandFace(Twist2.Zero, dt);
                    bool fresh = F.Now - _rel.LastUpdateTime < 0.5;
                    if (_phaseT > F.A.DockSettleTime && fresh && _rel.Accepted >= 8 && _rel.SigmaLateral < 0.01 && _rel.SigmaYaw < 0.06) Go("servo");
                    else if (_phaseT > F.A.DockObserveTimeout)
                    {
                        if (_rel.Accepted == 0) return Fail(FailureClass.Perception, "no tag observation of the target face");
                        Go("servo");
                    }
                    return TaskState.Running;
                }
                case "servo":
                {
                    if (conn.Present && conn.State == ConnectorState.MagnetCaptured) { CommandFace(Twist2.Zero, dt); Pulse(DockingCommand.EngageHooks); Go("hooks"); return TaskState.Running; }
                    if (conn.Present && conn.State == ConnectorState.Obstructed) { Pulse(DockingCommand.Release); return Fail(FailureClass.Docking, "obstructed"); }
                    if (conn.Present && conn.State == ConnectorState.Free && conn.FailedAttempts > _handledFails) { F.Log("dock_jam", S.Id, c.Docker, "jam -> back off"); return Backoff(dt); }
                    double g = _rel.Gap, y = _rel.Lateral, psi = _rel.Yaw;
                    bool ok = _rel.PredictsCapture(F.A.DockSigmaConfidence, F.A.DockCaptureLateral, F.A.DockCaptureYaw);
                    double v = g > F.A.DockNearGap ? F.A.DockCloseSpeedFar : F.A.DockCloseSpeedNear;
                    // covariance-tied speed: near the commit gap the estimate must already predict capture
                    if (!ok && g < F.A.DockCommitGap + 0.04)
                    {
                        v = 0; _holdT += dt;
                        if (_holdT > 2.0 || g < F.A.DockCommitGap - 0.03) { F.Log("dock_uncertain", S.Id, c.Docker, $"sig_lat={_rel.SigmaLateral:0.0000} sig_yaw={_rel.SigmaYaw:0.000} lat={y:0.0000} yaw={psi:0.000} -> back off"); return Backoff(dt); }
                    }
                    else _holdT = 0;
                    // lateral correction slows closing when far off axis
                    double off = Math.Abs(y) / Math.Max(0.05, g);
                    if (off > 0.2) v *= 0.3;
                    if (g < 0.004) v = F.A.DockPushSpeed;
                    if (g < F.A.DockApproachCommandGap && conn.Present && (conn.State == ConnectorState.Free || conn.State == ConnectorState.Released) && _lastApproach + 0.5 < F.Now)
                    { Pulse(DockingCommand.Approach); _lastApproach = F.Now; }
                    double vy = Clamp(F.A.DockKLat * y, 0.03), w = Clamp(F.A.DockKYaw * psi, 0.15);
                    CommandFace(new Twist2(v, vy, w), dt);
                    if (_phaseT > 30) return Backoff(dt);
                    return TaskState.Running;
                }
                case "hooks":
                {
                    CommandFace(Twist2.Zero, dt);
                    if (conn.State == ConnectorState.HookAEngaged || conn.State == ConnectorState.HookBEngaged)
                    {
                        if (conn.SensedA && conn.SensedB && !_verifySent) { Pulse(DockingCommand.Verify); _verifySent = true; }
                    }
                    else if (conn.State == ConnectorState.LatchedVerified) { if (!_testSent) { Pulse(DockingCommand.TestLoad); _testSent = true; _phaseT = 0; } else if (_phaseT > 1.0) return Latched(conn); }
                    else if (conn.State == ConnectorState.LoadValidated) return Latched(conn);
                    else if (conn.State == ConnectorState.Partial || conn.State == ConnectorState.FalseEngagement) { F.Log("dock_fault", S.Id, c.Docker, conn.State + " -> release and retry"); Pulse(DockingCommand.Release); Go("release_retry"); }
                    else if (conn.State == ConnectorState.ReleaseRequested || conn.State == ConnectorState.Free || conn.State == ConnectorState.Released) Go("release_retry");
                    else if (conn.State == ConnectorState.VerificationLost) { Pulse(DockingCommand.Release); Go("release_retry"); }
                    if (_phaseT > 15) { Pulse(DockingCommand.Release); Go("release_retry"); }
                    return TaskState.Running;
                }
                case "release_retry":
                {
                    CommandFace(Twist2.Zero, dt);
                    if (conn.State == ConnectorState.Released || conn.State == ConnectorState.Free) return Backoff(dt);
                    if (conn.State == ConnectorState.StuckHook && _phaseT > 0.5) { Pulse(DockingCommand.Release); _phaseT = 0; }
                    if (_phaseT > 10) return Fail(FailureClass.Docking, "release did not complete (" + conn.State + ")");
                    return TaskState.Running;
                }
                case "backoff":
                {
                    if (_rel.Gap >= F.A.BackoffStandoff || _phaseT > 12) { Go("observe"); _verifySent = _testSent = false; return TaskState.Running; }
                    CommandFace(new Twist2(-0.06, 0, 0), dt);
                    return TaskState.Running;
                }
            }
            return TaskState.Running;
        }

        private double _lastApproach = double.NegativeInfinity;
        private bool _verifySent, _testSent;
        private static double Clamp(double v, double m) => v > m ? m : v < -m ? -m : v;
        private void Go(string p) { SetPhase(p); _phaseT = 0; }

        private int _handledFails;
        private TaskState Backoff(double dt)
        {
            _handledFails = Math.Max(_handledFails, Conn().FailedAttempts);
            _backoffs++;
            if (_backoffs > F.A.DockMaxBackoffs) return Fail(FailureClass.Docking, "too many back-offs (" + _backoffs + ")");
            Go("backoff"); _verifySent = _testSent = false;
            return TaskState.Running;
        }

        private TaskState Latched(ConnectorReport conn)
        {
            var c = Spec();
            F.Log("dock_latched", S.Id, c.Docker, $"{conn.State} after {_backoffs} back-offs, gap_est={_rel.Gap:0.0000} lat_est={_rel.Lateral:0.0000}");
            F.FormAssembly(c, conn.State, S.Id);
            return TaskState.Succeeded;
        }

        public override void Abort()
        {
            var c = Spec();
            var conn = Conn();
            if (conn.Present && (conn.State == ConnectorState.Approach || conn.State == ConnectorState.Obstructed)) Pulse(DockingCommand.Release);
        }
    }

    // ================================================================================================ assemble
    /// <summary>A sequence of docking operations forming a target shape (each step: pre-dock navigation + dock).</summary>
    public sealed class AssembleBehavior : TaskBehavior
    {
        private List<CouplingSpec> _steps; private int _k; private DockBehavior _cur;
        public override IEnumerable<string> Rovers() { foreach (var c in Steps()) { yield return c.Docker; yield return c.Target; } }
        private List<CouplingSpec> Steps() => _steps ?? (_steps = S.Couplings("sequence"));
        public override void Start() { SetPhase("step0"); }
        public override TaskState Tick(double dt)
        {
            var st = Steps();
            if (_k >= st.Count) return TaskState.Succeeded;
            if (_cur == null) { _cur = new DockBehavior(st[_k], F, S); _cur.Start(); }
            var s = _cur.Tick(dt);
            if (s == TaskState.Failed) return Fail(_cur.FailClass, $"step {_k} ({st[_k].Id}): {_cur.FailReason}");
            if (s == TaskState.Succeeded) { _k++; _cur = null; SetPhase("step" + _k); }
            // idle members hold still
            foreach (var ag in F.Agents) if (!ag.CommandSet && ag.OwnerTask == S.Id) { ag.Command = ag.Tracker.Hold(Twist2.Zero, dt); ag.CommandSet = true; }
            return _k >= st.Count ? TaskState.Succeeded : TaskState.Running;
        }
    }

    // ================================================================================================ transport
    /// <summary>Move an assembly (with payload) to a goal; requires verified support before moving and keeps checking that every
    /// coupling of the assembly stays verified (release invariant / payload support).</summary>
    public sealed class TransportBehavior : TaskBehavior
    {
        private Navigator _nav; private double _check;
        public override IEnumerable<string> Rovers() { yield return S.Str("agent"); }
        public override TaskState Tick(double dt)
        {
            string rover = S.Str("agent");
            var ag = F.AgentOf(rover);
            if (Phase == "start")
            {
                if (F.Belief != null)
                {
                    var sup = F.Belief.CheckSupport();
                    if (!sup.Ok) return Fail(FailureClass.Support, "support not verified before transport: " + sup.Reason);
                    var p = S.Str("payload");
                    if (p != null && F.Belief.Payloads.TryGetValue(p, out var pl)) foreach (var s in pl.SupportingRovers) if (!ag.Has(s)) return Fail(FailureClass.Support, "payload supporter " + s + " not in the assembly");
                }
                _nav = new Navigator(F, rover, S.PoseOf("goal"), S.Num("cruise", F.A.TransportCruiseFraction), S.Id);
                var tol = S.Nums("tol"); if (tol != null) { _nav.PosTol = tol[0]; _nav.YawTol = tol.Length > 1 ? tol[1] : 0.1; }
                SetPhase("move");
            }
            _check += dt;
            if (_check > 0.25 && F.Belief != null)
            {
                _check = 0;
                foreach (var kv in F.CouplingFaces)
                {
                    if (!ag.Has(kv.Value[0]) || !ag.Has(kv.Value[2])) continue;
                    if (F.Belief.Couplings.TryGetValue(kv.Key, out var cp) && !cp.Removed && !cp.IsVerified)
                    { ag.Command = Twist2.Zero; ag.CommandSet = true; return Fail(FailureClass.Docking, "coupling " + kv.Key + " lost verification (" + cp.State + ")"); }
                }
                var sup = F.Belief.CheckSupport();
                if (!sup.Ok) { ag.Command = Twist2.Zero; ag.CommandSet = true; return Fail(FailureClass.Support, sup.Reason); }
            }
            var s2 = _nav.Tick(dt);
            if (s2 == TaskState.Failed) return Fail(_nav.FailClass, _nav.Failure);
            return s2;
        }
    }

    // ================================================================================================ separate
    /// <summary>Verify support on the remaining graph (release invariant) before releasing, undock, split the agents and clear.</summary>
    public sealed class SeparateBehavior : TaskBehavior
    {
        private CouplingSpec _c; private Navigator _nav; private double _t;
        public override IEnumerable<string> Rovers() { var c = Spec(); yield return c.Docker; yield return c.Target; }
        private CouplingSpec Spec() => _c ?? (_c = new CouplingSpec { Docker = S.Str("docker"), DockerFace = S.Str("docker_face", "front"), Target = S.Str("target"), TargetFace = S.Str("target_face", "rear") });
        public override TaskState Tick(double dt)
        {
            var c = Spec(); var ch = F.Channel(c.Docker);
            _t += dt;
            var ag = F.AgentOf(c.Docker);
            if (Phase != "clear") { ag.Command = ag.Tracker.Hold(Twist2.Zero, dt); ag.CommandSet = true; }
            if (Phase == "start")
            {
                if (F.Belief != null && F.Belief.Couplings.ContainsKey(c.Id))
                {
                    var d = F.Belief.CanRelease(c.Id);
                    if (!d.Allowed) return Fail(FailureClass.Support, "release refused: " + d.Reason);
                }
                F.Channel(c.Docker).Pulse(c.DockerFace, DockingCommand.Release);
                F.Log("connector_cmd", S.Id, c.Docker, "Release");
                SetPhase("release"); _t = 0;
                return TaskState.Running;
            }
            if (Phase == "release")
            {
                var r = ch.Connector(c.DockerFace);
                if (r.State == ConnectorState.Released || r.State == ConnectorState.Free)
                {
                    F.SplitOff(c.Id, c.Docker, S.Id);
                    var fa = ch.Spec.Face(c.DockerFace).InBody;
                    var cur = F.AgentOf(c.Docker).MemberPose(c.Docker);
                    // move the docker away along its face normal
                    double cl = S.Num("clear_distance", F.A.SeparateClearDistance);
                    var away = Se2.Compose(cur, new Pose2(-cl * Math.Cos(fa.Yaw), -cl * Math.Sin(fa.Yaw), 0));
                    _nav = new Navigator(F, c.Docker, away, 0.3, S.Id) { Ignore = new[] { c.Target }, PosTol = 0.05, YawTol = 0.1 };
                    SetPhase("clear");
                }
                else if (r.State == ConnectorState.StuckHook && _t > 0.5) { ch.Pulse(c.DockerFace, DockingCommand.Release); _t = 0; }
                else if (_t > 8) return Fail(FailureClass.Docking, "release not completed: " + r.State);
                return TaskState.Running;
            }
            var s = _nav.Tick(dt);
            if (s == TaskState.Failed) return Fail(_nav.FailClass, "clear: " + _nav.Failure);
            return s;
        }
    }
}
