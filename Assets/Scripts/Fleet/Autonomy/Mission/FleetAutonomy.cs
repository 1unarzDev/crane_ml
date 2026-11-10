using System;
using System.Collections.Generic;
using Crane.Fleet.Docking;
using Crane.Fleet.Graph;
using Crane.Fleet.Perception;
using DockProfile = Crane.Fleet.Profile.DockingProfile;

namespace Crane.Fleet
{
    public enum FailureClass { None, Geometry, Docking, Traction, Perception, Planner, Support, Timeout, Configuration }

    public sealed class AutonomyEvent
    {
        public double T; public string Kind, Task, Agent, Detail;
        public override string ToString() => $"[{T:0.00}] {Kind} task={Task} agent={Agent} {Detail}";
    }

    /// <summary>Per-rover I/O channel and proprioceptive front-end (encoder differencing, gyro averaging). Owned by FleetAutonomy.</summary>
    public sealed class RoverChannel
    {
        public static readonly string[] FaceNames = { "front", "rear", "left", "right" };
        public RoverSpec Spec; public IObservationSource Source; public IActuatorSink Sink;
        public Agent Agent;
        internal readonly List<EncoderSample> Enc = new List<EncoderSample>(64);
        internal readonly List<GyroSample> Gyro = new List<GyroSample>(128);
        internal readonly List<RelPoseObservation> Tags = new List<RelPoseObservation>(32);
        internal EncoderSample LastEnc; internal bool HasEnc; internal bool Still;
        internal double GyroSum, GyroT0 = double.NaN; internal int GyroN;
        public readonly double[] MeasuredRates = new double[4], CommandedRates = new double[4];
        public readonly ConnectorReport[] Connectors = new ConnectorReport[4];
        public LiftReport Lift;
        internal string PulseFace; internal DockingCommand PulseCmd;
        internal double LiftTarget; internal bool Gripper;
        public WheelHealthMonitor Health;
        public ConnectorReport Connector(string face) { for (int i = 0; i < 4; i++) if (FaceNames[i] == face) return Connectors[i]; return default; }
        public void Pulse(string face, DockingCommand c) { PulseFace = face; PulseCmd = c; }
        public void SetLift(double h, bool grip) { LiftTarget = h; Gripper = grip; }
    }

    /// <summary>One controlled rigid body: a single rover or a latched assembly. Behaviours write <see cref="Command"/> each tick.</summary>
    public sealed class Agent
    {
        public string Id;
        public AssemblyModel Model;
        public AssemblyCapability Cap;
        public BodyEkf Ekf;
        public readonly WheelAllocator Alloc = new WheelAllocator();
        public TrajectoryTracker Tracker;
        public Twist2 Command; public bool CommandSet;
        public string OwnerTask;
        internal double[] Wheels;
        internal double[] KinRows;
        public readonly List<RoverChannel> Members = new List<RoverChannel>();
        public Pose2 Pose => Ekf.Pose;
        public Pose2 MemberPose(string roverId) => Se2.Compose(Ekf.Pose, Model.ModulePose(roverId));
        public bool Has(string roverId) => Model.Contains(roverId);
        /// <summary>Velocity increment of member <paramref name="roverId"/>'s base over dt from the EKF twist.</summary>
        public Pose2 MemberIncrement(string roverId, double dt)
        {
            var mp = Model.ModulePose(roverId); var v = Ekf.Velocity;
            // module twist = Ad(M^-1) v
            double c = Math.Cos(mp.Yaw), s = Math.Sin(mp.Yaw);
            double vx = v.Vx - v.Wz * mp.Y, vy = v.Vy + v.Wz * mp.X;
            return Se2.Exp(new Twist2(c * vx + s * vy, -s * vx + c * vy, v.Wz), dt);
        }
        /// <summary>Assembly twist that moves frame <paramref name="frameInAgent"/> with twist <paramref name="frameTwist"/> (expressed in that frame).</summary>
        public static Twist2 BodyTwistForFrame(Pose2 frameInAgent, Twist2 frameTwist)
        {
            double c = Math.Cos(frameInAgent.Yaw), s = Math.Sin(frameInAgent.Yaw);
            double rx = c * frameTwist.Vx - s * frameTwist.Vy, ry = s * frameTwist.Vx + c * frameTwist.Vy;
            double w = frameTwist.Wz;
            return new Twist2(rx + w * frameInAgent.Y, ry - w * frameInAgent.X, w);
        }
    }

    public sealed class FleetConfig
    {
        public readonly List<RoverSpec> Rovers = new List<RoverSpec>();
        public readonly Dictionary<string, Pose2> InitialPoses = new Dictionary<string, Pose2>();
        public double InitialPosSigma = 0.02, InitialYawSigma = 0.02;
        public PlanMap Map;
        public AutonomyAssumptions Assume = new AutonomyAssumptions();
        public DockProfile DockingProfile;
        public DockingAssumptions DockingAssume = new DockingAssumptions();
        public ICapabilityBuilder CapabilityBuilder = new CapabilityLibraryBuilder();
        /// <summary>When true, wheel health is identified online from encoders (dead_free is kinematically invisible on a 4-mecanum base;
        /// see README). When false the health is only changed through <see cref="FleetAutonomy.ReportWheelHealth"/>.</summary>
        public bool IdentifyWheelHealth = true;
    }

    /// <summary>
    /// The autonomy stack for a fleet. One instance per simulation; call <see cref="Step"/> once per control tick.
    /// Deterministic for identical inputs (no wall clock, no unordered iteration, no randomness).
    /// </summary>
    public sealed class FleetAutonomy
    {
        public readonly FleetConfig Config;
        public readonly AutonomyAssumptions A;
        public readonly IClock Clock;
        public readonly LatticePlanner Planner;
        public readonly List<RoverChannel> Channels = new List<RoverChannel>();
        public readonly List<Agent> Agents = new List<Agent>();
        public readonly List<AutonomyEvent> Events = new List<AutonomyEvent>();
        public readonly ModuleGraph Belief;
        public readonly Dictionary<string, Pose2> Blackboard = new Dictionary<string, Pose2>();
        private readonly Dictionary<string, double> _blackboardRange = new Dictionary<string, double>();
        public readonly Dictionary<string, string[]> CouplingFaces = new Dictionary<string, string[]>(); // id -> docker, dockerFace, target, targetFace
        /// <summary>Payload COM in the frame of the agent carrying it (updated on merges and splits).</summary>
        public readonly Dictionary<string, (string rover, Pose2 inRover)> PayloadAnchors = new Dictionary<string, (string, Pose2)>();
        public MissionExecutive Executive { get; private set; }
        /// <summary>World-frame self observations offered to / accepted by the estimators (diagnostics).</summary>
        public int WorldPoseOffered, WorldPoseAccepted;
        public double Now => Clock.Now;
        private int _agentSeq;
        private double _lastStep = double.NaN;

        public FleetAutonomy(FleetConfig cfg, IClock clock, IReadOnlyList<IObservationSource> sources, IReadOnlyList<IActuatorSink> sinks)
        {
            Config = cfg; A = cfg.Assume; Clock = clock;
            if (cfg.DockingProfile != null) { A.BindDockingProfile(cfg.DockingProfile); A.DockCaptureLateral += cfg.DockingProfile.MagnetPullInRange.Value; }
            Planner = new LatticePlanner(cfg.Map, A);
            Belief = cfg.DockingProfile != null ? new ModuleGraph(cfg.DockingProfile, cfg.DockingAssume) : null;
            foreach (var spec in cfg.Rovers)
            {
                var ch = new RoverChannel { Spec = spec, Health = new WheelHealthMonitor(A) };
                foreach (var s in sources) if (s.RoverId == spec.Id) ch.Source = s;
                foreach (var s in sinks) if (s.RoverId == spec.Id) ch.Sink = s;
                if (ch.Source == null || ch.Sink == null) throw new ArgumentException("missing source or sink for rover " + spec.Id);
                Channels.Add(ch);
                var model = AssemblyModel.Single(spec);
                var ag = NewAgent(spec.Id, model, cfg.InitialPoses[spec.Id], cfg.InitialPosSigma, cfg.InitialYawSigma);
                ag.Members.Add(ch); ch.Agent = ag;
                Belief?.AddRover(spec.Id, cfg.InitialPoses[spec.Id]);
            }
        }

        public RoverChannel Channel(string roverId) { foreach (var c in Channels) if (c.Spec.Id == roverId) return c; throw new KeyNotFoundException(roverId); }
        public Agent AgentOf(string roverId) => Channel(roverId).Agent;

        private Agent NewAgent(string id, AssemblyModel model, Pose2 pose, double ps, double ys)
        {
            var cap = (AssemblyCapability)Config.CapabilityBuilder.Build(model, A);
            var ag = new Agent { Id = id, Model = model, Cap = cap, Tracker = new TrajectoryTracker(A), Wheels = new double[model.WheelCount] };
            ag.Ekf = new BodyEkf(model, cap, A, pose, ps, ys, Clock.Now);
            ag.Alloc.Bind(model, cap); ag.KinRows = KinematicRows.Build(model, true);
            ag.Tracker.SetPath(null, cap);
            Agents.Add(ag);
            return ag;
        }

        public void Log(string kind, string task, string agent, string detail) => Events.Add(new AutonomyEvent { T = Now, Kind = kind, Task = task, Agent = agent, Detail = detail });

        // ------------------------------------------------------------------------------------------ mission
        public void LoadMission(MissionSpec m)
        {
            foreach (var c in m.InitialCouplings) FormAssembly(c, ConnectorState.LatchedVerified, "initial");
            foreach (var p in m.InitialPayloads) AddPayload(p.Id, p.Type, p.Supporters, new Pose2(p.ComX, p.ComY, 0));
            Executive = new MissionExecutive(this, m);
        }

        public void AddPayload(string id, string type, List<string> supporters, Pose2 comWorld)
        {
            if (Belief == null) return;
            Belief.AddPayload(id, type, new Vec2(comWorld.X, comWorld.Y), supporters.ToArray());
            var anchor = supporters[0];
            PayloadAnchors[id] = (anchor, Se2.Between(AgentOf(anchor).MemberPose(anchor), comWorld));
            // footprint of the payload in the carrying agent's frame
            var ag = AgentOf(anchor);
            var size = Config.DockingProfile.Payloads[type].SizeLwh.Value;
            var inAgent = Se2.Compose(ag.Model.ModulePose(anchor), PayloadAnchors[id].inRover);
            var yawW = 0.0; foreach (var s in supporters) { yawW = AgentOf(s).MemberPose(s).Yaw; break; }
            ag.Model.ExtraFootprint.Add(new Box2("payload:" + id, inAgent.X, inAgent.Y, 0, size[0] / 2, size[1] / 2));
            ag.Model.Version++;
            RebuildCapability(ag, "payload " + id + " loaded");
        }

        // ------------------------------------------------------------------------------------------ structure changes
        /// <summary>Merge the docker's agent into the target's agent after a verified latch.</summary>
        public Agent FormAssembly(CouplingSpec c, ConnectorState state, string task)
        {
            var at = AgentOf(c.Target); var ad = AgentOf(c.Docker);
            if (at == ad) return at;
            var model = AssemblyModel.Merge(at.Model, c.Target, c.TargetFace, ad.Model, c.Docker, c.DockerFace, "asm" + (++_agentSeq));
            // pose: target agent's frame; fuse with the docker's view of it (information weighting of the 3x3 pose blocks)
            var cov = new double[9]; at.Ekf.PoseCov(cov);
            var dockerModuleInNew = model.ModulePose(c.Docker);
            var fromDocker = Se2.Compose(ad.MemberPose(c.Docker), Se2.Inverse(dockerModuleInNew));
            var covD = new double[9]; ad.Ekf.PoseCov(covD);
            var fused = FusePoses(at.Pose, cov, fromDocker, covD, out var fcov);
            var biases = new List<double>();
            foreach (var m in at.Model.Modules) biases.Add(at.Ekf.GyroBias(at.Model.IndexOf(m.Spec.Id)));
            foreach (var m in ad.Model.Modules) biases.Add(ad.Ekf.GyroBias(ad.Model.IndexOf(m.Spec.Id)));
            Agents.Remove(at); Agents.Remove(ad);
            var ag = NewAgent(model.Id, model, fused, 0, 0);
            ag.Ekf.Reset(fused, fcov, Now);
            for (int i = 0; i < biases.Count; i++) ag.Ekf.SetBias(i, biases[i], A.InitialBiasSigma * A.InitialBiasSigma * 0.25);
            foreach (var m in model.Modules) { var ch = Channel(m.Spec.Id); ag.Members.Add(ch); ch.Agent = ag; }
            string cid = c.Id;
            CouplingFaces[cid] = new[] { c.Docker, c.DockerFace, c.Target, c.TargetFace };
            if (Belief != null && !Belief.Couplings.ContainsKey(cid)) Belief.AddCoupling(cid, c.Target, c.TargetFace, c.Docker, c.DockerFace, state);
            Log("assembly_formed", task, ag.Id, cid + " members=" + Members(ag));
            return ag;
        }

        /// <summary>Split the listed rovers out of their assembly (after a release). Each leaving rover becomes its own agent.</summary>
        public void SplitOff(string couplingId, string leaving, string task)
        {
            var ag = AgentOf(leaving);
            var leavingSet = ConnectedComponentWithout(ag, couplingId, leaving);
            var cov = new double[9]; ag.Ekf.PoseCov(cov);
            var j = new double[9];
            Agents.Remove(ag);
            var rest = ag.Model.Without(leavingSet, "asm" + (++_agentSeq), out var restFrame);
            var leaveModel = ag.Model.Without(Complement(ag, leavingSet), leavingSet.Count == 1 ? leaving : "asm" + (++_agentSeq), out var leaveFrame);
            Agent Make(AssemblyModel m, Pose2 frameInOld)
            {
                if (m.Modules.Count == 1) m.Id = m.Modules[0].Spec.Id;
                var pose = Se2.Compose(ag.Pose, frameInOld);
                Se2.JacComposeLeft(ag.Pose, frameInOld, j);
                var c2 = (double[])cov.Clone(); var tmp = new double[9];
                Lin.Congruence(j, c2, tmp, 3);
                // payload footprints stay with the group that keeps all supporters
                m.ExtraFootprint.Clear();
                var na = NewAgent(m.Id, m, pose, 0, 0);
                na.Ekf.Reset(pose, c2, Now);
                for (int i = 0; i < m.Modules.Count; i++) na.Ekf.SetBias(i, ag.Ekf.GyroBias(ag.Model.IndexOf(m.Modules[i].Spec.Id)), A.InitialBiasSigma * A.InitialBiasSigma * 0.25);
                foreach (var mm in m.Modules) { var ch = Channel(mm.Spec.Id); na.Members.Add(ch); ch.Agent = na; }
                return na;
            }
            var ra = Make(rest, restFrame); var la = Make(leaveModel, leaveFrame);
            foreach (var kv in PayloadAnchors)
            {
                var target = ra.Has(kv.Value.rover) ? ra : la;
                if (Belief != null && Belief.Payloads.TryGetValue(kv.Key, out var p))
                {
                    bool all = true; foreach (var s in p.SupportingRovers) all &= target.Has(s);
                    if (all)
                    {
                        var size = Config.DockingProfile.Payloads[TypeOf(kv.Key)].SizeLwh.Value;
                        var inAgent = Se2.Compose(target.Model.ModulePose(kv.Value.rover), kv.Value.inRover);
                        target.Model.ExtraFootprint.Add(new Box2("payload:" + kv.Key, inAgent.X, inAgent.Y, 0, size[0] / 2, size[1] / 2));
                        target.Model.Version++; RebuildCapability(target, "payload footprint");
                    }
                }
            }
            if (Belief != null && Belief.Couplings.ContainsKey(couplingId)) Belief.Release(couplingId, force: false);
            Log("assembly_split", task, ag.Id, couplingId + " -> " + ra.Id + " + " + la.Id);
        }

        private readonly Dictionary<string, string> _payloadTypes = new Dictionary<string, string>();
        private string TypeOf(string payloadId)
        {
            foreach (var kv in Config.DockingProfile.Payloads)
                if (Belief.Payloads.TryGetValue(payloadId, out var p) && Math.Abs(kv.Value.Mass.Value - p.Mass) < 1e-9) return kv.Key;
            return "box_long";
        }

        private HashSet<string> ConnectedComponentWithout(Agent ag, string couplingId, string start)
        {
            var set = new HashSet<string> { start };
            bool grew = true;
            while (grew)
            {
                grew = false;
                foreach (var kv in CouplingFaces)
                {
                    if (kv.Key == couplingId) continue;
                    if (Belief != null && Belief.Couplings.TryGetValue(kv.Key, out var cp) && cp.Removed) continue;
                    string a = kv.Value[0], b = kv.Value[2];
                    if (!ag.Has(a) || !ag.Has(b)) continue;
                    if (set.Contains(a) && set.Add(b)) grew = true;
                    if (set.Contains(b) && set.Add(a)) grew = true;
                }
            }
            return set;
        }

        private static List<string> Complement(Agent ag, HashSet<string> s) { var l = new List<string>(); foreach (var m in ag.Model.Modules) if (!s.Contains(m.Spec.Id)) l.Add(m.Spec.Id); return l; }
        private static string Members(Agent ag) { var l = new List<string>(); foreach (var m in ag.Model.Modules) l.Add(m.Spec.Id); return string.Join(",", l); }

        private static Pose2 FusePoses(Pose2 a, double[] ca, Pose2 b, double[] cb, out double[] cov)
        {
            var ia = new double[9]; var ib = new double[9]; var w = new double[18];
            var cov0 = new double[9];
            for (int i = 0; i < 9; i++) { ca[i] += i % 4 == 0 ? 1e-12 : 0; cb[i] += i % 4 == 0 ? 1e-12 : 0; }
            Lin.Invert(ca, ia, 3, w); Lin.Invert(cb, ib, 3, w);
            var sum = new double[9]; for (int i = 0; i < 9; i++) sum[i] = ia[i] + ib[i];
            Lin.Invert(sum, cov0, 3, w);
            double dx = b.X - a.X, dy = b.Y - a.Y, dth = Se2.Wrap(b.Yaw - a.Yaw);
            // x = a + cov * Ib * (b - a)
            var t = new double[9]; Lin.Mul(cov0, ib, t, 3, 3, 3);
            cov = cov0;
            return new Pose2(a.X + t[0] * dx + t[1] * dy + t[2] * dth, a.Y + t[3] * dx + t[4] * dy + t[5] * dth, Se2.Wrap(a.Yaw + t[6] * dx + t[7] * dy + t[8] * dth));
        }

        public void RebuildCapability(Agent ag, string reason)
        {
            var cap = (AssemblyCapability)Config.CapabilityBuilder.Build(ag.Model, A);
            ag.Cap = cap; ag.Ekf.Rebind(ag.Model, cap); ag.Alloc.Bind(ag.Model, cap); ag.KinRows = KinematicRows.Build(ag.Model, true); ag.Tracker.UpdateCapability(cap);
            Log("capability_changed", ag.OwnerTask, ag.Id, $"v{cap.Version} rank={cap.ActuatedRank} {reason}");
        }

        /// <summary>External health report (motor-driver fault flag, operator, test). Triggers capability rebuild and replanning.</summary>
        public void ReportWheelHealth(string roverId, int wheel, CapHealth h)
        {
            var ag = AgentOf(roverId); int m = ag.Model.IndexOf(roverId);
            ag.Model.Modules[m].Health[wheel] = h; ag.Model.Version++;
            Channel(roverId).Health.Identified[wheel] = h;
            RebuildCapability(ag, $"{roverId} wheel {wheel} -> {h.Mode}");
        }

        // ------------------------------------------------------------------------------------------ tick
        private readonly double[] _cov3 = new double[9], _kin = new double[4];

        /// <summary>One control tick: read every source, update estimators, run the mission executive, write every sink.</summary>
        public void Step(double dt)
        {
            double now = Now;
            foreach (var ch in Channels)
            {
                ch.Source.ReadEncoders(ch.Enc); ch.Source.ReadGyro(ch.Gyro); ch.Source.ReadTagObservations(ch.Tags);
                for (int i = 0; i < 4; i++) ch.Connectors[i] = ch.Source.ReadConnector(RoverChannel.FaceNames[i]);
                ch.Lift = ch.Source.ReadLift();
            }
            // proprioception and world-frame self observations
            foreach (var ag in Agents) { ag.CommandSet = false; ProcessProprio(ag, now); }
            foreach (var ch in Channels) ProcessSelfObservations(ch);
            foreach (var ag in Agents) ag.Ekf.PredictTo(now);
            // health identification
            if (Config.IdentifyWheelHealth)
                foreach (var ch in Channels) CheckHealth(ch, dt);
            // objects seen by own cameras go to the blackboard
            foreach (var ch in Channels) foreach (var o in ch.Tags)
                {
                    if (o.SourceKind != ObservationSourceKind.OwnCamera || o.AmbiguousPose) continue;
                    string child = FrameOwner(o.FrameChild);
                    if (IsRover(child)) continue;
                    // keep the closest view (object pose error grows with range); refresh when a view is as close
                    double range = Math.Sqrt(o.Pose.P.X * o.Pose.P.X + o.Pose.P.Y * o.Pose.P.Y);
                    if (_blackboardRange.TryGetValue(child, out double best) && range > best + 0.05) continue;
                    _blackboardRange[child] = range;
                    Blackboard[child] = Se2.Compose(ch.Agent.MemberPose(ch.Spec.Id), Se2.FromPose(o.Pose));
                }
            UpdateBelief();
            Executive?.Tick(dt);
            // actuation
            foreach (var ag in Agents)
            {
                var cmd = ag.CommandSet ? ag.Command : ag.Tracker.Hold(Twist2.Zero, dt);
                ag.Alloc.Allocate(cmd, ag.Wheels);
                for (int m = 0; m < ag.Model.Modules.Count; m++)
                {
                    var ch = Channel(ag.Model.Modules[m].Spec.Id);
                    for (int j = 0; j < 4; j++) ch.CommandedRates[j] = ag.Wheels[4 * m + j];
                    ch.Sink.SetWheelSpeeds(ag.Wheels[4 * m], ag.Wheels[4 * m + 1], ag.Wheels[4 * m + 2], ag.Wheels[4 * m + 3]);
                }
            }
            foreach (var ch in Channels)
            {
                for (int i = 0; i < 4; i++)
                {
                    string f = RoverChannel.FaceNames[i];
                    ch.Sink.CommandConnector(f, ch.PulseFace == f ? ch.PulseCmd : DockingCommand.None);
                }
                ch.PulseFace = null; ch.PulseCmd = DockingCommand.None;
                ch.Sink.SetLift(ch.LiftTarget, ch.Gripper);
                var ag = ch.Agent; ag.Ekf.PoseCov(_cov3);
                if (ch.Sink is IEstimatePublisher pub)
                {
                    var mp = ag.MemberPose(ch.Spec.Id);
                    pub.PublishEstimate(now, mp, _cov3);
                }
            }
            _lastStep = now;
        }

        public bool IsRover(string id) { foreach (var c in Channels) if (c.Spec.Id == id) return true; return false; }
        public static string FrameOwner(string frame) { int i = frame.IndexOf('/'); return i < 0 ? frame : frame.Substring(0, i); }

        private readonly List<(double t, int kind, int module, int idx)> _events = new List<(double, int, int, int)>(256);

        private void ProcessProprio(Agent ag, double now)
        {
            _events.Clear();
            for (int m = 0; m < ag.Model.Modules.Count; m++)
            {
                var ch = Channel(ag.Model.Modules[m].Spec.Id);
                for (int i = 0; i < ch.Enc.Count; i++) _events.Add((ch.Enc[i].Stamp, 0, m, i));
                for (int i = 0; i < ch.Gyro.Count; i++) _events.Add((ch.Gyro[i].Stamp, 1, m, i));
            }
            _events.Sort((a, b) => a.t != b.t ? a.t.CompareTo(b.t) : a.kind != b.kind ? a.kind.CompareTo(b.kind) : a.module != b.module ? a.module.CompareTo(b.module) : a.idx.CompareTo(b.idx));
            foreach (var e in _events)
            {
                var ch = Channel(ag.Model.Modules[e.module].Spec.Id);
                if (e.kind == 0)
                {
                    var s = ch.Enc[e.idx];
                    if (!ch.HasEnc) { ch.LastEnc = s; ch.HasEnc = true; continue; }
                    double dtE = s.Stamp - ch.LastEnc.Stamp;
                    if (dtE < A.EncoderMinInterval) continue;
                    double rpc = 2 * Math.PI / ch.Spec.CountsPerRev;
                    double q = rpc / dtE / Math.Sqrt(6.0) * 1.5 + 0.005;
                    bool still = s.Fl == ch.LastEnc.Fl && s.Fr == ch.LastEnc.Fr && s.Rl == ch.LastEnc.Rl && s.Rr == ch.LastEnc.Rr;
                    ch.Still = still;
                    if (still && ag.Model.Modules.Count == 1 || still && AllStill(ag, s.Stamp))
                    {
                        for (int j = 0; j < 4; j++) ch.MeasuredRates[j] = 0;
                        ag.Ekf.ZeroVelocity(s.Stamp - 0.5 * dtE, A.ZuptSigmaLin, A.ZuptSigmaYaw);
                        ch.LastEnc = s;
                        continue;
                    }
                    for (int j = 0; j < 4; j++)
                    {
                        double rate = (s[j] - ch.LastEnc[j]) * rpc / dtE;
                        ch.MeasuredRates[j] = rate;
                        ag.Ekf.WheelRate(e.module, j, s.Stamp - 0.5 * dtE, rate, q);
                    }
                    ch.LastEnc = s;
                }
                else
                {
                    var g = ch.Gyro[e.idx];
                    if (double.IsNaN(ch.GyroT0)) { ch.GyroT0 = g.Stamp; ch.GyroSum = 0; ch.GyroN = 0; }
                    ch.GyroSum += g.RateZ; ch.GyroN++;
                    double span = g.Stamp - ch.GyroT0;
                    if (span >= A.GyroMinInterval && ch.GyroN > 0)
                    {
                        double mean = ch.GyroSum / ch.GyroN;
                        double sig = ch.Spec.GyroNoiseDensity / Math.Sqrt(Math.Max(span, 1e-3)) * 1.5 + 2e-4;
                        if (ch.Still && ch.CommandedRates[0] == 0 && ch.CommandedRates[1] == 0 && ch.CommandedRates[2] == 0 && ch.CommandedRates[3] == 0)
                            ag.Ekf.GyroAtRest(e.module, ch.GyroT0 + 0.5 * span, mean, sig);
                        else ag.Ekf.Gyro(e.module, ch.GyroT0 + 0.5 * span, mean, sig);
                        ch.GyroT0 = g.Stamp; ch.GyroSum = 0; ch.GyroN = 0;
                    }
                }
            }
        }

        private bool AllStill(Agent ag, double t)
        {
            foreach (var m in ag.Model.Modules)
            {
                var c = Channel(m.Spec.Id);
                for (int j = 0; j < 4; j++) if (Math.Abs(c.MeasuredRates[j]) > 1e-9) return false;
            }
            return true;
        }

        private void ProcessSelfObservations(RoverChannel ch)
        {
            var ag = ch.Agent;
            foreach (var o in ch.Tags)
            {
                if (o.FrameParent != "world") continue;
                if (FrameOwner(o.FrameChild) != ch.Spec.Id) continue;
                if (o.AmbiguousPose && A.WorldAmbiguousInflation <= 0) continue;
                Marginal(o.Covariance, _cov3);
                // a flagged two-branch PnP solution of a horizontal top tag differs mostly in tilt; its planar part stays usable
                // with an inflated covariance (the innovation gate still rejects a flipped yaw)
                if (o.AmbiguousPose) for (int i = 0; i < 9; i++) _cov3[i] *= A.WorldAmbiguousInflation;
                int before = ag.Ekf.AcceptedPoseUpdates; WorldPoseOffered++;
                ag.Ekf.WorldPose(ag.Model.IndexOf(ch.Spec.Id), o.StampAcquired, Se2.FromPose(o.Pose), _cov3);
                if (ag.Ekf.AcceptedPoseUpdates > before) WorldPoseAccepted++;
            }
        }

        private static readonly int[] MarginalIdx = { 0, 1, 5 };
        /// <summary>Planar (x, y, yaw) marginal of a Perception 6x6 [dp; dtheta] covariance.</summary>
        public static void Marginal(Matrix c6, double[] c3)
        {
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) c3[3 * i + j] = c6[MarginalIdx[i], MarginalIdx[j]];
        }

        private void CheckHealth(RoverChannel ch, double dt)
        {
            var ag = ch.Agent; int m = ag.Model.IndexOf(ch.Spec.Id);
            // kinematic rates implied by the EKF twist at this module
            var v = ag.Ekf.Velocity;
            var rows = ag.KinRows;
            for (int j = 0; j < 4; j++) { int i = 4 * m + j; _kin[j] = rows[3 * i] * v.Vx + rows[3 * i + 1] * v.Vy + rows[3 * i + 2] * v.Wz; }
            if (ch.Health.Update(ch.CommandedRates, ch.MeasuredRates, _kin, dt))
            {
                bool changed = false;
                for (int j = 0; j < 4; j++)
                    if (ch.Health.Identified[j].Mode != ag.Model.Modules[m].Health[j].Mode)
                    {
                        ag.Model.Modules[m].Health[j] = ch.Health.Identified[j]; changed = true;
                        Log("wheel_health_identified", ag.OwnerTask, ag.Id, $"{ch.Spec.Id} wheel {j} -> {ch.Health.Identified[j].Mode} gain {ch.Health.Identified[j].Gain:0.00}");
                    }
                if (changed) { ag.Model.Version++; RebuildCapability(ag, "online identification"); }
            }
        }

        private void UpdateBelief()
        {
            if (Belief == null) return;
            foreach (var ch in Channels) if (Belief.Rovers.TryGetValue(ch.Spec.Id, out var r)) r.Pose = ch.Agent.MemberPose(ch.Spec.Id);
            foreach (var kv in CouplingFaces)
            {
                if (!Belief.Couplings.TryGetValue(kv.Key, out var cp) || cp.Removed) continue;
                var rep = Channel(kv.Value[0]).Connector(kv.Value[1]);
                if (rep.Present) cp.State = rep.State;
            }
            foreach (var kv in PayloadAnchors)
                if (Belief.Payloads.TryGetValue(kv.Key, out var p))
                {
                    var w = Se2.Compose(AgentOf(kv.Value.rover).MemberPose(kv.Value.rover), kv.Value.inRover);
                    p.Com = new Vec2(w.X, w.Y);
                }
        }

        /// <summary>Other agents' footprints (world) for planning around them, inflated by their position uncertainty.</summary>
        public List<Box2> OtherFootprints(Agent self, ICollection<string> ignoreRovers = null)
        {
            var l = new List<Box2>();
            foreach (var ag in Agents)
            {
                if (ag == self) continue;
                bool skip = false;
                if (ignoreRovers != null) foreach (var m in ag.Model.Modules) if (ignoreRovers.Contains(m.Spec.Id)) skip = true;
                if (skip) continue;
                double infl = 2 * ag.Ekf.PosSigma + 0.03;
                foreach (var b in ag.Model.Footprint()) l.Add(b.Transformed(ag.Pose).Inflated(infl));
            }
            return l;
        }
    }

    /// <summary>Optional: a sink that also forwards the rover's own estimate (Perception PeerObserver needs it; ROS /estimate).</summary>
    public interface IEstimatePublisher
    {
        void PublishEstimate(double stamp, Pose2 pose, double[] cov3);
    }
}
