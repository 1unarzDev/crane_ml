using System;
using System.Collections.Generic;
using Crane.Fleet.Autonomy;
using Crane.Fleet.Docking;
using Crane.Fleet.Perception;
using Crane.Fleet.Perception.Evaluator;
using Crane.Fleet.Perception.Rover;
using UnityEngine;
using PPose = Crane.Fleet.Perception.Pose;

namespace Crane.Fleet.Sim {
    /// <summary>
    /// Evaluator-side host of the perception core: a FleetSimulator whose truth trajectories are the live rigid bodies, recorded every physics
    /// step (frames requested at a past acquisition time get the last recorded state at or before it). Rovers only ever get an
    /// IObservationProvider. Tag observations use the perception core's own camera mounts and tag geometry (SensorSuite/RoverGeometry), which
    /// are not yet the asset manifest's frames (logged as an integration gap). Planar truth (z = 0, yaw only), as the perception core assumes.
    /// </summary>
    public sealed class FleetPerceptionHost {
        public readonly FleetSimulator Sim;
        readonly Dictionary<string, List<(double t, double x, double y, double yaw, double vx, double vy, double wz, double wvx, double wvy)>> hist =
            new Dictionary<string, List<(double, double, double, double, double, double, double, double, double)>>();
        readonly Dictionary<string, IObservationProvider> providers = new Dictionary<string, IObservationProvider>();
        readonly Dictionary<string, RoverRig> rigs = new Dictionary<string, RoverRig>();
        public bool CameraLost(string id) => rigs.TryGetValue(id, out var r) && r.CameraLost;

        public FleetPerceptionHost(FleetSpec spec, IEnumerable<RoverRig> rovers, long seed) {
            var profile = FleetProfile.Parse(System.IO.File.ReadAllText(spec.Profile.Source));
            var scene = Scenario.Parse(System.IO.File.ReadAllText(spec.Scenario.Source));
            var truth = new EvaluatorTruth();
            foreach (var r in rovers) {
                string id = r.D.Id;
                var h = hist[id] = new List<(double, double, double, double, double, double, double, double, double)>();
                truth.SetTrajectory(id, t => State(h, t));
            }
            Sim = new FleetSimulator(profile, scene, truth, FleetRng.For(seed, "perception").NextU64());
            foreach (var r in rovers) { providers[r.D.Id] = new LossyProvider(Sim.ProviderFor(r.D.Id), this); rigs[r.D.Id] = r; }
        }

        static RoverTruthState State(List<(double t, double x, double y, double yaw, double vx, double vy, double wz, double wvx, double wvy)> h, double t) {
            if (h.Count == 0) return new RoverTruthState { Pose = PPose.Identity };
            int lo = 0, hi = h.Count - 1;
            if (t >= h[hi].t) lo = hi;
            else while (lo < hi) { int mid = (lo + hi + 1) / 2; if (h[mid].t <= t + 1e-12) lo = mid; else hi = mid - 1; }
            var s = h[lo];
            return new RoverTruthState { Pose = PPose.FromYaw(s.x, s.y, 0, s.yaw), LinVelWorld = new Vec3(s.wvx, s.wvy, 0), AngVelWorld = new Vec3(0, 0, s.wz),
                Vx = s.vx, Vy = s.vy, Wz = s.wz };
        }

        public void Record(double t, IEnumerable<RoverRig> rovers) {
            foreach (var r in rovers) {
                var tr = r.Truth();
                double c = Math.Cos(tr.Yaw), s = Math.Sin(tr.Yaw);
                var h = hist[r.D.Id];
                h.Add((t, tr.X, tr.Y, tr.Yaw, tr.Body.Vx, tr.Body.Vy, tr.Body.Wz, c * tr.Body.Vx - s * tr.Body.Vy, s * tr.Body.Vx + c * tr.Body.Vy));
                if (h.Count > 4000) h.RemoveRange(0, 1000);   // frames are requested at most ~0.2 s back (latency)
            }
        }

        public IObservationProvider Provider(string id) => providers[id];

        /// <summary>Failure injection on the rover side of the firewall: a lost camera delivers no tag observations.</summary>
        sealed class LossyProvider : IObservationProvider {
            readonly IObservationProvider inner; readonly FleetPerceptionHost host;
            public LossyProvider(IObservationProvider inner, FleetPerceptionHost host) { this.inner = inner; this.host = host; }
            public string RoverId => inner.RoverId;
            public SensorSuite Suite => inner.Suite;
            public IReadOnlyList<RelPoseObservation> PollTagObservations(double now) {
                var l = inner.PollTagObservations(now);
                return host.CameraLost(RoverId) ? (IReadOnlyList<RelPoseObservation>)Array.Empty<RelPoseObservation>() : l;
            }
            public IReadOnlyList<EncoderReading> PollEncoders(double now) => inner.PollEncoders(now);
            public IReadOnlyList<ImuReading> PollImu(double now) => inner.PollImu(now);
            public IReadOnlyList<RangeReading> PollTof(string mountName, double now) => inner.PollTof(mountName, now);
            public IReadOnlyList<LidarScan> PollLidar(double now) => inner.PollLidar(now);
            public void ReportSelfEstimate(double stamp, PPose pose, Matrix covariance) => inner.ReportSelfEstimate(stamp, pose, covariance);
        }
    }

    /// <summary>
    /// Couplings between docking faces, created on demand: a connector Approach on (rover, face) pairs it with the nearest other rover's face
    /// whose frame faces it (normals opposed within the guide acceptance, within <see cref="PairRange"/>). One docking-core ConnectorMachine per
    /// coupling, stepped every physics step with the live face-to-face pose; LATCHED_VERIFIED creates a FixedJoint (follower -> leader,
    /// collision between them off); RELEASED removes it (separation).
    /// </summary>
    public sealed class CouplingManager {
        public sealed class Coupling {
            public string Id;
            public RoverRig A, B;          // A = the face that was commanded (the approaching rover), B = its partner
            public string FaceA, FaceB;
            public ConnectorMachine Machine;
            public DockingCommand Cmd = DockingCommand.None;
            public FacePose Pose;
            public double PrevGap = double.NaN, Closing;
            public FixedJoint Joint;
        }

        readonly FleetRun run;
        readonly FleetSpec spec;
        readonly ContactModel model;
        public readonly List<Coupling> All = new List<Coupling>();
        public double PairRange = 0.6;
        public Action<Coupling> OnLatched;

        public CouplingManager(FleetRun run, FleetSpec spec) {
            this.run = run; this.spec = spec;
            model = new ContactModel(DockingProfileBuilder.Build(spec), new DockingAssumptions());
            run.IgnoreContact = (a, b) => { foreach (var c in All) if (c.Joint != null && ((c.A == a && c.B == b) || (c.A == b && c.B == a))) return true; return false; };
        }

        public Coupling Of(RoverRig r, string face) {
            foreach (var c in All) if ((c.A == r && c.FaceA == face) || (c.B == r && c.FaceB == face)) return c;
            return null;
        }

        public void Command(RoverRig r, string face, DockingCommand cmd) {
            var c = Of(r, face);
            if (c == null) {
                if (cmd != DockingCommand.Approach) return;
                c = Pair(r, face);
                if (c == null) { run.Event("connector_no_partner", r.D.Id, new Dictionary<string, object> { ["face"] = face }); return; }
            }
            c.Cmd = cmd;
        }

        Coupling Pair(RoverRig r, string face) {
            Transform fa = r.Frames["face_" + face];
            double best = PairRange; RoverRig br = null; string bf = null;
            double accept = spec.P["docking.guide_acceptance_half_angle"].Double * 4;
            foreach (var o in run.Rovers) {
                if (o == r) continue;
                foreach (var f in o.D.Faces) {
                    Transform fb = o.Frames["face_" + f.Name];
                    double d = (fb.position - fa.position).magnitude;
                    var p = Relative(fa, fb);
                    if (d < best && Math.Abs(p.Yaw) < accept && p.Gap > -0.05) { best = d; br = o; bf = f.Name; }
                }
            }
            if (br == null) return null;
            var c = new Coupling { A = r, FaceA = face, B = br, FaceB = bf, Id = $"{br.D.Id}.{bf}-{r.D.Id}.{face}" };
            c.Machine = new ConnectorMachine(model, FleetRng.For(run.Opt.Seed, "coupling/" + c.Id).NextU64(), c.Id);
            All.Add(c);
            run.Event("connector_paired", r.D.Id, new Dictionary<string, object> { ["coupling"] = c.Id, ["distance"] = best });
            return c;
        }

        /// <summary>Face b relative to face a (a's +x = outward normal): gap along the normal, lateral, height, yaw error from mating.</summary>
        public static FacePose Relative(Transform a, Transform b) {
            Vector3 l = Quaternion.Inverse(a.rotation) * (b.position - a.position);
            FleetFrames.Attitude(a.rotation, out double ya, out _, out _);
            FleetFrames.Attitude(b.rotation, out double yb, out _, out _);
            return new FacePose(-l.x, l.z, Angles.Wrap(yb - ya - Math.PI), l.y);
        }

        public void Step(double dt) {
            foreach (var c in All) {
                c.Pose = Relative(c.A.Frames["face_" + c.FaceA], c.B.Frames["face_" + c.FaceB]);
                c.Closing = double.IsNaN(c.PrevGap) ? 0 : (c.PrevGap - c.Pose.Gap) / dt;
                c.PrevGap = c.Pose.Gap;
                var res = c.Machine.Step(dt, new ConnectorInputs { Pose = c.Pose, ClosingSpeed = c.Closing, Command = c.Cmd, ReleaseAuthorized = true });
                foreach (var e in res.Events)
                    run.Event("connector", c.A.D.Id, new Dictionary<string, object> { ["coupling"] = c.Id, ["event"] = e.Kind.ToString(), ["from"] = ConnectorStateNames.ToProfileName(e.From),
                        ["to"] = ConnectorStateNames.ToProfileName(e.To), ["trigger"] = e.Trigger?.ToString(), ["detail"] = e.Detail, ["gap"] = c.Pose.Gap, ["lateral"] = c.Pose.Lateral, ["yaw"] = c.Pose.Yaw });
                if (res.Action != RequestedAction.None) run.Event("connector_action", c.A.D.Id, new Dictionary<string, object> { ["coupling"] = c.Id, ["action"] = res.Action.ToString() });
                var st = c.Machine.State;
                if (ConnectorStateNames.IsVerified(st) && c.Joint == null) {
                    c.Joint = c.A.Root.AddComponent<FixedJoint>();
                    c.Joint.connectedBody = c.B.Body;
                    c.Joint.enableCollision = false;
                    run.Event("coupled", c.A.D.Id, new Dictionary<string, object> { ["coupling"] = c.Id, ["joint"] = "FixedJoint (approaching rover -> partner), created at the current relative pose",
                        ["gap"] = c.Pose.Gap, ["lateral"] = c.Pose.Lateral, ["yaw"] = c.Pose.Yaw, ["height"] = c.Pose.Height,
                        ["settled_lateral"] = c.Machine.Settled.Lateral, ["settled_yaw"] = c.Machine.Settled.Yaw });
                    OnLatched?.Invoke(c);
                }
                if ((st == ConnectorState.Released || st == ConnectorState.Free) && c.Joint != null) {
                    UnityEngine.Object.DestroyImmediate(c.Joint);
                    c.Joint = null;
                    run.Event("separated", c.A.D.Id, new Dictionary<string, object> { ["coupling"] = c.Id });
                }
                // hooks and LEDs
                Color led = Led(st);
                float nits = (float)spec.O["visual.led_nits"].Double;
                foreach (var (rv, face) in new[] { (c.A, c.FaceA), (c.B, c.FaceB) }) {
                    if (rv.Visual == null) continue;
                    rv.Visual.SetFaceHooks(face, c.Machine.HookPosA, c.Machine.HookPosB);
                    rv.Visual.SetLed(face, led, nits);
                }
            }
        }

        public static Color Led(ConnectorState s) {
            switch (s) {
                case ConnectorState.Approach: return new Color(0.6f, 0.75f, 1f);
                case ConnectorState.MagnetCaptured: return Color.white;
                case ConnectorState.HookAEngaged: case ConnectorState.HookBEngaged: return new Color(1f, 0.65f, 0.1f);
                case ConnectorState.LatchedVerified: return new Color(0.1f, 1f, 0.2f);
                case ConnectorState.LoadValidated: return new Color(0.1f, 0.9f, 0.8f);
                case ConnectorState.ReleaseRequested: return new Color(0.95f, 0.1f, 0.6f);
                case ConnectorState.Free: case ConnectorState.Released: return Color.black;
                default: return Color.red;
            }
        }

        public ConnectorStatus Status(RoverRig r, string face) {
            var c = Of(r, face);
            if (c == null) return new ConnectorStatus { Face = face, State = "FREE" };
            return new ConnectorStatus { Face = face, Coupling = c.Id, State = ConnectorStateNames.ToProfileName(c.Machine.State), SensedA = c.Machine.SensedA, SensedB = c.Machine.SensedB,
                Coupled = c.Joint != null, FailedAttempts = c.Machine.FailedAttempts, RetriesExhausted = c.Machine.RetriesExhausted };
        }
    }

    /// <summary>IRoverIO over a RoverRig: commands into the plant/lifter/connectors, observations from the perception host and CRANE odometry.</summary>
    public sealed class SimRoverIO : IRoverIO {
        public readonly RoverRig Rig;
        readonly FleetPerceptionHost perception;
        readonly CouplingManager couplings;
        readonly FleetRun run;
        public double LiftTarget, GripTarget = double.NaN;
        public string Controller = "none", Phase = "";
        readonly List<string> faces = new List<string>();

        public SimRoverIO(RoverRig rig, FleetPerceptionHost perception, CouplingManager couplings, FleetRun run) {
            Rig = rig; this.perception = perception; this.couplings = couplings; this.run = run;
            foreach (var f in rig.D.Faces) faces.Add(f.Name);
            GripTarget = rig.GripperOpening;
        }

        public string Id => Rig.D.Id;
        public string Role => Rig.D.Role;
        public RoverDescription Description => Rig.D;
        public IObservationProvider Sensors => perception.Provider(Rig.D.Id);
        public OdometryEstimate Odometry {
            get {
                var o = Rig.Odometry;
                return new OdometryEstimate { T = run.T, X = o.X, Y = o.Y, Yaw = o.Yaw, Twist = o.LastTwist };
            }
        }
        public double LiftPosition => Rig.Lift;
        public double GripperOpening => Rig.GripperOpening;
        public bool GripperHolding { get; set; }
        public IReadOnlyList<string> Faces => faces;
        public ConnectorStatus Connector(string face) => couplings.Status(Rig, face);
        public void CommandTwist(Twist body) { Rig.Command = body; }
        public void CommandLift(double q) { LiftTarget = q; }
        public void CommandGripper(double opening) { GripTarget = opening; }
        public void CommandConnector(string face, DockingCommand cmd) => couplings.Command(Rig, face, cmd);
        public void Report(string controller, string phase) {
            if (controller == Controller && phase == Phase) return;
            Controller = controller; Phase = phase;
            run.Event("controller", Rig.D.Id, new Dictionary<string, object> { ["controller"] = controller, ["phase"] = phase });
        }
    }
}
