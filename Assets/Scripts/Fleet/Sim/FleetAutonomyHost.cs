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
        readonly EvaluatorTruth truth;
        readonly Scenario scene;
        readonly ulong seed;
        // tagged objects (payloads): pose history of the object's base_link and its tag; observations generated with the core's TagObserver
        readonly Dictionary<string, (TagSpec tag, List<(double t, double x, double y, double yaw)> hist)> objects =
            new Dictionary<string, (TagSpec, List<(double, double, double, double)>)>();
        readonly Dictionary<string, long> objCursor = new Dictionary<string, long>();
        readonly Dictionary<string, List<RelPoseObservation>> objQueue = new Dictionary<string, List<RelPoseObservation>>();
        public bool CameraLost(string id) => rigs.TryGetValue(id, out var r) && r.CameraLost;

        public FleetPerceptionHost(FleetSpec spec, IEnumerable<RoverRig> rovers, long seed) {
            var profile = FleetProfile.Parse(System.IO.File.ReadAllText(spec.Profile.Source));
            var scene = Scenario.Parse(System.IO.File.ReadAllText(spec.Scenario.Source));
            truth = new EvaluatorTruth();
            this.scene = scene;
            foreach (var r in rovers) {
                string id = r.D.Id;
                var h = hist[id] = new List<(double, double, double, double, double, double, double, double, double)>();
                truth.SetTrajectory(id, t => State(h, t));
            }
            this.seed = FleetRng.For(seed, "perception").NextU64();
            Sim = new FleetSimulator(profile, scene, truth, this.seed);
            // the selected profile's overlay entry decides (profiles.<key>.manifest_tags): v1 yes; v0 keeps the profile-derived mounts its evidence used
            if (spec.O.TryGet("profiles." + spec.ProfileKey + ".manifest_tags")?.Bool == true) foreach (var r in rovers) ApplyManifestTags(r);
            foreach (var r in rovers) { providers[r.D.Id] = new LossyProvider(Sim.ProviderFor(r.D.Id), this); rigs[r.D.Id] = r; }
        }

        /// <summary>When the rover's asset manifest declares its tags (CAD v1: a 30 mm tag on a 45 deg bevel above each dock face, a top tag),
        /// the perception geometry uses those mounts and sizes instead of the profile-derived ones (v0: a vertical face tag in the pocket).
        /// Tag ids stay the perception core's (unique per rover index). Mount poses are in base_link (same frame as the profile face frames the
        /// v0 mounts are built from). Observations still resolve to the partner's base_link, so docking consumers need no change.</summary>
        void ApplyManifestTags(RoverRig r) {
            var a = r.D.Asset;
            if (a == null || a.Tags.Count == 0) return;
            var byName = new Dictionary<string, AssetManifest.Tag>();
            foreach (var t in a.Tags) byName[t.Name] = t;
            foreach (var spec in Sim.GeometryOf(r.D.Id).Tags) {
                AssetManifest.Tag t = null;
                if (spec.Name.StartsWith("face_")) { if (a.FaceTags.TryGetValue(spec.Name.Substring(5), out var tn)) byName.TryGetValue(tn, out t); }
                else if (spec.Name == "top") byName.TryGetValue("tag_top", out t);
                if (t == null || t.Parent != "base_link") continue;
                var R = AssetManifest.OutwardRotation(t.Origin.Rpy);
                spec.MountInOwner = new PPose(new Mat3(R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8]), new Vec3(t.Origin.Xyz[0], t.Origin.Xyz[1], t.Origin.Xyz[2]));
                spec.Size = t.Size;
                spec.Provenance = "asset_manifest " + t.Name;
            }
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

        public void RecordObject(double t, string id, Pose2 basePose, TagSpec tag) {
            if (!objects.TryGetValue(id, out var o)) objects[id] = o = (tag, new List<(double, double, double, double)>());
            o.hist.Add((t, basePose.X, basePose.Y, basePose.Yaw));
            if (o.hist.Count > 4000) o.hist.RemoveRange(0, 1000);
        }

        static (double x, double y, double yaw) At(List<(double t, double x, double y, double yaw)> h, double t) {
            var s = h[0];
            for (int i = h.Count - 1; i >= 0; i--) if (h[i].t <= t + 1e-12) { s = h[i]; break; }
            return (s.x, s.y, s.yaw);
        }

        /// <summary>Own-camera observations of tagged objects (child "&lt;object&gt;/base_link"), generated at each camera frame with the perception
        /// core's TagObserver (noise, dropout, latency, ambiguity, occlusion by the scene and other rover bodies), delivered by StampDelivered.</summary>
        public IReadOnlyList<RelPoseObservation> PollObjectObservations(string roverId, double now) {
            if (!objQueue.TryGetValue(roverId, out var q)) objQueue[roverId] = q = new List<RelPoseObservation>();
            var result = new List<RelPoseObservation>();
            if (objects.Count == 0 || CameraLost(roverId)) { q.Clear(); return result; }
            foreach (var cam in Sim.SuiteOf(roverId).Cameras) {
                string key = roverId + "/" + cam.Name;
                long next = objCursor.TryGetValue(key, out var c) ? c : 1, last = (long)Math.Floor(now * cam.RateHz + 1e-9);
                for (long k = next; k <= last; k++) {
                    double t = k / cam.RateHz;
                    var obs = truth.At(roverId, t);
                    PPose camWorld = obs.Pose.Compose(cam.OpticalInRover);
                    var camMotion = new BodyMotion(obs.LinVelWorld, obs.AngVelWorld);
                    var occ = new List<Obb>(scene.Boxes);
                    foreach (var id in truth.RoverIds) if (id != roverId) occ.Add(Sim.GeometryOf(id).BodyBox(truth.At(id, t).Pose, id + "/body"));
                    foreach (var kv in objects) {
                        if (kv.Value.hist.Count == 0) continue;
                        var (x, y, yaw) = At(kv.Value.hist, t);
                        var tag = kv.Value.tag;
                        var rng = Rng.ForStream(seed, $"objdet/{roverId}/{cam.Name}/{k}/{tag.Id}");
                        var det = TagObserver.Detect(camWorld, camMotion, cam.K, PPose.FromYaw(x, y, 0, yaw).Compose(tag.MountInOwner), BodyMotion.Still, tag, occ,
                            Sim.Config.Visibility, Sim.Config.Noise, cam.LatencyS, t, rng);
                        if (!det.Detected) continue;
                        PPose baseTag = cam.OpticalInRover.Compose(det.CamTag);
                        Matrix covBaseTag = PoseUncertainty.RotateInto(cam.OpticalInRover.R, det.CovarianceCam);
                        PPose mInv = tag.MountInOwner.Inverse();
                        q.Add(new RelPoseObservation { StampAcquired = t, StampDelivered = det.StampDelivered, SourceId = key, SourceKind = ObservationSourceKind.OwnCamera,
                            TagId = tag.Id, AmbiguousPose = det.Pnp.Ambiguous, FrameParent = roverId + "/base_link", FrameChild = kv.Key + "/base_link",
                            Pose = baseTag.Compose(mInv), Covariance = PoseUncertainty.RigidShift(baseTag, mInv, covBaseTag) });
                    }
                }
                objCursor[key] = last + 1;
            }
            q.Sort((a, b) => a.StampDelivered.CompareTo(b.StampDelivered));
            int n = 0; while (n < q.Count && q[n].StampDelivered <= now) n++;
            for (int i = 0; i < n; i++) { var o = q[i].Clone(); o.Age = now - o.StampAcquired; result.Add(o); }
            q.RemoveRange(0, n);
            return result;
        }

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
            public bool LinkUp = true;
            public RequestedAction LastAction;
        }

        readonly FleetRun run;
        readonly FleetSpec spec;
        readonly ContactModel model;
        public readonly List<Coupling> All = new List<Coupling>();
        public double PairRange = 0.6;
        public Action<Coupling> OnLatched;

        /// <summary>Reciprocal faces (profile docking.hook_count = 1, CAD v1: one J-hook + one pin per face). The connector machine's HOOK_A is
        /// the initiating rover's (A) own hook and HOOK_B the partner's (B) hook on the mating face, so each rover's own hook shows and senses
        /// its own side. With two hooks per face (v0) both rovers show the machine's A/B hooks as before.</summary>
        public readonly bool Reciprocal;

        /// <summary>(own, partner) hook value for rover <paramref name="rv"/> of coupling <paramref name="c"/>: machine (A, B) for the initiator,
        /// (B, A) for the partner when faces are reciprocal; (A, B) for both otherwise.</summary>
        public (T own, T partner) OwnPartner<T>(Coupling c, RoverRig rv, T a, T b) => Reciprocal && rv == c.B ? (b, a) : (a, b);

        public CouplingManager(FleetRun run, FleetSpec spec) {
            this.run = run; this.spec = spec;
            Reciprocal = spec.P.TryGet("docking.hook_count") != null && spec.P["docking.hook_count"].Double == 1;
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

        /// <summary>Create the coupling (connector machine, state FREE) for this face if a facing partner face is in range; null otherwise.</summary>
        public Coupling TryPair(RoverRig r, string face) => Of(r, face) ?? Pair(r, face, quiet: true);

        Coupling Pair(RoverRig r, string face, bool quiet = false) {
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
            if (Of(br, bf) != null) return null;   // the partner face is already in another coupling
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
            // idle couplings whose faces separated again are dropped (proximity pairing must not lock a face to a passer-by)
            All.RemoveAll(c => c.Joint == null && (c.Machine.State == ConnectorState.Free || c.Machine.State == ConnectorState.Released) && c.Cmd == DockingCommand.None
                && (c.A.Frames["face_" + c.FaceA].position - c.B.Frames["face_" + c.FaceB].position).magnitude > PairRange * 1.2);
            foreach (var c in All) {
                c.Pose = Relative(c.A.Frames["face_" + c.FaceA], c.B.Frames["face_" + c.FaceB]);
                c.Closing = double.IsNaN(c.PrevGap) ? 0 : (c.PrevGap - c.Pose.Gap) / dt;
                c.PrevGap = c.Pose.Gap;
                // partner link (reciprocal v1 faces): the partner's hook report reaches the connector only while both rovers' links are up; the docking
                // core applies the link latency and fails safe when the report goes stale (not engaged; release of a verified connection refused).
                bool linkUp = !c.A.LinkLost && !c.B.LinkLost;
                if (linkUp != c.LinkUp) { c.LinkUp = linkUp; run.Event(linkUp ? "partner_link_up" : "partner_link_down", c.A.D.Id, new Dictionary<string, object> { ["coupling"] = c.Id, ["reciprocal"] = Reciprocal }); }
                var res = c.Machine.Step(dt, new ConnectorInputs { Pose = c.Pose, ClosingSpeed = c.Closing, Command = c.Cmd, ReleaseAuthorized = true, PartnerLinkUp = linkUp });
                foreach (var e in res.Events)
                    run.Event("connector", c.A.D.Id, new Dictionary<string, object> { ["coupling"] = c.Id, ["event"] = e.Kind.ToString(), ["from"] = ConnectorStateNames.ToProfileName(e.From),
                        ["to"] = ConnectorStateNames.ToProfileName(e.To), ["trigger"] = e.Trigger?.ToString(), ["detail"] = e.Detail, ["gap"] = c.Pose.Gap, ["lateral"] = c.Pose.Lateral, ["yaw"] = c.Pose.Yaw });
                c.LastAction = res.Action;
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
                    var (own, partner) = OwnPartner(c, rv, c.Machine.HookPosA, c.Machine.HookPosB);
                    rv.Visual.SetFaceHooks(face, own, partner);   // v1 manifests have one hook per face (_a = own); v0 two (_a, _b)
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
