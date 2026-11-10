using System;
using System.Collections.Generic;
using Crane.Fleet.Docking;
using Crane.Fleet.Perception;
using DockProfile = Crane.Fleet.Profile.DockingProfile;

namespace Crane.Fleet.Autonomy.Tests.Harness
{
    /// <summary>
    /// Minimal deterministic 2-D kinematic world for headless mission runs (TEST-SIDE ONLY: it holds ground truth).
    /// Plant per rigid group: wheel commands -> first-order motor lag -> per-wheel Gauss-Markov slip -> least-squares body twist
    /// from the live wheels (locked wheels as hard constraints, dead-free wheels excluded, weak wheels clamped) with the floor's
    /// lateral efficiency -> contact tracking lag -> integration. Encoders (quantised, from motor-side wheel angle), gyro (bias
    /// random walk + white noise), own-camera tag detections through Crane.Fleet.Perception (corner noise, PnP, latency,
    /// dropout, occlusion), optional overhead camera, connector machines from Crane.Fleet.Docking fed with the true face pose.
    /// </summary>
    public sealed class KinematicWorld
    {
        public sealed class Rover
        {
            public RoverSpec Spec; public int Index;
            public Pose2 Pose; public Twist2 Vel;       // truth (body twist of the rover's own base)
            public readonly double[] Cmd = new double[4], Wheel = new double[4], Angle = new double[4], PrevAngle = new double[4], Slip = new double[4];
            public readonly CapHealth[] Health = { CapHealth.Ok, CapHealth.Ok, CapHealth.Ok, CapHealth.Ok };
            public double GyroBias; public Rng Rng; public double AccelX, AccelY; public long AccK;
            public RoverGeometry Geometry;
            public readonly List<CameraMount3> Cameras = new List<CameraMount3>();
            public Group Group; public Pose2 InGroup;
            public double LiftHeight, LiftTarget, LiftJamUntil = double.NegativeInfinity; public bool GripCmd, GripClosed, ForceLimited; public string Holding;
            public readonly Dictionary<string, DockingCommand> ConnCmd = new Dictionary<string, DockingCommand>();
            // sensor queues
            public readonly List<EncoderSample> EncQ = new List<EncoderSample>();
            public readonly List<GyroSample> GyroQ = new List<GyroSample>();
            public readonly List<RelPoseObservation> TagQ = new List<RelPoseObservation>();
            public readonly List<AccelSample> AccQ = new List<AccelSample>();
            public double PrevVx, PrevVy; public bool HasPrevV;
            public long EncK, GyroK; public readonly Dictionary<string, long> CamK = new Dictionary<string, long>();
            public Pose2 Estimate; public double[] EstimateCov = new double[9];
        }

        public sealed class Group
        {
            public readonly List<Rover> Members = new List<Rover>();
            public Rover Root => Members[0];
            public Twist2 Vel;   // body twist of the root
            public double ShoveVx, ShoveVy, ShoveUntil = double.NegativeInfinity;   // external push (world frame), unseen by the wheels
            public bool Blocked;
        }

        public sealed class Landmark { public string Name; public Pose2 Pose; public TagSpec Tag; }
        public readonly List<Landmark> Landmarks = new List<Landmark>();
        public bool DeckLoadCells;
        /// <summary>Rovers that receive an absolute pose fix (stand-in for LiDAR scan matching) every ScanFixPeriod s, sigma ScanFixSigma.</summary>
        public readonly HashSet<string> ScanMatchRovers = new HashSet<string>(); public double ScanFixPeriod = 0.5, ScanFixSigma = 0.02;   // cargo decks report load (LiftReport.DeckLoadValid)
        public double AccelNoise = 0.02;

        /// <summary>A tag on a static structure (door jamb, wall). Its base frame is planar at <paramref name="pose"/>; the tag faces +x of it.</summary>
        public Landmark AddLandmark(string name, Pose2 pose)
        {
            var l = new Landmark { Name = name, Pose = pose, Tag = new TagSpec { Id = 800 + Landmarks.Count, OwnerId = name, Name = "landmark", Size = Profile.TopTagSize, MountInOwner = TagSpec.OnVerticalFace(new Vec3(0, 0, 0.08), 0) } };
            Landmarks.Add(l); return l;
        }

        /// <summary>Push a rover's group with a world-frame velocity for a duration (another rover shoving it): the wheels do not see it.</summary>
        public void Shove(string rover, double vx, double vy, double duration) { var g = R(rover).Group; g.ShoveVx = vx; g.ShoveVy = vy; g.ShoveUntil = Time + duration; }

        public sealed class Pair
        {
            public string Docker, DockerFace, Target, TargetFace;
            public ConnectorMachine Machine;
            public double PrevGap = double.NaN;
            public bool Welded;
            public bool Obstruction;
            public RequestedAction LastAction;
        }

        public sealed class PayloadBody
        {
            public string Id, Type; public double Lx, Ly, Mass; public Pose2 Pose;
            public string CarriedBy;      // rover id the payload rests on (moves with its group), or lifter holding it
            public Pose2 InCarrier;
            public List<string> Supporters = new List<string>();
            public TagSpec Tag;
            public bool Dropped;
        }

        public readonly FleetProfile Profile;
        public readonly DockProfile DockingProfile;
        public readonly DockingAssumptions DockAssume;
        public readonly ContactModel Contact;
        public readonly PlanMap Map;
        public readonly ulong Seed;
        public double Time;
        public readonly List<Rover> Rovers = new List<Rover>();
        public readonly List<Group> Groups = new List<Group>();
        public readonly List<Pair> Pairs = new List<Pair>();
        public readonly List<PayloadBody> Payloads = new List<PayloadBody>();
        public readonly List<string> Collisions = new List<string>();
        public readonly List<Obb> StaticOccluders = new List<Obb>();
        public readonly VisibilityParams Vis = new VisibilityParams();
        public readonly TagNoiseParams Noise = new TagNoiseParams();
        public double CameraRateHz = 20, MotorTau = 0.05, TrackTau = 0.04, SlipTau = 0.2, InitialBiasSigma = 0.002;
        public double TrueLateralEfficiency;   // may differ from the profile value the autonomy assumes (model mismatch)
        public double SlipSigma;
        public bool Overhead; public Pose OverheadPose; public CameraIntrinsics OverheadK; public double OverheadRate = 10, OverheadLatency = 0.15;
        public Pose2 OverheadSurveyError;
        public int UnsafeReleases;

        public KinematicWorld(FleetProfile profile, DockProfile dp, PlanMap map, ulong seed, string floor = "hard_tile")
        {
            Profile = profile; DockingProfile = dp; DockAssume = new DockingAssumptions(); Contact = new ContactModel(dp, DockAssume); Map = map; Seed = seed;
            TrueLateralEfficiency = profile.Num("floors." + floor + ".lateral_efficiency");
            SlipSigma = profile.Num("floors." + floor + ".slip_sigma_frac");
            foreach (var o in map.Obstacles) StaticOccluders.Add(new Obb(o.Id, new Vec3(o.Cx, o.Cy, 1.0), new Vec3(o.Hx, o.Hy, 1.0), o.Yaw));
        }

        public Rover AddRover(RoverSpec spec, Pose2 pose)
        {
            var r = new Rover { Spec = spec, Index = Rovers.Count + 1, Pose = pose, Rng = Rng.ForStream(Seed, "world/" + spec.Id) };
            r.GyroBias = InitialBiasSigma * r.Rng.Gaussian();
            for (int i = 0; i < 4; i++) r.Slip[i] = 1;
            r.Geometry = new RoverGeometry(Profile, spec.Id, r.Index);
            var b = Profile.BodySizeLwh;
            if (spec.HasFrontCamera) r.Cameras.Add(new CameraMount3 { Name = "cam_front", Facing = "front", OpticalInRover = CameraMount.InRover(new Vec3(b[0] / 2 - 0.01, 0, b[2] * 0.8), 0), K = Profile.Camera.Intrinsics, RateHz = CameraRateHz, LatencyS = Profile.Camera.Latency });
            if (spec.HasRearCamera) r.Cameras.Add(new CameraMount3 { Name = "cam_rear", Facing = "rear", OpticalInRover = CameraMount.InRover(new Vec3(-(b[0] / 2 - 0.01), 0, b[2] * 0.8), Math.PI), K = Profile.Camera.Intrinsics, RateHz = CameraRateHz, LatencyS = Profile.Camera.Latency });
            var g = new Group(); g.Members.Add(r); r.Group = g; r.InGroup = Pose2.Identity; Groups.Add(g);
            Rovers.Add(r);
            return r;
        }

        public Rover R(string id) { foreach (var r in Rovers) if (r.Spec.Id == id) return r; throw new KeyNotFoundException(id); }

        public Pair AddPair(string docker, string dockerFace, string target, string targetFace)
        {
            var p = new Pair { Docker = docker, DockerFace = dockerFace, Target = target, TargetFace = targetFace };
            ulong s = Seed * 1000003UL + (ulong)Pairs.Count * 7919UL + 17UL;
            p.Machine = new ConnectorMachine(Contact, s, docker + "." + dockerFace + "-" + target + "." + targetFace);
            Pairs.Add(p);
            return p;
        }

        /// <summary>Pre-latch a pair (pre-assembled group): the docker is snapped to the mated pose and welded.</summary>
        public void PreLatch(Pair p)
        {
            // a real capture-engage-verify sequence on a deterministic connector (no catch misses or false positives)
            p.Machine = new ConnectorMachine(new ContactModel(DockingProfile, DockAssume.Deterministic()), Seed + 99, p.Machine.Id);
            var seq = new[] { DockingCommand.Approach, DockingCommand.None, DockingCommand.EngageHooks };
            for (int k = 0; k < 200; k++)
            {
                var cmd = k < seq.Length ? seq[k] : k == 150 ? DockingCommand.Verify : k == 160 ? DockingCommand.TestLoad : DockingCommand.None;
                p.Machine.Step(0.02, new ConnectorInputs { Pose = new FacePose(0, k == 0 ? 0.02 : 0.0, 0), ClosingSpeed = 0.01, Command = cmd, ReleaseAuthorized = true });
            }
            if (!ConnectorStateNames.IsVerified(p.Machine.State)) throw new InvalidOperationException("pre-latch failed: " + p.Machine.State);
            var t = R(p.Target); var d = R(p.Docker);
            d.Pose = AssemblyModel.MatedPose(t.Pose, t.Spec.Face(p.TargetFace), d.Spec.Face(p.DockerFace));
            Weld(t, d); p.Welded = true;
        }

        public PayloadBody AddPayload(string id, string type, Pose2 pose, params string[] supporters)
        {
            var spec = DockingProfile.Payloads[type];
            var pb = new PayloadBody { Id = id, Type = type, Lx = spec.SizeLwh.Value[0], Ly = spec.SizeLwh.Value[1], Mass = spec.Mass.Value, Pose = pose };
            pb.Supporters.AddRange(supporters);
            if (supporters.Length > 0) { pb.CarriedBy = supporters[0]; pb.InCarrier = Se2.Between(R(supporters[0]).Pose, pose); }
            pb.Tag = new TagSpec { Id = 900 + Payloads.Count, OwnerId = id, Name = "payload_side", Size = Profile.TopTagSize, MountInOwner = TagSpec.OnVerticalFace(new Vec3(-pb.Lx / 2 - 0.002, 0, 0.20), Math.PI) };
            Payloads.Add(pb);
            return pb;
        }

        // ----------------------------------------------------------------------------------- groups
        private void Weld(Rover a, Rover b)
        {
            var ga = a.Group; var gb = b.Group;
            if (ga == gb) return;
            foreach (var m in gb.Members) { m.InGroup = Se2.Between(ga.Root.Pose, m.Pose); m.Group = ga; ga.Members.Add(m); }
            Groups.Remove(gb);
        }

        private void Unweld(Rover leaving, Pair releasedPair)
        {
            var g = leaving.Group;
            // connected component of `leaving` over welded pairs other than the released one
            var comp = new HashSet<Rover> { leaving }; bool grew = true;
            while (grew)
            {
                grew = false;
                foreach (var p in Pairs)
                {
                    if (!p.Welded || p == releasedPair) continue;
                    var a = R(p.Docker); var b = R(p.Target);
                    if (a.Group != g) continue;
                    if (comp.Contains(a) && comp.Add(b)) grew = true;
                    if (comp.Contains(b) && comp.Add(a)) grew = true;
                }
            }
            if (comp.Count == g.Members.Count) return;
            var ng = new Group { Vel = Twist2.Zero };
            foreach (var m in g.Members) if (comp.Contains(m)) ng.Members.Add(m);
            g.Members.RemoveAll(m => comp.Contains(m));
            foreach (var gg in new[] { g, ng }) foreach (var m in gg.Members) { m.Group = gg; m.InGroup = Se2.Between(gg.Root.Pose, m.Pose); }
            g.Vel = Twist2.Zero;
            Groups.Add(ng);
        }

        // ----------------------------------------------------------------------------------- step
        public void Step(double dt)
        {
            foreach (var r in Rovers) Array.Copy(r.Angle, r.PrevAngle, 4);
            Time += dt;
            foreach (var g in Groups) StepGroup(g, dt);
            ResolveFaceContacts();
            CheckCollisions();
            StepConnectors(dt);
            StepLifts(dt);
            foreach (var p in Payloads) if (p.CarriedBy != null) p.Pose = Se2.Compose(R(p.CarriedBy).Pose, p.InCarrier);
            Sample(dt);
        }

        private readonly List<double[]> _rows = new List<double[]>();

        private void StepGroup(Group g, double dt)
        {
            double a = 1 - Math.Exp(-dt / MotorTau);
            double rho = Math.Exp(-dt / SlipTau), q = SlipSigma * Math.Sqrt(1 - rho * rho);
            // normal equations of the weighted LSQ: min sum (row_i xi - g_i)^2 over live wheels, subject to locked rows = 0
            var AtA = new double[9]; var Atb = new double[3];
            var locked = new List<double[]>();
            foreach (var m in g.Members)
            {
                var s = m.Spec;
                double c = Math.Cos(m.InGroup.Yaw), sn = Math.Sin(m.InGroup.Yaw), px = m.InGroup.X, py = m.InGroup.Y;
                for (int j = 0; j < 4; j++)
                {
                    var h = m.Health[j];
                    double lim = s.MaxWheelSpeed * (h.Mode == CapHealthMode.Weak ? h.Gain : 1);
                    double target = Math.Max(-lim, Math.Min(lim, m.Cmd[j]));
                    m.Slip[j] = 1 + (m.Slip[j] - 1) * rho + q * m.Rng.Gaussian();
                    s.WheelRow(j, out double ax, out double ay, out double aw);
                    if (h.Mode == CapHealthMode.DeadLocked)
                    {
                        // a locked wheel's rollers still roll freely: kinematic constraint (no lateral-efficiency loss)
                        m.Wheel[j] = 0; locked.Add(new[] { ax * c - ay * sn, ax * sn + ay * c, ax * (-c * py + sn * px) + ay * (sn * py + c * px) + aw }); continue;
                    }
                    ay /= TrueLateralEfficiency;
                    var row = new[] { ax * c - ay * sn, ax * sn + ay * c, ax * (-c * py + sn * px) + ay * (sn * py + c * px) + aw };
                    if (h.Mode == CapHealthMode.DeadFree) continue;
                    m.Wheel[j] += (target - m.Wheel[j]) * a;
                    double ground = m.Wheel[j] * m.Slip[j];
                    for (int r = 0; r < 3; r++) { Atb[r] += row[r] * ground; for (int k = 0; k < 3; k++) AtA[3 * r + k] += row[r] * row[k]; }
                }
            }
            var xi = SolveConstrained(AtA, Atb, locked);
            double b = 1 - Math.Exp(-dt / TrackTau);
            g.Vel = g.Vel + (xi - g.Vel) * b;
            var root = g.Root;
            var newRoot = Se2.Advance(root.Pose, g.Vel, dt);
            if (Time <= g.ShoveUntil) newRoot = new Pose2(newRoot.X + g.ShoveVx * dt, newRoot.Y + g.ShoveVy * dt, newRoot.Yaw);
            // rigid contact with static structure: the body stops, the wheels keep turning (they slip on the floor)
            g.Blocked = false;
            foreach (var m in g.Members)
            {
                var bb = Body(m); var nb = new Box2(bb.Id, 0, 0, 0, bb.Hx, bb.Hy).Transformed(Se2.Compose(newRoot, m.InGroup));
                foreach (var o in Map.Obstacles) if (Box2.Overlap(nb, o)) { g.Blocked = true; break; }
                foreach (var p in Payloads)
                {
                    if (p.CarriedBy != m.Spec.Id) continue;
                    var pb = new Box2(p.Id, 0, 0, 0, p.Lx / 2, p.Ly / 2).Transformed(Se2.Compose(Se2.Compose(newRoot, m.InGroup), p.InCarrier));
                    foreach (var o in Map.Obstacles) if (Box2.Overlap(pb, o)) { g.Blocked = true; break; }
                }
            }
            if (g.Blocked) { newRoot = root.Pose.Compose(Pose2.Identity); newRoot = new Pose2(newRoot.X, newRoot.Y, newRoot.Yaw); g.Vel = Twist2.Zero; Collisions.Add($"{Time:0.00} {root.Spec.Id} group blocked by structure"); }
            double shx = Time <= g.ShoveUntil ? g.ShoveVx : 0, shy = Time <= g.ShoveUntil ? g.ShoveVy : 0;
            foreach (var m in g.Members)
            {
                var old = m.Pose;
                m.Pose = Se2.Compose(newRoot, m.InGroup);
                // accelerometer: body-frame acceleration of the member from the group twist plus the external push
                {
                    double c0 = Math.Cos(m.InGroup.Yaw), s0 = Math.Sin(m.InGroup.Yaw);
                    double wvx = g.Vel.Vx - g.Vel.Wz * m.InGroup.Y, wvy = g.Vel.Vy + g.Vel.Wz * m.InGroup.X;
                    double bvx = c0 * wvx + s0 * wvy, bvy = -s0 * wvx + c0 * wvy;
                    double cy = Math.Cos(m.Pose.Yaw), sy = Math.Sin(m.Pose.Yaw);
                    bvx += cy * shx + sy * shy; bvy += -sy * shx + cy * shy;
                    if (!m.HasPrevV) { m.PrevVx = bvx; m.PrevVy = bvy; m.HasPrevV = true; }
                    double w = g.Vel.Wz;
                    m.AccelX = (bvx - m.PrevVx) / dt - w * bvy; m.AccelY = (bvy - m.PrevVy) / dt + w * bvx;
                    m.PrevVx = bvx; m.PrevVy = bvy;
                }
                var d = Se2.Between(old, m.Pose);
                m.Vel = new Twist2(d.X / dt, d.Y / dt, d.Yaw / dt);
                // wheel angles: driven wheels integrate motor speed; dead-free wheels roll with the ground
                for (int j = 0; j < 4; j++)
                {
                    if (m.Health[j].Mode == CapHealthMode.DeadFree)
                    {
                        m.Spec.WheelRow(j, out double ax, out double ay, out double aw);
                        m.Wheel[j] = ax * m.Vel.Vx + ay / TrueLateralEfficiency * m.Vel.Vy + aw * m.Vel.Wz;
                    }
                    m.Angle[j] += m.Wheel[j] * dt;
                }
            }
        }

        private static Twist2 SolveConstrained(double[] AtA, double[] Atb, List<double[]> locked)
        {
            // projector onto null(locked)
            var q = new List<double[]>();
            foreach (var v0 in locked)
            {
                var v = (double[])v0.Clone();
                foreach (var bq in q) { double d = v[0] * bq[0] + v[1] * bq[1] + v[2] * bq[2]; for (int t = 0; t < 3; t++) v[t] -= d * bq[t]; }
                double n = Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
                if (n > 1e-9) q.Add(new[] { v[0] / n, v[1] / n, v[2] / n });
            }
            var P = new double[9];
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) { double s = i == j ? 1 : 0; foreach (var bq in q) s -= bq[i] * bq[j]; P[3 * i + j] = s; }
            // solve (P AtA P + (I - P)) x = P Atb
            var t1 = new double[9]; var M = new double[9];
            Lin.Mul(P, AtA, t1, 3, 3, 3); Lin.Mul(t1, P, M, 3, 3, 3);
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) M[3 * i + j] += (i == j ? 1 : 0) - P[3 * i + j];
            for (int i = 0; i < 3; i++) M[4 * i] += 1e-12;
            var rhs = new double[3]; for (int i = 0; i < 3; i++) rhs[i] = P[3 * i] * Atb[0] + P[3 * i + 1] * Atb[1] + P[3 * i + 2] * Atb[2];
            var inv = new double[9]; var w = new double[18];
            if (!Lin.Invert(M, inv, 3, w)) return Twist2.Zero;
            var x = new double[3]; for (int i = 0; i < 3; i++) x[i] = inv[3 * i] * rhs[0] + inv[3 * i + 1] * rhs[1] + inv[3 * i + 2] * rhs[2];
            return new Twist2(x[0], x[1], x[2]);
        }

        /// <summary>True face pose of a pair (FacePose convention: B mate frame in A face frame).</summary>
        public Pose2 FacePoseOf(Pair p)
        {
            var a = R(p.Docker); var b = R(p.Target);
            var fa = Se2.Compose(a.Pose, a.Spec.Face(p.DockerFace).InBody);
            var mb = Se2.Compose(Se2.Compose(b.Pose, b.Spec.Face(p.TargetFace).InBody), new Pose2(0, 0, Math.PI));
            return Se2.Between(fa, mb);
        }

        private void ResolveFaceContacts()
        {
            foreach (var p in Pairs)
            {
                if (p.Welded) continue;
                var d = FacePoseOf(p);
                if (d.X < 0 && Math.Abs(d.Y) < 0.2 && Math.Abs(d.Yaw) < 0.6)
                {
                    // faces cannot interpenetrate: push the docker group back along its face normal
                    var a = R(p.Docker); var g = a.Group;
                    var fa = Se2.Compose(a.Pose, a.Spec.Face(p.DockerFace).InBody);
                    double c = Math.Cos(fa.Yaw), s = Math.Sin(fa.Yaw);
                    var shift = new Pose2(d.X * c, d.X * s, 0);
                    var nr = new Pose2(g.Root.Pose.X + shift.X, g.Root.Pose.Y + shift.Y, g.Root.Pose.Yaw);
                    foreach (var m in g.Members) m.Pose = Se2.Compose(nr, m.InGroup);
                    g.Vel = new Twist2(Math.Min(0, g.Vel.Vx), g.Vel.Vy, g.Vel.Wz) * 0.5;
                }
            }
        }

        private void CheckCollisions()
        {
            for (int gi = 0; gi < Groups.Count; gi++)
            {
                var g = Groups[gi];
                foreach (var m in g.Members)
                {
                    var box = Body(m);
                    if (!Map.InBounds(box)) Collisions.Add($"{Time:0.00} {m.Spec.Id} out of bounds");
                    foreach (var o in Map.Obstacles) if (Box2.Overlap(box, o)) { Collisions.Add($"{Time:0.00} {m.Spec.Id} hit {o.Id}"); break; }
                    for (int gj = gi + 1; gj < Groups.Count; gj++)
                        foreach (var o in Groups[gj].Members)
                        {
                            if (Touching(m, o)) continue;
                            if (Box2.Overlap(box, Body(o))) Collisions.Add($"{Time:0.00} {m.Spec.Id} hit {o.Spec.Id}");
                        }
                }
            }
            foreach (var p in Payloads)
            {
                if (p.CarriedBy == null) continue;
                var pb = new Box2(p.Id, p.Pose.X, p.Pose.Y, p.Pose.Yaw, p.Lx / 2, p.Ly / 2);
                foreach (var o in Map.Obstacles) if (Box2.Overlap(pb, o)) { Collisions.Add($"{Time:0.00} payload {p.Id} hit {o.Id}"); break; }
            }
        }

        private bool Touching(Rover a, Rover b)
        {
            foreach (var p in Pairs)
            {
                if (!((p.Docker == a.Spec.Id && p.Target == b.Spec.Id) || (p.Docker == b.Spec.Id && p.Target == a.Spec.Id))) continue;
                var d = FacePoseOf(p);
                if (d.X < 0.06 && Math.Abs(d.Y) < 0.2) return true;
            }
            return false;
        }

        public Box2 Body(Rover m) => new Box2(m.Spec.Id, m.Pose.X, m.Pose.Y, m.Pose.Yaw, m.Spec.BodyLength / 2, m.Spec.OverallWidth / 2);

        private void StepConnectors(double dt)
        {
            foreach (var p in Pairs)
            {
                var d = FacePoseOf(p);
                double closing = double.IsNaN(p.PrevGap) ? 0 : (p.PrevGap - d.X) / dt;
                p.PrevGap = d.X;
                var a = R(p.Docker);
                a.ConnCmd.TryGetValue(p.DockerFace, out var cmd);
                var inp = new ConnectorInputs
                {
                    Pose = new FacePose(d.Y, Math.Max(d.X, -0.001), d.Yaw), ClosingSpeed = closing, Command = cmd,
                    Obstruction = p.Obstruction, ReleaseAuthorized = TrueSupportAllowsRelease(p),
                };
                if (cmd != DockingCommand.None) p.LastAction = RequestedAction.None;
                var res = p.Machine.Step(dt, inp);
                if (res.Action != RequestedAction.None) p.LastAction = res.Action;
                var st = p.Machine.State;
                bool held = st == ConnectorState.MagnetCaptured || st == ConnectorState.HookAEngaged || st == ConnectorState.HookBEngaged || st == ConnectorState.Partial
                            || st == ConnectorState.LatchedVerified || st == ConnectorState.LoadValidated || st == ConnectorState.FalseEngagement || st == ConnectorState.VerificationLost
                            || st == ConnectorState.ReleaseRequested || st == ConnectorState.StuckHook;
                if (held && !p.Welded)
                {
                    // magnets pull the faces together: snap to the mated pose plus the settled residual
                    var t = R(p.Target);
                    var mated = AssemblyModel.MatedPose(t.Pose, t.Spec.Face(p.TargetFace), a.Spec.Face(p.DockerFace));
                    var res2 = p.Machine.Settled;
                    var faceInA = a.Spec.Face(p.DockerFace).InBody;
                    var resid = Se2.Compose(Se2.Compose(faceInA, new Pose2(0, -res2.Lateral, -res2.Yaw)), Se2.Inverse(faceInA));
                    var newA = Se2.Compose(mated, resid);
                    var g = a.Group; var delta = Se2.Compose(newA, Se2.Inverse(a.Pose));
                    foreach (var m in g.Members) m.Pose = Se2.Compose(delta, m.Pose);
                    Weld(t, a); p.Welded = true;
                }
                else if (!held && p.Welded)
                {
                    if (!TrueSupportAllowsRelease(p)) UnsafeReleases++;
                    p.Welded = false; Unweld(a, p);
                }
                foreach (var e in res.Events) if (e.Kind == EventKind.UnsafeRelease) UnsafeReleases++;
            }
            foreach (var r in Rovers) r.ConnCmd.Clear();
        }

        /// <summary>Hardware interlock stand-in: a verified coupling may open only if no payload rests on both of its sides.</summary>
        private bool TrueSupportAllowsRelease(Pair p)
        {
            foreach (var pl in Payloads)
                if (pl.CarriedBy != null && pl.Supporters.Contains(p.Docker) && pl.Supporters.Contains(p.Target)) return false;
            return true;
        }

        private void StepLifts(double dt)
        {
            foreach (var r in Rovers)
            {
                if (!r.Spec.HasLift) continue;
                double sp = 0.05 * dt;
                double held = 0; if (r.Holding != null) held = Payloads.Find(x => x.Id == r.Holding).Mass;
                bool over = held > r.Spec.LiftCapacity, jam = Time < r.LiftJamUntil;
                double dh = Math.Max(-sp, Math.Min(sp, r.LiftTarget - r.LiftHeight));
                if (jam && dh > 0) dh = 0;
                if (over && dh > 0 && r.LiftHeight >= 0.005) dh = 0;      // motor at its force limit: the load does not come up
                r.LiftHeight += dh;
                r.ForceLimited = over && r.LiftTarget > r.LiftHeight + 0.002;
                if (r.GripCmd && !r.GripClosed)
                {
                    r.GripClosed = true;
                    foreach (var p in Payloads)
                    {
                        var rel = Se2.Between(r.Pose, p.Pose);
                        if (p.CarriedBy == null && Math.Abs(rel.Y) < 0.04 && rel.X > 0.2 && rel.X < 0.4 && Math.Abs(rel.Yaw) < 0.1)
                        { r.Holding = p.Id; p.CarriedBy = r.Spec.Id; p.InCarrier = rel; p.Supporters.Clear(); }
                    }
                }
                if (!r.GripCmd && r.GripClosed)
                {
                    r.GripClosed = false;
                    if (r.Holding != null)
                    {
                        var p = Payloads.Find(x => x.Id == r.Holding);
                        r.Holding = null; p.CarriedBy = null; p.Supporters.Clear();
                        var corners = new List<Vec2>();
                        foreach (var o in Rovers)
                        {
                            if (o == r || !o.Spec.HasDeck) continue;
                            var rel = Se2.Between(o.Pose, p.Pose);
                            if (Math.Abs(rel.X) < o.Spec.DeckLength / 2 + p.Lx / 2 && Math.Abs(rel.Y) < o.Spec.DeckWidth / 2 + p.Ly / 2)
                            {
                                p.Supporters.Add(o.Spec.Id);
                                foreach (var (cx, cy) in new[] { (1, 1), (1, -1), (-1, 1), (-1, -1) }) corners.Add(o.Pose.TransformPoint(new Vec2(cx * o.Spec.DeckLength / 2, cy * o.Spec.DeckWidth / 2)));
                            }
                        }
                        // statically supported only if the payload's centre of mass lies over the (convex hull of the) touched decks
                        bool supported = p.Supporters.Count > 0 && Hull2.InsideMargin(Hull2.Convex(corners), p.Pose.Position) >= 0;
                        if (supported) { p.CarriedBy = p.Supporters[0]; p.InCarrier = Se2.Between(R(p.CarriedBy).Pose, p.Pose); }
                        else { p.Supporters.Clear(); p.Dropped = true; }
                    }
                }
            }
        }

        // ----------------------------------------------------------------------------------- sensors
        private void Sample(double dt)
        {
            foreach (var r in Rovers)
            {
                double encDt = 1.0 / r.Spec.EncoderRateHz, gyDt = 1.0 / r.Spec.ImuRateHz;
                while ((r.EncK + 1) * encDt <= Time + 1e-9)
                {
                    r.EncK++;
                    double cpr = r.Spec.CountsPerRev;
                    double ts = r.EncK * encDt, f = Math.Max(0, Math.Min(1, 1 - (Time - ts) / dt));
                    long C(int j) => (long)Math.Floor((r.PrevAngle[j] + (r.Angle[j] - r.PrevAngle[j]) * f) * cpr / (2 * Math.PI));
                    r.EncQ.Add(new EncoderSample(r.EncK * encDt, C(0), C(1), C(2), C(3)));
                }
                while ((r.GyroK + 1) * gyDt <= Time + 1e-9)
                {
                    r.GyroK++;
                    r.GyroBias += r.Spec.GyroBiasInstability * Math.Sqrt(gyDt) * r.Rng.Gaussian();
                    r.GyroQ.Add(new GyroSample(r.GyroK * gyDt, r.Vel.Wz + r.GyroBias + r.Spec.GyroNoiseDensity / Math.Sqrt(gyDt) * r.Rng.Gaussian()));
                }
                while ((r.AccK + 1) * gyDt <= Time + 1e-9)
                {
                    r.AccK++;
                    r.AccQ.Add(new AccelSample(r.AccK * gyDt, r.AccelX + AccelNoise * r.Rng.Gaussian(), r.AccelY + AccelNoise * r.Rng.Gaussian()));
                }
                foreach (var cam in r.Cameras)
                {
                    r.CamK.TryGetValue(cam.Name, out long k);
                    double phase = 0.013 * r.Index;
                    while (phase + (k + 1) / cam.RateHz <= Time + 1e-9) { k++; CameraFrame(r, cam, k, phase + k / cam.RateHz); }
                    r.CamK[cam.Name] = k;
                }
            }
            if (Overhead)
            {
                long k = (long)Math.Floor(Time * OverheadRate + 1e-9);
                if (k != _ohK) { _ohK = k; OverheadFrame(k, k / OverheadRate); }
            }
        }
        private long _ohK = -1;

        private List<Obb> Occluders(Rover except)
        {
            var l = new List<Obb>(StaticOccluders);
            foreach (var o in Rovers) if (o != except) l.Add(o.Geometry.BodyBox(Se2.ToPose(o.Pose), o.Spec.Id + "/body"));
            return l;
        }

        private void CameraFrame(Rover r, CameraMount3 cam, long k, double t)
        {
            var camWorld = Se2.ToPose(r.Pose).Compose(cam.OpticalInRover);
            var camMo = new BodyMotion(new Vec3(r.Vel.Vx * Math.Cos(r.Pose.Yaw) - r.Vel.Vy * Math.Sin(r.Pose.Yaw), r.Vel.Vx * Math.Sin(r.Pose.Yaw) + r.Vel.Vy * Math.Cos(r.Pose.Yaw), 0), new Vec3(0, 0, r.Vel.Wz));
            var occ = Occluders(r);
            foreach (var o in Rovers)
            {
                if (o == r) continue;
                if (Se2.Distance(o.Pose, r.Pose) > 2.5) continue;
                foreach (var tag in o.Geometry.Tags)
                {
                    var rng = Rng.ForStream(Seed, $"det/{r.Spec.Id}/{cam.Name}/{k}/{tag.Id}");
                    var tagWorld = Se2.ToPose(o.Pose).Compose(tag.MountInOwner);
                    var det = TagObserver.Detect(camWorld, camMo, cam.K, tagWorld, BodyMotion.Still, tag, occ, Vis, Noise, cam.LatencyS, t, rng);
                    if (!det.Detected) continue;
                    Pose baseTag = cam.OpticalInRover.Compose(det.CamTag);
                    Matrix covBaseTag = PoseUncertainty.RotateInto(cam.OpticalInRover.R, det.CovarianceCam);
                    Pose mInv = tag.MountInOwner.Inverse();
                    r.TagQ.Add(new RelPoseObservation
                    {
                        StampAcquired = t, StampDelivered = det.StampDelivered, SourceId = r.Spec.Id + "/" + cam.Name, SourceKind = ObservationSourceKind.OwnCamera,
                        TagId = tag.Id, AmbiguousPose = det.Pnp.Ambiguous, FrameParent = r.Spec.Id + "/base_link", FrameChild = o.Spec.Id + "/base_link",
                        Pose = baseTag.Compose(mInv), Covariance = PoseUncertainty.RigidShift(baseTag, mInv, covBaseTag),
                    });
                }
            }
            foreach (var l in Landmarks)
            {
                if (Se2.Distance(l.Pose, r.Pose) > 2.5) continue;
                var rng = Rng.ForStream(Seed, $"det/{r.Spec.Id}/{cam.Name}/{k}/{l.Tag.Id}");
                var tagWorld = Se2.ToPose(l.Pose).Compose(l.Tag.MountInOwner);
                var det = TagObserver.Detect(camWorld, camMo, cam.K, tagWorld, BodyMotion.Still, l.Tag, occ, Vis, Noise, cam.LatencyS, t, rng);
                if (!det.Detected) continue;
                Pose baseTag = cam.OpticalInRover.Compose(det.CamTag);
                Matrix covBaseTag = PoseUncertainty.RotateInto(cam.OpticalInRover.R, det.CovarianceCam);
                Pose mInv = l.Tag.MountInOwner.Inverse();
                r.TagQ.Add(new RelPoseObservation
                {
                    StampAcquired = t, StampDelivered = det.StampDelivered, SourceId = r.Spec.Id + "/" + cam.Name, SourceKind = ObservationSourceKind.OwnCamera,
                    TagId = l.Tag.Id, AmbiguousPose = det.Pnp.Ambiguous, FrameParent = r.Spec.Id + "/base_link", FrameChild = l.Name + "/base_link",
                    Pose = baseTag.Compose(mInv), Covariance = PoseUncertainty.RigidShift(baseTag, mInv, covBaseTag),
                });
            }
            foreach (var p in Payloads)
            {
                if (Se2.Distance(p.Pose, r.Pose) > 3.0) continue;
                var rng = Rng.ForStream(Seed, $"det/{r.Spec.Id}/{cam.Name}/{k}/{p.Tag.Id}");
                var tagWorld = Se2.ToPose(p.Pose).Compose(p.Tag.MountInOwner);
                var det = TagObserver.Detect(camWorld, camMo, cam.K, tagWorld, BodyMotion.Still, p.Tag, occ, Vis, Noise, cam.LatencyS, t, rng);
                if (!det.Detected) continue;
                Pose baseTag = cam.OpticalInRover.Compose(det.CamTag);
                Matrix covBaseTag = PoseUncertainty.RotateInto(cam.OpticalInRover.R, det.CovarianceCam);
                Pose mInv = p.Tag.MountInOwner.Inverse();
                r.TagQ.Add(new RelPoseObservation
                {
                    StampAcquired = t, StampDelivered = det.StampDelivered, SourceId = r.Spec.Id + "/" + cam.Name, SourceKind = ObservationSourceKind.OwnCamera,
                    TagId = p.Tag.Id, AmbiguousPose = det.Pnp.Ambiguous, FrameParent = r.Spec.Id + "/base_link", FrameChild = p.Id + "/base_link",
                    Pose = baseTag.Compose(mInv), Covariance = PoseUncertainty.RigidShift(baseTag, mInv, covBaseTag),
                });
            }
        }

        private void OverheadFrame(long k, double t)
        {
            var occ = Occluders(null);
            var bias = OverheadSurveyError;
            foreach (var o in Rovers)
            {
                foreach (var tag in o.Geometry.Tags)
                {
                    if (tag.Name != "top") continue;
                    var rng = Rng.ForStream(Seed, $"det/overhead/{k}/{tag.Id}");
                    var tagWorld = Se2.ToPose(o.Pose).Compose(tag.MountInOwner);
                    var det = TagObserver.Detect(OverheadPose, BodyMotion.Still, OverheadK, tagWorld, BodyMotion.Still, tag, occ, Vis, Noise, OverheadLatency, t, rng);
                    if (!det.Detected) continue;
                    Pose worldTag = OverheadPose.Compose(det.CamTag);
                    Matrix cov = PoseUncertainty.RotateInto(OverheadPose.R, det.CovarianceCam);
                    Pose mInv = tag.MountInOwner.Inverse();
                    var pose = worldTag.Compose(mInv);
                    var p2 = Se2.Compose(new Pose2(bias.X, bias.Y, bias.Yaw), Se2.FromPose(pose));
                    var c6 = PoseUncertainty.RigidShift(worldTag, mInv, cov);
                    // survey error (4 mm, 0.2 deg) as covariance
                    c6[0, 0] += 1.6e-5; c6[1, 1] += 1.6e-5; c6[5, 5] += 1.2e-5;
                    var obs = new RelPoseObservation
                    {
                        StampAcquired = t, StampDelivered = det.StampDelivered, SourceId = "overhead", SourceKind = ObservationSourceKind.Overhead, TagId = tag.Id,
                        AmbiguousPose = det.Pnp.Ambiguous, FrameParent = "world", FrameChild = o.Spec.Id + "/base_link", Pose = Se2.ToPose(p2, pose.P.Z), Covariance = c6,
                    };
                    foreach (var r in Rovers) r.TagQ.Add(obs);   // broadcast
                }
            }
        }
    }

    // ===================================================================================================== I/O adapters
    public sealed class HarnessClock : IClock { public double Now { get; set; } }

    public sealed class HarnessSource : IObservationSource, IInertialSource, IPoseFixSource
    {
        private readonly KinematicWorld _w; private readonly KinematicWorld.Rover _r;
        public HarnessSource(KinematicWorld w, KinematicWorld.Rover r) { _w = w; _r = r; }
        public string RoverId => _r.Spec.Id;
        public void ReadEncoders(List<EncoderSample> into) { into.Clear(); into.AddRange(_r.EncQ); _r.EncQ.Clear(); }
        public void ReadGyro(List<GyroSample> into) { into.Clear(); into.AddRange(_r.GyroQ); _r.GyroQ.Clear(); }
        public void ReadTagObservations(List<RelPoseObservation> into)
        {
            into.Clear();
            for (int i = _r.TagQ.Count - 1; i >= 0; i--)
                if (_r.TagQ[i].StampDelivered <= _w.Time + 1e-9) { into.Add(_r.TagQ[i]); _r.TagQ.RemoveAt(i); }
            into.Sort((a, b) => a.StampAcquired != b.StampAcquired ? a.StampAcquired.CompareTo(b.StampAcquired) : a.TagId.CompareTo(b.TagId));
        }
        public ConnectorReport ReadConnector(string face)
        {
            foreach (var p in _w.Pairs)
            {
                bool mine = (p.Docker == _r.Spec.Id && p.DockerFace == face) || (p.Target == _r.Spec.Id && p.TargetFace == face);
                if (!mine) continue;
                var m = p.Machine;
                if (p.Docker != _r.Spec.Id && m.State == ConnectorState.Free) continue;
                return new ConnectorReport
                {
                    Present = true, State = m.State, SensedA = m.SensedA, SensedB = m.SensedB, FailedAttempts = m.FailedAttempts, RetriesExhausted = m.RetriesExhausted,
                    Action = LastAction(p), PartnerRover = p.Docker == _r.Spec.Id ? p.Target : p.Docker, PartnerFace = p.Docker == _r.Spec.Id ? p.TargetFace : p.DockerFace,
                };
            }
            return default;
        }
        private static RequestedAction LastAction(KinematicWorld.Pair p) => p.Machine.RetriesExhausted ? RequestedAction.Abandon : p.LastAction;
        public LiftReport ReadLift()
        {
            var l = new LiftReport { Present = _r.Spec.HasLift, Height = _r.LiftHeight, GripperClosed = _r.GripClosed, HoldingObject = _r.Holding != null, ForceLimited = _r.ForceLimited };
            if (_w.DeckLoadCells && _r.Spec.HasDeck)
            {
                l.DeckLoadValid = true;
                foreach (var p in _w.Payloads) if (p.CarriedBy != null && p.Supporters.Contains(_r.Spec.Id)) l.DeckLoadKg += p.Mass / p.Supporters.Count;
            }
            return l;
        }
        public void ReadAccel(List<AccelSample> into) { into.Clear(); into.AddRange(_r.AccQ); _r.AccQ.Clear(); }
        private double _nextFix;
        public void ReadPoseFixes(List<PoseFix> into)
        {
            into.Clear();
            if (!_w.ScanMatchRovers.Contains(_r.Spec.Id) || _w.Time < _nextFix) return;
            _nextFix = _w.Time + _w.ScanFixPeriod;
            double sg = _w.ScanFixSigma;
            var p = new Pose2(_r.Pose.X + sg * _r.Rng.Gaussian(), _r.Pose.Y + sg * _r.Rng.Gaussian(), _r.Pose.Yaw + 0.5 * sg * _r.Rng.Gaussian());
            into.Add(new PoseFix { Stamp = _w.Time, Pose = p, Cov3 = new[] { sg * sg, 0, 0, 0, sg * sg, 0, 0, 0, 0.25 * sg * sg }, SourceId = "scan_match" });
        }
    }

    public sealed class HarnessSink : IActuatorSink, IEstimatePublisher
    {
        private readonly KinematicWorld.Rover _r;
        public HarnessSink(KinematicWorld.Rover r) { _r = r; }
        public string RoverId => _r.Spec.Id;
        public void SetWheelSpeeds(double fl, double fr, double rl, double rr) { _r.Cmd[0] = fl; _r.Cmd[1] = fr; _r.Cmd[2] = rl; _r.Cmd[3] = rr; }
        public void CommandConnector(string face, DockingCommand command) { if (command != DockingCommand.None) _r.ConnCmd[face] = command; }
        public void SetLift(double heightTarget, bool gripperClosed) { _r.LiftTarget = heightTarget; _r.GripCmd = gripperClosed; }
        public void PublishEstimate(double stamp, Pose2 pose, double[] cov3) { _r.Estimate = pose; Array.Copy(cov3, _r.EstimateCov, 9); }
    }
}
