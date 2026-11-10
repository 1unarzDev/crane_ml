using System;
using System.Collections.Generic;
using System.Text;
using Crane.Fleet.Docking;
using UnityEngine;

namespace Crane.Fleet.Sim {
    /// <summary>
    /// The phase-2 demo mission (overlay "demo"), scripted per rover as a queue of steps that wait on shared flags:
    /// the scout drives behind the shelf and finds the long payload (camera field of view, range and a ray to the payload);
    /// the lifter picks it off its stand; cargo_1 parks in the assembly zone and cargo_2 docks front-to-rear on it, driven by the
    /// docking core's ConnectorMachine fed with the live relative pose of the two faces every physics step; at LATCHED_VERIFIED a
    /// FixedJoint couples the two bodies; the lifter sets the payload on the coupled decks (released to physics) and clears;
    /// the coupled pair carries it through the doorway to the delivery zone while the scout watches.
    /// Controllers steer from evaluator truth (scripted demo; logged in the manifest).
    /// </summary>
    public sealed class DemoMission : IFleetMission {
        // ---------------------------------------------------------------- steps
        abstract class Step {
            public string Name;
            public virtual void Begin(DemoMission m, RoverRig r) { }
            /// <summary>Set r.Command (or leave it); return true when done.</summary>
            public abstract bool Tick(DemoMission m, RoverRig r);
        }

        sealed class GoTo : Step {
            public Func<DemoMission, (double x, double y, double yaw)> Target;
            public double Speed, Tol = double.NaN;   // m/s
            double t0;
            public override void Begin(DemoMission m, RoverRig r) { t0 = m.run.T; }
            public override bool Tick(DemoMission m, RoverRig r) {
                var (x, y, yaw) = Target(m);
                r.Command = GoToController.Command(m.Truth(r), x, y, yaw, m.kp, m.kyaw, Speed, m.wmax, out double dist, out double ey);
                double tol = double.IsNaN(Tol) ? m.posTol : Tol;
                if (dist < tol && Math.Abs(ey) < m.yawTol) { r.Command = new Twist(); return true; }
                if (m.run.T - t0 > m.timeout) { m.Log("step_timeout", r.D.Id, new Dictionary<string, object> { ["step"] = Name, ["dist"] = dist, ["yaw_err"] = ey }); return true; }
                return false;
            }
        }

        sealed class Lambda : Step {
            public Func<DemoMission, RoverRig, bool> F;
            public Action<DemoMission, RoverRig> OnBegin;
            public override void Begin(DemoMission m, RoverRig r) { OnBegin?.Invoke(m, r); }
            public override bool Tick(DemoMission m, RoverRig r) => F(m, r);
        }

        // ---------------------------------------------------------------- state
        FleetRun run;
        FleetSpec spec;
        JsonNode D;
        double kp, kyaw, wmax, posTol, yawTol, timeout, cruise, carry, creep;
        readonly Dictionary<string, Queue<Step>> queues = new Dictionary<string, Queue<Step>>();
        readonly Dictionary<string, Step> current = new Dictionary<string, Step>();
        readonly Dictionary<string, (double x, double y, double yaw)> hold = new Dictionary<string, (double, double, double)>();
        readonly HashSet<string> flags = new HashSet<string>();
        RoverRig scout, lifter, leader, follower;
        string leaderFace, followerFace;
        public Rigidbody Payload;
        BoxCollider payloadCol;
        double[] payloadSize;
        string payloadMode = "static";
        Vector3 carryPos; Quaternion carryRot; double carryLift0;
        double liftTarget, gripTarget, gripMin, gripMax, liftSpeed, gripRate;
        ConnectorMachine machine;
        DockingCommand dockCmd = DockingCommand.None;
        FacePose facePose;
        double prevGap = double.NaN;
        FixedJoint coupling;
        Pose2 placedRel; bool placed;
        double maxSlip;
        double deliveredAt = double.NaN;
        readonly Dictionary<string, object> results = new Dictionary<string, object>();

        public string Variant => "demo";
        public string ControllerInput => "evaluator_truth (scripted demo controllers; docking machine fed from truth face poses)";
        public bool Done { get; private set; }
        public ConnectorMachine Machine => machine;
        public bool Coupled => coupling != null;
        public string PayloadMode => payloadMode;

        public DemoMission(FleetSpec spec) {
            this.spec = spec;
            D = spec.O["demo"];
        }

        public Pose2 StartPose(string id) { double[] s = D["rovers." + id + ".start"].Doubles; return new Pose2(s[0], s[1], s[2]); }
        public string StepOf(string id) => current.TryGetValue(id, out var s) && s != null ? s.Name : (queues.TryGetValue(id, out var q) && q.Count > 0 ? "pending" : "hold");

        Pose2 Truth(RoverRig r) { var t = r.Truth(); return new Pose2(t.X, t.Y, t.Yaw); }
        void Log(string kind, string rover, Dictionary<string, object> f) => run.Event(kind, rover, f);
        void Flag(string f, string rover) { if (flags.Add(f)) Log("flag", rover, new Dictionary<string, object> { ["flag"] = f }); }

        // ---------------------------------------------------------------- build
        public IEnumerable<GameObject> Build(FleetRun r) {
            run = r;
            r.IgnoreContact = (a, b) => Coupled && ((a == leader && b == follower) || (a == follower && b == leader));
            JsonNode c = D["controller"];
            kp = c["kp_pos"].Double; kyaw = c["kp_yaw"].Double; posTol = c["pos_tolerance"].Double; yawTol = c["yaw_tolerance"].Double;
            scout = r.Rover("scout_1"); lifter = r.Rover("lifter_1");
            double vmax = scout.D.Geometry.MaxSpeedXy;
            timeout = c["step_timeout_s"].Double; cruise = c["cruise_fraction"].Double * vmax; carry = c["carry_fraction"].Double * vmax; creep = c["creep_speed"].Double;
            leader = r.Rover(D["cargo.leader"].String); follower = r.Rover(D["cargo.follower"].String);
            leaderFace = D["cargo.leader_face"].String; followerFace = D["cargo.follower_face"].String;
            wmax = c["yaw_fraction"].Double * scout.D.Geometry.MaxYawRate;

            // payload on its stand (profile size and mass; render from the payload manifest when present)
            string type = D["payload.type"].String;
            payloadSize = spec.P["payloads." + type + ".size_lwh"].Doubles;
            double mass = spec.P["payloads." + type + ".mass"].Double;
            double[] pp = D["payload.pose"].Doubles;
            double standH = D["payload.stand_height"].Double;
            double[] standSz = D["payload.stand_size"].Doubles;
            var root = new GameObject("payload_" + type);
            root.transform.SetPositionAndRotation(FleetFrames.World(pp[0], pp[1], standH + spec.O["physics.spawn_lift"].Double), FleetFrames.WorldYaw(pp[2]));
            Payload = root.AddComponent<Rigidbody>();
            Payload.mass = (float)mass;
            Payload.interpolation = RigidbodyInterpolation.None;
            Payload.solverIterations = (int)spec.O["physics.solver_iterations"].Double;
            var colGo = new GameObject("collision_payload");
            colGo.transform.SetParent(root.transform, false);
            colGo.transform.localPosition = FleetFrames.Local(0, 0, payloadSize[2] / 2);
            payloadCol = colGo.AddComponent<BoxCollider>();
            payloadCol.size = FleetFrames.LocalScale(payloadSize[0], payloadSize[1], payloadSize[2]);
            double bf = spec.O["physics.body_friction"].Double;
            payloadCol.sharedMaterial = new PhysicsMaterial("payload") { dynamicFriction = (float)bf, staticFriction = (float)bf };
            payloadCol.providesContacts = true;
            string pm = System.IO.Path.Combine(spec.Directory, spec.O["visual.payload_manifest"].String);
            if (System.IO.File.Exists(pm)) AssetVisual.BuildPayload(AssetManifest.Load(pm), "payload_" + type, root.transform);
            else FleetGeometry.Visual(PrimitiveType.Cube, "render_payload", root.transform, FleetFrames.Local(0, 0, payloadSize[2] / 2), Quaternion.identity,
                FleetFrames.LocalScale(payloadSize[0], payloadSize[1], payloadSize[2]), new Color(0.72f, 0.55f, 0.32f));
            var stand = FleetGeometry.SolidBox("payload_stand", null, FleetFrames.World(pp[0], pp[1], standH / 2), FleetFrames.WorldScale(standSz[0], standSz[1], standH),
                new Color(0.35f, 0.38f, 0.42f), r.World.FloorMaterial);
            stand.transform.rotation = FleetFrames.WorldYaw(pp[2]);

            // the coupling's connector machine (one per coupling), seeded from the run seed
            var profile = DockingProfileBuilder.Build(spec);
            machine = new ConnectorMachine(new ContactModel(profile, new DockingAssumptions()), FleetRng.For(r.Opt.Seed, "coupling").NextU64(),
                $"{leader.D.Id}.{leaderFace}-{follower.D.Id}.{followerFace}");

            // lifter actuators (profile values)
            JsonNode lr = spec.P["roles.lifter"];
            liftSpeed = lr["lift_speed"].Double;
            gripMax = lr["gripper_opening"].Double;
            gripMin = lifter.Visual != null && lifter.Visual.Manifest.GripperOpeningRange != null ? lifter.Visual.Manifest.GripperOpeningRange[0] : 0;
            gripRate = (gripMax - gripMin) / lr["gripper_close_time"].Double;
            lifter.Lift = 0; liftTarget = 0;
            lifter.GripperOpening = gripMax; gripTarget = gripMax;
            return new[] { root, stand };
        }

        // ---------------------------------------------------------------- script
        public void Start(FleetRun r) {
            foreach (var rv in r.Rovers) {
                queues[rv.D.Id] = new Queue<Step>();
                var t = Truth(rv);
                hold[rv.D.Id] = (t.X, t.Y, t.Yaw);
                if (rv.Visual != null) foreach (var f in rv.D.Faces) { rv.Visual.SetFaceHooks(f.Name, 0, 0); rv.Visual.SetLed(f.Name, Color.black, 0); }
            }
            BuildScout(); BuildLifter(); BuildCargo();
            Log("mission_start", "", new Dictionary<string, object> { ["payload"] = D["payload.type"].String, ["coupling"] = machine.Id });
        }

        static (double, double, double) P3(double[] a, double yaw) => (a[0], a[1], yaw);
        static double Bearing(double[] from, double[] to) => Math.Atan2(to[1] - from[1], to[0] - from[0]);

        void Q(RoverRig r, Step s) => queues[r.D.Id].Enqueue(s);
        Step Go(string name, double x, double y, double yaw, double speed) => new GoTo { Name = name, Target = _ => (x, y, yaw), Speed = speed };
        Step WaitFlag(string f) => new Lambda { Name = "wait_" + f, F = (m, r) => { r.Command = m.HoldCommand(r); return m.flags.Contains(f); } };
        Step WaitUntil(double t) => new Lambda { Name = "wait_t" + t.ToString("0"), F = (m, r) => { r.Command = m.HoldCommand(r); return m.run.T >= t; } };
        Step WaitFor(double s) { double end = double.NaN; return new Lambda { Name = "wait_" + s.ToString("0.0") + "s", OnBegin = (m, r) => end = m.run.T + s, F = (m, r) => { r.Command = m.HoldCommand(r); return m.run.T >= end; } }; }
        Step Do(string name, Action<DemoMission, RoverRig> a) => new Lambda { Name = name, F = (m, r) => { a(m, r); return true; } };

        void BuildScout() {
            var s = D["scout"];
            double[] view = s["view"].Doubles, payload = D["payload.pose"].Doubles;
            foreach (var w in s["route"].Items) { var p = w.Doubles; Q(scout, Go("route", p[0], p[1], 0, cruise)); }
            Q(scout, Go("to_view", view[0], view[1], Bearing(view, payload), cruise));
            double tmax = s["find_timeout_s"].Double, range = s["find_max_range"].Double;
            string cam = "sensor_" + s["find_camera"].String;
            double hfov = spec.P["sensors.mono_camera_ov9281.hfov"].Double;
            double t0 = 0;
            Q(scout, new Lambda { Name = "find_payload", OnBegin = (m, r) => t0 = m.run.T, F = (m, r) => {
                r.Command = m.HoldCommand(r);
                if (m.Visible(r, cam, hfov, range, out double dist, out double bearing)) {
                    m.results["find"] = new Dictionary<string, object> { ["t"] = m.run.T, ["range"] = dist, ["bearing_off_axis"] = bearing, ["camera"] = cam };
                    m.Log("payload_found", r.D.Id, new Dictionary<string, object> { ["range"] = dist, ["bearing_off_axis"] = bearing, ["camera"] = cam, ["method"] = "fov+range+raycast (truth geometry)" });
                    m.Flag("payload_found", r.D.Id);
                    return true;
                }
                if (m.run.T - t0 > tmax) {
                    m.Log("payload_not_found", r.D.Id, new Dictionary<string, object> { ["after_s"] = tmax });
                    m.Flag("payload_found", r.D.Id);   // the mission continues (logged): the demo is scripted
                    return true;
                }
                return false;
            } });
            Q(scout, WaitFor(s["linger_s"].Double));
            Q(scout, WaitFlag("lifter_waiting"));
            double[] watch = s["watch"].Doubles, look = s["watch_look"].Doubles;
            double wy = Bearing(watch, look);
            foreach (var w in s["watch_route"].Items) { var p = w.Doubles; Q(scout, Go("to_watch", p[0], p[1], wy, cruise)); }
            Q(scout, Go("to_watch", watch[0], watch[1], wy, cruise));
            Q(scout, Do("at_watch", (m, r) => m.Flag("scout_watching", r.D.Id)));
            // watch: keep the camera on the payload
            Q(scout, new Lambda { Name = "watch", F = (m, r) => {
                var pp = m.Payload.position;
                double yaw = Math.Atan2(pp.z - r.Body.position.z, pp.x - r.Body.position.x);
                r.Command = GoToController.Command(m.Truth(r), watch[0], watch[1], yaw, m.kp, m.kyaw, m.cruise, m.wmax, out _, out _);
                return false;
            } });
        }

        void BuildLifter() {
            var L = D["lifter"];
            Q(lifter, WaitUntil(L["depart_s"].Double));
            foreach (var w in L["route"].Items) { var p = w.Doubles; Q(lifter, Go("route", p[0], p[1], 0, cruise)); }
            double[] wait = L["wait"].Doubles;
            Q(lifter, Go("to_wait", wait[0], wait[1], Math.PI, cruise));
            Q(lifter, Do("waiting", (m, r) => m.Flag("lifter_waiting", r.D.Id)));
            Q(lifter, WaitFlag("payload_found"));
            // grip geometry from the asset: finger x in the base frame (carriage origin + finger origin)
            double fingerX = FingerX();
            double[] pp = D["payload.pose"].Doubles;
            double inset = L["grip_inset"].Double, gap = L["pre_grip_gap"].Double;
            double east = pp[0] + payloadSize[0] / 2;                  // payload long axis along x, lifter faces west (yaw pi)
            double gx = east - inset + fingerX;
            Q(lifter, Do("fork_to_grip_height", (m, r) => m.liftTarget = L["grip_lift"].Double));
            Q(lifter, Go("pre_grip", gx + gap, pp[1], Math.PI, cruise));
            Q(lifter, LiftReached());
            Q(lifter, Go("creep_in", gx, pp[1], Math.PI, creep));
            Q(lifter, Do("close_gripper", (m, r) => m.gripTarget = Math.Max(m.gripMin, m.payloadSize[1])));
            Q(lifter, GripReached());
            Q(lifter, Do("attach", (m, r) => m.Attach()));
            Q(lifter, Do("lift", (m, r) => m.liftTarget = L["carry_lift"].Double));
            Q(lifter, LiftReached());
            Q(lifter, Do("lifted", (m, r) => { m.Log("payload_lifted", r.D.Id, new Dictionary<string, object> { ["lift"] = r.Lift }); m.Flag("payload_lifted", r.D.Id); }));
            Q(lifter, Go("back_out", gx + gap, pp[1], Math.PI, carry));
            foreach (var w in L["carry_route"].Items) { var p = w.Doubles; Q(lifter, Go("carry", p[0], p[1], Math.PI, carry)); }
            Q(lifter, WaitFlag("coupled"));
            double clear = L["place_clearance"].Double, pre = L["place_pre_gap"].Double;
            // place pose: lifter front face clear of the leader's front face; payload over both decks
            Func<DemoMission, (double, double, double)> place = m => {
                var lp = m.Truth(m.leader);
                var front = m.leader.D.Face("front").Pose;
                double fx = lp.X + Math.Cos(lp.Yaw) * front.X, fy = lp.Y + Math.Sin(lp.Yaw) * front.X;
                double lx = fx + (m.lifter.D.Face("front").Pose.X + clear) * Math.Cos(lp.Yaw), ly = fy + (m.lifter.D.Face("front").Pose.X + clear) * Math.Sin(lp.Yaw);
                return (lx, ly, Angles.Wrap(lp.Yaw + Math.PI));
            };
            Q(lifter, new GoTo { Name = "pre_place", Target = m => { var (x, y, yaw) = place(m); return (x - Math.Cos(yaw) * pre, y - Math.Sin(yaw) * pre, yaw); }, Speed = carry });
            Q(lifter, new GoTo { Name = "place", Target = place, Speed = creep });
            Q(lifter, Do("open_gripper", (m, r) => { m.Release(); m.gripTarget = m.gripMax; }));
            Q(lifter, GripReached());
            Q(lifter, WaitFor(L["settle_s"].Double));
            Q(lifter, Do("placed", (m, r) => m.CheckPlaced()));
            Q(lifter, new GoTo { Name = "back_off", Target = m => { var (x, y, yaw) = place(m); return (x - Math.Cos(yaw) * pre, y - Math.Sin(yaw) * pre, yaw); }, Speed = creep * 3 });
            double[] cl = L["clear"].Doubles;
            Q(lifter, Do("fork_down", (m, r) => m.liftTarget = 0));
            Q(lifter, Go("clear", cl[0], cl[1], Math.PI, cruise));
            Q(lifter, Do("clear", (m, r) => m.Flag("lifter_clear", r.D.Id)));
        }

        double FingerX() {
            var m = lifter.Visual?.Manifest;
            if (m == null) return lifter.D.Faces[0].Pose.X + 0.087;   // primitive fallback only
            var carriage = m.FindPart(m.LiftPart);
            var finger = m.FindPart(m.Fingers[0].part);
            return carriage.Origin.Xyz[0] + finger.Origin.Xyz[0];
        }

        Step LiftReached() => new Lambda { Name = "lift_moving", F = (m, r) => { r.Command = m.HoldCommand(r); return Math.Abs(r.Lift - m.liftTarget) < 1e-6; } };
        Step GripReached() => new Lambda { Name = "gripper_moving", F = (m, r) => { r.Command = m.HoldCommand(r); return Math.Abs(r.GripperOpening - m.gripTarget) < 1e-6; } };

        void BuildCargo() {
            var C = D["cargo"];
            double[] asm = C["assembly"].Doubles;
            Q(leader, WaitUntil(C["depart_s"].Double));
            foreach (var w in C["route"].Items) { var p = w.Doubles; Q(leader, Go("route", p[0], p[1], 0, cruise)); }
            Q(leader, Do("leader_on_lane", (m, r) => m.Flag("leader_on_lane", r.D.Id)));
            Q(leader, Go("to_assembly", asm[0], asm[1], asm[2], cruise));
            Q(leader, Do("parked", (m, r) => m.Flag("leader_parked", r.D.Id)));
            Q(leader, WaitFlag("coupled"));
            Q(leader, WaitFlag("payload_placed"));
            Q(leader, WaitFlag("lifter_clear"));
            Q(leader, WaitFlag("scout_watching"));
            foreach (var w in C["pair_route"].Items) { var p = w.Doubles; Q(leader, Pair("pair_route", p[0], p[1], 0)); }
            double[] del = C["delivery_leader"].Doubles;
            Q(leader, Pair("to_delivery", del[0], del[1], del[2]));
            Q(leader, Do("arrived", (m, r) => m.Flag("pair_arrived", r.D.Id)));

            Q(follower, WaitFlag("leader_on_lane"));
            foreach (var w in C["route"].Items) { var p = w.Doubles; Q(follower, Go("route", p[0], p[1], 0, cruise)); }
            Q(follower, WaitFlag("leader_parked"));
            double pre = C["dock_pre_gap"].Double;
            Q(follower, new GoTo { Name = "pre_dock", Target = m => m.DockTarget(-pre), Speed = cruise, Tol = posTol });
            Q(follower, Dock(C));
            Q(follower, new Lambda { Name = "coupled_slave", F = (m, r) => false });   // the leader commands both bodies from now on
        }

        /// <summary>Follower pose that mates its face with the leader's face, shifted by <paramref name="along"/> along its heading.</summary>
        (double, double, double) DockTarget(double along) {
            var lp = Truth(leader);
            Pose2 t = CadSwapCheck.MatingPose(lp, leader.D.Face(leaderFace).Pose, follower.D.Face(followerFace).Pose);
            return (t.X + Math.Cos(t.Yaw) * along, t.Y + Math.Sin(t.Yaw) * along, t.Yaw);
        }

        Step Dock(JsonNode C) {
            double v = C["dock_speed"].Double, vf = C["dock_final_speed"].Double, contact = C["dock_contact_gap"].Double;
            double capturedAt = double.NaN;
            bool retreating = false;
            return new Lambda { Name = "dock", OnBegin = (m, r) => m.dockCmd = DockingCommand.Approach, F = (m, r) => {
                var st = m.machine.State;
                bool captured = st == ConnectorState.MagnetCaptured || st == ConnectorState.HookAEngaged || st == ConnectorState.HookBEngaged || st == ConnectorState.Partial;
                if (st == ConnectorState.Approach || captured || st == ConnectorState.LatchedVerified) {
                    // creep onto the mating pose (2 mm past it, so the faces press together and the leader holds)
                    var (x, y, yaw) = m.DockTarget(0.002);
                    r.Command = GoToController.Command(m.Truth(r), x, y, yaw, m.kp, m.kyaw, captured ? vf : v, m.wmax, out _, out _);
                }
                else r.Command = m.HoldCommand(r);
                if (captured && double.IsNaN(capturedAt)) capturedAt = m.run.T;
                if (st == ConnectorState.MagnetCaptured && m.dockCmd != DockingCommand.EngageHooks && (m.facePose.Gap <= contact || m.run.T - capturedAt > 1.5)) {
                    m.dockCmd = DockingCommand.EngageHooks;
                    m.Log("hooks_closing", r.D.Id, new Dictionary<string, object> { ["gap"] = m.facePose.Gap, ["lateral"] = m.facePose.Lateral, ["yaw"] = m.facePose.Yaw });
                }
                if ((st == ConnectorState.HookAEngaged || st == ConnectorState.HookBEngaged) && m.machine.SensedA && m.machine.SensedB) m.dockCmd = DockingCommand.Verify;
                if (st == ConnectorState.LatchedVerified && m.coupling == null) { m.Couple(); return true; }
                // failed attempt: release (if needed), back off to the pre-dock pose, approach again (machine counts retries)
                if (st == ConnectorState.Partial || st == ConnectorState.FalseEngagement) m.dockCmd = DockingCommand.Release;
                if ((st == ConnectorState.Free || st == ConnectorState.Released) && m.run.T > 0) {
                    if (m.machine.RetriesExhausted) { m.Log("dock_abandoned", r.D.Id, new Dictionary<string, object> { ["attempts"] = m.machine.FailedAttempts }); return true; }
                    if (!retreating && m.machine.FailedAttempts > 0) { retreating = true; m.dockCmd = DockingCommand.None; m.Log("dock_retry", r.D.Id, new Dictionary<string, object> { ["attempt"] = m.machine.FailedAttempts }); }
                    if (retreating) {
                        var (x, y, yaw) = m.DockTarget(-C["dock_pre_gap"].Double);
                        r.Command = GoToController.Command(m.Truth(r), x, y, yaw, m.kp, m.kyaw, v * 2, m.wmax, out double d, out _);
                        if (d < m.posTol) { retreating = false; m.dockCmd = DockingCommand.Approach; capturedAt = double.NaN; }
                    }
                }
                return false;
            } };
        }

        Step Pair(string name, double x, double y, double yaw) {
            return new Lambda { Name = name, F = (m, r) => {
                var c = GoToController.Command(m.Truth(r), x, y, yaw, m.kp, m.kyaw, m.carry, m.wmax * 0.5, out double dist, out double ey);
                bool done = dist < m.posTol && Math.Abs(ey) < m.yawTol;
                if (done) c = new Twist();
                r.Command = c;
                return done;
            } };
        }

        Twist HoldCommand(RoverRig r) {
            var (x, y, yaw) = hold[r.D.Id];
            return GoToController.Command(Truth(r), x, y, yaw, kp, kyaw, cruise, wmax, out _, out _);
        }

        // ---------------------------------------------------------------- per step
        public void PreStep(FleetRun r) {
            // connector machine from the live relative pose of the two faces
            facePose = FaceRelative(leader.Frames["face_" + leaderFace], follower.Frames["face_" + followerFace]);
            double closing = double.IsNaN(prevGap) ? 0 : (prevGap - facePose.Gap) / r.Dt;
            prevGap = facePose.Gap;
            var res = machine.Step(r.Dt, new ConnectorInputs { Pose = facePose, ClosingSpeed = closing, Command = dockCmd, ReleaseAuthorized = false });
            foreach (var e in res.Events)
                Log("connector", follower.D.Id, new Dictionary<string, object> { ["coupling"] = machine.Id, ["event"] = e.Kind.ToString(), ["from"] = ConnectorStateNames.ToProfileName(e.From),
                    ["to"] = ConnectorStateNames.ToProfileName(e.To), ["trigger"] = e.Trigger?.ToString(), ["detail"] = e.Detail, ["gap"] = facePose.Gap, ["lateral"] = facePose.Lateral, ["yaw"] = facePose.Yaw });
            if (res.Action != RequestedAction.None) Log("connector_action", follower.D.Id, new Dictionary<string, object> { ["action"] = res.Action.ToString() });

            foreach (var rv in r.Rovers) {
                string id = rv.D.Id;
                if (!current.TryGetValue(id, out var s) || s == null) {
                    if (queues[id].Count > 0) {
                        s = current[id] = queues[id].Dequeue();
                        Log("step_start", id, new Dictionary<string, object> { ["step"] = s.Name });
                        s.Begin(this, rv);
                    }
                }
                if (s != null) {
                    if (s.Tick(this, rv)) {
                        Log("step_end", id, new Dictionary<string, object> { ["step"] = s.Name });
                        current[id] = null;
                        var t = Truth(rv);
                        hold[id] = (t.X, t.Y, t.Yaw);
                    }
                }
                else if (!(Coupled && rv == follower)) rv.Command = HoldCommand(rv);
            }
            if (Coupled) {
                // the coupled follower moves rigidly with the leader: v_f = v_l + w x r_f (shared body frame; same heading)
                Twist c = leader.Command;
                var lp = Truth(leader); var fp = Truth(follower);
                double dx = fp.X - lp.X, dy = fp.Y - lp.Y, cs = Math.Cos(lp.Yaw), sn = Math.Sin(lp.Yaw);
                double rx = cs * dx + sn * dy, ry = -sn * dx + cs * dy;
                follower.Command = new Twist(c.Vx - c.Wz * ry, c.Vy + c.Wz * rx, c.Wz);
            }

            // lifter actuators (profile speeds) and the carried payload
            lifter.Lift = Toward(lifter.Lift, liftTarget, liftSpeed * r.Dt);
            lifter.GripperOpening = Toward(lifter.GripperOpening, gripTarget, gripRate * r.Dt);
            if (payloadMode == "carried") {
                Vector3 pos = lifter.Body.position + lifter.Body.rotation * carryPos + Vector3.up * (float)(lifter.Lift - carryLift0);
                Payload.MovePosition(pos);
                Payload.MoveRotation(lifter.Body.rotation * carryRot);
            }
        }

        static double Toward(double v, double target, double step) => v < target ? Math.Min(target, v + step) : Math.Max(target, v - step);

        public void PostStep(FleetRun r) {
            // hooks and LEDs from the machine (both mated faces show the coupling's hook positions)
            Color led = LedColor(machine.State);
            float nits = (float)spec.O["visual.led_nits"].Double;
            foreach (var (rv, face) in new[] { (leader, leaderFace), (follower, followerFace) }) {
                if (rv.Visual == null) continue;
                rv.Visual.SetFaceHooks(face, machine.HookPosA, machine.HookPosB);
                rv.Visual.SetLed(face, led, nits);
            }
            if (placed) {
                Pose2 rel = Relative(Truth(leader), PayloadPlanar());
                maxSlip = Math.Max(maxSlip, Math.Sqrt(Sq(rel.X - placedRel.X) + Sq(rel.Y - placedRel.Y)));
            }
            if (flags.Contains("pair_arrived") && double.IsNaN(deliveredAt)) {
                var p = PayloadPlanar();
                double[] z = spec.S["zones.delivery.rect"].Doubles;
                bool inside = p.X >= z[0] && p.X <= z[2] && p.Y >= z[1] && p.Y <= z[3];
                deliveredAt = r.T;
                results["delivered"] = new Dictionary<string, object> { ["t"] = r.T, ["payload_in_delivery_zone"] = inside, ["payload_x"] = p.X, ["payload_y"] = p.Y,
                    ["max_slip_on_decks"] = maxSlip, ["payload_height"] = Payload.position.y };
                Log(inside ? "delivered" : "delivery_missed", leader.D.Id, (Dictionary<string, object>)results["delivered"]);
            }
            if (!double.IsNaN(deliveredAt) && r.T > deliveredAt + 2.0) Done = true;
        }

        static double Sq(double v) => v * v;

        static Color LedColor(ConnectorState s) {
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

        /// <summary>Face B relative to face A (A's frame: +x outward normal): gap along A's normal, lateral, height, yaw error from mating.</summary>
        static FacePose FaceRelative(Transform a, Transform b) {
            Vector3 l = Quaternion.Inverse(a.rotation) * (b.position - a.position);
            FleetFrames.Attitude(a.rotation, out double ya, out _, out _);
            FleetFrames.Attitude(b.rotation, out double yb, out _, out _);
            return new FacePose(-l.x, l.z, Angles.Wrap(yb - ya - Math.PI), l.y);
        }

        bool Visible(RoverRig r, string frame, double hfov, double maxRange, out double dist, out double offAxis) {
            dist = double.NaN; offAxis = double.NaN;
            if (!r.Frames.TryGetValue(frame, out var cam)) return false;
            Vector3 target = payloadCol.bounds.center, from = cam.position, d = target - from;
            dist = d.magnitude;
            offAxis = Vector3.Angle(cam.forward, d) * Math.PI / 180.0;
            if (dist > maxRange || offAxis > hfov / 2) return false;
            return run.Phys.Raycast(from, d.normalized, out RaycastHit hit, (float)dist + 0.1f) && hit.collider == payloadCol;
        }

        void Attach() {
            payloadMode = "carried";
            Payload.isKinematic = true;
            carryPos = Quaternion.Inverse(lifter.Body.rotation) * (Payload.position - lifter.Body.position);
            carryRot = Quaternion.Inverse(lifter.Body.rotation) * Payload.rotation;
            carryLift0 = lifter.Lift;
            // static check the mission does not model: payload mass vs lift capacity, and the combined CoM against the front axle
            double cap = spec.P["roles.lifter.lift_capacity"].Double, pm = Payload.mass;
            double payloadX = carryPos.z, lm = lifter.D.Mass, axle = lifter.D.Wheels[0].Axle.X;
            double comX = (pm * payloadX + lm * lifter.D.ComOffset[0]) / (pm + lm);
            var f = new Dictionary<string, object> { ["payload_kg"] = pm, ["capacity_kg"] = cap, ["over_capacity"] = pm > cap,
                ["payload_x_in_lifter"] = payloadX, ["combined_com_x"] = comX, ["front_axle_x"] = axle, ["would_tip_forward"] = comX > axle,
                ["note"] = "carry is kinematic: lifter load and tipping are not simulated" };
            results["lift_static_check"] = f;
            Log("payload_gripped", lifter.D.Id, f);
        }

        void Release() {
            payloadMode = "dynamic";
            Payload.isKinematic = false;
            Payload.linearVelocity = Vector3.zero;
            Payload.angularVelocity = Vector3.zero;
            Log("payload_released", lifter.D.Id, new Dictionary<string, object> { ["height_bottom"] = Payload.position.y });
        }

        void Couple() {
            coupling = follower.Root.AddComponent<FixedJoint>();
            coupling.connectedBody = leader.Body;
            coupling.enableCollision = false;
            var f = new Dictionary<string, object> { ["coupling"] = machine.Id, ["joint"] = "FixedJoint (follower -> leader), created at the current relative pose",
                ["gap"] = facePose.Gap, ["lateral"] = facePose.Lateral, ["yaw"] = facePose.Yaw, ["height"] = facePose.Height,
                ["settled_lateral"] = machine.Settled.Lateral, ["settled_yaw"] = machine.Settled.Yaw };
            results["coupling"] = f;
            Log("coupled", follower.D.Id, f);
            Flag("coupled", follower.D.Id);
        }

        Pose2 PayloadPlanar() => FleetFrames.Planar(Payload.transform);

        static Pose2 Relative(Pose2 a, Pose2 b) {
            double c = Math.Cos(a.Yaw), s = Math.Sin(a.Yaw), dx = b.X - a.X, dy = b.Y - a.Y;
            return new Pose2(c * dx + s * dy, -s * dx + c * dy, Angles.Wrap(b.Yaw - a.Yaw));
        }

        void CheckPlaced() {
            placed = true;
            placedRel = Relative(Truth(leader), PayloadPlanar());
            // support: penetration against every collider of both decks, payload tilt and bottom height
            double maxPen = 0; int touching = 0;
            foreach (var rv in new[] { leader, follower })
                foreach (var col in rv.Root.GetComponentsInChildren<BoxCollider>()) {
                    if (Physics.ComputePenetration(payloadCol, payloadCol.transform.position, payloadCol.transform.rotation, col, col.transform.position, col.transform.rotation, out _, out float depth)) {
                        maxPen = Math.Max(maxPen, depth); touching++;
                    }
                }
            FleetFrames.Attitude(Payload.rotation, out _, out double pitch, out double roll);
            var f = new Dictionary<string, object> { ["rel_leader_x"] = placedRel.X, ["rel_leader_y"] = placedRel.Y, ["rel_leader_yaw"] = placedRel.Yaw,
                ["bottom_height"] = (double)Payload.position.y, ["pitch"] = pitch, ["roll"] = roll, ["max_penetration"] = maxPen, ["overlapping_colliders"] = touching,
                ["speed"] = (double)Payload.linearVelocity.magnitude };
            results["placed"] = f;
            Log("payload_placed", lifter.D.Id, f);
            Flag("payload_placed", lifter.D.Id);
        }

        // ---------------------------------------------------------------- logs
        public void LogState(StringBuilder b) {
            string N(double v) => FleetJson.Num(v);
            b.Append("\"connector\":{\"state\":").Append(FleetJson.Quote(ConnectorStateNames.ToProfileName(machine.State)))
                .Append(",\"hook_a\":").Append(N(machine.HookPosA)).Append(",\"hook_b\":").Append(N(machine.HookPosB))
                .Append(",\"sensed\":[").Append(machine.SensedA ? "true" : "false").Append(',').Append(machine.SensedB ? "true" : "false").Append(']')
                .Append(",\"gap\":").Append(N(facePose.Gap)).Append(",\"lateral\":").Append(N(facePose.Lateral)).Append(",\"yaw\":").Append(N(facePose.Yaw))
                .Append(",\"height\":").Append(N(facePose.Height)).Append(",\"joint\":").Append(coupling != null ? "true" : "false").Append('}');
            b.Append(",\"lifter\":{\"lift\":").Append(N(lifter.Lift)).Append(",\"gripper_opening\":").Append(N(lifter.GripperOpening)).Append('}');
            Vector3 p = Payload.position;
            FleetFrames.ToEnu(p, out double x, out double y, out double z);
            FleetFrames.Attitude(Payload.rotation, out double yaw, out double pitch, out double roll);
            b.Append(",\"payload\":{\"mode\":").Append(FleetJson.Quote(payloadMode)).Append(",\"x\":").Append(N(x)).Append(",\"y\":").Append(N(y)).Append(",\"z\":").Append(N(z))
                .Append(",\"yaw\":").Append(N(yaw)).Append(",\"pitch\":").Append(N(pitch)).Append(",\"roll\":").Append(N(roll)).Append('}');
        }

        public void Finish(FleetRun r, Dictionary<string, object> res) {
            results["connector_final_state"] = ConnectorStateNames.ToProfileName(machine.State);
            results["connector_failed_attempts"] = machine.FailedAttempts;
            results["flags"] = new List<string>(flags);
            results["max_slip_on_decks"] = maxSlip;
            res["demo"] = results;
        }
    }
}
