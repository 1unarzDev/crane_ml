using System;
using System.Collections.Generic;
using System.Text;
using Crane.Fleet.Docking;
using Crane.Fleet.Perception;
using UnityEngine;
using PPose = Crane.Fleet.Perception.Pose;

namespace Crane.Fleet.Sim {
    /// <summary>
    /// The mission run CLOSED LOOP by the autonomy library (crane_ml Tools/Fleet/core_cs/Crane.Fleet.Autonomy, imported as assembly
    /// Crane.Fleet.Autonomy): CRANE provides one IObservationSource and one IActuatorSink per rover and calls FleetAutonomy.Step(dt) every
    /// physics step. Sources deliver only what the rover could measure: its own quantised encoders and raw gyro (from the plant), tag
    /// observations from the perception core (rover tags, plus payload tags generated here with the core's TagObserver), connector
    /// reports from the simulator-side ConnectorMachine, lift/gripper driver state. Sinks take wheel-speed set-points straight into the
    /// plant (no truth-steered controller), connector pulses, lift and gripper targets. No truth reaches the library.
    /// Simulator conventions the library relies on: the payload's base_link is its tagged end-face centre (so the library's fixed grasp
    /// offsets work for a long box); the payload sits on a 3 cm pallet so a gripper at lift 0 holds it and full lift clears the decks.
    /// </summary>
    public sealed class AutonomyMission : IFleetMission {
        readonly FleetSpec spec;
        readonly JsonNode missionJson;
        readonly string missionText;
        FleetRun run;
        CouplingManager couplings;
        FleetPerceptionHost perception;
        FleetAutonomy fleet;
        readonly Clock clock = new Clock();
        readonly Dictionary<string, Source> sources = new Dictionary<string, Source>();
        readonly Dictionary<string, Sink> sinks = new Dictionary<string, Sink>();
        // payload (one object for now, from the mission's "crane_payload" block)
        string payloadId, payloadType;
        double[] payloadSize;
        public Rigidbody Payload;
        BoxCollider payloadCol;
        TagSpec payloadTag;
        string payloadMode = "static";
        RoverRig carrier;
        Vector3 carryPos; Quaternion carryRot; double carryLift0;
        // lifter load model (overlay lift_load): lift force limit, payload weight + inertial reaction on the lifter body, tip check
        string loadModel; double liftForceMax, liftOffHeight, tipLimit;
        bool liftStalled, lifterTipped, loadApplied, loadEverApplied, forceLimitedNow; double maxCarrierTilt;
        string payloadMassSource = "profile";
        double liftSpeed, gripRate, gripMax, gripMin;
        int autonomyEventIndex;
        double doneAt = double.NaN;
        readonly Dictionary<string, object> results = new Dictionary<string, object>();
        readonly Dictionary<string, TaskState> lastState = new Dictionary<string, TaskState>();
        readonly List<object> beliefVsTruth = new List<object>();
        int disagreements;
        bool hostVerdictGiven;
        double believedAt = double.NaN;

        sealed class Clock : IClock { public double Now { get; set; } }

        public AutonomyMission(FleetSpec spec, string missionPath) {
            this.spec = spec;
            missionText = System.IO.File.ReadAllText(missionPath);
            missionJson = JsonNode.FromText(missionText);
        }

        public string Variant => "autonomy";
        public string ControllerInput => "estimate (Crane.Fleet.Autonomy: own encoders, gyro, perception-core tag observations; wheel-speed commands into the plant)";
        public bool Done { get; private set; }
        public IDictionary<string, object> Labels => new Dictionary<string, object> {
            ["mission_id"] = missionJson.Has("mission_id") ? missionJson["mission_id"].String : "", ["mission_sha256"] = Sha(missionText),
            ["autonomy"] = "Crane.Fleet.Autonomy (closed loop)", ["payload_class"] = payloadType ?? "none",
            ["payload_kg"] = Payload != null ? (object)(double)Payload.mass : null, ["lift_load_model"] = spec.O["lift_load.model"].String,
            ["interface_pair_types"] = "hermaphroditic_hook2_magnet+hermaphroditic_hook2_magnet" };
        public string PayloadMode => payloadMode;
        public FleetAutonomy Fleet => fleet;
        public CouplingManager Couplings => couplings;

        static string Sha(string s) {
            using var h = System.Security.Cryptography.SHA256.Create();
            var b = h.ComputeHash(Encoding.UTF8.GetBytes(s)); var sb = new StringBuilder();
            foreach (byte x in b) sb.Append(x.ToString("x2"));
            return sb.ToString();
        }

        /// <summary>Landmark tags: the scenario's scenes-v1 tags (x_v1.tags: door jambs, bays, slots) when it has them, else the
        /// overlay's warehouse_a door-jamb tags. (scenes v1; untested in Unity at the time of writing.)</summary>
        IEnumerable<(string name, double[] pose)> LandmarkTags() {
            var v1 = spec.V1();
            if (v1.Present && v1.Tags.Count > 0) {
                foreach (var t in v1.Landmarks()) yield return (t.Name, new[] { t.X, t.Y, t.Yaw });
                yield break;
            }
            foreach (var lm in spec.O["landmarks.tags"].Items) yield return (lm["name"].String, lm["pose"].Doubles);
        }

        public Pose2 StartPose(string id) {
            // the start slot the rover believes in (the mission's initial_estimates, else the demo start slots)
            JsonNode ie = missionJson.TryGet("crane_start." + id) ?? spec.O["demo.rovers." + id + ".start"];
            double[] s = ie.Doubles;
            return new Pose2(s[0], s[1], s[2]);
        }

        public string StepOf(string id) {
            if (fleet == null) return "init";
            foreach (var r in fleet.Executive.Records)
                if (r.State == TaskState.Running && r.Behavior != null)
                    foreach (var rv in r.Behavior.Rovers()) if (fleet.AgentOf(rv) != null && fleet.AgentOf(rv).Has(id) || rv == id) return r.Spec.Id + ":" + r.Behavior.Phase;
            return "idle";
        }

        // ------------------------------------------------------------------ build
        public IEnumerable<GameObject> Build(FleetRun r) {
            run = r;
            couplings = new CouplingManager(r, spec);
            couplings.OnLatched = c => results["coupling_" + c.Id] = new Dictionary<string, object> { ["t"] = r.T, ["lateral"] = c.Pose.Lateral, ["yaw"] = c.Pose.Yaw, ["gap"] = c.Pose.Gap };
            perception = new FleetPerceptionHost(spec, r.Rovers, r.Opt.Seed);
            var roots = new List<GameObject>();
            JsonNode p = missionJson.TryGet("crane_payload");
            if (p != null) {
                payloadId = p["id"].String; payloadType = p["type"].String;
                payloadSize = spec.P["payloads." + payloadType + ".size_lwh"].Doubles;
                double[] pose = p["pose"].Doubles;   // centre of the payload footprint, yaw
                double pallet = p["pallet_height"].Double;
                var root = new GameObject(payloadId);
                root.transform.SetPositionAndRotation(FleetFrames.World(pose[0], pose[1], pallet + spec.O["physics.spawn_lift"].Double), FleetFrames.WorldYaw(pose[2]));
                Payload = root.AddComponent<Rigidbody>();
                Payload.mass = (float)(p.Has("mass_kg") ? p["mass_kg"].Double : spec.P["payloads." + payloadType + ".mass"].Double);
                payloadMassSource = p.Has("mass_kg") ? "mission crane_payload.mass_kg (CRANE variant)" : "profile payloads." + payloadType + ".mass";
                Payload.solverIterations = (int)spec.O["physics.solver_iterations"].Double;
                var cg = new GameObject("collision_payload"); cg.transform.SetParent(root.transform, false);
                cg.transform.localPosition = FleetFrames.Local(0, 0, payloadSize[2] / 2);
                payloadCol = cg.AddComponent<BoxCollider>();
                payloadCol.size = FleetFrames.LocalScale(payloadSize[0], payloadSize[1], payloadSize[2]);
                double bf = spec.O["physics.body_friction"].Double;
                payloadCol.sharedMaterial = new PhysicsMaterial("payload") { dynamicFriction = (float)bf, staticFriction = (float)bf };
                payloadCol.providesContacts = true;
                string pm = System.IO.Path.Combine(spec.Directory, spec.O["visual.payload_manifest"].String);
                if (System.IO.File.Exists(pm)) AssetVisual.BuildPayload(AssetManifest.Load(pm), "payload_" + payloadType, root.transform);
                roots.Add(root);
                var pal = FleetGeometry.SolidBox("pallet", null, FleetFrames.World(pose[0], pose[1], pallet / 2), FleetFrames.WorldScale(payloadSize[0] * 0.9, payloadSize[1] * 0.9, pallet),
                    new Color(0.45f, 0.30f, 0.18f), r.World.FloorMaterial);
                pal.transform.rotation = Quaternion.Euler(0f, (float)(-pose[2] * 180.0 / Math.PI), 0f);   // world-axis box: pure yaw (WorldYaw is for rover frames)
                roots.Add(pal);
                // the payload's tag on its -x end face (autonomy harness convention), base_link = that face's centre
                double tagSize = spec.P["sensors.apriltag.top_tag_size"].Double;
                double tagZ = Math.Min(payloadSize[2] - tagSize / 2, payloadSize[2] / 2) + pallet;
                payloadTag = new TagSpec { Id = 900, OwnerId = payloadId, Name = "payload_end", Size = Math.Min(tagSize, payloadSize[2] * 0.9),
                    MountInOwner = TagSpec.OnVerticalFace(new Vec3(-0.002, 0, tagZ), Math.PI) };
            }
            JsonNode lr = spec.P["roles.lifter"];
            loadModel = spec.O["lift_load.model"].String;
            liftForceMax = lr["lift_capacity"].Double * spec.Gravity * spec.O["lift_load.force_margin"].Double;
            liftOffHeight = spec.O["lift_load.lift_off_height"].Double; tipLimit = spec.O["lift_load.tip_limit_rad"].Double;
            liftSpeed = lr["lift_speed"].Double; gripMax = lr["gripper_opening"].Double;
            foreach (var rv in r.Rovers) if (rv.D.Role == "lifter") { rv.Lift = 0; rv.GripperOpening = gripMax; gripMin = rv.Visual?.Manifest.GripperOpeningRange?[0] ?? 0.04; }
            gripRate = (gripMax - gripMin) / lr["gripper_close_time"].Double;
            // render-only landmark plates (white with a black frame; not a decodable tag image)
            double ls = spec.O["landmarks.size"].Double, lh = spec.O["landmarks.height"].Double;
            foreach (var (lmName, lp) in LandmarkTags()) {
                var go = new GameObject("landmark_" + lmName);
                go.transform.SetPositionAndRotation(FleetFrames.World(lp[0], lp[1], 0), FleetFrames.WorldYaw(lp[2]));
                FleetGeometry.Visual(PrimitiveType.Cube, "frame", go.transform, FleetFrames.Local(0.002, 0, lh), Quaternion.identity, FleetFrames.LocalScale(0.004, ls * 1.25, ls * 1.25), Color.black);
                FleetGeometry.Visual(PrimitiveType.Cube, "face", go.transform, FleetFrames.Local(0.004, 0, lh), Quaternion.identity, FleetFrames.LocalScale(0.002, ls * 0.75, ls * 0.75), Color.white);
                roots.Add(go);
            }
            return roots;
        }

        /// <summary>The payload's base_link (tagged end-face centre) in the world, planar.</summary>
        Pose2 PayloadBase() {
            var c = FleetFrames.Planar(Payload.transform);
            return new Pose2(c.X - Math.Cos(c.Yaw) * payloadSize[0] / 2, c.Y - Math.Sin(c.Yaw) * payloadSize[0] / 2, c.Yaw);
        }

        public void Start(FleetRun r) {
            perception.Record(r.T, r.Rovers);
            var cfg = new FleetConfig {
                Map = BuildMap(),
                DockingProfile = DockingProfileBuilder.Build(spec)
            };
            var profile = FleetProfile.Parse(System.IO.File.ReadAllText(spec.Profile.Source));
            string floor = spec.S["arena.floor"].String;
            var srcList = new List<IObservationSource>(); var sinkList = new List<IActuatorSink>();
            foreach (var rv in r.Rovers) {
                cfg.Rovers.Add(RoverSpec.FromProfile(profile, rv.D.Id, rv.D.Role, floor));
                cfg.InitialPoses[rv.D.Id] = StartPose(rv.D.Id);   // believed start slot, never the per-seed truth
                var s = sources[rv.D.Id] = new Source(this, rv); var k = sinks[rv.D.Id] = new Sink(this, rv);
                srcList.Add(s); sinkList.Add(k);
                rv.SampleSink = s;
                if (rv.Visual != null) foreach (var f in rv.D.Faces) { rv.Visual.SetFaceHooks(f.Name, 0, 0); rv.Visual.SetLed(f.Name, Color.black, 0); }
            }
            // landmark tags (door jambs): known poses for the library, detections through the perception host's object channel
            int lmId = 950;
            foreach (var (lmName, p) in LandmarkTags()) {
                var pose = new Pose2(p[0], p[1], p[2]);
                cfg.Landmarks[lmName] = pose;
                perception.RecordObject(r.T, lmName, pose, new TagSpec { Id = lmId++, OwnerId = lmName, Name = "landmark",
                    Size = spec.O["landmarks.size"].Double, MountInOwner = TagSpec.OnVerticalFace(new Vec3(0.002, 0, spec.O["landmarks.height"].Double), 0) });
            }
            if (missionJson.Has("crane_initial_sigma")) { double[] sg = missionJson["crane_initial_sigma"].Doubles; cfg.InitialPosSigma = sg[0]; cfg.InitialYawSigma = sg[1]; }
            clock.Now = r.T;
            fleet = new FleetAutonomy(cfg, clock, srcList, sinkList);
            fleet.LoadMission(MissionSpec.Parse(missionText));
            run.Event("autonomy_start", "", new Dictionary<string, object> { ["mission"] = Labels["mission_id"], ["rovers"] = r.Rovers.Count, ["map_boxes"] = cfg.Map.Obstacles.Count });
        }

        PlanMap BuildMap() {
            double[] size = spec.S["arena.size_xy"].Doubles;
            var map = new PlanMap(0, 0, size[0], size[1]);
            foreach (var o in spec.S["obstacles"].Items) {
                double[] c = o["centre"].Doubles, sz = o["size"].Doubles;
                if (c[2] - sz[2] / 2 > 0.3) continue;   // elevated (gantry beam): does not block driving
                map.AddBox(o["id"].String, c[0], c[1], sz[0], sz[1]);
            }
            JsonNode door = spec.S.TryGet("doorway");
            foreach (var w in spec.S["walls"].Items) {
                double[] a = w["from"].Doubles;
                if (door != null && door["in_wall"].String == w["id"].String)
                    map.AddWallWithDoor(w["id"].String, a[0], w["thickness"].Double, door["centre_y"].Double, run.World.DoorwayWidth);
            }
            // The payload is a map obstacle under its object id; the library excludes the grasped object from the final grasp plan by that id
            // (PlanRequest.IgnoreObstacleIds), so the id here must equal the mission object id (library fix for v2 seed 2, 2026-11-10).
            if (Payload != null) {
                var c = FleetFrames.Planar(Payload.transform);
                map.AddBox(payloadId, c.X, c.Y, payloadSize[0], payloadSize[1], c.Yaw);
            }
            return map;
        }

        // ------------------------------------------------------------------ step
        public void PreStep(FleetRun r) {
            clock.Now = r.T;
            fleet.Step(r.Dt);
            foreach (var rv in r.Rovers) {
                var k = sinks[rv.D.Id];
                rv.Plant.WheelCommandOverride = k.Wheels;
                if (rv.D.Role == "lifter") Actuate(rv, k, r.Dt);
            }
            couplings.Step(r.Dt);
            if (payloadMode == "carried" && carrier != null) {
                Vector3 at = carrier.Body.position + carrier.Body.rotation * carryPos + Vector3.up * (float)(carrier.Lift - carryLift0);
                Payload.MovePosition(at);
                Payload.MoveRotation(carrier.Body.rotation * carryRot);
                if (loadModel == "physical") ApplyLoad(r, at);
            }
            if (loadModel == "physical") CheckTip(r);
            // mirror the library's events and task records into the run log
            for (; autonomyEventIndex < fleet.Events.Count; autonomyEventIndex++) {
                var e = fleet.Events[autonomyEventIndex];
                run.Event("autonomy", e.Agent ?? "", new Dictionary<string, object> { ["event"] = e.Kind, ["task"] = e.Task, ["detail"] = e.Detail, ["t_autonomy"] = e.T });
            }
            foreach (var rec in fleet.Executive.Records) {
                lastState.TryGetValue(rec.Spec.Id, out var prev);
                if (rec.State != prev && rec.State == TaskState.Succeeded) CheckClaim(rec);
                lastState[rec.Spec.Id] = rec.State;
            }
            if (fleet.Executive.BelievedComplete && !hostVerdictGiven) {
                // host verdict (library contract): only the evaluator's truth delivery check makes the mission a success.
                // "At rest" is part of the check, so wait (up to 3 s) for the payload to settle after the last motion.
                if (double.IsNaN(believedAt)) believedAt = r.T;
                var pt = PayloadTruth();
                bool settled = pt == null || (double)pt["speed"] < 0.01 && (string)pt["state"] != "carried";
                if (settled || r.T - believedAt > 3.0) {
                    hostVerdictGiven = true;
                    bool ok = pt == null || (bool)pt["delivered"];
                    string reason = pt == null ? "no payload" : $"payload {pt["state"]} at ({pt["x"]:0.00}, {pt["y"]:0.00}), in delivery zone {pt["in_delivery_zone"]}, speed {pt["speed"]:0.000} m/s, judged {r.T - believedAt:0.00} s after believed-complete";
                    fleet.Executive.ConfirmByHost(ok, reason);
                    results["host_verdict"] = new Dictionary<string, object> { ["t"] = r.T, ["ok"] = ok, ["reason"] = reason };
                    run.Event("host_verdict", "", new Dictionary<string, object> { ["ok"] = ok, ["reason"] = reason });
                }
            }
            if (fleet.Executive.Done && double.IsNaN(doneAt) && (hostVerdictGiven || !fleet.Executive.BelievedComplete)) {
                doneAt = r.T;
                run.Event(fleet.Executive.Succeeded ? "mission_succeeded" : "mission_failed", "", new Dictionary<string, object> {
                    ["first_failure_class"] = fleet.Executive.FirstFailureClass.ToString(), ["first_failure"] = fleet.Executive.FirstFailure });
            }
            if (!double.IsNaN(doneAt) && r.T > doneAt + 2.0) Done = true;
        }

        void Actuate(RoverRig rv, Sink k, double dt) {
            double travel = rv.Visual?.Manifest.LiftRange?[1] ?? spec.P["roles.lifter.lift_travel"].Double;
            double target = Math.Max(0, Math.Min(travel, k.LiftTarget));
            // lift force limit: raising a carried payload heavier than the actuator can hold (profile lift_capacity x overlay margin) stalls
            bool raising = target > rv.Lift && payloadMode == "carried" && carrier == rv;
            forceLimitedNow = raising && loadModel == "physical" && Payload.mass * spec.Gravity > liftForceMax;   // LiftReport.ForceLimited
            if (forceLimitedNow) {
                if (!liftStalled) {
                    liftStalled = true;
                    run.Event("lift_stalled", rv.D.Id, new Dictionary<string, object> { ["payload_kg"] = (double)Payload.mass, ["lift_force_max_n"] = liftForceMax,
                        ["needed_n"] = Payload.mass * spec.Gravity, ["height"] = rv.Lift, ["evaluator_only"] = true });
                }
            }
            else rv.Lift = Toward(rv.Lift, target, liftSpeed * dt);
            bool canHold = Payload != null && payloadMode != "carried" && GripWindow(rv);
            double gripTarget = k.GripperClosed ? (canHold ? Math.Max(gripMin, payloadSize[1]) : gripMin) : gripMax;
            rv.GripperOpening = Toward(rv.GripperOpening, gripTarget, gripRate * dt);
            bool closedReached = k.GripperClosed && Math.Abs(rv.GripperOpening - gripTarget) < 1e-9;
            if (closedReached && canHold && payloadMode != "carried") Attach(rv);
            if (!k.GripperClosed && payloadMode == "carried" && carrier == rv) Release();
            sources[rv.D.Id].GripperClosed = closedReached;
        }

        static double Toward(double v, double t, double s) => v < t ? Math.Min(t, v + s) : Math.Max(t, v - s);

        /// <summary>Gripper geometry check: finger line inside the payload's length near its end, payload between the open fingers,
        /// finger height within the payload side. Truth geometry on the simulator side (the gripper's physics), not robot knowledge.</summary>
        bool GripWindow(RoverRig rv) {
            var m = rv.Visual?.Manifest;
            double fingerX = m != null ? m.FindPart(m.LiftPart).Origin.Xyz[0] + m.FindPart(m.Fingers[0].part).Origin.Xyz[0] : 0.297;
            double fingerZ0 = m != null ? m.FindPart(m.LiftPart).Origin.Xyz[2] + m.FindPart(m.Fingers[0].part).Origin.Xyz[2] : 0.0;
            var t = rv.Truth();
            var c = FleetFrames.Planar(Payload.transform);
            double fx = t.X + Math.Cos(t.Yaw) * fingerX, fy = t.Y + Math.Sin(t.Yaw) * fingerX;
            double ca = Math.Cos(c.Yaw), sa = Math.Sin(c.Yaw), dx = fx - c.X, dy = fy - c.Y;
            double along = ca * dx + sa * dy, across = -sa * dx + ca * dy;
            double yawErr = Math.Abs(Angles.Wrap(t.Yaw - c.Yaw));
            double fingerZ = rv.D.OriginHeight + fingerZ0 + rv.Lift, bottom = Payload.position.y;
            bool inLength = Math.Abs(along) <= payloadSize[0] / 2 - 0.01, centred = Math.Abs(across) <= (gripMax - payloadSize[1]) / 2;
            bool height = fingerZ >= bottom - 0.01 && fingerZ <= bottom + payloadSize[2];
            return inLength && centred && height && (yawErr < 0.2 || Math.Abs(yawErr - Math.PI) < 0.2);
        }

        void Attach(RoverRig rv) {
            payloadMode = "carried"; carrier = rv;
            Payload.isKinematic = true;
            carryPos = Quaternion.Inverse(rv.Body.rotation) * (Payload.position - rv.Body.position);
            carryRot = Quaternion.Inverse(rv.Body.rotation) * Payload.rotation;
            carryLift0 = rv.Lift;
            double cap = spec.P["roles.lifter.lift_capacity"].Double, pm = Payload.mass, comX = (pm * carryPos.z + rv.D.Mass * rv.D.ComOffset[0]) / (pm + rv.D.Mass);
            var f = new Dictionary<string, object> { ["payload"] = payloadId, ["payload_kg"] = pm, ["capacity_kg"] = cap, ["over_capacity"] = pm > cap,
                ["combined_com_x"] = comX, ["front_axle_x"] = rv.D.Wheels[0].Axle.X, ["would_tip_forward"] = comX > rv.D.Wheels[0].Axle.X,
                ["note"] = loadModel == "physical" ? "lift_load physical: payload mass, CoM and inertia join the lifter rigid body once lifted off; lift force limited"
                    : "carry is kinematic: lifter load and tipping are not simulated" };
            results["lift_static_check"] = f;
            run.Event("payload_gripped", rv.D.Id, f);
        }

        /// <summary>
        /// Lifter load once the lift has raised the payload off its support (lift_off_height above the grip height): the payload's mass, CoM and
        /// box inertia are added to the carrier's rigid body (composite rigid body; restored on release), so PhysX integrates weight, inertia,
        /// load transfer and tipping implicitly. Off-diagonal products of inertia of the composite are dropped (the payload sits on the
        /// carrier's centre line, so they are small). A first version applied m (g - a) explicitly at the payload CoM; that is unstable when the
        /// payload outweighs the carrier's effective mass at that point (box_small at 0.41 m: about 0.7 kg) and was replaced (2026-11-10).
        /// </summary>
        void ApplyLoad(FleetRun r, Vector3 payloadPos) {
            if (loadApplied || carrier.Lift - carryLift0 < liftOffHeight) return;
            loadApplied = true; loadEverApplied = true;
            var b = carrier.Body;
            saved = (carrier, b.mass, b.centerOfMass, b.inertiaTensor, b.inertiaTensorRotation);
            float M = b.mass, m = Payload.mass;
            Vector3 cOld = b.centerOfMass;
            Vector3 pLocal = b.transform.InverseTransformPoint(payloadPos + Payload.rotation * Payload.centerOfMass);
            Vector3 cNew = (M * cOld + m * pLocal) / (M + m);
            // payload box inertia about its CoM, in carrier local axes (Unity local: x = -left, y = up, z = forward; payload aligned with the carrier)
            float lx = (float)payloadSize[0], ly = (float)payloadSize[1], lz = (float)payloadSize[2];
            var Ip = new Vector3(m / 12f * (lz * lz + lx * lx), m / 12f * (lx * lx + ly * ly), m / 12f * (ly * ly + lz * lz));   // about local x, y, z
            Vector3 Ic = b.inertiaTensor;   // carrier, about cOld, rotation identity (RoverRig)
            Vector3 d1 = cOld - cNew, d2 = pLocal - cNew;
            Vector3 Par(Vector3 d, float mass) => mass * new Vector3(d.y * d.y + d.z * d.z, d.x * d.x + d.z * d.z, d.x * d.x + d.y * d.y);
            b.mass = M + m;
            b.centerOfMass = cNew;
            b.inertiaTensor = Ic + Par(d1, M) + Ip + Par(d2, m);
            b.inertiaTensorRotation = Quaternion.identity;
            run.Event("payload_load_on", carrier.D.Id, new Dictionary<string, object> { ["payload_kg"] = (double)m, ["composite_mass"] = (double)b.mass,
                ["composite_com_forward"] = (double)cNew.z, ["model"] = "composite rigid body" });
        }

        (RoverRig rig, float mass, Vector3 com, Vector3 inertia, Quaternion rot)? saved;

        void RemoveLoad() {
            if (saved == null) return;
            var s = saved.Value; var b = s.rig.Body;
            b.mass = s.mass; b.centerOfMass = s.com; b.inertiaTensor = s.inertia; b.inertiaTensorRotation = s.rot;
            saved = null;
            run.Event("payload_load_off", s.rig.D.Id, new Dictionary<string, object>());
        }

        /// <summary>Evaluator tip check on the lifter: |pitch| or |roll| above lift_load.tip_limit_rad. A tipped lifter drops what it carries.</summary>
        void CheckTip(FleetRun r) {
            foreach (var rv in r.Rovers) {
                if (rv.D.Role != "lifter") continue;
                var t = rv.Truth();
                double tilt = Math.Max(Math.Abs(t.Pitch), Math.Abs(t.Roll));
                maxCarrierTilt = Math.Max(maxCarrierTilt, tilt);
                if (tilt > tipLimit && !lifterTipped) {
                    lifterTipped = true;
                    run.Event("tip", rv.D.Id, new Dictionary<string, object> { ["pitch"] = t.Pitch, ["roll"] = t.Roll, ["carrying"] = payloadMode == "carried" && carrier == rv,
                        ["evaluator_only"] = true });
                    if (payloadMode == "carried" && carrier == rv) Release();
                }
            }
        }

        void Release() {
            RemoveLoad(); loadApplied = false;
            payloadMode = "dynamic"; carrier = null;
            Payload.isKinematic = false;
            Payload.linearVelocity = Vector3.zero; Payload.angularVelocity = Vector3.zero;
            run.Event("payload_released", "", new Dictionary<string, object> { ["bottom_height"] = (double)Payload.position.y });
        }

        public void PostStep(FleetRun r) {
            perception.Record(r.T, r.Rovers);
            if (Payload != null) perception.RecordObject(r.T, payloadId, PayloadBase(), payloadTag);
        }

        // ------------------------------------------------------------------ evaluator truth checks
        JsonNode TaskJson(string id) { foreach (var t in missionJson["tasks"].Items) if (t["id"].String == id) return t; return null; }

        /// <summary>Payload support from truth: which deck colliders it rests on (within 5 mm), its bottom height and tilt.</summary>
        public Dictionary<string, object> PayloadTruth() {
            if (Payload == null) return null;
            var sup = new List<object>();
            bool floor = false;
            Vector3 probe = payloadCol.transform.position + Vector3.down * 0.005f;
            var hits = new Collider[64];
            int n = run.Phys.Physics.OverlapBox(probe, payloadCol.size / 2, hits, payloadCol.transform.rotation, ~0, QueryTriggerInteraction.Ignore);
            for (int i = 0; i < n; i++) {
                var col = hits[i];
                if (col == payloadCol) continue;
                if (col.name == "floor") floor = true;
                foreach (var r in run.Rovers) if (col.transform.IsChildOf(r.Root.transform) && col.name.Contains("deck") && !sup.Contains(r.D.Id)) sup.Add(r.D.Id);
            }
            FleetFrames.Attitude(Payload.rotation, out _, out double pitch, out double roll);
            var c = FleetFrames.Planar(Payload.transform);
            double[] z = spec.S["zones.delivery.rect"].Doubles;
            bool inZone = c.X >= z[0] && c.X <= z[2] && c.Y >= z[1] && c.Y <= z[3];
            double tilt = Math.Max(Math.Abs(pitch), Math.Abs(roll));
            bool onDecks = payloadMode != "carried" && sup.Count > 0 && !floor && tilt < 0.1;
            string state = payloadMode == "carried" ? "carried" : onDecks ? "on_decks" : floor ? "on_floor" : "unsupported_or_tilted";
            return new Dictionary<string, object> { ["state"] = state, ["supporters"] = sup, ["touches_floor"] = floor, ["x"] = c.X, ["y"] = c.Y,
                ["bottom_height"] = (double)Payload.position.y, ["tilt"] = tilt, ["speed"] = (double)Payload.linearVelocity.magnitude,
                ["in_delivery_zone"] = inZone, ["delivered"] = inZone && onDecks && Payload.linearVelocity.magnitude < 0.01 };
        }

        /// <summary>The library declared a task succeeded: check the same claim against evaluator truth and log any disagreement.</summary>
        void CheckClaim(TaskRecord rec) {
            var t = TaskJson(rec.Spec.Id);
            string type = rec.Spec.Type, truthSays = null; bool agree = true;
            switch (type) {
                case "goto":
                case "transport": {
                    var r = run.Rover(t["agent"].String); var tr = r.Truth(); double[] g = t["goal"].Doubles;
                    double d = Math.Sqrt((tr.X - g[0]) * (tr.X - g[0]) + (tr.Y - g[1]) * (tr.Y - g[1]));
                    double tol = t.Has("tol") ? t["tol"].Doubles[0] : 0.1;
                    agree = d <= Math.Max(0.1, 2 * tol);
                    truthSays = $"{r.D.Id} truth ({tr.X:0.00}, {tr.Y:0.00}) is {d:0.00} m from the goal ({g[0]:0.00}, {g[1]:0.00})" + (run.IsJammed(r) ? ", jammed" : "");
                    if (type == "transport" && t.Has("payload")) {
                        var p = PayloadTruth();
                        bool del = p != null && (bool)p["in_delivery_zone"] && (string)p["state"] == "on_decks";   // at rest is checked by the host verdict
                        agree &= del;
                        truthSays += $"; payload {p?["state"]} at ({p?["x"]:0.00}, {p?["y"]:0.00}), delivered={del}";
                    }
                    break;
                }
                case "dock": {
                    var c = couplings.Of(run.Rover(t["docker"].String), t["docker_face"].String);
                    agree = c != null && c.Joint != null;
                    truthSays = c != null ? $"coupling {c.Id} {ConnectorStateNames.ToProfileName(c.Machine.State)}, joint={(c.Joint != null)}" : "no coupling";
                    break;
                }
                case "lift": {
                    agree = payloadMode == "carried";
                    truthSays = "payload " + payloadMode;
                    break;
                }
                case "handoff": {
                    var p = PayloadTruth();
                    var want = new List<string>(); foreach (var x in t["supporters"].Items) want.Add(x.String);
                    var sup = p != null ? (List<object>)p["supporters"] : new List<object>();
                    agree = p != null && (string)p["state"] == "on_decks" && sup.Count == want.Count;
                    truthSays = $"payload {p?["state"]}, on decks of [{string.Join(",", sup)}] (wanted [{string.Join(",", want)}]), bottom {p?["bottom_height"]:0.000} m";
                    break;
                }
                case "explore": truthSays = "target seen by the agent's own camera (perception core)"; break;
                default: truthSays = "not checked"; break;
            }
            var row = new Dictionary<string, object> { ["t"] = run.T, ["task"] = rec.Spec.Id, ["type"] = type, ["library_believed"] = "succeeded",
                ["truth"] = truthSays, ["agree"] = agree };
            beliefVsTruth.Add(row);
            if (!agree) disagreements++;
            run.Event(agree ? "claim_confirmed" : "claim_disputed", "", row);
        }

        // ------------------------------------------------------------------ logs
        public void LogState(StringBuilder b) {
            string N(double v) => FleetJson.Num(v);
            b.Append("\"autonomy\":{\"tasks\":{");
            bool first = true;
            foreach (var rec in fleet.Executive.Records) {
                if (!first) b.Append(','); first = false;
                b.Append(FleetJson.Quote(rec.Spec.Id)).Append(":{\"state\":").Append(FleetJson.Quote(rec.State.ToString())).Append(",\"attempts\":").Append(rec.Attempts)
                    .Append(",\"phase\":").Append(FleetJson.Quote(rec.Behavior?.Phase ?? "")).Append('}');
            }
            b.Append("},\"estimates\":{");
            first = true;
            foreach (var rv in run.Rovers) {
                var ag = fleet.AgentOf(rv.D.Id);
                if (ag == null) continue;
                if (!first) b.Append(','); first = false;
                var e = ag.MemberPose(rv.D.Id);
                var cov = sinks[rv.D.Id].LastCov ?? new double[9];
                b.Append(FleetJson.Quote(rv.D.Id)).Append(":{\"x\":").Append(N(e.X)).Append(",\"y\":").Append(N(e.Y)).Append(",\"yaw\":").Append(N(e.Yaw))
                    .Append(",\"agent\":").Append(FleetJson.Quote(ag.Id)).Append(",\"cov3\":[");
                for (int i = 0; i < cov.Length; i++) { if (i > 0) b.Append(','); b.Append(N(cov[i])); }
                b.Append("]}");
            }
            b.Append("}}");
            var c0 = couplings.All.Count > 0 ? couplings.All[0] : null;
            b.Append(",\"connector\":{\"state\":").Append(FleetJson.Quote(c0 != null ? ConnectorStateNames.ToProfileName(c0.Machine.State) : "FREE"));
            if (c0 != null) b.Append(",\"coupling\":").Append(FleetJson.Quote(c0.Id)).Append(",\"gap\":").Append(N(c0.Pose.Gap)).Append(",\"lateral\":").Append(N(c0.Pose.Lateral)).Append(",\"yaw\":").Append(N(c0.Pose.Yaw))
                .Append(",\"joint\":").Append(c0.Joint != null ? "true" : "false");
            b.Append('}');
            if (Payload != null) {
                FleetFrames.ToEnu(Payload.position, out double x, out double y, out double z);
                b.Append(",\"payload\":{\"mode\":").Append(FleetJson.Quote(payloadMode)).Append(",\"x\":").Append(N(x)).Append(",\"y\":").Append(N(y)).Append(",\"z\":").Append(N(z)).Append('}');
            }
        }

        public void Finish(FleetRun r, Dictionary<string, object> res) {
            var tasks = new List<object>();
            foreach (var rec in fleet.Executive.Records)
                tasks.Add(new Dictionary<string, object> { ["id"] = rec.Spec.Id, ["type"] = rec.Spec.Type, ["state"] = rec.State.ToString(), ["attempts"] = rec.Attempts,
                    ["started"] = rec.Started, ["ended"] = rec.Ended, ["class"] = rec.Class.ToString(), ["reason"] = rec.Reason });
            results["tasks"] = tasks;
            results["done"] = fleet.Executive.Done;
            results["library_believed_complete"] = fleet.Executive.BelievedComplete;
            results["succeeded"] = fleet.Executive.Succeeded;
            results["first_failure_class"] = fleet.Executive.FirstFailureClass.ToString();
            results["first_failure"] = fleet.Executive.FirstFailure;
            results["lift_load"] = new Dictionary<string, object> { ["model"] = loadModel, ["payload_kg"] = Payload != null ? (object)(double)Payload.mass : null,
                ["payload_mass_source"] = payloadMassSource, ["lift_force_max_n"] = liftForceMax, ["lift_stalled"] = liftStalled, ["load_applied"] = loadEverApplied, ["load_model"] = "composite rigid body while lifted",
                ["lifter_tipped"] = lifterTipped, ["max_lifter_tilt_rad"] = maxCarrierTilt };
            if (Payload != null) {
                var c = FleetFrames.Planar(Payload.transform);
                double[] z = spec.S["zones.delivery.rect"].Doubles;
                results["payload_final"] = new Dictionary<string, object> { ["x"] = c.X, ["y"] = c.Y, ["bottom_height"] = (double)Payload.position.y, ["mode"] = payloadMode,
                    ["in_delivery_zone"] = c.X >= z[0] && c.X <= z[2] && c.Y >= z[1] && c.Y <= z[3] };
            }
            var pt = PayloadTruth();
            results["payload_truth"] = pt;
            results["belief_vs_truth"] = beliefVsTruth;
            results["disagreements"] = disagreements;
            results["jams"] = run.Jams;
            bool delivered = pt != null && (bool)pt["delivered"];
            results["truth_outcome"] = delivered ? "delivered" : pt == null ? "no_payload" : "not_delivered: payload " + pt["state"];
            results["summary"] = $"library believed: {(fleet.Executive.BelievedComplete ? "complete" : "not complete (" + fleet.Executive.FirstFailureClass + ")")}, host verdict {(hostVerdictGiven ? (fleet.Executive.Succeeded ? "accepted" : "rejected") : "not requested")}; " +
                $"truth: {results["truth_outcome"]}; {disagreements} disputed claims; {run.Jams.Count} jams"
                + (liftStalled ? "; lift stalled (payload over the lift force limit)" : "") + (lifterTipped ? "; lifter tipped" : "");
            res["autonomy"] = results;
        }

        // ------------------------------------------------------------------ I/O
        sealed class Source : IObservationSource, IInertialSource, IRoverSampleSink, Crane.Fleet.Autonomy.IObservationSourceExtras {
            /// <summary>Library channel (IInertialSource): body-frame specific force, gravity removed, at the IMU rate.</summary>
            public void ReadAccel(List<AccelSample> into) { into.Clear(); foreach (var a in accel) into.Add(new AccelSample { Stamp = a.Stamp, Ax = a.Ax, Ay = a.Ay }); accel.Clear(); }
            readonly List<Crane.Fleet.Autonomy.WheelRateSample> rates = new List<Crane.Fleet.Autonomy.WheelRateSample>();
            readonly List<Crane.Fleet.Autonomy.AccelSample> accel = new List<Crane.Fleet.Autonomy.AccelSample>();
            public void OnWheelRates(double t, double[] c, double[] m) => rates.Add(new Crane.Fleet.Autonomy.WheelRateSample { Stamp = t, Commanded = c, Measured = m });
            public void OnAccel(double t, double ax, double ay) => accel.Add(new Crane.Fleet.Autonomy.AccelSample { Stamp = t, Ax = ax, Ay = ay });
            public void ReadWheelRates(List<Crane.Fleet.Autonomy.WheelRateSample> into) { into.Clear(); into.AddRange(rates); rates.Clear(); }
            public void ReadAccel(List<Crane.Fleet.Autonomy.AccelSample> into) { into.Clear(); into.AddRange(accel); accel.Clear(); }
            readonly AutonomyMission m; readonly RoverRig rig;
            readonly List<EncoderSample> enc = new List<EncoderSample>();
            readonly List<GyroSample> gyro = new List<GyroSample>();
            public bool GripperClosed;
            public Source(AutonomyMission m, RoverRig rig) { this.m = m; this.rig = rig; }
            public string RoverId => rig.D.Id;
            public void OnEncoder(double t, long[] ticks) => enc.Add(new EncoderSample(t, ticks[0], ticks[1], ticks[2], ticks[3]));
            public void OnGyro(double t, double rate) => gyro.Add(new GyroSample(t, rate));
            public void ReadEncoders(List<EncoderSample> into) { into.Clear(); into.AddRange(enc); enc.Clear(); }
            public void ReadGyro(List<GyroSample> into) { into.Clear(); into.AddRange(gyro); gyro.Clear(); }
            public void ReadTagObservations(List<RelPoseObservation> into) {
                into.Clear();
                into.AddRange(m.perception.Provider(rig.D.Id).PollTagObservations(m.run.T));
                into.AddRange(m.perception.PollObjectObservations(rig.D.Id, m.run.T));
            }
            public ConnectorReport ReadConnector(string face) {
                // the connector machine exists once a facing partner face is in range (the library pulses Approach only to a present machine)
                var c = m.couplings.TryPair(rig, face);
                if (c == null) return new ConnectorReport { Present = false, State = ConnectorState.Free };
                bool isA = c.A == rig;
                return new ConnectorReport { Present = true, State = c.Machine.State, SensedA = c.Machine.SensedA, SensedB = c.Machine.SensedB, Action = c.LastAction,
                    FailedAttempts = c.Machine.FailedAttempts, RetriesExhausted = c.Machine.RetriesExhausted,
                    PartnerRover = c.Machine.Captured || ConnectorStateNames.IsVerified(c.Machine.State) ? (isA ? c.B.D.Id : c.A.D.Id) : null,
                    PartnerFace = c.Machine.Captured || ConnectorStateNames.IsVerified(c.Machine.State) ? (isA ? c.FaceB : c.FaceA) : null };
            }
            public LiftReport ReadLift() {
                if (rig.D.Role == "lifter")
                    return new LiftReport { Present = true, Height = rig.Lift, GripperClosed = GripperClosed, HoldingObject = m.payloadMode == "carried" && m.carrier == rig,
                        ForceLimited = m.forceLimitedNow };
                // deck load cell (cargo decks): the payload's weight shared over the decks it rests on (simulated sensor from contact geometry)
                bool deck = rig.D.Payload.ContainsKey("deck_capacity");
                double kg = 0;
                if (deck && m.Payload != null && m.payloadMode != "carried") {
                    var pt = m.PayloadTruth();
                    var sup = (List<object>)pt["supporters"];
                    if (sup.Contains(rig.D.Id)) kg = m.Payload.mass / sup.Count;
                }
                return new LiftReport { Present = false, DeckLoadValid = deck, DeckLoadKg = kg };
            }
        }

        sealed class Sink : IActuatorSink, IEstimatePublisher {
            readonly AutonomyMission m; readonly RoverRig rig;
            public double[] Wheels = new double[4];
            public double LiftTarget; public bool GripperClosed;
            public double[] LastCov;
            public Sink(AutonomyMission m, RoverRig rig) { this.m = m; this.rig = rig; }
            public string RoverId => rig.D.Id;
            public void SetWheelSpeeds(double fl, double fr, double rl, double rr) { Wheels[0] = fl; Wheels[1] = fr; Wheels[2] = rl; Wheels[3] = rr; }
            public void CommandConnector(string face, DockingCommand command) => m.couplings.Command(rig, face, command);
            public void SetLift(double heightTarget, bool gripperClosed) { LiftTarget = heightTarget; GripperClosed = gripperClosed; }
            public void PublishEstimate(double stamp, Pose2 pose, double[] cov3) {
                LastCov = (double[])cov3.Clone();
                var c6 = new Matrix(6, 6);
                c6[0, 0] = cov3[0]; c6[0, 1] = cov3[1]; c6[1, 0] = cov3[3]; c6[1, 1] = cov3[4]; c6[0, 5] = cov3[2]; c6[5, 0] = cov3[6]; c6[1, 5] = cov3[5]; c6[5, 1] = cov3[7]; c6[5, 5] = cov3[8];
                c6[2, 2] = 1e-4; c6[3, 3] = 1e-4; c6[4, 4] = 1e-4;
                m.perception.Provider(rig.D.Id).ReportSelfEstimate(stamp, PPose.FromYaw(pose.X, pose.Y, 0, pose.Yaw), c6);
            }
        }
    }

    /// <summary>Receives a rover's own sensor samples as the rig produces them (encoders at the encoder rate, gyro at the IMU rate).</summary>
    public interface IRoverSampleSink {
        void OnEncoder(double t, long[] ticks);
        void OnGyro(double t, double rate);
        void OnWheelRates(double t, double[] commanded, double[] measured);
        void OnAccel(double t, double ax, double ay);
    }
}
