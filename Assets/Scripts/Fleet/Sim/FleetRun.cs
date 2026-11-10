using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using Sim.Performance;
using Unity.Collections;
using UnityEngine;

namespace Crane.Fleet.Sim {
    /// <summary>A scripted mission run by <see cref="FleetRun"/> instead of the qualification script.</summary>
    public interface IFleetMission {
        string Variant { get; }
        Pose2 StartPose(string roverId);
        /// <summary>After the rovers exist, before settling: create payloads, stands, couplings; return extra roots to adopt into the run's physics scene.</summary>
        IEnumerable<GameObject> Build(FleetRun run);
        void Start(FleetRun run);
        /// <summary>Before each physics step: set every rover's Command (and lifter, payload, connector state).</summary>
        void PreStep(FleetRun run);
        void PostStep(FleetRun run);
        bool Done { get; }
        string StepOf(string roverId);
        /// <summary>Extra per-row state (written under "mission" in states.jsonl).</summary>
        void LogState(StringBuilder b);
        void Finish(FleetRun run, Dictionary<string, object> results);
        string ControllerInput { get; }
        /// <summary>Cost-table keys for this mission (interface pair types, shape class, payload class).</summary>
        IDictionary<string, object> Labels { get; }
    }

    /// <summary>A mission that brings its own fleet (module graphs) instead of the profile's fleet_default.</summary>
    public interface IFleetComposer {
        /// <summary>Rover descriptions in spawn order, with each one's initial wheel health (null = all ok).</summary>
        IEnumerable<(RoverDescription d, WheelHealth[] health, double[] gain)> Fleet(FleetSpec spec);
        string Configuration { get; }
        string ConfigurationHash { get; }
    }

    public sealed class FleetRunOptions {
        public long Seed = 1;
        public string Variant;            // default: overlay qualification.variant
        public string OutRoot;            // default ~/.cache/crane-fleet/runs
        public string OutDir;             // explicit run directory (overrides OutRoot)
        public double? DoorwayWidth;
        public double? MaxDuration;
        public string SourceCommit = "";
        public bool SourceDirty;
        public string Launcher = "unknown";
        public IFleetMission Mission;
        /// <summary>Injected failures: "rover:target:mode@t;..." with target wheel_fl|wheel_fr|wheel_rl|wheel_rr (dead_free, dead_locked, weak, ok),
        /// camera (lost), encoders (lost). Applied when run time reaches t; each is logged as a failure_injected event.</summary>
        public string Failures = "";
        /// <summary>Extra labels merged over the defaults in manifest.json "labels" (harness: configuration id/hash, etc.).</summary>
        public Dictionary<string, object> Labels = new Dictionary<string, object>();
    }

    /// <summary>
    /// One seeded qualification run of the default fleet in the scenario, stepped manually
    /// (Physics.simulationMode = Script, fixed dt from the overlay) so it is the same in the editor and the player
    /// and independent of frame rate. Writes manifest.json, states.jsonl (evaluator truth, rover estimates and
    /// actuation in separate top-level fields), events.jsonl and trajectory.ctraj (crane-traj/1).
    /// </summary>
    public sealed class FleetRun {
        public readonly FleetSpec Spec;
        public readonly FleetRunOptions Opt;
        public readonly string RunId, Dir;
        public FleetWorld World;
        public readonly List<RoverRig> Rovers = new List<RoverRig>();
        public double T;          // run time since the end of settling
        public long Step;
        public bool Done;
        public string EndReason = "running";

        readonly double dt, maxDuration;
        readonly int stateEvery, trajEvery;
        StreamWriter states, events;
        readonly List<CraneTraj.Column> trajColumns = new List<CraneTraj.Column>();
        readonly List<string> trajEvents = new List<string>();
        readonly List<StepEvent> stepEvents = new List<StepEvent>();
        readonly Dictionary<string, (RoverTruth truth, double ox, double oy, double oyaw, double t)> stepStart =
            new Dictionary<string, (RoverTruth, double, double, double, double)>();
        readonly Dictionary<Collider, RoverRig> byCollider = new Dictionary<Collider, RoverRig>();
        readonly List<string> pendingContacts = new List<string>();
        readonly Dictionary<string, object> results = new Dictionary<string, object>();
        readonly Dictionary<string, bool> saturated = new Dictionary<string, bool>();
        DateTime wallStart;
        SimulationMode previousMode;
        public FleetPhysicsScene Phys;
        public double Dt => dt;
        /// <summary>Every event as it is logged (kind, rover, fields); stills and missions subscribe.</summary>
        public event Action<string, string, Dictionary<string, object>> EventRaised;
        public RoverRig Rover(string id) { foreach (var r in Rovers) if (r.D.Id == id) return r; throw new KeyNotFoundException(id); }
        IFleetMission M => Opt.Mission;

        public FleetRun(FleetSpec spec, FleetRunOptions opt) {
            Spec = spec; Opt = opt;
            Opt.Variant ??= opt.Mission != null ? opt.Mission.Variant : spec.O["qualification.variant"].String;
            RunId = RunIds.Make(spec.S["scenario_id"].String, Opt.Variant, Opt.Seed);
            string root = opt.OutRoot ?? Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), ".cache", "crane-fleet", "runs");
            Dir = opt.OutDir ?? Path.Combine(root, RunIds.DirectoryName(RunId));
            dt = spec.Dt;
            maxDuration = opt.MaxDuration ?? spec.O["qualification.max_duration_s"].Double;
            stateEvery = Math.Max(1, (int)Math.Round(1.0 / (spec.O["logging.state_hz"].Double * dt)));
            trajEvery = Math.Max(1, (int)Math.Round(1.0 / (spec.O["logging.traj_hz"].Double * dt)));
        }

        public static void ConfigurePhysics(FleetSpec spec) {
            Physics.simulationMode = SimulationMode.Script;
            Physics.gravity = new Vector3(0f, (float)-spec.Gravity, 0f);
            Physics.defaultSolverIterations = (int)spec.O["physics.solver_iterations"].Double;
            Physics.defaultSolverVelocityIterations = (int)spec.O["physics.solver_velocity_iterations"].Double;
        }

        public void Setup() {
            wallStart = DateTime.UtcNow;
            previousMode = Physics.simulationMode;
            ConfigurePhysics(Spec);
            Directory.CreateDirectory(Dir);
            Phys = FleetPhysicsScene.Create("fleet run " + RunId);
            World = new FleetWorld(Spec, Opt.DoorwayWidth);

            FloorProperties floor = Spec.Floor();
            var jitter = FleetRng.For(Opt.Seed, "start_jitter");
            double jxy = Spec.O["qualification.start_jitter_xy"].Double, jyaw = Spec.O["qualification.start_jitter_yaw"].Double;
            var fleet = new List<(RoverDescription d, WheelHealth[] health, double[] gain)>();
            if (M is IFleetComposer comp) fleet.AddRange(comp.Fleet(Spec));
            else foreach (var (fid, frole) in Spec.FleetDefault()) fleet.Add((Spec.Describe(fid, frole), null, null));
            foreach (var (desc, health0, gain0) in fleet) {
                string id = desc.Id;
                Pose2 nominal;
                if (M != null) nominal = M.StartPose(id);
                else { double[] s = Spec.O["qualification.rovers." + id + ".start"].Doubles; nominal = new Pose2(s[0], s[1], s[2]); }
                var pose = new Pose2(nominal.X + jitter.Uniform(-jxy, jxy), nominal.Y + jitter.Uniform(-jxy, jxy), nominal.Yaw + jitter.Uniform(-jyaw, jyaw));
                var rig = new RoverRig(Spec, desc, floor, Opt.Seed, World.Root.transform, pose);
                if (health0 != null) for (int i = 0; i < health0.Length; i++) { rig.Plant.Health[i] = health0[i]; if (gain0 != null) rig.Plant.HealthGain[i] = gain0[i]; }
                Rovers.Add(rig);
                foreach (var c in rig.Root.GetComponentsInChildren<Collider>()) { byCollider[c] = rig; c.providesContacts = true; }
                saturated[id] = false;
                foreach (string axis in new[] { "x", "y", "z", "yaw", "pitch", "roll", "speed" }) {
                    string name = id + "." + axis;
                    trajColumns.Add(new CraneTraj.Column { Name = name, Scale = CraneTraj.ScaleOf(name) });
                }
            }
            var roots = new List<GameObject> { World.Root };
            if (M != null) roots.AddRange(M.Build(this));
            foreach (var g in roots) Phys.Adopt(g);
            Physics.SyncTransforms();

            // settle on the floor with zero command, then start the clock, the estimators and the scripts
            int settle = (int)Math.Round(Spec.O["physics.settle_s"].Double / dt);
            for (int i = 0; i < settle; i++) {
                foreach (var r in Rovers) r.ApplyPlant();
                Phys.Simulate((float)dt);
            }
            foreach (var r in Rovers) {
                r.StartOdometry();
                RoverTruth t = r.Truth();
                if (M == null) r.Script = new QualificationScript(Spec, r.D.Id, r.D.Geometry, new Pose2(t.X, t.Y, t.Yaw), FleetRng.For(Opt.Seed, r.D.Id + "/script"));
                r.UpdateVisual();
            }
            Physics.ContactEvent += OnContacts;

            states = new StreamWriter(Path.Combine(Dir, "states.jsonl"), false, new UTF8Encoding(false)) { NewLine = "\n" };
            events = new StreamWriter(Path.Combine(Dir, "events.jsonl"), false, new UTF8Encoding(false)) { NewLine = "\n" };
            WriteManifest(false);
            Event("run_start", "", new Dictionary<string, object> { ["run_id"] = RunId, ["seed"] = Opt.Seed, ["settle_steps"] = settle });
            foreach (var r in Rovers) {
                RoverTruth t = r.Truth();
                Event("spawn", r.D.Id, new Dictionary<string, object> { ["role"] = r.D.Role, ["namespace"] = r.D.Namespace,
                    ["render"] = r.RenderSource, ["truth_x"] = t.X, ["truth_y"] = t.Y, ["truth_z"] = t.Z, ["truth_yaw"] = t.Yaw });
            }
            M?.Start(this);
            LogState();
            SampleTraj();
        }

        /// <summary>Advance one fixed physics step. Returns false when the run has ended.</summary>
        public bool Advance() {
            if (Done) return false;
            ApplyFailures();
            M?.PreStep(this);
            foreach (var r in Rovers) {
                RoverTruth t = r.Truth();
                if (M == null) {
                    stepEvents.Clear();
                    r.Command = r.Script.Command(T, new Pose2(t.X, t.Y, t.Yaw), stepEvents);
                    foreach (var e in stepEvents) OnStep(r, e, t);
                }
                r.ApplyPlant();
                bool sat = r.Plant.TractionScale < 1 - 1e-9;
                if (sat != saturated[r.D.Id]) {
                    saturated[r.D.Id] = sat;
                    Event(sat ? "traction_limit_on" : "traction_limit_off", r.D.Id, new Dictionary<string, object> { ["scale"] = r.Plant.TractionScale });
                }
            }
            Phys.Simulate((float)dt);
            Step++;
            T = Step * dt;
            foreach (var r in Rovers) { r.Sense(Step); r.UpdateVisual(); }
            DetectJams();
            M?.PostStep(this);
            foreach (string c in pendingContacts) events.WriteLine(c);
            pendingContacts.Clear();
            if (Step % stateEvery == 0) LogState();
            if (Step % trajEvery == 0) SampleTraj();

            bool all = M != null ? M.Done : true;
            if (M == null) foreach (var r in Rovers) all &= r.Script.Done;
            if (all) { Done = true; EndReason = "script_complete"; }
            else if (T >= maxDuration) { Done = true; EndReason = "max_duration"; }
            return !Done;
        }

        public void Finish() {
            Physics.ContactEvent -= OnContacts;
            var final = new Dictionary<string, object>();
            foreach (var r in Rovers) {
                RoverTruth t = r.Truth();
                final[r.D.Id] = new Dictionary<string, object> {
                    ["truth"] = new Dictionary<string, object> { ["x"] = t.X, ["y"] = t.Y, ["yaw"] = t.Yaw },
                    ["estimate"] = new Dictionary<string, object> { ["x"] = r.Odometry.X, ["y"] = r.Odometry.Y, ["yaw"] = r.Odometry.Yaw },
                    ["odometry_error_xy"] = Math.Sqrt(Sq(r.Odometry.X - t.X) + Sq(r.Odometry.Y - t.Y)),
                    ["odometry_error_yaw"] = Angles.Wrap(r.Odometry.Yaw - t.Yaw),
                    ["script_step"] = StepOf(r)
                };
            }
            results["final"] = final;
            results["jams"] = Jams;
            M?.Finish(this, results);
            Event("run_end", "", new Dictionary<string, object> { ["reason"] = EndReason, ["t"] = T, ["steps"] = Step });
            states.Dispose(); events.Dispose();
            WriteTraj();
            WriteManifest(true);
            Physics.simulationMode = previousMode;
        }

        List<(double t, string rover, string target, string mode)> failures;

        void ApplyFailures() {
            if (failures == null) {
                failures = new List<(double, string, string, string)>();
                foreach (string item in (Opt.Failures ?? "").Split(new[] { ';' }, StringSplitOptions.RemoveEmptyEntries)) {
                    var at = item.Split('@');
                    var p = at[0].Split(':');
                    if (p.Length != 3) throw new FormatException("failure spec " + item);
                    failures.Add((at.Length > 1 ? double.Parse(at[1], CultureInfo.InvariantCulture) : 0, p[0], p[1], p[2]));
                }
            }
            for (int i = failures.Count - 1; i >= 0; i--) {
                var f = failures[i];
                if (T < f.t - 1e-9) continue;
                failures.RemoveAt(i);
                var r = Rover(f.rover);
                int wi = r.D.Wheels.FindIndex(w => "wheel_" + w.Name == f.target);
                if (wi >= 0) {
                    // mode: dead_free | dead_locked | ok | weak | weak=<gain> (gain for the tractive plant; the body-level mecanum plant has one fraction)
                    string mode = f.mode; double gain = double.NaN;
                    if (mode.StartsWith("weak=")) { gain = double.Parse(mode.Substring(5), CultureInfo.InvariantCulture); mode = "weak"; }
                    r.Plant.Health[wi] = mode switch { "dead_free" => WheelHealth.DeadFree, "dead_locked" => WheelHealth.DeadLocked, "weak" => WheelHealth.Weak,
                        "ok" => WheelHealth.Ok, _ => throw new FormatException("wheel failure mode " + f.mode) };
                    if (r.Tractive != null && mode == "weak") r.Tractive.Gain[wi] = double.IsNaN(gain) ? r.Tractive.P.DefaultWeakGain : gain;
                }
                else if (f.target == "camera") r.CameraLost = f.mode == "lost";
                else if (f.target == "encoders") r.EncodersLost = f.mode == "lost";
                else throw new FormatException("failure target " + f.target);
                Event("failure_injected", f.rover, new Dictionary<string, object> { ["target"] = f.target, ["mode"] = f.mode, ["scheduled_t"] = f.t });
            }
        }

        Dictionary<string, object> Labels() {
            var l = new Dictionary<string, object> {
                ["scenario_id"] = Spec.S["scenario_id"].String, ["scenario_sha256"] = Spec.Scenario.Sha256,
                ["configuration"] = M is IFleetComposer fc ? fc.Configuration : "profile:" + Spec.P["profile_id"].String + ":fleet_default",
                ["configuration_hash"] = M is IFleetComposer fh ? fh.ConfigurationHash : Spec.Profile.Sha256,
                ["variant"] = Opt.Variant, ["seed"] = Opt.Seed, ["floor"] = Spec.S["arena.floor"].String,
                ["failure_state"] = string.IsNullOrEmpty(Opt.Failures) ? "none" : Opt.Failures,
                ["mission"] = M != null ? M.GetType().Name : "qualification", ["controller_input"] = M != null ? M.ControllerInput : "evaluator_truth"
            };
            if (M != null) foreach (var kv in M.Labels) l[kv.Key] = kv.Value;
            foreach (var kv in Opt.Labels) l[kv.Key] = kv.Value;
            return l;
        }

        string StepOf(RoverRig r) => M != null ? M.StepOf(r.D.Id) : r.Script.CurrentStep;

        public void Dispose() {
            World?.Destroy();
            World = null;
            Phys?.Dispose();
            Phys = null;
        }

        /// <summary>Run to the end synchronously (editor CLI, tests and the headless player).</summary>
        public void RunToEnd() {
            Setup();
            while (Advance()) { }
            Finish();
        }

        // ---- qualification metrics ----

        void OnStep(RoverRig r, StepEvent e, RoverTruth t) {
            string key = r.D.Id + "/" + e.Step;
            var f = new Dictionary<string, object> { ["step"] = e.Step, ["index"] = e.Index };
            if (e.Kind == "step_start") stepStart[key] = (t, r.Odometry.X, r.Odometry.Y, r.Odometry.Yaw, T);
            else {
                f["duration"] = e.Duration;
                f["timed_out"] = e.TimedOut;
                if (stepStart.TryGetValue(key, out var s) && (e.Step.StartsWith("strafe") || e.Step.StartsWith("rotate"))) {
                    // displacement in the start pose's body frame: commanded vs truth vs the rover's estimate
                    double c = Math.Cos(s.truth.Yaw), sn = Math.Sin(s.truth.Yaw);
                    double dx = t.X - s.truth.X, dy = t.Y - s.truth.Y;
                    double ec = Math.Cos(s.oyaw), es = Math.Sin(s.oyaw);
                    double ox = r.Odometry.X - s.ox, oy = r.Odometry.Y - s.oy;
                    var q = new Dictionary<string, object> {
                        ["commanded"] = new[] { e.Commanded.Vx * e.Duration, e.Commanded.Vy * e.Duration, e.Commanded.Wz * e.Duration },
                        ["truth"] = new[] { c * dx + sn * dy, -sn * dx + c * dy, UnwrappedYaw(s.truth.Yaw, t.Yaw, e.Commanded.Wz * e.Duration) },
                        ["estimate"] = new[] { ec * ox + es * oy, -es * ox + ec * oy, UnwrappedYaw(s.oyaw, r.Odometry.Yaw, e.Commanded.Wz * e.Duration) }
                    };
                    f["displacement_body"] = q;
                    results[key] = q;
                }
            }
            Event(e.Kind, r.D.Id, f);
        }

        /// <summary>Yaw change, unwrapped toward the commanded change (a full turn is not zero).</summary>
        static double UnwrappedYaw(double a, double b, double expected) {
            double d = Angles.Wrap(b - a);
            while (d < expected - Math.PI) d += 2 * Math.PI;
            while (d > expected + Math.PI) d -= 2 * Math.PI;
            return d;
        }

        /// <summary>Contacts are logged once per collider pair when the pair first carries impulse (> threshold), not on mere proximity
        /// (PhysX reports pairs inside the summed contact offsets with zero impulse); the pair re-arms when it separates.</summary>
        void OnContacts(PhysicsScene scene, NativeArray<ContactPairHeader>.ReadOnly headers) {
            for (int h = 0; h < headers.Length; h++) {
                var header = headers[h];
                for (int i = 0; i < header.pairCount; i++) {
                    ContactPair pair = header.GetContactPair(i);
                    Collider a = pair.collider, b = pair.otherCollider;
                    if (a == null || b == null) continue;
                    var key = (a, b);
                    if (pair.isCollisionExit) { loggedPairs.Remove(key); continue; }
                    double impulse = pair.impulseSum.magnitude;
                    if (impulse <= contactImpulseThreshold || loggedPairs.Contains(key)) continue;
                    byCollider.TryGetValue(a, out RoverRig ra);
                    byCollider.TryGetValue(b, out RoverRig rb);
                    if ((a.name == "floor" || b.name == "floor") && (a.name.StartsWith("contact_") || b.name.StartsWith("contact_"))) continue;  // wheel-floor
                    if (ra != null && rb != null && IgnoreContact != null && IgnoreContact(ra, rb)) continue;
                    loggedPairs.Add(key);
                    if (ra != null) lastContact[ra] = (T, rb != null ? rb.D.Id + ":" + b.name : b.name);
                    if (rb != null) lastContact[rb] = (T, ra != null ? ra.D.Id + ":" + a.name : a.name);
                    string na = ra != null ? ra.D.Id + ":" + a.name : a.name, nb = rb != null ? rb.D.Id + ":" + b.name : b.name;
                    pendingContacts.Add(EventLine("contact", ra != null ? ra.D.Id : rb != null ? rb.D.Id : "", new Dictionary<string, object> {
                        ["a"] = na, ["b"] = nb, ["impulse"] = impulse }));
                }
            }
        }

        // ---- evaluator-only stall/jam detection: wheels commanded, body not moving ----
        readonly Dictionary<RoverRig, (double t, string other)> lastContact = new Dictionary<RoverRig, (double, string)>();
        readonly Dictionary<RoverRig, double> stalledSince = new Dictionary<RoverRig, double>();
        readonly Dictionary<RoverRig, bool> jammed = new Dictionary<RoverRig, bool>();
        public readonly List<Dictionary<string, object>> Jams = new List<Dictionary<string, object>>();
        const double JamWheelRate = 1.0, JamSpeed = 0.02, JamYawRate = 0.05, JamAfter = 1.0;

        /// <summary>A rover is jammed when it commands wheel motion (max |wheel set-point| above 1 rad/s) while its true body speed stays below
        /// 2 cm/s and yaw rate below 0.05 rad/s for more than 1 s. Evaluator-only signal (truth), logged as jam_start / jam_end with the
        /// last contact partner. A rover being pushed against a wall by its own wheels is the typical case.</summary>
        void DetectJams() {
            foreach (var r in Rovers) {
                double cmd = 0;
                for (int i = 0; i < r.Plant.WheelCount; i++) cmd = Math.Max(cmd, Math.Abs(r.Plant.WheelCommand[i]));
                var tw = FleetFrames.BodyTwist(r.Body);
                bool still = Math.Sqrt(tw.Vx * tw.Vx + tw.Vy * tw.Vy) < JamSpeed && Math.Abs(tw.Wz) < JamYawRate;
                bool stalled = cmd > JamWheelRate && still;
                jammed.TryGetValue(r, out bool was);
                if (stalled) {
                    if (!stalledSince.ContainsKey(r)) stalledSince[r] = T;
                    if (!was && T - stalledSince[r] >= JamAfter) {
                        jammed[r] = true;
                        var t = r.Truth();
                        lastContact.TryGetValue(r, out var lc);
                        var f = new Dictionary<string, object> { ["since"] = stalledSince[r], ["x"] = t.X, ["y"] = t.Y, ["yaw"] = t.Yaw, ["wheel_cmd_max"] = cmd,
                            ["last_contact"] = lc.other, ["last_contact_t"] = lc.other != null ? lc.t : double.NaN, ["evaluator_only"] = true };
                        Jams.Add(new Dictionary<string, object>(f) { ["rover"] = r.D.Id, ["end"] = double.NaN });
                        Event("jam_start", r.D.Id, f);
                    }
                }
                else {
                    stalledSince.Remove(r);
                    if (was) {
                        jammed[r] = false;
                        for (int i = Jams.Count - 1; i >= 0; i--) if ((string)Jams[i]["rover"] == r.D.Id) { Jams[i]["end"] = T; break; }
                        Event("jam_end", r.D.Id, new Dictionary<string, object> { ["evaluator_only"] = true });
                    }
                }
            }
        }

        public bool IsJammed(RoverRig r) => jammed.TryGetValue(r, out bool j) && j;

        readonly HashSet<(Collider, Collider)> loggedPairs = new HashSet<(Collider, Collider)>();
        const double contactImpulseThreshold = 1e-4;
        /// <summary>Pairs a mission declares as expected (e.g. a coupled pair's mated faces).</summary>
        public Func<RoverRig, RoverRig, bool> IgnoreContact;

        // ---- logging ----

        static double Sq(double x) => x * x;
        static string N(double v) => FleetJson.Num(v);

        void LogState() {
            var b = new StringBuilder(2048);
            b.Append("{\"t\":").Append(N(T)).Append(",\"step\":").Append(Step);
            b.Append(",\"truth\":{");
            for (int i = 0; i < Rovers.Count; i++) {
                var r = Rovers[i];
                RoverTruth t = r.Truth();
                if (i > 0) b.Append(',');
                b.Append(FleetJson.Quote(r.D.Id)).Append(":{\"x\":").Append(N(t.X)).Append(",\"y\":").Append(N(t.Y)).Append(",\"z\":").Append(N(t.Z))
                    .Append(",\"yaw\":").Append(N(t.Yaw)).Append(",\"pitch\":").Append(N(t.Pitch)).Append(",\"roll\":").Append(N(t.Roll))
                    .Append(",\"vx\":").Append(N(t.Body.Vx)).Append(",\"vy\":").Append(N(t.Body.Vy)).Append(",\"wz\":").Append(N(t.Body.Wz)).Append('}');
            }
            b.Append("},\"estimate\":{");
            for (int i = 0; i < Rovers.Count; i++) {
                var r = Rovers[i];
                if (i > 0) b.Append(',');
                var o = r.Odometry;
                b.Append(FleetJson.Quote(r.D.Id)).Append(":{\"x\":").Append(N(o.X)).Append(",\"y\":").Append(N(o.Y)).Append(",\"yaw\":").Append(N(o.Yaw))
                    .Append(",\"vx\":").Append(N(o.LastTwist.Vx)).Append(",\"vy\":").Append(N(o.LastTwist.Vy)).Append(",\"wz\":").Append(N(o.LastTwist.Wz))
                    .Append(",\"gyro_wz\":").Append(N(r.Gyro.Last)).Append(",\"ticks\":[");
                for (int k = 0; k < r.Encoders.Ticks.Length; k++) { if (k > 0) b.Append(','); b.Append(r.Encoders.Ticks[k]); }
                b.Append("]}");
            }
            b.Append("},\"actuation\":{");
            for (int i = 0; i < Rovers.Count; i++) {
                var r = Rovers[i];
                if (i > 0) b.Append(',');
                b.Append(FleetJson.Quote(r.D.Id)).Append(":{\"cmd\":[").Append(N(r.Command.Vx)).Append(',').Append(N(r.Command.Vy)).Append(',').Append(N(r.Command.Wz)).Append(']');
                if (r.Plant is MecanumPlant p)
                    b.Append(",\"wheel_cmd\":").Append(Arr(p.WheelCommand)).Append(",\"wheel\":").Append(Arr(p.WheelSpeed))
                        .Append(",\"slip\":").Append(Arr(p.SlipFactor)).Append(",\"normal_load\":").Append(Arr(p.NormalLoad));
                else {
                    var tp = r.Tractive;
                    var f = new double[2 * tp.N];
                    for (int k = 0; k < tp.N; k++) { f[2 * k] = tp.Fx[k]; f[2 * k + 1] = tp.Fy[k]; }
                    b.Append(",\"wheel_cmd\":").Append(Arr(tp.WheelCommand)).Append(",\"wheel\":").Append(Arr(tp.WheelSpeed))
                        .Append(",\"normal_load\":").Append(Arr(tp.NormalLoad)).Append(",\"force_xy\":").Append(Arr(f))
                        .Append(",\"health\":[");
                    for (int k = 0; k < tp.N; k++) { if (k > 0) b.Append(','); b.Append(FleetJson.Quote(tp.Health[k] == WheelHealth.Weak ? "weak=" + N(tp.Gain[k]) : tp.Health[k].ToString())); }
                    b.Append(']');
                }
                b.Append(",\"traction_scale\":").Append(N(r.Plant.TractionScale)).Append(",\"step\":").Append(FleetJson.Quote(StepOf(r))).Append('}');
            }
            b.Append('}');
            if (M != null) { b.Append(",\"mission\":{"); M.LogState(b); b.Append('}'); }
            b.Append('}');
            states.WriteLine(b.ToString());
        }

        static string Arr(double[] a) {
            var b = new StringBuilder("[");
            for (int i = 0; i < a.Length; i++) { if (i > 0) b.Append(','); b.Append(N(a[i])); }
            return b.Append(']').ToString();
        }

        string EventLine(string kind, string rover, Dictionary<string, object> fields) {
            var d = new Dictionary<string, object> { ["t"] = T, ["step"] = Step, ["kind"] = kind, ["rover"] = rover };
            foreach (var kv in fields) d[kv.Key] = kv.Value;
            return FleetJson.Write(d);
        }

        public void Event(string kind, string rover, Dictionary<string, object> fields) {
            string line = EventLine(kind, rover, fields);
            events?.WriteLine(line);
            EventRaised?.Invoke(kind, rover, fields);
            var te = new Dictionary<string, object> { ["kind"] = kind, ["t"] = Math.Round(T, 4) };
            if (rover.Length > 0) te["agent"] = rover;
            if (fields.TryGetValue("step", out object s) && s is string) te["step"] = s;
            trajEvents.Add(FleetJson.Write(te));
        }

        void SampleTraj() {
            int k = 0;
            foreach (var r in Rovers) {
                RoverTruth t = r.Truth();
                foreach (double v in new[] { t.X, t.Y, t.Z, t.Yaw, t.Pitch, t.Roll, t.Speed }) trajColumns[k++].Values.Add(v);
            }
        }

        void WriteTraj() {
            double hz = 1.0 / (trajEvery * dt);
            var ids = new List<object>();
            foreach (var r in Rovers) ids.Add(r.D.Id);
            var header = new Dictionary<string, object> {
                ["build"] = new Dictionary<string, object> { ["recorder"] = "crane.fleet", ["unity_version"] = Application.unityVersion, ["source_commit"] = Opt.SourceCommit },
                ["course"] = new Dictionary<string, object> { ["family"] = Spec.S["scenario_id"].String, ["seed"] = Opt.Seed.ToString(CultureInfo.InvariantCulture), ["mode"] = Opt.Variant },
                ["frame"] = new Dictionary<string, object> { ["type"] = "enu", ["origin"] = "world" },
                ["hz"] = hz, ["t0_s"] = 0.0, ["outcome"] = "unknown", ["pose_dims"] = "6dof",
                ["robot"] = new Dictionary<string, object> { ["model"] = Spec.P["profile_id"].String, ["agents"] = ids },
                ["schema"] = CraneTraj.Schema, ["scene"] = Spec.S["scenario_id"].String, ["seed"] = Opt.Seed.ToString(CultureInfo.InvariantCulture),
                ["ticks"] = trajColumns[0].Values.Count, ["run_id"] = RunId
            };
            byte[] bytes = CraneTraj.Encode(FleetJson.Write(header), hz, trajColumns, trajEvents);
            File.WriteAllBytes(Path.Combine(Dir, "trajectory.ctraj"), bytes);
        }

        void WriteManifest(bool final) {
            var rovers = new List<object>();
            foreach (var r in Rovers)
                rovers.Add(new Dictionary<string, object> { ["id"] = r.D.Id, ["role"] = r.D.Role, ["namespace"] = r.D.Namespace, ["mass"] = r.D.Mass,
                    ["render"] = r.RenderSource, ["mesh_manifest"] = r.D.MeshManifestPath, ["mesh_manifest_present"] = r.D.MeshManifestPresent,
                    ["plant_model"] = r.D.PlantModel, ["module_type"] = r.D.ModuleType, ["type_hash"] = r.D.TypeHash,
                    ["wheels"] = r.D.WheelUnits.ConvertAll(w => (object)(w.Name + ":" + w.Kind.ToString().ToLowerInvariant())) });
            var m = new Dictionary<string, object> {
                ["schema"] = "crane.fleet_run_manifest/v0",
                ["run_id"] = RunId, ["seed"] = Opt.Seed, ["variant"] = Opt.Variant,
                ["git_commit"] = Opt.SourceCommit, ["git_dirty"] = Opt.SourceDirty,
                ["launcher"] = Opt.Launcher, ["unity_version"] = Application.unityVersion, ["platform"] = Application.platform.ToString(),
                ["graphics_device"] = SystemInfo.graphicsDeviceType.ToString(),
                ["profile"] = new Dictionary<string, object> { ["path"] = Spec.Profile.Source, ["sha256"] = Spec.Profile.Sha256, ["id"] = Spec.P["profile_id"].String },
                ["scenario"] = new Dictionary<string, object> { ["path"] = Spec.Scenario.Source, ["sha256"] = Spec.Scenario.Sha256, ["id"] = Spec.S["scenario_id"].String },
                ["overlay"] = new Dictionary<string, object> { ["path"] = Spec.Overlay.Source, ["sha256"] = Spec.Overlay.Sha256 },
                ["parameters"] = new Dictionary<string, object> {
                    ["dt"] = dt, ["state_hz"] = 1.0 / (stateEvery * dt), ["traj_hz"] = 1.0 / (trajEvery * dt), ["max_duration_s"] = maxDuration,
                    ["doorway_width"] = World != null ? World.DoorwayWidth : double.NaN, ["floor"] = Spec.S["arena.floor"].String,
                    ["controller_input"] = M != null ? M.ControllerInput : "evaluator_truth (qualification script only)",
                    ["mission"] = M != null ? M.GetType().Name : "qualification",
                    ["failures"] = Opt.Failures ?? "",
                    ["physics_scene"] = Phys != null ? Phys.Describe : "default",
                    ["estimate_source"] = "wheel encoders (ideal kinematics) + gyro heading; initialised from truth at t=0",
                    ["physics"] = "Unity PhysX Rigidbody, simulationMode=Script, frictionless wheel spheres, plant force at CoM"
                },
                ["files"] = new Dictionary<string, object> {
                    ["states"] = "states.jsonl: per row t, step, truth{id:{pose, body twist}} (evaluator only), estimate{id:{odometry, gyro, ticks}} (rover's own), actuation{id:{cmd, wheels, slip, loads}}",
                    ["events"] = "events.jsonl", ["trajectory"] = "trajectory.ctraj (crane-traj/1, truth, ENU, agents = rover ids)"
                },
                ["rovers"] = rovers,
                ["labels"] = Labels(),
                ["complete"] = final, ["end_reason"] = EndReason, ["t_end"] = T, ["steps"] = Step,
                ["wall_start_utc"] = wallStart.ToString("o", CultureInfo.InvariantCulture),
                ["wall_seconds"] = (DateTime.UtcNow - wallStart).TotalSeconds,
                ["results"] = results
            };
            File.WriteAllText(Path.Combine(Dir, "manifest.json"), FleetJson.Write(m, true) + "\n");
        }
    }
}
