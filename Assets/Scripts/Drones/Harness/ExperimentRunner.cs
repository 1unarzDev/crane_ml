// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using Crane.Drones.Autonomy;
using Sim.Physics.Aerial;

namespace Crane.Drones.Harness {
    /// <summary>Loaded worlds, vehicle, brake table and guard shared by the runs of one process.</summary>
    public sealed class HarnessContext {
        public readonly string RepoRoot; public readonly SplitGuard Guard;
        private readonly Dictionary<string, (ForestManifestWorld w, ForestThinning thin, string sha)> worlds = new Dictionary<string, (ForestManifestWorld, ForestThinning, string)>();
        private readonly Dictionary<string, (VehicleSpecV2 spec, double[] vs, double[] ds, string sha)> vehicles = new Dictionary<string, (VehicleSpecV2, double[], double[], string)>();
        public Action<string> Log = Console.WriteLine;

        public HarnessContext(string repoRoot, SplitGuard guard) { RepoRoot = repoRoot; Guard = guard; }

        public string Abs(string p) => Path.IsPathRooted(p) ? p : Path.GetFullPath(Path.Combine(RepoRoot, p));

        public (ForestManifestWorld w, ForestThinning thin, string sha) World(string manifest) {
            if (!worlds.TryGetValue(manifest, out var e)) {
                string p = Abs(manifest);
                var bytes = File.ReadAllBytes(p);
                string sha; using (var h = SHA256.Create()) sha = BitConverter.ToString(h.ComputeHash(bytes)).Replace("-", "").ToLowerInvariant();
                var w = ForestManifestWorld.FromJson(Encoding.UTF8.GetString(bytes));
                e = (w, new ForestThinning(w), sha); worlds[manifest] = e;
                Log($"[harness] loaded {manifest} seed={w.Seed} capsules={w.Count} sha256={sha.Substring(0, 12)}");
            }
            return e;
        }

        public (VehicleSpecV2 spec, double[] vs, double[] ds, string sha) Vehicle(string profile) {
            if (!vehicles.TryGetValue(profile, out var e)) {
                string p = Abs(profile);
                var prof = CraneAerialProfile.LoadFile(p, true, true);
                var spec = prof.Vehicle; spec.GroundHeight = -1000.0;   // as the lead's harness: terrain contact handled by metrics
                var bt = StopRule.SimulateBrakeTable(spec);
                string sha; using (var h = SHA256.Create()) sha = BitConverter.ToString(h.ComputeHash(File.ReadAllBytes(p))).Replace("-", "").ToLowerInvariant();
                e = (spec, bt.vs, bt.ds, sha); vehicles[profile] = e;
            }
            return e;
        }

        private readonly Dictionary<string, List<MissionSpec>> catalogs = new Dictionary<string, List<MissionSpec>>();
        private readonly Dictionary<string, List<MissionSpec>> fixtures = new Dictionary<string, List<MissionSpec>>();
        /// <summary>Register an in-memory world (tests, synthetic fixtures) under a pseudo manifest key.</summary>
        public void Register(string key, ForestManifestWorld w, List<MissionSpec> missions) { worlds[key] = (w, new ForestThinning(w), "fixture:" + key); fixtures[key] = missions; }
        public List<MissionSpec> Missions(string manifest, int cross, bool corridor, double cruiseAgl) {
            if (fixtures.TryGetValue(manifest, out var fx)) return fx;
            string k = manifest + "|" + cross + "|" + corridor + "|" + cruiseAgl.ToString("R", CultureInfo.InvariantCulture);
            if (!catalogs.TryGetValue(k, out var l)) {
                var w = World(manifest).w; var thin = World(manifest).thin; thin.Restore();
                l = MissionCatalog.Build(w, new AutonomyConfig { CruiseAgl = cruiseAgl }, cross, corridor); catalogs[k] = l;
            }
            return l;
        }
        public List<string> MissionNames(string manifest, int cross, bool corridor) { var o = new List<string>(); foreach (var m in Missions(manifest, cross, corridor, new AutonomyConfig().CruiseAgl)) o.Add(m.Name); return o; }
    }

    /// <summary>What the probe (second, replay pass) should watch and where to stop.</summary>
    public sealed class ReplayTarget {
        public long EndTick = -1; public string Outcome, CollisionObject; public Vec3 ContactP; public bool HasContact;
    }

    /// <summary>One executed run: row for results.jsonl plus what the labeller needs.</summary>
    public sealed class RunOutput {
        public Dictionary<string, object> Row; public MissionResult Result; public string Outcome; public long EndTick;
        public List<Vec3> Truth = new List<Vec3>(); public RunRecorder Rec; public string Label = "", LabelDetail = "";
        public Dictionary<string, object> Evidence, Bundle;
    }

    /// <summary>
    /// Executes RunSpecs on the .NET manifest world through the lead's AutonomousMission (Core is used as-is, through its
    /// public surface only): faults via frame callbacks and config, wind via StepPlant, comms via LossyCommsLink, N drones in
    /// lockstep, failure labelling and replay bundles via a deterministic second pass.
    /// </summary>
    public sealed class ExperimentRunner {
        public readonly HarnessContext Ctx;
        public ExperimentRunner(HarnessContext ctx) { Ctx = ctx; }

        sealed class Drone {
            public string Name; public AutonomousMission M; public RunRecorder Rec; public FaultInjector Faults; public WindField Wind;
            public ProbeRayWorld Probe; public bool Aborted; public string AbortReason = ""; public long LastGcsHbNs; public int Seq;
            public StreamWriter State; public double MaxLocalErr; public double WindSq; public long WindN;
            public List<long> DeadlineTicks = new List<long>(), WatchdogTicks = new List<long>(); public int LastDeadline, LastWd, LastCycles;
            public List<Dictionary<string, object>> Cycles = new List<Dictionary<string, object>>();
            public List<Vec3> Truth = new List<Vec3>(), Est = new List<Vec3>();
            public long EndTick = -1;
        }

        public RunOutput Run(RunSpec rs, string runDir, string logs, ReplayTarget probe = null) {
            var ctx = Ctx; var (world, thin, sha) = ctx.World(rs.Manifest); var veh = ctx.Vehicle(rs.Profile);
            var baseCfg = rs.BuildConfig();
            double keepS = rs.Dr.TryGetValue("forest_keep_stems", out var ks) && ks is double a ? a : 1.0;
            double keepB = rs.Dr.TryGetValue("forest_keep_branches", out var kb) && kb is double b ? b : 1.0;
            var catalog = ctx.Missions(rs.Manifest, rs.Cross, rs.Corridor, baseCfg.CruiseAgl);
            thin.Apply(keepS, keepB, (ulong)rs.ForestSeed * 1000003UL + rs.RunSeed * 7919UL + (ulong)rs.DrIndex);
            try { return RunInner(rs, runDir, logs, probe, world, sha, veh, baseCfg, catalog, thin); }
            finally { thin.Restore(); }
        }

        RunOutput RunInner(RunSpec rs, string runDir, string logs, ReplayTarget probe, ForestManifestWorld world, string sha,
                (VehicleSpecV2 spec, double[] vs, double[] ds, string sha) veh, AutonomyConfig baseCfg, List<MissionSpec> catalog, ForestThinning thin) {
            MissionSpec Find(string n) { foreach (var m in catalog) if (m.Name == n) return m; throw new ArgumentException($"mission {n} not in catalog of {rs.Manifest}"); }
            var names = new List<string> { rs.Mission }; if (rs.PeerMissions != null) names.AddRange(rs.PeerMissions);
            var drones = new List<Drone>();
            LossyCommsLink comms = rs.Comms != null ? new LossyCommsLink(rs.RunSeed ^ 0xC0FFEEUL, rs.Comms) : null;
            bool full = logs == "full" && runDir != null;
            for (int k = 0; k < names.Count; k++) {
                var cfg = baseCfg.Clone(); if (k > 0) cfg.RunSeed = rs.RunSeed + 1000UL * (ulong)k;
                var prior = new ManifestPriorMap(world, cfg, cfg.RunSeed);
                var rule = new StopRule(veh.vs, veh.ds, cfg.TauPrior, cfg.Margin, cfg.AgeWindow, cfg.AgeQuantile);
                var d = new Drone { Name = "d" + k, Rec = new RunRecorder { Drone = "d" + k } };
                IRayWorld sensorWorld = world;
                if (probe != null && k == 0) { d.Probe = new ProbeRayWorld(world); sensorWorld = d.Probe; }
                d.M = new AutonomousMission(Find(names[k]), cfg, veh.spec, sensorWorld, world, world, prior, d.Rec, rule) { PriorForBelief = prior };
                if (d.Probe != null && probe.HasContact) d.Probe.Watch = WatchSet(world, probe.CollisionObject, probe.ContactP, d.M.BodyRadius);
                d.Faults = new FaultInjector(k == 0 ? rs.Faults : null, cfg);
                var dr = d; var fi = d.Faults;
                d.M.DepthAcquired = f => { long av = f.AvailNs; fi.OnFrame("depth", f.AcqNs * 1e-9, ref av); f.AvailNs = av; };
                d.M.LidarCompleted = s => { long av = s.AvailNs; fi.OnFrame("lidar", s.StartNs * 1e-9, ref av); s.AvailNs = av; };
                d.Wind = new WindField(rs.WindMean, rs.WindDirDeg, rs.WindW20, cfg.RunSeed);
                if (d.Wind.Active) {
                    var plant = d.M.Plant; double dt = cfg.TickNs * 1e-9; var wf = d.Wind;
                    d.M.StepPlant = () => {
                        var st = plant.State; double agl = st.P[2] - world.GroundHeight(st.P[0], st.P[1]);
                        var wv = wf.Step(dt, agl, st.V); plant.MeanWindWorld = wv;
                        dr.WindSq += wv[0] * wv[0] + wv[1] * wv[1]; dr.WindN++;
                        plant.Advance(dt);
                    };
                }
                if (full) {
                    string dir = k == 0 ? runDir : Path.Combine(runDir, "peer" + k); Directory.CreateDirectory(dir);
                    d.State = new StreamWriter(Path.Combine(dir, "state.csv"));
                    var hdr = new StringBuilder("tick,sub,t_ns,px,py,pz,qw,qx,qy,qz,vx,vy,vz,wx,wy,wz,ax,ay,az");
                    int nr = veh.spec.Rotors.Length; for (int i = 0; i < nr; i++) hdr.Append(",rpm" + i); for (int i = 0; i < nr; i++) hdr.Append(",cmd" + i);
                    hdr.Append(",batt_v,batt_a,batt_soc"); d.State.WriteLine(hdr.ToString());
                }
                drones.Add(d);
            }
            if (comms != null) { var map = new Dictionary<string, RunRecorder>(); foreach (var d in drones) map[d.Name] = d.Rec; comms.RecorderFor = n => map.TryGetValue(n, out var r) ? r : (IAutonomyRecorder)drones[0].Rec; }
            double hbHz = HJson.Num(rs.Comms, "heartbeat_hz", 2.0), failsafe = HJson.Num(rs.Comms, "failsafe_timeout_s", 0.0);
            long hbPeriod = (long)Math.Round(1e9 / Math.Max(hbHz, 0.1));
            long tick = 0; double minSep = double.PositiveInfinity; var ci = CultureInfo.InvariantCulture;
            var tickNs = baseCfg.TickNs;
            while (true) {
                bool any = false;
                long tNs = tick * tickNs;
                // ---- comms: GCS heartbeat to every drone, drones broadcast estimated pose
                if (comms != null && tNs % hbPeriod == 0) {
                    foreach (var d in drones) if (!d.M.Done && !d.Aborted) {
                        comms.Send(new CommsMessage { From = "gcs", To = d.Name, Kind = "heartbeat", SentNs = tNs, Seq = d.Seq });
                        var p = d.M.Estimator.Current.P;
                        comms.Send(new CommsMessage { From = d.Name, To = "*", Kind = "pose", SentNs = tNs, Seq = d.Seq++, Payload = Encoding.ASCII.GetBytes(string.Format(ci, "{0:0.00},{1:0.00},{2:0.00}", p.X, p.Y, p.Z)) });
                    }
                }
                foreach (var d in drones) {
                    if (d.M.Done || d.Aborted) continue;
                    any = true;
                    if (comms != null) {
                        foreach (var m in comms.Receive(d.Name, tNs)) if (m.From == "gcs") d.LastGcsHbNs = tNs;
                        if (failsafe > 0 && (tNs - d.LastGcsHbNs) * 1e-9 > failsafe) {
                            d.Aborted = true; d.AbortReason = "datalink_loss_failsafe"; d.EndTick = d.M.Tick;
                            d.Rec.Event(tNs, "autonomy_arbiter", new Dictionary<string, object> { ["action"] = "abort", ["reason"] = "datalink_loss_failsafe", ["since_hb_s"] = (tNs - d.LastGcsHbNs) * 1e-9 });
                            continue;
                        }
                    }
                    d.Faults.Tick(d.M.TNs * 1e-9);
                    if (d.Probe != null) d.Probe.Tick = d.M.Tick;
                    d.M.Step();
                    AfterStep(d, probe, world);
                    if (d.M.Done) d.EndTick = d.M.Tick;
                    if (probe != null && probe.EndTick >= 0 && d == drones[0] && d.M.Tick > probe.EndTick && !d.M.Done) { d.Aborted = true; d.AbortReason = "replay_stop"; d.EndTick = d.M.Tick; }
                }
                if (comms != null) comms.ExpireBroadcasts(tNs);
                for (int i = 0; i < drones.Count; i++) for (int j = i + 1; j < drones.Count; j++) {
                    var pi = drones[i].M.Plant.State.P; var pj = drones[j].M.Plant.State.P;
                    minSep = Math.Min(minSep, Math.Sqrt((pi[0] - pj[0]) * (pi[0] - pj[0]) + (pi[1] - pj[1]) * (pi[1] - pj[1]) + (pi[2] - pj[2]) * (pi[2] - pj[2])));
                }
                tick++;
                if (!any) break;
            }
            foreach (var d in drones) { d.Faults.Restore(); d.State?.Dispose(); }
            var d0 = drones[0]; var r = d0.M.Result;
            string outcome = d0.Aborted && d0.AbortReason != "replay_stop" ? "comms_abort" : r.Outcome;
            var o = new RunOutput { Result = r, Outcome = outcome, EndTick = d0.EndTick, Rec = d0.Rec, Truth = d0.Truth };
            var row = new Dictionary<string, object>();
            foreach (var kv in rs.ToDict()) row[kv.Key] = kv.Value;
            row["harness"] = "crane_harness_dotnet_manifest_world"; row["forest_manifest_sha256"] = sha; row["physics_params_sha256"] = veh.sha;
            row["config_hash"] = HJson.Fnv64(d0.M.C.Canonical());
            row["outcome"] = outcome; row["end_tick"] = d0.EndTick;
            foreach (var kv in r.ToDict()) row["r_" + kv.Key] = kv.Value;
            if (d0.Aborted && outcome == "comms_abort") { row["r_Outcome"] = outcome; row["r_FailureCause"] = d0.AbortReason; row["r_TimeS"] = d0.M.TNs * 1e-9; row["r_Success"] = false; }
            row["max_local_err_m"] = d0.MaxLocalErr; row["wind_rms_mps"] = d0.WindN > 0 ? Math.Sqrt(d0.WindSq / d0.WindN) : 0.0;
            row["faults_blackout_frames"] = d0.Faults.BlackedOut; row["faults_delayed_frames"] = d0.Faults.Delayed;
            row["thin_dropped_owners"] = thin.DroppedOwners; row["thin_dropped_branches"] = thin.DroppedBranches;
            row["deadline_miss_ticks"] = d0.DeadlineTicks.Count; row["watchdog_trip_ticks"] = d0.WatchdogTicks.Count;
            if (comms != null) { row["comms_sent"] = comms.Sent; row["comms_dropped"] = comms.Dropped; row["comms_delivered"] = comms.Delivered; }
            if (drones.Count > 1) {
                row["min_interdrone_sep_m"] = minSep;
                var peers = new List<object>();
                for (int k = 1; k < drones.Count; k++) { var pr = drones[k].M.Result; peers.Add(new Dictionary<string, object> { ["mission"] = names[k], ["outcome"] = drones[k].Aborted ? "comms_abort" : pr.Outcome, ["collision"] = pr.Collision, ["time_s"] = pr.TimeS }); }
                row["peers"] = peers;
            }
            o.Row = row;
            if (probe != null) BuildEvidenceAndBundle(o, d0, probe, world, rs);
            if (runDir != null && logs != "none") WriteLogs(runDir, rs, drones, sha, veh.sha, full, world);
            return o;
        }

        void AfterStep(Drone d, ReplayTarget probe, ForestManifestWorld world) {
            var m = d.M; var st = m.Plant.State; long tNs = m.TNs;
            var p = Vec3.FromArray(st.P); d.Truth.Add(p);
            if (m.Tick % 10 == 0) d.Est.Add(m.Estimator.Current.P);
            double le = m.Estimator.LocalError.Norm; if (le > d.MaxLocalErr) d.MaxLocalErr = le;
            if (m.Result.DeadlineMisses != d.LastDeadline) { d.LastDeadline = m.Result.DeadlineMisses; d.DeadlineTicks.Add(m.Tick); }
            if (m.Arbiter.WatchdogTrips != d.LastWd) { d.LastWd = m.Arbiter.WatchdogTrips; d.WatchdogTicks.Add(m.Tick); }
            if (d.Probe != null) d.Probe.EndTick(tNs * 1e-9);
            if (d.State != null) {
                var sb = new StringBuilder(); var ci = CultureInfo.InvariantCulture;
                long tick = m.Tick;   // state after the plant step at t = tick * dt
                sb.Append(tick).Append(",0,").Append(tNs);
                foreach (var x in st.P) sb.Append(',').Append(x.ToString("R", ci));
                double qs = st.Q[0] < 0 ? -1 : 1; foreach (var x in st.Q) sb.Append(',').Append((qs * x).ToString("R", ci));
                foreach (var x in st.V) sb.Append(',').Append(x.ToString("R", ci));
                foreach (var x in st.Omega) sb.Append(',').Append(x.ToString("R", ci));
                sb.Append(",nan,nan,nan");
                foreach (var w in st.RotorSpeed) sb.Append(',').Append((w * 60.0 / (2 * Math.PI)).ToString("R", ci));
                var spec = m.Vehicle;
                for (int i = 0; i < st.RotorSpeed.Length; i++) sb.Append(',').Append((m.Plant.RotorCommand[i] / spec.Rotors[i].OmegaMax).ToString("R", ci));
                sb.Append(",nan,nan,").Append(st.Soc.ToString("R", ci));
                d.State.WriteLine(sb.ToString());
            }
            if (d.Probe != null && m.Result.Cycles != d.LastCycles) {
                d.LastCycles = m.Result.Cycles;
                var e = m.Estimator.LocalError;   // truth - estimate
                var est = m.Estimator.Current;
                m.Window.Index(est.P, out int w0);
                double bel = w0 >= 0 ? m.Window.Dist[w0] : double.PositiveInfinity;
                bool mapped = d.Probe.Watch.Count > 0 && Mapped(d.Probe.Watch, m.Planner.Name == "reactive" ? m.EphemeralMap : m.Map, world, e, probe.ContactP);
                d.Cycles.Add(new Dictionary<string, object> { ["t"] = tNs * 1e-9, ["mapped"] = mapped, ["err"] = e.ToArray(), ["belief_dist"] = bel, ["mode"] = m.Arbiter.Mode,
                    ["tau"] = m.Rule.Tau, ["speed"] = new Vec3(st.V[0], st.V[1], st.V[2]).Norm, ["truth_clear"] = m.LastHardClearance });
            }
        }

        /// <summary>True if any occupied voxel lies within (r + 0.1 m) of the watched capsules' axes near the contact,
        /// evaluated in the map (local, estimated) frame: local = world - (truth - estimate).</summary>
        static bool Mapped(HashSet<int> watch, SparseVoxelMap map, ForestManifestWorld w, Vec3 e, Vec3 contact) {
            double res = map.Resolution;
            foreach (int s in watch) {
                var a = new Vec3(w.Ax[s], w.Ay[s], w.Az[s]); var b = new Vec3(w.Bx[s], w.By[s], w.Bz[s]); double L = (b - a).Norm, r = Math.Max(w.R[s], 0);
                int n = Math.Max(1, (int)Math.Ceiling(L / 0.1));
                int rad = (int)Math.Ceiling((r + 0.1) / res);
                for (int i = 0; i <= n; i++) {
                    var q = Vec3.Lerp(a, b, (double)i / n); if ((q - contact).Norm > 1.5) continue;
                    var ql = q - e;
                    for (int dx = -rad; dx <= rad; dx++) for (int dy = -rad; dy <= rad; dy++) for (int dz = -rad; dz <= rad; dz++) {
                        var v = new Vec3(ql.X + dx * res, ql.Y + dy * res, ql.Z + dz * res);
                        if ((v - ql).Norm > r + 0.1 + 0.5 * res) continue;
                        if (map.Occupied(v)) return true;
                    }
                }
            }
            return false;
        }

        /// <summary>Watched set for a collision: the struck capsule plus same-owner capsules within body radius + 0.3 m of the contact.</summary>
        public static HashSet<int> WatchSet(ForestManifestWorld w, string collisionObject, Vec3 contact, double bodyRadius) {
            var hs = new HashSet<int>();
            int k = collisionObject.LastIndexOf("/seg", StringComparison.Ordinal);
            if (k < 0 || !int.TryParse(collisionObject.Substring(k + 4), NumberStyles.Integer, CultureInfo.InvariantCulture, out int seg)) return hs;
            hs.Add(seg); int owner = w.Owner[seg]; double reach = bodyRadius + 0.3;
            w.ForEachCapsuleIn(contact.X - 1.5, contact.X + 1.5, contact.Y - 1.5, contact.Y + 1.5, contact.Z - 1.5, contact.Z + 1.5, s => {
                if (w.Owner[s] == owner && w.R[s] > 0 && w.SegDist(s, contact.X, contact.Y, contact.Z) - w.R[s] < reach) hs.Add(s);
            });
            return hs;
        }

        // ------------------------------------------------------------------ labelling + replay bundle

        public const double ThinD = 0.04;

        void BuildEvidenceAndBundle(RunOutput o, Drone d, ReplayTarget probe, ForestManifestWorld w, RunSpec rs) {
            var m = d.M; var r = m.Result; var C = m.C; double tEnd = d.EndTick * C.TickNs * 1e-9;
            var ev = new Dictionary<string, object> { ["t_end_s"] = tEnd, ["outcome"] = o.Outcome };
            string label, detail = "";
            bool recentStall(double tc) {
                foreach (var tk in d.DeadlineTicks) if (tk * C.TickNs * 1e-9 > tc - 1.0) return true;
                foreach (var tk in d.WatchdogTicks) if (tk * C.TickNs * 1e-9 > tc - 1.0) return true;
                foreach (var f in rs.Faults) { string k = HJson.Text(f, "kind", ""); if ((k == "sensor_delay" || k == "sensor_blackout") && HJson.Num(f, "t0", 0) < tc && HJson.Num(f, "t1", 1e9) > tc - 1.0) return true; }
                return false;
            }
            if (o.Outcome == "comms_abort") { label = "comms"; detail = "datalink-loss failsafe aborted the mission"; }
            else if (o.Outcome == "collision") {
                double tc = r.CollisionTimeS, vrec = 0;
                int n = d.Truth.Count; for (int i = Math.Max(1, n - 400); i < n; i++) vrec = Math.Max(vrec, (d.Truth[i] - d.Truth[i - 1]).Norm / (C.TickNs * 1e-9));
                double tNeed = tc - m.Rule.Dist(vrec) / Math.Max(vrec, 0.5);
                bool mappedByNeed = false, mappedAtContact = false; double tFirstMapped = double.NaN; Vec3 eFirst = new Vec3(), eLast = new Vec3(); double belLast = double.NaN, truthLast = double.NaN; string modeLast = "";
                foreach (var cy in d.Cycles) {
                    double t = (double)cy["t"]; bool mp = (bool)cy["mapped"];
                    if (mp && double.IsNaN(tFirstMapped)) { tFirstMapped = t; eFirst = Vec3.FromArray((double[])cy["err"]); }
                    if (t <= tNeed && mp) mappedByNeed = true;
                    if (t <= tc) { mappedAtContact = mp; eLast = Vec3.FromArray((double[])cy["err"]); belLast = (double)cy["belief_dist"]; truthLast = (double)cy["truth_clear"]; modeLast = (string)cy["mode"]; }
                }
                // onset of the last no-safe-maneuver brake within 1.5 s before contact
                Dictionary<string, object> nsOn = null; double tOn = double.NaN, vOn = 0, clrOn = double.NaN, belOn = double.NaN;
                for (int i = 0; i < d.Cycles.Count; i++) {
                    var cy = d.Cycles[i]; double t = (double)cy["t"]; if (t > tc) break;
                    bool ns = (string)cy["mode"] == "no_safe_maneuver", prev = i > 0 && (string)d.Cycles[i - 1]["mode"] == "no_safe_maneuver";
                    if (ns && !prev && t >= tc - 1.5) { nsOn = cy; tOn = t; vOn = (double)cy["speed"]; clrOn = (double)cy["truth_clear"]; belOn = (double)cy["belief_dist"]; }
                }
                var pr = d.Probe; bool detByNeed = pr.FirstDetectS <= tNeed, viewByNeed = pr.FirstViewS <= tNeed;
                double dObj = double.PositiveInfinity; foreach (int s in pr.Watch) if (w.R[s] > 0) dObj = Math.Min(dObj, 2 * w.R[s]);
                int segId = -1; foreach (int s in pr.Watch) { segId = s; break; }
                string coll = r.CollisionObject; int k = coll.LastIndexOf("/seg", StringComparison.Ordinal); if (k >= 0) int.TryParse(coll.Substring(k + 4), out segId);
                double dStruck = segId >= 0 ? 2 * w.R[segId] : dObj; bool thin = dStruck < ThinD;
                double rInfl = m.BodyRadius + C.InflateExtra + 0.5 * C.MapRes;
                double dE = (eLast - eFirst).Norm; bool stall = recentStall(tc);
                if (mappedByNeed) {
                    if (stall) { label = "latency_stall"; detail = "mapped in time, deadline miss / watchdog / perception stall in the last 1 s"; }
                    else if (!mappedAtContact) { label = thin ? "thin_branch_miss" : "map_lag"; detail = "mapped in time, then carved free before contact"; }
                    else if (nsOn != null && vOn > 1.0 && clrOn < m.Rule.Brake(vOn)) { label = "brake_infeasible"; detail = $"no-safe-maneuver brake began {tc - tOn:0.00} s before contact at {vOn:0.0} m/s with {clrOn:0.00} m true clearance < brake distance {m.Rule.Brake(vOn):0.00} m (belief {belOn:0.00} m): an inevitable-collision state was entered while a primitive was still certified one cycle earlier; the straight-line brake cannot follow the certified curved primitive"; }
                    else if (dE > C.InflateExtra + 0.5 * C.MapRes) { label = "estimator_drift"; detail = $"local error changed {dE:0.00} m between first mapping and contact (> inflation budget {C.InflateExtra + 0.5 * C.MapRes:0.00} m)"; }
                    else if (belLast >= m.BodyRadius) { label = "geometry_graze"; detail = $"mapped and kept, but the belief put the obstacle {belLast:0.00} m (voxel centres) from the estimate at contact (body {m.BodyRadius:0.00}, inflation {rInfl:0.00}): quantization/inflation under-represents the surface"; }
                    else { label = "tracking_lag"; detail = $"belief knew the obstacle was inside the body radius ({belLast:0.00} m), the vehicle did not stop/turn in time"; }
                } else if (detByNeed) { label = stall ? "latency_stall" : "map_lag"; detail = "detected by t_need but not yet in the map"; }
                else if (viewByNeed) { label = thin ? "thin_branch_miss" : "sensor_miss"; detail = $"in view by t_need, every stereo return dropped (D = {dStruck * 100:0.0} cm) and no LiDAR return"; }
                else { label = "occlusion"; detail = "not in view (FOV or occlusion) by t_need"; }
                ev["t_contact_s"] = tc; ev["t_need_s"] = tNeed; ev["v_recent_mps"] = vrec; ev["stop_dist_m"] = m.Rule.Dist(vrec); ev["tau_s"] = m.Rule.Tau;
                ev["struck"] = coll; ev["struck_diam_m"] = dStruck; ev["thin"] = thin; ev["watch_capsules"] = pr.Watch.Count;
                ev["mapped_by_need"] = mappedByNeed; ev["mapped_at_contact"] = mappedAtContact; ev["t_first_mapped_s"] = tFirstMapped;
                ev["first_view_s"] = pr.FirstViewS; ev["first_detect_s"] = pr.FirstDetectS; ev["depth_hits"] = pr.DepthHits; ev["depth_drops"] = pr.DepthDrops; ev["lidar_hits"] = pr.LidarHits;
                ev["local_err_change_m"] = dE; ev["belief_dist_at_contact_m"] = belLast; ev["truth_clear_last_cycle_m"] = truthLast; ev["r_infl_m"] = rInfl; ev["body_radius_m"] = m.BodyRadius;
                ev["mode_at_contact"] = modeLast; ev["nosafe_onset_s"] = tOn; ev["nosafe_onset_speed_mps"] = vOn; ev["nosafe_onset_truth_clear_m"] = clrOn; ev["nosafe_onset_belief_m"] = belOn; ev["brake_dist_at_onset_m"] = nsOn != null ? m.Rule.Brake(vOn) : double.NaN; ev["recent_stall"] = stall; ev["impact_speed_mps"] = r.CollisionSpeed;
            } else if (o.Outcome == "stuck" || o.Outcome == "timeout") {
                double le = m.Estimator.LocalError.Norm;
                bool driftFault = rs.Faults.Exists(f => HJson.Text(f, "kind", "") == "drift_burst" || HJson.Text(f, "kind", "") == "align_jump");
                if (le > 1.0 || (driftFault && le > 0.5)) { label = "estimator_drift"; detail = $"stuck with local error {le:0.00} m"; }
                else if (d.WatchdogTicks.Count >= 3 && d.WatchdogTicks.Count > m.Arbiter.NoSafeStops) { label = "latency_stall"; detail = "stuck with repeated watchdog brakes"; }
                else { label = "local_minimum"; detail = $"{o.Outcome}: < {C.StuckProgressM} m progress in {C.StuckS} s; {m.Arbiter.NoSafeStops} no-safe stops, {m.Arbiter.Recoveries} recoveries"; }
                ev["local_err_m"] = le;
            } else { label = o.Outcome == "success" ? "" : "other"; }
            o.Label = label; o.LabelDetail = detail; ev["label"] = label; ev["detail"] = detail;
            o.Evidence = ev;
            o.Bundle = Bundle(o, d, w, rs, ev);
        }

        Dictionary<string, object> Bundle(RunOutput o, Drone d, ForestManifestWorld w, RunSpec rs, Dictionary<string, object> ev) {
            var m = d.M; var C = m.C; var pEnd = d.Truth.Count > 0 ? d.Truth[d.Truth.Count - 1] : m.Spec.Start; var e = m.Estimator.LocalError;
            double half = 7.0, z0 = pEnd.Z - 0.7, z1 = pEnd.Z + 0.7, res = m.Map.Resolution;
            var occ = new List<object>(); var map = m.Planner.Name == "reactive" ? m.EphemeralMap : m.Map;
            for (double x = pEnd.X - half; x <= pEnd.X + half; x += res) for (double y = pEnd.Y - half; y <= pEnd.Y + half; y += res) {
                bool any = false; double zz = 0;
                for (double z = z0; z <= z1; z += res) { var ql = new Vec3(x, y, z) - e; if (map.Occupied(ql)) { any = true; zz = z; break; } }
                if (any) occ.Add(new[] { Math.Round(x, 3), Math.Round(y, 3), Math.Round(zz, 3) });
            }
            var caps = new List<object>();
            w.ForEachCapsuleIn(pEnd.X - half, pEnd.X + half, pEnd.Y - half, pEnd.Y + half, z0, z1, s => {
                if (w.R[s] <= 0) return;
                if (Math.Max(w.Az[s], w.Bz[s]) + w.R[s] < z0 || Math.Min(w.Az[s], w.Bz[s]) - w.R[s] > z1) return;
                caps.Add(new Dictionary<string, object> { ["id"] = s, ["owner"] = w.OwnerName(s), ["a"] = new double[] { w.Ax[s], w.Ay[s], w.Az[s] }, ["b"] = new double[] { w.Bx[s], w.By[s], w.Bz[s] }, ["r"] = (double)w.R[s] });
            });
            var traj = new List<object>(); for (int i = 0; i < d.Truth.Count; i += 10) traj.Add(new[] { Math.Round(d.Truth[i].X, 3), Math.Round(d.Truth[i].Y, 3), Math.Round(d.Truth[i].Z, 3) });
            var est = new List<object>(); foreach (var q in d.Est) { est.Add(new[] { Math.Round(q.X, 3), Math.Round(q.Y, 3), Math.Round(q.Z, 3) }); }
            long tEndNs = d.EndTick * C.TickNs; var recent = new List<object>();
            foreach (var x in d.Rec.Events) if (x.t >= tEndNs - 3_000_000_000L && x.kind != "autonomy_sensor") { var f = new Dictionary<string, object>(x.f) { ["t_ns"] = x.t, ["kind"] = x.kind }; f.Remove("config"); recent.Add(f); }
            var lastPlan = d.Rec.Last("autonomy_plan"); var lastArb = d.Rec.Last("autonomy_arbiter");
            var cycles = new List<object>(); foreach (var cy in d.Cycles) if ((double)cy["t"] >= tEndNs * 1e-9 - 5.0) cycles.Add(cy);
            var route = new List<object>(); foreach (var q in m.Spec.Route) route.Add(q.ToArray());
            return new Dictionary<string, object> {
                ["schema"] = "crane-replay-bundle/1", ["run_id"] = rs.RunId, ["run_spec"] = rs.ToDict(), ["outcome"] = o.Outcome, ["label"] = o.Label, ["evidence"] = ev,
                ["t_end_s"] = tEndNs * 1e-9, ["end_tick"] = d.EndTick, ["frame"] = "world ENU (manifest frame); belief voxels mapped to world with the truth-estimate offset at the end tick",
                ["route"] = route, ["truth_traj_20hz"] = traj, ["est_traj_local_20hz"] = est, ["local_error_end"] = e.ToArray(),
                ["decision_at_failure"] = new Dictionary<string, object> { ["last_plan"] = Strip(lastPlan), ["last_arbiter"] = lastArb, ["mode"] = m.Arbiter.Mode, ["tau_s"] = m.Rule.Tau },
                ["belief_occupied_xy"] = occ, ["belief_band_m"] = new[] { z0, z1 }, ["voxel_m"] = res, ["truth_capsules"] = caps,
                ["cycles_last5s"] = cycles, ["events_last3s"] = recent, ["result"] = m.Result.ToDict() };
        }

        static Dictionary<string, object> Strip(Dictionary<string, object> f) { if (f == null) return null; var c = new Dictionary<string, object>(f); c.Remove("config"); return c; }

        // ------------------------------------------------------------------ crane-drone-log/1 writing

        void WriteLogs(string runDir, RunSpec rs, List<Drone> drones, string sha, string physSha, bool full, ForestManifestWorld w) {
            for (int k = 0; k < drones.Count; k++) {
                var d = drones[k]; string dir = k == 0 ? runDir : Path.Combine(runDir, "peer" + k); Directory.CreateDirectory(dir);
                var evs = new List<(long t, string kind, Dictionary<string, object> f)>(d.Rec.Events);
                // stable sort by time (events are emitted in tick order; keep emission order within a tick)
                var idx = new List<int>(); for (int i = 0; i < evs.Count; i++) idx.Add(i);
                idx.Sort((a, b) => evs[a].t != evs[b].t ? evs[a].t.CompareTo(evs[b].t) : a.CompareTo(b));
                using (var sw = new StreamWriter(Path.Combine(dir, "events.jsonl"))) foreach (int i in idx) {
                    var f = new Dictionary<string, object> { ["t_ns"] = evs[i].t, ["kind"] = evs[i].kind, ["drone"] = d.Name };
                    foreach (var kv in evs[i].f) f[kv.Key] = kv.Value;
                    sw.WriteLine(HJson.Write(f));
                }
                var c = d.M.C; long endNs = d.M.TNs;
                var man = new Dictionary<string, object> {
                    ["schema"] = "crane-drone-log/1", ["run_id"] = rs.RunId + (k == 0 ? "" : "-peer" + k), ["crane_git_sha"] = Environment.GetEnvironmentVariable("CRANE_GIT_SHA") ?? "unknown",
                    ["scene"] = "dotnet_manifest_world", ["scene_seed"] = rs.ForestSeed, ["split"] = rs.Split, ["forest_manifest_sha256"] = sha,
                    ["physics_profile"] = rs.Profile, ["physics_params_sha256"] = physSha, ["controller"] = "CraneMultirotorController (velocity mode) via InProcessControllerLink",
                    ["trajectory_spec"] = new Dictionary<string, object> { ["mission"] = k == 0 ? rs.Mission : rs.PeerMissions[k - 1], ["autonomy"] = d.M.Planner.Name, ["v_limit"] = c.VLimit, ["config_hash"] = HJson.Fnv64(c.Canonical()) },
                    ["disturbance"] = new Dictionary<string, object> { ["wind"] = rs.Wind, ["lighting"] = rs.Lighting, ["wind_mean_mps"] = rs.WindMean, ["wind_w20_mps"] = rs.WindW20, ["wind_dir_deg"] = rs.WindDirDeg },
                    ["unity_version"] = "none (.NET harness)", ["wall_clock_start_utc"] = DateTime.UtcNow.ToString("o", CultureInfo.InvariantCulture),
                    ["clock"] = new Dictionary<string, object> { ["unit"] = "ns", ["fixed_dt_ns"] = c.TickNs, ["substeps"] = 1, ["substep_dt_ns"] = c.TickNs, ["substep_hz"] = 1e9 / c.TickNs, ["state_every_substeps"] = 1, ["state_hz"] = 1e9 / c.TickNs },
                    ["frames"] = new Dictionary<string, object> { ["world"] = "ENU", ["body"] = "FLU", ["quat"] = "wxyz world_from_body" },
                    ["rotor_count"] = d.M.Vehicle.Rotors.Length, ["imu"] = new Dictionary<string, object> { ["rate_hz"] = 0, ["noise"] = null }, ["cameras"] = new List<object>(), ["lidars"] = new List<object>(),
                    ["extra"] = new Dictionary<string, object> { ["run_spec"] = rs.ToDict(), ["dr"] = rs.Dr, ["faults"] = rs.Faults, ["comms"] = rs.Comms, ["autonomy_config"] = c.Canonical(),
                        ["streams"] = full ? "state.csv + events.jsonl (autonomy_*, comms_*, collision)" : "events.jsonl only (logs=events; state.csv omitted)" },
                    ["end"] = new Dictionary<string, object> { ["end_tick"] = d.M.Tick, ["end_t_ns"] = endNs, ["counts"] = new Dictionary<string, object> { ["state"] = full ? d.M.Tick : 0, ["events"] = evs.Count } } };
                File.WriteAllText(Path.Combine(dir, "manifest.json"), HJson.Write(man));
            }
        }
    }
}
