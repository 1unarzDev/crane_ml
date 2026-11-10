// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using System.Collections.Generic;
using Crane.Drones.Autonomy;
using Sim.Physics.Aerial;

namespace Crane.Drones.Harness {
    /// <summary>
    /// Targeted domain randomization. A DR family (Tools/CraneHarness/dr_families.json) lists dimensions with a
    /// distribution; Sample() draws one value per dimension from a stream keyed on (dr seed, forest, mission, run seed,
    /// sample index) — NOT on speed or condition, so paired conditions share the same DR draw. Dimensions:
    ///  * AutonomyConfig fields (sensor noise, dropout, thin-branch parameters, latency and stall tails, estimator drift);
    ///  * wind (preset name), wind_dir_deg;
    ///  * forest_keep_stems / forest_keep_branches (thinning of the loaded manifest, .NET world only);
    ///  * lighting (name; logged and guarded, consumed by vision runs later).
    /// Every sampled value is written into the run spec, results row and log manifest.
    /// </summary>
    public static class DomainRandomizer {
        public static void Sample(SplitGuard g, RunSpec r, ulong drSeed) {
            if (!g.Families.TryGetValue(r.DrFamily, out var fam)) throw new SplitRefused("DR family " + r.DrFamily + " is not registered");
            var dims = HJson.Obj(HJson.Get(fam, "dims"));
            ulong key = Hash(drSeed, (ulong)r.ForestSeed, HJson.Fnv64(r.Mission), r.RunSeed, (ulong)r.DrIndex);
            var names = new List<string>(dims.Keys); names.Sort(StringComparer.Ordinal);   // order-independent of JSON layout
            foreach (var n in names) {
                var d = HJson.Obj(dims[n]); var rng = new Rng(key, Hash(0, 0, HJson.Fnv64(n), 0, 0));
                string dist = HJson.Text(d, "dist", "uniform");
                if (dist == "choice") { var vals = HJson.Strings(d, "values"); r.Dr[n] = vals[(int)Math.Min(vals.Count - 1, Math.Floor(rng.Uniform() * vals.Count))]; continue; }
                double lo = HJson.Num(d, "lo", 0), hi = HJson.Num(d, "hi", lo), u = rng.Uniform();
                r.Dr[n] = dist == "loguniform" ? Math.Exp(Math.Log(lo) + u * (Math.Log(hi) - Math.Log(lo))) : lo + u * (hi - lo);
            }
        }

        static ulong Hash(ulong a, ulong b, string c, ulong d, ulong e) {
            ulong h = Convert.ToUInt64(c, 16);
            ulong z = a * 0x9E3779B97F4A7C15UL ^ (b + 0x632BE59BD9B4E019UL) * 0xBF58476D1CE4E5B9UL ^ h ^ (d + 0x94D049BB133111EBUL) * 0xD6E8FEB86659FD93UL ^ (e + 1) * 0xA0761D6478BD642FUL;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL; z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL; return z ^ (z >> 31);
        }
    }

    /// <summary>
    /// Forest thinning on a run-private ForestManifestWorld: removed capsules get radius -1000 m, which no ray, clearance
    /// or prior query can reach (RayCapsule needs dist &lt; r + beam t; clearance subtracts r; the prior needs r &gt;= 3 cm).
    /// Restore() puts the original radii back. Owners (trees / mid-story) are dropped whole with p = 1 - keepStems; branch
    /// capsules (non-vertical or &lt; 35 % of the owner's max radius) with p = 1 - keepBranches. Only thinning is possible
    /// on one manifest; denser forests need new manifests (BuildManifestOnly).
    /// </summary>
    public sealed class ForestThinning {
        private readonly ForestManifestWorld w; private readonly float[] r0; public int DroppedOwners, DroppedBranches;
        public ForestThinning(ForestManifestWorld world) { w = world; r0 = (float[])world.R.Clone(); }
        public void Restore() { Array.Copy(r0, w.R, r0.Length); DroppedOwners = DroppedBranches = 0; }

        public void Apply(double keepStems, double keepBranches, ulong seed) {
            Restore();
            if (keepStems >= 1 && keepBranches >= 1) return;
            var rng = new Rng(seed, 0x7417);
            int no = w.OwnerNames.Length; var drop = new bool[no]; var maxR = new double[no];
            for (int i = 0; i < no; i++) { drop[i] = rng.Uniform() >= keepStems; if (drop[i]) DroppedOwners++; }
            for (int s = 0; s < w.Count; s++) maxR[w.Owner[s]] = Math.Max(maxR[w.Owner[s]], r0[s]);
            for (int s = 0; s < w.Count; s++) {
                int o = w.Owner[s];
                if (drop[o]) { w.R[s] = -1000f; continue; }
                double ux = w.Bx[s] - w.Ax[s], uy = w.By[s] - w.Ay[s], uz = w.Bz[s] - w.Az[s], L = Math.Sqrt(ux * ux + uy * uy + uz * uz);
                bool branch = (L > 1e-6 && Math.Abs(uz) / L < 0.8) || r0[s] < 0.35 * maxR[o];
                if (branch && keepBranches < 1 && DropBranch(seed, s) >= keepBranches) { w.R[s] = -1000f; DroppedBranches++; }
            }
        }
        static double DropBranch(ulong seed, int s) => DepthRaycastSensor.Hash01(seed, 0xB4A7C4, (ulong)s);
    }

    /// <summary>Mean wind plus Dryden gusts at TRUE height above the manifest terrain (the .NET plant runs with
    /// GroundHeight = -1000, so the plant's built-in turbulence would see ~1000 m AGL scales). Updated every 5 ms tick.</summary>
    public sealed class WindField {
        public readonly double[] Mean; public readonly CraneDrydenTurbulence Gust; public double[] Last = new double[3];
        public WindField(double meanMps, double dirDeg, double w20, ulong seed) {
            double a = dirDeg * Math.PI / 180.0; Mean = new[] { meanMps * Math.Cos(a), meanMps * Math.Sin(a), 0.0 };
            Gust = w20 > 0 ? new CraneDrydenTurbulence(w20, seed ^ 0x57494E44UL) : null;
        }
        public bool Active => Gust != null || Mean[0] != 0 || Mean[1] != 0;
        public double[] Step(double dt, double agl, double[] vel) {
            if (Gust == null) { Last = Mean; return Mean; }
            var g = Gust.Step(dt, Math.Max(agl, 0.5), Mean, vel);
            Last = new[] { Mean[0] + g[0], Mean[1] + g[1], Mean[2] + g[2] }; return Last;
        }
    }

    /// <summary>
    /// Scheduled faults, applied only through public seams (no Core edits):
    ///  * sensor_blackout {sensor: depth|lidar|all, t0, t1}: frames acquired in the window never become available;
    ///  * sensor_delay {sensor, t0, t1, extra_s}: availability pushed back by extra_s (perception stall);
    ///  * drift_burst {t0, t1, scale}: estimator local drift sigma x scale in the window (degraded VIO/LIO zone);
    ///  * align_jump {t, dx, dy}: global-alignment offset step (relocalization error);
    ///  * depth_invalid {t0, t1, p}: stereo invalid-pixel probability p in the window (texture-less/glare zone);
    ///  * thin_blind {t0, t1, w50_px}: thin-structure dropout midpoint raised to w50_px in the window.
    /// Config-mutating faults edit the run-private AutonomyConfig, which the estimator and sensors read every tick.
    /// </summary>
    public sealed class FaultInjector {
        private readonly List<Dictionary<string, object>> faults; private readonly AutonomyConfig c;
        private readonly double drift0, driftZ0, yaw0, inval0, w500, alx0, aly0;
        public readonly List<string> Active = new List<string>();
        public int BlackedOut, Delayed;
        public FaultInjector(List<Dictionary<string, object>> list, AutonomyConfig cfg) {
            faults = list ?? new List<Dictionary<string, object>>(); c = cfg;
            drift0 = c.LocalDriftSigma; driftZ0 = c.LocalDriftSigmaZ; yaw0 = c.YawDriftSigma; inval0 = c.InvalidP; w500 = c.ThinW50Px; alx0 = c.AlignOffsetX; aly0 = c.AlignOffsetY;
        }
        static bool In(Dictionary<string, object> f, double t) => t >= HJson.Num(f, "t0", 0) && t < HJson.Num(f, "t1", double.PositiveInfinity);
        static bool Sensor(Dictionary<string, object> f, string s) { string w = HJson.Text(f, "sensor", "all"); return w == "all" || w == s; }

        /// <summary>Call before each mission Step with the mission time [s].</summary>
        public void Tick(double t) {
            Active.Clear();
            double ds = 1, inval = inval0, w50 = w500, ax = alx0, ay = aly0;
            foreach (var f in faults) {
                string k = HJson.Text(f, "kind", "");
                if (k == "drift_burst" && In(f, t)) { ds *= HJson.Num(f, "scale", 10); Active.Add(k); }
                else if (k == "depth_invalid" && In(f, t)) { inval = HJson.Num(f, "p", 0.6); Active.Add(k); }
                else if (k == "thin_blind" && In(f, t)) { w50 = HJson.Num(f, "w50_px", 6); Active.Add(k); }
                else if (k == "align_jump" && t >= HJson.Num(f, "t", 0)) { ax += HJson.Num(f, "dx", 0); ay += HJson.Num(f, "dy", 0); Active.Add(k); }
                else if ((k == "sensor_blackout" || k == "sensor_delay") && In(f, t)) Active.Add(k);
            }
            c.LocalDriftSigma = drift0 * ds; c.LocalDriftSigmaZ = driftZ0 * ds; c.YawDriftSigma = yaw0 * ds;
            c.InvalidP = inval; c.ThinW50Px = w50; c.AlignOffsetX = ax; c.AlignOffsetY = ay;
        }

        public void OnFrame(string sensor, double acqS, ref long availNs) {
            foreach (var f in faults) {
                string k = HJson.Text(f, "kind", "");
                if (!Sensor(f, sensor) || !In(f, acqS)) continue;
                if (k == "sensor_blackout") { availNs = long.MaxValue / 2; BlackedOut++; return; }
                if (k == "sensor_delay") { availNs += (long)Math.Round(HJson.Num(f, "extra_s", 0.3) * 1e9); Delayed++; }
            }
        }

        public void Restore() { c.LocalDriftSigma = drift0; c.LocalDriftSigmaZ = driftZ0; c.YawDriftSigma = yaw0; c.InvalidP = inval0; c.ThinW50Px = w500; c.AlignOffsetX = alx0; c.AlignOffsetY = aly0; }
    }

    /// <summary>
    /// Lossy, delayed comms (ICommsLink) with scheduled outages: drop probability, fixed delay plus uniform jitter. Every
    /// send, drop and receive is reported to the endpoint's recorder (comms_send / comms_drop / comms_recv). Deterministic
    /// (own Rng stream). The autonomy stack does not consume comms yet; the harness uses heartbeats for link-loss
    /// failsafe and multi-drone situational metrics.
    /// </summary>
    public sealed class LossyCommsLink : ICommsLink {
        private readonly List<CommsMessage> q = new List<CommsMessage>(); private readonly Rng rng;
        public double DropP, DelayS, JitterS; public List<(double t0, double t1)> Outages = new List<(double, double)>();
        public Func<string, IAutonomyRecorder> RecorderFor = _ => NullRecorder.Instance;
        public int Sent, Dropped, Delivered;
        public LossyCommsLink(ulong seed, Dictionary<string, object> cfg) {
            rng = new Rng(seed, 0xC0115);
            DropP = HJson.Num(cfg, "drop_p", 0); DelayS = HJson.Num(cfg, "delay_s", 0.02); JitterS = HJson.Num(cfg, "jitter_s", 0.0);
            foreach (var o in HJson.List(cfg, "outages")) if (o is double[] a && a.Length == 2) Outages.Add((a[0], a[1]));
        }
        public bool LinkUp(double t) { foreach (var o in Outages) if (t >= o.t0 && t < o.t1) return false; return true; }
        public void Send(CommsMessage m) {
            Sent++; double t = m.SentNs * 1e-9; double u = rng.Uniform(), j = rng.Uniform();
            var rec = RecorderFor(m.From);
            if (!LinkUp(t) || u < DropP) {
                Dropped++;
                rec.Event(m.SentNs, "comms_drop", new Dictionary<string, object> { ["from"] = m.From, ["to"] = m.To, ["kind"] = m.Kind, ["seq"] = m.Seq, ["reason"] = LinkUp(t) ? "random_loss" : "outage" });
                return;
            }
            m.DeliverNs = m.SentNs + (long)Math.Round((DelayS + JitterS * j) * 1e9);
            q.Add(m);
            rec.Event(m.SentNs, "comms_send", new Dictionary<string, object> { ["from"] = m.From, ["to"] = m.To, ["kind"] = m.Kind, ["seq"] = m.Seq, ["deliver_ns"] = m.DeliverNs, ["bytes"] = m.Payload?.Length ?? 0 });
        }
        public IEnumerable<CommsMessage> Receive(string endpoint, long tNs) {
            var o = new List<CommsMessage>();
            for (int i = 0; i < q.Count; i++) if (q[i].DeliverNs <= tNs && (q[i].To == endpoint || (q[i].To == "*" && q[i].From != endpoint))) o.Add(q[i]);
            foreach (var m in o) {
                if (m.To != "*") q.Remove(m);
                Delivered++;
                RecorderFor(endpoint).Event(tNs, "comms_recv", new Dictionary<string, object> { ["from"] = m.From, ["to"] = endpoint, ["kind"] = m.Kind, ["seq"] = m.Seq, ["sent_ns"] = m.SentNs, ["age_s"] = (tNs - m.SentNs) * 1e-9 });
            }
            return o;
        }
        /// <summary>Broadcasts stay queued until every listed endpoint consumed them; call once per tick after all receives.</summary>
        public void ExpireBroadcasts(long tNs) { q.RemoveAll(m => m.To == "*" && m.DeliverNs <= tNs); }
    }
}
