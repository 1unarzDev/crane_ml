// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Reflection;
using Crane.Drones.Autonomy;

namespace Crane.Drones.Harness {
    /// <summary>
    /// Declarative experiment (schema crane-experiment/1, see Docs/DroneVisuals/HARNESS.md): forests and split, missions,
    /// speeds, run seeds, autonomy config overrides, DR family and sample count, wind/lighting, faults, comms, N drones.
    /// Expand() produces one fully resolved <see cref="RunSpec"/> per run; a RunSpec alone re-executes the run.
    /// </summary>
    public sealed class ExperimentSpec {
        public string Name = "experiment", Split, Profile = "Tools/Aerial/profiles/aos7_v5_f90_6s.json", Planner = "map_rhp";
        public List<(int seed, string manifest)> Forests = new List<(int, string)>();
        public int Cross = 10; public bool Corridor = true; public List<string> Only = new List<string>();
        public double[] Speeds = { 2.5 }; public ulong[] RunSeeds = { 1 };
        public Dictionary<string, object> Config = new Dictionary<string, object>();
        public string DrFamily = "none"; public int DrSamples = 1; public ulong DrSeed = 1;
        public string Wind = "calm", Lighting = "GoldenHour";
        public List<Dictionary<string, object>> Faults = new List<Dictionary<string, object>>();
        public Dictionary<string, object> Comms;
        public int NDrones = 1;
        public string Logs = "events";   // none | events | full
        public bool LabelFailures = true, BundleFailures = true, AllowHeldout;
        public string Raw = "";

        public static ExperimentSpec Load(string path, string repoRoot) => Parse(File.ReadAllText(path), repoRoot);

        public static ExperimentSpec Parse(string json, string repoRoot) {
            var o = HJson.Obj(MiniJson.Parse(json));
            if (HJson.Text(o, "schema", "") != "crane-experiment/1") throw new ArgumentException("experiment spec: schema must be crane-experiment/1");
            var e = new ExperimentSpec { Raw = json };
            e.Name = HJson.Text(o, "name", e.Name); e.Split = HJson.Text(o, "split", null);
            e.Profile = HJson.Text(o, "profile", e.Profile); e.Planner = HJson.Text(o, "planner", e.Planner);
            string pattern = HJson.Text(o, "manifest_pattern", "Assets/Drones/Environment/Manifests/forest_seed{seed}_manifest.json");
            foreach (var fo in HJson.List(o, "forests")) {
                var f = HJson.Obj(fo);
                if (f != null) e.Forests.Add(((int)HJson.Num(f, "seed", -1), HJson.Text(f, "manifest", pattern.Replace("{seed}", ((int)HJson.Num(f, "seed", -1)).ToString(CultureInfo.InvariantCulture)))));
                else if (fo is double s) e.Forests.Add(((int)s, pattern.Replace("{seed}", ((int)s).ToString(CultureInfo.InvariantCulture))));
            }
            var fs = HJson.Nums(o, "forest_seeds");
            if (fs != null) foreach (var s in fs) e.Forests.Add(((int)s, pattern.Replace("{seed}", ((int)s).ToString(CultureInfo.InvariantCulture))));
            var mi = HJson.Obj(HJson.Get(o, "missions"));
            if (mi != null) { e.Cross = (int)HJson.Num(mi, "cross", e.Cross); e.Corridor = HJson.Flag(mi, "corridor", e.Corridor); e.Only = HJson.Strings(mi, "only"); }
            e.Speeds = HJson.Nums(o, "speeds") ?? e.Speeds;
            var rs = HJson.Nums(o, "run_seeds"); if (rs != null) e.RunSeeds = Array.ConvertAll(rs, x => (ulong)x);
            e.Config = HJson.Obj(HJson.Get(o, "config")) ?? e.Config;
            var dr = HJson.Obj(HJson.Get(o, "dr"));
            if (dr != null) { e.DrFamily = HJson.Text(dr, "family", "none"); e.DrSamples = Math.Max(1, (int)HJson.Num(dr, "samples", 1)); e.DrSeed = (ulong)HJson.Num(dr, "seed", 1); }
            e.Wind = HJson.Text(o, "wind", e.Wind); e.Lighting = HJson.Text(o, "lighting", e.Lighting);
            foreach (var fo in HJson.List(o, "faults")) if (HJson.Obj(fo) != null) e.Faults.Add(HJson.Obj(fo));
            e.Comms = HJson.Obj(HJson.Get(o, "comms"));
            e.NDrones = Math.Max(1, (int)HJson.Num(o, "n_drones", 1));
            e.Logs = HJson.Text(o, "logs", e.Logs);
            e.LabelFailures = HJson.Flag(o, "label_failures", true); e.BundleFailures = HJson.Flag(o, "bundle_failures", true);
            e.AllowHeldout = HJson.Flag(o, "allow_heldout", false);
            if (e.Forests.Count == 0) throw new ArgumentException("experiment spec: no forests declared (guard fails closed)");
            return e;
        }

        /// <summary>Guard the whole experiment (seeds, DR family, wind, lighting, config overrides, every DR sample).</summary>
        public string Guard(SplitGuard g, bool allowHeldoutCli, out List<RunSpec> runs, Func<int, string, List<string>> missionNames) {
            var seeds = new List<int>(); foreach (var f in Forests) seeds.Add(f.seed);
            var winds = new List<string> { Wind }; var lights = new List<string> { Lighting };
            if (DrFamily != "none" && g.Families.TryGetValue(DrFamily, out var fam)) {
                var dims = HJson.Obj(HJson.Get(fam, "dims"));
                if (dims != null) {
                    if (HJson.Obj(HJson.Get(dims, "wind")) is Dictionary<string, object> wd) winds.AddRange(HJson.Strings(wd, "values"));
                    if (HJson.Obj(HJson.Get(dims, "lighting")) is Dictionary<string, object> ld) lights.AddRange(HJson.Strings(ld, "values"));
                }
            }
            bool allow = allowHeldoutCli && AllowHeldout;   // both the spec and the command line must opt in
            string split = g.Enforce(Split, seeds, DrFamily, winds, lights, allow);
            var cfgNums = new Dictionary<string, double>();
            foreach (var kv in Config) if (kv.Value is double d) cfgNums[kv.Key] = d;
            g.EnforceValues(split, cfgNums, "config override");
            runs = Expand(split, g, missionNames);
            foreach (var r in runs) g.EnforceValues(split, r.DrNumeric(), "DR sample " + r.RunId);
            return split;
        }

        public List<RunSpec> Expand(string split, SplitGuard g, Func<int, string, List<string>> missionNames) {
            var o = new List<RunSpec>();
            foreach (var f in Forests) {
                var names = missionNames(f.seed, f.manifest);
                if (Only.Count > 0) names = names.FindAll(n => Only.Contains(n));
                foreach (var v in Speeds) foreach (var s in RunSeeds) for (int mi = 0; mi < names.Count; mi++) for (int di = 0; di < (DrFamily == "none" ? 1 : DrSamples); di++) {
                    var r = new RunSpec {
                        Experiment = Name, Split = split, ForestSeed = f.seed, Manifest = f.manifest, Profile = Profile, Planner = Planner,
                        Cross = Cross, Corridor = Corridor, Mission = names[mi], Speed = v, RunSeed = s, DrFamily = DrFamily, DrIndex = di,
                        Wind = Wind, Lighting = Lighting, Config = Config, Faults = Faults, Comms = Comms, NDrones = NDrones
                    };
                    if (NDrones > 1) { r.PeerMissions = new List<string>(); for (int k = 1; k < NDrones; k++) r.PeerMissions.Add(names[(mi + k) % names.Count]); }
                    if (DrFamily != "none") DomainRandomizer.Sample(g, r, DrSeed);
                    r.Finish(g);
                    o.Add(r);
                }
            }
            return o;
        }
    }

    /// <summary>One fully resolved run. Serialized as run_spec.json; Replay re-executes from it alone.</summary>
    public sealed class RunSpec {
        public string Experiment, Split, Manifest, Profile, Planner, Mission, DrFamily = "none", Wind = "calm", Lighting = "GoldenHour";
        public int ForestSeed, Cross, DrIndex, NDrones = 1; public bool Corridor; public double Speed; public ulong RunSeed;
        public Dictionary<string, object> Config = new Dictionary<string, object>();
        public Dictionary<string, object> Dr = new Dictionary<string, object>();   // sampled DR values (numbers or names)
        public List<Dictionary<string, object>> Faults = new List<Dictionary<string, object>>();
        public Dictionary<string, object> Comms;
        public List<string> PeerMissions;
        public double WindMean, WindW20, WindDirDeg;
        public string RunId, PairKey, Cluster;

        public static string Fmt(double v) => v.ToString("0.###", CultureInfo.InvariantCulture);

        public void Finish(SplitGuard g) {
            string w = Dr.TryGetValue("wind", out var wo) && wo is string ws ? ws : Wind; Wind = w;
            if (Dr.TryGetValue("lighting", out var lo) && lo is string ls) Lighting = ls;
            if (g != null && g.WindPresets.TryGetValue(Wind, out var p)) { WindMean = p.mean; WindW20 = p.w20; }
            WindDirDeg = Dr.TryGetValue("wind_dir_deg", out var dd) && dd is double dv ? dv : 0.0;
            RunId = $"{Experiment}-f{ForestSeed}-{Mission}-v{Fmt(Speed)}-r{RunSeed}-d{DrIndex}";
            PairKey = $"f{ForestSeed}|{Mission}|r{RunSeed}|d{DrIndex}";
            Cluster = $"f{ForestSeed}|{Mission}";
        }

        public Dictionary<string, double> DrNumeric() {
            var d = new Dictionary<string, double>(); foreach (var kv in Dr) if (kv.Value is double x) d[kv.Key] = x; return d;
        }

        public Dictionary<string, object> ToDict() => new Dictionary<string, object> {
            ["schema"] = "crane-run-spec/1", ["run_id"] = RunId, ["pair_key"] = PairKey, ["cluster"] = Cluster, ["experiment"] = Experiment, ["split"] = Split,
            ["forest_seed"] = ForestSeed, ["manifest"] = Manifest, ["profile"] = Profile, ["planner"] = Planner, ["cross"] = Cross, ["corridor"] = Corridor,
            ["mission"] = Mission, ["speed"] = Speed, ["run_seed"] = (double)RunSeed, ["dr_family"] = DrFamily, ["dr_index"] = DrIndex, ["dr"] = Dr,
            ["wind"] = Wind, ["wind_mean_mps"] = WindMean, ["wind_w20_mps"] = WindW20, ["wind_dir_deg"] = WindDirDeg, ["lighting"] = Lighting,
            ["config"] = Config, ["faults"] = Faults, ["comms"] = Comms, ["n_drones"] = NDrones, ["peer_missions"] = PeerMissions };

        public static RunSpec FromDict(Dictionary<string, object> o) {
            var r = new RunSpec {
                RunId = HJson.Text(o, "run_id", ""), PairKey = HJson.Text(o, "pair_key", ""), Cluster = HJson.Text(o, "cluster", ""),
                Experiment = HJson.Text(o, "experiment", ""), Split = HJson.Text(o, "split", null), ForestSeed = (int)HJson.Num(o, "forest_seed", -1),
                Manifest = HJson.Text(o, "manifest", ""), Profile = HJson.Text(o, "profile", ""), Planner = HJson.Text(o, "planner", "map_rhp"),
                Cross = (int)HJson.Num(o, "cross", 0), Corridor = HJson.Flag(o, "corridor", true), Mission = HJson.Text(o, "mission", ""),
                Speed = HJson.Num(o, "speed", 2.5), RunSeed = (ulong)HJson.Num(o, "run_seed", 1), DrFamily = HJson.Text(o, "dr_family", "none"),
                DrIndex = (int)HJson.Num(o, "dr_index", 0), Wind = HJson.Text(o, "wind", "calm"), WindMean = HJson.Num(o, "wind_mean_mps", 0),
                WindW20 = HJson.Num(o, "wind_w20_mps", 0), WindDirDeg = HJson.Num(o, "wind_dir_deg", 0), Lighting = HJson.Text(o, "lighting", "GoldenHour"),
                NDrones = (int)HJson.Num(o, "n_drones", 1), Comms = HJson.Obj(HJson.Get(o, "comms")) };
            r.Config = HJson.Obj(HJson.Get(o, "config")) ?? new Dictionary<string, object>();
            r.Dr = HJson.Obj(HJson.Get(o, "dr")) ?? new Dictionary<string, object>();
            foreach (var f in HJson.List(o, "faults")) if (HJson.Obj(f) != null) r.Faults.Add(HJson.Obj(f));
            var pm = HJson.Strings(o, "peer_missions"); if (pm.Count > 0) r.PeerMissions = pm;
            return r;
        }

        /// <summary>Base config + overrides + DR config dims (planner, speed, seed last). Harness dims (wind, forest_*) are not config fields.</summary>
        public AutonomyConfig BuildConfig() {
            var c = new AutonomyConfig { Planner = Planner };
            foreach (var kv in Config) ConfigSetter.Set(c, kv.Key, kv.Value);
            foreach (var kv in Dr) if (ConfigSetter.IsField(kv.Key)) ConfigSetter.Set(c, kv.Key, kv.Value);
            c.VLimit = Speed; c.RunSeed = RunSeed;
            return c;
        }
    }

    public static class ConfigSetter {
        public static bool IsField(string name) => typeof(AutonomyConfig).GetField(name, BindingFlags.Public | BindingFlags.Instance) != null;

        public static void Set(AutonomyConfig c, string name, object value) {
            var f = typeof(AutonomyConfig).GetField(name, BindingFlags.Public | BindingFlags.Instance);
            if (f == null) throw new ArgumentException("AutonomyConfig has no field " + name);
            var t = f.FieldType; object v;
            if (t == typeof(double)) v = Convert.ToDouble(value, CultureInfo.InvariantCulture);
            else if (t == typeof(int)) v = Convert.ToInt32(value, CultureInfo.InvariantCulture);
            else if (t == typeof(long)) v = Convert.ToInt64(value, CultureInfo.InvariantCulture);
            else if (t == typeof(ulong)) v = Convert.ToUInt64(value, CultureInfo.InvariantCulture);
            else if (t == typeof(float)) v = Convert.ToSingle(value, CultureInfo.InvariantCulture);
            else if (t == typeof(bool)) v = value is bool b ? b : bool.Parse(Convert.ToString(value, CultureInfo.InvariantCulture));
            else if (t == typeof(string)) v = Convert.ToString(value, CultureInfo.InvariantCulture);
            else if (t == typeof(double[])) v = value is double[] a ? (double[])a.Clone() : Array.ConvertAll(Convert.ToString(value, CultureInfo.InvariantCulture).Split(':'), s => double.Parse(s, CultureInfo.InvariantCulture));
            else throw new ArgumentException("unsupported config field type " + t.Name);
            f.SetValue(c, v);
        }

        public static double GetNum(AutonomyConfig c, string name) => Convert.ToDouble(typeof(AutonomyConfig).GetField(name).GetValue(c), CultureInfo.InvariantCulture);
    }
}
