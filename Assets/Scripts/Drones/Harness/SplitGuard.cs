// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using System.Collections.Generic;
using System.IO;
using Crane.Drones.Autonomy;

namespace Crane.Drones.Harness {
    public sealed class SplitRefused : Exception { public SplitRefused(string m) : base(m) { } }

    /// <summary>
    /// Split guard for experiment specs (pure-C# twin of Tools/DroneData/splits.py and DroneSplitGuard, same
    /// splits.json, plus the DR-family registry Tools/CraneHarness/dr_families.json). Fails closed:
    ///  * every forest seed must be in a split; HELD-OUT seeds, winds, lightings or DR families are refused unless
    ///    allowHeldout (final evaluation only);
    ///  * one experiment = one split: all forest seeds, the DR family, wind and lighting must belong to the declared
    ///    split (dev wind/lighting values may appear in train, as in splits.json; held-out values never);
    ///  * any resolved parameter value (DR sample or explicit config override) inside a 'heldout_reserved' interval is
    ///    refused for non-heldout experiments.
    /// </summary>
    public sealed class SplitGuard {
        public readonly List<(string split, long lo, long hi)> Ranges = new List<(string, long, long)>();
        public readonly List<(string family, string split, List<string> values)> Disturbance = new List<(string, string, List<string>)>();
        public readonly Dictionary<string, (double lo, double hi)> Reserved = new Dictionary<string, (double, double)>();
        public readonly Dictionary<string, Dictionary<string, object>> Families = new Dictionary<string, Dictionary<string, object>>();
        public readonly Dictionary<string, (double mean, double w20)> WindPresets = new Dictionary<string, (double, double)>();
        public string SplitsSha = "", FamiliesSha = "";

        public static SplitGuard Load(string splitsPath, string familiesPath) {
            if (splitsPath == null || !File.Exists(splitsPath)) throw new SplitRefused("splits.json not found (" + splitsPath + "); refusing (guard fails closed)");
            if (familiesPath == null || !File.Exists(familiesPath)) throw new SplitRefused("dr_families.json not found (" + familiesPath + "); refusing");
            var g = new SplitGuard();
            string sj = File.ReadAllText(splitsPath), fj = File.ReadAllText(familiesPath);
            g.SplitsSha = HJson.Fnv64(sj); g.FamiliesSha = HJson.Fnv64(fj);
            var s = HJson.Obj(MiniJson.Parse(sj));
            if (HJson.Text(s, "schema", "") != "crane-drone-splits/1") throw new SplitRefused("not a crane-drone-splits/1 file");
            foreach (var ro in HJson.List(s, "seed_ranges")) { var r = HJson.Obj(ro); g.Ranges.Add((HJson.Text(r, "split", "?"), (long)HJson.Num(r, "start", 0), (long)HJson.Num(r, "end", -1))); }
            foreach (var d0 in HJson.List(s, "disturbance")) { var d = HJson.Obj(d0); g.Disturbance.Add((HJson.Text(d, "family", ""), HJson.Text(d, "split", ""), HJson.Strings(d, "values"))); }
            var f = HJson.Obj(MiniJson.Parse(fj));
            if (HJson.Text(f, "schema", "") != "crane-dr-families/1") throw new SplitRefused("not a crane-dr-families/1 file");
            var res = HJson.Obj(HJson.Get(f, "heldout_reserved"));
            if (res != null) foreach (var kv in res) if (kv.Value is double[] a && a.Length == 2) g.Reserved[kv.Key] = (a[0], a[1]);
            var fam = HJson.Obj(HJson.Get(f, "families"));
            if (fam != null) foreach (var kv in fam) g.Families[kv.Key] = HJson.Obj(kv.Value);
            var wp = HJson.Obj(HJson.Get(f, "wind_presets"));
            if (wp != null) foreach (var kv in wp) { var o = HJson.Obj(kv.Value); if (o != null) g.WindPresets[kv.Key] = (HJson.Num(o, "mean_mps", 0), HJson.Num(o, "w20_mps", 0)); }
            g.Validate();
            return g;
        }

        /// <summary>Registry self-consistency: non-heldout family ranges never touch a reserved interval.</summary>
        public void Validate() {
            foreach (var kv in Families) {
                string split = HJson.Text(kv.Value, "split", "");
                if (SplitOfName(split) == null) throw new SplitRefused($"DR family {kv.Key}: unknown split {split}");
                if (split == "heldout") continue;
                var dims = HJson.Obj(HJson.Get(kv.Value, "dims"));
                if (dims == null) continue;
                foreach (var d in dims) {
                    var o = HJson.Obj(d.Value); if (!Reserved.TryGetValue(d.Key, out var r)) continue;
                    double lo = HJson.Num(o, "lo", double.NaN), hi = HJson.Num(o, "hi", double.NaN);
                    if (!double.IsNaN(lo) && lo <= r.hi && hi >= r.lo) throw new SplitRefused($"DR family {kv.Key} dim {d.Key} [{lo},{hi}] overlaps held-out reserved [{r.lo},{r.hi}]");
                }
                foreach (var w in HJson.Strings(kv.Value, "wind")) if (DisturbanceSplit("wind", w) == "heldout") throw new SplitRefused($"DR family {kv.Key} uses held-out wind {w}");
                foreach (var w in HJson.Strings(kv.Value, "lighting")) if (DisturbanceSplit("lighting", w) == "heldout") throw new SplitRefused($"DR family {kv.Key} uses held-out lighting {w}");
            }
        }

        string SplitOfName(string n) { foreach (var r in Ranges) if (r.split == n) return n; return null; }

        public string SeedSplit(long seed) { foreach (var r in Ranges) if (seed >= r.lo && seed <= r.hi) return r.split; return "unassigned"; }

        /// <summary>Held-out wins (as splits.py); otherwise the first split listing the value; "unassigned" if none.</summary>
        public string DisturbanceSplit(string family, string value) {
            foreach (var d in Disturbance) if (d.family == family && d.split == "heldout" && d.values.Contains(value)) return "heldout";
            foreach (var d in Disturbance) if (d.family == family && d.values.Contains(value)) return d.split;
            return "unassigned";
        }

        bool DisturbanceAllowed(string family, string value, string split) {
            foreach (var d in Disturbance) if (d.family == family && d.split == split && d.values.Contains(value)) return true;
            return false;
        }

        /// <summary>Check a whole experiment before anything runs or is written. Returns the split.</summary>
        public string Enforce(string declaredSplit, IEnumerable<int> forestSeeds, string drFamily, IEnumerable<string> winds, IEnumerable<string> lightings, bool allowHeldout) {
            var problems = new List<string>();
            string split = declaredSplit;
            foreach (var s in forestSeeds) {
                string ss = SeedSplit(s);
                if (ss == "unassigned") problems.Add($"forest seed {s} is in no split");
                else if (split == null) split = ss;
                else if (ss != split) problems.Add($"forest seed {s} is {ss}, experiment split is {split}");
            }
            if (split == null) throw new SplitRefused("no forest seed declared; refusing (seed must be declared)");
            if (SplitOfName(split) == null) problems.Add($"unknown split {split}");
            if (drFamily != null && drFamily != "none") {
                if (!Families.TryGetValue(drFamily, out var fam)) problems.Add($"DR family {drFamily} is not registered");
                else if (HJson.Text(fam, "split", "") != split) problems.Add($"DR family {drFamily} belongs to split {HJson.Text(fam, "split", "")}, experiment split is {split}");
            }
            foreach (var w in winds) if (w != null) { if (DisturbanceSplit("wind", w) == "unassigned") problems.Add($"wind {w} is in no split"); else if (!DisturbanceAllowed("wind", w, split)) problems.Add($"wind {w} is not a {split} value"); }
            foreach (var l in lightings) if (l != null) { if (DisturbanceSplit("lighting", l) == "unassigned") problems.Add($"lighting {l} is in no split"); else if (!DisturbanceAllowed("lighting", l, split)) problems.Add($"lighting {l} is not a {split} value"); }
            bool heldout = split == "heldout";
            if (heldout && !allowHeldout) problems.Insert(0, "split is HELD-OUT");
            if (problems.Count > 0 && !(heldout && allowHeldout && problems.TrueForAll(p => p == "split is HELD-OUT")))
                throw new SplitRefused(string.Join("; ", problems) + (heldout ? "; refusing (pass --allow-heldout for the final evaluation only)" : "; refusing"));
            if (heldout) Console.Error.WriteLine("CRANE_HARNESS_HELDOUT_ALLOWED split=heldout");
            return split;
        }

        /// <summary>Refuse resolved values inside a reserved held-out interval (non-heldout experiments).</summary>
        public void EnforceValues(string split, IDictionary<string, double> values, string what) {
            if (split == "heldout") return;
            foreach (var kv in values)
                if (Reserved.TryGetValue(kv.Key, out var r) && kv.Value >= r.lo && kv.Value <= r.hi)
                    throw new SplitRefused($"{what}: {kv.Key}={kv.Value} lies in the held-out reserved interval [{r.lo},{r.hi}]; refusing");
        }

        public static string FindUp(string start, string rel) {
            var d = new DirectoryInfo(start);
            while (d != null) { var p = Path.Combine(d.FullName, rel); if (File.Exists(p)) return p; d = d.Parent; }
            return null;
        }
    }
}
