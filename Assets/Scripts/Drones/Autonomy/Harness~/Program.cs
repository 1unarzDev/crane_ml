// SPDX-License-Identifier: GPL-3.0-or-later
// Out-of-Unity runner for Crane.Drones.Autonomy missions on a Drone Forest manifest (analytic capsule world).
//   dotnet run -c Release -- --manifest PATH --out DIR [--cross N] [--speeds 2,2.5,3] [--seeds 1,2] [--planner map_rhp]
//       [--only NAME] [--events] [--no-corridor] [--set Field=Value ...]
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using Crane.Drones.Autonomy;
using Sim.Physics.Aerial;

static class Program {
    static string Arg(string[] a, string k, string d = null) { int i = Array.IndexOf(a, k); return i >= 0 && i + 1 < a.Length ? a[i + 1] : d; }

    static int Main(string[] args) {
        var ci = CultureInfo.InvariantCulture;
        string manifest = Arg(args, "--manifest"), outDir = Arg(args, "--out", "/tmp/autonomy_harness");
        string profile = Arg(args, "--profile", Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "../../../../../../../../Tools/Aerial/profiles/aos7_v5_f90_6s.json")));
        int cross = int.Parse(Arg(args, "--cross", "0"), ci);
        var speeds = Arg(args, "--speeds", "2.5").Split(',').Select(s => double.Parse(s, ci)).ToArray();
        var seeds = Arg(args, "--seeds", "1").Split(',').Select(s => ulong.Parse(s, ci)).ToArray();
        string only = Arg(args, "--only");
        bool events = Array.IndexOf(args, "--events") >= 0;
        Directory.CreateDirectory(outDir);
        var t0 = DateTime.UtcNow;
        var world = ForestManifestWorld.Load(manifest);
        Console.WriteLine($"[harness] manifest seed={world.Seed} capsules={world.Count} corridor={world.Corridor.Count} load={(DateTime.UtcNow - t0).TotalSeconds:0.0}s");
        var prof = CraneAerialProfile.LoadFile(profile, true, true);
        var spec = prof.Vehicle; spec.GroundHeight = -1000.0;
        var bt = StopRule.SimulateBrakeTable(spec);
        Console.WriteLine("[harness] brake table: " + string.Join(" ", bt.vs.Zip(bt.ds, (v, d) => $"{v:0.0}:{d:0.00}")));
        var baseCfg = new AutonomyConfig();
        baseCfg.Planner = Arg(args, "--planner", "map_rhp");
        for (int i = 0; i < args.Length - 1; i++) if (args[i] == "--set") Set(baseCfg, args[i + 1]);
        var missions = MissionCatalog.Build(world, baseCfg, cross, Array.IndexOf(args, "--no-corridor") < 0);
        if (only != null) missions = missions.Where(m => only.Split(',').Contains(m.Name)).ToList();
        string resPath = Path.Combine(outDir, "results.jsonl");
        using var res = new StreamWriter(resPath, true);
        foreach (var v in speeds) foreach (var s in seeds) foreach (var m in missions) {
            var c = baseCfg.Clone(); c.VLimit = v; c.RunSeed = s;
            var prior = new ManifestPriorMap(world, c, s);
            var rule = new StopRule(bt.vs, bt.ds, c.TauPrior, c.Margin, c.AgeWindow, c.AgeQuantile);
            IAutonomyRecorder rec = NullRecorder.Instance; StreamWriter ev = null;
            if (events) { ev = new StreamWriter(Path.Combine(outDir, $"events_{m.Name}_v{v:0.0}_s{s}.jsonl")); rec = new JsonlRecorder(ev); }
            var mission = new AutonomousMission(m, c, spec, world, world, world, prior, rec, rule) { PriorForBelief = prior };
            if (Array.IndexOf(args, "--debug") >= 0) {
                for (int k = 0; k < 400; k++) mission.Step();
                var R = mission.Router; var e = mission.Estimator.Current;
                Console.WriteLine($"[debug] ctg finite={R.FiniteCells}/{R.TotalCells} blocked={R.BlockedCells} start={R.CostToGo(e.P.X, e.P.Y)} soft={R.CostToGoSoft(e.P.X, e.P.Y)} goal={R.GoalLocal} p={e.P}");
                Console.WriteLine("[debug] phases " + string.Join(" ", AutonomousMission.PhaseNames.Zip(mission.PhaseWall, (a, b) => $"{a}={b:0.00}s")));
                return 0;
            }
            var r = mission.Run();
            Console.WriteLine("[phases] " + string.Join(" ", AutonomousMission.PhaseNames.Zip(mission.PhaseWall, (a, b) => $"{a}={b:0.0}s")));
            ev?.Dispose();
            var d = r.ToDict(); d["harness"] = "dotnet_manifest_world"; d["forest_manifest"] = Path.GetFileName(manifest);
            res.WriteLine(JsonlRecorder.Json(d)); res.Flush();
            Console.WriteLine($"[harness] {m.Name,-14} v={v:0.0} seed={s} -> {r.Outcome,-9} t={r.TimeS,6:0.0}s prog={r.ProgressFrac,5:0.00} minclr={r.MinClearanceHardM,6:0.00} twig={r.TwigContacts,3} age99={r.AgeP99S:0.000} nosafe={r.NoSafeStops} wd={r.WatchdogTrips} rec={r.Recoveries} viol={r.MarginViolations}/{r.MarginTicks} v̄={r.MeanSpeed:0.00} wall={r.WallS:0.0}s {r.CollisionObject}");
        }
        return 0;
    }

    static void Set(AutonomyConfig c, string kv) {
        var p = kv.Split('='); var f = typeof(AutonomyConfig).GetField(p[0]);
        if (f == null) throw new ArgumentException("no config field " + p[0]);
        object v = f.FieldType == typeof(double) ? double.Parse(p[1], CultureInfo.InvariantCulture) : f.FieldType == typeof(int) ? int.Parse(p[1]) :
            f.FieldType == typeof(bool) ? bool.Parse(p[1]) : f.FieldType == typeof(string) ? p[1] : f.FieldType == typeof(double[]) ? p[1].Split(':').Select(x => double.Parse(x, CultureInfo.InvariantCulture)).ToArray() : (object)ulong.Parse(p[1]);
        f.SetValue(c, v);
    }
}

sealed class JsonlRecorder : IAutonomyRecorder {
    private readonly TextWriter w;
    public JsonlRecorder(TextWriter w) { this.w = w; }
    public void Event(long tNs, string kind, Dictionary<string, object> f) {
        var d = new Dictionary<string, object> { ["t_ns"] = tNs, ["kind"] = kind };
        foreach (var kv in f) d[kv.Key] = kv.Value;
        w.WriteLine(Json(d));
    }
    public static string Json(object o) { var sb = new StringBuilder(); W(sb, o); return sb.ToString(); }
    static void W(StringBuilder sb, object o) {
        var ci = CultureInfo.InvariantCulture;
        switch (o) {
            case null: sb.Append("null"); break;
            case string s: sb.Append('"').Append(s.Replace("\\", "\\\\").Replace("\"", "\\\"")).Append('"'); break;
            case bool b: sb.Append(b ? "true" : "false"); break;
            case double d: sb.Append(double.IsFinite(d) ? d.ToString("R", ci) : "null"); break;
            case float f: sb.Append(float.IsFinite(f) ? f.ToString("R", ci) : "null"); break;
            case int or long or ulong: sb.Append(Convert.ToString(o, ci)); break;
            case Dictionary<string, object> m: sb.Append('{'); bool first = true; foreach (var kv in m) { if (!first) sb.Append(','); first = false; W(sb, kv.Key); sb.Append(':'); W(sb, kv.Value); } sb.Append('}'); break;
            case System.Collections.IEnumerable e: sb.Append('['); bool f1 = true; foreach (var x in e) { if (!f1) sb.Append(','); f1 = false; W(sb, x); } sb.Append(']'); break;
            default: sb.Append('"').Append(o.ToString()).Append('"'); break;
        }
    }
}
