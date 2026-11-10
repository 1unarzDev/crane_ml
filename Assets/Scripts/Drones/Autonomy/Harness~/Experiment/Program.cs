// SPDX-License-Identifier: GPL-3.0-or-later
// CRANE experiment harness CLI (.NET, no Unity). Build: dotnet build -c Release -o /tmp/crane_exp
//   run     --spec SPEC.json --out DIR [--shard i --nshards n] [--limit k] [--allow-heldout]
//   guard   --spec SPEC.json                      (split guard + expansion only; prints the run count)
//   replay  --out DIR --run-id ID [--allow-heldout] (deterministic re-execution to the failure + replay_bundle.json)
//   test    [TestName ...]                         (Assets/Scripts/Drones/Harness/Tests via the NUnit shim)
//   export-navpilot --spec SPEC.json --out DIR      (straight cross_k missions as crane-navpilot-stand/1 for the 2.5D twin)
//   test-lead --spec SPEC.json --lead RESULTS.jsonl [--only cross_8] (cross-harness reproduction of the lead's runs)
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using Crane.Drones.Autonomy;
using Crane.Drones.Harness;

static class Program {
    static string Arg(string[] a, string k, string d = null) { int i = Array.IndexOf(a, k); return i >= 0 && i + 1 < a.Length ? a[i + 1] : d; }

    static string Repo(string[] a) {
        var r = Arg(a, "--repo"); if (r != null) return Path.GetFullPath(r);
        foreach (var s in new[] { Directory.GetCurrentDirectory(), AppContext.BaseDirectory }) { var p = SplitGuard.FindUp(s, "Tools/DroneData/splits.json"); if (p != null) return Path.GetFullPath(Path.Combine(Path.GetDirectoryName(p), "..", "..")); }
        return "/home/lunarz/worktrees/crane-drones";
    }

    static int Main(string[] args) {
        if (args.Length == 0) { Console.Error.WriteLine("usage: run|guard|replay|test|test-lead ..."); return 2; }
        string repo = Repo(args); Environment.SetEnvironmentVariable("CRANE_REPO", repo);
        try {
            switch (args[0]) {
                case "test": return Tests(args.Skip(1).ToArray());
                case "guard": {
                    var ctx = Ctx(repo); var spec = ExperimentSpec.Load(Arg(args, "--spec"), repo);
                    var split = spec.Guard(ctx.Guard, args.Contains("--allow-heldout"), out var runs, (s, m) => { if (ctx.World(m).w.Seed != s) throw new SplitRefused("manifest seed mismatch"); return ctx.MissionNames(m, spec.Cross, spec.Corridor); });
                    Console.WriteLine($"[guard] split={split} runs={runs.Count} first={runs.FirstOrDefault()?.RunId}"); return 0;
                }
                case "run": {
                    var ctx = Ctx(repo); var spec = ExperimentSpec.Load(Arg(args, "--spec"), repo);
                    return ExperimentBatch.Run(ctx, spec, Arg(args, "--out"), int.Parse(Arg(args, "--shard", "0")), int.Parse(Arg(args, "--nshards", "1")), args.Contains("--allow-heldout"), int.Parse(Arg(args, "--limit", "-1")));
                }
                case "replay": {
                    var ctx = Ctx(repo); var rp = ExperimentBatch.ReplayById(ctx, Arg(args, "--out"), Arg(args, "--run-id"), args.Contains("--allow-heldout"));
                    Console.WriteLine($"[replay] {Arg(args, "--run-id")} -> {rp.Outcome} @tick {rp.EndTick} label={rp.Label} reproduced={rp.Row["replay_reproduced"]}\n  {rp.LabelDetail}");
                    return (bool)rp.Row["replay_reproduced"] ? 0 : 1;
                }
                case "test-lead": return TestLead(repo, args);
                case "export-navpilot": return ExportNavPilot(repo, args);
            }
        } catch (SplitRefused e) { Console.Error.WriteLine("[guard] REFUSED: " + e.Message); return 3; }
        Console.Error.WriteLine("unknown command " + args[0]); return 2;
    }

    static HarnessContext Ctx(string repo) => new HarnessContext(repo, SplitGuard.Load(Path.Combine(repo, "Tools/DroneData/splits.json"), Path.Combine(repo, "Tools/CraneHarness/dr_families.json")));

    static int Tests(string[] only) {
        int fail = 0, n = 0;
        foreach (var t in typeof(Program).Assembly.GetTypes().Where(t => t.Namespace == "Crane.Drones.Harness.Tests"))
            foreach (var mi in t.GetMethods().Where(m => m.GetCustomAttribute<NUnit.Framework.TestAttribute>() != null)) {
                if (only.Length > 0 && !only.Contains(mi.Name)) continue;
                n++; var sw = System.Diagnostics.Stopwatch.StartNew();
                try { mi.Invoke(Activator.CreateInstance(t), null); Console.WriteLine($"PASS {mi.Name} {sw.Elapsed.TotalSeconds:0.0}s"); }
                catch (TargetInvocationException e) { fail++; Console.WriteLine($"FAIL {mi.Name} {sw.Elapsed.TotalSeconds:0.0}s: {e.InnerException.GetType().Name}: {e.InnerException.Message}"); }
            }
        Console.WriteLine($"{n - fail}/{n} passed"); return fail;
    }

    /// <summary>Re-run selected runs of the lead's results.jsonl through the harness and compare outcome, end time,
    /// collision object, path length and min clearance (bitwise expected: same Core, same seeds).</summary>
    static int TestLead(string repo, string[] args) {
        var ctx = Ctx(repo); var spec = ExperimentSpec.Load(Arg(args, "--spec"), repo);
        spec.Guard(ctx.Guard, false, out var runs, (s, m) => ctx.MissionNames(m, spec.Cross, spec.Corridor));
        var lead = File.ReadAllLines(Arg(args, "--lead")).Select(l => HJson.Obj(MiniJson.Parse(l))).Where(o => o != null).ToList();
        var only = (Arg(args, "--only") ?? "").Split(',', StringSplitOptions.RemoveEmptyEntries);
        var runner = new ExperimentRunner(ctx); int bad = 0, n = 0;
        foreach (var rs in runs) {
            if (only.Length > 0 && !only.Contains(rs.Mission)) continue;
            var L = lead.FirstOrDefault(o => HJson.Text(o, "Name", "") == rs.Mission && Math.Abs(HJson.Num(o, "VLimit", -1) - rs.Speed) < 1e-9 && (ulong)HJson.Num(o, "RunSeed", 0) == rs.RunSeed);
            if (L == null) continue;
            n++; var o2 = runner.Run(rs, null, "none"); var r = o2.Result;
            bool same = HJson.Text(L, "Outcome", "") == o2.Outcome && Math.Abs(HJson.Num(L, "TimeS", -1) - r.TimeS) < 1e-9 && HJson.Text(L, "CollisionObject", "") == r.CollisionObject
                && Math.Abs(HJson.Num(L, "PathLengthM", -1) - r.PathLengthM) < 1e-9 && Math.Abs(HJson.Num(L, "MinClearanceHardM", -1) - r.MinClearanceHardM) < 1e-12;
            if (!same) bad++;
            Console.WriteLine($"[test-lead] {rs.RunId} lead={HJson.Text(L, "Outcome", "")}@{HJson.Num(L, "TimeS", 0):0.000}s {HJson.Text(L, "CollisionObject", "")} path={HJson.Num(L, "PathLengthM", 0):R} | harness={o2.Outcome}@{r.TimeS:0.000}s {r.CollisionObject} path={r.PathLengthM:R} -> {(same ? "IDENTICAL" : "DIFFERENT")}");
        }
        Console.WriteLine($"[test-lead] {n - bad}/{n} identical"); return bad;
    }

    /// <summary>Export each straight mission as a NavPilot stand in the route frame (start at the origin, goal on +x),
    /// heights as AGL shifted so the CRANE cruise altitude lands on NavPilot's 2.0 m slab. Vertical capsules crossing
    /// the slab become stems (one per owner, the thickest), everything else in the 1-3 m band becomes branches.</summary>
    static int ExportNavPilot(string repo, string[] args) {
        var ctx = Ctx(repo); var spec = ExperimentSpec.Load(Arg(args, "--spec"), repo); string outDir = Arg(args, "--out"); Directory.CreateDirectory(outDir);
        spec.Guard(ctx.Guard, false, out _, (s, m) => ctx.MissionNames(m, spec.Cross, spec.Corridor));
        var ci = System.Globalization.CultureInfo.InvariantCulture; double cruise = new AutonomyConfig().CruiseAgl;
        foreach (var f in spec.Forests) {
            var w = ctx.World(f.manifest).w; ctx.World(f.manifest).thin.Restore();
            foreach (var m in ctx.Missions(f.manifest, spec.Cross, spec.Corridor, cruise)) {
                if (!m.Name.StartsWith("cross_")) continue;
                if (spec.Only.Count > 0 && !spec.Only.Contains(m.Name)) continue;
                var a = m.Route[0]; var g = m.Goal; double th = Math.Atan2(g.Y - a.Y, g.X - a.X), c = Math.Cos(th), s = Math.Sin(th), L = (g - a).NormXY, hw = 12.0;
                double X0 = -6, X1 = L + 8, Y0 = -hw - 4, Y1 = hw + 4;
                (double x, double y) ToL(double x, double y) { double dx = x - a.X, dy = y - a.Y; return (c * dx + s * dy, -s * dx + c * dy); }
                double minx = double.MaxValue, maxx = double.MinValue, miny = double.MaxValue, maxy = double.MinValue;
                foreach (var (lx, ly) in new[] { (X0, Y0), (X1, Y0), (X0, Y1), (X1, Y1) }) { double wx = a.X + c * lx - s * ly, wy = a.Y + s * lx + c * ly; minx = Math.Min(minx, wx); maxx = Math.Max(maxx, wx); miny = Math.Min(miny, wy); maxy = Math.Max(maxy, wy); }
                var stems = new Dictionary<int, (double x, double y, double r)>(); var branches = new List<object>();
                w.ForEachCapsuleIn(minx, maxx, miny, maxy, -1e3, 1e3, k => {
                    if (w.R[k] <= 0) return;
                    var pa = ToL(w.Ax[k], w.Ay[k]); var pb = ToL(w.Bx[k], w.By[k]);
                    if (Math.Max(pa.x, pb.x) < X0 || Math.Min(pa.x, pb.x) > X1 || Math.Max(pa.y, pb.y) < Y0 || Math.Min(pa.y, pb.y) > Y1) return;
                    double za = w.Az[k] - w.GroundHeight(w.Ax[k], w.Ay[k]) - cruise + 2.0, zb = w.Bz[k] - w.GroundHeight(w.Bx[k], w.By[k]) - cruise + 2.0;
                    double ux = w.Bx[k] - w.Ax[k], uy = w.By[k] - w.Ay[k], uz = w.Bz[k] - w.Az[k], len = Math.Sqrt(ux * ux + uy * uy + uz * uz);
                    bool vertical = len > 1e-6 && Math.Abs(uz) / len >= 0.8;
                    if (vertical && Math.Min(za, zb) <= 2.0 && Math.Max(za, zb) >= 2.0) {
                        double u = Math.Abs(zb - za) > 1e-9 ? (2.0 - za) / (zb - za) : 0.5; var p = (x: pa.x + u * (pb.x - pa.x), y: pa.y + u * (pb.y - pa.y));
                        int o = w.Owner[k]; if (!stems.TryGetValue(o, out var cur) || w.R[k] > cur.r) stems[o] = (p.x, p.y, w.R[k]);
                        return;
                    }
                    if (Math.Max(za, zb) < 1.0 || Math.Min(za, zb) > 3.0) return;
                    branches.Add(new Dictionary<string, object> { ["id"] = branches.Count, ["stem"] = w.Owner[k], ["p0"] = new[] { pa.x, pa.y, za }, ["p1"] = new[] { pb.x, pb.y, zb }, ["d0_m"] = 2.0 * w.R[k], ["d1_m"] = 2.0 * w.R[k], ["crane_capsule"] = k });
                });
                var st = new List<object>(); foreach (var kv in stems.OrderBy(q => q.Key)) st.Add(new Dictionary<string, object> { ["id"] = st.Count, ["x"] = kv.Value.x, ["y"] = kv.Value.y, ["dbh_m"] = 2.0 * kv.Value.r, ["height_m"] = 15.0, ["crane_owner"] = w.OwnerNames[kv.Key] });
                var stand = new Dictionary<string, object> { ["schema"] = "crane-navpilot-stand/1", ["generator"] = "crane-harness export-navpilot/1", ["seed"] = f.seed, ["split"] = "dev",
                    ["family"] = new Dictionary<string, object> { ["source"] = "crane forest manifest", ["mission"] = m.Name, ["route_heading_rad"] = th, ["route_origin_enu"] = a.ToArray(), ["cruise_agl_m"] = cruise },
                    ["corridor"] = new Dictionary<string, object> { ["length_m"] = L, ["half_width_m"] = hw, ["start"] = new[] { 0.0, 0.0 }, ["goal"] = new[] { L, 0.0 }, ["bounds"] = new[] { X0, X1, Y0, Y1 } },
                    ["clearings"] = new List<object>(), ["stems"] = st, ["branches"] = branches, ["targets"] = new List<object>() };
                File.WriteAllText(Path.Combine(outDir, $"stand_f{f.seed}_{m.Name}.json"), HJson.Write(stand));
                Console.WriteLine($"[export] {m.Name} L={L.ToString("0.0", ci)} m stems={st.Count} branches={branches.Count}");
            }
        }
        return 0;
    }
}
