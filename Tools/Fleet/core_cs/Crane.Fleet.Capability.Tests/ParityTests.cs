using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.Json;
using Crane.Fleet;
using Xunit;

namespace Crane.Fleet.Capability.Tests
{
    /// <summary>Compares the C# motion capability with the independent NumPy reference over generated configurations.
    /// Writes the largest observed differences to CRANE_PARITY_REPORT if that variable is set.</summary>
    public class ParityTests
    {
        [Theory]
        [InlineData("capability_vectors.json")]
        [InlineData("testbed_vectors.json")]
        public void CSharpMatchesPythonReference_OnAllVectors(string file)
        {
            var path = Path.Combine(AppContext.BaseDirectory, file);
            using var doc = JsonDocument.Parse(File.ReadAllText(path));
            var vecs = doc.RootElement.GetProperty("vectors");
            Assert.True(vecs.GetArrayLength() >= 200, "need >= 200 parity configurations");
            double tolScalar = doc.RootElement.GetProperty("tolerances").GetProperty("scalar").GetDouble();
            double tolPoly = doc.RootElement.GetProperty("tolerances").GetProperty("polytope").GetDouble();
            var worst = new Dictionary<string, double>();
            var failures = new List<string>();
            int n = 0, holonomic = 0, rankDef = 0, nonComparable = 0;
            var modeCounts = new Dictionary<string, int>();

            foreach (var v in vecs.EnumerateArray())
            {
                n++;
                var id = v.GetProperty("id").GetString();
                var asm = Load(v.GetProperty("assembly"), modeCounts);
                var e = v.GetProperty("expect");
                var c = MotionAnalyzer.Analyze(asm);
                if (c.Holonomic) holonomic++;
                if (c.Mobility < 3) rankDef++;

                void Check(string what, double got, double want, double tol)
                {
                    double d = (double.IsInfinity(want) && double.IsInfinity(got)) ? 0 : Math.Abs(got - want);
                    if (double.IsNaN(d)) d = double.PositiveInfinity;
                    worst[what] = Math.Max(worst.TryGetValue(what, out var w) ? w : 0, d);
                    if (d > tol) failures.Add($"{id} {what}: got {got} want {want}");
                }
                if (c.RankC != e.GetProperty("rank_c").GetInt32()) failures.Add($"{id} rank_c {c.RankC} vs {e.GetProperty("rank_c").GetInt32()}");
                if (c.Mobility != e.GetProperty("mobility").GetInt32()) failures.Add($"{id} mobility");
                if (c.ActuatedRank != e.GetProperty("actuated_rank").GetInt32()) failures.Add($"{id} actuated_rank {c.ActuatedRank} vs {e.GetProperty("actuated_rank").GetInt32()}");
                var ids = e.GetProperty("driven_ids").EnumerateArray().Select(x => x.GetString()).ToArray();
                if (!ids.SequenceEqual(c.DrivenWheelIds)) { failures.Add($"{id} driven ids differ"); continue; }
                var A = e.GetProperty("A").EnumerateArray().Select(r => r.EnumerateArray().Select(x => x.GetDouble()).ToArray()).ToArray();
                for (int i = 0; i < A.Length; i++) for (int j = 0; j < 3; j++) Check("A", c.A[i][j], A[i][j], 1e-9 * Math.Max(1, Math.Abs(A[i][j])));
                var L = e.GetProperty("limits").EnumerateArray().Select(x => x.GetDouble()).ToArray();
                for (int i = 0; i < L.Length; i++) Check("limits", c.SpeedLimit[i], L[i], tolScalar * Math.Max(1, L[i]));
                if (e.TryGetProperty("B", out var Bp)) { var B = Bp.EnumerateArray().Select(r => r.EnumerateArray().Select(x => x.GetDouble()).ToArray()).ToArray();
                for (int i = 0; i < 3; i++) for (int j = 0; j < L.Length; j++) Check("B", c.B[i][j], B[i][j], 1e-8 * Math.Max(1, Math.Abs(B[i][j]))); }
                CheckMat(e.GetProperty("gram_hard"), GramHard(c), "gram_hard", id, Check, 1e-9);
                CheckMat(e.GetProperty("projector_mobility"), c.ProjectorMobility, "proj_mobility", id, Check, 1e-8);
                if (e.TryGetProperty("projector_controlled", out var pcp)) CheckMat(pcp, c.ProjectorControlled, "proj_controlled", id, Check, 1e-8);
                if (!e.TryGetProperty("axis_bounds", out var abp)) { nonComparable++; continue; }
                var ab = abp.EnumerateArray().Select(x => x.ValueKind == JsonValueKind.Null ? double.PositiveInfinity : x.GetDouble()).ToArray();
                var gb = c.AxisBounds();
                for (int i = 0; i < 6; i++) Check("axis_bound", gb[i], ab[i], 1e-8 * Math.Max(1, ab[i]));
                foreach (var r in e.GetProperty("rays").EnumerateArray())
                {
                    var u = r.GetProperty("u").EnumerateArray().Select(x => x.GetDouble()).ToArray();
                    var s = r.GetProperty("s"); double want = s.ValueKind == JsonValueKind.Null ? double.PositiveInfinity : s.GetDouble();
                    Check("ray", c.MaxAlong(u), want, 1e-8 * Math.Max(1, want));
                }
                foreach (var r in e.GetProperty("support").EnumerateArray())
                {
                    var u = r.GetProperty("u").EnumerateArray().Select(x => x.GetDouble()).ToArray();
                    Check("support", c.Support(u), r.GetProperty("h").GetDouble(), tolPoly * 10);
                }
            }
            var rep = Environment.GetEnvironmentVariable("CRANE_PARITY_REPORT");
            if (!string.IsNullOrEmpty(rep))
            {
                var sb = new StringBuilder();
                sb.AppendLine($"[{file}]");
                sb.AppendLine($"vectors={n} subspace_not_comparable={nonComparable} holonomic={holonomic} mobility_lt_3={rankDef} failures={failures.Count}");
                foreach (var kv in worst.OrderBy(k => k.Key)) sb.AppendLine($"max_abs_diff[{kv.Key}]={kv.Value:E3}");
                foreach (var kv in modeCounts.OrderBy(k => k.Key)) sb.AppendLine($"wheels[{kv.Key}]={kv.Value}");
                foreach (var f in failures.Take(40)) sb.AppendLine("FAIL " + f);
                File.AppendAllText(rep, sb.ToString());
            }
            Assert.True(failures.Count == 0, string.Join("\n", failures.Take(15)));
            Assert.True(holonomic > 5 && rankDef > 20, "vector set should span holonomic and constrained cases");
        }

        private static double[,] GramHard(MotionCapability c)
        {
            var g = new double[3, 3];
            foreach (var h in c.Hard) for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) g[i, j] += h.Row[i] * h.Row[j];
            return g;
        }

        private static void CheckMat(JsonElement want, double[,] got, string what, string id, Action<string, double, double, double> check, double tol)
        {
            int i = 0;
            foreach (var row in want.EnumerateArray())
            {
                int j = 0;
                foreach (var x in row.EnumerateArray()) { check(what, got[i, j], x.GetDouble(), tol * Math.Max(1, Math.Abs(x.GetDouble()))); j++; }
                i++;
            }
        }

        private static CapAssembly Load(JsonElement a, Dictionary<string, int> modeCounts)
        {
            var asm = new CapAssembly { Id = a.GetProperty("id").GetString() };
            foreach (var m in a.GetProperty("modules").EnumerateArray())
            {
                var mod = new CapModule { Id = m.GetProperty("id").GetString(), X = m.GetProperty("x").GetDouble(), Y = m.GetProperty("y").GetDouble(), Yaw = m.GetProperty("yaw").GetDouble() };
                foreach (var w in m.GetProperty("wheels").EnumerateArray())
                {
                    var h = w.GetProperty("health");
                    var mode = h.GetProperty("mode").GetString();
                    modeCounts[mode] = modeCounts.TryGetValue(mode, out var k) ? k + 1 : 1;
                    mod.Wheels.Add(new CapWheel
                    {
                        Id = w.GetProperty("id").GetString(), Kind = CapabilityBuilders.ParseKind(w.GetProperty("kind").GetString()),
                        X = w.GetProperty("x").GetDouble(), Y = w.GetProperty("y").GetDouble(), DriveYaw = w.GetProperty("drive_yaw").GetDouble(),
                        Radius = w.GetProperty("radius").GetDouble(), RollerAngle = w.GetProperty("roller_angle").GetDouble(),
                        MaxSpeed = w.GetProperty("max_speed").GetDouble(), Driven = w.GetProperty("driven").GetBoolean(),
                        Health = CapabilityBuilders.ParseHealth(mode, h.GetProperty("gain").GetDouble()),
                        Lateral = w.GetProperty("lateral").GetString() == "skid" ? CapLateralMode.Skid : CapLateralMode.Hard,
                        NormalLoad = w.GetProperty("normal_load").GetDouble(), LateralFriction = w.GetProperty("lateral_mu").GetDouble(),
                    });
                }
                asm.Modules.Add(mod);
            }
            return asm;
        }
    }
}
