using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;
using Crane.Fleet.Docking;
using Xunit;

namespace Crane.Fleet.Docking.Tests
{
    /// <summary>
    /// Parity with Tools/Fleet/docking_mc/model.py classify(). Vectors from parity/gen_parity_vectors.py.
    /// Capture must agree on every vector. Hook seating is allowed to differ in a precisely characterised way (see README).
    /// Set CRANE_PARITY_REPORT=path to write the list of differing vectors.
    /// </summary>
    public class ParityTests
    {
        public sealed class Vec
        {
            public string Source; public double G, A, R, TolY, TolPsi, H, Pitch, BandY, BandH, BandPsi; public double? Y, Psi;
            public bool Ok, PyCap; public int PyCode;
        }
        public sealed class Result { public Vec V; public bool Cap; public int Seated; public DockingOutcome Cs, Py; }

        private static (JsonElement meta, List<Vec> vecs) Load()
        {
            var doc = JsonDocument.Parse(File.ReadAllText(Path.Combine(AppContext.BaseDirectory, "parity_vectors.json")));
            var root = doc.RootElement;
            double D(JsonElement e, string n) => e.GetProperty(n).GetDouble();
            double? N(JsonElement e, string n) => e.GetProperty(n).ValueKind == JsonValueKind.Null ? (double?)null : e.GetProperty(n).GetDouble();
            var list = new List<Vec>();
            foreach (var e in root.GetProperty("vectors").EnumerateArray())
                list.Add(new Vec
                {
                    Source = e.GetProperty("source").GetString(), G = D(e, "g"), A = D(e, "a"), R = D(e, "r"), TolY = D(e, "tol_y"), TolPsi = D(e, "tol_psi"),
                    Y = N(e, "y"), Psi = N(e, "psi"), H = D(e, "h"), Pitch = D(e, "pitch"), BandY = D(e, "band_y"), BandH = D(e, "band_h"), BandPsi = D(e, "band_psi"),
                    Ok = e.GetProperty("ok").GetBoolean(), PyCap = e.GetProperty("py_cap").GetBoolean(), PyCode = e.GetProperty("py_code").GetInt32(),
                });
            return (root.GetProperty("meta"), list);
        }

        private static List<Result> Evaluate(JsonElement meta, List<Vec> vecs)
        {
            var prof = Fx.Profile();
            var assume = new DockingAssumptions();
            var res = new List<Result>();
            foreach (var v in vecs)
            {
                // g, a, r and tolerances are swept by the Python grid, so they come from the vector; everything else from profile/assumptions
                var m = new ContactModel(v.G, v.A, v.R, prof.MagnetHoldForce, prof.HookHalfSpan, v.TolY, v.TolPsi, prof.HookRatedLoad,
                                         prof.ConnectorComplianceLateral, prof.HookActuationTime, assume);
                bool cap = false; int seated = 0;
                if (v.Ok)
                {
                    cap = m.InsideEnvelope(v.Y.Value, v.Psi.Value, v.H, v.Pitch);
                    if (cap)
                    {
                        var r = m.Settle(v.Y.Value, v.Psi.Value, v.H, new Bands(v.BandY, v.BandPsi, v.BandH));
                        for (int h = 0; h < 2; h++) if (m.Seat(m.HookResidual(r, h), 1.0) == HookSeat.Seated) seated++;
                    }
                }
                res.Add(new Result { V = v, Cap = cap, Seated = seated, Cs = OutcomeClassifier.FromGeometry(cap, seated), Py = OutcomeClassifier.FromPythonCode(v.PyCode) });
            }
            return res;
        }

        [Fact]
        public void Vector_file_has_at_least_200_vectors_and_shared_constants_agree()
        {
            var (meta, vecs) = Load();
            Assert.True(vecs.Count >= 200, "vectors: " + vecs.Count);
            var prof = Fx.Profile(); var a = new DockingAssumptions();
            Assert.Equal(meta.GetProperty("py_ell").GetDouble(), a.GuideFaceHalfLength, 12);          // guide-face half length
            Assert.Equal(meta.GetProperty("py_hook_half_span").GetDouble(), prof.HookHalfSpan, 12);    // hook spacing/2 from the profile
            Assert.Equal(meta.GetProperty("py_band_y_sd").GetDouble(), prof.ConnectorComplianceLateral.Value, 12);
        }

        [Fact]
        public void Capture_agrees_on_every_vector()
        {
            var (meta, vecs) = Load();
            var bad = Evaluate(meta, vecs).Where(r => r.Cap != r.V.PyCap).ToList();
            Assert.True(bad.Count == 0, $"{bad.Count} capture disagreements, first: {bad.FirstOrDefault()?.V.Source}");
        }

        [Fact]
        public void Misses_and_captured_only_agree_exactly()
        {
            var (meta, vecs) = Load();
            var r = Evaluate(meta, vecs);
            Assert.All(r.Where(x => x.V.PyCode <= 1), x => Assert.Equal(x.Py, x.Cs));
            Assert.Contains(r, x => x.V.PyCode == 0); Assert.Contains(r, x => x.V.PyCode == 1);
        }

        [Fact]
        public void Csharp_hook_seating_is_never_more_permissive_than_python()
        {
            var (meta, vecs) = Load();
            foreach (var x in Evaluate(meta, vecs))
            {
                // python: both => centre residual within tol; one => best single hook within tol (ignores yaw tolerance)
                if (x.Cs == DockingOutcome.BothHooksVerified) Assert.Equal(3, x.V.PyCode);
                if (x.Cs == DockingOutcome.OneHook) Assert.True(x.V.PyCode >= 2);
            }
        }

        [Fact]
        public void Hook_disagreements_are_exactly_the_per_hook_and_yaw_tolerance_effects()
        {
            var (meta, vecs) = Load();
            var res = Evaluate(meta, vecs);
            var diff = res.Where(x => x.V.PyCode >= 2 && x.Cs != x.Py).ToList();
            Assert.NotEmpty(diff);              // the models do intentionally differ
            foreach (var d in diff)
            {
                // an explanation exists: the stricter C# rule fails either on the yaw lever arm at one hook or on the yaw tolerance
                var yS = Math.Sign(d.V.Y.Value) * Math.Min(Math.Abs(d.V.Y.Value), d.V.BandY);
                var pS = Math.Sign(d.V.Psi.Value) * Math.Min(Math.Abs(d.V.Psi.Value), d.V.BandPsi);
                double arm = 0.06 * Math.Sin(pS);
                double rh = Math.Min(Math.Abs(d.V.H), d.V.BandH);
                double worst = Math.Max(Math.Abs(yS + arm), Math.Abs(yS - arm));
                bool leverEffect = Math.Sqrt(worst * worst + rh * rh) > d.V.TolY - 1e-12;
                bool yawEffect = Math.Abs(pS) > d.V.TolPsi;
                Assert.True(leverEffect || yawEffect, $"unexplained difference: {d.V.Source} y={yS} psi={pS}");
            }
            var path = Environment.GetEnvironmentVariable("CRANE_PARITY_REPORT");
            if (!string.IsNullOrEmpty(path))
            {
                var lines = new List<string> { $"total={res.Count} capture_agree={res.Count(x => x.Cap == x.V.PyCap)} outcome_agree={res.Count(x => x.Cs == x.Py)} outcome_differ={res.Count(x => x.Cs != x.Py)}" };
                foreach (var g in res.Where(x => x.Cs != x.Py).GroupBy(x => $"python={x.Py} csharp={x.Cs}")) lines.Add($"{g.Key}: {g.Count()}");
                lines.Add("--- differing vectors (source, y_settled mm, psi_settled deg, tol_y mm, tol_psi deg, python, csharp)");
                foreach (var d in res.Where(x => x.Cs != x.Py))
                {
                    var yS = Math.Sign(d.V.Y.Value) * Math.Min(Math.Abs(d.V.Y.Value), d.V.BandY);
                    var pS = Math.Sign(d.V.Psi.Value) * Math.Min(Math.Abs(d.V.Psi.Value), d.V.BandPsi);
                    lines.Add($"{d.V.Source}, {yS * 1000:0.000}, {pS * 180 / Math.PI:0.000}, {d.V.TolY * 1000:0.0}, {d.V.TolPsi * 180 / Math.PI:0.0}, {d.Py}, {d.Cs}");
                }
                File.WriteAllLines(path, lines);
            }
        }
    }
}
