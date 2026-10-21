using System;
using System.Collections.Generic;
using System.Security.Cryptography;
using System.Text;
using UnityEngine;

namespace Sim.Underwater {
    public sealed class CraneProfileException : Exception {
        public CraneProfileException(string m) : base(m) { }
    }

    /// <summary>
    /// Loads a crane.underwater.vehicle_profile/v1 JSON (same schema as Tools/Underwater/vehicle_profile.py).
    /// Fails closed: bare numbers, unknown sources, missing locations and a PHYSICALLY_CALIBRATED claim are rejected.
    /// The SHA-256 of the exact bytes loaded is kept for replay metadata.
    /// </summary>
    public sealed class CraneUnderwaterProfile {
        public const string Schema = "crane.underwater.vehicle_profile/v1";
        public const string ResourcePrefix = "Underwater/";
        static readonly HashSet<string> Labels = new HashSet<string> {
            "IMPLEMENTED", "FIXTURE_VALIDATED", "LIVE_SIM_VALIDATED", "PHYSICALLY_CALIBRATED", "UNKNOWN", "NOT_RUN"
        };

        public string Name, EvidenceLabel, Sha256, SourceDescription;
        public CraneVehicleParams Params;
        public List<CraneThruster> Thrusters;

        public static CraneUnderwaterProfile LoadFromResources(string name) {
            var asset = Resources.Load<TextAsset>(ResourcePrefix + name);
            if (asset == null) throw new CraneProfileException($"underwater profile Resources/{ResourcePrefix}{name}.json not found");
            var p = LoadFromBytes(asset.bytes);
            p.SourceDescription = $"Resources/{ResourcePrefix}{name}";
            return p;
        }

        public static CraneUnderwaterProfile LoadFromBytes(byte[] bytes) {
            int skip = bytes.Length >= 3 && bytes[0] == 0xEF && bytes[1] == 0xBB && bytes[2] == 0xBF ? 3 : 0;
            var prof = Parse(Encoding.UTF8.GetString(bytes, skip, bytes.Length - skip));
            using (var sha = SHA256.Create()) prof.Sha256 = Hex(sha.ComputeHash(bytes));
            return prof;
        }

        static string Hex(byte[] h) {
            var sb = new StringBuilder(h.Length * 2);
            foreach (byte b in h) sb.Append(b.ToString("x2"));
            return sb.ToString();
        }

        public static CraneUnderwaterProfile Parse(string json) {
            var root = CraneMiniJson.Parse(json) as Dictionary<string, object> ?? throw new CraneProfileException("root must be an object");
            if (!(Obj(root, "schema") is string schema) || schema != Schema) throw new CraneProfileException($"schema must be {Schema}");
            string label = Obj(root, "evidenceLabel") as string;
            if (label == null || !Labels.Contains(label)) throw new CraneProfileException("evidenceLabel must be a CONTRACTS.md label");
            if (label == "PHYSICALLY_CALIBRATED") throw new CraneProfileException("a literature profile cannot claim PHYSICALLY_CALIBRATED");
            var sources = Dict(root, "sources");
            var rb = Dict(root, "rigidBody");
            var hy = Dict(root, "hydrodynamics");
            var th = Dict(root, "thrusters");

            var p = new CraneVehicleParams { Name = (Obj(root, "vehicle") as string) ?? "" };
            p.Mass = Num(rb, "massKg", sources);
            var ic = Dict(rb, "inertiaCgKgM2");
            p.InertiaCg = CraneLinAlg.Diag(Num(ic, "Ixx", sources, "inertia.Ixx"), Num(ic, "Iyy", sources, "inertia.Iyy"), Num(ic, "Izz", sources, "inertia.Izz"));
            p.RG = Vec3(Cited(rb, "rGMeters", sources));
            p.RB = Vec3(Cited(rb, "rBMeters", sources));
            p.Buoyancy = Num(rb, "buoyancyN", sources);
            if (rb.ContainsKey("gravityMps2")) p.Gravity = Num(rb, "gravityMps2", sources);
            if (rb.ContainsKey("weightN")) {
                double w = Num(rb, "weightN", sources);
                if (Math.Abs(w - p.Mass * p.Gravity) > 1e-3 * w) throw new CraneProfileException($"weightN {w} inconsistent with massKg*g {p.Mass * p.Gravity:F3}");
            }
            string[] dofs = { "u", "v", "w", "p", "q", "r" };
            var am = Dict(hy, "addedMass"); var ld = Dict(hy, "linearDamping"); var qd = Dict(hy, "quadraticDamping");
            var amD = new double[6]; var ldD = new double[6];
            for (int i = 0; i < 6; i++) {
                amD[i] = Num(am, dofs[i], sources, "addedMass." + dofs[i]);
                ldD[i] = Num(ld, dofs[i], sources, "linearDamping." + dofs[i]);
                p.QuadraticDamping[i] = Num(qd, dofs[i], sources, "quadraticDamping." + dofs[i]);
            }
            p.AddedMass = CraneLinAlg.Diag(amD);
            p.LinearDamping = CraneLinAlg.Diag(ldD);
            try { p.Validate(); } catch (ArgumentException e) { throw new CraneProfileException(e.Message); }

            double fwd = Num(th, "maxForwardN", sources), rev = Num(th, "maxReverseN", sources), tau = Num(th, "timeConstantS", sources);
            double deadband = th.ContainsKey("deadbandN") ? Num(th, "deadbandN", sources) : 0.0;
            var layout = Cited(th, "layout", sources) as List<object> ?? throw new CraneProfileException("thrusters.layout must be an array");
            var list = new List<CraneThruster>();
            foreach (var e in layout) {
                var d = e as Dictionary<string, object> ?? throw new CraneProfileException("thruster entry must be an object");
                list.Add(new CraneThruster((Obj(d, "name") as string) ?? "", Vec3(Obj(d, "positionM")), Vec3(Obj(d, "direction")), fwd, rev, tau, deadband));
            }
            return new CraneUnderwaterProfile { Name = p.Name, EvidenceLabel = label, Params = p, Thrusters = list };
        }

        static object Obj(Dictionary<string, object> d, string k) =>
            d.TryGetValue(k, out var v) ? v : throw new CraneProfileException($"missing key '{k}'");

        static Dictionary<string, object> Dict(Dictionary<string, object> d, string k) =>
            Obj(d, k) as Dictionary<string, object> ?? throw new CraneProfileException($"'{k}' must be an object");

        static object Cited(Dictionary<string, object> d, string key, Dictionary<string, object> sources, string path = null) {
            path = path ?? key;
            if (!(Obj(d, key) is Dictionary<string, object> n) || !n.ContainsKey("value"))
                throw new CraneProfileException($"{path}: coefficient must be {{value, source, location}}");
            if (!(n.TryGetValue("source", out var src) && src is string s && sources.ContainsKey(s)))
                throw new CraneProfileException($"{path}: unknown source id");
            if (!(n.TryGetValue("location", out var loc) && loc is string ls && ls.Length > 0))
                throw new CraneProfileException($"{path}: missing location");
            return n["value"];
        }

        static double Num(Dictionary<string, object> d, string key, Dictionary<string, object> sources, string path = null) {
            var v = Cited(d, key, sources, path);
            return v is double x ? x : throw new CraneProfileException($"{path ?? key}: value must be a number");
        }

        static double[] Vec3(object o) {
            var l = o as List<object>;
            if (l == null || l.Count != 3) throw new CraneProfileException("expected a 3-vector");
            return new[] { (double)l[0], (double)l[1], (double)l[2] };
        }
    }
}
