// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using Crane.Drones.Autonomy;

namespace Crane.Drones.Harness {
    /// <summary>Dependency-free JSON writer (shortest round-trip doubles, NaN/inf -> null) and typed readers over MiniJson.</summary>
    public static class HJson {
        static readonly CultureInfo CI = CultureInfo.InvariantCulture;
        public static string Write(object o) { var sb = new StringBuilder(); W(sb, o); return sb.ToString(); }

        static void W(StringBuilder sb, object o) {
            switch (o) {
                case null: sb.Append("null"); break;
                case string s: Str(sb, s); break;
                case bool b: sb.Append(b ? "true" : "false"); break;
                case double d: sb.Append(double.IsNaN(d) || double.IsInfinity(d) ? "null" : d.ToString("R", CI)); break;
                case float f: sb.Append(float.IsNaN(f) || float.IsInfinity(f) ? "null" : f.ToString("R", CI)); break;
                case int _: case long _: case ulong _: case uint _: case short _: sb.Append(Convert.ToString(o, CI)); break;
                case Vec3 v: W(sb, v.ToArray()); break;
                case IDictionary<string, object> m: {
                    sb.Append('{'); bool first = true;
                    foreach (var kv in m) { if (!first) sb.Append(','); first = false; Str(sb, kv.Key); sb.Append(':'); W(sb, kv.Value); }
                    sb.Append('}'); break;
                }
                case IEnumerable e: {
                    sb.Append('['); bool first = true;
                    foreach (var x in e) { if (!first) sb.Append(','); first = false; W(sb, x); }
                    sb.Append(']'); break;
                }
                default: Str(sb, o.ToString()); break;
            }
        }

        static void Str(StringBuilder sb, string s) {
            sb.Append('"');
            foreach (char c in s) {
                if (c == '"') sb.Append("\\\""); else if (c == '\\') sb.Append("\\\\"); else if (c == '\n') sb.Append("\\n");
                else if (c < 0x20) sb.Append("\\u").Append(((int)c).ToString("x4")); else sb.Append(c);
            }
            sb.Append('"');
        }

        public static Dictionary<string, object> Obj(object o) => o as Dictionary<string, object>;
        public static object Get(Dictionary<string, object> d, string k) => d != null && d.TryGetValue(k, out var v) ? v : null;
        public static double Num(Dictionary<string, object> d, string k, double def) => Get(d, k) is double x ? x : def;
        public static string Text(Dictionary<string, object> d, string k, string def) => Get(d, k) as string ?? def;
        public static bool Flag(Dictionary<string, object> d, string k, bool def) => Get(d, k) is bool b ? b : def;
        /// <summary>Numeric array: MiniJson returns double[] for all-number arrays, List&lt;object&gt; otherwise.</summary>
        public static double[] Nums(Dictionary<string, object> d, string k) {
            var v = Get(d, k);
            if (v is double[] a) return a;
            if (v is double s) return new[] { s };
            if (v is List<object> l) { var r = new List<double>(); foreach (var x in l) if (x is double y) r.Add(y); return r.ToArray(); }
            return null;
        }
        public static List<string> Strings(Dictionary<string, object> d, string k) {
            var r = new List<string>();
            if (Get(d, k) is List<object> l) foreach (var x in l) if (x is string s) r.Add(s);
            if (Get(d, k) is string one) r.Add(one);
            return r;
        }
        public static List<object> List(Dictionary<string, object> d, string k) {
            var v = Get(d, k);
            if (v is List<object> l) return l;
            if (v is double[] a) { var r = new List<object>(); foreach (var x in a) r.Add(x); return r; }
            return new List<object>();
        }

        /// <summary>FNV-1a 64-bit hex (stable across runtimes; used for run ids and spec digests).</summary>
        public static string Fnv64(string s) {
            ulong h = 14695981039346656037UL;
            foreach (byte b in Encoding.UTF8.GetBytes(s)) { h ^= b; h *= 1099511628211UL; }
            return h.ToString("x16");
        }
    }
}
