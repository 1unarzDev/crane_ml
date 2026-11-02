using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace Crane.Theater {
    /// <summary>A small JSON reader and writer (objects as Dictionary, arrays as List, numbers as double),
    /// so the theater depends on no domain assembly.</summary>
    public static class TJson {
        public static object Parse(string text) {
            int i = 0;
            object v = Value(text, ref i);
            Ws(text, ref i);
            if (i != text.Length) throw new FormatException($"json: trailing data at {i}");
            return v;
        }

        static void Ws(string s, ref int i) { while (i < s.Length && char.IsWhiteSpace(s[i])) i++; }

        static object Value(string s, ref int i) {
            Ws(s, ref i);
            if (i >= s.Length) throw new FormatException("json: unexpected end");
            char c = s[i];
            if (c == '{') {
                var o = new Dictionary<string, object>();
                i++; Ws(s, ref i);
                if (s[i] == '}') { i++; return o; }
                while (true) {
                    Ws(s, ref i);
                    string k = Str(s, ref i);
                    Ws(s, ref i);
                    if (s[i] != ':') throw new FormatException($"json: ':' expected at {i}");
                    i++;
                    o[k] = Value(s, ref i);
                    Ws(s, ref i);
                    if (s[i] == ',') { i++; continue; }
                    if (s[i] == '}') { i++; return o; }
                    throw new FormatException($"json: ',' or '}}' expected at {i}");
                }
            }
            if (c == '[') {
                var a = new List<object>();
                i++; Ws(s, ref i);
                if (s[i] == ']') { i++; return a; }
                while (true) {
                    a.Add(Value(s, ref i));
                    Ws(s, ref i);
                    if (s[i] == ',') { i++; continue; }
                    if (s[i] == ']') { i++; return a; }
                    throw new FormatException($"json: ',' or ']' expected at {i}");
                }
            }
            if (c == '"') return Str(s, ref i);
            if (s.Length - i >= 4 && string.CompareOrdinal(s, i, "true", 0, 4) == 0) { i += 4; return true; }
            if (s.Length - i >= 5 && string.CompareOrdinal(s, i, "false", 0, 5) == 0) { i += 5; return false; }
            if (s.Length - i >= 4 && string.CompareOrdinal(s, i, "null", 0, 4) == 0) { i += 4; return null; }
            int st = i;
            while (i < s.Length && "+-0123456789.eE".IndexOf(s[i]) >= 0) i++;
            if (st == i) throw new FormatException($"json: value expected at {i}");
            return double.Parse(s.Substring(st, i - st), NumberStyles.Float, CultureInfo.InvariantCulture);
        }

        static string Str(string s, ref int i) {
            if (s[i] != '"') throw new FormatException($"json: string expected at {i}");
            i++;
            var b = new StringBuilder();
            while (s[i] != '"') {
                char c = s[i++];
                if (c != '\\') { b.Append(c); continue; }
                char e = s[i++];
                switch (e) {
                    case 'n': b.Append('\n'); break;
                    case 't': b.Append('\t'); break;
                    case 'r': b.Append('\r'); break;
                    case 'b': b.Append('\b'); break;
                    case 'f': b.Append('\f'); break;
                    case 'u': b.Append((char)int.Parse(s.Substring(i, 4), NumberStyles.HexNumber)); i += 4; break;
                    default: b.Append(e); break;
                }
            }
            i++;
            return b.ToString();
        }

        // ---- typed access
        public static Dictionary<string, object> Obj(object o, string key) =>
            o is Dictionary<string, object> d && d.TryGetValue(key, out var v) ? v as Dictionary<string, object> : null;
        public static List<object> Arr(object o, string key) =>
            o is Dictionary<string, object> d && d.TryGetValue(key, out var v) ? v as List<object> : null;
        public static string Str(object o, string key, string fallback = null) =>
            o is Dictionary<string, object> d && d.TryGetValue(key, out var v) && v is string s ? s : fallback;
        public static double Num(object o, string key, double fallback = 0) =>
            o is Dictionary<string, object> d && d.TryGetValue(key, out var v) ? v is double x ? x : v is bool b ? (b ? 1 : 0) : fallback : fallback;
        public static bool Bool(object o, string key, bool fallback = false) =>
            o is Dictionary<string, object> d && d.TryGetValue(key, out var v) ? v is bool b ? b : v is double x ? x != 0 : fallback : fallback;
        public static bool Has(object o, string key) => o is Dictionary<string, object> d && d.ContainsKey(key) && d[key] != null;

        // ---- writing (sorted keys are the caller's business; invariant culture, round-trip floats)
        public static string Write(object v) { var b = new StringBuilder(); W(b, v); return b.ToString(); }

        static void W(StringBuilder b, object v) {
            switch (v) {
                case null: b.Append("null"); break;
                case string s: Q(b, s); break;
                case bool x: b.Append(x ? "true" : "false"); break;
                case double d: b.Append(F(d)); break;
                case float f: b.Append(F(f)); break;
                case int n: b.Append(n.ToString(CultureInfo.InvariantCulture)); break;
                case long n: b.Append(n.ToString(CultureInfo.InvariantCulture)); break;
                case IDictionary<string, object> o: {
                    b.Append('{'); bool first = true;
                    foreach (var kv in o) { if (!first) b.Append(','); first = false; Q(b, kv.Key); b.Append(':'); W(b, kv.Value); }
                    b.Append('}'); break;
                }
                case System.Collections.IEnumerable a: {
                    b.Append('['); bool first = true;
                    foreach (var x in a) { if (!first) b.Append(','); first = false; W(b, x); }
                    b.Append(']'); break;
                }
                default: Q(b, v.ToString()); break;
            }
        }

        public static string F(double d) =>
            double.IsNaN(d) || double.IsInfinity(d) ? "null" : Math.Round(d, 6).ToString("R", CultureInfo.InvariantCulture);

        static void Q(StringBuilder b, string s) {
            b.Append('"');
            foreach (char c in s) {
                if (c == '"' || c == '\\') b.Append('\\').Append(c);
                else if (c == '\n') b.Append("\\n");
                else if (c < 0x20) b.Append("\\u").Append(((int)c).ToString("x4"));
                else b.Append(c);
            }
            b.Append('"');
        }
    }
}
