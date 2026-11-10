using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace Crane.Fleet {
    /// <summary>Dependency-free JSON (objects are Dictionary&lt;string, object&gt;, arrays List&lt;object&gt;,
    /// numbers double). Plain C# so the core builds inside and outside Unity.</summary>
    public static class FleetJson {
        public static object Parse(string text) {
            int i = 0;
            object v = Value(text, ref i);
            Skip(text, ref i);
            if (i != text.Length) throw new FormatException($"trailing characters at {i}");
            return v;
        }

        static void Skip(string s, ref int i) {
            while (i < s.Length && char.IsWhiteSpace(s[i])) i++;
        }

        static object Value(string s, ref int i) {
            Skip(s, ref i);
            if (i >= s.Length) throw new FormatException("unexpected end");
            char c = s[i];
            if (c == '{') {
                var d = new Dictionary<string, object>();
                i++; Skip(s, ref i);
                if (s[i] == '}') { i++; return d; }
                while (true) {
                    Skip(s, ref i);
                    string k = Str(s, ref i);
                    Skip(s, ref i);
                    if (s[i] != ':') throw new FormatException($"':' expected at {i}");
                    i++;
                    d[k] = Value(s, ref i);
                    Skip(s, ref i);
                    if (s[i] == ',') { i++; continue; }
                    if (s[i] == '}') { i++; return d; }
                    throw new FormatException($"',' or '}}' expected at {i}");
                }
            }
            if (c == '[') {
                var a = new List<object>();
                i++; Skip(s, ref i);
                if (s[i] == ']') { i++; return a; }
                while (true) {
                    a.Add(Value(s, ref i));
                    Skip(s, ref i);
                    if (s[i] == ',') { i++; continue; }
                    if (s[i] == ']') { i++; return a; }
                    throw new FormatException($"',' or ']' expected at {i}");
                }
            }
            if (c == '"') return Str(s, ref i);
            if (s.Length - i >= 4 && string.CompareOrdinal(s, i, "true", 0, 4) == 0) { i += 4; return true; }
            if (s.Length - i >= 5 && string.CompareOrdinal(s, i, "false", 0, 5) == 0) { i += 5; return false; }
            if (s.Length - i >= 4 && string.CompareOrdinal(s, i, "null", 0, 4) == 0) { i += 4; return null; }
            int start = i;
            while (i < s.Length && "+-0123456789.eE".IndexOf(s[i]) >= 0) i++;
            if (start == i) throw new FormatException($"unexpected '{c}' at {i}");
            return double.Parse(s.Substring(start, i - start), NumberStyles.Float, CultureInfo.InvariantCulture);
        }

        static string Str(string s, ref int i) {
            if (s[i] != '"') throw new FormatException($"string expected at {i}");
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

        // ---- writing (deterministic: keys in insertion order, numbers round-trip invariant) ----

        public static string Num(double v) {
            if (double.IsNaN(v) || double.IsInfinity(v)) return "null";
            return v.ToString("R", CultureInfo.InvariantCulture);
        }

        public static string Quote(string s) {
            if (s == null) return "null";
            var b = new StringBuilder("\"");
            foreach (char c in s) {
                switch (c) {
                    case '"': b.Append("\\\""); break;
                    case '\\': b.Append("\\\\"); break;
                    case '\n': b.Append("\\n"); break;
                    case '\r': b.Append("\\r"); break;
                    case '\t': b.Append("\\t"); break;
                    default:
                        if (c < 0x20) b.Append("\\u").Append(((int)c).ToString("x4")); else b.Append(c);
                        break;
                }
            }
            return b.Append('"').ToString();
        }

        public static string Write(object v, bool pretty = false) {
            var b = new StringBuilder();
            Write(b, v, pretty, 0);
            return b.ToString();
        }

        static void Write(StringBuilder b, object v, bool pretty, int depth) {
            string nl = pretty ? "\n" + new string(' ', 2 * (depth + 1)) : "";
            string end = pretty ? "\n" + new string(' ', 2 * depth) : "";
            switch (v) {
                case null: b.Append("null"); break;
                case string s: b.Append(Quote(s)); break;
                case bool x: b.Append(x ? "true" : "false"); break;
                case double d: b.Append(Num(d)); break;
                case float f: b.Append(Num(f)); break;
                case int n: b.Append(n.ToString(CultureInfo.InvariantCulture)); break;
                case long l: b.Append(l.ToString(CultureInfo.InvariantCulture)); break;
                case IDictionary<string, object> d: {
                    b.Append('{');
                    bool first = true;
                    foreach (var kv in d) {
                        if (!first) b.Append(',');
                        first = false;
                        b.Append(nl).Append(Quote(kv.Key)).Append(pretty ? ": " : ":");
                        Write(b, kv.Value, pretty, depth + 1);
                    }
                    if (!first) b.Append(end);
                    b.Append('}');
                    break;
                }
                case System.Collections.IEnumerable e: {
                    b.Append('[');
                    bool first = true;
                    foreach (object x in e) {
                        if (!first) b.Append(pretty ? ", " : ",");
                        first = false;
                        Write(b, x, false, depth + 1);
                    }
                    b.Append(']');
                    break;
                }
                default: b.Append(Quote(Convert.ToString(v, CultureInfo.InvariantCulture))); break;
            }
        }
    }

    /// <summary>Typed access into a parsed JSON tree with paths like "chassis.wheel.radius". A leaf of
    /// the form {"value": x, "provenance": p} reads as x; <see cref="Provenance"/> returns p.</summary>
    public sealed class JsonNode {
        public readonly object Raw;
        public readonly string Path;

        public JsonNode(object raw, string path = "$") { Raw = raw; Path = path; }

        public static JsonNode FromText(string text) => new JsonNode(FleetJson.Parse(text));

        public bool Has(string key) => Raw is Dictionary<string, object> d && d.ContainsKey(key);

        public JsonNode this[string path] {
            get {
                object cur = Raw;
                string at = Path;
                foreach (string part in path.Split('.')) {
                    at += "." + part;
                    if (!(cur is Dictionary<string, object> d) || !d.TryGetValue(part, out cur))
                        throw new KeyNotFoundException($"fleet spec has no '{at}'");
                }
                return new JsonNode(cur, at);
            }
        }

        public JsonNode TryGet(string path) {
            try { return this[path]; }
            catch (KeyNotFoundException) { return null; }
        }

        /// <summary>The value of a provenance-labelled leaf, else the node itself.</summary>
        public object Unwrapped => Raw is Dictionary<string, object> d && d.ContainsKey("value") && d.ContainsKey("provenance")
            ? d["value"] : Raw;

        public string Provenance => Raw is Dictionary<string, object> d && d.TryGetValue("provenance", out object p)
            ? p as string : null;

        public double Double {
            get {
                object v = Unwrapped;
                if (v is double d) return d;
                throw new FormatException($"{Path} is not a number");
            }
        }

        public string String => Unwrapped as string ?? throw new FormatException($"{Path} is not a string");
        public bool Bool => Unwrapped is bool b ? b : throw new FormatException($"{Path} is not a bool");

        public double[] Doubles {
            get {
                if (!(Unwrapped is List<object> a)) throw new FormatException($"{Path} is not an array");
                var r = new double[a.Count];
                for (int i = 0; i < a.Count; i++) r[i] = a[i] is double d ? d : throw new FormatException($"{Path}[{i}] is not a number");
                return r;
            }
        }

        public List<JsonNode> Items {
            get {
                if (!(Unwrapped is List<object> a)) throw new FormatException($"{Path} is not an array");
                var r = new List<JsonNode>();
                for (int i = 0; i < a.Count; i++) r.Add(new JsonNode(a[i], $"{Path}[{i}]"));
                return r;
            }
        }

        public IEnumerable<KeyValuePair<string, JsonNode>> Members {
            get {
                if (!(Raw is Dictionary<string, object> d)) throw new FormatException($"{Path} is not an object");
                foreach (var kv in d) yield return new KeyValuePair<string, JsonNode>(kv.Key, new JsonNode(kv.Value, Path + "." + kv.Key));
            }
        }
    }
}
