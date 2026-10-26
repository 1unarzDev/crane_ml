using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace Sim.Course {
    /// <summary>An ordered JSON object. Truth, constraint spec and bad layouts all share this
    /// tree, so the generator self-check and the validator read the same data.</summary>
    public sealed class JObj : List<KeyValuePair<string, object>> {
        public object this[string key] {
            get {
                foreach (var pair in this) if (pair.Key == key) return pair.Value;
                return null;
            }
            set {
                for (int i = 0; i < Count; i++)
                    if (this[i].Key == key) { this[i] = new KeyValuePair<string, object>(key, value); return; }
                Add(new KeyValuePair<string, object>(key, value));
            }
        }

        public JObj Set(string key, object value) { this[key] = value; return this; }
        public bool Has(string key) { foreach (var pair in this) if (pair.Key == key) return true; return false; }
        public double Num(string key, double fallback = double.NaN) => this[key] is double v ? v : fallback;
        public string Str(string key) => this[key] as string;
        public JObj Obj(string key) => this[key] as JObj;
        public List<object> Arr(string key) => this[key] as List<object> ?? new List<object>();
    }

    /// <summary>Minimal JSON: numbers are doubles; the writer is canonical (insertion order, at
    /// most 6 decimals, no exponent) so the same tree always gives the same bytes and sha256.</summary>
    public static class Json {
        public static object Parse(string text) {
            int i = 0;
            object value = Value(text, ref i);
            Skip(text, ref i);
            if (i != text.Length) throw new FormatException($"trailing JSON at {i}");
            return value;
        }

        public static string Write(object value, bool pretty = false) {
            var sb = new StringBuilder();
            Emit(sb, value, pretty ? 0 : -1);
            return sb.ToString();
        }

        /// <summary>A double as canonical text: rounded to 6 decimals, "-0" printed as "0".</summary>
        public static string Number(double v) {
            if (double.IsNaN(v) || double.IsInfinity(v)) throw new ArgumentException("JSON has no NaN");
            v = Math.Round(v, 6, MidpointRounding.AwayFromZero);
            if (v == 0) v = 0;
            return v.ToString("0.######", CultureInfo.InvariantCulture);
        }

        /// <summary>Dotted path lookup ("indicator.color"); null when absent.</summary>
        public static object Path(object node, string path) {
            if (string.IsNullOrEmpty(path)) return node;
            foreach (string part in path.Split('.')) {
                if (!(node is JObj obj)) return null;
                node = obj[part];
            }
            return node;
        }

        static void Emit(StringBuilder sb, object v, int indent) {
            switch (v) {
                case null: sb.Append("null"); break;
                case bool b: sb.Append(b ? "true" : "false"); break;
                case string s: Quote(sb, s); break;
                case double d: sb.Append(Number(d)); break;
                case float f: sb.Append(Number(f)); break;
                case int n: sb.Append(n.ToString(CultureInfo.InvariantCulture)); break;
                case long n: sb.Append(n.ToString(CultureInfo.InvariantCulture)); break;
                case JObj o:
                    sb.Append('{');
                    for (int i = 0; i < o.Count; i++) {
                        if (i > 0) sb.Append(',');
                        Break(sb, indent < 0 ? -1 : indent + 1);
                        Quote(sb, o[i].Key);
                        sb.Append(indent >= 0 ? ": " : ":");
                        Emit(sb, o[i].Value, indent < 0 ? -1 : indent + 1);
                    }
                    if (o.Count > 0) Break(sb, indent);
                    sb.Append('}');
                    break;
                case System.Collections.IEnumerable list:
                    sb.Append('[');
                    bool first = true, nested = false;
                    foreach (object item in list) {
                        if (!first) sb.Append(',');
                        nested |= item is JObj;
                        if (nested) Break(sb, indent < 0 ? -1 : indent + 1); else if (!first && indent >= 0) sb.Append(' ');
                        Emit(sb, item, indent < 0 ? -1 : indent + 1);
                        first = false;
                    }
                    if (nested) Break(sb, indent);
                    sb.Append(']');
                    break;
                default: throw new ArgumentException($"no JSON for {v.GetType()}");
            }
        }

        // indent -1 is compact.
        static void Break(StringBuilder sb, int indent) {
            if (indent < 0) return;
            sb.Append('\n').Append(' ', 2 * indent);
        }

        static void Quote(StringBuilder sb, string s) {
            sb.Append('"');
            foreach (char c in s) {
                switch (c) {
                    case '"': sb.Append("\\\""); break;
                    case '\\': sb.Append("\\\\"); break;
                    case '\n': sb.Append("\\n"); break;
                    case '\t': sb.Append("\\t"); break;
                    case '\r': sb.Append("\\r"); break;
                    default:
                        if (c < 0x20) sb.Append("\\u").Append(((int)c).ToString("x4"));
                        else sb.Append(c);
                        break;
                }
            }
            sb.Append('"');
        }

        static void Skip(string t, ref int i) {
            while (i < t.Length && char.IsWhiteSpace(t[i])) i++;
        }

        static object Value(string t, ref int i) {
            Skip(t, ref i);
            if (i >= t.Length) throw new FormatException("unexpected end of JSON");
            char c = t[i];
            if (c == '{') {
                var obj = new JObj();
                i++;
                Skip(t, ref i);
                if (t[i] == '}') { i++; return obj; }
                while (true) {
                    Skip(t, ref i);
                    string key = (string)Value(t, ref i);
                    Skip(t, ref i);
                    if (t[i++] != ':') throw new FormatException($"expected : at {i}");
                    obj[key] = Value(t, ref i);
                    Skip(t, ref i);
                    if (t[i] == ',') { i++; continue; }
                    if (t[i++] == '}') return obj;
                    throw new FormatException($"expected , or }} at {i}");
                }
            }
            if (c == '[') {
                var list = new List<object>();
                i++;
                Skip(t, ref i);
                if (t[i] == ']') { i++; return list; }
                while (true) {
                    list.Add(Value(t, ref i));
                    Skip(t, ref i);
                    if (t[i] == ',') { i++; continue; }
                    if (t[i++] == ']') return list;
                    throw new FormatException($"expected , or ] at {i}");
                }
            }
            if (c == '"') {
                var sb = new StringBuilder();
                i++;
                while (t[i] != '"') {
                    if (t[i] == '\\') {
                        i++;
                        char e = t[i];
                        if (e == 'u') { sb.Append((char)Convert.ToInt32(t.Substring(i + 1, 4), 16)); i += 4; }
                        else sb.Append(e == 'n' ? '\n' : e == 't' ? '\t' : e == 'r' ? '\r' : e == 'b' ? '\b' : e == 'f' ? '\f' : e);
                    }
                    else sb.Append(t[i]);
                    i++;
                }
                i++;
                return sb.ToString();
            }
            if (t.Length - i >= 4 && string.CompareOrdinal(t, i, "true", 0, 4) == 0) { i += 4; return true; }
            if (t.Length - i >= 5 && string.CompareOrdinal(t, i, "false", 0, 5) == 0) { i += 5; return false; }
            if (t.Length - i >= 4 && string.CompareOrdinal(t, i, "null", 0, 4) == 0) { i += 4; return null; }
            int start = i;
            while (i < t.Length && "+-0123456789.eE".IndexOf(t[i]) >= 0) i++;
            if (start == i) throw new FormatException($"bad JSON at {i}");
            return double.Parse(t.Substring(start, i - start), NumberStyles.Float, CultureInfo.InvariantCulture);
        }
    }
}
