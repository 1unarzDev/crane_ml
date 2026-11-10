using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace Crane.Drones.Logging {
    /// <summary>Minimal deterministic JSON writer (invariant culture, round-trip doubles, NaN/Inf as null).</summary>
    public static class DroneJson {
        public static string Write(object value, bool indent = false) {
            var sb = new StringBuilder();
            Append(sb, value, indent ? 0 : -1);
            return sb.ToString();
        }

        public static string Num(double v) =>
            double.IsNaN(v) || double.IsInfinity(v) ? "null" : v.ToString("R", CultureInfo.InvariantCulture);

        private static void Nl(StringBuilder sb, int level) {
            if (level < 0) return;
            sb.Append('\n').Append(' ', level * 2);
        }

        private static void Append(StringBuilder sb, object v, int level) {
            int next = level < 0 ? -1 : level + 1;
            switch (v) {
                case null: sb.Append("null"); break;
                case string s: Str(sb, s); break;
                case bool b: sb.Append(b ? "true" : "false"); break;
                case double d: sb.Append(Num(d)); break;
                case float f: sb.Append(Num(f)); break;
                case int or long or short or byte or uint or ulong:
                    sb.Append(Convert.ToString(v, CultureInfo.InvariantCulture)); break;
                case IDictionary<string, object> dict: {
                    sb.Append('{');
                    bool first = true;
                    foreach (var kv in dict) {
                        if (!first) sb.Append(',');
                        first = false;
                        Nl(sb, next);
                        Str(sb, kv.Key);
                        sb.Append(level < 0 ? ":" : ": ");
                        Append(sb, kv.Value, next);
                    }
                    if (!first) Nl(sb, level);
                    sb.Append('}');
                    break;
                }
                case IEnumerable seq: {
                    sb.Append('[');
                    bool first = true;
                    foreach (object o in seq) {
                        if (!first) sb.Append(',');
                        first = false;
                        if (level >= 0 && !(o is double || o is float || o is int || o is long)) Nl(sb, next);
                        Append(sb, o, next);
                    }
                    sb.Append(']');
                    break;
                }
                default: Str(sb, Convert.ToString(v, CultureInfo.InvariantCulture)); break;
            }
        }

        private static void Str(StringBuilder sb, string s) {
            sb.Append('"');
            foreach (char c in s) {
                switch (c) {
                    case '"': sb.Append("\\\""); break;
                    case '\\': sb.Append("\\\\"); break;
                    case '\n': sb.Append("\\n"); break;
                    case '\r': sb.Append("\\r"); break;
                    case '\t': sb.Append("\\t"); break;
                    default:
                        if (c < 0x20) sb.Append("\\u").Append(((int)c).ToString("x4")); else sb.Append(c);
                        break;
                }
            }
            sb.Append('"');
        }
    }
}
