// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace Crane.Drones.Autonomy {
    /// <summary>
    /// Minimal JSON reader (no UnityEngine, no System.Text.Json). Objects -> Dictionary&lt;string,object&gt;, arrays of
    /// numbers -> double[] (compact for the 1.6 M manifest capsules), other arrays -> List&lt;object&gt;, numbers -> double.
    /// </summary>
    public static class MiniJson {
        public static object Parse(string s) { int i = 0; var v = Value(s, ref i); return v; }

        static void Ws(string s, ref int i) { while (i < s.Length && (s[i] == ' ' || s[i] == '\n' || s[i] == '\r' || s[i] == '\t')) i++; }

        static object Value(string s, ref int i) {
            Ws(s, ref i);
            char c = s[i];
            if (c == '{') return Obj(s, ref i);
            if (c == '[') return Arr(s, ref i);
            if (c == '"') return Str(s, ref i);
            if (c == 't') { i += 4; return true; }
            if (c == 'f') { i += 5; return false; }
            if (c == 'n') { i += 4; return null; }
            return Num(s, ref i);
        }

        static Dictionary<string, object> Obj(string s, ref int i) {
            var d = new Dictionary<string, object>(); i++;
            Ws(s, ref i);
            if (s[i] == '}') { i++; return d; }
            while (true) {
                Ws(s, ref i); string k = Str(s, ref i); Ws(s, ref i); i++; // ':'
                d[k] = Value(s, ref i); Ws(s, ref i);
                if (s[i] == ',') { i++; continue; }
                i++; return d;
            }
        }

        static object Arr(string s, ref int i) {
            i++; Ws(s, ref i);
            if (s[i] == ']') { i++; return new List<object>(); }
            // numeric fast path
            int save = i; var nums = new List<double>(8); bool numeric = true;
            while (true) {
                Ws(s, ref i); char c = s[i];
                if (!(c == '-' || (c >= '0' && c <= '9'))) { numeric = false; break; }
                nums.Add(Num(s, ref i)); Ws(s, ref i);
                if (s[i] == ',') { i++; continue; }
                if (s[i] == ']') { i++; break; }
                numeric = false; break;
            }
            if (numeric) return nums.ToArray();
            i = save; var l = new List<object>();
            while (true) {
                l.Add(Value(s, ref i)); Ws(s, ref i);
                if (s[i] == ',') { i++; continue; }
                i++; return l;
            }
        }

        static string Str(string s, ref int i) {
            i++; var sb = new StringBuilder();
            while (s[i] != '"') {
                if (s[i] == '\\') {
                    i++; char e = s[i];
                    if (e == 'n') sb.Append('\n'); else if (e == 't') sb.Append('\t'); else if (e == 'r') sb.Append('\r');
                    else if (e == 'u') { sb.Append((char)int.Parse(s.Substring(i + 1, 4), NumberStyles.HexNumber)); i += 4; }
                    else sb.Append(e);
                    i++;
                } else sb.Append(s[i++]);
            }
            i++; return sb.ToString();
        }

        static double Num(string s, ref int i) {
            int st = i;
            while (i < s.Length && "+-0123456789.eE".IndexOf(s[i]) >= 0) i++;
            return double.Parse(s.Substring(st, i - st), NumberStyles.Float, CultureInfo.InvariantCulture);
        }

        public static double D(object o, double def = double.NaN) => o is double d ? d : def;
        public static Dictionary<string, object> O(object o) => o as Dictionary<string, object>;
        public static List<object> L(object o) => o as List<object> ?? new List<object>();
        public static object Get(Dictionary<string, object> d, string k) => d != null && d.TryGetValue(k, out var v) ? v : null;
    }
}
