using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace Crane.Fleet.Perception
{
    /// <summary>
    /// Minimal dependency-free JSON reader (objects -> Dictionary, arrays -> List, numbers -> double,
    /// strings, bool, null). Exists so the library needs no System.Text.Json / Newtonsoft inside Unity.
    /// </summary>
    public static class MiniJson
    {
        public static object Parse(string s)
        {
            int i = 0; var v = Value(s, ref i); Ws(s, ref i);
            if (i != s.Length) throw new FormatException($"trailing JSON content at {i}");
            return v;
        }
        private static void Ws(string s, ref int i) { while (i < s.Length && char.IsWhiteSpace(s[i])) i++; }
        private static object Value(string s, ref int i)
        {
            Ws(s, ref i);
            if (i >= s.Length) throw new FormatException("unexpected end of JSON");
            char c = s[i];
            if (c == '{')
            {
                var d = new Dictionary<string, object>(); i++; Ws(s, ref i);
                if (s[i] == '}') { i++; return d; }
                while (true)
                {
                    Ws(s, ref i); string k = Str(s, ref i); Ws(s, ref i);
                    if (s[i++] != ':') throw new FormatException($"expected ':' at {i - 1}");
                    d[k] = Value(s, ref i); Ws(s, ref i);
                    if (s[i] == ',') { i++; continue; }
                    if (s[i] == '}') { i++; return d; }
                    throw new FormatException($"expected ',' or '}}' at {i}");
                }
            }
            if (c == '[')
            {
                var l = new List<object>(); i++; Ws(s, ref i);
                if (s[i] == ']') { i++; return l; }
                while (true)
                {
                    l.Add(Value(s, ref i)); Ws(s, ref i);
                    if (s[i] == ',') { i++; continue; }
                    if (s[i] == ']') { i++; return l; }
                    throw new FormatException($"expected ',' or ']' at {i}");
                }
            }
            if (c == '"') return Str(s, ref i);
            if (string.CompareOrdinal(s, i, "true", 0, 4) == 0) { i += 4; return true; }
            if (string.CompareOrdinal(s, i, "false", 0, 5) == 0) { i += 5; return false; }
            if (string.CompareOrdinal(s, i, "null", 0, 4) == 0) { i += 4; return null; }
            int st = i;
            while (i < s.Length && "+-0123456789.eE".IndexOf(s[i]) >= 0) i++;
            if (st == i) throw new FormatException($"bad JSON token at {i}");
            return double.Parse(s.Substring(st, i - st), CultureInfo.InvariantCulture);
        }
        private static string Str(string s, ref int i)
        {
            if (s[i] != '"') throw new FormatException($"expected string at {i}");
            i++; var sb = new StringBuilder();
            while (s[i] != '"')
            {
                char c = s[i++];
                if (c != '\\') { sb.Append(c); continue; }
                char e = s[i++];
                switch (e)
                {
                    case 'n': sb.Append('\n'); break; case 't': sb.Append('\t'); break; case 'r': sb.Append('\r'); break;
                    case 'b': sb.Append('\b'); break; case 'f': sb.Append('\f'); break;
                    case 'u': sb.Append((char)Convert.ToInt32(s.Substring(i, 4), 16)); i += 4; break;
                    default: sb.Append(e); break;
                }
            }
            i++; return sb.ToString();
        }
    }
}
