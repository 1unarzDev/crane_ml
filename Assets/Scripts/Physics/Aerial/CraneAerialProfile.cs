using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Security.Cryptography;
using System.Text;

namespace Sim.Physics.Aerial {
    /// <summary>
    /// Loads Tools/Aerial/profiles/*.json (schema crane-aerial-vehicle-profile-v1) into a
    /// VehicleSpecV2 and records the SHA-256 of the exact file bytes for replay headers.
    /// </summary>
    public sealed class CraneAerialProfile {
        public string Id, Path, Sha256;
        public VehicleSpecV2 Vehicle;

        public static CraneAerialProfile LoadFile(string path, bool withBattery = true,
            bool groundEffect = false) {
            byte[] bytes = File.ReadAllBytes(path);
            var profile = Parse(Encoding.UTF8.GetString(bytes), withBattery, groundEffect);
            profile.Path = path;
            profile.Sha256 = Hash(bytes);
            return profile;
        }

        /// <summary>Resolves a profile by name or path. Searches the supplied directories.</summary>
        public static string Resolve(string nameOrPath, params string[] searchDirectories) {
            if (File.Exists(nameOrPath)) return nameOrPath;
            foreach (string dir in searchDirectories) {
                if (string.IsNullOrEmpty(dir)) continue;
                string candidate = System.IO.Path.Combine(dir, nameOrPath + ".json");
                if (File.Exists(candidate)) return candidate;
            }
            throw new FileNotFoundException($"Aerial profile '{nameOrPath}' not found.");
        }

        public static string Hash(byte[] bytes) {
            using var sha = SHA256.Create();
            var sb = new StringBuilder();
            foreach (byte b in sha.ComputeHash(bytes)) sb.Append(b.ToString("x2"));
            return sb.ToString();
        }

        public static CraneAerialProfile Parse(string json, bool withBattery, bool groundEffect) {
            var root = (Dictionary<string, object>)MiniJson.Parse(json);
            if ((string)root["schema"] != "crane-aerial-vehicle-profile-v1")
                throw new InvalidDataException("Unsupported aerial profile schema.");
            var prm = (Dictionary<string, object>)root["parameters"];
            double Val(string key) => Convert.ToDouble(((Dictionary<string, object>)prm[key])["value"],
                CultureInfo.InvariantCulture);
            double ValOr(string key, double d) => prm.ContainsKey(key) ? Val(key) : d;
            double[] Vec(string key) {
                var list = (List<object>)((Dictionary<string, object>)prm[key])["value"];
                var r = new double[list.Count];
                for (int i = 0; i < r.Length; i++) r[i] = Convert.ToDouble(list[i], CultureInfo.InvariantCulture);
                return r;
            }
            var inertiaRows = (List<object>)((Dictionary<string, object>)prm["inertia"])["value"];
            var inertia = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    inertia[i, j] = Convert.ToDouble(((List<object>)inertiaRows[i])[j], CultureInfo.InvariantCulture);
            var rotorList = (List<object>)root["rotors"];
            var rotors = new RotorSpecV2[rotorList.Count];
            for (int i = 0; i < rotors.Length; i++) {
                var rd = (Dictionary<string, object>)rotorList[i];
                var pos = (List<object>)rd["position"];
                rotors[i] = new RotorSpecV2 {
                    Name = (string)rd["name"],
                    Position = new[] { Convert.ToDouble(pos[0], CultureInfo.InvariantCulture),
                        Convert.ToDouble(pos[1], CultureInfo.InvariantCulture),
                        Convert.ToDouble(pos[2], CultureInfo.InvariantCulture) },
                    Spin = Convert.ToInt32(rd["spin"], CultureInfo.InvariantCulture),
                    Kf = Val("kf"), Km = Val("km"), OmegaMax = Val("omega_max"),
                    TauUp = Val("tau_up"), TauDown = Val("tau_down"),
                    RotorDrag = Val("rotor_drag"), RollingMoment = Val("rolling_moment"),
                    Radius = Val("rotor_radius"), RotorInertia = ValOr("rotor_inertia", 0.0)
                };
            }
            BatterySpecV2 battery = null;
            if (withBattery && prm.ContainsKey("battery_resistance")) {
                battery = new BatterySpecV2 {
                    VFull = Val("battery_v_full"), VEmpty = Val("battery_v_empty"),
                    VNominal = Val("battery_v_nominal"), Resistance = Val("battery_resistance"),
                    CapacityAh = Val("battery_capacity_ah"), Efficiency = Val("battery_efficiency")
                };
            }
            return new CraneAerialProfile {
                Id = (string)root["id"],
                Vehicle = new VehicleSpecV2 {
                    Name = (string)root["id"], Mass = Val("mass"), Inertia = inertia, Rotors = rotors,
                    DragLinear = Vec("drag_linear"), DragQuadratic = Val("drag_quadratic"),
                    GroundEffect = groundEffect, Battery = battery
                }
            };
        }
    }

    /// <summary>Minimal JSON reader: objects, arrays, numbers, strings, true/false/null.</summary>
    internal static class MiniJson {
        public static object Parse(string text) {
            int i = 0;
            object value = ParseValue(text, ref i);
            Skip(text, ref i);
            if (i != text.Length) throw new InvalidDataException("Trailing JSON content.");
            return value;
        }

        private static void Skip(string s, ref int i) {
            while (i < s.Length && char.IsWhiteSpace(s[i])) i++;
        }

        private static object ParseValue(string s, ref int i) {
            Skip(s, ref i);
            if (i >= s.Length) throw new InvalidDataException("Unexpected end of JSON.");
            char c = s[i];
            if (c == '{') {
                var d = new Dictionary<string, object>();
                i++; Skip(s, ref i);
                if (s[i] == '}') { i++; return d; }
                while (true) {
                    Skip(s, ref i);
                    string key = ParseString(s, ref i);
                    Skip(s, ref i);
                    if (s[i++] != ':') throw new InvalidDataException("Expected ':'.");
                    d[key] = ParseValue(s, ref i);
                    Skip(s, ref i);
                    if (s[i] == ',') { i++; continue; }
                    if (s[i] == '}') { i++; return d; }
                    throw new InvalidDataException("Expected ',' or '}'.");
                }
            }
            if (c == '[') {
                var l = new List<object>();
                i++; Skip(s, ref i);
                if (s[i] == ']') { i++; return l; }
                while (true) {
                    l.Add(ParseValue(s, ref i));
                    Skip(s, ref i);
                    if (s[i] == ',') { i++; continue; }
                    if (s[i] == ']') { i++; return l; }
                    throw new InvalidDataException("Expected ',' or ']'.");
                }
            }
            if (c == '"') return ParseString(s, ref i);
            if (string.CompareOrdinal(s, i, "true", 0, 4) == 0) { i += 4; return true; }
            if (string.CompareOrdinal(s, i, "false", 0, 5) == 0) { i += 5; return false; }
            if (string.CompareOrdinal(s, i, "null", 0, 4) == 0) { i += 4; return null; }
            int start = i;
            while (i < s.Length && "+-0123456789.eE".IndexOf(s[i]) >= 0) i++;
            if (start == i) throw new InvalidDataException($"Unexpected JSON character '{c}'.");
            return double.Parse(s.Substring(start, i - start), CultureInfo.InvariantCulture);
        }

        private static string ParseString(string s, ref int i) {
            if (s[i] != '"') throw new InvalidDataException("Expected string.");
            i++;
            var sb = new StringBuilder();
            while (s[i] != '"') {
                if (s[i] == '\\') {
                    i++;
                    switch (s[i]) {
                        case 'n': sb.Append('\n'); break;
                        case 't': sb.Append('\t'); break;
                        case 'r': sb.Append('\r'); break;
                        case 'b': sb.Append('\b'); break;
                        case 'f': sb.Append('\f'); break;
                        case 'u':
                            sb.Append((char)Convert.ToInt32(s.Substring(i + 1, 4), 16));
                            i += 4; break;
                        default: sb.Append(s[i]); break;
                    }
                } else sb.Append(s[i]);
                i++;
            }
            i++;
            return sb.ToString();
        }
    }
}
