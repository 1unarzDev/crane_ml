using System;
using System.Collections.Generic;
using System.Globalization;

namespace Sim.Course {
    /// <summary>State programs (§5), recorded in truth and run by the state registry:
    /// fixed (the default: set before the run), toggle (opt-in), scripted (regressions), and
    /// the horn. <see cref="ValueAt"/> is the one meaning; course_truth.py state_at mirrors it.</summary>
    public static class Programs {
        public static JObj Fixed(string value) => new JObj().Set("type", "fixed").Set("value", value);

        public static JObj Toggle(string first, string second, double periodS, double phaseS) =>
            new JObj().Set("type", "toggle").Set("values", new List<object> { first, second })
                .Set("period_s", periodS).Set("phase_s", phaseS);

        public static JObj Scripted(string initial, IEnumerable<(double t, string value)> steps) {
            var list = new List<object>();
            foreach (var (t, v) in steps) list.Add(new JObj().Set("t", t).Set("value", v));
            return new JObj().Set("type", "scripted").Set("initial", initial).Set("steps", list);
        }

        /// <summary>H pp.56–57: one 1 s blast, or two 0.5 s blasts 0.5 s apart; signals 2 s
        /// apart, repeated; starting at sim time trigger_s.</summary>
        public static JObj Horn(double freqHz, string pattern, int repeats, double triggerS) =>
            new JObj().Set("type", "horn").Set("freq_hz", freqHz).Set("pattern", pattern)
                .Set("repeats", (double)repeats).Set("gap_s", 2.0).Set("trigger_s", triggerS);

        /// <summary>The value before the run starts: a toggle's value at t = 0 given its phase
        /// (course_truth.py program_value agrees), otherwise the value before any change.</summary>
        public static string Initial(JObj p) => ValueAt(p, p.Str("type") == "toggle" ? 0 : double.NegativeInfinity);

        /// <summary>The program's value at sim time t (seconds since the course was ready).</summary>
        public static string ValueAt(JObj p, double t) {
            switch (p.Str("type")) {
                case "fixed": return p.Str("value");
                case "toggle": {
                    var values = p.Arr("values");
                    if (double.IsNegativeInfinity(t)) t = 0;
                    long k = (long)Math.Floor((t + p.Num("phase_s")) / p.Num("period_s"));
                    return (string)values[(int)(((k % 2) + 2) % 2)];
                }
                case "scripted": {
                    string v = p.Str("initial");
                    foreach (JObj s in p.Arr("steps")) if (s.Num("t") <= t) v = s.Str("value");
                    return v;
                }
                case "horn": return HornOn(p, t) ? "on:" + Json.Number(p.Num("freq_hz")) : "off";
                default: throw new ArgumentException($"unknown program {p.Str("type")}");
            }
        }

        /// <summary>Times at which a program's value changes, up to <paramref name="until"/>.</summary>
        public static IEnumerable<double> Changes(JObj p, double until) {
            switch (p.Str("type")) {
                case "toggle": {
                    double period = p.Num("period_s"), phase = p.Num("phase_s");
                    for (double t = Math.Ceiling(phase / period) * period - phase; t <= until; t += period)
                        if (t > 0) yield return t;
                    break;
                }
                case "scripted":
                    foreach (JObj s in p.Arr("steps")) if (s.Num("t") <= until) yield return s.Num("t");
                    break;
                case "horn":
                    foreach (var (on, off) in Blasts(p)) { if (on > until) yield break; yield return on; yield return off; }
                    break;
            }
        }

        public static IEnumerable<(double on, double off)> Blasts(JObj p) {
            double length = p.Str("pattern") == "two" ? 1.5 : 1.0;
            for (int k = 0; k < (int)p.Num("repeats"); k++) {
                double start = p.Num("trigger_s") + k * (length + p.Num("gap_s"));
                if (p.Str("pattern") == "two") { yield return (start, start + 0.5); yield return (start + 1.0, start + 1.5); }
                else yield return (start, start + 1.0);
            }
        }

        static bool HornOn(JObj p, double t) {
            foreach (var (on, off) in Blasts(p)) if (t >= on && t < off) return true;
            return false;
        }

        public static string Hz(double v) => v.ToString("0.#", CultureInfo.InvariantCulture);
    }
}
