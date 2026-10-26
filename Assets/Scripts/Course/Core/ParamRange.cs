using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Reflection;

namespace Sim.Course {
    public enum ParamMode { Uniform, Fixed, Choice }

    /// <summary>How the generator picks each parameter: iid draws, or a box corner.</summary>
    public enum SampleMode { Iid, CornerLo, CornerHi, CornerMixed }

    /// <summary>One sampleable parameter: a uniform range, a fixed value, or a choice of
    /// options. Integer ranges draw whole numbers. Values are double, long or string.</summary>
    [Serializable]
    public sealed class ParamRange {
        public ParamMode mode;
        public double min;
        public double max;
        public bool integer;
        public string[] options = Array.Empty<string>();
        public string unit = "";

        public static ParamRange Uniform(double lo, double hi, string unit = "m") =>
            new ParamRange { mode = ParamMode.Uniform, min = lo, max = hi, unit = unit };
        public static ParamRange Int(int lo, int hi) =>
            new ParamRange { mode = ParamMode.Uniform, min = lo, max = hi, integer = true, unit = "count" };
        public static ParamRange Fixed(double value, string unit = "m") =>
            new ParamRange { mode = ParamMode.Fixed, min = value, max = value, unit = unit };
        public static ParamRange Choice(params string[] options) =>
            new ParamRange { mode = ParamMode.Choice, options = options, unit = "choice" };

        /// <summary>Always the first option unless pinned to another (e.g. a handbook default
        /// with an opt-in alternative).</summary>
        public static ParamRange Default(params string[] options) =>
            new ParamRange { mode = ParamMode.Fixed, options = options, unit = "choice" };

        bool IsChoice => options != null && options.Length > 0;

        public object Sample(CourseRng rng, SampleMode sampling) {
            if (mode == ParamMode.Fixed) return IsChoice ? options[0] : Box(min);
            switch (sampling) {
                case SampleMode.CornerLo: return Corner(false);
                case SampleMode.CornerHi: return Corner(true);
                case SampleMode.CornerMixed: return Corner(rng.Chance(0.5));
            }
            if (mode == ParamMode.Choice) return rng.Pick(options);
            return integer ? (object)(long)rng.Int((int)min, (int)max) : rng.Range(min, max);
        }

        public object Corner(bool hi) {
            if (mode == ParamMode.Fixed) return IsChoice ? options[0] : Box(min);
            if (mode == ParamMode.Choice) return hi ? options[options.Length - 1] : options[0];
            return Box(hi ? max : min);
        }

        /// <summary>A pinned value from <c>--crane-course-set key=value</c>.</summary>
        public object Parse(string text) {
            if (IsChoice) {
                if (Array.IndexOf(options, text) < 0)
                    throw new ArgumentException($"'{text}' is not one of {string.Join("|", options)}");
                return text;
            }
            double v = double.Parse(text, NumberStyles.Float, CultureInfo.InvariantCulture);
            return Box(v);
        }

        object Box(double v) => integer ? (object)(long)Math.Round(v) : v;

        public JObj Describe() {
            var o = new JObj().Set("mode", mode.ToString().ToLowerInvariant());
            if (IsChoice) o.Set("options", options.Cast<object>().ToList());
            else o.Set("min", min).Set("max", max).Set("integer", integer);
            return o.Set("unit", unit);
        }
    }

    /// <summary>Where a parameter's range comes from: the handbook (H, with page) or our design
    /// choice (D). Read by reflection into truth and by coverage tools.</summary>
    [AttributeUsage(AttributeTargets.Field)]
    public sealed class CourseParamAttribute : Attribute {
        public readonly string Cite;
        public readonly string Tag;
        public CourseParamAttribute(string tag, string cite = "") { Tag = tag; Cite = cite; }
    }

    public readonly struct ParamEntry {
        public readonly string Path;
        public readonly ParamRange Range;
        public readonly CourseParamAttribute Source;
        public ParamEntry(string path, ParamRange range, CourseParamAttribute source) {
            Path = path; Range = range; Source = source;
        }
    }

    /// <summary>The resolved parameters of one task, by path.</summary>
    public sealed class ParamValues {
        readonly SortedDictionary<string, object> values = new SortedDictionary<string, object>(StringComparer.Ordinal);
        public void Set(string path, object value) => values[path] = value;
        public double Num(string path) => values[path] is long n ? n : (double)values[path];
        public int Int(string path) => (int)(long)values[path];
        public string Str(string path) => (string)values[path];
        public IEnumerable<KeyValuePair<string, object>> All => values;
    }

    public static class ParamReflection {
        /// <summary>Every ParamRange field of a params object, sorted by ordinal path.</summary>
        public static IReadOnlyList<ParamEntry> Entries(object parameters) =>
            parameters.GetType().GetFields(BindingFlags.Public | BindingFlags.Instance)
                .Where(f => f.FieldType == typeof(ParamRange))
                .Select(f => new ParamEntry(f.Name, (ParamRange)f.GetValue(parameters),
                    f.GetCustomAttribute<CourseParamAttribute>() ?? new CourseParamAttribute("D")))
                .OrderBy(e => e.Path, StringComparer.Ordinal)
                .ToList();
    }
}
