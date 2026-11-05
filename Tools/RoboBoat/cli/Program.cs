using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.CompilerServices;
using UnityEngine;

namespace Sim.Course.Cli {
    /// <summary>coursegen: the course truth for a seed and options, from the generator files in
    /// Assets/ (compiled by reference) and the family assets Unity loads, byte-identical to
    /// CourseCli.Generate in the editor. Usage: README.md.</summary>
    static class Program {
        const string Usage =
            "usage: coursegen --seed N [--out FILE]   one truth (stdout without --out)\n" +
            "       coursegen --seeds A:B --out DIR   seeds A..B-1 as DIR/<family>-s<N>.json + failures.txt, one JSON summary line\n" +
            "         [--family roboboat_2026] [--mode iid|corner=lo|corner=hi|corner=mixed] [--set task.param=value]...\n" +
            "         [--root CRANE]                  the checkout to read (default: the one it was built from)\n" +
            "       coursegen --version | --sources | --help\n" +
            "exit 0 ok, 2 bad arguments, 3 stale build or a seed failed to generate";
        static readonly string[] Valued = { "--seed", "--seeds", "--out", "--family", "--mode", "--set", "--root" };
        static readonly string[] Switches = { "--version", "--sources", "--help", "-h" };

        static int Main(string[] args) {
            try { return Run(args); }
            catch (Exception e) when (e is ArgumentException || e is FormatException || e is IOException) {
                Console.Error.WriteLine($"CRANE_COURSE_FAILED {e.Message}\n{Usage}");
                return 2;
            }
        }

        static int Run(string[] args) {
            for (int i = 0; i < args.Length; i++) {
                if (Valued.Contains(args[i]) && i + 1 < args.Length) i++;
                else if (!Switches.Contains(args[i])) throw new ArgumentException($"unknown or incomplete argument {args[i]}");
            }
            if (args.Contains("--help") || args.Contains("-h")) {
                Console.WriteLine(Usage);
                return 0;
            }
            var meta = typeof(Program).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().ToList();
            // No trailing separator: on Windows "C:\crane\" would escape the quote around git's -C argument
            // (CourseIdentity.Git), and every truth would record crane_commit unknown.
            string root = Path.TrimEndingDirectorySeparator(Arg(args, "--root") ?? meta.First(m => m.Key == "coursegen.root").Value);
            var compiled = meta.Where(m => m.Key.StartsWith("coursegen.src:", StringComparison.Ordinal))
                .ToDictionary(m => m.Key.Substring("coursegen.src:".Length), m => m.Value.ToLowerInvariant());
            if (args.Contains("--sources")) {
                foreach (var kv in compiled.OrderBy(kv => kv.Key, StringComparer.Ordinal)) Console.WriteLine($"{kv.Key} {kv.Value}");
                return 0;
            }
            var stale = compiled.Where(kv => !File.Exists(Path.Combine(root, kv.Key)) ||
                                             CourseIdentity.FileSha256(Path.Combine(root, kv.Key)) != kv.Value).Select(kv => kv.Key).ToList();
            if (stale.Count > 0) {
                Console.Error.WriteLine($"CRANE_COURSE_FAILED stale build (sources changed: {string.Join(", ", stale)}); rebuild: dotnet build Tools/RoboBoat/cli -c Release");
                return 3;
            }
            CourseIdentity.Current = CourseIdentity.FromCheckout(root);
            if (args.Contains("--version")) {
                Console.WriteLine(Json.Write(CourseIdentity.Current.ToJson(), true));
                return 0;
            }

            var family = FamilyAssets.Load(root, Arg(args, "--family") ?? "roboboat_2026");
            var request = new CourseRequest { Mode = CourseRequest.ParseMode(Arg(args, "--mode")) };
            for (int i = 0; i + 1 < args.Length; i++) {
                if (args[i] != "--set") continue;
                int eq = args[i + 1].IndexOf('=');
                if (eq <= 0) throw new ArgumentException($"--set expects task.param=value, got {args[i + 1]}");
                request.Overrides[args[i + 1].Substring(0, eq)] = args[i + 1].Substring(eq + 1);
            }
            string output = Arg(args, "--out"), range = Arg(args, "--seeds"), seed = Arg(args, "--seed");
            if (range == null) {
                request.Seed = long.Parse(seed ?? throw new ArgumentException("--seed N or --seeds A:B"), CultureInfo.InvariantCulture);
                var truth = Generate(family, request);
                if (truth == null) return 3;
                string text = Json.Write(truth, true) + "\n";
                if (output == null) Console.Out.Write(text); else File.WriteAllText(output, text);
                Console.Error.WriteLine($"CRANE_COURSE_READY name={truth.Str("name")} sha256={CourseTruth.Sha(truth)} " +
                                        $"pair_key={Json.Path(truth, "generator.identity.pair_key")} truth={output ?? "stdout"}");
                return 0;
            }
            if (output == null) throw new ArgumentException("--seeds needs --out DIR");
            string[] ab = range.Split(':');
            long from = long.Parse(ab[0], CultureInfo.InvariantCulture), to = long.Parse(ab[1], CultureInfo.InvariantCulture);
            Directory.CreateDirectory(output);
            var failures = new List<string>();
            var clock = System.Diagnostics.Stopwatch.StartNew();
            int retried = 0;
            for (long s = from; s < to; s++) {
                request.Seed = s;
                var truth = Generate(family, request, failures);
                if (truth != null && ((JObj)Json.Path(truth, "generator.attempts")).Any(kv => (double)kv.Value > 1)) retried++;
                if (truth != null) File.WriteAllText(Path.Combine(output, $"{family.Name}-s{s}.json"), Json.Write(truth, true) + "\n");
            }
            File.WriteAllLines(Path.Combine(output, "failures.txt"), failures);
            Console.Error.WriteLine($"CRANE_COURSE_BATCH family={family.Name} seeds={from}:{to} failed={failures.Count} out={output}");
            Console.WriteLine(Json.Write(new JObj().Set("family", family.Name).Set("seeds", (double)(to - from))
                .Set("failed", (double)failures.Count).Set("retried", (double)retried).Set("ms", (double)clock.ElapsedMilliseconds)
                .Set("identity", Json.Path(CourseIdentity.Current.ToJson(), "")).Set("out", output)));
            return failures.Count == 0 ? 0 : 3;
        }

        static JObj Generate(CourseFamilyDefinition family, CourseRequest request, List<string> failures = null) {
            try { return CourseGenerator.Generate(family, request); }
            catch (CourseGenerationException e) {
                failures?.Add($"seed {request.Seed}: {e.Message}");
                Console.Error.WriteLine($"seed {request.Seed}: {e.Message}");
                return null;
            }
        }

        static string Arg(string[] args, string key) {
            int i = Array.IndexOf(args, key);
            return i >= 0 && i + 1 < args.Length ? args[i + 1] : null;
        }
    }

    /// <summary>Reads a family the way Unity deserializes it: the family asset, its task assets
    /// (class from the script guid), the catalog and the constraint spec, all by guid. Refuses a
    /// serialized field it does not know, so an asset change cannot be silently ignored.</summary>
    static class FamilyAssets {
        const string Dir = "Assets/Config/Course/Resources";

        public static CourseFamilyDefinition Load(string root, string name) {
            var guids = new Dictionary<string, string>();
            foreach (string meta in Directory.GetFiles(Path.Combine(root, "Assets"), "*.meta", SearchOption.AllDirectories)) {
                string line = File.ReadLines(meta).FirstOrDefault(l => l.StartsWith("guid: ", StringComparison.Ordinal));
                if (line != null) guids[line.Substring(6).Trim()] = meta.Substring(0, meta.Length - 5);
            }
            string familyPath = Directory.GetFiles(Path.Combine(root, Dir), "*.asset", SearchOption.AllDirectories)
                .FirstOrDefault(f => Fields(f).TryGetValue("familyName", out var n) && n == name)
                ?? throw new ArgumentException($"no course family '{name}' under {Dir}");
            var f = Fields(familyPath);
            Known(familyPath, f, "familyName", "tasks", "catalog", "boundary", "handbookId", "handbookSha256", "constraintSpec", "vessel", "hideSceneGroups");

            var tasks = new List<CourseTaskDefinition>();
            foreach (string reference in List(familyPath, "tasks")) {
                string assetPath = guids[Guid(reference)];
                var t = Fields(assetPath);
                Known(assetPath, t, "taskId");
                string script = Path.GetFileNameWithoutExtension(guids[Guid(t["m_Script"])]);
                var type = typeof(CourseTaskDefinition).Assembly.GetTypes().Single(x => x.Name == script && typeof(CourseTaskDefinition).IsAssignableFrom(x));
                var task = (CourseTaskDefinition)RuntimeHelpers.GetUninitializedObject(type);
                task.SetTaskId(t.TryGetValue("taskId", out var id) ? id : "");
                tasks.Add(task);
            }
            var family = (CourseFamilyDefinition)RuntimeHelpers.GetUninitializedObject(typeof(CourseFamilyDefinition));
            var b = Inline(f["boundary"]);
            family.Configure(f["familyName"], tasks, Catalog(guids[Guid(f["catalog"])]),
                File.ReadAllText(guids[Guid(f["constraintSpec"])]), Vessel(familyPath),
                new CourseBoundary { Back = Float(b["x"]), Length = Float(b["y"]), HalfWidth = Float(b["z"]) },
                f["handbookId"], f["handbookSha256"]);
            return family;
        }

        /// <summary>The family's serialized vessel block (four-space keys under "  vessel:").</summary>
        static VesselProfile Vessel(string path) {
            var v = new Dictionary<string, string>();
            bool on = false;
            foreach (string raw in File.ReadAllLines(path)) {
                if (raw == "  vessel:") { on = true; continue; }
                if (!on) continue;
                if (!raw.StartsWith("    ", StringComparison.Ordinal)) break;
                int colon = raw.IndexOf(':');
                v[raw.Substring(4, colon - 4)] = Scalar(raw.Substring(colon + 1));
            }
            if (v.Count == 0) throw new ArgumentException($"{path}: no vessel");
            return new VesselProfile {
                Beam = Double(v["Beam"]), Length = Double(v["Length"]), TurnRadius = Double(v["TurnRadius"]), Source = v["Source"]
            };
        }

        static CourseObjectCatalog Catalog(string path) {
            var entries = new List<CourseObjectCatalog.Entry>();
            CourseObjectCatalog.Entry e = null;
            bool inEntries = false;
            foreach (string raw in File.ReadAllLines(path)) {
                if (raw.StartsWith("  entries:", StringComparison.Ordinal)) { inEntries = true; continue; }
                if (!inEntries) continue;
                string line = raw.Trim();
                if (line.StartsWith("- ", StringComparison.Ordinal)) { e = new CourseObjectCatalog.Entry(); entries.Add(e); line = line.Substring(2); }
                else if (!raw.StartsWith("    ", StringComparison.Ordinal)) break;
                int colon = line.IndexOf(':');
                string key = line.Substring(0, colon), value = Scalar(line.Substring(colon + 1));
                var field = typeof(CourseObjectCatalog.Entry).GetField(key) ?? throw new ArgumentException($"{path}: unknown catalog field {key}");
                if (field.FieldType == typeof(string)) field.SetValue(e, value);
                else if (field.FieldType == typeof(float)) field.SetValue(e, Float(value));
                else if (field.FieldType.IsEnum) field.SetValue(e, Enum.ToObject(field.FieldType, int.Parse(value, CultureInfo.InvariantCulture)));
                else if (value != "{fileID: 0}") throw new ArgumentException($"{path}: catalog field {key} = {value} is not supported outside Unity");
            }
            var c = (CourseObjectCatalog)RuntimeHelpers.GetUninitializedObject(typeof(CourseObjectCatalog));
            c.SetEntries(entries);
            return c;
        }

        /// <summary>Top-level fields of a MonoBehaviour asset (two-space keys).</summary>
        static Dictionary<string, string> Fields(string path) {
            var d = new Dictionary<string, string>();
            foreach (string raw in File.ReadAllLines(path)) {
                if (raw.Length < 3 || raw[0] != ' ' || raw[1] != ' ' || raw[2] == ' ' || raw[2] == '-') continue;
                int colon = raw.IndexOf(':');
                if (colon > 0) d[raw.Substring(2, colon - 2)] = Scalar(raw.Substring(colon + 1));
            }
            return d;
        }

        static void Known(string path, Dictionary<string, string> fields, params string[] known) {
            foreach (string key in fields.Keys)
                if (!key.StartsWith("m_", StringComparison.Ordinal) && !known.Contains(key))
                    throw new ArgumentException($"{path}: serialized field '{key}' is not read by coursegen; teach Tools/RoboBoat/cli/Program.cs");
        }

        static IEnumerable<string> List(string path, string key) {
            bool on = false;
            foreach (string raw in File.ReadAllLines(path)) {
                if (raw == $"  {key}:") { on = true; continue; }
                if (!on) continue;
                if (!raw.StartsWith("  - ", StringComparison.Ordinal)) yield break;
                yield return raw.Substring(4);
            }
        }

        static Dictionary<string, string> Inline(string map) =>
            map.Trim('{', '}', ' ').Split(',').Select(p => p.Split(':'))
                .ToDictionary(p => p[0].Trim(), p => p.Length > 1 ? p[1].Trim() : "");

        static string Guid(string reference) => Inline(reference)["guid"];

        static string Scalar(string v) {
            v = v.Trim();
            if (v.Length >= 2 && v[0] == '\'' && v[v.Length - 1] == '\'') return v.Substring(1, v.Length - 2).Replace("''", "'");
            if (v.Length >= 2 && v[0] == '"') throw new ArgumentException($"double-quoted YAML scalar {v} is not supported");
            return v;
        }

        static double Double(string v) => double.Parse(v, NumberStyles.Float, CultureInfo.InvariantCulture);

        static float Float(string v) => float.Parse(v, NumberStyles.Float, CultureInfo.InvariantCulture);
    }
}
