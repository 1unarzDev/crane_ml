using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using UnityEditor;

namespace Sim.Course.Editor {
    /// <summary>Headless course tools (§6), run by Tools/RoboBoat/generate.sh:
    /// <c>-executeMethod Sim.Course.Editor.CourseCli.Generate --crane-course-family F
    /// (--crane-course-seed N | --seeds A:B) [--crane-course-mode M] [--crane-course-set k=v]
    /// --out FILE|DIR</c>, and <c>CourseCli.Validate --in DIR --out FILE</c> (the C# verdicts
    /// on every truth in DIR, for the C#/Python agreement test). No water or scene needed.</summary>
    public static class CourseCli {
        public static void Generate() => Exit(() => {
            string[] args = Environment.GetCommandLineArgs();
            var family = CourseBootstrap.FindFamily(Arg(args, "--crane-course-family") ?? "roboboat_2026");
            string output = Arg(args, "--out") ?? throw new ArgumentException("--out FILE|DIR");
            string range = Arg(args, "--seeds");
            if (range == null) {
                var truth = CourseGenerator.Generate(family, CourseBootstrap.Request(args, null));
                File.WriteAllText(output, Json.Write(truth, true) + "\n");
                Console.WriteLine($"CRANE_COURSE_READY name={truth.Str("name")} sha256={CourseTruth.Sha(truth)} truth={output}");
                return 0;
            }
            string[] ab = range.Split(':');
            long from = long.Parse(ab[0], CultureInfo.InvariantCulture), to = long.Parse(ab[1], CultureInfo.InvariantCulture);
            Directory.CreateDirectory(output);
            int failed = 0;
            var log = new List<string>();
            for (long s = from; s < to; s++) {
                var request = CourseBootstrap.Request(args, null);
                request.Seed = s;
                try {
                    var truth = CourseGenerator.Generate(family, request);
                    File.WriteAllText(Path.Combine(output, $"{family.Name}-s{s}.json"), Json.Write(truth, true) + "\n");
                }
                catch (CourseGenerationException e) { failed++; log.Add($"seed {s}: {e.Message}"); }
            }
            File.WriteAllLines(Path.Combine(output, "failures.txt"), log);
            Console.WriteLine($"CRANE_COURSE_BATCH family={family.Name} seeds={from}:{to} failed={failed} out={output}");
            return failed == 0 ? 0 : 3;
        });

        public static void Validate() => Exit(() => {
            string[] args = Environment.GetCommandLineArgs();
            string input = Arg(args, "--in"), output = Arg(args, "--out");
            var verdicts = new JObj();
            foreach (string file in Directory.GetFiles(input, "*.json").OrderBy(f => f, StringComparer.Ordinal)) {
                var truth = (JObj)Json.Parse(File.ReadAllText(file));
                var family = CourseBootstrap.FindFamily(truth.Str("family") ?? "roboboat_2026");
                var spec = ConstraintSpec.Parse(family.ConstraintSpecText);
                verdicts[Path.GetFileName(file)] = ConstraintEvaluator.Evaluate(spec, truth).Select(r => (object)r.ToJson()).ToList();
            }
            File.WriteAllText(output, Json.Write(verdicts, true) + "\n");
            return 0;
        });

        static void Exit(Func<int> body) {
            int code;
            try { code = body(); }
            catch (Exception e) { Console.Error.WriteLine($"CRANE_COURSE_FAILED {e}"); code = 2; }
            EditorApplication.Exit(code);
        }

        static string Arg(string[] args, string key) {
            int i = Array.IndexOf(args, key);
            return i >= 0 && i + 1 < args.Length ? args[i + 1] : null;
        }
    }
}
