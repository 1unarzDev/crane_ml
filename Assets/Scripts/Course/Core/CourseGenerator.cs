using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;

namespace Sim.Course {
    /// <summary>What to generate: seed, sampling mode and pinned parameters
    /// (<c>--crane-course-set task.param=value</c>).</summary>
    public sealed class CourseRequest {
        public long Seed;
        public SampleMode Mode = SampleMode.Iid;
        public SortedDictionary<string, string> Overrides = new SortedDictionary<string, string>(StringComparer.Ordinal);
        public VesselProfile Vessel = new VesselProfile();

        public static SampleMode ParseMode(string text) {
            switch ((text ?? "iid").Trim()) {
                case "iid": return SampleMode.Iid;
                case "corner=lo": return SampleMode.CornerLo;
                case "corner=hi": return SampleMode.CornerHi;
                case "corner=mixed": return SampleMode.CornerMixed;
                default: throw new ArgumentException($"sampling mode '{text}': iid | corner=lo|hi|mixed");
            }
        }

        public static string ModeName(SampleMode m) =>
            m == SampleMode.Iid ? "iid" : m == SampleMode.CornerLo ? "corner=lo" : m == SampleMode.CornerHi ? "corner=hi" : "corner=mixed";
    }

    public sealed class CourseGenerationException : Exception {
        public readonly IReadOnlyList<string> Rows;
        public CourseGenerationException(IReadOnlyList<string> rows)
            : base("CRANE_COURSE_FAILED violated=" + string.Join(",", rows)) { Rows = rows; }
    }

    /// <summary>Family + seed + overrides → truth, in pure C# (§2). Each task draws from its
    /// own fork of the seed, its parameters in sorted path order; after each task its rows are
    /// checked and only that task is re-forked on failure (≤50), then the layout (≤10).</summary>
    public static class CourseGenerator {
        public const int TaskRetries = 50, LayoutRetries = 10;
        public const double FootprintMargin = 1.5;  // D: room around a task's objects

        public static JObj Generate(CourseFamilyDefinition family, CourseRequest request) {
            var spec = ConstraintSpec.Parse(family.ConstraintSpecText);
            var root = new CourseRng(unchecked((ulong)request.Seed));
            CheckOverrides(family, request);
            var failed = new SortedDictionary<string, int>(StringComparer.Ordinal);
            for (int j = 0; j < LayoutRetries; j++) {
                var layoutRng = root.Fork(j == 0 ? "layout" : $"layout#{j}");
                double side = layoutRng.Sign();
                var result = TryLayout(family, request, spec, root, j, side, failed);
                if (result != null) return Finish(family, request, spec, result, side, j + 1, failed);
            }
            throw new CourseGenerationException(failed.Keys.ToList());
        }

        static List<GeneratedTask> TryLayout(CourseFamilyDefinition family, CourseRequest request, ConstraintSpec spec,
            CourseRng root, int j, double side, SortedDictionary<string, int> failed) {
            var boundary = family.Boundary;
            var occupancy = new Occupancy(boundary);
            var placed = new Dictionary<string, TaskLayout>();
            var idsByType = family.Tasks.GroupBy(t => t.TaskType).ToDictionary(g => g.Key, g => g.First().TaskId);
            var done = new List<GeneratedTask>();
            var global = spec.Rows.Where(r => r.Task == "course" && !r.Final).ToList();
            foreach (var task in family.Tasks) {
                GeneratedTask accepted = null;
                for (int k = 0; k <= TaskRetries && accepted == null; k++) {
                    string key = task.TaskId + (j > 0 ? $"@{j}" : "") + (k > 0 ? $"#{k}" : "");
                    var rng = root.Fork(key);
                    var values = Sample(task, rng, request);
                    var pc = new PlacementContext {
                        Side = side, Boundary = boundary, Occupancy = occupancy, Placed = placed, TaskIdsByType = idsByType, Params = values, Rng = rng
                    };
                    Pose2 anchor = task.Place(pc);
                    var layout = task.Generate(new TaskContext {
                        TaskId = task.TaskId, Rng = rng, Anchor = anchor, Side = side, Params = values,
                        Occupancy = occupancy, Catalog = family.Catalog, Vessel = request.Vessel, Placement = pc
                    });
                    if (layout.Footprint.Count == 0 && layout.Objects.Count > 0 && !layout.Scattered)
                        layout.Footprint = Geometry.Hull(layout.Objects.Select(o => o.At), FootprintMargin);
                    var candidate = new GeneratedTask { Definition = task, Params = values, Layout = layout, Attempts = k + 1 };
                    var trial = new List<GeneratedTask>(done) { candidate };
                    var core = CourseTruth.Core(family.Name, side, boundary, request.Vessel, trial, family.Catalog);
                    // This task's rows and those of every task already placed: a later task may
                    // not break an earlier one (e.g. block its aisle), and is re-forked if it does.
                    var rows = trial.SelectMany(t => t.Definition.Constraints(spec).Concat(t.Definition.PathChecks(spec, t.Layout)))
                        .Where(r => !r.Final).Concat(global);
                    var bad = rows.Select(r => ConstraintEvaluator.Evaluate(r, core)).Where(r => !r.Pass).Select(r => r.Id).ToList();
                    if (bad.Count == 0) accepted = candidate;
                    else foreach (string id in bad) Count(failed, $"{task.TaskId}:{id}");
                }
                if (accepted == null) return null;
                done.Add(accepted);
                placed[task.TaskId] = accepted.Layout;
                occupancy.Add(accepted.Layout, family.Catalog);
            }
            var full = CourseTruth.Core(family.Name, side, boundary, request.Vessel, done, family.Catalog);
            var finalBad = ConstraintEvaluator.Evaluate(spec, full).Where(r => !r.Pass).Select(r => r.Id).ToList();
            foreach (string id in finalBad) Count(failed, $"course:{id}");
            return finalBad.Count == 0 ? done : null;
        }

        static JObj Finish(CourseFamilyDefinition family, CourseRequest request, ConstraintSpec spec,
            List<GeneratedTask> tasks, double side, int layoutAttempts, SortedDictionary<string, int> rejected) {
            var core = CourseTruth.Core(family.Name, side, family.Boundary, request.Vessel, tasks, family.Catalog);
            var results = ConstraintEvaluator.Evaluate(spec, core).Select(r => (object)r.ToJson()).ToList();
            var overrides = new JObj();
            foreach (var kv in request.Overrides) overrides[kv.Key] = kv.Value;
            var attempts = new JObj().Set("layout", (double)layoutAttempts);
            foreach (var t in tasks) attempts[t.Definition.TaskId] = (double)t.Attempts;
            var generator = new JObj().Set("seed", (double)request.Seed).Set("mode", CourseRequest.ModeName(request.Mode))
                .Set("overrides", overrides).Set("family_sha256", family.Sha256()).Set("attempts", attempts);
            var rejections = new JObj();
            foreach (var kv in rejected) rejections[kv.Key] = (double)kv.Value;
            generator.Set("rejected", rejections);
            string name = $"{family.Name}-s{request.Seed.ToString(CultureInfo.InvariantCulture)}";
            return CourseTruth.Assemble(core, name, family.Name, family.HandbookId, family.HandbookSha256, generator, results);
        }

        /// <summary>Draws every parameter in sorted path order (so adding a pin never shifts the
        /// other draws), then applies pins.</summary>
        public static ParamValues Sample(CourseTaskDefinition task, CourseRng rng, CourseRequest request) {
            var values = new ParamValues();
            foreach (var e in task.ParamEntries) {
                object v = e.Range.Sample(rng, request.Mode);
                if (request.Overrides.TryGetValue($"{task.TaskId}.{e.Path}", out string pin)) v = e.Range.Parse(pin);
                values.Set(e.Path, v);
            }
            return values;
        }

        static void Count(SortedDictionary<string, int> counts, string key) =>
            counts[key] = counts.TryGetValue(key, out int n) ? n + 1 : 1;

        static void CheckOverrides(CourseFamilyDefinition family, CourseRequest request) {
            var known = new HashSet<string>(family.Tasks.SelectMany(t => t.ParamEntries.Select(e => $"{t.TaskId}.{e.Path}")));
            foreach (string key in request.Overrides.Keys)
                if (!known.Contains(key)) throw new ArgumentException($"--crane-course-set {key}: no such task parameter");
        }
    }
}
