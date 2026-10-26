using System.IO;

namespace Sim.Course {
    /// <summary>Where a course comes from (§1): generated from a family and seed, or replayed
    /// from a truth file. (2025 element-list specs keep their own builder, JsonCourseSource.)</summary>
    public interface ICourseSource {
        string Describe { get; }
        /// <summary>The truth; generated courses are in the course frame until moved to world.</summary>
        JObj Truth();
        bool InWorldFrame { get; }
    }

    public sealed class GeneratedCourseSource : ICourseSource {
        readonly CourseFamilyDefinition family;
        readonly CourseRequest request;
        public GeneratedCourseSource(CourseFamilyDefinition family, CourseRequest request) { this.family = family; this.request = request; }
        public string Describe => $"{family.Name} seed {request.Seed}";
        public JObj Truth() => CourseGenerator.Generate(family, request);
        public bool InWorldFrame => false;
    }

    public sealed class ReplayCourseSource : ICourseSource {
        readonly string path;
        public ReplayCourseSource(string path) { this.path = path; }
        public string Describe => $"replay {path}";
        public JObj Truth() => (JObj)Json.Parse(File.ReadAllText(path));
        public bool InWorldFrame => true;
    }
}
