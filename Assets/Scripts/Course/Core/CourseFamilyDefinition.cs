using System.Collections.Generic;
using System.Security.Cryptography;
using System.Text;
using UnityEngine;

namespace Sim.Course {
    /// <summary>A course family (RoboBoat2026.asset): the ordered task assets, the boundary,
    /// the catalog, the handbook it follows and its constraint spec. No layout logic.</summary>
    [CreateAssetMenu(menuName = "Course/Family")]
    public sealed class CourseFamilyDefinition : ScriptableObject {
        [SerializeField] string familyName = "roboboat_2026";
        [SerializeField] List<CourseTaskDefinition> tasks = new List<CourseTaskDefinition>();
        [SerializeField] CourseObjectCatalog catalog;
        [SerializeField, Tooltip("Course area (D): metres behind the start (the shore), length, half width.")]
        Vector3 boundary = new Vector3(10, 130, 65);
        [SerializeField] string handbookId = "RoboBoat-2026-Team-Handbook_111125";
        [SerializeField] string handbookSha256 = "";
        [SerializeField, Tooltip("Tools/RoboBoat/constraints/<family>.json (a link to this asset).")]
        TextAsset constraintSpec;

        string specText;

        public string Name => familyName;
        public IReadOnlyList<CourseTaskDefinition> Tasks => tasks;
        public CourseObjectCatalog Catalog => catalog;
        public string HandbookId => handbookId;
        public string HandbookSha256 => handbookSha256;
        public string ConstraintSpecText => specText ?? (constraintSpec != null ? constraintSpec.text : null);
        public CourseBoundary Boundary => new CourseBoundary { Back = boundary.x, Length = boundary.y, HalfWidth = boundary.z };

        /// <summary>Fills a family outside the asset pipeline (tests and the headless CLI).</summary>
        public void Configure(string name, IEnumerable<CourseTaskDefinition> taskList,
            CourseObjectCatalog objectCatalog, string spec, string handbook = null, string handbookSha = null) {
            familyName = name;
            tasks = new List<CourseTaskDefinition>(taskList);
            catalog = objectCatalog;
            specText = spec;
            boundary = new Vector3(10, 130, 65);
            if (handbook != null) handbookId = handbook;
            if (handbookSha != null) handbookSha256 = handbookSha;
        }

        /// <summary>sha256 of everything that sets the rules: tasks with their parameter ranges,
        /// boundary, catalog radii and labels, handbook and the constraint spec text.</summary>
        public string Sha256() {
            var taskList = new List<object>();
            foreach (var t in tasks) {
                var ranges = new JObj();
                foreach (var e in t.ParamEntries) ranges[e.Path] = e.Range.Describe().Set("tag", e.Source.Tag);
                taskList.Add(new JObj().Set("id", t.TaskId).Set("type", t.TaskType).Set("params", ranges));
            }
            var b = Boundary;
            var doc = new JObj().Set("name", familyName).Set("tasks", taskList)
                .Set("boundary", new List<object> { b.Back, b.Length, b.HalfWidth })
                .Set("catalog", catalog.Describe())
                .Set("handbook", new JObj().Set("id", handbookId).Set("sha256", handbookSha256))
                .Set("constraints", ConstraintSpecText ?? "");
            return Hex(Json.Write(doc));
        }

        public static string Hex(string text) {
            using (var sha = SHA256.Create()) {
                var sb = new StringBuilder();
                foreach (byte x in sha.ComputeHash(Encoding.UTF8.GetBytes(text))) sb.Append(x.ToString("x2"));
                return sb.ToString();
            }
        }
    }
}
