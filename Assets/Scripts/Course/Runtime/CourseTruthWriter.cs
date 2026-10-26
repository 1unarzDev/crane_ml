using System.IO;

namespace Sim.Course {
    /// <summary>Files (§4): course_truth.json, perception_truth.json beside it, and the
    /// states.jsonl change log.</summary>
    public static class CourseTruthWriter {
        public static string PerceptionPath(string truthPath) =>
            Path.Combine(Path.GetDirectoryName(Path.GetFullPath(truthPath)) ?? ".", "perception_truth.json");

        public static string StatesPath(string truthPath) =>
            Path.Combine(Path.GetDirectoryName(Path.GetFullPath(truthPath)) ?? ".", "states.jsonl");

        public static void Write(string path, JObj truth, JObj perception) {
            string dir = Path.GetDirectoryName(Path.GetFullPath(path));
            if (!string.IsNullOrEmpty(dir)) Directory.CreateDirectory(dir);
            File.WriteAllText(path, Json.Write(truth, true) + "\n");
            File.WriteAllText(PerceptionPath(path), Json.Write(perception, true) + "\n");
            File.WriteAllText(StatesPath(path), "");
        }

        public static void AppendState(string path, double t, string id, string value) {
            if (string.IsNullOrEmpty(path)) return;
            File.AppendAllText(path, Json.Write(new JObj().Set("t_sim", System.Math.Round(t, 3)).Set("id", id).Set("value", value)) + "\n");
        }
    }
}
