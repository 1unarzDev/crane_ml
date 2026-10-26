using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;

namespace Sim.Course {
    /// <summary>Which generator made a truth: the crane commit, whether the generator files were
    /// dirty, and the sha256 of the generator source set (code and family assets).</summary>
    public sealed class GeneratorVersion {
        public string CraneCommit = "unknown";
        public bool? CraneDirty;
        public string SourceSha256 = "unknown";

        public JObj ToJson() => new JObj().Set("crane_commit", CraneCommit)
            .Set("crane_dirty", CraneDirty).Set("source_sha256", SourceSha256);

        public static GeneratorVersion FromJson(JObj o) => new GeneratorVersion {
            CraneCommit = o.Str("crane_commit") ?? "unknown",
            CraneDirty = o["crane_dirty"] as bool?,
            SourceSha256 = o.Str("source_sha256") ?? "unknown"
        };
    }

    /// <summary>Course identity (docs: Tools/RoboBoat/cli/README.md), stamped into
    /// <c>generator.identity</c> of every generated truth by Unity and by the command-line build
    /// alike: seed, options, generator version, truth_sha256 and the pair key
    /// sha256(canonical {seed, options, source_sha256}). Added after the truth hash, so the
    /// hash and its goldens are unchanged.</summary>
    public static class CourseIdentity {
        public const int Version = 1;

        /// <summary>The generator source set, relative to the crane root: every file below these
        /// directories except Unity .meta files. Tools/RoboBoat/cli compiles the .cs files of the
        /// first two by reference.</summary>
        public static readonly string[] SourceDirs = {
            "Assets/Scripts/Course/Core", "Assets/Scripts/Course/RoboBoat/Tasks", "Assets/Config/Course/Resources"
        };

        /// <summary>Set by the host (Unity editor/player, the CLI); unknown until then.</summary>
        public static Func<GeneratorVersion> Resolve;
        static GeneratorVersion current;

        public static GeneratorVersion Current {
            get => current ?? (current = Resolve?.Invoke() ?? new GeneratorVersion());
            set => current = value;
        }

        public static List<string> SourceFiles(string root) {
            var files = new List<string>();
            foreach (string dir in SourceDirs) {
                string abs = Path.Combine(root, dir);
                if (!Directory.Exists(abs)) continue;
                foreach (string f in Directory.GetFiles(abs, "*", SearchOption.AllDirectories))
                    if (!f.EndsWith(".meta", StringComparison.Ordinal))
                        files.Add(Path.GetRelativePath(root, f).Replace('\\', '/'));
            }
            files.Sort(StringComparer.Ordinal);
            return files;
        }

        public static string FileSha256(string path) {
            using (var sha = SHA256.Create()) return Hex(sha.ComputeHash(File.ReadAllBytes(path)));
        }

        /// <summary>sha256 of the lines "relative/path sha256\n", sorted by path.</summary>
        public static string SourceSha256(string root) =>
            CourseFamilyDefinition.Hex(string.Concat(SourceFiles(root).Select(f => $"{f} {FileSha256(Path.Combine(root, f))}\n")));

        /// <summary>The version of the checkout at <paramref name="root"/>: source hash from disk,
        /// commit and dirty flag (of the source set only) from git when available.</summary>
        public static GeneratorVersion FromCheckout(string root, string commit = null) {
            var v = new GeneratorVersion { SourceSha256 = SourceSha256(root) };
            string head = Git(root, "rev-parse HEAD");
            v.CraneCommit = !string.IsNullOrEmpty(commit) ? commit : string.IsNullOrEmpty(head) ? "unknown" : head;
            if (head != null) v.CraneDirty = Git(root, "status --porcelain -- " + string.Join(" ", SourceDirs)).Length > 0;
            return v;
        }

        static string Git(string root, string args) {
            try {
                var p = Process.Start(new ProcessStartInfo("git", $"-C \"{root}\" {args}") {
                    RedirectStandardOutput = true, RedirectStandardError = true, UseShellExecute = false
                });
                string output = p.StandardOutput.ReadToEnd().Trim();
                p.WaitForExit();
                return p.ExitCode == 0 ? output : null;
            }
            catch (Exception) { return null; }
        }

        public static JObj Options(string family, CourseRequest request) {
            var set = new JObj();
            foreach (var kv in request.Overrides) set[kv.Key] = kv.Value;
            var options = new JObj().Set("family", family).Set("mode", CourseRequest.ModeName(request.Mode)).Set("set", set);
            // The vessel sizes the course; the default one stays implicit so its identities keep their pair keys.
            var vessel = request.Vessel.ToJson();
            if (Json.Write(vessel) != Json.Write(new VesselProfile().ToJson())) options.Set("vessel", vessel);
            return options;
        }

        public static string PairKey(long seed, JObj options, string sourceSha256) =>
            CourseFamilyDefinition.Hex(Json.Write(new JObj().Set("seed", (double)seed).Set("options", options)
                .Set("source_sha256", sourceSha256)));

        /// <summary>Writes generator.identity (and generator.crane_commit) from <see cref="Current"/>.</summary>
        public static void Stamp(JObj truth, string family, CourseRequest request) => Stamp(truth, family, request, Current);

        public static void Stamp(JObj truth, string family, CourseRequest request, GeneratorVersion version) {
            var generator = (JObj)truth["generator"];
            var options = Options(family, request);
            generator.Set("crane_commit", version.CraneCommit);
            generator.Set("identity", new JObj().Set("version", (double)Version).Set("seed", (double)request.Seed)
                .Set("options", options).Set("generator", version.ToJson())
                .Set("truth_sha256", CourseTruth.Sha(truth))
                .Set("pair_key", PairKey(request.Seed, options, version.SourceSha256)));
        }

        static string Hex(byte[] bytes) {
            var sb = new StringBuilder();
            foreach (byte x in bytes) sb.Append(x.ToString("x2"));
            return sb.ToString();
        }
    }
}
