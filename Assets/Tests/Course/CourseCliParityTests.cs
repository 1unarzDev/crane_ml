using System;
using System.Diagnostics;
using System.IO;
using System.Linq;
using NUnit.Framework;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace Sim.Course.Tests {
    /// <summary>Unity == Tools/RoboBoat/cli: every truth the CLI writes (seeds 1-50 per mode plus a
    /// pinned case, as Tools/RoboBoat/tests/test_cli.py) is regenerated in the editor from its
    /// identity (seed, options) and must match byte for byte, identity included. The test builds the
    /// tool when it is missing or stale; CRANE_COURSE_CLI_DIR (written by test_cli.py) overrides it
    /// with truths already on disk.</summary>
    public sealed class CourseCliParityTests {
        static string Root => Path.GetFullPath(Path.Combine(Application.dataPath, ".."));

        static CourseRequest FromIdentity(JObj identity) {
            var options = identity.Obj("options");
            var request = new CourseRequest {
                Seed = (long)identity.Num("seed"), Mode = CourseRequest.ParseMode(options.Str("mode"))
            };
            foreach (var kv in options.Obj("set")) request.Overrides[kv.Key] = (string)kv.Value;
            if (options.Obj("vessel") != null) request.Vessel = VesselProfile.FromJson(options.Obj("vessel"));
            return request;
        }

        static readonly string[] Modes = { "iid", "corner=lo", "corner=hi", "corner=mixed" };
        static string Cli => Path.Combine(Root, "Tools/RoboBoat/cli");
        static string Dll => Path.Combine(Cli, "bin/Release/net10.0/coursegen.dll");

        static (int code, string output) Dotnet(string args) {
            try {
                using (var p = Process.Start(new ProcessStartInfo("dotnet", args) {
                    WorkingDirectory = Cli, UseShellExecute = false, RedirectStandardOutput = true, RedirectStandardError = true
                })) {
                    var stderr = p.StandardError.ReadToEndAsync();
                    string stdout = p.StandardOutput.ReadToEnd();
                    p.WaitForExit();
                    return (p.ExitCode, stdout + stderr.Result);
                }
            }
            catch (Exception e) { return (-1, e.Message); }
        }

        static void Build() {
            var (code, output) = Dotnet("build -c Release -nologo -v q");  // the SDK from Tools/RoboBoat/cli/global.json
            if (code != 0) Assert.Fail($"cannot build {Cli} (needs the .NET SDK in global.json on PATH; or set CRANE_COURSE_CLI_DIR):\n{output}");
        }

        /// <summary>The CLI's truths in <paramref name="dir"/>, building the tool first when it is
        /// missing or older than the generator sources (exit 3 "stale build").</summary>
        static void WriteCliTruths(string dir) {
            if (!File.Exists(Dll)) Build();
            var runs = Modes.Select(m => $"--seeds 1:51 --mode {m} --out \"{Path.Combine(dir, m)}\"").ToList();
            runs.Add($"--seeds 1:11 --set supply_drop.water_hulls=1 --set harbor_alert.zone2_offset=7.5 --out \"{Path.Combine(dir, "pinned")}\"");
            foreach (string run in runs) {
                var (code, output) = Dotnet($"\"{Dll}\" {run}");
                if (code == 3 && output.Contains("stale build")) { Build(); (code, output) = Dotnet($"\"{Dll}\" {run}"); }
                if (code != 0) Assert.Fail($"coursegen {run} exited {code}:\n{output}");
            }
        }

        [Test, Timeout(1800000)]  // 210 truths take about 3 min in the editor
        public void CliTruthsEqualTheEditorsByteForByte() {
            string dir = Environment.GetEnvironmentVariable("CRANE_COURSE_CLI_DIR"), tmp = null;
            if (string.IsNullOrEmpty(dir)) WriteCliTruths(dir = tmp = Path.Combine(Path.GetTempPath(), "crane-course-cli-" + Guid.NewGuid().ToString("N")));
            try { Compare(dir); }
            finally { if (tmp != null && Directory.Exists(tmp)) Directory.Delete(tmp, true); }
        }

        static void Compare(string dir) {
            int n = 0;
            foreach (string file in Directory.GetFiles(dir, "*.json", SearchOption.AllDirectories).OrderBy(f => f, StringComparer.Ordinal)) {
                string cli = File.ReadAllText(file);
                var identity = (JObj)Json.Path(Json.Parse(cli), "generator.identity");
                var family = CourseBootstrap.FindFamily(identity.Obj("options").Str("family"));
                Assert.That(Json.Write(CourseGenerator.Generate(family, FromIdentity(identity)), true) + "\n", Is.EqualTo(cli), file);
                n++;
            }
            Debug.Log($"CRANE_COURSE_CLI_PARITY truths={n} dir={dir}");
            Assert.That(n, Is.GreaterThanOrEqualTo(210));
        }

        [Test]
        public void EditorKnowsItsGeneratorVersion() {
            var v = CourseIdentity.Current;
            Assert.That(v.SourceSha256, Is.EqualTo(CourseIdentity.SourceSha256(Root)));
            Assert.That(v.CraneCommit, Does.Match("^[0-9a-f]{40}$"));
            Assert.That(v.CraneDirty, Is.Not.Null);
        }

        [Test]
        public void RigFlagsGiveTheSameIdentityAndTheWorldMoveKeepsIt() {
            string[] args = { "player", "--crane-course-seed", "7", "--crane-course-mode", "corner=hi",
                              "--crane-course-set", "supply_drop.water_hulls=1", "--crane-seed", "99" };
            var request = CourseBootstrap.Request(args, null);
            var truth = CourseGenerator.Generate(CourseBootstrap.FindFamily("roboboat_2026"), request);
            var identity = (JObj)Json.Path(truth, "generator.identity");
            Assert.That(Json.Write(identity.Obj("options")),
                Is.EqualTo("{\"family\":\"roboboat_2026\",\"mode\":\"corner=hi\",\"set\":{\"supply_drop.water_hulls\":\"1\"}}"));
            Assert.That(identity.Num("seed"), Is.EqualTo(7));
            Assert.That(identity.Str("pair_key"),
                Is.EqualTo(CourseIdentity.PairKey(7, identity.Obj("options"), CourseIdentity.Current.SourceSha256)));
            string before = Json.Write(identity);
            CourseTruth.ToWorld(truth, new Pose2(3, -4, 1.2));
            Assert.That(Json.Write(Json.Path(truth, "generator.identity")), Is.EqualTo(before));
        }

        [Test]
        public void AVesselOtherThanTheDefaultIsPartOfTheIdentity() {
            var request = new CourseRequest { Seed = 7, Mode = SampleMode.CornerHi };
            var plain = CourseIdentity.Options("roboboat_2026", request);
            Assert.That(plain.Has("vessel"), Is.False);
            request.Vessel = new VesselProfile { Beam = 1.2, Source = "test" };
            var sized = CourseIdentity.Options("roboboat_2026", request);
            Assert.That(Json.Write(sized.Obj("vessel")), Is.EqualTo(Json.Write(request.Vessel.ToJson())));
            string sha = CourseIdentity.Current.SourceSha256;
            Assert.That(CourseIdentity.PairKey(7, sized, sha), Is.Not.EqualTo(CourseIdentity.PairKey(7, plain, sha)));
        }
    }
}
