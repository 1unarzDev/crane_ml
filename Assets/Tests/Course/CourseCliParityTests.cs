using System;
using System.IO;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

namespace Sim.Course.Tests {
    /// <summary>Unity == Tools/RoboBoat/cli: every truth the CLI wrote to CRANE_COURSE_CLI_DIR
    /// (Tools/RoboBoat/tests/test_cli.py, same variable) is regenerated in the editor from its
    /// identity (seed, options) and must match byte for byte, identity included.</summary>
    public sealed class CourseCliParityTests {
        static string Root => Path.GetFullPath(Path.Combine(Application.dataPath, ".."));

        static CourseRequest FromIdentity(JObj identity) {
            var options = identity.Obj("options");
            var request = new CourseRequest {
                Seed = (long)identity.Num("seed"), Mode = CourseRequest.ParseMode(options.Str("mode"))
            };
            foreach (var kv in options.Obj("set")) request.Overrides[kv.Key] = (string)kv.Value;
            return request;
        }

        [Test]
        public void CliTruthsEqualTheEditorsByteForByte() {
            string dir = Environment.GetEnvironmentVariable("CRANE_COURSE_CLI_DIR");
            if (string.IsNullOrEmpty(dir)) Assert.Ignore("CRANE_COURSE_CLI_DIR not set (written by Tools/RoboBoat/tests/test_cli.py)");
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
    }
}
