using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using NUnit.Framework;
using Sim.Course;
using Sim.Course.RoboBoat;
using UnityEngine;

namespace Sim.Course.Tests {
    /// <summary>Generator, evaluator and truth tests (design §7). The fuzz size comes from
    /// CRANE_COURSE_FUZZ (default 100 seeds per mode; 5000 for the full campaign).</summary>
    public sealed class CourseGeneratorTests {
        static string Root => Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
        static CourseFamilyDefinition Family => CourseBootstrap.FindFamily("roboboat_2026");
        static ConstraintSpec Spec => ConstraintSpec.Parse(Family.ConstraintSpecText);

        static JObj Generate(long seed, SampleMode mode = SampleMode.Iid, params string[] pins) {
            var request = new CourseRequest { Seed = seed, Mode = mode };
            foreach (string pin in pins) request.Overrides[pin.Split('=')[0]] = pin.Split('=')[1];
            return CourseGenerator.Generate(Family, request);
        }

        static IEnumerable<JObj> Elements(JObj t, string task) =>
            t.Arr("elements").Cast<JObj>().Where(e => e.Str("task") == task);

        [Test]
        public void SplitMix64AndFnvMatchTheirGoldens() {
            var rng = new CourseRng(0);
            Assert.That(rng.NextU64(), Is.EqualTo(0xE220A8397B1DCDAFUL));
            Assert.That(rng.NextU64(), Is.EqualTo(0x6E789E6AA1B965F4UL));
            Assert.That(rng.NextU64(), Is.EqualTo(0x06C45D188009454FUL));
            Assert.That(CourseRng.Fnv1a64(0, "layout"), Is.EqualTo(0xD72874B3DDB6AFFFUL));
            Assert.That(CourseRng.Fnv1a64(42, "entry_gates"), Is.EqualTo(0x401D84E61F1656BAUL));
        }

        [Test]
        public void ParamsAreSortedByPathAndRangesHold() {
            foreach (var task in Family.Tasks) {
                var paths = task.ParamEntries.Select(e => e.Path).ToList();
                Assert.That(paths, Is.EqualTo(paths.OrderBy(p => p, StringComparer.Ordinal).ToList()), task.TaskId);
            }
            var range = ParamRange.Uniform(1.83, 3.05);
            var rng = new CourseRng(3);
            for (int i = 0; i < 1000; i++) Assert.That((double)range.Sample(rng, SampleMode.Iid), Is.InRange(1.83, 3.05));
            Assert.That(range.Corner(false), Is.EqualTo(1.83));
            Assert.That(range.Corner(true), Is.EqualTo(3.05));
            Assert.That(ParamRange.Int(1, 3).Sample(rng, SampleMode.CornerHi), Is.EqualTo(3L));
            Assert.That(ParamRange.Default("separate", "on_mark").Sample(rng, SampleMode.CornerHi), Is.EqualTo("separate"));
            Assert.That(ParamRange.Default("separate", "on_mark").Parse("on_mark"), Is.EqualTo("on_mark"));
            Assert.Throws<ArgumentException>(() => ParamRange.Choice("red", "green").Parse("blue"));
        }

        [Test]
        public void SameSeedGivesTheGoldenSha() {
            // Goldens from the CoreCLR harness, which matched the Unity (Mono) editor bit for bit.
            // Update deliberately when the family, spec or generator changes.
            Assert.That(CourseTruth.Sha(Generate(1)), Is.EqualTo("2df67206280c2e0c052ca43546d7995e2c8ad42a7caa9ab3091c66701a7151a0"));
            Assert.That(CourseTruth.Sha(Generate(7, SampleMode.CornerHi)), Is.EqualTo("8d8753da74f06a9b8143e5221fff17acac047701d5c126c289b697e352872550"));
            Assert.That(Json.Write(Generate(11)), Is.EqualTo(Json.Write(Generate(11))));
        }

        [Test]
        public void LeafTasksChangeOnlyThemselves() {
            for (long seed = 20; seed < 25; seed++) {
                JObj a = Generate(seed), b = Generate(seed, SampleMode.Iid, "supply_drop.water_hulls=1", "harbor_alert.zone2_offset=7.5");
                foreach (string task in new[] { "entry_gates", "debris_clearance", "marina", "speed_challenge" })
                    Assert.That(Json.Write(Elements(b, task).ToList()), Is.EqualTo(Json.Write(Elements(a, task).ToList())), $"seed {seed} {task}");
            }
        }

        [Test]
        public void ChangingAnAnchorTaskMovesDependantsRigidly() {
            int compared = 0;
            for (long seed = 30; seed < 60 && compared < 3; seed++) {
                JObj a = Generate(seed, SampleMode.Iid, "entry_gates.row_separation=10", "entry_gates.gate2_yaw_deg=0");
                JObj b = Generate(seed, SampleMode.Iid, "entry_gates.row_separation=20", "entry_gates.gate2_yaw_deg=0");
                if (Attempts(a, "debris_clearance") != 1 || Attempts(b, "debris_clearance") != 1) continue;
                var ra = Relative(a); var rb = Relative(b);
                Assert.That(rb.Count, Is.EqualTo(ra.Count));
                for (int i = 0; i < ra.Count; i++) {
                    Assert.That(rb[i].X, Is.EqualTo(ra[i].X).Within(0.002), $"seed {seed} element {i}");
                    Assert.That(rb[i].Y, Is.EqualTo(ra[i].Y).Within(0.002), $"seed {seed} element {i}");
                }
                compared++;
            }
            Assert.That(compared, Is.GreaterThan(0));
        }

        static double Attempts(JObj t, string task) => ((JObj)Json.Path(t, "generator.attempts")).Num(task);

        // Debris elements relative to entry gate 2 (both gates face +x here).
        static List<P2> Relative(JObj t) {
            var g2 = t.Arr("gates").Cast<JObj>().First(g => g.Str("id") == "entry_gates.gate2");
            return Elements(t, "debris_clearance").Select(e => new P2(e.Num("x") - g2.Num("x"), e.Num("y") - g2.Num("y"))).ToList();
        }

        [Test]
        public void MarinaInvariantsAndDirectionTableHold() {
            for (long seed = 0; seed < 100; seed++) {
                var t = Generate(seed);
                var bays = t.Arr("bays").Cast<JObj>().ToList();
                foreach (var pier in bays.GroupBy(b => b.Str("pier")))
                    Assert.That(pier.Select(b => b.Obj("sign").Num("number")).OrderBy(n => n), Is.EqualTo(new[] { 1.0, 2.0, 3.0 }));
                var greens = bays.Where(b => b.Obj("indicator").Str("color") == "green").ToList();
                Assert.That(greens, Is.Not.Empty);
                Assert.That(bays.Where(b => b["occupied"] is bool o && o).All(b => b.Obj("indicator").Str("color") == "red"));
                double low = greens.Min(b => b.Obj("sign").Num("number"));
                Assert.That(greens.Count(b => b.Obj("sign").Num("number") == low), Is.EqualTo(1));
                var speed = (JObj)t.Arr("tasks").Cast<JObj>().First(x => x.Str("id") == "speed_challenge")["semantics"];
                Assert.That(speed.Str("direction"), Is.EqualTo(speed.Str("source_color") == "red" ? "ccw" : "cw"));
            }
        }

        [Test, Timeout(int.MaxValue)]
        public void FuzzEveryModePassesAllRowsWithFewRejections() {
            int n = int.TryParse(Environment.GetEnvironmentVariable("CRANE_COURSE_FUZZ"), out int v) ? v : 100;
            foreach (var mode in new[] { SampleMode.Iid, SampleMode.CornerLo, SampleMode.CornerHi, SampleMode.CornerMixed }) {
                int attempts = 0, tasks = 0;
                for (long seed = 0; seed < n; seed++) {
                    JObj t;
                    try { t = Generate(seed + 100000, mode); }
                    catch (CourseGenerationException e) { Assert.Fail($"{mode} seed {seed}: {e.Message}"); throw; }
                    foreach (JObj r in t.Arr("constraints")) Assert.That(r["pass"], Is.True, $"{mode} seed {seed} row {r.Str("id")}");
                    foreach (var kv in (JObj)Json.Path(t, "generator.attempts"))
                        if (kv.Key != "layout") { attempts += (int)(double)kv.Value; tasks++; }
                }
                double rate = (attempts - tasks) / (double)attempts;
                Debug.Log($"CRANE_COURSE_FUZZ mode={mode} seeds={n} task_attempt_rejection={rate:P2}");
                Assert.That(rate, Is.LessThan(0.05), mode.ToString());
            }
        }

        [Test]
        public void TasksTurnWithinTheirLimitsAndHullsDock() {
            var marina = new List<double>();
            var speed = new List<double>();
            for (long seed = 0; seed < 60; seed++) {
                var t = Generate(seed + 200000);
                JObj Task(string id) => t.Arr("tasks").Cast<JObj>().First(x => x.Str("id") == id);
                double heading = (double)Json.Path(Task("debris_clearance"), "params.heading_deg");
                Assert.That(Math.Abs(heading), Is.LessThanOrEqualTo(15), $"seed {seed}");
                marina.Add((double)Json.Path(Task("marina"), "params.yaw_deg"));
                speed.Add((double)Json.Path(Task("speed_challenge"), "params.yaw_deg"));
                // The speed gate lies beyond entry gate 2 and its opening faces the approach (from the
                // channel exit and from gate 2) within 60 deg.
                foreach (string row in new[] { "T3.beyond_gate2", "T3.approach_exit", "T3.approach_gate2" })
                    Assert.That(t.Arr("constraints").Cast<JObj>().First(r => r.Str("id") == row)["pass"], Is.True, $"seed {seed} {row}");
                var slips = (List<object>)Json.Path(Task("supply_drop"), "semantics.slips");
                Assert.That(slips.Count, Is.GreaterThanOrEqualTo(1), $"seed {seed}");
                var target = (string)Json.Path(Task("marina"), "semantics.target.bay");
                foreach (string slip in slips.Cast<string>()) {
                    var bay = t.Arr("bays").Cast<JObj>().First(b => b.Str("id") == slip);
                    Assert.That(slip, Is.Not.EqualTo(target), $"seed {seed}");
                    Assert.That(Json.Path(bay, "indicator.color"), Is.EqualTo("red"), $"seed {seed}");
                    Assert.That(bay["occupied"], Is.False, $"seed {seed}");
                }
            }
            Assert.That(marina.Min(), Is.LessThan(-45)); Assert.That(marina.Max(), Is.GreaterThan(45));
            Assert.That(marina.All(y => Math.Abs(y) <= 90));
            Assert.That(speed.All(y => Math.Abs(y) <= 60));
            Assert.That(speed.Min(), Is.LessThan(-30)); Assert.That(speed.Max(), Is.GreaterThan(30));
            // Free orientations stay free in corner modes; bounded limits (debris heading) go to their corners.
            var lo = Generate(5, SampleMode.CornerLo);
            Assert.That((double)Json.Path(lo.Arr("tasks").Cast<JObj>().First(x => x.Str("id") == "debris_clearance"), "params.heading_deg"), Is.EqualTo(-15));
        }

        [Test]
        public void BadLayoutsAreRejectedByTheirRows() {
            var files = Directory.GetFiles(Path.Combine(Root, "Tools/RoboBoat/tests/bad"), "*.json");
            Assert.That(files.Length, Is.EqualTo(30));
            foreach (string file in files) {
                var t = (JObj)Json.Parse(File.ReadAllText(file));
                var expect = t.Obj("expect");
                var failed = new HashSet<string>(ConstraintEvaluator.Evaluate(Spec, t).Where(r => !r.Pass).Select(r => r.Id));
                var want = expect.Arr("fail").Cast<string>().ToList();
                string name = Path.GetFileName(file);
                Assert.That(expect["exact"], Is.True, name);
                Assert.That(want.Count, Is.EqualTo(1), name);
                Assert.That(failed, Is.EquivalentTo(want), name);
            }
            var good = (JObj)Json.Parse(File.ReadAllText(Path.Combine(Root, "Tools/RoboBoat/tests/fixtures/roboboat_2026-s1.json")));
            Assert.That(ConstraintEvaluator.Evaluate(Spec, good).Where(r => !r.Pass).Select(r => r.Id), Is.Empty);
        }

        [Test]
        public void WorldFrameKeepsTheShaAndDistances() {
            var t = Generate(5);
            string before = CourseTruth.Sha(t);
            var gate = t.Arr("gates").Cast<JObj>().First();
            double cc = gate.Num("cc");
            CourseTruth.ToWorld(t, new Pose2(12.5, -3.25, 1.1));
            Assert.That(CourseTruth.Sha(t), Is.EqualTo(before));
            Assert.That(ConstraintEvaluator.Evaluate(Spec, t).Where(r => !r.Pass).Select(r => r.Id), Is.Empty, "rows are rigid-invariant");
            var g = t.Arr("gates").Cast<JObj>().First();
            JObj p = t.Arr("elements").Cast<JObj>().First(e => e.Str("id") == g.Str("port"));
            JObj s = t.Arr("elements").Cast<JObj>().First(e => e.Str("id") == g.Str("starboard"));
            Assert.That(P2.Dist(ConstraintEvaluator.Xy(p), ConstraintEvaluator.Xy(s)), Is.EqualTo(cc).Within(0.002));
            Assert.That(t.Str("schema"), Is.EqualTo("crane-roboboat-course-truth-v1"));
            foreach (JObj e in t.Arr("elements"))
                foreach (string key in new[] { "task", "label", "template", "x", "y", "radius" }) Assert.That(e.Has(key), key);
        }

        /// <summary>The same layout with the debris beacon colours swapped green<->red: each
        /// beacon's light changes colour and its id is renamed to match, positions unchanged.</summary>
        static JObj FlipBeacons(JObj truth) {
            string Swap(string v) => v == "green" ? "red" : v == "red" ? "green" : v;
            foreach (JObj st in truth.Arr("states")) {
                if (!st.Str("id").StartsWith("debris_clearance.") || !st.Str("id").Contains("_beacon")) continue;
                st.Set("initial", Swap(st.Str("initial")));
                var program = st.Obj("program");
                if (program.Has("value")) program.Set("value", Swap(program.Str("value")));
            }
            string text = Json.Write(truth).Replace("green_beacon", "beacon_swap_tmp").Replace("red_beacon", "green_beacon")
                .Replace("beacon_swap_tmp", "red_beacon");
            return (JObj)Json.Parse(text);
        }

        [Test]
        public void CameraModeLeaksNoColourOrNumeral() {
            JObj red = Generate(9, SampleMode.Iid, "speed_challenge.marker_color=red", "speed_challenge.sign_color=red");
            JObj green = FlipBeacons(Generate(9, SampleMode.Iid, "speed_challenge.marker_color=green", "speed_challenge.sign_color=green"));
            Assert.That(Json.Write(green), Does.Contain("debris_clearance.red_beacon"), "the flip has beacons to swap");
            Assert.That(Json.Write(red), Is.Not.EqualTo(Json.Write(green)));
            var catalog = Family.Catalog;
            string a = Json.Write(CourseTruth.Perception(red, catalog, LabelMode.Camera, 9, salt: 77));
            string b = Json.Write(CourseTruth.Perception(green, catalog, LabelMode.Camera, 9, salt: 77));
            Assert.That(a, Is.EqualTo(b), "camera-mode perception truth must not vary with sign, marker or beacon colour");
            foreach (JObj e in ((JObj)Json.Parse(a)).Arr("elements")) {
                string label = e.Str("label");
                Assert.That(label.Contains("light") || label == "banner" ? !label.Contains("red") && !label.Contains("green") && !label.Any(char.IsDigit) : true, label);
            }
            // Camera ids hash colour-free keys, depend on the salt and not on the logged seed.
            string[] colours = { "red", "green", "yellow", "black", "blue", "white", "orange" };
            foreach (string key in CourseTruth.CameraKeys(red).Values)
                Assert.That(colours.Any(key.Contains), Is.False, key);
            var ids77 = CourseTruth.DetectionIds(red, LabelMode.Camera, 9, 77).Values.ToList();
            var ids78 = CourseTruth.DetectionIds(red, LabelMode.Camera, 9, 78).Values.ToList();
            var otherSeed = CourseTruth.DetectionIds(red, LabelMode.Camera, 10, 77).Values.ToList();
            Assert.That(ids77, Is.Unique);
            Assert.That(ids77.Intersect(ids78), Is.Empty, "a new salt gives new ids");
            Assert.That(otherSeed, Is.EqualTo(ids77), "the seed does not enter camera ids");
            Assert.That(CourseTruth.DetectionIds(red, LabelMode.Camera, 9).Values, Is.EquivalentTo(CourseTruth.DetectionIds(red, LabelMode.Camera, 9).Values),
                "the run salt is fixed for the process");
            string sha = CourseTruth.Sha(red);
            CourseTruth.Perception(red, catalog, LabelMode.Camera, 9);
            Assert.That(CourseTruth.Sha(red), Is.EqualTo(sha), "ids never feed truth_sha256");
            string oracle = Json.Write(CourseTruth.Perception(red, catalog, LabelMode.Oracle, 9));
            Assert.That(oracle, Does.Contain("red_light_buoy").And.Contains("banner_1"));
            Assert.That(Json.Write(CourseTruth.Perception(red, catalog, LabelMode.Oracle, 9, salt: 1)), Is.EqualTo(oracle), "oracle ids are seed-stable");
        }

        [Test]
        public void ProgramsGiveTheirValuesAtTime() {
            var toggle = Programs.Toggle("red", "green", 10, 0);
            Assert.That(Programs.ValueAt(toggle, 0), Is.EqualTo("red"));
            Assert.That(Programs.ValueAt(toggle, 10.5), Is.EqualTo("green"));
            Assert.That(Programs.ValueAt(toggle, 21), Is.EqualTo("red"));
            Assert.That(Programs.Initial(toggle), Is.EqualTo("red"));
            Assert.That(Programs.Initial(Programs.Toggle("red", "green", 10, 15)), Is.EqualTo("green"), "initial respects the phase");
            Assert.That(Programs.Initial(Programs.Toggle("red", "green", 10, 25)), Is.EqualTo("red"));
            var horn = Programs.Horn(840, "two", 2, 100);
            Assert.That(Programs.ValueAt(horn, 100.2), Is.EqualTo("on:840"));
            Assert.That(Programs.ValueAt(horn, 100.7), Is.EqualTo("off"));
            Assert.That(Programs.ValueAt(horn, 101.2), Is.EqualTo("on:840"));
            Assert.That(Programs.ValueAt(horn, 103.6), Is.EqualTo("on:840"), "second signal 2 s after the first ends");
            Assert.That(Programs.Blasts(horn).Count(), Is.EqualTo(4));
        }

        [Test]
        public void LegacySpecStillParses() {
            var spec = JsonCourseSource.Parse(File.ReadAllText(Path.Combine(Root, "Tools/RoboBoat/courses/roboboat_2025.json")));
            Assert.That(spec.name, Is.EqualTo("roboboat_2025"));
            Assert.That(spec.elements.Length, Is.GreaterThan(10));
            Assert.That(CourseVisuals.InShape("circle", 0.5f, 0.5f));
            Assert.That(CourseVisuals.InShape("circle", 0.95f, 0.95f), Is.False);
            Assert.That(SignFace.Inked(SignFace.Style.Number, "1", 0.5f, 0.5f));
            Assert.That(SignFace.Inked(SignFace.Style.Number, "1", 0.05f, 0.5f), Is.False);
        }
    }
}

namespace Sim.Course.Tests {
    public sealed class LegacyCourseSpecTests {
        [Test]
        public void CourseSpecParsesElementsAndDockBays() {
            var spec = JsonCourseSource.Parse(@"{
                ""name"": ""t"", ""hide"": [""Marina""], ""elements"": [
                  {""template"": ""RedBuoy0"", ""label"": ""red_buoy"", ""x"": 4, ""y"": 1},
                  {""template"": ""Dock0"", ""yaw"": 3.14, ""bays"": [""red_circle"", ""blue_plus""],
                   ""occupied"": [1]}]}");
            Assert.That(spec.elements.Length, Is.EqualTo(2));
            Assert.That(spec.elements[0].scale, Is.EqualTo(1f), "scale defaults to 1");
            Assert.That(spec.elements[1].bays, Is.EqualTo(new[] { "red_circle", "blue_plus" }));
            Assert.That(spec.elements[1].occupied, Is.EqualTo(new[] { 1 }));
            Assert.Throws<ArgumentException>(() => JsonCourseSource.Parse(@"{""elements"": [{""label"": ""x""}]}"));
        }

        [Test]
        public void BannerShapesAreDrawnWhereExpected() {
            Assert.That(CourseVisuals.InShape("plus", 0.5f, 0.8f));
            Assert.That(CourseVisuals.InShape("plus", 0.8f, 0.8f), Is.False);
            Assert.That(CourseVisuals.InShape("triangle", 0.5f, 0.75f));
            Assert.That(CourseVisuals.InShape("triangle", 0.2f, 0.75f), Is.False);
            Assert.That(CourseVisuals.BannerColor("blue_square"), Is.EqualTo(new Color(0.1f, 0.3f, 0.9f)));
        }
    }
}
