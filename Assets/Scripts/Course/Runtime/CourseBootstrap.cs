using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using Sim.Controllers;
using Sim.Course.RoboBoat;
using Sim.Sensors.Vision;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Course {
    /// <summary>
    /// Builds the course when the scene loads, from CLI flags only (no scene edits):
    /// <c>--crane-course-family NAME</c> (or a <c>--crane-roboboat-course</c> JSON naming a
    /// family and seed; a 2025 element list goes to the legacy builder), <c>--crane-course-seed N</c>
    /// (defaults to <c>--crane-seed</c>), <c>--crane-course-mode iid|corner=lo|hi|mixed</c>,
    /// repeated <c>--crane-course-set task.param=value</c>, <c>--crane-course-replay truth.json</c>,
    /// <c>--crane-course-truth PATH</c> (alias <c>--crane-roboboat-course-truth</c>),
    /// <c>--crane-detection-labels oracle|camera</c>, <c>--crane-ros-course-truth</c>.
    /// Logs <c>CRANE_COURSE_READY name= seed= sha256=</c> (and the legacy READY line the rig waits for).
    /// </summary>
    public static class CourseBootstrap {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        static void Install() {
            string[] args = Environment.GetCommandLineArgs();
            if (Arg(args, "--crane-course-family") == null && Arg(args, "--crane-course-replay") == null &&
                Arg(args, JsonCourseSource.Flag) == null) return;
            bool done = false;
            SceneManager.sceneLoaded += (_, _) => {
                if (done) return;
                try { done = Run(args); }
                catch (Exception error) {
                    Debug.LogError($"CRANE_COURSE_FAILED {error.Message}\nCRANE_ROBOBOAT_COURSE_FAILED {error}");
                    done = true;
                }
            };
        }

        /// <summary>False when the loaded scene has no boat yet (a bootstrap scene).</summary>
        static bool Run(string[] args) {
            RoboBoatCourse.ApplyCurrentFromArgs();
            string legacy = Arg(args, JsonCourseSource.Flag);
            JObj spec = legacy != null ? (JObj)Json.Parse(File.ReadAllText(legacy)) : null;
            if (spec != null && !spec.Has("family") && Arg(args, "--crane-course-family") == null) {
                if (!RoboBoatCourse.BoatStart(out Vector2 legacyStart, out float legacyYaw)) return false;
                JsonCourseSource.Build(legacy, args);  // laid out from this same start
                AddStartReset(null, legacyStart, legacyYaw);
                return true;
            }
            if (!RoboBoatCourse.BoatStart(out Vector2 start, out float startYaw)) return false;

            ICourseSource source;
            CourseFamilyDefinition family;
            string replay = Arg(args, "--crane-course-replay");
            if (replay != null) {
                source = new ReplayCourseSource(replay);
                family = FindFamily(((JObj)Json.Parse(File.ReadAllText(replay))).Str("family"));
            }
            else {
                family = FindFamily(Arg(args, "--crane-course-family") ?? spec.Str("family"));
                source = new GeneratedCourseSource(family, Request(args, spec));
            }

            JObj truth = source.Truth();
            if (!source.InWorldFrame) CourseTruth.ToWorld(truth, new Pose2(start.x, start.y, startYaw));
            var (speed, toward) = RoboBoatCourse.CurrentNow();
            var mode = Arg(args, "--crane-detection-labels") == "camera" ? LabelMode.Camera : LabelMode.Oracle;
            truth.Set("currentSpeed", (double)speed).Set("currentToward", CourseFrame.Rad(toward))
                .Set("label_mode", mode == LabelMode.Camera ? "camera" : "oracle");
            string commit = Arg(args, "--crane-source-commit");
            var identity = (JObj)Json.Path(truth, "generator.identity");
            if (commit != null && (identity == null || Json.Path(identity, "generator.crane_commit") as string == "unknown")) {
                ((JObj)truth["generator"]).Set("crane_commit", commit);
                identity?.Obj("generator").Set("crane_commit", commit);
            }
            ulong seed = unchecked((ulong)(long)(Json.Path(truth, "generator.seed") is double s ? s : 0));

            var built = CourseBuilder.Build(truth, family.Catalog, mode, seed, family.HideSceneGroups);
            foreach (var sensor in UnityEngine.Object.FindObjectsByType<BoundingBox3D>(FindObjectsSortMode.None))
                sensor.ReplaceTargets(built.Targets);

            string truthPath = Arg(args, "--crane-course-truth") ?? Arg(args, "--crane-roboboat-course-truth");
            var host = new GameObject("Course State");
            host.transform.SetParent(built.Root, false);
            var registry = host.AddComponent<CourseStateRegistry>();
            if (truthPath != null)
                CourseTruthWriter.Write(truthPath, truth, CourseTruth.Perception(truth, family.Catalog, mode, seed));
            registry.Init(truth, built.States, truthPath != null ? CourseTruthWriter.StatesPath(truthPath) : null);
            if (Array.IndexOf(args, "--crane-ros-course-truth") >= 0)
                host.AddComponent<CourseTruthPublisher>().Init(truth, registry);

            AddStartReset(built.Root, start, startYaw);

            string name = truth.Str("name"), sha = CourseTruth.Sha(truth);
            Debug.Log($"CRANE_COURSE_READY name={name} seed={seed} sha256={sha} " +
                      $"pair_key={Json.Path(truth, "generator.identity.pair_key") ?? "none"} " +
                      $"source_sha256={Json.Path(truth, "generator.identity.generator.source_sha256") ?? "none"} placeholders={built.Placeholders} " +
                      $"labels={mode.ToString().ToLowerInvariant()} truth={truthPath ?? "none"}");
            Debug.Log($"CRANE_ROBOBOAT_COURSE_READY name={name} elements={truth.Arr("elements").Count} " +
                      $"bays={truth.Arr("bays").Count} start=({start.x:R},{start.y:R},{startYaw:R}) truth={truthPath ?? "none"}");
            return true;
        }

        /// <summary>The sim-only /crane/reset_to_start, on the course truth start pose.</summary>
        static void AddStartReset(Transform root, Vector2 start, float startYaw) {
            var host = new GameObject("Course Start Reset");
            if (root != null) host.transform.SetParent(root, false);
            host.AddComponent<RoboBoatStartReset>().Capture(RoboBoatStartReset.FindBoatBody(), start, startYaw);
        }

        public static CourseRequest Request(string[] args, JObj spec) {
            string seedText = Arg(args, "--crane-course-seed") ?? (spec?["seed"] is double d ? d.ToString(CultureInfo.InvariantCulture) : null)
                              ?? Arg(args, "--crane-seed") ?? "0";
            var request = new CourseRequest {
                Seed = long.Parse(seedText, CultureInfo.InvariantCulture),
                Mode = CourseRequest.ParseMode(Arg(args, "--crane-course-mode") ?? spec?.Str("mode"))
            };
            foreach (string pin in All(args, "--crane-course-set").Concat(spec?.Arr("set").Cast<string>() ?? Enumerable.Empty<string>())) {
                int eq = pin.IndexOf('=');
                if (eq <= 0) throw new ArgumentException($"--crane-course-set expects task.param=value, got {pin}");
                request.Overrides[pin.Substring(0, eq)] = pin.Substring(eq + 1);
            }
            return request;
        }

        public static CourseFamilyDefinition FindFamily(string name) {
            foreach (var f in Resources.LoadAll<CourseFamilyDefinition>(""))
                if (f.Name == name) return f;
            throw new ArgumentException($"no course family '{name}' under Resources");
        }

        static string Arg(string[] args, string key) {
            int i = Array.IndexOf(args, key);
            return i >= 0 && i + 1 < args.Length && !args[i + 1].StartsWith("--", StringComparison.Ordinal) ? args[i + 1] : null;
        }

        static IEnumerable<string> All(string[] args, string key) {
            for (int i = 0; i + 1 < args.Length; i++) if (args[i] == key) yield return args[i + 1];
        }
    }
}
